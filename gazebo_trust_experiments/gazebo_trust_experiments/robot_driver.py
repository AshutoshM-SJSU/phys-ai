from __future__ import annotations

import argparse
import json
import math
import random
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .astar import astar
from .config import apply_overrides, load_config
from .coordinates import cell_to_world, odom_to_world_pose, world_to_cell
from .movingai_map import load_movingai_map
from .paths import resolve_from_config


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _path_overlap(a: list[tuple[int, int]], b: list[tuple[int, int]]) -> float:
    if not a and not b:
        return 1.0
    union = set(a) | set(b)
    return 0.0 if not union else len(set(a) & set(b)) / len(union)


class AStarRobotDriver(Node):
    def __init__(self, config_path: Path, robot_id: str, map_override: str | None = None) -> None:
        super().__init__(f'{robot_id}_astar_driver')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        config = apply_overrides(load_config(config_path), map_file=map_override)
        self.config = config
        self.grid = load_movingai_map(resolve_from_config(config.map.file, config_path))
        matches = [robot for robot in config.robots if str(robot.get('id')) == robot_id]
        if len(matches) != 1:
            raise ValueError(f"Expected exactly one robot named '{robot_id}'")
        self.robot = matches[0]
        self.robot_id = robot_id
        self.goal_nodes = [tuple(int(v) for v in goal) for goal in self.robot['goal_nodes']]
        self.goal_selection = str(self.robot.get('goal_selection', 'random')).lower()
        self.goal_rng = random.Random(int(config.simulation.seed) * 1009 + sum((i + 1) * ord(ch) for i, ch in enumerate(robot_id)))
        self.goal_index = int(self.robot.get('initial_goal_index', 0)) % len(self.goal_nodes)
        if self.goal_selection == 'random' and bool(self.robot.get('randomize_initial_goal', True)):
            self.goal_index = self.goal_rng.randrange(len(self.goal_nodes))
        self.start_cell = tuple(int(v) for v in self.robot['start_cell'])
        self.initial_yaw = float(self.robot.get('start_yaw', 0.0))
        self.pose: tuple[float, float, float] | None = None
        self.path: list[tuple[int, int]] = []
        self.previous_path: list[tuple[int, int]] = []
        self.waypoint_index = 0
        self.control_steps = 0
        self.dynamic_blocked: set[tuple[int, int]] = set()
        self.minimum_scan = math.inf
        self.minimum_scan_angle = 0.0
        self.forward_minimum_scan = math.inf
        self.last_emergency_event = -1e9
        self.no_path_active_since: float | None = None
        self.delivery_started = 0.0
        self.last_motion_heading: float | None = None
        self.pending_map_replan = True
        self.motion_mode = 'TURN'
        self.aligned_cycles = 0
        self.stop_hold_cycles = 0

        self.event_pub = self.create_publisher(String, '/experiment/events', 100)
        self.cmd_pub = self.create_publisher(Twist, f'/{robot_id}/cmd_vel', 20)
        self.create_subscription(Odometry, f'/{robot_id}/odometry', self.on_odom, 30)
        self.create_subscription(OccupancyGrid, f'/{robot_id}/dynamic_map', self.on_map, 20)
        self.create_subscription(LaserScan, f'/{robot_id}/scan', self.on_scan, 30)
        self.create_timer(1.0 / config.planning.control_rate_hz, self.control)
        self.get_logger().info(f'Controller ready: {self.robot_id}, start={self.start_cell}, first_goal={self.current_goal}')
        self.emit('mission_started', goal=list(self.current_goal), role=self.robot.get('role', 'robot'))

    @property
    def current_goal(self) -> tuple[int, int]:
        return self.goal_nodes[self.goal_index]

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def emit(self, event_type: str, **details) -> None:
        msg = String()
        msg.data = json.dumps(
            {'event_type': event_type, 'sim_time': self.now(), 'robot_id': self.robot_id, 'details': details},
            separators=(',', ':'),
        )
        self.event_pub.publish(msg)

    def on_scan(self, msg: LaserScan) -> None:
        finite: list[tuple[float, float]] = []
        forward: list[float] = []
        half_angle = self.config.planning.emergency_forward_half_angle
        for index, value in enumerate(msg.ranges):
            if not (math.isfinite(value) and value > (msg.range_min + 0.02) and value <= msg.range_max):
                continue
            angle = _wrap_angle(msg.angle_min + index * msg.angle_increment)
            finite.append((value, angle))
            if abs(angle) <= half_angle:
                forward.append(value)
        if finite:
            self.minimum_scan, self.minimum_scan_angle = min(finite, key=lambda item: item[0])
        else:
            self.minimum_scan = math.inf
            self.minimum_scan_angle = 0.0
        self.forward_minimum_scan = min(forward) if forward else math.inf
        if self.minimum_scan < self.config.planning.near_collision_distance:
            now = self.now()
            if now - self.last_emergency_event > 0.5:
                self.emit('near_collision', minimum_range=self.minimum_scan)
                self.last_emergency_event = now

    def on_map(self, msg: OccupancyGrid) -> None:
        blocked: set[tuple[int, int]] = set()
        threshold = int(self.config.mapping.get('occupied_threshold', 50))
        for row in range(msg.info.height):
            source_y = self.grid.height - 1 - row
            for x in range(msg.info.width):
                value = msg.data[row * msg.info.width + x]
                if value >= threshold and not self.grid.is_blocked(x, source_y):
                    blocked.add((x, source_y))
        if blocked != self.dynamic_blocked:
            self.dynamic_blocked = blocked
            self.pending_map_replan = True

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = odom_to_world_pose(
            p.x, p.y, _yaw_from_quaternion(q.x, q.y, q.z, q.w),
            start_cell=self.start_cell,
            map_height=self.grid.height,
            cell_size=self.config.map.cell_size,
            initial_yaw=self.initial_yaw,
        )

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def overlay_grid(self, current_cell: tuple[int, int]):
        outer = self

        class OverlayGrid:
            width = outer.grid.width
            height = outer.grid.height

            def is_blocked(self, x: int, y: int) -> bool:
                # The robot is physically occupying its current cell, so stale or
                # self-generated dynamic evidence must never make A* reject its
                # own start state. Static geometry is still authoritative.
                if (x, y) == current_cell:
                    return outer.grid.is_blocked(x, y)
                return outer.grid.is_blocked(x, y) or (x, y) in outer.dynamic_blocked

        return OverlayGrid()

    def replan(self, reason: str) -> bool:
        if self.pose is None:
            return False
        current_cell = world_to_cell(
            self.pose[0], self.pose[1], map_height=self.grid.height, cell_size=self.config.map.cell_size
        )
        started = time.perf_counter()
        try:
            new_path = astar(
                self.overlay_grid(current_cell),
                current_cell,
                self.current_goal,
                allow_diagonal=self.config.planning.allow_diagonal,
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            new_path = []
        planning_ms = (time.perf_counter() - started) * 1000.0
        if not new_path:
            if self.no_path_active_since is None:
                self.no_path_active_since = self.now()
                self.emit('no_path_started', start=list(current_cell), goal=list(self.current_goal))
            self.emit(
                'replan_failed',
                reason=reason,
                planning_time_ms=planning_ms,
                start=list(current_cell),
                goal=list(self.current_goal),
                current_dynamic_blocked=current_cell in self.dynamic_blocked,
                goal_dynamic_blocked=self.current_goal in self.dynamic_blocked,
                dynamic_blocked_count=len(self.dynamic_blocked),
            )
            self.get_logger().warning(
                f'No A* path: start={current_cell} goal={self.current_goal} '
                f'start_dynamic_blocked={current_cell in self.dynamic_blocked} '
                f'goal_dynamic_blocked={self.current_goal in self.dynamic_blocked} '
                f'dynamic_blocked_count={len(self.dynamic_blocked)}'
            )
            self.path = []
            return False
        if self.no_path_active_since is not None:
            self.emit('no_path_ended', duration=self.now() - self.no_path_active_since)
            self.no_path_active_since = None
        overlap = _path_overlap(self.path, new_path)
        old_heading = None
        new_heading = None
        if len(self.path) >= 2:
            old_heading = math.atan2(self.path[1][1] - self.path[0][1], self.path[1][0] - self.path[0][0])
        if len(new_path) >= 2:
            new_heading = math.atan2(new_path[1][1] - new_path[0][1], new_path[1][0] - new_path[0][0])
        reversal = False
        if old_heading is not None and new_heading is not None:
            threshold = math.radians(float(self.config.metrics.get('reversal_heading_threshold_degrees', 120.0)))
            reversal = abs(_wrap_angle(new_heading - old_heading)) >= threshold
        self.previous_path = self.path
        self.path = new_path
        self.waypoint_index = 1 if len(new_path) > 1 else 0
        self.pending_map_replan = False
        self.motion_mode = 'TURN'
        self.aligned_cycles = 0
        self.stop_hold_cycles = self.config.planning.waypoint_stop_cycles
        self.emit(
            'replan',
            reason=reason,
            start=list(current_cell),
            goal=list(self.current_goal),
            path_cells=len(new_path),
            path_distance=max(0, len(new_path) - 1) * self.config.map.cell_size,
            planning_time_ms=planning_ms,
            route_overlap=overlap,
            route_reversal=reversal,
            path=[[int(cx), int(cy)] for cx, cy in new_path],
        )
        return True

    def complete_delivery(self) -> None:
        elapsed = self.now() - self.delivery_started
        completed_goal = self.current_goal
        self.emit('delivery_completed', goal=list(completed_goal), delivery_time=elapsed)
        previous_index = self.goal_index
        if self.goal_selection == 'random' and len(self.goal_nodes) > 1:
            candidates = [i for i in range(len(self.goal_nodes)) if i != previous_index]
            self.goal_index = self.goal_rng.choice(candidates)
        else:
            self.goal_index = (self.goal_index + 1) % len(self.goal_nodes)
        self.delivery_started = self.now()
        self.path = []
        self.pending_map_replan = True
        self.motion_mode = 'TURN'
        self.aligned_cycles = 0
        self.stop_hold_cycles = self.config.planning.waypoint_stop_cycles
        self.emit('delivery_started', goal=list(self.current_goal), goal_index=self.goal_index, selection=self.goal_selection)

    def control(self) -> None:
        if self.pose is None:
            return
        if self.delivery_started == 0.0:
            self.delivery_started = self.now()
        self.control_steps += 1
        scheduled = self.control_steps % self.config.planning.replan_every_steps == 0
        if not self.path or scheduled or self.pending_map_replan:
            reason = 'map_change' if self.pending_map_replan else ('scheduled' if scheduled else 'initial')
            if not self.replan(reason):
                self.stop()
                return

        x, y, yaw = self.pose
        # This is a trust / routing experiment, not a local-reactive-driving benchmark.
        # Side walls in narrow corridors must not make the robot "avoid" away from the
        # known A* centerline. Only an obstacle in the forward cone can trigger the
        # hard stop, and the response is stop + replan rather than blind steering.
        if self.forward_minimum_scan < self.config.planning.emergency_stop_distance:
            self.stop()
            self.pending_map_replan = True
            now = self.now()
            if now - self.last_emergency_event > 0.5:
                self.emit('emergency_stop', minimum_range=self.forward_minimum_scan, response='stop_and_replan')
                self.last_emergency_event = now
            return

        goal_x, goal_y = cell_to_world(
            *self.current_goal, map_height=self.grid.height, cell_size=self.config.map.cell_size
        )
        if math.hypot(goal_x - x, goal_y - y) <= self.config.planning.goal_tolerance:
            self.stop()
            self.complete_delivery()
            return
        if self.waypoint_index >= len(self.path):
            self.path = []
            return

        waypoint = self.path[self.waypoint_index]
        wx, wy = cell_to_world(*waypoint, map_height=self.grid.height, cell_size=self.config.map.cell_size)
        distance = math.hypot(wx - x, wy - y)

        # Make waypoint transitions explicit.  The robot must settle at each A*
        # cell centre before it is allowed to turn toward the next segment.
        if distance <= self.config.planning.waypoint_tolerance:
            self.stop()
            self.waypoint_index += 1
            self.motion_mode = 'TURN'
            self.aligned_cycles = 0
            self.stop_hold_cycles = self.config.planning.waypoint_stop_cycles
            self.emit('waypoint_reached', waypoint=list(waypoint), next_index=self.waypoint_index)
            return
        if self.stop_hold_cycles > 0:
            self.stop_hold_cycles -= 1
            self.stop()
            return

        desired_yaw = math.atan2(wy - y, wx - x)
        yaw_error = _wrap_angle(desired_yaw - yaw)
        command = Twist()

        # Deliberately discrete corridor follower: TURN -> DRIVE -> TURN.  This is
        # not a local-navigation benchmark.  With four-connected A*, every route
        # segment is axis aligned, so carving smooth arcs around corners only makes
        # the physical robot cut into known static walls.
        if self.motion_mode == 'DRIVE' and abs(yaw_error) > self.config.planning.drive_realign_tolerance:
            self.motion_mode = 'TURN'
            self.aligned_cycles = 0
            self.stop()
            self.emit('drive_realign', yaw_error=yaw_error, waypoint=list(waypoint))
            return

        if self.motion_mode == 'TURN':
            if abs(yaw_error) <= self.config.planning.drive_realign_tolerance:
                self.aligned_cycles += 1
                self.stop()
                if self.aligned_cycles >= self.config.planning.turn_settle_cycles:
                    self.motion_mode = 'DRIVE'
                    self.aligned_cycles = 0
                    self.emit('turn_complete', waypoint=list(waypoint), yaw_error=yaw_error)
                return
            self.aligned_cycles = 0
            turn = 2.4 * yaw_error
            command.angular.z = max(-self.config.planning.angular_speed, min(self.config.planning.angular_speed, turn))
        else:
            # DRIVE is intentionally straight.  If heading drift exceeds the small
            # re-alignment tolerance above we stop and rotate again instead of
            # steering an arc through a narrow corridor.
            speed = self.config.planning.linear_speed
            if distance < self.config.planning.corner_slow_distance:
                speed = min(speed, max(self.config.planning.min_linear_speed, 1.2 * distance))
            command.linear.x = speed
            command.angular.z = 0.0

        self.cmd_pub.publish(command)
        if self.control_steps <= 8:
            self.get_logger().info(f'cmd_vel linear={command.linear.x:.3f} angular={command.angular.z:.3f} waypoint={waypoint}')


def main() -> None:
    parser = argparse.ArgumentParser(description='Repeated-delivery A* controller for one physical Gazebo robot.')
    parser.add_argument('--config', required=True)
    parser.add_argument('--robot-id', required=True)
    parser.add_argument('--map-file', default=None)
    args = parser.parse_args()
    rclpy.init()
    node = AStarRobotDriver(Path(args.config).expanduser().resolve(), args.robot_id, args.map_file)
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
