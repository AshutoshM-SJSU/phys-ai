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
from .coordinates import cell_to_world, world_to_cell
from .movingai_map import load_movingai_map
from .paths import resolve_from_config


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _path_overlap(a: list[tuple[int, int]], b: list[tuple[int, int]]) -> float:
    if not a and not b:
        return 1.0
    union = set(a) | set(b)
    return 0.0 if not union else len(set(a) & set(b)) / len(union)


class AStarRobotDriver(Node):
    """A* navigation controller with map-aware local control.

    Important design choices:
      * Gazebo SceneBroadcaster ground truth is the only navigation pose.
        Wheel motion can never advance the route unless the chassis actually moves.
      * A* receives a soft wall-clearance cost so it prefers corridor centers.
      * Lidar is used continuously for front safety and left/right centering.
      * Recovery is backup -> reorient from the corrected current pose -> drive.
      * Visualization events contain only the remaining route and current pose.
    """

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
        seed_offset = sum((i + 1) * ord(ch) for i, ch in enumerate(robot_id))
        self.goal_rng = random.Random(int(config.simulation.seed) * 1009 + seed_offset)
        self.goal_index = int(self.robot.get('initial_goal_index', 0)) % len(self.goal_nodes)
        if self.goal_selection == 'random' and bool(self.robot.get('randomize_initial_goal', True)):
            self.goal_index = self.goal_rng.randrange(len(self.goal_nodes))

        self.start_cell = tuple(int(v) for v in self.robot['start_cell'])
        self.initial_yaw = float(self.robot.get('start_yaw', 0.0))

        # Authoritative physical pose from Gazebo's OdometryPublisher.
        # Gazebo publishes the model pose in its world frame. We anchor that
        # absolute pose once to the configured map start pose, then transform
        # every later absolute odometry sample directly into map coordinates.
        # Nothing is integrated from wheel travel, so a stuck chassis cannot
        # make virtual progress through the route.
        self.pose: tuple[float, float, float] | None = None
        self.have_ground_truth = False
        self.last_ground_truth_time = -1e9
        self.ground_truth_timeout = 0.75
        self.odom_anchor_raw: tuple[float, float, float] | None = None
        self.odom_to_map_yaw = 0.0
        self.map_anchor_xy = cell_to_world(
            *self.start_cell,
            map_height=self.grid.height,
            cell_size=self.config.map.cell_size,
        )

        # Latest lidar samples are used for local safety only, never localization.
        self.scan_points: list[tuple[float, float]] = []
        self.last_scan_time = 0.0
        self.minimum_scan = math.inf
        self.forward_minimum_scan = math.inf
        self.front_left_clearance = math.inf
        self.front_right_clearance = math.inf
        self.left_clearance = math.inf
        self.right_clearance = math.inf
        self.rear_clearance = math.inf

        self.path: list[tuple[int, int]] = []
        self.previous_path: list[tuple[int, int]] = []
        self.waypoint_index = 0
        self.control_steps = 0
        self.dynamic_blocked: set[tuple[int, int]] = set()
        self.pending_map_replan = True

        self.no_path_active_since: float | None = None
        self.delivery_started = 0.0
        self.last_emergency_event = -1e9
        self.last_route_progress_event = -1e9

        # Controller state.
        self.motion_mode = 'TURN'
        self.aligned_cycles = 0
        self.stop_hold_cycles = 0
        self.recovery_mode = 'NONE'  # NONE, BACKUP, TURN
        self.recovery_started = 0.0
        self.recovery_turn_started = 0.0
        self.recovery_attempts = 0

        # Recovery tuning. Conservative because the corridors are narrow.
        self.recovery_backup_speed = min(0.12, float(self.config.planning.linear_speed))
        self.recovery_backup_distance = 0.18
        self.recovery_backup_timeout = 2.5
        self.recovery_backup_origin: tuple[float, float] | None = None
        self.recovery_turn_timeout = 5.0
        self.max_recovery_attempts = 3
        self.rear_stop_distance = max(0.16, float(self.config.planning.emergency_stop_distance) * 0.85)

        # Physical footprint used for conservative lidar clearance.  The generated
        # robot body is 0.30 x 0.22 m and the wheel track extends beyond it.
        self.robot_half_width = 0.16
        self.robot_front_extent = 0.17
        self.footprint_margin = 0.055
        self.safe_front_clearance = self.robot_front_extent + self.footprint_margin
        self.safe_side_clearance = self.robot_half_width + self.footprint_margin

        # Stuck detection uses corrected pose, not wheel travel.
        self.progress_waypoint: tuple[int, int] | None = None
        self.best_waypoint_distance = math.inf
        self.last_progress_time = 0.0
        self.progress_epsilon = 0.025
        self.stuck_timeout = 1.6
        self.progress_grace_until = 0.0

        # Soft wall clearance for global planning. A hard one-cell inflation can
        # accidentally erase legitimate one-cell corridors, so use a cost instead.
        self.clearance_radius_cells = 2
        self.clearance_weight = 2.5

        self.event_pub = self.create_publisher(String, '/experiment/events', 100)
        self.cmd_pub = self.create_publisher(Twist, f'/{robot_id}/cmd_vel', 20)
        self.create_subscription(
            Odometry,
            f'/{robot_id}/odometry',
            self.on_ground_truth_odometry,
            50,
        )
        self.create_subscription(OccupancyGrid, f'/{robot_id}/dynamic_map', self.on_map, 20)
        self.create_subscription(LaserScan, f'/{robot_id}/scan', self.on_scan, 30)
        self.create_timer(1.0 / config.planning.control_rate_hz, self.control)

        self.get_logger().info(
            f'Controller ready: {self.robot_id}, start={self.start_cell}, first_goal={self.current_goal}'
        )
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

    def publish_velocity(self, linear: float = 0.0, angular: float = 0.0) -> None:
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.cmd_pub.publish(command)

    def stop(self) -> None:
        self.publish_velocity()

    # ------------------------------------------------------------------
    # Authoritative Gazebo physical pose
    # ------------------------------------------------------------------

    def on_ground_truth_odometry(self, msg: Odometry) -> None:
        """Transform Gazebo's absolute model pose into map coordinates.

        Gazebo's OdometryPublisher reports the model pose from the simulator
        world.  The Gazebo world axes / origin need not equal the MovingAI map
        axes / origin, so the first physical pose is used only to establish a
        fixed rigid transform to the configured map start pose.

        Every later update is transformed from the *absolute* Gazebo pose.
        We never integrate wheel deltas. Therefore wheel spin against a wall
        cannot move ``self.pose`` or advance waypoints / deliveries.
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        raw_x = float(p.x)
        raw_y = float(p.y)
        raw_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

        if self.odom_anchor_raw is None:
            self.odom_anchor_raw = (raw_x, raw_y, raw_yaw)
            self.odom_to_map_yaw = _wrap_angle(self.initial_yaw - raw_yaw)
            self.pose = (self.map_anchor_xy[0], self.map_anchor_xy[1], self.initial_yaw)
            self.have_ground_truth = True
            self.last_ground_truth_time = self.now()
            self.get_logger().info(
                f'Physical pose anchored for {self.robot_id}: '
                f'gazebo=({raw_x:.3f}, {raw_y:.3f}, {raw_yaw:.3f}) -> '
                f'map=({self.pose[0]:.3f}, {self.pose[1]:.3f}, {self.pose[2]:.3f})'
            )
            return

        anchor_x, anchor_y, _ = self.odom_anchor_raw
        dx = raw_x - anchor_x
        dy = raw_y - anchor_y

        c = math.cos(self.odom_to_map_yaw)
        s = math.sin(self.odom_to_map_yaw)
        map_x = self.map_anchor_xy[0] + c * dx - s * dy
        map_y = self.map_anchor_xy[1] + s * dx + c * dy
        map_yaw = _wrap_angle(raw_yaw + self.odom_to_map_yaw)

        self.pose = (map_x, map_y, map_yaw)
        self.have_ground_truth = True
        self.last_ground_truth_time = self.now()

    # ------------------------------------------------------------------
    # Lidar and map
    # ------------------------------------------------------------------

    def on_scan(self, msg: LaserScan) -> None:
        finite: list[tuple[float, float]] = []
        sectors: dict[str, list[float]] = {
            'front': [], 'front_left': [], 'front_right': [],
            'left': [], 'right': [], 'rear': [],
        }

        for index, value in enumerate(msg.ranges):
            if not (math.isfinite(value) and value > msg.range_min + 0.02 and value <= msg.range_max):
                continue
            angle = _wrap_angle(msg.angle_min + index * msg.angle_increment)
            finite.append((float(value), angle))
            deg = math.degrees(angle)

            if abs(deg) <= 18.0:
                sectors['front'].append(value)
            if 15.0 <= deg <= 65.0:
                sectors['front_left'].append(value)
            if -65.0 <= deg <= -15.0:
                sectors['front_right'].append(value)
            if 55.0 <= deg <= 115.0:
                sectors['left'].append(value)
            if -115.0 <= deg <= -55.0:
                sectors['right'].append(value)
            if abs(abs(deg) - 180.0) <= 30.0:
                sectors['rear'].append(value)

        self.scan_points = finite
        self.last_scan_time = self.now()
        self.minimum_scan = min((r for r, _ in finite), default=math.inf)
        self.forward_minimum_scan = min(sectors['front'], default=math.inf)
        self.front_left_clearance = min(sectors['front_left'], default=math.inf)
        self.front_right_clearance = min(sectors['front_right'], default=math.inf)
        self.left_clearance = min(sectors['left'], default=math.inf)
        self.right_clearance = min(sectors['right'], default=math.inf)
        self.rear_clearance = min(sectors['rear'], default=math.inf)

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

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def overlay_grid(self, current_cell: tuple[int, int]):
        outer = self

        class OverlayGrid:
            width = outer.grid.width
            height = outer.grid.height

            def is_blocked(self, x: int, y: int) -> bool:
                if x < 0 or y < 0 or x >= self.width or y >= self.height:
                    return True
                if (x, y) == current_cell:
                    return outer.grid.is_blocked(x, y)
                return outer.grid.is_blocked(x, y) or (x, y) in outer.dynamic_blocked

        return OverlayGrid()

    def remaining_path(self) -> list[tuple[int, int]]:
        if not self.path or self.waypoint_index >= len(self.path):
            return []
        return self.path[self.waypoint_index:]

    def emit_route_progress(self, force: bool = False) -> None:
        if self.pose is None:
            return
        now = self.now()
        if not force and now - self.last_route_progress_event < 0.35:
            return
        remaining = self.remaining_path()
        self.emit(
            'route_progress',
            pose_world=[self.pose[0], self.pose[1], self.pose[2]],
            remaining_path=[[int(x), int(y)] for x, y in remaining],
            goal=list(self.current_goal),
        )
        self.last_route_progress_event = now

    def reset_progress_tracking(self, waypoint: tuple[int, int] | None = None) -> None:
        self.progress_waypoint = waypoint
        self.best_waypoint_distance = math.inf
        now = self.now()
        self.last_progress_time = now
        self.progress_grace_until = now + 0.75

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
                clearance_radius=self.clearance_radius_cells,
                clearance_weight=self.clearance_weight,
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
                'replan_failed', reason=reason, planning_time_ms=planning_ms,
                start=list(current_cell), goal=list(self.current_goal),
                current_dynamic_blocked=current_cell in self.dynamic_blocked,
                goal_dynamic_blocked=self.current_goal in self.dynamic_blocked,
                dynamic_blocked_count=len(self.dynamic_blocked),
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
        self.recovery_mode = 'NONE'
        self.reset_progress_tracking(self.path[self.waypoint_index] if self.waypoint_index < len(self.path) else None)

        remaining = self.remaining_path()
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
            remaining_path=[[int(cx), int(cy)] for cx, cy in remaining],
            pose_world=[self.pose[0], self.pose[1], self.pose[2]],
        )
        return True

    # ------------------------------------------------------------------
    # Mission handling
    # ------------------------------------------------------------------

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
        self.recovery_mode = 'NONE'
        self.recovery_attempts = 0
        self.reset_progress_tracking()
        self.emit('delivery_started', goal=list(self.current_goal), goal_index=self.goal_index, selection=self.goal_selection)

    # ------------------------------------------------------------------
    # Local control and recovery
    # ------------------------------------------------------------------

    def local_wall_steering(self) -> float:
        """Small angular correction away from nearby side walls."""
        left = min(self.left_clearance, self.front_left_clearance)
        right = min(self.right_clearance, self.front_right_clearance)
        if not math.isfinite(left) or not math.isfinite(right):
            return 0.0

        # Only use wall centering when at least one side is genuinely nearby.
        influence_distance = max(0.36, self.config.planning.emergency_stop_distance * 1.8)
        if min(left, right) > influence_distance:
            return 0.0

        # Positive angular.z turns left. If left wall is closer, steer right.
        error = left - right
        correction = 1.25 * error
        return _clamp(correction, -0.40 * self.config.planning.angular_speed, 0.40 * self.config.planning.angular_speed)

    def begin_recovery(self, reason: str) -> None:
        if self.recovery_mode != 'NONE':
            return
        self.recovery_attempts += 1
        self.recovery_mode = 'BACKUP'
        self.recovery_started = self.now()
        self.recovery_backup_origin = (self.pose[0], self.pose[1]) if self.pose is not None else None
        self.aligned_cycles = 0
        self.emit(
            'recovery_started', reason=reason, attempt=self.recovery_attempts,
            pose_world=list(self.pose) if self.pose is not None else None,
            forward_range=self.forward_minimum_scan,
            rear_range=self.rear_clearance,
        )

    def recovery_control(self) -> bool:
        """Deterministic recovery: back up by measured world distance, replan, turn, drive."""
        if self.pose is None or self.recovery_mode == 'NONE':
            return False

        now = self.now()
        x, y, yaw = self.pose

        if self.recovery_mode == 'BACKUP':
            if self.recovery_backup_origin is None:
                self.recovery_backup_origin = (x, y)

            backed = math.hypot(
                x - self.recovery_backup_origin[0],
                y - self.recovery_backup_origin[1],
            )

            # Stop reversing if the rear footprint is no longer safe.
            if self.rear_clearance <= self.rear_stop_distance:
                self.stop()
                self.emit('recovery_backup_blocked', rear_range=self.rear_clearance, backed_distance=backed)
            elif backed < self.recovery_backup_distance and now - self.recovery_started < self.recovery_backup_timeout:
                self.publish_velocity(-self.recovery_backup_speed, 0.0)
                return True
            else:
                self.stop()
                self.emit('recovery_backup_complete', backed_distance=backed)

            # The robot is now at a different *physical* position. Replan from
            # that ground-truth position before deciding which way to turn.
            if not self.replan('recovery_after_backup'):
                self.recovery_mode = 'NONE'
                return True

            self.recovery_mode = 'TURN'
            self.recovery_turn_started = now
            self.aligned_cycles = 0
            return True

        # TURN always targets the first remaining waypoint of the newly planned
        # route. This prevents stale pre-collision headings from surviving recovery.
        if not self.path or self.waypoint_index >= len(self.path):
            self.stop()
            self.recovery_mode = 'NONE'
            self.pending_map_replan = True
            return True

        waypoint = self.path[self.waypoint_index]
        wx, wy = cell_to_world(
            *waypoint,
            map_height=self.grid.height,
            cell_size=self.config.map.cell_size,
        )
        desired_yaw = math.atan2(wy - y, wx - x)
        yaw_error = _wrap_angle(desired_yaw - yaw)

        if now - self.recovery_turn_started > self.recovery_turn_timeout:
            self.stop()
            self.recovery_mode = 'NONE'
            self.pending_map_replan = True
            self.emit('recovery_turn_timeout', yaw_error=yaw_error)
            return True

        tolerance = self.config.planning.drive_realign_tolerance
        if abs(yaw_error) <= tolerance:
            self.aligned_cycles += 1
            self.stop()
            if self.aligned_cycles >= self.config.planning.turn_settle_cycles:
                self.recovery_mode = 'NONE'
                self.motion_mode = 'DRIVE'
                self.aligned_cycles = 0
                self.reset_progress_tracking(waypoint)
                self.emit(
                    'recovery_completed',
                    attempt=self.recovery_attempts,
                    waypoint=list(waypoint),
                    yaw_error=yaw_error,
                )
            return True

        self.aligned_cycles = 0
        angular = _clamp(
            2.2 * yaw_error,
            -self.config.planning.angular_speed,
            self.config.planning.angular_speed,
        )
        self.publish_velocity(0.0, angular)
        return True

    def update_progress(self, waypoint: tuple[int, int], distance: float) -> bool:
        now = self.now()
        if waypoint != self.progress_waypoint:
            self.reset_progress_tracking(waypoint)
            self.best_waypoint_distance = distance
            return False
        if distance < self.best_waypoint_distance - self.progress_epsilon:
            self.best_waypoint_distance = distance
            self.last_progress_time = now
            return False
        if self.motion_mode != 'DRIVE':
            self.last_progress_time = now
            return False
        if now < self.progress_grace_until:
            return False
        return now - self.last_progress_time >= self.stuck_timeout

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def control(self) -> None:
        if self.pose is None:
            return
        if self.delivery_started == 0.0:
            self.delivery_started = self.now()

        self.control_steps += 1

        # Navigation is forbidden without fresh physical ground truth.
        if not self.have_ground_truth or self.now() - self.last_ground_truth_time > self.ground_truth_timeout:
            self.stop()
            return

        if self.recovery_mode == 'NONE':
            scheduled = self.control_steps % self.config.planning.replan_every_steps == 0
            if not self.path or scheduled or self.pending_map_replan:
                reason = 'map_change' if self.pending_map_replan else ('scheduled' if scheduled else 'initial')
                if not self.replan(reason):
                    self.stop()
                    return

        x, y, yaw = self.pose
        goal_x, goal_y = cell_to_world(
            *self.current_goal, map_height=self.grid.height, cell_size=self.config.map.cell_size
        )
        if math.hypot(goal_x - x, goal_y - y) <= self.config.planning.goal_tolerance:
            self.stop()
            self.complete_delivery()
            return

        if self.waypoint_index >= len(self.path):
            self.path = []
            self.stop()
            return

        waypoint = self.path[self.waypoint_index]
        wx, wy = cell_to_world(*waypoint, map_height=self.grid.height, cell_size=self.config.map.cell_size)
        distance = math.hypot(wx - x, wy - y)

        if self.recovery_mode != 'NONE':
            self.recovery_control()
            return

        if distance <= self.config.planning.waypoint_tolerance:
            self.stop()
            self.waypoint_index += 1
            self.motion_mode = 'TURN'
            self.aligned_cycles = 0
            self.stop_hold_cycles = self.config.planning.waypoint_stop_cycles
            next_wp = self.path[self.waypoint_index] if self.waypoint_index < len(self.path) else None
            self.reset_progress_tracking(next_wp)
            self.emit('waypoint_reached', waypoint=list(waypoint), next_index=self.waypoint_index)
            self.emit_route_progress(force=True)
            return

        if self.stop_hold_cycles > 0:
            self.stop_hold_cycles -= 1
            self.stop()
            return

        # A real front obstruction has priority over path following.
        emergency_front = max(self.config.planning.emergency_stop_distance, self.safe_front_clearance)
        corner_clearance = min(self.front_left_clearance, self.front_right_clearance)
        wheel_corner_risk = corner_clearance < self.safe_side_clearance
        if self.forward_minimum_scan < emergency_front or wheel_corner_risk:
            self.stop()
            now = self.now()
            if now - self.last_emergency_event > 0.5:
                self.emit(
                    'emergency_stop',
                    minimum_range=self.forward_minimum_scan,
                    corner_clearance=corner_clearance,
                    response='backup_replan_turn_recover',
                )
                self.last_emergency_event = now
            self.begin_recovery('forward_obstruction')
            return

        if self.update_progress(waypoint, distance):
            self.stop()
            self.emit('stuck_detected', waypoint=list(waypoint), distance_to_waypoint=distance)
            if self.recovery_attempts >= self.max_recovery_attempts:
                self.recovery_attempts = 0
                self.pending_map_replan = True
                self.reset_progress_tracking(waypoint)
                return
            self.begin_recovery('no_physical_progress')
            return

        desired_yaw = math.atan2(wy - y, wx - x)
        yaw_error = _wrap_angle(desired_yaw - yaw)

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
                    self.reset_progress_tracking(waypoint)
                    self.emit('turn_complete', waypoint=list(waypoint), yaw_error=yaw_error)
                return
            self.aligned_cycles = 0
            angular = _clamp(2.2 * yaw_error, -self.config.planning.angular_speed, self.config.planning.angular_speed)
            self.publish_velocity(0.0, angular)
            return

        # DRIVE: follow the global heading while lidar gently centers the chassis.
        speed = float(self.config.planning.linear_speed)
        if distance < self.config.planning.corner_slow_distance:
            speed = min(speed, max(self.config.planning.min_linear_speed, 1.1 * distance))

        # Slow before we reach the emergency-stop distance so inertia does not
        # carry a wheel into the corner.
        slow_distance = max(emergency_front * 1.8, 0.42)
        if self.forward_minimum_scan < slow_distance:
            scale = _clamp(
                (self.forward_minimum_scan - emergency_front)
                / max(0.05, slow_distance - emergency_front),
                0.25,
                1.0,
            )
            speed *= scale

        heading_correction = _clamp(1.3 * yaw_error, -0.35 * self.config.planning.angular_speed, 0.35 * self.config.planning.angular_speed)
        wall_correction = self.local_wall_steering()
        angular = _clamp(
            heading_correction + wall_correction,
            -0.55 * self.config.planning.angular_speed,
            0.55 * self.config.planning.angular_speed,
        )
        self.publish_velocity(speed, angular)
        self.emit_route_progress()


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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
