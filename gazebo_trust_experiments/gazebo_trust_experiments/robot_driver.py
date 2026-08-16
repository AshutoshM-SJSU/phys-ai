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


def _yaw_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _path_overlap(
    a: list[tuple[int, int]],
    b: list[tuple[int, int]],
) -> float:
    if not a and not b:
        return 1.0

    union = set(a) | set(b)
    if not union:
        return 0.0

    return len(set(a) & set(b)) / len(union)


class AStarRobotDriver(Node):
    """
    Physical A* path follower for one Gazebo robot.

    Normal motion:
        TURN -> DRIVE

    Recovery motion:
        DRIVE
          -> BACKUP
          -> RECOVERY_TURN
          -> DRIVE

    Recovery is triggered when:
      * forward lidar reports an obstruction, or
      * the robot commands forward motion but makes insufficient progress
        toward its current waypoint.

    The recovery controller intentionally backs away before rotating. This is
    useful in narrow corridors where a wheel or chassis corner can become
    physically wedged against Gazebo collision geometry.
    """

    def __init__(
        self,
        config_path: Path,
        robot_id: str,
        map_override: str | None = None,
    ) -> None:
        super().__init__(f"{robot_id}_astar_driver")

        self.set_parameters(
            [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
        )

        config = apply_overrides(
            load_config(config_path),
            map_file=map_override,
        )

        self.config = config
        self.grid = load_movingai_map(
            resolve_from_config(config.map.file, config_path)
        )

        matches = [
            robot
            for robot in config.robots
            if str(robot.get("id")) == robot_id
        ]

        if len(matches) != 1:
            raise ValueError(
                f"Expected exactly one robot named '{robot_id}'"
            )

        self.robot = matches[0]
        self.robot_id = robot_id

        # ------------------------------------------------------------
        # Mission / routing state
        # ------------------------------------------------------------

        self.goal_nodes = [
            tuple(int(v) for v in goal)
            for goal in self.robot["goal_nodes"]
        ]

        self.goal_selection = str(
            self.robot.get("goal_selection", "random")
        ).lower()

        seed_offset = sum(
            (i + 1) * ord(ch)
            for i, ch in enumerate(robot_id)
        )

        self.goal_rng = random.Random(
            int(config.simulation.seed) * 1009 + seed_offset
        )

        self.goal_index = (
            int(self.robot.get("initial_goal_index", 0))
            % len(self.goal_nodes)
        )

        if (
            self.goal_selection == "random"
            and bool(
                self.robot.get(
                    "randomize_initial_goal",
                    True,
                )
            )
        ):
            self.goal_index = self.goal_rng.randrange(
                len(self.goal_nodes)
            )

        self.start_cell = tuple(
            int(v)
            for v in self.robot["start_cell"]
        )

        self.initial_yaw = float(
            self.robot.get("start_yaw", 0.0)
        )

        self.pose: tuple[float, float, float] | None = None

        # Gazebo Harmonic may publish odometry in a world-fixed frame rather
        # than a robot-relative frame starting at (0, 0, 0).  Anchor the first
        # odometry sample to the configured map start pose and thereafter use
        # only odometry deltas.  This keeps Gazebo world coordinates, map-world
        # coordinates, A* cells, and heading calculations in one consistent
        # frame even when Gazebo's global origin is elsewhere in the world.
        self.odom_origin: tuple[float, float, float] | None = None

        self.path: list[tuple[int, int]] = []
        self.previous_path: list[tuple[int, int]] = []

        self.waypoint_index = 0
        self.control_steps = 0

        self.dynamic_blocked: set[tuple[int, int]] = set()

        # ------------------------------------------------------------
        # Lidar state
        # ------------------------------------------------------------

        self.minimum_scan = math.inf
        self.minimum_scan_angle = 0.0

        self.forward_minimum_scan = math.inf
        self.rear_minimum_scan = math.inf

        self.last_emergency_event = -1e9

        # ------------------------------------------------------------
        # Experiment state
        # ------------------------------------------------------------

        self.no_path_active_since: float | None = None

        self.delivery_started = 0.0
        self.last_motion_heading: float | None = None

        self.pending_map_replan = True

        # ------------------------------------------------------------
        # Normal motion state
        # ------------------------------------------------------------

        self.motion_mode = "TURN"

        self.aligned_cycles = 0
        self.stop_hold_cycles = 0

        # ------------------------------------------------------------
        # Recovery controller
        # ------------------------------------------------------------

        # Recovery modes:
        #
        # NONE
        # BACKUP
        # TURN
        #
        self.recovery_mode = "NONE"

        self.recovery_started = 0.0
        self.recovery_turn_started = 0.0

        self.recovery_attempts = 0

        #
        # These deliberately live here rather than in the YAML for the
        # first test. Once we know the behavior works, they can become
        # experiment configuration parameters.
        #

        # Seconds spent reversing after becoming stuck.
        self.recovery_backup_seconds = 0.65

        # Backward velocity.
        self.recovery_backup_speed = 0.13

        # Allow a recovery turn to continue for this many seconds before
        # giving up and forcing a replan.
        self.recovery_turn_timeout = 4.0

        # Maximum sequential recovery attempts before requesting a fresh
        # A* path.
        self.max_recovery_attempts = 3

        # Rear clearance threshold while backing.
        self.recovery_rear_stop_distance = max(
            0.12,
            self.config.planning.emergency_stop_distance * 0.8,
        )

        # ------------------------------------------------------------
        # Stuck detection
        # ------------------------------------------------------------

        # The robot must make at least this much progress toward the current
        # waypoint to reset the stuck timer.
        self.progress_epsilon = 0.035

        # How long forward motion may fail to make progress before the
        # recovery sequence begins.
        self.stuck_timeout = 1.35

        self.progress_waypoint: tuple[int, int] | None = None

        self.best_waypoint_distance = math.inf
        self.last_progress_time = 0.0

        # Ignore stuck detection briefly after a recovery or waypoint change.
        self.progress_grace_until = 0.0

        # ------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------

        self.event_pub = self.create_publisher(
            String,
            "/experiment/events",
            100,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            f"/{robot_id}/cmd_vel",
            20,
        )

        self.create_subscription(
            Odometry,
            f"/{robot_id}/odometry",
            self.on_odom,
            30,
        )

        self.create_subscription(
            OccupancyGrid,
            f"/{robot_id}/dynamic_map",
            self.on_map,
            20,
        )

        self.create_subscription(
            LaserScan,
            f"/{robot_id}/scan",
            self.on_scan,
            30,
        )

        self.create_timer(
            1.0 / config.planning.control_rate_hz,
            self.control,
        )

        self.get_logger().info(
            f"Controller ready: {self.robot_id}, "
            f"start={self.start_cell}, "
            f"first_goal={self.current_goal}"
        )

        self.emit(
            "mission_started",
            goal=list(self.current_goal),
            role=self.robot.get("role", "robot"),
        )

    # -----------------------------------------------------------------
    # Utility
    # -----------------------------------------------------------------

    @property
    def current_goal(self) -> tuple[int, int]:
        return self.goal_nodes[self.goal_index]

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def emit(
        self,
        event_type: str,
        **details,
    ) -> None:
        msg = String()

        msg.data = json.dumps(
            {
                "event_type": event_type,
                "sim_time": self.now(),
                "robot_id": self.robot_id,
                "details": details,
            },
            separators=(",", ":"),
        )

        self.event_pub.publish(msg)

    def publish_velocity(
        self,
        linear: float = 0.0,
        angular: float = 0.0,
    ) -> None:
        command = Twist()

        command.linear.x = float(linear)
        command.angular.z = float(angular)

        self.cmd_pub.publish(command)

    def stop(self) -> None:
        self.publish_velocity()

    # -----------------------------------------------------------------
    # Sensors
    # -----------------------------------------------------------------

    def on_scan(
        self,
        msg: LaserScan,
    ) -> None:
        finite: list[tuple[float, float]] = []

        forward: list[float] = []
        rear: list[float] = []

        half_angle = (
            self.config.planning.emergency_forward_half_angle
        )

        for index, value in enumerate(msg.ranges):

            if not (
                math.isfinite(value)
                and value > msg.range_min + 0.02
                and value <= msg.range_max
            ):
                continue

            angle = _wrap_angle(
                msg.angle_min
                + index * msg.angle_increment
            )

            finite.append((value, angle))

            if abs(angle) <= half_angle:
                forward.append(value)

            rear_error = abs(
                abs(angle) - math.pi
            )

            if rear_error <= half_angle:
                rear.append(value)

        if finite:
            self.minimum_scan, self.minimum_scan_angle = min(
                finite,
                key=lambda item: item[0],
            )
        else:
            self.minimum_scan = math.inf
            self.minimum_scan_angle = 0.0

        self.forward_minimum_scan = (
            min(forward)
            if forward
            else math.inf
        )

        self.rear_minimum_scan = (
            min(rear)
            if rear
            else math.inf
        )

        if (
            self.minimum_scan
            < self.config.planning.near_collision_distance
        ):
            now = self.now()

            if now - self.last_emergency_event > 0.5:

                self.emit(
                    "near_collision",
                    minimum_range=self.minimum_scan,
                )

                self.last_emergency_event = now

    def on_map(
        self,
        msg: OccupancyGrid,
    ) -> None:
        blocked: set[tuple[int, int]] = set()

        threshold = int(
            self.config.mapping.get(
                "occupied_threshold",
                50,
            )
        )

        for row in range(msg.info.height):

            source_y = (
                self.grid.height - 1 - row
            )

            for x in range(msg.info.width):

                value = msg.data[
                    row * msg.info.width + x
                ]

                if (
                    value >= threshold
                    and not self.grid.is_blocked(
                        x,
                        source_y,
                    )
                ):
                    blocked.add(
                        (x, source_y)
                    )

        if blocked != self.dynamic_blocked:

            self.dynamic_blocked = blocked
            self.pending_map_replan = True

    def on_odom(
        self,
        msg: Odometry,
    ) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        odom_yaw = _yaw_from_quaternion(
            q.x,
            q.y,
            q.z,
            q.w,
        )

        # The first odometry message defines Gazebo's pose for the configured
        # robot start position.  The controller cannot move before self.pose is
        # initialized, so this sample is a safe motion-free anchor.
        if self.odom_origin is None:
            self.odom_origin = (
                float(p.x),
                float(p.y),
                float(odom_yaw),
            )

            start_x, start_y = cell_to_world(
                *self.start_cell,
                map_height=self.grid.height,
                cell_size=self.config.map.cell_size,
            )

            self.pose = (
                start_x,
                start_y,
                self.initial_yaw,
            )

            self.get_logger().info(
                f"Odometry anchored for {self.robot_id}: "
                f"gazebo=({p.x:.3f}, {p.y:.3f}, {odom_yaw:.3f}) -> "
                f"map=({start_x:.3f}, {start_y:.3f}, {self.initial_yaw:.3f})"
            )

            self.emit(
                "localization_initialized",
                gazebo_pose=[float(p.x), float(p.y), float(odom_yaw)],
                map_pose=[float(start_x), float(start_y), float(self.initial_yaw)],
                start_cell=list(self.start_cell),
            )

            return

        origin_x, origin_y, origin_yaw = self.odom_origin

        # Translation since spawn, expressed in the Gazebo world frame.
        delta_x = float(p.x) - origin_x
        delta_y = float(p.y) - origin_y

        # Rotate that world-frame displacement into the map-world frame.
        # This is the fixed transform that maps Gazebo's initial robot heading
        # onto the configured start_yaw.
        frame_rotation = _wrap_angle(
            self.initial_yaw - origin_yaw
        )

        c = math.cos(frame_rotation)
        s = math.sin(frame_rotation)

        start_x, start_y = cell_to_world(
            *self.start_cell,
            map_height=self.grid.height,
            cell_size=self.config.map.cell_size,
        )

        world_x = (
            start_x
            + c * delta_x
            - s * delta_y
        )

        world_y = (
            start_y
            + s * delta_x
            + c * delta_y
        )

        world_yaw = _wrap_angle(
            self.initial_yaw
            + _wrap_angle(odom_yaw - origin_yaw)
        )

        self.pose = (
            world_x,
            world_y,
            world_yaw,
        )

    # -----------------------------------------------------------------
    # Planning
    # -----------------------------------------------------------------

    def overlay_grid(
        self,
        current_cell: tuple[int, int],
    ):
        outer = self

        class OverlayGrid:

            width = outer.grid.width
            height = outer.grid.height

            def is_blocked(
                self,
                x: int,
                y: int,
            ) -> bool:

                # Never let stale dynamic evidence mark the robot's own
                # current cell as unusable.
                if (x, y) == current_cell:
                    return outer.grid.is_blocked(
                        x,
                        y,
                    )

                return (
                    outer.grid.is_blocked(
                        x,
                        y,
                    )
                    or (x, y)
                    in outer.dynamic_blocked
                )

        return OverlayGrid()

    def reset_progress_tracking(
        self,
        waypoint: tuple[int, int] | None = None,
    ) -> None:
        self.progress_waypoint = waypoint
        self.best_waypoint_distance = math.inf

        now = self.now()

        self.last_progress_time = now

        # Brief grace period prevents immediately declaring the robot stuck
        # while it is settling after a stop or turn.
        self.progress_grace_until = now + 0.6

    def replan(
        self,
        reason: str,
    ) -> bool:
        if self.pose is None:
            return False

        current_cell = world_to_cell(
            self.pose[0],
            self.pose[1],
            map_height=self.grid.height,
            cell_size=self.config.map.cell_size,
        )

        started = time.perf_counter()

        try:

            new_path = astar(
                self.overlay_grid(current_cell),
                current_cell,
                self.current_goal,
                allow_diagonal=(
                    self.config.planning.allow_diagonal
                ),
            )

        except ValueError as exc:

            self.get_logger().error(
                str(exc)
            )

            new_path = []

        planning_ms = (
            time.perf_counter() - started
        ) * 1000.0

        if not new_path:

            if self.no_path_active_since is None:

                self.no_path_active_since = (
                    self.now()
                )

                self.emit(
                    "no_path_started",
                    start=list(current_cell),
                    goal=list(self.current_goal),
                )

            self.emit(
                "replan_failed",
                reason=reason,
                planning_time_ms=planning_ms,
                start=list(current_cell),
                goal=list(self.current_goal),
                current_dynamic_blocked=(
                    current_cell
                    in self.dynamic_blocked
                ),
                goal_dynamic_blocked=(
                    self.current_goal
                    in self.dynamic_blocked
                ),
                dynamic_blocked_count=len(
                    self.dynamic_blocked
                ),
            )

            self.get_logger().warning(
                f"No A* path: "
                f"start={current_cell} "
                f"goal={self.current_goal} "
                f"dynamic_blocked_count="
                f"{len(self.dynamic_blocked)}"
            )

            self.path = []

            return False

        if self.no_path_active_since is not None:

            self.emit(
                "no_path_ended",
                duration=(
                    self.now()
                    - self.no_path_active_since
                ),
            )

            self.no_path_active_since = None

        overlap = _path_overlap(
            self.path,
            new_path,
        )

        old_heading = None
        new_heading = None

        if len(self.path) >= 2:

            old_heading = math.atan2(
                self.path[1][1]
                - self.path[0][1],
                self.path[1][0]
                - self.path[0][0],
            )

        if len(new_path) >= 2:

            new_heading = math.atan2(
                new_path[1][1]
                - new_path[0][1],
                new_path[1][0]
                - new_path[0][0],
            )

        reversal = False

        if (
            old_heading is not None
            and new_heading is not None
        ):

            threshold = math.radians(
                float(
                    self.config.metrics.get(
                        "reversal_heading_threshold_degrees",
                        120.0,
                    )
                )
            )

            reversal = abs(
                _wrap_angle(
                    new_heading
                    - old_heading
                )
            ) >= threshold

        self.previous_path = self.path

        self.path = new_path

        self.waypoint_index = (
            1
            if len(new_path) > 1
            else 0
        )

        self.pending_map_replan = False

        self.motion_mode = "TURN"

        self.aligned_cycles = 0

        self.stop_hold_cycles = (
            self.config.planning.waypoint_stop_cycles
        )

        self.recovery_mode = "NONE"
        self.recovery_attempts = 0

        waypoint = (
            new_path[self.waypoint_index]
            if self.waypoint_index < len(new_path)
            else None
        )

        self.reset_progress_tracking(
            waypoint
        )

        self.emit(
            "replan",
            reason=reason,
            start=list(current_cell),
            goal=list(self.current_goal),
            path_cells=len(new_path),
            path_distance=(
                max(
                    0,
                    len(new_path) - 1,
                )
                * self.config.map.cell_size
            ),
            planning_time_ms=planning_ms,
            route_overlap=overlap,
            route_reversal=reversal,
            path=[
                [int(cx), int(cy)]
                for cx, cy in new_path
            ],
        )

        return True

    # -----------------------------------------------------------------
    # Delivery handling
    # -----------------------------------------------------------------

    def complete_delivery(self) -> None:

        elapsed = (
            self.now()
            - self.delivery_started
        )

        completed_goal = self.current_goal

        self.emit(
            "delivery_completed",
            goal=list(completed_goal),
            delivery_time=elapsed,
        )

        previous_index = self.goal_index

        if (
            self.goal_selection == "random"
            and len(self.goal_nodes) > 1
        ):

            candidates = [
                i
                for i in range(
                    len(self.goal_nodes)
                )
                if i != previous_index
            ]

            self.goal_index = (
                self.goal_rng.choice(
                    candidates
                )
            )

        else:

            self.goal_index = (
                self.goal_index + 1
            ) % len(self.goal_nodes)

        self.delivery_started = self.now()

        self.path = []

        self.pending_map_replan = True

        self.motion_mode = "TURN"

        self.aligned_cycles = 0

        self.stop_hold_cycles = (
            self.config.planning.waypoint_stop_cycles
        )

        self.recovery_mode = "NONE"
        self.recovery_attempts = 0

        self.reset_progress_tracking()

        self.emit(
            "delivery_started",
            goal=list(self.current_goal),
            goal_index=self.goal_index,
            selection=self.goal_selection,
        )

    # -----------------------------------------------------------------
    # Recovery
    # -----------------------------------------------------------------

    def begin_recovery(
        self,
        reason: str,
    ) -> None:

        # Don't restart an already-active recovery sequence.
        if self.recovery_mode != "NONE":
            return

        self.recovery_attempts += 1

        self.recovery_mode = "BACKUP"

        self.recovery_started = self.now()

        self.motion_mode = "TURN"
        self.aligned_cycles = 0

        self.get_logger().warning(
            f"{self.robot_id}: recovery "
            f"attempt {self.recovery_attempts}: "
            f"{reason}"
        )

        waypoint = None

        if (
            self.path
            and self.waypoint_index
            < len(self.path)
        ):
            waypoint = self.path[
                self.waypoint_index
            ]

        self.emit(
            "recovery_started",
            reason=reason,
            attempt=self.recovery_attempts,
            waypoint=(
                list(waypoint)
                if waypoint is not None
                else None
            ),
            forward_range=(
                self.forward_minimum_scan
            ),
            rear_range=(
                self.rear_minimum_scan
            ),
        )

    def finish_recovery(self) -> None:

        self.recovery_mode = "NONE"

        self.motion_mode = "DRIVE"

        self.aligned_cycles = 0

        self.reset_progress_tracking(
            self.path[self.waypoint_index]
            if (
                self.path
                and self.waypoint_index
                < len(self.path)
            )
            else None
        )

        self.progress_grace_until = (
            self.now() + 1.0
        )

        self.emit(
            "recovery_completed",
            attempt=self.recovery_attempts,
        )

    def recovery_control(
        self,
        waypoint: tuple[int, int],
        wx: float,
        wy: float,
    ) -> bool:
        """
        Execute one recovery-control cycle.

        Returns True whenever recovery owns cmd_vel for this cycle.
        """

        if (
            self.recovery_mode == "NONE"
            or self.pose is None
        ):
            return False

        now = self.now()

        x, y, yaw = self.pose

        # -------------------------------------------------------------
        # Stage 1: BACKUP
        # -------------------------------------------------------------

        if self.recovery_mode == "BACKUP":

            elapsed = (
                now
                - self.recovery_started
            )

            # Do not reverse into another wall.
            if (
                self.rear_minimum_scan
                <= self.recovery_rear_stop_distance
            ):
                self.stop()

                self.recovery_mode = "TURN"
                self.recovery_turn_started = now

                self.emit(
                    "recovery_backup_blocked",
                    rear_range=(
                        self.rear_minimum_scan
                    ),
                )

                return True

            if (
                elapsed
                < self.recovery_backup_seconds
            ):

                self.publish_velocity(
                    linear=-self.recovery_backup_speed,
                    angular=0.0,
                )

                return True

            self.stop()

            self.recovery_mode = "TURN"
            self.recovery_turn_started = now

            self.emit(
                "recovery_backup_complete",
                elapsed=elapsed,
            )

            return True

        # -------------------------------------------------------------
        # Stage 2: TURN toward current A* waypoint
        # -------------------------------------------------------------

        desired_yaw = math.atan2(
            wy - y,
            wx - x,
        )

        yaw_error = _wrap_angle(
            desired_yaw - yaw
        )

        if (
            now
            - self.recovery_turn_started
            > self.recovery_turn_timeout
        ):

            self.stop()

            self.emit(
                "recovery_turn_timeout",
                yaw_error=yaw_error,
            )

            self.recovery_mode = "NONE"

            self.pending_map_replan = True

            self.reset_progress_tracking(
                waypoint
            )

            return True

        tolerance = (
            self.config.planning.drive_realign_tolerance
        )

        if abs(yaw_error) <= tolerance:

            self.aligned_cycles += 1

            self.stop()

            if (
                self.aligned_cycles
                >= self.config.planning.turn_settle_cycles
            ):

                self.finish_recovery()

            return True

        self.aligned_cycles = 0

        turn_command = 2.6 * yaw_error

        angular = _clamp(
            turn_command,
            -self.config.planning.angular_speed,
            self.config.planning.angular_speed,
        )

        self.publish_velocity(
            linear=0.0,
            angular=angular,
        )

        return True

    # -----------------------------------------------------------------
    # Stuck detection
    # -----------------------------------------------------------------

    def update_progress(
        self,
        waypoint: tuple[int, int],
        distance: float,
    ) -> bool:
        """
        Return True if the robot appears physically stuck while driving.
        """

        now = self.now()

        if waypoint != self.progress_waypoint:

            self.reset_progress_tracking(
                waypoint
            )

            self.best_waypoint_distance = distance

            return False

        if (
            distance
            < self.best_waypoint_distance
            - self.progress_epsilon
        ):

            self.best_waypoint_distance = distance
            self.last_progress_time = now

            return False

        # Do not call normal TURN / settling behavior "stuck".
        if self.motion_mode != "DRIVE":
            self.last_progress_time = now
            return False

        if now < self.progress_grace_until:
            return False

        if (
            now - self.last_progress_time
            >= self.stuck_timeout
        ):
            return True

        return False

    # -----------------------------------------------------------------
    # Main control loop
    # -----------------------------------------------------------------

    def control(self) -> None:

        if self.pose is None:
            return

        if self.delivery_started == 0.0:
            self.delivery_started = self.now()

        self.control_steps += 1

        # Don't let ordinary scheduled replanning interrupt a physical
        # recovery maneuver.
        scheduled = (
            self.recovery_mode == "NONE"
            and self.control_steps
            % self.config.planning.replan_every_steps
            == 0
        )

        if (
            not self.path
            or scheduled
            or (
                self.pending_map_replan
                and self.recovery_mode == "NONE"
            )
        ):

            if self.pending_map_replan:
                reason = "map_change"

            elif scheduled:
                reason = "scheduled"

            else:
                reason = "initial"

            if not self.replan(reason):

                self.stop()
                return

        x, y, yaw = self.pose

        # -------------------------------------------------------------
        # Goal completion
        # -------------------------------------------------------------

        goal_x, goal_y = cell_to_world(
            *self.current_goal,
            map_height=self.grid.height,
            cell_size=self.config.map.cell_size,
        )

        if (
            math.hypot(
                goal_x - x,
                goal_y - y,
            )
            <= self.config.planning.goal_tolerance
        ):

            self.stop()
            self.complete_delivery()

            return

        if self.waypoint_index >= len(self.path):

            self.path = []
            self.stop()

            return

        # -------------------------------------------------------------
        # Current waypoint
        # -------------------------------------------------------------

        waypoint = self.path[
            self.waypoint_index
        ]

        wx, wy = cell_to_world(
            *waypoint,
            map_height=self.grid.height,
            cell_size=self.config.map.cell_size,
        )

        distance = math.hypot(
            wx - x,
            wy - y,
        )

        # -------------------------------------------------------------
        # Active recovery gets first control priority.
        # -------------------------------------------------------------

        if self.recovery_mode != "NONE":

            self.recovery_control(
                waypoint,
                wx,
                wy,
            )

            return

        # -------------------------------------------------------------
        # Waypoint reached
        # -------------------------------------------------------------

        if (
            distance
            <= self.config.planning.waypoint_tolerance
        ):

            self.stop()

            self.waypoint_index += 1

            self.motion_mode = "TURN"

            self.aligned_cycles = 0

            self.stop_hold_cycles = (
                self.config.planning.waypoint_stop_cycles
            )

            next_waypoint = (
                self.path[self.waypoint_index]
                if self.waypoint_index
                < len(self.path)
                else None
            )

            self.reset_progress_tracking(
                next_waypoint
            )

            self.emit(
                "waypoint_reached",
                waypoint=list(waypoint),
                next_index=self.waypoint_index,
            )

            return

        # -------------------------------------------------------------
        # Hold after waypoint / replan
        # -------------------------------------------------------------

        if self.stop_hold_cycles > 0:

            self.stop_hold_cycles -= 1

            self.stop()

            return

        # -------------------------------------------------------------
        # Detect physical obstruction.
        #
        # Rather than repeatedly doing:
        #
        #     stop -> replan -> same path -> stop
        #
        # first physically free the robot from the corner.
        # -------------------------------------------------------------

        if (
            self.forward_minimum_scan
            < self.config.planning.emergency_stop_distance
        ):

            now = self.now()

            if (
                now
                - self.last_emergency_event
                > 0.5
            ):

                self.emit(
                    "emergency_stop",
                    minimum_range=(
                        self.forward_minimum_scan
                    ),
                    response=(
                        "backup_turn_recover"
                    ),
                )

                self.last_emergency_event = now

            self.stop()

            self.begin_recovery(
                "forward_obstruction"
            )

            return

        # -------------------------------------------------------------
        # Detect wheel / chassis wedging.
        # -------------------------------------------------------------

        if self.update_progress(
            waypoint,
            distance,
        ):

            self.stop()

            self.emit(
                "stuck_detected",
                waypoint=list(waypoint),
                distance_to_waypoint=distance,
                no_progress_seconds=(
                    self.now()
                    - self.last_progress_time
                ),
            )

            if (
                self.recovery_attempts
                >= self.max_recovery_attempts
            ):

                self.get_logger().warning(
                    f"{self.robot_id}: "
                    "recovery limit reached; "
                    "forcing A* replan"
                )

                self.recovery_attempts = 0

                self.pending_map_replan = True

                self.reset_progress_tracking(
                    waypoint
                )

                return

            self.begin_recovery(
                "no_forward_progress"
            )

            return

        # -------------------------------------------------------------
        # Desired heading
        # -------------------------------------------------------------

        desired_yaw = math.atan2(
            wy - y,
            wx - x,
        )

        yaw_error = _wrap_angle(
            desired_yaw - yaw
        )

        # -------------------------------------------------------------
        # DRIVE -> TURN if heading has drifted.
        # -------------------------------------------------------------

        if (
            self.motion_mode == "DRIVE"
            and abs(yaw_error)
            > self.config.planning.drive_realign_tolerance
        ):

            self.motion_mode = "TURN"

            self.aligned_cycles = 0

            self.stop()

            self.emit(
                "drive_realign",
                yaw_error=yaw_error,
                waypoint=list(waypoint),
            )

            return

        # -------------------------------------------------------------
        # TURN
        # -------------------------------------------------------------

        if self.motion_mode == "TURN":

            if (
                abs(yaw_error)
                <= self.config.planning.drive_realign_tolerance
            ):

                self.aligned_cycles += 1

                self.stop()

                if (
                    self.aligned_cycles
                    >= self.config.planning.turn_settle_cycles
                ):

                    self.motion_mode = "DRIVE"

                    self.aligned_cycles = 0

                    self.reset_progress_tracking(
                        waypoint
                    )

                    self.emit(
                        "turn_complete",
                        waypoint=list(waypoint),
                        yaw_error=yaw_error,
                    )

                return

            self.aligned_cycles = 0

            turn = 2.4 * yaw_error

            angular = _clamp(
                turn,
                -self.config.planning.angular_speed,
                self.config.planning.angular_speed,
            )

            self.publish_velocity(
                linear=0.0,
                angular=angular,
            )

            return

        # -------------------------------------------------------------
        # DRIVE
        #
        # Keep this deliberately straight. Narrow corridors are more
        # predictable when orientation correction happens while stopped.
        # -------------------------------------------------------------

        speed = (
            self.config.planning.linear_speed
        )

        if (
            distance
            < self.config.planning.corner_slow_distance
        ):

            speed = min(
                speed,
                max(
                    self.config.planning.min_linear_speed,
                    1.2 * distance,
                ),
            )

        self.publish_velocity(
            linear=speed,
            angular=0.0,
        )

        if self.control_steps <= 8:

            self.get_logger().info(
                f"cmd_vel "
                f"linear={speed:.3f} "
                f"angular=0.000 "
                f"waypoint={waypoint}"
            )


def main() -> None:

    parser = argparse.ArgumentParser(
        description=(
            "Repeated-delivery A* controller "
            "for one physical Gazebo robot."
        )
    )

    parser.add_argument(
        "--config",
        required=True,
    )

    parser.add_argument(
        "--robot-id",
        required=True,
    )

    parser.add_argument(
        "--map-file",
        default=None,
    )

    args = parser.parse_args()

    rclpy.init()

    node = AStarRobotDriver(
        Path(args.config)
        .expanduser()
        .resolve(),
        args.robot_id,
        args.map_file,
    )

    try:

        rclpy.spin(node)

    finally:

        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
