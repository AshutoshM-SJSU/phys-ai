from __future__ import annotations

import argparse
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from gazebo_trust_experiments.coordinates import odom_to_world_pose, world_to_cell
from gazebo_trust_experiments.models import Claim
from .common import load_runtime


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _ray_cells(
    origin_x: float,
    origin_y: float,
    angle: float,
    distance: float,
    *,
    map_height: int,
    cell_size: float,
    step: float,
) -> list[tuple[int, int]]:
    cells: list[tuple[int, int]] = []
    samples = max(1, int(distance / step))
    for index in range(1, samples + 1):
        d = min(distance, index * step)
        cell = world_to_cell(
            origin_x + d * math.cos(angle),
            origin_y + d * math.sin(angle),
            map_height=map_height,
            cell_size=cell_size,
        )
        if not cells or cells[-1] != cell:
            cells.append(cell)
    return cells


class LidarReporter(Node):
    def __init__(self, config_path: str, robot_id: str) -> None:
        super().__init__(f'{robot_id}_lidar_reporter')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.path, self.cfg, self.grid = load_runtime(config_path)
        self.robot_id = robot_id
        matches = [robot for robot in self.cfg.robots if str(robot.get('id')) == robot_id]
        if len(matches) != 1:
            raise ValueError(f"Expected exactly one robot named '{robot_id}'")
        self.robot = matches[0]
        self.start_cell = tuple(int(v) for v in self.robot['start_cell'])
        self.initial_yaw = float(self.robot.get('start_yaw', 0.0))
        self.pose: tuple[float, float, float] | None = None
        self.last_sent: dict[tuple[tuple[int, int], str], float] = {}
        self.pub = self.create_publisher(String, '/claims/raw', 200)
        self.create_subscription(Odometry, f'/{robot_id}/odometry', self.on_odom, 30)
        self.create_subscription(LaserScan, f'/{robot_id}/scan', self.on_scan, 30)
        sensing = self.cfg.sensing
        self.ray_stride = max(1, int(sensing.get('ray_stride', 6)))
        self.free_step = float(sensing.get('free_space_step', self.cfg.map.cell_size * 0.5))
        self.dedupe_seconds = float(sensing.get('claim_dedupe_seconds', 0.75))
        self.max_free_cells_per_ray = max(1, int(sensing.get('max_free_cells_per_ray', 12)))
        self.self_exclusion_radius = float(sensing.get('self_exclusion_radius_m', 0.40))
        self.self_exclusion_cells = max(1, int(math.ceil(self.self_exclusion_radius / self.cfg.map.cell_size)))
        self.direct_pub = self.create_publisher(String, f'/{robot_id}/direct_claims', 100)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = odom_to_world_pose(
            p.x, p.y, _yaw(msg.pose.pose.orientation),
            start_cell=self.start_cell,
            map_height=self.grid.height,
            cell_size=self.cfg.map.cell_size,
            initial_yaw=self.initial_yaw,
        )

    def emit(self, cell: tuple[int, int], state: str, confidence: float) -> None:
        if not (0 <= cell[0] < self.grid.width and 0 <= cell[1] < self.grid.height):
            return
        now = self.now()
        key = (cell, state)
        if now - self.last_sent.get(key, -1e9) < self.dedupe_seconds:
            return
        self.last_sent[key] = now
        claim = Claim(
            source_id=self.robot_id,
            cell_x=cell[0],
            cell_y=cell[1],
            state=state,
            observation_time=now,
            reception_time=now,
            confidence=confidence,
            kind='direct',
            claim_id=f'{self.robot_id}:lidar:{now:.6f}:{cell[0]}:{cell[1]}:{state}',
        )
        out = String()
        out.data = claim.to_json()
        # Direct sensing is consumed locally immediately and also sent through the
        # simulated network for teammates. Keeping these paths separate prevents
        # a robot's own delayed network echo from masquerading as remote evidence.
        self.direct_pub.publish(out)
        self.pub.publish(out)

    def on_scan(self, scan: LaserScan) -> None:
        if self.pose is None:
            return
        x, y, yaw = self.pose
        for index in range(0, len(scan.ranges), self.ray_stride):
            measured = scan.ranges[index]
            hit = math.isfinite(measured) and measured < scan.range_max * 0.995
            distance = min(measured, scan.range_max) if math.isfinite(measured) else scan.range_max
            angle = yaw + scan.angle_min + index * scan.angle_increment
            cells = _ray_cells(
                x,
                y,
                angle,
                distance,
                map_height=self.grid.height,
                cell_size=self.cfg.map.cell_size,
                step=self.free_step,
            )
            if not cells:
                continue
            current_cell = world_to_cell(
                x, y, map_height=self.grid.height, cell_size=self.cfg.map.cell_size
            )

            def outside_self(cell: tuple[int, int]) -> bool:
                return max(abs(cell[0] - current_cell[0]), abs(cell[1] - current_cell[1])) > self.self_exclusion_cells

            free_cells = cells[:-1] if hit else cells
            for cell in free_cells[: self.max_free_cells_per_ray]:
                if outside_self(cell) and not self.grid.is_blocked(*cell):
                    self.emit(cell, 'free', 0.95)
            # Ignore very-near returns and any endpoint still inside the robot's
            # own footprint / safety halo. These are almost always self-returns
            # or contact geometry, not environmental obstacles.
            if hit and distance > self.self_exclusion_radius and outside_self(cells[-1]):
                self.emit(cells[-1], 'occupied', 1.0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--robot-id', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = LidarReporter(args.config, args.robot_id)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
