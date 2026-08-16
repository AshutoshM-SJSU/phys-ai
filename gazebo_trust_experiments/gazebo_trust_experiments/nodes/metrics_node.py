from __future__ import annotations

import argparse
import csv
import json
import math
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from gazebo_trust_experiments.astar import astar
from gazebo_trust_experiments.coordinates import odom_to_world_pose, world_to_cell
from .common import load_runtime


class RobotMetrics:
    def __init__(self) -> None:
        self.last_position: tuple[float, float] | None = None
        self.world_position: tuple[float, float] | None = None
        self.distance = 0.0
        self.delivery_distance_start = 0.0
        self.delivery_started_at: float | None = None
        self.delivery_times: list[float] = []
        self.delivery_distances: list[float] = []
        self.shortest_distances: list[float] = []
        self.current_shortest_distance: float | None = None
        self.replans = 0
        self.delivery_replans_start = 0
        self.replans_per_completed_delivery: list[int] = []
        self.no_path_time = 0.0
        self.no_path_active_since: float | None = None
        self.hesitation_time = 0.0
        self.last_odom_time: float | None = None
        self.reversals = 0
        self.emergency_stops = 0
        self.near_collisions = 0
        self.collisions = 0
        self.planning_times_ms: list[float] = []
        self.route_overlaps: list[float] = []


class MetricsNode(Node):
    def __init__(self, config_path: str, output_dir: str) -> None:
        super().__init__('metrics_collector')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.path, self.cfg, self.grid = load_runtime(config_path)
        self.out = Path(output_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.started_wall = time.time()
        self.events: list[dict] = []
        self.robots = {str(robot['id']): RobotMetrics() for robot in self.cfg.robots}
        self.robot_cfg = {str(robot['id']): robot for robot in self.cfg.robots}
        self.roles = {str(robot['id']): str(robot.get('role', 'robot')) for robot in self.cfg.robots}
        self.dynamic_truth: set[tuple[int, int]] = set()
        self.map_cells: dict[str, set[tuple[int, int]]] = defaultdict(set)
        self.map_error_samples: dict[str, list[float]] = defaultdict(list)
        self.malicious_claims: dict[str, dict] = {}
        self.malicious_delivered: set[str] = set()
        self.legitimate_delivered: set[str] = set()
        self.accepted_malicious: set[tuple[str, str]] = set()
        self.accepted_legitimate: set[tuple[str, str]] = set()
        self.persistence_started: dict[tuple[str, str], float] = {}
        self.persistence_values: list[float] = []
        self.hesitation_threshold = float(self.cfg.metrics.get('hesitation_velocity_threshold', 0.03))
        self.free_cell_count = sum(
            1 for y in range(self.grid.height) for x in range(self.grid.width) if not self.grid.is_blocked(x, y)
        )

        for robot_id in self.robots:
            self.create_subscription(Odometry, f'/{robot_id}/odometry', lambda msg, rid=robot_id: self.on_odom(rid, msg), 30)
            self.create_subscription(OccupancyGrid, f'/{robot_id}/dynamic_map', lambda msg, rid=robot_id: self.on_map(rid, msg), 10)
        self.create_subscription(String, '/experiment/events', self.on_event, 300)
        self.create_timer(1.0, self.flush)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, robot_id: str, msg: Odometry) -> None:
        metrics = self.robots[robot_id]
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        stamp = self.now()
        if metrics.last_position is not None:
            metrics.distance += math.dist(metrics.last_position, position)
        if metrics.last_odom_time is not None:
            dt = max(0.0, stamp - metrics.last_odom_time)
            speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            if metrics.delivery_started_at is not None and speed < self.hesitation_threshold:
                metrics.hesitation_time += dt
        metrics.last_position = position
        metrics.last_odom_time = stamp

        robot = self.robot_cfg[robot_id]
        start_cell = tuple(int(v) for v in robot['start_cell'])
        world_x, world_y, _ = odom_to_world_pose(
            position[0], position[1], 0.0,
            start_cell=start_cell,
            map_height=self.grid.height,
            cell_size=self.cfg.map.cell_size,
            initial_yaw=float(robot.get('start_yaw', 0.0)),
        )
        metrics.world_position = (world_x, world_y)

    def on_map(self, robot_id: str, msg: OccupancyGrid) -> None:
        occupied: set[tuple[int, int]] = set()
        threshold = int(self.cfg.mapping.get('occupied_threshold', 50))
        for row in range(msg.info.height):
            grid_y = self.grid.height - 1 - row
            for x in range(msg.info.width):
                if msg.data[row * msg.info.width + x] >= threshold and not self.grid.is_blocked(x, grid_y):
                    occupied.add((x, grid_y))
        self.map_cells[robot_id] = occupied
        error_cells = occupied ^ self.dynamic_truth
        self.map_error_samples[robot_id].append(len(error_cells) / max(1, self.free_cell_count))

        now = self.now()
        for claim_id, claim in self.malicious_claims.items():
            if claim.get('state') != 'occupied':
                continue
            key = (robot_id, claim_id)
            cell = tuple(claim.get('cell', []))
            influences = len(cell) == 2 and cell in occupied
            if influences and key not in self.persistence_started:
                self.persistence_started[key] = float(claim.get('sim_time', now))
            elif not influences and key in self.persistence_started:
                self.persistence_values.append(max(0.0, now - self.persistence_started.pop(key)))

    def _true_shortest_distance(self, robot_id: str, goal: tuple[int, int]) -> float:
        metrics = self.robots[robot_id]
        if metrics.world_position is None:
            return 0.0
        start = world_to_cell(
            metrics.world_position[0], metrics.world_position[1],
            map_height=self.grid.height, cell_size=self.cfg.map.cell_size,
        )
        truth = set(self.dynamic_truth)
        outer = self

        class TruthGrid:
            width = outer.grid.width
            height = outer.grid.height

            def is_blocked(self, x: int, y: int) -> bool:
                return outer.grid.is_blocked(x, y) or (x, y) in truth

        try:
            path = astar(TruthGrid(), start, goal, allow_diagonal=self.cfg.planning.allow_diagonal)
        except ValueError:
            return 0.0
        return max(0, len(path) - 1) * self.cfg.map.cell_size if path else 0.0

    def on_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.events.append(event)
        event_type = event.get('event_type', '')
        robot_id = str(event.get('robot_id', ''))
        details = event.get('details') or {}
        sim_time = float(event.get('sim_time', self.now()))

        cell_raw = details.get('cell', [])
        if event_type == 'temporary_obstacle_appeared' and len(cell_raw) == 2:
            self.dynamic_truth.add((int(cell_raw[0]), int(cell_raw[1])))
        elif event_type == 'temporary_obstacle_disappeared' and len(cell_raw) == 2:
            self.dynamic_truth.discard((int(cell_raw[0]), int(cell_raw[1])))
        elif event_type == 'malicious_claim':
            self.malicious_claims[str(details.get('claim_id'))] = {
                'cell': list(details.get('cell', [])),
                'state': details.get('state'),
                'sim_time': sim_time,
            }
        elif event_type == 'claim_delivered':
            claim_id = str(details.get('claim_id', ''))
            source_id = str(details.get('source_id', ''))
            if claim_id in self.malicious_claims or source_id == str(self.cfg.attack.get('robot_id', 'attacker_1')) and ':attack:' in claim_id:
                self.malicious_delivered.add(claim_id)
            else:
                self.legitimate_delivered.add(claim_id)
        elif event_type == 'claim_accepted':
            claim_id = str(details.get('claim_id', ''))
            if details.get('kind') == 'malicious' or claim_id in self.malicious_claims:
                self.accepted_malicious.add((robot_id, claim_id))
            else:
                self.accepted_legitimate.add((robot_id, claim_id))

        if robot_id not in self.robots:
            return
        metrics = self.robots[robot_id]
        if event_type in {'mission_started', 'delivery_started'}:
            metrics.delivery_started_at = sim_time
            metrics.delivery_distance_start = metrics.distance
            metrics.delivery_replans_start = metrics.replans
            goal_raw = details.get('goal', [])
            if len(goal_raw) == 2:
                metrics.current_shortest_distance = self._true_shortest_distance(
                    robot_id, (int(goal_raw[0]), int(goal_raw[1]))
                )
            else:
                metrics.current_shortest_distance = None
        elif event_type == 'delivery_completed':
            metrics.delivery_times.append(float(details.get('delivery_time', 0.0)))
            metrics.delivery_distances.append(max(0.0, metrics.distance - metrics.delivery_distance_start))
            metrics.shortest_distances.append(metrics.current_shortest_distance or 0.0)
            metrics.replans_per_completed_delivery.append(max(0, metrics.replans - metrics.delivery_replans_start))
            metrics.delivery_started_at = None
            metrics.current_shortest_distance = None
        elif event_type == 'replan':
            metrics.replans += 1
            metrics.planning_times_ms.append(float(details.get('planning_time_ms', 0.0)))
            metrics.route_overlaps.append(float(details.get('route_overlap', 0.0)))
            if bool(details.get('route_reversal', False)):
                metrics.reversals += 1
            if metrics.current_shortest_distance is None:
                start_raw = details.get('start', [])
                goal_raw = details.get('goal', [])
                if len(start_raw) == 2 and len(goal_raw) == 2:
                    truth = set(self.dynamic_truth)
                    outer = self
                    class TruthGrid:
                        width = outer.grid.width
                        height = outer.grid.height
                        def is_blocked(self, x: int, y: int) -> bool:
                            return outer.grid.is_blocked(x, y) or (x, y) in truth
                    try:
                        truth_path = astar(TruthGrid(), (int(start_raw[0]), int(start_raw[1])), (int(goal_raw[0]), int(goal_raw[1])), allow_diagonal=self.cfg.planning.allow_diagonal)
                    except ValueError:
                        truth_path = []
                    metrics.current_shortest_distance = max(0, len(truth_path) - 1) * self.cfg.map.cell_size if truth_path else 0.0
        elif event_type == 'no_path_started':
            if metrics.no_path_active_since is None:
                metrics.no_path_active_since = sim_time
        elif event_type == 'no_path_ended':
            if metrics.no_path_active_since is not None:
                metrics.no_path_time += max(0.0, sim_time - metrics.no_path_active_since)
                metrics.no_path_active_since = None
            else:
                metrics.no_path_time += float(details.get('duration', 0.0))
        elif event_type == 'emergency_stop':
            metrics.emergency_stops += 1
        elif event_type == 'near_collision':
            metrics.near_collisions += 1
        elif event_type == 'collision':
            metrics.collisions += 1

    @staticmethod
    def mean(values: list[float] | list[int]) -> float:
        return 0.0 if not values else float(sum(values)) / len(values)

    def summary(self) -> dict:
        now = self.now()
        per_robot: dict[str, dict] = {}
        benign_ids = [rid for rid, role in self.roles.items() if 'attacker' not in role]
        for robot_id, metrics in self.robots.items():
            ratios = [actual / shortest for actual, shortest in zip(metrics.delivery_distances, metrics.shortest_distances) if shortest > 0]
            no_path = metrics.no_path_time
            if metrics.no_path_active_since is not None:
                no_path += max(0.0, now - metrics.no_path_active_since)
            per_robot[robot_id] = {
                'role': self.roles[robot_id],
                'deliveries_completed': len(metrics.delivery_times),
                'average_delivery_time_s': self.mean(metrics.delivery_times),
                'distance_traveled_m': metrics.distance,
                'average_detour_ratio': self.mean(ratios),
                'no_path_time_s': no_path,
                'replans': metrics.replans,
                'replans_per_delivery': self.mean(metrics.replans_per_completed_delivery),
                'hesitation_time_s': metrics.hesitation_time,
                'route_reversals': metrics.reversals,
                'mean_planning_time_ms': self.mean(metrics.planning_times_ms),
                'route_stability_mean_overlap': self.mean(metrics.route_overlaps),
                'map_error_rate': self.mean(self.map_error_samples[robot_id]),
                'collisions': metrics.collisions,
                'near_collisions': metrics.near_collisions,
                'emergency_stops': metrics.emergency_stops,
                'safety_events': metrics.collisions + metrics.near_collisions + metrics.emergency_stops,
            }

        robot_count = max(1, len(self.robots))
        malicious_expected = len(self.malicious_delivered) * robot_count
        legit_expected = len(self.legitimate_delivered) * robot_count
        false_acceptance = len(self.accepted_malicious) / malicious_expected if malicious_expected else 0.0
        legitimate_acceptance = len(self.accepted_legitimate) / legit_expected if legit_expected else 1.0
        active_persistence = [max(0.0, now - started) for started in self.persistence_started.values()]
        persistence = [*self.persistence_values, *active_persistence]

        return {
            'simulation_time_s': now,
            'wall_elapsed_seconds': time.time() - self.started_wall,
            'benign_deliveries_completed': sum(per_robot[rid]['deliveries_completed'] for rid in benign_ids),
            'benign_average_delivery_time_s': self.mean([
                t for rid in benign_ids for t in self.robots[rid].delivery_times
            ]),
            'false_blockage_persistence_mean_s': self.mean(persistence),
            'false_acceptance_rate': false_acceptance,
            'false_rejection_rate': max(0.0, 1.0 - legitimate_acceptance),
            'malicious_claims_generated': len(self.malicious_claims),
            'malicious_claims_delivered': len(self.malicious_delivered),
            'legitimate_claims_delivered': len(self.legitimate_delivered),
            'trust_detection_delay_s': None,
            'recovery_time_s': None,
            'trust_metrics_status': 'not_applicable_full_trust_no_trust_model',
            'per_robot': per_robot,
            'event_counts': dict(self.event_counts()),
            'metric_notes': {
                'detour_ratio': 'Actual completed-delivery distance divided by shortest A* path under static map plus true temporary obstacles at delivery start.',
                'map_error_rate': 'Mean fraction of traversable map cells whose dynamic occupied/free estimate differs from current ground truth.',
                'route_stability': 'Mean Jaccard overlap between consecutive A* cell paths.',
                'safety_events': 'Collision events if instrumented, plus near-collision and emergency-stop events.',
                'trust_detection_delay': 'Not applicable in the full-trust preliminary configuration.',
                'recovery_time': 'Not applicable without an attacker-detection/trust threshold.',
            },
        }

    def event_counts(self):
        counts: dict[str, int] = defaultdict(int)
        for event in self.events:
            counts[event.get('event_type', 'unknown')] += 1
        return counts

    def flush(self) -> None:
        (self.out / 'events.jsonl').write_text(
            ''.join(json.dumps(event, sort_keys=True) + '\n' for event in self.events), encoding='utf-8'
        )
        summary = self.summary()
        (self.out / 'metrics_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        rows = summary['per_robot']
        with (self.out / 'metrics_by_robot.csv').open('w', newline='', encoding='utf-8') as handle:
            if rows:
                writer = csv.DictWriter(handle, fieldnames=['robot_id', *next(iter(rows.values())).keys()])
                writer.writeheader()
                for robot_id, values in rows.items():
                    writer.writerow({'robot_id': robot_id, **values})


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = MetricsNode(args.config, args.output_dir)
    try:
        rclpy.spin(node)
    finally:
        node.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
