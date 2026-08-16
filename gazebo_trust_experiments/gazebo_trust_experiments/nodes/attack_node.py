from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from gazebo_trust_experiments.attacks.base import AttackContext
from gazebo_trust_experiments.registry import create_attack_module
from .common import event_payload, load_runtime


class AttackNode(Node):
    def __init__(self, config_path: str) -> None:
        super().__init__('attack_manager')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.path, self.cfg, self.grid = load_runtime(config_path)
        self.enabled = bool(self.cfg.attack.get('enabled', True))
        self.attacker = str(self.cfg.attack.get('robot_id', 'attacker_1'))
        self.modules: list[tuple[dict, object]] = []
        for raw in self.cfg.attack.get('modules', []):
            self.modules.append((raw, create_attack_module(raw)))
        self.last_publish: dict[str, float] = {}
        self.dynamic_cells: set[tuple[int, int]] = set()
        self.historical_dynamic_cells: set[tuple[int, int]] = set()
        self.claim_pub = self.create_publisher(String, '/claims/raw', 200)
        self.event_pub = self.create_publisher(String, '/experiment/events', 100)
        self.create_subscription(String, '/experiment/events', self.on_event, 100)
        self.create_timer(0.1, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_event(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            details = payload.get('details') or {}
            cell = tuple(int(v) for v in details.get('cell', []))
        except Exception:
            return
        if len(cell) != 2:
            return
        if payload.get('event_type') == 'temporary_obstacle_appeared':
            self.dynamic_cells.add(cell)
            self.historical_dynamic_cells.add(cell)
        elif payload.get('event_type') == 'temporary_obstacle_disappeared':
            self.dynamic_cells.discard(cell)

    def tick(self) -> None:
        if not self.enabled:
            return
        now = self.now()
        for index, (raw, module) in enumerate(self.modules):
            name = str(raw.get('name', f'{raw.get("type", "attack")}_{index}'))
            start = float(raw.get('start_time', 0.0))
            end = float(raw.get('end_time', float('inf')))
            period = float(raw.get('publish_period', 2.0))
            if now < start or now > end or now - self.last_publish.get(name, -1e9) < period:
                continue
            self.last_publish[name] = now
            candidates = [tuple(int(v) for v in cell) for cell in raw.get('candidate_cells', [])]
            context = AttackContext(now, self.attacker, {}, set(self.dynamic_cells), candidates, set(self.historical_dynamic_cells))
            for claim in module.generate_claims(context):
                out = String()
                out.data = claim.to_json()
                self.claim_pub.publish(out)
                event = String()
                event.data = event_payload(
                    'malicious_claim',
                    now,
                    module=name,
                    source_id=claim.source_id,
                    claim_id=claim.claim_id,
                    cell=[claim.cell_x, claim.cell_y],
                    state=claim.state,
                )
                self.event_pub.publish(event)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = AttackNode(args.config)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
