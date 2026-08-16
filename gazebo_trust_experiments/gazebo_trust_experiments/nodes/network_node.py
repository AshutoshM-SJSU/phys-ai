from __future__ import annotations

import argparse
import heapq
import random

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from gazebo_trust_experiments.models import Claim
from .common import event_payload, load_runtime


class ClaimNetwork(Node):
    def __init__(self, config_path: str) -> None:
        super().__init__('claim_network')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        _, self.cfg, _ = load_runtime(config_path)
        self.delay = float(self.cfg.network.get('communication_delay_seconds', 0.0))
        self.loss = float(self.cfg.network.get('packet_loss_probability', 0.0))
        self.jitter = float(self.cfg.network.get('jitter_seconds', 0.0))
        self.rng = random.Random(self.cfg.simulation.seed)
        self.queue: list[tuple[float, int, str]] = []
        self.sequence = 0
        self.pub = self.create_publisher(String, '/shared/claims', 200)
        self.events = self.create_publisher(String, '/experiment/events', 100)
        self.create_subscription(String, '/claims/raw', self.on_claim, 200)
        self.create_timer(0.01, self.flush)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_claim(self, msg: String) -> None:
        try:
            claim = Claim.from_json(msg.data)
        except Exception as exc:
            self.get_logger().warning(f'Dropping malformed claim: {exc}')
            return
        if self.rng.random() < self.loss:
            event = String()
            event.data = event_payload('claim_dropped', self.now(), claim_id=claim.claim_id, source_id=claim.source_id)
            self.events.publish(event)
            return
        release = self.now() + max(0.0, self.delay + self.rng.uniform(-self.jitter, self.jitter))
        self.sequence += 1
        heapq.heappush(self.queue, (release, self.sequence, msg.data))

    def flush(self) -> None:
        now = self.now()
        while self.queue and self.queue[0][0] <= now:
            _, _, payload = heapq.heappop(self.queue)
            claim = Claim.from_json(payload)
            delivered = Claim(
                source_id=claim.source_id,
                cell_x=claim.cell_x,
                cell_y=claim.cell_y,
                state=claim.state,
                observation_time=claim.observation_time,
                reception_time=now,
                confidence=claim.confidence,
                kind=claim.kind,
                claim_id=claim.claim_id,
            )
            msg = String()
            msg.data = delivered.to_json()
            self.pub.publish(msg)
            event = String()
            event.data = event_payload(
                'claim_delivered',
                now,
                claim_id=claim.claim_id,
                source_id=claim.source_id,
                network_delay=max(0.0, now - claim.observation_time),
            )
            self.events.publish(event)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = ClaimNetwork(args.config)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
