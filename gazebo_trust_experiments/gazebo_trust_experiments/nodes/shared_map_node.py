from __future__ import annotations

import argparse

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from gazebo_trust_experiments.full_trust_map import FullTrustMap
from gazebo_trust_experiments.models import Claim
from .common import event_payload, load_runtime


class SharedMapNode(Node):
    """Per-robot operational dynamic map.

    Direct local sensing and remotely received claims are intentionally kept in
    separate stores. Direct local sensing has authority for cells the robot has
    personally observed. The network path ignores echoes of the robot's own
    claims so communication delay cannot turn its own observation into stale
    remote evidence.
    """

    def __init__(self, config_path: str, robot_id: str) -> None:
        super().__init__(f'{robot_id}_shared_map')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.path, self.cfg, self.grid = load_runtime(config_path)
        self.robot_id = robot_id
        self.local_map = FullTrustMap()
        self.remote_map = FullTrustMap()
        self.claim_count = 0
        self.changed_since_publish = False
        self.last_effective_occupied: set[tuple[int, int]] = set()
        self.pub = self.create_publisher(OccupancyGrid, f'/{robot_id}/dynamic_map', 10)
        self.events = self.create_publisher(String, '/experiment/events', 100)
        self.create_subscription(String, f'/{robot_id}/direct_claims', self.on_direct_claim, 200)
        self.create_subscription(String, '/shared/claims', self.on_shared_claim, 200)
        rate = float(self.cfg.mapping.get('publish_rate_hz', 2.0))
        self.create_timer(1.0 / max(rate, 0.1), self.publish_map)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _parse(self, msg: String) -> Claim | None:
        try:
            return Claim.from_json(msg.data)
        except Exception as exc:
            self.get_logger().warning(f'Invalid claim: {exc}')
            return None

    def _event(self, claim: Claim, changed: bool, channel: str) -> None:
        event = String()
        event.data = event_payload(
            'claim_accepted',
            self.now(),
            robot_id=self.robot_id,
            claim_id=claim.claim_id,
            source_id=claim.source_id,
            cell=[claim.cell_x, claim.cell_y],
            state=claim.state,
            kind=claim.kind,
            channel=channel,
            changed_map=changed,
        )
        self.events.publish(event)

    def on_direct_claim(self, msg: String) -> None:
        claim = self._parse(msg)
        if claim is None or claim.source_id != self.robot_id:
            return
        self.claim_count += 1
        changed = self.local_map.ingest(claim)
        self.changed_since_publish = self.changed_since_publish or changed
        self._event(claim, changed, 'direct_local')

    def on_shared_claim(self, msg: String) -> None:
        claim = self._parse(msg)
        if claim is None:
            return
        # Own observations already entered through /<robot>/direct_claims. Ignore
        # the delayed network echo instead of treating it like teammate evidence.
        if claim.source_id == self.robot_id and claim.kind == 'direct':
            return
        self.claim_count += 1
        changed = self.remote_map.ingest(claim)
        self.changed_since_publish = self.changed_since_publish or changed
        self._event(claim, changed, 'remote_shared')

    def effective_state(self, cell: tuple[int, int]) -> str | None:
        local = self.local_map.state(cell)
        if local is not None:
            return local
        return self.remote_map.state(cell)

    def effective_occupied(self) -> set[tuple[int, int]]:
        candidates = set(self.local_map.cells) | set(self.remote_map.cells)
        return {cell for cell in candidates if self.effective_state(cell) == 'occupied'}

    def publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.cfg.map.cell_size
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.orientation.w = 1.0
        occupied = self.effective_occupied()
        data: list[int] = []
        for grid_y in range(self.grid.height - 1, -1, -1):
            for x in range(self.grid.width):
                data.append(100 if self.grid.is_blocked(x, grid_y) or (x, grid_y) in occupied else 0)
        msg.data = data
        self.pub.publish(msg)

        effective_changed = occupied != self.last_effective_occupied
        if effective_changed:
            self.last_effective_occupied = set(occupied)
            event = String()
            event.data = event_payload(
                'map_changed',
                self.now(),
                robot_id=self.robot_id,
                dynamic_occupied_count=len(occupied),
                claim_count=self.claim_count,
            )
            self.events.publish(event)
        self.changed_since_publish = False


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--robot-id', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = SharedMapNode(args.config, args.robot_id)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
