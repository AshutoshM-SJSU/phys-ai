from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from .common import load_runtime


class SupervisorNode(Node):
    def __init__(self, config_path: str) -> None:
        super().__init__('experiment_supervisor')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.path, self.cfg, self.grid = load_runtime(config_path)
        self.events = self.create_publisher(String, '/experiment/events', 100)
        self.finished = False
        self.create_timer(0.1, self.tick)

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if self.finished or now < self.cfg.simulation.timeout_sim_seconds:
            return
        self.finished = True
        msg = String()
        msg.data = json.dumps(
            {
                'event_type': 'experiment_finished',
                'sim_time': now,
                'robot_id': '',
                'details': {'reason': 'simulation_timeout'},
            },
            separators=(',', ':'),
        )
        self.events.publish(msg)
        self.get_logger().info(f'Experiment reached {now:.3f} simulated seconds')
        self.create_timer(0.5, self.shutdown)

    def shutdown(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = SupervisorNode(args.config)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
