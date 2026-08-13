from __future__ import annotations

import argparse
import json
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from gazebo_trust_experiments.coordinates import cell_to_world
from gazebo_trust_experiments.environment import TemporaryObstacle
from .common import event_payload, load_runtime


class EnvironmentNode(Node):
    def __init__(self, config_path: str) -> None:
        super().__init__('temporary_obstacle_manager')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.path, self.cfg, self.grid = load_runtime(config_path)
        self.obstacles = [TemporaryObstacle.from_dict(item) for item in self.cfg.environment.get('temporary_obstacles', [])]
        self.active: set[str] = set()
        self.completed: set[str] = set()
        self.pub = self.create_publisher(String, '/experiment/events', 100)
        self.create_timer(0.1, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def call_service(self, service: str, request_type: str, request: str) -> bool:
        result = subprocess.run(
            [
                'gz', 'service', '-s', service,
                '--reqtype', request_type,
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', request,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0 or 'data: true' not in result.stdout.lower():
            self.get_logger().error(
                f'Gazebo service failed ({service}): stdout={result.stdout.strip()} stderr={result.stderr.strip()}'
            )
            return False
        return True

    def emit(self, event_type: str, obstacle: TemporaryObstacle) -> None:
        msg = String()
        msg.data = event_payload(
            event_type,
            self.now(),
            id=obstacle.obstacle_id,
            cell=list(obstacle.cell),
            size=list(obstacle.size),
        )
        self.pub.publish(msg)

    def spawn(self, obstacle: TemporaryObstacle) -> None:
        x, y = cell_to_world(
            *obstacle.cell,
            map_height=self.grid.height,
            cell_size=self.cfg.map.cell_size,
        )
        sx, sy, sz = obstacle.size
        sdf = (
            f"<sdf version='1.9'><model name='{obstacle.obstacle_id}'><static>true</static>"
            f"<pose>{x} {y} {sz / 2.0} 0 0 0</pose><link name='link'>"
            f"<collision name='collision'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>"
            f"<visual name='visual'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
            f"<material><ambient>{obstacle.color}</ambient><diffuse>{obstacle.color}</diffuse></material>"
            f"</visual></link></model></sdf>"
        )
        request = f'sdf: {json.dumps(sdf)}, name: {json.dumps(obstacle.obstacle_id)}'
        if self.call_service('/world/trust_experiment/create', 'gz.msgs.EntityFactory', request):
            self.active.add(obstacle.obstacle_id)
            self.emit('temporary_obstacle_appeared', obstacle)

    def remove(self, obstacle: TemporaryObstacle) -> None:
        request = f'name: {json.dumps(obstacle.obstacle_id)}, type: MODEL'
        if self.call_service('/world/trust_experiment/remove', 'gz.msgs.Entity', request):
            self.active.discard(obstacle.obstacle_id)
            self.completed.add(obstacle.obstacle_id)
            self.emit('temporary_obstacle_disappeared', obstacle)

    def tick(self) -> None:
        now = self.now()
        for obstacle in self.obstacles:
            if obstacle.obstacle_id in self.completed:
                continue
            if obstacle.appear_time <= now < obstacle.disappear_time and obstacle.obstacle_id not in self.active:
                self.spawn(obstacle)
            elif now >= obstacle.disappear_time and obstacle.obstacle_id in self.active:
                self.remove(obstacle)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = EnvironmentNode(args.config)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
