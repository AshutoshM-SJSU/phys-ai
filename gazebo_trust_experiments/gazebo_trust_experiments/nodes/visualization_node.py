from __future__ import annotations

import argparse
import json
import math
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from gazebo_trust_experiments.coordinates import cell_to_world
from .common import load_runtime


def _rgba(value: str) -> tuple[float, float, float, float]:
    parts = [float(v) for v in str(value).split()]
    while len(parts) < 4:
        parts.append(1.0)
    return tuple(parts[:4])  # type: ignore[return-value]


def _rgba_text(value: tuple[float, float, float, float]) -> str:
    return ' '.join(f'{v:.4f}' for v in value)


class VisualizationNode(Node):
    """Presentation overlays driven by the controller's remaining route.

    A route now starts at the controller's current physical pose and only
    includes the unconsumed A* waypoints.  This makes localization / path
    disagreement immediately visible instead of leaving a historical route
    painted behind the robot.
    """

    def __init__(self, config_path: str) -> None:
        super().__init__('experiment_visualization')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        _, self.cfg, self.grid = load_runtime(config_path)
        self.enabled = bool(self.cfg.visualization.get('dynamic_routes', True)) and not self.cfg.simulation.headless
        self.colors = {str(r['id']): _rgba(str(r.get('color', '0.2 0.6 1.0 1'))) for r in self.cfg.robots}
        self.route_models = {rid: f'viz_route_{rid}' for rid in self.colors}
        self.active_goal_models = {rid: f'viz_active_goal_{rid}' for rid in self.colors}
        self.goal_pool_model = 'viz_delivery_goal_pool'
        self._goals_drawn = False
        if self.enabled:
            self.create_subscription(String, '/experiment/events', self.on_event, 100)
            self.create_timer(1.5, self._draw_goal_pool_once)

    def _call_boolean_service(self, service: str, request_type: str, request: str, timeout_ms: int = 1200) -> bool:
        try:
            result = subprocess.run(
                ['gz', 'service', '-s', service, '--reqtype', request_type,
                 '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms), '--req', request],
                capture_output=True, text=True,
                timeout=max(1.5, timeout_ms / 1000 + 0.5), check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return result.returncode == 0 and 'data: true' in result.stdout.lower()

    def _remove_model(self, name: str) -> None:
        request = f'name: {json.dumps(name)}, type: MODEL'
        self._call_boolean_service('/world/trust_experiment/remove', 'gz.msgs.Entity', request, 800)

    def _spawn_sdf(self, name: str, sdf: str) -> bool:
        request = f'sdf: {json.dumps(sdf)}, name: {json.dumps(name)}'
        return self._call_boolean_service('/world/trust_experiment/create', 'gz.msgs.EntityFactory', request, 1800)

    def _draw_goal_pool_once(self) -> None:
        if self._goals_drawn or not self.enabled:
            return
        goals: list[tuple[int, int]] = []
        for robot in self.cfg.robots:
            for goal in robot.get('goal_nodes', []):
                if isinstance(goal, list) and len(goal) == 2:
                    cell = (int(goal[0]), int(goal[1]))
                    if cell not in goals:
                        goals.append(cell)
        if not goals:
            return
        visuals = []
        for idx, cell in enumerate(goals):
            x, y = cell_to_world(*cell, map_height=self.grid.height, cell_size=self.cfg.map.cell_size)
            visuals.append(
                f"<visual name='goal_{idx}'><pose>{x} {y} 0.009 0 0 0</pose>"
                "<geometry><cylinder><radius>0.16</radius><length>0.018</length></cylinder></geometry>"
                "<material><ambient>0.95 0.80 0.15 0.90</ambient><diffuse>0.95 0.80 0.15 0.90</diffuse>"
                "<emissive>0.30 0.20 0.01 0.25</emissive></material></visual>"
            )
        sdf = "<sdf version='1.9'><model name='%s'><static>true</static><link name='link'>%s</link></model></sdf>" % (
            self.goal_pool_model, ''.join(visuals))
        self._remove_model(self.goal_pool_model)
        self._goals_drawn = self._spawn_sdf(self.goal_pool_model, sdf)

    def _draw_active_goal(self, rid: str, goal: list[int]) -> None:
        x, y = cell_to_world(int(goal[0]), int(goal[1]), map_height=self.grid.height, cell_size=self.cfg.map.cell_size)
        rgba = _rgba_text(self.colors[rid])
        name = self.active_goal_models[rid]
        sdf = (
            f"<sdf version='1.9'><model name='{name}'><static>true</static><link name='link'>"
            f"<visual name='active_goal'><pose>{x} {y} 0.018 0 0 0</pose>"
            "<geometry><cylinder><radius>0.22</radius><length>0.036</length></cylinder></geometry>"
            f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><emissive>{rgba}</emissive></material>"
            "</visual></link></model></sdf>"
        )
        self._remove_model(name)
        self._spawn_sdf(name, sdf)

    def _draw_route(self, rid: str, path: list[list[int]], pose_world: list[float] | None = None) -> None:
        world_points: list[tuple[float, float]] = []
        if isinstance(pose_world, list) and len(pose_world) >= 2:
            world_points.append((float(pose_world[0]), float(pose_world[1])))
        for cell in path:
            if isinstance(cell, list) and len(cell) == 2:
                point = cell_to_world(int(cell[0]), int(cell[1]), map_height=self.grid.height, cell_size=self.cfg.map.cell_size)
                if not world_points or math.hypot(point[0] - world_points[-1][0], point[1] - world_points[-1][1]) > 0.03:
                    world_points.append(point)
        name = self.route_models[rid]
        self._remove_model(name)
        if len(world_points) < 2:
            return

        rgba = _rgba_text(self.colors[rid])
        visuals: list[str] = []
        for idx, (x, y) in enumerate(world_points):
            radius = 0.055 if idx == 0 else 0.075
            visuals.append(
                f"<visual name='node_{idx}'><pose>{x} {y} 0.013 0 0 0</pose>"
                f"<geometry><cylinder><radius>{radius}</radius><length>0.018</length></cylinder></geometry>"
                f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><emissive>{rgba}</emissive></material></visual>"
            )
        for idx, ((x0, y0), (x1, y1)) in enumerate(zip(world_points, world_points[1:])):
            dx, dy = x1 - x0, y1 - y0
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                continue
            mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            yaw = math.atan2(dy, dx)
            visuals.append(
                f"<visual name='segment_{idx}'><pose>{mx} {my} 0.010 0 0 {yaw}</pose>"
                f"<geometry><box><size>{length + 0.02:.4f} 0.085 0.012</size></box></geometry>"
                f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><emissive>{rgba}</emissive></material></visual>"
            )
        sdf = "<sdf version='1.9'><model name='%s'><static>true</static><link name='link'>%s</link></model></sdf>" % (
            name, ''.join(visuals))
        if not self._spawn_sdf(name, sdf):
            self.get_logger().warning(f'Could not draw world-aligned route overlay for {rid}')

    def on_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        rid = str(event.get('robot_id', ''))
        if rid not in self.colors:
            return
        details = event.get('details') or {}
        event_type = str(event.get('event_type', ''))
        if event_type in {'mission_started', 'delivery_started'}:
            goal = details.get('goal')
            if isinstance(goal, list) and len(goal) == 2:
                self._draw_active_goal(rid, goal)
        elif event_type in {'replan', 'route_progress'}:
            path = details.get('remaining_path', details.get('path', [])) or []
            pose_world = details.get('pose_world')
            if isinstance(path, list):
                self._draw_route(rid, path, pose_world if isinstance(pose_world, list) else None)
        elif event_type == 'delivery_completed':
            self._remove_model(self.route_models[rid])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = VisualizationNode(args.config)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
