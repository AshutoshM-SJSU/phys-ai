from __future__ import annotations

from pathlib import Path

from .config import ExperimentConfig
from .movingai_map import MovingAIMap


SUPPORTED_ATTACKS = {'false_obstacle', 'false_clearance', 'stale_reassertion'}


def validate_experiment(config: ExperimentConfig, grid: MovingAIMap, map_path: Path) -> list[str]:
    errors: list[str] = []
    if not map_path.is_file():
        errors.append(f'Map file does not exist: {map_path}')
        return errors

    def check_cell(cell, label: str, require_free: bool = True) -> None:
        try:
            x, y = int(cell[0]), int(cell[1])
        except Exception:
            errors.append(f'{label} must be [x, y]')
            return
        if not (0 <= x < grid.width and 0 <= y < grid.height):
            errors.append(f'{label} is outside map bounds: {(x, y)}')
        elif require_free and grid.is_blocked(x, y):
            errors.append(f'{label} is a static blocked cell: {(x, y)}')

    attacker_count = 0
    for robot in config.robots:
        rid = str(robot['id'])
        if 'attacker' in str(robot.get('role', '')):
            attacker_count += 1
        check_cell(robot['start_cell'], f'{rid}.start_cell')
        for index, goal in enumerate(robot['goal_nodes']):
            check_cell(goal, f'{rid}.goal_nodes[{index}]')
    if attacker_count != 1:
        errors.append(f'Experiment 2 requires exactly one delayed attacker robot; found {attacker_count}')

    obstacle_ids: set[str] = set()
    for index, obstacle in enumerate(config.environment.get('temporary_obstacles', [])):
        oid = str(obstacle.get('id', '')).strip()
        if not oid or oid in obstacle_ids:
            errors.append(f'environment.temporary_obstacles[{index}].id must be unique')
        obstacle_ids.add(oid)
        check_cell(obstacle.get('cell'), f'temporary obstacle {oid}.cell')
        if float(obstacle.get('disappear_time', 0.0)) <= float(obstacle.get('appear_time', 0.0)):
            errors.append(f'temporary obstacle {oid} must disappear after it appears')

    if config.attack.get('enabled', True):
        modules = config.attack.get('modules', [])
        if not modules:
            errors.append('attack.enabled is true but attack.modules is empty')
        for index, module in enumerate(modules):
            attack_type = str(module.get('type', ''))
            if attack_type not in SUPPORTED_ATTACKS:
                errors.append(f'attack.modules[{index}].type is unsupported: {attack_type}')
            if float(module.get('publish_period', 0.0)) <= 0:
                errors.append(f'attack.modules[{index}].publish_period must be positive')
            for cell_index, cell in enumerate(module.get('candidate_cells', [])):
                check_cell(cell, f'attack.modules[{index}].candidate_cells[{cell_index}]')

    return errors
