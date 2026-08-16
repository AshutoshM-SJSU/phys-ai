from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
from typing import Any

from .astar import astar
from .config import ExperimentConfig
from .movingai_map import MovingAIMap

Cell = tuple[int, int]


def _reachable_cells(grid: MovingAIMap, start: Cell) -> set[Cell]:
    if grid.is_blocked(*start):
        return set()
    seen = {start}
    stack = [start]
    while stack:
        x, y = stack.pop()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            cell = (nx, ny)
            if 0 <= nx < grid.width and 0 <= ny < grid.height and cell not in seen and not grid.is_blocked(nx, ny):
                seen.add(cell)
                stack.append(cell)
    return seen


def _spread_goal_pool(config: ExperimentConfig, grid: MovingAIMap, count: int) -> list[list[int]]:
    """Choose a deterministic, map-wide shared goal pool using farthest-point sampling."""
    starts = [tuple(int(v) for v in robot['start_cell']) for robot in config.robots]
    reachable_sets = [_reachable_cells(grid, start) for start in starts]
    if not reachable_sets:
        raise ValueError('No robot starts are available for automatic goal placement')
    common = set.intersection(*reachable_sets)
    edge_margin = max(0, int(config.environment.get('delivery_goal_pool', {}).get('edge_margin_cells', 2)))
    candidates = [
        cell for cell in common
        if edge_margin <= cell[0] < grid.width - edge_margin
        and edge_margin <= cell[1] < grid.height - edge_margin
        and cell not in starts
    ]
    if len(candidates) < count:
        candidates = [cell for cell in common if cell not in starts]
    if len(candidates) < count:
        raise ValueError(f'Only {len(candidates)} mutually reachable free cells exist for {count} delivery goals')

    cx, cy = (grid.width - 1) / 2.0, (grid.height - 1) / 2.0
    # Start near the map center so the pool is not merely four corners, then
    # repeatedly choose the cell farthest from all already selected goals.
    first = min(candidates, key=lambda c: ((c[0]-cx)**2 + (c[1]-cy)**2, c[1], c[0]))
    selected = [first]
    remaining = set(candidates)
    remaining.remove(first)
    minimum_separation = max(1, int(config.environment.get('delivery_goal_pool', {}).get('min_separation_cells', 4)))
    while len(selected) < count:
        ranked = sorted(
            remaining,
            key=lambda c: (
                min(abs(c[0]-s[0]) + abs(c[1]-s[1]) for s in selected),
                min(abs(c[0]-s[0]) + abs(c[1]-s[1]) for s in starts),
                -abs(c[0]-cx)-abs(c[1]-cy),
                -c[1], -c[0],
            ),
            reverse=True,
        )
        choice = next((c for c in ranked if min(abs(c[0]-s[0]) + abs(c[1]-s[1]) for s in selected) >= minimum_separation), ranked[0])
        selected.append(choice)
        remaining.remove(choice)
    return [[x, y] for x, y in selected]


def _robot(config: ExperimentConfig, robot_id: str) -> dict[str, Any]:
    for robot in config.robots:
        if str(robot.get('id')) == robot_id:
            return robot
    raise ValueError(f"Unknown placement robot_id: {robot_id}")


def _route(config: ExperimentConfig, grid: MovingAIMap, robot_id: str) -> list[Cell]:
    robot = _robot(config, robot_id)
    start = tuple(int(v) for v in robot['start_cell'])
    goals = robot.get('goal_nodes') or []
    if not goals:
        raise ValueError(f'Robot {robot_id} has no goal_nodes')
    goal_index = int(robot.get('initial_goal_index', 0)) % len(goals)
    goal = tuple(int(v) for v in goals[goal_index])
    path = astar(grid, start, goal, allow_diagonal=config.planning.allow_diagonal)
    if len(path) < 5:
        raise ValueError(f'Could not derive a usable static route for {robot_id}: {start} -> {goal}')
    return path


def _path_cell(path: list[Cell], fraction: float, forbidden: set[Cell]) -> Cell:
    fraction = min(0.95, max(0.05, float(fraction)))
    preferred = int(round(fraction * (len(path) - 1)))
    # Search outward from the preferred route index so placement is deterministic
    # but does not collide with another auto-placed object.
    for radius in range(len(path)):
        for index in (preferred - radius, preferred + radius):
            if 1 <= index < len(path) - 1:
                cell = path[index]
                if cell not in forbidden:
                    return cell
    raise ValueError('No unused free route cell was available for automatic placement')


def _resolve_cell(raw: dict[str, Any], config: ExperimentConfig, grid: MovingAIMap, forbidden: set[Cell]) -> Cell:
    cell = raw.get('cell')
    if isinstance(cell, (list, tuple)) and len(cell) == 2:
        return int(cell[0]), int(cell[1])
    placement = raw.get('placement') or {}
    robot_id = str(placement.get('robot_id', 'robot_0'))
    fraction = float(placement.get('route_fraction', 0.5))
    return _path_cell(_route(config, grid, robot_id), fraction, forbidden)


def _resolve_candidates(raw: dict[str, Any], config: ExperimentConfig, grid: MovingAIMap, forbidden: set[Cell]) -> list[list[int]]:
    cells = raw.get('candidate_cells')
    if isinstance(cells, list) and cells and all(isinstance(c, (list, tuple)) and len(c) == 2 for c in cells):
        return [[int(c[0]), int(c[1])] for c in cells]

    placement = raw.get('placement') or {}
    robot_id = str(placement.get('robot_id', 'robot_0'))
    fractions = placement.get('route_fractions', [0.35, 0.50, 0.65])
    path = _route(config, grid, robot_id)
    selected: list[list[int]] = []
    local_forbidden = set(forbidden)
    for fraction in fractions:
        cell = _path_cell(path, float(fraction), local_forbidden)
        selected.append([cell[0], cell[1]])
        local_forbidden.add(cell)
    return selected


def resolve_scenario(config: ExperimentConfig, grid: MovingAIMap) -> ExperimentConfig:
    """Resolve map-dependent 'auto' placements into concrete free grid cells.

    This keeps YAML portable across MovingAI maps and prevents hard-coded obstacle
    coordinates from accidentally landing inside static walls.
    """
    environment = deepcopy(config.environment)
    attack = deepcopy(config.attack)
    robots = deepcopy(config.robots)

    auto_robots = [robot for robot in robots if isinstance(robot.get('goal_nodes'), str) and str(robot.get('goal_nodes')).lower() == 'auto']
    if auto_robots:
        pool_cfg = environment.get('delivery_goal_pool', {}) or {}
        count = int(pool_cfg.get('count', max(int(r.get('goal_count', 12)) for r in auto_robots)))
        shared_pool = _spread_goal_pool(config, grid, count)
        for robot in robots:
            if isinstance(robot.get('goal_nodes'), str) and str(robot.get('goal_nodes')).lower() == 'auto':
                robot['goal_nodes'] = deepcopy(shared_pool)
                robot['resolved_goal_pool'] = {'strategy': 'auto_spread', 'count': len(shared_pool)}

    working = replace(config, robots=robots, environment=environment, attack=attack)
    forbidden: set[Cell] = set()

    for robot in robots:
        forbidden.add(tuple(int(v) for v in robot['start_cell']))
        for goal in robot.get('goal_nodes', []):
            forbidden.add(tuple(int(v) for v in goal))

    for obstacle in environment.get('temporary_obstacles', []):
        was_auto = not (isinstance(obstacle.get('cell'), (list, tuple)) and len(obstacle.get('cell')) == 2)
        cell = _resolve_cell(obstacle, working, grid, forbidden)
        if grid.is_blocked(*cell):
            raise ValueError(f"Resolved temporary obstacle {obstacle.get('id', '')} onto blocked cell {cell}")
        obstacle['cell'] = [cell[0], cell[1]]
        obstacle['resolved_placement'] = {
            'strategy': 'static_route_cell' if was_auto else 'explicit',
            'cell': [cell[0], cell[1]],
        }
        forbidden.add(cell)

    for module in attack.get('modules', []):
        if str(module.get('type', '')) != 'false_obstacle':
            continue
        module['candidate_cells'] = _resolve_candidates(module, working, grid, forbidden)

    return replace(config, robots=robots, environment=environment, attack=attack)
