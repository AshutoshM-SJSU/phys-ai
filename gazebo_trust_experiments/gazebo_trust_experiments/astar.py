from __future__ import annotations

import heapq
import math
from collections.abc import Iterable

from .movingai_map import MovingAIMap

Cell = tuple[int, int]


def _neighbors(grid: MovingAIMap, cell: Cell, allow_diagonal: bool) -> Iterable[tuple[Cell, float]]:
    x, y = cell
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
    if allow_diagonal:
        diagonal_cost = math.sqrt(2.0)
        moves += [
            (1, 1, diagonal_cost),
            (1, -1, diagonal_cost),
            (-1, 1, diagonal_cost),
            (-1, -1, diagonal_cost),
        ]

    for dx, dy, cost in moves:
        nx, ny = x + dx, y + dy
        if not (0 <= nx < grid.width and 0 <= ny < grid.height):
            continue
        if grid.is_blocked(nx, ny):
            continue
        if dx and dy:
            # Prevent diagonal corner-cutting through two touching walls.
            if grid.is_blocked(x + dx, y) or grid.is_blocked(x, y + dy):
                continue
        yield (nx, ny), cost


def _heuristic(a: Cell, b: Cell, allow_diagonal: bool) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if allow_diagonal:
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)
    return float(dx + dy)


def astar(grid: MovingAIMap, start: Cell, goal: Cell, *, allow_diagonal: bool = False) -> list[Cell]:
    for label, (x, y) in [('start', start), ('goal', goal)]:
        if not (0 <= x < grid.width and 0 <= y < grid.height):
            raise ValueError(f'{label} cell is outside the map: {(x, y)}')
        if grid.is_blocked(x, y):
            raise ValueError(f'{label} cell is blocked: {(x, y)}')

    frontier: list[tuple[float, int, Cell]] = [(0.0, 0, start)]
    came_from: dict[Cell, Cell | None] = {start: None}
    cost_so_far: dict[Cell, float] = {start: 0.0}
    tie_breaker = 0

    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current == goal:
            break

        for neighbor, edge_cost in _neighbors(grid, current, allow_diagonal):
            new_cost = cost_so_far[current] + edge_cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + _heuristic(neighbor, goal, allow_diagonal)
                tie_breaker += 1
                heapq.heappush(frontier, (priority, tie_breaker, neighbor))
                came_from[neighbor] = current

    if goal not in came_from:
        return []

    path = [goal]
    while path[-1] != start:
        parent = came_from[path[-1]]
        if parent is None:
            break
        path.append(parent)
    path.reverse()
    return path
