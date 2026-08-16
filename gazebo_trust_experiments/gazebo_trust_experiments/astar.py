from __future__ import annotations

import heapq
import math
from collections.abc import Callable
from typing import Protocol


class GridLike(Protocol):
    width: int
    height: int

    def is_blocked(self, x: int, y: int) -> bool: ...


Cell = tuple[int, int]


def _heuristic(a: Cell, b: Cell, allow_diagonal: bool) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if not allow_diagonal:
        return float(dx + dy)
    # Octile distance.
    return float(max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy))


def _static_clearance_penalty(
    grid: GridLike,
    x: int,
    y: int,
    radius: int,
    weight: float,
) -> float:
    """Soft penalty for cells close to blocked geometry.

    This does not close one-cell-wide corridors.  It simply makes A* prefer
    the middle of wider corridors when an equally valid route exists.
    """
    if radius <= 0 or weight <= 0.0:
        return 0.0

    nearest = None
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx == 0 and dy == 0:
                continue
            nx = x + dx
            ny = y + dy
            if nx < 0 or ny < 0 or nx >= grid.width or ny >= grid.height:
                # Treat map boundary as wall geometry.
                distance = math.hypot(dx, dy)
            elif grid.is_blocked(nx, ny):
                distance = math.hypot(dx, dy)
            else:
                continue
            if nearest is None or distance < nearest:
                nearest = distance

    if nearest is None:
        return 0.0

    # Strongest adjacent to a wall, smoothly fading to zero.
    normalized = max(0.0, (radius + 1.0 - nearest) / (radius + 1.0))
    return weight * normalized * normalized


def astar(
    grid: GridLike,
    start: Cell,
    goal: Cell,
    *,
    allow_diagonal: bool = False,
    clearance_radius: int = 0,
    clearance_weight: float = 0.0,
    extra_cost: Callable[[int, int], float] | None = None,
) -> list[Cell]:
    """Find a path on a GridLike object.

    The original public call shape remains valid.  The optional clearance
    penalty is used by the physical robot driver so routes prefer corridor
    centers without making narrow valid corridors impossible.
    """
    for name, cell in (("start", start), ("goal", goal)):
        x, y = cell
        if x < 0 or y < 0 or x >= grid.width or y >= grid.height:
            raise ValueError(f"A* {name} cell {cell} is outside the map")
        if grid.is_blocked(x, y):
            raise ValueError(f"A* {name} cell {cell} is blocked")

    if start == goal:
        return [start]

    cardinal = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0))
    diagonal = (
        (1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (-1, -1, math.sqrt(2.0)),
    )
    moves = cardinal + diagonal if allow_diagonal else cardinal

    frontier: list[tuple[float, int, Cell]] = []
    counter = 0
    heapq.heappush(frontier, (_heuristic(start, goal, allow_diagonal), counter, start))

    came_from: dict[Cell, Cell] = {}
    g_score: dict[Cell, float] = {start: 0.0}
    closed: set[Cell] = set()

    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        closed.add(current)
        cx, cy = current

        for dx, dy, move_cost in moves:
            nx = cx + dx
            ny = cy + dy
            neighbor = (nx, ny)

            if nx < 0 or ny < 0 or nx >= grid.width or ny >= grid.height:
                continue
            if grid.is_blocked(nx, ny):
                continue

            # Prevent diagonal corner cutting through two touching walls.
            if dx != 0 and dy != 0:
                if grid.is_blocked(cx + dx, cy) or grid.is_blocked(cx, cy + dy):
                    continue

            penalty = _static_clearance_penalty(
                grid, nx, ny, clearance_radius, clearance_weight
            )
            if extra_cost is not None:
                penalty += max(0.0, float(extra_cost(nx, ny)))

            tentative = g_score[current] + move_cost + penalty
            if tentative >= g_score.get(neighbor, math.inf):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative
            counter += 1
            f_score = tentative + _heuristic(neighbor, goal, allow_diagonal)
            heapq.heappush(frontier, (f_score, counter, neighbor))

    return []
