from __future__ import annotations

import math


def cell_to_world(cell_x: int, cell_y: int, *, map_height: int, cell_size: float) -> tuple[float, float]:
    return (
        (cell_x + 0.5) * cell_size,
        (map_height - cell_y - 0.5) * cell_size,
    )


def world_to_cell(x: float, y: float, *, map_height: int, cell_size: float) -> tuple[int, int]:
    return (
        int(x // cell_size),
        map_height - 1 - int(y // cell_size),
    )


def odom_to_world_pose(
    odom_x: float,
    odom_y: float,
    odom_yaw: float,
    *,
    start_cell: tuple[int, int],
    map_height: int,
    cell_size: float,
    initial_yaw: float = 0.0,
) -> tuple[float, float, float]:
    """Convert DiffDrive odometry, which starts at (0, 0, 0), into map-world coordinates."""
    start_x, start_y = cell_to_world(*start_cell, map_height=map_height, cell_size=cell_size)
    c = math.cos(initial_yaw)
    s = math.sin(initial_yaw)
    world_x = start_x + c * odom_x - s * odom_y
    world_y = start_y + s * odom_x + c * odom_y
    world_yaw = math.atan2(math.sin(initial_yaw + odom_yaw), math.cos(initial_yaw + odom_yaw))
    return world_x, world_y, world_yaw
