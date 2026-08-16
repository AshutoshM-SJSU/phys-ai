from __future__ import annotations

import argparse
from pathlib import Path

from .config import load_config
from .movingai_map import load_movingai_map
from .paths import resolve_from_config
from .world_generator import generate_sdf_world


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Generate a Gazebo SDF world from a MovingAI map.')
    parser.add_argument('--config', required=True, help='Experiment YAML file')
    parser.add_argument('--output', help='Output SDF path')
    return parser


def main() -> None:
    args = build_parser().parse_args()
    config = load_config(args.config)
    map_path = resolve_from_config(config.map.file, args.config)
    grid = load_movingai_map(map_path)
    output = Path(args.output) if args.output else Path('worlds') / f'{map_path.stem}.sdf'
    result = generate_sdf_world(
        grid,
        output,
        robots=config.robots,
        cell_size=config.map.cell_size,
        wall_height=config.map.wall_height,
        max_step_size=config.simulation.max_step_size,
        real_time_factor=config.simulation.real_time_factor,
    )
    print(result)


if __name__ == '__main__':
    main()
