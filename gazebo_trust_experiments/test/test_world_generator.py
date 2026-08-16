from pathlib import Path

from gazebo_trust_experiments.movingai_map import load_movingai_map
from gazebo_trust_experiments.world_generator import generate_sdf_world, horizontal_wall_runs


def test_horizontal_runs_and_sdf(tmp_path: Path) -> None:
    map_file = tmp_path / 'tiny.map'
    map_file.write_text(
        'type octile\nheight 2\nwidth 4\nmap\n@@..\n.@@@\n',
        encoding='utf-8',
    )
    grid = load_movingai_map(map_file)
    runs = horizontal_wall_runs(grid)
    assert [(r.x0, r.x1, r.y) for r in runs] == [(0, 1, 0), (1, 3, 1)]
    world = generate_sdf_world(grid, tmp_path / 'world.sdf')
    assert world.is_file()
    assert 'wall_00001' in world.read_text(encoding='utf-8')
