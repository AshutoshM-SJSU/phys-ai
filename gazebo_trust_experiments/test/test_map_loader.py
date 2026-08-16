from pathlib import Path

from gazebo_trust_experiments.movingai_map import load_movingai_map


def test_load_small_map(tmp_path: Path) -> None:
    map_file = tmp_path / 'tiny.map'
    map_file.write_text(
        'type octile\nheight 2\nwidth 3\nmap\n.@.\n...\n',
        encoding='utf-8',
    )
    grid = load_movingai_map(map_file)
    assert grid.width == 3
    assert grid.height == 2
    assert grid.is_blocked(1, 0)
    assert grid.is_passable(0, 1)
