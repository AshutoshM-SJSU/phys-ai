from pathlib import Path

from gazebo_trust_experiments.movingai_map import MovingAIMap
from gazebo_trust_experiments.world_generator import generate_sdf_world
from gazebo_trust_experiments.world_validation import validate_sdf_world


def test_generated_world_has_safe_numeric_geometry(tmp_path: Path):
    grid = MovingAIMap(width=4, height=4, rows=('....', '....', '....', '....'), source=tmp_path / 'map.map')
    robots = [
        {'id': 'r0', 'start_cell': [0, 0]},
        {'id': 'r1', 'start_cell': [3, 0]},
        {'id': 'r2', 'start_cell': [0, 3]},
    ]
    path = generate_sdf_world(grid, tmp_path / 'world.sdf', robots=robots, cell_size=0.5)
    assert validate_sdf_world(path) == []


def test_generated_world_avoids_extreme_joint_limits_and_dart_override(tmp_path: Path):
    grid = MovingAIMap(width=4, height=4, rows=('....', '....', '....', '....'), source=tmp_path / 'map.map')
    robots = [
        {'id': 'r0', 'start_cell': [0, 0]},
        {'id': 'r1', 'start_cell': [3, 0]},
        {'id': 'r2', 'start_cell': [0, 3]},
    ]
    path = generate_sdf_world(grid, tmp_path / 'world.sdf', robots=robots, cell_size=0.5)
    text = path.read_text()
    assert '1.79769e+308' not in text
    assert '<collision_detector>' not in text
    assert '<solver_type>' not in text
