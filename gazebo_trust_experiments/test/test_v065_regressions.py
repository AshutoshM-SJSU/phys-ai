from pathlib import Path

from gazebo_trust_experiments.config import load_config
from gazebo_trust_experiments.world_generator import generate_sdf_world
from gazebo_trust_experiments.movingai_map import MovingAIMap


def test_presentation_has_discrete_drive_parameters():
    cfg = load_config(Path(__file__).parents[1] / 'config' / 'presentation_experiment_2.yaml')
    assert cfg.planning.drive_realign_tolerance <= cfg.planning.heading_tolerance
    assert cfg.planning.turn_settle_cycles >= 2
    assert cfg.planning.waypoint_stop_cycles >= 1


def test_world_has_physical_perimeter(tmp_path):
    grid = MovingAIMap(width=3, height=3, rows=('...', '...', '...'), source=Path('synthetic.map'))
    out = generate_sdf_world(grid, tmp_path / 'world.sdf', robots=[], cell_size=0.5)
    text = out.read_text()
    for name in ('boundary_north', 'boundary_south', 'boundary_east', 'boundary_west'):
        assert name in text
