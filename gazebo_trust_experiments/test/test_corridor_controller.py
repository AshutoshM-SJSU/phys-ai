import math
from pathlib import Path

from gazebo_trust_experiments.config import load_config

ROOT = Path(__file__).resolve().parents[1]


def test_presentation_controller_is_corridor_conservative():
    cfg = load_config(ROOT / 'config' / 'presentation_experiment_2.yaml')
    assert cfg.planning.algorithm == 'astar'
    assert cfg.planning.allow_diagonal is False
    assert cfg.planning.linear_speed <= 0.22
    assert cfg.planning.control_rate_hz >= 12.0
    assert cfg.planning.waypoint_tolerance <= 0.06
    assert cfg.planning.heading_tolerance <= 0.10
    assert cfg.planning.emergency_forward_half_angle <= math.radians(45.0)


def test_headless_keeps_same_path_following_regime():
    cfg = load_config(ROOT / 'config' / 'experiment_2_ready.yaml')
    assert cfg.planning.algorithm == 'astar'
    assert cfg.planning.allow_diagonal is False
    assert cfg.planning.linear_speed <= 0.24
    assert cfg.planning.waypoint_tolerance <= 0.06
