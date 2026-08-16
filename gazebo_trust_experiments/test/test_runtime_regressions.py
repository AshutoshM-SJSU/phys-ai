from pathlib import Path

from gazebo_trust_experiments.movingai_map import MovingAIMap
from gazebo_trust_experiments.world_generator import generate_sdf_world


def test_generated_wheel_axis_is_model_y(tmp_path):
    grid = MovingAIMap(source=Path("synthetic.map"), width=3, height=3, rows=("...", "...", "..."))
    out = generate_sdf_world(grid, tmp_path / "world.sdf", robots=[{"id": "robot_0", "start_cell": [1, 1]}])
    text = Path(out).read_text()
    assert 'expressed_in="__model__"' in text
    assert '>0 1 0</xyz>' in text
    assert 'relative_to="left_wheel"' in text


def test_lidar_is_mounted_above_chassis(tmp_path):
    grid = MovingAIMap(source=Path("synthetic.map"), width=3, height=3, rows=("...", "...", "..."))
    out = generate_sdf_world(grid, tmp_path / "world.sdf", robots=[{"id": "robot_0", "start_cell": [1, 1]}])
    text = Path(out).read_text()
    assert 'relative_to="chassis">0.02 0 0.14 0 0 0</pose>' in text
    assert '<min>0.12</min>' in text


def test_reference_diff_drive_layout_is_canonical(tmp_path):
    grid = MovingAIMap(source=Path("synthetic.map"), width=3, height=3, rows=("...", "...", "..."))
    out = generate_sdf_world(grid, tmp_path / "world.sdf", robots=[{"id": "robot_0", "start_cell": [1, 1]}])
    text = Path(out).read_text()
    assert 'canonical_link="chassis"' in text
    assert 'relative_to="chassis">-0.055 0.13 0 -1.57079632679 0 0</pose>' in text
    assert '<wheel_separation>0.26</wheel_separation>' in text
    assert '<wheel_radius>0.065</wheel_radius>' in text
