from pathlib import Path

from gazebo_trust_experiments.movingai_map import MovingAIMap
from gazebo_trust_experiments.world_generator import generate_sdf_world


def test_world_does_not_bake_gui_guides_by_default(tmp_path: Path) -> None:
    grid = MovingAIMap(source=Path('synthetic.map'), width=5, height=5, rows=('.....',) * 5)
    robot = {
        'id': 'robot_0',
        'start_cell': [0, 0],
        'goal_nodes': [[4, 4], [0, 0]],
        'initial_goal_index': 0,
        'color': '0.1 0.4 0.8 1',
    }
    out = generate_sdf_world(
        grid,
        tmp_path / 'world.sdf',
        robots=[robot],
        cell_size=0.5,
        allow_diagonal=False,
        visualization={'enabled': True, 'show_goals': True, 'show_initial_routes': True},
    )
    text = out.read_text()
    assert 'delivery_goalposts' not in text
    assert 'robot_0_initial_route' not in text


def test_world_can_bake_gui_guides_for_debug(tmp_path: Path) -> None:
    grid = MovingAIMap(source=Path('synthetic.map'), width=5, height=5, rows=('.....',) * 5)
    robot = {
        'id': 'robot_0',
        'start_cell': [0, 0],
        'goal_nodes': [[4, 4], [0, 0]],
        'initial_goal_index': 0,
        'color': '0.1 0.4 0.8 1',
    }
    out = generate_sdf_world(
        grid, tmp_path / 'world_baked.sdf', robots=[robot],
        visualization={'enabled': True, 'show_goals': True, 'show_initial_routes': True},
        bake_visual_guides=True,
    )
    text = out.read_text()
    assert 'delivery_goalposts' in text
    assert 'robot_0_initial_route' in text


def test_world_can_disable_visual_guides(tmp_path: Path) -> None:
    grid = MovingAIMap(source=Path('synthetic.map'), width=3, height=3, rows=('...',) * 3)
    robot = {'id': 'robot_0', 'start_cell': [0, 0], 'goal_nodes': [[2, 2], [0, 0]]}
    out = generate_sdf_world(grid, tmp_path / 'world.sdf', robots=[robot], visualization={'enabled': False})
    text = out.read_text()
    assert 'robot_0_goal_markers' not in text
    assert 'robot_0_initial_route' not in text


def test_runner_uses_separate_server_and_gui_processes() -> None:
    runner = Path('gazebo_trust_experiments/runner.py').read_text()
    assert "['gz', 'sim', '-s', '-r'" in runner
    assert "['gz', 'sim', '-g', '--render-engine', 'ogre']" in runner
    assert "processes['gazebo_server']" in runner
    assert "processes['gazebo']" not in runner


def test_world_uses_explicit_stable_dart_profile(tmp_path: Path) -> None:
    grid = MovingAIMap(source=Path('synthetic.map'), width=3, height=3, rows=('...',) * 3)
    out = generate_sdf_world(grid, tmp_path / 'physics.sdf', collision_detector='fcl', solver_type='pgs')
    text = out.read_text()
    assert '<collision_detector>' not in text
    assert '<solver_type>' not in text
    assert '<max_contacts>8</max_contacts>' in text
    assert '<noise type=' not in text
