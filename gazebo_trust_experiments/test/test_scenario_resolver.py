from pathlib import Path

from gazebo_trust_experiments.config import load_config
from gazebo_trust_experiments.movingai_map import load_movingai_map
from gazebo_trust_experiments.scenario_resolver import resolve_scenario


def test_auto_placements_resolve_to_free_cells(tmp_path: Path):
    map_path = tmp_path / 'map.map'
    map_path.write_text(
        'type octile\nheight 8\nwidth 8\nmap\n'
        '........\n'
        '........\n'
        '..@@....\n'
        '..@@....\n'
        '........\n'
        '........\n'
        '........\n'
        '........\n',
        encoding='utf-8',
    )
    cfg_path = tmp_path / 'config.yaml'
    cfg_path.write_text(
        f'''name: test\noutput_dir: results\nmap:\n  file: {map_path}\n  cell_size: 0.5\n  wall_height: 1.0\nsimulation:\n  headless: true\n  render_engine: ogre\n  real_time_factor: 1.0\n  max_step_size: 0.002\n  timeout_sim_seconds: 10\n  seed: 1\nplanning:\n  algorithm: astar\nrobots:\n  - id: robot_0\n    role: benign_delivery\n    start_cell: [0, 0]\n    goal_nodes: [[7, 7], [0, 0]]\n  - id: robot_1\n    role: benign_delivery\n    start_cell: [7, 0]\n    goal_nodes: [[0, 7], [7, 0]]\n  - id: attacker_1\n    role: delayed_attacker\n    start_cell: [0, 7]\n    goal_nodes: [[7, 0], [0, 7]]\nenvironment:\n  temporary_obstacles:\n    - id: box1\n      cell: auto\n      placement: {{robot_id: robot_0, route_fraction: 0.45}}\n      appear_time: 1\n      disappear_time: 2\nattack:\n  enabled: true\n  robot_id: attacker_1\n  modules:\n    - name: fake\n      type: false_obstacle\n      start_time: 3\n      end_time: 8\n      publish_period: 1\n      candidate_cells: auto\n      placement:\n        robot_id: robot_0\n        route_fractions: [0.3, 0.5, 0.7]\n''',
        encoding='utf-8',
    )
    cfg = load_config(cfg_path)
    grid = load_movingai_map(map_path)
    resolved = resolve_scenario(cfg, grid)
    obstacle_cell = tuple(resolved.environment['temporary_obstacles'][0]['cell'])
    assert not grid.is_blocked(*obstacle_cell)
    candidates = resolved.attack['modules'][0]['candidate_cells']
    assert len(candidates) == 3
    assert all(not grid.is_blocked(*cell) for cell in candidates)
    assert obstacle_cell not in {tuple(cell) for cell in candidates}


def test_auto_goal_pool_is_shared_and_spread(tmp_path: Path):
    map_path = tmp_path / 'open.map'
    rows = ['.' * 16 for _ in range(16)]
    map_path.write_text(
        'type octile\nheight 16\nwidth 16\nmap\n' + '\n'.join(rows) + '\n',
        encoding='utf-8',
    )
    cfg_path = tmp_path / 'auto_goals.yaml'
    cfg_path.write_text(
        f'''name: goals
output_dir: results
map:
  file: {map_path}
  cell_size: 0.5
  wall_height: 1.0
simulation:
  headless: true
  render_engine: ogre
  real_time_factor: 1
  max_step_size: 0.01
  timeout_sim_seconds: 10
  seed: 7
planning:
  algorithm: astar
robots:
  - id: robot_0
    role: benign_delivery
    start_cell: [1, 1]
    goal_nodes: auto
    goal_count: 10
  - id: robot_1
    role: benign_delivery
    start_cell: [14, 1]
    goal_nodes: auto
    goal_count: 10
  - id: attacker_1
    role: delayed_attacker
    start_cell: [1, 14]
    goal_nodes: auto
    goal_count: 10
environment:
  delivery_goal_pool:
    count: 10
    min_separation_cells: 3
    edge_margin_cells: 2
attack:
  enabled: false
  modules: []
''',
        encoding='utf-8',
    )
    cfg = load_config(cfg_path)
    grid = load_movingai_map(map_path)
    resolved = resolve_scenario(cfg, grid)
    pools = [robot['goal_nodes'] for robot in resolved.robots]
    assert pools[0] == pools[1] == pools[2]
    assert len(pools[0]) == 10
    xs = {cell[0] for cell in pools[0]}
    ys = {cell[1] for cell in pools[0]}
    assert len(xs) >= 4 and len(ys) >= 4
    assert all(2 <= x < 14 and 2 <= y < 14 for x, y in pools[0])
