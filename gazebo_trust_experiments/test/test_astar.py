from pathlib import Path
from gazebo_trust_experiments.astar import astar
from gazebo_trust_experiments.movingai_map import MovingAIMap


def test_astar_routes_around_wall():
    grid = MovingAIMap(
        width=5,
        height=5,
        rows=(
            '.....',
            '.@@@.',
            '.....',
            '.....',
            '.....',
        ),
        source=Path('test.map'),
    )
    path = astar(grid, (0, 0), (4, 0))
    assert path[0] == (0, 0)
    assert path[-1] == (4, 0)
    assert all(not grid.is_blocked(x, y) for x, y in path)
