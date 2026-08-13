from pathlib import Path
import tempfile

from gazebo_trust_experiments.config import load_config


def test_experiment_two_config_shape():
    source = Path(__file__).parents[1] / 'config' / 'experiment_2_ready.yaml'
    text = source.read_text(encoding='utf-8').replace('../maps/room-32-32-4.map', 'dummy.map')
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'config.yaml'
        path.write_text(text, encoding='utf-8')
        config = load_config(path)
    assert len(config.robots) == 3
    assert config.mapping['mode'] == 'full_trust'
    assert {m['type'] for m in config.attack['modules']} == {'false_obstacle', 'false_clearance', 'stale_reassertion'}
