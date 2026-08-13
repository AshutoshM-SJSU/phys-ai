from gazebo_trust_experiments.full_trust_map import FullTrustMap
from gazebo_trust_experiments.models import Claim


def claim(state: str, reception: float, claim_id: str) -> Claim:
    return Claim('r1', 4, 5, state, reception, reception, 1.0, 'shared', claim_id)


def test_latest_received_claim_controls_cell():
    grid = FullTrustMap()
    assert grid.ingest(claim('occupied', 1.0, 'a'))
    assert (4, 5) in grid.occupied_cells()
    assert grid.ingest(claim('free', 2.0, 'b'))
    assert (4, 5) not in grid.occupied_cells()
    assert not grid.ingest(claim('occupied', 0.5, 'old'))
    assert grid.state((4, 5)) == 'free'
