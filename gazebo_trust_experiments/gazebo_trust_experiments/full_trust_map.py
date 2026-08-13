from __future__ import annotations

from dataclasses import dataclass

from .models import Claim


@dataclass
class CellState:
    state: str
    reception_time: float
    claim_id: str
    source_id: str
    kind: str


class FullTrustMap:
    """Default experiment map: every claim is accepted at face value.

    The latest received claim for a cell controls its dynamic state. This is intentionally
    policy-light. It gives the experiment infrastructure a neutral default without embedding
    any trust defense or comparison baseline.
    """

    def __init__(self) -> None:
        self.cells: dict[tuple[int, int], CellState] = {}

    def ingest(self, claim: Claim) -> bool:
        key = (claim.cell_x, claim.cell_y)
        previous = self.cells.get(key)
        if previous is not None and previous.reception_time > claim.reception_time:
            return False
        updated = CellState(
            state=claim.state,
            reception_time=claim.reception_time,
            claim_id=claim.claim_id,
            source_id=claim.source_id,
            kind=claim.kind,
        )
        changed = previous is None or previous.state != updated.state or previous.claim_id != updated.claim_id
        self.cells[key] = updated
        return changed

    def occupied_cells(self) -> set[tuple[int, int]]:
        return {cell for cell, value in self.cells.items() if value.state == 'occupied'}

    def state(self, cell: tuple[int, int]) -> str | None:
        value = self.cells.get(cell)
        return None if value is None else value.state
