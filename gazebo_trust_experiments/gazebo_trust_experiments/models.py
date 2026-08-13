from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Any, Literal

ClaimState = Literal['occupied', 'free']
ClaimKind = Literal['direct', 'shared', 'malicious']


@dataclass(frozen=True)
class Claim:
    source_id: str
    cell_x: int
    cell_y: int
    state: ClaimState
    observation_time: float
    reception_time: float
    confidence: float = 1.0
    kind: ClaimKind = 'shared'
    claim_id: str = ''

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(',', ':'))

    @classmethod
    def from_json(cls, payload: str) -> 'Claim':
        raw: dict[str, Any] = json.loads(payload)
        return cls(**raw)


@dataclass(frozen=True)
class ExperimentEvent:
    event_type: str
    sim_time: float
    robot_id: str = ''
    details: dict[str, Any] | None = None

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(',', ':'))
