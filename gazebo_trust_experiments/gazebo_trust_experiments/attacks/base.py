from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from gazebo_trust_experiments.models import Claim


@dataclass(frozen=True)
class AttackContext:
    sim_time: float
    attacker_id: str
    observed_cells: dict[tuple[int, int], str]
    true_dynamic_cells: set[tuple[int, int]]
    candidate_cells: list[tuple[int, int]]
    historical_dynamic_cells: set[tuple[int, int]] | None = None


class Attack(ABC):
    def __init__(self, parameters: dict[str, Any]) -> None:
        self.parameters = parameters

    @abstractmethod
    def generate_claims(self, context: AttackContext) -> list[Claim]:
        raise NotImplementedError
