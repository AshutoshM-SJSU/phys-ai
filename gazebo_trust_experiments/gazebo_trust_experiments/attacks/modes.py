from __future__ import annotations

from gazebo_trust_experiments.models import Claim
from .base import Attack, AttackContext


def _claim(ctx: AttackContext, cell: tuple[int, int], state: str, suffix: str, confidence: float) -> Claim:
    return Claim(
        source_id=ctx.attacker_id,
        cell_x=cell[0], cell_y=cell[1], state=state,
        observation_time=ctx.sim_time, reception_time=ctx.sim_time,
        confidence=confidence, kind='malicious',
        claim_id=f'{ctx.attacker_id}:{suffix}:{ctx.sim_time:.3f}:{cell[0]}:{cell[1]}',
    )


class FalseObstacleAttack(Attack):
    def generate_claims(self, context: AttackContext) -> list[Claim]:
        cells = context.candidate_cells[:int(self.parameters.get('claims_per_tick', 1))]
        confidence = float(self.parameters.get('confidence', 1.0))
        return [_claim(context, c, 'occupied', 'false_obstacle', confidence) for c in cells]


class FalseClearanceAttack(Attack):
    def generate_claims(self, context: AttackContext) -> list[Claim]:
        confidence = float(self.parameters.get('confidence', 1.0))
        cells = sorted(context.true_dynamic_cells)[:int(self.parameters.get('claims_per_tick', 1))]
        return [_claim(context, c, 'free', 'false_clearance', confidence) for c in cells]


class StaleReassertionAttack(Attack):
    def __init__(self, parameters: dict) -> None:
        super().__init__(parameters)
        self._seen_occupied: set[tuple[int, int]] = set()

    def generate_claims(self, context: AttackContext) -> list[Claim]:
        self._seen_occupied.update(context.true_dynamic_cells)
        self._seen_occupied.update(context.historical_dynamic_cells or set())
        stale = sorted(self._seen_occupied - context.true_dynamic_cells)
        confidence = float(self.parameters.get('confidence', 0.95))
        return [_claim(context, c, 'occupied', 'stale_reassertion', confidence)
                for c in stale[:int(self.parameters.get('claims_per_tick', 1))]]


class DelayedCompositeAttack(Attack):
    def __init__(self, parameters: dict) -> None:
        super().__init__(parameters)
        self.false_obstacle = FalseObstacleAttack(parameters.get('false_obstacle', parameters))
        self.false_clearance = FalseClearanceAttack(parameters.get('false_clearance', parameters))
        self.stale = StaleReassertionAttack(parameters.get('stale_reassertion', parameters))

    def generate_claims(self, context: AttackContext) -> list[Claim]:
        start = float(self.parameters.get('start_time', 60.0))
        if context.sim_time < start:
            # Honest reconnaissance is produced by the lidar reporter, not forged here.
            return []
        modes = self.parameters.get('modes', ['false_obstacle'])
        claims: list[Claim] = []
        if 'false_obstacle' in modes:
            claims.extend(self.false_obstacle.generate_claims(context))
        if 'false_clearance' in modes:
            claims.extend(self.false_clearance.generate_claims(context))
        if 'stale_reassertion' in modes:
            claims.extend(self.stale.generate_claims(context))
        return claims
