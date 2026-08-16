from __future__ import annotations

from .base import Attack, AttackContext, Claim


class ScriptedFalseObstacleAttack(Attack):
    """Inject one false occupied claim at a configured time and path index."""

    def __init__(self, parameters: dict) -> None:
        super().__init__(parameters)
        self._emitted = False

    def generate_claims(self, context: AttackContext) -> list[Claim]:
        if self._emitted:
            return []

        start_time = float(self.parameters.get('start_time', 30.0))
        if context.sim_time < start_time or not context.shortest_path:
            return []

        path_fraction = float(self.parameters.get('path_fraction', 0.5))
        path_fraction = min(1.0, max(0.0, path_fraction))
        index = round(path_fraction * (len(context.shortest_path) - 1))
        cell_x, cell_y = context.shortest_path[index]
        self._emitted = True

        return [
            Claim(
                robot_id=str(self.parameters.get('robot_id', 'attacker_1')),
                cell_x=cell_x,
                cell_y=cell_y,
                report_type='occupied',
                timestamp=context.sim_time,
                confidence=float(self.parameters.get('confidence', 1.0)),
            )
        ]
