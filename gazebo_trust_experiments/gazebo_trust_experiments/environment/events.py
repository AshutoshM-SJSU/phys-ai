from __future__ import annotations
from dataclasses import dataclass
from typing import Any

@dataclass(frozen=True)
class TemporaryObstacle:
    obstacle_id: str
    cell: tuple[int,int]
    appear_time: float
    disappear_time: float
    size: tuple[float,float,float] = (0.4,0.4,0.6)
    color: str = '0.9 0.35 0.05 1'

    @classmethod
    def from_dict(cls, raw: dict[str,Any]) -> 'TemporaryObstacle':
        cell=raw['cell']; return cls(
            obstacle_id=str(raw['id']), cell=(int(cell[0]),int(cell[1])),
            appear_time=float(raw['appear_time']), disappear_time=float(raw['disappear_time']),
            size=tuple(float(v) for v in raw.get('size',[0.4,0.4,0.6])),
            color=str(raw.get('color','0.9 0.35 0.05 1')),
        )
