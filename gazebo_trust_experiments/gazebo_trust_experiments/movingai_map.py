from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


PASSABLE = {'.', 'G', 'S'}
BLOCKED = {'@', 'O', 'T', 'W'}


class MapFormatError(ValueError):
    pass


@dataclass(frozen=True)
class MovingAIMap:
    width: int
    height: int
    rows: tuple[str, ...]
    source: Path

    def is_blocked(self, x: int, y: int) -> bool:
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return True
        return self.rows[y][x] in BLOCKED

    def is_passable(self, x: int, y: int) -> bool:
        return not self.is_blocked(x, y)


def load_movingai_map(path: str | Path) -> MovingAIMap:
    source = Path(path).expanduser().resolve()
    lines = source.read_text(encoding='utf-8').splitlines()
    if len(lines) < 5:
        raise MapFormatError(f'Map is too short: {source}')
    if not lines[0].lower().startswith('type '):
        raise MapFormatError("First line must begin with 'type'")

    try:
        height = int(lines[1].split()[1])
        width = int(lines[2].split()[1])
    except (IndexError, ValueError) as exc:
        raise MapFormatError('Invalid height or width header') from exc

    if lines[3].strip().lower() != 'map':
        raise MapFormatError("Fourth header line must be 'map'")

    rows = tuple(lines[4:4 + height])
    if len(rows) != height:
        raise MapFormatError(f'Expected {height} rows, found {len(rows)}')
    if any(len(row) != width for row in rows):
        raise MapFormatError('At least one map row has the wrong width')

    unknown = sorted({char for row in rows for char in row} - PASSABLE - BLOCKED)
    if unknown:
        raise MapFormatError(f'Unsupported map symbols: {unknown}')

    return MovingAIMap(width=width, height=height, rows=rows, source=source)
