from __future__ import annotations

import math
from pathlib import Path
from xml.etree import ElementTree as ET


class WorldValidationError(ValueError):
    pass


def _numbers(text: str | None, where: str) -> list[float]:
    if text is None:
        raise WorldValidationError(f'Missing numeric value at {where}')
    try:
        values = [float(x) for x in text.split()]
    except ValueError as exc:
        raise WorldValidationError(f'Non-numeric value at {where}: {text!r}') from exc
    if not values or any(not math.isfinite(v) for v in values):
        raise WorldValidationError(f'Non-finite numeric value at {where}: {text!r}')
    return values


def validate_sdf_world(path: str | Path, *, coordinate_limit: float = 1.0e4) -> list[str]:
    """Return validation errors for geometry / poses that can destabilize physics.

    This is deliberately conservative.  It catches the classes of malformed SDF
    values that can produce enormous AABBs before Gazebo is allowed to run.
    """
    sdf_path = Path(path)
    errors: list[str] = []
    try:
        root = ET.parse(sdf_path).getroot()
    except (ET.ParseError, OSError) as exc:
        return [f'Unable to parse generated SDF: {exc}']

    for index, pose in enumerate(root.findall('.//pose')):
        try:
            values = _numbers(pose.text, f'pose[{index}]')
            if len(values) not in {3, 6}:
                errors.append(f'pose[{index}] must contain 3 or 6 values, got {len(values)}')
            elif any(abs(v) > coordinate_limit for v in values[:3]):
                errors.append(f'pose[{index}] position exceeds safe coordinate limit: {values[:3]}')
        except WorldValidationError as exc:
            errors.append(str(exc))

    for index, size in enumerate(root.findall('.//box/size')):
        try:
            values = _numbers(size.text, f'box.size[{index}]')
            if len(values) != 3 or any(v <= 0 for v in values):
                errors.append(f'box.size[{index}] must be three positive values: {values}')
        except WorldValidationError as exc:
            errors.append(str(exc))

    for tag in ('radius', 'length', 'mass', 'ixx', 'iyy', 'izz'):
        for index, elem in enumerate(root.findall(f'.//{tag}')):
            try:
                values = _numbers(elem.text, f'{tag}[{index}]')
                if len(values) != 1 or values[0] <= 0:
                    errors.append(f'{tag}[{index}] must be positive: {values}')
            except WorldValidationError as exc:
                errors.append(str(exc))

    return errors
