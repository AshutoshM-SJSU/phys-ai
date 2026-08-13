from __future__ import annotations

from .attacks import ATTACKS


def create_attack_module(module: dict):
    attack_type = str(module.get('type', '')).strip()
    if attack_type not in ATTACKS:
        raise ValueError(f"Unknown attack type '{attack_type}'. Available: {sorted(ATTACKS)}")
    params = dict(module.get('parameters', {}))
    for key, value in module.items():
        if key not in {'type', 'parameters'}:
            params.setdefault(key, value)
    return ATTACKS[attack_type](params)
