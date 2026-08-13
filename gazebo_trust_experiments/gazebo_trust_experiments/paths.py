from __future__ import annotations

from pathlib import Path

PACKAGE_NAME = 'gazebo_trust_experiments'


def package_share() -> Path:
    # Import lazily so pure-Python tooling/tests can load path helpers even
    # outside a sourced ROS environment.
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(PACKAGE_NAME))


def resolve_from_config(value: str, config_path: str | Path) -> Path:
    """Resolve an input path using the YAML file as the reference directory.

    Relative paths such as ``../maps/room-32-32-4.map`` are always interpreted
    relative to the directory containing the YAML. If the caller is using a
    source-tree YAML whose resource has not been downloaded there yet, we also
    try the identically arranged installed package tree (share/<pkg>/config).
    """
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        resolved = candidate.resolve()
        if resolved.exists():
            return resolved
        raise FileNotFoundError(f'Input resource does not exist: {resolved}')

    config_path = Path(config_path).expanduser().resolve()
    beside_config = (config_path.parent / candidate).resolve()
    if beside_config.exists():
        return beside_config

    # Keep exactly the same relative semantics as a config file installed at
    # share/<package>/config/<name>.yaml.  Using package_share()/candidate here
    # would incorrectly turn ../maps/... into share/maps/....
    installed_config_dir = package_share() / 'config'
    installed = (installed_config_dir / candidate).resolve()
    if installed.exists():
        return installed

    raise FileNotFoundError(
        'Input resource could not be resolved. Tried:\n'
        f'  source/config-relative: {beside_config}\n'
        f'  installed/config-relative: {installed}\n'
        'If this is a MovingAI map, run scripts/download_maps.py or the '
        'bootstrap_experiment.sh script before building.'
    )


def resolve_output_from_config(value: str, config_path: str | Path) -> Path:
    """Resolve an output path relative to the YAML even when it does not exist."""
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return candidate.resolve()
    return (Path(config_path).expanduser().resolve().parent / candidate).resolve()
