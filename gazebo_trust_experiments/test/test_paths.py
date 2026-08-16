from pathlib import Path

import pytest

import gazebo_trust_experiments.paths as paths


def test_relative_map_is_resolved_from_yaml_directory(tmp_path: Path):
    config_dir = tmp_path / 'config'
    maps_dir = tmp_path / 'maps'
    config_dir.mkdir()
    maps_dir.mkdir()
    config = config_dir / 'experiment.yaml'
    config.write_text('x: 1\n', encoding='utf-8')
    map_file = maps_dir / 'room-32-32-4.map'
    map_file.write_text('dummy\n', encoding='utf-8')

    assert paths.resolve_from_config('../maps/room-32-32-4.map', config) == map_file.resolve()


def test_installed_fallback_preserves_config_relative_semantics(tmp_path: Path, monkeypatch):
    source_config_dir = tmp_path / 'source' / 'config'
    source_config_dir.mkdir(parents=True)
    source_config = source_config_dir / 'experiment.yaml'
    source_config.write_text('x: 1\n', encoding='utf-8')

    share = tmp_path / 'install' / 'share' / paths.PACKAGE_NAME
    (share / 'config').mkdir(parents=True)
    (share / 'maps').mkdir(parents=True)
    installed_map = share / 'maps' / 'room-32-32-4.map'
    installed_map.write_text('dummy\n', encoding='utf-8')
    monkeypatch.setattr(paths, 'package_share', lambda: share)

    result = paths.resolve_from_config('../maps/room-32-32-4.map', source_config)
    assert result == installed_map.resolve()


def test_missing_resource_reports_both_locations(tmp_path: Path, monkeypatch):
    config_dir = tmp_path / 'source' / 'config'
    config_dir.mkdir(parents=True)
    config = config_dir / 'experiment.yaml'
    config.write_text('x: 1\n', encoding='utf-8')
    share = tmp_path / 'install' / 'share' / paths.PACKAGE_NAME
    (share / 'config').mkdir(parents=True)
    monkeypatch.setattr(paths, 'package_share', lambda: share)

    with pytest.raises(FileNotFoundError) as exc:
        paths.resolve_from_config('../maps/missing.map', config)
    message = str(exc.value)
    assert 'source/config-relative' in message
    assert 'installed/config-relative' in message
