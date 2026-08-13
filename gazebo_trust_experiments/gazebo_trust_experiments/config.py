from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class SimulationConfig:
    headless: bool
    render_engine: str
    real_time_factor: float
    max_step_size: float
    timeout_sim_seconds: float
    seed: int
    startup_delay_seconds: float
    startup_timeout_seconds: float
    collision_detector: str
    solver_type: str


@dataclass(frozen=True)
class MapConfig:
    file: str
    cell_size: float
    wall_height: float


@dataclass(frozen=True)
class PlanningConfig:
    algorithm: str
    allow_diagonal: bool
    replan_every_steps: int
    control_rate_hz: float
    linear_speed: float
    angular_speed: float
    waypoint_tolerance: float
    goal_tolerance: float
    heading_tolerance: float
    corner_slow_distance: float
    min_linear_speed: float
    emergency_stop_distance: float
    near_collision_distance: float
    emergency_forward_half_angle: float
    drive_realign_tolerance: float
    turn_settle_cycles: int
    waypoint_stop_cycles: int


@dataclass(frozen=True)
class ExperimentConfig:
    name: str
    output_dir: str
    map: MapConfig
    simulation: SimulationConfig
    planning: PlanningConfig
    robots: list[dict[str, Any]]
    environment: dict[str, Any]
    network: dict[str, Any]
    attack: dict[str, Any]
    mapping: dict[str, Any]
    sensing: dict[str, Any]
    metrics: dict[str, Any]
    visualization: dict[str, Any]


def _require(mapping: dict[str, Any], key: str, where: str) -> Any:
    if key not in mapping:
        raise ConfigError(f"Missing required key '{where}.{key}'")
    return mapping[key]


def _cell(value: Any, where: str) -> tuple[int, int]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ConfigError(f"{where} must be [x, y]")
    return int(value[0]), int(value[1])


def load_config(path: str | Path) -> ExperimentConfig:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise ConfigError(f'Configuration file does not exist: {config_path}')
    raw = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if not isinstance(raw, dict):
        raise ConfigError('Top-level YAML value must be a mapping')

    map_raw = _require(raw, 'map', 'root')
    sim_raw = _require(raw, 'simulation', 'root')
    plan_raw = raw.get('planning', {})
    if not all(isinstance(x, dict) for x in (map_raw, sim_raw, plan_raw)):
        raise ConfigError('map, simulation, and planning must be mappings')

    map_cfg = MapConfig(
        file=str(_require(map_raw, 'file', 'map')),
        cell_size=float(map_raw.get('cell_size', 0.5)),
        wall_height=float(map_raw.get('wall_height', 1.0)),
    )
    sim_cfg = SimulationConfig(
        headless=bool(sim_raw.get('headless', True)),
        render_engine=str(sim_raw.get('render_engine', 'ogre')).lower(),
        real_time_factor=float(sim_raw.get('real_time_factor', 1.0)),
        max_step_size=float(sim_raw.get('max_step_size', 0.002)),
        timeout_sim_seconds=float(sim_raw.get('timeout_sim_seconds', 300.0)),
        seed=int(sim_raw.get('seed', 1)),
        startup_delay_seconds=float(sim_raw.get('startup_delay_seconds', 3.0)),
        startup_timeout_seconds=float(sim_raw.get('startup_timeout_seconds', 20.0)),
        collision_detector=str(sim_raw.get('collision_detector', 'fcl')).lower(),
        solver_type=str(sim_raw.get('solver_type', 'pgs')).lower(),
    )
    planning_cfg = PlanningConfig(
        algorithm=str(plan_raw.get('algorithm', 'astar')).lower(),
        allow_diagonal=bool(plan_raw.get('allow_diagonal', False)),
        replan_every_steps=int(plan_raw.get('replan_every_steps', 10)),
        control_rate_hz=float(plan_raw.get('control_rate_hz', 10.0)),
        linear_speed=float(plan_raw.get('linear_speed', 0.30)),
        angular_speed=float(plan_raw.get('angular_speed', 1.1)),
        waypoint_tolerance=float(plan_raw.get('waypoint_tolerance', 0.06)),
        goal_tolerance=float(plan_raw.get('goal_tolerance', 0.14)),
        heading_tolerance=float(plan_raw.get('heading_tolerance', 0.10)),
        corner_slow_distance=float(plan_raw.get('corner_slow_distance', 0.18)),
        min_linear_speed=float(plan_raw.get('min_linear_speed', 0.08)),
        emergency_stop_distance=float(plan_raw.get('emergency_stop_distance', 0.18)),
        near_collision_distance=float(plan_raw.get('near_collision_distance', 0.30)),
        emergency_forward_half_angle=float(plan_raw.get('emergency_forward_half_angle', 0.70)),
        drive_realign_tolerance=float(plan_raw.get('drive_realign_tolerance', 0.045)),
        turn_settle_cycles=int(plan_raw.get('turn_settle_cycles', 3)),
        waypoint_stop_cycles=int(plan_raw.get('waypoint_stop_cycles', 2)),
    )

    if map_cfg.cell_size <= 0 or map_cfg.wall_height <= 0:
        raise ConfigError('Map dimensions must be positive')
    if sim_cfg.max_step_size <= 0 or sim_cfg.real_time_factor <= 0 or sim_cfg.timeout_sim_seconds <= 0:
        raise ConfigError('Simulation timing values must be positive')
    if sim_cfg.render_engine != 'ogre':
        raise ConfigError("Only the 'ogre' rendering engine is supported for GUI rendering")
    if sim_cfg.collision_detector not in {'fcl', 'bullet', 'dart', 'ode'}:
        raise ConfigError('simulation.collision_detector must be one of: fcl, bullet, dart, ode')
    if sim_cfg.solver_type not in {'pgs', 'dantzig'}:
        raise ConfigError('simulation.solver_type must be pgs or dantzig')
    if planning_cfg.algorithm != 'astar':
        raise ConfigError("Only the 'astar' planner is implemented")
    if planning_cfg.replan_every_steps < 1 or planning_cfg.control_rate_hz <= 0:
        raise ConfigError('Planning cadence values must be positive')
    if planning_cfg.linear_speed <= 0 or planning_cfg.angular_speed <= 0:
        raise ConfigError('Planning speeds must be positive')
    if planning_cfg.waypoint_tolerance <= 0 or planning_cfg.goal_tolerance <= 0:
        raise ConfigError('Planning tolerances must be positive')
    if not (0 < planning_cfg.heading_tolerance < 1.0):
        raise ConfigError('planning.heading_tolerance must be between 0 and 1 radian')
    if not (0 < planning_cfg.drive_realign_tolerance <= planning_cfg.heading_tolerance):
        raise ConfigError('planning.drive_realign_tolerance must be positive and <= heading_tolerance')
    if planning_cfg.turn_settle_cycles < 1 or planning_cfg.waypoint_stop_cycles < 1:
        raise ConfigError('planning turn/waypoint settle cycles must be positive')

    robots = raw.get('robots', [])
    if not isinstance(robots, list) or len(robots) != 3:
        raise ConfigError('Experiment 2 requires exactly three robots')
    ids: set[str] = set()
    for index, robot in enumerate(robots):
        if not isinstance(robot, dict):
            raise ConfigError(f'robots[{index}] must be a mapping')
        rid = str(_require(robot, 'id', f'robots[{index}]')).strip()
        if not rid or rid in ids:
            raise ConfigError(f'Robot IDs must be unique and non-empty: {rid!r}')
        ids.add(rid)
        _cell(_require(robot, 'start_cell', f'robots[{index}]'), f'robots[{index}].start_cell')
        goals = robot.get('goal_nodes')
        if isinstance(goals, str) and goals.lower() == 'auto':
            if int(robot.get('goal_count', 12)) < 2:
                raise ConfigError(f'robots[{index}].goal_count must be at least 2 when goal_nodes is auto')
        else:
            if not isinstance(goals, list) or len(goals) < 2:
                raise ConfigError(f'robots[{index}].goal_nodes must contain at least two cells or be auto')
            for goal_index, goal in enumerate(goals):
                _cell(goal, f'robots[{index}].goal_nodes[{goal_index}]')

    attack = raw.get('attack', {'enabled': False, 'modules': []})
    if not isinstance(attack, dict):
        raise ConfigError('attack must be a mapping')
    modules = attack.get('modules', [])
    if not isinstance(modules, list):
        raise ConfigError('attack.modules must be a list')

    return ExperimentConfig(
        name=str(raw.get('name', config_path.stem)),
        output_dir=str(raw.get('output_dir', '../results')),
        map=map_cfg,
        simulation=sim_cfg,
        planning=planning_cfg,
        robots=robots,
        environment=raw.get('environment', {}) or {},
        network=raw.get('network', {}) or {},
        attack=attack,
        mapping=raw.get('mapping', {}) or {},
        sensing=raw.get('sensing', {}) or {},
        metrics=raw.get('metrics', {}) or {},
        visualization=raw.get('visualization', {}) or {},
    )


def apply_overrides(
    config: ExperimentConfig,
    *,
    map_file: str | None = None,
    headless: bool | None = None,
    real_time_factor: float | None = None,
    max_step_size: float | None = None,
    seed: int | None = None,
    replan_every_steps: int | None = None,
    output_dir: str | None = None,
    run_name: str | None = None,
) -> ExperimentConfig:
    return replace(
        config,
        name=config.name if not run_name else run_name,
        output_dir=config.output_dir if not output_dir else output_dir,
        map=replace(config.map, file=map_file) if map_file else config.map,
        simulation=replace(
            config.simulation,
            headless=config.simulation.headless if headless is None else headless,
            real_time_factor=config.simulation.real_time_factor if real_time_factor is None else real_time_factor,
            max_step_size=config.simulation.max_step_size if max_step_size is None else max_step_size,
            seed=config.simulation.seed if seed is None else seed,
        ),
        planning=replace(
            config.planning,
            replan_every_steps=config.planning.replan_every_steps if replan_every_steps is None else replan_every_steps,
        ),
    )
