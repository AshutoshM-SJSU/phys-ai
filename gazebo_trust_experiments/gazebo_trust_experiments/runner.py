from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import time
from dataclasses import asdict, replace
from pathlib import Path

from .config import apply_overrides, load_config
from .movingai_map import load_movingai_map
from .paths import resolve_from_config, resolve_output_from_config
from .preflight import validate_experiment
from .scenario_resolver import resolve_scenario
from .world_generator import generate_sdf_world
from .world_validation import validate_sdf_world


def _parse_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    normalized = value.strip().lower()
    if normalized in {'1', 'true', 'yes', 'on'}:
        return True
    if normalized in {'0', 'false', 'no', 'off'}:
        return False
    raise argparse.ArgumentTypeError(f'Invalid boolean value: {value}')


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Validate, prepare, and run Gazebo Experiment 2.')
    parser.add_argument('--config', required=True)
    parser.add_argument('--map-file', default=None)
    parser.add_argument('--headless', nargs='?', const=True, default=None, type=_parse_bool)
    parser.add_argument('--fast-headless', action='store_true', help='Run server-only with stable fast defaults (6x target RTF, 4 ms physics step unless explicitly overridden).')
    parser.add_argument('--real-time-factor', type=float, default=None)
    parser.add_argument('--max-step-size', type=float, default=None)
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--replan-every-steps', type=int, default=None)
    parser.add_argument('--output-dir', default=None)
    parser.add_argument('--run-name', default=None)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--no-controllers', action='store_true')
    parser.add_argument('--smoke-test', action='store_true', help='Generate the full world, start only Gazebo server, verify required topics, and keep physics alive briefly. No ROS experiment nodes are launched.')
    parser.add_argument('--smoke-test-seconds', type=float, default=8.0, help='Wall-clock seconds Gazebo must remain alive after readiness during --smoke-test.')
    return parser


def _terminate(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def _start(command: list[str], log_path: Path) -> subprocess.Popen:
    handle = log_path.open('wb')
    process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT)
    process._experiment_log_handle = handle  # type: ignore[attr-defined]
    return process


def _close_log(process: subprocess.Popen | None) -> None:
    if process is None:
        return
    handle = getattr(process, '_experiment_log_handle', None)
    if handle is not None:
        handle.close()


def _gz_topics() -> set[str]:
    try:
        result = subprocess.run(['gz', 'topic', '-l'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=3, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return set()
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def _wait_for_gazebo_server(process: subprocess.Popen, robot_ids: list[str], timeout_seconds: float) -> None:
    deadline = time.monotonic() + timeout_seconds
    required = {'/clock'} | {f'/{rid}/odometry' for rid in robot_ids}
    last_topics: set[str] = set()
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f'Gazebo server exited during startup with code {process.returncode}')
        last_topics = _gz_topics()
        if required.issubset(last_topics):
            return
        time.sleep(0.5)
    missing = sorted(required - last_topics)
    raise RuntimeError(f'Gazebo server did not become ready within {timeout_seconds:.1f}s; missing topics: {missing}')




def _existing_experiment_nodes(robot_ids: list[str]) -> list[str]:
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=4,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    known = {
        '/attack_manager', '/claim_network', '/experiment_supervisor',
        '/experiment_visualization', '/metrics_collector', '/environment_manager',
    }
    for rid in robot_ids:
        known.update({
            f'/{rid}_astar_driver', f'/{rid}_lidar_reporter', f'/{rid}_shared_map'
        })
    return sorted({line.strip() for line in result.stdout.splitlines() if line.strip() in known})

def _check_critical_processes(processes: dict[str, subprocess.Popen], run_dir: Path) -> None:
    failed = []
    for name, process in processes.items():
        if name in {'gui'}:
            continue
        code = process.poll()
        if code is not None and code != 0:
            failed.append(f'{name} exited with code {code} (see {run_dir / (name + ".log")})')
    if failed:
        raise RuntimeError('Experiment process startup failure:\n- ' + '\n- '.join(failed))

def main() -> None:
    args = build_parser().parse_args()
    config_path = Path(args.config).expanduser().resolve()
    headless_override = True if args.fast_headless else args.headless
    # Fast-headless should advance simulated time quickly without making the
    # contact solver unstable.  A 10 ms step was too coarse for the small
    # differential-drive wheels and could make DART's ODE collision backend
    # explode numerically.  4 ms remains much cheaper than the GUI profile.
    rtf_override = (6.0 if args.fast_headless and args.real_time_factor is None else args.real_time_factor)
    step_override = (0.004 if args.fast_headless and args.max_step_size is None else args.max_step_size)
    config = apply_overrides(
        load_config(config_path),
        map_file=args.map_file,
        headless=headless_override,
        real_time_factor=rtf_override,
        max_step_size=step_override,
        seed=args.seed,
        replan_every_steps=args.replan_every_steps,
        output_dir=args.output_dir,
        run_name=args.run_name,
    )
    map_path = resolve_from_config(config.map.file, config_path)
    grid = load_movingai_map(map_path)
    config = resolve_scenario(config, grid)
    errors = validate_experiment(config, grid, map_path)
    if errors:
        raise RuntimeError('Experiment preflight failed:\n- ' + '\n- '.join(errors))

    output_root = resolve_output_from_config(config.output_dir, config_path)
    run_id = f'{config.name}_seed_{config.simulation.seed}_{int(time.time())}'
    run_dir = output_root / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    world_path = run_dir / f'{map_path.stem}.sdf'
    effective_config_path = run_dir / 'effective_config.json'
    effective = asdict(config)
    effective['map']['file'] = str(map_path)
    effective['output_dir'] = str(output_root)
    effective_config_path.write_text(json.dumps(effective, indent=2), encoding='utf-8')
    generate_sdf_world(
        grid,
        world_path,
        robots=config.robots,
        cell_size=config.map.cell_size,
        wall_height=config.map.wall_height,
        max_step_size=config.simulation.max_step_size,
        real_time_factor=config.simulation.real_time_factor,
        allow_diagonal=config.planning.allow_diagonal,
        visualization=config.visualization,
        collision_detector=config.simulation.collision_detector,
        solver_type=config.simulation.solver_type,
        # Goal / route objects are presentation aids, not physical experiment
        # geometry.  Baking them into the render scene makes GPU lidar see them.
        bake_visual_guides=False,
    )
    world_errors = validate_sdf_world(world_path)
    if world_errors:
        raise RuntimeError('Generated-world validation failed before Gazebo startup:\n- ' + '\n- '.join(world_errors))
    manifest = {
        'run_id': run_id,
        'config_path': str(config_path),
        'map_path': str(map_path),
        'world_path': str(world_path),
        'created_unix': time.time(),
        'status': 'prepared',
        'preflight': 'passed',
    }
    manifest_path = run_dir / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    if args.prepare_only:
        print(f'Prepared experiment-ready run at {run_dir}')
        print('Resolved temporary obstacles:')
        for obstacle in config.environment.get('temporary_obstacles', []):
            print(f"  - {obstacle.get('id')}: {obstacle.get('cell')}")
        for module in config.attack.get('modules', []):
            if module.get('candidate_cells'):
                print(f"Resolved attack candidates for {module.get('name', module.get('type'))}: {module.get('candidate_cells')}")
        return
    for executable in ('gz', 'ros2'):
        if shutil.which(executable) is None:
            raise RuntimeError(f"Required executable '{executable}' was not found in PATH")

    robot_ids = [str(robot['id']) for robot in config.robots]
    existing = _existing_experiment_nodes(robot_ids)
    if existing:
        raise RuntimeError(
            'Existing experiment nodes were detected. Running two copies corrupts '
            'cmd_vel, maps, and metrics. Stop the old run first with:\n'
            '  ~/ros_ws/src/gazebo_trust_experiments/scripts/cleanup_experiment.sh\n'
            'Existing nodes: ' + ', '.join(existing)
        )

    if args.smoke_test:
        if args.smoke_test_seconds <= 0:
            raise RuntimeError('--smoke-test-seconds must be positive')
        smoke_log = run_dir / 'gazebo_smoke_test.log'
        process = _start(['gz', 'sim', '-s', '-r', '-v', '3', str(world_path)], smoke_log)
        try:
            _wait_for_gazebo_server(process, robot_ids, config.simulation.startup_timeout_seconds)
            deadline = time.monotonic() + args.smoke_test_seconds
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(
                        f'Gazebo physics smoke test failed: server exited with code {process.returncode}. '
                        f'See {smoke_log}'
                    )
                time.sleep(0.25)
            manifest['status'] = 'smoke_test_passed'
            manifest['smoke_test_seconds'] = args.smoke_test_seconds
            manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
            print(f'Gazebo physics smoke test PASSED for {args.smoke_test_seconds:.1f}s')
            print(f'Run artifacts: {run_dir}')
            return
        finally:
            _terminate(process)
            _close_log(process)

    # Run the Gazebo backend as the authoritative process.  In GUI mode Gazebo's
    # convenience launcher may return code 0 after spawning the client, which used
    # to make the experiment runner incorrectly kill a healthy simulation.
    gazebo_server_command = ['gz', 'sim', '-s', '-r', '-v', '2', str(world_path)]
    gazebo_gui_command = ['gz', 'sim', '-g', '--render-engine', 'ogre']
    bridge_topics = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    for robot in config.robots:
        rid = str(robot['id'])
        bridge_topics.extend([
            f'/{rid}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{rid}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{rid}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ])
    commands: list[tuple[str, list[str]]] = [
        ('gazebo_server', gazebo_server_command),
        ('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', *bridge_topics]),
        ('network', ['ros2', 'run', 'gazebo_trust_experiments', 'claim_network', '--config', str(effective_config_path)]),
        ('environment', ['ros2', 'run', 'gazebo_trust_experiments', 'environment_manager', '--config', str(effective_config_path)]),
        ('attack', ['ros2', 'run', 'gazebo_trust_experiments', 'attack_manager', '--config', str(effective_config_path)]),
        ('metrics', ['ros2', 'run', 'gazebo_trust_experiments', 'metrics_collector', '--config', str(effective_config_path), '--output-dir', str(run_dir)]),
        ('supervisor', ['ros2', 'run', 'gazebo_trust_experiments', 'experiment_supervisor', '--config', str(effective_config_path)]),
    ]
    for robot in config.robots:
        rid = str(robot['id'])
        commands.extend([
            (f'{rid}_lidar', ['ros2', 'run', 'gazebo_trust_experiments', 'lidar_reporter', '--config', str(effective_config_path), '--robot-id', rid]),
            (f'{rid}_map', ['ros2', 'run', 'gazebo_trust_experiments', 'shared_map_node', '--config', str(effective_config_path), '--robot-id', rid]),
        ])
        if not args.no_controllers:
            commands.append((f'{rid}_controller', ['ros2', 'run', 'gazebo_trust_experiments', 'astar_robot_driver', '--config', str(effective_config_path), '--robot-id', rid]))
    if not config.simulation.headless and bool(config.visualization.get('dynamic_routes', True)):
        commands.append(('visualization', ['ros2', 'run', 'gazebo_trust_experiments', 'experiment_visualization', '--config', str(effective_config_path)]))

    processes: dict[str, subprocess.Popen] = {}
    try:
        processes['gazebo_server'] = _start(commands[0][1], run_dir / 'gazebo_server.log')
        _wait_for_gazebo_server(processes['gazebo_server'], robot_ids, config.simulation.startup_timeout_seconds)
        time.sleep(config.simulation.startup_delay_seconds)
        if not config.simulation.headless:
            processes['gui'] = _start(gazebo_gui_command, run_dir / 'gazebo_gui.log')
            time.sleep(1.0)
        for name, command in commands[1:]:
            processes[name] = _start(command, run_dir / f'{name}.log')
            time.sleep(0.10)
        time.sleep(1.5)
        _check_critical_processes(processes, run_dir)
        manifest['status'] = 'running'
        manifest['commands'] = {name: command for name, command in commands}
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
        supervisor = processes['supervisor']
        while supervisor.poll() is None:
            if processes['gazebo_server'].poll() is not None:
                raise RuntimeError(f'Gazebo server exited early with code {processes["gazebo_server"].returncode}')
            _check_critical_processes(processes, run_dir)
            time.sleep(0.5)
        time.sleep(1.0)
        manifest['status'] = 'completed' if supervisor.returncode == 0 else 'failed'
        manifest['return_code'] = supervisor.returncode
    except KeyboardInterrupt:
        manifest['status'] = 'interrupted'
    except Exception as exc:
        manifest['status'] = 'failed'
        manifest['error'] = str(exc)
        raise
    finally:
        for process in reversed(list(processes.values())):
            _terminate(process)
        for process in processes.values():
            _close_log(process)
        manifest['finished_unix'] = time.time()
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
        print(f'Run artifacts: {run_dir}')


if __name__ == '__main__':
    main()
