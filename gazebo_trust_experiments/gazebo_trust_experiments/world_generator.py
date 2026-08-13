from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

from .astar import astar
from .coordinates import cell_to_world
from .movingai_map import MovingAIMap


@dataclass(frozen=True)
class WallRun:
    x0: int
    x1: int
    y: int

    @property
    def length_cells(self) -> int:
        return self.x1 - self.x0 + 1


def horizontal_wall_runs(grid: MovingAIMap) -> list[WallRun]:
    runs: list[WallRun] = []
    for y in range(grid.height):
        x = 0
        while x < grid.width:
            if not grid.is_blocked(x, y):
                x += 1
                continue
            start = x
            while x + 1 < grid.width and grid.is_blocked(x + 1, y):
                x += 1
            runs.append(WallRun(start, x, y))
            x += 1
    return runs


def _text(parent: ET.Element, tag: str, value: object) -> ET.Element:
    child = ET.SubElement(parent, tag)
    child.text = str(value)
    return child


def _material(visual: ET.Element, rgba: str) -> None:
    material = ET.SubElement(visual, 'material')
    _text(material, 'ambient', rgba)
    _text(material, 'diffuse', rgba)


def _inertial(link: ET.Element, mass: float, ixx: float, iyy: float, izz: float) -> None:
    inertial = ET.SubElement(link, 'inertial')
    _text(inertial, 'mass', mass)
    inertia = ET.SubElement(inertial, 'inertia')
    _text(inertia, 'ixx', ixx)
    _text(inertia, 'iyy', iyy)
    _text(inertia, 'izz', izz)
    _text(inertia, 'ixy', 0)
    _text(inertia, 'ixz', 0)
    _text(inertia, 'iyz', 0)


def _add_box(link: ET.Element, name: str, size: str, rgba: str) -> None:
    for kind in ('collision', 'visual'):
        part = ET.SubElement(link, kind, {'name': f'{name}_{kind}'})
        geom = ET.SubElement(part, 'geometry')
        box = ET.SubElement(geom, 'box')
        _text(box, 'size', size)
        if kind == 'visual':
            _material(part, rgba)


def _add_simple_diff_drive_robot(
    world: ET.Element,
    grid: MovingAIMap,
    robot: dict[str, Any],
    cell_size: float,
) -> None:
    """Add a self-contained differential-drive robot using Gazebo's canonical layout.

    The wheel and joint frame arrangement deliberately mirrors the official Gazebo
    Harmonic building_robot / DiffDrive tutorial, scaled down for 0.5 m grid cells.
    This avoids external TurtleBot model dependencies while using the same proven
    kinematic structure as Gazebo's reference vehicle.
    """
    robot_id = str(robot['id'])
    start = robot.get('start_cell')
    if not isinstance(start, list) or len(start) != 2:
        raise ValueError(f"Robot '{robot_id}' requires start_cell: [x, y]")
    sx, sy = int(start[0]), int(start[1])
    if not (0 <= sx < grid.width and 0 <= sy < grid.height):
        raise ValueError(f"Robot '{robot_id}' start cell is outside the map")
    if grid.is_blocked(sx, sy):
        raise ValueError(f"Robot '{robot_id}' start cell is blocked: {(sx, sy)}")

    x, y = cell_to_world(sx, sy, map_height=grid.height, cell_size=cell_size)
    yaw = float(robot.get('start_yaw', 0.0))
    model = ET.SubElement(world, 'model', {'name': robot_id, 'canonical_link': 'chassis'})
    _text(model, 'pose', f'{x} {y} 0.005 0 0 {yaw}')
    _text(model, 'self_collide', 'false')

    wheel_radius = 0.065
    wheel_width = 0.035
    wheel_separation = 0.26
    chassis_z = 0.082

    chassis = ET.SubElement(model, 'link', {'name': 'chassis'})
    pose = ET.SubElement(chassis, 'pose', {'relative_to': '__model__'})
    pose.text = f'0 0 {chassis_z} 0 0 0'
    _inertial(chassis, 2.2, 0.020, 0.030, 0.040)
    _add_box(chassis, 'body', '0.30 0.22 0.10', str(robot.get('color', '0.1 0.45 0.8 1')))

    # Exact reference orientation: cylinder wheels are rolled -90 degrees and
    # revolute around the model Y axis.  The axle is therefore left-right and
    # wheel rotation produces forward/backward motion along model X.
    for side, y_offset in [('left', wheel_separation / 2), ('right', -wheel_separation / 2)]:
        link = ET.SubElement(model, 'link', {'name': f'{side}_wheel'})
        wheel_pose = ET.SubElement(link, 'pose', {'relative_to': 'chassis'})
        wheel_pose.text = f'-0.055 {y_offset} 0 -1.57079632679 0 0'
        _inertial(link, 0.18, 0.00032, 0.00032, 0.00058)
        for kind in ('collision', 'visual'):
            part = ET.SubElement(link, kind, {'name': kind})
            geom = ET.SubElement(part, 'geometry')
            cylinder = ET.SubElement(geom, 'cylinder')
            _text(cylinder, 'radius', wheel_radius)
            _text(cylinder, 'length', wheel_width)
            if kind == 'collision':
                surface = ET.SubElement(part, 'surface')
                friction = ET.SubElement(surface, 'friction')
                ode = ET.SubElement(friction, 'ode')
                _text(ode, 'mu', 2.0)
                _text(ode, 'mu2', 2.0)
            else:
                _material(part, '0.06 0.06 0.06 1')

        joint = ET.SubElement(model, 'joint', {'name': f'{side}_wheel_joint', 'type': 'revolute'})
        joint_pose = ET.SubElement(joint, 'pose', {'relative_to': f'{side}_wheel'})
        joint_pose.text = '0 0 0 0 0 0'
        _text(joint, 'parent', 'chassis')
        _text(joint, 'child', f'{side}_wheel')
        axis = ET.SubElement(joint, 'axis')
        xyz = ET.SubElement(axis, 'xyz', {'expressed_in': '__model__'})
        xyz.text = '0 1 0'
        # Continuous wheel joints do not need astronomically large numeric
        # limits. Leaving the position range unspecified avoids feeding extreme
        # constants into the dynamics stack.

    # Low-friction trailing caster. Fixed geometry is cheaper and more stable
    # than an extra ball joint, while still preserving physical floor contact.
    caster = ET.SubElement(model, 'link', {'name': 'caster'})
    caster_pose = ET.SubElement(caster, 'pose', {'relative_to': 'chassis'})
    caster_pose.text = '0.105 0 -0.072 0 0 0'
    _inertial(caster, 0.04, 0.00005, 0.00005, 0.00005)
    for kind in ('collision', 'visual'):
        part = ET.SubElement(caster, kind, {'name': kind})
        geom = ET.SubElement(part, 'geometry')
        box = ET.SubElement(geom, 'box')
        _text(box, 'size', '0.045 0.045 0.020')
        if kind == 'collision':
            surface = ET.SubElement(part, 'surface')
            friction = ET.SubElement(surface, 'friction')
            ode = ET.SubElement(friction, 'ode')
            _text(ode, 'mu', 0.03)
            _text(ode, 'mu2', 0.03)
        else:
            _material(part, '0.15 0.15 0.15 1')
    caster_joint = ET.SubElement(model, 'joint', {'name': 'caster_joint', 'type': 'fixed'})
    _text(caster_joint, 'parent', 'chassis')
    _text(caster_joint, 'child', 'caster')

    lidar_link = ET.SubElement(model, 'link', {'name': 'lidar_link'})
    lidar_pose = ET.SubElement(lidar_link, 'pose', {'relative_to': 'chassis'})
    lidar_pose.text = '0.02 0 0.14 0 0 0'
    _inertial(lidar_link, 0.03, 0.00005, 0.00005, 0.00005)
    # Small visual mast makes robot orientation obvious in the GUI.
    lidar_visual = ET.SubElement(lidar_link, 'visual', {'name': 'lidar_visual'})
    geom = ET.SubElement(lidar_visual, 'geometry')
    cylinder = ET.SubElement(geom, 'cylinder')
    _text(cylinder, 'radius', 0.035)
    _text(cylinder, 'length', 0.025)
    _material(lidar_visual, '0.10 0.10 0.10 1')
    sensor = ET.SubElement(lidar_link, 'sensor', {'name': 'lidar', 'type': 'gpu_lidar'})
    _text(sensor, 'topic', f'/{robot_id}/scan')
    _text(sensor, 'update_rate', float(robot.get('lidar_update_rate', 10.0)))
    _text(sensor, 'always_on', 'true')
    _text(sensor, 'visualize', 'false')
    lidar = ET.SubElement(sensor, 'lidar')
    scan = ET.SubElement(lidar, 'scan')
    horizontal = ET.SubElement(scan, 'horizontal')
    _text(horizontal, 'samples', int(robot.get('lidar_samples', 180)))
    _text(horizontal, 'resolution', 1)
    _text(horizontal, 'min_angle', -3.14159265359)
    _text(horizontal, 'max_angle', 3.14159265359)
    range_el = ET.SubElement(lidar, 'range')
    _text(range_el, 'min', 0.12)
    _text(range_el, 'max', float(robot.get('lidar_range', 4.0)))
    _text(range_el, 'resolution', 0.01)
    noise = ET.SubElement(lidar, 'noise')
    _text(noise, 'type', 'gaussian')
    _text(noise, 'mean', 0.0)
    _text(noise, 'stddev', float(robot.get('lidar_noise_stddev', 0.01)))
    lidar_joint = ET.SubElement(model, 'joint', {'name': 'lidar_joint', 'type': 'fixed'})
    _text(lidar_joint, 'parent', 'chassis')
    _text(lidar_joint, 'child', 'lidar_link')

    diff_drive = ET.SubElement(model, 'plugin', {'filename': 'gz-sim-diff-drive-system', 'name': 'gz::sim::systems::DiffDrive'})
    _text(diff_drive, 'left_joint', 'left_wheel_joint')
    _text(diff_drive, 'right_joint', 'right_wheel_joint')
    _text(diff_drive, 'wheel_separation', wheel_separation)
    _text(diff_drive, 'wheel_radius', wheel_radius)
    _text(diff_drive, 'topic', f'/{robot_id}/cmd_vel')
    _text(diff_drive, 'odom_topic', f'/{robot_id}/odometry')
    _text(diff_drive, 'tf_topic', f'/{robot_id}/tf')
    _text(diff_drive, 'frame_id', f'{robot_id}/odom')
    _text(diff_drive, 'child_frame_id', f'{robot_id}/chassis')
    _text(diff_drive, 'odom_publish_frequency', int(robot.get('odom_publish_frequency', 15)))
    _text(diff_drive, 'max_linear_acceleration', float(robot.get('max_linear_acceleration', 0.8)))
    _text(diff_drive, 'max_angular_acceleration', float(robot.get('max_angular_acceleration', 2.0)))


def _add_route_and_goal_visuals(
    world: ET.Element,
    grid: MovingAIMap,
    robots: list[dict[str, Any]],
    cell_size: float,
    *,
    allow_diagonal: bool,
    visualization: dict[str, Any] | None,
) -> None:
    cfg = visualization or {}
    if not bool(cfg.get('enabled', True)):
        return
    show_routes = bool(cfg.get('show_initial_routes', True))
    show_goals = bool(cfg.get('show_goals', True))
    route_stride = max(1, int(cfg.get('route_marker_stride', 1)))
    route_radius = float(cfg.get('route_marker_radius', 0.075))
    route_height = float(cfg.get('route_marker_height', 0.018))
    goal_radius = float(cfg.get('goal_radius', 0.15))
    goal_height = float(cfg.get('goal_height', 0.80))
    goal_rgba = str(cfg.get('goal_color', '0.95 0.80 0.15 1'))
    dedupe_goals = bool(cfg.get('deduplicate_shared_goals', True))

    if show_goals:
        goal_cells: list[tuple[int, int]] = []
        for robot in robots:
            for goal in robot.get('goal_nodes', []):
                cell = tuple(int(v) for v in goal)
                if not dedupe_goals or cell not in goal_cells:
                    goal_cells.append(cell)
        if goal_cells:
            model = ET.SubElement(world, 'model', {'name': 'delivery_goalposts'})
            _text(model, 'static', 'true')
            link = ET.SubElement(model, 'link', {'name': 'link'})
            for index, goal in enumerate(goal_cells):
                gx, gy = cell_to_world(*goal, map_height=grid.height, cell_size=cell_size)
                pole = ET.SubElement(link, 'visual', {'name': f'goal_pole_{index}'})
                _text(pole, 'pose', f'{gx} {gy} {goal_height / 2} 0 0 0')
                geom = ET.SubElement(pole, 'geometry')
                cylinder = ET.SubElement(geom, 'cylinder')
                _text(cylinder, 'radius', goal_radius * 0.45)
                _text(cylinder, 'length', goal_height)
                _material(pole, goal_rgba)
                top = ET.SubElement(link, 'visual', {'name': f'goal_top_{index}'})
                _text(top, 'pose', f'{gx} {gy} {goal_height + goal_radius} 0 0 0')
                geom = ET.SubElement(top, 'geometry')
                sphere = ET.SubElement(geom, 'sphere')
                _text(sphere, 'radius', goal_radius)
                _material(top, goal_rgba)

    if show_routes:
        for robot in robots:
            rid = str(robot['id'])
            rgba = str(robot.get('color', '0.1 0.45 0.8 1'))
            start = tuple(int(v) for v in robot['start_cell'])
            goals = [tuple(int(v) for v in goal) for goal in robot.get('goal_nodes', [])]
            if not goals:
                continue
            initial_index = int(robot.get('initial_goal_index', 0)) % len(goals)
            path = astar(grid, start, goals[initial_index], allow_diagonal=allow_diagonal)
            if not path:
                continue
            model = ET.SubElement(world, 'model', {'name': f'{rid}_initial_route'})
            _text(model, 'static', 'true')
            link = ET.SubElement(model, 'link', {'name': 'link'})
            selected = path[::route_stride]
            if selected[-1] != path[-1]:
                selected.append(path[-1])
            for index, cell in enumerate(selected):
                px, py = cell_to_world(*cell, map_height=grid.height, cell_size=cell_size)
                visual = ET.SubElement(link, 'visual', {'name': f'route_{index:03d}'})
                _text(visual, 'pose', f'{px} {py} {route_height / 2 + 0.012} 0 0 0')
                geom = ET.SubElement(visual, 'geometry')
                cylinder = ET.SubElement(geom, 'cylinder')
                _text(cylinder, 'radius', route_radius)
                _text(cylinder, 'length', route_height)
                _material(visual, rgba)

def generate_sdf_world(
    grid: MovingAIMap,
    output_path: str | Path,
    *,
    robots: list[dict[str, Any]] | None = None,
    cell_size: float = 0.5,
    wall_height: float = 1.0,
    max_step_size: float = 0.002,
    real_time_factor: float = 1.0,
    allow_diagonal: bool = False,
    visualization: dict[str, Any] | None = None,
    collision_detector: str = 'fcl',
    solver_type: str = 'pgs',
    bake_visual_guides: bool = False,
) -> Path:
    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    sdf = ET.Element('sdf', {'version': '1.9'})
    world = ET.SubElement(sdf, 'world', {'name': 'trust_experiment'})

    # Keep the physics block intentionally minimal. Gazebo Harmonic's default
    # DART integration chooses its compatible collision backend internally.
    # Injecting backend-specific <dart> overrides here was not reliably honored
    # and made the generated world less portable.
    physics = ET.SubElement(world, 'physics', {'name': 'physics', 'type': 'ignored'})
    _text(physics, 'max_step_size', max_step_size)
    _text(physics, 'real_time_factor', real_time_factor)
    _text(physics, 'max_contacts', 8)

    for filename, name in [
        ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
        ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
        ('gz-sim-sensors-system', 'gz::sim::systems::Sensors'),
    ]:
        plugin = ET.SubElement(world, 'plugin', {'filename': filename, 'name': name})
        if name == 'gz::sim::systems::Sensors':
            _text(plugin, 'render_engine', 'ogre')

    _text(world, 'gravity', '0 0 -9.8')
    scene = ET.SubElement(world, 'scene')
    _text(scene, 'ambient', '0.7 0.7 0.7 1')
    _text(scene, 'background', '0.8 0.8 0.8 1')

    sun = ET.SubElement(world, 'light', {
        'name': 'sun',
        'type': 'directional',
    })
    _text(sun, 'cast_shadows', 'false')
    _text(sun, 'pose', '0 0 10 0 0 0')
    _text(sun, 'diffuse', '0.8 0.8 0.8 1')
    _text(sun, 'specular', '0.2 0.2 0.2 1')
    _text(sun, 'direction', '-0.5 0.5 -1')
    attenuation = ET.SubElement(sun, 'attenuation')
    _text(attenuation, 'range', '1000')
    _text(attenuation, 'constant', '0.9')
    _text(attenuation, 'linear', '0.01')
    _text(attenuation, 'quadratic', '0.001')

    floor = ET.SubElement(world, 'model', {'name': 'floor'})
    _text(floor, 'static', 'true')
    floor_link = ET.SubElement(floor, 'link', {'name': 'link'})
    floor_size_x = grid.width * cell_size
    floor_size_y = grid.height * cell_size
    for kind in ('collision', 'visual'):
        part = ET.SubElement(floor_link, kind, {'name': kind})
        geom = ET.SubElement(part, 'geometry')
        box = ET.SubElement(geom, 'box')
        _text(box, 'size', f'{floor_size_x} {floor_size_y} 0.05')
        if kind == 'visual':
            _material(part, '0.72 0.72 0.72 1')
    _text(floor, 'pose', f'{floor_size_x / 2} {floor_size_y / 2} -0.025 0 0 0')

    # Physical perimeter rails sit just outside the MovingAI grid.  Some benchmark
    # maps have traversable cells on the outer edge; without a physical guard the
    # differential-drive model can leave the finite floor after a controller fault.
    # These rails do not consume any map cell and therefore do not change A*.
    boundary_thickness = min(0.12, cell_size * 0.24)
    boundary_height = max(0.35, wall_height * 0.55)
    boundary_specs = [
        ('south', floor_size_x / 2, -boundary_thickness / 2, floor_size_x + 2 * boundary_thickness, boundary_thickness),
        ('north', floor_size_x / 2, floor_size_y + boundary_thickness / 2, floor_size_x + 2 * boundary_thickness, boundary_thickness),
        ('west', -boundary_thickness / 2, floor_size_y / 2, boundary_thickness, floor_size_y),
        ('east', floor_size_x + boundary_thickness / 2, floor_size_y / 2, boundary_thickness, floor_size_y),
    ]
    for name, bx, by, bsx, bsy in boundary_specs:
        model = ET.SubElement(world, 'model', {'name': f'boundary_{name}'})
        _text(model, 'static', 'true')
        _text(model, 'pose', f'{bx} {by} {boundary_height / 2} 0 0 0')
        link = ET.SubElement(model, 'link', {'name': 'link'})
        for kind in ('collision', 'visual'):
            part = ET.SubElement(link, kind, {'name': kind})
            geom = ET.SubElement(part, 'geometry')
            box = ET.SubElement(geom, 'box')
            _text(box, 'size', f'{bsx} {bsy} {boundary_height}')
            if kind == 'visual':
                _material(part, '0.10 0.12 0.16 1')

    for index, run in enumerate(horizontal_wall_runs(grid)):
        model = ET.SubElement(world, 'model', {'name': f'wall_{index:05d}'})
        _text(model, 'static', 'true')
        link = ET.SubElement(model, 'link', {'name': 'link'})
        length = run.length_cells * cell_size
        center_x = (run.x0 + run.x1 + 1) * cell_size / 2
        center_y = (grid.height - run.y - 0.5) * cell_size
        _text(model, 'pose', f'{center_x} {center_y} {wall_height / 2} 0 0 0')
        for kind in ('collision', 'visual'):
            part = ET.SubElement(link, kind, {'name': kind})
            geom = ET.SubElement(part, 'geometry')
            box = ET.SubElement(geom, 'box')
            _text(box, 'size', f'{length} {cell_size} {wall_height}')
            if kind == 'visual':
                _material(part, '0.18 0.20 0.24 1')

    robot_list = robots or []
    if bake_visual_guides:
        _add_route_and_goal_visuals(world, grid, robot_list, cell_size, allow_diagonal=allow_diagonal, visualization=visualization)
    for robot in robot_list:
        _add_simple_diff_drive_robot(world, grid, robot, cell_size)

    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(output, encoding='utf-8', xml_declaration=True)
    return output
