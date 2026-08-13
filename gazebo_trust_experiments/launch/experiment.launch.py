from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _runner_node(context: LaunchContext):
    pairs = [
        ('map_file', '--map-file'),
        ('real_time_factor', '--real-time-factor'),
        ('max_step_size', '--max-step-size'),
        ('seed', '--seed'),
        ('replan_every_steps', '--replan-every-steps'),
        ('output_dir', '--output-dir'),
        ('run_name', '--run-name'),
    ]
    arguments = [
        '--config', LaunchConfiguration('config').perform(context),
        '--headless', LaunchConfiguration('headless').perform(context),
    ]
    for launch_name, cli_name in pairs:
        value = LaunchConfiguration(launch_name).perform(context).strip()
        if value:
            arguments.extend([cli_name, value])
    if LaunchConfiguration('prepare_only').perform(context).lower() == 'true':
        arguments.append('--prepare-only')
    if LaunchConfiguration('no_controllers').perform(context).lower() == 'true':
        arguments.append('--no-controllers')
    return [Node(
        package='gazebo_trust_experiments',
        executable='experiment_runner',
        name='experiment_runner',
        output='screen',
        arguments=arguments,
    )]


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution([
        FindPackageShare('gazebo_trust_experiments'),
        'config',
        'experiment_2_ready.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('real_time_factor', default_value=''),
        DeclareLaunchArgument('max_step_size', default_value=''),
        DeclareLaunchArgument('seed', default_value=''),
        DeclareLaunchArgument('replan_every_steps', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('run_name', default_value=''),
        DeclareLaunchArgument('prepare_only', default_value='false'),
        DeclareLaunchArgument('no_controllers', default_value='false'),
        OpaqueFunction(function=_runner_node),
    ])
