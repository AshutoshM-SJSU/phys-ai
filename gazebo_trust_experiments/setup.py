from glob import glob
from setuptools import find_packages, setup

package_name = 'gazebo_trust_experiments'

setup(
    name=package_name,
    version='0.6.5',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='researcher',
    maintainer_email='researcher@example.com',
    description='Experiment-ready Gazebo physical validation for delayed multi-robot map attacks.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'experiment_runner = gazebo_trust_experiments.runner:main',
            'presentation_sim = gazebo_trust_experiments.presentation:main',
            'generate_world = gazebo_trust_experiments.generate_world_cli:main',
            'astar_robot_driver = gazebo_trust_experiments.robot_driver:main',
            'claim_network = gazebo_trust_experiments.nodes.network_node:main',
            'environment_manager = gazebo_trust_experiments.nodes.environment_node:main',
            'lidar_reporter = gazebo_trust_experiments.nodes.lidar_reporter:main',
            'attack_manager = gazebo_trust_experiments.nodes.attack_node:main',
            'shared_map_node = gazebo_trust_experiments.nodes.shared_map_node:main',
            'metrics_collector = gazebo_trust_experiments.nodes.metrics_node:main',
            'experiment_supervisor = gazebo_trust_experiments.nodes.supervisor_node:main',
            'experiment_visualization = gazebo_trust_experiments.nodes.visualization_node:main',
        ],
    },
)
