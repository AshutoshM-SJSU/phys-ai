#!/usr/bin/env bash
set -eo pipefail

# Stop only processes used by this experiment package plus Gazebo / bridge
# instances launched for its world. This is intended for the dedicated test VM.
pkill -f 'gazebo_trust_experiments' 2>/dev/null || true
pkill -f 'ros_gz_bridge.*parameter_bridge' 2>/dev/null || true
pkill -f 'gz sim' 2>/dev/null || true
sleep 1

echo 'Experiment processes stopped. Remaining matching ROS nodes:'
ros2 node list 2>/dev/null | grep -E 'astar_driver|lidar_reporter|shared_map|attack_manager|claim_network|experiment_supervisor|experiment_visualization|metrics_collector|environment_manager' || true
