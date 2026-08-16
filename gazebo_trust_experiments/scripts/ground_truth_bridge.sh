#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/ros_ws/install/setup.bash" ]; then
  source "$HOME/ros_ws/install/setup.bash"
fi

exec ros2 run ros_gz_bridge parameter_bridge \
  '/world/trust_experiment/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
