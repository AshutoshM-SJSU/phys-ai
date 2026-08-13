#!/usr/bin/env bash
set -eo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"

# ROS setup scripts may reference variables that are unset in a fresh shell.
# Do not enable `set -u` (nounset) while sourcing them.
source /opt/ros/jazzy/setup.bash

printf 'Package:   %s\n' "$PACKAGE_DIR"
printf 'Workspace: %s\n' "$WORKSPACE_DIR"

python3 "$PACKAGE_DIR/scripts/download_maps.py" --output "$PACKAGE_DIR/maps"

required_map="$PACKAGE_DIR/maps/room-32-32-4.map"
if [[ ! -f "$required_map" ]]; then
  printf 'ERROR: required map was not downloaded: %s\n' "$required_map" >&2
  exit 1
fi

cd "$WORKSPACE_DIR"
rosdep install --from-paths "$PACKAGE_DIR" --ignore-src -r -y --skip-keys ament_python

# Remove only this package's stale generated artifacts. This leaves other
# packages in the workspace alone.
rm -rf build/gazebo_trust_experiments install/gazebo_trust_experiments

colcon build --symlink-install --packages-select gazebo_trust_experiments

printf '\nBuild complete. In this terminal run:\n'
printf 'source %s/install/setup.bash\n' "$WORKSPACE_DIR"
printf '\nThen validate the experiment with:\n'
printf 'ros2 run gazebo_trust_experiments experiment_runner --config %s/config/experiment_2_ready.yaml --prepare-only\n' "$PACKAGE_DIR"
