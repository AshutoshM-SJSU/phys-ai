# v0.6.1 navigation-drive stabilization

This revision fixes the stationary-robot failure seen in v0.6.0. It separates direct local lidar evidence from delayed shared evidence, ignores a robot's own network echo, suppresses lidar returns inside a 0.40 m self-footprint halo, forces the current A* start cell to remain traversable with respect to dynamic evidence, emits detailed no-path diagnostics, and refuses to start when stale experiment nodes are already present. Use `scripts/cleanup_experiment.sh` before a new GUI run if a prior run was interrupted.

# Gazebo Experiment 2 Starter

This package is a runnable ROS 2 Jazzy / Gazebo Harmonic scaffold for the document's second experiment: three physical delivery robots in a known map, temporary physical obstacles, lidar observations, delayed malicious map reports, communication delay, asynchronous map updates, repeated A* deliveries, and experiment metrics.

The package intentionally contains **no trust defense implementations and no comparison baselines**. Every received report is accepted at full trust using a latest-received cell state. The project is therefore focused on making the physical experiment reliable before any defense policy is introduced.

## Implemented experiment behavior

- Exactly three differential-drive robots with collision geometry, inertia, wheel joints, acceleration limits, odometry, and 2D lidar.
- Two benign delivery robots and one physically moving delayed attacker.
- Repeated delivery cycles through configurable `goal_nodes`.
- Known MovingAI static map and A* global planning.
- Continuous Gazebo motion and a local controller with emergency turning around nearby obstacles.
- Physical temporary boxes spawned and removed by simulation time.
- Honest lidar reports from every robot during reconnaissance.
- Modular malicious modes: false obstacle, false clearance, and stale reassertion.
- Shared claim network with configurable delay, jitter, packet loss, and reception timestamps.
- Per-robot full-trust occupancy grids that trigger A* replanning while robots are moving.
- Simulation-time termination, process cleanup, per-process logs, event logs, JSON metrics, and CSV metrics.
- OGRE-only GUI rendering.

## Package layout

```text
config/experiment_2_ready.yaml          headless experiment configuration
config/presentation_experiment_2.yaml   OGRE GUI configuration
gazebo_trust_experiments/attacks/       modular attack modes
gazebo_trust_experiments/nodes/         sensing, network, maps, environment, metrics, supervisor
gazebo_trust_experiments/robot_driver.py repeated-delivery A* controller
gazebo_trust_experiments/world_generator.py physical robot and world SDF generation
scripts/download_maps.py                MovingAI map downloader
scripts/bootstrap_experiment.sh         dependency, map, and workspace build helper
```

## One-time bootstrap

From the package source directory:

```bash
cd ~/ros_ws/src/gazebo_trust_experiments
./scripts/bootstrap_experiment.sh
source ~/ros_ws/install/setup.bash
```

The YAML refers to the map as:

```yaml
map:
  file: ../maps/room-32-32-4.map
```

## Preflight without launching Gazebo

```bash
ros2 run gazebo_trust_experiments experiment_runner \
  --config ~/ros_ws/src/gazebo_trust_experiments/config/experiment_2_ready.yaml \
  --prepare-only
```

Preflight checks the three-robot structure, attacker count, start cells, goal nodes, obstacle cells, attack cells, event timing, map bounds, and static collisions. It then generates the exact SDF and effective configuration used for the run.

## Run Experiment 2 headless

```bash
ros2 run gazebo_trust_experiments experiment_runner \
  --config ~/ros_ws/src/gazebo_trust_experiments/config/experiment_2_ready.yaml \
  --headless true
```


## Physics smoke test

After any Gazebo physics / collision failure, run the generated-world smoke test before launching the full ROS experiment:

```bash
ros2 run gazebo_trust_experiments experiment_runner \
  --config ~/ros_ws/src/gazebo_trust_experiments/config/presentation_experiment_2.yaml \
  --smoke-test \
  --smoke-test-seconds 10
```

The smoke test generates the complete SDF, validates poses / dimensions / masses / inertias for finite positive values, starts only the Gazebo server, waits for `/clock` and robot odometry topics, then requires the physics server to remain alive for the requested interval. It does not launch A*, mapping, attacks, networking, or metrics. This isolates world / physics failures from ROS-side failures.

Version 0.6.3 also removes the unreliable DART collision-backend override, removes extreme synthetic wheel-joint limits, slightly lifts robot spawn poses above the floor, replaces the trailing caster sphere contact with a small low-friction box skid, and uses a 4 ms presentation physics step.

## Run the presentation version with OGRE

```bash
ros2 run gazebo_trust_experiments presentation_sim \
  --config ~/ros_ws/src/gazebo_trust_experiments/config/presentation_experiment_2.yaml
```

## Parameter overrides

```bash
ros2 run gazebo_trust_experiments experiment_runner \
  --config ~/ros_ws/src/gazebo_trust_experiments/config/experiment_2_ready.yaml \
  --headless true \
  --seed 31 \
  --real-time-factor 5.0 \
  --replan-every-steps 20 \
  --run-name experiment_2_seed_31
```

Equivalent launch form:

```bash
ros2 launch gazebo_trust_experiments experiment.launch.py \
  config:=~/ros_ws/src/gazebo_trust_experiments/config/experiment_2_ready.yaml \
  headless:=true \
  seed:=31
```

## Output files

Each run creates a unique directory under `results/` containing:

```text
manifest.json
 effective_config.json
<map>.sdf
events.jsonl
metrics_summary.json
metrics_by_robot.csv
gazebo.log
bridge.log
network.log
environment.log
attack.log
metrics.log
supervisor.log
<robot>_lidar.log
<robot>_map.log
<robot>_controller.log
```

The metrics collector currently reports deliveries, delivery time, physical distance, detour ratio, no-path time, replans, replans per delivery, hesitation, route reversals, planning time, route overlap, dynamic-map error, malicious claim acceptance, false blockage persistence, emergency stops, and near-collisions.

## Adding another attack mode

Implement the `Attack` interface under `gazebo_trust_experiments/attacks/`, register the class in `attacks/__init__.py`, and add another item to `attack.modules` in YAML. Multiple modules can be active in the same run with independent start times, end times, periods, candidate cells, and parameters.

## Important validation note

The source package has unit-tested configuration, map, attack, A*, full-trust map, and SDF-generation logic. Actual Gazebo execution must still be smoke-tested on the target ROS 2 Jazzy / Gazebo Harmonic machine because this archive is produced outside that runtime. The first real run should use `--prepare-only`, then GUI mode with a short timeout, then a full headless trial.

## Bootstrap / map troubleshooting

The bootstrap script intentionally does **not** enable Bash `nounset` (`set -u`)
because ROS 2 environment setup scripts can reference variables that are unset in
a fresh shell. Run:

```bash
cd ~/ros_ws/src/gazebo_trust_experiments
./scripts/bootstrap_experiment.sh
source ~/ros_ws/install/setup.bash
```

The experiment YAML keeps map paths relative to the YAML, for example:

```yaml
map:
  file: ../maps/room-32-32-4.map
```

If you need only to recover a missing map without re-extracting the package:

```bash
cd ~/ros_ws/src/gazebo_trust_experiments
python3 scripts/download_maps.py --output maps
cd ~/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select gazebo_trust_experiments
source install/setup.bash
```

## v0.3.2 map-aware scenario placement

The Experiment 2 YAML uses `cell: auto` for temporary physical obstacles and
`candidate_cells: auto` for false-obstacle attack targets. Before preflight,
the runner resolves these entries onto deterministic free cells along the
configured robots' static A* routes. The concrete resolved configuration is
written to each run directory as `effective_config.json`, and every runtime
node consumes that resolved configuration.

This avoids embedding map-specific coordinates that may fall inside walls when
the MovingAI map changes. Explicit `[x, y]` coordinates are still supported.


## v0.3.3 runtime fixes
- Fixed ROS 2 Jazzy duplicate `use_sim_time` parameter declarations in all nodes.
- Made wheel link / joint frames explicit and aligned wheel rotation with the model Y axle.
- Raised lidar above the chassis and ignores pathological returns at sensor minimum range.
- Bridges `cmd_vel` ROS->Gazebo only and odometry / scan Gazebo->ROS only.

## v0.4 presentation and runtime behavior

The GUI run is now launched as two Gazebo processes: an authoritative server (`gz sim -s -r`) and a separate OGRE GUI client (`gz sim -g --render-engine ogre`). The runner waits for `/clock` and all robot odometry topics before starting ROS-side controllers. This avoids treating the Gazebo convenience launcher's normal code-0 return as a simulation failure.

Presentation worlds include non-colliding, color-coded goal posts for every delivery node and dotted guides for each robot's initial A* route. The initial active goal is larger and emissive. These route guides are intentionally visualization-only and do not affect physics or A* planning. Replanning remains part of the runtime controller and metrics; the dotted Gazebo guide shows the initial planned route rather than pretending to be a dynamically redrawn trust route.

The default performance profile is tuned for a VirtualBox-style development machine: 200-250 Hz physics depending on config, lower odometry publication, 5-8 Hz lidar, 120-180 horizontal lidar samples, 8 Hz control, reduced map publication, coarser free-ray sampling, no sun shadows, merged static wall runs, and visual-only route markers with no collision geometry. Headless runs keep the same physical robot / collision logic but skip the GUI entirely.

## v0.5 preliminary experiment behavior

- DiffDrive odometry is converted from the robot-local odom origin into MovingAI world coordinates before A* planning and lidar ray projection. This fixes the previous case where a robot spawned away from world origin but the planner treated odometry `(0, 0)` as the map origin.
- All three robots share four visible delivery goalposts. Each robot begins with a deterministic first target, then selects its next target pseudo-randomly from the remaining goal nodes using the experiment seed.
- Presentation mode renders large gold goalposts plus the initial color-coded A* routes.
- Headless mode remains supported and is the primary high-throughput runner. `--fast-headless` forces server-only execution and uses an 8x target real-time factor with a 10 ms physics step unless explicitly overridden.
- Preliminary metrics are written continuously to `metrics_summary.json`, `metrics_by_robot.csv`, and `events.jsonl`. Implemented metrics include deliveries completed, average delivery time, detour ratio, no-path time, replans per delivery, hesitation time, route reversals, false blockage persistence, map error rate, false acceptance rate, false rejection rate, planning time, route stability, and safety-event counts. Trust detection delay and recovery time are intentionally reported as not applicable in the full-trust configuration because no trust detector exists in this starter.

### Recommended presentation run

```bash
ros2 run gazebo_trust_experiments presentation_sim \
  --config ~/ros_ws/src/gazebo_trust_experiments/config/presentation_experiment_2.yaml
```

### Recommended fast headless run

```bash
ros2 run gazebo_trust_experiments experiment_runner \
  --config ~/ros_ws/src/gazebo_trust_experiments/config/experiment_2_ready.yaml \
  --fast-headless
```


## v0.6.0 motion and mission update

- Robots now use a scaled version of Gazebo Harmonic's official two-wheel DiffDrive tutorial layout rather than the earlier ad-hoc chassis geometry. This stays self-contained and avoids requiring TurtleBot packages or Fuel downloads.
- A shared 12-goal pool is generated automatically from mutually reachable free map cells using deterministic farthest-point sampling. Goals are distributed across the interior of the map instead of being only corner points.
- Every robot chooses its next goal pseudo-randomly from the shared pool, excluding its just-completed goal. The seed makes sequences reproducible.
- Headless fast execution remains supported with `--fast-headless`; GUI visualization is not required for physics, sensing, attacks, planning, or metrics.
- Controller logs now print the first several velocity commands to make stationary-robot failures immediately diagnosable.


## v0.6.2 headless stability changes

- Explicit DART + FCL collision detection instead of the DART/ODE path that aborted with an ODE AABB assertion.
- PGS contact solver and max 8 contacts per pair for a lighter, more stable repeated-run profile.
- `--fast-headless` uses a 4 ms physics step and 6x target RTF. The old 10 ms step was too coarse for the small wheel/contact geometry.
- Lidar noise uses the current SDF child-element syntax (`<noise><type>gaussian</type>...`).
- Goal and route visuals are GUI-only Gazebo markers rather than SDF geometry, so the GPU lidar cannot mistake presentation graphics for obstacles.

If a headless server still aborts, preserve `gazebo_server.log`; the runner no longer intentionally selects ODE collision detection.

## v0.6.4 corridor-safe path following

This revision keeps A* as the authoritative global route and makes the physical
controller deliberately conservative in narrow MovingAI corridors. Robots now
rotate in place until aligned to the next A* cell center, drive to a much tighter
waypoint tolerance, slow near cell centers, and only apply emergency stopping to
obstacles in a forward lidar cone. It removes the previous blind reactive turn
that could steer a robot away from a valid known-static-map centerline. Gazebo
presentation routes are rendered with larger points plus a best-effort continuous
line strip for easier inspection.

## v0.6.5 corridor-following and visualization update

- A* routes are rendered as world-coordinate, visual-only floor overlays rather than MarkerManager point clouds.
- Active routes are replaced after every replan, so the visible route corresponds to the current A* path.
- Robots use an explicit TURN -> DRIVE state machine and settle at every grid-cell waypoint before changing direction.
- A robot never intentionally drives and turns at the same time. If heading drifts during a straight segment it stops and re-aligns.
- Physical perimeter rails are generated just outside the MovingAI map so a controller fault cannot send a robot off the finite floor.
