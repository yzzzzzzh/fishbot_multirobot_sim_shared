# Single-Go2 RACER + Swarm-LIO2 baseline

This directory contains the runtime-mounted code and configuration for a
single Unitree Go2 exploring the teaching-building environment. Swarm-LIO2
(ROS 2 Humble) estimates the robot's 3-D pose; RACER (ROS 1 Noetic) consumes
the planar adapter output and produces exploration trajectories.

## What is frozen here

- `racer_integration/`: ROS 1/ROS 2 gateway, velocity controller, recorder,
  and video renderer.
- `racer_adapter/`: ROS 2 frame adapter and the Go2 planar-map configuration.
- `legged_sim/` and `gazebo_sim/worlds/`: Go2 simulation package and teaching
  building worlds mounted at runtime.
- `exploration_manager/launch/`: RACER launch configuration.
- `baseline.env`: resolved baseline parameters.

The legacy launcher checks the ROS 1 image against the local image ID recorded
by the original baseline. The standalone launcher instead rebuilds its pinned
source tree into local images.

## One-click deployment

- Linux host with NVIDIA driver, Docker Engine, Docker Compose v2, and GPU
  container runtime.
- Internet access during the first build (ROS, GTSAM, NLopt, LKH and LibTorch
  build dependencies are downloaded by Docker).
- About 25 GB of free Docker storage. No sibling repository and no prebuilt
  local image is required.

Clone and run the exact 200-s single-Go2 baseline:

```bash
git clone https://github.com/yzzzzzzh/fishbot_multirobot_sim_shared.git
cd fishbot_multirobot_sim_shared
./run_one_click_baseline.sh
```

If the build must use a proxy, pass it explicitly rather than relying on a
possibly stale host proxy setting:

```bash
FISHBOT_HTTP_PROXY=http://proxy.example:port \\
FISHBOT_HTTPS_PROXY=http://proxy.example:port \\
./run_one_click_baseline.sh
```

The command builds the four required runtime images, starts the stack, waits
for the recorder to finish, and writes `run.npz`, `run.summary.json`, and logs
to `runs/`. The required `himloco` Go2 policy is included; Docker images and
generated run outputs are not committed.

## Run

The legacy overlay launcher below still requires the old sibling checkout. Use
the one-click command above for a standalone clone.

```bash
./run_first_success_baseline.sh 200 /tmp/go2_baseline_200s
```

For the exact original launcher method, retaining the original checkout's
runtime mounts:

```bash
./run_original_method_exact.sh 200 /tmp/go2_original_200s
```

Both scripts use these non-default planner values from `baseline.env`:

```text
RACER_CONSISTENCY_SIGN=1.0
RACER_FIRST_GRID_BONUS=6.0
RACER_MAP_WARMUP_SECONDS=8
RACER_PLANNER_MAX_VEL=1.2
RACER_PLANNER_MAX_ACC=0.6
RACER_MIN_TARGET_EXECUTION_TIME=8.0
```

The recorder creates `run.npz`, `run.summary.json`, and `recorder.log` in the
chosen container path. A valid run requires `final_valid: true` and
`emergency_stop_triggered: false` in `run.summary.json`.

## Render a video

Copy the recorded directory from `swarm_lio2_ros2`, then run:

```bash
python3 racer_integration/render_planar_quadruped_video.py \
  --run RUN_DIR/run.npz \
  --summary RUN_DIR/run.summary.json \
  --layout gazebo_sim/worlds/teaching_building_atrium.layout.json \
  --output RUN_DIR/exploration.mp4 \
  --thumbnail RUN_DIR/exploration.png \
  --fps 20 --speedup 8
```

The generated MP4 is a trajectory/map visualization, not a Gazebo camera
recording of the physical robot model.

## Before publishing on GitHub

Do not commit `runs/`, generated logs, videos, `.npz` files, Python caches,
or Docker images. The baseline's required Go2 policy is intentionally included
at `src/legged/qrc/go2_description/config/himloco/himloco.pt`; verify its
redistribution rights, and those of every other third-party model and
simulation asset, before making the repository public. Add a root `LICENSE`
only for code you own and create `THIRD_PARTY_NOTICES.md` listing RACER,
Swarm-LIO2, Unitree/Go2 assets, and each upstream license.
