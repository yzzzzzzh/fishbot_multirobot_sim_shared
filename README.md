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

The ROS 1 image is checked at launch against the local image ID recorded by
the baseline. Recording stops before it begins if the image differs.

## Prerequisites

- Linux host with NVIDIA driver, Docker Engine, Docker Compose v2, and GPU
  container runtime.
- A sibling checkout named `fishbot_multirobot_sim` containing the base compose
  files, Dockerfiles, and prebuilt images. If it is elsewhere, set
  `FISHBOT_REPO_DIR` to that checkout before launching.
- Local Docker images `fishbot_base`, `fishbot_multirobot_sim-legged`,
  `swarm-lio2-ros2`, and `fishbot_multirobot_sim-racer_ros1:latest`.

This directory uses paths relative to itself. It does not contain or
redistribute large Docker images, RL weights, caches, or run outputs.

## Run

Run the frozen shared-source baseline for 200 simulation seconds:

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
Docker images, or RL weights. Add a root `LICENSE` only for code you own, and
create `THIRD_PARTY_NOTICES.md` listing RACER, Swarm-LIO2, Unitree/Go2 assets,
and each upstream license. Verify redistribution rights for all model files
and simulation assets before including them in a public repository.
