# Drone bring-up (step 1 of swapping fishbots for quadrotors)

Proves the `quad_mid360` blueprint is flyable and its sensors are usable for
SLAM, before any of the 5-drone / 3D-extrinsic work is attempted.

## Run
```bash
docker compose -f docker-compose.yml -f docker-compose.drone.yml \
  up -d --force-recreate gazebo
docker exec fishbot_gazebo bash /fishbot_ws/src/gazebo_sim/drone_bringup/run.sh
```

## Result (last run)
```
hover altitude   : mean 1.504 m (target 1.50, offset 4 mm), peak-to-peak 10.9 mm over 6 s  PASS
square tracking  : 5/5 corners hit, worst err 0.120 m  PASS
lidar            : 180x64 grid, 9821 finite returns, range 0.12-7.29 m  PASS
```
Real-time factor is 1.0 with one drone.

## The model
`robots/quad_mid360/` is a normal blueprint, picked with `robot:=quad_mid360`.
The fishbot blueprints are untouched.

Physical parameters (mass 1.5 kg, inertias, rotor poses/constants) are the
OpenRobotics X3 UAV's; control gains are copied verbatim from the tuned
`/usr/share/ignition/ignition-gazebo6/worlds/multicopter_velocity_control.sdf`.
Geometry is re-authored with primitives rather than including the Fuel model,
because its visual meshes are `https://` URIs that would need Fuel reachable at
spawn time.

Two things are deliberately identical to the fishbot so nothing downstream had
to change:
* topic layout — `/<ns>/cmd_vel`, `/<ns>/imu`, `/<ns>/lidar_points/points`
* **lidar→IMU offset = 0.095 m** — Swarm-LIO2's `simulation.yaml` hard-codes
  `LI_extrinsic_T: [0, 0, -0.095]`, the fishbot's value. `mount.lidar_z` minus
  `mount.imu_z` must stay 0.095 or that config needs changing too.

## Interface differences vs a fishbot
| | fishbot | quad_mid360 |
|---|---|---|
| arming | n/a | **must publish `Bool true` on `/<ns>/enable`**, else cmd_vel is ignored |
| `cmd_vel` | (v, ω), nonholonomic | body-frame **(vx, vy, vz, yaw rate)**, holonomic |
| last command | latched | latched — publish an explicit zero Twist to stop |
| `wheel_odom` | published | no publisher (bridge idles) |

Holonomy makes the controller *simpler*: no rotate-in-place, no reversing to
break deadlocks. `linear.y` is a real sideways command.

## Tuning note
The plugin's inner velocity loop holds a commanded velocity to ~0.1 mm. Any
hover wobble comes from the **outer** position loop in `drone_bringup.py`:
`KP_Z = 1.2` gave 66.8 mm peak-to-peak, `KP_Z = 0.6` gives 10.9 mm. Keep the
outer gains gentle.

## Two traps found during bring-up
* **Sensors spawned into an already-running world ignore `update_rate`** and
  free-run at the physics step (250 Hz vs the configured 10 Hz), starving the
  GPU. Not model-specific: a stock fishbot added with `ros_gz_sim create` at
  t = 16000 s does it too. Only the launch path (which spawns during startup)
  honours `update_rate` — verified 100 ms lidar / 10 ms IMU.
* `spawn_robots.launch.py` passed `robot_description` untyped, so launch
  inferred its type by YAML-parsing the URDF. This blueprint's header comment
  contains `key: value` text, which xacro copies into the output, so launch died
  with *"Unable to parse the value of parameter robot_description as yaml"*. Now
  wrapped in `ParameterValue(..., value_type=str)` — a latent bug the fishbots
  only avoided by luck.

## 3D inter-robot extrinsics
The local Swarm-LIO2 port now defaults to full SE(3) trajectory alignment,
matching upstream's 3D Kabsch/SVD implementation. The former planar behavior is
still available with `multiuav/cross_world_transform_mode: se2_legacy`.

The upstream excitation gate uses σ2: two independent trajectory directions
are sufficient to determine a proper 3D rigid rotation. A genuinely 3D warm-up
is nevertheless preferable because it improves conditioning of roll, pitch and
z in noisy data.
* `fishbot_maze.world` walls are 1.0 m — drones fly over them.
