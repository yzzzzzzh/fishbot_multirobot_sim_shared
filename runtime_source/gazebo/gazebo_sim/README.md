# Gazebo Simulation Module

This workspace contains the `gazebo_sim` package, which keeps Gazebo worlds and robot blueprints in one place.

## Layout
- `gazebo_sim/worlds/` – ready-to-use Gazebo world files (e.g. `fishbot.world`).
- `gazebo_sim/robots/<robot_name>/` – blueprint folders; each robot has:
  - `config/*.yaml` with tunable parameters (link frames, geometry, plugin settings).
  - `urdf/*.xacro` that consumes the YAML file and builds the description used by Gazebo.
- `gazebo_sim/launch/` – launch files (e.g. `gazebo.launch.py`) that start Gazebo with the selected world.
## Parameterised robot example
`fishbot_v1` stores its main dimensions and plugin parameters in `robots/fishbot_v1/config/fishbot_v1.yaml`. The matching `robots/fishbot_v1/urdf/fishbot_v1.xacro` loads that YAML automatically, so tweaking the blueprint is as simple as editing the YAML file.

### Available robot blueprints
- `fishbot_v1` – entry-level differential drive with a planar LiDAR.
- `fishbot_v2_3d` – adds a generic 3D LiDAR (16 vertical beams) for richer perception.
- `fishbot_mid360` – Livox Mid-360 inspired configuration (360 x 59 deg FOV, 0.1–40 m range) for mobile robots that need dense surround coverage.[1]

## Running Gazebo and spawning robots
1. Build the workspace with `colcon build` and source `install/setup.bash`.
2. Launch Gazebo and the robot_state_publisher/spawner pipeline in one shot:
   ```bash
   ros2 launch gazebo_sim quick.launch.py
   ```
   `quick.launch.py` accepts several launch arguments, e.g.:
   - `robot:=fishbot_v1` – choose a blueprint subfolder from `robots/`.
   - `count:=2` – spawn multiple instances.
   - `name_prefix:=rover` – control the namespace/entity naming.
   - Pose arguments (`x`, `y`, `z`, `dx`, `dy`, `dz`, `yaw`, `yaw_step`) mirror the previous helper script.
   - `reference_frame:=world` if you want to target a different frame from Gazebo.

Behind the scenes each robot instance evaluates its xacro, publishes the description via `robot_state_publisher`, and is spawned into Gazebo using `gazebo_ros/spawn_entity.py`. Static TFs are automatically published from the URDF, so no extra helper nodes are required.

Need to spawn robots separately (e.g. against an already running Gazebo world)? Use:
```bash
ros2 launch gazebo_sim spawn_robots.launch.py robot:=fishbot_v1 count:=1
```

## Useful commands (run inside the container)

| Task                                         | Command                                                                                                                                                                                  |
| -------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Pause simulation                             | `ign service -s /world/default/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req "pause: true"`                                          |
| Resume simulation                            | `ign service -s /world/default/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req "pause: false"`                                         |
| Advance sim by N milliseconds *(multi-step)* | `ign service -s /world/default/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req "pause: true multi_step: 1"`                            |
| Slowdown (×0.5)                              | `ign service -s /world/default/set_physics --reqtype ignition.msgs.Physics --reptype ignition.msgs.Boolean --timeout 3000 --req "max_step_size: 0.0005 real_time_update_rate: 0"`        |
| Speedup (×2)                                 | `ign service -s /world/default/set_physics --reqtype ignition.msgs.Physics --reptype ignition.msgs.Boolean --timeout 3000 --req "max_step_size: 0.002 real_time_update_rate: 0"`         |
| Change real-time factor                      | `ign service -s /world/default/set_physics --reqtype ignition.msgs.Physics --reptype ignition.msgs.Boolean --timeout 3000 --req 'profile_name: "default_physics" real_time_factor: 0.5'` |


[1]: https://www.livoxtech.com/mid-360
