# Coordinated multi-robot exploration

Drives the 4 fishbots along collision-free, kinematically-feasible coverage
paths while Swarm-LIO2 + map_fusion build a shared map, then reports SLAM
accuracy against Gazebo ground truth.

## What it does
1. **Plan** (`arena.py`): transcribes every wall/pillar/block from
   `worlds/fishbot2.world` into an inflated occupancy grid (robot radius +
   margin) and runs A* + string-pull smoothing. bot2/bot4 are routed through the
   central tunnels into the right room; bot1/bot3 sweep the left room.
2. **Control** (`coordinated_exploration.py`): closed-loop pure-pursuit per
   robot. Feedback is the Gazebo ground-truth pose (`pose/info` bridged to TF).
   Lower-id robots have priority; higher-id robots yield when a neighbour is
   close and ahead, so robots never collide with each other.
3. **Evaluate**: on completion it saves the fused cloud, per-robot trajectories,
   and the Absolute Trajectory Error (SLAM estimate vs ground truth).

## Run
The sim must be up (`docker compose up -d gazebo swarm_lio2 map_fusion foxglove`).
```bash
docker exec fishbot_gazebo bash /fishbot_ws/src/gazebo_sim/coordinated_exploration/run.sh
docker exec fishbot_gazebo bash -lc \
  "cd /fishbot_ws/src/gazebo_sim/coordinated_exploration && python3 visualize.py"
```

## Outputs (`output/`)
| file | contents |
|------|----------|
| `global_map.ply` | fused SLAM point cloud of the whole arena |
| `traj_botN.csv` | t, ground-truth xy, SLAM xy (world frame) |
| `ate_report.txt` | ATE (RMSE) per robot |
| `exploration_result.png` | top-down map + trajectories |

## Notes / limitations
- Ground truth comes from `ign` `pose/info` (the world's `gt/odom` topic has no
  publisher). It is used both as control feedback and as the ATE reference.
- Planning uses the true world geometry (a scripted demo/benchmark), not online
  frontier exploration — the repo's autonomous `navigation` stack is not wired in.
- Do **not** `pkill -f parameter_bridge`: it also kills Gazebo's own
  cmd_vel/lidar/imu bridges. `run.sh` only kills the single bridge it starts.
