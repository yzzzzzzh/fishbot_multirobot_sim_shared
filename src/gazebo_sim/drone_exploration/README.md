# Single-drone SLAM in the maze

The `maze_exploration/` flow, flown: plan a collision-free A* tour, track it
closed-loop against ground truth while Swarm-LIO2 runs, report ATE, dump the
fused map.

## Run
```bash
docker compose -f docker-compose.yml -f docker-compose.drone.yml \
  up -d --force-recreate gazebo swarm_lio2 map_fusion
docker exec fishbot_gazebo bash /fishbot_ws/src/gazebo_sim/drone_exploration/run.sh
docker exec fishbot_gazebo bash -lc \
  "cd /fishbot_ws/src/gazebo_sim/drone_exploration && python3 visualize_drone.py"
```

## Result (last run)
```
samples        : 2350 over 131 s
path flown     : 36.4 m (3D)          planned tour 38.3 m, 4 corners + home
altitude       : 0.05 - 0.61 m
ATE 3D         : RMS 0.114 m  max 0.262 m
ATE 2D (xy)    : RMS 0.100 m  max 0.261 m
ATE z only     : RMS 0.055 m  max 0.112 m
back to spawn  : 0.084 m
fused map      : 146 015 points
```
`drone_result.png` (top-down map + trajectory), `drone_altitude_error.png`
(altitude and error vs time).

**Versus the ground robots**: the 5-fishbot maze run scored ATE 0.027–0.095 m.
The drone's comparable 2D figure is 0.100 m — same order, at the worse end.
Expected: a quadrotor has no wheel constraint, vibrates, and moves in 6 DOF.

Two things visible in `drone_altitude_error.png` worth knowing:
* **SLAM altitude drifts high**, from ~2 cm at t=20 s to ~7 cm by t=110 s. Real
  LIO z drift, not a frame convention: the SLAM origin is the pre-takeoff
  ground-truth pose, so `wz = z_rest + pz` is unbiased by construction.
* Error climbs to 0.26 m around t=55 s then **drops to 0.04 m at t=59 s**.
  Swarm-LIO2 has no loop closure, so this is drift reversing direction as the
  drone turns into the bottom-left corner, not a correction.

Caveat on the ATE number itself: ground truth arrives at 50 Hz and SLAM odom at
10 Hz, and the two are paired as "latest available" with no time sync. At
0.35 m/s that adds up to ~3.5 cm of apparent error. `maze_exploration.py` does
the same thing, so the comparison above is apples to apples, but the absolute
figure is pessimistic by a few cm.

## Design notes
* **Inflation 0.45 m, not the fishbot's 0.25.** The quad's circumscribed radius
  is 0.397 m (rotor tips at ±0.235 x, ±0.320 y) against the fishbot's 0.15 m.
  Corridors are 1.40 m wide (pitch 1.6 − thickness 0.2), leaving a 0.50 m free
  channel — verified plannable corner-to-corner before flying.
* **Yaw is held at the spawn value the whole flight.** The drone is holonomic,
  so it strafes instead of turning: the footprint never rotates, and there is no
  rotate-in-place and no reversing to break deadlocks.
* **Cruise 0.50 m with a ±0.12 m sine.** The lidar sits 0.15 m above base_link
  and the walls are 1.0 m, so this keeps it looking at wall rather than over it,
  while still making the trajectory genuinely 3D.
* **12 s disarmed on the ground before takeoff** — Swarm-LIO2 initialises its
  IMU against gravity and wants a still vehicle. Rotors off is as still as it gets.
* The monotonic carrot index matters here for the same reason it does for the
  fishbots: the tour ends where it starts, so a nearest-point search would
  declare "arrived" at t = 0.

## What this run does and does not show
It measures **flying LIO accuracy for one drone**. It says nothing about the
inter-robot problems (σ2 gate, teammate identity, extrinsics, ghost maps) that
dominated the multi-robot runs — those need ≥2 robots by definition.

Going multi-drone in 3D still benefits from a 3D-spanning warm-up and needs a
maze with walls taller than 1.0 m. The Swarm-LIO2 port now defaults to the
upstream-style full SE(3) `TrajMatching`; `se2_legacy` remains selectable for
reproducing earlier planar runs.
