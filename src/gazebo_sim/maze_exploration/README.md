# 5-robot coordinated SLAM in a maze

Generates a maze world, drives 5 fishbots along collision-free tours that make
them meet several times, and reports SLAM accuracy + inter-robot distances.

## Files
| file | role |
|------|------|
| `maze_gen.py` | deterministic maze (recursive backtracker + braiding + central chamber). Writes **both** `../worlds/fishbot_maze.world` and `maze_def.json` (single source of truth for walls/spawns) |
| `gridmap.py` | inflated occupancy grid + A* + string-pull smoothing |
| `maze_exploration.py` | 5-robot pure-pursuit controller, ground-truth feedback, avoidance, logging, ATE |
| `visualize_maze.py` | top-down map/trajectories + pair-distance plots |
| `run.sh` | starts the gt bridge and the controller (with a re-entrancy guard) |

## Run
```bash
docker compose up -d gazebo swarm_lio2 map_fusion foxglove     # 5 bots + maze (see docker-compose.yml)
docker exec fishbot_gazebo bash /fishbot_ws/src/gazebo_sim/maze_exploration/run.sh
docker exec fishbot_gazebo bash -lc \
  "cd /fishbot_ws/src/gazebo_sim/maze_exploration && python3 visualize_maze.py"
```
To change the maze, edit the parameters at the top of `maze_gen.py`, re-run it,
then restart the `gazebo` service.

## Avoidance design
Two tiers plus a deadlock breaker:
* **YIELD (0.70 m)** – stop for a *higher-priority* (lower-id) neighbour ahead.
* **HARD_STOP (0.50 m)** – *anyone* stops for *anyone* very close ahead.
* **Deadlock breaker** – head-on in a 1.4 m corridor freezes both (one yields,
  the other hard-stops). After 5 s blocked, the lower-priority robot reverses
  for 2.5 s to open a gap. Observed to need up to 4 repeats but always resolves.

## Results (last run)
All 5 tours completed; **15 close encounters**; **global min pair distance
0.49 m → no collision**; per-robot ATE 0.027–0.095 m; fused cloud 328 825 pts.

## Warm-up experiments (what actually drives fusion quality)

A 10 s warm-up in the open chamber, before dispersing, was tried two ways. Both
fail Swarm-LIO2's teammate identification, for *opposite* reasons:

| warm-up | fused map | inter-robot extrinsics (yaw error) | why |
|---|---|---|---|
| none | 328 k pts | bot2 0.14° ✓, bot4 16° ✗, bot5 7.4° ✗, bot3 none ✗ | teammates only briefly co-visible |
| **carousel** (same radius) | 358 k pts | **identities swapped** (est. "bot2" = bot5's true pose) | tangential motion is observable, but all five paths are the *same circle* → trajectory matching cannot tell them apart |
| **radial spokes** (own direction each) | 92 k pts | **none estimated at all** ✗ | paths are distinguishable, but radial motion is *along the line of sight* → almost no bearing change → degenerate for lidar observation |
| **concentric circles, distinct radii** | 466 k pts | bot2 **0.69°** ✓, bot3 **0.20°** ✓, bot4 ✗, bot5 ✗ | circles give observability, different radii give distinguishability → 2/4 correct |
| **staggered solo loops** (current) | **371 k pts, visually ghost-free** | bot2 **2.02°** ✓, bot3 **1.68°** ✓, bot4 **2.61°** ✓, bot5 not estimated | one moving cluster at a time → **zero mis-assignments, 3/4 correct** |

### The sigma2 gate predicts the outcome exactly

Measuring each robot's own excitation during its solo slot against the code's
`traj_matching_start_thresh = 8.0`:

| robot | slot | loop circumference | actually driven | sigma2 | extrinsic |
|---|---|---|---|---|---|
| bot2 | 39-61 s | 3.77 m | 2.44 m | 13.0 | ✅ 2.02° |
| bot3 | 61-83 s | 4.08 m | 3.07 m | 21.9 | ✅ 1.68° |
| bot4 | 83-105 s | 4.40 m | 3.57 m | 31.8 | ✅ 2.61° |
| bot5 | 105-127 s | 4.71 m | **1.65 m** | **1.6** | ❌ none |

Perfect correspondence: every robot that cleared sigma2 > 8 was estimated, the
one that did not was not. bot5 fails because it holds the *last* slot *and* the
*largest* loop, so it completes only 35 % of its circle before DISPERSE_T ends
the warm-up. Fix: raise `SOLO_SLOT` (22 s only gets 35-80 % of a loop driven
under pursuit) and/or give the last slot the smallest loop.

Net: fusion needs motion that is simultaneously **tangentially observable** and
**distinct per robot** — the concentric-circle warm-up satisfies both and is the
only configuration where more than one teammate locks in (bot3 goes from *never
estimated* to the most accurate of all, 0.20°). bot4/bot5 still mis-associate,
so rotated ghost streaks remain in the cloud. Per-robot LIO is unaffected and
excellent in every configuration (ATE 0.016–0.095 m).

Formation detail: each robot runs at `v_k = OMEGA * r_k` (OMEGA = 0.178 rad/s),
so the five rotate as a rigid body — the 72° spacing holds and the minimum
separation stays at 1.19 m, well clear of the 0.70 m yield radius.

## Known finding: inter-robot extrinsics don't fully converge
Per-robot LIO is excellent (2–9 cm ATE), but Swarm-LIO2's *relative* extrinsic
estimate is only good for bot2 (71.86° vs 72.0° true). bot3 never got a TF at
all; bot4 is off by 16°, bot5 by 7.4°. Their map contributions therefore land
rotated in the fused cloud (the "ghost" copies in `maze_result.png`).

Cause: Swarm-LIO2 identifies teammates by mutual observation + trajectory
matching, which needs the robots **close, in line of sight, and moving**. In
this run they are clustered but *stationary* for the first 12 s (SLAM init),
then spend ~100 s dispersed and occluded by maze walls. Only pairs that were
close *while moving* converged.

Likely fix (not implemented): add a "warm-up" phase where all 5 drive around the
open central chamber in mutual view for ~30 s before dispersing into the maze.

## Gotchas
- Never `pkill -f parameter_bridge` — it also kills Gazebo's cmd_vel/lidar/imu
  bridges. Kill only `maze_exploration.py` / `pose/info@tf2_msgs` by pattern.
- Stopping the host-side `docker exec` does **not** kill the in-container
  controller; a stale one publishes competing cmd_vel at 20 Hz and cancels the
  new one out. `run.sh` guards against this.
- Tours end at the spawn point, so the goal test must use a monotonic carrot
  index — a nearest-point search declares "arrived" at t=0.
