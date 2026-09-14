# 5-drone coordinated SLAM in the maze

## Run
```bash
DRONE_COUNT=5 DRONE_BOTS=1,2,3,4,5 DRONE_SPACING=1.2 \
  docker compose -f docker-compose.yml -f docker-compose.drone.yml \
  up -d --force-recreate gazebo swarm_lio2 map_fusion
docker exec fishbot_gazebo bash /fishbot_ws/src/gazebo_sim/drone_swarm/run.sh
docker exec fishbot_gazebo bash -lc \
  "cd /fishbot_ws/src/gazebo_sim/drone_swarm && python3 visualize_swarm.py"
```
RTF is 1.0 with 5 drones + 5 SLAM instances (the 5 fishbots ran at 0.22).

## Result (last run) — mixed, and the fusion is a regression

**Per-drone LIO: the best of any run so far.**
```
bot1: ATE3D=0.088 m  ATE2D=0.060 m  max=0.288 m
bot2: ATE3D=0.060 m  ATE2D=0.039 m  max=0.133 m
bot3: ATE3D=0.054 m  ATE2D=0.020 m  max=0.139 m
bot4: ATE3D=0.049 m  ATE2D=0.021 m  max=0.106 m
bot5: ATE3D=0.081 m  ATE2D=0.029 m  max=0.171 m
```
vs 5 fishbots 0.027–0.095 m, vs 1 drone 0.100 m. All 5 tours completed.

**Fusion: 1/4 clean, worse than the fishbots' 3/4.**
```
bot2: yaw err 103.43 deg | xy err 3.112 m   WRONG
bot3: yaw err   5.98 deg | xy err 0.180 m   borderline
bot4: yaw err 159.31 deg | xy err 2.898 m   WRONG
bot5: yaw err   1.66 deg | xy err 0.180 m   OK
```
Fused map 551 545 pts, `intensity` kept, so `swarm_result.png`'s right panel
colours every point by its source drone: bot1/bot5 sit on the walls, bot2/bot4
are diagonal ghost streaks across the whole maze. bot2/bot4 are **not clean
identity swaps** — their estimates match no teammate's true extrinsic. They are
converged-wrong solutions.

## Why fusion regressed: the warm-up's vertical separation backfired

Five quads are 0.64 m across and the chamber is 3.9 m. They cannot loop past
each other horizontally, so the drone whose slot it is climbs to 0.70 m while
the rest hold at 0.35 m. That makes a warm-up collision geometrically
impossible — and it also puts the mover **outside the observers' lidar cone**.

`quad_mid360.yaml` had `v_rays.max_angle: 0.35 rad = +20°`: the downward FOV was
widened to −40° for flying, the upward FOV was never touched. The mover passes
within 0.3–0.6 m horizontally of a parked drone while 0.35 m above it, which is
an elevation of up to 60°.

Measured against ground truth, from bot1 (which is the observer for **all four**
extrinsics, since map_origin is bot1's frame):

| mover | r | max elev | % of loop in FOV | longest visible stretch | extrinsic |
|---|---|---|---|---|---|
| bot2 | 0.90 | **32.3°** | 86 % | 12.9 s | ✗ 103° |
| bot3 | 0.80 | **25.8°** | 88 % | 9.8 s | ~ 5.98° |
| bot4 | 0.70 | **21.3°** | 93 % | 7.8 s | ✗ 159° |
| bot5 | 0.60 | **18.1°** | **100 %** | **16.1 s** | ✓ 1.66° |

bot5 is the only drone that never leaves the FOV and the only clean success.
Three of four movers exceed +20°, so the design defect is real and it is mine.

**Honest limit of this explanation**: it does not predict bot4, which was 93 %
visible and still landed 159° out. FOV fragmentation is a confirmed defect, not
a complete account. A single-variable story would be overfitting four points.

**The fix** (one line, untested): a real Livox mid360 covers **−7° to +52°**.
The `+20°` here is unrealistically narrow for the sensor being modelled. Setting
`v_rays.max_angle` to ~0.9 rad both matches the real device and removes the
blind spot, keeping vertical separation for collision safety.

## Inter-drone distances: a near miss, not a clean sheet
```
min horizontal gap while also within 0.13 m vertically : 0.782 m at t=164 s
rotor-tip contact bound                                : 0.79 m
```
So the conservative bound was breached by 8 mm during the tours, with avoidance
set to YIELD 1.30 / HARD_STOP 0.95. At 0.35 m/s the drones cannot stop inside
0.95 m. **This is a near miss, and the margin is gone.** Raise HARD_STOP, or cap
closing speed.

`min 3D distance 0.404 m` at t=29 is **not** a near miss: that is the warm-up's
intentional 0.35 m vertical separation. A quad is a 0.79 m × 0.11 m disc, not a
sphere — a naive 3D sphere test reports a collision that cannot physically
happen. The run's own console line uses the sphere test and is wrong; the disc
test in the diagnosis is the right one.

## What was carried over from earlier runs (and what it bought)
| lesson | applied | outcome |
|---|---|---|
| fusion is decided by the warm-up | staggered solo loops, one mover at a time | mechanism executed exactly (5 closed circles visible in `swarm_result.png`) |
| **the loop must CLOSE** — fishbot bot5 drove 35 % of its circle, a short arc is near-collinear, σ2 collapsed to 1.6 | time-parameterised, one full circle per slot; σ2 ≈ N·r²/2 ≥ 28 vs gate 8 | no drone failed for lack of excitation |
| last slot must not get the largest loop | radii descend 1.00→0.60 with slot order | last slot (bot5) is the *best* result now |
| identical paths → identity swaps | distinct radii | no clean swaps occurred |
| first match is locked forever | one mover per slot | — |
| monotonic carrot (tour ends at start) | done | no premature "arrived" |
| 12 s disarmed for IMU init | done | — |
| gentle outer gains (KP_Z 0.6) | done | — |
| inflation 0.45 for a 0.397 m quad | done | no wall strikes |
| lever arm: map close, not far | one sector each, flown through | — |
| **save the intensity channel** | done | paid off immediately: ghosts are attributable per drone |

The regression came from the one thing that had **no** precedent — vertical
separation, which ground robots cannot do — and it was not checked against the
sensor's FOV before flying.

## Outputs
`global_map.ply` (x,y,z,intensity), `traj_botN.csv`, `ate_report.txt`,
`extrinsics_report.txt`, `dist_pairs.csv`, `swarm_result.png`,
`swarm_distances.png`.
