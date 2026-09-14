# Six-drone SE(3) / legacy-SE(2) comparison

## Test definition

- World: `drone_racer.world`
- Robot: six `quad_mid360` UAVs
- Motion: the same 3D initialization and the same first 60 simulated seconds
  of the RACER trajectories
- Ground truth: Gazebo
- ATE: direct position RMSE after applying each UAV's known spawn
  origin/yaw to its local LIO output
- Middleware: ROS 2 Humble

Three runs must be distinguished:

1. **SE(3), multi-UAV enabled**: full cross-world rotation and xyz
   translation; six UAVs each connected to five teammates.
2. **Legacy SE(2), pure LIO control**: the old yaw+x+y parameters are retained,
   but teammate discovery is disabled with `actual_uav_num=1`. This isolates
   local LIO accuracy.
3. **Legacy SE(2), multi-UAV enabled**: teammate discovery is enabled. This run
   failed during the pre-replay 3D initialization and is reported as an
   integration failure, not as a valid local-LIO accuracy baseline.

## Valid local-LIO comparison

| UAV | SE(3) multi-UAV ATE (m) | Legacy SE(2) pure-LIO ATE (m) |
|---|---:|---:|
| bot1 / R0 | 0.174 | 0.204 |
| bot2 / R1 | 0.547 | 0.548 |
| bot3 / R2 | 0.286 | 0.228 |
| bot4 / R3 | 0.885 | 0.623 |
| bot5 / R6 | 0.149 | 0.159 |
| bot6 / R7 | 0.577 | 0.687 |

| Aggregate metric | SE(3) multi-UAV | Legacy SE(2) pure LIO |
|---|---:|---:|
| Pooled ATE RMSE | 0.508 m | 0.462 m |
| Mean per-UAV ATE | 0.436 m | 0.408 m |
| Median position error | 0.289 m | 0.298 m |
| 95th percentile | 0.994 m | 0.815 m |
| Maximum error | 1.564 m | 1.319 m |
| Tracking-vs-reference RMSE | 0.388 m | 0.393 m |
| Mean odometry rate | 19.98 Hz | 20.02 Hz |
| Samples | 7194 | 7206 |

The two valid runs therefore have comparable ego-LIO accuracy. Full SE(3) is
needed for cross-world relative pose; it is not expected to make independent
local LIO intrinsically more accurate.

## Cross-world relative-pose result

The SE(3) run initialized all six UAVs. A runtime `bot4 -> bot6` transform was:

- roll/pitch/yaw: `(-0.746, 2.857, -0.014)` degrees
- translation: `(0.901, 3.874, 0.463)` metres

The non-zero roll, pitch, and z confirm that the full transform survives the
solver, state, message, TF, and map-fusion chain.

An end-of-run snapshot also showed that relative-edge quality was not uniform.
Three direct observations had translation errors of `0.196–0.490 m` and
rotation errors of `1.685–2.953 deg`, while several graph-infected root edges
had translation errors of `0.838–4.175 m` and rotation errors of `38–49 deg`.
Thus the SE(3) implementation is functioning, but reflective-target identity
association and graph infection still require improvement before treating the
shared-frame estimate as uniformly accurate.

The legacy-SE(2) multi-UAV run produced no valid cross-world extrinsic on any of
the six nodes. Its temporary reflective-object trackers proliferated, no
trajectory match succeeded, odometry rate fell to `17.73 Hz`, and LIO had
already diverged before the measured 60-second replay began. Its pooled
position RMSE was `5794.955 m`; this is a failure signature, not a meaningful
SE(2) relative-pose ATE.

## Why an earlier original-code run could still have low ATE

The saved historical `output_final` run has per-UAV ATE values of
`0.286–0.497 m` (pooled `0.387 m`) over 180 seconds. Those CSV files contain
ego LIO and Gazebo truth, but no saved cross-world-extrinsic history, so they
do not establish that relative-pose initialization succeeded.

The new pure-LIO control reproduces the expected low-error behavior under the
legacy parameters (`0.159–0.687 m` per UAV). This isolates the new failure to
the active multi-UAV discovery/alignment path rather than the base LiDAR-inertial
odometry.

The 3D initialization contains a vertical climb followed by horizontal motion.
The full SE(3) excitation/alignment uses the vertical component. The legacy
solver discards z, roll, and pitch, so the climb supplies it no excitation and
different-height trajectories are harder to disambiguate. In this run that
left the temporary-target manager accumulating unmatched candidates.

