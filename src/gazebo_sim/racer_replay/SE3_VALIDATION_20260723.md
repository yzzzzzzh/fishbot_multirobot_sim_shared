# Swarm-LIO2 full-SE(3) validation

- Scenario: `drone_racer.world`
- Robot: `quad_mid360`
- Drones: `bot1..bot6` (RACER R0, R1, R2, R3, R6, R7)
- Replay: 60 simulated seconds after 3D initialization
- Middleware: ROS 2 Humble

## Runtime configuration

- `multiuav/cross_world_transform_mode = se3`
- `multiuav/mo_force_extrinsic_se2 = false`
- `multiuav/mo_ignore_z_meas = false`

All six drones initialized, each reported five connected teammates, and the
root log reported `[Initialization] Found all teammates`.

One runtime transform from `bot4/world` to bot6 was:

- roll/pitch/yaw: `(-0.746465, 2.857245, -0.013653)` degrees
- translation: `(0.901048, 3.873664, 0.463166)` metres

The non-zero roll, pitch, and z values show that the runtime transform was not
projected to yaw+x+y.

## Deterministic solver test

The test used non-zero roll, pitch, yaw and a translation with `z = 2.40 m`.

- rotation error: `1.0251e-16 rad`
- translation error: `4.44089e-16 m`
- mean point error: `9.82342e-16 m`
- legacy SE(2) check: translation z remained exactly zero

## Scope

The run validates the SE(3) solver, mode selection, six-DOF state propagation,
message/TF path, multi-drone initialization, and stable 60-second ego LIO. The
trajectory metrics are in
`output_se3_60s_20260723_clean/ate_report.txt`.

This does not establish that every reflectivity-based association edge is
high-accuracy. Some short-run graph edges were less accurate than others;
unique, staggered 3D initialization trajectories are recommended when shared
map alignment accuracy is the acceptance criterion.
