# Fishbot Navigation Module

This package hosts a lightweight per-robot navigation controller (`fishbot_navigation`).
It assumes every robot provides the topics `/ROBOT/odom` and `/ROBOT/cmd_vel` plus a
`/ROBOT/move_base/goal` topic where RViz publishes 2D Nav Goals.

## Features

- Goal-oriented proportional controller with heading alignment and slowdown near the waypoint.
- Works with arbitrary robot namespaces (e.g. `bot1`, `bot2`, ...).
- Publishes `/ROBOT/nav_path` (for RViz) and `/ROBOT/nav_status` (text state machine output).
- `/ROBOT/cancel_navigation` service instantly stops the robot.

## Topics & services

- Subscribes to `/ROBOT/wheel_odom` (configurable) and `/ROBOT/move_base/goal`.
- Publishes `/ROBOT/cmd_vel`, `/ROBOT/nav_status` and `/ROBOT/nav_path`.
- Exposes `/ROBOT/cancel_navigation` (`std_srvs/Trigger`), clearing the current goal.

## Parameters

All tunables live under `config/navigation.yaml` and are loaded for every robot through the launch file.
Key entries:

- `input_topic`/`output_topic`: remap odom/goal/cmd_vel topics per namespace.
- `controller`: gains, saturation and tolerances for the proportional controller.
- `align_final_heading`: align robot yaw with RViz goal after reaching the position.

## Usage

```bash
source /fishbot_ws/install/setup.bash
ros2 launch fishbot_navigation navigation.launch.py robots:=bot1,bot2
```
