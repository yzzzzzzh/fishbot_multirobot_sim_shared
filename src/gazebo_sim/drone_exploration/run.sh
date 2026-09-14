#!/usr/bin/env bash
# Single-drone SLAM run, executed inside fishbot_gazebo.
# Requires: docker compose -f docker-compose.yml -f docker-compose.drone.yml \
#             up -d --force-recreate gazebo swarm_lio2 map_fusion
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")"

# Re-entrancy guard. Exact patterns only -- NEVER a bare `pkill parameter_bridge`,
# which would also kill Gazebo's own cmd_vel/enable/lidar/imu bridges.
pkill -9 -f 'drone_exploration\.py' 2>/dev/null
pkill -9 -f 'pose/info@tf2_msgs' 2>/dev/null
sleep 1

echo "[run] starting ground-truth pose bridge (pose/info -> TF) ..."
ros2 run ros_gz_bridge parameter_bridge \
    /world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
    >/tmp/gt_bridge_drone_slam.log 2>&1 &
BRIDGE_PID=$!

cleanup() {
    echo "[run] stopping drone ..."
    ros2 topic pub -t 3 /bot1/cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1
    ros2 topic pub -t 3 /bot1/enable std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1
    kill "$BRIDGE_PID" 2>/dev/null
    pkill -9 -f 'pose/info@tf2_msgs' 2>/dev/null
}
trap cleanup EXIT INT TERM

sleep 3
echo "[run] launching drone exploration ..."
python3 -u drone_exploration.py     # -u: else the report sits in stdout's buffer
echo "[run] finished."
