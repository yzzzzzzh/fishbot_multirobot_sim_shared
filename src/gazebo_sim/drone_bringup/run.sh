#!/usr/bin/env bash
# Bring-up check for the quad_mid360 drone, run inside fishbot_gazebo.
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")"

# Re-entrancy guard. Kill only OUR processes, by exact pattern.
# NEVER a bare `pkill parameter_bridge`: that would also take out Gazebo's own
# cmd_vel/enable/lidar/imu bridges and freeze the robots.
pkill -9 -f 'drone_bringup\.py' 2>/dev/null
pkill -9 -f 'pose/info@tf2_msgs' 2>/dev/null
sleep 1

echo "[run] starting ground-truth pose bridge (pose/info -> TF) ..."
ros2 run ros_gz_bridge parameter_bridge \
    /world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
    >/tmp/gt_bridge_drone.log 2>&1 &
BRIDGE_PID=$!

cleanup() {
    echo "[run] stopping drone ..."
    ros2 topic pub -t 3 /bot1/cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1
    ros2 topic pub -t 3 /bot1/enable std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1
    # kill our own bridge by PID -- `ros2 run` leaves the real parameter_bridge
    # as a child, so kill the whole process group.
    kill -- -$(ps -o pgid= "$BRIDGE_PID" | tr -d ' ') 2>/dev/null || kill "$BRIDGE_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

sleep 3
echo "[run] launching bring-up ..."
# -u: unbuffered. Without it the report sits in stdout's block buffer whenever
# this script's output is redirected to a file, and only ROS's stderr logging
# shows up.
python3 -u drone_bringup.py
echo "[run] finished."
