#!/usr/bin/env bash
# Launch 5-robot maze exploration inside the fishbot_gazebo container.
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")"

# Re-entrancy guard: a previous controller left running would publish competing
# cmd_vel at 20 Hz and cancel ours out. Kill only OUR own processes by exact
# pattern -- never a bare `pkill parameter_bridge`, which would also take out
# Gazebo's cmd_vel/lidar/imu bridges.
pkill -9 -f 'maze_exploration\.py' 2>/dev/null
pkill -9 -f 'pose/info@tf2_msgs' 2>/dev/null
sleep 1

echo "[run] starting ground-truth pose bridge (pose/info -> TF) ..."
ros2 run ros_gz_bridge parameter_bridge \
    /world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
    >/tmp/gt_bridge_maze.log 2>&1 &
BRIDGE_PID=$!

cleanup() {
    echo "[run] stopping robots ..."
    for b in bot1 bot2 bot3 bot4 bot5; do
        ros2 topic pub -t 3 "/$b/cmd_vel" geometry_msgs/msg/Twist \
            '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1
    done
    kill "$BRIDGE_PID" 2>/dev/null   # only our own bridge, never pkill
}
trap cleanup EXIT INT TERM

sleep 3
echo "[run] launching maze exploration controller ..."
python3 maze_exploration.py
echo "[run] finished."
