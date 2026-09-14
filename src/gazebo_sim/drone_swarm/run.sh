#!/usr/bin/env bash
# 5-drone coordinated SLAM, run inside fishbot_gazebo. Requires:
#   DRONE_COUNT=5 DRONE_BOTS=1,2,3,4,5 DRONE_SPACING=1.2 \
#     docker compose -f docker-compose.yml -f docker-compose.drone.yml \
#     up -d --force-recreate gazebo swarm_lio2 map_fusion
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")"

# Re-entrancy guard: a stale controller publishes competing cmd_vel at 20 Hz and
# cancels the new one out. Exact patterns only -- NEVER a bare
# `pkill parameter_bridge`, which also kills Gazebo's own cmd_vel/enable/lidar/imu
# bridges and freezes everything.
pkill -9 -f 'drone_swarm\.py' 2>/dev/null
pkill -9 -f 'pose/info@tf2_msgs' 2>/dev/null
sleep 1

echo "[run] starting ground-truth pose bridge (pose/info -> TF) ..."
ros2 run ros_gz_bridge parameter_bridge \
    /world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
    >/tmp/gt_bridge_swarm.log 2>&1 &
BRIDGE_PID=$!

cleanup() {
    echo "[run] stopping drones ..."
    for b in bot1 bot2 bot3 bot4 bot5; do
        ros2 topic pub -t 2 "/$b/cmd_vel" geometry_msgs/msg/Twist '{}' >/dev/null 2>&1
        ros2 topic pub -t 2 "/$b/enable" std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1
    done
    kill "$BRIDGE_PID" 2>/dev/null
    pkill -9 -f 'pose/info@tf2_msgs' 2>/dev/null
}
trap cleanup EXIT INT TERM

sleep 3
echo "[run] launching 5-drone swarm ..."
python3 -u drone_swarm.py     # -u: else the report sits in stdout's block buffer
echo "[run] finished."
