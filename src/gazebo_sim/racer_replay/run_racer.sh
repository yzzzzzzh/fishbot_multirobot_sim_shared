#!/usr/bin/env bash
# 6-quad RACER-trajectory replay + Swarm-LIO2, run inside fishbot_gazebo.
# Bring the stack up first (repo root, host):
#   docker compose -f docker-compose.yml -f docker-compose.racer.yml up -d \
#     --force-recreate gazebo swarm_lio2 map_fusion
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")"

pkill -9 -f 'racer_replay\.py' 2>/dev/null
pkill -9 -f 'pose/info@tf2_msgs' 2>/dev/null
sleep 1

echo "[run] ground-truth pose bridge (pose/info -> TF) ..."
ros2 run ros_gz_bridge parameter_bridge \
    /world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
    >/tmp/gt_bridge_racer.log 2>&1 &
BRIDGE_PID=$!

cleanup() {
    echo "[run] stopping drones ..."
    for b in bot1 bot2 bot3 bot4 bot5 bot6; do
        ros2 topic pub -t 2 "/$b/cmd_vel" geometry_msgs/msg/Twist '{}' >/dev/null 2>&1
        ros2 topic pub -t 2 "/$b/enable" std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1
    done
    kill "$BRIDGE_PID" 2>/dev/null
    pkill -9 -f 'pose/info@tf2_msgs' 2>/dev/null
}
trap cleanup EXIT INT TERM

sleep 3
echo "[run] launching RACER replay ..."
python3 -u racer_replay.py
echo "[run] finished."
