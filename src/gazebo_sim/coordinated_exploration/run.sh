#!/usr/bin/env bash
# Launch coordinated multi-robot exploration inside the fishbot_gazebo container.
#   - bridges Gazebo ground-truth pose/info -> ROS TF (control feedback + ATE)
#   - runs the pure-pursuit exploration/controller node
# Usage (from host):  docker exec fishbot_gazebo bash /fishbot_ws/src/gazebo_sim/coordinated_exploration/run.sh
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")"

echo "[run] starting ground-truth pose bridge (pose/info -> TF) ..."
ros2 run ros_gz_bridge parameter_bridge \
    /world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
    >/tmp/gt_bridge.log 2>&1 &
BRIDGE_PID=$!

cleanup() {
    echo "[run] stopping robots and cleaning up ..."
    for b in bot1 bot2 bot3 bot4; do
        ros2 topic pub -t 3 "/$b/cmd_vel" geometry_msgs/msg/Twist \
            '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1
    done
    kill "$BRIDGE_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

sleep 3   # let the bridge discover poses
echo "[run] launching exploration controller ..."
python3 coordinated_exploration.py

echo "[run] finished."
