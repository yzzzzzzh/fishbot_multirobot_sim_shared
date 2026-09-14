#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
DISP="${DISPLAY:-:1}"
NAME="racer_v41_rviz"

DISPLAY="$DISP" xhost +local:root >/dev/null
docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" --network host \
  -e DISPLAY="$DISP" \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e QT_X11_NO_MITSHM=1 \
  -e REPLAY_PREFIX=/workspace/artifacts/racer_6uav_official/short_v41_no_progress_guard/run \
  -e REPLAY_WORLD=/workspace/src/gazebo_sim/worlds/drone_racer.world \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$ROOT":/workspace \
  fishbot_base:latest \
  bash -lc "source /opt/ros/humble/setup.bash && python3 -u /workspace/src/gazebo_sim/racer_replay/rviz_npz_dual_pub.py" \
  >/dev/null

sleep 3
docker exec -d "$NAME" bash -lc \
  "source /opt/ros/humble/setup.bash && rviz2 -d /workspace/src/gazebo_sim/racer_replay/v41_observed_clouds.rviz --ros-args -r __node:=rviz_v41_observed_clouds >/tmp/rviz_clouds.log 2>&1"
docker exec -d "$NAME" bash -lc \
  "source /opt/ros/humble/setup.bash && rviz2 -d /workspace/src/gazebo_sim/racer_replay/v41_ground_truth.rviz --ros-args -r __node:=rviz_v41_ground_truth >/tmp/rviz_truth.log 2>&1"
echo "RViz2 windows launched on $DISP; stop with: docker rm -f $NAME"
