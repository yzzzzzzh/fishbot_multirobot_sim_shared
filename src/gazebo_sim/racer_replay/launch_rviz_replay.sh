#!/usr/bin/env bash
# Visualise the SAVED 600s RACER-replay results in RViz — one window per drone,
# on the host desktop (no sim re-run). Loads output_600s_v2/. Usage: ./launch_rviz_replay.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
DISP="${DISPLAY:-:1}"
DIR="${1:-output_600s_v2}"          # which output_*/ run dir to visualise (arg 1)
IMG=fishbot_base:latest
BOTS="bot1 bot2 bot3 bot4 bot5 bot6"

echo "[1/4] allow container X access on $DISP"
DISPLAY="$DISP" xhost +local:root >/dev/null

echo "[2/4] generate per-drone rviz configs"
for b in $BOTS; do sed "s|/BOT/|/$b/|g" "$HERE/replay_template.rviz" > "$HERE/cfg_$b.rviz"; done

echo "[3/4] start latched data publisher (fishbot_base container)"
docker rm -f rviz_replay >/dev/null 2>&1 || true
docker run -d --name rviz_replay --network host \
  -e DISPLAY="$DISP" -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 -e REPLAY_DIR="/data/$DIR" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HERE":/data \
  -v /home/yunze/racer_lite_3d:/racer \
  "$IMG" bash -lc "source /opt/ros/humble/setup.bash && python3 -u /data/rviz_replay_pub.py" >/dev/null
sleep 8
docker logs rviz_replay 2>&1 | tail -1

echo "[4/4] open one RViz per drone on $DISP"
for b in $BOTS; do
  docker exec -d rviz_replay bash -lc \
    "source /opt/ros/humble/setup.bash && DISPLAY=$DISP rviz2 -d /data/cfg_$b.rviz --ros-args -r __node:=rviz_$b >/tmp/rviz_$b.log 2>&1"
  sleep 1.5
done
echo "done — 6 RViz windows should be on the $DISP desktop."
echo "stop everything with:  docker rm -f rviz_replay"
