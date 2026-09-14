#!/usr/bin/env bash
# Build and run the frozen first-success single-Go2 baseline from this clone.
set -euo pipefail

root_dir=$(cd "$(dirname "$0")" && pwd)
cd "$root_dir"

set -a
. "$root_dir/baseline.env"
set +a

compose=(
  -f docker-compose.oneclick.yml
  -f docker-compose.racer-live.oneclick.yml
  -f docker-compose.quadruped.oneclick.yml
  -f docker-compose.legged.oneclick.yml
  -f docker-compose.legged-single.oneclick.yml
  -f docker-compose.oneclick.images.yml
  -f docker-compose.oneclick.results.yml
)

# The four runtime services and all source/Dockerfiles they consume are in this
# repository. Docker downloads normal OS/ROS build dependencies on first use.
# The final `gazebo` service is the Go2 image, so build its Gazebo parent first.
docker compose "${compose[@]}" build base
docker build --build-arg BASE_IMAGE=fishbot_base:latest \
  -f docker/Dockerfile.gazebo -t fishbot_multirobot_sim-gazebo:latest .
docker compose "${compose[@]}" build swarm_lio2 racer_ros1 gazebo
docker compose "${compose[@]}" stop racer_controller racer_ros1 swarm_lio2 gazebo >/dev/null 2>&1 || true
docker rm -f fishbot_gazebo swarm_lio2_ros2 racer_ros1 racer_controller >/dev/null 2>&1 || true
docker run --rm --ipc=host fishbot_base:latest bash -lc \
  "find /dev/shm -maxdepth 1 -type f \\( -name 'fastrtps_*' -o -name 'sem.fastrtps_*' \\) -delete"
docker compose "${compose[@]}" up -d --force-recreate gazebo swarm_lio2 racer_ros1 racer_controller

until [ "$(docker logs --since 15s swarm_lio2_ros2 2>&1 | grep -c 'ikd-tree size')" -gt 0 ]; do
  sleep 5
done
sleep 10
docker exec fishbot_gazebo bash -lc \
  'source /opt/ros/humble/setup.bash; ros2 daemon stop >/dev/null 2>&1; timeout 20 ros2 param set /bot1/twist_to_control_input auto_trot true'

run_name=${1:-"one_click_$(date +%Y%m%d_%H%M%S)"}
host_run_dir="$root_dir/runs/$run_name"
container_run_dir="/runs/$run_name"
mkdir -p "$host_run_dir"
docker inspect racer_ros1 --format '{{.Image}}' > "$host_run_dir/racer_ros1_image.txt"
docker exec swarm_lio2_ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  source /opt/swarm_lio_ws/install/setup.bash
  exec /usr/bin/python3 /racer_integration/ros2_exploration_recorder.py \\
    --bots 1 --duration 200 --duration-basis sim --platform quadruped \\
    --wait-for-tracking --progress-interval 30 --divergence-error 5.0 \\
    --cloud-bounds -50 -25 0 50 25 1 --output-prefix '$container_run_dir/run' \\
    --ros-args -p use_sim_time:=true > '$container_run_dir/recorder.log' 2>&1
"
test -f "$host_run_dir/run.summary.json"
printf 'Completed self-contained first-success baseline; output: %s\n' "$host_run_dir"
