#!/usr/bin/env bash
# Run the frozen first-success single-Go2 baseline for a requested number of sim seconds.
set -euo pipefail

shared_dir=$(cd "$(dirname "$0")" && pwd)
repo_dir=${FISHBOT_REPO_DIR:-"$shared_dir/../fishbot_multirobot_sim"}
export FISHBOT_SHARED_DIR="$shared_dir"
if [ ! -f "$repo_dir/docker-compose.yml" ]; then
  printf 'FISHBOT_REPO_DIR must point to fishbot_multirobot_sim; got %s\n' "$repo_dir" >&2
  exit 2
fi
duration=${1:-1000}
run_dir=${2:-/tmp/first_success_baseline_$(date +%Y%m%d_%H%M%S)}

set -a
. "$shared_dir/baseline.env"
set +a

compose=(
  -f "$repo_dir/docker-compose.yml"
  -f "$repo_dir/docker-compose.racer-live.yml"
  -f "$repo_dir/docker-compose.quadruped.yml"
  -f "$repo_dir/docker-compose.legged.yml"
  -f "$repo_dir/docker-compose.legged-single.yml"
  -f "$shared_dir/docker-compose.first-success.yml"
)

docker compose "${compose[@]}" stop racer_controller racer_ros1 swarm_lio2 gazebo >/dev/null 2>&1 || true
docker run --rm --ipc=host fishbot_base:latest bash -lc \
  "find /dev/shm -maxdepth 1 -type f \\( -name 'fastrtps_*' -o -name 'sem.fastrtps_*' \\) -delete"
docker compose "${compose[@]}" up -d --force-recreate gazebo swarm_lio2 racer_ros1 racer_controller

until [ "$(docker logs --since 15s swarm_lio2_ros2 2>&1 | grep -c 'ikd-tree size')" -gt 0 ]; do
  sleep 5
done
sleep 10
docker exec fishbot_gazebo bash -lc \
  'source /opt/ros/humble/setup.bash; ros2 daemon stop >/dev/null 2>&1; timeout 20 ros2 param set /bot1/twist_to_control_input auto_trot true'
docker exec racer_ros1 bash -lc 'printenv | sort' > "$shared_dir/last_racer_ros1_environment.txt"
docker inspect racer_ros1 --format '{{.Image}}' > "$shared_dir/last_racer_ros1_image.txt"
expected_image='sha256:c070b48a127a615497d3ba5e8c7bd6809daf7221e2b4877621cb08d9f82924b2'
actual_image=$(<"$shared_dir/last_racer_ros1_image.txt")
if [ "$actual_image" != "$expected_image" ]; then
  printf 'wrong ROS1 image: expected %s, got %s\n' "$expected_image" "$actual_image" >&2
  exit 1
fi
docker exec -d swarm_lio2_ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  source /opt/swarm_lio_ws/install/setup.bash
  mkdir -p '$run_dir'
  exec /usr/bin/python3 /racer_integration/ros2_exploration_recorder.py \
    --bots 1 --duration '$duration' --duration-basis sim --platform quadruped \
    --wait-for-tracking --progress-interval 30 --divergence-error 5.0 \
    --cloud-bounds -50 -25 0 50 25 1 --output-prefix '$run_dir/run' \
    --ros-args -p use_sim_time:=true > '$run_dir/recorder.log' 2>&1
"
printf 'Started frozen first-success baseline: %s sim s, output %s\n' "$duration" "$run_dir"
