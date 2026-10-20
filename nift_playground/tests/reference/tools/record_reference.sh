#!/usr/bin/env bash
# Record one reference run of the real ROS 2 stack (sim_kinematic profile) and extract it.
# Usage: record_reference.sh <workspace dir> <bag dir> <label> <route> [sim_route_run args ...]
#   <workspace dir>  a colcon workspace with src/ (git ls-files of the repo) built with --symlink-install
#   <bag dir>        where the raw run dirs (bags, launch.log) go; not in the repo
#   <label>          output folder name under tests/reference/runs/
# Each run gets its own container with --network none, so runs can go in parallel without DDS crosstalk
# and sim_route_run's "another stack is running" check only sees its own container.
set -euo pipefail
[ $# -ge 4 ] || { echo "usage: $0 <ws> <bagdir> <label> <route> [args...]"; exit 2; }
WS=$(realpath "$1"); BAGS=$(realpath "$2"); LABEL=$3; ROUTE=$4; shift 4
HERE=$(cd "$(dirname "$0")" && pwd)
REF=$(dirname "$HERE")
mkdir -p "$BAGS" "$REF/runs"
rm -rf "${BAGS:?}/$LABEL"
docker run --rm --network none \
  -v "$WS:/ws" -v "$BAGS:/runs" -v "$REF:/ref" \
  nift_pub_deps:humble bash -c "
    source /ws/install/setup.bash && cd /ws/src
    ros2 run nift_carla sim_route_run $ROUTE --out /runs/$LABEL $* > /runs/$LABEL.log 2>&1; code=\$?
    python3 /ref/tools/extract_reference.py /runs/$LABEL /ref/runs/$LABEL 2>&1 | grep -v rosbag2_storage
    gzip -n -c /runs/$LABEL.log > /ref/runs/$LABEL/route_run.log.gz
    gzip -n -c /runs/$LABEL/launch.log > /ref/runs/$LABEL/launch.log.gz
    cp /runs/$LABEL/run.json /ref/runs/$LABEL/run.json
    chown -R $(id -u):$(id -g) /runs /ref/runs
    echo \"$LABEL exit \$code\"
  "
