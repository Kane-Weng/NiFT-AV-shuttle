#!/usr/bin/env bash
# record_reference.sh plus the gap probe (ros_hook/sitecustomize.py) in kinematic_sim_node.
# Usage: record_probe.sh <workspace dir> <bag dir> <label> <route> [sim_route_run args ...]
# Writes the extracted run to <bag dir>/extracted/<label> (move it to tests/reference/runs/ by hand) and the
# probe log to <bag dir>/<label>.probe.jsonl.
set -euo pipefail
[ $# -ge 4 ] || { echo "usage: $0 <ws> <bagdir> <label> <route> [args...]"; exit 2; }
WS=$(realpath "$1"); BAGS=$(realpath "$2"); LABEL=$3; ROUTE=$4; shift 4
HERE=$(cd "$(dirname "$0")" && pwd)
REF=$(cd "$HERE/../../reference" && pwd)
mkdir -p "$BAGS/extracted"
rm -rf "${BAGS:?}/$LABEL" "$BAGS/extracted/$LABEL" "$BAGS/$LABEL.probe.jsonl"
docker run --rm --network none \
  -v "$WS:/ws" -v "$BAGS:/runs" -v "$REF/tools:/reftools:ro" -v "$HERE/ros_hook:/hook:ro" \
  nift_pub_deps:humble bash -c "
    source /ws/install/setup.bash && cd /ws/src
    export PYTHONPATH=/hook:\$PYTHONPATH GNSS_PROBE=/runs/$LABEL.probe.jsonl
    ros2 run nift_carla sim_route_run $ROUTE --out /runs/$LABEL $* > /runs/$LABEL.log 2>&1; code=\$?
    unset GNSS_PROBE
    python3 /reftools/extract_reference.py /runs/$LABEL /runs/extracted/$LABEL 2>&1 | grep -v rosbag2_storage
    gzip -n -c /runs/$LABEL.log > /runs/extracted/$LABEL/route_run.log.gz
    gzip -n -c /runs/$LABEL/launch.log > /runs/extracted/$LABEL/launch.log.gz
    cp /runs/$LABEL/run.json /runs/extracted/$LABEL/run.json
    chown -R $(id -u):$(id -g) /runs
    echo \"$LABEL exit \$code\"
  "
