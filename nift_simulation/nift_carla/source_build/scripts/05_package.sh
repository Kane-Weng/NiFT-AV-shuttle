#!/usr/bin/env bash
# Step 5: package the CARLA server (Shipping, LinuxNoEditor). Apply patches/cook_twin_only.DefaultGame.ini.patch and
# patches/default_map_twin.DefaultEngine.ini.patch first (McityMap + Town02 cooked, McityMap_Main the default map).
# Needs ~55 GB free (cook, stage and archive each ~16-18 GB). Output: carla/Dist/CARLA_Shipping_<tag>/LinuxNoEditor/.
#   setsid nohup bash 05_package.sh >> phase5.log 2>&1 &
set -uo pipefail
B="${CARLA_BUILD_DIR:-$HOME/carla_build}"; L="${CARLA_BUILD_LOGS:-$B/logs}"; IMG=nift/carla-build:20.04
log(){ echo "=== $(date -Is) $*"; }; mins(){ echo $(( ($(date +%s) - $1) / 60 )); }
T0=$(date +%s); log "phase 5 start; free $(df -h --output=avail / | tail -1 | tr -d ' ')"
docker run --rm --name carla-build-p5 -e HOME=/home/builder -e UE4_ROOT="$B/UnrealEngine_4.26" -v "$B:$B" -v "$L:$L" -w "$B/carla" $IMG \
  bash -c 'echo "##### make package $(date -Is)"; make package ARGS="--no-zip"; echo "##### exit $? $(date -Is)"' > "$L/phase5_package.log" 2>&1
R=$?
log "make package exit $R after $(mins $T0) min; free $(df -h --output=avail / | tail -1 | tr -d ' ')"
ls -d "$B/carla/Dist"/*/LinuxNoEditor 2>/dev/null && du -sh "$B/carla/Dist"/*/LinuxNoEditor
P=$(ls -d "$B/carla/Dist"/*/LinuxNoEditor 2>/dev/null | head -1)
if [ -n "$P" ]; then
  log "cooked maps:"; find "$P/CarlaUE4/Content/Paks" -name '*.pak' -exec ls -la {} \; 2>/dev/null | awk '{print $5, $9}'
  ls "$P/CarlaUE4/Content/McityMap/OpenDrive" "$P/CarlaUE4/Content/McityMap/Nav" 2>&1 | head
fi
log "phase 5 done"
