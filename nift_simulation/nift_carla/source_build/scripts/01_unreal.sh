#!/usr/bin/env bash
# Step 1: toolchain image, clones (Unreal fork over SSH: your GitHub account must be linked to Epic Games; CARLA
# 0.9.15; the Mcity twin, source_version only), then Setup.sh + GenerateProjectFiles.sh + make for Unreal in the
# container. Idempotent: re-running skips finished steps. Logs: $L/phase1_*.log. Run detached:
#   setsid nohup bash 01_unreal.sh > phase1.log 2>&1 &
# CARLA_BUILD_DIR (default ~/carla_build) is bind-mounted at the same path inside the container.
set -uo pipefail
B="${CARLA_BUILD_DIR:-$HOME/carla_build}"
L="${CARLA_BUILD_LOGS:-$B/logs}"
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
IMG=nift/carla-build:20.04
mkdir -p "$B" "$L"
log(){ echo "=== $(date -Is) $*"; }
mins(){ echo $(( ($(date +%s) - $1) / 60 )); }
log "phase 1 start on $(hostname); free $(df -h --output=avail / | tail -1 | tr -d ' ')"

# ---- 1. toolchain image + downloads, in parallel -------------------------------------------------------------
T0=$(date +%s)
( if ! docker image inspect $IMG >/dev/null 2>&1; then
    log "image build"; docker build -t $IMG --build-arg UID=$(id -u) --build-arg GID=$(id -g) "$DIR/../docker" -f "$DIR/../docker/Dockerfile.toolchain" \
      > "$L/phase1_image.log" 2>&1 && log "image done ($(mins $T0) min)" || { log "IMAGE FAILED"; exit 1; }
  fi ) & P_IMG=$!
( if [ ! -d "$B/UnrealEngine_4.26/.git" ]; then
    log "clone UE fork"; git clone --depth 1 -b carla git@github.com:CarlaUnreal/UnrealEngine.git "$B/UnrealEngine_4.26" \
      > "$L/phase1_clone_ue.log" 2>&1 && log "clone UE done ($(mins $T0) min, $(du -sh "$B/UnrealEngine_4.26" | cut -f1))" \
      || { log "CLONE UE FAILED"; exit 1; }
  fi ) & P_UE=$!
( if [ ! -d "$B/carla/.git" ]; then
    log "clone carla 0.9.15"; git clone --depth 1 -b 0.9.15 https://github.com/carla-simulator/carla.git "$B/carla" \
      > "$L/phase1_clone_carla.log" 2>&1 && log "clone carla done ($(mins $T0) min)" || { log "CLONE CARLA FAILED"; exit 1; }
  fi ) & P_CARLA=$!
( if [ ! -d "$B/mcity-digital-twin/.git" ]; then
    log "clone twin (LFS: source_version only)"
    GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 https://github.com/mcity/mcity-digital-twin.git "$B/mcity-digital-twin" \
      > "$L/phase1_clone_twin.log" 2>&1 \
    && (cd "$B/mcity-digital-twin" && git lfs pull -I "CARLA/source_version/**" >> "$L/phase1_clone_twin.log" 2>&1) \
    && log "clone twin done ($(mins $T0) min, $(du -sh "$B/mcity-digital-twin" | cut -f1))" || { log "CLONE TWIN FAILED"; exit 1; }
  fi ) & P_TWIN=$!
wait $P_IMG;   R_IMG=$?
wait $P_UE;    R_UE=$?
wait $P_CARLA; R_CARLA=$?
wait $P_TWIN;  R_TWIN=$?
log "downloads: image=$R_IMG ue=$R_UE carla=$R_CARLA twin=$R_TWIN; tree $(du -sh "$B" | cut -f1), free $(df -h --output=avail / | tail -1 | tr -d ' ')"
if [ $R_IMG -ne 0 ] || [ $R_UE -ne 0 ]; then log "prerequisites failed; stop"; exit 1; fi

# ---- 2. Unreal: Setup.sh + GenerateProjectFiles.sh + make, one container (Setup.sh may apt-install) ----------
UE="$B/UnrealEngine_4.26"
if [ ! -x "$UE/Engine/Binaries/Linux/UE4Editor" ]; then
  T1=$(date +%s); log "UE build (Setup.sh, GenerateProjectFiles.sh, make) → phase1_ue_build.log"
  docker run --rm --name carla-build-ue -e HOME=/home/builder -v "$B:$B" -w "$UE" $IMG bash -c '
      echo "##### Setup.sh $(date -Is)";               yes 2>/dev/null | ./Setup.sh; [ ${PIPESTATUS[1]} -eq 0 ] || exit 11
      echo "##### GenerateProjectFiles.sh $(date -Is)"; ./GenerateProjectFiles.sh      || exit 12
      echo "##### make $(date -Is)";                    make                           || exit 13
      echo "##### done $(date -Is)"' > "$L/phase1_ue_build.log" 2>&1
  R=$?
  if [ $R -ne 0 ]; then log "UE BUILD FAILED (exit $R: 11=Setup 12=GenProj 13=make) after $(mins $T1) min"; exit 1; fi
  log "UE build done in $(mins $T1) min"
fi
log "phase 1 done in $(mins $T0) min; UE tree $(du -sh "$UE" | cut -f1), free $(df -h --output=avail / | tail -1 | tr -d ' ')"
ls -la "$UE/Engine/Binaries/Linux/UE4Editor"
