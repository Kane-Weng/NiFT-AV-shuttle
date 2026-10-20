#!/usr/bin/env bash
# Step 2: CARLA 0.9.15 content + PythonAPI + CarlaUE4Editor inside the build container, McityMap source content
# copied in, then a headless derived-data (shader) warm-up. Apply patches/carla_setup_urls.patch first (dead
# download links in Util/BuildTools/Setup.sh) and drop patches/carla_tools/NiftVehicleSetup.* into
# Unreal/CarlaUE4/Plugins/CarlaTools/Source/CarlaTools/{Public,Private}/ before `make CarlaUE4Editor`.
# Idempotent. Logs: $L/phase2_*.log.    setsid nohup bash 02_carla.sh >> phase2.log 2>&1 &
set -uo pipefail
B="${CARLA_BUILD_DIR:-$HOME/carla_build}"
L="${CARLA_BUILD_LOGS:-$B/logs}"
IMG=nift/carla-build:20.04
mkdir -p "$L"
log(){ echo "=== $(date -Is) $*"; }
mins(){ echo $(( ($(date +%s) - $1) / 60 )); }
T0=$(date +%s)
log "phase 2 start; free $(df -h --output=avail / | tail -1 | tr -d ' ')"

# McityMap source content (host side, plain copy)
if [ ! -d "$B/carla/Unreal/CarlaUE4/Content/McityMap" ]; then
  mkdir -p "$B/carla/Unreal/CarlaUE4/Content" && cp -r "$B/mcity-digital-twin/CARLA/source_version/McityMap" "$B/carla/Unreal/CarlaUE4/Content/McityMap" && log "McityMap copied ($(du -sh "$B/carla/Unreal/CarlaUE4/Content/McityMap" | cut -f1))"
fi

# CARLA 0.9.15 content: Update.sh points at carla-assets.s3.amazonaws.com, which answers 403 (2026-10-13); the same
# tarball is on CARLA's Backblaze mirror. Done host-side, Update.sh is then skipped (.version present).
CONTENT="$B/carla/Unreal/CarlaUE4/Content/Carla"; CID=$(tac "$B/carla/Util/ContentVersions.txt" | egrep -m 1 . | rev | cut -d' ' -f1 | rev)
if [ ! -f "$CONTENT/.version" ]; then
  T1=$(date +%s); mkdir -p "$B/dl"
  curl -sS -L -C - -o "$B/dl/Content_$CID.tar.gz" "https://carla-assets.s3.us-east-005.backblazeb2.com/$CID.tar.gz" || { log "CONTENT DOWNLOAD FAILED"; exit 1; }
  log "content downloaded ($(du -sh "$B/dl/Content_$CID.tar.gz" | cut -f1), $(mins $T1) min)"
  rm -rf "$B/dl/Content" && mkdir -p "$B/dl/Content" && tar -xzf "$B/dl/Content_$CID.tar.gz" -C "$B/dl/Content" || { log "CONTENT EXTRACT FAILED"; exit 1; }
  mkdir -p "$CONTENT" && cp -a "$B/dl/Content"/* "$CONTENT"/ && echo "$CID" > "$CONTENT/.version" && rm -rf "$B/dl/Content"
  log "content installed ($(du -sh "$CONTENT" | cut -f1), $(ls "$CONTENT" | tr '\n' ' '))"
fi

docker run --rm --name carla-build-p2 -e HOME=/home/builder -e UE4_ROOT="$B/UnrealEngine_4.26" -v "$B:$B" -v "$L:$L" -w "$B/carla" $IMG bash -c '
  set -u
  step(){ echo "##### $1 $(date -Is)"; }
  if [ ! -f Unreal/CarlaUE4/Content/Carla/.version ]; then
    step "Update.sh (content)"; ./Update.sh || exit 21
  fi
  step "make PythonAPI";      make PythonAPI      || exit 22
  step "make CarlaUE4Editor"; make CarlaUE4Editor || exit 23
  step "DerivedDataCache fill (headless)"
  "$UE4_ROOT/Engine/Binaries/Linux/UE4Editor-Cmd" "$PWD/Unreal/CarlaUE4/CarlaUE4.uproject" -run=DerivedDataCache -fill \
      -NullRHI -unattended -nopause -nosplash -stdout -FullStdOutLogOutput > "'"$L"'/phase2_ddc_fill.log" 2>&1 || exit 24
  step "done"' > "$L/phase2_build.log" 2>&1
R=$?
if [ $R -ne 0 ]; then log "PHASE 2 FAILED (exit $R: 21=Update 22=PythonAPI 23=CarlaUE4Editor 24=DDC) after $(mins $T0) min"; exit 1; fi
log "phase 2 done in $(mins $T0) min; carla tree $(du -sh "$B/carla" | cut -f1), free $(df -h --output=avail / | tail -1 | tr -d ' ')"
ls "$B/carla/PythonAPI/carla/dist/" 2>/dev/null
