#!/usr/bin/env bash
# Build the nift/carla-twin image: carlasim/carla:0.9.15 + the Mcity Digital Twin content
# (https://github.com/mcity/mcity-digital-twin, MIT; it is never copied into this repository).
#
# Prerequisites (on the GPU host):
#   export DIGITAL_TWIN_DIR=<a folder outside this repo>
#   GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/mcity/mcity-digital-twin.git "$DIGITAL_TWIN_DIR"
#   (cd "$DIGITAL_TWIN_DIR" && git lfs pull --include="CARLA/packaged_version/linux/**")
set -euo pipefail

IMAGE="${CARLA_IMAGE:-nift/carla-twin:0.9.15}"
BASE="${CARLA_BASE_IMAGE:-carlasim/carla:0.9.15}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -z "${DIGITAL_TWIN_DIR:-}" ]]; then
  echo "ERROR: set DIGITAL_TWIN_DIR to your clone of https://github.com/mcity/mcity-digital-twin" >&2
  exit 1
fi
CONTENT="$DIGITAL_TWIN_DIR/CARLA/packaged_version/linux"
XODR="$CONTENT/OpenDrive/McityMap_Main.xodr"
UMAP="$CONTENT/McityMap_Main.umap"
if [[ ! -f "$XODR" ]]; then
  echo "ERROR: $XODR not found — is DIGITAL_TWIN_DIR right?" >&2
  exit 1
fi
# LFS pointer files are ~130 bytes; the real .umap is megabytes
if [[ ! -f "$UMAP" || $(stat -c %s "$UMAP") -lt 10000 ]]; then
  echo "ERROR: $UMAP is missing or still a Git LFS pointer. Run:" >&2
  echo "  (cd \"$DIGITAL_TWIN_DIR\" && git lfs pull --include=\"CARLA/packaged_version/linux/**\")" >&2
  exit 1
fi

echo "Content: $CONTENT ($(du -sh "$CONTENT" | cut -f1))"
echo "Building $IMAGE from $BASE ..."
docker build --build-arg "BASE_IMAGE=$BASE" -f "$DIR/Dockerfile" -t "$IMAGE" "$CONTENT"
echo "Done. Start the server with: bash $DIR/run_carla.sh"
