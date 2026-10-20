#!/usr/bin/env bash
# Build nift/carla-shuttle:0.9.15 from the source-built package (step 5 of source_build/README.md).
#   PACKAGE_DIR=.../Dist/CARLA_Shipping_<tag>/LinuxNoEditor bash build_source.sh
# The package (16 GB) is copied inside a container and committed, so Docker never has to ingest it as a build context.
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${CARLA_IMAGE:-nift/carla-shuttle:0.9.15}"
BASE="${CARLA_BASE_IMAGE:-nift/carla-runtime-base:20.04}"
PKG="${PACKAGE_DIR:?set PACKAGE_DIR to the LinuxNoEditor folder of the package}"
test -f "$PKG/CarlaUE4.sh" || { echo "no CarlaUE4.sh in $PKG" >&2; exit 1; }
test -f "$PKG/CarlaUE4/Content/McityMap/OpenDrive/McityMap_Main.xodr" || { echo "McityMap OpenDrive missing in $PKG" >&2; exit 1; }
echo "base image $BASE"; docker build -q -t "$BASE" -f "$DIR/Dockerfile.runtime" "$DIR"
docker rm -f carla-img-build >/dev/null 2>&1 || true
echo "copying $(du -sh "$PKG" | cut -f1) package into a container (debug symbols left out)"
docker run --name carla-img-build --user root -v "$PKG:/src:ro" "$BASE" bash -c \
  'cd /src && tar -c --exclude="*.debug" --exclude="*.sym" . | tar -x -C /home/carla && chown -R carla:carla /home/carla && test -f /home/carla/CarlaUE4.sh'
docker commit --change 'USER carla' --change 'WORKDIR /home/carla' \
  --change 'CMD ["bash", "CarlaUE4.sh", "-RenderOffScreen", "-nosound", "-carla-rpc-port=2000"]' \
  -m "source-built CARLA 0.9.15 + digital twin map (2025) + vehicle.nift.shuttle" carla-img-build "$IMAGE"
docker rm carla-img-build >/dev/null
docker image inspect "$IMAGE" --format "built $IMAGE: {{.Size}} bytes"
