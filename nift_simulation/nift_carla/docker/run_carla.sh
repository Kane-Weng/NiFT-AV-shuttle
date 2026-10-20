#!/usr/bin/env bash
# Run the CARLA server container (digital twin) (needs NVIDIA Container Toolkit).
#
# Environment overrides:
#   CARLA_IMAGE    image name              (default nift/carla-shuttle:0.9.15, the source build;
#                                           nift/carla-twin:0.9.15 = stock CARLA + twin, use the Micra)
#   CARLA_CONTAINER container name         (default nift-carla)
#   CARLA_GPUS     docker --gpus value     (default all; e.g. device=1)
#   CARLA_PORT     RPC port                (default 2000; also uses PORT+1, PORT+2)
#   CARLA_QUALITY  Epic | Low              (default Epic; use Low on small GPUs)
#   CARLA_RENDER   offscreen | window      (default offscreen; window needs an X display)
#   CARLA_RESX/Y   window / render size    (default 1280x720; e.g. 320x240 to save VRAM)
#   CARLA_EXTRA    extra CarlaUE4.sh flags
#   CARLA_DETACH   1 = run in the background (docker run -d, restart unless-stopped); stop with
#                  `docker stop nift-carla`, follow logs with `docker logs -f nift-carla`
set -euo pipefail

IMAGE="${CARLA_IMAGE:-nift/carla-shuttle:0.9.15}"
PORT="${CARLA_PORT:-2000}"
NAME="${CARLA_CONTAINER:-nift-carla}"
GPUS="${CARLA_GPUS:-all}"
QUALITY="${CARLA_QUALITY:-Epic}"
RENDER="${CARLA_RENDER:-offscreen}"
RESX="${CARLA_RESX:-1280}"
RESY="${CARLA_RESY:-720}"

FLAGS=(-nosound "-quality-level=${QUALITY}" "-carla-rpc-port=${PORT}" "-ResX=${RESX}" "-ResY=${RESY}")
if [[ "${CARLA_DETACH:-0}" == "1" ]]; then
  DOCKER_ARGS=(-d --restart unless-stopped --name "$NAME" --gpus "$GPUS" --net=host -e NVIDIA_DRIVER_CAPABILITIES=all)
else
  DOCKER_ARGS=(--rm -it --name "$NAME" --gpus "$GPUS" --net=host -e NVIDIA_DRIVER_CAPABILITIES=all)
fi

if [[ "$RENDER" == "window" ]]; then
  DOCKER_ARGS+=(-e "DISPLAY=${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
else
  FLAGS+=(-RenderOffScreen)
fi
if [[ -n "${CARLA_EXTRA:-}" ]]; then
  # shellcheck disable=SC2206
  FLAGS+=(${CARLA_EXTRA})
fi

# A stopped container left under the same name (a previous detached run) blocks `docker run`; drop it.
# A running one is reported instead of being replaced underneath whoever is using it.
state="$(docker inspect -f '{{.State.Status}}' "$NAME" 2>/dev/null || true)"
if [[ "$state" == "running" || "$state" == "restarting" ]]; then
  echo "$NAME is already running; stop it first with: docker stop $NAME" >&2
  exit 1
elif [[ -n "$state" ]]; then
  docker rm "$NAME" >/dev/null
fi

echo "docker run ${DOCKER_ARGS[*]} $IMAGE bash CarlaUE4.sh ${FLAGS[*]}"
exec docker run "${DOCKER_ARGS[@]}" "$IMAGE" bash CarlaUE4.sh "${FLAGS[@]}"
