#!/usr/bin/env bash
# Fallback for hosts WITHOUT the NVIDIA Container Toolkit (no `--gpus` support, no sudo to install it).
# Bind-mounts the host's NVIDIA driver device nodes, user-space libraries and Vulkan ICD into the
# container instead. Same env knobs as run_carla.sh
# (CARLA_IMAGE/CONTAINER/PORT/QUALITY/RENDER/RESX/RESY/EXTRA/DETACH).
# Requires: /dev/nvidia* present and the host driver libraries in NVIDIA_LIBDIR
# (default /usr/lib/x86_64-linux-gnu, the Ubuntu layout).
set -euo pipefail

IMAGE="${CARLA_IMAGE:-nift/carla-twin:0.9.15}"
PORT="${CARLA_PORT:-2000}"
NAME="${CARLA_CONTAINER:-nift-carla}"
QUALITY="${CARLA_QUALITY:-Epic}"
RENDER="${CARLA_RENDER:-offscreen}"
RESX="${CARLA_RESX:-1280}"
RESY="${CARLA_RESY:-720}"
LIBDIR="${NVIDIA_LIBDIR:-/usr/lib/x86_64-linux-gnu}"

FLAGS=(-nosound "-quality-level=${QUALITY}" "-carla-rpc-port=${PORT}" "-ResX=${RESX}" "-ResY=${RESY}")
if [[ "${CARLA_DETACH:-0}" == "1" ]]; then
  DOCKER_ARGS=(-d --restart unless-stopped --name "$NAME" --net=host)
else
  DOCKER_ARGS=(--rm -it --name "$NAME" --net=host)
fi

# device nodes
for d in /dev/nvidia*; do DOCKER_ARGS+=(--device "$d:$d"); done
# driver user-space libraries (real files + their soname symlinks); docker resolves host symlinks
while IFS= read -r f; do
  DOCKER_ARGS+=(-v "$LIBDIR/$f:$LIBDIR/$f:ro")
done < <(ls "$LIBDIR" | grep -E '^(libnvidia-|libcuda\.|libcudadebugger|libGLX_nvidia|libEGL_nvidia|libGLESv[12]_[a-zA-Z]*nvidia)')
# Vulkan / EGL vendor ICDs
DOCKER_ARGS+=(-v /usr/share/vulkan/icd.d/nvidia_icd.json:/etc/vulkan/icd.d/nvidia_icd.json:ro)
[[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]] && \
  DOCKER_ARGS+=(-v /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json:ro)
DOCKER_ARGS+=(-e VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json -e NVIDIA_DRIVER_CAPABILITIES=all)

if [[ "$RENDER" == "window" ]]; then
  DOCKER_ARGS+=(-e "DISPLAY=${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
else
  FLAGS+=(-RenderOffScreen)
fi
if [[ -n "${CARLA_EXTRA:-}" ]]; then
  # shellcheck disable=SC2206
  FLAGS+=(${CARLA_EXTRA})
fi

echo "docker run (host-driver mounts) $IMAGE bash CarlaUE4.sh ${FLAGS[*]}"
exec docker run "${DOCKER_ARGS[@]}" "$IMAGE" bash CarlaUE4.sh "${FLAGS[@]}"
