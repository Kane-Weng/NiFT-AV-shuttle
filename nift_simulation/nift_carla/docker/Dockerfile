# CARLA 0.9.15 server with the digital twin content (Mcity Digital Twin, MIT) baked in.
#
# Build context must be the *packaged Linux content* of the digital twin
# (CARLA/packaged_version/linux inside https://github.com/mcity/mcity-digital-twin),
# pulled with Git LFS.  Use docker/build.sh which checks that for you:
#
#   export DIGITAL_TWIN_DIR=<your clone of the twin>
#   bash nift_simulation/nift_carla/docker/build.sh
#
# Install steps follow the digital twin README ("Packaged Version of CARLA"):
#   1. copy the content to CarlaUE4/Content/McityMap
#   2. append the McityMap cook/stage lines to CarlaUE4/Config/DefaultGame.ini
ARG BASE_IMAGE=carlasim/carla:0.9.15
FROM ${BASE_IMAGE}

USER root
COPY --chown=carla:carla . /home/carla/CarlaUE4/Content/McityMap/
RUN printf '\n+MapsToCook=(FilePath="/Game/Carla/Maps/McityMap")\n+DirectoriesToAlwaysStageAsUFS=(Path="McityMap/OpenDrive")\n+DirectoriesToAlwaysStageAsUFS=(Path="McityMap/Nav")\n' \
      >> /home/carla/CarlaUE4/Config/DefaultGame.ini \
 && chown carla:carla /home/carla/CarlaUE4/Config/DefaultGame.ini \
 && test -f /home/carla/CarlaUE4/Content/McityMap/OpenDrive/McityMap_Main.xodr

USER carla
WORKDIR /home/carla
# Default: headless server on the standard RPC port. Override flags via run_carla.sh.
CMD ["bash", "CarlaUE4.sh", "-RenderOffScreen", "-nosound", "-carla-rpc-port=2000"]
