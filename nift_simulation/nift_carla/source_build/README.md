# Building the CARLA server from source with the NiFT shuttle

The stock CARLA package cannot take a new vehicle. To have `vehicle.nift.shuttle` (wheelbase 2.082 m, tracks
0.908 / 0.984 m, 15° lock, 500 kg, the real body) in the simulator, CARLA 0.9.15 is built from source with the
Unreal 4.26 editor, the shuttle is added as a vehicle, and the result is packaged into the server image
`nift/carla-shuttle:0.9.15`. Every step here runs from the command line; nothing is done by hand in the
Unreal editor. One machine does it all in about a day of mostly unattended work (times below are from a 20-thread
CPU with 62 GB RAM; a 4 GB GPU is enough to build and to run the checks slowly).

The output image is what `docker/run_carla.sh` starts by default (`nift/carla-shuttle:0.9.15`). Once built, the
image can be moved to other machines with `docker save` / `docker load` (17 GB), and the `ego_blueprint` default in
`config/carla_params.yaml` already points at the shuttle.

> **Note.** Steps 3 and 4 need the shuttle's coloured CAD model as an FBX. That model is not part of this
> repository, so these scripts show how the vehicle was built rather than something a visitor can rerun as is.
> Any other vehicle mesh goes through the same steps. Without the source build, use the stock image from
> `docker/build.sh` with `ego_blueprint:=vehicle.nissan.micra`.

## Requirements

- Linux x86_64, Docker with the NVIDIA Container Toolkit, an NVIDIA driver, X11 (only for looking at the editor).
- ~220 GB free: Unreal source + build 130 GB, CARLA + content 35 GB, derived-data cache 38 GB, the package 18 GB.
  Packaging needs ~55 GB free at that moment (cook, stage and archive each copy ~16–18 GB).
- A GitHub account linked to Epic Games (the Unreal fork `CarlaUnreal/UnrealEngine` is private until then) with SSH
  access set up: `git ls-remote git@github.com:CarlaUnreal/UnrealEngine.git carla` must answer.
- Blender 3.6 (portable tarball; 4.x/5.x files are not readable by it, so the colour step's FBX export is the input).
- Python 3 on the host for the checks; the CARLA 0.9.15 client wheel (`pip install carla==0.9.15`).

Everything builds inside `docker/Dockerfile.toolchain` (Ubuntu 20.04, clang-10, the Unreal runtime libraries),
bind-mounting `CARLA_BUILD_DIR` (default `~/carla_build`) at the same path inside and outside, so the editor can also be
opened from the container on the host's screen with the GPU.

## Steps

| step | script | what | time |
|---|---|---|---|
| 1 | `scripts/01_unreal.sh` | toolchain image, clone Unreal fork + CARLA 0.9.15 + Mcity twin (source version), `Setup.sh`, `make` | 1 h 20 min |
| 2 | `scripts/02_carla.sh` | CARLA content (from CARLA's mirror), `make PythonAPI`, `make CarlaUE4Editor`, McityMap source content, shader warm-up | 6 h (warm-up 5 h 50) |
| 3 | `scripts/03_rig_shuttle.py` | Blender: the coloured shuttle FBX → one 74k-triangle mesh bound to CARLA's vehicle skeleton, collision and raycast meshes, light mesh | 10 s |
| 4 | `scripts/04_build_vehicle.py`, `scripts/04_materials.py` | headless editor: import, blueprints from the Micra, physics asset, light mesh + light components, factory entry; materials (smoked glass, logo vinyl, lenses, CARLA's vehicle-lights material) | 2 × 4 min |
| 5 | `scripts/05_package.sh` | Shipping package with McityMap + Town02 cooked, McityMap_Main as the default map | 35 min |
| 6 | `../docker/build_source.sh` | the server image from the package | 10 min |

Before step 2, apply `patches/carla_setup_urls.patch` to `carla/Util/BuildTools/Setup.sh` (two download hosts in
CARLA 0.9.15 are gone: Boost and libpng) and copy `patches/carla_tools/NiftVehicleSetup.{h,cpp}` into
`carla/Unreal/CarlaUE4/Plugins/CarlaTools/Source/CarlaTools/{Public,Private}/`. That small C++ library does what Unreal
4.26's Python cannot: reshape the physics asset (convex body from the collision mesh, kinematic spheres for the wheels),
set the blueprint's raycast collision mesh, add components to the vehicle blueprint (the light mesh, light components
copied from a stock vehicle) and list a blueprint's components. After changing it, rebuild with `make CarlaUE4Editor`
(about a minute). Before step 5, apply the two `.ini` patches.

Step 3 needs CARLA's vehicle skeleton, `VehicleSkeleton.rar` from the "Add a new vehicle" tutorial, unpacked to
`$CARLA_BUILD_DIR/tools/skeleton/`, and the coloured shuttle FBX (made from the shuttle's CAD model, not public) next to the
script or via `--src`. Its outputs (`SK_nift_shuttle.fbx`, `SMC_nift_shuttle.fbx`, `SM_sc_nift_shuttle.fbx`,
`SM_nift_shuttle_lights.fbx`, `textures/`) go to `$CARLA_BUILD_DIR/tools/rig/` for step 4.

Step 4 runs in the headless editor:

```bash
B=${CARLA_BUILD_DIR:-$HOME/carla_build}
docker run --rm -e HOME=/home/builder -v "$B:$B" -w "$B/carla" nift/carla-build:20.04 \
  "$B/UnrealEngine_4.26/Engine/Binaries/Linux/UE4Editor-Cmd" "$B/carla/Unreal/CarlaUE4/CarlaUE4.uproject" \
  -run=pythonscript -script="$B/tools/04_build_vehicle.py" -NullRHI -unattended -nopause -nosplash -stdout -FullStdOutLogOutput
```

(the same for `04_materials.py`, after `04_build_vehicle.py`). The commandlet segfaults on exit after the script; the
assets are saved before, so that is harmless. Re-running is safe.

## Look and lights

- **Glass**: CARLA's car-glass materials only work on static meshes, and the shuttle's panes are part of its skeletal
  mesh, so `04_materials.py` makes its own translucent `M_Nift_Glass` (one-sided: every CAD pane is a closed ~6 mm slab,
  so one surface per wall faces the camera). Opacity 0.55 = the ~45 % see-through measured per pane on garage photos;
  the darker lower bands 0.75.
- **Logos**: one material per decal texture (UV channel 1) that blends from that glass to the logo by the texture's
  alpha; the far side's logos show mirrored through the glass, as on the real shuttle.
- **Lights** work like CARLA's stock vehicles: `vehicle.set_light_state(...)` (or Traffic Manager) switches them.
  1. The body's `Light_*` slots are the lenses switched off.
  2. `SM_NiftShuttle_Lights` is a separate mesh on a component tagged `emissive` with an instance of CARLA's
     `M_VehicleLights_Master`. That material picks a face's light function from where its UVs fall in an 8-cell atlas
     (`GenericMaterials/Vehicles/Lights/LightColorsCoord`). Step 3 copies each lens strip's outer face once per
     function (rear: position, brake; front: position, low beam, high beam; the lower corner strips also blink) a few
     mm outward and puts its UVs in that cell.
  3. Light components (spot / rect / point) copied from the Lincoln 2020 sit at the strips. `BaseVehiclePawn` groups
     them by name (`front|back-<function>-l|r-N`) and switches them with the light state; the blinkers only flash when
     blinker components exist.
- Rims keep the CAD's yellow (not visible on any photo). The CAD's own amber side strips sit under the side overhang,
  out of sight, which is why the lower corner strips blink as well.

## Checks

- Editor with the GPU on the screen: `UE4Editor CarlaUE4.uproject /Game/McityMap/McityMap_Main -vulkan` from the
  toolchain container with `--gpus all`, `-e DISPLAY`, the X11 socket mounted and the container user's uid equal to
  yours. The Play button does not react to synthetic clicks; for an API check start the game instead:
  `UE4Editor CarlaUE4.uproject /Game/Carla/Maps/Town02 -game -windowed -ResX=1600 -ResY=1000 -vulkan`, which is a
  CARLA server from the uncooked content.
- `checks/carla_vehicle_check.py --filter shuttle` against any running server: blueprint present, wheelbase, tracks,
  wheel radius, steer limit, mass, then a straight drive and (`--steer 1`) the turning radius from yaw rate.
  `checks/spectator_shot.py` parks the spectator on a spawned shuttle for screenshots.
- `checks/look_check.py --out DIR`: camera images of a parked shuttle by day and by night with each CARLA light state
  (position, low / high beam, brake, blinkers), to check the glass, the logos and the lights.
- With the stack: `carla_check_alignment --map <map generated from the 2025 OpenDRIVE>` (the source world is the twin's **2025**
  OpenDRIVE, with the north-east roundabout), the bridge's start-up lines ("Ego geometry … wheelbase 2.082 m, front
  track 0.908 m", steering map ≈ identity), and `sim_route_run <route> --backend carla --map-file <that map>`.

## Gotchas, in the order they bit

- Blender's armature object must be named exactly `Armature`, otherwise Unreal makes it a sixth root bone and the mesh
  no longer matches CARLA's skeleton (the import then silently answers "No" to merging skeletons).
- Unreal names an imported skeletal mesh `<name>_<fbx node>`; an in-place rename fails, so the script renames through
  a temporary package.
- A `Material` created from Python needs `used_with_skeletal_mesh = True`, or the game shows the default checkerboard
  on the vehicle ("missing bUsedWithSkeletalMesh" in the log).
- The shuttle's skeletal mesh shares the Micra's `Skeleton` asset (same five bones), so the Micra's animation
  blueprint works unchanged after duplication.
- `-run=pythonscript` with `-NullRHI` is enough for imports, blueprint edits and material compiles; no display needed.
- The derived-data cache fill (`-run=DerivedDataCache -fill`) exits 1 because three stock CARLA blueprints
  (procedural water, Houdini pipeline, multiple lights) do not compile; the cache is complete anyway.
- Texture streaming kept the logo textures at a low mip on the vehicle (blurred text): they are `never_stream`.
- In `-game` mode from uncooked content, a material created or changed since the last run shows Unreal's grey checker
  until its shaders finish compiling (watch "shaders left to compile" in the log); the cooked package has them.
- Python cannot add components to a blueprint in 4.26 (no subobject API), hence `AddBlueprintStaticMeshComponent` and
  `CopyBlueprintComponent` in the helper.
- `make package` fails if the disk fills during the final archive copy; the staged build in
  `Unreal/CarlaUE4/Saved/StagedBuilds/LinuxNoEditor` is already the complete package and can be moved into `Dist/`.
