# nift_bringup

This package starts the whole NiFT shuttle stack, on the real vehicle or in simulation, from one launch file. A **profile** names the parts to start. **Preflight checks** test each part before anything moves. A small desktop **launcher** does the same from a window. Back to the [main README](../README.md). The wiki pages [nift_bringup](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_bringup) and [launcher](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/launcher) explain the design in depth.

## Quick start

```bash
ros2 launch nift_bringup system.launch.py                                   # list the profiles
ros2 launch nift_bringup system.launch.py profile:=sim_kinematic           # full stack, no CARLA or GPU needed
ros2 launch nift_bringup system.launch.py profile:=sim                     # full stack in CARLA
ros2 launch nift_bringup system.launch.py profile:=shuttle map_file:=<your map>.osm
ros2 launch nift_bringup system.launch.py profile:=shuttle perception:=false rviz:=false
ros2 launch nift_bringup system.launch.py profile:=none gnss:=true         # one component alone
ros2 launch nift_bringup system.launch.py profile:=sim dry_run:=true       # print the plan and the checks, start nothing
ros2 run nift_bringup launcher                                              # the same, from a window
```

## How it fits

```
config/profiles.yaml ──► profiles.py ──► preflight.py ──► components/*.py ──► the nodes
   (named sets)          (rules, plan)     (checks)          (what to start)
```

- **Components.** The stack is split into twelve switchable parts: `can`, `gnss`, `localization`, `filter`, `perception`, `planning`, `static_path`, `longitudinal`, `lateral`, `bag`, `rviz` and `web`. Each one is a module in [`nift_bringup/components/`](./nift_bringup/components/) that says what it starts and how it is checked. Any of them can be switched per run, e.g. `perception:=false`.
- **Profiles.** [`config/profiles.yaml`](./config/profiles.yaml) holds named sets of components plus a mode, `real` or `sim`. `shuttle` is full autonomy on the vehicle, `sim` is the same in CARLA, and `sim_kinematic` runs without CARLA or a GPU.
- **Rules.** [`profiles.py`](./nift_bringup/profiles.py) checks the rules between components before anything starts. For example `planning` needs a pose source, `longitudinal` needs `can` and `gnss`, and `lateral` needs a path. A broken rule stops the launch with the reason. With `perception` off, the controller runs with no safety filter, and the plan prints a warning.
- **Preflight.** [`preflight.py`](./nift_bringup/preflight.py) runs every enabled component's check in parallel in a few seconds. It checks the CAN interface (exists, up at 500 kbit/s, not bus-off), the GNSS server (reachable, authenticated, both antennas reporting), the LiDAR topic, the Lanelet2 map, the CARLA server, disk space for the bag, a display for RViz and the rosbridge port. It also checks that nothing else already publishes a component's output topics, such as a second launch still writing `/to_can_bus`. Any FAIL stops the launch. `checks:=false` skips the checks.
- **Real or sim.** `mode: sim` includes the simulation backend from `nift_carla` instead of the CAN bridge, and points `beacon_node` at the backend's GNSS server emulator. The backend is CARLA by default, or the kinematic model with `sim_backend:=kinematic`.
- **Checkpoint.** `waypoint_node` and `trajectory_controller` start only after `beacon_node` reports that the GNSS server accepted its key, so the vehicle cannot move before localization is available.

## Maps

`map_file:=<name>` picks one of the maps that `nift_waypoint` installs from the repo's [`maps/`](../maps/), and `map_file:=<path>` takes any Lanelet2 file. The default, on the shuttle and in sim, is `lanelet2_odr2024.osm`, which we generated from the public digital twin's 2024 OpenDRIVE file.

## Settings

The launch switches components on and off. Each node's own limits live in its config file, e.g. [`beacon_node.yaml`](../nift_sensor/config/beacon_node.yaml), [`localization.yaml`](../nift_sensor/config/localization.yaml), [`planning.yaml`](../nift_control/config/planning.yaml) and [`vehicle.yaml`](../nift_common/nift_common/vehicle.yaml). `ros2 launch nift_bringup system.launch.py --show-args` lists every launch argument.

`shuttle_config.py` only re-exports `ShuttleConfig` and `VehicleState` from [nift_common](../nift_common/), so older imports keep working.

## The launcher

`ros2 run nift_bringup launcher [--profile <name>] [--dry-run]` opens a window with the components as stations on a transit-style map. A click switches a station on or off, a double-click switches it on with everything it needs, and a gray station says what it is missing. The side panel picks a preset and real or sim, takes launch arguments and shows the exact `ros2 launch` command. **Check** runs the preflight and marks each station. **Start** closes the window and runs the command in the same terminal, so Ctrl-C works as usual.

## Tests

```bash
colcon test --packages-select nift_bringup && colcon test-result --verbose
```

The tests cover the profile rules and every shipped profile, the launcher's logic without Qt, and the preflight checks that need no hardware (CAN link parsing, the outputs check, and the GNSS check against `nift_carla`'s server emulator). flake8 and pep257 run too.
