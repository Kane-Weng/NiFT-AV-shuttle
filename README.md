# NiFT Autonomous Shuttle — ROS 2 autonomy stack

[![CI](https://github.com/Kane-Weng/NiFT-AV-shuttle/actions/workflows/ci.yml/badge.svg)](https://github.com/Kane-Weng/NiFT-AV-shuttle/actions/workflows/ci.yml) [![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

The software stack for the **NiFT autonomous shuttle**, a low-speed self-driving shuttle developed and tested at [Mcity](https://mcity.umich.edu/), the University of Michigan's autonomous-vehicle proving ground. It takes the vehicle from raw sensor data to drive-by-wire CAN commands:

- GNSS RTK localization and LiDAR obstacle detection
- Lanelet2 HD-map route planning with smooth path generation
- Pure Pursuit steering and ADRC speed control, behind a layer of safety filters
- A CARLA simulation that drives the stack through the **same CAN interface as the real vehicle**
- A live web dashboard for monitoring and operator control

> **Status:** this repository is being published package by package. Packages are listed below as they land. The [wiki](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki) explains the engineering behind.

<!-- Demo GIF / video goes here (ledger unit U10). -->

## Architecture

```
┌──────────────────────────── SENSING & LOCALIZATION ────────────────────────────┐
│  GNSS RTK beacons → position + heading        LiDAR point cloud → obstacles    │
│  TF broadcaster: map → base_link                               [nift_sensor]   │
└───────────────────────────────────────┬────────────────────────────────────────┘
                                        │ /gnss/fix  /gnss/heading_deg  /gnss/vel  /obstacles
                                        ▼
┌───────────────────────────────── PATH PLANNING ────────────────────────────────┐
│  Lanelet2 HD map → route → smooth waypoint path              [nift_waypoint]   │
└───────────────────────────────────────┬────────────────────────────────────────┘
                                        │ /planned_path
                                        ▼
┌────────────────────────────── TRAJECTORY CONTROL ──────────────────────────────┐
│  Pure Pursuit (lateral) · ADRC (longitudinal) · safety filters · mode FSM      │
│  encodes gear / brake / pedal / steering into CAN frames      [nift_control]   │
└───────────────────────────────────────┬────────────────────────────────────────┘
                                        │ /to_can_bus  (command and E-stop frames)
                                        ▼
┌─────────────────────────────────── ACTUATION ──────────────────────────────────┐
│  Real shuttle:  SocketCAN → drive-by-wire ECU                                  │
│  Simulation:    CARLA + Mcity Digital Twin; a virtual ECU decodes the same     │
│                 CAN frames                                       [nift_carla]  │
└────────────────────────────────────────────────────────────────────────────────┘

  Web dashboard (nift_web + nift_web_frontend): live map, telemetry and operator
  controls, connected to the ROS graph through rosbridge
```

The simulator replaces only what the hardware provides (GNSS beacons, the LiDAR cloud and the drive-by-wire ECU), so the code that drives the real shuttle is the code tested in simulation.

## Packages

| Package | Role | Status |
|---|---|---|
| `nift_bringup` | Vehicle constants (single source of truth), system launch | coming soon |
| `nift_sensor` | GNSS localization, LiDAR obstacle detection, TF broadcasting | coming soon |
| `nift_waypoint` | Lanelet2 routing, smooth path generation | coming soon |
| `nift_control` | Pure Pursuit + ADRC control, safety filters, CAN encoding | coming soon |
| `nift_simulation` | Vehicle model (URDF) and the CARLA bridge with its virtual ECU | coming soon |
| `nift_web`, `nift_web_frontend` | rosbridge server and React dashboard | coming soon |

## Vehicle

| Parameter | Value |
|---|---|
| Length × width × height | 3.055 m × 1.588 m × 1.905 m |
| Wheelbase | 2.082 m |
| Wheel radius | 0.228 m |
| Max steering angle | 15° (0.2618 rad) |
| Max speed | 4.92 m/s (~17.7 km/h) |
| Max acceleration / deceleration | 0.8 / 7.0 m/s² |

## Getting started

Requires Ubuntu 22.04 and ROS 2 Humble. The repository is a colcon workspace's `src/` directory:

```bash
mkdir -p ~/nift_ws && cd ~/nift_ws
git clone https://github.com/Kane-Weng/NiFT-AV-shuttle.git src
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

<!-- Launch instructions arrive with the bringup unit (U9). -->

## Documentation

The [project wiki](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki) covers the hardware (CAN, LiDAR, GNSS), the middleware (ROS 2, Linux) and the software stack (localization, planning, control, perception, simulation).

## Author

**Kane Weng** — designed and wrote the autonomy software stack.

## Acknowledgements

- Chun Ho (Jimmy) Wang co-wrote parts of the route planner (`nift_waypoint`) and the Gazebo simulation.
- The NiFT shuttle team at the University of Michigan.
- Mcity, for the test facility and the public [Mcity Digital Twin](https://github.com/mcity/mcity-digital-twin).

## License

[Apache-2.0](LICENSE)
