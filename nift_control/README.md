# nift_control

`nift_control` is the control layer of the NiFT autonomous shuttle. It takes the planned path, the shuttle's pose and speed and the obstacles in front of it, and turns them into a steering angle, a gear, a brake level and a pedal level. It packs those into the shuttle's drive-by-wire CAN frames and publishes them on `/to_can_bus`. On the shuttle, ros2_socketcan writes them onto the CAN bus. In simulation, the CARLA bridge's virtual ECU decodes the same frames, so the code that runs in simulation is the code that drives the shuttle.

The wiki explains each part in depth: [nift_control](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_control), [trajectory_controller](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/trajectory_controller), [dbw_interface](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/dbw_interface) and [longitudinal_test](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/longitudinal_test).

## How it fits

```
waypoint_node ──/planned_path──┐
shuttle_tf_node ──TF map→base_link──┤
beacon_node ──/gnss/vel──────────────┼──> trajectory_controller ──/to_can_bus──> CAN bus (or CARLA virtual ECU)
lidar_node ──/obstacles─────────┘
```

`trajectory_controller` runs two timer loops.

1. **Path loop, 10 Hz.** Pure Pursuit picks a point on the path a few metres ahead and steers toward it (`lateral_controller: stanley` selects the Stanley controller instead). The target speed drops near the goal, with cross-track error and with steering. A safety filter then limits the speed (and, for one filter, the steering) against the obstacles. Every filter plans its stops with the shuttle's measured braking, 0.5 m/s² after a 0.7 s delay.
2. **Actuation loop, 20 Hz.** An ADRC speed controller turns the target speed into a control effort. The vehicle bridge, a small state machine, turns that effort into gear, brake and pedal, and never changes direction while the shuttle rolls. The CAN encoder checks the values and packs the `0x101` frame.

The node has five modes: `IDLE`, `AUTONOMOUS` (follow the path), `MANUAL` (speed and steering from `/teleop_cmd_vel`, through ADRC), `SAFE_STOP` (neutral and full brake until `/resume`) and `ESTOP`. `/safe_stop` is the soft stop: from `AUTONOMOUS` or `MANUAL` it enters `SAFE_STOP`, as a watchdog does, and `/resume` returns to `IDLE`. `/trigger_estop` enters `ESTOP` from any mode. It sends the hardware E-stop frame `0x100` and full brake every tick, and it latches. To recover, restart the node, then power-cycle the shuttle. Watchdogs move it to `SAFE_STOP` when GNSS or the localization goes stale, when perception goes silent, or when an obstacle comes inside the collision zone. In `MANUAL` the collision zone is checked behind the shuttle as well while it reverses.

## Contents

| Path | What it is |
|---|---|
| `nift_control/trajectory_controller.py` | The node: ROS wiring, watchdogs, services, diagnostics |
| `nift_control/modes/` | One handler per mode; the node delegates both loops to the active one |
| `nift_control/controllers/` | Lateral controllers (Pure Pursuit, Stanley) and longitudinal controllers (ADRC, PID) |
| `nift_control/filters/` | Safety filters: `distance_stop`, `cbf` (default), `potential_field`, `none` |
| `nift_control/dbw_interface/` | CAN frame builders and the vehicle bridge state machine. The wire format itself lives in `nift_common.can_protocol` |
| `nift_control/utils/` | `longitudinal_test`: guarded test passes on the shuttle, plus a tuner GUI and a live monitor |
| `config/planning.yaml` | Path following and safety filter parameters, including the braking the filters plan with |
| `config/adrc_params.yaml` | ADRC gains and the bridge's zero-speed tolerance |
| `config/garage_pad.yaml` | Example test pad for the `longitudinal_test monitor` map |

## Running

The controller is normally started by the system launch in `nift_bringup`, which loads both config files and sets `filter_type`, `lateral_enabled` and `localization_source` from the chosen profile. To run it on its own:

```bash
ros2 run nift_control trajectory_controller --ros-args \
  --params-file $(ros2 pkg prefix nift_control)/share/nift_control/config/planning.yaml \
  --params-file $(ros2 pkg prefix nift_control)/share/nift_control/config/adrc_params.yaml
```

Then drive it through its services, which all use `std_srvs/Trigger`.

```bash
ros2 service call /start_navigation std_srvs/srv/Trigger     # IDLE -> AUTONOMOUS (needs a path, fresh GNSS and obstacles)
ros2 service call /toggle_manual_mode std_srvs/srv/Trigger   # IDLE or AUTONOMOUS <-> MANUAL
ros2 service call /safe_stop std_srvs/srv/Trigger            # AUTONOMOUS or MANUAL -> SAFE_STOP (resumable)
ros2 service call /trigger_estop std_srvs/srv/Trigger        # -> ESTOP: hardware E-stop frame and full brake, latched
ros2 service call /resume std_srvs/srv/Trigger               # SAFE_STOP -> IDLE (refused in ESTOP)
```

`longitudinal_test` runs scripted passes on the real shuttle to measure its speed and steering response, for example `ros2 run nift_control longitudinal_test adrc --speed 1.0`. Never run it next to `trajectory_controller`, because both publish `/to_can_bus`. See the [longitudinal_test](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/longitudinal_test) wiki page for the tests and their guards.

## Tests

```bash
colcon build --packages-up-to nift_control
colcon test --packages-select nift_control && colcon test-result --verbose
```

| Test | What it covers |
|---|---|
| `test_can_interface.py` | The CAN wire format: golden bytes, round trips, steering rounding and limits, brake priority, malformed frames |
| `test_localization_source.py` | The controller node itself: the GNSS and localization watchdogs, the DEGRADED speed cap, start and MANUAL refusals, which steering `SAFE_STOP` holds |
| `test_trajectory_controller.py` | The node again: `/safe_stop` in every mode, the E-stop frame and its latch, the speed sign while braking to forward, no `SAFE_STOP` from a stale obstacle reading, MANUAL's check behind while reversing, `lateral_controller`, the braking parameters |
| `test_safety_filters.py` | The filters plan with the measured braking: stopping distance, the speed caps, a simulated approach at 1 to 2 m/s, no stale close reading, the rear check |
| `test_vehicle_bridge.py` | The bridge state machine: no direct edge between forward and reverse, the brake floor, the speed sign in every state |
| `test_steer_util.py` | The steering sequence parser, the predicted turn while stopping, and how the `0x301` acknowledgement follows a command |
| `test_longitudinal_ack.py` | `longitudinal_test` decodes the `0x301` acknowledgement with the shared steering decoder |
| `test_flake8.py`, `test_pep257.py` | Code and docstring style |

## License

Apache-2.0. Maintainer: Kane Weng.
