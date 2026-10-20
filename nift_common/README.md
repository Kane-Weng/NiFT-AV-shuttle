# nift_common

The facts that every NiFT package shares, written down once: the shuttle's size and limits, where its sensors sit, the drive-by-wire CAN protocol, and the map origin. It has no nodes and depends on no other nift package, so every other package can depend on it.

More depth on the wiki: [nift_common](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_common).

## What it holds

| File | What it holds | Read by |
|---|---|---|
| `nift_common/vehicle.yaml` | the shuttle's geometry, kinematic limits and sensor mounts (GNSS antennas, LiDAR) | `vehicle.py`; the URDF in `shuttle_description`, through `xacro.load_yaml` |
| `nift_common/vehicle.py` | `ShuttleConfig` (the YAML's values plus derived ones: `base_link_x`, `max_curvature`, `min_turn_radius`) and `VehicleState` | control, sensor, waypoint, simulation |
| `nift_common/can_protocol.py` | the drive-by-wire CAN protocol: IDs, byte layout, steering encoding, `Gear`, pure encode and decode functions | `nift_control` (its `can_interface` builds the ROS frames on top), `nift_sensor`, `nift_carla` |
| `nift_common/site.py` | the map origin, near the Mcity test track in Ann Arbor; the `map_origin_lat` / `map_origin_lon` launch arguments default to it | bringup, sensor, waypoint, simulation |

## Using it

```python
from nift_common.vehicle import ShuttleConfig
from nift_common.can_protocol import compute_steering

shuttle = ShuttleConfig()
shuttle.wheel_base, shuttle.max_steering_angle, shuttle.min_turn_radius   # 2.082, 0.2618, 7.77
compute_steering(5.18)                                                    # (0xE2, 0x07)
```

`can_protocol.py` has no ROS import. A frame is anything with `id`, `dlc`, `data`, `is_extended`, `is_rtr` and `is_error`, so the same decoder works on a `can_msgs/Frame`, in a test, or in the simulator.

## Changing a value

Edit `vehicle.yaml` only. With `colcon build --symlink-install`, Python sees the change at once. The URDF reads the YAML when xacro runs, so relaunch whatever loads the robot description.

A few files can't import Python: the RViz panel (C++), the web dashboard (TypeScript) and the CARLA asset build scripts (Blender, Unreal). Each keeps a literal copy on a line marked `owner: nift_common/vehicle.yaml` (or `site.py`). `test/test_consistency.py` checks that every copy still matches.

Tuning doesn't belong here: controller gains, filter noise, thresholds and timeouts stay with the node that uses them, in its own package's YAML.

## Build and test

```bash
colcon build --symlink-install --packages-select nift_common
colcon test --packages-select nift_common && colcon test-result --verbose
```

The tests cover the YAML-to-dataclass mapping and the derived values (`test_vehicle.py`), the CAN encoding against the vendor's worked example plus round trips and malformed frames (`test_can_protocol.py`), the copies in other languages and the URDF (`test_consistency.py`), and flake8 and pep257. The consistency tests read the other packages' sources from the repository and skip when only the install is present. A checked file that is missing from the repository fails its test.

## License

Apache-2.0.
