# nift_bringup

This package holds the shared vehicle configuration of the NiFT shuttle stack. The launch file that starts the whole system will also live here. It is not published yet.

## ShuttleConfig

`ShuttleConfig` is a frozen Python dataclass with the shuttle's geometry and limits. Every package that needs a vehicle constant imports it from here, so each value is defined in one place. Because the dataclass is frozen, no node can change a constant at runtime.

```python
from nift_bringup.shuttle_config import ShuttleConfig

config = ShuttleConfig()
print(config.wheel_base)  # 2.082
```

| Field | Value | Meaning |
|---|---|---|
| `length`, `width`, `height` | 3.055 m, 1.588 m, 1.905 m | Body size |
| `wheel_base` | 2.082 m | Distance between the front and rear axles |
| `front_overhang`, `rear_overhang` | 0.483 m, 0.490 m | Distance from each axle to the end of the body |
| `wheel_radius`, `wheel_width` | 0.228 m, 0.213 m | Wheel size |
| `wheel_tread` | 0.735 m | Wheel tread |
| `max_steering_angle` | 0.2618 rad (15°) | Steering limit |
| `max_speed` | 4.917 m/s (17.7 km/h) | Speed limit |
| `max_accel`, `max_decel` | 0.8 m/s², 7.0 m/s² | Acceleration and braking limits |
| `mass` | 500 kg | Vehicle mass |
| `cog_height` | 0.658 m | Height of the center of gravity |

The wheelbase and the two overhangs add up to the body length.

## VehicleState

`VehicleState` is the state that the trajectory controller updates every cycle. It holds the position `x`, `y` in the `map` frame, the heading `yaw` and the speed `vel`. The position comes from the `map → base_link` transform, and the controller treats it as the midpoint between the axles. `x_rear` and `y_rear` hold the rear axle position, because Pure Pursuit steers from the rear axle.

## Tests

```bash
colcon test --packages-select nift_bringup && colcon test-result --verbose
```

This runs the flake8 and pep257 style checks.
