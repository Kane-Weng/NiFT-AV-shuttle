# shuttle_description

The NiFT shuttle as a robot model. The URDF (built from xacro files) describes the body, the four wheels with Ackermann steering joints, the LiDAR and the two GNSS antennas. `robot_state_publisher` turns it into the static part of the TF tree, which the sensor nodes need on the shuttle and in simulation alike. The package also holds the RViz configs that the whole stack uses.

More depth on the wiki: [shuttle_description](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/shuttle_description) and [URDF and Xacro](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/URDF-and-Xacro).

## What it holds

| Path | What |
|---|---|
| `description/vehicle.urdf.xacro` | Entry point. It includes the three files below |
| `description/vehicle.xacro` | `base_link`, `base_footprint`, the body, the wheel and steering joints (plus `colors.xacro` and `inertial_macros.xacro`) |
| `description/lidar.xacro` | The LiDAR link `velodyne` and its tilted mount |
| `description/gps.xacro` | The antenna links `gps_frame_1` (front) and `gps_frame_2` (rear) |
| `launch/robot_view.launch.py` | The model alone in RViz, with a slider for every movable joint |
| `rviz/navigation.rviz` | The main operator view: map, route, LiDAR, obstacles, and the three panels from [`nift_rviz_plugins`](../nift_rviz_plugins) |
| `rviz/navigation_topdown.rviz` | A second, render-only window that follows the shuttle from above |
| `rviz/robot_view.rviz` | The view that `robot_view.launch.py` opens |
| `test/test_description.py` | Checks that `base_link` is the only root of the model, that the RViz views use frames the stack publishes, and that `robot_view.launch.py` defaults to wall time |

## The model

Every size and mount comes from [`nift_common/vehicle.yaml`](../../nift_common/nift_common/vehicle.yaml), which the xacro files load with `xacro.load_yaml`. Change a value there only. A test in `nift_common` checks that the xacro files keep no copies of those numbers.

The body is drawn as simple boxes with the real outer size (3.055 × 1.588 × 1.905 m): a white lower body, a smoked glass cabin, a white roof and an orange front bumper so the heading reads at a glance. The wheels are cylinders with the real radius (0.228 m) and width, 2.082 m apart front to back. The team's CAD meshes are not part of this repository.

`base_link` is the root. It sits mid-wheelbase, 0.9525 m above the ground, and `base_footprint` is the ground point below it. `shuttle_tf_node` (in `nift_sensor`) publishes `map → base_link`, so the URDF and the localization together form one connected tree. Nothing in the stack publishes `/joint_states`, so the wheel frames are not in TF while the stack runs. That is why the wheels are drawn on the chassis link.

## Running it

```bash
colcon build --symlink-install --packages-up-to shuttle_description
source install/setup.bash
ros2 launch shuttle_description robot_view.launch.py
```

`robot_view.rviz` uses `base_link` as its fixed frame, because this launch file publishes no `map` frame. `use_sim_time` defaults to false, as on the shuttle, and applies to all three nodes. Pass `use_sim_time:=true` only next to a simulator that publishes `/clock`.

In the full stack, `nift_bringup` starts `robot_state_publisher` with this URDF in every mode and opens `rviz/navigation.rviz` (and `navigation_topdown.rviz` with `rviz_layout:=split`).

## Credits

The two navigation RViz configs were co-written with Chun Ho (Jimmy) Wang.

## License

Apache-2.0.
