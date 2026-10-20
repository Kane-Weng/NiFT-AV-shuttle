# nift_rviz_plugins

Three RViz 2 panels that turn RViz into the shuttle's operator console. They work the same on the real shuttle and in simulation. The panels are C++ Qt widgets, exported to RViz through `pluginlib`.

More depth on the wiki: [nift_rviz_plugins](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_rviz_plugins).

## The panels

| Panel (dock title) | What it does | ROS interfaces |
|---|---|---|
| `NavPanel` (Navigation) | Start navigation, switch manual mode, Stop (resumable) and E-STOP (latching), Resume, a line with the last service reply, keyboard teleop (WASD or arrows), live telemetry, camera buttons for the saved views | calls `/start_navigation`, `/toggle_manual_mode`, `/safe_stop`, `/trigger_estop`, `/resume` (`std_srvs/Trigger`); publishes `/teleop_cmd_vel` (`geometry_msgs/Twist`, 10 Hz while the teleop zone has focus); subscribes `/nav_telemetry` (`std_msgs/String`) |
| `TfMonitorPanel` (TF Monitor) | One line with the shuttle's position and heading in the map, or a red warning when the transform is missing | looks up `map → base_link` in TF, 10 Hz |
| `NodeCheckerPanel` (System Health) | A green, yellow, red or grey mark for every node that reports on `/diagnostics`, with the message under any entry that is not OK | subscribes `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) |

The services and the telemetry belong to `trajectory_controller` in `nift_control`. The teleop zone sends a manual-mode request on the first key press. The panel reads the controller's mode from the `STATUS` line of `/nav_telemetry`. In `SAFE_STOP` it shows Resume. In `ESTOP` it freezes every control but E-STOP, because the E-stop latches until the controller restarts and the shuttle is power-cycled. Either view ends when the telemetry stops for 2 s, which means the controller was restarted or is gone. The panel's steering limit is a copy of `max_steering_deg` in `nift_common/vehicle.yaml`, on a line marked `owner:`, and a test in `nift_common` checks that it still matches.

## Using it

The panels load with `shuttle_description/rviz/navigation.rviz`, which the full stack opens. To add one to another config, use **Panels → Add New Panel → nift_rviz_plugins**.

```bash
colcon build --symlink-install --packages-up-to nift_rviz_plugins
source install/setup.bash
rviz2 -d $(ros2 pkg prefix shuttle_description)/share/shuttle_description/rviz/navigation.rviz
```

Each panel creates its own small ROS node and spins it from a Qt timer, so ROS callbacks never block the RViz window. Text sizes follow the panel font rather than pixels, so the panels look the same on a normal and a HiDPI screen.

## Tests

`colcon test --packages-select nift_rviz_plugins` runs the ament linters: uncrustify, cppcheck, lint_cmake and xmllint. cpplint and the copyright check are switched off in `CMakeLists.txt`.

## License

Apache-2.0.
