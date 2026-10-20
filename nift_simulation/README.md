# nift_simulation

`nift_simulation` holds the vehicle model, the RViz tools and the simulator of the NiFT shuttle. The model and the RViz tools are used on the real shuttle too. It holds three packages:

| Package | What it is |
|---|---|
| [`shuttle_description`](./shuttle_description) | What the vehicle is: a box-shaped URDF with the real sizes and sensor mounts, behind the TF tree and RViz. Also the RViz configs |
| [`nift_carla`](./nift_carla) | The simulator, with two backends. The CARLA bridge connects to a CARLA 0.9.15 server that runs the public [Mcity Digital Twin](https://github.com/mcity/mcity-digital-twin). The kinematic backend runs without CARLA or a GPU (and without a LiDAR) |
| [`nift_rviz_plugins`](./nift_rviz_plugins) | Three RViz panels: navigation controls, a TF monitor and node health. Used with either backend and on the shuttle |

Both simulator backends replace only what the hardware provides: the drive-by-wire ECU, the GNSS beacons and the LiDAR. The autonomy stack runs its real-vehicle code. It sends the same CAN frames, and `beacon_node` reads GNSS over the same Socket.io protocol as on the shuttle.

Wiki: [nift_carla](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_carla), [shuttle_description](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/shuttle_description), [nift_rviz_plugins](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_rviz_plugins).

## Vehicle model

The URDF draws the shuttle as boxes and cylinders with its real sizes. Every size and mount comes from `nift_common/vehicle.yaml`, which both the xacro files and the Python code read. The [shuttle_description README](./shuttle_description/README.md) has the details.

| Value | |
|---|---|
| Wheelbase | 2.082 m |
| Length × width × height | 3.055 m × 1.588 m × 1.905 m |
| Wheel radius | 0.228 m |
| Steering | Ackermann, ±15° at the front wheels |
| LiDAR `velodyne` | 1.53 m ahead of `base_link`, 0.96 m up, pitched 15° down |
| GNSS antennas `gps_frame_1` / `gps_frame_2` | ±1.041 m from `base_link`, 0.96 m up |

The simulator places its LiDAR and antennas at these same mounts, so the TF tree matches where the sim samples the sensors.

## Simulation

The simulator starts through `nift_bringup`, together with the stack:

| Command | What runs |
|---|---|
| `ros2 launch nift_bringup system.launch.py profile:=sim_kinematic` | the kinematic backend and the full stack minus perception. No CARLA, no GPU |
| `ros2 launch nift_bringup system.launch.py profile:=sim` | the CARLA backend and everything the shuttle runs: `beacon_node`, `shuttle_tf_node`, `lidar_node`, `waypoint_node`, `trajectory_controller`, RViz, gated on the beacon `auth_ok` as on the shuttle |
| `ros2 launch nift_bringup system.launch.py profile:=sim_filter_bench` | the safety-filter bench in CARLA: a spawn, a static path and scripted obstacles from `nift_carla/config/scenarios/filter_bench.yaml` |

Any shuttle profile also runs in simulation with `mode:=sim`, and `sim_backend:=kinematic` picks the kinematic backend. Teleport the vehicle with RViz **2D Pose Estimate**. The [nift_carla README](./nift_carla/README.md) covers the contract, the CARLA server setup, the parameters and the tools.

Gazebo was the simulator before CARLA. It was removed because it could not exercise the CAN path, and two backends with different interfaces doubled every interface change. The kinematic backend avoids both. It takes the same CAN frames through the same virtual ECU, and everything except the pose integration is shared code with the CARLA bridge.

## Topics in simulation

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/to_can_bus` | `can_msgs/Frame` | stack → sim | the shuttle's CAN commands, decoded by the virtual ECU |
| `/scan` | `sensor_msgs/PointCloud2` | sim → stack | frame `velodyne`, 10 Hz; CARLA backend only |
| `/clock` | `rosgraph_msgs/Clock` | sim → all | sim time; every node runs with `use_sim_time` |
| `/ackermann_like_controller/odom` | `nav_msgs/Odometry` | sim → tools | ground truth (the name dates from the Gazebo setup) |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz → sim | teleports the vehicle |
| `/carla/dbw_state`, `/carla/gps_health` | `std_msgs` | sim → tools | virtual ECU state, injected GNSS dropout |

GNSS reaches the stack the way it does on the shuttle, through `beacon_node` over Socket.io, not as a ROS topic.

---

**Maintainer**: Kane Weng. License: Apache-2.0.
