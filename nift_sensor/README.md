# nift_sensor

`nift_sensor` is the perception and localization package. It bridges the physical world into the ROS graph: raw GPS signals become a position + heading + velocity, raw LiDAR point clouds become obstacle polygons, and everything gets anchored into the coordinate frame that the rest of the stack expects.

The package has four ROS nodes and one command line tool. The [wiki](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_sensor) explains each of them in depth.

| Executable | What it does |
|---|---|
| `beacon_node` | Reads the front and rear RTK beacons from Mcity's Octane server, checks the feed, publishes `/gnss/*` |
| `shuttle_tf_node` | Turns each fix and heading straight into the `map → base_link` transform (`direct` mode) |
| `localization_node` | An EKF on the two antennas, the GNSS speed and our own steering; publishes `map → base_link` predicted to now (`ekf` mode) and a status |
| `lidar_node` | Turns each LiDAR scan into obstacle polygons on `/obstacles` |
| `octane_logger` | Not a ROS node: records the raw Octane feed and the network link to a JSONL file |

---

## Overview

```
Mcity Octane RTK server (real)  ─┐
  or nift_carla Octane emulator ─┴─ socket.io → [beacon_node] → /gnss/fix, /gnss/heading_deg, /gnss/vel
                                                              → /gnss/raw/front, /gnss/raw/rear, /gnss/hygiene

/gnss/fix + /gnss/heading_deg  → [shuttle_tf_node]   → TF map → base_link          (direct, shadow)
/gnss/raw/* + /gnss/vel + our 0x101 CAN commands
                               → [localization_node] → TF map → base_link          (ekf)
                                                     → /localization/odom, /localization/status

LiDAR (Velodyne) or CARLA LiDAR → /scan → [lidar_node] → /obstacles (2D polygon per obstacle)
```

Each input comes from the shuttle's hardware or from `nift_carla`'s stand-in in simulation, never both; the kinematic simulation has no LiDAR.

---

## How It Works

### GNSS Localization: `beacon_node`

The shuttle uses **two GPS antennas** (front and rear) for RTK localization. Having two antennas is what enables precise heading without an IMU: the vector from the rear antenna to the front antenna points in exactly the direction the vehicle is facing.

`beacon_node` connects via Socket.io to the Mcity Octane RTK server, an infrastructure service that provides centimeter-accurate positioning using a fixed base station at Mcity. The node subscribes to a real-time data stream, filters for the shuttle's specific beacon IDs, and normalizes the data format. The ROS-free part (the `.env` lookup, payload normalization, and a `probe()` that checks `auth_ok` and both antennas) lives in `nift_sensor/octane_client.py`, shared with the `system.launch.py` preflight check.

**Feed checks** (`nift_sensor/gnss_hygiene.py`): Octane repeats positions, delivers some late or out of order, and sometimes relays an older position under a fresh stamp. Every delivery is checked per antenna (late, stuck, older epoch, clock jump) and as a front / rear pair (antenna distance, heading change, midpoint step). Only a pair that passes is published. Every delivery also goes out on `/gnss/raw/front` / `/gnss/raw/rear` (status FIX if used, NO_FIX if dropped), and every dropped one on `/gnss/hygiene` with its verdict. By default the antenna distance is learned at start-up, from the latest 20 pairs once 3/4 of them agree within 5 cm. At rest every pair counts. While driving only pairs whose stamps are close enough to come from one receiver epoch count, so a node restarted while driving also recovers (after 2 to 8 s in the sim's feed model, longer at top speed). A distance learned while driving is learned again at the next stop. `hygiene.enabled:=false` (launch: `gnss_checks:=false`) turns the checks off.

**Simulation** runs the same code: the CARLA bridge (`nift_carla`) serves the same Socket.io protocol from its Octane emulator, and `system.launch.py` (any `mode: sim` profile) points `MCITY_OCTANE_*` at it.

**Stale data:** if either antenna has not delivered a new position, or no pair has passed the checks, for `gnss_timeout_s`, the node stops publishing until a fresh pair is accepted. An Octane outage shows up as silence on `/gnss/*`, and `trajectory_controller` then SAFE_STOPs.

**Heading computation:**
```
heading = atan2(Δeast, Δnorth)   Δ = front − rear antenna, in metres
```
Both antenna positions are converted to metres east and north first (WGS84 radii of curvature at their mean latitude), so the result is a compass bearing: degrees from true north, clockwise, published directly.

**Speed:** the RTK server provides velocity directly (`/gnss/vel`). Octane reports it in **knots** (measured on the shuttle: reported speed ÷ position-derived speed = 1.94); `octane_client.normalize()` converts to m/s, so every subscriber sees m/s.

**Published topics:**

| Topic | Type | Content |
|---|---|---|
| `/gnss/fix` | `sensor_msgs/NavSatFix` | Front antenna lat/lon/altitude |
| `/gnss/heading_deg` | `std_msgs/Float64` | Compass heading in degrees (0 = North, clockwise) |
| `/gnss/vel` | `geometry_msgs/TwistStamped` | Speed in `twist.linear.x` (m/s) |
| `/gnss/raw/front`, `/gnss/raw/rear` | `sensor_msgs/NavSatFix` | Every delivery as received; FIX = used, NO_FIX = dropped |
| `/gnss/hygiene` | `std_msgs/String` | JSON per dropped delivery: antenna, verdict, stamp, lat / lon |

---

### Coordinate Bridge: `shuttle_tf_node`

The rest of the stack lives in the **map frame** (a fixed, UTM-projected coordinate system). But GPS gives us lat/lon, and the vehicle's body is in **base_link** (moving with the vehicle). This node bridges them.

1. `/gnss/fix` is the front antenna. Its lat/lon is projected to UTM zone 17N relative to the map origin (`map_origin_lat` / `map_origin_lon`): the antenna's position in `map`.
2. `/gnss/heading_deg` is a compass bearing from true north, but the map's +y is UTM grid north. The two differ by the grid convergence (1.82° at Mcity); the node measures it at the map origin and uses it to turn the bearing into a map yaw (0 = +x, counter-clockwise).
3. The antenna sits `gnss.offset_x` ahead of `base_link` (mid-wheelbase). Rotated by the yaw and subtracted, that gives `base_link`'s position, which with the yaw is the `map → base_link` transform.

The fix and the heading of one sample arrive as two messages. `beacon_node` sends the fix first, so the node sends the transform when the heading arrives, with the fix that came just before it. Each fix is paired with the heading of its own sample, and nothing is sent before the first heading. A fix whose heading is lost is skipped.

Nothing else publishes into this TF chain (no wheel odometry, on the shuttle or in CARLA), so the node publishes `map → base_link` directly.

---

### Localization Filter: `localization_node`

`shuttle_tf_node`'s pose is as old as the fix (0.1–0.2 s) and freezes when the feed pauses. `localization_node` wraps `localization_filter.py`, an EKF on a kinematic bicycle model. It fuses both antennas' raw positions, the GNSS speed, and the steering, gear and brake of our own `0x101` drive commands (only while `trajectory_controller` is actually driving), predicts the pose to now at 50 Hz, and carries it across short GNSS gaps. `/localization/status` says OK, DEGRADED (bridging) or LOST.

Only one node publishes `map → base_link`; the system launch picks the mode:

| Mode | `map → base_link` from | `localization_node` |
|---|---|---|
| `direct` (default) | `shuttle_tf_node` | off |
| `ekf` | `localization_node` | publishes TF, odom and status; the controller follows the status |
| `shadow` | `shuttle_tf_node` | odom and status only, for recording beside the direct transform |

**Not measured: the antenna lever arm on the shuttle.** The antennas' positions (`gnss_front_x` / `gnss_rear_x`, ±1.041 m from `base_link`) come from `nift_common`'s `vehicle.yaml`, which the URDF, the sims, `shuttle_tf_node` and `localization_node` all use. The 1.041 m is half the wheelbase, a placeholder, not a measurement (driving data suggest 1.2–1.6 m for the front antenna). A wrong value shifts the pose along the heading by the error. To measure it, use the centre of each antenna (its phase centre, often marked on the housing). Measure along the centre line, level with the ground, from the front axle (the line through the front wheel centres) to the front antenna, and from the rear axle to the rear antenna. Then `gnss_front_x` = 1.041 m + the front antenna's distance ahead of the front axle, and `gnss_rear_x` = −1.041 m − the rear antenna's distance behind the rear axle (a distance the other way counts negative). Note any sideways offset from the centre line too. Change the values in `vehicle.yaml` only.

---

### LiDAR Obstacle Detection: `lidar_node`

The LiDAR produces a dense 3D point cloud (~100k points per scan at 10 Hz). The node runs a multi-stage pipeline to reduce that to simple 2D polygons representing obstacles in the vehicle's path.

1. **Voxel downsample**: one point per voxel (`voxel_size`), so the later steps get a few thousand points instead of ~100k.
2. **RANSAC ground removal**: RANSAC (Random Sample Consensus) fits a plane to the dominant surface, the road. It picks 3 random points, fits a plane, counts the points within `ransac_distance_threshold` of it, and repeats (`ransac_max_iterations`). The plane with the most inliers is the road; its inliers are dropped as ground.
3. **ROI crop**: keeps the non-ground points inside the box ahead of the vehicle (`roi.*`, in the sensor frame).
4. **Tilt correction**: rotates the points into `base_link` with the LiDAR's static transform from the URDF (full 3D rotation), so dropping z later doesn't shorten distances when the sensor is tilted. Without that transform the frame is dropped.
5. **Height gate**, on in simulation only (`min_obstacle_height` is 0.2 m in `perception_carla.yaml`, 0 = off on the shuttle): drops points less than `min_obstacle_height` above the ground under the vehicle, the road surface a single RANSAC plane misses on a crowned road.
6. **DBSCAN clustering** (Density-Based Spatial Clustering) in the flat `base_link` x-y plane. Unlike K-means it doesn't need the number of clusters: a point with at least `dbscan_min_points` neighbours within `dbscan_eps` is a core point, clusters grow outward from core points, and points too sparse for a cluster are noise. Clusters outside `min_cluster_size`–`max_cluster_size` points are dropped.
7. **Convex hull** of each cluster, in 2D: the smallest convex polygon around its points, the obstacle footprint.

Output: `/obstacles`, one `PolygonStamped` per obstacle in `base_link`, all with the cloud's stamp. A frame with no obstacle publishes one empty polygon, so consumers drop the previous ones.

**Debug topics** (for RViz during development; the three clouds only with `publish_debug_clouds`, default on):

| Topic | Content |
|---|---|
| `/ground_points` | Points removed as road (step 2) |
| `/nonground_points` | Points left after step 2 |
| `/front_points` | Points left after the ROI crop (step 3) |
| `/roi_visual` | 3D wireframe box of the ROI region |

---

### Raw feed recorder: `octane_logger`

A plain Python program (no ROS) that logs every Octane event exactly as it arrives, plus a TCP and Wi-Fi probe once a second, to `<out>/octane_<time>.jsonl`, and a summary with gaps per antenna at the end. Use it to map where GNSS drops out on the track and whether the shuttle's own link was the cause.

```bash
ros2 run nift_sensor octane_logger --out octane_logs --minutes 10
```

---

## Key Parameters

### beacon_node

`config/beacon_node.yaml` holds every parameter with its code default.

| Parameter | Default | Description |
|---|---|---|
| `gnss_timeout_s` | `0.5` | Stop publishing `/gnss/*` once an antenna's last new position, or the last accepted pair, is this old |
| `hygiene.enabled` | `true` | The feed checks; false publishes every new stamp |
| `hygiene.*` | see the YAML | Limits of the per-antenna and pair checks |

Real-world credentials come from `nift_sensor/.env` (copy `.env_example`; `.env` is not committed to git).

### localization_node

`config/localization.yaml` holds the shuttle's values (mode, GNSS delay, steering and brake inputs, filter thresholds). The geometry and the map origin default to `nift_common`.

`filter.bridge_s` (1.5 s) must stay at least 0.3 s below `beacon_node`'s `hygiene.reset_s` (2.0 s). After `reset_s` without a good pair, the feed checks take the next consistent pair, even a wrong one such as swapped antennas or a held offset. The filter has to be LOST by then, so the shuttle stops instead of following it. On the sim's fault bench, `bridge_s` 2.0 s already followed a held 1 m offset. A test checks the two files.

### lidar_node

`system.launch.py` loads `config/perception.yaml` on the shuttle and `nift_carla`'s `config/perception_carla.yaml` in simulation (`perception_yaml:=` for another file); the code default applies to anything a file leaves out.

| Parameter | Code default | Shuttle | Simulation | Description |
|---|---|---|---|---|
| `voxel_size` | `0.1` m | `0.05` m | `0.05` m | Downsampling grid resolution |
| `ransac_distance_threshold` | `0.15` m | `0.15` m | `0.25` m | Max distance from the ground plane to count as ground |
| `ransac_max_iterations` | `120` | `120` | `200` | RANSAC fitting iterations |
| `min_obstacle_height` | `0.0` (off) | off | `0.20` m | Height gate above the ground under the vehicle |
| `dbscan_eps` | `0.45` m | `0.6` m | `0.6` m | Neighborhood radius for cluster membership |
| `dbscan_min_points` | `8` | `3` | `6` | Minimum neighbours for a core point |
| `min_cluster_size` / `max_cluster_size` | `8` / `5000` | `3` / `5000` | `6` / `5000` | Clusters outside this many points are dropped |
| `roi.min_x / max_x` | `0.3 / 12.0` m | same | same | Forward detection range |
| `roi.min_y / max_y` | `−4.0 / 4.0` m | same | `−2.5 / 2.5` m | Lateral detection width |
| `roi.min_z / max_z` | `−1.5 / 2.0` m | same | same | Vertical range |

### shuttle_tf_node

| Parameter | Default | Description |
|---|---|---|
| `gnss.offset_x` | `1.041` m (`vehicle.yaml` `gnss_front_x`) | Front GNSS antenna distance ahead of `base_link` (mid-wheelbase); not measured, see above |
| `gnss.offset_y` | `0.0` m | Lateral GNSS offset |
| `map_origin_lat` | `42.3005` (`nift_common.site`) | UTM projection origin latitude (Mcity) |
| `map_origin_lon` | `−83.6987` (`nift_common.site`) | UTM projection origin longitude (Mcity) |

---

## Tests

```bash
pip3 install -r src/nift_sensor/requirements.txt
colcon build --packages-up-to nift_sensor
colcon test --packages-select nift_sensor && colcon test-result --verbose
```

Unit tests cover the feed checks (including a restart while driving), the localization filter and node, the Octane helpers and the heading chain (each fix paired with its own sample's heading); flake8 and pep257 run too.

---

> This package is launched by the system bringup, see the [main README](../README.md#getting-started) for launch commands.

**Maintainer**: Kane Weng (kaichi@umich.edu), License: Apache-2.0
