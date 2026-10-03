# nift_sensor

`nift_sensor` is the perception and localization package. It bridges the physical world into the ROS graph: raw GPS signals become a position + heading + velocity, raw LiDAR point clouds become obstacle polygons, and everything gets anchored into the coordinate frame that the rest of the stack expects.

The package contains three nodes that run continuously alongside each other.

---

## Overview

```
Hardware / Simulation
─────────────────────────────────────────────────────────────────────
Mcity Octane RTK server  (real)  ─┐
  or nift_carla Octane emulator (sim) ─┘ → [beacon_node]
                                           → /gnss/fix
                                           → /gnss/heading_deg
                                           → /gnss/vel

LiDAR (Velodyne) → /scan → [lidar_node]
                          → /obstacles  (2D polygon per obstacle cluster)

/gnss/fix + /gnss/heading_deg → [shuttle_tf_node]
                               → TF: map → base_link  (real and CARLA sim)
```

---

## How It Works

### GNSS Localization — `beacon_node`

The shuttle uses **two GPS antennas** (front and rear) for RTK localization. Having two antennas is what enables precise heading without an IMU: the vector from the rear antenna to the front antenna points in exactly the direction the vehicle is facing.

`beacon_node` connects via Socket.io to the Mcity Octane RTK server — a local infrastructure that provides centimeter-accurate positioning using a fixed base station at Mcity. The node subscribes to a real-time data stream, filters for the shuttle's specific beacon IDs, and normalizes the data format.

**Simulation** runs the same code. The CARLA bridge (`nift_carla`, published later) serves the same Socket.io protocol from its Octane emulator, and its launch files point `MCITY_OCTANE_*` at it.

**Stale data.** If either antenna has not delivered a new position for `gnss_timeout_s`, the node stops publishing until both report again. An Octane outage shows up as silence on `/gnss/*`, and `trajectory_controller` then brings the shuttle to a safe stop.

**Heading computation:**
```
heading = atan2(Δlon, Δlat)   where Δ = front_antenna − rear_antenna
```
This gives a compass bearing (degrees from North, clockwise) that is published directly.

**Speed:** the RTK server provides velocity directly (`/gnss/vel`).

**Published topics:**

| Topic | Type | Content |
|---|---|---|
| `/gnss/fix` | `sensor_msgs/NavSatFix` | Front antenna lat/lon/altitude |
| `/gnss/heading_deg` | `std_msgs/Float64` | Compass heading in degrees (0 = North, clockwise) |
| `/gnss/vel` | `geometry_msgs/TwistStamped` | Speed in `twist.linear.x` (m/s) |

---

### LiDAR Obstacle Detection — `lidar_node`

The LiDAR produces a dense 3D point cloud (~100k points per scan at 10 Hz). The node runs a multi-stage pipeline to reduce that to simple 2D polygons representing obstacles in the vehicle's path.

**Pipeline:**

```
Raw PointCloud2 (/scan)
    │
    ▼ 1. Voxel Downsampling
    │   Grid-quantize all points to a configurable voxel size (default 0.05 m).
    │   Reduces ~100k points to a few thousand for fast processing.
    │
    ▼ 2. ROI Crop
    │   Discard anything outside: x ∈ [0.3, 12.0] m (forward),
    │   y ∈ [−4.0, 4.0] m (lateral), z ∈ [−1.5, 2.0] m (vertical).
    │   Keeps only the region ahead of the vehicle where obstacles matter.
    │
    ▼ 3. RANSAC Ground Removal
    │   RANSAC (Random Sample Consensus) fits a plane to the dominant surface —
    │   the road. It works by randomly picking 3 points, fitting a plane,
    │   counting how many other points are "close enough" (inliers within
    │   0.15 m), and repeating. The plane with the most inliers is the road.
    │   All points within that threshold are discarded as ground.
    │
    ▼ 4. DBSCAN Clustering
    │   DBSCAN (Density-Based Spatial Clustering) groups the remaining points
    │   into objects. Unlike K-means, it doesn't require you to specify how many
    │   clusters to find — it discovers them based on density.
    │   Key parameters: eps (neighborhood radius = 0.6 m) and min_points (3).
    │   Any point with at least min_points neighbors within eps is a "core point"
    │   of a cluster; clusters grow outward from core points. Points too sparse
    │   to form a cluster are labeled as noise and discarded.
    │
    ▼ 5. Convex Hull
    │   For each cluster, project the 3D points into 2D (X-Y plane) and compute
    │   the convex hull — the smallest convex polygon containing all points.
    │   This becomes the obstacle footprint.
    │
    ▼ 6. TF Transform
        Apply the sensor → base_link transform (full 3D rotation, handles LiDAR
        tilt) before projecting to 2D, so all obstacles are in the vehicle's
        body frame regardless of how the sensor is mounted.

Output: /obstacles (one PolygonStamped per obstacle, in base_link frame)
```

**Debug topics** (for RViz visualization during development):

| Topic | Content |
|---|---|
| `/ground_points` | Points classified as road (after RANSAC) |
| `/nonground_points` | Non-ground points before clustering |
| `/front_points` | ROI-cropped cloud |
| `/roi_visual` | 3D wireframe box of the ROI region |

---

### Coordinate Bridge — `shuttle_tf_node`

The rest of the stack lives in the **map frame** (a fixed, UTM-projected coordinate system). But GPS gives us lat/lon, and the vehicle's body is in **base_link** (moving with the vehicle). This node bridges them.

**Conversion chain:**
```
GPS lat/lon  →  UTM Zone 17N (meters East/North)
                │
                │  subtract GNSS antenna offset
                │  (front antenna `gnss.offset_x` ahead of base_link = mid-wheelbase, rotated by heading)
                ▼
             base_link position in map frame
                │
                │  heading_deg → quaternion
                ▼
             map → base_link transform
```

Nothing else publishes into this TF chain (no wheel odometry, on the shuttle or in CARLA), so the node always publishes `map → base_link` directly.

**Open question: the antenna lever arm on the shuttle.** `config/shuttle_tf.yaml` sets `gnss.offset_x: 1.3` under the node name `shuttle_dynamic_broadcaster`, but `launch/shuttle_tf.launch.py` starts the node as `shuttle_tf_broadcaster`. So the file does not apply, and the code default of 1.041 m is used. The URDF puts the antennas at ±1.04 m. Which value matches the shuttle has to be decided before either is changed.

---

## Key Parameters

### beacon_node

| Parameter | Default | Description |
|---|---|---|
| `gnss_timeout_s` | `0.5` | Stop publishing `/gnss/*` once either antenna has had no new position this long |

Real-world credentials come from `nift_sensor/.env` (not committed to git).

### lidar_node

| Parameter | Default | Description |
|---|---|---|
| `voxel_size` | `0.05` m | Downsampling grid resolution |
| `ransac_distance_threshold` | `0.15` m | Max distance from ground plane to be classified as ground |
| `ransac_max_iterations` | `120` | RANSAC fitting iterations |
| `dbscan_eps` | `0.6` m | Neighborhood radius for cluster membership |
| `dbscan_min_points` | `3` | Minimum points to form a cluster core |
| `roi.min_x / max_x` | `0.3 / 12.0` m | Forward detection range |
| `roi.min_y / max_y` | `−4.0 / 4.0` m | Lateral detection width |
| `roi.min_z / max_z` | `−1.5 / 2.0` m | Vertical range |

### shuttle_tf_node

| Parameter | Default | Description |
|---|---|---|
| `gnss.offset_x` | `1.041` m | Front GNSS antenna distance ahead of `base_link` (mid-wheelbase); see the open question above |
| `gnss.offset_y` | `0.0` m | Lateral GNSS offset |
| `map_origin_lat` | `42.3005` | UTM projection origin latitude (Mcity) |
| `map_origin_lon` | `−83.6987` | UTM projection origin longitude (Mcity) |

---

> This package is launched automatically by the system bringup — see the [main README](../README.md) for launch commands.

**Maintainer**: Kane Weng (kaichi@umich.edu) — License: Apache-2.0
