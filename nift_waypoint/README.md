# nift_waypoint

`nift_waypoint` turns a goal pose into a path the shuttle can drive. It finds a route on a Lanelet2 HD map from the shuttle's position to the goal. Then it joins the route to the shuttle and to the goal with smooth Bezier curves, and publishes the result as a `nav_msgs/Path` for the controller.

---

## Overview

```
/gnss/fix ──────┐
/initialpose ───┤
/goal_pose ─────┼──► [waypoint_node] ──► /planned_path ──► nift_control
TF map→base ────┘          │
                           └──► RViz markers (map, route, start and goal)
```

The node plans once for every new goal. It publishes the path with transient local durability, so a subscriber that joins later, like the controller, still receives the latest path.

---

## How it works

### Lanelet2 maps

A Lanelet2 map describes a road network as *lanelets*. A lanelet is one directed lane segment with a left and a right border. Its centerline is the ideal driving line. Lanelets that share border points are connected, so the map forms a graph of lanes. The type of each border line, solid or dashed, decides where a lane change is allowed. The map can also hold regulatory elements such as speed limits and stop lines. Maps are stored as OpenStreetMap XML (`.osm`) in latitude and longitude. The node projects them into flat UTM metres around a map origin (`map_origin_lat`, `map_origin_lon`).

On the shuttle the stack uses an HD map of the Mcity test track. That map is not part of this repository. Any Lanelet2 map works, for example the example map that ships with Lanelet2 (see [Running it](#running-it)).

### Choosing the start and the goal

- **Start.** Each GNSS fix on `/gnss/fix` is projected into the map frame. RViz's "2D Pose Estimate" (`/initialpose`) can set the start by hand, until the next GNSS fix replaces it.
- **Goal.** A pose on `/goal_pose`, from RViz's "2D Goal Pose" or the web dashboard. Only its position is used.
- Both points snap to their nearest lanelet. If a goal arrives before the start is known, the node keeps it and plans as soon as a start position arrives.

### Routing

Lanelet2's routing graph finds the shortest route on the lane graph from the start lanelet to the goal lanelet (`getRoute`, then `shortestPath`). The route respects lane directions and the traffic rules of the map.

`utils/lanelet_planner.py` also has an A\* search that rejects any route whose curvature is tighter than the shuttle's turning circle. The minimum turning radius comes from Ackermann geometry, R = L / tan(δ_max) = 2.082 m / tan(15°) ≈ 7.8 m, with a 10% margin. The node does not call this search yet, so the search parameters below have no effect for now.

### Building the path

The path has three parts.

```
start ──► [entry curve] ──► [lanelet centerlines] ──► [exit curve] ──► goal
```

1. **Lanelet centerlines.** The centerlines of the route's lanelets are joined in order. The first one is cut at the point closest to the start and the last one at the point closest to the goal.
2. **Entry curve.** If the start is more than 0.5 m from the first centerline point, a cubic Bezier curve joins them. It leaves the start along the shuttle's heading and arrives along the lane's direction.
3. **Exit curve.** If the goal is more than 0.5 m from the last centerline point, a second Bezier curve joins them.

Each Bezier curve has `bridge_bezier_pts` segments, and each of its two control handles is 0.7 times the distance between its end points. Finally the whole path is thinned to every `path_downsample_stride`-th point, always keeping the last point, and published in the `map` frame.

### Fallback without the map

The node skips the lane graph and publishes a single Bezier curve from the start to the goal in three cases.

- The goal is more than `offroad_distance_threshold_m` (15 m) from the nearest lanelet.
- `/force_offroad` is `true`.
- The router finds no route.

If even that path is too short to use, the node publishes an empty path, which tells the controller to stop.

### Known limitations

- **Heading.** The node updates the shuttle's heading only when a message arrives on the odometry topic. The CARLA simulation publishes it, but nothing on the shuttle does yet. On the shuttle the heading then stays at 0 (east), which bends the entry curve and the fallback curve.
- **Lane changes.** If the shortest route contains a lane change, the centerlines of two side-by-side lanelets are joined end to start, so the path turns back on itself near the lane change.

---

## Running it

The node needs a map. Lanelet2's own example map (a part of Karlsruhe, Germany, installed with the `lanelet2` package) works without any Mcity data.

```bash
MAP=$(ros2 pkg prefix lanelet2_maps)/share/lanelet2_maps/res/mapping_example.osm
ros2 run nift_waypoint waypoint_node --ros-args \
  -p map_file:=$MAP -p map_origin_lat:=49.0 -p map_origin_lon:=8.4
```

In RViz, set the fixed frame to `map` and add the `/lanelet_map_markers`, `/navigation_waypoints` and `/start_goal_markers` marker arrays and the `/planned_path` path. Then place a start with "2D Pose Estimate" and a goal with "2D Goal Pose".

The same works from the command line. Start the echo in a second terminal first, then send the start and the goal from a third.

```bash
# terminal 2
ros2 topic echo /planned_path

# terminal 3
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 1139.1, y: 535.3}}}}"
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1117.1, y: 592.4}}}"
```

On the example map this goal is about 60 m away, around a 90° bend, and the planned path follows nine lanelets for about 72 m. On the shuttle the system bringup starts the node with the Mcity map and origin.

---

## Topics

### Subscriptions

| Topic | Type | Description |
|---|---|---|
| `/gnss/fix` | `sensor_msgs/NavSatFix` | Shuttle position (front GNSS antenna) |
| `/ackermann_like_controller/odom` | `nav_msgs/Odometry` | Heading, and a position fallback before the first GNSS fix. After the first fix, each message makes the node read the heading from the `map → base_link` transform instead. The CARLA bridge publishes ground truth here. The name dates from an earlier simulator |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Goal, from RViz or the web dashboard |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Manual start position (RViz "2D Pose Estimate") |
| `/force_offroad` | `std_msgs/Bool` | Always use the direct Bezier fallback |


### Publications

| Topic | Type | Description |
|---|---|---|
| `/planned_path` | `nav_msgs/Path` | The path for the controller, latched (transient local) |
| `/lanelet_map_markers` | `visualization_msgs/MarkerArray` | Lanelet borders and lane direction arrows, every `map_publish_period_s` |
| `/navigation_waypoints` | `visualization_msgs/MarkerArray` | The three path parts in different colours |
| `/start_goal_markers` | `visualization_msgs/MarkerArray` | Start (green) and goal (red) spheres |

The node also reports its state on `/diagnostics` as `Lanelet2 Router`.

---

## Parameters

The defaults are in [`config/waypoint_params.yaml`](config/waypoint_params.yaml).

### Map

| Parameter | Default | Description |
|---|---|---|
| `map_file` | none, required | Path to the Lanelet2 `.osm` map |
| `map_origin_lat` | `42.3005` | Latitude of the `map` frame origin (Mcity) |
| `map_origin_lon` | `-83.6987` | Longitude of the `map` frame origin (Mcity) |
| `fix_topic` | `/gnss/fix` | GNSS topic |
| `odom_topic` | `/ackermann_like_controller/odom` | Odometry topic |
| `map_publish_period_s` | `1.0` | How often the map markers are republished |

### Path

| Parameter | Default | Description |
|---|---|---|
| `path_downsample_stride` | `5` | Publish every Nth path point |
| `bridge_bezier_pts` | `8` | Segments in each Bezier curve (at least 4) |
| `offroad_distance_threshold_m` | `15.0` | Goal distance from the map that triggers the direct fallback |

### A\* search (not called yet)

| Parameter | Default | Description |
|---|---|---|
| `planning_min_turn_radius_margin` | `1.10` | Margin on the minimum turning radius |
| `relaxed_turn_radius_margin` | `0.85` | A looser margin, not used by the code |
| `curvature_check_point_step` | `3` | Spacing of the three points used to measure curvature |
| `max_candidate_routes` | `40` | Routes to try before giving up |
| `max_search_expansions` | `3000` | Search expansions before giving up |
| `allow_lane_changes_in_search` | `false` | Whether the search may change lanes |

---

**Authors:** Kane Weng and Chun Ho (Jimmy) Wang. **Maintainer:** Kane Weng (kaichi@umich.edu). **License:** Apache-2.0.
