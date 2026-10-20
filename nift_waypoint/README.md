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

The repository's [`maps/`](../maps) folder holds `lanelet2_odr2024.osm`, a map we generated from the public digital twin of the test track, and `setup.py` installs it into `share/nift_waypoint/maps/`. The shuttle and the simulation both use it by default. It is MIT licensed, see [`maps/README.md`](../maps/README.md). Any Lanelet2 map works, for example the example map that ships with Lanelet2 (see [Running it](#running-it)).

### Choosing the start and the goal

- **Start.** Each GNSS fix on `/gnss/fix` is projected into the map frame. RViz's "2D Pose Estimate" (`/initialpose`) can set the start by hand. That manual start holds while GNSS fixes stay within `manual_start_max_offset_m` (5 m) of it, and for good when there is no GNSS. A fix farther away replaces it. So a manual start can correct a GNSS start nearby, but on the shuttle a start far from the shuttle lasts only until the next fix.
- **Heading.** When a goal arrives, the node reads the shuttle's heading from the TF transform `map → base_link`, which `shuttle_tf_node` in `nift_sensor` builds from the two GNSS antennas. The simulation feeds the same transform through the same node. If the transform is missing, the node logs a warning and plans with the last heading it knew (east before the first one).
- **Goal.** A pose on `/goal_pose`, from RViz's "2D Goal Pose" or the web dashboard. Only its position is used.
- Both points snap to their nearest lanelet. If a goal arrives before the start is known, the node keeps it and plans as soon as a start position arrives.

### Routing

Lanelet2's routing graph finds the shortest route on the lane graph from the start lanelet to the goal lanelet (`getRoute`, then `shortestPath`). The route respects lane directions and the traffic rules of the map.

The route is not chosen for the turning radius. `utils/lanelet_planner.py` has an A\* search that rejects any route whose curvature is tighter than the shuttle's turning circle, but the node does not call it. The minimum turning radius comes from Ackermann geometry, R = L / tan(δ_max) = 2.082 m / tan(15°) ≈ 7.8 m (`ShuttleConfig.min_turn_radius` in `nift_common`). On the shipped map the A\* search finds no route at all for five of the sim's nine test routes, because their corners are tighter than that (route 11 has a 2 m corner). The node would then fall back to the direct curve that ignores the roads. So the node only checks the finished path and warns (see [Turning radius check](#turning-radius-check)).

### Building the path

The path has three parts.

```
start ──► [entry curve] ──► [lanelet centerlines] ──► [exit curve] ──► goal
```

1. **Lanelet centerlines.** The centerlines of the route's lanelets are joined in order. The first one is cut at the point closest to the start and the last one at the point closest to the goal. A lane change is different, because there Lanelet2's route steps to a lanelet beside the current one, not after it. The path keeps to the old lane up to the middle of the side-by-side lanelets and moves across on a cubic S-curve. The S-curve starts and ends along the lanes, 10 m before and after that middle. If it bends tighter than the planning radius (7.8 m × `planning_min_turn_radius_margin`), it is tried longer, up to 30 m each way, as far as the route leaves room.
2. **Entry curve.** If the start is more than 0.5 m from the first centerline point, a cubic Bezier curve joins them. It leaves the start along the shuttle's heading and arrives along the lane's direction.
3. **Exit curve.** If the goal is more than 0.5 m from the last centerline point, a second Bezier curve joins them.

Each Bezier curve has `bridge_bezier_pts` segments, and each of its two control handles is 0.7 times the distance between its end points. The lane change S-curves have a point every metre and handles of a third of that distance, which spreads the move across evenly. Finally the whole path is thinned to every `path_downsample_stride`-th point, always keeping the last point, and published in the `map` frame.

### Turning radius check

Before it publishes a path, the node measures the path's tightest curve. It uses the full path before thinning, curves included, and fits a circle through every three points that are `curvature_check_point_step` apart. Three points that turn straight back count as a radius of 0. If the tightest radius is below the shuttle's full lock radius (7.77 m), the node logs a warning with the place and its diagnostics go to WARN. The path is still published. This is a warning only. The node doesn't look for another route, and the controller still gets the tight curve. Several of the sim's test routes on the shipped map get this warning, for example route 11 with its 2 m corner.

### Fallback without the map

The node skips the lane graph and publishes a single Bezier curve from the start to the goal in three cases.

- The goal is more than `offroad_distance_threshold_m` (15 m) from the nearest lanelet.
- `/force_offroad` is `true`.
- The router finds no route.

If even that path is too short to use, the node publishes an empty path, which tells the controller to stop.

### Known limitations

- **Start point.** The start is the front GNSS antenna's position, about 1 m ahead of `base_link`, not the centre of the shuttle.
- **Turning radius.** Only the warning above checks it. A route through a corner tighter than the shuttle can turn is still published, and it is up to the operator to notice.
- **Where lane changes happen.** A lane change always crosses in the middle of the side-by-side lanelets, whatever the lane markings say there. Close to the start, the goal or another lane change there is less room, so the S-curve is shorter and tighter, and the turning radius check may warn.

---

## Running it

The node needs a map. Lanelet2's own example map (a part of Karlsruhe, Germany, installed with the `lanelet2` package) works without anything from the shuttle.

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

On the example map this goal is about 60 m away, around a 90° bend, and the planned path follows nine lanelets for about 72 m. Without TF the node warns that it has no heading and uses east, which only bends the entry curve.

To load the generated map instead, keep the default origin from `nift_common`:

```bash
ros2 run nift_waypoint waypoint_node --ros-args \
  -p map_file:=$(ros2 pkg prefix nift_waypoint)/share/nift_waypoint/maps/lanelet2_odr2024.osm
```

In the full stack the system bringup in `nift_bringup` starts the node and picks the map with its `map_file` launch argument.

### Tests

`colcon test --packages-select nift_waypoint` runs the linters and `test/test_waypoint_planning.py`. Those tests plan on Lanelet2's example map. They cover a route that starts with a lane change, the README demo route, the manual start and the turning radius warning.

---

## Topics

### Subscriptions

| Topic | Type | Description |
|---|---|---|
| `/gnss/fix` | `sensor_msgs/NavSatFix` | Shuttle position (front GNSS antenna) |
| TF `map → base_link` | tf2 | Heading, read when a goal arrives (from `shuttle_tf_node`, in simulation too) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Goal, from RViz or the web dashboard |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Manual start position (RViz "2D Pose Estimate"). It holds while GNSS fixes stay within `manual_start_max_offset_m` of it |
| `/force_offroad` | `std_msgs/Bool` | Always use the direct Bezier fallback |


### Publications

| Topic | Type | Description |
|---|---|---|
| `/planned_path` | `nav_msgs/Path` | The path for the controller, latched (transient local) |
| `/lanelet_map_markers` | `visualization_msgs/MarkerArray` | Lanelet borders and lane direction arrows, every `map_publish_period_s`. All markers are drawn at road level, `base_link_height` below the `map` origin |
| `/navigation_waypoints` | `visualization_msgs/MarkerArray` | The three path parts in different colours |
| `/start_goal_markers` | `visualization_msgs/MarkerArray` | Start (green) and goal (red) spheres |

The node also reports its state on `/diagnostics` as `Lanelet2 Router`, with the start source (GNSS or manual) and the tightest curve of the last path. The status is WARN while that curve is tighter than the shuttle can turn.

---

## Parameters

The defaults are in [`config/waypoint_params.yaml`](config/waypoint_params.yaml).

### Map

| Parameter | Default | Description |
|---|---|---|
| `map_file` | none, required | Path to the Lanelet2 `.osm` map |
| `map_origin_lat` | `42.3005` | Latitude of the `map` frame origin. The default comes from `nift_common/site.py`, and the system bringup passes its own launch argument |
| `map_origin_lon` | `-83.6987` | Longitude of the `map` frame origin (the same) |
| `fix_topic` | `/gnss/fix` | GNSS topic |
| `manual_start_max_offset_m` | `5.0` | A manual start from `/initialpose` holds while GNSS fixes stay within this distance of it. A fix farther away replaces it |
| `map_publish_period_s` | `1.0` | How often the map markers are republished |

### Path

| Parameter | Default | Description |
|---|---|---|
| `path_downsample_stride` | `5` | Publish every Nth path point |
| `bridge_bezier_pts` | `8` | Segments in each Bezier curve (at least 4) |
| `offroad_distance_threshold_m` | `15.0` | Goal distance from the map that triggers the direct fallback |

### Turning radius and the A\* search

| Parameter | Default | Description |
|---|---|---|
| `planning_min_turn_radius_margin` | `1.10` | Margin on the minimum turning radius. Lane change S-curves aim for the radius with this margin. The A\* search would use it too |
| `curvature_check_point_step` | `3` | Spacing of the three points used to measure curvature, for the turning radius check and the lane change S-curves |
| `relaxed_turn_radius_margin` | `0.85` | Not used. It was meant for a looser retry of the A\* search |
| `max_candidate_routes` | `40` | A\* search only (not called). Routes to try before giving up |
| `max_search_expansions` | `3000` | A\* search only (not called). Search expansions before giving up |
| `allow_lane_changes_in_search` | `false` | A\* search only (not called). Whether the search may change lanes |

---

More detail, with diagrams: the wiki pages [nift_waypoint](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_waypoint) and [waypoint_node](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/waypoint_node).

**Authors:** Kane Weng and Chun Ho (Jimmy) Wang. **Maintainer:** Kane Weng (kaichi@umich.edu). **License:** Apache-2.0.
