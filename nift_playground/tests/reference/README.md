# Reference runs of the real ROS 2 stack

These are recordings of the real stack in the kinematic sim. The playground is checked against them: same route, same map, same parameters, and the trajectory, outcome and SAFE_STOP timing should match within the spread shown below.

## What was run

- **Stack.** `ros2 launch nift_bringup system.launch.py profile:=sim_kinematic rviz:=false` with the route's spawn, started by `sim_route_run` (`nift_carla/tools/route_run.py`). The profile runs `kinematic_sim_node`, `beacon_node`, `shuttle_tf_node` (or `localization_node` with `localization:=ekf`), `waypoint_node` and `trajectory_controller`. There is no RViz and no web component.
- **Map.** `maps/lanelet2_odr2024.osm`, the default for `mode: sim`.
- **Routes.** Every route of `nift_carla/config/test_routes.yaml` that runs on that map: 01, 02, 03, 04, 05, 11 and 12. There are no routes 06 to 10 in the file. Routes 13 and 14 need the 2025 map and were not run.
- **Settings.** `time_factor` 5 (the tool's default for the kinematic backend), ideal GNSS feed, no noise (`carla_params.yaml` defaults), lockstep on. The GNSS-loss runs and `det_04_tf1` use `time_factor` 1, as `sim_gnss_bench` does.
- **Repo state.** Public repo at `902436a` (branch `master`, clean). The workspace was a copy of `git ls-files --cached --others --exclude-standard` without `_local_no_commit/`, built with `colcon build --symlink-install`. Image `nift_pub_deps:humble` (`a94b4c3190`, ROS Humble, rclpy 3.3.22, python-socketio 5.17.0). Recorded on 2026-10-20.
- **Re-recorded (2026-10-20, later).** Every run that uses `shuttle_tf_node` was recorded again after its fix: each fix is now paired with the heading of its own sample, where the node used to take the previous sample's heading. That is `direct_*`, `det_*` and both GNSS-loss runs. The workspace was `e043fe7` plus only the sensing fixes of that change (`nift_sensor`, the `shuttle_description` launch and RViz files, `nift_common`'s tests), so other work in progress is not in them. The `ekf_*` runs don't use `shuttle_tf_node` and are the 2026-10-20 recordings from `902436a`. The beacon_node fix of the same change (the antenna distance learned while driving too) gives the same result as before on a start at rest.

## Results

| Route | What | direct | ekf |
|---|---|---|---|
| 01 | long straight, south | arrived, 120.1 s | arrived, 120.1 s |
| 02 | straight, north | arrived, 104.8 s | arrived, 104.8 s |
| 03 | left turn through a turn pocket | arrived, 48.1 s | arrived, 48.1 s |
| 04 | right turn | arrived, 54.5 s | arrived, 54.4 s |
| 05 | block loop, three right turns | arrived, 114.2 s | arrived, 114.3 s |
| 11 | south-east loop entry, 1.9 m corner | **stalled** 26 m before the end, as the routes file says | **stalled**, same place |
| 12 | west blocks, 1.1 m corner | arrived, 112.8 s (CTE max 2.37 m) | arrived, 112.5 s (CTE max 2.30 m) |

Times are sim seconds from the start of navigation to IDLE. `index.csv.gz` has one row per run with the main numbers. Two things the playground should reproduce, because the real stack does them too:

- The shuttle comes to rest about 2.8 to 2.9 m past the end of the path on routes 01 to 04, and 1.4 to 1.6 m past it on route 05. It reaches the path end at 1.5 to 1.8 m/s and then brakes. Route 12 ends slowly after its corner and rests 0.15 to 0.2 m short. See `arrival_v` and `rest_vs_end_m` in `index.csv.gz`.
- Route 11 stalls in its tight corner with both localization modes.

## GNSS loss

Both cases end in SAFE_STOP, raised by `trajectory_controller`'s own GNSS watchdog.

| Run | Fault | Last fix (t_rel) | beacon_node "GNSS stale" | SAFE_STOP | At rest |
|---|---|---|---|---|---|
| `gnss_outage30s_01` | `gnss_faults:=outage_30s_straight` (route 01, 30 s dropout of both antennas after 40 m) | fault starts 23.75 s | 24.25 s | 24.55 s | 28.5 s |
| `gnss_deadzone_04` | `gnss_dead_zones:=route04_turn` (route 04, no GNSS once base_link is 15° into the turn) | 35.85 s | 36.40 s | 36.70 s | 40.2 s |

In both, beacon_node pauses `/gnss/*` 0.55 s after the last fix and the controller stops 0.3 s later ("GNSS lost while AUTONOMOUS (last /gnss/vel 0.35 s ago) -> SAFE STOP"). The braking from about 1.9 m/s to rest takes about 4 s. `events`, `gnss_fault`, `telemetry` and `odom` show the sequence.

## Determinism

The ROS run is not bit-exact, but it is close. Each node is its own process and DDS delivery timing varies a little, so a frame now and then lands one 50 ms tick earlier or later.

The numbers come from `tools/compare_runs.py`, which uses the same method as `tests/validation/reference_compare.py`. The runs are lined up on the first tick the shuttle moves. Position is compared at the same time, but only while both runs still navigate. Shape is the distance between the two driven lines, each only up to the shorter run's length. All pairs in each group were compared and the table shows the largest value.

| Compared | Position while navigating | Shape | Yaw | Speed | Steering command | Nav duration | Final position |
|---|---|---|---|---|---|---|---|
| Route 04, 3 repeats | ≤ 1.3 cm | ≤ 0.2 cm | ≤ 0.03° | ≤ 0.007 m/s | ≤ 0.2° | ≤ 0.02 s | ≤ 1.3 cm |
| Route 04, 5 runs (3 repeats, the main run, time_factor 1) | ≤ 1.6 cm | ≤ 0.3 cm | ≤ 0.04° | ≤ 0.008 m/s | ≤ 0.2° | ≤ 0.02 s | ≤ 1.4 cm |
| Route 05, 4 runs | ≤ 6.0 cm | ≤ 6.5 cm | ≤ 0.66° | ≤ 0.08 m/s | ≤ 0.5° | ≤ 0.06 s | ≤ 15 cm |
| direct vs ekf, routes 01 to 11 | ≤ 10 cm | ≤ 4.4 cm | ≤ 0.6° | ≤ 0.10 m/s | ≤ 1.0° | ≤ 0.1 s | ≤ 16 cm |
| direct vs ekf, route 12 | 58 cm (ekf is 0.3 s ahead after the tight corner) | 6 cm | 3.9° | 0.28 m/s | 2.8° | 0.3 s | 5 cm |

The route metrics agree even more closely across the route 04 repeats. CTE RMS is the same to 1 mm, CTE max within 2 mm, arrival speed within 3 mm/s and the rest position within 1.5 cm. `time_factor` 1 and 5 give the same results within this spread. The repeats ran three containers at a time, so the spread includes a loaded machine. Route 05 spreads more since the re-recording. In `det_05_b` the shuttle reached the path end 3 cm/s faster and rests 13 cm further on. `det_05_c` started one tick early, and under the load `shuttle_tf_node` sent no transform for about half of its fixes. It now sends the transform only once the heading of the same sample has come, so a fix whose heading comes late is skipped and the controller works on the transform before it. The first recordings had route 05 within 3 cm. Among the references, direct_01 vs ekf_01 differ by one tick at the start (0.05 s).

**Suggested tolerance** for the playground against the `direct_*` runs (its own timing differs from DDS timing, so allow a few times the ROS spread): the same outcome and status sequence, shape within 5 cm, position while navigating within 10 cm, nav duration within 0.2 s, final position within 10 cm, and SAFE_STOP within one 50 ms tick plus 0.1 s of the reference. Route 12 is sensitive after its 1.1 m corner, so compare its shape, not its timing. The `spread/*.json` files hold the measured numbers against the first run of each group.

## Files

```
README.md
index.csv.gz             one row per run: outcome, duration, path length, CTE, arrival, rest, final distance to goal
runs/<label>/            one run
  summary.json           route, spawn, goal, outcome, timing, status sequence, all route_metrics numbers
  odom.csv.gz            ground-truth base_link: t, t_rel, x, y, yaw, v (20 Hz, sim time)
  pose_tf.csv.gz         the pose the stack believes (map -> base_link on /tf)
  cmd.csv.gz             trajectory_controller's 0x101 frames: gear (1 = forward), brake, pedal, steer_deg
  telemetry.csv.gz       /nav_telemetry: status (NAVIGATING, IDLE, SAFE_STOP), target_v, curr_v, dist_end, cte, next_wp
  dbw.csv.gz             the virtual ECU's state: mode, plant, effective gear, pedal, brake, wheel angle
  gnss_fix.csv.gz        /gnss/fix: lat, lon, status
  loc_status.csv.gz      /localization/status (ekf runs only)
  gnss_fault.csv.gz      fault start and end events (fault runs only)
  events.csv.gz          /rosout at WARN and above
  path.csv.gz            the planned path (map frame)
  run.json               sim_route_run's own record (command, outcome)
  route_run.log.gz       sim_route_run's console output
  launch.log.gz          the whole stack's log (gzipped too: the .gitignore drops *.log)
spread/                  compare_runs.py output: repeats of 04 and 05, direct vs ekf per route
tools/
  record_reference.sh    record one run in a fresh container and extract it
  record_batch.sh        several runs, N containers at a time
  extract_reference.py   bag -> the files above (needs ROS)
  compare_runs.py        compare runs, the validation method (numpy only, no ROS)
```

Run labels: `direct_<route>` and `ekf_<route>` are the references. `det_04_*` and `det_05_*` are repeats for the spread. `gnss_*` are the GNSS-loss cases.

**Time.** `t` is sim time. `t_rel` is 0 at the stamp of the controller's first forward 0x101 frame, which is when navigation starts. That stamp is exact sim time, so `t_rel` lines up between runs. Odom, cmd, TF and GNSS times come from message stamps. Telemetry, ECU state and log lines carry no sim stamp, so their receive time is mapped to sim time through the odom stamps, which is good to about one tick. Before the start the shuttle stands still, so values at negative `t_rel` are not compared. The shuttle stays still for the first 0.3 s after t_rel 0 while the brake releases.

**Format.** Every table is a gzipped CSV (`<name>.csv.gz`, written with mtime 0 so the bytes only change when the rows do). The repo's `.gitignore` drops `*.csv`, and the plain files were 30 MB. Read one with `gzip.open(path, 'rt')` and `csv.DictReader`, or `zcat`. The readers here and in `tests/validation/` take `<name>.csv.gz` first and fall back to `<name>.csv`, which is what `playground.cli` writes.

The raw bags are not kept here (145 MB). They can be recorded again with the commands below.

## Commands

```bash
# workspace (once)
WS=$(mktemp -d); mkdir -p $WS/ws/src
git ls-files --cached --others --exclude-standard | grep -v '^_local_no_commit/' \
  | rsync -a --files-from=- . $WS/ws/src/
docker run --rm -v $WS/ws:/ws nift_pub_deps:humble bash -c \
  'source /opt/ros/humble/setup.bash && cd /ws && colcon build --symlink-install'

T=nift_playground/tests/reference/tools
# references, three containers at a time
$T/record_batch.sh $WS/ws $WS/bags 3 direct_01:01 direct_02:02 direct_03:03 direct_04:04 direct_05:05 direct_11:11 direct_12:12
$T/record_batch.sh $WS/ws $WS/bags 3 ekf_01:01:localization:=ekf ekf_04:04:localization:=ekf   # and so on
# GNSS loss
$T/record_batch.sh $WS/ws $WS/bags 2 "gnss_outage30s_01:01 --time-factor 1:gnss_faults:=outage_30s_straight" \
  "gnss_deadzone_04:04 --time-factor 1:gnss_dead_zones:=route04_turn"
# spread
python3 $T/compare_runs.py nift_playground/tests/reference/runs/det_04_a nift_playground/tests/reference/runs/det_04_b
```

Each run gets its own container with `--network none`. So runs in parallel don't see each other over DDS, and `sim_route_run`'s check for another running stack only sees its own container. A route takes 20 to 40 s of wall time at `time_factor` 5, and about 70 s for route 04 at `time_factor` 1.

## Issues seen while recording

Nothing in the packages was changed. These go to the report.

- **Shutdown tracebacks.** Every run ends with KeyboardInterrupt tracebacks inside `destroy_node()` of `trajectory_controller`, `waypoint_node`, `beacon_node` and `shuttle_tf_node` (exit code -2). `sim_route_run` sends SIGINT to the whole process group and `ros2 launch` forwards its own SIGINT, so each node most likely gets two. Harmless for the results, but noisy in `launch.log`.
- **Not a bug: the beacon_node "disconnect handler error".** In 3 runs (direct_04, ekf_01, gnss_deadzone_04) `launch.log` shows a TypeError from `_on_disconnect()`, which takes no `reason` argument. python-socketio 5.17 handles that. It catches the TypeError and calls the handler again without the argument (the legacy path in `Client._trigger_event`). In those 3 runs the second SIGINT arrived during that retry, inside the logger call, so the log shows the TypeError as the context of a KeyboardInterrupt. It is the shutdown issue above, not a socketio incompatibility. Adding a `reason=None` parameter would still make the log clearer.
- **`sim_route_run` records no git state** when the workspace is a copy without `.git` (`run.json` has `"git": {}`). The repo state is in this README instead.
