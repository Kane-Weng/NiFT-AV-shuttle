# nift_playground architecture (the contract)

Owner: the architect (wave A, 2026-10-20). Every other agent codes against this file. If you need a change, propose it in your report and say which section. This file wins over `NIGHT_PLAN.md`, `plan.md` and the draft in `web/src/protocol.ts`.

Ground rules (from `NIGHT_PLAN.md`): nothing is committed, our nodes run unchanged, the map is `maps/lanelet2_odr2024.osm`, the code runs on Python 3.10 (Humble) and 3.12 (Pyodide), and nothing reads the private repo.

## 1. Layout

```
nift_playground/
  ARCH.md  README.md  COLCON_IGNORE
  py/                       sys.path root for the `fakeros` and `playground` packages
    fakeros/                package `fakeros` (core) AND sys.path root for rclpy/, tf2_ros/, diagnostic_updater/
      _core.py              spike code (wave A copy), the base for the fakeros agent
      _generated/           msgs agent: spec.json and the message runtime; fakeros.install() calls _generated.install()
      _msgs.py msg_spec.json   the spike's message loader, unused since wave A, deleted by the fakeros agent
      rclpy/ tf2_ros/ diagnostic_updater/
    fakelanelet2/           sys.path root: lanelet2/ (io, projection, core, geometry, routing, traffic_rules)
    fakesocketio/           sys.path root: socketio/ (client stand-in and the in-process hub)
    stubs/                  sys.path root: pyproj/, aiohttp/, dotenv/ (tiny import stand-ins, section 5)
    playground/             package `playground`: paths, launch set, sim backend, Octane bridge, runner, session API
  web/                      Vite + TypeScript page, three.js view, panels, the Pyodide worker
  tools/                    gen_msgs/ (msgs agent), build_bundle.py, bundle manifest
  tests/                    pytest (native), Node + Pyodide tests, Lanelet2 equivalence, browser e2e
    spike/                  the Phase 0 spike loop, kept as the scaffold's smoke test
```

Change from `NIGHT_PLAN.md`: `py/stubs/` is new. It holds three import stand-ins that are not ROS, Lanelet2 or socket.io (section 5).

`playground.paths.setup(repo_root)` puts these on `sys.path`, in this order, first: `py/fakeros`, `py/fakelanelet2`, `py/fakesocketio`, `py/stubs`, `py`, then the repo packages `nift_common`, `nift_sensor`, `nift_waypoint`, `nift_control`, `nift_simulation/nift_carla`, `nift_bringup`. Natively `repo_root` is the checkout. In Pyodide it is `/nift`, where the bundle unpacks the same repo-relative paths (section 9). Then it calls `fakeros.install()`. Natively the stand-ins shadow a real ROS install, so never source ROS in a playground test shell.

## 2. The five runtime nodes, unchanged

| launch name | class (module) | used for |
|---|---|---|
| `kinematic_sim` | `KinematicSimNode` (`nift_carla.kinematic_sim_node`) | the vehicle: virtual ECU, plant, bicycle, GNSS, Octane, `/clock` |
| `beacon_node` | `BeaconNode` (`nift_sensor.beacon_node`) | Octane client, feed checks, `/gnss/*` |
| `shuttle_dynamic_broadcaster` | `ShuttleDynamicBroadcaster` (`nift_sensor.shuttle_tf_node`) | TF `map -> base_link` (direct, shadow) |
| `localization_node` | `LocalizationNode` (`nift_sensor.localization_node`) | the EKF (ekf, shadow) |
| `waypoint_node` | `WaypointNode` (`nift_waypoint.waypoint_node`) | Lanelet2 routes on `/planned_path` |
| `trajectory_controller` | `TrajectoryController` (`nift_control.trajectory_controller`) | Pure Pursuit, ADRC, watchdogs, CAN |

`shuttle_dynamic_broadcaster` and `localization_node` are the two providers of `map -> base_link`. Which one runs is the localization mode (section 6), so a run has five or six nodes.

### 2.1 Can `kinematic_sim_node.py` run unchanged? Yes

We construct `KinematicSimNode()` from the repo file and never call its `start()`, `main()` or `_run()`. Two things change around it, and nothing inside it.

1. **The Octane server.** `__init__` builds `OctaneEmulator(...)` and calls `.start()`, which starts an aiohttp server in a thread. Pyodide has no threads or sockets. Before constructing the node, the sim backend sets the module attribute `nift_carla.kinematic_sim_node.OctaneEmulator = playground.octane_bridge.BrowserOctane`. `BrowserOctane` has the same constructor (`port, api_key, namespace, host, log`), the same `start()`, `publish(payloads)`, `stop()`, and the same `port`, `emitted` and `clients_authed` attributes that the diagnostics read. It registers itself on the fake socket.io hub at `http://127.0.0.1:<port>` and plays the protocol of `octane_emulator.py` (auth with the key, `auth_ok`, join channels, `beacon_update` to the `beacon` channel). The payloads are still built by the repo's `sim_gnss.GnssSim` and `octane_emulator.beacon_payload`, so knots, compass heading and ISO stamps are the real code paths.
2. **The tick thread.** The runner calls `node._tick_once()` once per tick, the method the thread would call. With `time_factor 0` it never sleeps, and with `lockstep false` it never waits on its `Condition`. `threading.Lock`, `Condition`, `Thread` (constructed, never started) and `queue.Queue` all work in Pyodide without threads.

The module still imports `aiohttp` and `socketio` (through `octane_emulator.py`) and `pyproj` (through `carla_geo.py`). Section 5 lists the stand-ins. `CarlaGeo` uses the Lanelet2 stand-in's `UtmProjector` for map to WGS84, as with real Lanelet2. Its pyproj transformers are built but only used for CARLA world coordinates, which the kinematic sim never asks for.

So there is no `web_sim_node.py`. `tests/spike/web_sim_node.py` stays only as the spike's smoke test. The sim-gnss agent puts the two changes above in `playground/sim_backend.py` (construct, tick, read-only accessors for the page) and `playground/octane_bridge.py`.

Private names the wrapper touches, all read or called, never patched in place: `kinematic_sim_node.OctaneEmulator` (module attribute, replaced before construction), `KinematicSimNode._tick_once()`, `KinematicSimNode._sim_ns` and `.dt` (asserted against the world clock), `KinematicSimNode.gnss` (only for live GNSS faults, section 8.4). A native test fails loudly if any of them disappears.

### 2.2 Why the spike's shortcuts go away

The spike fed TF and `/gnss/vel` straight from the true pose. Now the chain is the real one: `GnssSim` -> `beacon_payload` -> `BrowserOctane` -> fake socket.io -> `beacon_node` (feed checks) -> `/gnss/fix`, `/gnss/heading_deg`, `/gnss/vel`, `/gnss/raw/*`, `/gnss/hygiene` -> `shuttle_tf_node` or `localization_node` -> TF -> `waypoint_node` and `trajectory_controller`.

## 3. ROS APIs the nodes use (scope for fakeros and msgs)

Everything below must behave like Humble's rclpy. "kin" is kinematic_sim, "bcn" beacon_node, "tf" shuttle_tf_node, "loc" localization_node, "wp" waypoint_node, "ctl" trajectory_controller (with its modes, filters, controllers and dbw_interface).

### 3.1 rclpy

| API | used by | notes |
|---|---|---|
| `rclpy.init(args=None, signal_handler_options=...)`, `ok()`, `shutdown()`, `try_shutdown()` | all `main()` | `main()` is never called, but the names must import |
| `rclpy.signals.SignalHandlerOptions` | kin (`main`, inside try) | |
| `rclpy.executors.SingleThreadedExecutor`, `ExternalShutdownException` | kin, bcn, tf, loc, wp, ctl | import only |
| `Node(name, **kwargs)` | all | kwargs seen: `parameter_overrides` (tests). The launch name remap applies (section 6) |
| `declare_parameter(name, default)` and `(name, default, ParameterDescriptor(dynamic_typing=True))` | all | rclpy typing: the default's type, unless `dynamic_typing`. A YAML override of the wrong type raises, as on ROS. Defaults seen: float, int, bool, str, `[]`, list of float, list of str |
| `declare_parameters(namespace='', parameters=[(name, default), ...])` | ctl | |
| `get_parameter(name).value` | all | `.value` only |
| `create_publisher(type, topic, qos)` | all | qos is an int depth, `qos_profile_sensor_data` or a `QoSProfile` |
| `create_subscription(type, topic, cb, qos)` | all | callbacks may be lambdas |
| `create_service(Trigger, name, cb)` | kin, ctl | relative names resolve to `/name` |
| `create_timer(period_s, cb)` | bcn (0.02), loc (1/50), wp (1.0), ctl (0.1, 0.05), Updater (1.0) | section 7 for when they fire |
| `get_clock().now()` -> `Time`; `.nanoseconds`, `.to_msg()`, `Time - Time` -> `Duration` (`.nanoseconds`) | all | |
| `rclpy.time.Time()`, `Time(nanoseconds=)`, `Time(seconds=, nanoseconds=)`, `Time.from_msg(stamp)` | kin, bcn, loc, wp, ctl | `Time()` is "latest" for TF lookups |
| `get_logger().debug/info/warn/warning/error/fatal(msg, throttle_duration_sec=)` | all | throttle per call site, on the node clock |
| `rclpy.logging.get_logger(name)` | ctl (`can_interface.py`, module level) | must keep working after a reset |
| `rclpy.qos`: `QoSProfile(history, depth, reliability, durability)`, `HistoryPolicy.KEEP_LAST`, `ReliabilityPolicy.RELIABLE`, `DurabilityPolicy.TRANSIENT_LOCAL`, `qos_profile_sensor_data` | bcn, tf, loc, wp, ctl | tf reaches it as `rclpy.qos.qos_profile_sensor_data` after `import rclpy` |
| `destroy_node()` | all `main()` | |

Not used by any of the five: clients, actions, `add_on_set_parameters_callback`, `count_subscribers`, `wait_for_service`, callback groups, multi-threaded executors. So a `set_param` from the page cannot change a running node (section 8.3).

### 3.2 tf2_ros

| API | used by |
|---|---|
| `TransformBroadcaster(node)`, `.sendTransform(TransformStamped)` | tf, loc |
| `Buffer()`, `TransformListener(buffer, node)` | wp, ctl (ctl imports them as `tf2_ros.buffer.Buffer` and `tf2_ros.transform_listener.TransformListener`) |
| `buffer.lookup_transform('map', 'base_link', rclpy.time.Time())` | wp, ctl. Latest transform. A missing one raises `TransformException` (ctl catches that, wp catches `Exception`) |
| `tf2_ros.TransformException` | ctl |

Nothing publishes `/tf_static` in the playground (no robot_state_publisher, section 6.3).

### 3.3 diagnostic_updater

`Updater(node)` (period 1.0 s), `.setHardwareID(str)`, `.add(name, fn)`. The task gets a `DiagnosticStatusWrapper` and uses `.summary(level, message)` and `.add(key, value)`, and returns it. Levels are `diagnostic_msgs.msg.DiagnosticStatus.OK/WARN/ERROR`, which are `bytes` in rclpy (`b'\x00'`). Used by all five. The fakeros agent checks the real Updater's status name format (`"<node>: <task>"`) and its period parameter in the Humble image.

### 3.4 std_srvs and messages

| package | types | by |
|---|---|---|
| std_srvs | `Trigger` (`.Request()`, `.Response()` with `success`, `message`) | kin, ctl |
| std_msgs | `String`, `Bool`, `Float64`, `Header`, `ColorRGBA` | kin, bcn, tf, loc, wp, ctl |
| geometry_msgs | `TransformStamped`, `TwistStamped`, `Twist`, `PoseStamped`, `PoseWithCovarianceStamped`, `PolygonStamped`, `Point`, `Point32` | all |
| nav_msgs | `Odometry`, `Path` | kin, loc, wp, ctl |
| sensor_msgs | `NavSatFix` (const `COVARIANCE_TYPE_UNKNOWN`), `NavSatStatus` (`STATUS_FIX`, `STATUS_NO_FIX`, `SERVICE_GPS`) | bcn, tf, loc, wp |
| diagnostic_msgs | `DiagnosticStatus` (bytes `level`, consts), `KeyValue`, `DiagnosticArray` | all |
| visualization_msgs | `Marker` (kwargs constructor, consts `ADD`, `DELETEALL`, `LINE_LIST`, `SPHERE`, ...), `MarkerArray` | wp, ctl |
| can_msgs | `Frame` | kin, loc, ctl |
| rosgraph_msgs | `Clock` | kin |
| rcl_interfaces | `ParameterDescriptor(dynamic_typing=True)` | kin, bcn, loc |
| builtin_interfaces | `Time`, `Duration` | ctl (`can_interface.py`), fakeros |
| tf2_msgs | `TFMessage` | fakeros tf2_ros |

The msgs agent's `py/fakeros/_generated/` provides these (160 types from 14 packages, checked against real rclpy in `tests/test_msgs.py`). `fakeros.install()` calls `_generated.install()` and nothing else, since the spike's `_msgs.py` claims the same names. Message rules that matter: kwargs constructors, nested defaults, `byte` fields and constants as `bytes` (`DiagnosticStatus.OK == b'\x00'`), fixed arrays as `FixedArray` (`Frame.data`, covariances), unbounded numeric arrays as `array.array`, deep copy on publish. rclpy asserts field types on assignment. The fake does the same in native tests (strict mode), and may switch it off in the browser for speed if the perf agent shows it matters.

## 4. Lanelet2 calls (scope for the lanelet2 agent)

| call | where |
|---|---|
| `lanelet2.io.Origin(lat, lon)` | wp, tf, loc, `carla_geo` |
| `lanelet2.projection.UtmProjector(origin)` with `.forward(GPSPoint) -> BasicPoint3d` (`.x .y .z`) | wp, tf, loc, `carla_geo` |
| `UtmProjector.reverse(BasicPoint3d) -> GPSPoint` (`.lat .lon .alt`) | `carla_geo.map_to_wgs` (the sim's GNSS) |
| `lanelet2.core.GPSPoint(lat, lon, alt)`, `BasicPoint2d(x, y)`, `BasicPoint3d(x, y, z)` | wp, tf, loc, `carla_geo` |
| `lanelet2.io.load(path, projector) -> LaneletMap` | wp |
| `map.laneletLayer`: `len()`, iteration | wp, visualizer |
| `lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle)` | wp |
| `lanelet2.routing.RoutingGraph(map, rules)` | wp |
| `graph.getRoute(start, goal, 0)` -> route or `None`; `route.shortestPath()` -> path (`len`, iteration) or `None` | `LaneletPlanner.find_feasible_lanelet_path` |
| `graph.following(lanelet, withLaneChanges)` | `LaneletPlanner.search_alternative_feasible_path` (not called by waypoint_node today; cheap to keep) |
| `lanelet2.geometry.findNearest(map.laneletLayer, BasicPoint2d, 1)` -> `[(distance, lanelet)]` | wp |
| `lanelet.id`, `.centerline` (points with `.x .y`), `.leftBound`, `.rightBound` | planner, visualizer |

`UtmProjector` uses the origin's UTM zone and subtracts the origin (Lanelet2's `useOffset` default), so `forward(origin)` is (0, 0). Forward and reverse must round-trip to well under a millimetre, because the GNSS chain goes map -> WGS84 (sim) -> map (tf, loc, wp). The centerline must match Lanelet2's computed centerline, and the routing graph must match Lanelet2's for `Germany`/`Vehicle` (successors, lane changes, routing cost 0 = distance). The equivalence test against real Lanelet2 is the proof.

## 5. socket.io and the other import stand-ins (scope for sim-gnss)

`beacon_node` uses exactly these python-socketio client calls:

- `socketio.Client(reconnection=True, reconnection_attempts=0, reconnection_delay=1)`
- `@sio.on(event, namespace=ns)` for `connect`, `auth_ok`, `beacon_update`, `v2x_BSM`, `disconnect`
- `sio.emit('auth', {'x-api-key': key}, namespace=ns)`, `sio.emit('join', {'channel': 'beacon'|'v2x_obu_parsed'}, namespace=ns)`
- `sio.connect(url, namespaces=[ns])`, `sio.connected`, `sio.disconnect()` (in `main()`)

`octane_client.probe()` also uses `socketio.Client(reconnection=False)` and `connect(..., wait_timeout=)`, but only the bringup preflight calls it. Keep the signature, nothing more.

How it is fed. `socketio` (in `py/fakesocketio/`) holds an in-process hub keyed by URL. `BrowserOctane` registers as the server at `http://127.0.0.1:8765` (`carla_params.yaml` `octane.port`). `connect()` to an unknown URL raises `socketio.exceptions.ConnectionError`, as the real client does on the first try. Every delivery (connect handshake, `auth_ok`, `beacon_update`) goes onto the fakeros world's callback queue (`world.call_soon(fn, *args)`), so it runs after the current callback, in order, like a message. Nothing is delivered inside `publish()`. A wrong key gets no reply, as in `octane_emulator.py`.

`beacon_node` reads its server, key, namespace and beacon IDs from `octane_client.octane_config()`, which calls `dotenv.dotenv_values` on a `.env` it looks for and lets the environment win. The launch set (section 6) puts all five `MCITY_OCTANE_*` keys in `os.environ` before it builds `beacon_node`, from `carla_params.yaml` `octane`, exactly as `nift_bringup/components/gnss.py` does.

The stand-ins in `py/stubs/`:

| module | why | contents |
|---|---|---|
| `dotenv` | `octane_config()` imports `dotenv_values` | `dotenv_values(path)` parses `KEY=VALUE` lines, `{}` for `None` |
| `aiohttp` (`aiohttp.web`) | `octane_emulator.py` imports `from aiohttp import web` | empty module; any use raises `NotImplementedError` |
| `pyproj` | `carla_geo.py` imports it and builds transformers | `CRS.from_proj4`, `Transformer.from_crs` construct; `.transform()` raises `NotImplementedError` (only CARLA world conversions call it) |

`socketio.AsyncServer` exists as a name that raises on use, for the same reason. Natively the real python-dotenv may shadow the stub. That is fine, but the bundle must never contain a `.env` file (README section 3: no credentials).

## 6. The launch set (mirrors `sim_kinematic`)

`playground/launch_set.py` (runner agent) builds the node list the way `system.launch.py` does for `profile:=sim_kinematic`, with `time_factor:=0 lockstep:=false` and `rviz:=false`. It gets the component set from the repo's own `nift_bringup.profiles.resolve(load('nift_bringup/config/profiles.yaml'), 'sim_kinematic', cli)`. That module has no launch imports, so it runs in the browser too. `cli` carries the page's launch options (6.4). The localization mode is `plan.localization_mode`, the one source of truth, as in bringup.

### 6.1 Nodes, parameters, remaps

Parameter sources are applied in the order listed. Each source is a YAML file (its top-level key is a node name or `/**`) or a dict (launch_ros writes a dict as a `/**` file). Section 6.2 has the precedence rule. Paths are repo-relative.

| # | node (launch name) | sources | other |
|---|---|---|---|
| 1 | `kinematic_sim` | dict: `carla_params.yaml` `carla_bridge.ros__parameters`, flattened; dict: `{map_origin_lat, map_origin_lon}`, `spawn.mode: map` + `spawn.x/y/yaw` when a spawn is given, `time_factor: 0.0`, `lockstep: false`, plus `nift_carla.sim_gnss.launch_overrides(gnss_feed, gnss_dead_zones, gnss_seed, sets, gnss_faults, catalogue)` with `config/gnss_dead_zones.yaml` and `config/gnss_faults.yaml` | the `OctaneEmulator` swap (2.1). This is `kinematic_sim.launch.py`'s `_build()` |
| 2 | `beacon_node` (node's own name `nift_beacon_node`) | `nift_sensor/config/beacon_node.yaml`; `{use_sim_time: true}`; `{hygiene.enabled: gnss_checks == 'true'}` | env: the five `MCITY_OCTANE_*` from `carla_params.yaml` `octane` |
| 3 | `shuttle_dynamic_broadcaster` (direct, shadow) | `nift_sensor/config/shuttle_tf.yaml`; `{use_sim_time}`; `{map_origin_lat, map_origin_lon}`; `{gnss.offset_x: ShuttleConfig().gnss_front_x}` | |
| 4 | `localization_node` (ekf, shadow) | `nift_sensor/config/localization.yaml`; `{use_sim_time}`; node-scoped `{mode, antenna.front_x_m, antenna.rear_x_m, map_origin_lat, map_origin_lon, gnss_delay_s, speed_point_x_m}` | `components/localization.py` `filter_params()`: `gnss_delay_s` 0.061 on the real feed model else 0.0, `speed_point_x_m = -base_link_x` (kinematic) |
| 5 | `waypoint_node` | `nift_waypoint/config/waypoint_params.yaml`; `{use_sim_time}`; `{map_origin_lat, map_origin_lon}`; `{map_file: <root>/maps/lanelet2_odr2024.osm}` | see 6.3 on `map_publish_period_s` |
| 6 | `trajectory_controller` | `{use_sim_time}`; `nift_control/config/planning.yaml`; `nift_control/config/adrc_params.yaml`; `{lateral_enabled: true}`; node-scoped `{localization_source, filter_type}` | `localization_source` is `filter` in ekf, else `gnss`. `filter_type` is `plan.args['filter_type']` (`none` without perception) |

Construction order is the launch's order of readiness. kinematic_sim, then beacon_node ("Octane emulator listening"), then 3 and/or 4, then the runner drains the world until beacon_node has logged `auth_ok` (zero sim time passes), then waypoint_node, then trajectory_controller ("planning and control start once beacon_node reports auth_ok"). Then the playground's own nodes (6.3). Timer ties go by creation order (section 7), so this order is part of the contract.

`map_origin_lat/lon` come from `plan.args` (defaults `nift_common.site`). The spawn comes from the page or a route of `nift_carla/config/test_routes.yaml`, else `carla_params.yaml`'s (101.52, 162.66, -1.612).

### 6.2 Parameter precedence (what fakeros must reproduce)

rcl merges all parameter files into one table of node-name keys, in the order each key first appears across the files. Within one key a later file wins. rclpy then applies the keys that match the node (`/**`, `name`, `/name`) in that table order, so a later key wins over an earlier one. Example: for trajectory_controller, `{use_sim_time}` creates the `/**` key first, so a `filter_type` in a later dict would lose to `planning.yaml`'s node-scoped value. That is why bringup writes a node-scoped override file (`components/common.py` `node_params`), and why the table above marks some sources node-scoped. Nested YAML maps flatten to dotted names (`plant.a_lim`). A node ignores values it never declares.

The launch name replaces the node's own name for parameters, logging and diagnostics (`nift_beacon_node` becomes `beacon_node`). fakeros gets a remap table `{own name: launch name}` from the launch set.

### 6.3 Playground deviations and extras (each one is deliberate)

- `time_factor 0`, `lockstep false`. These are launch arguments. The runner's fixed order (section 7) already gives what lockstep gives, a command age of 0.
- `gnss_seed` defaults to 1 in the playground. `0`, carla_params' value, means a new random seed each run, which breaks replay. Only the real-feed model and noise draw random numbers.
- `rviz` is off and robot_state_publisher does not run. The page draws instead, and none of the five nodes reads the URDF frames.
- `waypoint_node` `map_publish_period_s`. It rebuilds about 70k `Point` messages for `/lanelet_map_markers` every second, and nothing in the playground subscribes. The perf agent measures it first. If it costs more than a few percent at 10x, the launch set overrides it to 3600 s, the only parameter override of a repo node, listed in the page's "about" box.
- Playground nodes, ours, not repo code: `playground_operator` (publishes `/goal_pose`, `/initialpose`, `/teleop_cmd_vel`, the 0x100 frame for `dbw_estop`, calls the Trigger services), `playground_observer` (subscribes like the web dashboard, section 8.2), and, only with the `perception` option, `playground_perception`, which publishes `/obstacles` from the page's obstacles the way `lidar_node` does (base_link frame, one `PolygonStamped` per obstacle with one shared stamp, an empty polygon to clear, 10 Hz, only what lies in the sensor's range and field of view, from the ground-truth pose). With `perception` on, `filter_type` is `planning.yaml`'s (`cbf`) or the page's choice. `nift_bringup.profiles.resolve` refuses perception on the kinematic backend, so the runner resolves with perception off and applies this extra on top.

### 6.4 Launch options (the page's switches)

| option | values | maps to |
|---|---|---|
| `localization` | `direct` (default), `ekf`, `shadow` | `localization:=<mode>` |
| `gnss_feed` | `ideal` (default), `real` | `gnss_feed:=` |
| `gnss_faults` | `''`, a scenario name of `gnss_faults.yaml`, or a list of fault dicts | `gnss_faults:=` (a list goes in as a temporary YAML file) |
| `gnss_dead_zones` | `''` or set names | `gnss_dead_zones:=` |
| `gnss_seed` | int, default 1 | `gnss_seed:=` |
| `gnss_checks` | `true` (default), `false` | `gnss_checks:=` |
| `perception` | `false` (default), `true` | playground extra (6.3) |
| `filter_type` | `''`, `distance_stop`, `cbf`, `potential_field`, `none` | `filter_type:=` (only with perception) |
| `spawn` | `{x, y, yaw}` or a route id `'01'`..`'12'` | `spawn_x/y/yaw:=` |
| `params` | `{node: {name: value}}` | node-scoped override source appended last for that node |

Changing any of them restarts the stack (`reset`).

## 7. The deterministic runner

One process, one sim clock, one thread. Sim time starts as `kinematic_sim_node` does: the first tick is at `START_NS = 1 s`, and every tick adds `dt = fixed_delta_seconds` (0.05 s). `t` on the page is sim time minus 1 s.

**Before the first tick.** The world clock is `START_NS - dt`, which equals the sim node's `_sim_ns` before its first tick. Nodes are constructed then, so a timer of period `p` is first due at `START_NS - dt + p`. (On ROS the clock is 0 until the first `/clock`. Only start-up ages differ, and those are huge either way.)

**One tick, `runner.step()`:**

1. `world.now_ns = t = previous + dt`.
2. `sim.tick()` calls `KinematicSimNode._tick_once()`. It applies the command in force over the last `dt`, publishes `/carla/dbw_state`, hands the GNSS payloads to `BrowserOctane`, publishes odom and `/clock`. The runner asserts `node._sim_ns == t`. What it sends is held in the queue.
3. Timers: while a timer is due at or before `t`, pop the one with the smallest (due, creation sequence), call it, and reschedule it by rcl's rule: `next = due + period`, and if `next < t`, skip ahead by whole periods to the first `next >= t`. Nothing is delivered in between. Every callback sees `now() = t`. So a timer faster than the tick fires 1 or 2 times per tick. beacon_node's 50 Hz timer under a 20 Hz `/clock` alternates 1 and 2 fires (30 Hz on average), which matches the reference ROS runs (`tests/reference/`), and a timer whose next due time equals `t` fires again in the same tick, as rcl would.
4. `world.drain()`: deliver everything, FIFO, cascading, each callback to completion. The sim's messages and socket.io events come first, then what the timers sent.
5. The observer has seen everything. The session queues outgoing messages (section 8).

This is the ROS order. On ROS every node is its own process, so when `/clock` reaches `t` every due timer fires on what had already arrived, and what the sim and the timers send at `t` arrives afterwards. trajectory_controller therefore acts on the GNSS fix of `t - 2 dt` (sim, then beacon_node's timer, then TF, then the controller's timer). `tests/validation/README.md` shows it: with this order route 02 matches ROS to 0.0 mm over 105 s, routes 01 to 05 and 11 stay within 3.1 cm in direct and ekf (the ROS repeat spread is 3.0 cm), and all 16 cases meet the validation bounds. The first version of this contract delivered the sim's messages before the timers and drained after each timer. That put the controller on the fix of `t` itself and the shuttle 13 to 24 cm ahead of ROS.

The controller's frame for time `t` is delivered to the virtual ECU in step 4 of tick `t` and moves the shuttle in step 2 of tick `t + dt`. That is lockstep's timing, with no wall-clock wait.

fakeros implements steps 2 to 4 as `world.step(t_ns, pre=callable)`, where `pre` is the sim tick, and steps 3 and 4 as `world.fire_timers(t_ns)`. The spike's `World.run_until`, which fires each timer at its own due time and drains after each, is kept only for `tests/spike`.

**Page commands** are applied between ticks, at the current `t`, followed by a drain. A command that arrives during a tick waits for it to end. Each applied command is logged with its tick index, which makes replay exact (8.5).

**Speed, pause, step.** The worker's JavaScript loop owns wall-clock pacing. Each loop turn it computes how many ticks are owed (`wall elapsed x speed / dt`, capped by a time budget of about 12 ms per turn so the worker stays responsive), calls `session.advance(n)`, posts the messages it returns, and yields (`setTimeout(0)` or a `MessageChannel`). `pause` stops ticking. `step {n}` runs exactly `n` ticks (default 1) while paused. `speed {factor}` sets the factor (0.1 to 100, or `'max'` for as fast as the budget allows). The achieved real-time factor goes out in `state.rtf`. Native tests call `session.advance(n)` with no pacing at all.

**Determinism.** The same launch options and the same command log give the same trace, natively and in Pyodide (measured agreement about 3e-9 m on the full stack with the GNSS and EKF chain, because the maths libraries differ; the spike alone agreed to 1e-12 m). Wall-clock values exist only in diagnostics strings (`Tick wall`, `Real-time factor`) and `state.rtf`, never in the control path. Compare traces without them.

**Reset** throws the world away (`world.reset()`), keeps the interpreter and the imported modules, and rebuilds the launch set. Phase 0 measured a flat heap over 50 resets. Module-level objects (for example `can_interface.logger`) must keep working across a reset, so fakeros keeps one `world` object and resets it in place.

## 8. Worker protocol

JSON objects with a `type` field, posted with `postMessage` both ways. Units are SI and the `map` frame (x east, y north, metres, yaw in radians counter-clockwise from +x). The Python side lives in `playground/session.py` (runner agent): `Session.handle(json_text) -> json_text` (a list of messages) and `Session.advance(n) -> json_text`, so native tests speak exactly what the worker speaks. `web/src/protocol.ts` is the TypeScript mirror and must match this section.

Any page message may carry `id` (a string). The worker answers it with a `reply`.

### 8.1 Page to worker

| type | fields | effect |
|---|---|---|
| `init` | `baseUrl`, `mapUrl`, `bundleUrl?` (the Python bundle; default next to the page), `launch?` (6.4), `scenario?` (8.5), `stateHz?` (default 10) | load Pyodide, the bundle and the map, build the launch set, post `progress` then `ready` |
| `goal` | `x`, `y`, `yaw?` | `playground_operator` publishes `PoseStamped` on `/goal_pose` (frame `map`, yaw as the orientation, the planner ignores it) |
| `cmd` | `name` | `start` -> `/start_navigation`; `estop` and `stop` -> `/trigger_estop` (SAFE_STOP, the same service as nift_web's E-STOP button; the stack has no softer stop); `resume` -> `/resume`; `manual` -> `/toggle_manual_mode`; `dbw_estop` -> a 0x100 E-stop frame on `/to_can_bus` (the virtual ECU latches and holds the brake); `power_cycle` -> `/carla/dbw_power_cycle`. The service's `success` and `message` come back in `reply` |
| `teleop` | `v`, `steer` | `/teleop_cmd_vel` `Twist` (`linear.x = v`, `angular.z = steer`, as the controller reads it), MANUAL mode only |
| `initialpose` | `x`, `y`, `yaw` | `/initialpose` (the sim teleports, waypoint_node takes it as the start) |
| `pause`, `resume_sim` | | stop or restart pacing (worker only, not logged) |
| `step` | `n?` | run `n` ticks (default 1) |
| `speed` | `factor` (number or `'max'`) | pacing factor |
| `set_param` | `node`, `name`, `value` | allowed only for the whitelist in `web/src/panels/presets.ts` (8 `trajectory_controller` parameters). Staged into the launch option `params`. The nodes read parameters only at start-up, so it takes effect on the next `reset`. The reply says so |
| `obstacle_add` | `oid`, `poly` (list of `[x, y]`) or `box` (`{x, y, yaw, length, width}`) | add to the world's obstacles |
| `obstacle_move` | `oid`, `x`, `y`, `yaw?` | move it (translation of its centroid, optional rotation) |
| `obstacle_remove` | `oid` | remove it |
| `set_obstacles` | `obstacles` (list of `{oid, poly}`) | replace them all (the draft's message) |
| `gnss_fault` | `preset`, `params?` | section 8.4 |
| `reset` | `launch?` (partial 6.4) | rebuild the stack with these options. The page's obstacles survive a reset (logged as a `set_obstacles` at tick 0), and so does a pause |
| `scenario_save` | | reply with a `scenario` message |
| `scenario_load` | `scenario` | reset with its launch options and replay its commands at their ticks |

Obstacles only reach the stack when the `perception` option is on. Otherwise the page draws them and says the safety filter is `none` in `sim_kinematic`.

### 8.2 Worker to page

| type | when | fields |
|---|---|---|
| `progress` | during `init` and `reset` | `stage`, `pct` |
| `ready` | after `init` and every `reset` | `pyodide`, `python`, `launch` (resolved options and the node list with each node's parameter sources), `presets` (`gnss_faults`: the catalogue names plus the live presets, `routes`: test routes 01 to 12 with spawn and goal) |
| `state` | at `stateHz` while running, after every `step` and every command | below |
| `path` | when `/planned_path` changes | `points` (`[x, y]` list, empty clears), `t` |
| `goal_ack` | the first `/planned_path` after a `goal` | `x`, `y`, `ok` (`points >= 2`), `reason?` (the planner's last warning) |
| `log` | as nodes log (batched per `advance`) | `t`, `level` (`debug/info/warn/error/fatal`), `node`, `text` |
| `diag` | when `/diagnostics` brings news (about 1 Hz) | `items`: latest per status name, `{name, hardwareId, level 0..3, message, values}` |
| `obstacles` | when the set changes | `polygons` (world, map frame, with `oid`), `perceived` (what the last `/obstacles` held, in map frame, or `null` without perception) |
| `event` | as they happen | `kind`: `gnss_fault` (from `/carla/gnss_fault`), `gps_health` (`/carla/gps_health`), `mode` (STATUS changed), `arrived` (NAVIGATING to IDLE); `t`; `data` |
| `reply` | answers a message with `id` | `id`, `ok`, `message` |
| `scenario` | answer to `scenario_save` | `scenario` |
| `error` | Python exception | `message`, `traceback`, `fatal` |

`state`:

```
{ type: 'state', t, tick, paused, factor, rtf,
  pose: {x, y, yaw} | null,     ground truth (/ackermann_like_controller/odom), null until the first odom
  poseEst: {x, y, yaw} | null,  what the stack believes (TF map -> base_link)
  speed, steer,                 truth m/s (null until the first odom); front wheel angle rad (/carla/dbw_state applied.wheel_deg)
  status,                       /nav_telemetry STATUS: IDLE | NAVIGATING | MANUAL | SAFE_STOP | ESTOP, null until the first telemetry
  mode,                         ControlMode name: NAVIGATING maps to AUTONOMOUS, null until the first telemetry
  targetSpeed, cte, distEnd,    /nav_telemetry numbers (null when '---')
  locStatus, locMessage,        /localization/status OK | DEGRADED | LOST, or null in direct mode
  gear, ecu, pedal, brake,      /carla/dbw_state: eff_gear D | R | N, ECU mode as the ECU writes it, lower case (no_command | active | timeout | estop), applied throttle and brake 0..1
  gnss: {ok, stale, fixAge},    /carla/gps_health, beacon stale gate, age of the last /gnss/fix
  goal: [x, y] | null }
```

Every field comes from a topic the observer subscribes to, as nift_web reads them, except `pose` truth, which the sim publishes on odom for evaluation tools anyway.

### 8.3 Differences from the draft `web/src/protocol.ts`

ARCH wins. The frontend agent updates `protocol.ts`.

- `state` adds `tick`, `paused`, `factor`, `rtf`, `poseEst`, `status`, `targetSpeed`, `cte`, `distEnd`, `locMessage`, `ecu`, `pedal`, `brake`, `gnss`, `goal`. `locStatus` may be `null` (direct mode has no filter). `gear` is `D | R | N` (no `P`: the shuttle has no park on CAN).
- `cmd` names are `start | stop | estop | resume | manual | dbw_estop | power_cycle`. `reset` is its own message, because it carries launch options.
- `step` takes `n`. `speed.factor` also takes `'max'`.
- New page messages: `teleop`, `initialpose`, `set_param`, `obstacle_add/move/remove`, `gnss_fault`, `reset`, `scenario_save`, `scenario_load`. `set_obstacles` stays, with `{oid, poly}` items. Obstacle ids are `oid` everywhere (2026-10-20), because `id` is the reply id of any page message.
- New worker messages: `event`, `reply`, `scenario`, `error`. `ready` carries the resolved launch and the presets. `log.node` and `log.t` are always set. `diag` items add `hardwareId`. `obstacles` adds `perceived`.
- Kept as drafted: `init {baseUrl, mapUrl}`, `progress`, `path`, `goal_ack`, `pause`, `resume_sim`, `goal {x, y, yaw?}`.

### 8.4 GNSS fault presets

Two kinds.

1. **Scheduled sim faults** (`sim_gnss.GnssFault`: dropout, jump, noise, stuck, freeze, swap, delay, burst, clock). These are the faithful path: the launch option `gnss_faults` (a scenario of `gnss_faults.yaml` or a list of faults with `t_s`, `at_m` or `box` triggers) and a `reset`, exactly as `gnss_faults:=` on ROS. "Now" presets (for example "dropout 1.5 s now") are faults with `t_s = now + dt`. The sim-gnss agent picks one of two ways and proves it does not change a no-fault trace: rebuild `node.gnss` as a new `GnssSim` from the same parameters plus the fault (the node reads `self.gnss` every tick), or replay the command log from a reset with the fault scheduled. Replay is exact but costs a re-run (about 0.5 s of wall time per minute of sim in Pyodide).
2. **Live link faults** in `BrowserOctane`, which model the Octane server or the Wi-Fi, outside every node: `outage` (both antennas silent for `duration_s`), `outage_front`, `outage_rear`, `repeat` (deliver the last payload again with the same `updated`, a stuck relay). They apply at the next tick and are logged like commands.

`presets` in `ready` lists both kinds with their parameters.

### 8.5 Scenarios

```
{ v: 1, launch: {...6.4...}, events: [ {tick, msg}, ... ], note? }
```

`events` is the log of applied page messages that change the sim (`goal`, `cmd`, `teleop`, `initialpose`, `obstacle_*`, `set_obstacles`, `gnss_fault`), with the tick they were applied after. Pacing messages are not in it. The page puts a scenario in the URL hash as base64url of the deflated JSON. Loading replays it deterministically.

## 9. Bundle and Pyodide

- **Pyodide** 0.27.7 (the spike's version), loaded in a Web Worker. Packages: `numpy` (localization_filter, controller, filters) and `pyyaml` (ShuttleConfig, parameter files, profiles). Nothing else: no pyproj, lxml, scipy or micropip. The map parses with `xml.etree` from the standard library.
- **Python files** go into one zip, unpacked to `/nift` with repo-relative paths. `tools/build_bundle.py` (pyodide agent) builds it from `tools/bundle_manifest.txt`, a list of globs. The manifest starts as:
  - `nift_common/nift_common/{__init__,site,vehicle,can_protocol}.py`, `nift_common/nift_common/vehicle.yaml`
  - `nift_sensor/nift_sensor/{__init__,beacon_node,gnss_hygiene,octane_client,shuttle_tf_node,localization_node,localization_filter}.py`, `nift_sensor/config/{beacon_node,shuttle_tf,localization}.yaml`
  - `nift_waypoint/nift_waypoint/{__init__,waypoint_node}.py`, `nift_waypoint/nift_waypoint/utils/*.py`, `nift_waypoint/config/waypoint_params.yaml`
  - `nift_control/nift_control/{__init__,trajectory_controller}.py`, `nift_control/nift_control/{controllers,filters,modes,dbw_interface}/*.py`, `nift_control/config/{planning,adrc_params}.yaml`
  - `nift_simulation/nift_carla/nift_carla/{__init__,kinematic_sim_node,carla_dbw,shuttle_plant,kinematic_model,sim_gnss,octane_emulator,carla_geo}.py`, `nift_simulation/nift_carla/config/{carla_params,gnss_faults,gnss_dead_zones,test_routes}.yaml`
  - `nift_bringup/nift_bringup/{__init__,profiles}.py`, `nift_bringup/config/profiles.yaml`
  - `nift_playground/py/**/*.py` and `nift_playground/py/fakeros/_generated/spec.json` (never `tools/gen_msgs/golden.json`, which is test data)
- A native test runs a full session under `sys.addaudithook`, records every `open` and `import` of a repo file, and fails if one is missing from the manifest. The builder refuses any path matching `.env*`, and anything outside the repo.
- **The map** is not in the zip. The page fetches `maps/lanelet2_odr2024.osm` once to draw it (before Pyodide loads, as the plan asks), and the worker writes the same bytes to `/nift/maps/lanelet2_odr2024.osm` for `waypoint_node`. The MIT notice of the Mcity Digital Twin goes with it (`maps/README.md`).
- Load order: page and map first, Pyodide when the sim starts, `progress` stages `pyodide`, `packages`, `bundle`, `map`, `nodes`, `ready`.

## 10. Tests

| test | where | who |
|---|---|---|
| spike smoke loop (`tests/spike/run_spike.py --repo .`) | native | architect (done: byte-identical to the Phase 0 evidence, with the spike's and with the generated messages) |
| fakeros semantics (timers on the clock grid, rcl catch-up, QoS latching, params precedence, remaps, TF, diagnostics) | `tests/fakeros/test_fakeros_*.py` | fakeros |
| messages against the Humble golden file | `tests/test_msgs.py` | msgs |
| Lanelet2 stand-in against real Lanelet2 (odr2024 and the Lanelet2 example map, random start and goal pairs) | `tests/lanelet2/` (Docker for the real side) | lanelet2 |
| GNSS chain: payload -> beacon_node -> tf / loc agrees with the true pose; fault presets | `tests/gnss/` | sim-gnss |
| launch set equals bringup's (run `nift_bringup.components` with stub `launch`, `launch_ros` and `ament_index_python` and compare the captured `Node(...)` parameters) | `tests/test_launch_set.py` | runner |
| session: goal, start, arrive; GNSS loss SAFE_STOP; replay determinism | `tests/test_session.py` | runner |
| same scenarios in Node + Pyodide, traces equal to native | `tests/pyodide/` | pyodide |
| against recorded ROS runs (`tests/reference/runs/*`) | `tests/test_reference.py` | validation (wave C) |

Run native tests with `python3 -m pytest nift_playground/tests` and no ROS sourced. For 3.10, run them in `nift_pub_deps:humble` with `PYTHONPATH` unset.

## 11. Ownership (wave B)

Each agent owns only these paths. Shared files are named once.

| agent | owns |
|---|---|
| architect | `ARCH.md`, `COLCON_IGNORE`, `README.md` (stub until the docs agent), `tests/spike/` |
| msgs (wave A) | `py/fakeros/_generated/**`, `tools/gen_msgs/**`, `tests/test_msgs.py` |
| lanelet2 (wave A) | `py/fakelanelet2/**`, `tests/lanelet2/**` |
| frontend-shell (wave A) | `web/` except the paths below |
| fakeros | `py/fakeros/**` except `_generated/`, `tests/fakeros/`. (The spike's `_msgs.py` and `msg_spec.json` were moved out on 2026-10-20.) |
| sim-gnss | `py/fakesocketio/**`, `py/stubs/**`, `py/playground/{sim_backend,octane_bridge}.py`, `tests/gnss/` |
| runner | `py/playground/{__init__,paths,launch_set,runner,session,operator,observer,perception,scenario,cli}.py`, `tests/test_launch_set.py`, `tests/test_session.py`, `tests/conftest.py` |
| pyodide | `web/src/worker/**`, `tools/build_bundle.py`, `tools/bundle_manifest.txt`, `tests/pyodide/**`, `tests/test_bundle_manifest.py`, `.github/workflows/playground.yml` (new file only) |
| frontend-panels | `web/src/panels/**`, `web/src/scenario/**`, and `web/src/protocol.ts` (bring it in line with section 8) |

`playground/cli.py` is a native headless runner (`python3 -m playground.cli --route 04`) that prints the outcome and writes traces in the format of `tests/reference/runs/`, so validation can compare.

## 12. Open risks

1. **Private names.** The wrapper relies on `kinematic_sim_node.OctaneEmulator`, `_tick_once`, `_sim_ns`, `dt` and `gnss` (2.1). A rename in the node breaks the playground, not the stack. A native test watches them.
2. **Timer semantics.** The spike fired timers between ticks. Section 7 fires them on the clock grid, as ROS does under `/clock`. Spike traces will change, and that is expected.
3. **Lanelet2 equivalence.** The centerline algorithm and the routing graph (lane changes, Germany rules) must match real Lanelet2, or paths differ from the shuttle's. Only the equivalence test can say.
4. **Map markers cost** (6.3). About 70k messages per second of sim, for nobody. Measure, then maybe override `map_publish_period_s`.
5. **Pyodide threading.** `threading.Lock`, `Condition` and an unstarted `Thread` must construct in Pyodide. Expected to work, to be checked by the pyodide agent on the first boot.
6. **Start-up.** On ROS, planning and control start some wall seconds after `auth_ok`, and nodes start at ROS time 0. Here they start at once, at `START_NS - dt`. Only start-up ages differ. Validation compares after the first path.
7. **QoS.** Every message is delivered, as the subscriber always keeps up. DDS could drop under load (KEEP_LAST depth 1). Not modelled.
8. **Randomness.** `gnss_seed 0` means random. The playground pins it (6.3).
9. **Secrets.** `octane_config()` reads a `.env` if it finds one. The runner sets all five `MCITY_OCTANE_*`, so a local `.env` never reaches beacon_node, and the bundle refuses `.env` files.
10. **Message strictness and speed.** Deep copies and type checks cost time per message. The perf agent decides whether the browser runs non-strict.

## 12. Decisions after wave B (2026-10-20)

- Obstacle ids are `oid` in every message.
- `ready.presets.gnss_faults` is one flat list, each item with `kind` set to `scheduled`, `sim` or `link` (the panels read this shape; `launch_set.presets()` and `sim_backend.gnss_fault_presets()` are to agree on it).
- `ready.launch.options` holds the resolved launch options.
- `event` for a mode change carries `{status, mode, from}`.
- The worker owns pause and speed and writes `state.paused` and `state.factor` into every state it posts.

