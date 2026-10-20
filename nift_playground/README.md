# nift_playground

The NiFT shuttle's own ROS 2 nodes, running in a web page. The page loads Python in the browser with [Pyodide](https://pyodide.org) (CPython compiled to WebAssembly). Our five simulation nodes run there unchanged, on a small fake ROS that lives in the same Python process. You can set a goal on the map, start the shuttle, cut its GNSS and watch the real controller react. No ROS install and no server are needed.

The wiki page [Web Playground](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/Web-Playground) explains the idea and the design with diagrams. The contract every part is built against is [ARCH.md](ARCH.md).

> [!NOTE]
> This folder isn't a ROS package. It has a `COLCON_IGNORE`, so `colcon build` skips it. The page is live at [kane-weng.github.io/nift_playground](https://kane-weng.github.io/nift_playground/).

## What runs

The playground runs the stack of the `sim_kinematic` profile of `nift_bringup`, with the same parameter files:

| Node | Package | Job |
|---|---|---|
| `kinematic_sim_node` | `nift_carla` | the vehicle: virtual ECU, plant, bicycle model, GNSS model |
| `beacon_node` | `nift_sensor` | GNSS client and feed checks |
| `shuttle_tf_node` or `localization_node` | `nift_sensor` | the pose, `map → base_link` |
| `waypoint_node` | `nift_waypoint` | Lanelet2 route to the goal |
| `trajectory_controller` | `nift_control` | Pure Pursuit, ADRC, watchdogs, SAFE_STOP |

The Python files are imported from this repo as they are. Anything a browser can't run is replaced around the node, never inside it:

- **Fake ROS** (`py/fakeros/`). `rclpy`, `tf2_ros` and `diagnostic_updater` as the nodes use them, on one simulated clock, plus 160 message types generated from the Humble `.msg` and `.srv` files.
- **Lanelet2 stand-in** (`py/fakelanelet2/`). The few Lanelet2 calls `waypoint_node` makes, in pure Python: loading, UTM projection, nearest lanelet and routing.
- **socket.io stand-in** (`py/fakesocketio/`) and `BrowserOctane` (`py/playground/octane_bridge.py`). The GNSS server and the link to `beacon_node`, in process. The payloads are still built by the sim's own GNSS code.
- **Import stubs** (`py/stubs/`). Tiny `dotenv`, `aiohttp` and `pyproj` modules, so imports that the kinematic sim never uses still succeed.
- **The runner** (`py/playground/`). It builds the launch set from `nift_bringup.profiles`, steps the sim one 50 ms tick at a time, and speaks the worker protocol to the page.

## What a visitor can do

- Click a goal on the map, start navigation, stop, E-stop and resume, as in our RViz panel and the web dashboard.
- Drive by hand in manual mode with the keyboard.
- Pause, step, and change the speed of the sim clock.
- Drop obstacles on the map and drag them around (they reach the controller when the perception option is on).
- Trigger GNSS faults, such as an outage, a jump or a frozen feed, and watch the watchdogs.
- Switch the localization mode (`direct`, `ekf`, `shadow`), the GNSS feed model and the safety filter, then reset.
- Share a run as a link. The URL holds the launch options and every command, and replays exactly.

The map is drawn from the Lanelet2 file in JavaScript first. Pyodide loads only when the sim starts.

## The 3D world

The **World** button on the map shows the test site in 3D under our lanes: roads, curbs, markings, buildings, signs, poles, traffic lights and trees. It's off by default. The first click loads about 2 MB (with progress on the button), later clicks only show or hide it. It works in the top-down and the 3D view and in both themes. In the top-down view the lanes, the path and the shuttle are drawn over roofs and trees, like RViz overlays. In 3D they're depth tested against the world.

The site rises about 13 m from one end to the other, but the stack works on a flat map. So the page lifts what it draws onto the ground with a height grid (1 m cells) while the world is shown: the lanes, the path, the goal, obstacles, the trail and the shuttle, which also tilts with the slope. This is display only. The nodes, the sim and every message stay 2D, and clicks on the map still give map x and y.

The files are in `web/public/world/` (a Draco-compressed `world.glb` of 1.8 MB, `height.bin.gz`, `trees.json`, `world.json` and `NOTICE`). The Draco decoder comes from the pinned `three` package and is served next to them. `tools/world3d/build_world.sh` rebuilds them from the public twin with Blender and glTF-Transform. Its header lists the inputs, steps and versions, and `world.json` records the source commit and what was left out. What goes in:

- The twin's static world from its USD copy, in flat colours, with no textures at all. Repeated props are GPU instances, so the 1 M triangles take about 250 draw calls.
- Our own low-poly trees at the positions and species of the twin's 2,009 plants. The twin's plant models are left out because their source isn't credited. Its scale factors also made some species 30 to 45 m tall, so each species is set to a plausible height.
- The twin's frame is the `.xodr` transverse Mercator. One fixed similarity transform (−1.8174°, scale 1.000209) moves it into our map frame. 88.7 % of the 34,784 lanelet nodes fall on the twin's road surface, and the rest lie within 0.5 m (median) of its edge.

Left out on purpose, because they carry logos or brands or their licence isn't clear: the street light banners, the Block M disc on the roundabout island, the wrapped electrical boxes, a mailbox, the painted rock, the shuttle stop sign, the tunnel signage, the plant cards on the GPS blocker, all foliage and every texture. Brand names are stripped from every name in the files, and `test/world.test.ts` fails if one comes back.

Frame rate with the world on, measured in headless Chrome with a 1400 × 860 window on a laptop's integrated Radeon 610M: 60 fps with vsync in both views. Without the frame cap it draws about 320 fps (about 170 at a pixel ratio of 2), against about 900 without the world. The world adds about 27 MB of JavaScript heap.

## Run it locally

You need Python 3.10 or newer with `numpy`, `pyyaml` and `pytest`, and Node 22.18 or newer. Don't source ROS in the shell you use here. The stand-ins must win on `sys.path`.

### The page

```bash
cd nift_playground/web
npm ci
npm run dev        # builds the Python bundle, then serves http://localhost:5173
```

`npm run dev` and `npm run build` first run `npm run bundle`, which zips the repo's Python files into `web/public/py/` with `tools/build_bundle.py`. The dev server serves the map from the repo's `maps/` folder, so it isn't copied. `npm run build` writes a static site to `web/dist/`, and `npm run preview` serves it. Set `PLAYGROUND_BASE` (for example `/NiFT-AV-shuttle/`) when the site is served from a sub-path.

The page runs the real stack in Pyodide by default. `?sim=mock` swaps in a mock worker that starts at once and has no Python, which is handy while working on the page. Other URL options are `?theme=dark|light`, `?cam=ortho|persp`, `?world=1` (3D world on at start) and `#scenario=<code>`.

Pyodide loads from the jsDelivr CDN, so the page needs network on the first visit. Set `VITE_PYODIDE_URL` at build time to host it yourself.

### Headless, natively

The same session the page drives runs from the command line, without a browser.

```bash
cd nift_playground/py
python3 -m playground.cli --list                               # the test routes
python3 -m playground.cli --route 03                           # drive route 03, print the outcome
python3 -m playground.cli --route 04 --localization ekf --gnss-faults <scenario> --out /tmp/run04
python3 -m playground.cli --scenario /tmp/run04/scenario.json  # replay a saved run
```

A run takes a few seconds. The exit code is 0 when the shuttle arrived. `--out` writes traces in the format of `tests/reference/runs/`, plus `scenario.json` and `url.txt`, the same run as a link for the page. The fault scenario names come from `nift_simulation/nift_carla/config/gnss_faults.yaml`. Routes 13 and 14 need the 2025 map and don't run on the playground's map.

### Tests

| What | Command | Notes |
|---|---|---|
| Native suite | `python3 -m pytest nift_playground/tests -q` (repo root) | fake ROS, messages, Lanelet2 stand-in, GNSS chain, launch set, session, bundle manifest, validation against ROS. About 5,200 tests, about a minute |
| All reference cases | `NIFT_PLAYGROUND_SLOW=1 python3 -m pytest nift_playground/tests/validation` | all 16 ROS reference cases, not only the two quick ones |
| Python 3.10 | the same command inside `nift_pub_deps:humble`, with `PYTHONPATH` unset and ROS not sourced | Humble's Python |
| Node + Pyodide | `cd nift_playground/tests/pyodide && npm ci && node run_compare.mjs` | runs the scenarios in `scenarios/` natively and in Pyodide 0.27.7 and compares every message. Also runs the worker's own `core.ts` and its pacing |
| Worker pacing | `node --test test_pacer.mjs` (same folder) | no Pyodide needed |
| Perf and memory | `node perf.mjs` (same folder) | first-load size, boot time, speed, heap |
| Worker in a browser | `node browser_smoke.mjs` (same folder, after `npm ci` in `web/`) | the worker alone in headless Chrome, boots Pyodide from the CDN, drives route 04. Needs network |
| Page | `cd nift_playground/web && npx tsc --noEmit && npm test` | type check and unit tests |
| Page smoke | `npm run build && npm run smoke` (in `web/`) | headless Chrome clicks through every panel against the mock worker, then turns on the 3D world and takes top-down and 3D screenshots of it |
| Page end to end | `npm run e2e` (in `web/`) | builds the page and drives the real stack in headless Chrome through its own buttons. Boot, route 04 to arrival, a GNSS dropout to SAFE_STOP and Resume, Stop and Resume, E-STOP and Reset, pause and step, presets, reset. Needs network |
| Lanelet2 against the real one | `python3 nift_playground/tests/lanelet2/compare_real.py` | needs Docker and the `nift_pub_deps:humble` image. Refreshes the fixtures the native suite uses |

GitHub runs most of these checks in `.github/workflows/playground.yml` on every push that touches the playground, the packages it loads or the map. That covers the native suite on 3.10 and 3.12, Node + Pyodide, the page build, and the page end to end and smoke in headless Chrome. `.github/workflows/deploy-playground.yml` builds the page and publishes it to GitHub Pages.

## Layout

```
nift_playground/
  ARCH.md                the contract: layout, launch set, tick order, worker protocol, bundle
  py/
    fakeros/             rclpy, tf2_ros, diagnostic_updater, generated messages (_generated/)
    fakelanelet2/        lanelet2 stand-in, with a node cache for the NiFT map, NOTICE and LICENSES/
    fakesocketio/        socketio client stand-in and its in-process hub
    stubs/               dotenv, aiohttp, pyproj import stand-ins
    playground/          launch set, sim backend, Octane bridge, runner, session, operator, observer, CLI
  web/                   Vite + TypeScript + React + three.js page
    src/scene/           map view, the 3D world (world.ts, loaded on demand) and its height grid
    public/world/        the 3D world's files and their NOTICE (built by tools/world3d)
    src/panels/          operator panel, telemetry, playground controls, log
    src/worker/          the Pyodide worker (core.ts runs in the browser and in Node tests)
    src/mock/            a mock worker for page work without Python
  tools/
    build_bundle.py      zips the repo's Python files the nodes need (bundle_manifest.txt)
    gen_msgs/            message generator from the Humble .msg and .srv files
    world3d/             builds web/public/world/ from the public twin (Blender, glTF-Transform)
  tests/                 pytest suites, pyodide/ (Node harness), lanelet2/, reference/ (real ROS runs),
                         validation/ (playground against them), investigations/
```

## How it stays faithful

- **It runs the repo's files.** `tools/build_bundle.py` zips the node files, their libraries and their YAML from this checkout. Nothing is copied or rewritten. A native test records every file a full session opens or imports and fails if the bundle manifest misses one.
- **The launch is bringup's.** The node list and parameter sources come from `nift_bringup.profiles`, and a test checks them against what `system.launch.py` builds for ten option sets.
- **The fake ROS behaves like Humble.** Timer catch-up, parameter precedence, QoS latching, remaps, TF lookups and diagnostics were checked against the Humble sources. The messages are checked field by field against real `rclpy`.
- **The tick order is ROS's.** On ROS every node is its own process. When `/clock` reaches a new time, each timer fires on what had already arrived, so the controller acts on the GNSS fix from two ticks earlier. The runner fires all due timers first and delivers afterwards, which gives the same delay (ARCH.md section 7).
- **Lanelet2 is bit-for-bit.** On the NiFT map and on Lanelet2's example map the stand-in gives the same projections, centerlines, graph links, nearest lanelets and routes as real Lanelet2 1.2.3, including ties. When two lanelets are equally near, Lanelet2's answer depends on the order of its C++ containers. A clean-room module (`lanelet2/_stdorder.py`) reproduces that order from a written specification.
- **Native and browser agree.** The same scenario in native Python and in Pyodide gives the same messages, with numbers equal to about 3e-9.
- **Against the real stack.** `tests/reference/` holds runs of the real ROS stack in Docker on the test routes, direct and EKF, plus two GNSS-loss cases. `tests/validation/` drives the same 16 cases and compares them. Every case has the same outcome, status sequence and planned path. Route 02 with the EKF matches ROS to 0.0 mm over 105 s, and the other routes stay within a few centimetres, about the spread between repeated ROS runs. The largest is route 05 in direct mode at 10 cm, against the fastest of four ROS runs that differ by up to 6 cm among themselves. In the GNSS-loss cases the position matches within 1 mm and SAFE_STOP comes within 6 ms. The details are in `tests/validation/README.md`.
- **Even the surprises match.** With the EKF and the realistic GNSS feed, route 04 ends in SAFE_STOP. The real stack does the same, at the same sim time with the same log lines, because the feed model draws an outage longer than the filter can bridge. See `tests/investigations/ekf_realfeed/`.

## Limits

- **No LiDAR.** The kinematic profile has no point cloud. With the perception option, `/obstacles` comes from the page's obstacles and the true pose, in the format `lidar_node` publishes, not from a simulated scan.
- **Kinematic vehicle.** The motion model is the kinematic bicycle and the shuttle plant of `kinematic_sim_node`, not CARLA's physics.
- **No DDS jitter.** Messages follow ROS's tick order, but always the same way, and nothing is dropped. On ROS a frame sometimes lands a tick earlier or later, so repeated runs spread by a few centimetres. Runs here are deterministic.
- **One parameter differs.** `waypoint_node`'s `map_publish_period_s` is raised to 3600 s, because rebuilding the map markers every second cost most of the run time and nothing in the page reads them.
- **Parameters apply on reset.** The nodes read their parameters at start-up, so a changed parameter takes effect on the next reset.
- **Cost.** The first visit downloads about 9 MB. That is Pyodide, numpy and pyyaml from the CDN, our 0.7 MB bundle and the map. The 3D world adds about 2 MB when it's turned on. Booting takes a few seconds. The full stack runs about 12 times faster than real time in Pyodide at the `max` speed, so the higher speed settings are capped by what the machine can do. A tab uses a few hundred MB of memory, mostly the fixed cost of Pyodide, and it stays flat over many resets.

## Licences

The code in this folder is Apache-2.0, as the rest of the repo.

- **Map.** `maps/lanelet2_odr2024.osm` is derived from the OpenDRIVE file of the digital twin at [mcity/mcity-digital-twin](https://github.com/mcity/mcity-digital-twin), which is MIT licensed. The page ships that notice with the map, as `maps/README.md` does. The shuttle is drawn as a box with its real dimensions.
- **3D world.** `web/public/world/` is derived from the same twin's USD copy (tag 1.9.1), MIT licensed, © 2024 Quantum Signal AI LLC and The Regents of the University of Michigan. Its notice and the list of changes are in `web/public/world/NOTICE`, which is served with the files. No textures, logos, aerial imagery or plant models of the twin are included, and the trees are our own shapes. This project isn't affiliated with or endorsed by the owners of the twin or of the test facility.
- **Lanelet2 example map.** `tests/lanelet2/maps/mapping_example.osm` is Lanelet2's example map, BSD 3-Clause, with its licence beside it. It is test data only and not part of the page.
- **Lanelet2 stand-in.** Parts of it translate algorithms of Lanelet2 (BSD-3-Clause), Boost (BSL-1.0) and GeographicLib (MIT) into Python and keep those licences. [py/fakelanelet2/NOTICE](py/fakelanelet2/NOTICE) lists the sources per file, and the licence texts are in `py/fakelanelet2/LICENSES/`. The C++ container ordering is a clean-room implementation (`_stdorder.py`, written from `CLEANROOM_SPEC.md`).
- **Pyodide** (MPL-2.0) and its packages load from the CDN and aren't vendored here.
