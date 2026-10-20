# nift_web_frontend

`nift_web_frontend` is the browser dashboard for the NiFT shuttle. It is a React and TypeScript single-page app, built with Vite. It connects to the ROS 2 graph through rosbridge ([nift_web](../nift_web/README.md)) and shows the shuttle on a map with its route, the obstacles ahead, the controller's telemetry and the health of every node. It also sends goals, mode changes, E-stop and manual driving commands.

This folder is not a ROS package. It holds a `COLCON_IGNORE` file, so `colcon build` skips it. More depth is on the wiki page [nift_web](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_web).

---

## Overview

```
                      /gnss/fix, /gnss/heading_deg, /planned_path,
                      /lanelet_map_markers, /obstacles, /tf          ┌──────────────────┐
                    ───────────────────────────────────────────────► │ MapView          │ ── /goal_pose ──────────►
                      /nav_telemetry                                 ├──────────────────┤
rosbridge_websocket ───────────────────────────────────────────────► │ ControlPanel     │ ── /teleop_cmd_vel ─────► rosbridge_websocket
  ws://localhost:9090 /gnss/vel, /nav_telemetry                      ├──────────────────┤ ── /start_navigation,
                    ───────────────────────────────────────────────► │ TelemetryPanel   │    /toggle_manual_mode,
                      /obstacles                                     ├──────────────────┤    /safe_stop, /trigger_estop,
                    ───────────────────────────────────────────────► │ ObstacleRadar    │    /resume (services) ──►
                      /diagnostics                                   ├──────────────────┤
                    ───────────────────────────────────────────────► │ NodeHealthChecker│
                                                                     └──────────────────┘
```

The five panels run in the browser. Both `rosbridge_websocket` ends are the same server, drawn twice so the arrows read left to right.

---

## How it works

### The ROS connection

The app uses [roslibjs](https://github.com/RobotWebTools/roslibjs) (`roslib`) to keep one WebSocket open to rosbridge. The URL is `ws://localhost:9090` unless `VITE_ROS_URL` says otherwise. A React context (`RosContext`) shares that one `ROSLIB.Ros` object, so no component opens its own connection.

When the connection drops, the app tries again every 3 seconds. A dot in the header shows the state: yellow while connecting, green when connected, red when disconnected.

### ROS hooks

Components don't call roslib directly. Small hooks in `src/hooks/` subscribe when a component mounts and clean up when it unmounts.

| Hook | Returns | Example |
|---|---|---|
| `useRos()` | the shared `ROSLIB.Ros`, or `null` before it connects | |
| `useRosTopic<T>(name, type, throttleMs?)` | the latest message as `T \| null` | `useRosTopic<NavSatFix>('/gnss/fix', 'sensor_msgs/NavSatFix')` |
| `useRosPublisher(name, type)` | a `publish(msg)` function | `useRosPublisher('/goal_pose', 'geometry_msgs/PoseStamped')` |
| `useRosService<T>(name, type)` | a `call(request, onResult?)` function. `onResult` gets the reply, or an error when there is no connection, no service or no reply within 5 s | `useRosService<TriggerResponse>('/trigger_estop', 'std_srvs/Trigger')` |
| `useTrigger(name)` | `[run, state]` for a `std_srvs/Trigger` button. `state` is pending, ok, refused (with the controller's message) or error | `useTrigger('/start_navigation')` |
| `useObstacles(inMap?)` | the obstacles of the latest LiDAR frame, in `base_link` and, with `inMap`, in `map` | `useObstacles(true)` |

### Coordinates

The stack works in a flat `map` frame, metres east and north of a fixed origin (42.3005° N, 83.6987° W, at Mcity). Leaflet draws in latitude and longitude. `src/utils/coordinates.ts` converts between the two with `proj4` and UTM zone 17N, the same projection Lanelet2's `UtmProjector` uses. A map click goes the other way: latitude and longitude become `map` x and y, published as a `PoseStamped` on `/goal_pose`.

`lidar_node` publishes `/obstacles` in `base_link`, one polygon per obstacle. All polygons of one LiDAR frame carry the same stamp, and a frame without obstacles sends one empty polygon. `useObstacles` collects the polygons of the latest frame the same way `trajectory_controller` does. For the map it also reads `/tf` and moves each polygon into `map` with the latest `map → base_link` transform. The radar uses the `base_link` points as they are.

The origin, the UTM zone and the shuttle's size and steering limit are copies of the values in [nift_common](../nift_common/). Each copy carries an `owner: nift_common/...` comment, and `nift_common/test/test_consistency.py` checks that the copies still match.

### Panels

| Panel | Shows | Sends |
|---|---|---|
| `MapView` | OpenStreetMap tiles, the Lanelet2 road network, the planned path, the obstacles of the latest LiDAR frame, the goal and the shuttle as an arrow turned to its heading | a goal on `/goal_pose` when the user clicks the map in goal mode |
| `ControlPanel` | AUTO, MANUAL, SAFE STOP or E-STOP, read from the `STATUS` line of `/nav_telemetry`, and the controller's reply to each button | AUTO: Set Goal, Start Navigation, Stop, E-STOP. MANUAL: `geometry_msgs/Twist` on `/teleop_cmd_vel` from W/A/S/D or the arrow keys, Space to stop, speed and steering sliders, Stop, E-STOP. SAFE STOP: Resume and E-STOP. E-STOP: a notice and E-STOP |
| `TelemetryPanel` | the `/nav_telemetry` text as a table, with a coloured status, and a speed history chart from `/gnss/vel` | |
| `ObstacleRadar` | the obstacles of the latest LiDAR frame in the shuttle's own frame, with rings for the collision zone (0.8 m), the hard stop (2 m) and the influence radius (10 m) | |
| `NodeHealthChecker` | one row per `/diagnostics` status, with the worst level as a summary bar | |

Each button that calls a controller service shows the reply. Start Navigation reads "Navigation Started" only when the controller accepts. A refusal shows the controller's reason under the button, such as "Cannot start: no valid path loaded." E-STOP reads "E-STOP FAILED" when the call can't reach the controller.

There are two stop buttons. **Stop** is the pause. It calls `/safe_stop`, the controller brakes into SAFE_STOP, and Resume (`/resume`) continues from there. It is enabled while NAVIGATING or in MANUAL, shown disabled in IDLE, and hidden in SAFE_STOP and ESTOP. **E-STOP** is the hardware E-stop, and the button says that it latches. It calls `/trigger_estop`, which latches the controller's `ESTOP`. The controller then refuses to start, resume or switch mode until it restarts, so the panel shows a notice and E-STOP only. The shuttle's own E-stop latch also needs a power cycle, as the controller's reply says.

In manual mode the panel publishes a `Twist` every 100 ms while a drive key is held. When the last key is released it sends one zero `Twist` and then nothing. `trajectory_controller` keeps the last `/teleop_cmd_vel` it got, so that zero is what stops the shuttle. When the page loses focus it no longer sees key releases, so it releases every key and sends the zero. Space always sends one zero. `angular.z` of the `Twist` carries the steering angle in radians, which is how `trajectory_controller` reads `/teleop_cmd_vel`. Steering is mirrored when reversing, so the left key always turns left.

### Known limitations

- `trajectory_controller` has no timeout on `/teleop_cmd_vel`. If the connection drops while a key is held, the zero never arrives and the shuttle keeps the last command. Stop it with E-STOP from another tool or the shuttle's own emergency stop.
- The panel takes the mode from `/nav_telemetry`. The controller publishes nothing in IDLE until something changes, so after a controller restart the panel keeps the last status it saw (e.g. E-STOP) until a new one arrives. Reloading the page clears it.
- The map shows obstacles only after a `map → base_link` transform has arrived on `/tf`. The radar doesn't need it.
- The radar's rings (0.8 m, 2 m, 10 m) are fixed in the code. They mirror the safety filter's settings in `nift_control` but aren't checked against them.
- rosbridge has no login. See [nift_web](../nift_web/README.md#security).

---

## Project structure

```
src/
├── App.tsx                  root layout, RosContext, connection indicator
├── main.tsx                 React entry point
├── index.css                Tailwind base and Leaflet overrides
├── vite-env.d.ts            type of VITE_ROS_URL
├── hooks/                   useRos, useRosTopic, useRosPublisher, useRosService, useTrigger, useObstacles
├── utils/coordinates.ts     map frame <-> latitude and longitude (proj4, UTM 17N)
├── utils/obstacles.ts       obstacles of one LiDAR frame, base_link -> map
├── utils/control.ts         control panel logic (mode, service replies, manual driving)
├── types/ros.ts             TypeScript shapes of the ROS messages used
└── components/              MapView, ControlPanel, TelemetryPanel, ObstacleRadar, NodeHealthChecker
test/                        unit tests for utils/obstacles.ts and utils/control.ts
```

Main dependencies: `react`, `roslib`, `leaflet` with `react-leaflet`, `proj4`, `recharts`, and `tailwindcss` for styling.

---

## Setup and running

Needs Node.js 20 or newer. Install the dependencies once, from this folder:

```bash
npm ci
```

Start rosbridge (`ros2 launch nift_web web.launch.py`, or `web:=true` on the system launch), then the dev server:

```bash
npm run dev                                   # http://localhost:5173
VITE_ROS_URL=ws://<host>:9090 npm run dev     # rosbridge on another machine
```

`VITE_ROS_URL` can also go in a `.env.local` file in this folder, which git ignores. The dev server listens on all interfaces, so a laptop on the same network can open `http://<host>:5173`. The app runs in the laptop's browser, so `localhost` then means the laptop. Set `VITE_ROS_URL=ws://<host>:9090` for that case.

For a static build, run `npm run build`. It type-checks with `tsc` and writes the site to `dist/`, which git ignores. `npm run preview` serves that build.

`npm test` runs the unit tests in `test/` with Node's own test runner, so it needs no extra packages. It needs Node.js 22.6 or newer, which runs TypeScript directly.

---

**Maintainer**: Kane Weng (kaichi@umich.edu). **License**: Apache-2.0.
