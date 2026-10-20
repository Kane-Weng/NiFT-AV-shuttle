# nift_web_frontend

`nift_web_frontend` is the browser dashboard for the NiFT shuttle. It is a React and TypeScript single-page app, built with Vite. It connects to the ROS 2 graph through rosbridge ([nift_web](../nift_web/README.md)) and shows the shuttle on a map with its route, the obstacles ahead, the controller's telemetry and the health of every node. It also sends goals, mode changes, E-stop and manual driving commands.

This folder is not a ROS package. It holds a `COLCON_IGNORE` file, so `colcon build` skips it. More depth is on the wiki page [nift_web](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_web).

---

## Overview

```
                      /gnss/fix, /gnss/heading_deg, /planned_path,
                      /lanelet_map_markers, /obstacles               ┌──────────────────┐
                    ───────────────────────────────────────────────► │ MapView          │ ── /goal_pose ──────────►
                      /nav_telemetry                                 ├──────────────────┤
rosbridge_websocket ───────────────────────────────────────────────► │ ControlPanel     │ ── /teleop_cmd_vel ─────► rosbridge_websocket
  ws://localhost:9090 /gnss/vel, /nav_telemetry                      ├──────────────────┤ ── /start_navigation,
                    ───────────────────────────────────────────────► │ TelemetryPanel   │    /toggle_manual_mode,
                      /obstacles                                     ├──────────────────┤    /trigger_estop,
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

Components don't call roslib directly. Four small hooks in `src/hooks/` subscribe when a component mounts and clean up when it unmounts.

| Hook | Returns | Example |
|---|---|---|
| `useRos()` | the shared `ROSLIB.Ros`, or `null` before it connects | |
| `useRosTopic<T>(name, type, throttleMs?)` | the latest message as `T \| null` | `useRosTopic<NavSatFix>('/gnss/fix', 'sensor_msgs/NavSatFix')` |
| `useRosPublisher(name, type)` | a `publish(msg)` function | `useRosPublisher('/goal_pose', 'geometry_msgs/PoseStamped')` |
| `useRosService(name, type)` | a `call(request)` function | `useRosService('/trigger_estop', 'std_srvs/Trigger')` |

### Coordinates

The stack works in a flat `map` frame, metres east and north of a fixed origin (42.3005° N, 83.6987° W, at Mcity). Leaflet draws in latitude and longitude. `src/utils/coordinates.ts` converts between the two with `proj4` and UTM zone 17N, the same projection Lanelet2's `UtmProjector` uses. A map click goes the other way: latitude and longitude become `map` x and y, published as a `PoseStamped` on `/goal_pose`.

The origin, the UTM zone and the shuttle's size and steering limit are copies of the values in [nift_common](../nift_common/). Each copy carries an `owner: nift_common/...` comment, and `nift_common/test/test_consistency.py` checks that the copies still match.

### Panels

| Panel | Shows | Sends |
|---|---|---|
| `MapView` | OpenStreetMap tiles, the Lanelet2 road network, the planned path, the obstacle polygon, the goal and the shuttle as an arrow turned to its heading | a goal on `/goal_pose` when the user clicks the map in goal mode |
| `ControlPanel` | AUTO, MANUAL or SAFE STOP, read from the `STATUS` line of `/nav_telemetry` | AUTO: Set Goal, Start Navigation, E-STOP. MANUAL: `geometry_msgs/Twist` on `/teleop_cmd_vel` at 10 Hz from W/A/S/D or the arrow keys, Space to stop, speed and steering sliders. SAFE STOP: Resume and E-STOP |
| `TelemetryPanel` | the `/nav_telemetry` text as a table, with a coloured status, and a speed history chart from `/gnss/vel` | |
| `ObstacleRadar` | the obstacle polygon in the shuttle's own frame, with rings for the collision zone (0.8 m), the hard stop (2 m) and the influence radius (10 m) | |
| `NodeHealthChecker` | one row per `/diagnostics` status, with the worst level as a summary bar | |

In manual mode, `angular.z` of the `Twist` carries the steering angle in radians, which is how `trajectory_controller` reads `/teleop_cmd_vel`. Steering is mirrored when reversing, so the left key always turns left.

### Known limitations

- `MapView` draws `/obstacles` as if its points were in the `map` frame. `lidar_node` publishes them in `base_link`, so the polygon on the map appears near the map origin rather than next to the shuttle. `ObstacleRadar` reads the same topic in `base_link` and is correct.
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
├── hooks/                   useRos, useRosTopic, useRosPublisher, useRosService
├── utils/coordinates.ts     map frame <-> latitude and longitude (proj4, UTM 17N)
├── types/ros.ts             TypeScript shapes of the ROS messages used
└── components/              MapView, ControlPanel, TelemetryPanel, ObstacleRadar, NodeHealthChecker
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

---

**Maintainer**: Kane Weng (kaichi@umich.edu). **License**: Apache-2.0.
