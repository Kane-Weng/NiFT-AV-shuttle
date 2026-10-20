# nift_web

`nift_web` lets a web browser talk to the shuttle's ROS 2 graph. It is a small launch-only package. It starts `rosbridge_websocket`, a standard community server from [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite). The server turns topics and services into JSON messages over a WebSocket, following the [rosbridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md). The browser app that uses it is [nift_web_frontend](../nift_web_frontend/README.md).

More depth, with diagrams, is on the wiki page [nift_web](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/nift_web) and on [rosbridge](https://github.com/Kane-Weng/NiFT-AV-shuttle/wiki/rosbridge).

---

## How it fits

```
NiFT stack (every node) ◄──── topics, services ────► rosbridge_websocket ◄──── WebSocket :9090, JSON ────► browser
                                                      (this package)                                       (nift_web_frontend)
```

The browser loads the React app from its own server (Vite, port 5173 by default). The app then opens `ws://localhost:9090`, or the URL in `VITE_ROS_URL`, and from there it subscribes, publishes and calls services like any other ROS node. rosbridge doesn't filter or change the data. It only translates between ROS messages and JSON.

The dashboard reads `/gnss/fix`, `/gnss/heading_deg`, `/gnss/vel`, `/planned_path`, `/lanelet_map_markers`, `/obstacles`, `/tf`, `/nav_telemetry` and `/diagnostics`. It sends `/goal_pose`, `/teleop_cmd_vel` and the controller's services `/start_navigation`, `/toggle_manual_mode`, `/safe_stop`, `/trigger_estop` and `/resume`, and it shows each service's reply.

---

## Running it

rosbridge comes from apt:

```bash
sudo apt install ros-humble-rosbridge-server
```

Start the bridge on its own:

```bash
ros2 launch nift_web web.launch.py                      # listens on 0.0.0.0:9090
ros2 launch nift_web web.launch.py address:=127.0.0.1   # this machine only
```

Or add `web:=true` to the system launch in [nift_bringup](../nift_bringup/README.md), which includes this launch file. Then start the dashboard as its [README](../nift_web_frontend/README.md) shows.

| Argument | Default | Meaning |
|---|---|---|
| `port` | `9090` | WebSocket port |
| `address` | `0.0.0.0` | Bind address. `0.0.0.0` accepts connections from the whole network |

The launch file also sets `max_message_size` to 10 MB. The Lanelet2 map arrives as one large `MarkerArray`, and roslibjs can't put fragmented messages back together, so a smaller limit would drop the map without an error.

---

## Security

rosbridge has no login. Anyone who can reach the port can read every topic, publish commands and call services, including the controller's start and stop services. On a shared network, bind to `127.0.0.1` (`address:=127.0.0.1`) and reach the dashboard through an SSH tunnel, or keep the shuttle on a private network.

---

## Package structure

```
nift_web/
├── launch/
│   └── web.launch.py    starts rosbridge_websocket
├── CMakeLists.txt       installs launch/
└── package.xml
```

---

**Maintainer**: Kane Weng (kaichi@umich.edu). **License**: Apache-2.0.
