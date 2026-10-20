"""
playground_observer: reads the stack the way the web dashboard does, and turns it into the page's messages (ARCH 8.2).

It subscribes to the topics nift_web reads, plus the sim's ground truth, and looks up TF `map -> base_link` like the controller. It has no timers and publishes nothing, so it never changes the stack. The session asks it for a `state` snapshot and for the messages it queued (path, goal_ack, event, diag, obstacles).

`Trace` records the same topics in the format of `tests/reference/runs/` for the CLI and the tests.
"""

import json
import math

from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
import rclpy.time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

ODOM = '/ackermann_like_controller/odom'
MODE_OF_STATUS = {'NAVIGATING': 'AUTONOMOUS', 'IDLE': 'IDLE', 'MANUAL': 'MANUAL', 'SAFE_STOP': 'SAFE_STOP',
                  'ESTOP': 'ESTOP'}
GEAR = {'FORWARD': 'D', 'REVERSE': 'R', 'NEUTRAL': 'N'}
LOC_LEVELS = {0: 'OK', 1: 'DEGRADED', 2: 'LOST'}


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def level_int(level) -> int:
    return level[0] if isinstance(level, (bytes, bytearray)) else int(level)


def parse_telemetry(text: str) -> dict:
    """route_metrics.parse_telemetry: /nav_telemetry text to {field: value}."""
    out = {}
    for line in text.splitlines():
        if ':' in line:
            k, v = line.split(':', 1)
            out[k.strip()] = v.strip()
    return out


def first_number(value):
    """'1.50 m/s' -> 1.5; '--- m' or missing -> None."""
    try:
        return float(value.split()[0])
    except (AttributeError, IndexError, ValueError):
        return None


def _finite(v):
    return v if v is None or math.isfinite(v) else None


class PlaygroundObserver(Node):

    def __init__(self, runner=None, **kwargs):
        super().__init__('playground_observer', **kwargs)
        self.runner = runner
        self.out = []                   # queued page messages (dicts)
        self.trace = None               # a Trace, when the runner records one
        self.pose = None                # truth (x, y, yaw)
        self.speed = None               # truth m/s, None until the first odom
        self.dbw = {}
        self.telemetry = {}
        self.status = None
        self.loc = None                 # (level word, message)
        self.gps_ok = True
        self.gnss_stale = False
        self.fix_rx_ns = None
        self.goal = None
        self.path = None                # [[x, y], ...]
        self.path_t = None
        self._goal_pending = None       # goal waiting for its first /planned_path: (x, y, n_warn_before)
        self.planner_warnings = 0       # how many WARN+ lines waypoint_node logged
        self.planner_last_warning = ''  # the last one, for goal_ack.reason
        self.diag = {}                  # status name -> item
        self.perceived = None           # map-frame polygons of the last /obstacles batch
        self._perceived_stamp = None
        self._perceived_batch = []
        self.nav_runs = 0               # NAVIGATING stretches seen

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, ODOM, self._odom, 10)
        self.create_subscription(String, '/carla/dbw_state', self._dbw, 10)
        self.create_subscription(String, '/nav_telemetry', self._telemetry, 10)
        self.create_subscription(DiagnosticStatus, '/localization/status', self._loc_status, 10)
        self.create_subscription(Bool, '/carla/gps_health', self._gps_health, 10)
        self.create_subscription(String, '/carla/gnss_fault', self._gnss_fault, 10)
        self.create_subscription(NavSatFix, '/gnss/fix', self._fix, qos_profile_sensor_data)
        self.create_subscription(Path, '/planned_path', self._path, latched)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._diagnostics, 50)
        self.create_subscription(PolygonStamped, '/obstacles', self._obstacle, 50)
        self.create_subscription(Frame, '/to_can_bus', self._can, 50)
        self.create_subscription(TFMessage, '/tf', self._tf, 100)

    # ------------------------------------------------------------ helpers
    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def t_page(self) -> float:
        """Page time: sim time minus the first tick's 1 s."""
        return round(self.now_ns() / 1e9 - 1.0, 6)

    def emit(self, msg: dict):
        self.out.append(msg)

    def event(self, kind: str, data=None):
        self.emit({'type': 'event', 'kind': kind, 't': self.t_page(), 'data': data})

    def take(self) -> list:
        out, self.out = self.out, []
        return out

    def on_log(self, t, level, node, text):
        """Called by the runner for every log line (logs are not a topic in fakeros)."""
        if node == 'waypoint_node' and level in ('warn', 'error', 'fatal'):
            self.planner_warnings += 1
            self.planner_last_warning = text
        if node == 'beacon_node':
            if text.startswith('GNSS stale'):
                self.gnss_stale = True
            elif text.startswith('GNSS fresh again'):
                self.gnss_stale = False
        if self.trace is not None and level in ('warn', 'error', 'fatal'):
            self.trace.row('events', [self.now_ns() / 1e9, level.upper(), node, text.replace('\n', ' | ')])

    def pose_est(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        p = tr.transform.translation
        return {'x': p.x, 'y': p.y, 'yaw': yaw_of(tr.transform.rotation)}

    # ------------------------------------------------------------ callbacks
    def _odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.speed = msg.twist.twist.linear.x
        if self.trace is not None:
            t = stamp_ns(msg.header.stamp) / 1e9
            self.trace.row('odom', [t, *self.pose, math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)])

    def _tf(self, msg):
        if self.trace is None:
            return
        for tr in msg.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'base_link':
                p = tr.transform.translation
                self.trace.row('pose_tf', [stamp_ns(tr.header.stamp) / 1e9, p.x, p.y, yaw_of(tr.transform.rotation)])

    def _dbw(self, msg):
        try:
            self.dbw = json.loads(msg.data)
        except ValueError:
            return
        if self.trace is not None:
            d, a = self.dbw, self.dbw.get('applied') or {}
            self.trace.row('dbw', [self.now_ns() / 1e9, d.get('mode'), d.get('plant'), d.get('engaged'),
                                   d.get('eff_gear'), d.get('eff_pedal'), d.get('eff_brake'), d.get('v_cmd'),
                                   d.get('v_set'), a.get('wheel_deg'), a.get('throttle'), a.get('brake'), d.get('t')])

    def _telemetry(self, msg):
        f = parse_telemetry(msg.data)
        self.telemetry = f
        status = f.get('STATUS')
        if self.trace is not None and status:
            self.trace.row('telemetry', [self.now_ns() / 1e9, status, f.get('Filter', ''),
                                         *[first_number(f.get(k)) for k in ('Target V', 'Curr V', 'Obs Dist',
                                                                            'Dist End', 'CTE')],
                                         f.get('Next WP', '')])
            self.trace.status.append((self.now_ns() / 1e9, status))
        if status and status != self.status:
            prev, self.status = self.status, status
            self.event('mode', {'status': status, 'mode': MODE_OF_STATUS.get(status, status), 'from': prev})
            if status == 'NAVIGATING':
                self.nav_runs += 1
            if prev == 'NAVIGATING' and status == 'IDLE':
                self.event('arrived', {'pose': self._pose_dict(), 'goal': self.goal})

    def _loc_status(self, msg):
        word = LOC_LEVELS.get(level_int(msg.level), 'LOST')
        self.loc = (word, msg.message)
        if self.trace is not None:
            self.trace.row('loc_status', [self.now_ns() / 1e9, level_int(msg.level), msg.message,
                                          json.dumps({kv.key: kv.value for kv in msg.values})])

    def _gps_health(self, msg):
        self.gps_ok = bool(msg.data)
        self.event('gps_health', {'ok': self.gps_ok})

    def _gnss_fault(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            data = {'text': msg.data}
        self.event('gnss_fault', data)
        if self.trace is not None:
            self.trace.row('gnss_fault', [self.now_ns() / 1e9, msg.data])

    def _fix(self, msg):
        self.fix_rx_ns = self.now_ns()
        if self.trace is not None:
            self.trace.row('gnss_fix', [stamp_ns(msg.header.stamp) / 1e9, msg.latitude, msg.longitude,
                                        int(msg.status.status)])

    def _goal(self, msg):
        p = msg.pose.position
        self.goal = [p.x, p.y]
        self._goal_pending = (p.x, p.y, self.planner_warnings)

    def _path(self, msg):
        pts = [[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses]
        if self.trace is not None:
            self.trace.paths.append(pts)
        if pts != self.path:
            self.path, self.path_t = pts, self.t_page()
            self.emit({'type': 'path', 'points': pts, 't': self.path_t})
        if self._goal_pending is not None:
            gx, gy, n_warn = self._goal_pending
            self._goal_pending = None
            ok = len(pts) >= 2
            ack = {'type': 'goal_ack', 'x': gx, 'y': gy, 'ok': ok}
            if not ok and self.planner_warnings > n_warn:
                ack['reason'] = self.planner_last_warning
            elif not ok:
                ack['reason'] = 'waypoint_node published an empty path'
            self.emit(ack)

    def _diagnostics(self, msg):
        items = []
        for st in msg.status:
            item = {'name': st.name, 'hardwareId': st.hardware_id, 'level': level_int(st.level),
                    'message': st.message, 'values': {kv.key: kv.value for kv in st.values}}
            self.diag[st.name] = item
            items.append(item)
        if items:
            self.emit({'type': 'diag', 'items': sorted(self.diag.values(), key=lambda i: i['name'])})

    def _obstacle(self, msg):
        st = stamp_ns(msg.header.stamp)
        if st != self._perceived_stamp:
            self._perceived_stamp, self._perceived_batch = st, []
        pose = self.pose
        if msg.polygon.points and pose is not None:
            x0, y0, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            self._perceived_batch.append([[x0 + c * p.x - s * p.y, y0 + s * p.x + c * p.y]
                                          for p in msg.polygon.points])
        batch = [[[round(v, 3) for v in pt] for pt in poly] for poly in self._perceived_batch]
        if batch != self.perceived:
            self.perceived = batch
            if self.runner is not None:
                self.runner.obstacles_changed()

    def _can(self, msg):
        if self.trace is None or msg.id != 0x101:
            return
        from nift_common.can_protocol import decode_command_frame
        try:
            c = decode_command_frame(msg)
        except ValueError:
            return
        self.trace.row('cmd', [stamp_ns(msg.header.stamp) / 1e9, int(c.gear), int(c.brake), int(c.pedal),
                               float(c.steering_angle_deg)])

    # ------------------------------------------------------------ state
    def _pose_dict(self):
        if self.pose is None:
            return None
        return {'x': self.pose[0], 'y': self.pose[1], 'yaw': self.pose[2]}

    def snapshot(self) -> dict:
        """The `state` message's stack fields (ARCH 8.2)."""
        f, d = self.telemetry, self.dbw
        applied = d.get('applied') or {}
        status = f.get('STATUS')
        wheel = applied.get('wheel_deg')
        fix_age = None if self.fix_rx_ns is None else round((self.now_ns() - self.fix_rx_ns) / 1e9, 3)
        return {
            'pose': self._pose_dict(),
            'poseEst': self.pose_est(),
            'speed': self.speed,
            'steer': math.radians(wheel) if wheel is not None else 0.0,
            'status': status,
            'mode': MODE_OF_STATUS.get(status, status),
            'targetSpeed': _finite(first_number(f.get('Target V'))),
            'cte': _finite(first_number(f.get('CTE'))),
            'distEnd': _finite(first_number(f.get('Dist End'))),
            'locStatus': self.loc[0] if self.loc else None,
            'locMessage': self.loc[1] if self.loc else None,
            'gear': GEAR.get(d.get('eff_gear')),
            'ecu': d.get('mode'),
            'pedal': applied.get('throttle'),
            'brake': applied.get('brake'),
            'gnss': {'ok': self.gps_ok, 'stale': self.gnss_stale, 'fixAge': fix_age},
            'goal': self.goal,
        }


# ================================================================ trace

TRACE_COLUMNS = {
    'odom': ['t', 't_rel', 'x', 'y', 'yaw', 'v'],
    'pose_tf': ['t', 't_rel', 'x', 'y', 'yaw'],
    'cmd': ['t', 't_rel', 'gear', 'brake', 'pedal', 'steer_deg'],
    'telemetry': ['t', 't_rel', 'status', 'filter', 'target_v', 'curr_v', 'obs_dist', 'dist_end', 'cte', 'next_wp'],
    'dbw': ['t', 't_rel', 'mode', 'plant', 'engaged', 'eff_gear', 'eff_pedal', 'eff_brake', 'v_cmd', 'v_set',
            'wheel_deg', 'throttle', 'brake', 'ecu_t'],
    'gnss_fix': ['t', 't_rel', 'lat', 'lon', 'status'],
    'loc_status': ['t', 't_rel', 'level', 'message', 'values'],
    'gnss_fault': ['t', 't_rel', 'event'],
    'events': ['t', 't_rel', 'level', 'node', 'msg'],
}


class Trace:
    """
    Rows of the reference run files (tests/reference/README.md), in sim time.

    Rows are stored without t_rel. `t_rel_zero()` finds it as extract_reference.py does: the stamp of the controller's first FORWARD 0x101 frame at or after the start of navigation (minus 1 s).
    """

    def __init__(self):
        self.rows = {k: [] for k in TRACE_COLUMNS}
        self.paths = []
        self.status = []                # (t, STATUS) of /nav_telemetry

    def row(self, name, values):
        self.rows[name].append(values)

    def nav_window(self):
        """route_metrics.nav_window: (t start, t end, status after) of the first NAVIGATING stretch."""
        t0 = None
        for t, s in self.status:
            if t0 is None:
                if s == 'NAVIGATING':
                    t0 = t
            elif s != 'NAVIGATING':
                return t0, t, s
        return t0, None, None

    def t_rel_zero(self) -> float:
        t_start = self.nav_window()[0]
        for r in self.rows['cmd']:
            if r[1] == 1 and r[0] >= (t_start or 0.0) - 1.0:
                return r[0]
        if t_start is not None:
            return t_start
        return self.rows['odom'][0][0] if self.rows['odom'] else 0.0

    def tables(self) -> dict:
        """{file stem: (header, rows with t_rel)} plus path."""
        t0 = self.t_rel_zero()
        out = {}
        for name, cols in TRACE_COLUMNS.items():
            out[name] = (cols, [[r[0], r[0] - t0, *r[1:]] for r in self.rows[name]])
        last = next((p for p in reversed(self.paths) if len(p) >= 2), [])
        out['path'] = (['x', 'y'], [list(p) for p in last])
        return out

    def write(self, out_dir, skip_empty=('loc_status', 'gnss_fault')):
        import csv
        import os
        os.makedirs(out_dir, exist_ok=True)
        for name, (cols, rows) in self.tables().items():
            if name in skip_empty and not rows:
                continue
            with open(os.path.join(out_dir, f'{name}.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(cols)
                for r in rows:
                    w.writerow([f'{v:.6f}' if isinstance(v, float) else v for v in r])
