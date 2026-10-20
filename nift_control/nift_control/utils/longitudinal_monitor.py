#!/usr/bin/env python3
"""
Live read-only window for a running longitudinal_test pass.

File: longitudinal_monitor.py
Date: Oct 8, 2026 (pad map and steering row: Oct 17, 2026)

Description:
  Live window for the longitudinal_test pass running in another terminal: the phase, how far the
  shuttle has gone against --max-dist, where the tool predicts it will stop, the speed against
  --max-speed, the steering (steering passes), and a top-down map of the garage pad. Read-only: it
  listens to /long_test/state, GNSS, /planned_path and /nav_telemetry and publishes nothing, so it
  cannot change a pass. Open it once per session; it keeps the last pass on screen until the next
  one starts. The map also works with no pass running (e.g. while trajectory_controller drives a
  planned path).

  Distance bar, 0 to --max-dist (100 %):
    solid          distance from the start fix (GNSS positions)
    hatched        on to the predicted stop: distance + 0.3 s reaction + v² / (2 · --assumed-decel)
    amber dashes   --max-dist − --brake-room: the drive phase ends when the predicted stop reaches
                   it
    red edge       --max-dist: a predicted stop past it aborts the pass
  The predicted stop is only as good as --assumed-decel. Still moving past --max-dist means the
  brakes are weaker than assumed: stop the shuttle by hand. Ctrl-C goes to the test's terminal, not
  to this window.

  Pad map (config/garage_pad.yaml, map frame turned so the pad's long axis runs left to right, NE
  on the right):
    grey dashes    where the shuttle drove on 2026-10-08: the pad is at least this big
    white box      the shuttle (front edge bright), at base_link = front antenna −
                   antenna_ahead_of_base_link
    white line     its trail;  cyan: /planned_path
    amber          where it stops if braked now: during a pass the tool's prediction, along the arc
                   of the commanded steering; otherwise 0.5 s + v² / (2 · 0.4 m/s²) straight ahead
    arrows / x     start poses / goals from the config; the nearest start is measured against the
                   shuttle

Usage:
  ros2 run nift_control longitudinal_test monitor [--pad <garage_pad.yaml>]
"""

from collections import deque
import json
import math
import os
import time
import tkinter as tk

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from nift_common.vehicle import ShuttleConfig
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String
import yaml

from ..dbw_interface import Gear

STALE_S = 0.5       # the test publishes at 20 Hz; silence this long = no pass running
REDRAW_MS = 50
TRAIL_S = 90.0      # trail length
# m/s², the measured brake (2026-10-08), for the stop marker when no pass runs
BRAKE_NOW = 0.4

W, H, X0, X1 = 1380, 540, 30, 730   # layout in 96-dpi pixels, scaled to the screen's dpi
MX0, MX1, MY0, MY1 = 780, 1350, 52, 300   # pad map area
FONT = 'Arial'
BG, FG, DIM, GREY, TRACK = '#1e1e1e', '#e8e8e8', '#8a8a8a', '#4a4a4a', '#2e2e2e'
GREEN, AMBER, RED, BLUE, TEAL, CYAN = (
    '#27ae60',
    '#e67e22',
    '#c0392b',
    '#2c6fbb',
    '#16a085',
    '#48c9e0',
)
PHASE_COLOR = {
    'wait_gnss': GREY,
    'static': BLUE,
    'arm': BLUE,
    'ramp': GREEN,
    'pedal': GREEN,
    'adrc': GREEN,
    'coast': TEAL,
    'brake': AMBER,
    'adrc_stop': AMBER,
    'abort': RED,
    'hold': BLUE,
    'steer_park': BLUE,
    'runup': GREEN,
    'pattern': TEAL,
}
MOVING = (
    'ramp',
    'release',
    'pedal',
    'coast',
    'brake',
    'adrc',
    'adrc_stop',
    'abort',
    'runup',
    'pattern',
)
SHUTTLE = ShuttleConfig()


def zone_color(x, soft, hard):
    return GREEN if x < soft else AMBER if x < hard else RED


class PadMap:
    """
    Convert positions for garage_pad.yaml.

    lat/lon -> map frame (UTM 17N minus the origin) -> pad frame (s along, v across).
    """

    def __init__(self, path):
        from pyproj import Proj  # only the monitor needs it
        with open(path) as f:
            cfg = yaml.safe_load(f)['garage_pad']
        lat0, lon0 = cfg['map_origin']
        self._proj = Proj(proj='utm', zone=int((lon0 + 180) // 6) + 1, ellps='WGS84')
        self._ox, self._oy = self._proj(lon0, lat0)
        self.axis = math.radians(cfg['axis_yaw_deg'])
        self.ant_ahead = float(cfg['antenna_ahead_of_base_link'])
        self.area = [tuple(p) for p in cfg['driven_area']]
        self.starts = [dict(s, xy=self.xy(s['lat'], s['lon'])) for s in cfg.get('starts', [])]
        self.goals = [dict(g, xy=self.xy(g['lat'], g['lon'])) for g in cfg.get('goals', [])]
        lat_a, lon_a = (
            (self.starts[0]['lat'], self.starts[0]['lon']) if self.starts else (lat0, lon0)
        )
        a, b = self.xy(lat_a, lon_a), self.xy(lat_a + 1e-4, lon_a)
        # true north in the map frame (≈ 88.2°)
        self.north_yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        pts = [self.pad(*p) for p in self.area] + [
            self.pad(*o['xy']) for o in self.starts + self.goals
        ]
        self.s_lo, self.s_hi = min(p[0] for p in pts) - 3.0, max(p[0] for p in pts) + 3.0
        self.v_lo, self.v_hi = min(p[1] for p in pts) - 3.0, max(p[1] for p in pts) + 3.0
        self.k = min((MX1 - MX0) / (self.s_hi - self.s_lo), (MY1 - MY0) / (self.v_hi - self.v_lo))

    def xy(self, lat, lon):
        x, y = self._proj(lon, lat)
        return x - self._ox, y - self._oy

    def yaw(self, compass_deg):
        """Map-frame yaw of a compass heading (rear -> front antenna)."""
        return self.north_yaw - math.radians(compass_deg)

    def pad(self, x, y):
        c, s = math.cos(self.axis), math.sin(self.axis)
        return x * c + y * s, -x * s + y * c

    def screen(self, x, y):
        s, v = self.pad(x, y)
        cy = (MY0 + MY1) / 2
        return (
            MX0 + (MX1 - MX0 - self.k * (self.s_hi - self.s_lo)) / 2 + (s - self.s_lo) * self.k,
            cy - (v - (self.v_lo + self.v_hi) / 2) * self.k,
        )


def arc(x, y, yaw, length, steer_deg, step=0.25):
    """Return points along a circle of curvature tan(δ) / L from (x, y, yaw), + = left."""
    kappa = math.tan(math.radians(steer_deg)) / SHUTTLE.wheel_base
    pts, n = [(x, y)], max(1, int(length / step))
    for i in range(1, n + 1):
        s = length * i / n
        if abs(kappa) < 1e-6:
            pts.append((x + s * math.cos(yaw), y + s * math.sin(yaw)))
        else:
            pts.append((x + (math.sin(yaw + kappa * s) - math.sin(yaw)) / kappa,
                        y - (math.cos(yaw + kappa * s) - math.cos(yaw)) / kappa))
    return pts


class LongitudinalMonitor(Node):

    def __init__(self, pad_path):
        super().__init__('longitudinal_monitor')
        self.state = None
        self.t_rx = None
        self.open = True
        self.fix = self.t_fix = self.heading = self.t_head = None
        self.v = 0.0
        self.trail = deque()
        self.path = []
        self.telemetry = self.t_tel = None
        self.pad, self.pad_error = None, None
        try:
            self.pad = PadMap(pad_path)
        except Exception as e:                  # the bars still work without the map
            self.pad_error = f'no pad map: {e}'
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/long_test/state', self._on_state, 10)
        self.create_subscription(NavSatFix, '/gnss/fix', self._on_fix, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/gnss/heading_deg', self._on_heading, qos_profile_sensor_data
        )
        self.create_subscription(TwistStamped, '/gnss/vel', self._on_vel, qos_profile_sensor_data)
        self.create_subscription(Path, '/planned_path', self._on_path, latched)
        self.create_subscription(String, '/nav_telemetry', self._on_tel, 10)

        self.root = tk.Tk()
        self.root.title('NiFT longitudinal test monitor')
        self.root.resizable(False, False)
        # stays visible while the test's terminal has focus
        self.root.attributes('-topmost', True)
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self.k = self.root.winfo_fpixels('1i') / 96.0
        self.c = tk.Canvas(
            self.root, width=W * self.k, height=H * self.k, bg=BG, highlightthickness=0
        )
        self.c.pack()
        self._redraw()

    # ------------------------------------------------------------------ inputs
    def _on_state(self, msg: String):
        self.state, self.t_rx = json.loads(msg.data), time.monotonic()

    def _on_fix(self, msg: NavSatFix):
        now = time.monotonic()
        new = (msg.latitude, msg.longitude) != self.fix
        self.fix, self.t_fix = (msg.latitude, msg.longitude), now
        if new and self.pad is not None and self.heading is not None:
            self.trail.append((now, *self._base_link()[:2]))
            while self.trail and now - self.trail[0][0] > TRAIL_S:
                self.trail.popleft()

    def _on_heading(self, msg: Float64):
        self.heading, self.t_head = msg.data, time.monotonic()

    def _on_vel(self, msg: TwistStamped):
        self.v = abs(msg.twist.linear.x)

    def _on_path(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_tel(self, msg: String):
        self.telemetry, self.t_tel = msg.data, time.monotonic()

    def _base_link(self):
        x, y = self.pad.xy(*self.fix)
        yaw = self.pad.yaw(self.heading)
        return x - self.pad.ant_ahead * math.cos(yaw), y - self.pad.ant_ahead * math.sin(yaw), yaw

    # ------------------------------------------------------------------ drawing
    def _close(self):
        self.open = False
        self.root.after_cancel(self._after)
        self.root.destroy()

    def _font(self, px, bold=False):
        return (FONT, -round(px * self.k), 'bold') if bold else (FONT, -round(px * self.k))

    def _redraw(self):
        # first, so a drawing error cannot freeze it
        self._after = self.root.after(REDRAW_MS, self._redraw)
        self.c.delete('all')
        live = self.state is not None and time.monotonic() - self.t_rx < STALE_S
        if self.state is None:
            self.c.create_text(
                X1 / 2 + X0 / 2,
                H / 2,
                text='waiting for /long_test/state …',
                fill=DIM,
                font=self._font(20),
            )
        else:
            self._draw(self.state, time.monotonic() - self.t_rx)
        self._draw_map(self.state if live else None)
        self.c.scale('all', 0, 0, self.k, self.k)

    def _draw(self, s, age):
        c, f, live = self.c, self._font, age < STALE_S
        phase = s['phase'].replace('_', ' ').upper()

        c.create_text(X0, 20, anchor='w', fill=DIM, font=f(14), text=f"{s['test'].upper()} pass")
        c.create_rectangle(
            X0, 36, X1, 106, outline='', fill=PHASE_COLOR.get(s['phase'], GREY) if live else GREY
        )
        c.create_text((X0 + X1) / 2, 71, fill='white', font=f(40, True) if live else f(28, True),
                      text=phase if live else f'NOT RUNNING  (last: {phase})')
        c.create_text(X0, 128, anchor='w', fill=FG, font=f(17),
                      text=f"{Gear(s['gear']).name}    pedal {s['pedal']}    brake {s['brake']}")
        if not live:
            c.create_text(
                X1, 128, anchor='e', fill=DIM, font=f(14), text=f'no update for {age:.0f} s'
            )

        d, stop, soft, hard = s['dist'], s['stop_dist'], s['soft_dist'], s['max_dist']
        c.create_text(X0, 170, anchor='w', fill=FG, font=f(19, True),
                      text=f'Distance  {d:.1f} / {hard:g} m   ({100 * d / hard:.0f} %)')
        c.create_text(
            X1, 170, anchor='e', fill=FG, font=f(16), text=f'predicted stop {stop:.1f} m'
        )
        self._bar(190, 238, hard, d, soft, zone_color(d, soft, hard), 5 if hard <= 50 else 10,
                  ahead=stop, ahead_color=zone_color(stop, soft, hard))
        c.create_text(X0, 276, anchor='w', fill=DIM, font=f(12),
                      text='solid: travelled    hatched: to the predicted stop    '
                           'amber dashes: drive ends    red edge: --max-dist')

        v, vs, vmax, tgt = s['v'], s['soft_speed'], s['max_speed'], s['target_v']
        c.create_text(
            X0,
            314,
            anchor='w',
            fill=FG,
            font=f(19, True),
            text=f'Speed  {v:.2f} m/s' + (f'    target {tgt:.2f}' if tgt is not None else ''),
        )
        c.create_text(X1, 314, anchor='e', fill=DIM, font=f(14),
                      text=f'drive ends at {vs:.2f}, abort above {vmax:g}')
        self._bar(334, 370, vmax, v, vs, zone_color(v, vs, vmax), 0.5, marker=tgt)

        if s.get('test', '').startswith('steer') or s.get('steer'):
            ack = s.get('steer_ack')
            step = s.get('step')
            c.create_text(
                X0,
                414,
                anchor='w',
                fill=FG,
                font=f(19, True),
                text=f"Steer  {s.get('steer', 0):+.1f}°"
                + (f"   step {step[0]}/{step[1]}" if step else ''),
            )
            c.create_text(X0 + 300, 414, anchor='w', fill=DIM, font=f(16),
                          text='CAN 0x301: ' + ('—' if ack is None else f'{ack:+.1f}°'))
            mt = s.get('max_turn')
            if mt:
                turn, come = abs(s.get('turn', 0.0)), s.get('turn_to_come', 0.0)
                c.create_text(
                    X1,
                    414,
                    anchor='e',
                    font=f(16, True),
                    fill=zone_color(turn + come, 0.8 * mt, mt),
                    text=f"turn {s.get('turn', 0.0):+.0f}° (+{come:.0f} stopping) / {mt:g}°",
                )

        if d >= hard:
            status, color = 'past --max-dist: stop the shuttle by hand if it is not stopping', RED
        elif s['abort']:
            status, color = f"ABORT: {s['abort']}", RED
        elif s['ended_by']:
            status, color = f"drive ended: {s['ended_by']}", GREY
        else:
            return
        c.create_rectangle(X0, 462, X1, 502, outline='', fill=color)
        c.create_text((X0 + X1) / 2, 482, fill='white', font=f(17, True), text=status)

    def _bar(
        self, y0, y1, full, value, soft, color, step, ahead=None, ahead_color=None, marker=None
    ):
        """
        Draw a horizontal bar 0..full.

        Fill to value, optional hatched run on to ahead, amber dashes at soft.
        """
        c, k = self.c, self.k
        x = lambda u: X0 + (X1 - X0) * max(0.0, min(u, full)) / full  # noqa: E731
        c.create_rectangle(X0, y0, X1, y1, fill=TRACK, outline='')
        if ahead is not None and ahead > value:
            c.create_rectangle(
                x(value), y0, x(ahead), y1, fill=ahead_color, stipple='gray50', outline=''
            )
        c.create_rectangle(X0, y0, x(value), y1, fill=color, outline='')
        c.create_line(x(soft), y0 - 6, x(soft), y1 + 6, fill=AMBER, width=3 * k, dash=(6, 4))
        c.create_line(X1, y0 - 6, X1, y1 + 6, fill=RED, width=4 * k)
        if marker is not None:
            c.create_line(x(marker), y0 - 4, x(marker), y1 + 4, fill='white', width=3 * k)
        for i in range(int(full / step + 1e-9) + 1):
            c.create_text(
                x(i * step), y1 + 16, fill=DIM, font=self._font(13), text=f'{i * step:g}'
            )

    def _draw_map(self, s):
        c, f, k, now = self.c, self._font, self.k, time.monotonic()
        c.create_text(MX0, 20, anchor='w', fill=DIM, font=f(14), text='Garage pad   (NE →)')
        if self.pad is None:
            c.create_text(
                (MX0 + MX1) / 2, (MY0 + MY1) / 2, fill=DIM, font=f(14), text=self.pad_error or ''
            )
            return
        pm, P = self.pad, self.pad.screen
        flat = lambda pts: [u for p in pts for u in P(*p)]  # noqa: E731
        c.create_rectangle(MX0 - 6, MY0 - 6, MX1 + 6, MY1 + 6, outline=GREY)
        c.create_polygon(*flat(pm.area), outline=DIM, fill='#262626', dash=(6, 4), width=2 * k)
        x5 = MX0 + 8
        c.create_line(x5, MY1 - 8, x5 + 5 * pm.k, MY1 - 8, fill=DIM, width=3 * k)
        c.create_text(x5 + 5 * pm.k + 6, MY1 - 8, anchor='w', fill=DIM, font=f(12), text='5 m')
        for st in pm.starts:
            x, y = st['xy']
            yaw = pm.yaw(st['heading'])
            c.create_line(
                *P(x, y),
                *P(x + 2.0 * math.cos(yaw), y + 2.0 * math.sin(yaw)),
                fill=GREEN,
                width=3 * k,
                arrow='last',
                arrowshape=(10, 12, 5),
            )
            c.create_text(*P(x, y), anchor='n', fill=GREEN, font=f(11), text=st['name'])
        for g in pm.goals:
            gx, gy = P(*g['xy'])
            c.create_line(gx - 5, gy - 5, gx + 5, gy + 5, fill=FG, width=2 * k)
            c.create_line(gx - 5, gy + 5, gx + 5, gy - 5, fill=FG, width=2 * k)
            c.create_text(gx, gy + 8, anchor='n', fill=FG, font=f(11), text=g['name'])
        if len(self.path) >= 2:
            c.create_line(*flat(self.path), fill=CYAN, width=2 * k)
        if len(self.trail) >= 2:
            c.create_line(*flat([(x, y) for _, x, y in self.trail]), fill=FG, width=2 * k)

        lines, y_txt = [], MY1 + 26
        fresh = self.fix is not None and self.heading is not None and now - self.t_fix < 2.0
        if fresh:
            bx, by, yaw = self._base_link()
            # the tool's prediction, on the commanded arc
            if s is not None and s.get('phase') in MOVING:
                ahead, steer, how = (
                    max(0.0, s['stop_dist'] - s['dist']),
                    s.get('steer', 0.0),
                    'if aborted now',
                )
            else:
                ahead, steer, how = (
                    0.5 * self.v + self.v**2 / (2 * BRAKE_NOW),
                    0.0,
                    f'braked now ({BRAKE_NOW} m/s²)',
                )
            if ahead > 0.05:
                fx, fy = bx + SHUTTLE.length / 2 * math.cos(
                    yaw
                ), by + SHUTTLE.length / 2 * math.sin(yaw)
                pts = arc(fx, fy, yaw, ahead, steer)
                c.create_line(*flat(pts), fill=AMBER, width=3 * k, dash=(5, 3))
                ex, ey = P(*pts[-1])
                c.create_oval(ex - 5, ey - 5, ex + 5, ey + 5, outline=AMBER, width=2 * k)
            hl, hw = SHUTTLE.length / 2, SHUTTLE.width / 2
            cs, sn = math.cos(yaw), math.sin(yaw)
            corner = lambda a, b: (bx + a * cs - b * sn, by + a * sn + b * cs)  # noqa: E731
            box = [corner(hl, hw), corner(-hl, hw), corner(-hl, -hw), corner(hl, -hw)]
            c.create_polygon(*flat(box), outline=FG, fill='', width=2 * k)
            c.create_line(*flat([box[0], box[3]]), fill='white', width=4 * k)
            lines.append((f'GNSS  v {self.v:.2f} m/s   heading {self.heading:.1f}°   '
                          f'fix {now - self.t_fix:.1f} s old', FG))
            # nearest start pose, measured along / across its heading
            if pm.starts:
                ax, ay = pm.xy(*self.fix)
                st = min(pm.starts, key=lambda o: math.hypot(o['xy'][0] - ax, o['xy'][1] - ay))
                sy = pm.yaw(st['heading'])
                dx, dy = ax - st['xy'][0], ay - st['xy'][1]
                along, across = dx * math.cos(sy) + dy * math.sin(sy), -dx * math.sin(
                    sy
                ) + dy * math.cos(sy)
                herr = ((self.heading - st['heading'] + 180) % 360) - 180
                ok = abs(along) < 1.0 and abs(across) < 0.5 and abs(herr) < 5
                lines.append(
                    (
                        f"{st['name']}:  {along:+.1f} m ahead   {across:+.1f} m left   "
                        f'heading {herr:+.1f}°'
                        + ('   ✓ lined up' if ok else ''),
                        GREEN if ok else DIM,
                    )
                )
            lines.append((f'amber: where it stops {how}', AMBER))
        else:
            lines.append(('no GNSS fix / heading', RED))
        if self.telemetry is not None and now - self.t_tel < 2.0:
            status = next(
                (
                    ln.split(':', 1)[1].strip()
                    for ln in self.telemetry.splitlines()
                    if 'STATUS' in ln
                ),
                self.telemetry.splitlines()[0] if self.telemetry else '',
            )
            lines.append((f'trajectory_controller: {status}', CYAN))
        lines.append(
            (
                'grey dashes: driven on 10-08   white: trail   cyan: planned path   '
                'arrows: start poses',
                DIM,
            )
        )
        for text, color in lines:
            c.create_text(MX0, y_txt, anchor='w', fill=color, font=f(14), text=text)
            y_txt += 26


def run_monitor(pad=None, args=None):
    """Entry point of `longitudinal_test monitor`."""
    pad = pad or os.path.join(
        get_package_share_directory('nift_control'), 'config', 'garage_pad.yaml'
    )
    rclpy.init(args=args)
    node = LongitudinalMonitor(pad)
    try:
        while rclpy.ok() and node.open:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.root.update()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C race: rclpy's SIGINT handler can shut the context down mid-spin
        if rclpy.ok():
            raise
    finally:
        if node.open:
            node._close()
        node.destroy_node()
        rclpy.try_shutdown()
