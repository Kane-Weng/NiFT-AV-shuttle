#!/usr/bin/env python3
"""
Metrics of one sim test-route run, from its bag.

File: route_metrics.py

Description:
  The route metrics for a bag of one route (recorded
  by sim_route_run, by the launcher's bag component, or by hand), on either sim backend:

    completed         /nav_telemetry goes NAVIGATING -> IDLE, never SAFE_STOP / ESTOP.
                      Still NAVIGATING at the end of the bag is a failure: "stalled" if
                      the shuttle stood still STILL_S at the end, else "not finished"
    CTE RMS/max/mean  ground-truth base_link (/ackermann_like_controller/odom) against
                      the last /planned_path, signed (+ = left of the path). Leaves out
                      the first SKIP_START m and the last SKIP_END m of the path, and
                      samples below MOVING_V
    heading error     odom yaw minus the heading of the nearest path segment, same samples
    steering rate     |d steering| / dt between consecutive 0x101 frames on /to_can_bus
                      while NAVIGATING (road-wheel degrees, sim time); above CHATTER_DEG_S
                      counts as chatter
    cruise speed      median speed on the path interior outside the turn windows
    arrival           ground-truth speed when /nav_telemetry leaves NAVIGATING for IDLE,
                      and where: signed distance to the path end (- = short of it)
    rest vs path end  signed distance along the lane direction (the path's last END_BACK
                      m: the last segment alone can be a sideways exit hook, as on route
                      03) of the first sample below STILL_V after the last motion; + =
                      past the end. Drift: how far it moved after that (creep)
    path radius       smallest 3-point radius (points 1 m apart, path resampled at 0.25 m):
                      interior (as for CTE), entry leg (first SKIP_START m), exit leg
                      (last END_BACK m)
    turn windows      stretches of the path tighter than TURN_R_MAX m, padded by TURN_PAD
                      m: CTE, heading error max, steering rate max, min speed in each

  Time: bag receive times are wall clock, so with time_factor != 1 every duration comes
  from message stamps. Odom header stamps are sim time; CAN frames carry the controller's
  sim-time stamp (wall -> sim through the odom stamps for a frame without one);
  /nav_telemetry (a String) is mapped wall -> sim through the odom stamps.

  The odom samples are matched to the path by progress along it (a window around the
  previous match), so a route that passes near itself (a loop) does not jump legs.

  The plain functions (geometry, time, steering) are unit-tested
  (test/test_route_metrics.py); sim_route_run and sim_route_plot import this module.

Usage:
  ros2 run nift_carla sim_route_metrics <bag | run dir> [<bag | run dir> ...] [--json f]
  ros2 run nift_carla sim_route_metrics nift_bags/routes/04_right_turn_20261011_213000
  A run dir of sim_route_run holds the bag in bag/. Several bags: one table row each.
"""

import argparse
from dataclasses import dataclass, field
import json
import math
import os
import re
import sys

from nift_common.can_protocol import decode_command_frame, Gear, STEER_MAX_DEG
import numpy as np

ODOM = '/ackermann_like_controller/odom'
PATH = '/planned_path'
TELEMETRY = '/nav_telemetry'
CAN = '/to_can_bus'
GOAL = '/goal_pose'

STILL_V = 0.05          # m/s: at rest
MOVING_V = 0.3          # m/s: slower samples are left out of CTE and heading error
STILL_S = 2.0           # s (sim) below STILL_V that count as stopped
SKIP_START = 5.0        # m of path start left out of CTE (entry leg from the antenna)
SKIP_END = 3.0          # m of path end left out of CTE (exit leg, goal stop)
END_BACK = 5.0          # m: the lane direction at the path end is taken over the last END_BACK
CHATTER_DEG_S = 60.0    # steering rate that counts as chatter
FULL_LOCK_DEG = STEER_MAX_DEG - 0.1    # |steering| at or above = full lock
RESAMPLE_DS = 0.25      # m: path resampling for radius and turn windows
RADIUS_STEP = 4         # resampled points each side of the 3-point radius (1 m)
TURN_R_MAX = 30.0       # m: a turn window is where the path is tighter than this
TURN_CHORD = 5.0        # m: heading change measured over a centred chord this long
TURN_PAD = 8.0          # m added before and after each turn window
TURN_MERGE = 3.0        # m: turn stretches closer than this are one window


# ---------------------------------------------------------------- bag reading

def read_bag(path, topics):
    """Read {topic: [(receive time [s, wall], message)]} of `topics` from a rosbag2 dir."""
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in out:
            out[topic].append((t_ns * 1e-9, deserialize_message(data, types[topic])))
    return out


def find_bag(arg):
    """Return (bag dir, run dir or None) of a rosbag2 dir or a sim_route_run dir (bag/)."""
    arg = os.path.abspath(os.path.expanduser(arg))
    if os.path.isfile(os.path.join(arg, 'metadata.yaml')):
        parent = os.path.dirname(arg)
        run_dir = parent if os.path.isfile(os.path.join(parent, 'run.json')) else None
        return arg, run_dir
    if os.path.isfile(os.path.join(arg, 'bag', 'metadata.yaml')):
        return os.path.join(arg, 'bag'), arg
    if os.path.isdir(os.path.join(arg, 'bag')):
        raise FileNotFoundError(f'{arg}/bag has no metadata.yaml: the recorder did not close '
                                f'it (try: ros2 bag reindex {arg}/bag)')
    raise FileNotFoundError(f'{arg}: not a bag (no metadata.yaml) nor a sim_route_run dir')


def run_info(run_dir):
    """Return run.json of sim_route_run next to the bag, {} if there is none."""
    if not run_dir:
        return {}
    try:
        with open(os.path.join(run_dir, 'run.json')) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


# ---------------------------------------------------------------- time, angles, CAN

def stamp_s(stamp):
    """Convert a builtin_interfaces/Time to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_deg(a):
    """Wrap angle(s) in degrees to [-180, 180)."""
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def wall_to_sim(t, wall, sim):
    """
    Map bag receive times (wall clock) to sim time through (wall, sim) pairs of messages.

    Linear between the pairs; outside them, extrapolated with the mean rate of the whole
    span (np.interp alone would clamp: everything before the first pair would get its time).
    """
    t = np.asarray(t, float)
    wall, sim = np.asarray(wall, float), np.asarray(sim, float)
    if len(wall) == 0:
        return t.copy()
    if len(wall) == 1:
        return sim[0] + (t - wall[0])
    rate = (sim[-1] - sim[0]) / (wall[-1] - wall[0]) if wall[-1] > wall[0] else 1.0
    out = np.interp(t, wall, sim)
    out = np.where(t < wall[0], sim[0] + (t - wall[0]) * rate, out)
    return np.where(t > wall[-1], sim[-1] + (t - wall[-1]) * rate, out)


def steer_rates(t, steer, min_dt=1e-3):
    """
    Return |d steering| / dt [deg/s] between consecutive frames: (later frame's time, rate).

    Pairs closer than min_dt are skipped: on a mode change the controller sends a second
    frame with the same stamp (IDLE's stop frame), which would give an infinite rate.
    """
    t, steer = np.asarray(t, float), np.asarray(steer, float)
    if len(t) < 2:
        return np.zeros(0), np.zeros(0)
    dt, ds = np.diff(t), np.diff(steer)
    keep = dt >= min_dt
    return t[1:][keep], np.abs(ds[keep]) / dt[keep]


def decode_commands(frames):
    """
    Return [(receive time, stamp [s], CommandFrame)] of the 0x101 frames in [(t, Frame)].

    Other IDs and frames decode_command_frame rejects (DLC < 5, unknown gear) are skipped.
    """
    out = []
    for t, f in frames:
        try:
            out.append((t, stamp_s(f.header.stamp), decode_command_frame(f)))
        except ValueError:
            pass
    return out


# ---------------------------------------------------------------- telemetry

def parse_telemetry(text):
    """Split /nav_telemetry text into {field: value}, e.g. {'Target V': '1.50 m/s'}."""
    out = {}
    for line in text.splitlines():
        if ':' in line:
            k, v = line.split(':', 1)
            out[k.strip()] = v.strip()
    return out


def first_number(value):
    """Return the number of '1.50 m/s' (1.5); None for '--- m' or a missing field."""
    try:
        return float(value.split()[0])
    except (AttributeError, IndexError, ValueError):
        return None


def status_sequence(statuses):
    """Drop consecutive duplicates: [NAVIGATING, NAVIGATING, IDLE] -> [NAVIGATING, IDLE]."""
    seq = []
    for s in statuses:
        if not seq or seq[-1] != s:
            seq.append(s)
    return seq


def nav_window(status):
    """
    Return (t start, t end, status after) of the first NAVIGATING stretch of [(t, status)].

    t end and status after are None while it is still NAVIGATING at the end; all three
    are None if it never navigated.
    """
    t0 = None
    for t, s in status:
        if t0 is None:
            if s == 'NAVIGATING':
                t0 = t
        elif s != 'NAVIGATING':
            return t0, t, s
    return t0, None, None


# ---------------------------------------------------------------- path geometry

def clean_path(path):
    """Return the (N, 2) path without zero-length segments, and its cumulative length."""
    path = np.asarray(path, float)
    seg = np.hypot(*np.diff(path, axis=0).T)
    path = path[np.concatenate([[True], seg > 1e-6])]
    s_cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
    return path, s_cum


def project(path, s_cum, pts, back=2.0, ahead=10.0, relock=3.0):
    """
    Match points (in driving order) to the path: (signed CTE + = left, s, segment index).

    Each point takes the nearest segment within [s_prev - back, s_prev + ahead] of the
    previous match, so a path that passes near itself does not make s jump to another leg.
    When the global nearest segment is more than `relock` m closer, the match restarts
    from it (lost track).
    """
    pts = np.atleast_2d(np.asarray(pts, float))
    a, d = path[:-1], np.diff(path, axis=0)
    len2 = np.maximum((d ** 2).sum(1), 1e-12)
    rx = pts[:, 0:1] - a[None, :, 0]
    ry = pts[:, 1:2] - a[None, :, 1]
    u = np.clip((rx * d[None, :, 0] + ry * d[None, :, 1]) / len2[None, :], 0.0, 1.0)
    dist = np.hypot(rx - u * d[None, :, 0], ry - u * d[None, :, 1])
    cross = d[None, :, 0] * ry - d[None, :, 1] * rx
    s_all = s_cum[:-1][None, :] + u * np.sqrt(len2)[None, :]
    seg_s0, seg_s1 = s_cum[:-1], s_cum[1:]

    n = len(pts)
    cte, s, idx = np.zeros(n), np.zeros(n), np.zeros(n, int)
    prev = None
    for i in range(n):
        j = int(np.argmin(dist[i]))
        if prev is not None:
            win = (seg_s1 >= prev - back) & (seg_s0 <= prev + ahead)
            if win.any():
                jw = int(np.argmin(np.where(win, dist[i], np.inf)))
                if dist[i, jw] <= dist[i, j] + relock:
                    j = jw
        cte[i] = math.copysign(dist[i, j], cross[i, j]) if dist[i, j] > 0 else 0.0
        s[i], idx[i] = s_all[i, j], j
        prev = s[i]
    return cte, s, idx


def segment_headings(path):
    """Return the heading [rad] of every path segment."""
    d = np.diff(path, axis=0)
    return np.arctan2(d[:, 1], d[:, 0])


def resample(path, ds=RESAMPLE_DS):
    """Resample the path every ds m along its length: (points, s)."""
    path, s = clean_path(path)
    si = np.arange(0.0, s[-1], ds)
    return np.c_[np.interp(si, s, path[:, 0]), np.interp(si, s, path[:, 1])], si


def radius3(path, step=RADIUS_STEP):
    """Return the 3-point radius at every point, from the points `step` before and after."""
    path = np.asarray(path, float)
    r = np.full(len(path), np.inf)          # inf on straights and at the ends
    if len(path) <= 2 * step:
        return r
    p0, p1, p2 = path[:-2 * step], path[step:-step], path[2 * step:]
    a = np.linalg.norm(p1 - p0, axis=1)
    b = np.linalg.norm(p2 - p1, axis=1)
    c = np.linalg.norm(p2 - p0, axis=1)
    area2 = np.abs((p1[:, 0] - p0[:, 0]) * (p2[:, 1] - p0[:, 1])
                   - (p1[:, 1] - p0[:, 1]) * (p2[:, 0] - p0[:, 0]))
    with np.errstate(divide='ignore', invalid='ignore'):
        r[step:-step] = np.where(area2 > 1e-9, a * b * c / (2.0 * area2), np.inf)
    return r


def turn_windows(path_rs, s_rs, chord=TURN_CHORD, r_max=TURN_R_MAX, pad=TURN_PAD,
                 merge=TURN_MERGE):
    """
    Return [(s0, s1, heading change [deg])] where a resampled path is tighter than r_max.

    Curvature = heading change over a centred chord; stretches less than `merge` m apart
    are one window; each window is padded by `pad` m (clipped to the path).
    """
    if len(path_rs) < 3:
        return []
    ds = s_rs[1] - s_rs[0]
    d = np.diff(path_rs, axis=0)
    h = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    h = np.r_[h, h[-1]]
    n = max(int(chord / ds / 2), 1)
    k = np.zeros_like(h)
    if len(h) > 2 * n:
        k[n:-n] = (h[2 * n:] - h[:-2 * n]) / chord
    on = np.abs(k) > 1.0 / r_max
    gap = int(round(merge / ds))
    wins, i = [], 0
    while i < len(on):
        if not on[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(on) and (on[j + 1] or on[j + 1:j + 1 + gap].any()):
            j += 1
        dh = math.degrees(h[min(j + n, len(h) - 1)] - h[max(i - n, 0)])
        wins.append((float(max(s_rs[i] - pad, 0.0)), float(min(s_rs[j] + pad, s_rs[-1] + ds)),
                     dh))
        i = j + 1
    return wins


def end_direction(path, s_cum, back=END_BACK):
    """Return the lane direction at the path end: from the point `back` m before to the end."""
    i = max(int(np.searchsorted(s_cum, s_cum[-1] - back)) - 1, 0)
    e = path[-1] - path[i]
    return e / max(np.linalg.norm(e), 1e-9)


def along_end(path, s_cum, pts, back=END_BACK):
    """Return the signed distance of points past the path end along end_direction (+ = past)."""
    return (np.atleast_2d(pts) - path[-1]) @ end_direction(path, s_cum, back)


def distance_to_go(path, s_cum, s, pts, back=END_BACK):
    """
    Return the distance to go to the path end [m] of matched points: + before, - past it.

    In the path's last `back` m and past its end: along the lane direction (end_direction),
    as the arrival and rest metrics measure it. Before that: the arc length left (s from
    project()), shifted to join the lane-direction distance continuously.
    """
    s = np.asarray(s, float)
    s_back = max(s_cum[-1] - back, 0.0)
    p_back = np.array([np.interp(s_back, s_cum, path[:, 0]),
                       np.interp(s_back, s_cum, path[:, 1])])
    d_back = -along_end(path, s_cum, p_back, back)[0]
    lane = -along_end(path, s_cum, pts, back)
    return np.where(s >= s_back, lane, d_back + (s_back - s))


def rest_index(v, still_v=STILL_V, moving_v=MOVING_V):
    """Return the first sample below still_v after the last one above moving_v (or None)."""
    v = np.asarray(v, float)
    moving = np.flatnonzero(v > moving_v)
    if not len(moving):
        return None
    still = np.flatnonzero(v[moving[-1]:] < still_v)
    return int(moving[-1] + still[0]) if len(still) else None


def still_for(t, v, still_v=STILL_V):
    """Return how long [s] the speed has been below still_v at the end of the samples."""
    v = np.asarray(v, float)
    above = np.flatnonzero(v >= still_v)
    if not len(above):
        return float(t[-1] - t[0]) if len(t) else 0.0
    i = above[-1] + 1
    return float(t[-1] - t[i]) if i < len(t) else 0.0


# ---------------------------------------------------------------- one run

@dataclass
class Run:
    """Everything the metrics and the plots need from one bag; times in sim seconds."""

    path: np.ndarray            # (N, 2) last /planned_path, map frame
    od: np.ndarray              # (M, 5) odom: t, x, y, yaw, speed
    status: list                # [(t, STATUS)] from /nav_telemetry
    tel: np.ndarray             # (K, 3) t, target speed, distance to the path end
    can: np.ndarray             # (F, 5) 0x101 frames: t, gear, brake, pedal, steering [deg]
    goal: tuple = None          # (x, y) of the last /goal_pose, if recorded
    n_paths: int = 1            # /planned_path messages with >= 2 poses
    bag: str = ''
    info: dict = field(default_factory=dict)    # run.json of sim_route_run, if any


def load_run(arg):
    """Load a Run from a bag dir or a sim_route_run dir."""
    bag_path, run_dir = find_bag(arg)
    bag = read_bag(bag_path, [PATH, ODOM, TELEMETRY, CAN, GOAL])
    if not bag[ODOM]:
        raise ValueError(f'{bag_path}: no {ODOM} (sim ground truth): not a sim bag?')
    paths = [m for _, m in bag[PATH] if len(m.poses) >= 2]
    if not paths:
        raise ValueError(f'{bag_path}: no {PATH} with at least 2 poses')
    path = np.array([(p.pose.position.x, p.pose.position.y) for p in paths[-1].poses])

    rows = sorted(((stamp_s(m.header.stamp), t, m) for t, m in bag[ODOM]),
                  key=lambda r: (r[0], r[1]))
    wall = np.array([w for _, w, _ in rows])
    sim = np.array([s for s, _, _ in rows])
    od = np.array([(s, m.pose.pose.position.x, m.pose.pose.position.y,
                    yaw_of(m.pose.pose.orientation),
                    math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
                   for s, _, m in rows])

    t_tel = wall_to_sim([t for t, _ in bag[TELEMETRY]], wall, sim)
    status, tel = [], []
    for t, (_, m) in zip(t_tel, bag[TELEMETRY]):
        f = parse_telemetry(m.data)
        if 'STATUS' not in f:
            continue
        status.append((float(t), f['STATUS']))
        tv, de = first_number(f.get('Target V')), first_number(f.get('Dist End'))
        if tv is not None and de is not None:
            tel.append((float(t), tv, de))

    cmds = decode_commands(bag[CAN])
    t_mapped = wall_to_sim([t for t, _, _ in cmds], wall, sim)
    can = [(ts if ts > 0 else float(tm), int(c.gear), c.brake, c.pedal, c.steering_angle_deg)
           for tm, (_, ts, c) in zip(t_mapped, cmds)]
    can = np.array(can, float).reshape(-1, 5)
    can = can[np.argsort(can[:, 0], kind='stable')]

    goal = None
    if bag[GOAL]:
        g = bag[GOAL][-1][1].pose.position
        goal = (g.x, g.y)
    return Run(path=path, od=od, status=status, tel=np.array(tel).reshape(-1, 3), can=can,
               goal=goal, n_paths=len(paths), bag=bag_path, info=run_info(run_dir))


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else None


def _max_abs(x):
    return float(np.max(np.abs(x))) if len(x) else None


def _finite(x):
    return float(x) if x is not None and np.isfinite(x) else None


def track(run):
    """Match the odom to the path: path, s_cum, cte, s, heading error [deg], masks, windows."""
    path, s_cum = clean_path(run.path)
    od = run.od
    t_start, t_end, _ = nav_window(run.status)
    od = od[od[:, 0] >= (t_start if t_start is not None else od[0, 0]) - 1.0]
    cte, s, seg = project(path, s_cum, od[:, 1:3])
    herr = wrap_deg(np.degrees(od[:, 3] - segment_headings(path)[seg]))
    length = float(s_cum[-1])
    in_nav = od[:, 0] <= (t_end if t_end is not None else np.inf)
    interior = (s > SKIP_START) & (s < length - SKIP_END) & in_nav
    path_rs, s_rs = resample(path)
    return {'path': path, 's_cum': s_cum, 'od': od, 'cte': cte, 's': s, 'herr': herr,
            'length': length, 'mask': interior & (od[:, 4] > MOVING_V), 'interior': interior,
            'path_rs': path_rs, 's_rs': s_rs, 'windows': turn_windows(path_rs, s_rs),
            't_start': t_start, 't_end': t_end}


def compute(run):
    """Return the metrics dict of one run (None where a metric does not apply)."""
    tr = track(run)
    od, cte, s, herr, mask = tr['od'], tr['cte'], tr['s'], tr['herr'], tr['mask']
    path, s_cum, length = tr['path'], tr['s_cum'], tr['length']
    t_start, t_end, after = nav_window(run.status)
    seq = status_sequence(st for _, st in run.status)
    navigated = t_start is not None

    # outcome
    still_end = still_for(od[:, 0], od[:, 4])
    if not navigated:
        outcome = 'not_started'
    elif after is None:
        outcome = 'stalled' if still_end >= STILL_S else 'not_finished'
    elif after == 'IDLE':
        outcome = 'completed' if not {'SAFE_STOP', 'ESTOP'} & set(seq) else 'safe_stop'
    else:
        outcome = after.lower()
    completed = outcome == 'completed'

    # steering from the 0x101 frames while NAVIGATING (FORWARD: IDLE's stop frames are NEUTRAL)
    can = run.can
    t_lo = t_start if navigated else np.inf
    t_hi = t_end if t_end is not None else np.inf
    nav_can = can[(can[:, 0] >= t_lo) & (can[:, 0] < t_hi) & (can[:, 1] == Gear.FORWARD)]
    t_sr, sr = steer_rates(nav_can[:, 0], nav_can[:, 4])
    s_sr = np.interp(t_sr, od[:, 0], s) if len(t_sr) else np.zeros(0)

    # arrival, rest
    v_arr = d_arr = None
    if after == 'IDLE':
        v_arr = float(np.interp(t_end, od[:, 0], od[:, 4]))
        p_arr = np.array([np.interp(t_end, od[:, 0], od[:, 1]),
                          np.interp(t_end, od[:, 0], od[:, 2])])
        d_arr = float(along_end(path, s_cum, p_arr)[0])
    k = rest_index(od[:, 4])
    rest = rest_drift = rest_xy = None
    if k is not None:
        rest_xy = od[k, 1:3]
        rest = float(along_end(path, s_cum, rest_xy)[0])
        rest_drift = float(np.max(np.hypot(*(od[k:, 1:3] - rest_xy).T)))

    nav = od[:, 0] >= t_lo
    driven = float(np.sum(np.hypot(*np.diff(od[nav, 1:3], axis=0).T))) if nav.sum() > 1 else 0.0

    # path radius
    srs = tr['s_rs']
    r3 = radius3(tr['path_rs'])
    inner = (srs > SKIP_START) & (srs < length - SKIP_END)
    r_min = float(r3[inner].min()) if inner.any() else None
    r_min_s = float(srs[inner][np.argmin(r3[inner])]) if inner.any() else None
    entry, exit_ = srs < SKIP_START, srs > length - END_BACK
    r_entry = float(r3[entry].min()) if entry.any() else None
    r_exit = float(r3[exit_].min()) if exit_.any() else None

    # turn windows
    turns, in_turn = [], np.zeros(len(od), bool)
    for s0, s1, dh in tr['windows']:
        w = (s >= s0) & (s <= s1)
        in_turn |= w
        m = mask & w
        wr = (s_sr >= s0) & (s_sr <= s1)
        wv = tr['interior'] & w & nav
        tight = (srs >= s0) & (srs <= s1)
        turns.append({
            's0': s0, 's1': s1, 'heading_change_deg': dh,
            'cte_rms': _rms(cte[m]), 'cte_max': _max_abs(cte[m]),
            'heading_err_max_deg': _max_abs(herr[m]),
            'steer_rate_max_deg_s': float(sr[wr].max()) if wr.any() else None,
            'v_min': float(od[wv, 4].min()) if wv.any() else None,
            'path_r_min': _finite(r3[tight].min()) if tight.any() else None})
    straight = mask & ~in_turn
    cruise = float(np.median(od[straight, 4])) if straight.any() else None

    if navigated:
        nav_s = float((t_end if t_end is not None else od[-1, 0]) - t_start)
    else:
        nav_s = None
    return {
        'bag': run.bag, 'route': route_label(run),
        'name': run.info.get('name') or bag_name(run.bag),
        'outcome': outcome, 'completed': completed, 'status_sequence': seq,
        'path_length': length, 'path_points': int(len(run.path)), 'paths_in_bag': run.n_paths,
        'goal': list(run.goal) if run.goal else run.info.get('goal'),
        'nav_duration_s': nav_s, 'driven': driven,
        'cte_rms': _rms(cte[mask]), 'cte_max': _max_abs(cte[mask]),
        'cte_mean': float(np.mean(cte[mask])) if mask.any() else None,
        'heading_err_rms_deg': _rms(herr[mask]), 'heading_err_max_deg': _max_abs(herr[mask]),
        'steer_rate_max_deg_s': float(sr.max()) if len(sr) else None,
        'steer_rate_over_chatter': int(np.sum(sr > CHATTER_DEG_S)),
        'last_steer_deg': float(nav_can[-1, 4]) if len(nav_can) else None,
        'cruise_speed': cruise, 'peak_speed': float(od[nav, 4].max()) if nav.any() else None,
        'arrival_speed': v_arr, 'arrival_vs_end': d_arr,
        'rest_vs_end': rest, 'rest_drift': rest_drift,
        'rest_xy': [float(v) for v in rest_xy] if rest_xy is not None else None,
        'still_at_end_s': still_end,
        'stop_s': float(s[-1]), 'stop_xy': [float(v) for v in od[-1, 1:3]],
        'stop_cte': float(cte[-1]),
        'path_r_min': _finite(r_min), 'path_r_min_s': r_min_s,
        'path_r_entry': _finite(r_entry), 'path_r_exit': _finite(r_exit),
        'turns': turns}


def bag_name(bag):
    """Return the name of a run from its bag path: the run dir for <run>/bag, else the bag dir."""
    bag = bag.rstrip(os.sep)
    base = os.path.basename(bag)
    return os.path.basename(os.path.dirname(bag)) if base == 'bag' else base


def route_label(run):
    """Return the route id ('04') from run.json, else the leading digits of the run's name."""
    if run.info.get('route'):
        return str(run.info['route'])
    name = bag_name(run.bag)
    m = re.match(r'(\d+)_', name)
    return m.group(1) if m else name


# ---------------------------------------------------------------- output

def _f(x, fmt='{:.2f}', none='–'):
    return none if x is None else fmt.format(x)


def _radius(r):
    return 'straight' if r is None or r > 1000.0 else f'{r:.1f} m'


def outcome_text(m):
    """Return 'yes', or why not: '**no**: stalled 52 m in, at full lock'."""
    if m['completed']:
        return 'yes'
    why = {'stalled': f'stalled {m["driven"]:.0f} m in',
           'not_finished': 'still driving when the bag ends',
           'not_started': 'never NAVIGATING', 'safe_stop': 'SAFE_STOP', 'estop': 'ESTOP'}
    text = why.get(m['outcome'], m['outcome'])
    lock = m['last_steer_deg'] is not None and abs(m['last_steer_deg']) >= FULL_LOCK_DEG
    if m['outcome'] == 'stalled' and lock:
        text += ', at full lock'
    return f'**no**: {text}'


TABLE_HEAD = ('| Route | Completed | CTE RMS / max | Heading error max | Steering rate max '
              '| Arrival speed | Rest vs path end |\n|---|---|---|---|---|---|---|')


def table_row(m):
    """Return one row of the route baseline table."""
    ok = m['completed']
    cte = f'{m["cte_rms"]:.2f} / {m["cte_max"]:.2f} m' if ok and m['cte_rms'] is not None else '–'
    cells = [m['route'], outcome_text(m), cte,
             _f(m['heading_err_max_deg'], '{:.0f}°'),
             _f(m['steer_rate_max_deg_s'], '{:.0f} °/s'),
             _f(m['arrival_speed'] if ok else None, '{:.2f} m/s'),
             _f(m['rest_vs_end'] if ok else None, '{:+.2f} m')]
    return '| ' + ' | '.join(cells) + ' |'


def summary(m):
    """Return the human-readable report of one run."""
    seq = ' -> '.join(m['status_sequence']) or 'no /nav_telemetry'
    paths = f' (last of {m["paths_in_bag"]} in the bag)' if m['paths_in_bag'] > 1 else ''
    r_at = (f' at s {m["path_r_min_s"]:.0f} m'
            if m['path_r_min_s'] is not None and m['path_r_min'] else '')
    lines = [
        f'{m["name"]}  ({m["bag"]})',
        f'  outcome      {m["outcome"]}  ({seq})',
        f'  path         {m["path_length"]:.1f} m, {m["path_points"]} points{paths}; driven '
        f'{m["driven"]:.1f} m in {_f(m["nav_duration_s"], "{:.1f}")} s (sim)',
        f'  CTE          RMS {_f(m["cte_rms"])} m, max {_f(m["cte_max"])} m, mean '
        f'{_f(m["cte_mean"], "{:+.2f}")} m  (+ = left; first {SKIP_START:g} m, last '
        f'{SKIP_END:g} m and < {MOVING_V:g} m/s left out)',
        f'  heading err  RMS {_f(m["heading_err_rms_deg"], "{:.1f}")}°, max '
        f'{_f(m["heading_err_max_deg"], "{:.1f}")}°',
        f'  steering     rate max {_f(m["steer_rate_max_deg_s"], "{:.0f}")} °/s '
        f'({m["steer_rate_over_chatter"]} steps above {CHATTER_DEG_S:g} °/s); last command '
        f'{_f(m["last_steer_deg"], "{:+.1f}")}°',
        f'  speed        cruise {_f(m["cruise_speed"])} m/s, peak {_f(m["peak_speed"])} m/s',
        f'  arrival      {_f(m["arrival_speed"])} m/s at {_f(m["arrival_vs_end"], "{:+.2f}")} m '
        f'from the path end  (- = short)',
        f'  rest         {_f(m["rest_vs_end"], "{:+.2f}")} m vs the path end along the lane '
        f'(+ = past), drift after rest {_f(m["rest_drift"])} m',
        f'  path radius  min {_radius(m["path_r_min"])}{r_at}; entry leg '
        f'{_radius(m["path_r_entry"])}, exit leg {_radius(m["path_r_exit"])}']
    if not m['completed']:
        lines.append(f'  stopped      at s {m["stop_s"]:.1f} of {m["path_length"]:.1f} m, '
                     f'{m["stop_cte"]:+.2f} m off the path, still for '
                     f'{m["still_at_end_s"]:.1f} s at the end')
    for i, w in enumerate(m['turns'], 1):
        lines.append(
            f'  turn {i}  {w["heading_change_deg"]:+5.0f}° s {w["s0"]:5.1f}-{w["s1"]:5.1f} m: '
            f'CTE RMS/max {_f(w["cte_rms"])}/{_f(w["cte_max"])} m, heading max '
            f'{_f(w["heading_err_max_deg"], "{:.0f}")}°, steer rate max '
            f'{_f(w["steer_rate_max_deg_s"], "{:.0f}")} °/s, v min {_f(w["v_min"])} m/s, '
            f'path radius {_radius(w["path_r_min"])}')
    return '\n'.join(lines)


def write_reports(m, out_dir):
    """Write metrics.json and metrics.txt (summary and table row) into out_dir."""
    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump(m, f, indent=2)
    with open(os.path.join(out_dir, 'metrics.txt'), 'w') as f:
        f.write(summary(m) + '\n\n' + TABLE_HEAD + '\n' + table_row(m) + '\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+', help='rosbag2 dir(s), or sim_route_run dir(s)')
    ap.add_argument('--json', help='write the metrics here (a list for several bags)')
    args = ap.parse_args(argv)

    results = []
    for b in args.bags:
        try:
            m = compute(load_run(b))
        except (OSError, ValueError) as e:
            print(f'{b}: {e}', file=sys.stderr)
            return 2
        results.append(m)
        print(summary(m) + '\n')
    print(TABLE_HEAD)
    for m in results:
        print(table_row(m))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(results[0] if len(results) == 1 else results, f, indent=2)
    return 0 if all(m['completed'] for m in results) else 1


if __name__ == '__main__':
    sys.exit(main())
