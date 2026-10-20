#!/usr/bin/env python3
"""
Compare reference runs of one route with each other (the spread of the ROS run) or a playground run with a reference.

    python3 compare_runs.py <run dir> <run dir> [<run dir> ...] [--json out.json]

Each run dir holds odom, cmd and summary.json as written by extract_reference.py (odom.csv.gz, ...) or by
playground.cli (odom.csv, ...); either is read. The first dir is the base. It uses the same method as
tests/validation/reference_compare.py:

  time axis      the runs are lined up on the first tick the shuttle moves (onset). On ROS the first 0x101 frame
                 sometimes reaches the virtual ECU one tick late, which shifts a whole run by 0.05 s against its
                 own t_rel (0 = the controller's first FORWARD frame). onset_shift reports it
  pos_nav_max    max distance between the two ground-truth base_link positions at the same time, while both
                 still navigate [m] (the end is judged by final and dur)
  pos_max        the same over the whole overlap
  shape_max      max distance from each run's driven polyline to the other's, each run only up to the other's
                 driven length, so a different rest point along the path does not count [m]
  yaw_max        max yaw difference at the same time [deg]
  v_max          max speed difference at the same time [m/s]
  steer_max      max steering command difference at the same time [deg]
  dur, final     nav duration difference [s] and final-position difference [m]
Only plain Python and numpy: it runs without ROS.
"""

import argparse
import csv
import gzip
import json
import math
import os

import numpy as np

DT = 0.05


def read_csv(d, name):
    """Rows of <d>/<name>.gz, else <d>/<name>; [] if neither exists."""
    for path, opener in ((os.path.join(d, name + '.gz'), gzip.open), (os.path.join(d, name), open)):
        if os.path.exists(path):
            with opener(path, 'rt', newline='') as f:
                return list(csv.DictReader(f))
    return []


def load(d):
    od = np.array([[float(r[k]) for k in ('t_rel', 'x', 'y', 'yaw', 'v')] for r in read_csv(d, 'odom.csv')])
    cmd = np.array([[float(r['t_rel']), float(r['steer_deg'])] for r in read_csv(d, 'cmd.csv')
                    if r['gear'] == '1'])
    with open(os.path.join(d, 'summary.json')) as f:
        summary = json.load(f)
    return {'od': od, 'cmd': cmd.reshape(-1, 2), 'summary': summary}


def onset(od):
    """t_rel of the first odom sample with the shuttle moving."""
    moving = od[(od[:, 0] >= 0) & (od[:, 4] > 0)]
    return float(moving[0, 0]) if len(moving) else 0.0


def nav_end(summary):
    if summary.get('nav_end_t') is not None and summary.get('t_rel_zero') is not None:
        return summary['nav_end_t'] - summary['t_rel_zero']
    return math.inf


def seg_dist(pts, line):
    """Distance of each point in pts to the polyline line."""
    a, b = line[:-1], line[1:]
    ab = b - a
    l2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = np.empty(len(pts))
    for i, p in enumerate(pts):
        u = np.clip(((p - a) * ab).sum(1) / l2, 0, 1)
        out[i] = np.sqrt((((a + u[:, None] * ab) - p) ** 2).sum(1)).min()
    return out


def arc(xy):
    return np.r_[0.0, np.cumsum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))]


def compare(base, other):
    s0, s1 = base['summary'], other['summary']
    on0, on1 = onset(base['od']), onset(other['od'])
    a, b = base['od'].copy(), other['od'].copy()
    a[:, 0] -= on0
    b[:, 0] -= on1
    t = np.arange(0.0, min(a[-1, 0], b[-1, 0]), DT)
    xa = [np.interp(t, a[:, 0], a[:, k]) for k in range(1, 5)]
    xb = [np.interp(t, b[:, 0], b[:, k]) for k in range(1, 5)]
    pos = np.hypot(xa[0] - xb[0], xa[1] - xb[1])
    nav = (t < nav_end(s0) - on0) & (t < nav_end(s1) - on1)
    pos_nav = pos[nav] if nav.any() else pos
    dyaw = np.degrees(np.abs((xa[2] - xb[2] + math.pi) % (2 * math.pi) - math.pi))
    dv = np.abs(xa[3] - xb[3])
    steer = None
    c0, c1 = base['cmd'].copy(), other['cmd'].copy()
    if len(c0) > 1 and len(c1) > 1:
        c0[:, 0] -= on0
        c1[:, 0] -= on1
        tc = t[(t >= max(c0[0, 0], c1[0, 0])) & (t <= min(c0[-1, 0], c1[-1, 0]))]
        if len(tc):
            steer = float(np.max(np.abs(np.interp(tc, c0[:, 0], c0[:, 1]) - np.interp(tc, c1[:, 0], c1[:, 1]))))
    mov_a = base['od'][base['od'][:, 0] >= 0][:, 1:3]
    mov_b = other['od'][other['od'][:, 0] >= 0][:, 1:3]
    sa, sb = arc(mov_a), arc(mov_b)
    shape = float(max(seg_dist(mov_b[sb <= sa[-1]][::5], mov_a).max(),
                      seg_dist(mov_a[sa <= sb[-1]][::5], mov_b).max()))
    d0, d1 = s0.get('nav_duration_s'), s1.get('nav_duration_s')
    f0, f1 = s0['final_pose'], s1['final_pose']
    return {'onset_shift': round(on0 - on1, 3), 'pos_nav_max': float(pos_nav.max()), 'pos_max': float(pos.max()),
            'shape_max': shape, 'yaw_max': float(dyaw.max()), 'v_max': float(dv.max()), 'steer_max': steer,
            'dur': (d1 - d0) if (d0 is not None and d1 is not None) else None,
            'final': math.hypot(f1['x'] - f0['x'], f1['y'] - f0['y']),
            'outcome': [s0['metrics']['outcome'], s1['metrics']['outcome']]}


def _r(v, n):
    return None if v is None else round(v, n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--json')
    a = ap.parse_args()
    base = load(a.runs[0])
    out = {}
    print(f'base {a.runs[0]}')
    for d in a.runs[1:]:
        c = compare(base, load(d))
        out[os.path.basename(os.path.normpath(d))] = c
        print(f'  {d}: onset {c["onset_shift"]:+.2f} s  pos_nav {c["pos_nav_max"]:.4f} m  shape {c["shape_max"]:.4f} m  '
              f'yaw {c["yaw_max"]:.3f} deg  v {c["v_max"]:.4f} m/s  steer {_r(c["steer_max"], 3)} deg  '
              f'dur {_r(c["dur"], 3)} s  final {c["final"]:.4f} m  {c["outcome"]}')
    if a.json:
        with open(a.json, 'w') as f:
            json.dump({'base': os.path.basename(os.path.normpath(a.runs[0])), 'vs': out}, f, indent=2)


if __name__ == '__main__':
    main()
