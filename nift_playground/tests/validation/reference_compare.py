"""
Compare a playground run with a reference ROS run of `tests/reference/runs/`.

Both dirs hold the files `extract_reference.py` and `playground.cli` write (odom.csv, cmd.csv, path.csv, events.csv, summary.json). The reference runs keep the tables gzipped (odom.csv.gz and so on), and either form is read. Only plain Python and numpy, no ROS.

The time axis differs from `compare_runs.py` in one way. There t_rel 0 is the stamp of the controller's first FORWARD frame. Here the runs are also lined up on the first tick the shuttle moves (`onset`). On ROS the first 0x101 frame sometimes reaches the virtual ECU one tick late, because `kinematic_sim_node` only waits for a frame (lockstep) once the ECU is ACTIVE. Then the whole ROS run is 0.05 s late against its own t_rel, which is about 10 cm at cruise speed. That happened in 3 of the 23 reference runs (direct_01, direct_12, ekf_01). Lining up on the onset removes it. `onset_shift` reports it.
"""

import csv
import gzip
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REFERENCE_RUNS = os.path.normpath(os.path.join(HERE, '..', 'reference', 'runs'))
DT = 0.05


def _rows(d, name):
    """Rows of <d>/<name>.gz (the reference runs, gzipped) or <d>/<name> (a playground run); [] if neither."""
    for path, opener in ((os.path.join(d, name + '.gz'), gzip.open), (os.path.join(d, name), open)):
        if os.path.exists(path):
            with opener(path, 'rt', newline='') as f:
                return list(csv.DictReader(f))
    return []


def load(d):
    od = np.array([[float(r[k]) for k in ('t_rel', 'x', 'y', 'yaw', 'v')] for r in _rows(d, 'odom.csv')])
    cmd = [r for r in _rows(d, 'cmd.csv')]
    path = np.array([[float(r['x']), float(r['y'])] for r in _rows(d, 'path.csv')]).reshape(-1, 2)
    events = [(float(r['t_rel']), r['level'], r['node'], r['msg']) for r in _rows(d, 'events.csv')]
    with open(os.path.join(d, 'summary.json')) as f:
        summary = json.load(f)
    return {'dir': d, 'od': od, 'cmd': cmd, 'path': path, 'events': events, 'summary': summary}


def onset(run):
    """t_rel of the first odom sample with the shuttle moving."""
    od = run['od']
    moving = od[(od[:, 0] >= 0) & (od[:, 4] > 0)]
    return float(moving[0, 0]) if len(moving) else 0.0


def first_frame_latency(run):
    """Sim seconds from the controller's first 0x101 frame to the tick the virtual ECU first applies one."""
    dbw = _rows(run['dir'], 'dbw.csv')
    if not run['cmd'] or not dbw:
        return None
    first = float(run['cmd'][0]['t'])
    active = next((float(r['ecu_t']) for r in dbw if r['mode'] == 'active'), None)
    return None if active is None else round(active - first, 3)


def _seg_dist(pts, line):
    a, b = line[:-1], line[1:]
    ab = b - a
    l2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = np.empty(len(pts))
    for i, p in enumerate(pts):
        u = np.clip(((p - a) * ab).sum(1) / l2, 0, 1)
        out[i] = np.sqrt((((a + u[:, None] * ab) - p) ** 2).sum(1)).min()
    return out


def _arc(xy):
    return np.r_[0.0, np.cumsum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))]


def _nav_end(run):
    s = run['summary']
    if s.get('nav_end_t') is not None and s.get('t_rel_zero') is not None:
        return s['nav_end_t'] - s['t_rel_zero']
    return math.inf


def event_t(run, node, text):
    return next((t for t, _lvl, n, msg in run['events'] if n == node and text in msg and t >= 0), None)


def compare(ref, pg):
    """Metrics of pg against ref. Positions in m, angles in deg, times in s."""
    s0, s1 = ref['summary'], pg['summary']
    m0, m1 = s0.get('metrics') or {}, s1.get('metrics') or {}
    on0, on1 = onset(ref), onset(pg)
    a, b = ref['od'].copy(), pg['od'].copy()
    a[:, 0] -= on0
    b[:, 0] -= on1
    t = np.arange(0.0, min(a[-1, 0], b[-1, 0]), DT)
    xa = [np.interp(t, a[:, 0], a[:, k]) for k in range(1, 5)]
    xb = [np.interp(t, b[:, 0], b[:, k]) for k in range(1, 5)]
    pos = np.hypot(xa[0] - xb[0], xa[1] - xb[1])
    # while both still navigate: the end (a 10 Hz goal check) is judged by `final` and `dur`
    nav = t < _nav_end(ref) - on0
    nav &= t < _nav_end(pg) - on1
    pos_nav = pos[nav] if nav.any() else pos
    dyaw = np.degrees(np.abs((xa[2] - xb[2] + math.pi) % (2 * math.pi) - math.pi))
    dv = np.abs(xa[3] - xb[3])
    mov_a = ref['od'][ref['od'][:, 0] >= 0][:, 1:3]
    mov_b = pg['od'][pg['od'][:, 0] >= 0][:, 1:3]
    track = float(max(_seg_dist(mov_b[::5], mov_a).max(), _seg_dist(mov_a[::5], mov_b).max()))
    # shape only: each run's samples up to the other's driven length, so a different rest point
    # (along the path) does not count
    sa, sb = _arc(mov_a), _arc(mov_b)
    shape = float(max(_seg_dist(mov_b[sb <= sa[-1]][::5], mov_a).max(),
                      _seg_dist(mov_a[sa <= sb[-1]][::5], mov_b).max()))
    p0, p1 = ref['path'], pg['path']
    path_max = float(np.abs(p0 - p1).max()) if p0.shape == p1.shape else math.inf
    d0, d1 = s0.get('nav_duration_s'), s1.get('nav_duration_s')
    f0, f1 = s0['final_pose'], s1['final_pose']
    out = {
        'outcome': [m0.get('outcome'), m1.get('outcome')],
        'status_sequence': [m0.get('status_sequence'), m1.get('status_sequence')],
        'path_points': [len(p0), len(p1)],
        'path_max': path_max,
        'onset_shift': round((on0 - on1), 3),
        'first_frame_latency': [first_frame_latency(ref), first_frame_latency(pg)],
        'pos_max': float(pos.max()),
        'pos_nav_max': float(pos_nav.max()),
        'shape_max': shape,
        'yaw_max': float(dyaw.max()),
        'v_max': float(dv.max()),
        'track_max': track,
        'dur': (d1 - d0) if (d0 is not None and d1 is not None) else None,
        'final': math.hypot(f1['x'] - f0['x'], f1['y'] - f0['y']),
    }
    # GNSS-loss runs: beacon_node's stale gate and the controller's SAFE_STOP, t_rel of the log lines
    for key, node, text in (('stale', 'beacon_node', 'GNSS stale'),
                            ('safe_stop', 'trajectory_controller', 'SAFE STOP')):
        e0, e1 = event_t(ref, node, text), event_t(pg, node, text)
        out[key + '_t'] = [e0, e1]
        out[key + '_dt'] = (e1 - e0) if (e0 is not None and e1 is not None) else None
    return out


def compare_dirs(ref_dir, pg_dir):
    return compare(load(ref_dir), load(pg_dir))


if __name__ == '__main__':
    import sys
    for pg_dir in sys.argv[2:]:
        lab = os.path.basename(os.path.normpath(pg_dir))
        ref_dir = sys.argv[1] if sys.argv[1] != '-' else os.path.join(REFERENCE_RUNS, lab)
        c = compare_dirs(ref_dir, pg_dir)
        print(lab, json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in c.items()}))
