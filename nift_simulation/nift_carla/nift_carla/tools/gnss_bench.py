#!/usr/bin/env python3
"""
Score the stack against the GNSS fault scenarios (the fault-injection bench).

File: gnss_bench.py

Description:
  For each scenario of config/gnss_faults.yaml (all, or the ones named): sim_route_run on
  the scenario's route with gnss_faults:=<scenario>, then a score from the run's bag
  against the scenario's target:
    outcome      completed / safe_stop / stalled / timeout (sim_route_run's run.json)
    stop         SAFE_STOP seen (/nav_telemetry), and how long after the first fault started
    pose error   at every ground-truth odom sample from NAVIGATING to rest: the stack's
                 believed base_link (the latest map -> base_link on /tf RECEIVED by then,
                 which is what trajectory_controller's TF lookup returns) against ground
                 truth: position (m) and heading (deg). Max over the fault window (first
                 start .. last end + SETTLE_S, or the whole run for a scenario without fault
                 events) and over the run
    staleness    age of that transform's stamp at each sample (s), max over the same window
    CTE          max |CTE| (route_metrics, metrics.json)
  Pass / fail against each target key (stop, max_error_m, max_heading_error_deg,
  max_cte_m); one row per scenario in <out>/scorecard.md and scorecard.json; each run in
  <out>/<scenario>/ (sim_route_run's run dir).

  Time: odom stamps are sim time; /tf and /carla/gnss_fault receive times are mapped to sim
  time through the odom (receive, stamp) pairs; fault event times are sim time. Run at time
  factor 1 (the default): the pose error then includes the pipeline's own delay as it is,
  while a faster time factor stretches it in sim time.

Usage:
  ros2 run nift_carla sim_gnss_bench --list
  ros2 run nift_carla sim_gnss_bench --out ~/bench/base    # every scenario, kinematic backend
  ros2 run nift_carla sim_gnss_bench dropout_1.5s_turn control_turn --out /tmp/b
  ros2 run nift_carla sim_gnss_bench --out ~/bench/base --resume   # skip scenarios already run
  ros2 run nift_carla sim_gnss_bench --score-only ~/bench/base     # re-score the runs there
"""

import argparse
from datetime import datetime
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

STILL_V = 0.05          # m/s: at rest
SETTLE_S = 2.0          # s after the last fault end still counted as the fault window
MAP, BASE = 'map', 'base_link'


# --------------------------------------------------------------------
#  Pure scoring
# --------------------------------------------------------------------
def wrap_deg(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def belief_errors(tf, od, t0, t1):
    """
    Compare believed vs true pose at the odom samples in [t0, t1].

    tf: (N, 5) rows (receive time [sim s], stamp [sim s], x, y, yaw), any order.
    od: (M, 5) (t, x, y, yaw, v).
    Returns (t, position error [m], heading error [deg], age [s], lateral error [m]: the
    part across the true heading, which is what moves the vehicle off its path; the rest is
    along-track lag); NaN before the first.
    """
    od = np.asarray(od, float)
    m = (od[:, 0] >= t0) & (od[:, 0] <= t1)
    t, x, y, yaw = od[m, 0], od[m, 1], od[m, 2], od[m, 3]
    tf = np.asarray(tf, float).reshape(-1, 5)
    if not len(tf):
        nan = np.full(len(t), np.nan)
        return t, nan, nan, nan, nan
    tf = tf[np.argsort(tf[:, 0], kind='stable')]
    i = np.searchsorted(tf[:, 0], t, side='right') - 1           # latest transform received by t
    ok = i >= 0
    j = np.clip(i, 0, None)
    dx, dy = tf[j, 2] - x, tf[j, 3] - y
    pos = np.where(ok, np.hypot(dx, dy), np.nan)
    lat = np.where(ok, np.abs(-dx * np.sin(yaw) + dy * np.cos(yaw)), np.nan)
    head = np.where(ok, np.abs(wrap_deg(np.degrees(tf[j, 4] - yaw))), np.nan)
    age = np.where(ok, t - tf[j, 1], np.nan)
    return t, pos, head, age, lat


def fault_window(events, t_end):
    """
    Return (first start, last end + SETTLE_S) from /carla/gnss_fault events.

    An unfinished fault runs to t_end.
    """
    starts = [e['t'] for e in events if e.get('edge') == 'start']
    if not starts:
        return None
    ends = {e['index']: e['t'] for e in events if e.get('edge') == 'end'}
    started = {e['index'] for e in events if e.get('edge') == 'start'}
    last = t_end if started - set(ends) else max(ends.values())
    return min(starts), min(t_end, last + SETTLE_S)


def check(score, target):
    """Return (passed, reasons) of a score against a scenario target."""
    reasons = []
    if 'stop' in target:
        if target['stop'] and not score['stop']:
            reasons.append('no stop')
        elif not target['stop'] and score['stop']:
            reasons.append('SAFE_STOP')
    for key, val, unit in (
        ('max_error_m', 'err_max', 'm'),
        ('max_heading_error_deg', 'head_err_max', 'deg'),
        ('max_cte_m', 'cte_max', 'm'),
    ):
        if key in target:
            v = score.get(val)
            if v is None or not math.isfinite(v):
                reasons.append(f'{val} n/a')
            elif v > target[key]:
                reasons.append(f'{val} {v:.2f} > {target[key]:g} {unit}')
    return not reasons, reasons


# --------------------------------------------------------------------
#  Bag -> score
# --------------------------------------------------------------------
def _f(x, nd=2):
    return None if x is None or not math.isfinite(x) else round(float(x), nd)


def score_run(run_dir, name, scenario):
    from nift_carla.tools import route_metrics as rm
    run = rm.load_run(run_dir)
    bag = rm.read_bag(run.bag, ['/tf', '/carla/gnss_fault', rm.ODOM])
    pairs = np.array([(t, rm.stamp_s(m.header.stamp)) for t, m in bag[rm.ODOM]])
    w, s = pairs[:, 0], pairs[:, 1]
    rows = [
        (
            t,
            rm.stamp_s(tr.header.stamp),
            tr.transform.translation.x,
            tr.transform.translation.y,
            rm.yaw_of(tr.transform.rotation),
        )
        for t, m in bag['/tf']
        for tr in m.transforms
        if tr.header.frame_id == MAP and tr.child_frame_id == BASE
    ]
    tf = np.array(rows, float).reshape(-1, 5)
    if len(tf):
        tf[:, 0] = rm.wall_to_sim(tf[:, 0], w, s)
    events = [json.loads(m.data) for _, m in bag['/carla/gnss_fault']]

    od = run.od
    t_nav = next((t for t, st in run.status if st == 'NAVIGATING'), od[0, 0])
    t_ss = next((t for t, st in run.status if st == 'SAFE_STOP'), None)
    t_end = od[-1, 0]
    if t_ss is not None:
        still = (od[:, 0] > t_ss) & (od[:, 4] < STILL_V)
        t_end = float(od[still, 0][0]) if still.any() else t_end
    win = fault_window(events, t_end)
    tt, pos, head, age, lat = belief_errors(tf, od, t_nav, t_end)
    inwin = (tt >= win[0]) & (tt <= win[1]) if win else np.ones(len(tt), bool)
    mx = lambda a, m: (  # noqa: E731
        float(np.nanmax(a[m])) if m.any() and np.isfinite(a[m]).any() else float('nan')
    )
    metrics = {}
    mpath = os.path.join(run_dir, 'metrics.json')
    if os.path.exists(mpath):
        with open(mpath) as f:
            metrics = json.load(f)
    info = run.info or {}
    first = win[0] if win else None
    sc = {
        'scenario': name,
        'route': scenario.get('route'),
        'outcome': info.get('outcome', metrics.get('outcome')),
        'stop': t_ss is not None,
        'stop_after_fault_s': _f(t_ss - first) if (t_ss is not None and first) else None,
        'fault_window_s': [_f(win[0]), _f(win[1])] if win else None,
        'err_max': _f(mx(pos, inwin)),
        'err_run_max': _f(mx(pos, np.isfinite(pos))),
        'lat_err_max': _f(mx(lat, inwin)),
        'head_err_max': _f(mx(head, inwin), 1),
        'age_max': _f(mx(age, inwin)),
        'cte_max': _f(metrics.get('cte_max')),
        'driven_m': _f(metrics.get('driven'), 1),
        'events': len(events),
        'tf_samples': int(len(tf)),
    }
    sc['passed'], sc['reasons'] = check(sc, scenario.get('target') or {})
    return sc


# --------------------------------------------------------------------
#  Running and reporting
# --------------------------------------------------------------------
def load_catalogue(path=None):
    import yaml
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory('nift_carla'), 'config', 'gnss_faults.yaml'
        )
    with open(path) as f:
        return yaml.safe_load(f)['scenarios']


def run_scenario(name, scenario, run_dir, backend, time_factor, extra='', tries=6):
    """
    Run sim_route_run for one scenario.

    Retried while a (transient) process on this machine looks like a stack.
    """
    cmd = [
        'ros2',
        'run',
        'nift_carla',
        'sim_route_run',
        str(scenario['route']),
        '--backend',
        backend,
        '--time-factor',
        f'{time_factor:g}',
        '--out',
        run_dir,
        '--extra',
        f'gnss_faults:={name} {extra}'.strip(),
    ]
    for _ in range(tries):
        p = subprocess.run(cmd, capture_output=True, text=True)
        with open(run_dir.rstrip('/') + '.console.log', 'w') as f:
            f.write('$ ' + ' '.join(cmd) + '\n' + p.stdout + p.stderr)
        if p.returncode == 2 and 'already running' in (p.stdout + p.stderr):
            time.sleep(5.0)
            continue
        return p.returncode
    return 2


def _cell(x, fmt='{:.2f}'):
    return '–' if x is None else fmt.format(x)


def scorecard_md(scores, catalogue, header=''):
    lines = [
        header,
        '',
        '| Scenario | Route | Outcome | Stop (s after fault) | Pose error max: fault / run (m) | '
        'of it lateral (m) '
        '| Heading err max (°) | TF age max (s) | CTE max (m) | Target | Result |',
        '|---|---|---|---|---|---|---|---|---|---|---|',
    ]
    for s in scores:
        tgt = ', '.join(
            f'{k} {v}' for k, v in (catalogue.get(s['scenario'], {}).get('target') or {}).items()
        )
        stop = ('yes, ' + _cell(s['stop_after_fault_s'], '{:+.2f}')) if s['stop'] else 'no'
        lines.append(
            f"| {s['scenario']} | {s['route']} | {s['outcome']} | {stop} | {_cell(s['err_max'])} "
            "/ "
            f"{_cell(s['err_run_max'])} | {_cell(s.get('lat_err_max'))} | "
            f"{_cell(s['head_err_max'], '{:.1f}')} | "
            f"{_cell(s['age_max'])} | "
            f"{_cell(s['cte_max'])} | {tgt} | "
            f"{'PASS' if s['passed'] else 'FAIL: ' + '; '.join(s['reasons'])} |"
        )
    n = sum(s['passed'] for s in scores)
    lines += ['', f'{n} of {len(scores)} scenarios pass.']
    return '\n'.join(lines) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Run and score the GNSS fault scenarios (config/gnss_faults.yaml).'
    )
    ap.add_argument('scenarios', nargs='*', help='scenario names (default: all)')
    ap.add_argument('--list', action='store_true', help='print the scenarios and exit')
    ap.add_argument('--catalogue', help='scenario file (default: the installed gnss_faults.yaml)')
    ap.add_argument('--out', help='output dir (default <repo>/nift_bags/gnss_bench/<date>)')
    ap.add_argument('--backend', choices=('kinematic', 'carla'), default='kinematic')
    ap.add_argument('--time-factor', type=float, default=1.0)
    ap.add_argument(
        '--extra', default='', help="more launch arguments for every run, e.g. 'gnss_feed:=real'"
    )
    ap.add_argument(
        '--resume', action='store_true', help='skip scenarios whose run dir already has run.json'
    )
    ap.add_argument('--score-only', metavar='DIR', help='score the runs in DIR (no driving)')
    a = ap.parse_args(argv)

    cat = load_catalogue(a.catalogue)
    if a.list:
        for name, sc in cat.items():
            print(f"{name:32s} route {sc['route']}  target {sc.get('target')}")
        return 0
    unknown = [n for n in a.scenarios if n not in cat]
    if unknown:
        sys.exit(f'sim_gnss_bench: unknown scenario(s) {unknown}; --list shows them')
    names = a.scenarios or list(cat)
    out = a.score_only or a.out
    if not out:
        from nift_bringup.components.bag import bag_dir
        out = os.path.join(bag_dir(), 'gnss_bench', datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)

    scores = []
    for k, name in enumerate(names, 1):
        run_dir = os.path.join(out, name)
        if not a.score_only and not (
            a.resume and os.path.exists(os.path.join(run_dir, 'run.json'))
        ):
            t0 = time.monotonic()
            rc = run_scenario(name, cat[name], run_dir, a.backend, a.time_factor, a.extra)
            print(
                f'[{k}/{len(names)}] {name}: sim_route_run exit {rc} ({time.monotonic() - t0:.0f} '
                's)',
                flush=True,
            )
        if not os.path.exists(os.path.join(run_dir, 'run.json')):
            print(f'  {name}: no run.json, not scored', flush=True)
            continue
        try:
            s = score_run(run_dir, name, cat[name])
        except Exception as e:              # a broken run must not stop the bench
            print(f'  {name}: cannot score ({e!r})', flush=True)
            continue
        scores.append(s)
        print(
            f"  {name}: {s['outcome']}, pose err {s['err_max']} m, "
            f"{'PASS' if s['passed'] else 'FAIL'} "
            f"{'; '.join(s['reasons'])}",
            flush=True,
        )
    header = (
        f'# GNSS fault bench — {datetime.now():%Y-%m-%d %H:%M}, backend {a.backend}, time factor '
        f'{a.time_factor:g}{", extra " + a.extra if a.extra else ""}'
    )
    with open(os.path.join(out, 'scorecard.json'), 'w') as f:
        json.dump(scores, f, indent=1)
    with open(os.path.join(out, 'scorecard.md'), 'w') as f:
        f.write(scorecard_md(scores, cat, header))
    print(f'scorecard: {os.path.join(out, "scorecard.md")}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
