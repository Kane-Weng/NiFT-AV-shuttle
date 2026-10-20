"""
Run one test route headless and natively, the way `sim_route_run` runs it on ROS, and write a trace.

    cd nift_playground/py
    python3 -m playground.cli --route 03 [--localization ekf] [--gnss-faults dropout_1.5s_turn] [--out DIR]
    python3 -m playground.cli --scenario run.json --out DIR      # replay a saved scenario
    python3 -m playground.cli --list

Everything goes through the worker protocol (`Session.handle_obj`), so the run is also a scenario the page can load: `scenario.json` in the output holds it, and `url.txt` the URL hash. The trace files have the format of `tests/reference/runs/` (README there), so the validation tools compare them directly.

Steps, as sim_route_run: ready (GNSS fix and TF flowing), goal until a /planned_path ends at it, /start_navigation (retried), then drive until
  arrived    /nav_telemetry leaves NAVIGATING for IDLE and the shuttle is still for STILL_S
  safe_stop  SAFE_STOP / ESTOP
  stalled    still for --stall-s while NAVIGATING
  timeout    the route's timeout_s of sim time
Exit code 0 when the route arrived, 1 when it did not, 2 when the run could not be set up.
"""

import argparse
import json
import math
import os
import sys
import time

STILL_V = 0.05          # route_metrics.STILL_V
MOVING_V = 0.3          # route_metrics.MOVING_V
STILL_S = 2.0           # route_metrics.STILL_S
GOAL_MATCH_M = 3.0      # route_run.GOAL_MATCH_M


class SetupError(Exception):
    pass


class RouteRun:
    """Drive a Session through one route; every step is a page message."""

    def __init__(self, session, say=None):
        self.s = session
        self.say = say or (lambda msg: None)
        self.state = {}
        self.paths = []                 # (tick, points)
        self.msgs = []                  # every worker message, when keep_msgs
        self.keep_msgs = False
        self.statuses = []              # (t, status) from state messages

    # ------------------------------------------------------------ protocol
    def _take(self, msgs):
        for m in msgs:
            if m['type'] == 'state':
                self.state = m
                if m.get('status') and (not self.statuses or self.statuses[-1][1] != m['status']):
                    self.statuses.append((m['t'], m['status']))
            elif m['type'] == 'path':
                self.paths.append((self.s.runner.tick, m['points']))
            elif m['type'] == 'error':
                raise SetupError(f"worker error: {m['message']}\n{m.get('traceback', '')}")
        if self.keep_msgs:
            self.msgs += msgs
        return msgs

    def send(self, msg):
        out = self._take(self.s.handle_obj(dict(msg, id='cli')))
        return next((m for m in out if m['type'] == 'reply' and m.get('id') == 'cli'), None)

    def advance(self, n=1):
        return self._take(self.s.advance_obj(n))

    def run_for(self, seconds, until=None):
        dt = self.s.runner.dt_ns / 1e9
        for _ in range(max(1, int(round(seconds / dt)))):
            self.advance(1)
            if until is not None and until():
                return True
        return until() if until is not None else True

    # ------------------------------------------------------------ steps
    def ready(self, timeout_s=10.0):
        ok = self.run_for(timeout_s, lambda: self.state.get('poseEst') is not None
                          and self.state.get('gnss', {}).get('fixAge') is not None)
        if not ok:
            raise SetupError(f'stack not ready after {timeout_s:.0f} s (sim): no GNSS fix or no map -> base_link')
        self.say(f'ready at t={self.state["t"]:.2f} s: GNSS fix and TF flowing')

    def goal(self, gx, gy, gyaw=None, timeout_s=30.0):
        n_seen = len(self.paths)
        t_end = self.s.runner.t + timeout_s
        while self.s.runner.t < t_end:
            self.send({'type': 'goal', 'x': gx, 'y': gy, 'yaw': gyaw})
            self.run_for(2.0, lambda: len(self.paths) > n_seen)
            for _, pts in self.paths[n_seen:]:
                if len(pts) < 2:
                    raise SetupError('waypoint_node published an empty /planned_path: no route to the goal')
                if math.hypot(pts[-1][0] - gx, pts[-1][1] - gy) <= GOAL_MATCH_M:
                    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
                    self.say(f'path: {len(pts)} points, {length:.1f} m')
                    return pts
            n_seen = len(self.paths)
        raise SetupError(f'no /planned_path to the goal ({gx}, {gy}) within {timeout_s:.0f} s')

    def start(self, attempts=10):
        reply = None
        for _ in range(attempts):
            reply = self.send({'type': 'cmd', 'name': 'start'})
            if reply and reply['ok']:
                self.say(f'/start_navigation: {reply["message"]}')
                return
            self.run_for(1.0)
        raise SetupError(f'/start_navigation refused: {reply["message"] if reply else "no reply"}')

    def drive(self, timeout_s, stall_s=10.0):
        """route_run.drive(), in sim time: return (outcome, detail)."""
        t0 = self.s.runner.t
        n_seen = len(self.statuses)
        seen_nav = left = still_since = None
        moved = False
        last_print = t0
        while True:
            self.advance(1)
            st = self.state
            now, v = st['t'], abs(st.get('speed') or 0.0)
            moved = moved or v > MOVING_V
            if v >= STILL_V:
                still_since = None
            elif still_since is None:
                still_since = now
            still = now - still_since if still_since is not None else 0.0
            for t, s in self.statuses[n_seen:]:
                if seen_nav is None and s == 'NAVIGATING':
                    seen_nav = t
                elif seen_nav is not None and left is None and s != 'NAVIGATING':
                    left = (t, s)
            n_seen = len(self.statuses)
            if now - last_print >= 10.0:
                last_print = now
                self.say(f'sim {now - t0:6.1f} s  v {v:4.2f} m/s  {st.get("status") or "-":<10} '
                         f'dist end {st.get("distEnd")}  CTE {st.get("cte")}')
            if left is not None:
                after = f'after {left[0] - (seen_nav if seen_nav is not None else t0):.1f} s (sim)'
                if left[1] == 'IDLE' and still >= STILL_S:
                    return 'arrived', f'NAVIGATING -> IDLE {after}, at rest'
                if left[1] in ('SAFE_STOP', 'ESTOP') and (still >= STILL_S or now - left[0] > 15):
                    return left[1].lower(), f'{left[1]} {after}'
                if left[1] not in ('IDLE', 'SAFE_STOP', 'ESTOP'):
                    return left[1].lower(), f'left NAVIGATING for {left[1]} {after}'
            elif still >= stall_s and seen_nav is not None:
                where = 'in NAVIGATING' if moved else 'without moving off'
                return 'stalled', f'still for {still:.1f} s (sim) {where}, {now - t0:.1f} s after the start'
            if now - t0 > timeout_s:
                return 'timeout', f'not done after {timeout_s:.0f} s (sim)'


# ================================================================ metrics and files

def route_metrics(trace, goal):
    """nift_carla's route_metrics on the trace (native only: numpy), or None if it cannot run."""
    try:
        import numpy as np
        from nift_carla.tools import route_metrics as rm
    except ImportError:
        return None
    tables = trace.tables()
    path = np.array(tables['path'][1], float).reshape(-1, 2)
    if len(path) < 2:
        return None
    od = np.array([r[:1] + r[2:6] for r in tables['odom'][1]], float).reshape(-1, 5)
    tel = [(r[0], r[4], r[7]) for r in tables['telemetry'][1] if r[4] is not None and r[7] is not None]
    can = np.array([[r[0], r[2], r[3], r[4], r[5]] for r in tables['cmd'][1]], float).reshape(-1, 5)
    run = rm.Run(path=path, od=od, status=list(trace.status), tel=np.array(tel, float).reshape(-1, 3), can=can,
                 goal=tuple(goal) if goal else None, n_paths=sum(1 for p in trace.paths if len(p) >= 2))
    m = rm.compute(run)
    return json.loads(json.dumps({k: v for k, v in m.items() if k != 'bag'}, default=_jsonable))


def _jsonable(o):
    try:
        import numpy as np
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
    except ImportError:
        pass
    return str(o)


def write_run(out_dir, session, trace, info):
    from .scenario import encode, save
    os.makedirs(out_dir, exist_ok=True)
    trace.write(out_dir)
    sc = dict(session.scenario(note=info.get('name')), ticks=session.runner.tick)
    save(os.path.join(out_dir, 'scenario.json'), sc)
    with open(os.path.join(out_dir, 'url.txt'), 'w') as f:
        f.write('#' + encode(sc) + '\n')
    with open(os.path.join(out_dir, 'summary.json'), 'w') as f:
        json.dump(info, f, indent=2, default=_jsonable)


def summary(trace, route, outcome, detail, metrics, wall_s, launch_desc) -> dict:
    t_start, t_end, after = trace.nav_window()
    od = trace.rows['odom']
    last = od[-1] if od else None
    goal = route.get('goal') if route else None
    return {
        'route': route.get('id') if route else None, 'name': route.get('name') if route else None,
        'description': route.get('description') if route else None,
        'spawn': route.get('spawn') if route else None, 'goal': goal[:2] if goal else None,
        'run_outcome': outcome, 'run_detail': detail, 'runtime': 'playground native',
        'python': sys.version.split()[0], 'launch': launch_desc['options'],
        'nodes': [n['name'] for n in launch_desc['nodes']], 'wall_s': round(wall_s, 2),
        't_rel_zero': trace.t_rel_zero(), 'nav_start_t': t_start, 'nav_end_t': t_end, 'status_after_nav': after,
        'nav_duration_s': (t_end - t_start) if t_start is not None and t_end is not None else None,
        'final_pose': {'x': last[1], 'y': last[2], 'yaw': last[3]} if last else None,
        'final_dist_to_goal': math.hypot(last[1] - goal[0], last[2] - goal[1]) if last and goal else None,
        'samples': {'odom': len(od), 'cmd': len(trace.rows['cmd']), 'telemetry': len(trace.rows['telemetry'])},
        'metrics': metrics,
    }


# ================================================================ main

def run_route(route_id, launch=None, out=None, timeout=None, stall_s=10.0, say=print, repo=None):
    """One route end to end. Return (exit code, summary dict, session)."""
    from .session import Session
    from . import launch_set as ls
    w0 = time.perf_counter()
    s = Session(repo)
    s.record = True
    route = dict(ls.routes()[route_id], id=route_id)
    rr = RouteRun(s, say)
    opts = dict(launch or {}, spawn=route_id)
    reply = rr.send({'type': 'init', 'launch': opts})
    if not reply or not reply['ok']:
        raise SetupError(f'init failed: {reply}')
    say(f'route {route_id} {route["name"]}: {route.get("description", "")}')
    try:
        rr.ready()
        gx, gy, gyaw = route['goal']
        rr.goal(gx, gy, gyaw)
        rr.start()
        outcome, detail = rr.drive(timeout if timeout is not None else route.get('timeout_s', 240), stall_s)
    except SetupError as e:
        say(f'setup failed: {e}')
        return 2, {'run_outcome': 'setup_failed', 'run_detail': str(e)}, s
    wall = time.perf_counter() - w0
    rr.run_for(1.0)             # the last telemetry and odom after the end, as the bag would hold
    trace = s.runner.observer.trace
    metrics = route_metrics(trace, route['goal'][:2])
    info = summary(trace, route, outcome, detail, metrics, wall, s.launch.describe())
    sim_s = s.runner.t
    say(f'{outcome}: {detail}; {sim_s:.1f} s sim in {wall:.1f} s wall ({sim_s / wall:.0f}x)')
    if out:
        write_run(out, s, trace, info)
        say(f'trace written to {out}')
    return (0 if outcome == 'arrived' else 1), info, s


def replay(path, out=None, ticks=None, say=print, repo=None):
    """Replay a saved scenario for `ticks` ticks (default: its own `ticks`, else 5 s after its last event)."""
    from .scenario import load
    from .session import Session
    sc = load(path)
    s = Session(repo)
    s.record = True
    out_msgs = s.handle_obj({'type': 'scenario_load', 'scenario': sc, 'id': 'cli'})
    err = [m for m in out_msgs if m['type'] == 'error']
    if err:
        raise SetupError(err[0]['message'])
    last = sc['events'][-1]['tick'] if sc['events'] else 0
    n = ticks if ticks is not None else sc.get('ticks', last + int(round(5.0 / (s.runner.dt_ns / 1e9))))
    s.advance_obj(n)
    trace = s.runner.observer.trace
    say(f'replayed {len(sc["events"])} event(s) over {n} ticks; final status {s.runner.observer.status}')
    if out:
        write_run(out, s, trace, {'scenario': path, 'ticks': n, 'final_status': s.runner.observer.status})
    return s


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python3 -m playground.cli', description=__doc__.split('\n\n')[0])
    ap.add_argument('--route', help='test route id (03) of test_routes.yaml')
    ap.add_argument('--scenario', help='replay a scenario.json (or a file with the URL hash)')
    ap.add_argument('--list', action='store_true', help='list the routes')
    ap.add_argument('--localization', choices=('direct', 'ekf', 'shadow'), default='direct')
    ap.add_argument('--gnss-feed', choices=('ideal', 'real'), default='ideal')
    ap.add_argument('--gnss-faults', default='', help='a scenario of gnss_faults.yaml')
    ap.add_argument('--gnss-dead-zones', default='', help='dead-zone set names, comma-separated')
    ap.add_argument('--gnss-seed', type=int, default=1)
    ap.add_argument('--gnss-checks', choices=('true', 'false'), default='true')
    ap.add_argument('--param', action='append', default=[], metavar='NODE.NAME=VALUE',
                    help='a node parameter override (YAML value), e.g. trajectory_controller.max_speed=1.5')
    ap.add_argument('--timeout', type=float, help="sim seconds after the start (default: the route's timeout_s)")
    ap.add_argument('--stall-s', type=float, default=10.0)
    ap.add_argument('--ticks', type=int, help='scenario replay: ticks to run')
    ap.add_argument('--out', help='trace directory')
    ap.add_argument('--repo', help='repo root (default: this checkout)')
    a = ap.parse_args(argv)

    from . import paths
    paths.setup(a.repo)
    from . import launch_set as ls
    if a.list:
        for rid, r in ls.routes().items():
            print(f'{rid}  {r["name"]:<36} {r.get("description", "")}')
        return 0
    if a.scenario:
        replay(a.scenario, a.out, a.ticks, repo=a.repo)
        return 0
    if not a.route:
        ap.error('--route or --scenario is needed')
    import yaml
    params = {}
    for p in a.param:
        key, eq, value = p.partition('=')
        node, dot, name = key.partition('.')
        if not eq or not dot or not node or not name or value == '':
            ap.error(f'--param {p}: expected NODE.NAME=VALUE')
        params.setdefault(node, {})[name] = yaml.safe_load(value)
    launch = {'localization': a.localization, 'gnss_feed': a.gnss_feed, 'gnss_faults': a.gnss_faults,
              'gnss_dead_zones': a.gnss_dead_zones, 'gnss_seed': a.gnss_seed, 'gnss_checks': a.gnss_checks,
              'params': params}
    try:
        code, _, _ = run_route(a.route, launch, a.out, a.timeout, a.stall_s, repo=a.repo)
    except SetupError as e:
        print(f'setup failed: {e}', file=sys.stderr)
        return 2
    return code


if __name__ == '__main__':
    sys.exit(main())
