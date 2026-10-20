#!/usr/bin/env python3
"""
Drive one playground session through a scenario and record what it says, natively or in Pyodide.

File: drive.py

Description:
  The same file runs in both runtimes. Natively, run_compare.mjs calls it as a script. In Pyodide
  the harness copies it into /nift and calls run(). It speaks only the worker protocol of ARCH.md
  section 8: Session.handle(json) for page messages and Session.advance(n) for ticks, exactly what
  web/src/worker/core.ts calls. So a trace that matches here means the browser runs the same sim.

  A scenario is ARCH.md section 8.5's format plus "ticks", the total number of ticks to run. Each
  event is applied after its tick, as the session's own replay does.

  The trace drops the values that depend on the wall clock (ARCH.md section 7): state.rtf, the
  diagnostics' wall time and real-time factor, and the runtime versions in ready. Everything else
  must match between the runtimes.

  python3 drive.py SCENARIO.json [--repo ROOT] [--out trace.json] [--chunk N]
"""

import argparse
import json
import platform
import re
import sys
import time
from pathlib import Path

WALL_KEYS = re.compile(r'wall|real.?time|rtf', re.IGNORECASE)
MAP_REL = 'maps/lanelet2_odr2024.osm'


def setup(repo_root):
    py = str(Path(repo_root) / 'nift_playground' / 'py')
    if py not in sys.path:
        sys.path.insert(0, py)
    from playground import paths
    paths.setup(str(repo_root))


def _unroot(obj, root):
    """Replace the repo root in every string by <root>, so /nift and the checkout compare equal."""
    if isinstance(obj, str):
        return re.sub(re.escape(root) + r'(?=/|$)', '<root>', obj)
    if isinstance(obj, list):
        return [_unroot(v, root) for v in obj]
    if isinstance(obj, dict):
        return {k: _unroot(v, root) for k, v in obj.items()}
    return obj


def scrub(msg, root=None):
    """Remove wall-clock values and the repo root from one protocol message. Returns it."""
    if root:
        msg = _unroot(msg, str(root))
    t = msg.get('type')
    if t == 'state':
        for k in ('rtf', 'factor', 'paused'):
            msg.pop(k, None)
    elif t == 'ready':
        for k in ('pyodide', 'python', 'bootMs'):
            msg.pop(k, None)
    elif t == 'diag':
        for item in msg.get('items', []):
            vals = item.get('values')
            if isinstance(vals, dict):
                for k in [k for k in vals if WALL_KEYS.search(k)]:
                    vals[k] = '<wall>'
            elif isinstance(vals, list):
                for kv in vals:
                    if isinstance(kv, dict) and WALL_KEYS.search(str(kv.get('key', ''))):
                        kv['value'] = '<wall>'
            if WALL_KEYS.search(str(item.get('message', ''))):
                item['message'] = '<wall>'
    elif t == 'log' and WALL_KEYS.search(str(msg.get('text', ''))):
        msg['text'] = re.sub(r'\d+(\.\d+)?', '#', msg['text'])
    return msg


def run(scenario, repo_root, chunk=100):
    """Run a scenario dict. Returns {'messages': [[tick, msg], ...], 'summary': {...}}."""
    setup(repo_root)
    from playground.session import Session

    session = Session(repo_root=str(repo_root))
    trace = []
    tick = 0

    def take(text):
        for m in json.loads(text):
            if m.get('type') == 'error':
                raise RuntimeError(f"session error at tick {tick}: {m.get('message')}\n{m.get('traceback', '')}")
            trace.append([tick, scrub(m, repo_root)])

    init = {'type': 'init', 'launch': scenario.get('launch', {}),
            'mapPath': str(Path(repo_root) / MAP_REL)}
    w0 = time.perf_counter()
    take(session.handle(json.dumps(init)))
    w_init = time.perf_counter() - w0

    events = sorted(scenario.get('events', []), key=lambda e: e['tick'])
    total = int(scenario['ticks'])
    w0 = time.perf_counter()

    def advance_to(target):
        nonlocal tick
        while tick < target:
            n = min(chunk, target - tick)
            text = session.advance(n)
            tick += n
            take(text)

    for i, ev in enumerate(events):
        advance_to(ev['tick'])
        msg = dict(ev['msg'])
        msg.setdefault('id', f'ev{i}')
        take(session.handle(json.dumps(msg)))
    advance_to(total)
    w_run = time.perf_counter() - w0

    states = [m for _, m in trace if m.get('type') == 'state']
    last = states[-1] if states else {}
    events_out = [m for _, m in trace if m.get('type') == 'event']
    summary = {
        'python': platform.python_version(),
        'platform': sys.platform,
        'ticks': tick,
        'sim_s': round(tick * 0.05, 3),
        'init_s': round(w_init, 3),
        'run_s': round(w_run, 3),
        'x_realtime': round(tick * 0.05 / w_run, 1) if w_run > 0 else None,
        'final_status': last.get('status'),
        'final_mode': last.get('mode'),
        'final_pose': last.get('pose'),
        'final_speed': last.get('speed'),
        'events': [[e.get('kind'), e.get('t')] for e in events_out],
        'messages': len(trace),
    }
    return {'messages': trace, 'summary': summary}


def main(argv=None):
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description='Run a playground scenario and write its trace.')
    ap.add_argument('scenario', type=Path)
    ap.add_argument('--repo', type=Path, default=here.parents[2])
    ap.add_argument('--out', type=Path)
    ap.add_argument('--chunk', type=int, default=100)
    a = ap.parse_args(argv)
    result = run(json.loads(a.scenario.read_text()), a.repo.resolve(), a.chunk)
    text = json.dumps(result)
    if a.out:
        a.out.write_text(text)
    print(json.dumps(result['summary']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
