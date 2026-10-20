#!/usr/bin/env python3
"""Spike 0.2: the unchanged nift_control trajectory_controller on fakeros, closed loop with the real sim cores.

Copied from the Phase 0 spike as the scaffold's smoke test (wave A). It now runs against this repo:
    python3 nift_playground/tests/spike/run_spike.py --repo . --out <dir> [--scenario nominal|gnss_loss|all]
Writes <out>/<scenario>_<runtime>.csv (one row per sim tick) and <out>/summary_<runtime>.json.
"""
import argparse
import csv
import json
import math
import platform
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PY = HERE.parents[1] / 'py'          # nift_playground/py


def setup_paths(repo: Path):
    for p in (PY / 'fakeros', PY, HERE, repo / 'nift_common', repo / 'nift_control', repo / 'nift_simulation/nift_carla'):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))


def flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        key = f'{prefix}.{k}' if prefix else k
        if isinstance(v, dict):
            out.update(flatten(v, key))
        else:
            out[key] = v
    return out


def node_params(yaml_path: Path, node: str):
    import yaml
    return flatten(yaml.safe_load(yaml_path.read_text())[node]['ros__parameters'])


def test_path(step=0.5):
    """Straight 30 m, left 90 deg R 10 m, straight 20 m, right 90 deg R 12 m, straight 25 m (map frame)."""
    pts, x, y, yaw = [(0.0, 0.0)], 0.0, 0.0, 0.0

    def straight(length):
        nonlocal x, y
        for _ in range(int(round(length / step))):
            x, y = x + step * math.cos(yaw), y + step * math.sin(yaw)
            pts.append((x, y))

    def arc(radius, angle):        # angle > 0 = left
        nonlocal x, y, yaw
        n = int(round(abs(angle) * radius / step))
        dth = angle / n
        for _ in range(n):
            cx, cy = x - math.copysign(radius, angle) * math.sin(yaw), y + math.copysign(radius, angle) * math.cos(yaw)
            yaw += dth
            x, y = cx + math.copysign(radius, angle) * math.sin(yaw), cy - math.copysign(radius, angle) * math.cos(yaw)
            pts.append((x, y))

    straight(30.0)
    arc(10.0, math.pi / 2)
    straight(20.0)
    arc(12.0, -math.pi / 2)
    straight(25.0)
    return pts


def run(repo: Path, scenario: str, out_dir: Path, runtime: str, t_max=120.0):
    setup_paths(repo)
    from fakeros._core import world, QoSProfile, DurabilityPolicy
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Path as PathMsg
    from geometry_msgs.msg import PoseStamped
    from std_srvs.srv import Trigger

    world.reset()
    logs = []
    world.log_sink = lambda lv, name, text: logs.append((round(world.now_ns / 1e9 - 1.0, 3), lv, name, text))
    world.log_level = 20

    params = node_params(repo / 'nift_control/config/planning.yaml', 'trajectory_controller')
    params.update(node_params(repo / 'nift_control/config/adrc_params.yaml', 'trajectory_controller'))
    params.update({'filter_type': 'none', 'localization_source': 'gnss'})     # sim_kinematic: no perception
    world.params['trajectory_controller'] = params
    sim_params = node_params(repo / 'nift_simulation/nift_carla/config/carla_params.yaml', 'carla_bridge')
    world.params['kinematic_sim'] = {k: v for k, v in sim_params.items()
                                     if k.startswith(('plant.', 'dbw.cmd_timeout', 'fixed_delta'))}

    from nift_common.vehicle import ShuttleConfig
    blx = ShuttleConfig().base_link_x
    from web_sim_node import WebSimNode
    rclpy.init()
    sim = WebSimNode(spawn=(blx, 0.0, 0.0), gnss_off_after_s=21.0 if scenario == 'gnss_loss' else None)

    import nift_control.trajectory_controller as tc_mod           # the repo file, unchanged
    ctrl = tc_mod.TrajectoryController()

    # operator: a planner-like latched path at 0.5 s, /start_navigation at 1.0 s
    op = Node('operator')
    path_pub = op.create_publisher(PathMsg, '/planned_path', QoSProfile(depth=1,
                                                                         durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start = op.create_client(Trigger, '/start_navigation')
    pts = test_path()
    state = {'path_sent': False, 'started': None, 'was_auto': False, 'done_at': None}
    rows = []

    def operator_tick():
        t = world.now_ns / 1e9 - 1.0
        if not state['path_sent'] and t >= 0.5:
            msg = PathMsg()
            msg.header.frame_id = 'map'
            for x, y in pts:
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.pose.position.x, ps.pose.position.y = x, y
                ps.pose.orientation.w = 1.0
                msg.poses.append(ps)
            path_pub.publish(msg)
            state['path_sent'] = True
        if state['started'] is None and t >= 1.0:
            r = start.call(Trigger.Request())
            state['started'] = (r.success, r.message)
        x, y, yaw = sim.car.base_link()
        pedal, brake, hold = sim.applied
        mode = ctrl._mode.name
        state['was_auto'] |= mode == 'AUTONOMOUS'
        if state['was_auto'] and mode != 'AUTONOMOUS' and state['done_at'] is None:
            state['done_at'] = t
        rows.append((round(t, 3), x, y, yaw, sim.car.v, sim.car.steer_deg, pedal, brake, int(hold), mode,
                     sim.ecu.mode(world.now_ns / 1e9).value))

    op.create_timer(0.05, operator_tick)           # created last: runs after sim + controller at each tick time

    def stop():         # 1 s after the shuttle has stopped once it left AUTONOMOUS (cap 20 s)
        if state['done_at'] is None:
            return False
        after = world.now_ns / 1e9 - 1.0 - state['done_at']
        return after > 20.0 or (after > 1.0 and abs(sim.car.v) < 0.01)

    w0 = time.perf_counter()
    world.run_until(world.now_ns + int(t_max * 1e9), stop=stop)
    wall = time.perf_counter() - w0
    sim_s = world.now_ns / 1e9 - 1.0

    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / f'{scenario}_{runtime}.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'yaw', 'v', 'wheel_deg', 'pedal', 'brake', 'hold', 'ctrl_mode', 'ecu_mode'])
        w.writerows(rows)
    with open(out_dir / f'{scenario}_{runtime}_log.txt', 'w') as f:
        for t, lv, name, text in logs:
            f.write(f'[{t:8.3f}] {lv} [{name}] {text}\n')
    return {
        'scenario': scenario, 'runtime': runtime, 'python': platform.python_version(),
        'sim_s': round(sim_s, 2), 'wall_s': round(wall, 3), 'x_realtime': round(sim_s / wall, 1),
        'start_navigation': state['started'], 'left_autonomous_at_s': state['done_at'], 'final_mode': ctrl._mode.name,
        'final_v': round(sim.car.v, 4), 'ticks': len(rows), 'world': dict(world.stats),
        'errors': [l[3] for l in logs if l[1] >= 40][:5],
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--scenario', default='all')
    ap.add_argument('--runtime', default='native')
    a = ap.parse_args(argv)
    scenarios = ['nominal', 'gnss_loss'] if a.scenario == 'all' else [a.scenario]
    results = [run(a.repo, s, a.out, a.runtime) for s in scenarios]
    (a.out / f'summary_{a.runtime}.json').write_text(json.dumps(results, indent=1))
    for r in results:
        print(json.dumps({k: v for k, v in r.items() if k != 'world'}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
