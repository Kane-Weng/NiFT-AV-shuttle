#!/usr/bin/env python3
"""
Turn one sim_route_run run dir (bag + run.json) into the reference files the playground is checked against.

Runs inside the ROS 2 Humble workspace (it needs rosbag2_py and nift_carla's route_metrics):

    python3 extract_reference.py <run dir> <out dir>

Writes into <out dir> (every table gzipped, as <name>.csv.gz: the repo's .gitignore drops *.csv):
  summary.json   route, spawn, goal, outcome, timing, status sequence, route_metrics numbers
  odom.csv       ground-truth base_link from /ackermann_like_controller/odom: x, y, yaw, speed
  pose_tf.csv    the pose the stack believes, map -> base_link from /tf
  cmd.csv        the controller's 0x101 frames on /to_can_bus: gear, brake, pedal, steering (deg)
  telemetry.csv  /nav_telemetry: STATUS and the numeric fields
  dbw.csv        /carla/dbw_state: the virtual ECU's mode and what it applied
  gnss_fix.csv   /gnss/fix: lat, lon, status
  loc_status.csv /localization/status (only with localization:=ekf or shadow)
  gnss_fault.csv /carla/gnss_fault events (only with a fault scenario)
  events.csv     /rosout lines at WARN and above
  path.csv       the last /planned_path: x, y

Every row has t (sim time [s], from message stamps or mapped from receive time through the odom stamps) and
t_rel (t minus the stamp of the controller's first FORWARD 0x101 frame after the start, which is the
moment navigation started, in exact sim time).
"""

import csv
import gzip
import io
import json
import math
import os
import sys

from nift_carla.tools import route_metrics as rm
import numpy as np

EXTRA = ['/tf', '/carla/dbw_state', '/gnss/fix', '/localization/status', '/carla/gnss_fault', '/rosout']


def write_csv(path, header, rows):
    """Write path + '.gz' (gzip, mtime 0 so the same rows give the same bytes)."""
    with gzip.GzipFile(path + '.gz', 'wb', mtime=0) as gz, io.TextIOWrapper(gz, newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([f'{v:.6f}' if isinstance(v, float) else v for v in r])


def main(run_dir, out):
    os.makedirs(out, exist_ok=True)
    run = rm.load_run(run_dir)
    m = rm.compute(run)
    bag = rm.read_bag(run.bag, EXTRA + [rm.TELEMETRY])
    od = run.od
    # wall -> sim mapping through the odom stamps (as route_metrics does)
    raw = rm.read_bag(run.bag, [rm.ODOM])[rm.ODOM]
    pairs = sorted((rm.stamp_s(msg.header.stamp), t) for t, msg in raw)
    sim_s = np.array([p[0] for p in pairs])
    wall_s = np.array([p[1] for p in pairs])

    def to_sim(walls):
        return [float(x) for x in rm.wall_to_sim(walls, wall_s, sim_s)]

    t_start, t_end, after = rm.nav_window(run.status)
    # t_rel = 0 at the controller's first FORWARD 0x101 frame: a sim-time stamp, so it is exact and the same
    # event in every run (the /nav_telemetry time is mapped from receive time and jitters by tens of ms)
    fwd = run.can[(run.can[:, 1] == rm.Gear.FORWARD) & (run.can[:, 0] >= (t_start or 0.0) - 1.0)]
    t0 = float(fwd[0, 0]) if len(fwd) else (t_start if t_start is not None else float(od[0, 0]))

    write_csv(os.path.join(out, 'odom.csv'), ['t', 't_rel', 'x', 'y', 'yaw', 'v'],
              [(float(r[0]), float(r[0] - t0), float(r[1]), float(r[2]), float(r[3]), float(r[4])) for r in od])

    rows = []
    for _, msg in bag['/tf']:
        for tr in msg.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'base_link':
                t = rm.stamp_s(tr.header.stamp)
                p, q = tr.transform.translation, tr.transform.rotation
                rows.append((t, t - t0, p.x, p.y, rm.yaw_of(q)))
    rows.sort(key=lambda r: r[0])
    write_csv(os.path.join(out, 'pose_tf.csv'), ['t', 't_rel', 'x', 'y', 'yaw'], rows)

    write_csv(os.path.join(out, 'cmd.csv'), ['t', 't_rel', 'gear', 'brake', 'pedal', 'steer_deg'],
              [(float(c[0]), float(c[0] - t0), int(c[1]), int(c[2]), int(c[3]), float(c[4])) for c in run.can])

    tel = bag[rm.TELEMETRY]
    rows = []
    for t, (_, msg) in zip(to_sim([w for w, _ in tel]), tel):
        f = rm.parse_telemetry(msg.data)
        if 'STATUS' not in f:
            continue
        rows.append((t, t - t0, f['STATUS'], f.get('Filter', ''),
                     *[rm.first_number(f.get(k)) for k in ('Target V', 'Curr V', 'Obs Dist', 'Dist End', 'CTE')],
                     f.get('Next WP', '')))
    write_csv(os.path.join(out, 'telemetry.csv'),
              ['t', 't_rel', 'status', 'filter', 'target_v', 'curr_v', 'obs_dist', 'dist_end', 'cte', 'next_wp'], rows)

    dbw = bag['/carla/dbw_state']
    rows = []
    for t, (_, msg) in zip(to_sim([w for w, _ in dbw]), dbw):
        try:
            d = json.loads(msg.data)
        except ValueError:
            continue
        a = d.get('applied') or {}
        rows.append((t, t - t0, d.get('mode'), d.get('plant'), d.get('engaged'), d.get('eff_gear'),
                     d.get('eff_pedal'), d.get('eff_brake'), d.get('v_cmd'), d.get('v_set'),
                     a.get('wheel_deg'), a.get('throttle'), a.get('brake'), d.get('t')))
    write_csv(os.path.join(out, 'dbw.csv'),
              ['t', 't_rel', 'mode', 'plant', 'engaged', 'eff_gear', 'eff_pedal', 'eff_brake', 'v_cmd', 'v_set',
               'wheel_deg', 'throttle', 'brake', 'ecu_t'], rows)

    fixes = bag['/gnss/fix']
    write_csv(os.path.join(out, 'gnss_fix.csv'), ['t', 't_rel', 'lat', 'lon', 'status'],
              [(rm.stamp_s(msg.header.stamp), rm.stamp_s(msg.header.stamp) - t0, msg.latitude, msg.longitude,
                msg.status.status) for _, msg in fixes])

    if bag['/localization/status']:
        ls = bag['/localization/status']
        write_csv(os.path.join(out, 'loc_status.csv'), ['t', 't_rel', 'level', 'message', 'values'],
                  [(t, t - t0, int.from_bytes(bytes(msg.level), 'little') if isinstance(msg.level, (bytes, list))
                    else int(msg.level), msg.message, json.dumps({kv.key: kv.value for kv in msg.values}))
                   for t, (_, msg) in zip(to_sim([w for w, _ in ls]), ls)])
    if bag['/carla/gnss_fault']:
        gf = bag['/carla/gnss_fault']
        write_csv(os.path.join(out, 'gnss_fault.csv'), ['t', 't_rel', 'event'],
                  [(t, t - t0, msg.data) for t, (_, msg) in zip(to_sim([w for w, _ in gf]), gf)])

    # /rosout stamps are wall clock (the logger does not use sim time): map the receive time instead
    ro = bag['/rosout']
    rows = []
    for t, (_, msg) in zip(to_sim([w for w, _ in ro]), ro):
        if msg.level >= 30:   # WARN, ERROR, FATAL
            rows.append((t, t - t0, {30: 'WARN', 40: 'ERROR', 50: 'FATAL'}.get(msg.level, msg.level), msg.name,
                         msg.msg.replace('\n', ' | ')))
    write_csv(os.path.join(out, 'events.csv'), ['t', 't_rel', 'level', 'node', 'msg'], rows)

    write_csv(os.path.join(out, 'path.csv'), ['x', 'y'], [(float(x), float(y)) for x, y in run.path])

    info = run.info or {}
    summary = {
        'route': info.get('route'), 'name': info.get('name'), 'description': info.get('description'),
        'spawn': info.get('spawn'), 'goal': info.get('goal'),
        'run_outcome': info.get('outcome'), 'run_detail': info.get('detail'),
        'time_factor': info.get('time_factor'), 'launch_command': info.get('command'),
        'wall_s': info.get('wall_s'),
        't_rel_zero': t0, 'nav_start_t': t_start, 'nav_end_t': t_end, 'status_after_nav': after,
        'nav_duration_s': (t_end - t_start) if (t_start is not None and t_end is not None) else None,
        'final_pose': {'x': float(od[-1, 1]), 'y': float(od[-1, 2]), 'yaw': float(od[-1, 3])},
        'final_dist_to_goal': (math.hypot(od[-1, 1] - info['goal'][0], od[-1, 2] - info['goal'][1])
                               if info.get('goal') else None),
        'samples': {'odom': int(len(od)), 'cmd': int(len(run.can)), 'telemetry': len(tel)},
        'metrics': {k: v for k, v in m.items() if k != 'bag'},
    }
    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    print(f'{out}: {summary["metrics"]["outcome"]}, nav {summary["nav_duration_s"]}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
