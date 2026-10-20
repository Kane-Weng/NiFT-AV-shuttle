#!/usr/bin/env python3
"""
Evaluate a rosbag of a CARLA route against ground truth.

File: eval_route.py

Description:
  Post-run evaluation of a rosbag recorded during a CARLA route:

    ground-truth CTE   /ackermann_like_controller/odom (bridge ground truth, map frame)
                       vs. the latest /planned_path
    GNSS pose RMSE     /gnss/fix + /gnss/heading_deg (-> base_link via the antenna
                       lever arm) vs. ground truth at the nearest odom stamp
    heading error      /gnss/heading_deg vs. ground-truth yaw
    speed stats        from odom

  This is the baseline run (and the degraded-GNSS runs with gps.noise_* /
  gps.dropout_* set on the bridge).

Usage:
  ros2 bag record -o run1 /planned_path /ackermann_like_controller/odom /gnss/fix
      /gnss/heading_deg /nav_telemetry /clock
  ros2 run nift_carla carla_eval_route run1 [--lever-arm <m>]
"""

import argparse
import bisect
import math
import statistics
import sys

from nift_carla.carla_geo import CarlaGeo
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
from nift_common.vehicle import ShuttleConfig
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(path, topics):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=''),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'
        ),
    )
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in out:
            out[topic].append((t_ns * 1e-9, deserialize_message(data, types[topic])))
    return out


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cte(px, py, path):
    best = float('inf')
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
        d = math.hypot(px - (ax + t * vx), py - (ay + t * vy))
        best = min(best, d)
    return best


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument('bag')
    ap.add_argument('--datum', nargs=2, type=float, default=[MAP_ORIGIN_LAT, MAP_ORIGIN_LON],
                    metavar=('LAT', 'LON'))
    ap.add_argument(
        '--lever-arm',
        type=float,
        default=ShuttleConfig().gnss_front_x,
        help='front antenna x offset from base_link (m; default: vehicle.yaml gnss_front_x)',
    )
    ap.add_argument('--odom', default='/ackermann_like_controller/odom')
    args = ap.parse_args(argv)

    bag = read_bag(args.bag, ['/planned_path', args.odom, '/gnss/fix', '/gnss/heading_deg'])
    geo = CarlaGeo(args.datum[0], args.datum[1])

    paths = [
        (t, [(p.pose.position.x, p.pose.position.y) for p in m.poses])
        for t, m in bag['/planned_path']
    ]
    paths = [(t, p) for t, p in paths if len(p) >= 2]
    odom = [(t, m) for t, m in bag[args.odom]]
    if not odom:
        print('no odometry in bag')
        return 1
    odom_t = [t for t, _ in odom]

    # ground-truth CTE against the path active at each odom sample
    ctes, speeds = [], []
    for t, m in odom:
        active = [p for pt, p in paths if pt <= t]
        if active:
            ctes.append(cte(m.pose.pose.position.x, m.pose.pose.position.y, active[-1]))
        speeds.append(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))

    # GNSS-derived pose vs ground truth
    headings = [(t, m.data) for t, m in bag['/gnss/heading_deg']]
    heading_t = [t for t, _ in headings]
    pos_err, head_err = [], []
    for t, fix in bag['/gnss/fix']:
        if not headings:
            break
        i = min(bisect.bisect_left(heading_t, t), len(headings) - 1)
        h = math.radians(headings[i][1])
        fx, fy = geo.wgs_to_map(fix.latitude, fix.longitude)
        bx, by = fx - args.lever_arm * math.sin(h), fy - args.lever_arm * math.cos(h)
        j = min(bisect.bisect_left(odom_t, t), len(odom) - 1)
        gt = odom[j][1].pose.pose
        pos_err.append(math.hypot(bx - gt.position.x, by - gt.position.y))
        gt_yaw = yaw_of(gt.orientation)
        yaw = math.pi / 2 - h
        head_err.append(
            math.degrees(abs(math.atan2(math.sin(yaw - gt_yaw), math.cos(yaw - gt_yaw))))
        )

    def stats(name, xs, unit):
        if not xs:
            print(f'{name:22s} n/a')
            return
        xs_sorted = sorted(xs)
        print(
            f'{name:22s} n={len(xs):5d}  mean {statistics.mean(xs):7.3f}  rms '
            f'{math.sqrt(sum(v*v for v in xs)/len(xs)):7.3f}  '
            f'p95 {xs_sorted[int(0.95*len(xs))-1]:7.3f}  max {max(xs):7.3f} {unit}'
        )

    print(
        f'bag {args.bag}: {len(odom)} odom, {len(bag["/gnss/fix"])} fixes, {len(paths)} path(s), '
        f'duration {odom_t[-1]-odom_t[0]:.1f} s'
    )
    stats('ground-truth CTE', ctes, 'm')
    stats('GNSS pose error', pos_err, 'm')
    stats('heading error', head_err, 'deg')
    stats('speed', speeds, 'm/s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
