#!/usr/bin/env python3
"""
Find spawn poses on straight lanelet chains, or list CARLA spawn points.

File: pick_spawn.py

Description:
  Helps choose where to put the shuttle on Mcity.

  Without --host: scans the Lanelet2 map for straight lanelet chains
  (>= --straight-min metres, heading spread <= --max-spread deg) and prints
  their start pose + evenly spaced waypoints in the map frame — paste into a
  scenario YAML (spawn_pose / waypoints).

  With --host: lists CARLA's spawn points converted to the map frame with the
  nearest lanelet id / distance, so a `spawn.x/y/yaw` for carla_params.yaml can
  be picked that is both on a CARLA road and on the Lanelet2 network.

Usage:
  ros2 run nift_carla carla_pick_spawn --straight-min 70
  ros2 run nift_carla carla_pick_spawn --host <ip>
"""

import argparse
import math
import sys

from nift_carla.carla_geo import CarlaGeo
from nift_carla.tools.lanelet_utils import (
    centerline_xy,
    default_map_path,
    heading_spread_deg,
    load_map,
    polyline_length,
    resample,
)
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON


def straight_chains(ll_map, min_len: float, max_spread: float):
    """Greedy: extend each lanelet through unique successors while the chain stays straight."""
    from lanelet2 import routing
    from lanelet2 import traffic_rules as tr
    rules = tr.create(tr.Locations.Germany, tr.Participants.Vehicle)
    graph = routing.RoutingGraph(ll_map, rules)
    out = []
    for ll in ll_map.laneletLayer:
        chain = [ll]
        pts = centerline_xy(ll)
        while True:
            nxt = graph.following(chain[-1])
            if len(nxt) != 1:
                break
            cand = pts + centerline_xy(nxt[0])[1:]
            if heading_spread_deg(cand) > max_spread or nxt[0] in chain:
                break
            chain.append(nxt[0])
            pts = cand
        length = polyline_length(pts)
        if length >= min_len and heading_spread_deg(pts) <= max_spread:
            out.append((length, [c.id for c in chain], pts))
    # de-duplicate chains that are suffixes of longer ones
    out.sort(key=lambda t: -t[0])
    kept = []
    seen = set()
    for length, ids, pts in out:
        if ids[0] in seen:
            continue
        seen.update(ids)
        kept.append((length, ids, pts))
    return kept


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument('--map', default=default_map_path())
    ap.add_argument('--datum', nargs=2, type=float, default=[MAP_ORIGIN_LAT, MAP_ORIGIN_LON],
                    metavar=('LAT', 'LON'))
    ap.add_argument('--host', default='')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--straight-min', type=float, default=70.0)
    ap.add_argument('--max-spread', type=float, default=3.0)
    ap.add_argument('--step', type=float, default=10.0, help='waypoint spacing (m)')
    ap.add_argument('--top', type=int, default=8)
    args = ap.parse_args(argv)

    ll_map = load_map(args.map, args.datum[0], args.datum[1])
    geo = CarlaGeo(args.datum[0], args.datum[1])

    if not args.host:
        chains = straight_chains(ll_map, args.straight_min, args.max_spread)
        print(
            f'{len(chains)} straight chains >= {args.straight_min} m (spread <= {args.max_spread} '
            'deg):'
        )
        for length, ids, pts in chains[:args.top]:
            yaw = math.atan2(pts[-1][1] - pts[0][1], pts[-1][0] - pts[0][0])
            wps = resample(pts, args.step)
            lat, lon = geo.map_to_wgs(*pts[0])
            print(
                f'\n# lanelets {ids}  length {length:.1f} m  start ({pts[0][0]:.2f}, '
                f'{pts[0][1]:.2f}) '
                f'yaw {yaw:.4f} rad  lat/lon ({lat:.7f}, {lon:.7f})'
            )
            print(
                f'spawn_pose: {{x: {pts[0][0]:.2f}, y: {pts[0][1]:.2f}, z: 0.5, yaw: {yaw:.6f}}}'
            )
            print('waypoints:')
            for x, y in wps:
                print(f'  - [{x:.2f}, {y:.2f}]')
        return 0

    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    cmap = client.get_world().get_map()
    geo = CarlaGeo.from_opendrive(cmap.to_opendrive(), args.datum[0], args.datum[1])
    from lanelet2 import geometry as lg
    from lanelet2.core import BasicPoint2d
    print('idx   map x     map y    yaw(rad)   nearest lanelet  dist(m)')
    for i, tf in enumerate(cmap.get_spawn_points()):
        x, y = geo.carla_to_map(tf.location.x, tf.location.y)
        yaw = geo.carla_yaw_to_map_yaw(tf.location.x, tf.location.y, tf.rotation.yaw)
        near = lg.findNearest(ll_map.laneletLayer, BasicPoint2d(x, y), 1)
        if near:
            d, ll = near[0]
            print(f'{i:3d}  {x:8.2f}  {y:8.2f}  {yaw:8.4f}   {ll.id:10d}   {d:6.2f}')
        else:
            print(f'{i:3d}  {x:8.2f}  {y:8.2f}  {yaw:8.4f}   -')
    return 0


if __name__ == '__main__':
    sys.exit(main())
