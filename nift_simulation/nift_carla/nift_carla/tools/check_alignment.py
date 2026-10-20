#!/usr/bin/env python3
"""
Measure how well a Lanelet2 map lines up with the CARLA driving lanes.

File: check_alignment.py

Description:
  Quantifies how well a Lanelet2 map (default: the sim's generated map)
  lines up with the digital twin's OpenDRIVE: every resampled Lanelet2 centerline
  point is projected onto the nearest CARLA driving-lane waypoint (queried at the
  road's height, road_height.py) and the lateral offset is reported.

  Interpretation
    p95 < 0.5 m, |mean bias| < 0.3 m  -> good; the stack can drive CARLA roads
                                          from Lanelet2 routes
    constant bias vector               -> datum / geoReference mismatch
    similarity fit with |rotation| or |scale-1| clearly non-zero and a small
    residual                           -> projection / sign mistake in carla_geo
    similarity fit ~identity but residual still large, few lanelets aligned
                                       -> the two MAPS disagree (a hand-made
                                          lanelet2 vs surveyed OpenDRIVE); no
                                          transform fixes it — regenerate the
                                          Lanelet2 map from the .xodr

  Results on 2026-10-13 (--step 2, source-built 2025 world), the generated 2025 map:
    median 0.019 m, p95 0.157 m,
    544/545 lanelets within 1 m -> aligned (lanelet 97 near map (42, 115) is 12-16 m off).
  Results on 2026-10-11 (--step 1, McityMap_Main, the packaged 2024 world):
    generated 2024 map                       median 0.024 m, p95 0.164 m, 506/507 lanelets
                                             within 1 m -> aligned (the one left: a
                                             two-way lane CARLA doesn't return as Driving)
  Until 2026-10-11 the waypoints were queried at z = 0 while the twin's roads sit at
  z 266..279 m, which returned other roads.

Usage:
  ros2 run nift_carla carla_check_alignment --host <ip> [--step 5] [--worst 10]
"""

import argparse
import math
import statistics
import sys

import carla
from nift_carla.carla_geo import CarlaGeo
from nift_carla.tools.lanelet_utils import centerline_xy, default_map_path, load_map, resample
from nift_carla.tools.road_height import RoadHeight
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
import numpy as np


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--map', default=default_map_path())
    ap.add_argument('--datum', nargs=2, type=float, default=[MAP_ORIGIN_LAT, MAP_ORIGIN_LON],
                    metavar=('LAT', 'LON'))
    ap.add_argument('--step', type=float, default=5.0)
    ap.add_argument('--worst', type=int, default=10)
    args = ap.parse_args(argv)

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    cmap = client.get_world().get_map()
    geo = CarlaGeo.from_opendrive(cmap.to_opendrive(), args.datum[0], args.datum[1])
    ll_map = load_map(args.map, args.datum[0], args.datum[1])
    # query at the road's height: at z = 0 get_waypoint picks other roads
    height = RoadHeight(cmap)

    offsets, dx, dy, worst, P, Q, per_ll = [], [], [], [], [], [], {}
    for ll in ll_map.laneletLayer:
        for x, y in resample(centerline_xy(ll), args.step):
            cx, cy, _ = geo.map_to_carla(x, y)
            # Bidirectional too: the twin's two-way lanes (lanelets tagged one_way=no) are that
            # type in CARLA
            wp = cmap.get_waypoint(
                carla.Location(x=cx, y=cy, z=height(cx, cy)),
                project_to_road=True,
                lane_type=carla.LaneType.Driving | carla.LaneType.Bidirectional,
            )
            if wp is None:
                continue
            ex, ey = wp.transform.location.x - cx, wp.transform.location.y - cy
            d = math.hypot(ex, ey)
            offsets.append(d)
            mx, my = geo.carla_to_map(wp.transform.location.x, wp.transform.location.y)
            dx.append(mx - x)
            dy.append(my - y)
            worst.append((d, ll.id, x, y))
            P.append((x, y))
            Q.append((mx, my))
            per_ll.setdefault(ll.id, []).append(d)

    if not offsets:
        print('no samples — is the Mcity map loaded?')
        return 1
    offsets_sorted = sorted(offsets)
    p95 = offsets_sorted[int(0.95 * len(offsets_sorted)) - 1]
    print(f'{len(offsets)} samples from {len(ll_map.laneletLayer)} lanelets, step {args.step} m')
    print(
        f'lateral offset: mean {statistics.mean(offsets):.3f}  median '
        f'{statistics.median(offsets):.3f}  '
        f'p95 {p95:.3f}  max {max(offsets):.3f} m'
    )
    print(
        f'mean bias vector (map frame, CARLA lane - lanelet): ({statistics.mean(dx):+.3f}, '
        f'{statistics.mean(dy):+.3f}) m'
    )
    print(f'\nworst {args.worst}:')
    for d, lid, x, y in sorted(worst, reverse=True)[:args.worst]:
        print(f'  {d:6.2f} m  lanelet {lid}  map ({x:.1f}, {y:.1f})')

    # Similarity fit (scale, rotation, translation) on robust inliers: tells a projection
    # error in carla_geo (fit != identity, small residual) from disagreeing maps.
    P, Q = np.array(P), np.array(Q)
    keep = np.ones(len(P), bool)
    for _ in range(5):
        s, R, t = _similarity(P[keep], Q[keep])
        res = np.hypot(*((s * (R @ P.T)).T + t - Q).T)
        keep = res < max(1.0, float(np.percentile(res[keep], 80)))
    rot = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    aligned = sum(1 for v in per_ll.values() if statistics.median(v) < 1.0)
    print(
        f'\nsimilarity fit on {int(keep.sum())} inliers: scale {s:.5f}, rotation {rot:+.3f} deg, '
        f'translation ({t[0]:+.2f}, {t[1]:+.2f}) m; residual over all points median '
        f'{float(np.median(res)):.2f} p90 {float(np.percentile(res, 90)):.2f} m'
    )
    print(f'lanelets with median offset < 1 m: {aligned} / {len(per_ll)}')

    ok = p95 < 0.5 and math.hypot(statistics.mean(dx), statistics.mean(dy)) < 0.3
    fit_explains = float(np.percentile(res, 95)) < 0.5         # after the fit the points agree
    if ok:
        verdict = 'aligned'
    elif fit_explains or abs(rot) > 0.5 or abs(s - 1.0) > 0.01:
        verdict = 'systematic transform error — check datum / georeference / y-flip in carla_geo'
    else:
        verdict = (
            'maps disagree locally (no global transform fixes it) — the map was not generated '
            'from this OpenDRIVE: maps/utils/odr2lanelet.py'
        )
    print('\nRESULT:', verdict)
    return 0 if ok else 2


def _similarity(P, Q):
    pc, qc = P.mean(0), Q.mean(0)
    Pc, Qc = P - pc, Q - qc
    U, S, Vt = np.linalg.svd(Pc.T @ Qc)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1] *= -1
        R = Vt.T @ U.T
    s = S.sum() / (Pc ** 2).sum()
    return s, R, qc - s * (R @ pc)


if __name__ == '__main__':
    sys.exit(main())
