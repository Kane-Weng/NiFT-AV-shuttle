#!/usr/bin/env python3
"""
Check a CARLA server with the digital twin: map, georeference, blueprints, tick rate.

File: smoke_test.py

Description:
  First check of a CARLA server carrying the Mcity Digital Twin.  No ROS.

    1. server version, McityMap_Main listed
    2. load_world, spawn points, <geoReference> matches the expected tmerc
    3. CARLA GNSS sensor vs. pyproj tmerc at a few spawn points (metre error)
    4. blueprint survey: wheelbase / bbox of candidate ego vehicles
    5. synchronous-mode tick benchmark with a LiDAR attached

Usage:
  ros2 run nift_carla carla_smoke_test --host <ip> [--port 2000] [--map McityMap_Main]
      [--skip-load]
  python3 -m nift_carla.tools.smoke_test --host <ip>
"""

import argparse
import math
import statistics
import sys
import time

import carla
from nift_carla.carla_geo import CarlaGeo, MCITY_TMERC, parse_georeference
from nift_carla.carla_lidar import lidar_attributes
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
from nift_common.vehicle import ShuttleConfig

CANDIDATES = ['vehicle.micro.microlino', 'vehicle.mini.cooper_s_2021', 'vehicle.nissan.micra',
              'vehicle.seat.leon', 'vehicle.toyota.prius', 'vehicle.mini.cooper_s']
NIFT_WHEELBASE = ShuttleConfig().wheel_base     # blueprint survey targets
NIFT_LENGTH = ShuttleConfig().length


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--map', default='McityMap_Main')
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--skip-load', action='store_true', help='use the currently loaded world')
    ap.add_argument('--ticks', type=int, default=200)
    ap.add_argument('--datum', nargs=2, type=float, default=[MAP_ORIGIN_LAT, MAP_ORIGIN_LON],
                    metavar=('LAT', 'LON'))
    args = ap.parse_args(argv)

    results = []

    def report(name, ok, detail=''):
        results.append((name, ok))
        print(f'[{"PASS" if ok else "FAIL"}] {name}  {detail}')

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    # 1 ---------------------------------------------------------------
    version = client.get_server_version()
    maps = client.get_available_maps()
    print(f'server {version}, client {client.get_client_version()}')
    print('available maps:', *maps, sep='\n  ')
    report('map listed', any(m.endswith(args.map) for m in maps), f'looking for {args.map}')

    # 2 ---------------------------------------------------------------
    world = client.get_world()
    if not args.skip_load and not world.get_map().name.endswith(args.map):
        t0 = time.perf_counter()
        world = client.load_world(args.map)
        print(f'load_world took {time.perf_counter() - t0:.1f} s')
    cmap = world.get_map()
    spawn_points = cmap.get_spawn_points()
    report('spawn points', len(spawn_points) > 0, f'{len(spawn_points)} points, map {cmap.name}')
    georef = parse_georeference(cmap.to_opendrive())
    print('georeference:', georef)
    report(
        'georeference',
        georef == MCITY_TMERC,
        '' if georef == MCITY_TMERC else f'expected {MCITY_TMERC}',
    )
    geo = CarlaGeo(args.datum[0], args.datum[1], georef)
    print(
        f'grid convergence {geo.grid_convergence_deg():.3f} deg; datum -> CARLA '
        f'{geo.map_to_carla(0, 0)}'
    )

    orig = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    bp_lib = world.get_blueprint_library()
    actors = []
    try:
        # 3 -----------------------------------------------------------
        errs = []
        gnss_bp = bp_lib.find('sensor.other.gnss')
        for i in sorted({0, len(spawn_points) // 2, len(spawn_points) - 1}):
            tf = spawn_points[i]
            veh = world.try_spawn_actor(bp_lib.filter('vehicle.tesla.model3')[0], tf)
            if veh is None:
                continue
            actors.append(veh)
            box = {}
            g = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=veh)
            actors.append(g)
            g.listen(lambda d: box.__setitem__('m', d))
            for _ in range(3):
                world.tick()
            g.stop()
            if 'm' not in box:
                continue
            m = box['m']
            loc = g.get_transform().location
            lat, lon = geo.carla_to_wgs(loc.x, loc.y)
            e = haversine_m(m.latitude, m.longitude, lat, lon)
            errs.append(e)
            mx, my = geo.carla_to_map(loc.x, loc.y)
            print(
                f'  spawn {i}: CARLA ({loc.x:.1f}, {loc.y:.1f}) map ({mx:.1f}, {my:.1f}) '
                f'carla-gnss ({m.latitude:.7f}, {m.longitude:.7f}) tmerc ({lat:.7f}, {lon:.7f}) '
                f'err {e:.3f} m'
            )
            veh.destroy()
            g.destroy()
            actors.remove(veh)
            actors.remove(g)
        report(
            'CARLA GNSS vs tmerc',
            bool(errs) and max(errs) < 1.0,
            (
                f'max {max(errs):.3f} m (bridge uses tmerc, so this is informational)'
                if errs
                else 'no samples'
            ),
        )

        # 4 -----------------------------------------------------------
        print(
            'blueprint survey (target wheelbase %.3f m, length %.3f m):'
            % (NIFT_WHEELBASE, NIFT_LENGTH)
        )
        best = None
        tf = spawn_points[0]
        for bp_id in CANDIDATES:
            found = bp_lib.filter(bp_id)
            if not found:
                print(f'  {bp_id:32s} not available')
                continue
            veh = world.try_spawn_actor(found[0], tf)
            if veh is None:
                print(f'  {bp_id:32s} spawn failed')
                continue
            world.tick()
            pc = veh.get_physics_control()
            ws = [
                (w.position.x / 100.0, w.position.y / 100.0, w.position.z / 100.0)
                for w in pc.wheels
            ]
            wb = 0.0
            if len(ws) >= 4:
                fx, fy = (ws[0][0] + ws[1][0]) / 2, (ws[0][1] + ws[1][1]) / 2
                rx, ry = (ws[2][0] + ws[3][0]) / 2, (ws[2][1] + ws[3][1]) / 2
                wb = math.hypot(fx - rx, fy - ry)
            ext = veh.bounding_box.extent
            steer = pc.wheels[0].max_steer_angle if pc.wheels else 0.0
            score = abs(wb - NIFT_WHEELBASE) + 0.3 * abs(2 * ext.x - NIFT_LENGTH)
            print(
                f'  {bp_id:32s} wheelbase {wb:.3f} m  L {2*ext.x:.2f} W {2*ext.y:.2f} H '
                f'{2*ext.z:.2f} m  '
                f'mass {pc.mass:.0f} kg  max_steer {steer:.0f} deg  score {score:.3f}'
            )
            if best is None or score < best[1]:
                best = (bp_id, score)
            veh.destroy()
        report('blueprint survey', best is not None, f'closest: {best[0]}' if best else '')

        # 5 -----------------------------------------------------------
        veh = world.try_spawn_actor(
            bp_lib.filter(best[0] if best else 'vehicle.tesla.model3')[0], tf
        )
        actors.append(veh)
        lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
        for k, v in lidar_attributes(0.05).items():
            if lidar_bp.has_attribute(k):
                lidar_bp.set_attribute(k, v)
        counts = []
        lidar = world.spawn_actor(
            lidar_bp,
            carla.Transform(carla.Location(x=1.5, z=2.0), carla.Rotation(pitch=-15.0)),
            attach_to=veh,
        )
        actors.append(lidar)
        lidar.listen(lambda d: counts.append(len(d)))
        times = []
        for _ in range(args.ticks):
            t0 = time.perf_counter()
            world.tick()
            times.append((time.perf_counter() - t0) * 1000.0)
        lidar.stop()
        times.sort()
        p95 = times[int(0.95 * len(times)) - 1]
        report(
            'tick benchmark',
            p95 < 50.0,
            f'mean {statistics.mean(times):.1f} ms, p95 {p95:.1f} ms (real time needs < 50 ms); '
            f'lidar points/tick mean {statistics.mean(counts) if counts else 0:.0f} (expect ~1440 '
            'minus misses)',
        )
    finally:
        for a in reversed(actors):
            try:
                if hasattr(a, 'stop'):
                    a.stop()
                a.destroy()
            except Exception:
                pass
        settings = world.get_settings()
        settings.synchronous_mode = orig.synchronous_mode
        settings.fixed_delta_seconds = orig.fixed_delta_seconds
        world.apply_settings(settings)

    print('\nSummary:')
    for name, ok in results:
        print(f'  {"PASS" if ok else "FAIL"}  {name}')
    return 0 if all(ok for _, ok in results) else 1


if __name__ == '__main__':
    sys.exit(main())
