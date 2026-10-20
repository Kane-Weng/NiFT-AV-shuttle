#!/usr/bin/env python3
"""
Draw the Lanelet2 map into the running CARLA world for a visual alignment check.

File: draw_lanelets.py

Description:
  Draws the NiFT Lanelet2 map (centerlines green, bounds blue) into the running
  CARLA world with debug lines, so the alignment between the Lanelet2 map and
  the digital twin's roads can be eyeballed from the spectator / a camera.

Usage:
  ros2 run nift_carla carla_draw_lanelets --host <ip> [--life 0] [--no-bounds]
"""

import argparse
import sys

import carla
from nift_carla.carla_geo import CarlaGeo
from nift_carla.tools.lanelet_utils import default_map_path, load_map
from nift_carla.tools.road_height import RoadHeight
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--map', default=default_map_path())
    ap.add_argument('--datum', nargs=2, type=float, default=[MAP_ORIGIN_LAT, MAP_ORIGIN_LON],
                    metavar=('LAT', 'LON'))
    ap.add_argument('--life', type=float, default=0.0, help='seconds; 0 = persistent')
    ap.add_argument('--z', type=float, default=0.3, help='height above ground')
    ap.add_argument('--no-bounds', action='store_true')
    args = ap.parse_args(argv)

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    cmap = world.get_map()
    geo = CarlaGeo.from_opendrive(cmap.to_opendrive(), args.datum[0], args.datum[1])
    ll_map = load_map(args.map, args.datum[0], args.datum[1])
    height = RoadHeight(cmap)

    def ground(cx, cy):
        hit = world.ground_projection(carla.Location(x=cx, y=cy, z=1000.0), 2000.0)
        if hit is not None:
            return hit.location.z
        return height(cx, cy)

    def draw(pts, color, thickness):
        prev = None
        for p in pts:
            cx, cy, _ = geo.map_to_carla(p.x, p.y)
            loc = carla.Location(x=cx, y=cy, z=ground(cx, cy) + args.z)
            if prev is not None:
                world.debug.draw_line(
                    prev, loc, thickness=thickness, color=color, life_time=args.life
                )
            prev = loc

    n = 0
    for ll in ll_map.laneletLayer:
        draw(ll.centerline, carla.Color(0, 255, 0), 0.1)
        if not args.no_bounds:
            draw(ll.leftBound, carla.Color(0, 80, 255), 0.05)
            draw(ll.rightBound, carla.Color(0, 80, 255), 0.05)
        n += 1
    print(f'drew {n} lanelets (green centerlines{"" if args.no_bounds else ", blue bounds"})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
