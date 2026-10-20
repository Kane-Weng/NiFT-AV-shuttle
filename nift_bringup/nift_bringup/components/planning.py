"""
Launch component for waypoint_node and its Lanelet2 map.

File: planning.py

Description:
  Component `planning`: waypoint_node, Lanelet2 routes on /planned_path (after beacon auth_ok).

  Map
    map_file:= (profiles.py): a file in nift_waypoint's maps/ or a path; '' = lanelet2_odr2024.osm
    on the shuttle and in sim (profiles.DEFAULT_MAPS).

  Check
    the Lanelet2 map loads the way waypoint_node loads it (UtmProjector at the map origin) and has
    lanelets; load warnings are a WARN.
"""

import os

from launch_ros.actions import Node

from ..profiles import map_path
from .base import Component
from .common import FAIL, map_origin, OK, share, sim_time, WARN


class Planning(Component):
    name = 'planning'

    def outputs(self, plan):
        return ['/planned_path']

    def check(self, plan, graph):
        path = map_path(plan.args['map_file'])
        if not os.path.isfile(path):
            return [(FAIL, f'map {path} does not exist (map_file:={plan.args["map_file"]})')]
        import lanelet2
        from lanelet2.projection import UtmProjector
        o = map_origin(plan)
        projector = UtmProjector(lanelet2.io.Origin(o['map_origin_lat'], o['map_origin_lon']))
        lanelet_map, errors = lanelet2.io.loadRobust(path, projector)
        n = len(lanelet_map.laneletLayer)
        name = os.path.basename(path)
        if n == 0:
            return [(FAIL, f'{name} has no lanelets')]
        out = [(OK, f'{name}: {n} lanelets')]
        if errors:
            out.append((WARN, f'{name}: {len(errors)} load warning(s), first: {errors[0]}'))
        return out

    def actions(self, plan):
        return [Node(
            package='nift_waypoint', executable='waypoint_node', name='waypoint_node',
            output='screen',
            parameters=[share('nift_waypoint', 'config', 'waypoint_params.yaml'), sim_time(plan),
                        map_origin(plan), {'map_file': map_path(plan.args['map_file'])}])]
