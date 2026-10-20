"""
Launch component that publishes a scenario path in simulation.

File: static_path.py

Description:
  Component `static_path` (sim only): static_waypoint_publisher publishes scenario_file's
  `waypoints` as /planned_path, in place of waypoint_node.

  Check
    scenario_file has at least two [x, y] waypoints (static_waypoint_publisher dies otherwise).
"""

from launch_ros.actions import Node
import yaml

from .base import Component
from .common import FAIL, OK


class StaticPath(Component):
    name = 'static_path'

    def outputs(self, plan):
        return ['/planned_path']

    def check(self, plan, graph):
        path = plan.args['scenario_file']
        try:
            with open(path) as f:
                waypoints = (yaml.safe_load(f) or {}).get('waypoints') or []
        except OSError as e:
            return [(FAIL, f'scenario_file: {e}')]
        if len(waypoints) < 2 or not all(isinstance(w, list) and len(w) == 2 for w in waypoints):
            return [(FAIL, f'{path}: needs `waypoints:` with at least two [x, y] entries')]
        return [(OK, f'{len(waypoints)} waypoints')]

    def actions(self, plan):
        return [Node(
            package='nift_carla', executable='static_waypoint_publisher',
            name='static_waypoint_publisher',
            output='screen', parameters=[{'use_sim_time': plan.sim,
                                          'scenario_file': plan.args['scenario_file']}])]
