"""
Launch component for lidar_node.

File: perception.py

Description:
  Component `perception`: lidar_node -> /obstacles. Parameters: perception_yaml, else
  nift_sensor/config/perception.yaml (real) or nift_carla/config/perception_carla.yaml (sim).

  Check
    the parameter file exists; real: something already publishes lidar_node's input topic (/scan).
    There is no LiDAR driver in this workspace, so on the shuttle that is a FAIL until one runs —
    otherwise trajectory_controller's /obstacles watchdog would refuse /start_navigation anyway.
    sim: SKIP, the CARLA bridge publishes /scan.
"""

import os

from launch_ros.actions import Node
import yaml

from .base import Component
from .common import FAIL, OK, share, SKIP


def _params_file(plan):
    return plan.args['perception_yaml'] or (
        share('nift_carla', 'config', 'perception_carla.yaml') if plan.sim
        else share('nift_sensor', 'config', 'perception.yaml'))


class Perception(Component):
    name = 'perception'

    def outputs(self, plan):
        return ['/obstacles']

    def check(self, plan, graph):
        path = _params_file(plan)
        if not os.path.isfile(path):
            return [(FAIL, f'perception_yaml {path} does not exist')]
        if plan.sim:
            return [(SKIP, '/scan comes from the CARLA bridge')]
        with open(path) as f:
            params = (((yaml.safe_load(f) or {}).get('lidar_node') or {})
                      .get('ros__parameters') or {})
        topic = params.get('input_topic', '/scan')
        pubs = graph.publishers(topic)
        if not pubs:
            return [(FAIL,
                     f'nothing publishes {topic}: no LiDAR driver running '
                     '(none in this workspace). '
                     'Start one, or perception:=false (safety filter off)')]
        return [(OK, f'{topic} published by {", ".join(pubs)}')]

    def actions(self, plan):
        return [Node(
            package='nift_sensor', executable='lidar_node', name='lidar_node', output='screen',
            parameters=[{'use_sim_time': plan.sim, 'lidar_frame': 'velodyne'},
                        _params_file(plan)])]
