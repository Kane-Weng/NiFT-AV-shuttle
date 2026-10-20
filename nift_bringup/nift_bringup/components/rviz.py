"""
Launch component for RViz.

File: rviz.py

Description:
  Component `rviz`: RViz with shuttle_description's navigation.rviz.
    split      rviz_layout:=split adds a second, render-only window (navigation_topdown.rviz: the
               same displays, top-down view following the shuttle, no panels, so no second teleop
               publisher); the window manager places it.
    satellite  if maps/satellite/mcity_sat.urdf exists (maps/utils/make_sat_overlay.py;
               git-ignored, the imagery may not be redistributed), a robot_state_publisher in
               namespace /satellite puts it on /satellite/robot_description for navigation.rviz's
               "Satellite" display. Its only link is `map`, so it adds no TF.

  Check
    a display to open the window on (DISPLAY or WAYLAND_DISPLAY); whether the satellite layer is
    on.
"""

import os

from launch_ros.actions import Node

from .base import Component
from .common import FAIL, OK, repo_root, share, sim_time


def satellite_urdf():
    """<repo>/maps/satellite/mcity_sat.urdf if make_sat_overlay.py has written it, else None."""
    path = repo_root() / 'maps' / 'satellite' / 'mcity_sat.urdf'
    return path if path.is_file() else None


class Rviz(Component):
    name = 'rviz'

    def check(self, plan, graph):
        display = os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')
        if not display:
            return [(FAIL, 'no display (DISPLAY unset: ssh without -X?): rviz:=false')]
        urdf = satellite_urdf()
        return [(OK, f'display {display}'),
                (OK, f'satellite layer: {urdf.parent}' if urdf else
                     'satellite layer off: '
                     'python3 maps/utils/make_sat_overlay.py --fetch makes it')]

    def actions(self, plan):
        actions = [Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            parameters=[sim_time(plan)],
            arguments=['-d', share('shuttle_description', 'rviz', 'navigation.rviz')])]
        if plan.args['rviz_layout'] == 'split':
            actions.append(Node(
                package='rviz2', executable='rviz2', name='rviz2_topdown', output='screen',
                parameters=[sim_time(plan)],
                arguments=['-d', share('shuttle_description', 'rviz', 'navigation_topdown.rviz')]))
        urdf = satellite_urdf()
        if urdf:
            actions.append(Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                name='satellite_publisher',
                namespace='satellite', output='log',
                parameters=[{'robot_description': urdf.read_text()}, sim_time(plan)]))
        return actions
