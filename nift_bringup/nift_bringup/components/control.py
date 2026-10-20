"""
Launch component for trajectory_controller (longitudinal and lateral).

File: control.py

Description:
  Components `longitudinal` and `lateral`: one trajectory_controller (after beacon auth_ok).
    lateral_enabled      = `lateral` on (off: /start_navigation refused, MANUAL only)
    filter_type          = the plan's (none when perception is off), else planning.yaml's
    localization_source  = filter in mode ekf (`filter` alone: localization_node's TF, odom and
                           status), else gnss (direct, shadow, neither on: today's /gnss/vel
                           watchdog and speed)

  Check
    only the generic outputs check (preflight.py): nothing else may already publish /to_can_bus — a
    forgotten longitudinal_test, tuner or second launch would fight this controller for the
    vehicle.
"""

from launch_ros.actions import Node

from .base import Component
from .common import node_params, share, sim_time


def localization_source(plan) -> str:
    """
    Return trajectory_controller's localization_source.

    filter only when localization_node publishes the TF, else gnss.
    """
    return 'filter' if plan.localization_mode == 'ekf' else 'gnss'


class Control(Component):
    name = 'longitudinal'

    def outputs(self, plan):
        return ['/to_can_bus']

    def actions(self, plan):
        params = [sim_time(plan),
                  share('nift_control', 'config', 'planning.yaml'),
                  share('nift_control', 'config', 'adrc_params.yaml'),
                  {'lateral_enabled': 'lateral' in plan.on}]
        # node-scoped file: a dict (wildcard) loses to planning.yaml's value
        overrides = {'localization_source': localization_source(plan)}
        if plan.args['filter_type']:
            overrides['filter_type'] = plan.args['filter_type']
        params.append(node_params('trajectory_controller', overrides))
        return [Node(package='nift_control', executable='trajectory_controller',
                     name='trajectory_controller',
                     output='screen', parameters=params)]
