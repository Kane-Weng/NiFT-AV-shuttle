"""
Launch components for the two providers of map -> base_link.

File: localization.py

Description:
  Components `localization` and `filter`: the two providers of map -> base_link from the GNSS
  fixes. Which of them are on is the mode (profiles.localization_mode(), the plan's one source of
  truth):
    direct  `localization` alone (default): shuttle_tf_node, the latest accepted front / rear pair,
            as it arrives
    ekf     `filter` alone: localization_node in mode ekf in its place: the filter publishes the
            TF, predicted to now, and carries it across short GNSS gaps; trajectory_controller gets
            localization_source filter (control.py)
    shadow  both: shuttle_tf_node keeps the TF, localization_node (mode shadow) only publishes
            /localization/odom and /localization/status, so nothing downstream changes (for
            recording the filter beside today's TF)

  shuttle_tf_node: gnss.offset_x — the front antenna's lever arm, ShuttleConfig.gnss_front_x
  (owner: nift_common vehicle.yaml), one value for sim and the shuttle; passed explicitly, so the
  launch sets it whatever shuttle_tf.yaml says. The real value is still to be measured.
  localization_node: nift_sensor config/localization.yaml, then (node-scoped, so they beat the
  yaml) the mode, both lever arms from ShuttleConfig and the map origin, as for shuttle_tf_node. In
  sim also:
    gnss_delay_s     0 on the ideal feed, SIM_REAL_FEED_DELAY_S on gnss_feed:=real ('' =
                     carla_params.yaml's feed); on the shuttle the yaml's
    speed_point_x_m  where the sim's GNSS speed is taken: the rear axle in the kinematic sim, the
                     actor's velocity in CARLA (its origin = base_link within 0.07 m); on the
                     shuttle the default, the front receiver

  Checks: nothing before launch (the input is gnss, checked there). shuttle_tf_node publishes on
  /tf, which other nodes share, so no outputs; the filter's output is /localization/status, which
  nothing else may publish (a forgotten localization_node would feed trajectory_controller a second
  filter's status).
"""

from launch_ros.actions import Node
from nift_common.vehicle import ShuttleConfig
import yaml

from .base import Component
from .common import map_origin, node_params, share, sim_time

# mean age of a position at its first delivery on the sim's real-feed model
SIM_REAL_FEED_DELAY_S = 0.061


def sim_feed_real(plan) -> bool:
    """
    Tell whether the sim backend runs the real-feed model.

    True for gnss_feed:=real, or for '' with carla_params.yaml's feed enabled.
    """
    if plan.args['gnss_feed']:
        return plan.args['gnss_feed'] == 'real'
    with open(share('nift_carla', 'config', 'carla_params.yaml')) as f:
        params = yaml.safe_load(f)['carla_bridge']['ros__parameters']
    return bool(params.get('gps', {}).get('feed', {}).get('enabled', False))


def filter_params(plan, mode) -> dict:
    """localization_node's parameters set by the launch (on top of config/localization.yaml)."""
    shuttle = ShuttleConfig()
    params = {'mode': mode, 'antenna.front_x_m': shuttle.gnss_front_x,
              'antenna.rear_x_m': shuttle.gnss_rear_x,
              **map_origin(plan)}
    if plan.sim:
        params['gnss_delay_s'] = SIM_REAL_FEED_DELAY_S if sim_feed_real(plan) else 0.0
        params['speed_point_x_m'] = -shuttle.base_link_x if plan.kinematic else 0.0
    return params


class Localization(Component):
    name = 'localization'

    def actions(self, plan):
        return [Node(
            package='nift_sensor', executable='shuttle_tf_node',
            name='shuttle_dynamic_broadcaster',
            output='screen',
            parameters=[share('nift_sensor', 'config', 'shuttle_tf.yaml'), sim_time(plan),
                        map_origin(plan),
                        {'gnss.offset_x': ShuttleConfig().gnss_front_x}])]


class Filter(Component):
    name = 'filter'

    def outputs(self, plan):
        return ['/localization/status']

    def actions(self, plan):
        # ekf (filter alone) | shadow (with localization): the node's mode
        mode = plan.localization_mode
        return [Node(
            package='nift_sensor', executable='localization_node', name='localization_node',
            output='screen',
            parameters=[share('nift_sensor', 'config', 'localization.yaml'), sim_time(plan),
                        node_params('localization_node', filter_params(plan, mode))])]
