"""
Launch the kinematic sim backend, the no-GPU stand-in for the CARLA bridge.

File: kinematic_sim.launch.py
Author: Kane Weng

Description:
  Sim-side half of the kinematic backend: kinematic_sim_node, the no-GPU stand-in
  for the CARLA bridge (no CARLA server, no LiDAR, no scenario obstacles).
  Included by system.launch.py when sim_backend:=kinematic. Nothing here uses
  use_sim_time — the node IS the clock.

  Parameters come from the CARLA bridge's file: its carla_bridge section is handed
  to the node as is, so the plant, Octane, GNSS and spawn settings exist once
  (carla_params.yaml) and both backends read the same values. The node ignores
  the CARLA-only keys it does not declare.

Launch arguments:
  carla_params     parameter file (default: nift_carla/config/carla_params.yaml)
  map_origin_lat   map datum latitude (default: nift_common.site)
  map_origin_lon   map datum longitude (default: nift_common.site)
  spawn_x/y/yaw    override spawn pose (map frame, rad); '' keeps the yaml value
  time_factor      '' keeps the yaml value (1.0); 0 = as fast as possible
  lockstep         true | false; '' keeps the yaml value
  gnss_feed        real (the real Octane feed model, sim_gnss.py FeedConfig) | ideal;
                   '' keeps the yaml value
  gnss_dead_zones  set names from config/gnss_dead_zones.yaml, comma-separated
                   (no GNSS inside); '' = the yaml's
  gnss_seed        GNSS random seed (0 = new each run); '' keeps the yaml value
  gnss_faults      a fault scenario of config/gnss_faults.yaml (or a .yaml file); '' = none

Usage:
  ros2 launch nift_carla kinematic_sim.launch.py [spawn_x:=… spawn_y:=… spawn_yaw:=…]
      [time_factor:=0]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nift_carla.sim_gnss import launch_overrides
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
import yaml


def _build(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731

    with open(cfg('carla_params')) as f:
        shared = yaml.safe_load(f)['carla_bridge']['ros__parameters']

    overrides = {
        'map_origin_lat': float(cfg('map_origin_lat')),
        'map_origin_lon': float(cfg('map_origin_lon')),
    }
    if cfg('spawn_x') != '' and cfg('spawn_y') != '':
        overrides['spawn.mode'] = 'map'
        overrides['spawn.x'] = float(cfg('spawn_x'))
        overrides['spawn.y'] = float(cfg('spawn_y'))
        overrides['spawn.yaw'] = float(cfg('spawn_yaw') or 0.0)
    if cfg('time_factor'):
        overrides['time_factor'] = float(cfg('time_factor'))
    if cfg('lockstep'):
        overrides['lockstep'] = cfg('lockstep').lower() in ('true', '1')
    share = os.path.join(get_package_share_directory('nift_carla'), 'config')
    sets, catalogue = {}, {}
    if cfg('gnss_dead_zones') or cfg('gnss_faults'):       # fault boxes may name a dead-zone set
        with open(os.path.join(share, 'gnss_dead_zones.yaml')) as f:
            sets = yaml.safe_load(f)['sets']
    if cfg('gnss_faults'):
        with open(os.path.join(share, 'gnss_faults.yaml')) as f:
            catalogue = yaml.safe_load(f)
    overrides.update(
        launch_overrides(
            cfg('gnss_feed'),
            cfg('gnss_dead_zones'),
            cfg('gnss_seed'),
            sets,
            cfg('gnss_faults'),
            catalogue,
        )
    )

    return [Node(
        package='nift_carla',
        executable='kinematic_sim_node',
        name='kinematic_sim',
        output='screen',
        emulate_tty=True,
        parameters=[shared, overrides],
    )]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('nift_carla'), 'config', 'carla_params.yaml']
    )
    return LaunchDescription([
        DeclareLaunchArgument('carla_params', default_value=default_params),
        DeclareLaunchArgument('map_origin_lat', default_value=str(MAP_ORIGIN_LAT)),
        DeclareLaunchArgument('map_origin_lon', default_value=str(MAP_ORIGIN_LON)),
        DeclareLaunchArgument('spawn_x', default_value=''),
        DeclareLaunchArgument('spawn_y', default_value=''),
        DeclareLaunchArgument('spawn_yaw', default_value=''),
        DeclareLaunchArgument('time_factor', default_value=''),
        DeclareLaunchArgument('lockstep', default_value=''),
        DeclareLaunchArgument('gnss_feed', default_value=''),
        DeclareLaunchArgument('gnss_dead_zones', default_value=''),
        DeclareLaunchArgument('gnss_seed', default_value=''),
        DeclareLaunchArgument('gnss_faults', default_value=''),
        OpaqueFunction(function=_build),
    ])
