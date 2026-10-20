"""
Launch the CARLA bridge and, with a scenario file, the scenario spawner.

File: carla_bridge.launch.py
Author: Kane Weng

Description:
  Sim-side half of the CARLA backend: the bridge node (ego vehicle, sensors,
  /clock, teleport, virtual ECU on /to_can_bus) and, when a scenario file is
  given, the scenario spawner.  Included by system.launch.py (mode sim).  Nothing
  here uses use_sim_time — the bridge IS the clock.

Launch arguments:
  carla_host       CARLA server host (default: localhost)
  carla_port       CARLA RPC port (default: 2000)
  carla_params     bridge parameter file (default: nift_carla/config/carla_params.yaml)
  scenario_file    YAML with `obstacles:`; '' = no spawner (default: '')
  map_origin_lat   map datum latitude (default: nift_common.site)
  map_origin_lon   map datum longitude (default: nift_common.site)
  spawn_x/y/yaw    override spawn pose (map frame, rad); '' keeps the yaml value
  ego_blueprint    override the ego blueprint; '' keeps the yaml value
  time_factor      '' keeps the yaml value
  lockstep         true | false; '' keeps the yaml value
  gnss_feed        real (the real Octane feed model, sim_gnss.py FeedConfig) | ideal;
                   '' keeps the yaml value
  gnss_dead_zones  set names from config/gnss_dead_zones.yaml, comma-separated
                   (no GNSS inside); '' = the yaml's
  gnss_seed        GNSS random seed (0 = new each run); '' keeps the yaml value
  gnss_faults      a fault scenario of config/gnss_faults.yaml (or a .yaml file); '' = none

Usage:
  ros2 launch nift_carla carla_bridge.launch.py carla_host:=<ip>
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

    overrides = {
        'carla_host': cfg('carla_host'),
        'carla_port': int(cfg('carla_port')),
        'map_origin_lat': float(cfg('map_origin_lat')),
        'map_origin_lon': float(cfg('map_origin_lon')),
    }
    if cfg('spawn_x') != '' and cfg('spawn_y') != '':
        overrides['spawn.mode'] = 'map'
        overrides['spawn.x'] = float(cfg('spawn_x'))
        overrides['spawn.y'] = float(cfg('spawn_y'))
        overrides['spawn.yaw'] = float(cfg('spawn_yaw') or 0.0)
    if cfg('ego_blueprint'):
        overrides['ego_blueprint'] = cfg('ego_blueprint')
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

    actions = [
        Node(
            package='nift_carla',
            executable='carla_bridge_node',
            name='carla_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[cfg('carla_params'), overrides],
        )
    ]
    if cfg('scenario_file'):
        actions.append(Node(
            package='nift_carla',
            executable='carla_scenario_node',
            name='carla_scenario',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'scenario_file': cfg('scenario_file'),
                'carla_host': cfg('carla_host'),
                'carla_port': int(cfg('carla_port')),
                'map_origin_lat': float(cfg('map_origin_lat')),
                'map_origin_lon': float(cfg('map_origin_lon')),
            }],
        ))
    return actions


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('nift_carla'), 'config', 'carla_params.yaml']
    )
    return LaunchDescription([
        DeclareLaunchArgument('carla_host', default_value='localhost'),
        DeclareLaunchArgument('carla_port', default_value='2000'),
        DeclareLaunchArgument('carla_params', default_value=default_params),
        DeclareLaunchArgument('scenario_file', default_value=''),
        DeclareLaunchArgument('map_origin_lat', default_value=str(MAP_ORIGIN_LAT)),
        DeclareLaunchArgument('map_origin_lon', default_value=str(MAP_ORIGIN_LON)),
        DeclareLaunchArgument('spawn_x', default_value=''),
        DeclareLaunchArgument('spawn_y', default_value=''),
        DeclareLaunchArgument('spawn_yaw', default_value=''),
        DeclareLaunchArgument('ego_blueprint', default_value=''),
        DeclareLaunchArgument('time_factor', default_value=''),
        DeclareLaunchArgument('lockstep', default_value=''),
        DeclareLaunchArgument('gnss_feed', default_value=''),
        DeclareLaunchArgument('gnss_dead_zones', default_value=''),
        DeclareLaunchArgument('gnss_seed', default_value=''),
        DeclareLaunchArgument('gnss_faults', default_value=''),
        OpaqueFunction(function=_build),
    ])
