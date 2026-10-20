"""
Always-on robot_state_publisher with the shuttle URDF.

File: description.py

Description:
  Always on, not a profile component: robot_state_publisher with the shuttle URDF
  (base_link -> velodyne / gps_frame_*), which lidar_node and the TF tree need in both modes.
  Nothing to check.
"""

from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from .base import Component
from .common import share, sim_time


class Description(Component):
    name = 'description'

    def actions(self, plan):
        urdf = share('shuttle_description', 'description', 'vehicle.urdf.xacro')
        xacro = ParameterValue(Command([FindExecutable(name='xacro'), ' ', urdf]), value_type=str)
        return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                     output='screen',
                     parameters=[{'robot_description': xacro}, sim_time(plan)])]
