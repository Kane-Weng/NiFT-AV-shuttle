#!/usr/bin/env python3
"""
Launch the TF tree for the NiFT shuttle.

File: shuttle_tf.launch.py
Author: Kane Weng
Date: Feb 21, 2026

Description:
  Launch file to initialize the TF (Transform) tree for the NiFT shuttle.
  Spawns the dynamic localization broadcaster (map -> base_link) and sets up
  static transforms for sensor frames relative to the vehicle's true center.

Usage:
  ros2 launch nift_sensor shuttle_tf.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sensor = get_package_share_directory('nift_sensor')

    return LaunchDescription([
        # 1. Dynamic Broadcaster (Map -> base_link)
        # Calculates base_link position dynamically based on raw GNSS + heading.
        Node(
            package='nift_sensor',
            executable='shuttle_tf_node',
            name='shuttle_tf_broadcaster',
            parameters=[os.path.join(pkg_sensor, 'config', 'shuttle_tf.yaml')]
        ),

        # 2. Static Broadcaster (base_link -> lidar_link)
        # TODO: NEEDS REVISION. The translation/rotation arguments below are currently
        # PLACEHOLDERS.
        # Physical measurements of the LiDAR mount relative to the vehicle center (base_link)
        # must be taken and updated here before running perception nodes.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_lidar',
            arguments=[
                '--x', '1.53', '--y', '0.0', '--z', '0.96',
                '--yaw', '0.0', '--pitch', '0.2618', '--roll', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'
            ]
        )
    ])
