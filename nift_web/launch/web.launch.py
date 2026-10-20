"""
Start rosbridge_websocket so a browser can talk to the ROS 2 graph.

File: web.launch.py
Author: Kane Weng
Date: April 21, 2026

Description:
  Rosbridge is a translator between non-ROS environments (Web browser, for example)
  to talk to the local ROS 2 network. It creates a standard Websocket connection,
  passing messages back and forth using JSON.

  This script starts the server by launching rosbridge_websocket node.
    - port: 9090 (standard port WebSocket server listen on; web interface need to connect to this)
    - address: 0.0.0.0 (tell the server to listen on all available network interfaces)

Usage:
  ros2 launch nift_web web.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description with the rosbridge_websocket node."""
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090',
                              description='rosbridge WebSocket port'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='rosbridge bind address'),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
                'retry_startup_delay': 5.0,
                # Default fragment_size is 1 MB; roslib.js has no fragment
                # reassembly, so large messages are silently dropped.
                # 10 MB covers the largest expected topic (lanelet MarkerArray).
                'max_message_size': 10000000,
            }],
            output='screen',
        ),
    ])
