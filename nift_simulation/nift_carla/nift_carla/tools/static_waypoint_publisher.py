#!/usr/bin/env python3
"""
Publish the waypoints of a scenario YAML as /planned_path.

File: static_waypoint_publisher.py
Author: Kane Weng

Description:
  Reads a list of [x, y] waypoints from a YAML scenario file and publishes
  them as a nav_msgs/Path on /planned_path with latched QoS.

  This is a simulation-only replacement for waypoint_node: it bypasses the
  full Lanelet2 routing pipeline so you can feed any custom path directly to
  trajectory_controller for filter testing.

  The path is re-published every 5 seconds so any node that (re)starts after
  this node will still receive it via the TRANSIENT_LOCAL QoS.

ROS parameters:
  scenario_file (str): Absolute path to the YAML scenario file.
                       Must contain a 'waypoints' key: list of [x, y] pairs.

Usage:
  Launched by nift_bringup system.launch.py as the `static_path` component
  (profile sim_filter_bench).
  ros2 run nift_carla static_waypoint_publisher --ros-args -p scenario_file:=<yaml>
"""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import yaml


class StaticWaypointPublisher(Node):

    def __init__(self):
        super().__init__('static_waypoint_publisher')
        self.declare_parameter('scenario_file', '')

        scenario_file = self.get_parameter('scenario_file').value
        if not scenario_file:
            self.get_logger().fatal('scenario_file parameter is empty — cannot publish path')
            raise RuntimeError('scenario_file parameter is empty')

        with open(scenario_file, 'r') as f:
            config = yaml.safe_load(f)

        waypoints = config.get('waypoints', [])
        if len(waypoints) < 2:
            self.get_logger().fatal(
                f'Need at least 2 waypoints in {scenario_file} (got {len(waypoints)})'
            )
            raise RuntimeError('Not enough waypoints in scenario file')

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(Path, '/planned_path', latched_qos)
        self._waypoints = waypoints
        self._publish()
        # trajectory_controller uses TRANSIENT_LOCAL and gets the cached message
        # on join — it doesn't need this timer.  RViz2's Path display uses VOLATILE
        # durability and only receives messages published after it subscribes, so a
        # slow periodic re-publish is required for the path to appear in RViz.
        # 5 s is long enough to avoid flooding stale nodes from a previous launch.
        self.create_timer(5.0, self._publish)
        self.get_logger().info(
            f'Static path loaded: {len(waypoints)} waypoints from {scenario_file}'
        )

    def _publish(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in self._waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StaticWaypointPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C race: rclpy's SIGINT handler can shut the context down mid-construction or
        # mid-spin (RCLError). With the context still alive it is a real error.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()    # Ctrl-C: rclpy's SIGINT handler has already shut the context down


if __name__ == '__main__':
    main()
