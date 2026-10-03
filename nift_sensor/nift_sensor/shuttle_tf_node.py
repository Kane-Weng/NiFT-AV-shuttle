#!/usr/bin/env python3
"""
TF broadcaster that turns GNSS fixes into the map to base_link transform.

File: shuttle_tf_node.py
Author: Kane Weng
Date: Feb 21, 2026

Description:
  The primary localization bridge for the NiFT shuttle. Converts raw GNSS data
  (lat/lon + compass heading) into the local Cartesian map frame and broadcasts
  the appropriate ROS 2 transform.

  Publishes `map → base_link` directly. There is no competing TF chain (no
  wheel-odometry TF publisher, on the shuttle or in CARLA), so a direct
  transform is safe.

Usage:
  ros2 run nift_sensor shuttle_tf_node
"""

import math

import diagnostic_msgs.msg
import diagnostic_updater
from geometry_msgs.msg import TransformStamped
import lanelet2
from lanelet2.projection import UtmProjector
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


class ShuttleDynamicBroadcaster(Node):

    def __init__(self):
        super().__init__('shuttle_dynamic_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.current_yaw_enu = 0.0

        self.declare_parameter('gnss.offset_x', 1.041)
        self.declare_parameter('gnss.offset_y', 0.0)
        self.declare_parameter('map_origin_lat', 42.3005)
        self.declare_parameter('map_origin_lon', -83.6987)

        self.gnss_offset_x = self.get_parameter('gnss.offset_x').value
        self.gnss_offset_y = self.get_parameter('gnss.offset_y').value
        map_origin_lat = self.get_parameter('map_origin_lat').value
        map_origin_lon = self.get_parameter('map_origin_lon').value

        # UTM projector
        origin = lanelet2.io.Origin(map_origin_lat, map_origin_lon)
        self.projector = UtmProjector(origin)

        ref = self.projector.forward(
            lanelet2.core.GPSPoint(map_origin_lat, map_origin_lon, 0.0)
        )
        self.map_origin_x = ref.x
        self.map_origin_y = ref.y

        self.get_logger().info(
            f'TF broadcaster initialised — publishes map → base_link, '
            f'origin=({map_origin_lat}, {map_origin_lon}), '
            f'gnss_offset_x={self.gnss_offset_x:.3f}m'
        )

        self.sub_fix = self.create_subscription(
            NavSatFix, '/gnss/fix', self.gps_fix_callback, rclpy.qos.qos_profile_sensor_data)
        self.sub_heading = self.create_subscription(
            Float64, '/gnss/heading_deg', self.heading_callback, rclpy.qos.qos_profile_sensor_data)

        self.last_fix_time = None
        self.diag_updater = diagnostic_updater.Updater(self)
        self.diag_updater.setHardwareID('tf_broadcaster')
        self.diag_updater.add('TF Bridge', self._diagnostics_callback)

    def _diagnostics_callback(self, stat):
        if self.last_fix_time is None:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.WARN, 'Waiting for initial /gnss/fix...')
        else:
            time_diff = (self.get_clock().now() - self.last_fix_time).nanoseconds / 1e9
            if time_diff > 2.0:
                stat.summary(
                    diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                    f'GNSS STALE (No fix for {time_diff:.1f}s)')
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Publishing map → base_link')
        return stat

    def heading_callback(self, msg: Float64):
        # Convert compass heading (0=North, CW) to ROS ENU yaw (0=East, CCW)
        self.current_yaw_enu = math.radians(90.0 - msg.data)

    def gps_fix_callback(self, msg: NavSatFix):
        self.last_fix_time = self.get_clock().now()

        # 1. lat/lon → UTM → local map frame (no display offset; map origin = map_origin_lat/lon)
        utm = self.projector.forward(
            lanelet2.core.GPSPoint(msg.latitude, msg.longitude, msg.altitude)
        )
        sensor_x = utm.x - self.map_origin_x
        sensor_y = utm.y - self.map_origin_y

        # 2. Subtract the GNSS antenna offset (rotated into map frame) to get base_link
        cos_y = math.cos(self.current_yaw_enu)
        sin_y = math.sin(self.current_yaw_enu)
        base_x = sensor_x - (self.gnss_offset_x * cos_y - self.gnss_offset_y * sin_y)
        base_y = sensor_y - (self.gnss_offset_x * sin_y + self.gnss_offset_y * cos_y)

        pub_x, pub_y, pub_yaw = base_x, base_y, self.current_yaw_enu

        # 3. Publish the transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(pub_x)
        t.transform.translation.y = float(pub_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(pub_yaw / 2.0)
        t.transform.rotation.w = math.cos(pub_yaw / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = None
    try:
        node = ShuttleDynamicBroadcaster()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C race: rclpy's SIGINT handler can shut the context down mid-construction or
        # mid-spin (RCLError). With the context still alive it is a real error.
        if rclpy.ok():
            raise
    rclpy.try_shutdown()    # Ctrl-C: rclpy's SIGINT handler has already shut the context down


if __name__ == '__main__':
    main()
