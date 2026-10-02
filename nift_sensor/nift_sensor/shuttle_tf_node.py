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

  Operates in two modes controlled by the `use_sim_mode` parameter:

  - Real-World Mode (use_sim_mode == False):
    Publishes `map → base_link` directly. There is no competing TF chain in the
    real-world stack (no wheel-odometry TF publisher), so a direct transform is safe.

  - Simulation Mode (use_sim_mode == True):
    ros2_control publishes `odom → base_link` from Gazebo physics. Publishing
    `map → base_link` directly would create a TF cycle (two paths from `map` to
    `base_link`), causing TF2 to silently reject the GPS transform and leave the
    system driven by stale odometry.

    Instead, this node computes and publishes `map → odom` — the standard GPS
    localization correction transform — using:

      T(map→odom) = T(GPS-derived map→base_link) × inv(T(odom→base_link))

    This eliminates the cycle: ros2_control owns `odom → base_link`, and this node
    owns `map → odom`. Together they give a consistent `map → base_link` path.

Usage:
  ros2 run nift_sensor shuttle_tf_node
  ros2 run nift_sensor shuttle_tf_node --ros-args -p use_sim_mode:=true
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
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ShuttleDynamicBroadcaster(Node):

    def __init__(self):
        super().__init__('shuttle_dynamic_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.current_yaw_enu = 0.0

        self.declare_parameter('use_sim_mode', False)
        self.declare_parameter('gnss.offset_x', 1.041)
        self.declare_parameter('gnss.offset_y', 0.0)
        self.declare_parameter('map_origin_lat', 42.3005)
        self.declare_parameter('map_origin_lon', -83.6987)

        self.use_sim_mode = self.get_parameter('use_sim_mode').value
        self.gnss_offset_x = self.get_parameter('gnss.offset_x').value
        self.gnss_offset_y = self.get_parameter('gnss.offset_y').value
        map_origin_lat = self.get_parameter('map_origin_lat').value
        map_origin_lon = self.get_parameter('map_origin_lon').value

        # In simulation, look up odom→base_link to avoid publishing map→base_link
        # directly (which would conflict with the ros2_control TF chain)
        if self.use_sim_mode:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # UTM projector
        origin = lanelet2.io.Origin(map_origin_lat, map_origin_lon)
        self.projector = UtmProjector(origin)

        ref = self.projector.forward(
            lanelet2.core.GPSPoint(map_origin_lat, map_origin_lon, 0.0)
        )
        self.map_origin_x = ref.x
        self.map_origin_y = ref.y

        mode_str = ('SIMULATION (publishes map → odom)' if self.use_sim_mode
                    else 'REAL-WORLD (publishes map → base_link)')
        self.get_logger().info(
            f'TF broadcaster initialised — mode={mode_str}, '
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
                mode = 'map → odom' if self.use_sim_mode else 'map → base_link'
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, f'Publishing {mode}')
        return stat

    def heading_callback(self, msg: Float64):
        # Convert compass heading (0=North, CW) to ROS ENU yaw (0=East, CCW)
        self.current_yaw_enu = math.radians(90.0 - msg.data)

    def gps_fix_callback(self, msg: NavSatFix):
        self.last_fix_time = self.get_clock().now()

        # 1. lat/lon → UTM → local map frame (no display offset; map origin = Gazebo world origin)
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

        if self.use_sim_mode:
            # 3. (Sim only) Compute map→odom = T(map→base_link) × inv(T(odom→base_link))
            #    This prevents the TF cycle caused by ros2_control also publishing odom→base_link.
            try:
                odom_tf = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            except TransformException:
                self.get_logger().debug('Waiting for odom → base_link TF; skipping GPS frame')
                return

            ox = odom_tf.transform.translation.x
            oy = odom_tf.transform.translation.y
            oq = odom_tf.transform.rotation
            oyaw = math.atan2(
                2.0 * (oq.w * oq.z + oq.x * oq.y), 1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z))

            # inv(T_odom_base): translation = R_oyaw^T * (-t_odom), rotation = -oyaw
            cos_oy, sin_oy = math.cos(oyaw), math.sin(oyaw)
            inv_tx = -(cos_oy * ox + sin_oy * oy)
            inv_ty = sin_oy * ox - cos_oy * oy

            # T_map_odom translation = R_base_yaw * inv_t + t_base
            pub_x = cos_y * inv_tx - sin_y * inv_ty + base_x
            pub_y = sin_y * inv_tx + cos_y * inv_ty + base_y
            pub_yaw = self.current_yaw_enu - oyaw
            child_frame = 'odom'
        else:
            pub_x, pub_y, pub_yaw = base_x, base_y, self.current_yaw_enu
            child_frame = 'base_link'

        # 4. Publish the transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = child_frame
        t.transform.translation.x = float(pub_x)
        t.transform.translation.y = float(pub_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(pub_yaw / 2.0)
        t.transform.rotation.w = math.cos(pub_yaw / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ShuttleDynamicBroadcaster()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    rclpy.try_shutdown()    # Ctrl-C: rclpy's SIGINT handler has already shut the context down


if __name__ == '__main__':
    main()
