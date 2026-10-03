#!/usr/bin/env python3
"""
GNSS localization node that reads the shuttle's front and rear RTK beacons.

File: beacon_node.py
Author: Jimmy Wang, Kane Weng
Date: April 11, 2026

Description:
  GNSS hardware interface for the NiFT shuttle.

  Connects to the Mcity Octane Socket.io RTK server and receives live
  dual-GNSS beacon updates for the front and rear antennas. Normalizes
  the payload, then routes it through the publishing logic.
  Stale gate: if either antenna has not delivered a *new* position
  (`updated` changed) for `gnss_timeout_s` (node clock), nothing is
  published until both report again, so an Octane outage shows up
  downstream as silence instead of a frozen fix republished at 50 Hz.

  Simulation runs this same code: nift_carla's Octane emulator serves the
  same protocol, and its launch file points MCITY_OCTANE_* at it.

  Publishing logic (_publish_gnss):
  - /gnss/fix         — front antenna position only (lat/lon/alt)
  - /gnss/heading_deg — compass heading of the rear -> front vector
  - /gnss/vel         — TwistStamped with linear.x = speed in m/s

Usage:
  ros2 run nift_sensor beacon_node
"""

from datetime import datetime, timezone
import math
import os
from pathlib import Path
import threading

import diagnostic_msgs.msg
import diagnostic_updater
from dotenv import load_dotenv
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
import socketio
from std_msgs.msg import Float64

# ----------------------------
# Utility functions
# ----------------------------


def iso8601_to_ros_time(node: Node, s: str):
    """
    Convert ISO8601 like '2026-01-30T19:37:12.593+00:00' to ROS time.

    If parsing fails, fallback to node clock now().
    """
    if not s:
        return node.get_clock().now().to_msg()
    try:
        dt = datetime.fromisoformat(s)
        if dt.tzinfo is None:
            dt = dt.replace(tzinfo=timezone.utc)
        ts = dt.timestamp()
        sec = int(ts)
        nanosec = int((ts - sec) * 1e9)
        return rclpy.time.Time(seconds=sec, nanoseconds=nanosec).to_msg()
    except Exception:
        return node.get_clock().now().to_msg()


def normalize(payload: dict) -> dict:
    """
    Normalize two observed schemas into one dict.

    The result has the keys id, ts, lat, lon, alt, elev, speed, heading and source.
    """
    beacon_id = payload.get('id') or payload.get('idFixed') or payload.get('idTemporary')

    # Schema A: state.dynamics
    if (isinstance(payload.get('state'), dict)
            and isinstance(payload['state'].get('dynamics'), dict)):
        dyn = payload['state']['dynamics']
        return {
            'id': beacon_id,
            'ts': dyn.get('updated') or payload.get('updated'),
            'lat': dyn.get('latitude'),
            'lon': dyn.get('longitude'),
            'alt': dyn.get('altitude', 0.0),
            'elev': dyn.get('elevation'),
            'speed': dyn.get('velocity'),
            'heading': dyn.get('heading'),
            'source': 'state_dynamics',
        }

    # Schema B: flat J2735
    return {
        'id': beacon_id,
        'ts': payload.get('updated'),
        'lat': payload.get('latitude'),
        'lon': payload.get('longitude'),
        'alt': 0.0,
        'elev': payload.get('elevation'),
        'speed': payload.get('speed'),
        'heading': payload.get('heading'),
        'source': 'j2735_bsm',
    }


def compute_heading_from_beacons(lat_r, lon_r, lat_f, lon_f):
    """
    Compute the compass heading of the vector pointing from the rear to the front antenna.

    The result is (heading_rad, heading_deg), where 0 deg = North and 90 deg = East.
    """
    R = 6371000.0
    lat_r_rad = math.radians(lat_r)
    lon_r_rad = math.radians(lon_r)
    lat_f_rad = math.radians(lat_f)
    lon_f_rad = math.radians(lon_f)

    lat_avg = 0.5 * (lat_r_rad + lat_f_rad)
    dx = (lon_f_rad - lon_r_rad) * math.cos(lat_avg) * R
    dy = (lat_f_rad - lat_r_rad) * R

    heading_rad = math.atan2(dx, dy)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360.0

    return heading_rad, heading_deg


# ----------------------------
# Node
# ----------------------------

class BeaconNode(Node):

    def __init__(self):
        super().__init__('nift_beacon_node')

        # Publishers
        self.pub_fix = self.create_publisher(NavSatFix, '/gnss/fix', qos_profile_sensor_data)
        self.pub_vel = self.create_publisher(TwistStamped, '/gnss/vel', qos_profile_sensor_data)
        self.pub_heading = self.create_publisher(
            Float64, '/gnss/heading_deg', qos_profile_sensor_data)

        # Diagnostics
        self.diag_updater = diagnostic_updater.Updater(self)
        self.diag_updater.setHardwareID('gnss_beacon')
        self.diag_updater.add('GNSS Interface', self._diagnostics_callback)

        self._init_real_mode()

    # =========================================================================
    # REAL-WORLD MODE
    # =========================================================================

    def _init_real_mode(self):
        """Load .env credentials and connect to the Mcity Octane RTK server over Socket.io."""
        env_path = Path(__file__).resolve()
        for parent in [env_path.parent, *env_path.parents]:
            candidate = parent / '.env'
            if candidate.exists():
                load_dotenv(candidate)
                break
        else:
            load_dotenv()

        self.api_key = os.getenv('MCITY_OCTANE_KEY')
        self.server = os.getenv('MCITY_OCTANE_SERVER', 'https://octane.invalid')
        self.namespace = os.getenv('MCITY_OCTANE_NAMESPACE', '/octane')
        self.filter_id_front = os.getenv('MCITY_FRONT_BEACON_ID', '')
        self.filter_id_rear = os.getenv('MCITY_REAR_BEACON_ID', '')
        self.data_lock = threading.Lock()
        self.latest_by_id = {}
        self.gnss_timeout_s = float(self.declare_parameter('gnss_timeout_s', 0.5).value)
        self._stale = False

        if not self.api_key:
            raise RuntimeError('MCITY_OCTANE_KEY not set. Create .env based on .env.example')

        self.sio = socketio.Client(
            reconnection=True, reconnection_attempts=0, reconnection_delay=1)

        @self.sio.on('connect', namespace=self.namespace)
        def _on_connect():
            self.get_logger().info('socket connected, sending auth')
            self.sio.emit('auth', {'x-api-key': self.api_key}, namespace=self.namespace)

        @self.sio.on('auth_ok', namespace=self.namespace)
        def _on_auth_ok(_data):
            self.get_logger().info('auth_ok, joining channels beacon + v2x_obu_parsed')
            self.sio.emit('join', {'channel': 'beacon'}, namespace=self.namespace)
            self.sio.emit('join', {'channel': 'v2x_obu_parsed'}, namespace=self.namespace)

        @self.sio.on('beacon_update', namespace=self.namespace)
        def _on_beacon_update(data):
            self._handle_payload(data)

        @self.sio.on('v2x_BSM', namespace=self.namespace)
        def _on_v2x_bsm(data):
            self._handle_payload(data)

        @self.sio.on('disconnect', namespace=self.namespace)
        def _on_disconnect():
            self.get_logger().warn('socket disconnected')

        self.get_logger().info(f'connecting to {self.server} namespace={self.namespace}')
        self.sio.connect(self.server, namespaces=[self.namespace])

        # 50 Hz timer — only publishes when both beacons have reported at least once
        self.timer = self.create_timer(0.02, self._publish_latest)
        self.get_logger().info('Real-world GNSS mode active')

    def _handle_payload(self, payload: dict):
        n = normalize(payload)
        beacon_id = n.get('id')
        if beacon_id is None:
            return
        if beacon_id not in [self.filter_id_front, self.filter_id_rear]:
            return
        now_ns = self.get_clock().now().nanoseconds
        with self.data_lock:
            prev = self.latest_by_id.get(beacon_id)
            # An event that repeats the previous position (same `updated`) is not fresh data
            same = prev is not None and n.get('ts') is not None and n.get('ts') == prev.get('ts')
            n['rx_ns'] = prev['rx_ns'] if same else now_ns
            self.latest_by_id[beacon_id] = n

    def _publish_latest(self):
        with self.data_lock:
            front = self.latest_by_id.get(self.filter_id_front)
            rear = self.latest_by_id.get(self.filter_id_rear)

        if front is None or rear is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        age_f = (now_ns - front['rx_ns']) / 1e9
        age_r = (now_ns - rear['rx_ns']) / 1e9
        if max(age_f, age_r) > self.gnss_timeout_s:
            if not self._stale:
                self._stale = True
                self.get_logger().warn(
                    f'GNSS stale (no new position: front {age_f:.2f} s, rear {age_r:.2f} s) '
                    f'-> /gnss/* paused until both beacons report again')
            return
        if self._stale:
            self._stale = False
            self.get_logger().info('GNSS fresh again -> /gnss/* resumed')

        lat_f = front.get('lat')
        lon_f = front.get('lon')
        lat_r = rear.get('lat')
        lon_r = rear.get('lon')

        if lat_f is None or lon_f is None or lat_r is None or lon_r is None:
            return

        elev_f = front.get('elev')
        alt_f = float(elev_f) if elev_f is not None else float(front.get('alt', 0.0))

        stamp = iso8601_to_ros_time(self, front.get('ts'))

        spd = front.get('speed')
        speed_mps = float(spd) if spd is not None else None

        self._publish_gnss(
            float(lat_f), float(lon_f), float(lat_r), float(lon_r), alt_f, stamp, speed_mps)

    # =========================================================================
    # SHARED PUBLISHING LOGIC
    # =========================================================================

    def _publish_gnss(
        self,
        lat_f: float,
        lon_f: float,
        lat_r: float,
        lon_r: float,
        alt_f: float,
        stamp,
        speed_mps,
    ):
        """
        Publish the GNSS topics for the latest front and rear beacon pair.

          - /gnss/fix         uses the front antenna position only
          - /gnss/heading_deg is computed from the rear -> front antenna vector
          - /gnss/vel         publishes speed_mps in TwistStamped.twist.linear.x
        """
        # 1. Fix — front antenna only
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'gps'
        fix.latitude = lat_f
        fix.longitude = lon_f
        fix.altitude = alt_f
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_fix.publish(fix)

        # 2. Heading — rear -> front vector
        _, heading_deg = compute_heading_from_beacons(lat_r, lon_r, lat_f, lon_f)
        heading_msg = Float64()
        heading_msg.data = heading_deg
        self.pub_heading.publish(heading_msg)

        # 3. Velocity
        if speed_mps is not None:
            tw = TwistStamped()
            tw.header = fix.header
            tw.twist.linear.x = speed_mps
            self.pub_vel.publish(tw)

    # =========================================================================
    # DIAGNOSTICS
    # =========================================================================

    def _diagnostics_callback(self, stat):
        if hasattr(self, 'sio') and self.sio.connected and self._stale:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                         'Connected, but beacon data STALE')
        elif hasattr(self, 'sio') and self.sio.connected:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Connected to RTK Server')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, 'DISCONNECTED from Server')
        return stat


def main():
    rclpy.init()
    node = None
    try:
        node = BeaconNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C race: rclpy's SIGINT handler can shut the context down mid-construction or
        # mid-spin (RCLError). With the context still alive it is a real error.
        if rclpy.ok():
            raise
    finally:
        if hasattr(node, 'sio'):
            try:
                node.sio.disconnect()
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()    # Ctrl-C: rclpy's SIGINT handler has already shut the context down


if __name__ == '__main__':
    main()
