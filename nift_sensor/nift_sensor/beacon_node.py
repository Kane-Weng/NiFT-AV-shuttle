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
  the payload, checks it (gnss_hygiene.BeaconHygiene: late, stuck and
  inconsistent data is dropped; parameters hygiene.*), then routes the
  last accepted front / rear pair through the publishing logic at 50 Hz.
  Stale gate: if either antenna has not delivered a fresh position, or
  no pair has passed the checks, for `gnss_timeout_s` (node clock),
  nothing is published until both report again, so an Octane outage or
  a bad feed shows up downstream as silence instead of a frozen or wrong
  fix republished at 50 Hz. hygiene.enabled:=false goes back to the old
  rule (fresh = `updated` changed; every delivery is published).

  Simulation runs this same code: nift_carla's Octane emulator serves the
  same protocol, and its launch file points MCITY_OCTANE_* at it.

  Publishing logic (_publish_gnss):
  - /gnss/fix         — front antenna position only (lat/lon/alt)
  - /gnss/heading_deg — compass heading of the rear -> front vector
  - /gnss/vel         — TwistStamped with linear.x = speed in m/s (Octane reports knots;
                        octane_client.normalize() converts, measured 2026-10-08)
  Raw feed (every delivery of the two beacons, as received):
  - /gnss/raw/front, /gnss/raw/rear — NavSatFix stamped with `updated`; status FIX if the
                        delivery is in the published stream, NO_FIX if a check dropped it
  - /gnss/hygiene     — std_msgs/String JSON per dropped delivery or re-anchor:
                        {"antenna", "verdict", "updated", "lat", "lon"}; counts in /diagnostics.
                        Includes no_pair (a new position whose partner is > 0.5 s old: passed
                        the per-antenna checks, not pair-checked; localization_node uses it as
                        an unpaired position), which is not logged

Usage:
  ros2 run nift_sensor beacon_node
"""

from dataclasses import fields
from datetime import datetime, timezone
import json
import math
import threading

import diagnostic_msgs.msg
import diagnostic_updater
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
import socketio
from std_msgs.msg import Float64, String

from .gnss_hygiene import BeaconHygiene, HygieneConfig
from .octane_client import normalize, octane_config  # normalize: also imported from here by tests

# verdicts of deliveries that are in the published stream
USED = ('accepted', 'reanchored', 'repeat')
# not logged
QUIET = ('accepted', 'repeat', 'repeat_unused', 'learning', 'no_pair', 'duplicate', 'ignored')
# not on /gnss/hygiene either (a re-anchor is on it)
TOPIC_QUIET = tuple(v for v in QUIET if v != 'no_pair')


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


def compute_heading_from_beacons(lat_r, lon_r, lat_f, lon_f):
    """
    Compute the compass heading of the vector pointing from the rear to the front antenna.

    The result is (heading_rad, heading_deg), where 0 deg = North and 90 deg = East.
    """
    lat_r_rad = math.radians(lat_r)
    lon_r_rad = math.radians(lon_r)
    lat_f_rad = math.radians(lat_f)
    lon_f_rad = math.radians(lon_f)

    # WGS84 radii of curvature (one sphere radius for both axes: 0.1 deg off on diagonals at Mcity)
    a, e2 = 6378137.0, 6.69437999014e-3
    lat_avg = 0.5 * (lat_r_rad + lat_f_rad)
    w = 1.0 - e2 * math.sin(lat_avg) ** 2
    r_east = a / math.sqrt(w)                   # prime vertical
    r_north = a * (1.0 - e2) / w ** 1.5         # meridian
    dx = (lon_f_rad - lon_r_rad) * math.cos(lat_avg) * r_east
    dy = (lat_f_rad - lat_r_rad) * r_north

    heading_rad = math.atan2(dx, dy)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360.0

    return heading_rad, heading_deg


# ----------------------------
# Node
# ----------------------------

class BeaconNode(Node):

    def __init__(self, **kwargs):
        # kwargs: Node options (tests: parameter_overrides)
        super().__init__('nift_beacon_node', **kwargs)

        # Publishers
        self.pub_fix = self.create_publisher(NavSatFix, '/gnss/fix', qos_profile_sensor_data)
        self.pub_vel = self.create_publisher(TwistStamped, '/gnss/vel', qos_profile_sensor_data)
        self.pub_heading = self.create_publisher(Float64, '/gnss/heading_deg',
                                                 qos_profile_sensor_data)
        self.pub_raw = {'front': self.create_publisher(NavSatFix, '/gnss/raw/front',
                                                       qos_profile_sensor_data),
                        'rear': self.create_publisher(NavSatFix, '/gnss/raw/rear',
                                                      qos_profile_sensor_data)}
        self.pub_hygiene = self.create_publisher(String, '/gnss/hygiene', 10)

        # Diagnostics
        self.diag_updater = diagnostic_updater.Updater(self)
        self.diag_updater.setHardwareID('gnss_beacon')
        self.diag_updater.add('GNSS Interface', self._diagnostics_callback)

        self._init_real_mode()

    # =========================================================================
    # REAL-WORLD MODE
    # =========================================================================

    def _init_real_mode(self):
        """
        Load the Octane settings and connect to the Octane server.

        Loads .env credentials and initializes the Socket.io connection to the Mcity Octane RTK
        server.
        """
        # nift_sensor/.env; MCITY_OCTANE_* already in the environment win
        cfg = octane_config()
        self.api_key = cfg.get('MCITY_OCTANE_KEY')
        self.server = cfg['MCITY_OCTANE_SERVER']
        self.namespace = cfg['MCITY_OCTANE_NAMESPACE']
        self.filter_id_front = cfg['MCITY_FRONT_BEACON_ID']
        self.filter_id_rear = cfg['MCITY_REAR_BEACON_ID']
        self.data_lock = threading.Lock()
        self.latest_by_id = {}
        self.gnss_timeout_s = float(self.declare_parameter('gnss_timeout_s', 0.5).value)
        self._stale = False
        self.hygiene = None
        if self.declare_parameter('hygiene.enabled', True).value:
            # 3 or 3.0 both work from the command line
            dyn = ParameterDescriptor(dynamic_typing=True)
            cfg = HygieneConfig(**{
                f.name: type(f.default)(
                    self.declare_parameter(f'hygiene.{f.name}', f.default, dyn).value)
                for f in fields(HygieneConfig)})
            self.hygiene = BeaconHygiene(self.filter_id_front, self.filter_id_rear, cfg)
            self.get_logger().info(f'beacon feed checks on: {cfg}')
        else:
            self.get_logger().warn(
                'beacon feed checks OFF (hygiene.enabled false): every new stamp is published')

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
        antenna = 'front' if beacon_id == self.filter_id_front else 'rear'
        now_ns = self.get_clock().now().nanoseconds
        with self.data_lock:
            if self.hygiene is not None:
                before = (self.hygiene.baseline_m, self.hygiene.baseline_moving)
                verdict = self.hygiene.add(n, now_ns / 1e9)
                learned = (self.hygiene.baseline_m, self.hygiene.baseline_moving)
                learned = learned if learned != before else None
            else:
                prev = self.latest_by_id.get(beacon_id)
                # An event that repeats the previous position (same `updated`) is not fresh data
                same = (prev is not None and n.get('ts') is not None
                        and n.get('ts') == prev.get('ts'))
                n['rx_ns'] = prev['rx_ns'] if same else now_ns
                self.latest_by_id[beacon_id] = n
                verdict, learned = 'accepted', None
        self._publish_raw(antenna, n, verdict in USED)
        if learned is not None:
            baseline, moving = learned
            how = 'learned while moving, learned again at the next stop' if moving else 'learned'
            self.get_logger().info(
                f'antenna baseline {baseline:.3f} m ({how}): feed checks active')
        if verdict not in TOPIC_QUIET:
            self.pub_hygiene.publish(String(data=json.dumps(
                {'antenna': antenna, 'verdict': verdict, 'updated': n.get('ts'),
                 'lat': n.get('lat'), 'lon': n.get('lon')})))
        if verdict not in QUIET:
            if verdict == 'reanchored':
                self.get_logger().warn(
                    f'GNSS feed checks re-anchored on a consistent pair ({antenna} delivery)')
            else:
                self.get_logger().warn(f'GNSS {antenna} delivery dropped: {verdict}',
                                       throttle_duration_sec=1.0)

    def _publish_raw(self, antenna, n, used):
        """One delivery as received (both beacons), for bags and the later filter."""
        if n.get('lat') is None or n.get('lon') is None:
            return
        fix = NavSatFix()
        fix.header.stamp = iso8601_to_ros_time(self, n.get('ts'))
        fix.header.frame_id = f'gps_{antenna}'
        fix.latitude = float(n['lat'])
        fix.longitude = float(n['lon'])
        elev = n.get('elev')
        fix.altitude = float(elev) if elev is not None else float(n.get('alt') or 0.0)
        fix.status.status = NavSatStatus.STATUS_FIX if used else NavSatStatus.STATUS_NO_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_raw[antenna].publish(fix)

    def _publish_latest(self):
        now_ns = self.get_clock().now().nanoseconds
        with self.data_lock:
            if self.hygiene is not None:
                h = self.hygiene
                if h.accepted is None:
                    return
                # copies: a repeat updates `ts` in place
                front, rear = (dict(x) for x in h.accepted)
                age_f, age_r = now_ns / 1e9 - h.new_rx[0], now_ns / 1e9 - h.new_rx[1]
                age_p = now_ns / 1e9 - h.accepted_rx
            else:
                front = self.latest_by_id.get(self.filter_id_front)
                rear = self.latest_by_id.get(self.filter_id_rear)
                if front is None or rear is None:
                    return
                age_f = (now_ns - front['rx_ns']) / 1e9
                age_r = (now_ns - rear['rx_ns']) / 1e9
                age_p = 0.0

        if max(age_f, age_r, age_p) > self.gnss_timeout_s:
            if not self._stale:
                self._stale = True
                checked = (f', last consistent pair {age_p:.2f} s' if self.hygiene is not None
                           else '')
                self.get_logger().warn(
                    f'GNSS stale (no new position: front {age_f:.2f} s, '
                    f'rear {age_r:.2f} s{checked}) '
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

        spd = front.get('speed')               # m/s: normalize() already converted Octane's knots
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
        if getattr(self, 'hygiene', None) is None:
            stat.add('feed checks', 'off')
            return stat
        with self.data_lock:
            counts, baseline = dict(self.hygiene.counts), self.hygiene.baseline_m
            moving = self.hygiene.baseline_moving
        stat.add('antenna baseline (m)', 'learning' if baseline is None else f'{baseline:.3f}')
        if moving:
            stat.add('antenna baseline learned', 'while moving (learned again at the next stop)')
        for k in ('accepted', 'repeat', 'reanchored', 'out_of_order', 'stuck', 'older_position',
                  'baseline', 'heading', 'step', 'time_jump'):
            stat.add(f'feed check {k}', str(counts.get(k, 0)))
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
