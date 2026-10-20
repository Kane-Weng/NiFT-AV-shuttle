#!/usr/bin/env python3
"""
ROS node that runs the localization filter and publishes the map pose.

File: localization_node.py

Description:
  The localization filter (localization_filter.LocalizationFilter, an EKF on a kinematic bicycle)
  as a ROS node:
  the pose of base_link in the map frame from the two GNSS antennas, the GNSS speed and the
  steering the stack commands, predicted to "now" and carried across short GNSS gaps, with a status
  that says how long and how far it has been carried.

    mode ekf     publishes the TF map -> base_link (in place of shuttle_tf_node),
                 /localization/odom, status
    mode shadow  /localization/odom and status only: shuttle_tf_node keeps the TF, nothing
                 downstream changes

  In (topic, type: what is used)
    /gnss/raw/front, /gnss/raw/rear  NavSatFix: every Octane delivery (beacon_node). Status FIX =
                    in the published stream, i.e. a new position the feed checks passed as a pair
                    (or a repeat of one: the filter skips repeats). NO_FIX = dropped: not used, but
                    remembered, so while moving a FIX re-delivery of a position that was dropped
                    (bags from before beacon_node marked those repeat_unused) is not taken as new
    /gnss/hygiene   String JSON: deliveries the feed checks dropped. Verdict no_pair (a new
                    position that passed the per-antenna checks, but its partner is > 0.5 s old:
                    one antenna out) carries lat / lon and is used as an unpaired position: it can
                    carry a confident filter, not revive a bridging one
    /gnss/vel       TwistStamped: the accepted pair's front speed, 50 Hz; used once per new front
                    position (its stamp), unsigned
    /to_can_bus     can_msgs/Frame 0x101: our own drive command. The steering angle (front wheel, +
                    = left), the gear (sign of the speed) and the brake (a deceleration after the
                    brake's dead time) are used only while the stack drives the wheels, i.e. while
                    trajectory_controller's /nav_telemetry STATUS is one of steer_input_states (and
                    fresh). Otherwise (IDLE, the Xbox controller driving:
                    the ECU then ignores 0x101) the filter runs without steering: the turn is
                    assumed to go on
    /nav_telemetry  String: trajectory_controller's "STATUS:    <state>" line
  Out
    TF map -> base_link   (ekf mode) 50 Hz, stamped now, the pose predicted to now; also while LOST
                          (the pose is then dead-reckoned and must not be driven on: watch the
                          status)
    /localization/odom    nav_msgs/Odometry, map -> base_link, 50 Hz: pose + covariance (x, y,
                          yaw); twist in base_link (v, v * kappa * base_link_x, v * kappa) + speed
                          variance
    /localization/status  diagnostic_msgs/DiagnosticStatus, 50 Hz: level OK / WARN (= DEGRADED) /
                          ERROR (= LOST), message "<LEVEL> <reason>[: why it does not (re)start]",
                          values age_s, dist_m, sigma_pos_m, sigma_yaw_deg, steering_input,
                          antenna_baseline_m (+ _source: configured / measured), rear_antenna_x_m,
                          counts
    /diagnostics          the same summary (hardware id localization)

  Parameters: mode; map_origin_lat / map_origin_lon (as shuttle_tf_node); antenna.front_x_m,
  antenna.rear_x_m, antenna.y_m (from base_link, + = forward / left), speed_point_x_m (where the
  GNSS speed is measured: the front receiver on the shuttle, the rear axle in the kinematic sim),
  base_link_x_m (base_link ahead of the rear axle), wheelbase_m; gnss_delay_s (mean age of a
  position at its first delivery: 0 for the sim's ideal feed, ~0.061 for its real-feed model);
  steer_input_states, telemetry_timeout_s; brake.min_count, brake.decel_mps2, brake.dead_time_s (0
  decel = no braking input); filter.<field> for every other FilterConfig field (thresholds:
  ok_age_s, bridge_s, bridge_m, ...). The geometry, brake.min_count and the map origin default to
  nift_common (vehicle.yaml, site.py); config/localization.yaml has the shuttle's other values.
  Lever arms: the front one defines base_link (as shuttle_tf_node's gnss.offset_x). If the
  antennas' measured distance at start-up is not front - rear, the filter adopts it for the rear
  one (logged once, and in the status values), so a wrong lever arm only shifts the pose, as in
  direct mode.

Usage:
  ros2 run nift_sensor localization_node --ros-args --params-file <config/localization.yaml> -p
  mode:=shadow
"""

from dataclasses import fields
import json
import math

from can_msgs.msg import Frame
import diagnostic_msgs.msg
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import diagnostic_updater
from geometry_msgs.msg import TransformStamped, TwistStamped
import lanelet2
from lanelet2.projection import UtmProjector
from nav_msgs.msg import Odometry
from nift_common.can_protocol import CAN_ID_COMMAND, decode_command_frame, Gear
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
from nift_common.vehicle import ShuttleConfig
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .gnss_hygiene import stamp_s as iso_stamp_s
from .localization_filter import FilterConfig, LocalizationFilter, MOVING_MPS, StampClock

SHUTTLE = ShuttleConfig()
MODES = ('ekf', 'shadow')
LEVEL = {'OK': DiagnosticStatus.OK, 'DEGRADED': DiagnosticStatus.WARN,
         'LOST': DiagnosticStatus.ERROR}
# FilterConfig fields set directly by name
NODE_PARAMS = {'wheelbase_m', 'base_link_x_m', 'speed_point_x_m'}
ANTENNA_PARAMS = {'front_antenna_x_m': 'antenna.front_x_m', 'rear_antenna_x_m': 'antenna.rear_x_m',
                  'antenna_y_m': 'antenna.y_m'}


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def telemetry_state(text: str):
    """
    Return the STATUS value of a /nav_telemetry message.

    trajectory_controller's /nav_telemetry -> its STATUS value ('NAVIGATING', 'SAFE_STOP', ...), or
    None.
    """
    for line in text.splitlines():
        if line.startswith('STATUS:'):
            return line.split(':', 1)[1].strip()
    return None


class LocalizationNode(Node):

    def __init__(self, **kwargs):
        # kwargs: Node options (tests: parameter_overrides)
        super().__init__('localization_node', **kwargs)
        p = lambda name, default: self.declare_parameter(name, default).value      # noqa: E731
        self.mode = str(p('mode', 'ekf'))
        if self.mode not in MODES:
            raise ValueError(f'mode:={self.mode}: expected one of {MODES} '
                             "('direct' = run shuttle_tf_node instead)")
        lat0, lon0 = (float(p('map_origin_lat', MAP_ORIGIN_LAT)),
                      float(p('map_origin_lon', MAP_ORIGIN_LON)))
        self.projector = UtmProjector(lanelet2.io.Origin(lat0, lon0))
        ref = self.projector.forward(lanelet2.core.GPSPoint(lat0, lon0, 0.0))
        self.origin = (ref.x, ref.y)

        # 1 or 1.0 both work from the command line
        dyn = ParameterDescriptor(dynamic_typing=True)
        values = {}
        for f in fields(FilterConfig):
            name = ANTENNA_PARAMS.get(f.name, f.name if f.name in NODE_PARAMS
                                      else f'filter.{f.name}')
            values[f.name] = type(f.default)(self.declare_parameter(name, f.default, dyn).value)
        self.cfg = FilterConfig(**values)
        self.filter = LocalizationFilter(self.cfg)
        self.clock_map = StampClock(delay_s=float(p('gnss_delay_s', 0.0)))
        self.steer_states = tuple(str(s) for s in p('steer_input_states', ['NAVIGATING', 'MANUAL',
                                                                           'SAFE_STOP']))
        self.telemetry_timeout_s = float(p('telemetry_timeout_s', 0.5))
        self.brake_min = int(p('brake.min_count', SHUTTLE.brake_full_count))
        self.brake_decel = float(p('brake.decel_mps2', 0.6))
        self.brake_dead_s = float(p('brake.dead_time_s', 0.47))
        self.map_frame, self.base_frame = str(p('map_frame', 'map')), str(p('base_frame',
                                                                            'base_link'))
        rate = float(p('publish_rate_hz', 50.0))

        # state
        # (lat, lon, used) of each antenna's previous delivery
        self._last_raw = {'front': None, 'rear': None}
        # stamp [ns] of a new front position -> its measurement time
        self._front_new = {}
        self._vel_stamp = None
        self._gnss_speed = 0.0                          # the last /gnss/vel [m/s]
        self._tele = (None, None)                       # (STATUS, node time [s] it arrived)
        # last 0x101: (time [s], steer_deg, gear, brake)
        self._cmd = None
        # (time [s], brake on) changes, for the dead time
        self._brake_hist = []
        self._input = None                              # the input last handed to the filter
        # the last gear that drives (+1 FORWARD, -1 REVERSE)
        self._direction = 0
        self._baseline_logged = False
        self.counts = {'raw_fix': 0, 'raw_dropped': 0, 'repeat_of_dropped': 0, 'unpaired': 0,
                       'speed': 0,
                       'can_0x101': 0}

        self.tf_broadcaster = TransformBroadcaster(self) if self.mode == 'ekf' else None
        self.pub_odom = self.create_publisher(Odometry, '/localization/odom', 10)
        self.pub_status = self.create_publisher(DiagnosticStatus, '/localization/status', 10)
        self.create_subscription(NavSatFix, '/gnss/raw/front', lambda m: self.raw_callback('front',
                                                                                           m),
                                 qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gnss/raw/rear', lambda m: self.raw_callback('rear',
                                                                                          m),
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/gnss/hygiene', self.hygiene_callback, 50)
        self.create_subscription(TwistStamped, '/gnss/vel', self.vel_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Frame, '/to_can_bus', self.can_callback, 50)
        self.create_subscription(String, '/nav_telemetry', self.telemetry_callback, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.diag = diagnostic_updater.Updater(self)
        self.diag.setHardwareID('localization')
        self.diag.add('Localization', self._diagnostics_callback)
        self._last_status = None
        c = self.cfg
        self.get_logger().info(
            f'localization filter, mode {self.mode} '
            f'({"TF map -> base_link" if self.tf_broadcaster else "no TF"}); '
            f'antennas {c.front_antenna_x_m:+.3f} / {c.rear_antenna_x_m:+.3f} m from base_link, '
            f'speed at {c.speed_point_x_m:+.3f} m, gnss delay {self.clock_map.delay_s:.3f} s, '
            f'bridging {c.bridge_s} s / '
            f'{c.bridge_m} m, steering input in {list(self.steer_states)}')

    # ------------------------------------------------------------------ #
    #  Inputs                                                            #
    # ------------------------------------------------------------------ #
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _to_map(self, lat, lon):
        p = self.projector.forward(lanelet2.core.GPSPoint(lat, lon, 0.0))
        return p.x - self.origin[0], p.y - self.origin[1]

    def raw_callback(self, antenna: str, msg: NavSatFix):
        lat, lon = msg.latitude, msg.longitude
        prev = self._last_raw[antenna]
        if msg.status.status < NavSatStatus.STATUS_FIX:             # dropped by the feed checks
            self.counts['raw_dropped'] += 1
            self._last_raw[antenna] = (lat, lon, False)
            return
        if prev is not None and (prev[0], prev[1]) == (lat, lon) and not prev[2] \
                and self._moving(self._now()):
            # re-delivery of a dropped position: not new
            self.counts['repeat_of_dropped'] += 1
            # (at rest the same position is the current one)
            return
        self.counts['raw_fix'] += 1
        self._last_raw[antenna] = (lat, lon, True)
        now = self._now()
        t = self.clock_map.measurement_time(stamp_s(msg.header.stamp), now)
        x, y = self._to_map(lat, lon)
        verdict = self.filter.update_antenna(t, antenna, x, y, paired=True)
        if antenna == 'front' and verdict != 'repeat':
            key = Time.from_msg(msg.header.stamp).nanoseconds
            self._front_new[key] = t
            if len(self._front_new) > 64:
                self._front_new.pop(next(iter(self._front_new)))

    def hygiene_callback(self, msg: String):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get('verdict') != 'no_pair' or d.get('lat') is None or d.get('lon') is None:
            # (bags before 3.2b carry no position)
            return
        antenna = d.get('antenna')
        if antenna not in ('front', 'rear'):
            return
        self.counts['unpaired'] += 1
        stamp = iso_stamp_s(d.get('updated'))
        now = self._now()
        t = self.clock_map.measurement_time(stamp, now) if stamp is not None else now
        self._last_raw[antenna] = (d['lat'], d['lon'], True)
        x, y = self._to_map(d['lat'], d['lon'])
        self.filter.update_antenna(t, antenna, x, y, paired=False)

    def _moving(self, now: float) -> bool:
        """
        Tell whether the shuttle is moving.

        The shuttle moves (at rest positions stay bit-identical for minutes, as gnss_hygiene's
        repeat rule knows): the filter's speed, dead-reckoned through a stop; before it starts, the
        last GNSS speed (which freezes when beacon_node goes silent).
        """
        pose = self.filter.pose_at(now)
        return abs(pose.v if pose is not None else self._gnss_speed) >= MOVING_MPS

    def vel_callback(self, msg: TwistStamped):
        self._gnss_speed = abs(msg.twist.linear.x)
        key = Time.from_msg(msg.header.stamp).nanoseconds
        if key == self._vel_stamp:
            # the 50 Hz republish of the same delivery
            return
        self._vel_stamp = key
        t = self._front_new.pop(key, None)
        if t is None:
            # a repeat, or a delivery the filter skipped
            return
        self.counts['speed'] += 1
        self.filter.update_speed(t, abs(msg.twist.linear.x))

    def can_callback(self, frame: Frame):
        if frame.id != CAN_ID_COMMAND or frame.is_extended:
            return
        try:
            cmd = decode_command_frame(frame)
        except ValueError:
            return
        now = self._now()
        self.counts['can_0x101'] += 1
        brake_on = cmd.brake >= self.brake_min > 0
        if not self._brake_hist or self._brake_hist[-1][1] != brake_on:
            self._brake_hist.append((now, brake_on))
            self._brake_hist = self._brake_hist[-8:]
        self._cmd = (now, cmd.steering_angle_deg, cmd.gear, cmd.brake)
        self._update_input(now)

    def telemetry_callback(self, msg: String):
        state = telemetry_state(msg.data)
        if state is not None:
            self._tele = (state, self._now())
            self._update_input(self._now())

    def in_control(self, now: float) -> bool:
        """
        Tell whether our steering commands drive the wheels.

        Our 0x101 drives the wheels: the controller says so, recently, and commands keep coming.
        """
        state, t = self._tele
        fresh_cmd = self._cmd is not None and now - self._cmd[0] <= self.telemetry_timeout_s
        return state in self.steer_states and now - t <= self.telemetry_timeout_s and fresh_cmd

    def _braking(self, now: float) -> bool:
        """
        Return the brake state the vehicle feels now.

        The brake as the vehicle feels it now: the commanded one dead_time_s ago.
        """
        on = False
        for t, state in self._brake_hist:
            if t <= now - self.brake_dead_s:
                on = state
        return on

    def _update_input(self, now: float):
        if self.in_control(now):
            _, steer, gear, _ = self._cmd
            if gear in (Gear.FORWARD, Gear.REVERSE):
                self._direction = 1 if gear == Gear.FORWARD else -1
            # NEUTRAL (SAFE_STOP, IDLE hold): it rolls on the way it was going
            direction = self._direction
            decel = self.brake_decel if self.brake_decel > 0.0 and self._braking(now) else 0.0
            new = (round(steer, 2), direction, decel)
        else:
            new = (None, 0, 0.0)
            self._direction = 0
        if new != self._input:
            self._input = new
            self.filter.set_input(now, *new)

    # ------------------------------------------------------------------ #
    #  Output                                                            #
    # ------------------------------------------------------------------ #
    def tick(self):
        now = self._now()
        self._update_input(now)                 # timeouts, and the brake's dead time
        st = self.filter.status(now)
        self._last_status = st
        if self.filter.baseline_measured is not None and not self._baseline_logged:
            self._baseline_logged = True
            c = self.cfg
            self.get_logger().warn(
                f'antenna distance measured {self.filter.baseline_measured:.3f} m, '
                f'the lever arms say '
                f'{self.filter.baseline_configured:.3f} m (front {c.front_antenna_x_m:+.3f}, rear '
                f'{c.rear_antenna_x_m:+.3f}): using rear {self.filter.rear_antenna_x:+.3f} m. '
                f'The front one stays (it defines base_link): if it is wrong too, '
                f'the pose is shifted along the heading, as in direct '
                f'mode. Measure gnss_front_x / gnss_rear_x (nift_common vehicle.yaml)')
        stamp = Time(nanoseconds=int(now * 1e9)).to_msg()
        self.pub_status.publish(self._status_msg(st))
        pose = self.filter.pose_at(now)
        if pose is None:
            return
        qz, qw = math.sin(pose.yaw / 2.0), math.cos(pose.yaw / 2.0)
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id, t.child_frame_id = self.map_frame, self.base_frame
            t.transform.translation.x, t.transform.translation.y = float(pose.x), float(pose.y)
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf_broadcaster.sendTransform(t)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id, o.child_frame_id = self.map_frame, self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = float(pose.x), float(pose.y)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        cov = [0.0] * 36
        # x, y, yaw in the 6x6 (x y z roll pitch yaw)
        for i, a in enumerate((0, 1, 5)):
            for j, b in enumerate((0, 1, 5)):
                cov[a * 6 + b] = float(pose.cov[i, j])
        for a in (2, 3, 4):
            cov[a * 6 + a] = 1e6                                    # not estimated
        o.pose.covariance = cov
        o.twist.twist.linear.x = float(pose.v)
        o.twist.twist.linear.y = float(pose.v * pose.kappa * self.cfg.base_link_x_m)
        o.twist.twist.angular.z = float(pose.yaw_rate)
        tcov = [0.0] * 36
        tcov[0] = float(pose.v_var)
        o.twist.covariance = tcov
        self.pub_odom.publish(o)

    def _values(self, st):
        c = self.filter.counts
        steering = self._input is not None and self._input[0] is not None
        vals = {'age_s': f'{st.age_s:.3f}', 'dist_m': f'{st.dist_m:.2f}',
                'sigma_pos_m': f'{st.sigma_pos_m:.3f}',
                'sigma_yaw_deg': f'{st.sigma_yaw_deg:.2f}', 'steering_input': 'on' if steering
                else 'off',
                'mode': self.mode, 'antenna_baseline_m': f'{self.filter.baseline:.3f}',
                'antenna_baseline_source': 'configured' if self.filter.baseline_measured is None
                else 'measured',
                'rear_antenna_x_m': f'{self.filter.rear_antenna_x:+.3f}'}
        for k in ('accepted', 'rejected', 'distrusted', 'unpaired_ignored', 'late', 'initialised',
                  'init_bad_pair', 'init_bad_yaw'):
            vals[f'filter {k}'] = str(c.get(k, 0))
        vals.update({k: str(v) for k, v in self.counts.items()})
        return vals

    @staticmethod
    def _message(st) -> str:
        """'LOST uninitialised: antenna distance 2.88 m vs configured 2.08 m ...', 'OK', ..."""
        return f'{st.level} {st.reason}'.strip() + (f': {st.detail}' if st.detail else '')

    def _status_msg(self, st) -> DiagnosticStatus:
        msg = DiagnosticStatus(level=LEVEL[st.level], name='localization',
                               hardware_id='localization',
                               message=self._message(st))
        msg.values = [KeyValue(key=k, value=v) for k, v in self._values(st).items()]
        return msg

    def _diagnostics_callback(self, stat):
        st = self._last_status
        if st is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'starting')
            return stat
        stat.summary(LEVEL[st.level], self._message(st))
        for k, v in self._values(st).items():
            stat.add(k, v)
        return stat


def main():
    rclpy.init()
    node = None
    try:
        node = LocalizationNode()
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
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
