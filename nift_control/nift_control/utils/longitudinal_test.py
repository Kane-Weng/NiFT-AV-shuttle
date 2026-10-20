#!/usr/bin/env python3
"""
Scripted, guarded test passes that identify the shuttle's speed and steering response.

File: longitudinal_test.py
Date: Oct 2, 2026

Description:
  Scripted, guarded longitudinal tests for identifying the shuttle's speed plant (pedal / brake ->
  speed) and for checking ADRC in closed loop, plus steering passes (steer, steer_drive) for the
  steering response. One run = one pass from standstill that ends braked and stopped. Steering is
  held at 0 except in the steering passes. The wiki page longitudinal_test describes the field
  procedure.

  Tests
    static   NEUTRAL + full brake, no motion: GNSS message rates and the speed reading at rest
    ramp     breakaway: brake released at pedal 0 for --release-secs (the shuttle needs ~2 s to
             engage after a brake release, measured 2026-10-08), then the pedal rises slowly until
             the shuttle moves, then the brake stops it
    step     open loop: pedal P until the speed levels off, optional coast, then brake B until
             stopped
    adrc     closed loop: ADRC + NiftVehicleBridge (as in trajectory_controller) to speed V, hold,
             then target 0 (ADRC stops it); if a soft limit ends the drive instead, brake --brake
             stops it
    steer    parked steering steps: NEUTRAL + full brake, the wheel follows --seq
             ("angle:secs,..."; + = left). The RESULT line says, per step, how the 0x301 steering
             value followed the command: jumps there in one frame (an echo of the command) or moves
             over several frames (a measured wheel angle).
    steer_drive  moving steering pass: ADRC to --speed with the wheel straight, after --runup m
             the wheel follows --seq, then ADRC stops the shuttle with the wheel held at the last
             angle. Extra guard: the heading change since the start (GNSS heading) plus the turn
             still to come while braking must stay below --max-turn.
    tune     interactive: the Tkinter tuner (longitudinal_tuner.py) — speed / steer by hand through
             ADRC, live ADRC re-tuning, manual CAN frames, E-stop. NOT a guarded pass: none of the
             limits or guards below apply.
    monitor  live window for the pass running in another terminal: phase, distance and speed
             against the limits (longitudinal_monitor.py). Read-only: it only listens to
             /long_test/state.
  adrc and tune start from trajectory_controller's gains in adrc_params.yaml; --b0 / --w_cl /
  --k_eso override.

  Limits. Soft — the drive phase (pedal / coast / ADRC) ends early and the planned brake phase
  follows:
    speed >= 90 % of --max-speed, or distance + predicted stop >= --max-dist minus --brake-room.
  Guards (moving tests). Any trip -> ABORT: brake (--abort-brake) until stopped, then hold.
    distance travelled from the start fix + predicted stopping distance >= --max-dist
    /gnss/vel above --max-speed
    no /gnss/vel or /gnss/fix for --gnss-timeout (1.0 s; beacon_node stops publishing 0.5 s into a
    beacon pause, so a pause of 1.5 s aborts: 3 in 34 min on 2026-10-08, against 18 with the old
    0.5 s). The possible blind distance (v x timeout) is added to the predicted stopping distance.
    Ctrl-C (a second Ctrl-C sends one hold frame and quits at once)
    steer_drive only: heading change + turn while stopping >= --max-turn, or no
    /gnss/heading_deg for --gnss-timeout
    An abort, a soft limit and the ADRC stop keep the wheel where it is: braking never moves
    the steering.
  The distance comes from GNSS positions, not /gnss/vel, so it holds even if the speed scale is
  wrong. Defaults come from a garage test session on the shuttle (2026-10-08): the weakest abort
  brake measured 0.17 m/s² by position, so --assumed-decel is 0.15; the fastest pass reached 1.33
  m/s, so --max-speed is 2.0. /gnss/vel is m/s since beacon_node converts Octane's knots.

  Publishes /to_can_bus at 20 Hz — never run it next to trajectory_controller or a second
  longitudinal_test. /long_test/state (std_msgs/String, JSON, 20 Hz): phase, command, target,
  speed, distance, predicted stop, limits, ended_by, abort reason, steering command / 0x301 value,
  GNSS fix and heading, heading change. The final RESULT line (JSON) summarises the pass.

Usage:
  ros2 run nift_control longitudinal_test static
  ros2 run nift_control longitudinal_test ramp
  ros2 run nift_control longitudinal_test step --pedal 60 --brake 80 [--coast 3]
  ros2 run nift_control longitudinal_test adrc --speed 2.0 [--b0 0.5 --w_cl 2.0 --k_eso 5.0]
  ros2 run nift_control longitudinal_test steer --seq 15:5,0:5,-15:5,0:5
  ros2 run nift_control longitudinal_test steer_drive --speed 1.0 --seq 10:2,-10:2,0:3
      [--runup 6 --max-turn 75]
  ros2 run nift_control longitudinal_test tune [--b0 0.5 --w_cl 2.0 --k_eso 5.0]
  ros2 run nift_control longitudinal_test monitor [--pad garage_pad.yaml]
  common (not tune / monitor): --max-dist 25 --max-speed 2.0 --abort-brake 150 --assumed-decel 0.15
  sim: append --ros-args -p use_sim_time:=true
"""

import argparse
from collections import deque
import json
import math
import os
import signal
import sys

from ament_index_python.packages import get_package_share_directory
from can_msgs.msg import Frame
from geometry_msgs.msg import TwistStamped
from nift_common.can_protocol import CAN_CMD_MAX, CAN_ID_ACK, decode_steering, DLC_COMMAND
from nift_common.vehicle import ShuttleConfig
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String
import yaml

from ..controllers import ADRCController
from ..dbw_interface import create_command_frame, Gear, NiftVehicleBridge
from .steer_util import (
    ack_steps,
    ack_summary,
    parse_seq,
    seq_angle,
    seq_duration,
    turn_after_stop_deg,
    wrap_deg,
)

CONTROL_HZ = 20
BEACON_GATE_S = 0.5     # beacon_node's stale gate: it goes silent this long into a beacon pause
GNSS_WAIT_S = 10.0    # refuse to start without GNSS by then
START_MAX_V = 0.2     # m/s — must start from standstill
STOPPED_V = 0.05    # m/s
STOPPED_HOLD_S = 1.0     # this long below STOPPED_V = stopped
ARM_S = 1.0     # gear engaged under full brake before the test starts
FINAL_HOLD_S = 2.0     # NEUTRAL + full brake at the end of every run
ABORT_MAX_S = 8.0     # abort braking ends after this even without a confirmed stop (GNSS lost)
# planned brake / ADRC stop: longest phase (a noisy speed at rest never reads 'stopped')
BRAKE_MAX_S = 15.0
REACTION_S = 0.3     # guard look-ahead before the brake bites
SOFT_V_FRAC = 0.9     # the drive phase ends at this fraction of --max-speed
NO_MOTION_S = 8.0     # step / adrc: no motion this long -> end the drive phase
PLATEAU_WIN_S = 2.0     # step: window of the speed-slope fit
PLATEAU_SLOPE = 0.05    # m/s² — |slope| below this = speed has levelled off
# m/s — every sample in the window must be at least this (no standstill / dead time)
PLATEAU_MIN_V = 0.3
V_INT_PHASES = (
    'release',
    'ramp',
    'pedal',
    'coast',
    'brake',
    'adrc',
    'adrc_stop',
    'abort',
    'runup',
    'pattern',
)
# ∫ /gnss/vel dt runs over these only: the final hold adds rest noise and its
# GNSS gaps (not guarded there) used to void vel_over_pos
EARTH_R = 6371000.0
# steer: the shuttle moved this far from the start fix while parked -> end the pass
PARK_MOVE_M = 0.3
WHEELBASE_M = ShuttleConfig().wheel_base


def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def adrc_gains(a):
    """
    Return the ADRC gains and bridge zero-speed tolerance as trajectory_controller gets them.

    They come from the installed adrc_params.yaml, with --b0 / --w_cl / --k_eso on top.
    """
    path = os.path.join(get_package_share_directory('nift_control'), 'config', 'adrc_params.yaml')
    with open(path) as f:
        p = yaml.safe_load(f)['trajectory_controller']['ros__parameters']
    gains = dict(p['adrc'])
    for k in ('b0', 'w_cl', 'k_eso'):
        if getattr(a, k) is not None:
            gains[k] = getattr(a, k)
    return gains, p['vehicle_bridge']['zero_speed_tolerance']


def parse_args(argv):
    argv = list(argv)
    # "--seq -15:2,15:2" would read as an option: glue it on
    for i in range(len(argv) - 1):
        if argv[i] == '--seq' and argv[i + 1].startswith('-'):
            argv[i:i + 2] = [f'--seq={argv[i + 1]}']
            break
    ap = argparse.ArgumentParser(prog='longitudinal_test', description=__doc__.split('Usage:')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        '--max-dist', type=float, default=25.0, help='distance budget from the start [m]'
    )
    common.add_argument('--max-speed', type=float, default=2.0, help='speed cap [m/s]')
    common.add_argument(
        '--abort-brake', type=int, default=150, help='brake count used by an abort'
    )
    common.add_argument(
        '--brake-room',
        type=float,
        default=5.0,
        help='distance kept free for the planned brake phase before the hard limit [m]',
    )
    common.add_argument(
        '--gnss-timeout',
        type=float,
        default=1.0,
        help='abort when /gnss/vel or /gnss/fix is silent this long [s]; '
        'beacon_node already waits '
        f'{BEACON_GATE_S} s, so a beacon pause of this + {BEACON_GATE_S} s aborts',
    )
    common.add_argument(
        '--assumed-decel',
        type=float,
        default=0.15,
        help='deceleration the distance guard assumes for an abort [m/s²]; 0.8 x the weakest '
        'abort brake measured on the shuttle (0.17 m/s² by position, 2026-10-08)',
    )
    gains = argparse.ArgumentParser(add_help=False)
    gains.add_argument('--b0', type=float)
    gains.add_argument('--w_cl', type=float)
    gains.add_argument('--k_eso', type=float)
    sub = ap.add_subparsers(dest='test', required=True)
    s = sub.add_parser('static', parents=[common], help='brake hold, no motion')
    s.add_argument('--secs', type=float, default=20.0)
    s = sub.add_parser('ramp', parents=[common], help='breakaway pedal ramp')
    s.add_argument(
        '--release-secs',
        type=float,
        default=3.0,
        help='brake released at pedal 0 this long before the ramp starts (drive engagement) [s]',
    )
    s.add_argument('--rate', type=float, default=5.0, help='pedal rise [counts/s]')
    s.add_argument('--max-pedal', type=int, default=120)
    s.add_argument(
        '--move-speed', type=float, default=0.3, help='speed that counts as moving [m/s]'
    )
    s.add_argument('--brake', type=int, default=80)
    s = sub.add_parser('step', parents=[common], help='open-loop pedal step, coast, brake step')
    s.add_argument('--pedal', type=int, required=True)
    s.add_argument('--brake', type=int, required=True)
    s.add_argument(
        '--coast', type=float, default=0.0, help='seconds with pedal and brake at 0 before braking'
    )
    s.add_argument(
        '--plateau-hold',
        type=float,
        default=3.0,
        help='keep the pedal this long once levelled [s]',
    )
    s.add_argument('--pedal-secs', type=float, default=20.0, help='longest pedal phase [s]')
    s = sub.add_parser('adrc', parents=[common, gains], help='closed-loop ADRC speed step')
    s.add_argument('--speed', type=float, required=True, help='target [m/s]')
    s.add_argument(
        '--hold', type=float, default=5.0, help='seconds at target once reached (90 %%)'
    )
    s.add_argument('--drive-secs', type=float, default=30.0, help='longest drive phase [s]')
    s.add_argument(
        '--brake', type=int, default=80, help='brake count if a soft limit ends the drive phase'
    )
    s = sub.add_parser(
        'steer', parents=[common], help='parked steering steps (NEUTRAL + full brake)'
    )
    s.add_argument(
        '--seq',
        default='5:3,0:3',
        help='steering steps "angle:secs,..." in degrees, + = left (default: the direction check)',
    )
    s = sub.add_parser(
        'steer_drive', parents=[common, gains], help='steering steps while driving at --speed'
    )
    s.add_argument('--speed', type=float, default=1.0, help='ADRC target [m/s]')
    s.add_argument(
        '--runup',
        type=float,
        default=6.0,
        help='straight distance before the steering starts; also waits for 90 %% of --speed [m]',
    )
    s.add_argument(
        '--seq',
        required=True,
        help='steering steps "angle:secs,..." in degrees, + = left; the wheel '
        'stays at the last angle while ADRC stops the shuttle',
    )
    s.add_argument(
        '--max-turn',
        type=float,
        default=75.0,
        help='abort when the heading change + the turn while stopping reaches this [deg]',
    )
    s.add_argument('--drive-secs', type=float, default=30.0, help='longest run-up [s]')
    s.add_argument(
        '--brake', type=int, default=80, help='brake count if a soft limit ends the drive phase'
    )
    sub.add_parser('tune', parents=[gains], help='interactive tuner GUI (no limits or guards)')
    s = sub.add_parser(
        'monitor', help='live window of the running pass + garage pad map (read-only)'
    )
    s.add_argument('--pad', help='pad map config (default: the installed config/garage_pad.yaml)')
    a = ap.parse_args(argv)
    if getattr(a, 'gnss_timeout', 1.0) <= 0:
        ap.error('--gnss-timeout must be > 0')
    for name, lo, hi in (
        ('pedal', 0, CAN_CMD_MAX),
        ('brake', 0, CAN_CMD_MAX),
        ('abort_brake', 1, CAN_CMD_MAX),
        ('max_pedal', 1, CAN_CMD_MAX),
    ):
        if getattr(a, name, None) is not None and not lo <= getattr(a, name) <= hi:
            ap.error(f'--{name.replace("_", "-")} must be {lo}..{hi}')
    if a.test in ('steer', 'steer_drive'):
        try:
            a.seq_parsed = parse_seq(a.seq)
        except ValueError as e:
            ap.error(f'--seq: {e}')
    if a.test == 'steer_drive' and not 0.0 < a.max_turn <= 120.0:
        ap.error('--max-turn must be in (0, 120] deg')
    if a.test in ('adrc', 'steer_drive') and not 0.0 < a.speed <= 0.85 * a.max_speed:
        ap.error(f'--speed must be > 0 and <= 85 % of --max-speed ({0.85 * a.max_speed:.2f} m/s; '
                 'the soft limit sits at 90 %)')
    return a


class LongitudinalTest(Node):

    def __init__(self, a):
        super().__init__('longitudinal_test')
        self.a = a
        self.can_pub = self.create_publisher(Frame, 'to_can_bus', 10)
        self.state_pub = self.create_publisher(String, '/long_test/state', 10)
        self.create_subscription(TwistStamped, '/gnss/vel', self._on_vel, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gnss/fix', self._on_fix, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/gnss/heading_deg', self._on_heading, qos_profile_sensor_data
        )
        self.create_subscription(Frame, 'from_can_bus', self._on_can, 100)

        # GNSS
        self.v = 0.0
        self.t_vel = self.t_fix = None
        self.fix = None
        self.start_fix = None
        self.dist = 0.0
        self.n_vel = self.n_fix = 0
        self.v_int = 0.0                       # ∫ /gnss/vel dt over the pass (speed-scale check)
        self.gnss_gap = False                  # a /gnss/vel gap during the pass voids that check
        self._v_prev = None
        self.v_hist = deque()                  # (t, v) for the plateau fit
        self.v_rest = []                       # static: speeds at rest
        self.heading = self.t_head = None      # compass deg (rear -> front antenna), beacon_node
        self.start_heading = None
        # heading change since the start, deg, + = left (CCW)
        self.turn = 0.0
        self.turn_max = 0.0

        # Steering (steer / steer_drive): the commanded wheel angle and what 0x301 reports back
        self.steer = 0.0
        self.steer_ack = None
        self.acks = []                         # (t, deg) of every 0x301
        self.steer_events = []                 # (t, new, prev) at every change of the command
        self.seq = getattr(a, 'seq_parsed', None)
        self.seq_step = None

        # ADRC (adrc test only)
        self.ctrl = self.bridge = None
        if a.test in ('adrc', 'steer_drive'):
            gains, zero_tol = adrc_gains(a)
            self.gains = gains
            self.ctrl = ADRCController(delta=1.0 / CONTROL_HZ, b0=gains['b0'], w_cl=gains['w_cl'],
                                       k_eso=gains['k_eso'])
            self.bridge = NiftVehicleBridge(zero_speed_tolerance=zero_tol)
            self.get_logger().info(
                f"ADRC b0 {gains['b0']}  w_cl {gains['w_cl']}  k_eso {gains['k_eso']}"
            )

        # Sequencer
        self.phase = 'wait_gnss'
        self.t0 = self.t_phase = self._now()
        self.cmd = (Gear.NEUTRAL, CAN_CMD_MAX, 0)
        self.target_v = None
        self.abort_reason = None
        self.finished = False
        self.result = {'test': a.test, 'args': {k: v for k, v in vars(a).items() if k != 'test'}}
        self._slow_since = None
        self._plateau_t = None
        self._reached_t = None
        self._t_last = None
        self.create_timer(1.0 / CONTROL_HZ, self._tick)
        self.get_logger().info(
            f'{a.test}: waiting for /gnss/vel and /gnss/fix (start from standstill)'
        )

    # ====================================================================== #
    #  Inputs                                                                #
    # ====================================================================== #

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_vel(self, msg: TwistStamped):
        now = self._now()
        v = abs(msg.twist.linear.x)
        if self.start_fix is not None and self._v_prev is not None and self.phase in V_INT_PHASES:
            if now - self._v_prev[0] > self.a.gnss_timeout:
                self.gnss_gap = True
            else:
                self.v_int += 0.5 * (v + self._v_prev[1]) * (now - self._v_prev[0])
        self._v_prev = (now, v)
        self.v, self.t_vel = v, now
        self.n_vel += 1
        if self.phase == 'static':
            self.v_rest.append(v)
        self.v_hist.append((now, v))
        while self.v_hist and now - self.v_hist[0][0] > PLATEAU_WIN_S:
            self.v_hist.popleft()

    def _on_heading(self, msg: Float64):
        self.heading, self.t_head = msg.data, self._now()
        if self.start_heading is not None:
            # compass is clockwise, + = left
            self.turn = -wrap_deg(self.heading - self.start_heading)
            self.turn_max = max(self.turn_max, abs(self.turn))

    def _on_can(self, msg: Frame):
        # 0x301, the DBW's ack of 0x101 (same layout): only its steering bytes are used
        if msg.id != CAN_ID_ACK or msg.dlc < DLC_COMMAND:
            return
        self.steer_ack = decode_steering(msg.data[3], msg.data[4])
        self.acks.append((self._now(), self.steer_ack))

    def _set_steer(self, angle, now):
        if angle != self.steer:
            self.steer_events.append((now, angle, self.steer))
            self.get_logger().info(f'steer {self.steer:+.1f} -> {angle:+.1f} deg')
        self.steer = angle

    def _on_fix(self, msg: NavSatFix):
        self.fix, self.t_fix = (msg.latitude, msg.longitude), self._now()
        self.n_fix += 1
        if self.start_fix is not None:
            self.dist = haversine(*self.start_fix, *self.fix)

    # ====================================================================== #
    #  Sequencer — 20 Hz                                                     #
    # ====================================================================== #

    def _goto(self, phase, now):
        self.get_logger().info(
            f'phase {self.phase} -> {phase}  (v {self.v:.2f} m/s, dist {self.dist:.1f} m)'
        )
        self.phase, self.t_phase = phase, now
        self._slow_since = None

    def abort(self, reason):
        if self.phase in ('abort', 'hold', 'done') or self.abort_reason:
            return
        self.abort_reason = reason
        self.result['abort_v'] = round(self.v, 2)
        self.result['abort_dist'] = round(self.dist, 2)
        # parked: already NEUTRAL + full brake, just end the steps
        if self.phase == 'steer_park':
            self.get_logger().error(f'ABORT: {reason} — holding')
            self._goto('hold', self._now())
            return
        self.get_logger().error(f'ABORT: {reason} — braking ({self.a.abort_brake})')
        self._goto('abort', self._now())

    def _stopped(self, now):
        """Return True once the speed has stayed below STOPPED_V for STOPPED_HOLD_S."""
        if self.v >= STOPPED_V:
            self._slow_since = None
            return False
        if self._slow_since is None:
            self._slow_since = now
        return now - self._slow_since >= STOPPED_HOLD_S

    def _stop_dist(self):
        """
        Return the distance from the start at which an abort brake would stop us (conservative).

        The position may be --gnss-timeout old when a GNSS abort fires, the brake bites after
        REACTION_S, then v²/2a.
        """
        return (
            self.dist
            + self.v * (self.a.gnss_timeout + REACTION_S)
            + self.v**2 / (2 * self.a.assumed_decel)
        )

    def _drive_limit(self):
        """Soft limit: why the drive phase must end now (planned brake follows), or None."""
        if self.v >= SOFT_V_FRAC * self.a.max_speed:
            return 'speed limit'
        if self._stop_dist() >= self.a.max_dist - self.a.brake_room:
            return 'distance budget'
        return None

    def _end_drive(self, reason, phase, now):
        self.result.setdefault('ended_by', reason)
        self._goto(phase, now)

    def _check_guards(self, now):
        a = self.a
        if self.t_vel is None or now - self.t_vel > a.gnss_timeout:
            return f'no /gnss/vel for {a.gnss_timeout:g} s'
        if self.t_fix is None or now - self.t_fix > a.gnss_timeout:
            return f'no /gnss/fix for {a.gnss_timeout:g} s'
        if self.v > a.max_speed:
            return f'speed {self.v:.2f} > {a.max_speed} m/s'
        pred = self._stop_dist()
        if pred >= a.max_dist:
            return (
                f'distance {self.dist:.1f} m + stopping {pred - self.dist:.1f} m >= {a.max_dist} m'
            )
        if a.test == 'steer_drive':
            if self.t_head is None or now - self.t_head > a.gnss_timeout:
                return f'no /gnss/heading_deg for {a.gnss_timeout:g} s'
            turn = abs(self.turn) + self._turn_to_come()
            if turn >= a.max_turn:
                return (f'heading change {abs(self.turn):.0f} deg + {self._turn_to_come():.0f} '
                        f'while stopping >= {a.max_turn:g} deg')
        return None

    def _turn_to_come(self):
        """Return the turn still to come if an abort brakes now, wheel held (as _stop_dist)."""
        return turn_after_stop_deg(
            self.v, self.steer, WHEELBASE_M, self.a.gnss_timeout + REACTION_S, self.a.assumed_decel
        )

    def _tick(self):
        now = self._now()
        # sim time reads 0 until /clock arrives: start timing here
        if self._t_last is None:
            self.t_phase = now
        dt = 1.0 / CONTROL_HZ if self._t_last is None else max(now - self._t_last, 1e-3)
        self._t_last = now
        a, t_in = self.a, now - self.t_phase
        moving_phase = self.phase not in (
            'wait_gnss',
            'static',
            'steer_park',
            'abort',
            'hold',
            'done',
        )
        if moving_phase:
            reason = self._check_guards(now)
            if reason:
                self.abort(reason)
                t_in = 0.0
        self.target_v = None
        p = self.phase

        if p == 'wait_gnss':
            self.cmd = (Gear.NEUTRAL, CAN_CMD_MAX, 0)
            fresh = (self.t_vel is not None and self.t_fix is not None
                     and now - self.t_vel < a.gnss_timeout and now - self.t_fix < a.gnss_timeout)
            if self.seq is not None:           # steering passes also need the GNSS heading
                fresh = fresh and self.t_head is not None and now - self.t_head < a.gnss_timeout
            if fresh and self.v > START_MAX_V:
                self._finish(f'not at standstill (/gnss/vel {self.v:.2f} m/s)')
            elif fresh:
                if a.test == 'static':
                    self.n_vel = self.n_fix = 0
                    self._goto('static', now)
                elif a.test == 'steer':
                    self._start_pass(now)
                    self._goto('steer_park', now)
                else:
                    self._goto('arm', now)
            elif t_in > GNSS_WAIT_S:
                self._finish('no GNSS within 10 s (is beacon_node running and authenticated?)')

        elif p == 'static':
            self.cmd = (Gear.NEUTRAL, CAN_CMD_MAX, 0)
            if t_in >= a.secs:
                v = np.array(self.v_rest) if self.v_rest else np.zeros(1)
                self.result.update(
                    vel_hz=round(self.n_vel / t_in, 1),
                    fix_hz=round(self.n_fix / t_in, 1),
                    v_rest_mean=round(float(v.mean()), 3),
                    v_rest_std=round(float(v.std()), 3),
                    v_rest_max=round(float(v.max()), 3),
                )
                self._goto('hold', now)

        elif p == 'arm':
            self.cmd = (Gear.FORWARD, CAN_CMD_MAX, 0)
            if t_in >= ARM_S:
                self._start_pass(now)
                self._goto(
                    {'ramp': 'release', 'step': 'pedal', 'adrc': 'adrc', 'steer_drive': 'runup'}[
                        a.test
                    ],
                    now,
                )

        # parked steering steps: NEUTRAL + full brake throughout
        elif p == 'steer_park':
            self.cmd = (Gear.NEUTRAL, CAN_CMD_MAX, 0)
            self.seq_step, angle = seq_angle(self.seq, t_in)
            self._set_steer(angle, now)
            if self.dist > PARK_MOVE_M:
                self.abort(f'moved {self.dist:.2f} m while parked')
            elif t_in >= seq_duration(self.seq):
                self._goto('hold', now)

        # wheel straight, ADRC to --speed, at least --runup metres
        elif p == 'runup':
            self.target_v = a.speed
            self.cmd = self._adrc(a.speed, dt)
            if self.dist >= a.runup and self.v >= 0.9 * a.speed:
                self.result['pattern_start_v'] = round(self.v, 2)
                self.result['pattern_start_dist'] = round(self.dist, 2)
                self._goto('pattern', now)
            else:
                why = ('time' if t_in >= a.drive_secs
                       else 'no motion' if t_in >= NO_MOTION_S and self.v < STOPPED_V
                       else self._drive_limit())
                if why:
                    self._end_drive(
                        why,
                        'brake' if why in ('speed limit', 'distance budget') else 'adrc_stop',
                        now,
                    )

        elif p == 'pattern':                   # the steering sequence at --speed
            self.target_v = a.speed
            self.cmd = self._adrc(a.speed, dt)
            self.seq_step, angle = seq_angle(self.seq, t_in)
            self._set_steer(angle, now)
            if t_in >= seq_duration(self.seq):
                self._end_drive('steering done', 'adrc_stop', now)
            elif self._drive_limit():
                self._end_drive(self._drive_limit(), 'brake', now)

        # brake off, pedal 0: let the drive engage before the ramp
        elif p == 'release':
            self.cmd = (Gear.FORWARD, 0, 0)
            if self.v >= a.move_speed:         # it creeps at pedal 0
                self.result['breakaway_pedal'] = 0
                self.result['breakaway_t'] = round(t_in, 2)
                self._end_drive('creep', 'brake', now)
            elif t_in >= a.release_secs:
                self._goto('ramp', now)

        elif p == 'ramp':
            pedal = int(min(a.max_pedal, a.rate * t_in))
            self.cmd = (Gear.FORWARD, 0, pedal)
            if self.v >= a.move_speed:
                self.result['breakaway_pedal'] = pedal
                # from the ramp start, after the release
                self.result['breakaway_t'] = round(t_in, 2)
                self._end_drive('moved', 'brake', now)
            elif pedal >= a.max_pedal and t_in >= a.max_pedal / a.rate + 3.0:
                self.result['breakaway_pedal'] = None
                self.get_logger().warn(f'no motion up to pedal {a.max_pedal}')
                self._end_drive('no motion', 'brake', now)
            elif self._drive_limit():
                self._end_drive(self._drive_limit(), 'brake', now)

        elif p == 'pedal':
            self.cmd = (Gear.FORWARD, 0, a.pedal)
            levelled = self._levelled()
            if levelled and self._plateau_t is None:
                self._plateau_t = now
                self.result['plateau_t'] = round(t_in, 2)
                self.get_logger().info(f'speed levelled at {self.v:.2f} m/s')
            why = (
                'levelled'
                if self._plateau_t is not None and now - self._plateau_t >= a.plateau_hold
                else (
                    'time'
                    if t_in >= a.pedal_secs
                    else (
                        'no motion'
                        if t_in >= NO_MOTION_S and self.v < STOPPED_V
                        else self._drive_limit()
                    )
                )
            )
            if why:
                self.result['pedal_end_v'] = round(self.v, 2)
                self.result['pedal_secs'] = round(t_in, 2)
                self.result['levelled'] = self._plateau_t is not None
                self._end_drive(
                    why, 'coast' if a.coast > 0 and why != 'no motion' else 'brake', now
                )

        elif p == 'coast':
            self.cmd = (Gear.FORWARD, 0, 0)
            # downhill: it can speed up
            if (t_in >= a.coast or self.v < STOPPED_V or self.v >= 0.95 * a.max_speed
                    or self._stop_dist() >= a.max_dist - a.brake_room):
                self.result['coast_end_v'] = round(self.v, 2)
                self.result['coast_secs'] = round(t_in, 2)
                self._goto('brake', now)

        elif p == 'brake':
            self.result.setdefault('brake_start_v', round(self.v, 2))
            self.cmd = (Gear.FORWARD, a.brake, 0)
            if self._stopped(now):
                self.result['brake_secs'] = round(t_in - STOPPED_HOLD_S, 2)
                self._goto('hold', now)
            elif t_in >= BRAKE_MAX_S:
                self.result['brake_timeout'] = True
                self.get_logger().warn(
                    f'no confirmed stop after {BRAKE_MAX_S:.0f} s of braking (v {self.v:.2f} m/s)'
                )
                self._goto('hold', now)

        elif p == 'adrc':
            self.target_v = a.speed
            self.cmd = self._adrc(a.speed, dt)
            if self._reached_t is None and self.v >= 0.9 * a.speed:
                self._reached_t = now
                self.result['reach90_t'] = round(t_in, 2)
            why = ('held' if self._reached_t is not None and now - self._reached_t >= a.hold
                   else 'time' if t_in >= a.drive_secs
                   else 'no motion' if t_in >= NO_MOTION_S and self.v < STOPPED_V
                   else self._drive_limit())
            # a soft limit means ADRC misbehaves: open-loop brake, not ADRC
            if why:
                self.result['adrc_end_v'] = round(self.v, 2)
                self._end_drive(
                    why, 'brake' if why in ('speed limit', 'distance budget') else 'adrc_stop', now
                )

        elif p == 'adrc_stop':
            self.target_v = 0.0
            self.cmd = self._adrc(0.0, dt)
            if self._stopped(now) or t_in >= BRAKE_MAX_S:
                self._goto('hold', now)

        elif p == 'abort':
            self.cmd = (Gear.FORWARD, max(a.abort_brake, getattr(a, 'brake', 0) or 0), 0)
            if self._stopped(now) or t_in >= ABORT_MAX_S:
                self._goto('hold', now)

        elif p == 'hold':
            self.cmd = (Gear.NEUTRAL, CAN_CMD_MAX, 0)
            if t_in >= FINAL_HOLD_S:
                self._finish(None)

        if p != 'done':
            self.result['max_v'] = round(max(self.result.get('max_v', 0.0), self.v), 2)
            self._publish(now, p)              # p = the phase that chose this command

    def _start_pass(self, now):
        """Distance, ∫v and heading change count from here."""
        self.start_fix, self.dist, self.v_int = self.fix, 0.0, 0.0
        self.result['start_fix'] = [round(x, 7) for x in self.start_fix]
        self.t_pass0 = now
        if self.heading is not None:
            self.start_heading, self.turn, self.turn_max = self.heading, 0.0, 0.0
            self.result['start_heading'] = round(self.heading, 2)

    def _levelled(self):
        """
        Return True when the speed has levelled off.

        That is a flat fit over the last PLATEAU_WIN_S of this phase, all of it moving.
        """
        win = [(t, v) for t, v in self.v_hist if t >= self.t_phase]
        if len(win) < 5:
            return False
        t, v = np.array(win).T
        if t[-1] - t[0] < 0.8 * PLATEAU_WIN_S or v.min() < PLATEAU_MIN_V:
            return False
        return abs(np.polyfit(t - t[0], v, 1)[0]) < PLATEAU_SLOPE

    def _adrc(self, target, dt):
        u = self.ctrl.compute(self.v, target, dt)
        gear, brake, pedal = self.bridge.map_signals(u, target, self.v)
        return gear, int(brake), int(pedal)

    def _publish(self, now, phase):
        a = self.a
        gear, brake, pedal = self.cmd
        self.can_pub.publish(
            create_command_frame(
                gear,
                brake,
                pedal,
                self.steer,
                state_label=f'long_test:{phase}',
                time_msg=self.get_clock().now().to_msg(),
            )
        )
        self.state_pub.publish(
            String(
                data=json.dumps(
                    {
                        't': round(now, 3),
                        'test': a.test,
                        'phase': phase,
                        'gear': int(gear),
                        'brake': brake,
                        'pedal': pedal,
                        'target_v': self.target_v,
                        'v': round(self.v, 3),
                        'dist': round(self.dist, 2),
                        'stop_dist': round(self._stop_dist(), 2),
                        'max_dist': a.max_dist,
                        'soft_dist': a.max_dist - a.brake_room,
                        'max_speed': a.max_speed,
                        'soft_speed': SOFT_V_FRAC * a.max_speed,
                        'ended_by': self.result.get('ended_by'),
                        'abort': self.abort_reason,
                        'steer': self.steer,
                        'steer_ack': None if self.steer_ack is None else round(self.steer_ack, 2),
                        'step': (
                            None
                            if self.seq is None or self.seq_step is None
                            else [self.seq_step + 1, len(self.seq)]
                        ),
                        'fix': None if self.fix is None else [round(x, 7) for x in self.fix],
                        'start_fix': (
                            None
                            if self.start_fix is None
                            else [round(x, 7) for x in self.start_fix]
                        ),
                        'heading': None if self.heading is None else round(self.heading, 2),
                        'turn': round(self.turn, 1),
                        'turn_to_come': round(self._turn_to_come(), 1),
                        'max_turn': getattr(a, 'max_turn', None),
                    }
                )
            )
        )

    def _finish(self, error):
        if self.finished:
            return
        self.finished = True
        self.phase = 'done'
        self.result['error'] = error
        self.result['abort'] = self.abort_reason
        self.result['end_dist'] = round(self.dist, 2)
        # speed scale: ∫ /gnss/vel dt vs straight-line distance
        if self.dist > 5.0 and not self.gnss_gap:
            self.result['vel_over_pos'] = round(self.v_int / self.dist, 3)
        if self.ctrl is not None:
            self.result['adrc'] = self.gains
        if self.seq is not None:
            t0 = getattr(self, 't_pass0', None) or (
                self.steer_events[0][0] if self.steer_events else 0.0
            )
            steps = ack_steps(self.steer_events, self.acks)
            for st in steps:
                st['t'] = round(st['t'] - t0, 2)
            self.result.update(
                steer_events=[[round(t - t0, 2), new] for t, new, _ in self.steer_events],
                acks_received=len(self.acks),
                ack=ack_summary(steps),
                ack_steps=steps,
                turn_end_deg=round(self.turn, 1),
                turn_max_deg=round(self.turn_max, 1),
            )
        log = self.get_logger().error if (error or self.abort_reason) else self.get_logger().info
        log('RESULT ' + json.dumps(self.result))

    def send_hold(self):
        """One NEUTRAL + full-brake frame (used on the way out)."""
        self.can_pub.publish(
            create_command_frame(
                Gear.NEUTRAL,
                CAN_CMD_MAX,
                0,
                0,
                state_label='long_test:exit',
                time_msg=self.get_clock().now().to_msg(),
            )
        )


def main(args=None):
    argv = remove_ros_args(sys.argv if args is None else args)[1:]
    a = parse_args(argv)
    # its own node and Ctrl-C handling; tkinter only imported here
    if a.test == 'tune':
        from .longitudinal_tuner import run_tuner
        run_tuner(*adrc_gains(a), args=args)
        return
    if a.test == 'monitor':
        from .longitudinal_monitor import run_monitor
        run_monitor(pad=a.pad, args=args)
        return
    # Own the signals: the first Ctrl-C must brake the shuttle, not tear the node down mid-pass.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LongitudinalTest(a)
    hits = [0]

    def on_signal(signum, _frame):
        hits[0] += 1
        if hits[0] == 1 and node.phase not in ('wait_gnss', 'static', 'hold', 'done'):
            node.abort('Ctrl-C')
        elif hits[0] == 1:
            node._finish('interrupted')
        else:
            node.finished = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.send_hold()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
