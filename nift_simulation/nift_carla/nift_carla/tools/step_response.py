#!/usr/bin/env python3
"""
Measure open-loop pedal and brake step responses of the simulated shuttle.

File: step_response.py
Author: Kane Weng

Description:
  Open-loop pedal / brake step responses of the simulated shuttle, measured
  through the same path the controller uses: CAN 0x101 frames on /to_can_bus
  -> bridge virtual ECU -> the bridge plant (dbw.plant).  Used to shape the bridge's
  powertrain (vehicle.* in carla_params.yaml) so that full pedal from rest and
  full brake match ShuttleConfig.max_accel / max_decel.

  Needs a running bridge and nothing else driving the ego (no trajectory_controller):
    ros2 launch nift_carla carla_bridge.launch.py carla_host:=<ip>

  Each step starts from rest at --reset-pose (teleport via /initialpose), so every
  step sees the same road.  Frames are sent at 20 Hz on sim time and stamped with
  it, like trajectory_controller's (works with the bridge's lockstep).
    pedal step  FORWARD, pedal p for --step-s (or until --v-cap)
    brake step  full pedal up to --brake-from m/s, then FORWARD, brake b until stopped;
                decel = slope between v0 - 0.3 and 0.5 m/s (interpolated crossings)
  Speed is ground truth from /ackermann_like_controller/odom.  The default pose
  (lanelet 46138 start) is ~1.4 deg downhill: raw numbers include about +0.24 m/s^2
  on acceleration and -0.24 m/s^2 on deceleration.

Usage:
  ros2 run nift_carla carla_step_response
  ros2 run nift_carla carla_step_response --pedal 255 --brake 255 --csv /tmp/steps.csv
"""

import argparse
import csv
import json
import math

from can_msgs.msg import Frame
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nift_common.can_protocol import CAN_CMD_MAX, Gear
from nift_common.vehicle import ShuttleConfig
from nift_control.dbw_interface.can_interface import create_command_frame
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

_CFG = ShuttleConfig()
MAX_ACCEL, MAX_DECEL, MAX_SPEED = _CFG.max_accel, _CFG.max_decel, _CFG.max_speed


class StepResponse(Node):

    def __init__(self):
        super().__init__(
            'carla_step_response',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.cmd = (Gear.NEUTRAL, CAN_CMD_MAX, 0)
        self.v = 0.0
        self.samples = []       # (odom stamp, v): one per bridge tick
        self.ecu_mode = None
        self.pub = self.create_publisher(Frame, '/to_can_bus', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(
            Odometry, '/ackermann_like_controller/odom', self._odom_callback, 10
        )
        self.create_subscription(String, '/carla/dbw_state', self._dbw_callback, 10)
        self.create_timer(0.05, self._send)

    def _odom_callback(self, msg):
        self.v = msg.twist.twist.linear.x
        self.samples.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, self.v))

    def _dbw_callback(self, msg):
        # Mode: no_command/active/timeout/estop
        self.ecu_mode = json.loads(msg.data).get('mode')

    def _send(self):
        """Build and publish a frame from self.cmd, stamped with the current sim time."""
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        gear, brake, pedal = self.cmd
        self.pub.publish(
            create_command_frame(gear, brake, pedal, 0.0, 'step_response', time_msg=now.to_msg())
        )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def run_for(self, sim_s, until=None):
        """
        Spin for sim_s seconds of sim time (or until until(v)); return [(t, v)].

        From the start, one sample per tick timed by the odom stamp (the node clock lags odom
        by a tick: /clock is published after the sensors).
        """
        while self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        t0, i0 = self.now(), len(self.samples)
        while self.now() - t0 < sim_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until(self.v):
                break
        return [(t - t0, v) for t, v in self.samples[i0:]]

    def rest(self, pose):
        """
        Reset to a known state.

        Run before every test to have each test start stopped and in the same place.
        """
        self.cmd = (Gear.NEUTRAL, CAN_CMD_MAX, 0)
        if pose is not None:
            x, y, yaw = pose
            m = PoseWithCovarianceStamped()
            m.header.frame_id = 'map'
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(
                yaw / 2
            )
            self.pose_pub.publish(m)

        # let the teleport happen and the car settle onto the road
        self.run_for(1.5)

        # Wait up to 10 s more for a real stop
        self.run_for(10.0, until=lambda v: abs(v) < 0.02)

        if self.ecu_mode != 'active':
            self.get_logger().warn(
                f'virtual ECU is "{self.ecu_mode}", not active: is something else latched/sending?'
            )


def _at(trace, t):
    """Speed at time t (linear interpolation), None if the trace is shorter."""
    for (t0, v0), (t1, v1) in zip(trace, trace[1:]):
        if t0 <= t <= t1:
            return v0 + (v1 - v0) * (t - t0) / (t1 - t0) if t1 > t0 else v1
    return None


def _t_cross(trace, v_target):
    """First time the (falling) speed crosses v_target, interpolated between samples."""
    for (t0, v0), (t1, v1) in zip(trace, trace[1:]):
        if v0 >= v_target > v1:
            return t0 + (t1 - t0) * (v0 - v_target) / (v0 - v1)
    return None


def pedal_step(node, pedal, step_s, v_cap, pose):
    node.rest(pose)
    node.cmd = (Gear.FORWARD, 0, pedal)
    trace = node.run_for(step_s, until=lambda v: v >= v_cap)
    v2 = _at(trace, 2.0)
    t1 = next((t for t, v in trace if v >= 1.0), None)      # first time to v=1 m/s
    t_end, v_end = trace[-1]
    return trace, {
        'a_0_2s': None if v2 is None else v2 / 2.0,         # launch accel
        'a_mean': v_end / t_end if t_end > 0 else None,     # avg accel
        't_to_1ms': t1,
        'v_end': v_end,
        't_end': t_end,
    }


def brake_step(node, brake, v_from, pose):
    node.rest(pose)
    # Full pedal until v >= v_from
    node.cmd = (Gear.FORWARD, 0, CAN_CMD_MAX)
    node.run_for(30.0, until=lambda v: v >= v_from)

    # Guard: if below 90% of the target, return empty result
    v0 = node.v
    if v0 < 0.9 * v_from:     # e.g. the bridge reconnected and respawned the ego mid-step
        return [], {'v0': v0, 'decel': None, 't_stop': 0.0, 'dist': 0.0}

    # Switch to break
    node.cmd = (Gear.FORWARD, brake, 0)
    trace = node.run_for(15.0, until=lambda v: v < 0.1)
    t_end, v_end = trace[-1]
    dist = sum(0.5 * (va + vb) * (tb - ta) for (ta, va), (tb, vb) in zip(trace, trace[1:]))
    # A 4 m/s stop lasts ~10 ticks, so v0 / t_stop swings ~15 % with one tick of timing.  Take the
    # slope between interpolated crossings instead, clear of the onset and the final lock-up.
    t_hi, t_lo = _t_cross(trace, v0 - 0.3), _t_cross(trace, 0.5)
    decel = (
        (v0 - 0.3 - 0.5) / (t_lo - t_hi)
        if t_hi is not None and t_lo is not None and t_lo > t_hi
        else None
    )
    return trace, {
        'v0': v0,
        'decel': decel,
        't_stop': t_end,
        'dist': dist,
    }


def _f(x, fmt):
    return '-' if x is None else format(x, fmt)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument('--pedal', type=int, nargs='*', default=[64, 128, 192, CAN_CMD_MAX],
                    help=f'pedal steps (0-{CAN_CMD_MAX})')
    ap.add_argument('--brake', type=int, nargs='*', default=[64, 128, CAN_CMD_MAX],
                    help=f'brake steps (0-{CAN_CMD_MAX})')
    ap.add_argument('--step-s', type=float, default=6.0, help='pedal step length, sim s')
    ap.add_argument(
        '--v-cap', type=float, default=6.0, help='end a pedal step early at this speed, m/s'
    )
    ap.add_argument('--brake-from', type=float, default=4.0, help='speed to brake from, m/s')
    ap.add_argument(
        '--reset-pose',
        type=float,
        nargs=3,
        default=[101.52, 162.66, -1.612],
        metavar=('X', 'Y', 'YAW'),
        help='map-frame start pose for every step (default: carla_params.yaml spawn)',
    )
    ap.add_argument('--no-reset', action='store_true', help='do not teleport between steps')
    ap.add_argument('--csv', help='write every v(t) trace here')
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = StepResponse()
    pose = None if args.no_reset else tuple(args.reset_pose)
    rows = []
    try:
        node.rest(pose)
        print(
            f'targets (ShuttleConfig): full pedal {MAX_ACCEL} m/s^2, full brake {MAX_DECEL} '
            'm/s^2, '
            f'max speed {MAX_SPEED:.2f} m/s',
            flush=True,
        )
        print(
            f'{"step":>11s}  {"a 0-2 s":>8s}  {"a mean":>7s}  {"t to 1 m/s":>10s}  {"v end":>11s} '
            ' '
            f'{"decel":>7s}  {"stop in":>13s}',
            flush=True,
        )
        for p in args.pedal:
            trace, m = pedal_step(node, p, args.step_s, args.v_cap, pose)
            rows.append((f'pedal {p}', trace))
            print(
                f'{"pedal " + str(p):>11s}  {_f(m["a_0_2s"], "8.2f")}  {_f(m["a_mean"], "7.2f")}  '
                f'{_f(m["t_to_1ms"], "8.2f")} s  {m["v_end"]:5.2f} @{m["t_end"]:4.1f}s',
                flush=True,
            )
        for b in args.brake:
            trace, m = brake_step(node, b, args.brake_from, pose)
            rows.append((f'brake {b}', trace))
            print(f'{"brake " + str(b):>11s}  {"":8s}  {"":7s}  {"":10s}  from {m["v0"]:4.2f}  '
                  f'{_f(m["decel"], "7.2f")}  {m["t_stop"]:4.2f} s {m["dist"]:4.2f} m', flush=True)
        node.rest(None)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C race: rclpy's SIGINT handler can shut the context down mid-construction or
        # mid-spin (RCLError). With the context still alive it is a real error.
        if rclpy.ok():
            raise
    finally:
        if args.csv and rows:
            with open(args.csv, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['step', 't_s', 'v_mps'])
                for name, trace in rows:
                    w.writerows((name, round(t, 3), round(v, 4)) for t, v in trace)
            print(f'traces written to {args.csv}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
