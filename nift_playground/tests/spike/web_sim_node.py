"""Spike stand-in for kinematic_sim_node on fakeros: same VirtualEcu -> ShuttlePlant -> KinematicBicycle tick.

Differences from kinematic_sim_node (spike shortcuts, all listed in report.md):
  - no thread / lockstep: fakeros' world fires this timer first at each tick time, so the controller's frame
    computed at t is applied on the next tick (what lockstep gives on ROS);
  - no Octane emulator / beacon_node / shuttle_tf_node: the true pose goes straight out as TF map->base_link and
    the speed as /gnss/vel (the real chain is a later phase);
  - gnss_off_after_s: stop TF + /gnss/vel from then on (GNSS loss), for the watchdog check.
"""
import math

from rclpy.node import Node
from can_msgs.msg import Frame
from geometry_msgs.msg import TransformStamped, TwistStamped

from nift_carla.carla_dbw import CAN_FULL_SCALE, EcuConfig, EcuMode, VirtualEcu
from nift_carla.kinematic_model import STEER_LIMIT_DEG, BicycleConfig, KinematicBicycle
from nift_carla.shuttle_plant import PEDAL_MAP_2026_10_08, PlantConfig, ShuttlePlant
from nift_common.vehicle import ShuttleConfig
from tf2_ros import TransformBroadcaster

SHUTTLE = ShuttleConfig()


class WebSimNode(Node):
    def __init__(self, spawn=(0.0, 0.0, 0.0), gnss_off_after_s=None):
        super().__init__('kinematic_sim')
        p = self.declare_parameter
        p('fixed_delta_seconds', 0.05)
        p('dbw.cmd_timeout_s', 0.5)
        _pc = PlantConfig()
        p('plant.pedal_map', [float(x) for pt in PEDAL_MAP_2026_10_08 for x in pt])
        for k in ('max_speed', 'a_lim', 'tau', 't_eng', 'cmd_delay', 'brake_a0', 'brake_k_v', 'brake_a_min',
                  'brake_full'):
            p(f'plant.{k}', getattr(_pc, k))
        p('steer.tau_s', 0.0)
        p('steer.rate_deg_s', 0.0)
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.dt = float(g('fixed_delta_seconds'))
        flat = [float(x) for x in g('plant.pedal_map')]
        self.plant = ShuttlePlant(PlantConfig(
            pedal_map=tuple(zip(flat[0::2], flat[1::2])), max_speed=float(g('plant.max_speed')),
            a_lim=float(g('plant.a_lim')), tau=float(g('plant.tau')), t_eng=float(g('plant.t_eng')),
            cmd_delay=float(g('plant.cmd_delay')), brake_a0=float(g('plant.brake_a0')),
            brake_k_v=float(g('plant.brake_k_v')), brake_a_min=float(g('plant.brake_a_min')),
            brake_full=float(g('plant.brake_full'))))
        self.car = KinematicBicycle(BicycleConfig(wheelbase=SHUTTLE.wheel_base, max_steer_deg=STEER_LIMIT_DEG,
                                                  steer_tau_s=float(g('steer.tau_s')),
                                                  steer_rate_deg_s=float(g('steer.rate_deg_s'))))
        self.car.reset(*spawn)
        self.ecu = VirtualEcu(EcuConfig(cmd_timeout_s=float(g('dbw.cmd_timeout_s')),
                                        wheel_max_steer_deg=STEER_LIMIT_DEG, steer_sign=1.0, map_wheelbase=0.0))
        self.gnss_off_ns = None if gnss_off_after_s is None else self.get_clock().now().nanoseconds + int(
            gnss_off_after_s * 1e9)
        self.applied = (0.0, 0.0, False)

        self.create_subscription(Frame, '/to_can_bus', self._can_callback, 10)
        self.pub_vel = self.create_publisher(TwistStamped, '/gnss/vel', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(self.dt, self._tick)

    def _can_callback(self, frame):
        self.ecu.on_frame(frame, self.get_clock().now().nanoseconds / 1e9)

    def _tick(self):
        """The command in force at t - dt moves the shuttle for dt; publish the state at t."""
        now = self.get_clock().now()
        t0 = now.nanoseconds / 1e9 - self.dt
        out = self.ecu.control(t0)
        hold = self.ecu.mode(t0) != EcuMode.ACTIVE
        if hold:
            pedal, brake, neutral, reverse = 0.0, CAN_FULL_SCALE, True, False
        else:
            pedal, brake = out.throttle * CAN_FULL_SCALE, out.brake * CAN_FULL_SCALE
            neutral, reverse = out.neutral, out.reverse
        v = self.plant.step(t0, self.dt, self.car.v, pedal=pedal, brake=brake, neutral=neutral, reverse=reverse,
                            active=True)
        self.car.step(self.dt, v, out.steer * STEER_LIMIT_DEG)
        self.applied = (pedal, brake, hold)

        if self.gnss_off_ns is not None and now.nanoseconds >= self.gnss_off_ns:
            return
        x, y, yaw = self.car.base_link()
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.tf.sendTransform(t)
        vel = TwistStamped()
        vel.header.stamp, vel.header.frame_id = now.to_msg(), 'base_link'
        vel.twist.linear.x = abs(self.car.v)
        self.pub_vel.publish(vel)
