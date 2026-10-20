#!/usr/bin/env python3
"""
Play the shuttle without CARLA or a GPU: virtual ECU, plant, bicycle model and GNSS.

File: kinematic_sim_node.py
Author: Kane Weng

Description:
  The kinematic sim backend: plays the shuttle for the NiFT stack on any
  machine, without CARLA or a GPU. It serves the CARLA bridge's contract
  (carla_bridge_node.py) minus the LiDAR, so the stack runs the same code:

    OUT  Octane beacon_update (socket.io)
              127.0.0.1:<octane.port>/octane, front / rear antenna
              (sim_gnss.py -> octane_emulator.py; beacon_node runs its real branch)
         /ackermann_like_controller/odom  nav_msgs/Odometry (ground truth, frame `map`)
         /clock                           rosgraph_msgs/Clock
         /carla/gps_health                std_msgs/Bool
              (false while a simulated GPS dropout is active)
         /carla/gnss_fault                std_msgs/String
              (JSON: a scheduled GNSS fault starts / ends)
         /carla/dbw_state                 std_msgs/String
              (virtual ECU + plant state as JSON, every tick)
         /diagnostics                     (hardware id `kinematic_sim`)
    IN   /to_can_bus                      can_msgs/Frame (0x101 command, 0x100 E-stop)
         /initialpose                     geometry_msgs/PoseWithCovarianceStamped
              (teleport, map frame)
    SRV  /carla/dbw_power_cycle           std_srvs/Trigger (key off/on: clears the E-stop latch)
    No /scan: lidar_node has nothing to read, so perception (and the safety filter) cannot
    run on this backend.
    No TF, as the bridge: robot_state_publisher and shuttle_tf_node provide it.

  One tick (fixed_delta_seconds of sim time, 20 Hz), in the bridge's order:
    lockstep wait -> virtual ECU (carla_dbw.py) -> shuttle plant (shuttle_plant.py)
    -> bicycle model (kinematic_model.py) -> GNSS -> odom -> /clock -> teleports
    -> pacing (time_factor)

    steering  the ECU runs without a steering map and with the CAN sign, so steer x full
              lock is the commanded front-wheel angle: the model has the shuttle's own
              geometry.
    speed     the plant's speed, fed back as its own measurement (nothing else moves the
              vehicle).
    hold      when the ECU holds (no command yet, watchdog, E-stop), the plant gets a
              full-brake command: the shuttle's measured 0.45 s dead time and braking.
              CARLA's physics hold stops about 10x harder; what the real DBW does is
              unmeasured.
    world     flat: z = 0, the antennas at datum_alt_m + base_link_height + antenna_z.
    time      sim time starts at 1 s (no stamp is 0, which many ROS APIs read as "unset")
              and moves in whole fixed_delta_seconds steps.

  Parameters: the bridge's, with the same names and defaults, for everything both backends share
  (kinematic_sim.launch.py hands this node carla_params.yaml's carla_bridge section, so the plant,
  Octane and GNSS settings live in one file; the node ignores what it does not declare), plus
  steer.tau_s / steer.rate_deg_s (kinematic_model.py; 0 = instant, unmeasured on the shuttle).

Usage:
  ros2 launch nift_carla kinematic_sim.launch.py
  ros2 run nift_carla kinematic_sim_node --ros-args --params-file <params.yaml>
"""

import json
import math
import queue
import signal
import sys
import threading
import time
from typing import Tuple

from can_msgs.msg import Frame
import diagnostic_msgs.msg
import diagnostic_updater
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nift_carla.carla_dbw import CAN_FULL_SCALE, EcuConfig, EcuMode, VirtualEcu
from nift_carla.carla_geo import CarlaGeo
from nift_carla.kinematic_model import BicycleConfig, KinematicBicycle, STEER_LIMIT_DEG
from nift_carla.octane_emulator import OctaneEmulator
from nift_carla.shuttle_plant import PEDAL_MAP_2026_10_08, PlantConfig, ShuttlePlant
from nift_carla.sim_gnss import (
    dead_zones_from_flat,
    faults_from_param,
    feed_from_params,
    feed_param_defaults,
    GnssSim,
)
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
from nift_common.vehicle import ShuttleConfig
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger

SHUTTLE = ShuttleConfig()

START_NS = 1_000_000_000        # sim time of the first tick


def _trace(msg: str):
    """Shutdown tracing that does not depend on the rclpy logger being alive."""
    print(f'[kinematic_sim] {msg}', file=sys.stderr, flush=True)


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _sim_stamp(ns: int):
    return rclpy.time.Time(nanoseconds=ns).to_msg()


class KinematicSimNode(Node):

    def __init__(self, **kwargs):
        super().__init__('kinematic_sim', **kwargs)     # kwargs: e.g. parameter_overrides (tests)

        # ------------------------------------------------------------ #
        #  Parameters (names and defaults as carla_bridge_node.py)     #
        # ------------------------------------------------------------ #
        p = self.declare_parameter
        p('fixed_delta_seconds', 0.05)
        p('time_factor', 1.0)           # 1 = real time, 0 = as fast as possible
        p('publish_clock', True)
        p('map_origin_lat', MAP_ORIGIN_LAT)
        p('map_origin_lon', MAP_ORIGIN_LON)
        p('datum_alt_m', 0.0)
        p('spawn.mode', 'map')          # map | geo (carla_index needs CARLA)
        p('spawn.x', 0.0)
        p('spawn.y', 0.0)
        p('spawn.yaw', 0.0)             # rad, map frame
        p('spawn.lat', 0.0)
        p('spawn.lon', 0.0)
        p('spawn.heading_deg', 0.0)     # compass, 0 = north, CW
        p('dbw.cmd_timeout_s', 0.5)     # no 0x101 for this long -> hold
        _pc = PlantConfig()
        # count, m/s, count, m/s, ...
        p('plant.pedal_map', [float(x) for pt in PEDAL_MAP_2026_10_08 for x in pt])
        p('plant.max_speed', _pc.max_speed)
        p('plant.a_lim', _pc.a_lim)
        p('plant.tau', _pc.tau)
        p('plant.t_eng', _pc.t_eng)
        p('plant.cmd_delay', _pc.cmd_delay)
        p('plant.brake_a0', _pc.brake_a0)
        p('plant.brake_k_v', _pc.brake_k_v)
        p('plant.brake_a_min', _pc.brake_a_min)
        p('plant.brake_full', _pc.brake_full)
        # wheel lag behind the command (kinematic_model.py); 0 = none
        p('steer.tau_s', 0.0)
        p('steer.rate_deg_s', 0.0)      # wheel rate limit; 0 = none
        # before each tick, wait for the frame computed from the last /clock
        p('lockstep', False)
        # wall-clock cap on that wait (only while the ECU is ACTIVE)
        p('lockstep_timeout_s', 0.1)
        p('antenna_offset_x', SHUTTLE.gnss_front_x)     # front +x, rear -x
        p('antenna_z', SHUTTLE.gnss_z)
        p('base_link_height', SHUTTLE.base_link_height)
        p('gps.rate_divider', 1)
        p('gps.noise_stddev_m', 0.0)
        p('gps.noise_heading', False)
        p('gps.dropout_period_s', 0.0)
        p('gps.dropout_duration_s', 0.0)
        p('gps.seed', 0)
        # real Octane feed model (sim_gnss.py)
        for name, default in feed_param_defaults().items():
            p(name, default)
        # [] has no ROS type of its own
        p('gps.dead_zones', [], ParameterDescriptor(dynamic_typing=True))
        # scheduled GNSS faults: a YAML list (launch arg gnss_faults, sim_gnss.py)
        p('gps.faults', '')
        p('gps.faults_name', '')        # the scenario's name, echoed on /carla/gnss_fault
        p('octane.port', 8765)
        p('octane.api_key', 'sim')
        p('octane.namespace', '/octane')
        p('octane.front_id', 'SIM-FRONT')
        p('octane.rear_id', 'SIM-REAR')
        p('can_topic', '/to_can_bus')
        p('odom_topic', '/ackermann_like_controller/odom')
        p('odom_frame', 'map')
        p('base_frame', 'base_link')

        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.dt = float(g('fixed_delta_seconds'))
        self._dt_ns = round(self.dt * 1e9)
        self.time_factor = float(g('time_factor'))
        self.publish_clock = bool(g('publish_clock'))
        self.lockstep = bool(g('lockstep'))
        self.lockstep_timeout = float(g('lockstep_timeout_s'))
        self.datum_alt = float(g('datum_alt_m'))
        self.antenna_elevation = (
            self.datum_alt + float(g('base_link_height')) + float(g('antenna_z'))
        )
        self.gps_divider = max(1, int(g('gps.rate_divider')))
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.geo = CarlaGeo(float(g('map_origin_lat')), float(g('map_origin_lon')))

        # the ECU's steer output x STEER_LIMIT_DEG = the commanded front-wheel angle, CAN sign (+ =
        # left)
        self._ecu_cfg = EcuConfig(
            cmd_timeout_s=float(g('dbw.cmd_timeout_s')),
            wheel_max_steer_deg=STEER_LIMIT_DEG,
            steer_sign=1.0,
            map_wheelbase=0.0,
        )
        flat = [float(x) for x in g('plant.pedal_map')]
        if len(flat) < 4 or len(flat) % 2:
            raise ValueError(
                'plant.pedal_map must be [count, m/s, count, m/s, ...] with >= 2 points'
            )
        pedal_map = tuple(zip(flat[0::2], flat[1::2]))
        if any(b[0] <= a[0] for a, b in zip(pedal_map, pedal_map[1:])):
            raise ValueError('plant.pedal_map counts must increase')
        self.plant = ShuttlePlant(
            PlantConfig(
                pedal_map=pedal_map,
                max_speed=float(g('plant.max_speed')),
                a_lim=float(g('plant.a_lim')),
                tau=float(g('plant.tau')),
                t_eng=float(g('plant.t_eng')),
                cmd_delay=float(g('plant.cmd_delay')),
                brake_a0=float(g('plant.brake_a0')),
                brake_k_v=float(g('plant.brake_k_v')),
                brake_a_min=float(g('plant.brake_a_min')),
                brake_full=float(g('plant.brake_full')),
            )
        )
        self.car = KinematicBicycle(
            BicycleConfig(
                wheelbase=SHUTTLE.wheel_base,
                max_steer_deg=STEER_LIMIT_DEG,
                steer_tau_s=float(g('steer.tau_s')),
                steer_rate_deg_s=float(g('steer.rate_deg_s')),
            )
        )
        self.car.reset(*self._spawn_pose())
        self.gnss = GnssSim(
            float(g('antenna_offset_x')),
            (g('octane.front_id'), g('octane.rear_id')),
            noise_stddev_m=float(g('gps.noise_stddev_m')),
            noise_heading=bool(g('gps.noise_heading')),
            dropout_period_s=float(g('gps.dropout_period_s')),
            dropout_duration_s=float(g('gps.dropout_duration_s')),
            seed=int(g('gps.seed')),
            feed=feed_from_params(g),
            dead_zones=dead_zones_from_flat(g('gps.dead_zones') or []),
            faults=faults_from_param(g('gps.faults')),
        )
        self.gnss_faults_name = g('gps.faults_name')
        self.get_logger().info(f'GNSS: {self.gnss.summary()}')

        # ------------------------------------------------------------ #
        #  ROS interface                                               #
        # ------------------------------------------------------------ #
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_odom = self.create_publisher(Odometry, g('odom_topic'), 10)
        self.pub_gps_health = self.create_publisher(Bool, '/carla/gps_health', 10)
        self.pub_gnss_fault = self.create_publisher(String, '/carla/gnss_fault', 10)
        self.pub_dbw_state = self.create_publisher(String, '/carla/dbw_state', 10)
        self.create_subscription(Frame, g('can_topic'), self._can_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._initialpose_callback, 10
        )
        self.create_service(Trigger, '/carla/dbw_power_cycle', self._power_cycle_callback)
        self.diag = diagnostic_updater.Updater(self)
        self.diag.setHardwareID('kinematic_sim')
        self.diag.add('Kinematic sim', self._diagnostics_callback)

        # GNSS goes out the way Octane delivers it (raises if the port is taken)
        self.octane = OctaneEmulator(port=int(g('octane.port')), api_key=g('octane.api_key'),
                                     namespace=g('octane.namespace'), log=self.get_logger())
        self.octane.start()

        # ------------------------------------------------------------ #
        #  Shared state                                                #
        # ------------------------------------------------------------ #
        self._lock = threading.Lock()
        # lockstep: CAN callback wakes the tick thread
        self._cond = threading.Condition(self._lock)
        self.ecu = VirtualEcu(self._ecu_cfg)
        self._dbw_status = {}
        self._cmd_stamp_ns = -1     # header stamp of the latest valid 0x101
        self._clock_ns = -1         # stamp of the latest /clock published
        self._lockstep_waits = 0
        self._lockstep_timeouts = 0
        self._lockstep_wait_ms = 0.0    # mean
        self._teleport_q: 'queue.Queue[Tuple[float, float, float]]' = queue.Queue()
        self._tick_count = 0
        self._sim_ns = START_NS - self._dt_ns   # the first tick lands on START_NS
        self._sim_t = self._sim_ns / 1e9
        self._tick_ms = 0.0
        self._rtf = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._run, name='kinematic_tick', daemon=True)

        x, y, yaw = self.car.base_link()
        b, c = self.car.cfg, self.plant.cfg
        steering = (f'lag {b.steer_tau_s} s, rate {b.steer_rate_deg_s} deg/s'
                    if b.steer_tau_s or b.steer_rate_deg_s else 'instant')
        self.get_logger().info(
            f'Kinematic sim (no CARLA, no LiDAR): spawn map ({x:.2f}, {y:.2f}, yaw {yaw:.2f}), '
            'wheelbase '
            f'{b.wheelbase:.3f} m, full lock {STEER_LIMIT_DEG:.0f} deg = '
            f'{b.wheelbase / math.tan(math.radians(STEER_LIMIT_DEG)):.2f} m radius, steering '
            f'{steering}, '
            f'dt {self.dt} s, time factor {self.time_factor}, lockstep '
            f'{"on" if self.lockstep else "off"}'
        )
        self.get_logger().info(
            f'Shuttle plant: pedal map {[(int(p), v) for p, v in c.pedal_map]} m/s, motor loop '
            f'a_lim {c.a_lim} m/s^2 '
            f'tau {c.tau} s, engagement {c.t_eng} s, dead time {c.cmd_delay} s, brake '
            f'{c.brake_a0} {c.brake_k_v:+} v '
            f'(>= {c.brake_a_min}, full at {c.brake_full:.0f}), governor {c.max_speed} m/s; ECU '
            'holds = full brake'
        )

    def _spawn_pose(self) -> Tuple[float, float, float]:
        """
        Return the base_link spawn pose in the map frame from spawn.*.

        map: x, y, yaw; geo: lat, lon, compass heading.
        """
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        mode = g('spawn.mode')
        if mode == 'map':
            return float(g('spawn.x')), float(g('spawn.y')), float(g('spawn.yaw'))
        # as the bridge: 90 - heading turns a compass heading into a ROS yaw
        if mode == 'geo':
            x, y = self.geo.wgs_to_map(float(g('spawn.lat')), float(g('spawn.lon')))
            return x, y, math.radians(90.0 - float(g('spawn.heading_deg')))
        raise ValueError(
            f'spawn.mode "{mode}": the kinematic sim spawns by map or geo (carla_index needs '
            'CARLA)'
        )

    # ================================================================== #
    #  ROS callbacks (executor thread)                                   #
    # ================================================================== #
    def _can_callback(self, frame: Frame):
        with self._cond:
            n = self.ecu.counters['frames']
            self.ecu.on_frame(frame, self._sim_t)
            if self.ecu.counters['frames'] > n:     # a valid 0x101
                self._cmd_stamp_ns = (
                    frame.header.stamp.sec * 1_000_000_000 + frame.header.stamp.nanosec
                )
                self._cond.notify()

    def _power_cycle_callback(self, _request, response):
        with self._lock:
            self.ecu.power_cycle()
        self.get_logger().warn('Virtual ECU power-cycled: E-stop latch and last command cleared.')
        response.success = True
        response.message = 'virtual ECU power-cycled'
        return response

    def _initialpose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        self._teleport_q.put((pose.position.x, pose.position.y, _yaw_from_quat(pose.orientation)))

    def _diagnostics_callback(self, stat):
        dbw = self._dbw_status
        stat.add('Backend', 'kinematic (no CARLA, no LiDAR)')
        stat.add('Sim time', f'{self._sim_t:.2f} s')
        stat.add('Tick wall', f'{self._tick_ms:.2f} ms')
        stat.add('Real-time factor', f'{self._rtf:.2f}')
        stat.add('ECU mode', dbw.get('mode', '-'))
        stat.add(
            'CAN cmd age', 'none' if dbw.get('cmd_age') is None else f'{dbw["cmd_age"]:.2f} s'
        )
        stat.add(
            'CAN frames',
            f'{dbw.get("frames", 0)} cmd, {dbw.get("estop_frames", 0)} E-stop, '
            f'{dbw.get("bad_frames", 0)} bad, {dbw.get("timeouts", 0)} timeouts',
        )
        if self.lockstep:
            stat.add(
                'Lockstep',
                f'{self._lockstep_waits} waits, mean {self._lockstep_wait_ms:.1f} ms, '
                f'{self._lockstep_timeouts} timeouts',
            )
        stat.add('Wheelbase', f'{self.car.cfg.wheelbase:.3f} m')
        stat.add('GPS health', 'OK' if self.gnss.ok else 'DROPOUT')
        stat.add('Octane emulator',
                 f'port {self.octane.port}, {self.octane.clients_authed} client(s) authenticated, '
                 f'{self.octane.emitted} beacon_update sent')
        if dbw.get('mode') == EcuMode.ESTOP.value:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                         'E-stop latched — call /carla/dbw_power_cycle to release')
        elif dbw.get('mode') == EcuMode.ACTIVE.value:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Running — driving (CAN)')
        else:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.OK, 'Running — no CAN command, holding brake'
            )
        return stat

    # ================================================================== #
    #  Tick thread                                                       #
    # ================================================================== #
    def start(self):
        self._thread.start()

    def stop(self):
        _trace('stop(): stopping tick thread')
        self._running = False
        with self._cond:
            self._cond.notify_all()
        self._thread.join(timeout=2.0)
        try:
            self.octane.stop()
        except Exception as ex:
            _trace(f'stop(): Octane emulator did not stop cleanly ({ex!r})')
        _trace('stop(): done')

    @property
    def alive(self) -> bool:
        return self._thread.is_alive()

    def _run(self):
        self.get_logger().info('Kinematic sim running.')
        try:
            while self._running:
                self._tick_once()
        except Exception as ex:     # programming error: log and stop the loop (main() then exits)
            self.get_logger().fatal(f'Kinematic sim crashed: {ex!r}')
            self._running = False

    def _tick_once(self):
        """
        Run one simulation step.

        The command at the current sim time moves the shuttle for dt, then time advances.
        """
        t0 = time.perf_counter()
        if self.lockstep:
            self._lockstep_wait()

        self._apply_dbw()

        self._sim_ns += self._dt_ns
        self._sim_t = self._sim_ns / 1e9
        stamp = _sim_stamp(self._sim_ns)
        self._tick_count += 1

        x, y, yaw = self.car.base_link()
        if self._tick_count % self.gps_divider == 0:
            self._publish_gnss(self._sim_ns, x, y, yaw)
        self._publish_odom(stamp, x, y, yaw)
        with self._lock:
            self._clock_ns = self._sim_ns
        if self.publish_clock:
            self.pub_clock.publish(Clock(clock=stamp))

        self._service_teleports()

        elapsed = time.perf_counter() - t0
        self._tick_ms = elapsed * 1000.0
        if self.time_factor > 0.0:
            target = self.dt / self.time_factor
            if elapsed < target:
                time.sleep(target - elapsed)
        total = time.perf_counter() - t0
        self._rtf = self.dt / total if total > 0 else 0.0

    def _lockstep_wait(self):
        """
        Hold the tick until the controller's 0x101 for the last /clock has arrived.

        As the bridge does, and only while the ECU is ACTIVE.
        """
        t0 = time.perf_counter()
        with self._cond:
            if self.ecu.mode(self._sim_t) != EcuMode.ACTIVE:
                return
            ok = self._cond.wait_for(
                lambda: self._cmd_stamp_ns >= self._clock_ns or not self._running,
                timeout=self.lockstep_timeout,
            )
        waited_ms = (time.perf_counter() - t0) * 1000.0
        self._lockstep_waits += 1
        self._lockstep_wait_ms += (waited_ms - self._lockstep_wait_ms) / self._lockstep_waits
        if not ok:
            self._lockstep_timeouts += 1

    def _apply_dbw(self):
        """
        Run virtual ECU -> plant -> bicycle for the coming dt; publish /carla/dbw_state.

        Not ACTIVE (power-on, watchdog, E-stop): the plant gets a full-brake command
        (pedal 0, brake 255, NEUTRAL), so the hold has the shuttle's dead time and braking.
        """
        with self._lock:
            out = self.ecu.control(self._sim_t)
            status = self.ecu.status(self._sim_t)
        hold = status['mode'] != EcuMode.ACTIVE.value
        if hold:
            pedal, brake, neutral, reverse = 0.0, CAN_FULL_SCALE, True, False
        else:
            pedal, brake = out.throttle * CAN_FULL_SCALE, out.brake * CAN_FULL_SCALE
            neutral, reverse = out.neutral, out.reverse
        v = self.plant.step(
            self._sim_t,
            self.dt,
            self.car.v,
            pedal=pedal,
            brake=brake,
            neutral=neutral,
            reverse=reverse,
            active=True,
        )
        wheel_deg = out.steer * STEER_LIMIT_DEG
        self.car.step(self.dt, v, wheel_deg)
        self._dbw_status = status
        status = dict(
            status,
            t=round(self._sim_t, 3),
            applied={
                'throttle': round(pedal / CAN_FULL_SCALE, 3),
                'brake': round(brake / CAN_FULL_SCALE, 3),
                'steer': round(out.steer, 3),
                'wheel_deg': round(self.car.steer_deg, 2),
                'reverse': reverse,
                'hold': hold,
            },
            **self.plant.status(),
        )
        self.pub_dbw_state.publish(String(data=json.dumps(status)))

    def _publish_gnss(self, sim_ns, x, y, yaw):
        was_ok = self.gnss.ok
        payloads = self.gnss.step(sim_ns, x, y, math.cos(yaw), math.sin(yaw), abs(self.car.v),
                                  self.antenna_elevation, self.geo.map_to_wgs)
        if self.gnss.ok != was_ok:
            self.pub_gps_health.publish(Bool(data=self.gnss.ok))
        for ev in self.gnss.new_fault_events():
            self.pub_gnss_fault.publish(
                String(data=json.dumps(dict(ev, scenario=self.gnss_faults_name)))
            )
        if payloads:
            self.octane.publish(payloads)

    def _publish_odom(self, stamp, x, y, yaw):
        msg = Odometry()
        msg.header = Header(stamp=stamp, frame_id=self.odom_frame)
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = self.car.v
        msg.twist.twist.angular.z = self.car.yaw_rate
        self.pub_odom.publish(msg)

    def _service_teleports(self):
        try:
            while True:
                x, y, yaw = self._teleport_q.get_nowait()
                self.car.reset(x, y, yaw)
                self.plant.reset()
                self.get_logger().info(f'Teleported to map ({x:.2f}, {y:.2f}, yaw {yaw:.2f})')
        except queue.Empty:
            pass


def main(args=None):
    # SIGINT / SIGTERM end the loop below, so stop() always runs (tick thread joined, Octane port
    # closed)
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):  # older rclpy
        rclpy.init(args=args)
    stop = threading.Event()

    def _on_signal(signum, _frame):
        _trace(f'received signal {signum}, shutting down')
        stop.set()
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    node = KinematicSimNode()
    node.start()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop.is_set() and rclpy.ok() and node.alive:
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.stop()
        executor.remove_node(node)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        _trace('exiting')


if __name__ == '__main__':
    main()
