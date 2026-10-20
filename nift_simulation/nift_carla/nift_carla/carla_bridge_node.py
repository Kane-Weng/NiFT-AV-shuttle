#!/usr/bin/env python3
"""
Bridge the NiFT stack to a CARLA server: ego, sensors, clock, virtual ECU and GNSS.

File: carla_bridge_node.py
Author: Kane Weng

Description:
  Thin bridge between a CARLA 0.9.15 server running the Mcity Digital Twin and
  the NiFT ROS 2 stack. The stack drives the ego with the same CAN frames it
  sends to the shuttle:

    OUT  Octane beacon_update (socket.io)
              127.0.0.1:<octane.port>/octane, front / rear antenna
              (octane_emulator.py; beacon_node runs its real branch)
         /scan                            sensor_msgs/PointCloud2
              (frame `velodyne`, RELIABLE, 10 Hz)
         /ackermann_like_controller/odom  nav_msgs/Odometry (ground truth, frame `map`)
         /clock                           rosgraph_msgs/Clock
         /carla/gps_health                std_msgs/Bool
              (false while a simulated GPS dropout is active)
         /carla/gnss_fault                std_msgs/String
              (JSON: a scheduled GNSS fault starts / ends)
         /carla/dbw_state                 std_msgs/String
              (virtual ECU state as JSON, every tick)
         /diagnostics                     (hardware id `carla_bridge`)
    IN   /to_can_bus                      can_msgs/Frame (0x101 command, 0x100 E-stop)
         /initialpose                     geometry_msgs/PoseWithCovarianceStamped
              (teleport, map frame)
    SRV  /carla/dbw_power_cycle           std_srvs/Trigger (key off/on: clears the E-stop latch)

  The bridge publishes NO TF: robot_state_publisher (URDF) provides
  base_link -> velodyne / gps_frame_*, and shuttle_tf_node derives
  map -> base_link from /gnss/* exactly as on the real vehicle.

  Design notes
    * The bridge owns the CARLA world in synchronous mode and is the /clock
      source; a dedicated thread makes every CARLA call.  ROS callbacks only
      store state under a lock.
    * Antenna fixes are computed by the bridge from the ego transform through
      the OpenDRIVE geoReference (see carla_geo.py) rather than CARLA's GNSS
      sensor, which uses a Mercator approximation, and served the way the
      Mcity Octane server serves them (octane_emulator.py). The GNSS model,
      GPS noise / dropout included, is sim_gnss.py,
      shared with the kinematic sim; a dropout means no beacon_update events,
      as when Octane goes quiet.
    * CAN frames go to the virtual ECU (carla_dbw.py), which turns them into
      pedal / brake / wheel values before each tick.  With `dbw.plant: shuttle`
      (default) the shuttle plant (shuttle_plant.py, fitted to the garage test
      of 2026-10-08) turns pedal / brake into the shuttle's speed response and
      the bridge sets that longitudinal speed on the ego (set_target_velocity,
      lateral and vertical motion left to the physics); the ego's own
      powertrain only acts in the hold modes.  With `dbw.plant: carla` the
      pedal / brake drive CARLA's torque model as carla.VehicleControl.  The
      front-wheel limit is set to the shuttle's full lock so steer = +-1 is full
      lock.  `lockstep: true` holds each tick until the controller's frame for
      the last published /clock has arrived, so every run sees the same
      command timing.
    * base_link is the mid-wheelbase point on the ground plane projection of
      the vehicle, measured from the blueprint's wheel positions after spawn;
      antennas and the LiDAR are placed relative to it at the shuttle's sensor
      mounts (nift_common vehicle.yaml, which the URDF loads too).

Usage:
  ros2 run nift_carla carla_bridge_node --ros-args --params-file <carla_params.yaml>
      -p carla_host:=<ip>
"""

from dataclasses import replace
import json
import math
import os
import queue
import signal
import sys
import threading
import time
from typing import Optional, Tuple

from can_msgs.msg import Frame
import carla
import diagnostic_msgs.msg
import diagnostic_updater
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nift_carla.carla_dbw import CAN_FULL_SCALE, EcuConfig, EcuMode, inner_wheel_deg, VirtualEcu
from nift_carla.carla_geo import CarlaGeo
from nift_carla.carla_lidar import carla_lidar_to_xyz, lidar_attributes, pack_xyz32
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
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger

SHUTTLE = ShuttleConfig()


def _trace(msg: str):
    """Shutdown tracing that does not depend on the rclpy logger being alive."""
    print(f'[carla_bridge] {msg}', file=sys.stderr, flush=True)


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _sim_stamp(ns: int):
    return rclpy.time.Time(nanoseconds=ns).to_msg()


def shape_powertrain(pc, torque_nm=0.0, max_rpm=0.0, final_ratio=0.0, single_gear=False,
                     engine_damping=-1.0, brake_torque_nm=0.0):
    """
    Reshape a carla.VehiclePhysicsControl's drivetrain toward the shuttle.

    0 / negative / False = keep.

      torque_nm        flat engine torque up to max_rpm, so pedal -> wheel torque is linear
      max_rpm          engine speed limit; with one gear it sets the top speed
      final_ratio      differential ratio
      single_gear      one forward gear, like the shuttle's single-speed motor (no shift pauses)
      engine_damping   the same damping at every pedal position. The Micra's 2.0 at zero
                       throttle vs 0.15 at full makes part pedal a speed limit (0.31
                       plateaus at ~1 m/s), not a torque
      brake_torque_nm  per-wheel brake torque at brake = 1
    """
    if torque_nm > 0.0:
        rpm = max_rpm if max_rpm > 0.0 else pc.max_rpm
        pc.torque_curve = [carla.Vector2D(0.0, torque_nm), carla.Vector2D(rpm, torque_nm)]
    if max_rpm > 0.0:
        pc.max_rpm = max_rpm
    if final_ratio > 0.0:
        pc.final_ratio = final_ratio
    if single_gear:
        pc.forward_gears = [carla.GearPhysicsControl(ratio=1.0, down_ratio=0.0, up_ratio=1.0)]
    if engine_damping >= 0.0:
        pc.damping_rate_full_throttle = engine_damping
        pc.damping_rate_zero_throttle_clutch_engaged = engine_damping
        pc.damping_rate_zero_throttle_clutch_disengaged = engine_damping
    if brake_torque_nm > 0.0:
        wheels = list(pc.wheels)
        for w in wheels:
            w.max_brake_torque = brake_torque_nm
        pc.wheels = wheels
    return pc


class CarlaBridgeNode(Node):

    def __init__(self):
        super().__init__('carla_bridge')

        # ------------------------------------------------------------ #
        #  Parameters                                                  #
        # ------------------------------------------------------------ #
        p = self.declare_parameter
        # connection / world
        p('carla_host', 'localhost')
        p('carla_port', 2000)
        p('carla_timeout_s', 60.0)      # RPC timeout while connecting / loading the map
        # RPC timeout once running (a stuck tick() must raise, not hang)
        p('carla_tick_timeout_s', 10.0)
        p('map_name', 'McityMap_Main')
        p('load_map', True)
        p('reconnect_period_s', 2.0)
        p('fixed_delta_seconds', 0.05)
        p('time_factor', 1.0)           # 1 = real time, 0 = as fast as possible
        p('max_substeps', 10)
        p('max_substep_delta', 0.01)
        p('publish_clock', True)
        p('spectator_follow', False)
        p('collision_sensor', True)     # attach sensor.other.collision to the ego and report hits
        # geo
        p('map_origin_lat', MAP_ORIGIN_LAT)
        p('map_origin_lon', MAP_ORIGIN_LON)
        p('datum_alt_m', 0.0)
        # ego the shuttle itself (source-built server); vehicle.nissan.micra = closest stock
        # wheelbase (survey 2026-09-07)
        p('ego_blueprint', 'vehicle.nift.shuttle')
        p('ego_role_name', 'ego')
        p('spawn.mode', 'map')          # map | geo | carla_index
        p('spawn.x', 0.0)
        p('spawn.y', 0.0)
        p('spawn.yaw', 0.0)             # rad, map frame
        p('spawn.lat', 0.0)
        p('spawn.lon', 0.0)
        p('spawn.heading_deg', 0.0)     # compass, 0 = north, CW
        p('spawn.index', 0)
        p('spawn.z_offset', 0.3)
        # vehicle
        p('vehicle.mass', float(SHUTTLE.mass))
        # front-wheel limit; 0 = keep the blueprint's. The ECU maps steer = +-1 to whatever
        # limit the server reports back. With dbw.steer_map the bridge sets the limit itself
        # (full shuttle lock on this ego).
        p('vehicle.max_steer_angle_deg', float(SHUTTLE.max_steering_deg))
        # powertrain shaping (shape_powertrain); defaults keep the blueprint
        p('vehicle.engine_torque_nm', 0.0)
        p('vehicle.max_rpm', 0.0)
        p('vehicle.final_ratio', 0.0)
        p('vehicle.single_gear', False)
        p('vehicle.engine_damping', -1.0)
        p('vehicle.brake_torque_nm', 0.0)
        # virtual ECU (carla_dbw.py)
        p('dbw.cmd_timeout_s', 0.5)     # no 0x101 for this long -> full brake + hand brake
        p('dbw.steer_sign', -1.0)       # CAN +steer = left (ROS), CARLA +steer = right
        # shuttle = speed-command plant (shuttle_plant.py); carla = torque model
        p('dbw.plant', 'shuttle')
        # steer the ego on the shuttle's turning circle (carla_dbw.inner_wheel_deg)
        p('dbw.steer_map', True)
        # shuttle plant (shuttle_plant.py; defaults = the garage fit of 2026-10-08)
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
        # before each tick, wait for the frame computed from the last /clock
        p('lockstep', False)
        # wall-clock cap on that wait (only while the ECU is ACTIVE)
        p('lockstep_timeout_s', 0.1)
        # sensors (the shuttle's mounts, as in the URDF)
        p('antenna_offset_x', SHUTTLE.gnss_front_x)     # front +x, rear -x
        p('antenna_z', SHUTTLE.gnss_z)
        p('base_link_height', SHUTTLE.base_link_height)
        p('lidar.x', SHUTTLE.lidar_x)
        p('lidar.y', SHUTTLE.lidar_y)
        p('lidar.z', SHUTTLE.lidar_z)
        p('lidar.pitch_deg', -SHUTTLE.lidar_pitch_deg)  # UE sign: negative = nose down
        p('lidar.min_range', 0.3)
        p('lidar.range', 12.0)
        p('lidar.channels', 8)
        p('lidar.h_samples', 180)
        p('lidar.h_fov_deg', 180.0)
        p('lidar.v_fov_deg', 15.0)
        p('lidar.noise_stddev', 0.0)
        p('lidar.publish_divider', 2)   # ticks per /scan message (20 Hz / 2 = 10 Hz)
        p('lidar.wait_s', 0.04)         # max wait after tick() for the LiDAR packet of that frame
        p('gps.rate_divider', 1)
        p('gps.noise_stddev_m', 0.0)
        p('gps.noise_heading', False)   # independent noise per antenna (corrupts heading)
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
        # Octane emulator (beacon_node's env must match: nift_bringup system.launch.py reads these)
        p('octane.port', 8765)
        p('octane.api_key', 'sim')
        p('octane.namespace', '/octane')
        p('octane.front_id', 'SIM-FRONT')   # not the shuttle's IDs: a sim client sent to the real
        p('octane.rear_id', 'SIM-REAR')     # Octane by mistake must match nothing
        # topics
        p('can_topic', '/to_can_bus')
        p('odom_topic', '/ackermann_like_controller/odom')
        p('odom_frame', 'map')
        p('base_frame', 'base_link')
        p('scan_topic', '/scan')
        p('scan_frame', 'velodyne')

        # A shorthand helper g(name) to extract the value of a ROS 2 node parameter
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.host = g('carla_host')
        self.port = int(g('carla_port'))
        self.timeout = float(g('carla_timeout_s'))
        self.map_name = g('map_name')
        self.load_map = bool(g('load_map'))
        self.reconnect_period = float(g('reconnect_period_s'))
        self.dt = float(g('fixed_delta_seconds'))
        self.time_factor = float(g('time_factor'))
        self.publish_clock = bool(g('publish_clock'))
        self.spectator_follow = bool(g('spectator_follow'))
        self.datum_lat = float(g('map_origin_lat'))
        self.datum_lon = float(g('map_origin_lon'))
        self.datum_alt = float(g('datum_alt_m'))
        wheel_limit = float(g('vehicle.max_steer_angle_deg')) or SHUTTLE.max_steering_deg
        self._ecu_cfg = EcuConfig(
            cmd_timeout_s=float(g('dbw.cmd_timeout_s')),
            wheel_max_steer_deg=wheel_limit,
            steer_sign=float(g('dbw.steer_sign')),
            cmd_wheelbase=SHUTTLE.wheel_base,
        )
        self.steer_map = bool(g('dbw.steer_map'))
        # front-wheel limit _apply_physics sets: the parameter until the ego's geometry is known
        # (_spawn_ego)
        self._wheel_limit_deg = float(g('vehicle.max_steer_angle_deg'))
        self.lockstep = bool(g('lockstep'))
        self.lockstep_timeout = float(g('lockstep_timeout_s'))
        plant_kind = str(g('dbw.plant'))
        if plant_kind not in ('shuttle', 'carla'):
            raise ValueError(f'dbw.plant must be "shuttle" or "carla", not "{plant_kind}"')
        self.plant: Optional[ShuttlePlant] = None
        if plant_kind == 'shuttle':
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
        self.antenna_offset_x = float(g('antenna_offset_x'))
        self.antenna_z = float(g('antenna_z'))
        self.base_link_height = float(g('base_link_height'))
        self.scan_divider = max(1, int(g('lidar.publish_divider')))
        self.lidar_wait = float(g('lidar.wait_s'))
        self.lidar_min_range = float(g('lidar.min_range'))
        self.gps_divider = max(1, int(g('gps.rate_divider')))
        self.gnss = GnssSim(
            self.antenna_offset_x,
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
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=5)

        # Publishers
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_scan = self.create_publisher(PointCloud2, g('scan_topic'), reliable)
        self.pub_odom = self.create_publisher(Odometry, g('odom_topic'), 10)
        self.pub_gps_health = self.create_publisher(Bool, '/carla/gps_health', 10)
        self.pub_gnss_fault = self.create_publisher(String, '/carla/gnss_fault', 10)
        self.pub_dbw_state = self.create_publisher(String, '/carla/dbw_state', 10)
        self.scan_frame = g('scan_frame')
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')

        # Subscribers / services
        self.create_subscription(Frame, g('can_topic'), self._can_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._initialpose_callback, 10
        )
        self.create_service(Trigger, '/carla/dbw_power_cycle', self._power_cycle_callback)

        # Diagnostics updater
        self.diag = diagnostic_updater.Updater(self)
        self.diag.setHardwareID('carla_bridge')
        self.diag.add('CARLA Bridge', self._diagnostics_callback)

        # GNSS goes out the way Octane delivers it. Started before CARLA so a port clash fails
        # fast and beacon_node can connect while the map loads (raises if the port is taken).
        self.octane = OctaneEmulator(port=int(g('octane.port')), api_key=g('octane.api_key'),
                                     namespace=g('octane.namespace'), log=self.get_logger())
        self.octane.start()

        # ------------------------------------------------------------ #
        #  Shared state                                                #
        # ------------------------------------------------------------ #
        self._lock = threading.Lock()
        # lockstep: CAN callback wakes the tick thread
        self._cond = threading.Condition(self._lock)
        self.ecu = VirtualEcu(self._ecu_cfg)            # replaced (powered on) at every ego spawn
        self._dbw_status = {}
        self._cmd_stamp_ns = -1     # header stamp of the latest valid 0x101
        self._clock_ns = -1         # stamp of the latest /clock published
        self._lockstep_waits = 0
        self._lockstep_timeouts = 0
        self._lockstep_wait_ms = 0.0    # mean
        self._teleport_q: 'queue.Queue[Tuple[float, float, float]]' = queue.Queue()
        self._lidar_q: 'queue.Queue' = queue.Queue()
        self._running = True
        self._cleaned = False
        self._ego_ids = []          # actor ids we spawned, for out-of-thread cleanup
        self._connected = False
        self._sim_t = 0.0
        # (frame, ns) of the first tick after connecting: sim time is counted from there in
        # whole fixed_delta steps. elapsed_seconds is a float running sum: converted to ns, a few
        # steps per 1000 s come out 49 999 999 / 50 000 001 ns, enough to skip a 20 Hz timer.
        self._frame0 = None
        self._tick_ms = 0.0
        self._rtf = 0.0
        self._last_error = ''
        self._tick_count = 0
        self._verify_wheels = False     # set by a teleport, checked after the next tick
        self._collisions = 0
        self._last_collision = ''
        self._last_collision_t = -1e9
        self.collision = None

        # CARLA handles (tick thread only)
        self.client = None
        self.world = None
        self.carla_map = None
        self.geo: Optional[CarlaGeo] = None
        self.ego = None
        self.lidar = None
        self._orig_settings = None
        self._dx = 0.0            # actor origin -> base_link, along forward (m)
        self._dy = 0.0            # actor origin -> base_link, along right (m)
        self._ground_z = 0.0      # actor origin -> ground, local z (negative)
        self.wheelbase = 0.0
        self.front_track = 0.0

        self._thread = threading.Thread(target=self._run, name='carla_tick', daemon=True)

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
        stat.add('Host', f'{self.host}:{self.port}')
        stat.add('Map', str(self.map_name))
        stat.add('Sim time', f'{self._sim_t:.2f} s')
        stat.add('Tick wall', f'{self._tick_ms:.1f} ms')
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
        stat.add('Wheelbase', f'{self.wheelbase:.3f} m')
        stat.add('GPS health', 'OK' if self.gnss.ok else 'DROPOUT')
        stat.add('Octane emulator',
                 f'port {self.octane.port}, {self.octane.clients_authed} client(s) authenticated, '
                 f'{self.octane.emitted} beacon_update sent')
        stat.add(
            'Collisions',
            f'{self._collisions}'
            + (f' (last: {self._last_collision})' if self._collisions else ''),
        )
        if not self._connected:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                         f'Not connected to CARLA ({self._last_error or "connecting"})')
        elif dbw.get('mode') == EcuMode.ESTOP.value:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                         'E-stop latched — call /carla/dbw_power_cycle to release')
        elif dbw.get('mode') == EcuMode.ACTIVE.value:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Connected — driving (CAN)')
        else:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.OK,
                'Connected — no CAN command, holding brake',
            )
        return stat

    # ================================================================== #
    #  Tick thread                                                       #
    # ================================================================== #
    def start(self):
        self._thread.start()

    def stop(self):
        """Stop the tick thread; if it does not finish, clean the server up from here."""
        _trace('stop(): stopping tick thread')
        self._running = False
        self._thread.join(timeout=8.0)
        if self._thread.is_alive():
            _trace('stop(): tick thread still busy — cleaning up CARLA with a fresh client')
        self._cleanup_out_of_thread()
        try:
            self.octane.stop()
        except Exception as ex:     # never block the CARLA cleanup above on it
            _trace(f'stop(): Octane emulator did not stop cleanly ({ex!r})')
        _trace('stop(): done')

    @property
    def alive(self) -> bool:
        return self._thread.is_alive()

    def _run(self):
        """Run the main loop with auto-reconnect."""
        while self._running:
            try:
                self._connect()
                self._spawn_ego()
                self._attach_lidar()
                if bool(self.get_parameter('collision_sensor').value):
                    self._attach_collision_sensor()
                self._connected = True
                self._last_error = ''
                self.get_logger().info('CARLA bridge running.')
                while self._running:
                    self._tick_once()
            except RuntimeError as ex:          # RPC timeout / server gone
                self._connected = False
                self._last_error = str(ex).splitlines()[0][:120] if str(ex) else 'RuntimeError'
                self.get_logger().error(
                    f'CARLA error: {self._last_error} — reconnecting in '
                    f'{self.reconnect_period:.0f}s'
                )
                try:
                    if self.world is not None:
                        self._leave_sync_mode(self.world)
                except Exception:
                    pass
                self._drop_actors()
                for _ in range(int(self.reconnect_period / 0.1)):
                    if not self._running:
                        break
                    time.sleep(0.1)
            except Exception as ex:             # programming error: log and stop the loop
                self._connected = False
                self._last_error = repr(ex)
                self.get_logger().fatal(f'CARLA bridge crashed: {ex!r}')
                self._running = False
        self._cleanup()

    # ------------------------------------------------------------------ #
    def _connect(self):
        """
        Connect and prepare the world.

        Loads the map, turns on synchronous mode and builds the CarlaGeo converter
        for coordinate conversion.
        """
        self.get_logger().info(f'Connecting to CARLA at {self.host}:{self.port} ...')
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        version = self.client.get_server_version()
        self.world = self.client.get_world()
        current = self.world.get_map().name
        if self.load_map and not current.endswith(self.map_name):
            self.get_logger().info(
                f'Loading map {self.map_name} (current: {current}) — this can take a while'
            )
            self.world = self.client.load_world(self.map_name)
        self.carla_map = self.world.get_map()
        self.get_logger().info(f'Connected to CARLA {version}, map {self.carla_map.name}')
        self.client.set_timeout(float(self.get_parameter('carla_tick_timeout_s').value))

        # Turns on synchronous mode with a fixed 0.05 s step and physics substepping
        self._orig_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.dt
        settings.substepping = True
        settings.max_substep_delta_time = float(self.get_parameter('max_substep_delta').value)
        settings.max_substeps = int(self.get_parameter('max_substeps').value)
        self.world.apply_settings(settings)
        self._frame0 = None

        # Build the coordinate converter
        self.geo = CarlaGeo.from_opendrive(
            self.carla_map.to_opendrive(), self.datum_lat, self.datum_lon
        )
        if self.geo.georef_is_fallback:
            self.get_logger().warn(
                'No <geoReference> in the server OpenDRIVE — using the built-in Mcity tmerc string'
            )
        self.get_logger().info(
            f'Georeference: {self.geo.georef} | grid convergence '
            f'{self.geo.grid_convergence_deg():.2f} deg'
        )

    # ------------------------------------------------------------------ #
    def _spawn_ego(self):
        """Spawn the ego car, trying 4 different heights in case a spawn fails."""
        bp_lib = self.world.get_blueprint_library()
        bp_id = self.get_parameter('ego_blueprint').value
        candidates = bp_lib.filter(bp_id)
        if not candidates:
            raise RuntimeError(f'ego blueprint "{bp_id}" not found on server')
        bp = candidates[0]
        bp.set_attribute('role_name', self.get_parameter('ego_role_name').value)

        tf = self._spawn_transform()
        # actor origin sits above the ground by roughly half its height; try a few heights
        base_z = tf.location.z
        z_off = float(self.get_parameter('spawn.z_offset').value)
        self.ego = None
        for attempt in range(4):
            tf.location.z = base_z + z_off + 0.5 * attempt
            self.ego = self.world.try_spawn_actor(bp, tf)
            if self.ego is not None:
                break
        if self.ego is None:
            raise RuntimeError(f'could not spawn {bp.id} near {tf.location}')
        self._ego_ids.append(self.ego.id)
        self.get_logger().info(f'Spawned {bp.id} id={self.ego.id}')

        self._apply_physics()
        self.world.tick()
        self._measure_geometry()    # Find base_link
        if self.steer_map and self.wheelbase > 0.0:
            # steer = +-1 must reach the inner-wheel angle of full shuttle lock on this ego
            self._wheel_limit_deg = inner_wheel_deg(
                SHUTTLE.max_steering_deg, SHUTTLE.wheel_base, self.wheelbase, self.front_track
            )
            self._apply_physics()
            self.world.tick()
        self._power_on_ecu()

    def _spawn_transform(self) -> carla.Transform:
        """Compute where to spawn the car in CARLA (3 spawn modes: map/geo/carla_index)."""
        mode = self.get_parameter('spawn.mode').value
        # Mode 1: pick one of CARLA's built-in spawn points
        if mode == 'carla_index':
            pts = self.carla_map.get_spawn_points()
            idx = int(self.get_parameter('spawn.index').value) % max(1, len(pts))
            tf = pts[idx]
            self.get_logger().info(f'Spawn from CARLA spawn point {idx}: {tf.location}')
            return tf
        # Mode 2: latitude, longitude and compass heading. 90 - heading converts a compass heading
        # into a ROS yaw.
        if mode == 'geo':
            lat = float(self.get_parameter('spawn.lat').value)
            lon = float(self.get_parameter('spawn.lon').value)
            x, y = self.geo.wgs_to_map(lat, lon)
            yaw = math.radians(90.0 - float(self.get_parameter('spawn.heading_deg').value))
        # Mode 3: x, y and yaw in your ROS map frame
        else:
            x = float(self.get_parameter('spawn.x').value)
            y = float(self.get_parameter('spawn.y').value)
            yaw = float(self.get_parameter('spawn.yaw').value)

        # Map & CARLA coordinates
        cx, cy, _ = self.geo.map_to_carla(x, y)
        cyaw = self.geo.map_yaw_to_carla_yaw_deg(x, y, yaw)
        loc = carla.Location(x=cx, y=cy, z=self._ground_height(cx, cy))
        self.get_logger().info(
            f'Spawn at map ({x:.2f}, {y:.2f}, yaw {yaw:.2f}) -> CARLA ({cx:.2f}, {cy:.2f}, '
            f'{loc.z:.2f}, yaw {cyaw:.1f})'
        )
        return carla.Transform(loc, carla.Rotation(yaw=cyaw))

    def _ground_height(self, cx: float, cy: float) -> float:
        """
        Return z of the ground at (cx, cy) in CARLA world coordinates.

        The twin's roads sit at ~276 m (OpenDRIVE elevation), so cast the probe ray from
        well above (z = +1000) and fall back to the OpenDRIVE road height.
        """
        try:
            hit = self.world.ground_projection(carla.Location(x=cx, y=cy, z=1000.0), 2000.0)
            if hit is not None:
                return hit.location.z
        except Exception:
            pass
        wp = self.carla_map.get_waypoint(carla.Location(x=cx, y=cy, z=0.0), project_to_road=True)
        return wp.transform.location.z if wp is not None else 0.0

    def _apply_physics(self):
        pc = self.ego.get_physics_control()
        pc.mass = float(self.get_parameter('vehicle.mass').value)
        max_steer_deg = self._wheel_limit_deg
        if max_steer_deg > 0.0:
            wheels = list(pc.wheels)
            for i, w in enumerate(wheels):
                w.max_steer_angle = max_steer_deg if i < 2 else 0.0
            pc.wheels = wheels
        # CARLA scales steer down with speed (steering_curve: km/h -> factor); the shuttle does
        # not. Except at standstill: the plant pushes the ego along its heading with no yaw rate,
        # and at the steering map's 19.4 deg inner wheel the steered front tyres scrub it back to
        # rest every tick (stall from rest at >= 14.5 deg commands, 20261010_steer_map). 0.9 x the
        # limit at 0 km/h, full lock from 1 km/h.
        pc.steering_curve = [
            carla.Vector2D(0.0, 0.9),
            carla.Vector2D(1.0, 1.0),
            carla.Vector2D(200.0, 1.0),
        ]
        gp = lambda n: self.get_parameter(f'vehicle.{n}').value  # noqa: E731
        shape_powertrain(
            pc,
            torque_nm=float(gp('engine_torque_nm')),
            max_rpm=float(gp('max_rpm')),
            final_ratio=float(gp('final_ratio')),
            single_gear=bool(gp('single_gear')),
            engine_damping=float(gp('engine_damping')),
            brake_torque_nm=float(gp('brake_torque_nm')),
        )
        self.ego.apply_physics_control(pc)

    def _power_on_ecu(self):
        """
        Reset the ECU for a new ego vehicle.

        New ego = new vehicle: fresh ECU, steer = +-1 mapped to the wheel limit the server reports.
        """
        pc = self.ego.get_physics_control()
        wheel_deg = float(pc.wheels[0].max_steer_angle)
        mapped = self.steer_map and self.wheelbase > 0.0
        with self._lock:
            self.ecu = VirtualEcu(replace(self._ecu_cfg, wheel_max_steer_deg=wheel_deg,
                                          map_wheelbase=self.wheelbase if mapped else 0.0,
                                          map_track=self.front_track if mapped else 0.0))
        if mapped:
            self.get_logger().info(
                f'Steering map: shuttle wheelbase {SHUTTLE.wheel_base:.3f} m -> ego wheelbase '
                f'{self.wheelbase:.3f} m, '
                f'front track {self.front_track:.3f} m: {SHUTTLE.max_steering_deg:.1f} deg '
                'command = inner wheel '
                f'{wheel_deg:.2f} deg, turning radius {SHUTTLE.min_turn_radius:.2f} m'
            )
        else:
            self.get_logger().warn(
                'Steering map off: the ego turns on its own wheelbase, not the shuttle\'s'
            )
        curve = [(round(v.x, 1), round(v.y, 2)) for v in pc.steering_curve]
        self.get_logger().info(
            f'Virtual ECU on: front-wheel limit {wheel_deg:.1f} deg, steering curve {curve}, '
            f'mass {pc.mass:.0f} kg, cmd timeout {self._ecu_cfg.cmd_timeout_s:.2f} s, '
            f'lockstep {"on" if self.lockstep else "off"}'
        )
        self.get_logger().info(
            f'Powertrain: torque {[(round(v.x), round(v.y, 1)) for v in pc.torque_curve]}, '
            f'max_rpm {pc.max_rpm:.0f}, '
            f'{len(pc.forward_gears)} gear(s), final ratio {pc.final_ratio:.3f}, engine damping '
            f'{pc.damping_rate_full_throttle:.2f}/'
            f'{pc.damping_rate_zero_throttle_clutch_engaged:.2f}, '
            f'brake torque {pc.wheels[0].max_brake_torque:.0f} Nm/wheel'
        )
        if self.plant is not None:
            self.plant.reset()
            c = self.plant.cfg
            self.get_logger().info(
                f'Shuttle plant: pedal map {[(int(p), v) for p, v in c.pedal_map]} m/s, motor '
                f'loop a_lim {c.a_lim} m/s^2 '
                f'tau {c.tau} s, engagement {c.t_eng} s, dead time {c.cmd_delay} s, brake '
                f'{c.brake_a0} {c.brake_k_v:+} v '
                f'(>= {c.brake_a_min}, full at {c.brake_full:.0f}), governor {c.max_speed} m/s '
                f'(CARLA powertrain only holds)'
            )
        else:
            self.get_logger().info('Plant: CARLA torque model (dbw.plant: carla)')

    def _measure_geometry(self):
        """Locate base_link (mid-wheelbase) and the ground relative to the actor origin."""
        tf = self.ego.get_transform()
        fwd, right = tf.get_forward_vector(), tf.get_right_vector()
        pc = self.ego.get_physics_control()
        ws = [np.array([w.position.x, w.position.y, w.position.z]) / 100.0 for w in pc.wheels]
        if len(ws) >= 4:
            front = (ws[0] + ws[1]) / 2.0
            rear = (ws[2] + ws[3]) / 2.0
            self.wheelbase = float(np.linalg.norm(front[:2] - rear[:2]))
            self.front_track = float(np.linalg.norm(ws[0][:2] - ws[1][:2]))
            mid = (front + rear) / 2.0
            d = mid - np.array([tf.location.x, tf.location.y, tf.location.z])
            self._dx = float(d[0] * fwd.x + d[1] * fwd.y)
            self._dy = float(d[0] * right.x + d[1] * right.y)
        else:
            self.wheelbase = self.front_track = 0.0
            self._dx = self._dy = 0.0
        bb = self.ego.bounding_box
        self._ground_z = float(bb.location.z - bb.extent.z)
        self.get_logger().info(
            f'Ego geometry: wheelbase {self.wheelbase:.3f} m (shuttle {SHUTTLE.wheel_base:.3f}), '
            f'front track {self.front_track:.3f} m, '
            f'base_link offset '
            f'({self._dx:+.3f}, {self._dy:+.3f}) m, ground at z {self._ground_z:+.3f} m, '
            f'bbox {2*bb.extent.x:.2f} x {2*bb.extent.y:.2f} x {2*bb.extent.z:.2f} m')

    # ------------------------------------------------------------------ #
    def _attach_lidar(self):
        bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        gp = self.get_parameter
        attrs = lidar_attributes(self.dt,
                                 channels=int(gp('lidar.channels').value),
                                 h_samples=int(gp('lidar.h_samples').value),
                                 range_max=float(gp('lidar.range').value),
                                 h_fov_deg=float(gp('lidar.h_fov_deg').value),
                                 v_fov_deg=float(gp('lidar.v_fov_deg').value),
                                 noise_stddev=float(gp('lidar.noise_stddev').value))
        for k, v in attrs.items():
            if bp.has_attribute(k):
                bp.set_attribute(k, v)
        loc = carla.Location(x=self._dx + float(gp('lidar.x').value),
                             y=self._dy - float(gp('lidar.y').value),      # ROS +y (left) -> UE -y
                             z=self._ground_z + self.base_link_height + float(gp('lidar.z').value))
        rot = carla.Rotation(pitch=float(gp('lidar.pitch_deg').value))
        self.lidar = self.world.spawn_actor(bp, carla.Transform(loc, rot), attach_to=self.ego)
        self._ego_ids.append(self.lidar.id)
        self.lidar.listen(self._lidar_q.put)
        self.get_logger().info(f'LiDAR attached at {loc} pitch {rot.pitch:.1f} deg: {attrs}')

    def _attach_collision_sensor(self):
        bp = self.world.get_blueprint_library().find('sensor.other.collision')
        self.collision = self.world.spawn_actor(bp, carla.Transform(), attach_to=self.ego)
        self._ego_ids.append(self.collision.id)
        self.collision.listen(self._on_collision)

    def _on_collision(self, event):
        other = event.other_actor
        desc = f'{other.type_id} (id {other.id})' if other is not None else 'unknown'
        self._collisions += 1
        self._last_collision = desc
        now = time.monotonic()
        if now - self._last_collision_t > 1.0:          # rate-limit the log, not the count
            imp = event.normal_impulse
            self.get_logger().warn(
                f'COLLISION with {desc}, impulse ({imp.x:.0f}, {imp.y:.0f}, {imp.z:.0f}) '
                f'at sim t={self._sim_t:.2f}s'
            )
        self._last_collision_t = now

    # ------------------------------------------------------------------ #
    """
    Notes on HOW TIME WORKS IN A SIMULATOR

    Two clocks in the system
    - Wall time: the real clock on the computer
    - Sim time: the clock inside the simulated world. (Only moves forward when the
                simulator computes another step of physics)

    CARLA's two mode
    - Async mode (default): server runs physics as fast as it can
                            (data doesn't line up)
                            -->
                            clients read whatever state is current
    - Sync mode (used here): server waits; world advances when a client calls
                             world.tick() & each call advances it by
                             fixed_delta_seconds
                             (substep help CARLA splits one tick for stable physics)
    """

    def _tick_once(self):
        """Run one simulation step."""
        # Time control
        t0 = time.perf_counter()        # wall-clock stopwatch
        if self.lockstep:
            self._lockstep_wait()

        # Ego car control from the latest CAN frames, applied to the step below
        self._apply_dbw()

        self.world.tick()               # advance physics 0.05 s
        snap = self.world.get_snapshot()
        if self._frame0 is None:
            self._frame0 = (snap.frame, round(snap.timestamp.elapsed_seconds * 1e9))
        sim_ns = self._frame0[1] + (snap.frame - self._frame0[0]) * round(self.dt * 1e9)
        self._sim_t = sim_ns / 1e9
        stamp = _sim_stamp(sim_ns)
        self._tick_count += 1
        if self._verify_wheels:
            self._verify_wheels = False
            self._check_wheels()

        # Read the car's pose + publish every sensor
        base, fwd, right = self._base_link_pose()
        if self._tick_count % self.gps_divider == 0:
            self._publish_gnss(sim_ns, base, fwd)
        self._publish_odom(stamp, base, fwd, right)
        self._publish_scan(stamp, snap.frame)

        # Advance time for the rest of the ROS system
        with self._lock:
            self._clock_ns = sim_ns
        if self.publish_clock:
            self.pub_clock.publish(Clock(clock=stamp))

        # Handle any /initialpose requests & camera move
        self._service_teleports()
        if self.spectator_follow:
            self._follow_spectator(base, fwd)

        # Pacing step: slow the bridge if work finished early
        elapsed = time.perf_counter() - t0
        self._tick_ms = elapsed * 1000.0
        if self.time_factor > 0.0:
            target = self.dt / self.time_factor
            if elapsed < target:
                time.sleep(target - elapsed)

        # Real-time factor (RTF)
        total = time.perf_counter() - t0
        self._rtf = self.dt / total if total > 0 else 0.0

    # ------------------------------------------------------------------ #
    def _base_link_pose(self):
        tf = self.ego.get_transform()
        fwd, right = tf.get_forward_vector(), tf.get_right_vector()
        base = carla.Location(x=tf.location.x + fwd.x * self._dx + right.x * self._dy,
                              y=tf.location.y + fwd.y * self._dx + right.y * self._dy,
                              z=tf.location.z + self._ground_z + self.base_link_height)
        return base, fwd, right

    def _lockstep_wait(self):
        """
        Hold the tick until the controller's 0x101 for the last published /clock has arrived.

        The controller stamps each frame with its sim-time now, which is that /clock stamp.
        Only while the ECU is ACTIVE: IDLE sends no frames, and a latched E-stop needs none.
        """
        t0 = time.perf_counter()
        with self._cond:
            if self.ecu.mode(self._sim_t) != EcuMode.ACTIVE:
                return
            ok = self._cond.wait_for(
                lambda: self._cmd_stamp_ns >= self._clock_ns, timeout=self.lockstep_timeout
            )
        waited_ms = (time.perf_counter() - t0) * 1000.0
        self._lockstep_waits += 1
        self._lockstep_wait_ms += (waited_ms - self._lockstep_wait_ms) / self._lockstep_waits
        if not ok:
            self._lockstep_timeouts += 1

    def _apply_dbw(self):
        """
        Apply the virtual ECU output to the ego for the next tick.

        Publishes the state on /carla/dbw_state. Shuttle plant: the ECU's pedal / brake
        counts go through the fitted speed model and the resulting longitudinal speed is set
        on the ego (the lateral and vertical components of its velocity are kept); the CARLA
        pedals are then idle, except the service brake while the plant holds at rest. Hold
        modes (power-on, watchdog, E-stop) stay with the physics brake. CARLA plant:
        VehicleControl as before.
        """
        with self._lock:
            out = self.ecu.control(self._sim_t)
            status = self.ecu.status(self._sim_t)
        ctrl = carla.VehicleControl(throttle=out.throttle, steer=out.steer, brake=out.brake,
                                    hand_brake=out.hand_brake, reverse=out.reverse)
        plant = {}
        if self.plant is not None:
            vel = self.ego.get_velocity()
            fwd = self.ego.get_transform().get_forward_vector()
            v_long = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z
            v_set = self.plant.step(
                self._sim_t,
                self.dt,
                v_long,
                pedal=out.throttle * CAN_FULL_SCALE,
                brake=out.brake * CAN_FULL_SCALE,
                neutral=out.neutral,
                reverse=out.reverse,
                active=status['mode'] == EcuMode.ACTIVE.value,
            )
            if v_set is not None:
                ctrl.throttle, ctrl.brake = 0.0, (1.0 if v_set == 0.0 else 0.0)
                dv = v_set - v_long
                self.ego.set_target_velocity(
                    carla.Vector3D(vel.x + fwd.x * dv, vel.y + fwd.y * dv, vel.z + fwd.z * dv)
                )
            plant = self.plant.status()
        self.ego.apply_control(ctrl)
        self._dbw_status = status
        status = dict(status, t=round(self._sim_t, 3),
                      applied={'throttle': round(ctrl.throttle, 3), 'brake': round(ctrl.brake, 3),
                               'steer': round(ctrl.steer, 3), 'reverse': ctrl.reverse,
                               'hand_brake': ctrl.hand_brake}, **plant)
        self.pub_dbw_state.publish(String(data=json.dumps(status)))

    # ------------------------------------------------------------------ #
    def _publish_gnss(self, sim_ns, base, fwd):
        """
        Send both antenna fixes to the Octane emulator (sim_gnss.py).

        Publishes /carla/gps_health on a dropout edge.
        """
        was_ok = self.gnss.ok
        vel = self.ego.get_velocity()
        payloads = self.gnss.step(sim_ns, base.x, base.y, fwd.x, fwd.y,
                                  # ground speed, like an RTK receiver
                                  math.hypot(vel.x, vel.y),
                                  self.datum_alt + base.z + self.antenna_z, self.geo.carla_to_wgs)
        if self.gnss.ok != was_ok:
            self.pub_gps_health.publish(Bool(data=self.gnss.ok))
        for ev in self.gnss.new_fault_events():
            self.pub_gnss_fault.publish(
                String(data=json.dumps(dict(ev, scenario=self.gnss_faults_name)))
            )
        if payloads:
            self.octane.publish(payloads)

    def _publish_odom(self, stamp, base, fwd, right):
        x, y, yaw = self.geo.carla_pose_to_map(base.x, base.y, fwd.x, fwd.y)
        vel = self.ego.get_velocity()
        ang = self.ego.get_angular_velocity()
        msg = Odometry()
        msg.header = Header(stamp=stamp, frame_id=self.odom_frame)
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z
        msg.twist.twist.linear.y = -(vel.x * right.x + vel.y * right.y + vel.z * right.z)
        msg.twist.twist.angular.z = -math.radians(ang.z)
        self.pub_odom.publish(msg)

    def _publish_scan(self, stamp, frame_no):
        if self._tick_count % self.scan_divider != 0:
            return
        latest = None
        try:
            # sensor data for this frame arrives a few ms after world.tick() returns
            latest = self._lidar_q.get(timeout=self.lidar_wait)
            while True:
                latest = self._lidar_q.get_nowait()
        except queue.Empty:
            pass
        if latest is None:
            return
        xyz = carla_lidar_to_xyz(latest, self.lidar_min_range)
        self.pub_scan.publish(pack_xyz32(Header(stamp=stamp, frame_id=self.scan_frame), xyz))

    def _service_teleports(self):
        moved = False
        try:
            while True:
                x, y, yaw = self._teleport_q.get_nowait()
                self._teleport(x, y, yaw)
                moved = True
        except queue.Empty:
            pass
        if moved:
            self.world.tick()
            self.ego.set_simulate_physics(True)
            # Re-enabling physics rebuilds the wheels from the blueprint (brake torque back to 550
            # Nm, steer limit to 70 deg -> the ECU's steer scale would be 4.7x off); engine
            # settings survive.
            self._apply_physics()
            self._verify_wheels = True      # physics changes show up after the next tick

    def _check_wheels(self):
        wheel_deg = float(self.ego.get_physics_control().wheels[0].max_steer_angle)
        if abs(wheel_deg - self.ecu.cfg.wheel_max_steer_deg) > 0.1:
            self.get_logger().error(
                f'Front-wheel limit {wheel_deg:.1f} deg after teleport, but the ECU scales '
                f'steer by {self.ecu.cfg.wheel_max_steer_deg:.1f} deg'
            )

    def _teleport(self, x: float, y: float, yaw: float):
        cx, cy, _ = self.geo.map_to_carla(x, y)
        cyaw = self.geo.map_yaw_to_carla_yaw_deg(x, y, yaw)
        r = math.radians(cyaw)
        fx, fy = math.cos(r), math.sin(r)
        # UE right vector for yaw (z-up, left-handed)
        rx, ry = -fy, fx
        ox = cx - fx * self._dx - rx * self._dy
        oy = cy - fy * self._dx - ry * self._dy
        oz = (
            self._ground_height(ox, oy)
            - self._ground_z
            + float(self.get_parameter('spawn.z_offset').value)
        )
        self.ego.set_simulate_physics(False)
        self.ego.set_transform(
            carla.Transform(carla.Location(x=ox, y=oy, z=oz), carla.Rotation(yaw=cyaw))
        )
        self.ego.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        self.ego.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        if self.plant is not None:
            self.plant.reset()
        self.get_logger().info(
            f'Teleported to map ({x:.2f}, {y:.2f}, yaw {yaw:.2f}) -> CARLA ({ox:.2f}, {oy:.2f}, '
            f'{oz:.2f})'
        )

    def _follow_spectator(self, base, fwd):
        spec = self.world.get_spectator()
        loc = carla.Location(x=base.x - fwd.x * 8.0, y=base.y - fwd.y * 8.0, z=base.z + 4.0)
        yaw = math.degrees(math.atan2(fwd.y, fwd.x))
        spec.set_transform(carla.Transform(loc, carla.Rotation(pitch=-15.0, yaw=yaw)))

    # ------------------------------------------------------------------ #
    def _drop_actors(self):
        for a in (self.collision, self.lidar, self.ego):
            try:
                if a is not None:
                    if hasattr(a, 'stop'):
                        a.stop()
                    a.destroy()
            except Exception:
                pass
        self.lidar = self.ego = self.collision = None

    def _leave_sync_mode(self, world):
        """
        Switch the world back to asynchronous mode.

        Must happen BEFORE destroying actors: in synchronous mode actor.destroy() blocks
        until a tick that nobody sends (10 s timeout each).
        """
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

    def _cleanup(self):
        if self._cleaned:
            return
        try:
            if self.world is not None:
                self._leave_sync_mode(self.world)
        except Exception:
            pass
        self._drop_actors()
        self._cleaned = True
        self._ego_ids.clear()
        _trace('CARLA bridge stopped; world back to asynchronous mode, actors destroyed.')

    def _cleanup_out_of_thread(self):
        """
        Clean up like _cleanup, with a fresh short-timeout client.

        Used when the tick thread is stuck.
        """
        if self._cleaned:
            return
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(5.0)
            world = client.get_world()
            self._leave_sync_mode(world)
            for aid in list(self._ego_ids):
                actor = world.get_actor(aid)
                if actor is not None:
                    try:
                        if hasattr(actor, 'stop'):
                            actor.stop()
                        actor.destroy()
                    except Exception:
                        pass
            self._cleaned = True
            self._ego_ids.clear()
            print(
                'CARLA bridge: server restored to asynchronous mode, actors destroyed.', flush=True
            )
        except Exception as ex:  # server unreachable: nothing more we can do
            print(f'CARLA bridge: could not clean up server ({ex})', flush=True)


def main(args=None):
    # Handle SIGINT/SIGTERM ourselves so shutdown always runs the CARLA cleanup, even while the
    # tick thread is blocked inside an RPC; rclpy's default handler would only stop the executor.
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

    node = CarlaBridgeNode()
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
    # the tick thread is a daemon; if a CARLA RPC is still hanging, do not wait on it
    os._exit(0)


if __name__ == '__main__':
    main()
