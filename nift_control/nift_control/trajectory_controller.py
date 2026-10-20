#!/usr/bin/env python3
"""
Trajectory controller node: path tracking, safety filter and speed control for the shuttle.

File: trajectory_controller.py
Author: Kane Weng
Date: Apr 15, 2026

Description:
  Unified trajectory execution and vehicle actuation node for the NiFT shuttle.
  Combines lateral path tracking, a pluggable safety filter, and longitudinal
  speed control into a single node with two decoupled timer loops.

  Architecture
  ────────────
  Outer loop  (10 Hz)  _path_loop:
    Delegates to the active ControlModeHandler.path_loop().

  Inner loop  (20 Hz)  _actuate_loop:
    Delegates to the active ControlModeHandler.actuate_loop().
    Localization watchdog first, by localization_source:
      gnss    (default; shuttle_tf_node's TF) in AUTONOMOUS or MANUAL, no /gnss/vel for
              gnss_timeout_s → SAFE_STOP (pose comes from the latest TF and speed from
              the last /gnss/vel, both of which would otherwise freeze silently).
              /start_navigation and the switch to MANUAL are refused while GNSS is stale.
      filter  (localization_node's TF, which carries the pose through short GNSS gaps)
              speed from /localization/odom; in AUTONOMOUS or MANUAL, /localization/status
              LOST, or no status / odom for localization_status_timeout_s (the node died)
              → SAFE_STOP. DEGRADED (bridging a gap) does not stop: the target speed is
              capped at degraded_speed_cap_mps. /start_navigation and the switch to MANUAL
              need the status OK.
    SAFE_STOP keeps the steering it was commanding when it began (safe_stop_hold_steering,
    default on with filter, off with gnss: steering 0 as before), so the shuttle brakes on
    its arc instead of rolling out of a turn with the wheels straight.
    /obstacles watchdog (any safety filter but 'none'): in AUTONOMOUS, no usable
    /obstacles for obstacle_timeout_s → SAFE_STOP, since the filter would otherwise
    see an empty road; /start_navigation is refused while they are stale. A message
    counts once it is usable: a clear, or polygons moved into the map frame.

  Mode handlers
  ─────────────
  Each ControlMode (IDLE, AUTONOMOUS, MANUAL, ESTOP) has a dedicated handler in
  modes/control_modes.py.  All mode transitions go through _set_mode(), which calls
  handler.on_enter() for one-shot resets and stop outputs.  This file contains only
  ROS2 wiring — no per-mode branching logic.

  Safety filter
  ─────────────
  The SafetyFilter interface (filters/filter_base.py) decouples obstacle-handling
  from both the path planner and the ADRC controller.  filter_type 'none' passes
  commands through (system.launch.py uses it when perception is off).

  Lateral / longitudinal
  ──────────────────────
  lateral_enabled: false (system.launch.py without `lateral`) refuses
  /start_navigation, so only MANUAL is reachable: ADRC speed control from
  /teleop_cmd_vel, which needs /gnss/vel (filter: /localization/odom + status)
  but no path or TF.

  Simulation
  ──────────
  There is no sim mode: the node always drives through CAN frames on
  to_can_bus. In simulation nift_carla's virtual ECU decodes those frames,
  so the code that runs is the code that drives the shuttle.

Usage:
  ros2 run nift_control trajectory_controller
"""

import math

from can_msgs.msg import Frame
import diagnostic_msgs.msg
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
from geometry_msgs.msg import PolygonStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from nift_common.vehicle import VehicleState
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

from .controllers import (  # noqa: F401  (StanleyController: the commented-out alternative)
    ADRCController,
    PurePursuitController,
    StanleyController,
)
from .dbw_interface import NiftVehicleBridge, VehicleMode
from .filters import (
    DistanceStopFilter,
    FilterStatus,
    KinematicCBFFilter,
    PassThroughFilter,
    PotentialFieldAvoidanceFilter,
)
from .modes import ControlMode, HANDLERS

PATH_LOOP_HZ = 10      # Outer loop: lateral control + speed planning  [Hz]
ACTUATION_HZ = 20      # Inner loop: ADRC + CAN                        [Hz]

LOCALIZATION_SOURCES = ('gnss', 'filter')
# /localization/status level → state (localization_node: OK / WARN = DEGRADED / ERROR = LOST).
# Any other level (STALE, unknown) counts as LOST.
LOC_LEVELS = {0: 'OK', 1: 'DEGRADED', 2: 'LOST'}


def _level_int(level) -> int:
    r"""Return DiagnosticStatus.level as an int (rclpy gives the byte field as bytes, b'\x01')."""
    return level[0] if isinstance(level, (bytes, bytearray)) else int(level)


class TrajectoryController(Node):

    def __init__(self, **kwargs):
        # kwargs: Node options (tests: parameter_overrides)
        super().__init__('trajectory_controller', **kwargs)

        # ------------------------------------------------------------------ #
        #  Parameters                                                        #
        # ------------------------------------------------------------------ #
        self.declare_parameters(namespace='', parameters=[
            ('goal_tolerance', 1.5),
            ('pure_pursuit_ld_min', 4.0),
            ('pure_pursuit_ld_max', 8.0),
            ('base_speed', 3.5),
            ('cte_tolerance', 1.5),
            ('decel_distance', 6.0),
            ('min_crawl_speed', 0.5),
            ('min_safe_dist', 2.0),
            ('safe_dist_gain', 1.5),
            ('filter_type', 'cbf'),   # 'distance_stop' | 'cbf' | 'potential_field' | 'none'
            ('collision_zone_dist', 0.8),     # auto-trigger SAFE_STOP if obstacle closer than this
            ('gnss_timeout_s', 0.3),     # SAFE_STOP if no /gnss/vel this long while driving
            ('lateral_enabled', True),    # false: /start_navigation refused, MANUAL only
            # SAFE_STOP if no usable /obstacles this long in AUTONOMOUS
            ('obstacle_timeout_s', 0.5),
            # 'gnss' (watchdog + speed: /gnss/vel) | 'filter' (localization_node)
            ('localization_source', 'gnss'),
            # filter: SAFE_STOP if no /localization/status (or odom) this long
            ('localization_status_timeout_s', 0.3),
            # filter: target speed limit while the localization is DEGRADED
            ('degraded_speed_cap_mps', 1.0),
            # filter: cap once DEGRADED has lasted this long (0 = at once)
            ('degraded_cap_after_s', 0.0),
        ])
        self._localization_source = self.get_parameter('localization_source').value
        if self._localization_source not in LOCALIZATION_SOURCES:
            raise ValueError(f'localization_source "{self._localization_source}": '
                             f'expected one of {LOCALIZATION_SOURCES}')
        self._loc_filter = self._localization_source == 'filter'
        # SAFE_STOP keeps the steering it was commanding (default: on with the filter, which
        # dead-reckons the stop
        # on that arc; off with gnss = steering 0 as before)
        self.declare_parameter('safe_stop_hold_steering', self._loc_filter)

        self.declare_parameter('adrc.b0', 0.5)
        self.declare_parameter('adrc.w_cl', 2.0)
        self.declare_parameter('adrc.k_eso', 5.0)
        self.declare_parameter('vehicle_bridge.zero_speed_tolerance', 0.1)

        self._goal_tolerance = self.get_parameter('goal_tolerance').value
        self._ld_min = self.get_parameter('pure_pursuit_ld_min').value
        self._ld_max = self.get_parameter('pure_pursuit_ld_max').value
        self._base_speed = self.get_parameter('base_speed').value
        self._cte_tolerance = self.get_parameter('cte_tolerance').value
        self._decel_distance = self.get_parameter('decel_distance').value
        self._min_crawl_speed = self.get_parameter('min_crawl_speed').value
        self._collision_zone_dist = self.get_parameter('collision_zone_dist').value
        self._gnss_timeout_s = self.get_parameter('gnss_timeout_s').value
        self._lateral_enabled = self.get_parameter('lateral_enabled').value
        self._obstacle_timeout_s = self.get_parameter('obstacle_timeout_s').value
        self._loc_timeout_s = float(self.get_parameter('localization_status_timeout_s').value)
        self._degraded_cap = float(self.get_parameter('degraded_speed_cap_mps').value)
        self._degraded_cap_after_s = float(self.get_parameter('degraded_cap_after_s').value)
        self._safe_stop_hold_steering = bool(self.get_parameter('safe_stop_hold_steering').value)
        if (
            self._loc_timeout_s <= 0.0
            or self._degraded_cap < 0.0
            or self._degraded_cap_after_s < 0.0
        ):
            raise ValueError(
                'localization_status_timeout_s must be > 0, degraded_speed_cap_mps and '
                'degraded_cap_after_s >= 0'
            )
        min_safe_dist = self.get_parameter('min_safe_dist').value
        _safe_dist_gain = self.get_parameter('safe_dist_gain').value
        filter_type = self.get_parameter('filter_type').value
        b0 = self.get_parameter('adrc.b0').value
        w_cl = self.get_parameter('adrc.w_cl').value
        k_eso = self.get_parameter('adrc.k_eso').value
        zero_speed_tol = self.get_parameter('vehicle_bridge.zero_speed_tolerance').value

        # ------------------------------------------------------------------ #
        #  Controllers, bridge, safety filter                                #
        # ------------------------------------------------------------------ #
        self.lateral_ctrl = None   # Instantiated on first path receipt
        self.longitudinal_ctrl = ADRCController(
            delta=1.0 / ACTUATION_HZ, b0=b0, w_cl=w_cl, k_eso=k_eso
        )
        self.bridge = NiftVehicleBridge(zero_speed_tolerance=zero_speed_tol)

        _FILTERS = {
            'distance_stop': lambda: DistanceStopFilter(min_safe_dist, _safe_dist_gain),
            'cbf': lambda: KinematicCBFFilter(min_safe_dist=min_safe_dist, gamma=1.0),
            'potential_field': lambda: PotentialFieldAvoidanceFilter(),
            'none': lambda: PassThroughFilter(),
        }
        if filter_type not in _FILTERS:
            self.get_logger().warn(
                f'Unknown filter_type "{filter_type}", falling back to "cbf"'
            )
        self.safety_filter = _FILTERS.get(filter_type, _FILTERS['cbf'])()
        self._obstacle_watchdog = not isinstance(self.safety_filter, PassThroughFilter)
        if filter_type == 'none':
            self.get_logger().warn(
                'SAFETY FILTER OFF (filter_type none): '
                'nothing slows or stops the shuttle for obstacles'
            )
        if not self._lateral_enabled:
            self.get_logger().info(
                'Lateral control disabled: /start_navigation refused, MANUAL only'
            )
        if self._loc_filter:
            self.get_logger().info(
                'Localization source: filter (speed from /localization/odom; '
                'LOST or no /localization/status for '
                f'{self._loc_timeout_s:.2f} s -> SAFE STOP; '
                f'DEGRADED caps the speed at {self._degraded_cap:.2f} m/s)'
            )
        if self._safe_stop_hold_steering:
            self.get_logger().info(
                'SAFE STOP holds the steering it was commanding (safe_stop_hold_steering)'
            )

        # ------------------------------------------------------------------ #
        #  State                                                             #
        # ------------------------------------------------------------------ #
        self.state = VehicleState()   # {x, y, yaw, vel, x_rear, y_rear}
        self._waypoints = []
        self._current_wp_idx = 0
        self._obstacles = np.array([])
        self._obstacles_stamp = None           # header stamp of the lidar frame in _obstacles
        self._mode = ControlMode.IDLE

        # Nominal commands (from path loop) and safe commands (after safety filter)
        self._v_nom = 0.0
        self._delta_nom = 0.0
        self._v_safe = 0.0
        self._delta_safe = 0.0

        # Manual command buffers
        self._manual_v = 0.0
        self._manual_delta = 0.0

        self._idle_hold_brake = False   # IDLE re-sends a stop frame each tick after an active mode
        self._last_steer_deg = 0.0      # steering of the last drive / stop frame sent [deg]
        self._safe_stop_steer_deg = 0.0  # steering SAFE_STOP sends [deg] (set on entry)

        self._last_actuate_time = self.get_clock().now()
        self._path_period_s = 1.0 / PATH_LOOP_HZ
        self._last_gnss_time = None   # node-clock arrival of the last /gnss/vel
        self._last_obstacle_time = None  # node-clock arrival of the last usable /obstacles

        # localization_source filter: the last /localization/status and odom
        self._loc_level = None           # 'OK' | 'DEGRADED' | 'LOST' (None: no status yet)
        self._loc_message = ''           # its message, e.g. 'DEGRADED bridging'
        # its values (age_s = data age, dist_m = bridged distance, ...)
        self._loc_values = {}
        self._last_loc_status_time = None   # node-clock arrival of the last /localization/status
        self._last_loc_odom_time = None     # node-clock arrival of the last /localization/odom
        self._degraded_since = None      # node time the current DEGRADED spell began
        self._cap_bit_this_spell = False
        # degraded: DEGRADED spells that began while driving; cap_bites: spells in which the cap
        # lowered the
        # target speed; capped_s: time it did; stops: SAFE_STOPs from the localization watchdog
        self._loc_counts = {'degraded': 0, 'cap_bites': 0, 'capped_s': 0.0, 'stops': 0}

        # ------------------------------------------------------------------ #
        #  TF                                                                #
        # ------------------------------------------------------------------ #
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------ #
        #  Pubs / Subs                                                       #
        # ------------------------------------------------------------------ #
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.path_sub = self.create_subscription(
            Path, '/planned_path', self._path_callback, path_qos
        )
        self.vel_sub = self.create_subscription(
            TwistStamped, '/gnss/vel', self._velocity_callback, qos_profile_sensor_data
        )
        self.obs_sub = self.create_subscription(
            PolygonStamped, '/obstacles', self._obstacle_callback, 50
        )
        self.teleop_sub = self.create_subscription(
            Twist, '/teleop_cmd_vel', self._teleop_callback, 10
        )
        if self._loc_filter:
            self.loc_status_sub = self.create_subscription(
                DiagnosticStatus, '/localization/status', self._loc_status_callback, 10)
            self.loc_odom_sub = self.create_subscription(
                Odometry, '/localization/odom', self._loc_odom_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/waypoint_target_marker', 10)
        self.telemetry_pub = self.create_publisher(String, '/nav_telemetry', 10)
        self.can_pub = self.create_publisher(Frame, 'to_can_bus', 10)

        # ------------------------------------------------------------------ #
        #  Services                                                          #
        # ------------------------------------------------------------------ #
        self.srv = self.create_service(Trigger, 'start_navigation', self._start_nav_callback)
        self.srv_manual = self.create_service(
            Trigger, 'toggle_manual_mode', self._toggle_manual_callback
        )
        self.srv_estop = self.create_service(
            Trigger, 'trigger_estop', self._estop_service_callback
        )
        self.srv_resume = self.create_service(Trigger, 'resume', self._resume_callback)

        # ------------------------------------------------------------------ #
        #  Diagnostics                                                       #
        # ------------------------------------------------------------------ #
        self.diag_updater = diagnostic_updater.Updater(self)
        self.diag_updater.setHardwareID('control_stack')
        self.diag_updater.add('Trajectory Controller', self._diagnostics_callback)

        # ------------------------------------------------------------------ #
        #  Timers                                                            #
        # ------------------------------------------------------------------ #
        self.path_timer = self.create_timer(1.0 / PATH_LOOP_HZ, self._path_loop)
        self.actuate_timer = self.create_timer(1.0 / ACTUATION_HZ, self._actuate_loop)

        self.get_logger().info(
            "Trajectory Controller ready. Waiting for '/start_navigation' service call..."
        )

    # ====================================================================== #
    #  Mode management                                                       #
    # ====================================================================== #

    def _set_mode(self, new_mode: ControlMode) -> None:
        if new_mode == self._mode:
            return
        prev = self._mode
        self._mode = new_mode
        HANDLERS[new_mode].on_enter(self, prev)

    # ====================================================================== #
    #  Callbacks                                                             #
    # ====================================================================== #

    def _teleop_callback(self, msg: Twist):
        self._manual_v = msg.linear.x
        self._manual_delta = msg.angular.z

    def _velocity_callback(self, msg: TwistStamped):
        self._last_gnss_time = self.get_clock().now()
        if self._loc_filter:
            # speed from /localization/odom (carried through gaps); /gnss/vel's age is only shown
            return
        self._set_speed(msg.twist.linear.x)

    def _set_speed(self, speed: float):
        # Flip sign in reverse so ADRC error is always defined consistently
        if self.bridge.state == VehicleMode.REVERSE:
            self.state.vel = -abs(speed)
        else:
            self.state.vel = abs(speed)

    def _loc_odom_callback(self, msg: Odometry):
        """
        Take the speed for ADRC from the filter (localization_source filter).

        twist.linear.x is the filter's speed along base_link (signed by the gear it saw); the
        sign is set as for /gnss/vel (by the bridge state), so ADRC sees the same convention.
        """
        self._last_loc_odom_time = self.get_clock().now()
        self._set_speed(msg.twist.twist.linear.x)

    def _loc_status_callback(self, msg: DiagnosticStatus):
        now = self.get_clock().now()
        self._last_loc_status_time = now
        level = LOC_LEVELS.get(_level_int(msg.level), 'LOST')
        if level == 'DEGRADED' and self._loc_level != 'DEGRADED':
            self._degraded_since = now
            self._cap_bit_this_spell = False
            if self._mode in (ControlMode.AUTONOMOUS, ControlMode.MANUAL):
                self._loc_counts['degraded'] += 1
        elif level != 'DEGRADED':
            self._degraded_since = None
        self._loc_level = level
        self._loc_message = msg.message
        self._loc_values = {kv.key: kv.value for kv in msg.values}

    def _obstacle_callback(self, msg: PolygonStamped):
        """
        Transform obstacle polygon from base_link to map frame, store as Nx2 array.

        lidar_node publishes in base_link (tilt-corrected). Since base_link has no pitch/roll
        for a ground vehicle, a yaw-only 2D rigid transform to map is exact.
        lidar_node sends one polygon per obstacle, all stamped with the cloud's stamp: polygons
        with the same stamp are added together, a new stamp replaces them, an empty one clears.
        """
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if not msg.polygon.points:
            self._obstacles = np.array([])
            self._obstacles_stamp = stamp
            self._last_obstacle_time = self.get_clock().now()
            return

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF base_link→map not available: {ex}; dropping obstacle msg')
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        pts = np.array([[pt.x, pt.y] for pt in msg.polygon.points], dtype=np.float64)
        pts_map = np.column_stack([
            cos_y * pts[:, 0] - sin_y * pts[:, 1] + tx,
            sin_y * pts[:, 0] + cos_y * pts[:, 1] + ty,
        ])
        if stamp == self._obstacles_stamp and self._obstacles.size:
            self._obstacles = np.vstack([self._obstacles, pts_map])
        else:
            self._obstacles = pts_map
        self._obstacles_stamp = stamp
        self._last_obstacle_time = self.get_clock().now()

    def _path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            self._waypoints = []
            self.lateral_ctrl = None
            self._current_wp_idx = 0
            if self._mode == ControlMode.AUTONOMOUS:
                self._set_mode(ControlMode.IDLE)
            self.get_logger().warning(
                'Received /planned_path with fewer than 2 poses; cleared route'
            )
            return

        self._waypoints = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self._current_wp_idx = 0

        # ====================================================================#
        # ||||||||||||||||||  LATERAL CONTROLLER CHOICE  |||||||||||||||||||||#
        # ====================================================================#
        # self.lateral_ctrl = StanleyController(
        #     self._waypoints, k=0.5, v_min=0.5
        # )

        self.lateral_ctrl = PurePursuitController(
            self._waypoints, self._ld_min, self._ld_max, self._base_speed
        )
        # ====================================================================#
        # ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||#
        # ====================================================================#

        self.get_logger().info(f'New path loaded: {len(self._waypoints)} waypoints')

    def _start_nav_callback(self, _request, response):
        if self._mode == ControlMode.ESTOP:
            response.success = False
            response.message = 'Cannot start: Vehicle is E-STOPPED. Restart the node to recover.'
            return response
        if self._mode == ControlMode.SAFE_STOP:
            response.success = False
            response.message = 'Cannot start: Vehicle is in SAFE STOP. Call /resume first.'
            return response

        if not self._lateral_enabled:
            response.success = False
            response.message = 'Cannot start: lateral control is disabled (lateral_enabled false).'
            return response
        if len(self._waypoints) < 2:
            response.success = False
            response.message = 'Cannot start: no valid path loaded.'
            return response
        if self._loc_filter:
            problem = self._loc_not_ok()
            if problem:
                response.success = False
                response.message = f"Cannot start: localization not OK ({problem})."
                return response
        elif self._gnss_stale():
            response.success = False
            response.message = f"Cannot start: no fresh GNSS ({self._gnss_age_str()})."
            return response
        if self._obstacle_watchdog and self._obstacles_stale():
            response.success = False
            response.message = (
                f"Cannot start: no fresh /obstacles ({self._obstacles_age_str()}); "
                'perception down? (without perception: filter_type none)'
            )
            return response

        self._set_mode(ControlMode.AUTONOMOUS)
        self.get_logger().info('Navigation started.')
        response.success = True
        response.message = 'Navigation started successfully.'
        return response

    def _toggle_manual_callback(self, _request, response):
        if self._mode == ControlMode.ESTOP:
            response.success = False
            response.message = (
                'Cannot switch mode: Vehicle is E-STOPPED. Restart the node to recover.'
            )
            return response
        if self._mode == ControlMode.SAFE_STOP:
            response.success = False
            response.message = 'Cannot switch mode: Vehicle is in SAFE STOP. Call /resume first.'
            return response

        if self._mode == ControlMode.MANUAL:
            new_mode = ControlMode.IDLE
            msg = ('Switched to IDLE mode (Hit button to start navigation).')
        elif self._loc_filter and self._loc_not_ok():
            response.success = False
            response.message = (
                f"Cannot switch to MANUAL: localization not OK ({self._loc_not_ok()})."
            )
            return response
        elif not self._loc_filter and self._gnss_stale():
            response.success = False
            response.message = f"Cannot switch to MANUAL: no fresh GNSS ({self._gnss_age_str()})."
            return response
        else:
            new_mode = ControlMode.MANUAL
            msg = 'Switched to MANUAL mode.'

        self._set_mode(new_mode)
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    def _estop_service_callback(self, _request, response):
        self._request_safe_stop()
        response.success = True
        response.message = 'SAFE STOP engaged. Call /resume to return to IDLE.'
        return response

    def _resume_callback(self, _request, response):
        if self._mode != ControlMode.SAFE_STOP:
            response.success = False
            response.message = (
                f"Not in SAFE_STOP (currently {self._mode.name}); nothing to resume."
            )
            return response
        self._set_mode(ControlMode.IDLE)
        self.get_logger().info('Resumed from SAFE STOP. Returned to IDLE.')
        response.success = True
        response.message = 'Resumed. Vehicle returned to IDLE. Call /start_navigation when ready.'
        return response

    def _diagnostics_callback(self, stat):
        stat.add('Control Mode', self._mode.name)
        stat.add('Bridge State', str(self.bridge.state.name))
        stat.add('Active Waypoints', str(len(self._waypoints)))
        stat.add('Current WP IDX', str(self._current_wp_idx))
        stat.add('Filter Status', self.safety_filter.filter_status.value)
        stat.add('Nominal Speed', f"{self._v_nom:.2f} m/s")
        stat.add('Obs Distance', f"{self.safety_filter.last_obs_dist:.2f} m")
        stat.add('GNSS', self._gnss_age_str())
        stat.add(
            'Obstacles',
            self._obstacles_age_str() if self._obstacle_watchdog else 'filter none (not watched)',
        )
        if self._loc_filter:
            stat.add('Localization', self._loc_str())
            stat.add('Degraded Speed Cap', self._cap_str())
            stat.add('Localization Stops', str(self._loc_counts['stops']))
        if self._safe_stop_hold_steering:
            stat.add('Safe Stop Steering', f"held at {self._safe_stop_steer_deg:+.2f} deg"
                     if self._mode == ControlMode.SAFE_STOP else 'holds the last command')

        if self._mode == ControlMode.ESTOP:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.ERROR, 'E-STOPPED — node restart required'
            )
        elif self._mode == ControlMode.SAFE_STOP:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.WARN, 'SAFE STOP — call /resume to recover'
            )
        elif self._mode == ControlMode.AUTONOMOUS and len(self._waypoints) < 2:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.ERROR, 'Navigating but no valid path loaded!'
            )
        elif (
            self.safety_filter.filter_status != FilterStatus.CLEAR
            and self._mode == ControlMode.AUTONOMOUS
        ):
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.WARN,
                'Safety filter '
                f"{self.safety_filter.filter_status.value.lower().replace('_', ' ')}",
            )
        elif (self._loc_filter and self._loc_level == 'DEGRADED'
              and self._mode in (ControlMode.AUTONOMOUS, ControlMode.MANUAL)):
            capped = self._speed_cap() is not None
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'Localization DEGRADED' + (
                f": speed capped at {self._degraded_cap:.2f} m/s" if capped else ''))
        elif self._mode == ControlMode.AUTONOMOUS:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Actively following route')
        elif self._mode == ControlMode.MANUAL:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Manual Control Active')
        elif len(self._waypoints) >= 2:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.OK,
                'Route loaded, waiting for start_navigation',
            )
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Idle: waiting for goal')
        return stat

    # ====================================================================== #
    #  Outer loop — 10 Hz                                                    #
    # ====================================================================== #

    def _path_loop(self):
        HANDLERS[self._mode].path_loop(self)

    # ====================================================================== #
    #  Inner loop — 20 Hz                                                    #
    # ====================================================================== #

    def _actuate_loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_actuate_time).nanoseconds / 1e9
        self._last_actuate_time = now
        # GNSS watchdog: pose (latest TF) and speed would otherwise freeze at their last values
        if (
            self._mode in (ControlMode.AUTONOMOUS, ControlMode.MANUAL)
            and not self._loc_filter
            and self._gnss_stale()
        ):
            self.get_logger().error(
                f"GNSS lost while {self._mode.name} ({self._gnss_age_str()}) -> SAFE STOP")
            self._request_safe_stop()
        # Localization watchdog (filter): LOST, or localization_node silent (its TF and odom would
        # freeze)
        if self._mode in (ControlMode.AUTONOMOUS, ControlMode.MANUAL) and self._loc_filter:
            problem = self._loc_unusable()
            if problem:
                self.get_logger().error(
                    f"Localization unusable while {self._mode.name} ({problem}) -> SAFE STOP"
                )
                self._loc_counts['stops'] += 1
                self._request_safe_stop()
        # /obstacles watchdog: the safety filter would otherwise keep using the last (or no)
        # obstacles
        if (
            self._mode == ControlMode.AUTONOMOUS
            and self._obstacle_watchdog
            and self._obstacles_stale()
        ):
            self.get_logger().error(
                f"Perception lost while AUTONOMOUS ({self._obstacles_age_str()}) -> SAFE STOP"
            )
            self._request_safe_stop()
        HANDLERS[self._mode].actuate_loop(self, dt, now)

    # ====================================================================== #
    #  Helpers                                                               #
    # ====================================================================== #

    def _request_safe_stop(self):
        self._set_mode(ControlMode.SAFE_STOP)

    def _gnss_age(self):
        """Seconds since the last /gnss/vel arrived (node clock), None if none yet."""
        if self._last_gnss_time is None:
            return None
        return (self.get_clock().now() - self._last_gnss_time).nanoseconds / 1e9

    def _gnss_stale(self) -> bool:
        age = self._gnss_age()
        return age is None or age > self._gnss_timeout_s

    def _gnss_age_str(self) -> str:
        age = self._gnss_age()
        return 'no /gnss/vel received yet' if age is None else f"last /gnss/vel {age:.2f} s ago"

    def _obstacles_age(self):
        """Seconds since the last usable /obstacles (node clock), None if none yet."""
        if self._last_obstacle_time is None:
            return None
        return (self.get_clock().now() - self._last_obstacle_time).nanoseconds / 1e9

    def _obstacles_stale(self) -> bool:
        age = self._obstacles_age()
        return age is None or age > self._obstacle_timeout_s

    def _obstacles_age_str(self) -> str:
        age = self._obstacles_age()
        return (
            'no usable /obstacles yet'
            if age is None
            else f"last usable /obstacles {age:.2f} s ago"
        )

    # ---- localization_source filter ---- #

    def _age_of(self, t):
        """Seconds since node time t (node clock), None if t is None."""
        return None if t is None else (self.get_clock().now() - t).nanoseconds / 1e9

    def _loc_unusable(self) -> str:
        """
        Say why the filter's pose / speed must not be driven on ('' = they may).

        Reasons: no fresh /localization/status, status LOST (or an unknown level), or no fresh
        /localization/odom. DEGRADED is usable.
        """
        age = self._age_of(self._last_loc_status_time)
        if age is None:
            return 'no /localization/status received yet'
        if age > self._loc_timeout_s:
            return f"last /localization/status {age:.2f} s ago: localization_node down?"
        if self._loc_level == 'LOST':
            return self._loc_detail()
        age = self._age_of(self._last_loc_odom_time)
        if age is None:
            return 'no /localization/odom received yet'
        if age > self._loc_timeout_s:
            return f"last /localization/odom {age:.2f} s ago"
        return ''

    def _loc_not_ok(self) -> str:
        """Why the localization is not OK ('' = OK): starting and switching to MANUAL need OK."""
        problem = self._loc_unusable()
        if problem or self._loc_level == 'OK':
            return problem
        return self._loc_detail() + '; wait for OK'

    def _loc_detail(self) -> str:
        """Return the last status as text, e.g. 'DEGRADED bridging, data 0.620 s old'."""
        level, msg = self._loc_level or 'LOST', self._loc_message.strip()
        text = msg if msg.startswith(level) else f"{level} {msg}".strip()
        if 'age_s' in self._loc_values:
            text += f", data {self._loc_values['age_s']} s old"
        if level != 'OK' and 'dist_m' in self._loc_values:
            text += f", {self._loc_values['dist_m']} m bridged"
        return text

    def _loc_str(self) -> str:
        age = self._age_of(self._last_loc_status_time)
        if age is None:
            return 'no /localization/status received yet'
        return f"{self._loc_detail()} (status {age:.2f} s ago)"

    def _speed_cap(self):
        """
        Return the speed limit [m/s] while the localization is DEGRADED.

        The limit applies once DEGRADED has lasted degraded_cap_after_s. None when there is
        none (always None with gnss).
        """
        if not self._loc_filter or self._loc_level != 'DEGRADED' or self._degraded_since is None:
            return None
        if (
            self._degraded_cap_after_s > 0.0
            and 0.0 <= self._age_of(self._degraded_since) < self._degraded_cap_after_s
        ):
            return None
        return self._degraded_cap

    def _limit_speed(self, v: float, dt: float) -> float:
        """
        Return v with the DEGRADED cap on its magnitude (sign kept: MANUAL reverses).

        Counts the bites. dt = the calling loop's period [s], for the time capped.
        """
        cap = self._speed_cap()
        if cap is None or abs(v) <= cap:
            return v
        if not self._cap_bit_this_spell:
            self._cap_bit_this_spell = True
            self._loc_counts['cap_bites'] += 1
        self._loc_counts['capped_s'] += dt
        return math.copysign(cap, v)

    def _cap_str(self) -> str:
        c = self._loc_counts
        active = ' (capping)' if self._speed_cap() is not None else ''
        return (
            f"{self._degraded_cap:.2f} m/s{active}; bit in {c['cap_bites']} of {c['degraded']} "
            'DEGRADED '
            f"spells while driving, {c['capped_s']:.1f} s capped"
        )

    def _loc_short(self) -> str:
        """
        Return one telemetry word or three.

        E.g. 'OK', 'DEGRADED 0.620 s (cap 1.0)', 'LOST 1.520 s', 'no status'.
        """
        age = self._age_of(self._last_loc_status_time)
        if age is None:
            return 'no status'
        if age > self._loc_timeout_s:
            return f"no status for {age:.1f} s"
        text = self._loc_level
        if self._loc_level != 'OK' and 'age_s' in self._loc_values:
            text += f" {self._loc_values['age_s']} s"
        if self._speed_cap() is not None:
            text += f" (cap {self._degraded_cap:.1f})"
        return text

    def _get_robot_pose(self) -> bool:
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.state.x = trans.transform.translation.x
            self.state.y = trans.transform.translation.y
            q = trans.transform.rotation
            self.state.yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            return True
        except TransformException as ex:
            self.get_logger().info(f'TF map → base_link unavailable: {ex}')
            return False

    def _publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.SPHERE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5
        marker.scale.x = marker.scale.y = marker.scale.z = 0.6
        marker.color.a, marker.color.r = 1.0, 1.0
        self.marker_pub.publish(marker)

    def _publish_telemetry(self, status, target_v, dist_end, cte, manual: bool = False):
        msg = String(data=(
            f"STATUS:    {status}\n"
            f"Target V:  {target_v:.2f} m/s\n"
            f"Curr V:    {self.state.vel:.2f} m/s\n"
            f"Obs Dist:  {self.safety_filter.last_obs_dist:.2f} m\n"
            f"Filter:    {self.safety_filter.filter_status.value}\n"
            f"Dist End:  {dist_end:.2f} m\n"
            f"CTE:       {cte:.2f} m\n"
            f"Next WP:   {self._current_wp_idx + 1} / {len(self._waypoints)}"
        ))

        if (manual):
            msg = String(data=(
                f"STATUS:    {status}\n"
                f"Target V:  {target_v:.2f} m/s\n"
                f"Curr V:    {self.state.vel:.2f} m/s\n"
                f"Obs Dist:  --- m\n"
                f"Filter:    ---\n"
                f"Dist End:  --- m\n"
                f"CTE:       --- m\n"
                f"Next WP:   ---"
            ))
        if self._loc_filter:
            msg.data += f"\nLoc:       {self._loc_short()}"

        self.telemetry_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryController()
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
        rclpy.try_shutdown()    # Ctrl-C: rclpy's SIGINT handler has already shut the context down


if __name__ == '__main__':
    main()
