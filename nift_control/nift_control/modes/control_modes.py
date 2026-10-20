"""
One handler per operating mode of the trajectory controller.

File: control_modes.py
Author: Kane Weng
Date: Apr 25, 2026

Description:
  Concrete ControlModeHandler implementations for the NiFT trajectory controller.
  All four handlers subclass ControlModeHandler and are registered in HANDLERS,
  a module-level dict keyed by ControlMode. TrajectoryController imports this
  dict and delegates both timer callbacks through it, so there is no per-mode
  branching logic in the main node file.

  _IdleHandler
  ────────────
    - on_enter zeros all command state
    - arriving from an active mode (AUTONOMOUS, MANUAL, SAFE_STOP): resets the
      longitudinal controller & bridge, then holds the vehicle — a stop frame
      (NEUTRAL, full brake) every actuation tick, like SAFE_STOP, so the stop
      never depends on one frame or on the DBW keeping its last command.
    - at startup (never active) it sends nothing.

  _EstopHandler
  ─────────────
    - Does nothing in both timer loops.
    - on_enter immediately publishes a CAN E-stop frame
    - resets all controllers, and logs at ERROR level
    - Terminal safe state (only a node restart can clear it)

  _AutonomousHandler
  ──────────────────
  path_loop:
    - reads pose from TF
    - runs active lateral controller to compute (v_nom, δ_nom)
    - applies CTE-based & steering-limit speed scaling (ramps down near the goal)
    - passes result through the active SafetyFilter
    - stores (v_safe, δ_safe) for the inner loop.

  actuate_loop:
    - feeds v_safe into the ADRC longitudinal controller
    - maps the effort through the NiftVehicleBridge FSM
    - publishes a CAN frame.

  _ManualHandler
  ──────────────
  path_loop:
    - Safety filter intentionally bypassed (manual is an operator override)

  actuate_loop:
    - routes teleop commands through ADRC and the bridge FSM to CAN.

  _SafeStopHandler
  ────────────────
    - NEUTRAL + full brake every actuation tick until /resume (→ IDLE)
    - steering: 0, or with safe_stop_hold_steering the angle of the last frame
      sent before the stop, held for the whole stop (also at standstill), so
      the shuttle brakes on the arc it was driving. Every SAFE_STOP trigger
      (watchdogs, collision zone, /trigger_estop) gets the same rule.

  Localization DEGRADED (localization_source filter): AUTONOMOUS's target speed
  and MANUAL's commanded speed are capped by node._limit_speed() (a no-op with gnss).

  HANDLERS
  ────────
  Module-level registry mapping each ControlMode to its singleton handler instance.
  Import this dict to wire a TrajectoryController to the mode system.

Usage:
  from .control_modes import HANDLERS
"""

import math

from nift_common.can_protocol import CAN_CMD_MAX, STEER_MAX_DEG  # full brake, steering limit
from nift_common.vehicle import ShuttleConfig
import numpy as np

from ..dbw_interface import create_command_frame, create_estop_frame, Gear
from ..filters import FilterStatus
from ..filters.filter_utils import _min_obstacle_distance
from .mode_base import ControlMode, ControlModeHandler

config = ShuttleConfig()


# ─── Concrete handlers ──────────────────────────────────────────────────────── #

class _IdleHandler(ControlModeHandler):
    """
    Parked between missions.

    - on_enter zeros all command state
    - arriving from an active mode: resets longitudinal controller & bridge and
      starts holding the brake (stop frame now and every actuation tick)
    - at startup (never active): sends nothing
    """

    def on_enter(self, node, prev_mode: ControlMode) -> None:
        node._v_nom = node._delta_nom = 0.0
        node._v_safe = node._delta_safe = 0.0
        node._idle_hold_brake = prev_mode in (
            ControlMode.AUTONOMOUS, ControlMode.MANUAL, ControlMode.SAFE_STOP)
        if not node._idle_hold_brake:
            return
        node.longitudinal_ctrl.reset()
        node.bridge.reset()
        now = node.get_clock().now().to_msg()
        node._publish_telemetry('IDLE', 0.0, 0.0, 0.0)
        node._last_steer_deg = 0.0
        node.can_pub.publish(create_command_frame(
            Gear.NEUTRAL, CAN_CMD_MAX, 0, 0.0, 'stop', time_msg=now,
        ))

    def path_loop(self, _node) -> None:
        pass

    def actuate_loop(self, node, _dt: float, now) -> None:
        # Hold the stop (e.g. after reaching the goal at speed) instead of going silent after one
        # frame: the DBW's behaviour without commands is not specified.
        if node._idle_hold_brake:
            node._last_steer_deg = 0.0
            node.can_pub.publish(create_command_frame(
                Gear.NEUTRAL, CAN_CMD_MAX, 0, 0.0, 'stop', time_msg=now.to_msg(),
            ))


class _EstopHandler(ControlModeHandler):
    """
    Does nothing in both timer loops.

    - on_enter immediately publishes a CAN E-stop frame
    - resets all controllers, and logs at ERROR level
    - Terminal safe state (only a node restart can clear it)
    """

    def on_enter(self, node, _prev_mode: ControlMode) -> None:
        node.longitudinal_ctrl.reset()
        node.bridge.reset()
        node.can_pub.publish(create_estop_frame(
            time_msg=node.get_clock().now().to_msg(),
        ))
        node._publish_telemetry('ESTOP', 0.0, 0.0, 0.0)
        node.get_logger().error('E-STOP TRIGGERED! All automatic and manual operations halted.')

    def path_loop(self, _node) -> None:
        pass

    def actuate_loop(self, _node, _dt: float, _now) -> None:
        pass


class _AutonomousHandler(ControlModeHandler):
    """
    Autonomous Navigation Mode.

    path_loop:
    - reads pose from TF
    - runs active lateral controller to compute (v_nom, δ_nom)
    - applies CTE-based & steering-limit speed scaling (ramps down near the goal)
    - passes result through the active SafetyFilter
    - stores (v_safe, δ_safe) for the inner loop.

    actuate_loop:
    - feeds v_safe into the ADRC longitudinal controller
    - maps the effort through the NiftVehicleBridge FSM
    - publishes a CAN frame.
    """

    def on_enter(self, node, _prev_mode: ControlMode) -> None:
        node.longitudinal_ctrl.reset()
        node.bridge.reset()

    def path_loop(self, node) -> None:
        if not node._get_robot_pose():
            return
        if len(node._waypoints) < 2 or node.lateral_ctrl is None:
            return

        node.state.x_rear = node.state.x - config.base_link_x * math.cos(node.state.yaw)
        node.state.y_rear = node.state.y - config.base_link_x * math.sin(node.state.yaw)

        base_speed = node._base_speed

        # ---- Goal check + deceleration ramp ---- #
        last_wp = node._waypoints[-1]
        dist_final = math.hypot(
            last_wp[0] - node.state.x_rear, last_wp[1] - node.state.y_rear
        )

        if dist_final < node._goal_tolerance:
            node.get_logger().info('Arrived at destination.')
            node._current_wp_idx = 0
            node._set_mode(ControlMode.IDLE)
            return

        if dist_final < node._decel_distance:
            decel_ratio = np.clip(
                (dist_final - node._goal_tolerance)
                / (node._decel_distance - node._goal_tolerance),
                0.0, 1.0,
            )
            base_speed = (node._min_crawl_speed
                          + (node._base_speed - node._min_crawl_speed) * decel_ratio)

        # ---- Pure Pursuit: lateral steering ---- #
        steering_angle, tx, ty, node._current_wp_idx = node.lateral_ctrl.compute(
            node.state, node._current_wp_idx
        )
        node._publish_marker(tx, ty)

        # ---- CTE-based speed scaling ---- #
        prev_idx = max(node._current_wp_idx - 1, 0)
        p1 = np.array(node._waypoints[prev_idx])
        p2 = np.array(node._waypoints[node._current_wp_idx])
        seg_len = np.linalg.norm(p2 - p1)
        d = p2 - p1
        r = p1 - np.array([node.state.x_rear, node.state.y_rear])
        cte = 0.0 if seg_len < 1e-6 else abs(d[0] * r[1] - d[1] * r[0]) / seg_len

        speed_factor = np.clip(1.0 - (cte / node._cte_tolerance), 0.5, 1.0)
        steer_limit_factor = 1.0 - (abs(steering_angle) / config.max_steering_angle) * 0.4
        target_speed = float(np.clip(
            base_speed * speed_factor * steer_limit_factor, 0.0, config.max_speed
        ))
        target_steer = float(np.clip(
            steering_angle, -config.max_steering_angle, config.max_steering_angle
        ))
        # Localization DEGRADED (bridging a GNSS gap): speed cap
        # (localization_source filter; none with gnss)
        target_speed = node._limit_speed(target_speed, node._path_period_s)

        node._v_nom = target_speed
        node._delta_nom = target_steer

        # ---- Safety filter ---- #
        node._v_safe, node._delta_safe = node.safety_filter.filter(
            target_speed, target_steer, node.state, node._obstacles
        )

        filter_status = node.safety_filter.filter_status
        if filter_status != FilterStatus.CLEAR:
            node.get_logger().warn(
                f"[{filter_status.value}] Obstacle at {node.safety_filter.last_obs_dist:.2f} m",
                throttle_duration_sec=1.0,
            )

        # Auto-trigger SAFE_STOP if something penetrates the collision zone.
        # The filter already stops the vehicle at hard_stop_dist; this escalates
        # to a human-reviewed SAFE_STOP if an obstacle gets dangerously close despite that.
        if node.safety_filter.last_obs_dist < node._collision_zone_dist:
            node.get_logger().error(
                f"Obstacle within collision zone ({node.safety_filter.last_obs_dist:.2f} m "
                f"< {node._collision_zone_dist:.2f} m) — triggering SAFE STOP"
            )
            node._request_safe_stop()
            return

        node._publish_telemetry('NAVIGATING', node._v_safe, dist_final, cte)

    def actuate_loop(self, node, dt: float, now) -> None:
        u = node.longitudinal_ctrl.compute(node.state.vel, node._v_safe, dt)
        gear, brake, throttle = node.bridge.map_signals(u, node._v_safe, node.state.vel)
        node._last_steer_deg = math.degrees(node._delta_safe)
        node.can_pub.publish(create_command_frame(
            gear, brake, throttle, node._last_steer_deg, 'active',
            time_msg=now.to_msg(),
        ))


class _SafeStopHandler(ControlModeHandler):
    """
    Recoverable software stop.

    - on_enter: zeros commands, sends one stop CAN frame, logs WARN
    - path_loop: publishes telemetry only (keeps dashboard live)
    - actuate_loop: periodically re-sends brake frame to hold vehicle stopped
    - Exited via /resume service → IDLE (then /start_navigation as normal)
    - Steering: 0, or (safe_stop_hold_steering) the angle of the last frame sent before
      the stop, fixed on entry and held until /resume, also at standstill: the shuttle
      brakes on its arc (the wheels only straighten in IDLE's stop frame after /resume).
      A non-finite last angle holds 0.
    """

    def on_enter(self, node, _prev_mode: ControlMode) -> None:
        node._v_nom = node._delta_nom = 0.0
        node._v_safe = node._delta_safe = 0.0
        held = node._last_steer_deg if node._safe_stop_hold_steering else 0.0
        node._safe_stop_steer_deg = (float(np.clip(held, -STEER_MAX_DEG, STEER_MAX_DEG))
                                     if math.isfinite(held) else 0.0)
        node._last_steer_deg = node._safe_stop_steer_deg
        node.longitudinal_ctrl.reset()
        node.bridge.reset()
        now = node.get_clock().now().to_msg()
        node._publish_telemetry('SAFE_STOP', 0.0, 0.0, 0.0)
        node.can_pub.publish(create_command_frame(
            Gear.NEUTRAL, CAN_CMD_MAX, 0, node._safe_stop_steer_deg, 'safe_stop', time_msg=now,
        ))
        node.get_logger().warn(
            'SAFE STOP engaged. Vehicle held. Call /resume to return to IDLE.'
            + (f' Steering held at {node._safe_stop_steer_deg:+.2f} deg.'
               if node._safe_stop_hold_steering else '')
        )

    def path_loop(self, node) -> None:
        node._publish_telemetry('SAFE_STOP', 0.0, 0.0, 0.0)

    def actuate_loop(self, node, _dt: float, now) -> None:
        # Periodically re-assert brakes so vehicle stays stopped even if CAN glitches
        node.can_pub.publish(create_command_frame(
            Gear.NEUTRAL, CAN_CMD_MAX, 0, node._safe_stop_steer_deg, 'safe_stop',
            time_msg=now.to_msg(),
        ))


class _ManualHandler(ControlModeHandler):
    """
    Manual Navigation Mode.

    path_loop:
    - Safety filter intentionally bypassed (manual is an operator override)

    actuate_loop:
    - routes teleop commands through ADRC and the bridge FSM to CAN.
    """

    def on_enter(self, node, _prev_mode: ControlMode) -> None:
        # Zero buffers so the vehicle doesn't jump on stale teleop data
        node._manual_v = 0.0
        node._manual_delta = 0.0
        node.longitudinal_ctrl.reset()
        node.bridge.reset()

    def path_loop(self, node) -> None:
        # Hard interlock: trigger SAFE_STOP if an obstacle enters the collision zone.
        # The full safety filter is still bypassed — this is purely a physical limit.
        # Obstacles are stored in map frame, so we need a fresh pose for the check.
        if node._obstacles.size > 0 and node._get_robot_pose():
            edge_dist = _min_obstacle_distance(node.state, node._obstacles)
            if edge_dist < node._collision_zone_dist:
                node.get_logger().error(
                    f"[MANUAL] Obstacle within collision zone ({edge_dist:.2f} m "
                    f"< {node._collision_zone_dist:.2f} m) — triggering SAFE STOP"
                )
                node._request_safe_stop()
                return

        # Safety filter output intentionally bypassed — manual is an operator override
        node._publish_telemetry('MANUAL', node._manual_v, 0.0, 0.0, manual=True)

    def actuate_loop(self, node, dt: float, now) -> None:
        # localization DEGRADED cap (filter; unchanged with gnss)
        v_ref = node._limit_speed(node._manual_v, dt)
        u = node.longitudinal_ctrl.compute(node.state.vel, v_ref, dt)
        gear, brake, throttle = node.bridge.map_signals(u, v_ref, node.state.vel)
        node._last_steer_deg = math.degrees(node._manual_delta)
        node.can_pub.publish(create_command_frame(
            gear, brake, throttle, node._last_steer_deg, 'active',
            time_msg=now.to_msg(),
        ))


HANDLERS: dict[ControlMode, ControlModeHandler] = {
    ControlMode.IDLE: _IdleHandler(),
    ControlMode.AUTONOMOUS: _AutonomousHandler(),
    ControlMode.MANUAL: _ManualHandler(),
    ControlMode.ESTOP: _EstopHandler(),
    ControlMode.SAFE_STOP: _SafeStopHandler(),
}
