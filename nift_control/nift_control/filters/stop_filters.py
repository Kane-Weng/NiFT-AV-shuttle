"""
Safety filters that slow or stop the shuttle: distance stop and kinematic CBF.

File: stop_filters.py
Author: Kane Weng
Date: Apr 27, 2026

Description:
  Hard-stop safety filter implementations for the NiFT control stack.

  All speed decisions use _min_corridor_clearance, which ignores obstacles
  that are laterally beside the shuttle (|lat| > corridor_half).  This lets
  the vehicle pass parked objects without triggering a stop.
  _min_obstacle_distance is retained for last_obs_dist (telemetry) so the
  dashboard always reflects the nearest obstacle regardless of corridor.

  DistanceStopFilter
  ──────────────────
  Hard stop: zeroes velocity when any corridor obstacle is closer than
    safe_dist = min_safe_dist + max(safe_dist_gain * |v|, stopping_distance(|v|))
  stopping_distance uses the shuttle's measured braking (filter_utils), so the
  margin above min_safe_dist always covers a real stop.
  Includes hysteresis (RESUME_BUFFER = 0.5 m) to prevent oscillation at the
  threshold.  Steering passes through unchanged.
  Status: CLEAR ↔ BLOCKED

  KinematicCBFFilter
  ──────────────────
  Soft constraint: caps velocity via a Control Barrier Function
    v_safe = min(v_nom, gamma * h(x), braking_speed_cap(h(x)))
    where h(x) = corridor_dist - min_safe_dist
  gamma * h alone would ask for a deceleration of gamma * v (2 m/s² at 2 m/s),
  far more than the shuttle's brake gives. braking_speed_cap is the highest
  speed that still stops within h at the measured braking (filter_utils).
  Reports OBJECT_DETECTED while decelerating (v_safe > 0) and BLOCKED if the
  CBF bound reaches zero.  Steering passes through unchanged.

Usage:
  from .stop_filters import DistanceStopFilter, KinematicCBFFilter
"""
import numpy as np  # noqa: F401  (unused)

from .filter_base import FilterStatus, SafetyFilter
from .filter_utils import (
    _min_corridor_clearance,
    _min_obstacle_distance,
    BRAKE_DECEL_MPS2,
    BRAKE_DELAY_S,
    braking_speed_cap,
    stopping_distance,
)


class DistanceStopFilter(SafetyFilter):
    """
    Distance-based hard stop with velocity-scaled threshold and hysteresis.

    Halts the vehicle (v → 0) when any corridor obstacle is closer than:
        safe_dist = min_safe_dist + max(safe_dist_gain * |v|, stopping_distance(|v|))

    Resumes when the obstacle clears safe_dist + RESUME_BUFFER.
    Steering is passed through unchanged.
    """

    _RESUME_BUFFER = 0.5    # [m] extra clearance required before clearing BLOCKED

    def __init__(self, min_safe_dist: float = 2.0, safe_dist_gain: float = 1.5,
                 brake_decel: float = BRAKE_DECEL_MPS2, brake_delay: float = BRAKE_DELAY_S):
        """
        Store the filter parameters.

        Parameters
        ----------
        min_safe_dist
            Minimum clearance [m] from vehicle front face to obstacle.
        safe_dist_gain
            Speed multiplier [s] for dynamic stopping distance.
        brake_decel
            The shuttle's braking [m/s²] (filter_utils.BRAKE_DECEL_MPS2).
        brake_delay
            Time [s] from a brake command to the speed reacting.

        """
        self._min_safe_dist = min_safe_dist
        self._safe_dist_gain = safe_dist_gain
        self._brake_decel = brake_decel
        self._brake_delay = brake_delay
        self._filter_status = FilterStatus.CLEAR
        self._last_obs_dist = float('inf')

    @property
    def filter_status(self) -> FilterStatus:
        return self._filter_status

    @property
    def last_obs_dist(self) -> float:
        return self._last_obs_dist

    def filter(self, v_nom, delta_nom, state, obstacles):
        self._last_obs_dist = _min_obstacle_distance(state, obstacles)
        corridor_dist = _min_corridor_clearance(state, obstacles)

        # The speed margin covers at least a real stop at the measured braking
        safe_dist = self._min_safe_dist + max(
            self._safe_dist_gain * abs(state.vel),
            stopping_distance(state.vel, self._brake_decel, self._brake_delay))
        resume_dist = safe_dist + self._RESUME_BUFFER

        if corridor_dist < safe_dist:
            self._filter_status = FilterStatus.BLOCKED
        elif corridor_dist > resume_dist:
            self._filter_status = FilterStatus.CLEAR

        v_safe = 0.0 if self._filter_status == FilterStatus.BLOCKED else v_nom
        return v_safe, delta_nom


class KinematicCBFFilter(SafetyFilter):
    """
    Kinematic Control Barrier Function (CBF) Safety Filter.

    Smoothly bounds velocity as corridor obstacles are approached rather than
    cutting to zero abruptly.  Reports OBJECT_DETECTED while the CBF is
    actively limiting speed, and BLOCKED only if the bound reaches zero.
    Steering passes through unchanged.
    """

    def __init__(self, min_safe_dist: float = 2.0, gamma: float = 1.0,
                 brake_decel: float = BRAKE_DECEL_MPS2, brake_delay: float = BRAKE_DELAY_S):
        """
        Store the filter parameters.

        Parameters
        ----------
        min_safe_dist
            Minimum clearance [m] from vehicle front face to obstacle.
        gamma
            CBF gain.  Higher = later, harder braking.
            Lower = earlier, smoother deceleration.
        brake_decel
            The shuttle's braking [m/s²] (filter_utils.BRAKE_DECEL_MPS2).
        brake_delay
            Time [s] from a brake command to the speed reacting.

        """
        self._min_safe_dist = min_safe_dist
        self._gamma = gamma
        self._brake_decel = brake_decel
        self._brake_delay = brake_delay
        self._filter_status = FilterStatus.CLEAR
        self._last_obs_dist = float('inf')

    @property
    def filter_status(self) -> FilterStatus:
        return self._filter_status

    @property
    def last_obs_dist(self) -> float:
        return self._last_obs_dist

    def filter(self, v_nom, delta_nom, state, obstacles):
        self._last_obs_dist = _min_obstacle_distance(state, obstacles)
        corridor_dist = _min_corridor_clearance(state, obstacles)

        if corridor_dist == float('inf'):
            self._filter_status = FilterStatus.CLEAR
            return v_nom, delta_nom

        # CBF safety function h(x) = corridor_dist - min_safe_dist
        h_x = corridor_dist - self._min_safe_dist
        # gamma * h alone needs a deceleration of gamma * v. The braking cap keeps the
        # speed low enough for the shuttle to stop within h at its measured braking.
        max_allowed_vel = max(0.0, min(
            self._gamma * h_x, braking_speed_cap(h_x, self._brake_decel, self._brake_delay)))

        if v_nom > max_allowed_vel:
            v_safe = max_allowed_vel
            self._filter_status = (
                FilterStatus.BLOCKED if v_safe == 0.0 else FilterStatus.OBJECT_DETECTED
            )
        else:
            v_safe = v_nom
            self._filter_status = FilterStatus.CLEAR

        return v_safe, delta_nom
