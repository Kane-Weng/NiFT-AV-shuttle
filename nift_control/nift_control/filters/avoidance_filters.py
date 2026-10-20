"""
Obstacle avoidance safety filter based on a potential field.

File: avoidance_filters.py
Author: Kane Weng
Date: Apr 27, 2026

Description:
  Object-avoidance safety filter implementations for the NiFT control stack.
  All classes subclass SafetyFilter from filter_base.py and use the shared
  helpers from filter_utils.py.

  PotentialFieldAvoidanceFilter
  ─────────────────────────────
  Reactive lateral evasion using Artificial Potential Fields.

  Steering: a repulsive offset proportional to how close the nearest forward
  obstacle is inside influence_radius is added to delta_nom.  The full forward
  obstacle set is used so the vehicle reacts to anything in its half-space,
  including obstacles that are laterally beside it.

  Speed: corridor-aware taper using _min_corridor_clearance.  Only obstacles
  inside the driving corridor penalise speed, so purely lateral objects trigger
  steering only with no deceleration.

    corridor_clearance >= influence_radius  → v_nom (no change)
    hard_stop_dist < corridor_clearance
                    < influence_radius      → v_nom * (clearance / influence_radius)
    corridor_clearance <= hard_stop_dist    → 0  (hard stop backstop)

  Status:
    CLEAR           — no obstacle within influence_radius
    OBJECT_DETECTED — obstacle present; steering or speed is being modified
    BLOCKED         — corridor obstacle inside hard_stop_dist; speed zeroed

Usage:
  from .avoidance_filters import PotentialFieldAvoidanceFilter
"""
import math

import numpy as np

from .filter_base import FilterStatus, SafetyFilter
from .filter_utils import _min_corridor_clearance, config, VEHICLE_RADIUS


class PotentialFieldAvoidanceFilter(SafetyFilter):
    """
    Lateral evasion filter.

    It steers around forward obstacles and tapers speed only for obstacles
    inside the driving corridor.
    """

    def __init__(
        self,
        influence_radius: float = 10.0,
        max_steer_offset: float = math.radians(15.0),
        hard_stop_dist: float = 2.0,
    ):
        """
        Store the filter parameters.

        Parameters
        ----------
        influence_radius
            Distance [m] at which repulsion and deceleration begin.
        max_steer_offset
            Maximum lateral steering offset added [rad].
        hard_stop_dist
            Corridor clearance [m] below which vehicle hard-stops.

        """
        self._influence_radius = influence_radius
        self._max_steer_offset = max_steer_offset
        self._hard_stop_dist = hard_stop_dist
        self._filter_status = FilterStatus.CLEAR
        self._last_obs_dist = float('inf')

    @property
    def filter_status(self) -> FilterStatus:
        return self._filter_status

    @property
    def last_obs_dist(self) -> float:
        return self._last_obs_dist

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def filter(self, v_nom, delta_nom, state, obstacles):
        if obstacles.size == 0:
            self._filter_status = FilterStatus.CLEAR
            return v_nom, delta_nom

        vehicle_pos = np.array([state.x, state.y])
        heading_vec = np.array([math.cos(state.yaw), math.sin(state.yaw)])
        vecs_to_obs = obstacles - vehicle_pos
        dists = np.linalg.norm(vecs_to_obs, axis=1)

        front_mask = np.dot(vecs_to_obs, heading_vec) > 0
        if not np.any(front_mask):
            self._filter_status = FilterStatus.CLEAR
            return v_nom, delta_nom

        valid_vecs = vecs_to_obs[front_mask]
        valid_dists = dists[front_mask]
        closest_idx = np.argmin(valid_dists)
        closest_vec = valid_vecs[closest_idx]
        self._last_obs_dist = max(0.0, valid_dists[closest_idx] - VEHICLE_RADIUS)

        if self._last_obs_dist > self._influence_radius:
            self._filter_status = FilterStatus.CLEAR
            return v_nom, delta_nom

        # ---- Speed: corridor-aware taper and hard stop ---- #
        corridor_clearance = _min_corridor_clearance(state, obstacles)
        if corridor_clearance <= self._hard_stop_dist:
            v_safe = 0.0
            self._filter_status = FilterStatus.BLOCKED
        elif corridor_clearance < self._influence_radius:
            speed_scale = min(1.0, corridor_clearance / self._influence_radius)
            v_safe = float(v_nom * speed_scale)
            self._filter_status = FilterStatus.OBJECT_DETECTED
        else:
            # Obstacle is in range for steering but outside corridor — no speed penalty
            v_safe = v_nom
            self._filter_status = FilterStatus.OBJECT_DETECTED

        # ---- Steering: repulsive lateral offset ---- #
        angle_to_obs = math.atan2(closest_vec[1], closest_vec[0])
        relative_angle = self._normalize_angle(angle_to_obs - state.yaw)

        steer_dir = -1.0 if relative_angle >= 0.0 else 1.0
        repulsion_strength = 1.0 - (self._last_obs_dist / self._influence_radius)
        steer_offset = steer_dir * self._max_steer_offset * repulsion_strength
        delta_safe = float(np.clip(
            delta_nom + steer_offset, -config.max_steering_angle, config.max_steering_angle
        ))

        return v_safe, delta_safe
