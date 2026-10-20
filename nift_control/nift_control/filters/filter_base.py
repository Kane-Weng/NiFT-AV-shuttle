"""
Interface and status enum for the pluggable safety filters.

File: filter_base.py
Author: Kane Weng
Date: Apr 15, 2026

Description:
  Abstract base class and status enum for all pluggable safety filters in the
  NiFT control stack.

  FilterStatus
  ────────────
  Three-valued enum reported by every filter each tick:
    CLEAR           — no obstacle within detection range; commands pass through
    OBJECT_DETECTED — obstacle present; filter is modifying commands (steering
                      offset or speed taper) but the vehicle is still moving
    BLOCKED         — corridor obstacle too close; vehicle speed zeroed

  SafetyFilter
  ────────────
  The SafetyFilter sits between the path planner's nominal command (v_nom, δ_nom)
  and the ADRC longitudinal controller.  Called at path-loop frequency (10 Hz),
  it must return a safe (v_safe, δ_safe) pair.

  Interface contract
  ──────────────────
  filter(v_nom, delta_nom, state, obstacles) → (v_safe, delta_safe)
    v_nom / v_safe    — longitudinal speed [m/s]
    delta_nom / safe  — steering angle [rad]
    state             — VehicleState (x, y, yaw, vel, x_rear, y_rear)
    obstacles         — Nx2 float64 array of obstacle points in map frame,
                        or an empty array when no obstacles are present

  filter_status — current FilterStatus value (abstract; implemented per subclass)
  is_blocked    — True when filter_status == BLOCKED (concrete, derived)
  last_obs_dist — most recent minimum obstacle clearance [m], for telemetry

  Concrete implementations live in stop_filters.py and avoidance_filters.py.
  PassThroughFilter (below, filter_type 'none') is the no-op one, for running
  without perception.

Usage:
  from .filter_base import SafetyFilter, FilterStatus, PassThroughFilter
"""

from abc import ABC, abstractmethod
from enum import Enum

from nift_common.vehicle import VehicleState
import numpy as np


class FilterStatus(Enum):
    CLEAR = 'CLEAR'
    OBJECT_DETECTED = 'OBJECT_DETECTED'
    BLOCKED = 'BLOCKED'


class SafetyFilter(ABC):
    """
    Interface for real-time safety filters on nominal control commands.

    Implementations receive a nominal (v, δ) from the 10 Hz path loop and
    return a safe (v, δ) for the 20 Hz actuation loop.
    """

    @property
    @abstractmethod
    def filter_status(self) -> FilterStatus:
        """Return the current obstacle awareness state of this filter."""
        pass

    @property
    def is_blocked(self) -> bool:
        """Return True when the filter has zeroed vehicle speed."""
        return self.filter_status == FilterStatus.BLOCKED

    @property
    @abstractmethod
    def last_obs_dist(self) -> float:
        """Most recent minimum obstacle distance [m] — used for telemetry."""
        pass

    @abstractmethod
    def filter(
        self,
        v_nom: float,
        delta_nom: float,
        state: VehicleState,
        obstacles: np.ndarray,
    ) -> tuple:
        pass


class PassThroughFilter(SafetyFilter):
    """
    Pass commands through unchanged (filter_type 'none').

    Commands pass unchanged and no obstacle is ever reported,
    so the collision-zone SAFE_STOP never fires either. system.launch.py selects it
    when `perception` is off; nothing then slows or stops the shuttle for obstacles.
    """

    @property
    def filter_status(self) -> FilterStatus:
        return FilterStatus.CLEAR

    @property
    def last_obs_dist(self) -> float:
        return float('inf')

    def filter(self, v_nom, delta_nom, state, obstacles) -> tuple:
        return v_nom, delta_nom
