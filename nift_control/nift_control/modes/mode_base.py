"""
Operating mode enum and handler interface for the trajectory controller.

File: mode_base.py
Author: Kane Weng
Date: Apr 25, 2026

Description:
  Abstract base class and operating-mode enum for the NiFT trajectory
  controller's pluggable mode system.

  ControlMode
  ───────────
  Enum defining the five mutually-exclusive operating states of the vehicle:
    IDLE       — standstill, no active commands; waiting for a navigation request
    AUTONOMOUS — following a loaded path under full closed-loop control
    MANUAL     — operator in direct control via teleop; safety filter bypassed
    ESTOP      — hardware/catastrophic emergency stop; all motion suppressed until node restart
    SAFE_STOP  — recoverable software stop; call /resume service to return to IDLE

  ControlModeHandler
  ──────────────────
  ABC that every concrete mode handler must satisfy.  TrajectoryController
  holds one handler per ControlMode in the HANDLERS registry and delegates
  its two timer callbacks through the active handler:

    on_enter(node, prev_mode)   — called once by _set_mode() on every transition;
                                  use for one-shot outputs (stop frame) and state resets
    path_loop(node)             — called at 10 Hz; owns lateral control, safety filter,
                                  and sim-mode output for this mode
    actuate_loop(node, dt, now) — called at 20 Hz (real-world only); owns ADRC,
                                  bridge FSM, and CAN output for this mode

  The node argument is the live TrajectoryController instance.  It is left
  unannotated to avoid a circular import; handlers access node state and
  helpers directly via attribute access.

Usage:
  from .mode_base import ControlMode, ControlModeHandler
"""

from abc import ABC, abstractmethod
from enum import Enum


class ControlMode(Enum):
    IDLE = 0
    AUTONOMOUS = 1
    MANUAL = 2
    ESTOP = 3
    SAFE_STOP = 4   # recoverable software stop — call /resume to return to IDLE


class ControlModeHandler(ABC):
    """Interface for a single operating mode of the trajectory controller."""

    @abstractmethod
    def on_enter(self, node, prev_mode: ControlMode) -> None:
        """One-shot setup called by _set_mode() on every inbound transition."""
        pass

    @abstractmethod
    def path_loop(self, node) -> None:
        """10 Hz outer-loop body for this mode."""
        pass

    @abstractmethod
    def actuate_loop(self, node, dt: float, now) -> None:
        """20 Hz inner-loop body for this mode (real-world only)."""
        pass
