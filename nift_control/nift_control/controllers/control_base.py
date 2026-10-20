"""
Interfaces for the pluggable lateral and longitudinal controllers.

File: control_base.py
Author: Kane Weng
Date: Apr 16, 2026

Description:
  Abstract base classes (interfaces) for all pluggable controllers in the
  NiFT control stack.

  LongitudinalController
    - Accepts (current_speed, target_speed, dt)
    - Returns a scalar control effort u ∈ [-255, 255] suitable for direct CAN actuation

  LateralController
    - Accepts a VehicleState and the current waypoint index
    - Returns (steering_angle [rad], lookahead_x, lookahead_y, updated_wp_idx)

Usage:
  from .control_base import LongitudinalController, LateralController
"""
from abc import ABC, abstractmethod


class LongitudinalController(ABC):
    @abstractmethod
    def compute(self, current_speed: float, target_speed: float, dt: float) -> float:
        pass

    @abstractmethod
    def reset(self):
        pass


class LateralController(ABC):
    @abstractmethod
    def compute(self, curr_x: float, curr_y: float, curr_yaw: float, curr_speed: float,
                local_path: list) -> tuple:
        pass

    @abstractmethod
    def reset(self):
        pass
