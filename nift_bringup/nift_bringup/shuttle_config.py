#!/usr/bin/env python3
"""
Vehicle constants and the shared robot state for the NiFT shuttle.

File: shuttle_config.py
Author: Kane Weng
Date: Apr 2, 2026

Description:
  - ShuttleConfig: Defines the static physical, geometric, and kinematic properties of the shuttle.
  - VehicleState: Defines the robot state used for trajectory_controller.py mainly.

Usage:
  from nift_bringup.shuttle_config import ShuttleConfig

  config = ShuttleConfig()
  print(f"Vehicle Wheelbase: {config.wheel_base} m")
  print(f"Max Steering Angle: {config.max_steering_angle} rad")
"""
from dataclasses import dataclass


@dataclass(frozen=True)  # Read-only
class ShuttleConfig:
    """Static parameters for the NiFT shuttle."""

    # Geometric Parameters [m]
    wheel_radius: float = 0.228
    wheel_width: float = 0.213
    wheel_base: float = 2.082
    wheel_tread: float = 0.73533
    front_overhang: float = 0.4830572
    rear_overhang: float = 0.48977042
    height: float = 1.905
    length: float = 3.055
    width: float = 1.588

    # Kinematic and Dynamic Parameters
    max_steering_angle: float = 0.2618    # [rad] (~15°)
    max_speed: float = 4.91744            # [m/s]
    max_accel: float = 0.8                # [m/s²]
    max_decel: float = 7.0                # [m/s²]
    mass: float = 500.0                   # [kg]
    cog_height: float = 0.658             # [m]


# Robot state
@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vel: float = 0.0
    x_rear: float = 0.0
    y_rear: float = 0.0
