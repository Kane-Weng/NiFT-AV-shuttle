"""
The shuttle's geometry, limits and sensor mounts as a frozen dataclass, read from vehicle.yaml.

File: vehicle.py

Description:
  The shuttle's physical facts for Python, read from vehicle.yaml next to this file (the same
  file the URDF loads).
  - ShuttleConfig: geometry, kinematic limits, sensor mounts, plus a few derived values.
  - VehicleState: the robot state the controller works with.

Usage:
  from nift_common.vehicle import ShuttleConfig

  shuttle = ShuttleConfig()
  shuttle.wheel_base, shuttle.max_steering_angle, shuttle.min_turn_radius
"""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

YAML_PATH = Path(__file__).with_name('vehicle.yaml')
_V = yaml.safe_load(YAML_PATH.read_text())


@dataclass(frozen=True)  # Read-only
class ShuttleConfig:
    """Static parameters of the NiFT shuttle (vehicle.yaml)."""

    # Geometric Parameters [m]
    wheel_radius: float = _V['wheel_radius']
    wheel_width: float = _V['wheel_width']
    wheel_base: float = _V['wheel_base']
    wheel_tread: float = _V['wheel_tread']
    front_overhang: float = _V['front_overhang']
    rear_overhang: float = _V['rear_overhang']
    height: float = _V['height']
    length: float = _V['length']
    width: float = _V['width']
    base_link_height: float = _V['base_link_height']

    # Kinematic and Dynamic Parameters
    # [deg] full lock = the CAN command limit
    max_steering_deg: float = _V['max_steering_deg']
    max_steering_angle: float = math.radians(_V['max_steering_deg'])    # [rad] the same
    max_speed: float = _V['max_speed']            # [m/s]
    max_accel: float = _V['max_accel']            # [m/s²]
    max_decel: float = _V['max_decel']            # [m/s²]
    mass: float = _V['mass']                      # [kg]
    cog_height: float = _V['cog_height']          # [m]
    brake_full_count: int = _V['brake_full_count']

    # Sensor mounts, in base_link [m]
    gnss_front_x: float = _V['gnss_front_x']
    gnss_rear_x: float = _V['gnss_rear_x']
    gnss_z: float = _V['gnss_z']
    lidar_x: float = _V['lidar_x']
    lidar_y: float = _V['lidar_y']
    lidar_z: float = _V['lidar_z']
    lidar_pitch_deg: float = _V['lidar_pitch_deg']

    @property
    def base_link_x(self) -> float:
        """base_link ahead of the rear axle [m] (mid-wheelbase)."""
        return 0.5 * self.wheel_base

    @property
    def max_curvature(self) -> float:
        """Curvature at full lock [1/m]."""
        return math.tan(self.max_steering_angle) / self.wheel_base

    @property
    def min_turn_radius(self) -> float:
        """Rear-axle turn radius at full lock [m]."""
        return 1.0 / self.max_curvature


# Robot state
@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vel: float = 0.0
    x_rear: float = 0.0
    y_rear: float = 0.0
