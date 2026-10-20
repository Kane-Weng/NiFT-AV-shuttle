"""
Kinematic bicycle model of the NiFT shuttle for the kinematic sim backend.

File: kinematic_model.py
Author: Kane Weng

Description:
  Kinematic bicycle model of the NiFT shuttle: the pose integrator of the
  kinematic sim backend, which stands in for CARLA's physics on machines
  without a GPU. The speed comes from the shuttle plant (shuttle_plant.py), as
  in the CARLA bridge; this module only turns that speed and the front-wheel
  angle the virtual ECU decoded into a pose.

    reference   the rear-axle centre, which a bicycle model moves without side slip:
                    dx/dt = v cos(yaw),  dy/dt = v sin(yaw),  dyaw/dt = v tan(delta) / L
                L = the shuttle's wheel_base (nift_common vehicle.yaml), so full lock
                is its min_turn_radius circle (7.77 m). base_link (mid-wheelbase, as in
                the URDF and the CARLA bridge) sits L / 2 ahead of it.
    one tick    speed and wheel angle are held over dt and the arc is integrated
                exactly, so the turning radius does not depend on dt.
    steering    the wheel follows the command at once, clipped to full lock, unless
                steer_tau_s (first-order lag) or steer_rate_deg_s (rate limit) is set.
                The shuttle's steering response has not been measured, so both default
                to 0 (kinematic sim plan, D5).
    not modelled  tyre slip, load transfer, terrain (the world is flat). At the
                  shuttle's speeds (governor 11 mph) the slip is small; CARLA's
                  PhysX tyres are what the comparison runs measure against.

  Map frame (yaw CCW from +x), metres and radians; the wheel angle in degrees with
  the CAN sign (+ = left). No ROS in here, so it is unit-tested on its own
  (test/test_kinematic_model.py).

Usage:
  car = KinematicBicycle(BicycleConfig())
  car.reset(x, y, yaw)              # base_link pose: spawn / teleport, at rest, wheels straight
  car.step(dt, v, steer_deg)        # v from ShuttlePlant.step, steer_deg from the virtual ECU
  x, y, yaw = car.base_link()
  car.v, car.yaw_rate, car.steer_deg
"""

from dataclasses import dataclass
import math
from typing import Tuple

from nift_common.vehicle import ShuttleConfig

SHUTTLE = ShuttleConfig()
SHUTTLE_WHEELBASE = SHUTTLE.wheel_base          # m
STEER_LIMIT_DEG = SHUTTLE.max_steering_deg      # front-wheel full lock = the CAN command limit


def wrap_angle(a: float) -> float:
    """Angle in [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class BicycleConfig:
    wheelbase: float = SHUTTLE_WHEELBASE
    max_steer_deg: float = STEER_LIMIT_DEG  # front-wheel limit
    # first-order lag of the wheel behind the command; 0 = none
    steer_tau_s: float = 0.0
    steer_rate_deg_s: float = 0.0           # wheel rate limit; 0 = none


class KinematicBicycle:

    def __init__(self, config: BicycleConfig = BicycleConfig()):
        self.cfg = config
        self.reset(0.0, 0.0, 0.0)

    def reset(self, x: float, y: float, yaw: float) -> None:
        """Put base_link at (x, y, yaw), at rest, wheels straight (spawn, teleport)."""
        self.yaw = wrap_angle(yaw)
        half = 0.5 * self.cfg.wheelbase
        self.x = x - half * math.cos(self.yaw)      # rear-axle centre
        self.y = y - half * math.sin(self.yaw)
        self.v = 0.0
        self.steer_deg = 0.0
        self.yaw_rate = 0.0

    def step(self, dt: float, v: float, steer_cmd_deg: float) -> None:
        """Advance dt at speed v [m/s, signed] with the wheel commanded to steer_cmd_deg."""
        self._steer(dt, steer_cmd_deg)
        # curvature [1/m], + = left
        k = math.tan(math.radians(self.steer_deg)) / self.cfg.wheelbase
        ds = v * dt
        dyaw = k * ds
        if abs(dyaw) < 1e-9:
            self.x += ds * math.cos(self.yaw)
            self.y += ds * math.sin(self.yaw)
        else:                                       # exact arc of radius 1 / k
            self.x += (math.sin(self.yaw + dyaw) - math.sin(self.yaw)) / k
            self.y -= (math.cos(self.yaw + dyaw) - math.cos(self.yaw)) / k
            self.yaw = wrap_angle(self.yaw + dyaw)
        self.v = v
        self.yaw_rate = v * k

    def base_link(self) -> Tuple[float, float, float]:
        """(x, y, yaw) of base_link, L / 2 ahead of the rear axle."""
        half = 0.5 * self.cfg.wheelbase
        return self.x + half * math.cos(self.yaw), self.y + half * math.sin(self.yaw), self.yaw

    def _steer(self, dt: float, cmd_deg: float) -> None:
        lim = self.cfg.max_steer_deg
        d = min(max(cmd_deg, -lim), lim) - self.steer_deg
        if self.cfg.steer_tau_s > 0.0:
            d *= 1.0 - math.exp(-dt / self.cfg.steer_tau_s)     # exact for a command held over dt
        if self.cfg.steer_rate_deg_s > 0.0:
            m = self.cfg.steer_rate_deg_s * dt
            d = min(max(d, -m), m)
        self.steer_deg += d
