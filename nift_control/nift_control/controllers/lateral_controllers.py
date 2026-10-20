"""
Lateral (steering) controllers: Pure Pursuit and Stanley.

File: lateral_controllers.py
Author: Kane Weng
Date: Apr 16, 2026

Description:
  Concrete lateral (steering) controllers for path tracking on the NiFT shuttle.
  All classes subclass LateralController and return the shared four-tuple:
    (steering_angle [rad], ref_x, ref_y, updated_wp_idx)

  PurePursuitController
  ─────────────────────
  Geometric path tracker operating from the vehicle's rear-axle reference point.
    - Adaptive lookahead distance: linearly scales with speed ratio
      (ld_min at 0 m/s → ld_max at base_speed).
    - Circle-segment intersection: solves the quadratic for each candidate
      segment within a 20-segment search window.
    - Ackermann steering formula: δ = atan2(2 * L * sin(α), ld)
    - ref point returned: the lookahead intersection point.

  StanleyController
  ─────────────────
  Front-axle path tracker based on the Stanley method (Thrun et al., 2006).
  Uses the nearest path segment to the front axle instead of a lookahead
  circle, making it more accurate at low speed and on tight curves.

  Control law:
    δ = θ_e + atan2(k · e, max(|v|, v_min))
    where:
      θ_e  = path_heading − vehicle_yaw          (heading error, rad)
      e    = signed CTE at front axle [m]
             positive when vehicle is to the RIGHT of path → steer left
      k    = cross-track gain (tune: higher = more aggressive correction)
      v_min= minimum speed for the atan term (avoids div-by-zero at standstill)

  Waypoint index is advanced to the nearest segment found so the controller
  never tracks a segment behind the vehicle.

Usage:
  from .lateral_controllers import PurePursuitController, StanleyController
"""
from dataclasses import dataclass  # noqa: F401  (unused)
import math

from nift_common.vehicle import ShuttleConfig, VehicleState
import numpy as np

from .control_base import LateralController

shuttle = ShuttleConfig()


class PurePursuitController(LateralController):

    def __init__(self, waypoints, ld_min, ld_max, base_speed):
        # Convert to numpy array ONCE upon initialization for massive speed gains
        self._waypoints = np.array(waypoints)
        self._ld_min = ld_min
        self._ld_max = ld_max
        self._base_speed = base_speed

    def compute(self, state: VehicleState, current_wp_idx: int):
        """
        Calculate the pure pursuit steering angle.

        Returns: (steering_angle, target_x, target_y, updated_wp_idx)
        """
        # Define Adaptive Lookahead (ld)
        speed_ratio = np.clip(abs(state.vel) / self._base_speed, 0.0, 1.0)
        ld = self._ld_min + (self._ld_max - self._ld_min) * speed_ratio

        # Get Target Point
        tx, ty, new_idx = self._get_lookahead_point(state.x_rear, state.y_rear, ld, current_wp_idx)

        # Pure Pursuit Math
        dx = tx - state.x_rear
        dy = ty - state.y_rear

        # Angle to the lookahead point
        alpha = math.atan2(dy, dx) - state.yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))    # Normalize alpha

        # Ackermann Steering Formula
        steering_angle = math.atan2(2.0 * shuttle.wheel_base * math.sin(alpha), ld)

        return steering_angle, tx, ty, new_idx

    def _get_lookahead_point(self, curr_x_rear, curr_y_rear, lookahead_dist, current_wp_idx):
        best_point = None
        search_end_idx = min(current_wp_idx + 20, len(self._waypoints) - 1)  # Need to verify
        updated_idx = current_wp_idx

        # Define the current position vector once outside the loop
        curr_pos = np.array([curr_x_rear, curr_y_rear])

        for i in range(current_wp_idx, search_end_idx):
            # Directly reference the pre-computed numpy array rows
            p1 = self._waypoints[i]
            p2 = self._waypoints[i + 1]

            V = p2 - p1
            D = p1 - curr_pos

            a = np.dot(V, V)
            b = 2 * np.dot(D, V)
            c = np.dot(D, D) - lookahead_dist**2
            discriminant = b**2 - 4 * a * c

            if discriminant >= 0:
                discriminant = math.sqrt(discriminant)
                t2 = (-b + discriminant) / (2 * a)

                if 0 <= t2 <= 1:
                    best_point = p1 + t2 * V
                    if i > current_wp_idx:
                        updated_idx = i
                    break

        if best_point is None:
            target = self._waypoints[min(current_wp_idx + 1, len(self._waypoints) - 1)]
            return target[0], target[1], updated_idx

        return best_point[0], best_point[1], updated_idx

    def reset(self):
        pass


class StanleyController(LateralController):

    def __init__(self, waypoints, k: float = 0.5, v_min: float = 0.5):
        """
        Store the path and the gains.

        Parameters
        ----------
        waypoints
            List of [x, y] path points.
        k
            Cross-track error gain. Higher = more aggressive lateral
            correction; tune to avoid oscillation.
        v_min
            Minimum speed [m/s] used in the atan term to prevent
            divide-by-zero at standstill (soft floor, not a speed limit).

        """
        self._waypoints = np.array(waypoints)
        self._k = k
        self._v_min = v_min

    def compute(self, state: VehicleState, current_wp_idx: int):
        """
        Compute Stanley steering angle from the front-axle reference point.

        Returns: (steering_angle [rad], front_axle_x, front_axle_y, updated_wp_idx)
        """
        # ---- Front-axle position ---- #
        x_f = state.x + shuttle.wheel_base * math.cos(state.yaw)
        y_f = state.y + shuttle.wheel_base * math.sin(state.yaw)
        front_axle = np.array([x_f, y_f])

        # ---- Nearest segment to front axle (search up to 30 segments ahead) ---- #
        search_end = min(current_wp_idx + 30, len(self._waypoints) - 2)
        candidates = self._waypoints[current_wp_idx: search_end + 1]
        dists = np.linalg.norm(candidates - front_axle, axis=1)
        nearest_idx = current_wp_idx + int(np.argmin(dists))

        # Clamp so p2 always exists
        seg_idx = min(nearest_idx, len(self._waypoints) - 2)
        p1 = self._waypoints[seg_idx]
        p2 = self._waypoints[seg_idx + 1]

        # ---- Heading error θ_e ---- #
        path_yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        theta_e = math.atan2(math.sin(path_yaw - state.yaw),
                             math.cos(path_yaw - state.yaw))

        # ---- Signed cross-track error e ---- #
        # cross(seg, to_front) > 0  →  front axle is to the LEFT of path
        # We want e > 0 when to the RIGHT (needs left correction), so negate.
        seg_vec = p2 - p1
        to_front = front_axle - p1
        seg_len = float(np.linalg.norm(seg_vec)) + 1e-9
        cross = seg_vec[0] * to_front[1] - seg_vec[1] * to_front[0]
        e = -float(cross) / seg_len

        # ---- Stanley formula ---- #
        v_eff = max(abs(state.vel), self._v_min)
        delta = theta_e + math.atan2(self._k * e, v_eff)
        delta = math.atan2(math.sin(delta), math.cos(delta))   # normalize to (-π, π)

        return delta, x_f, y_f, nearest_idx

    def reset(self):
        pass
