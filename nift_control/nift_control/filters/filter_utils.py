"""
Geometry helpers shared by the safety filters.

File: filter_utils.py
Author: Kane Weng
Date: Apr 27, 2026

Description:
  Shared geometry helpers used by all SafetyFilter implementations.

  VEHICLE_RADIUS
    Half-diagonal of the vehicle bounding box [m].  Used as a conservative
    edge approximation when only a scalar clearance is needed.

  _min_obstacle_distance(state, obstacles, behind=False) → float
    Minimum Euclidean clearance from the vehicle's bounding edge to any
    forward obstacle point (dot-product gate drops obstacles behind the
    vehicle).  Suitable for telemetry and for steering-trigger decisions
    where all forward obstacles matter.  behind=True looks backwards instead
    (MANUAL's collision check while reversing).

  _min_corridor_clearance(state, obstacles, corridor_half) → float
    Minimum forward clearance from the vehicle's front face to obstacles
    that fall inside the driving corridor (|lateral offset| ≤ corridor_half).
    Purely lateral obstacles are excluded so the vehicle is not penalised for
    objects it is already passing beside.  Used by all filters for speed
    reduction / blocking decisions.

  BRAKE_DECEL_MPS2, BRAKE_DELAY_S
    The shuttle's measured braking, which every filter plans its stops with.

  stopping_distance(v, decel, delay) → float
    Distance [m] the shuttle needs to stop from speed v: it rolls on for the
    delay before the brake acts, then brakes at decel.

  braking_speed_cap(room, decel, delay) → float
    The highest speed [m/s] from which the shuttle still stops within room [m],
    the inverse of stopping_distance.
"""
import math

from nift_common.vehicle import ShuttleConfig, VehicleState
import numpy as np

config = ShuttleConfig()
VEHICLE_RADIUS = math.sqrt(config.length ** 2 + config.width ** 2) / 2
_CORRIDOR_HALF = config.width / 2.0 + 0.3   # path half-width + 0.3 m buffer

# The shuttle's measured braking (longitudinal_test, garage sessions of October 2026).
# Full brake gave 0.5 to 0.65 m/s², the same from brake count 40 to 255, and the speed
# reacted 0.5 to 0.7 s after a brake command. The filters take the weak end of both, so
# they never assume more braking than the shuttle has. trajectory_controller passes its
# parameters brake_decel_mps2 and brake_delay_s, which default to these.
BRAKE_DECEL_MPS2 = 0.5     # [m/s²]
BRAKE_DELAY_S = 0.7        # [s]


def stopping_distance(v: float, decel: float = BRAKE_DECEL_MPS2,
                      delay: float = BRAKE_DELAY_S) -> float:
    """
    Distance [m] to stop from speed v [m/s].

    The shuttle rolls v · delay before the brake acts, then brakes over v² / (2 · decel).
    """
    v = abs(v)
    return v * delay + v * v / (2.0 * decel)


def braking_speed_cap(room: float, decel: float = BRAKE_DECEL_MPS2,
                      delay: float = BRAKE_DELAY_S) -> float:
    """
    Highest speed [m/s] from which the shuttle still stops within room [m].

    It is the inverse of stopping_distance, so stopping_distance(cap) == room.
    It is 0 when room <= 0.
    """
    if room <= 0.0:
        return 0.0
    return math.sqrt((decel * delay) ** 2 + 2.0 * decel * room) - decel * delay


def _min_obstacle_distance(state: VehicleState, obstacles: np.ndarray,
                           behind: bool = False) -> float:
    """
    Minimum clearance from the vehicle's outer bound to any forward obstacle.

    With behind=True it measures to the obstacles behind the vehicle instead.
    """
    if obstacles.size == 0:
        return float('inf')

    vehicle_pos = np.array([state.x, state.y])
    heading_vec = np.array([math.cos(state.yaw), math.sin(state.yaw)])

    vecs_to_obs = obstacles - vehicle_pos
    lon = np.dot(vecs_to_obs, heading_vec)
    valid_vecs = vecs_to_obs[lon < 0 if behind else lon > 0]

    if valid_vecs.size == 0:
        return float('inf')

    min_center_dist = float(np.min(np.linalg.norm(valid_vecs, axis=1)))
    return max(0.0, min_center_dist - VEHICLE_RADIUS)


def _min_corridor_clearance(
    state: VehicleState,
    obstacles: np.ndarray,
    corridor_half: float = _CORRIDOR_HALF,
) -> float:
    """
    Minimum forward clearance from the vehicle's bounding edge to corridor obstacles.

    Only obstacles inside the driving corridor count (lateral gate unchanged — purely
    lateral obstacles are excluded so the vehicle is not penalized for objects it is
    already passing).
    """
    if obstacles.size == 0:
        return float('inf')

    vehicle_pos = np.array([state.x, state.y])
    heading_vec = np.array([math.cos(state.yaw), math.sin(state.yaw)])
    perp_vec = np.array([-math.sin(state.yaw), math.cos(state.yaw)])

    vecs_to_obs = obstacles - vehicle_pos
    lon_all = np.dot(vecs_to_obs, heading_vec)

    forward_mask = lon_all > 0
    if not np.any(forward_mask):
        return float('inf')

    vecs_fwd = vecs_to_obs[forward_mask]
    lon_fwd = lon_all[forward_mask]
    lat_abs = np.abs(np.dot(vecs_fwd, perp_vec))

    corridor_mask = lat_abs <= corridor_half
    if not np.any(corridor_mask):
        return float('inf')

    lon_clearances = np.maximum(0.0, lon_fwd[corridor_mask] - VEHICLE_RADIUS)
    return float(np.min(lon_clearances))
