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

  _min_obstacle_distance(state, obstacles) → float
    Minimum Euclidean clearance from the vehicle's bounding edge to any
    forward obstacle point (dot-product gate drops obstacles behind the
    vehicle).  Suitable for telemetry and for steering-trigger decisions
    where all forward obstacles matter.

  _min_corridor_clearance(state, obstacles, corridor_half) → float
    Minimum forward clearance from the vehicle's front face to obstacles
    that fall inside the driving corridor (|lateral offset| ≤ corridor_half).
    Purely lateral obstacles are excluded so the vehicle is not penalised for
    objects it is already passing beside.  Used by all filters for speed
    reduction / blocking decisions.
"""
import math

from nift_common.vehicle import ShuttleConfig, VehicleState
import numpy as np

config = ShuttleConfig()
VEHICLE_RADIUS = math.sqrt(config.length ** 2 + config.width ** 2) / 2
_CORRIDOR_HALF = config.width / 2.0 + 0.3   # path half-width + 0.3 m buffer


def _min_obstacle_distance(state: VehicleState, obstacles: np.ndarray) -> float:
    """Minimum clearance from the vehicle's outer bound to any forward obstacle."""
    if obstacles.size == 0:
        return float('inf')

    vehicle_pos = np.array([state.x, state.y])
    heading_vec = np.array([math.cos(state.yaw), math.sin(state.yaw)])

    vecs_to_obs = obstacles - vehicle_pos
    valid_vecs = vecs_to_obs[np.dot(vecs_to_obs, heading_vec) > 0]

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
