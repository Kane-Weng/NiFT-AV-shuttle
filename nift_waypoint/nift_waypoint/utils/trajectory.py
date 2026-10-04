"""
Build smooth Bezier legs that join free space to the Lanelet2 road network.

File: trajectory.py
Author: Kane Weng
Date: Apr 11, 2026

Description:
  Trajectory generation utilities for bridging unstructured free-space
  with structured Lanelet2 road networks. Uses smooth cubic Bézier curves
  to prevent sharp discontinuities when entering or exiting the map graph.
"""

import math

from .geometry import cubic_bezier_pts, segment_heading


def build_entry_leg_abs(vehicle_pos_abs, vehicle_heading_rad, lanelet_pts_abs, bridge_bezier_pts):
    """
    Generate a smooth entry curve from the vehicle's pose to the Lanelet path.

    The curve joins the vehicle's current unconstrained pose to the first node of the
    structured Lanelet path.
    """
    if not lanelet_pts_abs:
        return []

    p0 = (vehicle_pos_abs.x, vehicle_pos_abs.y)
    p3 = lanelet_pts_abs[0]
    dist = math.hypot(p3[0] - p0[0], p3[1] - p0[1])

    # If the vehicle is essentially already on the start node, skip the entry leg
    if dist < 0.5:
        return []

    if len(lanelet_pts_abs) >= 2:
        entry_heading = segment_heading(
            *lanelet_pts_abs[0], *lanelet_pts_abs[1],
            fallback_rad=vehicle_heading_rad,
        )
    else:
        entry_heading = vehicle_heading_rad

    pts = cubic_bezier_pts(p0, vehicle_heading_rad, p3, entry_heading, bridge_bezier_pts)
    # Drop the final point so it doesn't duplicate the first lanelet point
    return pts[:-1]


def build_exit_leg_abs(lanelet_pts_abs, goal_abs, bridge_bezier_pts):
    """
    Generate a smooth exit curve from the Lanelet path to the goal.

    The curve joins the final node of the Lanelet path to the user's unconstrained goal pose.
    """
    if not lanelet_pts_abs:
        return []

    p0 = lanelet_pts_abs[-1]
    p3 = (goal_abs.x, goal_abs.y)
    dist = math.hypot(p3[0] - p0[0], p3[1] - p0[1])

    # If the final lanelet node is essentially the goal, skip the exit leg
    if dist < 0.5:
        return []

    if len(lanelet_pts_abs) >= 2:
        exit_heading = segment_heading(*lanelet_pts_abs[-2], *lanelet_pts_abs[-1])
    else:
        exit_heading = math.atan2(p3[1] - p0[1], p3[0] - p0[0])

    goal_heading = math.atan2(p3[1] - p0[1], p3[0] - p0[0])
    pts = cubic_bezier_pts(p0, exit_heading, p3, goal_heading, bridge_bezier_pts)
    # Drop the first point so it doesn't duplicate the last lanelet point
    return pts[1:]


def build_offroad_pts_abs(start_abs, heading_rad, goal_abs, bridge_bezier_pts):
    """Generate a direct Bézier curve from start to goal, completely ignoring the map."""
    p0 = (start_abs.x, start_abs.y)
    p3 = (goal_abs.x, goal_abs.y)
    goal_heading = math.atan2(p3[1] - p0[1], p3[0] - p0[0])
    return cubic_bezier_pts(p0, heading_rad, p3, goal_heading, bridge_bezier_pts)
