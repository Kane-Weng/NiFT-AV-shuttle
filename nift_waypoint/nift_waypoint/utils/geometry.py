"""
Pure 2D geometry helpers for curves, downsampling, quaternions and feasibility checks.

File: geometry.py
Author: Kane Weng
Date: Apr 11, 2026

Description:
  Pure 2D geometry and mathematical utilities. No ROS or Lanelet2 dependencies.
  Provides high-performance functions for curve generation, spatial downsampling,
  quaternion conversions, and trajectory feasibility checks.
"""

import math


def circumradius(p1, p2, p3):
    """
    Calculate the circumradius of a triangle formed by three 2D points.

    Used for estimating the turning radius of a discrete path segment.

    Parameters
    ----------
    p1, p2, p3 : tuple of (float, float)
        The three 2D coordinate pairs forming the triangle.

    Returns
    -------
    float or None
        The circumradius in meters.
        Returns None if any side is < 1mm (degenerate).
        Returns float('inf') if the points are perfectly collinear.

    """
    a = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    b = math.hypot(p3[0] - p2[0], p3[1] - p2[1])
    c = math.hypot(p3[0] - p1[0], p3[1] - p1[1])

    if a < 1e-3 or b < 1e-3 or c < 1e-3:
        return None

    area_twice = abs(
        (p2[0] - p1[0]) * (p3[1] - p1[1]) -
        (p2[1] - p1[1]) * (p3[0] - p1[0])
    )

    if area_twice < 1e-6:
        return float('inf')

    area = 0.5 * area_twice
    return (a * b * c) / (4.0 * area)


def heading_from_quaternion(q):
    """
    Extract the 2D yaw (radians, ENU) from a ROS quaternion.

    Parameters
    ----------
    q : geometry_msgs.msg.Quaternion
        A quaternion-like object containing x, y, z, w fields.

    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def segment_heading(x1, y1, x2, y2, fallback_rad=0.0):
    """
    Calculate the directional heading (radians) between two 2D points.

    Parameters
    ----------
    x1, y1, x2, y2 : float
        Start and end points.
    fallback_rad : float, optional
        The default heading to return if the points are too close together.

    """
    dx, dy = x2 - x1, y2 - y1
    if math.hypot(dx, dy) < 1e-6:
        return fallback_rad
    return math.atan2(dy, dx)


def cubic_bezier_pts(p0, h0, p3, h3, n_pts):
    """
    Generate an array of evenly-spaced points along a cubic Bezier curve.

    Uses tangent scaling based on the Euclidean distance between endpoints.

    Parameters
    ----------
    p0, p3 : tuple of (float, float)
        Start and end coordinates.
    h0, h3 : float
        Tangent headings (radians) at the start and end points.
    n_pts : int
        Number of subdivisions to generate. Output length is n_pts + 1.

    """
    dx = p3[0] - p0[0]
    dy = p3[1] - p0[1]
    dist = math.hypot(dx, dy)

    # Return a straight line if endpoints are practically touching
    if dist < 0.1:
        return [p0, p3]

    tangent_scale = dist * 0.70
    c1 = (p0[0] + tangent_scale * math.cos(h0), p0[1] + tangent_scale * math.sin(h0))
    c2 = (p3[0] - tangent_scale * math.cos(h3), p3[1] - tangent_scale * math.sin(h3))

    pts = []
    for i in range(n_pts + 1):
        t = i / n_pts
        mt = 1.0 - t
        x = mt**3 * p0[0] + 3 * mt**2 * t * c1[0] + 3 * mt * t**2 * c2[0] + t**3 * p3[0]
        y = mt**3 * p0[1] + 3 * mt**2 * t * c1[1] + 3 * mt * t**2 * c2[1] + t**3 * p3[1]
        pts.append((x, y))

    return pts


def append_unique_xy(points, x, y):
    """
    Append (x, y) to a list only if it differs from the last entry.

    Uses a fast Manhattan distance check (1e-6 m tolerance) to reject duplicates.
    """
    if points:
        last_x, last_y = points[-1]
        if abs(last_x - x) < 1e-6 and abs(last_y - y) < 1e-6:
            return
    points.append((x, y))


def downsample_path_points(path_points, stride):
    """
    Return a stride-sampled copy of the path, preserving the endpoints.

    Utilizes highly-optimized native Python slicing.

    Parameters
    ----------
    path_points : list of tuple
        The full resolution array of coordinate pairs.
    stride : int
        The step size for downsampling (e.g., 5 takes every 5th point).

    """
    if len(path_points) <= 2 or stride <= 1:
        return path_points

    # Native slice is C-optimized and significantly faster than a for-loop
    sampled = path_points[::stride]

    # Guarantee the final destination point is never truncated
    if sampled[-1] != path_points[-1]:
        sampled.append(path_points[-1])

    return sampled


def format_radius(radius):
    """Format a floating-point turn radius into a readable log string."""
    if math.isinf(radius):
        return 'inf'
    return f'{radius:.2f} m'
