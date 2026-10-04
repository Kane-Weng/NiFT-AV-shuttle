"""
Project 2D points onto polylines and trim polylines at those projections.

File: polyline.py
Author: Kane Weng
Date: Apr 11, 2026

Description:
  2D polyline projection and trimming utilities.
  Depends only on the geometry module; no ROS or Lanelet2 required.
"""

import math

from .geometry import append_unique_xy


def project_point_to_polyline(polyline, query_point_abs):
    """
    Project a 2D query point onto the nearest segment of a polyline.

    Parameters
    ----------
    polyline : list of (float, float)
        Ordered sequence of (x, y) vertices.
    query_point_abs : object with .x and .y attributes
        The point to project (e.g. lanelet2.core.BasicPoint2d).

    Returns
    -------
    dict
        seg_index  : index of the segment's starting vertex
        point      : (x, y) coordinates of the projected point
        arc_length : cumulative distance along the polyline to the projection
        dist_sq    : squared Euclidean distance from the query to the projection

    """
    if len(polyline) == 1:
        return {
            'seg_index': 0,
            'point': polyline[0],
            'arc_length': 0.0,
            'dist_sq': ((query_point_abs.x - polyline[0][0]) ** 2
                        + (query_point_abs.y - polyline[0][1]) ** 2),
        }

    qx, qy = query_point_abs.x, query_point_abs.y
    best = None
    cumulative_length = 0.0

    # Using zip for fast pairwise segment iteration
    for idx, ((x1, y1), (x2, y2)) in enumerate(zip(polyline, polyline[1:])):
        dx, dy = x2 - x1, y2 - y1
        seg_len_sq = dx * dx + dy * dy
        seg_len = math.sqrt(seg_len_sq)

        if seg_len_sq <= 1e-12:
            proj_x, proj_y = x1, y1
            arc_length = cumulative_length
        else:
            t = max(0.0, min(1.0, ((qx - x1) * dx + (qy - y1) * dy) / seg_len_sq))
            proj_x = x1 + t * dx
            proj_y = y1 + t * dy
            arc_length = cumulative_length + t * seg_len

        dist_sq = (qx - proj_x) ** 2 + (qy - proj_y) ** 2

        if best is None or dist_sq < best['dist_sq']:
            best = {
                'seg_index': idx,
                'point': (proj_x, proj_y),
                'arc_length': arc_length,
                'dist_sq': dist_sq,
            }

        cumulative_length += seg_len

    return best


def trim_polyline_from_point(polyline, query_point_abs):
    """Return the suffix of a polyline starting at the projection of a query point."""
    projection = project_point_to_polyline(polyline, query_point_abs)
    trimmed = []
    append_unique_xy(trimmed, *projection['point'])
    for point in polyline[projection['seg_index'] + 1:]:
        append_unique_xy(trimmed, *point)
    return trimmed


def trim_polyline_to_point(polyline, query_point_abs):
    """Return the prefix of a polyline ending at the projection of a query point."""
    projection = project_point_to_polyline(polyline, query_point_abs)
    trimmed = []
    for point in polyline[:projection['seg_index'] + 1]:
        append_unique_xy(trimmed, *point)
    append_unique_xy(trimmed, *projection['point'])
    return trimmed


def trim_polyline_between_points(polyline, start_point_abs, goal_point_abs, logger=None):
    """Return the segment of a polyline strictly between start and goal projections."""
    start_proj = project_point_to_polyline(polyline, start_point_abs)
    goal_proj = project_point_to_polyline(polyline, goal_point_abs)

    if goal_proj['arc_length'] < start_proj['arc_length']:
        if logger is not None:
            logger.warn(
                'Goal projects behind start on the same lanelet; '
                'using direct clipped segment'
            )
        return [start_proj['point'], goal_proj['point']]

    trimmed = []
    append_unique_xy(trimmed, *start_proj['point'])
    for point in polyline[start_proj['seg_index'] + 1:goal_proj['seg_index'] + 1]:
        append_unique_xy(trimmed, *point)
    append_unique_xy(trimmed, *goal_proj['point'])
    return trimmed
