"""
Build and publish the RViz markers for the Lanelet2 map, the route and the goal.

File: visualizer.py
Author: Kane Weng
Date: Apr 11, 2026

Description:
  Decoupled RViz visualization module. Handles the assembly and publishing of
  MarkerArrays for the Lanelet2 map boundaries, route paths, and goal states.
  Keeps the core navigation node clear of UI/rendering boilerplate.
"""

import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class WaypointVisualizer:
    """Wraps ROS 2 publishers and handles RViz marker formatting."""

    def __init__(self, node, coord_transform):
        self.node = node
        self._coord = coord_transform

        self.map_marker_pub = self.node.create_publisher(MarkerArray, '/lanelet_map_markers', 10)
        self.start_goal_marker_pub = self.node.create_publisher(
            MarkerArray, '/start_goal_markers', 10)
        self.route_marker_pub = self.node.create_publisher(
            MarkerArray, '/navigation_waypoints', 10)

    def publish_map_markers(self, lanelet_map):
        """Render the Lanelet2 map boundaries and directional chevrons."""
        marker_array = MarkerArray()
        stamp = self.node.get_clock().now().to_msg()

        delete_marker = Marker(action=Marker.DELETEALL, ns='lanelet_map', id=0)
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = stamp
        marker_array.markers.append(delete_marker)

        boundary_marker = Marker(ns='lanelet_map', id=1, type=Marker.LINE_LIST, action=Marker.ADD)
        boundary_marker.header.frame_id = 'map'
        boundary_marker.header.stamp = stamp
        boundary_marker.pose.orientation.w = 1.0
        boundary_marker.scale.x = 0.12
        boundary_marker.color = ColorRGBA(r=0.92, g=0.92, b=0.92, a=1.0)

        for ll in lanelet_map.laneletLayer:
            self._append_linestring_segments(boundary_marker, ll.leftBound)
            self._append_linestring_segments(boundary_marker, ll.rightBound)

        marker_array.markers.append(boundary_marker)
        self.map_marker_pub.publish(marker_array)

    def publish_start_goal_markers(self, start_local, goal_local):
        """Publish prominent spheres for the current GPS/Odom start and chosen goal."""
        marker_array = MarkerArray()
        stamp = self.node.get_clock().now().to_msg()

        delete_marker = Marker(action=Marker.DELETEALL, ns='start_goal', id=0)
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = stamp
        marker_array.markers.append(delete_marker)

        marker_id = 1

        if start_local is not None:
            start_marker = Marker(ns='start_goal', id=marker_id, type=Marker.SPHERE,
                                  action=Marker.ADD)
            start_marker.header.frame_id = 'map'
            start_marker.header.stamp = stamp
            start_marker.pose.position.x = start_local.x
            start_marker.pose.position.y = start_local.y
            start_marker.pose.position.z = 0.5
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 1.2
            start_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            marker_array.markers.append(start_marker)
            marker_id += 1

        if goal_local is not None:
            goal_marker = Marker(ns='start_goal', id=marker_id, type=Marker.SPHERE,
                                 action=Marker.ADD)
            goal_marker.header.frame_id = 'map'
            goal_marker.header.stamp = stamp
            goal_marker.pose.position.x = goal_local.x
            goal_marker.pose.position.y = goal_local.y
            goal_marker.pose.position.z = 0.5
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 1.2
            goal_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker_array.markers.append(goal_marker)

        self.start_goal_marker_pub.publish(marker_array)

    def clear_route_markers(self):
        """Wipes the active route from RViz."""
        marker_array = MarkerArray()
        delete_marker = Marker(action=Marker.DELETEALL, ns='route', id=0)
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.node.get_clock().now().to_msg()
        marker_array.markers.append(delete_marker)
        self.route_marker_pub.publish(marker_array)

    def publish_route_markers_3phase(self, entry_pts_abs, lanelet_pts_abs, exit_pts_abs):
        """Draws the planned path segmented into Entry, Main Lanelet, and Exit phases."""
        marker_array = MarkerArray()
        stamp = self.node.get_clock().now().to_msg()

        delete_marker = Marker(action=Marker.DELETEALL, ns='route', id=0)
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = stamp
        marker_array.markers.append(delete_marker)

        if lanelet_pts_abs:
            marker_array.markers.extend([
                self._make_phase_line_marker(
                    'lanelet_route_path', 1, lanelet_pts_abs,
                    ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0), stamp),
                self._make_phase_node_marker(
                    'lanelet_route_nodes', 2, lanelet_pts_abs,
                    ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0), stamp)
            ])
        if entry_pts_abs:
            marker_array.markers.extend([
                self._make_phase_line_marker(
                    'entry_leg', 3, entry_pts_abs,
                    ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0), stamp, line_width=0.35),
                self._make_phase_node_marker(
                    'entry_leg_nodes', 5, entry_pts_abs,
                    ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0), stamp)
            ])
        if exit_pts_abs:
            marker_array.markers.extend([
                self._make_phase_line_marker(
                    'exit_leg', 4, exit_pts_abs,
                    ColorRGBA(r=0.75, g=0.0, b=1.0, a=1.0), stamp, line_width=0.35),
                self._make_phase_node_marker(
                    'exit_leg_nodes', 6, exit_pts_abs,
                    ColorRGBA(r=0.75, g=0.0, b=1.0, a=1.0), stamp)
            ])

        self.route_marker_pub.publish(marker_array)

    # =========================================================================
    # Internal Render Helpers
    # =========================================================================

    def _make_phase_line_marker(self, ns, marker_id, pts_abs, color, stamp, line_width=0.4):
        """Generate a LINE_STRIP marker for a given set of absolute points."""
        m = Marker(ns=ns, id=marker_id, type=Marker.LINE_STRIP, action=Marker.ADD)
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.pose.orientation.w = 1.0
        m.scale.x = line_width
        m.color = color

        # Optimize loop by using constructor kwargs and building a list
        points = []
        for x_abs, y_abs in pts_abs:
            px, py = self._coord.to_display_xy(x_abs, y_abs)
            points.append(Point(x=px, y=py, z=0.25))
        m.points.extend(points)
        return m

    def _make_phase_node_marker(self, ns, marker_id, pts_abs, color, stamp):
        """Generate a SPHERE_LIST marker to highlight specific nodes/waypoints."""
        m = Marker(ns=ns, id=marker_id, type=Marker.SPHERE_LIST, action=Marker.ADD)
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.6
        m.color = color

        points = []
        for x_abs, y_abs in pts_abs:
            px, py = self._coord.to_display_xy(x_abs, y_abs)
            points.append(Point(x=px, y=py, z=0.25))
        m.points.extend(points)
        return m

    def _append_linestring_segments(self, marker, linestring):
        """Append sequential point pairs from a Lanelet2 linestring to a LINE_LIST marker."""
        pts = list(linestring)
        # Pythonic pairwise iteration is faster than `for i in range(len(pts)-1)`
        for pt1, pt2 in zip(pts, pts[1:]):
            x1, y1 = self._coord.to_display_xy(pt1.x, pt1.y)
            x2, y2 = self._coord.to_display_xy(pt2.x, pt2.y)
            marker.points.extend([
                Point(x=x1, y=y1, z=0.0),
                Point(x=x2, y=y2, z=0.0)
            ])

    def _append_lane_direction_chevron(self, marker, lanelet_obj):
        """Calculate and append a directional chevron to the center of a lanelet."""
        centerline = list(lanelet_obj.centerline)
        if len(centerline) < 2:
            return

        # Find the middle segment of the centerline
        segment_index = max(0, (len(centerline) - 1) // 2)
        start_pt = centerline[segment_index]
        end_pt = centerline[segment_index + 1]

        x1, y1 = self._coord.to_display_xy(start_pt.x, start_pt.y)
        x2, y2 = self._coord.to_display_xy(end_pt.x, end_pt.y)
        dx, dy = x2 - x1, y2 - y1
        seg_len = math.hypot(dx, dy)

        if seg_len < 1e-3:
            return

        tx, ty = dx / seg_len, dy / seg_len
        nx, ny = -ty, tx
        cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)

        chevron_length = min(1.1, max(0.55, 0.45 * seg_len))
        chevron_width = 0.35 * chevron_length

        apex = Point(
            x=cx + 0.5 * chevron_length * tx,
            y=cy + 0.5 * chevron_length * ty,
            z=0.05
        )
        left_tail = Point(
            x=cx - 0.5 * chevron_length * tx + 0.5 * chevron_width * nx,
            y=cy - 0.5 * chevron_length * ty + 0.5 * chevron_width * ny,
            z=0.05
        )
        right_tail = Point(
            x=cx - 0.5 * chevron_length * tx - 0.5 * chevron_width * nx,
            y=cy - 0.5 * chevron_length * ty - 0.5 * chevron_width * ny,
            z=0.05
        )

        # LINE_LIST requires pairs of points (A->B, B->C)
        marker.points.extend([left_tail, apex, right_tail, apex])
