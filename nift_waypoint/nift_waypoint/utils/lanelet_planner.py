"""
Search Lanelet2 networks for routes that the shuttle can actually turn through.

File: lanelet_planner.py
Author: Kane Weng
Date: Apr 11, 2026

Description:
  Graph search and feasibility evaluation for Lanelet2 networks.
  Uses A* to find kinematically feasible routes through the lanelet map.
"""

import heapq
import math

from .geometry import append_unique_xy, circumradius
from .polyline import (
    trim_polyline_between_points,
    trim_polyline_from_point,
    trim_polyline_to_point,
)


class LaneletPlanner:
    """Handles routing graph search, path trimming, and trajectory feasibility checks."""

    def __init__(
        self,
        routing_graph,
        logger,
        max_search_expansions=3000,
        max_candidate_routes=40,
        allow_lane_changes_in_search=False,
        curvature_check_point_step=3,
        planning_min_turn_radius_m=0.0
    ):
        self.routing_graph = routing_graph
        self.logger = logger
        self.max_search_expansions = max_search_expansions
        self.max_candidate_routes = max_candidate_routes
        self.allow_lane_changes_in_search = allow_lane_changes_in_search
        self.curvature_check_point_step = curvature_check_point_step
        self.planning_min_turn_radius_m = planning_min_turn_radius_m

        # Graph node caches to prevent redundant recalculations
        self._lanelet_length_cache = {}
        self._lanelet_center_cache = {}

    def find_feasible_lanelet_path(self, start_lanelet, goal_lanelet):
        """Attempt to find the shortest feasible route using Lanelet2's native router."""
        if start_lanelet.id == goal_lanelet.id:
            return [start_lanelet], 'Using same-lanelet route'

        route = self.routing_graph.getRoute(start_lanelet, goal_lanelet, 0)
        if route is None:
            return None, 'lanelet2 shortest-path lookup returned no route'

        shortest_path = route.shortestPath()
        if shortest_path is None or len(shortest_path) == 0:
            return None, 'lanelet2 shortestPath() returned no lanelets'

        shortest_lanelets = list(shortest_path)
        return shortest_lanelets, f'Using shortest route with {len(shortest_lanelets)} lanelets'

    def search_alternative_feasible_path(
        self,
        start_lanelet,
        goal_lanelet,
        start_point_abs,
        goal_point_abs,
        min_turn_radius_threshold=None,
    ):
        """Run a custom A* search for paths that honor the vehicle's turn radius."""
        queue = []
        search_counter = 0
        candidate_count = 0
        expansions = 0

        start_cost = self._lanelet_length(start_lanelet)
        start_estimate = start_cost + self._heuristic_lanelet_cost(start_lanelet, goal_lanelet)
        heapq.heappush(
            queue,
            (start_estimate, start_cost, search_counter, [start_lanelet], {start_lanelet.id}),
        )
        search_counter += 1

        while queue and expansions < self.max_search_expansions:
            _, path_cost, _, lanelet_path, visited_ids = heapq.heappop(queue)
            expansions += 1
            current_lanelet = lanelet_path[-1]

            if current_lanelet.id == goal_lanelet.id:
                candidate_count += 1
                feasible, min_radius = self.is_lanelet_path_feasible(
                    lanelet_path,
                    start_point_abs,
                    goal_point_abs,
                    min_turn_radius_threshold,
                )
                if feasible:
                    return lanelet_path, 'Found valid route'
                if candidate_count >= self.max_candidate_routes:
                    break
                continue

            for next_lanelet in self.routing_graph.following(
                current_lanelet,
                self.allow_lane_changes_in_search,
            ):
                if next_lanelet.id in visited_ids:
                    continue

                next_path = lanelet_path + [next_lanelet]
                next_visited = set(visited_ids)
                next_visited.add(next_lanelet.id)
                next_cost = path_cost + self._lanelet_length(next_lanelet)
                next_estimate = next_cost + self._heuristic_lanelet_cost(
                    next_lanelet, goal_lanelet)
                heapq.heappush(
                    queue,
                    (next_estimate, next_cost, search_counter, next_path, next_visited),
                )
                search_counter += 1

        return None, 'No path found'

    def build_path_points_abs(self, lanelets, start_point_abs=None, goal_point_abs=None):
        """Assembles a continuous point array from a sequence of Lanelets."""
        path_points_abs = []

        for idx, ll in enumerate(lanelets):
            centerline = [(pt.x, pt.y) for pt in ll.centerline]

            if len(centerline) == 0:
                continue

            if len(lanelets) == 1 and start_point_abs is not None and goal_point_abs is not None:
                centerline = trim_polyline_between_points(
                    centerline, start_point_abs, goal_point_abs, logger=self.logger
                )
            elif idx == 0 and start_point_abs is not None:
                centerline = trim_polyline_from_point(centerline, start_point_abs)
            elif idx == len(lanelets) - 1 and goal_point_abs is not None:
                centerline = trim_polyline_to_point(centerline, goal_point_abs)

            for x_abs, y_abs in centerline:
                append_unique_xy(path_points_abs, x_abs, y_abs)

        return path_points_abs

    def is_lanelet_path_feasible(self, lanelets, start_point_abs, goal_point_abs,
                                 min_turn_radius_threshold=None):
        """Build the absolute path and evaluate its curvature feasibility."""
        path_points_abs = self.build_path_points_abs(
            lanelets,
            start_point_abs=start_point_abs,
            goal_point_abs=goal_point_abs,
        )
        return self.evaluate_path_feasibility(path_points_abs, min_turn_radius_threshold)

    def evaluate_path_feasibility(self, path_points_abs, min_turn_radius_threshold=None):
        """Check if the tightest turn in the trajectory violates the vehicle constraints."""
        if min_turn_radius_threshold is None:
            min_turn_radius_threshold = self.planning_min_turn_radius_m

        min_radius = float('inf')
        step = self.curvature_check_point_step

        if len(path_points_abs) < (2 * step + 1):
            return True, min_radius

        # Highly optimized sliding window via parallel list slicing
        for p1, p2, p3 in zip(path_points_abs, path_points_abs[step:], path_points_abs[2 * step:]):
            radius = circumradius(p1, p2, p3)
            if radius is None:
                continue
            min_radius = min(min_radius, radius)

        if math.isinf(min_radius):
            return True, min_radius

        return min_radius >= min_turn_radius_threshold, min_radius

    # =========================================================================
    # Internal Caching Helpers
    # =========================================================================

    def _lanelet_length(self, lanelet_obj):
        """Calculate and cache the arc length of a Lanelet's centerline."""
        lanelet_id = lanelet_obj.id
        if lanelet_id in self._lanelet_length_cache:
            return self._lanelet_length_cache[lanelet_id]

        pts = [(pt.x, pt.y) for pt in lanelet_obj.centerline]

        # High performance sum utilizing generator expression and pairwise zip
        length = sum(math.hypot(p2[0] - p1[0], p2[1] - p1[1]) for p1, p2 in zip(pts, pts[1:]))

        self._lanelet_length_cache[lanelet_id] = length
        return length

    def _lanelet_center_xy(self, lanelet_obj):
        """Calculate and cache the approximate midpoint of a Lanelet."""
        lanelet_id = lanelet_obj.id
        if lanelet_id in self._lanelet_center_cache:
            return self._lanelet_center_cache[lanelet_id]

        pts = [(pt.x, pt.y) for pt in lanelet_obj.centerline]
        center = pts[len(pts) // 2] if pts else (0.0, 0.0)

        self._lanelet_center_cache[lanelet_id] = center
        return center

    def _heuristic_lanelet_cost(self, lanelet_obj, goal_lanelet):
        """Estimate the A* cost as the Euclidean distance between two Lanelet centers."""
        cx, cy = self._lanelet_center_xy(lanelet_obj)
        gx, gy = self._lanelet_center_xy(goal_lanelet)
        return math.hypot(gx - cx, gy - cy)
