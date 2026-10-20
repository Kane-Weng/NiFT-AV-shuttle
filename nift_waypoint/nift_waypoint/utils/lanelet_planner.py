"""
Route on Lanelet2 networks, join the route into a path and measure its curves.

File: lanelet_planner.py
Author: Kane Weng
Date: Apr 11, 2026

Description:
  Graph search and path building for Lanelet2 networks.
  The node takes Lanelet2's shortest route (find_feasible_lanelet_path), joins its
  centerlines into one path (build_path_points_abs, with an S-curve at each lane change)
  and measures the path's tightest curve (tightest_curve).
  search_alternative_feasible_path is an A* search that rejects routes tighter than
  the shuttle can turn. The node does not call it.
"""

import heapq
import math

from .geometry import append_unique_xy, circumradius, cubic_bezier_pts, segment_heading
from .polyline import (
    polyline_length,
    polyline_point_at,
    polyline_slice,
    project_point_to_polyline,
    trim_polyline_between_points,
    trim_polyline_from_point,
    trim_polyline_to_point,
)

# A lane change in a route (a left or right neighbour, not a successor) is driven on a cubic
# S-curve from one lane onto the other. Its half lengths are tried in this order, and the first
# curve that stays within the planning turning radius is used. Between straight lanes 3.5 m apart
# the 10 m halves already bend at about 19 m radius, so the longer ones are for wider moves,
# several lanes at once, or a lane change on a bend.
LANE_CHANGE_HALF_LENGTHS_M = (10.0, 15.0, 20.0, 30.0)
LANE_CHANGE_POINT_SPACING_M = 1.0   # point spacing on that S-curve


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
        """Return Lanelet2's shortest route. It is not checked against the turning radius."""
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
        """
        Assemble a continuous point array from a sequence of Lanelets.

        Lanelets that succeed each other are joined end to start, and the first and last
        centerlines are cut at the start and the goal. A lane change splits the route into runs.
        The path leaves one run and joins the next on an S-curve, so it moves across to the
        neighbouring lane instead of going back to the start of its lanelet.
        """
        runs, heads, tails = self._route_runs(lanelets, start_point_abs, goal_point_abs)
        if len(runs) == 1:
            return runs[0]
        return self._join_runs(runs, heads, tails)

    def _is_successor(self, lanelet_a, lanelet_b):
        """Tell whether lanelet_b follows lanelet_a end to start, as opposed to a lane change."""
        return any(ll.id == lanelet_b.id for ll in self.routing_graph.following(lanelet_a, False))

    def _route_runs(self, lanelets, start_point_abs, goal_point_abs):
        """
        Split a route at its lane changes into runs of lanelets joined end to start.

        Side by side lanelets of a lane change form one group. The path keeps to the first one up
        to the middle of the group and goes on from the middle of the last one, so a new run starts
        there. Returns the runs as point lists, plus the lane heading at the start (heads) and the
        end (tails) of each run where a lane change cut it, else None.
        """
        groups = []
        for ll in lanelets:
            if groups and not self._is_successor(groups[-1][-1], ll):
                groups[-1].append(ll)
            else:
                groups.append([ll])

        runs, heads, tails = [[]], [None], [None]
        for idx, group in enumerate(groups):
            first = idx == 0 and start_point_abs is not None
            last = idx == len(groups) - 1 and goal_point_abs is not None
            centerline = [(pt.x, pt.y) for pt in group[0].centerline]

            if len(centerline) == 0:
                continue

            if len(group) == 1:
                if first and last:
                    centerline = trim_polyline_between_points(
                        centerline, start_point_abs, goal_point_abs, logger=self.logger
                    )
                elif first:
                    centerline = trim_polyline_from_point(centerline, start_point_abs)
                elif last:
                    centerline = trim_polyline_to_point(centerline, goal_point_abs)

                for x_abs, y_abs in centerline:
                    append_unique_xy(runs[-1], x_abs, y_abs)
                continue

            target = [(pt.x, pt.y) for pt in group[-1].centerline]
            from_len, to_len = polyline_length(centerline), polyline_length(target)
            lo = self._fraction_along(centerline, start_point_abs) if first else 0.0
            hi = self._fraction_along(target, goal_point_abs) if last else 1.0
            if hi < lo:
                self.logger.warn('Goal projects behind start across a lane change; '
                                 'using direct clipped segment')
                append_unique_xy(runs[-1], *polyline_point_at(centerline, lo * from_len)[0])
                append_unique_xy(runs[-1], *polyline_point_at(target, hi * to_len)[0])
                continue

            middle = min(max(0.5, lo), hi)
            for x_abs, y_abs in polyline_slice(centerline, lo * from_len, middle * from_len):
                append_unique_xy(runs[-1], x_abs, y_abs)
            tails[-1] = polyline_point_at(centerline, middle * from_len)[1]
            runs.append([])
            heads.append(polyline_point_at(target, middle * to_len)[1])
            tails.append(None)
            for x_abs, y_abs in polyline_slice(target, middle * to_len, hi * to_len):
                append_unique_xy(runs[-1], x_abs, y_abs)

        return runs, heads, tails

    @staticmethod
    def _fraction_along(polyline, point_abs):
        """Return where a point projects onto a polyline, as a share of its length."""
        length = polyline_length(polyline)
        if length <= 0.0:
            return 0.0
        return project_point_to_polyline(polyline, point_abs)['arc_length'] / length

    def _join_runs(self, runs, heads, tails):
        """Join the runs of a route with one S-curve per lane change."""
        lengths = [polyline_length(run) for run in runs]
        path_points_abs = []
        resume_arc = 0.0    # where the run picks up after the previous S-curve
        for idx, run in enumerate(runs):
            if idx == len(runs) - 1:
                for x_abs, y_abs in polyline_slice(run, resume_arc, lengths[idx]):
                    append_unique_xy(path_points_abs, x_abs, y_abs)
                break

            # A run between two lane changes gives half of its length to each S-curve
            nxt = idx + 1
            max_back = lengths[idx] if idx == 0 else lengths[idx] / 2.0
            max_ahead = lengths[nxt] if nxt == len(runs) - 1 else lengths[nxt] / 2.0
            curve, back, ahead = self._lane_change_curve(
                run, runs[nxt], tails[idx], heads[nxt], max_back, max_ahead)

            for x_abs, y_abs in polyline_slice(run, resume_arc, lengths[idx] - back) + curve:
                append_unique_xy(path_points_abs, x_abs, y_abs)
            resume_arc = ahead

        return path_points_abs

    def _lane_change_curve(self, from_run, to_run, from_heading, to_heading, max_back,
                           max_ahead):
        """
        Build the S-curve that leaves from_run and joins to_run at a lane change.

        It starts up to max_back before the end of from_run and ends up to max_ahead into
        to_run, along the lane at both ends. The half lengths of LANE_CHANGE_HALF_LENGTHS_M are
        tried in turn, and the first curve within the planning turning radius is kept. If none
        is, the longest curve that fits is used, and the node's turning radius check reports it.
        Returns the curve's points and the lengths it took from both runs.
        """
        from_len = polyline_length(from_run)
        for half in LANE_CHANGE_HALF_LENGTHS_M:
            back, ahead = min(half, max_back), min(half, max_ahead)
            p0, h0 = polyline_point_at(from_run, from_len - back)
            p3, h3 = polyline_point_at(to_run, ahead)
            h0 = h0 if h0 is not None else from_heading
            h3 = h3 if h3 is not None else to_heading
            if h0 is None or h3 is None:
                h0 = h3 = segment_heading(*p0, *p3)
            n_pts = max(8, int(math.ceil(math.hypot(p3[0] - p0[0], p3[1] - p0[1])
                                         / LANE_CHANGE_POINT_SPACING_M)))
            # Handles of a third of the chord spread the move across evenly: between straight
            # lanes d apart over a length L the curve bends at about L^2 / (6 d) at its ends
            curve = cubic_bezier_pts(p0, h0, p3, h3, n_pts, handle_scale=1.0 / 3.0)
            feasible, _ = self.evaluate_path_feasibility(curve)
            if feasible or (back >= max_back and ahead >= max_ahead):
                break
        return curve, back, ahead

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

        min_radius, _ = self.tightest_curve(path_points_abs)

        if math.isinf(min_radius):
            return True, min_radius

        return min_radius >= min_turn_radius_threshold, min_radius

    def tightest_curve(self, path_points_abs):
        """
        Find the tightest curve of a path.

        The radius is measured through every three points that are curvature_check_point_step
        apart. Three points in a line that turn back count as radius 0, since the circle through
        them is a straight line too. Returns the smallest radius and the middle point of its
        three points, or (inf, None) when the path is too short or straight.
        """
        min_radius, where = float('inf'), None
        step = self.curvature_check_point_step

        # Highly optimized sliding window via parallel list slicing
        for p1, p2, p3 in zip(path_points_abs, path_points_abs[step:], path_points_abs[2 * step:]):
            radius = circumradius(p1, p2, p3)
            if radius == float('inf') and ((p2[0] - p1[0]) * (p3[0] - p2[0])
                                           + (p2[1] - p1[1]) * (p3[1] - p2[1])) < 0.0:
                radius = 0.0
            if radius is not None and radius < min_radius:
                min_radius, where = radius, p2

        return min_radius, where

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
