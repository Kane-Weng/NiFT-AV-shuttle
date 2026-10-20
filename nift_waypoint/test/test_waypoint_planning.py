"""
Tests of the route planning in nift_waypoint, on Lanelet2's example map.

The map is mapping_example.osm from ros-humble-lanelet2-maps, a part of Karlsruhe. It is the
map of the package README's demo. Covered here: the S-curve at a lane change
(LaneletPlanner.build_path_points_abs), the manual start from /initialpose and the turning radius
warning of waypoint_node. The node's callbacks are called directly and its path publisher is
recorded, so nothing spins and no DDS round trip is involved.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import lanelet2
from lanelet2.projection import UtmProjector
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import Locations, Participants
from nift_common.vehicle import ShuttleConfig
from nift_waypoint.utils.geometry import append_unique_xy
from nift_waypoint.utils.lanelet_planner import LaneletPlanner
from nift_waypoint.utils.polyline import (
    polyline_length,
    polyline_point_at,
    polyline_slice,
    trim_polyline_from_point,
    trim_polyline_to_point,
)
from nift_waypoint.waypoint_node import WaypointNode
import pytest
import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

MAP = os.path.join(get_package_share_directory('lanelet2_maps'), 'res', 'mapping_example.osm')
ORIGIN = (49.0, 8.4)                    # the README demo's map_origin_lat / _lon
R_FULL = ShuttleConfig().min_turn_radius
R_PLAN = 1.10 * R_FULL                  # planning_min_turn_radius_margin

# On lanelet 45080. The shortest route to the goal starts with a lane change to the left
# (45080 -> 45068), then follows 45070 .. 45076 end to start.
LANE_CHANGE_START, LANE_CHANGE_GOAL = (1231.7, 549.0), (1162.5, 569.6)
# The README demo. Nine lanelets end to start, around a 90 degree bend.
DEMO_START, DEMO_GOAL = (1139.1, 535.3), (1117.1, 592.4)


class Recorder:

    def __init__(self):
        self.items = []

    def publish(self, msg):
        self.items.append(msg)

    def warn(self, msg):
        self.items.append(msg)


def turn_angles_deg(points):
    """Return the change of heading at every inner point of a polyline, in degrees."""
    angles = []
    for p0, p1, p2 in zip(points, points[1:], points[2:]):
        h0 = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        h1 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        angles.append(abs(math.degrees((h1 - h0 + math.pi) % (2.0 * math.pi) - math.pi)))
    return angles


def joined_end_to_start(lanelets, start, goal):
    """Join the centerlines end to start and cut them at start and goal, as the node used to."""
    points = []
    for idx, ll in enumerate(lanelets):
        centerline = [(pt.x, pt.y) for pt in ll.centerline]
        if idx == 0:
            centerline = trim_polyline_from_point(centerline, start)
        elif idx == len(lanelets) - 1:
            centerline = trim_polyline_to_point(centerline, goal)
        for x, y in centerline:
            append_unique_xy(points, x, y)
    return points


# --------------------------------------------------------------------------------------------- #
#  LaneletPlanner on the example map                                                            #
# --------------------------------------------------------------------------------------------- #
@pytest.fixture(scope='module')
def world():
    lanelet_map = lanelet2.io.load(MAP, UtmProjector(lanelet2.io.Origin(*ORIGIN)))
    rules = lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle)
    graph = RoutingGraph(lanelet_map, rules)
    planner = LaneletPlanner(graph, Recorder(), planning_min_turn_radius_m=R_PLAN)
    return lanelet_map, graph, planner


def route(world, start_xy, goal_xy):
    lanelet_map, _, planner = world
    start, goal = lanelet2.core.BasicPoint2d(*start_xy), lanelet2.core.BasicPoint2d(*goal_xy)
    start_ll = lanelet2.geometry.findNearest(lanelet_map.laneletLayer, start, 1)[0][1]
    goal_ll = lanelet2.geometry.findNearest(lanelet_map.laneletLayer, goal, 1)[0][1]
    lanelets, _ = planner.find_feasible_lanelet_path(start_ll, goal_ll)
    return lanelets, start, goal


def successor_flags(graph, lanelets):
    return [any(ll.id == b.id for ll in graph.following(a, False))
            for a, b in zip(lanelets, lanelets[1:])]


def test_polyline_helpers():
    line = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    assert polyline_length(line) == pytest.approx(20.0)
    point, heading = polyline_point_at(line, 15.0)
    assert point == pytest.approx((10.0, 5.0)) and heading == pytest.approx(math.pi / 2)
    assert polyline_point_at(line, -1.0) == ((0.0, 0.0), 0.0)
    assert polyline_point_at(line, 99.0)[0] == (10.0, 10.0)
    assert polyline_point_at([(1.0, 2.0)], 0.0) == ((1.0, 2.0), None)
    assert polyline_slice(line, 5.0, 15.0) == [(5.0, 0.0), (10.0, 0.0), (10.0, 5.0)]


def test_tightest_curve(world):
    """A path that turns back on a line counts as radius 0, a right angle as its corner."""
    planner = world[2]
    back = [(float(x), 0.0) for x in range(6)] + [(float(x), 0.0) for x in range(4, -1, -1)]
    assert planner.tightest_curve(back)[0] == 0.0
    corner = [(float(x), 0.0) for x in range(6)] + [(5.0, float(y)) for y in range(1, 6)]
    radius, where = planner.tightest_curve(corner)
    assert radius == pytest.approx(3.0 / math.sqrt(2.0)) and where == (5.0, 0.0)
    assert planner.tightest_curve([(float(x), 0.0) for x in range(9)]) == (float('inf'), None)


def test_lane_change_moves_across_without_turning_back(world):
    """A route with a lane change gives a smooth path, where joining end to start turned back."""
    _, graph, planner = world
    lanelets, start, goal = route(world, LANE_CHANGE_START, LANE_CHANGE_GOAL)
    assert [ll.id for ll in lanelets[:2]] == [45080, 45068]
    assert successor_flags(graph, lanelets) == [False] + [True] * (len(lanelets) - 2)

    # Joined end to start, the path ran back to the start of 45068: two hairpins of 177 degrees
    assert sorted(turn_angles_deg(joined_end_to_start(lanelets, start, goal)))[-2] > 170.0

    path = planner.build_path_points_abs(lanelets, start, goal)
    assert max(turn_angles_deg(path)) < 15.0
    assert planner.tightest_curve(path)[0] >= R_PLAN
    assert all(b[0] < a[0] for a, b in zip(path, path[1:]))         # always on towards the goal
    assert path[0] == pytest.approx(LANE_CHANGE_START, abs=0.1)
    assert path[-1] == pytest.approx(LANE_CHANGE_GOAL, abs=0.1)
    assert polyline_length(path) == pytest.approx(72.6, abs=1.0)


def test_lane_change_at_the_start_still_goes_forward(world):
    """With the start past the middle of the first lanelet, the S-curve leaves from the start."""
    lanelet_map, _, planner = world
    centerline = [(pt.x, pt.y) for pt in lanelet_map.laneletLayer[45080].centerline]
    start_xy = polyline_point_at(centerline, 0.95 * polyline_length(centerline))[0]
    lanelets, start, goal = route(world, start_xy, LANE_CHANGE_GOAL)
    assert [ll.id for ll in lanelets[:2]] == [45080, 45068]

    path = planner.build_path_points_abs(lanelets, start, goal)
    assert path[0] == pytest.approx(start_xy, abs=1e-6)
    assert max(turn_angles_deg(path)) < 45.0
    assert all(b[0] < a[0] for a, b in zip(path, path[1:]))


def test_route_without_lane_change_is_joined_as_before(world):
    """The README demo route has no lane change, and its path is the same as before the fix."""
    _, graph, planner = world
    lanelets, start, goal = route(world, DEMO_START, DEMO_GOAL)
    assert len(lanelets) == 9 and all(successor_flags(graph, lanelets))
    assert planner.build_path_points_abs(lanelets, start, goal) == (
        joined_end_to_start(lanelets, start, goal))


# --------------------------------------------------------------------------------------------- #
#  waypoint_node: manual start and the turning radius warning                                   #
# --------------------------------------------------------------------------------------------- #
@pytest.fixture(scope='module')
def ros():
    rclpy.init(args=['--ros-args', '-p', f'map_file:={MAP}',
                     '-p', f'map_origin_lat:={ORIGIN[0]}', '-p', f'map_origin_lon:={ORIGIN[1]}',
                     '-p', 'map_publish_period_s:=3600.0'])
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    n = WaypointNode()
    n.path_pub = Recorder()
    yield n
    n.destroy_node()


def send_initial_pose(node, x, y):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.w = 1.0
    node.initial_pose_callback(msg)


def send_fix(node, x, y):
    """Send the GNSS fix of a point given in map coordinates."""
    x_abs, y_abs = node._coord.display_to_lanelet_xy(x, y)
    gps = node.projector.reverse(lanelet2.core.BasicPoint3d(x_abs, y_abs, 0.0))
    node.fix_callback(NavSatFix(latitude=gps.lat, longitude=gps.lon, altitude=gps.alt))


def send_goal(node, x, y):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.w = 1.0
    node.goal_callback(msg)


def start_xy(node):
    return node._coord.to_display_xy(node.latest_start_point_abs.x, node.latest_start_point_abs.y)


def last_path(node):
    return [(p.pose.position.x, p.pose.position.y) for p in node.path_pub.items[-1].poses]


def diagnostics(node):
    return node._diagnostics_callback(DiagnosticStatusWrapper())


def test_manual_start_holds_against_nearby_fixes(node):
    """A start from /initialpose is no longer moved by the next GNSS fix when that fix is near."""
    send_initial_pose(node, *DEMO_START)
    send_fix(node, DEMO_START[0] + 3.0, DEMO_START[1])      # within manual_start_max_offset_m
    assert node.manual_start_selected
    assert start_xy(node) == pytest.approx(DEMO_START)
    send_goal(node, *DEMO_GOAL)
    first = last_path(node)[0]
    assert math.dist(first, DEMO_START) < 0.6                 # the path starts at the manual start
    assert dict((v.key, v.value) for v in diagnostics(node).values)['Start Source'] == (
        'manual (/initialpose)')


def test_far_fix_replaces_manual_start(node):
    """A fix farther than manual_start_max_offset_m from the manual start wins, as before."""
    send_initial_pose(node, *DEMO_START)
    far = (DEMO_START[0] + 20.0, DEMO_START[1])
    send_fix(node, *far)
    assert not node.manual_start_selected
    assert start_xy(node) == pytest.approx(far, abs=1e-6)
    send_fix(node, DEMO_START[0] + 25.0, DEMO_START[1])       # GNSS moves the start again
    assert start_xy(node) == pytest.approx((DEMO_START[0] + 25.0, DEMO_START[1]), abs=1e-6)


def test_turn_radius_warning_on_a_tight_path(node):
    """A path with a curve tighter than full lock is published with a warning and a WARN status."""
    node._force_offroad_callback(Bool(data=True))
    send_initial_pose(node, *DEMO_START)                      # no TF: the heading stays east
    send_goal(node, DEMO_START[0] - 10.0, DEMO_START[1])      # straight behind: it turns back
    assert len(last_path(node)) >= 2                          # still published
    assert node.path_too_tight and node.path_min_radius_m < R_FULL
    stat = diagnostics(node)
    assert stat.level == DiagnosticStatus.WARN and 'tighter' in stat.message
    node._force_offroad_callback(Bool(data=False))


def test_no_warning_on_the_demo_route(node):
    """The README demo route bends gently, so the status stays OK."""
    send_initial_pose(node, *DEMO_START)
    send_goal(node, *DEMO_GOAL)
    assert not node.path_too_tight and node.path_min_radius_m >= R_FULL
    stat = diagnostics(node)
    assert stat.level == DiagnosticStatus.OK
    values = dict((v.key, v.value) for v in stat.values)
    assert values['Tightest Curve'] != 'none' and values['Shuttle Turn Radius'] == (
        f'{R_FULL:.2f} m')
