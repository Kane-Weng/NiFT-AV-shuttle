#!/usr/bin/env python3
"""
Plan Lanelet2 routes from the shuttle's position to a goal and publish them as a path.

File: waypoint_node.py
Author: Jimmy Wang, Kane Weng
Date: Apr 11, 2026

Description:
  The central orchestration node for the NiFT Shuttle's global navigation.

  Responsibilities:
  - Maintains localization state (GPS/Odometry -> TF -> Lanelet Map).
  - Listens for Goal Poses from RViz or upstream mission control.
  - Delegates graph searches to the LaneletPlanner.
  - Constructs a continuous, 3-phase path (Entry -> Map Graph -> Exit).
  - Delegates all RViz rendering to the WaypointVisualizer.
  - Publishes the final unrolled nav_msgs/Path for the Pure Pursuit controller.
"""

from dataclasses import dataclass
import math

import diagnostic_msgs.msg
import diagnostic_updater
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import lanelet2
from lanelet2.projection import UtmProjector
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import Locations, Participants
from nav_msgs.msg import Odometry, Path
# Physical constants: immutable single source of truth
from nift_bringup.shuttle_config import ShuttleConfig
import rclpy
import rclpy.duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
import rclpy.time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

# Decoupled navigation, math, and rendering modules
from .utils import (
    build_entry_leg_abs,
    build_exit_leg_abs,
    build_offroad_pts_abs,
    CoordTransform,
    downsample_path_points,
    heading_from_quaternion,
    LaneletPlanner,
    WaypointVisualizer,
)


@dataclass
class WaypointConfig:
    """Holds runtime ROS 2 parameters specific to the navigation behavior."""

    map_file: str
    map_origin_lat: float
    map_origin_lon: float
    fix_topic: str
    odom_topic: str
    map_publish_period_s: float
    path_downsample_stride: int
    planning_min_turn_radius_margin: float
    relaxed_turn_radius_margin: float
    curvature_check_point_step: int
    max_candidate_routes: int
    max_search_expansions: int
    allow_lane_changes_in_search: bool
    offroad_distance_threshold_m: float
    bridge_bezier_pts: int


class WaypointNode(Node):
    """ROS 2 Node that bridges localization, map routing, and trajectory publishing."""

    def __init__(self):
        super().__init__('waypoint_node')

        # ---------------------------------------------------------
        # 1. Configuration & Physics Setup
        # ---------------------------------------------------------
        self.shuttle = ShuttleConfig()
        self.config = self._load_parameters()

        if not self.config.map_file:
            self.get_logger().error('map_file parameter is empty')
            raise RuntimeError('map_file parameter is empty')
        if abs(math.tan(self.shuttle.max_steering_angle)) < 1e-6:
            self.get_logger().error('shuttle max_steering_angle is too small')
            raise RuntimeError('shuttle max_steering_angle is too small')

        # ---------------------------------------------------------
        # 2. State Initialization
        # ---------------------------------------------------------
        self.fix_count = 0
        self.odom_count = 0
        self.latest_goal = None

        # Current localization states
        self.latest_map_point_local = None
        self.latest_start_lanelet = None
        self.latest_start_point_abs = None
        self.vehicle_heading_rad = 0.0

        # Goal states
        self.latest_goal_point_local = None
        self.latest_goal_point_abs = None

        self.pending_goal_until_start = False
        self.manual_start_selected = False
        self.force_offroad = False

        # ---------------------------------------------------------
        # 3. Dynamic Constraints Calculations
        # ---------------------------------------------------------
        # Ackermann steering geometry
        self.vehicle_min_turn_radius_m = (
            self.shuttle.wheel_base / math.tan(self.shuttle.max_steering_angle))
        self.planning_min_turn_radius_m = (
            self.vehicle_min_turn_radius_m * self.config.planning_min_turn_radius_margin)

        # ---------------------------------------------------------
        # 4. Map & Coordinate Transformations
        # ---------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'Loading map from: {self.config.map_file}')

        origin = lanelet2.io.Origin(self.config.map_origin_lat, self.config.map_origin_lon)
        self.projector = UtmProjector(origin)
        self.lanelet_map = lanelet2.io.load(self.config.map_file, self.projector)

        projected_origin = self.projector.forward(
            lanelet2.core.GPSPoint(self.config.map_origin_lat, self.config.map_origin_lon, 0.0)
        )

        # Initialize the coordinate transformer and UI visualizer
        self._coord = CoordTransform(projected_origin.x, projected_origin.y)
        self.visualizer = WaypointVisualizer(self, self._coord)

        # ---------------------------------------------------------
        # 5. Routing Graph & Planner Initialization
        # ---------------------------------------------------------
        self.traffic_rules = lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle)
        self.routing_graph = RoutingGraph(self.lanelet_map, self.traffic_rules)
        self.get_logger().info(
            f'Map loaded successfully. Lanelet count: {len(self.lanelet_map.laneletLayer)}')

        self.planner = LaneletPlanner(
            routing_graph=self.routing_graph,
            logger=self.get_logger(),
            max_search_expansions=self.config.max_search_expansions,
            max_candidate_routes=self.config.max_candidate_routes,
            allow_lane_changes_in_search=self.config.allow_lane_changes_in_search,
            curvature_check_point_step=self.config.curvature_check_point_step,
            planning_min_turn_radius_m=self.planning_min_turn_radius_m
        )

        # ---------------------------------------------------------
        # 6. ROS Pub/Sub & Timers
        # ---------------------------------------------------------
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub_fix = self.create_subscription(
            NavSatFix, self.config.fix_topic, self.fix_callback, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(
            Odometry, self.config.odom_topic, self.odom_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.sub_initial_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        self.sub_force_offroad = self.create_subscription(
            Bool, '/force_offroad', self._force_offroad_callback, 10)

        self.path_pub = self.create_publisher(Path, '/planned_path', latched_qos)

        self.diag_updater = diagnostic_updater.Updater(self)
        self.diag_updater.setHardwareID('nav_stack')
        self.diag_updater.add('Lanelet2 Router', self._diagnostics_callback)

        self.visualizer.publish_map_markers(self.lanelet_map)
        self.map_timer = self.create_timer(
            self.config.map_publish_period_s,
            lambda: self.visualizer.publish_map_markers(self.lanelet_map)
        )

    # =========================================================================
    # Diagnostics
    # =========================================================================

    def _diagnostics_callback(self, stat):
        """Report the health and state of the navigation stack to the diagnostic aggregator."""
        stat.add('Map File', self.config.map_file)
        stat.add('GNSS Fix Count', str(self.fix_count))
        stat.add('Force Offroad Enabled', str(self.force_offroad))

        if not hasattr(self, 'lanelet_map') or len(self.lanelet_map.laneletLayer) == 0:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                         'Lanelet2 Map is EMPTY or failed to load!')
        elif self.latest_start_lanelet is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                         'Waiting for valid start location...')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                         'Map loaded and localized to Lanelet network')
        return stat

    # =========================================================================
    # Sensor Callbacks (Localization Updating)
    # =========================================================================

    def fix_callback(self, msg: NavSatFix):
        """Update the vehicle's start position using raw GNSS coordinates."""
        if self.manual_start_selected:
            return

        if self.fix_count == 0:
            self.get_logger().info(f'Received first GPS fix on {self.config.fix_topic}')

        map_point_abs = self.projector.forward(
            lanelet2.core.GPSPoint(msg.latitude, msg.longitude, msg.altitude))
        display_x, display_y = self._coord.to_display_xy(map_point_abs.x, map_point_abs.y)

        self.latest_map_point_local = lanelet2.core.BasicPoint2d(display_x, display_y)
        self.latest_start_point_abs = lanelet2.core.BasicPoint2d(map_point_abs.x, map_point_abs.y)

        # Snap vehicle to the nearest lanelet
        nearest = lanelet2.geometry.findNearest(
            self.lanelet_map.laneletLayer, self.latest_start_point_abs, 1)

        if len(nearest) == 0:
            self.latest_start_lanelet = None
            self.get_logger().warn('No nearest start lanelet found')
            return

        self.latest_start_lanelet = nearest[0][1]
        self.visualizer.publish_start_goal_markers(
            self.latest_map_point_local, self.latest_goal_point_local)

        # If a goal was placed before the GNSS initialized, process it now
        if self.pending_goal_until_start and self.latest_goal is not None:
            self.pending_goal_until_start = False
            self.get_logger().info('Re-processing queued goal_pose after GPS start update')
            self._handle_goal(self.latest_goal)

        self.fix_count += 1

    def odom_callback(self, msg: Odometry):
        """Update the vehicle heading and provides fallback positioning if GNSS fails."""
        self.vehicle_heading_rad = heading_from_quaternion(msg.pose.pose.orientation)
        if self.manual_start_selected:
            return

        # If GNSS is active, rely on TF to get the base_link in the map frame
        if self.fix_count > 0:
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
                self.vehicle_heading_rad = heading_from_quaternion(tf_stamped.transform.rotation)
            except Exception:
                pass
            return

        # Fallback to pure Odometry if TF/GNSS is down
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            start_x = tf_stamped.transform.translation.x
            start_y = tf_stamped.transform.translation.y
            self.vehicle_heading_rad = heading_from_quaternion(tf_stamped.transform.rotation)
        except Exception:
            start_x = msg.pose.pose.position.x
            start_y = msg.pose.pose.position.y

        self.latest_map_point_local = lanelet2.core.BasicPoint2d(start_x, start_y)
        start_x_abs, start_y_abs = self._coord.display_to_lanelet_xy(start_x, start_y)
        self.latest_start_point_abs = lanelet2.core.BasicPoint2d(start_x_abs, start_y_abs)

        nearest = lanelet2.geometry.findNearest(
            self.lanelet_map.laneletLayer, self.latest_start_point_abs, 1)

        if len(nearest) == 0:
            self.latest_start_lanelet = None
            self.visualizer.publish_start_goal_markers(
                self.latest_map_point_local, self.latest_goal_point_local)
            return

        self.latest_start_lanelet = nearest[0][1]
        self.visualizer.publish_start_goal_markers(
            self.latest_map_point_local, self.latest_goal_point_local)
        self.odom_count += 1

        if self.pending_goal_until_start and self.latest_goal is not None:
            self.pending_goal_until_start = False
            self._handle_goal(self.latest_goal)

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Let the user manually override the start position via RViz '2D Pose Estimate'."""
        start_x = msg.pose.pose.position.x
        start_y = msg.pose.pose.position.y

        self.latest_map_point_local = lanelet2.core.BasicPoint2d(start_x, start_y)
        start_x_abs, start_y_abs = self._coord.display_to_lanelet_xy(start_x, start_y)
        self.latest_start_point_abs = lanelet2.core.BasicPoint2d(start_x_abs, start_y_abs)

        nearest = lanelet2.geometry.findNearest(
            self.lanelet_map.laneletLayer, self.latest_start_point_abs, 1)

        if len(nearest) == 0:
            self.latest_start_lanelet = None
            self.visualizer.publish_start_goal_markers(
                self.latest_map_point_local, self.latest_goal_point_local)
            return

        self.latest_start_lanelet = nearest[0][1]
        self.visualizer.publish_start_goal_markers(
            self.latest_map_point_local, self.latest_goal_point_local)

        if self.pending_goal_until_start and self.latest_goal is not None:
            self.pending_goal_until_start = False
            self._handle_goal(self.latest_goal)

    def _force_offroad_callback(self, msg: Bool):
        self.force_offroad = msg.data

    def goal_callback(self, msg: PoseStamped):
        """Triggered when RViz or a mission planner sends a new '2D Nav Goal'."""
        self.latest_goal = msg
        self._handle_goal(msg)

    # =========================================================================
    # Route Planning Pipeline
    # =========================================================================

    def _handle_goal(self, msg: PoseStamped):
        """Validate the goal, pick the offroad fallback if needed, and start planning."""
        if self.latest_start_lanelet is None:
            self.pending_goal_until_start = True
            return

        self.visualizer.clear_route_markers()

        # Convert requested goal to absolute map space
        goal_x_abs, goal_y_abs = self._coord.display_to_lanelet_xy(
            msg.pose.position.x, msg.pose.position.y)
        goal_pt = lanelet2.core.BasicPoint2d(goal_x_abs, goal_y_abs)

        self.latest_goal_point_abs = goal_pt
        self.latest_goal_point_local = lanelet2.core.BasicPoint2d(
            msg.pose.position.x, msg.pose.position.y)

        nearest = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, goal_pt, 1)

        if len(nearest) == 0:
            self.get_logger().warn('No nearest goal lanelet found in map.')
            return

        goal_dist, goal_lanelet = nearest[0]
        self.visualizer.publish_start_goal_markers(
            self.latest_map_point_local, self.latest_goal_point_local)

        # Force offroad fallback if requested or if goal is wildly far from the road network
        if self.force_offroad or goal_dist > self.config.offroad_distance_threshold_m:
            if self.latest_start_point_abs is not None:
                pts_abs = build_offroad_pts_abs(
                    self.latest_start_point_abs,
                    self.vehicle_heading_rad,
                    goal_pt,
                    self.config.bridge_bezier_pts
                )
                path_msg = self._abs_pts_to_path_msg(pts_abs)

                if len(path_msg.poses) >= 2:
                    self.path_pub.publish(path_msg)
                    self.visualizer.publish_route_markers_3phase(pts_abs, [], [])
                    return

            self.publish_empty_path()
            self.visualizer.clear_route_markers()
            return

        self.plan_and_publish(self.latest_start_lanelet, goal_lanelet)

    def plan_and_publish(self, start_lanelet, goal_lanelet):
        """Execute the 3-phase routing strategy: Entry Curve -> Graph Route -> Exit Curve."""
        # Phase 2: Route through the Lanelet2 graph
        lanelets, route_search_summary = self.planner.find_feasible_lanelet_path(
            start_lanelet, goal_lanelet)

        # Fallback to direct offroad path if the graph cannot connect the points
        if not lanelets:
            if self.latest_start_point_abs is not None and self.latest_goal_point_abs is not None:
                pts_abs = build_offroad_pts_abs(
                    self.latest_start_point_abs,
                    self.vehicle_heading_rad,
                    self.latest_goal_point_abs,
                    self.config.bridge_bezier_pts
                )
                path_msg = self._abs_pts_to_path_msg(pts_abs)
                if len(path_msg.poses) >= 2:
                    self.path_pub.publish(path_msg)
                    self.visualizer.publish_route_markers_3phase(pts_abs, [], [])
                    return
            self.publish_empty_path()
            self.visualizer.clear_route_markers()
            return

        # Extract the absolute coordinates of the graph route
        lanelet_pts_abs = self.planner.build_path_points_abs(
            lanelets,
            start_point_abs=self.latest_start_point_abs,
            goal_point_abs=self.latest_goal_point_abs,
        )

        # Phase 1: Smooth entry from unconstrained vehicle pose to the start of the lanelet
        entry_pts_abs = build_entry_leg_abs(
            self.latest_start_point_abs,
            self.vehicle_heading_rad,
            lanelet_pts_abs,
            self.config.bridge_bezier_pts
        )

        # Phase 3: Smooth exit from the lanelet to the unconstrained goal pose
        exit_pts_abs = build_exit_leg_abs(
            lanelet_pts_abs,
            self.latest_goal_point_abs,
            self.config.bridge_bezier_pts
        )

        # Combine, downsample, and publish
        all_pts_abs = entry_pts_abs + lanelet_pts_abs + exit_pts_abs
        sampled_pts = downsample_path_points(all_pts_abs, self.config.path_downsample_stride)

        if len(sampled_pts) < 2:
            self.publish_empty_path()
            self.visualizer.clear_route_markers()
            return

        path_msg = self._abs_pts_to_path_msg(sampled_pts)
        self.path_pub.publish(path_msg)
        self.visualizer.publish_route_markers_3phase(entry_pts_abs, lanelet_pts_abs, exit_pts_abs)

    def publish_empty_path(self):
        """Send an empty path to halt the controller."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        self.path_pub.publish(path_msg)

    # =========================================================================
    # Helpers
    # =========================================================================

    def _abs_pts_to_path_msg(self, pts_abs):
        """Convert an array of absolute UTM points into a formatted nav_msgs/Path."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Pre-bind the header and orientation to avoid redundant creation in the loop
        for x_abs, y_abs in pts_abs:
            px, py = self._coord.to_display_xy(x_abs, y_abs)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        return path_msg

    def _load_parameters(self) -> WaypointConfig:
        """Load, validate, and return ROS parameters inside a typed WaypointConfig dataclass."""
        self.declare_parameter('map_file', '')
        self.declare_parameter('map_origin_lat', 42.3005)
        self.declare_parameter('map_origin_lon', -83.6987)
        self.declare_parameter('fix_topic', '/gnss/fix')
        self.declare_parameter('odom_topic', '/ackermann_like_controller/odom')
        self.declare_parameter('map_publish_period_s', 1.0)
        self.declare_parameter('path_downsample_stride', 5)
        self.declare_parameter('planning_min_turn_radius_margin', 1.10)
        self.declare_parameter('relaxed_turn_radius_margin', 0.85)
        self.declare_parameter('curvature_check_point_step', 3)
        self.declare_parameter('max_candidate_routes', 40)
        self.declare_parameter('max_search_expansions', 3000)
        self.declare_parameter('allow_lane_changes_in_search', False)
        self.declare_parameter('offroad_distance_threshold_m', 15.0)
        self.declare_parameter('bridge_bezier_pts', 8)

        return WaypointConfig(
            map_file=self.get_parameter('map_file').value,
            map_origin_lat=self.get_parameter('map_origin_lat').value,
            map_origin_lon=self.get_parameter('map_origin_lon').value,
            fix_topic=self.get_parameter('fix_topic').value,
            odom_topic=self.get_parameter('odom_topic').value,
            map_publish_period_s=self.get_parameter('map_publish_period_s').value,
            path_downsample_stride=max(1, int(self.get_parameter('path_downsample_stride').value)),
            planning_min_turn_radius_margin=float(
                self.get_parameter('planning_min_turn_radius_margin').value),
            relaxed_turn_radius_margin=float(
                self.get_parameter('relaxed_turn_radius_margin').value),
            curvature_check_point_step=max(
                1, int(self.get_parameter('curvature_check_point_step').value)),
            max_candidate_routes=max(1, int(self.get_parameter('max_candidate_routes').value)),
            max_search_expansions=max(1, int(self.get_parameter('max_search_expansions').value)),
            allow_lane_changes_in_search=bool(
                self.get_parameter('allow_lane_changes_in_search').value),
            offroad_distance_threshold_m=float(
                self.get_parameter('offroad_distance_threshold_m').value),
            bridge_bezier_pts=max(4, int(self.get_parameter('bridge_bezier_pts').value)),
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C race: rclpy's SIGINT handler can shut the context down mid-construction or
        # mid-spin (RCLError). With the context still alive it is a real error.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()    # Ctrl-C: rclpy's SIGINT handler has already shut the context down


if __name__ == '__main__':
    main()
