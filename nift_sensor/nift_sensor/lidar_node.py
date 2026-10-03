#!/usr/bin/env python3
"""
LiDAR obstacle detection node for the NiFT shuttle.

File: lidar_node.py
Author: Kane Weng
Date: April 10, 2026

Description:
  The primary LiDAR perception node for the NiFT shuttle.
  It ingests raw 3D point clouds and outputs 2D obstacle footprints.

  - PIPELINE:
    1. Ground Segmentation: Uses Open3D RANSAC to remove the road surface.
    2. ROI Cropping: Filters the remaining points to a specific 3D box in front of the vehicle.
    3. Height gate (min_obstacle_height > 0): after tilt correction to base_link, drops points
       lower than min_obstacle_height above the ground under the vehicle — road surface the
       single RANSAC plane misses on crowned or sloped roads.
    4. Clustering: Uses Scikit-Learn DBSCAN to group the cropped points into distinct obstacles.
    5. Bounding: Uses SciPy to wrap each cluster in a 2D Convex Hull.
    6. Publishing: Broadcasts the hulls as PolygonStamped messages for the planner, one per
       obstacle, all with the cloud's stamp; a frame with no obstacle publishes one empty
       "clear" polygon, so consumers never keep a stale obstacle.

  - VISUALIZATION & DIAGNOSTICS:
    Publishes debug point clouds (ground, non-ground, front ROI) and a 3D wireframe
    marker of the ROI to RViz. Integrates with the standard ROS diagnostic updater
    to report perception health and data staleness.

Usage:
  ros2 run nift_sensor lidar_node
"""

import time

import diagnostic_msgs.msg
import diagnostic_updater
from geometry_msgs.msg import Point, Point32, PolygonStamped
import numpy as np
import open3d as o3d
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial import ConvexHull
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sklearn.cluster import DBSCAN
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

# ----------------------------
# Utility functions
# ----------------------------


def make_cloud_msg(header, xyz):
    """
    Convert a numpy array of (X, Y, Z) coordinates into a ROS 2 PointCloud2 message.

    Used for debugging and visualization.
    """
    return point_cloud2.create_cloud_xyz32(header, xyz.tolist())


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """
    Reduces point cloud density by keeping one representative point per voxel.

    Uses np.unique on quantized 3D grid indices — fully vectorized, no Python loops.
    Applied before RANSAC to reduce cost across the entire pipeline.
    """
    if voxel_size <= 0.0 or len(points) == 0:
        return points
    quantized = np.floor(points / voxel_size).astype(np.int32)
    _, unique_indices = np.unique(quantized, axis=0, return_index=True)
    return points[unique_indices]


def ransac_ground_segmentation_o3d(points, distance_threshold=0.15, max_iterations=120):
    """
    Apply RANSAC via Open3D to find the largest plane in the cloud (the ground).

    It returns two numpy arrays, the ground points (inliers) and the obstacle points (outliers).
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    _, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold, ransac_n=3, num_iterations=max_iterations)

    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return np.asarray(inlier_cloud.points), np.asarray(outlier_cloud.points)


def dbscan_2d_sklearn(points_xy, eps=0.45, min_points=8):
    """
    Projects points to 2D and clusters them based on density using Scikit-Learn's DBSCAN.

    It returns a list of arrays. Each array holds the indices of the points in one cluster.
    """
    if len(points_xy) == 0:
        return []
    clustering = DBSCAN(eps=eps, min_samples=min_points).fit(points_xy)
    labels = clustering.labels_

    return [np.where(labels == cid)[0] for cid in range(labels.max() + 1)
            if len(np.where(labels == cid)[0]) > 0]


def convex_hull_2d_scipy(points_xy):
    """
    Wrap a 2D cluster of points in a tight bounding polygon using SciPy's ConvexHull.

    It returns the ordered points that make up the perimeter of the shape.
    """
    if len(points_xy) < 3:
        return points_xy
    try:
        return points_xy[ConvexHull(points_xy).vertices]
    except Exception:
        # Fallback to returning raw points if the hull calculation fails (e.g., collinear points)
        return points_xy


def apply_transform_3d(points: np.ndarray, tf) -> np.ndarray:
    """
    Apply a full 3D rigid transform (rotation + translation) to an Nx3 point array.

    Using the full rotation matrix (not just yaw) is required when the source
    frame has pitch or roll — e.g. a downward-tilted LiDAR. Without this,
    simply dropping Z after a yaw-only rotation leaves foreshortened X distances.
    """
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    t = np.array([tf.transform.translation.x,
                  tf.transform.translation.y,
                  tf.transform.translation.z])
    return (R @ points.T).T + t

# ----------------------------
# Node
# ----------------------------


class LidarObstacleNode(Node):

    def __init__(self):
        super().__init__('LidarObstacleNode')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('obstacles_topic', '/obstacles')
        self.declare_parameter('lidar_frame', 'velodyne')
        self.declare_parameter('publish_debug_clouds', True)

        # Algorithm constraints
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ransac_max_iterations', 120),
                ('ransac_distance_threshold', 0.15),
                ('dbscan_eps', 0.45),
                ('dbscan_min_points', 8),
                ('min_cluster_size', 8),
                ('max_cluster_size', 5000),
                ('min_hull_points', 3),
                ('roi.min_x', 0.3),
                ('roi.max_x', 12.0),
                ('roi.min_y', -4.0),
                ('roi.max_y', 4.0),
                ('roi.min_z', -1.5),
                ('roi.max_z', 2.0),
                ('voxel_size', 0.1),
                ('min_obstacle_height', 0.0),   # m above the ground under the vehicle; 0 = off
                # base_link above the ground (URDF base_footprint_joint)
                ('base_link_height', 0.9525),
            ]
        )

        self.ransac_max_iterations = self.get_parameter('ransac_max_iterations').value
        self.ransac_distance_threshold = self.get_parameter('ransac_distance_threshold').value
        self.dbscan_eps = self.get_parameter('dbscan_eps').value
        self.dbscan_min_points = self.get_parameter('dbscan_min_points').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        self.min_hull_points = self.get_parameter('min_hull_points').value
        self.roi = {
            'min_x': self.get_parameter('roi.min_x').value,
            'max_x': self.get_parameter('roi.max_x').value,
            'min_y': self.get_parameter('roi.min_y').value,
            'max_y': self.get_parameter('roi.max_y').value,
            'min_z': self.get_parameter('roi.min_z').value,
            'max_z': self.get_parameter('roi.max_z').value,
        }

        self._voxel_size = self.get_parameter('voxel_size').value
        self.min_obstacle_height = self.get_parameter('min_obstacle_height').value
        self.base_link_height = self.get_parameter('base_link_height').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.publish_debug_clouds = self.get_parameter('publish_debug_clouds').value

        # TF — used to transform sensor-frame points into map frame before 2D projection
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS interface setup
        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter('input_topic').value,
            self._pointcloud_callback,
            2
        )
        self.obstacles_pub = self.create_publisher(
            PolygonStamped, self.get_parameter('obstacles_topic').value, 50)

        if self.publish_debug_clouds:
            self.ground_pub = self.create_publisher(PointCloud2, '/ground_points', 10)
            self.nonground_pub = self.create_publisher(PointCloud2, '/nonground_points', 10)
            self.front_pub = self.create_publisher(PointCloud2, '/front_points', 10)

        self.roi_marker_pub = self.create_publisher(Marker, '/roi_visual', 10)
        self.create_timer(1.0, self.publish_roi_marker)

        # Diagnostics setup
        self.last_cloud_time = None
        self.last_obstacle_count = 0
        self.diag_updater = diagnostic_updater.Updater(self)
        self.diag_updater.setHardwareID('lidar_perception')
        self.diag_updater.add('Obstacle Detection', self._diagnostics_callback)

        self.get_logger().info('LiDAR obstacle publisher is ready')

    # ================= CALLBACKS ====================
    def _diagnostics_callback(self, stat):
        """
        Update the ROS diagnostics system with the current health of the node.

        Warns if data hasn't arrived, and errors if the point cloud stream drops.
        """
        stat.add('Last Obstacle Count', str(self.last_obstacle_count))
        if self.last_cloud_time is None:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.WARN,
                'Waiting for initial PointCloud data...')
        else:
            time_diff = (self.get_clock().now() - self.last_cloud_time).nanoseconds / 1e9
            if time_diff > 2.0:
                stat.summary(
                    diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                    f'PointCloud STALE ({time_diff:.1f}s)')
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Processing normal')
        return stat

    def _pointcloud_callback(self, msg):
        """
        Run the main pipeline on each new PointCloud2 message.

        Orchestrates voxel downsampling, ground removal, ROI cropping, clustering,
        hull generation, and publishing.
        """
        t_start = time.perf_counter()
        self.last_cloud_time = self.get_clock().now()

        # Extract X, Y, Z into an (N, 3) float32 array, ignoring NaNs
        raw_pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)

        if isinstance(raw_pts, np.ndarray):
            # Pre-allocate an empty (N, 3) float32 array
            xyz = np.empty((raw_pts.shape[0], 3), dtype=np.float32)
            # Explicitly map the structured fields to bypass casting errors
            xyz[:, 0] = raw_pts['x']
            xyz[:, 1] = raw_pts['y']
            xyz[:, 2] = raw_pts['z']
        else:
            # Fallback if it returns a generator (list of tuples)
            xyz = np.array(list(raw_pts), dtype=np.float32)

        xyz = voxel_downsample(xyz, self._voxel_size)

        # 1) Ground segmentation using Open3D RANSAC
        ground_pts, nonground_pts = ransac_ground_segmentation_o3d(
            xyz,
            distance_threshold=self.ransac_distance_threshold,
            max_iterations=self.ransac_max_iterations,
        )

        # 2) Vectorized ROI crop — no Python loop
        if len(nonground_pts) > 0:
            r = self.roi
            roi_mask = (
                (nonground_pts[:, 0] >= r['min_x']) & (nonground_pts[:, 0] <= r['max_x']) &
                (nonground_pts[:, 1] >= r['min_y']) & (nonground_pts[:, 1] <= r['max_y']) &
                (nonground_pts[:, 2] >= r['min_z']) & (nonground_pts[:, 2] <= r['max_z'])
            )
            front_pts = nonground_pts[roi_mask]
        else:
            front_pts = np.empty((0, 3), dtype=np.float32)

        # Publish intermediate clouds for RViz debugging
        if self.publish_debug_clouds:
            self.ground_pub.publish(make_cloud_msg(msg.header, ground_pts))
            self.nonground_pub.publish(make_cloud_msg(msg.header, nonground_pts))
            self.front_pub.publish(make_cloud_msg(msg.header, front_pts))

        if len(front_pts) < self.min_cluster_size:
            self.get_logger().debug(
                f'Frame processed in {(time.perf_counter() - t_start) * 1000:.1f} ms '
                f'| front_pts={len(front_pts)} (below min_cluster_size, skipped clustering)'
            )
            self._publish_clear(msg.header.stamp)
            return

        # 3) Correct for sensor tilt: transform points into base_link before projecting to 2D.
        #    Dropping Z in the tilted sensor frame foreshortens X distances. Applying the full
        #    3D rotation to base_link first puts the XY plane truly horizontal.
        #    base_link is a static URDF transform — always available, no localization required.
        try:
            tf = self.tf_buffer.lookup_transform('base_link', self.lidar_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'TF {self.lidar_frame}→base_link not available: {ex}; dropping frame')
            return

        front_pts_base = apply_transform_3d(front_pts, tf)

        # 3b) Height gate: base_link z + base_link_height = height above the ground under the
        #     vehicle
        if self.min_obstacle_height > 0.0:
            above = front_pts_base[:, 2] + self.base_link_height >= self.min_obstacle_height
            front_pts_base = front_pts_base[above]
            if len(front_pts_base) < self.min_cluster_size:
                self._publish_clear(msg.header.stamp)
                return

        # 4) DBSCAN clustering in the flat base_link XY plane
        clusters = dbscan_2d_sklearn(
            front_pts_base[:, :2], eps=self.dbscan_eps, min_points=self.dbscan_min_points)

        obstacle_count = 0
        for cluster_indices in clusters:
            if not (self.min_cluster_size <= len(cluster_indices) <= self.max_cluster_size):
                continue

            cluster_pts_2d = front_pts_base[cluster_indices, :2]

            # 5) Convex Hull using SciPy
            hull_xy = convex_hull_2d_scipy(cluster_pts_2d)
            if len(hull_xy) < self.min_hull_points:
                continue

            # Publish in base_link frame — tilt-corrected and ready for a flat 2D map transform
            poly = PolygonStamped()
            poly.header.stamp = msg.header.stamp
            poly.header.frame_id = 'base_link'
            for p in hull_xy:
                poly.polygon.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

            self.obstacles_pub.publish(poly)
            obstacle_count += 1

        if obstacle_count == 0:
            self._publish_clear(msg.header.stamp)

        self.last_obstacle_count = obstacle_count
        self.get_logger().debug(
            f'Frame processed in {(time.perf_counter() - t_start) * 1000:.1f} ms '
            f'| raw_voxel_pts={len(xyz)} | front_pts={len(front_pts)} | obstacles={obstacle_count}'
        )

    def _publish_clear(self, stamp):
        """Publish an empty polygon when nothing is detected, so consumers drop old obstacles."""
        clear_poly = PolygonStamped()
        clear_poly.header.stamp = stamp
        clear_poly.header.frame_id = 'base_link'
        self.obstacles_pub.publish(clear_poly)
        self.last_obstacle_count = 0

    def publish_roi_marker(self):
        """
        Publish a semi-transparent red wireframe box to RViz.

        The box represents the active 3D Region of Interest constraints.
        """
        marker = Marker()
        marker.header.frame_id = self.lidar_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.a = 0.5
        marker.color.r = 1.0

        x_lim = [self.roi['min_x'], self.roi['max_x']]
        y_lim = [self.roi['min_y'], self.roi['max_y']]
        z_lim = [self.roi['min_z'], self.roi['max_z']]

        def add_line(p1, p2):
            marker.points.extend([
                Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])),
                Point(x=float(p2[0]), y=float(p2[1]), z=float(p2[2]))
            ])

        # Draw the Bottom, Top, and Pillar edges of the box
        for z in z_lim:
            add_line([x_lim[0], y_lim[0], z], [x_lim[1], y_lim[0], z])
            add_line([x_lim[1], y_lim[0], z], [x_lim[1], y_lim[1], z])
            add_line([x_lim[1], y_lim[1], z], [x_lim[0], y_lim[1], z])
            add_line([x_lim[0], y_lim[1], z], [x_lim[0], y_lim[0], z])

        add_line([x_lim[0], y_lim[0], z_lim[0]], [x_lim[0], y_lim[0], z_lim[1]])
        add_line([x_lim[1], y_lim[0], z_lim[0]], [x_lim[1], y_lim[0], z_lim[1]])
        add_line([x_lim[1], y_lim[1], z_lim[0]], [x_lim[1], y_lim[1], z_lim[1]])
        add_line([x_lim[0], y_lim[1], z_lim[0]], [x_lim[0], y_lim[1], z_lim[1]])

        self.roi_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarObstacleNode()
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
