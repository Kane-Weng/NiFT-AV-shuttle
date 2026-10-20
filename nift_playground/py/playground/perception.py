"""
playground_perception: `/obstacles` from the page's obstacles, only with the `perception` launch option (ARCH 6.3).

It stands in for lidar_node, which needs `/scan`. It publishes what lidar_node publishes: in `base_link`, one PolygonStamped per obstacle with one shared stamp, or one empty polygon to clear, at 10 Hz. It sees only what lies in lidar_node's region of interest (perception_carla.yaml `roi`, a box ahead of the sensor), each polygon clipped to it. The pose is the ground truth from the sim's odom, so this is a perfect sensor: no points, no clustering, no misses.
"""

import math

from geometry_msgs.msg import Point32, PolygonStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import yaml

from . import paths

PERIOD_S = 0.1          # lidar_node runs per scan, the VLP-16 at 10 Hz
ROI_FILE = 'nift_simulation/nift_carla/config/perception_carla.yaml'


def roi_box():
    """lidar_node's ROI (lidar frame) moved to base_link by the sensor's x offset: (min_x, max_x, min_y, max_y)."""
    from nift_common.vehicle import ShuttleConfig
    with open(paths.repo_path(ROI_FILE)) as f:
        roi = yaml.safe_load(f)['lidar_node']['ros__parameters']['roi']
    dx = ShuttleConfig().lidar_x
    return (roi['min_x'] + dx, roi['max_x'] + dx, roi['min_y'], roi['max_y'])


def clip(poly, box):
    """Sutherland-Hodgman: polygon [(x, y)] clipped to the axis-aligned box (min_x, max_x, min_y, max_y)."""
    x0, x1, y0, y1 = box
    edges = ((0, x0, 1), (0, x1, -1), (1, y0, 1), (1, y1, -1))     # (axis, bound, inside sign)
    out = list(poly)
    for axis, bound, sign in edges:
        if not out:
            break
        src, out = out, []
        for i, cur in enumerate(src):
            prev = src[i - 1]
            cin = (cur[axis] - bound) * sign >= 0
            pin = (prev[axis] - bound) * sign >= 0
            if cin != pin:
                k = (bound - prev[axis]) / (cur[axis] - prev[axis])
                out.append((prev[0] + k * (cur[0] - prev[0]), prev[1] + k * (cur[1] - prev[1])))
            if cin:
                out.append(cur)
    return out


class PlaygroundPerception(Node):

    def __init__(self, runner=None, **kwargs):
        super().__init__('playground_perception', **kwargs)
        self.runner = runner
        self.box = roi_box()
        self.pose = None
        self.pub = self.create_publisher(PolygonStamped, '/obstacles', 50)
        self.create_subscription(Odometry, '/ackermann_like_controller/odom', self._odom, 10)
        self.timer = self.create_timer(PERIOD_S, self._tick)

    def _odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.pose = (p.position.x, p.position.y,
                     math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _tick(self):
        if self.pose is None:
            return
        stamp = self.get_clock().now().to_msg()
        x0, y0, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        polys = []
        for poly in (self.runner.obstacles.values() if self.runner is not None else ()):
            local = [(c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0)) for x, y in poly]
            seen = clip(local, self.box)
            if len(seen) >= 3:
                polys.append(seen)
        if not polys:
            msg = PolygonStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'base_link'
            self.pub.publish(msg)
            return
        for poly in polys:
            msg = PolygonStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'base_link'
            for px, py in poly:
                msg.polygon.points.append(Point32(x=float(px), y=float(py), z=0.0))
            self.pub.publish(msg)
