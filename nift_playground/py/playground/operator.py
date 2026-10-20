"""
playground_operator: what the page's buttons do, as a ROS node (ARCH 6.3 and 8.1).

It publishes `/goal_pose`, `/initialpose`, `/teleop_cmd_vel` and the 0x100 E-stop frame, and calls the Trigger services, the way nift_web and RViz do on the shuttle. It is ours, not repo code. It has no timers, so it never changes when other nodes' timers fire.
"""

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nift_control.dbw_interface.can_interface import create_estop_frame
from rclpy.node import Node
from std_srvs.srv import Trigger
from can_msgs.msg import Frame

# page cmd name -> service (ARCH 8.1)
SERVICES = {
    'start': '/start_navigation',
    'estop': '/trigger_estop',
    'stop': '/safe_stop',
    'resume': '/resume',
    'manual': '/toggle_manual_mode',
    'power_cycle': '/carla/dbw_power_cycle',
}

# RViz's 2D Pose Estimate covariance (x, y, yaw), what the shuttle's operators send
_INITIALPOSE_COV = {0: 0.25, 7: 0.25, 35: 0.06853891909122467}


def _quat(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class PlaygroundOperator(Node):

    def __init__(self, **kwargs):
        kwargs.pop('runner', None)
        super().__init__('playground_operator', **kwargs)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_teleop = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        self.pub_can = self.create_publisher(Frame, '/to_can_bus', 10)
        self._triggers = {name: self.create_client(Trigger, srv) for name, srv in SERVICES.items()}

    def goal(self, x: float, y: float, yaw=None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = _quat(float(yaw or 0.0))
        self.pub_goal.publish(msg)

    def initialpose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = _quat(float(yaw))
        for i, v in _INITIALPOSE_COV.items():
            msg.pose.covariance[i] = v
        self.pub_initialpose.publish(msg)

    def teleop(self, v: float, steer: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(steer)
        self.pub_teleop.publish(msg)

    def dbw_estop(self):
        self.pub_can.publish(create_estop_frame(True, self.get_clock().now().to_msg()))

    def call(self, name: str, drain=None):
        """
        Call the Trigger service of page command `name`; return (success, message).

        `drain` runs the world's queue until the reply is there, for a fakeros whose `call_async` answers on the queue.
        """
        client = self._triggers[name]
        srv = SERVICES[name]
        ready = getattr(client, 'service_is_ready', None)
        if ready is not None and not ready():
            return False, f'{srv} is not offered (is its node running?)'
        req = Trigger.Request()
        if hasattr(client, 'call_async'):
            fut = client.call_async(req)
            if not fut.done() and drain is not None:
                drain()
            if not fut.done():
                return False, f'{srv}: no reply'
            resp = fut.result()
        else:
            resp = client.call(req)
        if resp is None:
            return False, f'{srv}: no reply'
        return bool(resp.success), str(resp.message)
