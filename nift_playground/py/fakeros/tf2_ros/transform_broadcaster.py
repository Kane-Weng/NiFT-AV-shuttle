"""tf2_ros.TransformBroadcaster: /tf, depth 100, as tf2_ros' DynamicBroadcasterQoS."""
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage


class TransformBroadcaster:
    def __init__(self, node, qos=None):
        if qos is None:
            qos = QoSProfile(depth=100)
        self.pub_tf = node.create_publisher(TFMessage, '/tf', qos)

    def sendTransform(self, transform):
        if not isinstance(transform, list):
            if hasattr(transform, '__iter__'):
                transform = list(transform)
            else:
                transform = [transform]
        self.pub_tf.publish(TFMessage(transforms=transform))
