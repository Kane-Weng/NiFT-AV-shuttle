"""tf2_ros.TransformListener: /tf (depth 100) and /tf_static (latched) into a Buffer, on the node."""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class TransformListener:
    def __init__(self, buffer, node, *, spin_thread=False, qos=None, static_qos=None):
        if qos is None:
            qos = QoSProfile(depth=100)
        if static_qos is None:
            static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    history=HistoryPolicy.KEEP_LAST)
        self.buffer = buffer
        self.node = node
        self.tf_sub = node.create_subscription(TFMessage, '/tf', self.callback, qos)
        self.tf_static_sub = node.create_subscription(TFMessage, '/tf_static', self.static_callback,
                                                      static_qos)

    def unregister(self):
        self.node.destroy_subscription(self.tf_sub)
        self.node.destroy_subscription(self.tf_static_sub)

    def callback(self, data):
        who = 'default_authority'
        for transform in data.transforms:
            self.buffer.set_transform(transform, who)

    def static_callback(self, data):
        who = 'default_authority'
        for transform in data.transforms:
            self.buffer.set_transform_static(transform, who)
