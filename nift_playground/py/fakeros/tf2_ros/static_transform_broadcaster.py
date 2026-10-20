"""tf2_ros.StaticTransformBroadcaster: /tf_static latched, all transforms sent so far, one per child."""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class StaticTransformBroadcaster:
    def __init__(self, node, qos=None):
        if qos is None:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub_tf = node.create_publisher(TFMessage, '/tf_static', qos)
        self.net_message = TFMessage()
        self._child_frame_ids = set()

    def sendTransform(self, transform):
        if not isinstance(transform, list):
            if hasattr(transform, '__iter__'):
                transform = list(transform)
            else:
                transform = [transform]
        for t_in in transform:
            if t_in.child_frame_id not in self._child_frame_ids:
                self._child_frame_ids.add(t_in.child_frame_id)
                self.net_message.transforms.append(t_in)
            else:
                for i, t in enumerate(self.net_message.transforms):
                    if t_in.child_frame_id == t.child_frame_id:
                        self.net_message.transforms[i] = t_in
        self.pub_tf.publish(self.net_message)
