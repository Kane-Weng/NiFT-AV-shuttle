"""
beacon_node -> shuttle_tf_node: a shuttle at a map pose gives that pose back on the TF.

Each TF pairs a fix with the heading of the same beacon_node sample (fix first, then heading).

The antennas' lat/lon come from the node's own projector, the heading from beacon_node's
rear -> front bearing (true north), so the grid convergence between true and map north
(1.82 deg at Mcity) is in the loop.
"""
import math

from builtin_interfaces.msg import Time
import lanelet2
from nift_common.vehicle import ShuttleConfig
from nift_sensor.beacon_node import BeaconNode, compute_heading_from_beacons
from nift_sensor.shuttle_tf_node import ShuttleDynamicBroadcaster
import pytest
import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

ANTENNA_X = ShuttleConfig().gnss_front_x  # antennas at +-this; shuttle_tf's default gnss.offset_x


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = ShuttleDynamicBroadcaster()
    sent = []
    n.tf_broadcaster.sendTransform = sent.append
    n.sent = sent
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _latlon(node, x, y):
    gps = node.projector.reverse(
        lanelet2.core.BasicPoint3d(x + node.map_origin_x, y + node.map_origin_y, 0.0))
    return gps.lat, gps.lon


def _sample(node, x, y, yaw, sec=0):
    """beacon_node's messages for a shuttle at map (x, y, yaw): (/gnss/fix, /gnss/heading_deg)."""
    c, s = math.cos(yaw), math.sin(yaw)
    lat_f, lon_f = _latlon(node, x + ANTENNA_X * c, y + ANTENNA_X * s)
    lat_r, lon_r = _latlon(node, x - ANTENNA_X * c, y - ANTENNA_X * s)
    _, heading_deg = compute_heading_from_beacons(lat_r, lon_r, lat_f, lon_f)
    fix = NavSatFix()
    fix.header.stamp.sec = sec
    fix.latitude, fix.longitude = lat_f, lon_f
    return fix, Float64(data=heading_deg)


def _pose(tf):
    t = tf.transform
    return t.translation.x, t.translation.y, 2.0 * math.atan2(t.rotation.z, t.rotation.w)


def _drive(node, x, y, yaw):
    """Feed a shuttle at map (x, y, yaw) as beacon_node does (fix, then heading); return the TF."""
    fix, heading = _sample(node, x, y, yaw)
    node.gps_fix_callback(fix)
    node.heading_callback(heading)
    return _pose(node.sent[-1])


def test_true_north_is_grid_convergence_east_of_map_north(node):
    assert math.degrees(node.true_north_yaw) == pytest.approx(90.0 - 1.82, abs=0.01)


@pytest.mark.parametrize('yaw_deg', [90.0, -90.0, 0.0, 180.0, 45.0, -135.0, -91.6])
@pytest.mark.parametrize('x, y', [(0.0, 0.0), (101.5, 162.7), (-43.0, -72.0)])
def test_pose_round_trip(node, x, y, yaw_deg):
    px, py, pyaw = _drive(node, x, y, math.radians(yaw_deg))
    d = pyaw - math.radians(yaw_deg)
    err = math.degrees(math.atan2(math.sin(d), math.cos(d)))
    assert err == pytest.approx(0.0, abs=0.01)      # was 1.82 deg (yaw = 90 - heading)
    assert math.hypot(px - x, py - y) < 0.005


def _yaw_err_deg(a, b):
    return abs(math.degrees(math.atan2(math.sin(a - b), math.cos(a - b))))


def test_each_fix_takes_the_heading_of_its_own_sample(node):
    """
    Pair each fix with the heading of its own sample.

    A turn: the heading changes 3 deg per sample. Each TF must carry the yaw of the fix's own
    sample, not the previous one (the old node paired each fix with the heading before it).
    """
    node.pending_fix = None
    for i in range(1, 6):
        x, y, yaw = 10.0 + i, 20.0, math.radians(30.0 + 3.0 * i)
        fix, heading = _sample(node, x, y, yaw, sec=i)
        node.gps_fix_callback(fix)
        node.heading_callback(heading)
        px, py, pyaw = _pose(node.sent[-1])
        assert node.sent[-1].header.stamp.sec == i
        assert _yaw_err_deg(pyaw, yaw) < 0.01
        assert math.hypot(px - x, py - y) < 0.005


def test_a_fix_waits_for_its_heading(node):      # node: the rclpy context
    """Send nothing for the first fix until its heading came (the old node sent yaw 0)."""
    n = ShuttleDynamicBroadcaster()
    sent = []
    n.tf_broadcaster.sendTransform = sent.append
    try:
        fix, heading = _sample(n, 5.0, 5.0, math.radians(120.0), sec=1)
        n.gps_fix_callback(fix)
        assert sent == []
        n.heading_callback(heading)
        assert len(sent) == 1 and _yaw_err_deg(_pose(sent[0])[2], math.radians(120.0)) < 0.01
        n.heading_callback(heading)                     # a heading alone never re-sends a used fix
        assert len(sent) == 1
    finally:
        n.destroy_node()


def test_lost_or_swapped_messages_never_pair_two_samples(node):
    """
    Never pair two samples when a message is lost or late.

    Best-effort QoS can lose a message, and two topics can arrive out of order. A TF is either
    the fix and heading of one sample, or not sent.
    """
    node.pending_fix = None
    poses = [(1.0 * i, 2.0, math.radians(10.0 * i)) for i in range(8)]
    msgs = [_sample(node, *p, sec=i) for i, p in enumerate(poses)]
    before = len(node.sent)
    node.gps_fix_callback(msgs[1][0])                   # heading 1 lost
    node.gps_fix_callback(msgs[2][0])
    node.heading_callback(msgs[2][1])                   # -> sample 2
    node.heading_callback(msgs[3][1])                   # fix 3 lost: nothing
    node.heading_callback(msgs[4][1])                   # sample 4 arrives swapped: nothing yet
    node.gps_fix_callback(msgs[4][0])
    node.gps_fix_callback(msgs[5][0])                   # ... and the next sample is paired again
    node.heading_callback(msgs[5][1])                   # -> sample 5
    out = node.sent[before:]
    assert [tf.header.stamp.sec for tf in out] == [2, 5]
    for tf in out:
        x, y, yaw = poses[tf.header.stamp.sec]
        px, py, pyaw = _pose(tf)
        assert _yaw_err_deg(pyaw, yaw) < 0.01 and math.hypot(px - x, py - y) < 0.005


def test_beacon_node_publishes_the_fix_before_the_heading():
    """Check the order shuttle_tf_node relies on to pair a fix with its own sample's heading."""
    order = []

    class Pub:
        def __init__(self, name):
            self.name = name

        def publish(self, msg):
            order.append(self.name)

    class Beacon:
        pub_fix, pub_heading, pub_vel = Pub('fix'), Pub('heading'), Pub('vel')

    BeaconNode._publish_gnss(Beacon(), 42.3, -83.7, 42.29998, -83.7, 0.0, Time(), 1.0)
    assert order == ['fix', 'heading', 'vel']
