"""
beacon_node -> shuttle_tf_node: a shuttle at a map pose gives that pose back on the TF.

The antennas' lat/lon come from the node's own projector, the heading from beacon_node's
rear -> front bearing (true north), so the grid convergence between true and map north
(1.82 deg at Mcity) is in the loop.
"""
import math

import lanelet2
from nift_common.vehicle import ShuttleConfig
from nift_sensor.beacon_node import compute_heading_from_beacons
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


def _drive(node, x, y, yaw):
    """Feed the beacons of a shuttle at map (x, y, yaw); return the map -> base_link sent."""
    c, s = math.cos(yaw), math.sin(yaw)
    lat_f, lon_f = _latlon(node, x + ANTENNA_X * c, y + ANTENNA_X * s)
    lat_r, lon_r = _latlon(node, x - ANTENNA_X * c, y - ANTENNA_X * s)
    _, heading_deg = compute_heading_from_beacons(lat_r, lon_r, lat_f, lon_f)
    node.heading_callback(Float64(data=heading_deg))
    fix = NavSatFix()
    fix.latitude, fix.longitude = lat_f, lon_f
    node.gps_fix_callback(fix)
    t = node.sent[-1].transform
    return t.translation.x, t.translation.y, 2.0 * math.atan2(t.rotation.z, t.rotation.w)


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
