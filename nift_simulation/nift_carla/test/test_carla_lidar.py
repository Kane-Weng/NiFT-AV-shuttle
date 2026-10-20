from nift_carla.carla_lidar import carla_lidar_to_xyz, lidar_attributes, pack_xyz32
import numpy as np
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class _FakeMeasurement:

    def __init__(self, pts):
        self.raw_data = np.asarray(pts, dtype=np.float32).tobytes()


def test_lidar_attributes_full_sweep_per_tick():
    a = lidar_attributes(0.05)
    assert a['rotation_frequency'] == '20.000'
    assert a['points_per_second'] == str(180 * 8 * 20)
    assert a['horizontal_fov'] == '180.000'
    assert a['dropoff_general_rate'] == '0.0'


def test_carla_lidar_to_xyz_flips_y_and_drops_short():
    m = _FakeMeasurement([[1.0, 2.0, 0.5, 0.9], [0.1, 0.1, 0.0, 0.5], [3.0, -1.0, -0.2, 0.1]])
    xyz = carla_lidar_to_xyz(m, min_range=0.3)
    assert xyz.shape == (2, 3)
    np.testing.assert_allclose(xyz[0], [1.0, -2.0, 0.5])
    np.testing.assert_allclose(xyz[1], [3.0, 1.0, -0.2])
    assert carla_lidar_to_xyz(_FakeMeasurement([])).shape == (0, 3)


def test_pack_round_trip():
    xyz = np.array([[1.0, 2.0, 3.0], [-4.0, 5.5, 0.25]], dtype=np.float32)
    hdr = Header()
    hdr.frame_id = 'velodyne'
    msg = pack_xyz32(hdr, xyz)
    assert msg.width == 2 and msg.point_step == 12 and msg.header.frame_id == 'velodyne'
    back = np.array(
        [[p[0], p[1], p[2]] for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'))]
    )
    np.testing.assert_allclose(back, xyz)
