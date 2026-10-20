"""
Make the CARLA ray-cast LiDAR match the shuttle LiDAR and pack its points.

File: carla_lidar.py
Author: Kane Weng

Description:
  Helpers to make CARLA's `sensor.lidar.ray_cast` look like the Gazebo ray
  sensor defined in shuttle_description/description/lidar.xacro:

    180 horizontal samples over +-90 deg, 8 vertical channels over +-15 deg,
    0.3 .. 12 m range, 10 Hz, PointCloud2 in frame `velodyne`.

  CARLA sweeps `rotation_frequency * horizontal_fov * dt` degrees per tick and
  wraps within `horizontal_fov`, so with rotation_frequency = 1 / fixed_delta
  every simulation tick yields one complete 180 deg sweep.  Points per channel
  per tick = points_per_second / (channels / fixed_delta).

  CARLA lidar points are in the sensor's local UE frame (x fwd, y RIGHT, z up);
  ROS is x fwd, y LEFT, z up  ->  flip y.  Rays with no hit are simply absent.
"""

from typing import Dict

import numpy as np

# Mirrors lidar.xacro
H_SAMPLES = 180
V_CHANNELS = 8
H_FOV_DEG = 180.0
V_FOV_DEG = 15.0
RANGE_MIN = 0.3
RANGE_MAX = 12.0


def lidar_attributes(fixed_delta: float, channels: int = V_CHANNELS,
                     h_samples: int = H_SAMPLES, range_max: float = RANGE_MAX,
                     h_fov_deg: float = H_FOV_DEG, v_fov_deg: float = V_FOV_DEG,
                     noise_stddev: float = 0.0) -> Dict[str, str]:
    """Blueprint attribute dict for sensor.lidar.ray_cast giving one full sweep per tick."""
    ticks_per_second = 1.0 / float(fixed_delta)
    return {
        'channels': str(int(channels)),
        'range': f'{float(range_max):.3f}',
        'upper_fov': f'{float(v_fov_deg):.3f}',
        'lower_fov': f'{-float(v_fov_deg):.3f}',
        'horizontal_fov': f'{float(h_fov_deg):.3f}',
        'rotation_frequency': f'{ticks_per_second:.3f}',
        'points_per_second': str(int(round(h_samples * channels * ticks_per_second))),
        'sensor_tick': '0.0',                 # capture every tick; the bridge decimates
        'dropoff_general_rate': '0.0',        # CARLA default 0.45 — must be zeroed
        'dropoff_intensity_limit': '0.0',
        'dropoff_zero_intensity': '0.0',
        'atmosphere_attenuation_rate': '0.0',
        'noise_stddev': f'{float(noise_stddev):.4f}',
    }


def carla_lidar_to_xyz(measurement, min_range: float = RANGE_MIN) -> np.ndarray:
    """Convert a carla.LidarMeasurement to (N,3) float32 points in ROS axes."""
    # Raw bytes -> float32 array
    buf = np.frombuffer(bytes(measurement.raw_data), dtype=np.float32)
    if buf.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    # Drop intensity, keep x,y,z
    pts = buf.reshape(-1, 4)[:, :3].copy()
    pts[:, 1] *= -1.0                           # UE left-handed -> ROS

    # Filter by min_range
    if min_range > 0.0:
        r2 = np.einsum('ij,ij->i', pts, pts)    # compute distance^2 per point
        pts = pts[r2 >= min_range * min_range]

    return np.ascontiguousarray(pts, dtype=np.float32)


def pack_xyz32(header, xyz: np.ndarray):
    """Pack an (N,3) float32 array into a dense sensor_msgs/PointCloud2 (x, y, z FLOAT32)."""
    from sensor_msgs.msg import PointCloud2, PointField
    xyz = np.ascontiguousarray(xyz, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(xyz.shape[0])
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = xyz.tobytes()
    return msg
