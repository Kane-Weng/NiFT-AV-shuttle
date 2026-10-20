import math

from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
from nift_common.vehicle import ShuttleConfig, VehicleState, YAML_PATH
import yaml


def test_every_yaml_value_is_a_field():
    raw = yaml.safe_load(YAML_PATH.read_text())
    cfg = ShuttleConfig()
    for key, value in raw.items():
        assert getattr(cfg, key) == value, key


def test_values():
    cfg = ShuttleConfig()
    assert cfg.wheel_base == 2.082
    assert cfg.max_steering_deg == 15.0
    assert cfg.max_steering_angle == math.radians(15.0)
    assert cfg.max_speed == 4.91744


def test_full_lock_never_exceeds_the_limit_in_degrees():
    # 0.2618 rad was 15.000035°: over the CAN limit, a warning on every full-lock frame
    assert math.degrees(ShuttleConfig().max_steering_angle) <= ShuttleConfig().max_steering_deg


def test_derived():
    cfg = ShuttleConfig()
    assert cfg.base_link_x == 1.041
    assert abs(cfg.min_turn_radius - 7.770) < 1e-3
    assert abs(cfg.max_curvature * cfg.min_turn_radius - 1.0) < 1e-12


def test_frozen_and_state():
    cfg = ShuttleConfig()
    try:
        cfg.wheel_base = 3.0
    except Exception:
        pass
    assert cfg.wheel_base == 2.082
    assert VehicleState().vel == 0.0


def test_site():
    assert (MAP_ORIGIN_LAT, MAP_ORIGIN_LON) == (42.3005, -83.6987)
