"""
Unit tests of the kinematic bicycle model (kinematic_model.py).

The model moves the shuttle in the kinematic sim backend: the shuttle's turning circle,
exact arcs at any tick length, reverse, and the optional steering lag / rate limit.
"""
import math

from nift_carla.kinematic_model import BicycleConfig, KinematicBicycle, wrap_angle
from nift_common.vehicle import ShuttleConfig
import pytest

SHUTTLE = ShuttleConfig()
L = SHUTTLE.wheel_base
LOCK = SHUTTLE.max_steering_deg                 # full lock [deg]
R_FULL = SHUTTLE.min_turn_radius                # the shuttle's full-lock circle (rear axle)


def drive(car, secs, dt, v, steer):
    for _ in range(int(round(secs / dt))):
        car.step(dt, v, steer)


def test_full_lock_radius_is_the_shuttles():
    cfg = BicycleConfig()
    assert (cfg.wheelbase, cfg.max_steer_deg) == (L, LOCK)
    assert L / math.tan(math.radians(LOCK)) == pytest.approx(R_FULL, rel=1e-12)


@pytest.mark.parametrize('dt', [0.01, 0.05, 0.1])
@pytest.mark.parametrize('steer', [LOCK, -LOCK])
def test_full_lock_circle_any_dt(dt, steer):
    car = KinematicBicycle()
    car.reset(10.0, -5.0, 0.4)
    side = math.copysign(1.0, steer)                # + = centre on the left
    cx = car.x - side * R_FULL * math.sin(0.4)
    cy = car.y + side * R_FULL * math.cos(0.4)
    for _ in range(int(round(20.0 / dt))):
        car.step(dt, 1.2, steer)
        assert math.hypot(car.x - cx, car.y - cy) == pytest.approx(R_FULL, abs=1e-6)
    assert car.yaw_rate == pytest.approx(side * 1.2 / R_FULL)


def test_full_circle_returns_to_start():
    car = KinematicBicycle()
    car.reset(3.0, 4.0, -2.0)
    start = car.base_link()
    n, dt = 400, 0.05
    drive(car, n * dt, dt, 2.0 * math.pi * R_FULL / (n * dt), LOCK)
    for a, b in zip(car.base_link(), start):
        assert a == pytest.approx(b, abs=1e-9)


def test_straight_line():
    car = KinematicBicycle()
    car.reset(1.0, 2.0, 0.7)
    drive(car, 1.0, 0.05, 2.0, 0.0)
    x, y, yaw = car.base_link()
    assert (x, y, yaw) == pytest.approx(
        (1.0 + 2.0 * math.cos(0.7), 2.0 + 2.0 * math.sin(0.7), 0.7)
    )
    assert car.v == 2.0 and car.yaw_rate == 0.0


def test_reverse_with_left_lock_turns_the_nose_right():
    car = KinematicBicycle()
    car.reset(0.0, 0.0, 0.0)
    drive(car, 1.0, 0.05, -1.0, LOCK)
    x, _, yaw = car.base_link()
    assert x < 0.0                                  # went backwards
    assert yaw < 0.0 and car.yaw_rate < 0.0         # nose swings right (clockwise)


def test_reset_puts_base_link_mid_wheelbase():
    car = KinematicBicycle()
    car.reset(5.0, 6.0, math.pi / 2)
    assert car.base_link() == pytest.approx((5.0, 6.0, math.pi / 2))
    assert (car.x, car.y) == pytest.approx((5.0, 6.0 - L / 2))
    assert car.v == 0.0 and car.steer_deg == 0.0


def test_yaw_stays_wrapped():
    car = KinematicBicycle()
    drive(car, 100.0, 0.05, 3.0, LOCK)              # about six circles
    assert -math.pi <= car.yaw <= math.pi
    assert wrap_angle(3.0 * math.pi) == pytest.approx(math.pi)


def test_steer_clipped_to_full_lock():
    car = KinematicBicycle()
    car.step(0.05, 0.0, LOCK + 10.0)
    assert car.steer_deg == LOCK
    car.step(0.05, 0.0, -LOCK - 25.0)
    assert car.steer_deg == -LOCK


def test_steer_instant_by_default():
    car = KinematicBicycle()
    car.step(0.05, 0.0, 7.5)
    assert car.steer_deg == 7.5


@pytest.mark.parametrize('dt', [0.01, 0.05])
def test_steer_lag(dt):
    car = KinematicBicycle(BicycleConfig(steer_tau_s=0.2))
    drive(car, 0.2, dt, 0.0, 10.0)
    assert car.steer_deg == pytest.approx(10.0 * (1.0 - math.exp(-1.0)))     # one time constant


def test_steer_rate_limit():
    car = KinematicBicycle(BicycleConfig(steer_rate_deg_s=30.0))
    drive(car, 0.2, 0.05, 0.0, 10.0)
    assert car.steer_deg == pytest.approx(6.0)
    drive(car, 0.2, 0.05, 0.0, 10.0)
    assert car.steer_deg == pytest.approx(10.0)
    drive(car, 0.1, 0.05, 0.0, -10.0)
    assert car.steer_deg == pytest.approx(7.0)
