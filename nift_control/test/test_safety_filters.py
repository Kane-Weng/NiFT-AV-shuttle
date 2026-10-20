"""
Test that the safety filters plan with the shuttle's measured braking.

The filters assume 0.5 m/s² after 0.7 s, so a shuttle that brakes only that hard still stops
near min_safe_dist. Also tested: no filter keeps a stale close reading, and the rear check that
MANUAL uses while reversing.
"""

import math

from nift_common.vehicle import VehicleState
from nift_control.filters import (
    DistanceStopFilter,
    FilterStatus,
    KinematicCBFFilter,
    PotentialFieldAvoidanceFilter,
)
from nift_control.filters.filter_utils import (
    _min_obstacle_distance,
    BRAKE_DECEL_MPS2,
    BRAKE_DELAY_S,
    braking_speed_cap,
    stopping_distance,
    VEHICLE_RADIUS,
)
import numpy as np
import pytest

MIN_SAFE = 2.0


def at(clearance, side=1, lateral=0.0):
    """One obstacle point clearance m from the bounding edge, ahead (+1) or behind (-1)."""
    return np.array([[side * (clearance + VEHICLE_RADIUS), lateral]])


def state(v=0.0, x=0.0):
    return VehicleState(x=x, y=0.0, yaw=0.0, vel=v)


# ---------------------------------------------------------------- the braking model

def test_measured_braking_constants():
    """Use the weak end of what the shuttle measured: 0.5 to 0.65 m/s², 0.5 to 0.7 s delay."""
    assert BRAKE_DECEL_MPS2 == 0.5 and BRAKE_DELAY_S == 0.7


@pytest.mark.parametrize('v, dist', [(0.0, 0.0), (1.0, 1.7), (1.5, 3.3), (2.0, 5.4), (-2.0, 5.4)])
def test_stopping_distance(v, dist):
    assert stopping_distance(v) == pytest.approx(dist)


@pytest.mark.parametrize('room', [0.05, 0.5, 1.7, 3.3, 5.4, 12.0])
def test_braking_speed_cap_is_the_inverse_of_stopping_distance(room):
    assert stopping_distance(braking_speed_cap(room)) == pytest.approx(room)


def test_braking_speed_cap_edges():
    assert braking_speed_cap(0.0) == 0.0 and braking_speed_cap(-1.0) == 0.0
    assert braking_speed_cap(float('inf')) == float('inf')
    assert braking_speed_cap(5.4) == pytest.approx(2.0)
    assert braking_speed_cap(5.4, decel=2.0, delay=0.0) > 4.0      # a stronger brake allows more


# ---------------------------------------------------------------- CBF

def test_cbf_never_asks_for_more_braking_than_measured():
    """
    Cap the CBF speed at braking_speed_cap(h).

    The old cap gamma * h asked for gamma * v of deceleration, 2 m/s² at 2 m/s. The new cap is
    never above braking_speed_cap(h), nor above the old one.
    """
    f = KinematicCBFFilter(min_safe_dist=MIN_SAFE, gamma=1.0)
    for clearance in np.linspace(2.05, 12.0, 60):
        v, _ = f.filter(10.0, 0.0, state(), at(clearance))
        h = clearance - MIN_SAFE
        assert v <= braking_speed_cap(h) + 1e-9 and v <= h + 1e-9
        assert v == pytest.approx(min(h, braking_speed_cap(h)))
    assert f.filter_status == FilterStatus.OBJECT_DETECTED


@pytest.mark.parametrize('v, clearance', [(1.0, 3.7), (1.5, 5.3), (2.0, 7.4)])
def test_cbf_starts_slowing_where_a_real_stop_needs_it(v, clearance):
    """At speed v the cap bites at min_safe_dist + stopping_distance(v), not min_safe_dist + v."""
    f = KinematicCBFFilter(min_safe_dist=MIN_SAFE, gamma=1.0)
    assert f.filter(v, 0.0, state(v), at(clearance + 0.01))[0] == pytest.approx(v)
    assert f.filter(v, 0.0, state(v), at(clearance - 0.05))[0] < v


def test_cbf_blocked_at_the_margin_and_clear_without_obstacles():
    f = KinematicCBFFilter(min_safe_dist=MIN_SAFE)
    assert f.filter(1.0, 0.1, state(), at(MIN_SAFE - 0.1)) == (0.0, 0.1)
    assert f.filter_status == FilterStatus.BLOCKED
    assert f.filter(1.0, 0.1, state(), np.array([])) == (1.0, 0.1)
    assert f.filter_status == FilterStatus.CLEAR and f.last_obs_dist == float('inf')


# ---------------------------------------------------------------- distance_stop

@pytest.mark.parametrize('v, old, new', [
    (0.5, 2.75, 2.75), (1.0, 3.5, 3.7), (1.5, 4.25, 5.3), (2.0, 5.0, 7.4),
])
def test_distance_stop_margin_covers_a_real_stop(v, old, new):
    """Stop at min_safe_dist + max(safe_dist_gain * v, stopping_distance(v)), never closer."""
    f = DistanceStopFilter(MIN_SAFE, 1.5)
    assert f.filter(v, 0.0, state(v), at(new + 0.01))[0] == v
    assert f.filter_status == FilterStatus.CLEAR
    assert f.filter(v, 0.0, state(v), at(new - 0.01))[0] == 0.0
    assert f.filter_status == FilterStatus.BLOCKED
    assert new >= old


def test_filters_take_the_braking_from_their_arguments():
    weak = DistanceStopFilter(MIN_SAFE, 1.5, brake_decel=0.25, brake_delay=1.0)
    assert weak.filter(1.0, 0.0, state(1.0), at(MIN_SAFE + 2.9))[0] == 0.0   # needs 3.0 m
    cbf = KinematicCBFFilter(MIN_SAFE, brake_decel=0.25, brake_delay=1.0)
    assert cbf.filter(5.0, 0.0, state(), at(MIN_SAFE + 3.0))[0] == pytest.approx(1.0)


# ---------------------------------------------------------------- closed loop

def approach(f, v0, start=12.0, dt=0.01):
    """
    Drive a point shuttle at v0 towards a standing obstacle start m ahead of its edge.

    It follows the filter's speed (10 Hz path loop) with the measured brake, 0.5 m/s² at most,
    0.7 s after the command, and never speeds up. Return the clearance and the speed after 60 s.
    """
    obstacle = np.array([[start + VEHICLE_RADIUS, 0.0]])
    x, v, target = 0.0, v0, v0
    pending = [v0] * int(round(BRAKE_DELAY_S / dt))      # commands still in the dead time
    for k in range(int(60 / dt)):
        if k % 10 == 0:
            target, _ = f.filter(v0, 0.0, state(v, x), obstacle)
        pending.append(target)
        cmd = pending.pop(0)
        if cmd < v:
            v = max(cmd, v - BRAKE_DECEL_MPS2 * dt)
        x += v * dt
    return start - x, v


FILTERS = {
    'cbf': lambda **kw: KinematicCBFFilter(MIN_SAFE, **kw),
    'distance_stop': lambda **kw: DistanceStopFilter(MIN_SAFE, 1.5, **kw),
    'potential_field': lambda **kw: PotentialFieldAvoidanceFilter(**kw),
}


@pytest.mark.parametrize('name', FILTERS)
@pytest.mark.parametrize('v0', [1.0, 1.5, 2.0])
def test_a_shuttle_with_the_measured_brake_stops_near_the_margin(name, v0):
    """
    Stop within the last 0.35 m of the 2 m margin, far from the 0.8 m collision zone.

    The 10 Hz loop adds up to 0.1 s to the brake's delay, so the stop can reach a little into
    the margin.
    """
    clearance, v = approach(FILTERS[name](), v0)
    assert v < 0.01 and clearance > MIN_SAFE - 0.35, (v, clearance)


@pytest.mark.parametrize('name, v0', [('cbf', 1.5), ('cbf', 2.0), ('distance_stop', 2.0)])
def test_the_old_strong_brake_assumption_ran_into_the_obstacle(name, v0):
    """
    Show what the filters did before, with a brake strong enough to stop at once.

    The shuttle with the measured brake then ends inside the collision zone or past the
    obstacle.
    """
    clearance, _ = approach(FILTERS[name](brake_decel=1e9, brake_delay=0.0), v0)
    assert clearance < 0.8


# ---------------------------------------------------------------- stale readings

@pytest.mark.parametrize('make', [
    lambda: KinematicCBFFilter(MIN_SAFE),
    lambda: DistanceStopFilter(MIN_SAFE, 1.5),
    lambda: PotentialFieldAvoidanceFilter(),
])
def test_no_stale_close_reading_once_the_obstacle_is_gone(make):
    """
    Forget the nearest obstacle once it is gone.

    last_obs_dist feeds the collision-zone SAFE_STOP. The potential field filter kept the last
    close value when the obstacles cleared or moved behind, so the next start went straight
    back to SAFE_STOP.
    """
    f = make()
    f.filter(1.0, 0.0, state(), at(0.5))
    assert f.last_obs_dist == pytest.approx(0.5)
    f.filter(1.0, 0.0, state(), np.array([]))
    assert f.last_obs_dist == float('inf') and f.filter_status == FilterStatus.CLEAR
    f.filter(1.0, 0.0, state(), at(0.5))
    f.filter(1.0, 0.0, state(), at(0.5, side=-1))       # now behind the shuttle
    assert f.last_obs_dist == float('inf') and f.filter_status == FilterStatus.CLEAR


# ---------------------------------------------------------------- the rear check (MANUAL)

def test_min_obstacle_distance_ahead_or_behind():
    s = state()
    behind, ahead = at(0.4, side=-1), at(3.0)
    both = np.vstack([behind, ahead])
    assert _min_obstacle_distance(s, behind) == float('inf')
    assert _min_obstacle_distance(s, behind, behind=True) == pytest.approx(0.4)
    assert _min_obstacle_distance(s, both) == pytest.approx(3.0)
    assert _min_obstacle_distance(s, both, behind=True) == pytest.approx(0.4)
    assert _min_obstacle_distance(s, ahead, behind=True) == float('inf')
    turned = VehicleState(x=0.0, y=0.0, yaw=math.pi, vel=0.0)      # heading -x: behind is +x
    assert _min_obstacle_distance(turned, ahead, behind=True) == pytest.approx(3.0)
