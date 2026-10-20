"""
Test the scoring of sim_gnss_bench (tools/gnss_bench.py).

Believed vs true pose from the latest RECEIVED transform, the fault window from
/carla/gnss_fault events, and the pass rules against a scenario target.
"""
import math

from nift_carla.tools.gnss_bench import belief_errors, check, fault_window
import numpy as np
import pytest


def straight(t0, t1, v=2.0, dt=0.05):
    t = np.arange(t0, t1 + 1e-9, dt)
    return np.c_[t, v * t, np.zeros_like(t), np.zeros_like(t), np.full_like(t, v)]


def test_frozen_belief_error_grows_with_the_distance_driven():
    od = straight(0.0, 4.0)
    # transforms stop at t = 0.95
    live = [(t, t, x, y, yaw) for t, x, y, yaw, _ in od if t < 1.0]
    t, pos, head, age, lat = belief_errors(live, od, 0.5, 3.0)
    assert pos[t <= 0.95] == pytest.approx(0.0)
    assert pos[-1] == pytest.approx(2.0 * (3.0 - 0.95)) and age[-1] == pytest.approx(3.0 - 0.95)
    # lag along the straight: nothing lateral
    assert np.all(head == 0.0) and lat == pytest.approx(0.0)


def test_latest_received_not_latest_stamped_and_nan_before_the_first():
    od = straight(0.0, 1.0)
    # heading 350 deg vs truth 0: 10 deg error; 0.3 m sideways
    tf = [(0.5, 0.5, 1.0, 0.3, math.radians(350.0)),
          # an old stamp that arrives late is still what the lookup gets
          (0.9, 0.2, 0.4, 0.0, 0.0)]
    t, pos, head, age, lat = belief_errors(tf, od, 0.0, 1.0)
    assert np.isnan(pos[t < 0.5]).all()
    k = np.argmin(abs(t - 0.6))
    assert pos[k] == pytest.approx(math.hypot(0.2, 0.3)) and lat[k] == pytest.approx(0.3)
    assert head[k] == pytest.approx(10.0) and age[k] == pytest.approx(0.1)
    k = np.argmin(abs(t - 1.0))
    assert pos[k] == pytest.approx(1.6) and age[k] == pytest.approx(0.8)


def test_fault_window():
    ev = [{'t': 10.0, 'index': 0, 'edge': 'start'}, {'t': 11.5, 'index': 0, 'edge': 'end'},
          {'t': 12.0, 'index': 1, 'edge': 'start'}, {'t': 13.0, 'index': 1, 'edge': 'end'}]
    # + SETTLE_S after the last end
    assert fault_window(ev, t_end=40.0) == (10.0, 15.0)
    # fault 1 never ended: to the end
    assert fault_window(ev[:3], t_end=20.0) == (10.0, 20.0)
    assert fault_window([], t_end=20.0) is None


def test_check():
    good = {'stop': False, 'err_max': 0.05, 'head_err_max': 1.0, 'cte_max': 0.3}
    assert check(good, {'stop': False, 'max_error_m': 0.1}) == (True, [])
    assert check(dict(good, stop=True), {'stop': False}) == (False, ['SAFE_STOP'])
    assert check(good, {'stop': True}) == (False, ['no stop'])
    ok, why = check(dict(good, err_max=0.4, cte_max=None), {'max_error_m': 0.1, 'max_cte_m': 0.5,
                                                            'max_heading_error_deg': 2.0})
    assert not ok and why == ['err_max 0.40 > 0.1 m', 'cte_max n/a']
