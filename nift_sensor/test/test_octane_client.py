"""octane_client.normalize(): the two Octane payload schemas -> one dict, speed in m/s."""
from nift_sensor.octane_client import KNOTS_TO_MPS, normalize
import pytest


def _state_dynamics(velocity):
    return {'id': '27386B', 'serviced': '2026-10-08T21:28:06.100+00:00',
            'state': {'dynamics': {'latitude': 42.2988415, 'longitude': -83.6996012,
                                   'elevation': 270.1,
                                   'heading': 46.6, 'velocity': velocity,
                                   'updated': '2026-10-08T21:28:06.080+00:00'}}}


def test_knots_constant():
    assert KNOTS_TO_MPS == pytest.approx(1852.0 / 3600.0)
    assert 1.0 / KNOTS_TO_MPS == pytest.approx(1.944, abs=1e-3)


def test_state_dynamics_velocity_is_knots_converted_to_mps():
    # garage session 2026-10-08: pedal 70 levelled at 1.63 on the wire = 0.84 m/s by position
    n = normalize(_state_dynamics(1.63))
    assert n['source'] == 'state_dynamics'
    assert n['id'] == '27386B'
    assert n['ts'] == '2026-10-08T21:28:06.080+00:00'       # dynamics.updated, not serviced
    assert (n['lat'], n['lon'], n['elev'], n['heading']) == (42.2988415, -83.6996012, 270.1, 46.6)
    assert n['speed'] == pytest.approx(1.63 * KNOTS_TO_MPS)
    assert n['speed'] == pytest.approx(0.839, abs=1e-3)


def test_state_dynamics_missing_velocity_stays_none():
    assert normalize(_state_dynamics(None))['speed'] is None


def test_flat_schema_passes_speed_through():
    p = {'idFixed': '350182', 'updated': 'x', 'latitude': 1.0, 'longitude': 2.0, 'elevation': 3.0,
         'speed': 4.0, 'heading': 5.0}
    n = normalize(p)
    assert n['source'] == 'j2735_bsm'
    assert (n['id'], n['ts'], n['lat'], n['lon'], n['elev'], n['speed'], n['heading']) == \
        ('350182', 'x', 1.0, 2.0, 3.0, 4.0, 5.0)
