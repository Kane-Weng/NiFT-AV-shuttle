"""
Test nift_sensor's octane_logger end to end against the sim's Octane emulator.

Auth, joins, only the front / rear beacons logged (others counted), relay timing kept,
net probe lines, summary with a rear-antenna gap.
"""
import json
import threading
import time

from nift_carla.octane_emulator import beacon_payload, OctaneEmulator
from nift_sensor.octane_logger import OctaneLogger
import pytest

KEY = 'sim'


@pytest.fixture
def emu():
    e = OctaneEmulator(port=0, api_key=KEY)
    e.start()
    yield e
    e.stop()


def test_logger_records_the_feed(emu, tmp_path):
    cfg = {
        'MCITY_OCTANE_SERVER': f'http://127.0.0.1:{emu.port}',
        'MCITY_OCTANE_KEY': KEY,
        'MCITY_OCTANE_NAMESPACE': '/octane',
        'MCITY_FRONT_BEACON_ID': 'F',
        'MCITY_REAR_BEACON_ID': 'R',
    }
    lines = []
    log = OctaneLogger(cfg, str(tmp_path), net_period=0.2, status_period=0.5, printer=lines.append)
    th = threading.Thread(target=log.run, daemon=True)
    th.start()
    t0 = time.monotonic()
    while emu.clients_authed == 0 or not log.connected:
        assert time.monotonic() - t0 < 10.0, 'logger never authenticated'
        time.sleep(0.05)
    time.sleep(0.3)                                       # joins land
    # 1.5 s at 20 Hz; the rear goes quiet for 0.8 s mid-way
    for k in range(30):
        sim_ns = int((10.0 + 0.05 * k) * 1e9)
        out = [beacon_payload('F', sim_ns, 42.3 + k * 1e-6, -83.7, 277.0, 45.0, 1.0),
               beacon_payload('OTHER', sim_ns, 42.0, -83.0, 270.0, 0.0, 0.0)]
        if not 10 <= k < 26:
            out.append(beacon_payload('R', sim_ns, 42.3, -83.7 + k * 1e-6, 277.0, 45.0, 1.0))
        emu.publish(out)
        time.sleep(0.05)
    time.sleep(0.3)
    log.stop_event.set()
    th.join(10.0)
    assert not th.is_alive()

    recs = [json.loads(x) for x in open(log.path)]
    evs = [r['ev'] for r in recs]
    assert {'connect', 'auth_ok', 'join', 'beacon_update', 'net'} <= set(evs)
    ids = {r['data']['id'] for r in recs if r['ev'] == 'beacon_update'}
    assert ids == {'F', 'R'}                              # OTHER only counted
    assert all(r['t'] > 0 and r['mono'] > 0 for r in recs)
    s = json.load(open(log.path + '.summary.json'))
    assert s['antennas']['F']['new_positions'] >= 25 and s['antennas']['R']['gaps_over_0.5s'] == 1
    assert s['other_beacon_messages'] >= 25
    assert 'state.dynamics.updated' in s['payload_keys']
    assert any('F:' in x and 'R:' in x for x in lines)  # status lines
