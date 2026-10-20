"""
octane_logger's pure parts.

/proc/net/wireless parsing, the per-antenna tracker (new positions, waits, gaps, other beacons,
payload keys). The end-to-end run against the sim's Octane emulator is in nift_carla's tests.
"""
from nift_sensor.octane_logger import read_wireless, Tracker


def payload(bid, updated, lat=42.3, lon=-83.7):
    return {'id': bid, 'state': {'dynamics': {'latitude': lat, 'longitude': lon,
                                              'updated': updated,
                                              'velocity': 1.0, 'heading': 45.0}}}


def test_read_wireless(tmp_path):
    p = tmp_path / 'wireless'
    p.write_text('Inter-| sta-|   Quality        |   Discarded packets'
                 '               | Missed | WE\n'
                 ' face | tus | link level noise |  nwid  crypt   frag  retry'
                 '   misc | beacon | 22\n'
                 'wlp0s20f3: 0000   56.  -54.  -256        0      0      0'
                 '      0     12        0\n')
    assert read_wireless(str(p)) == {'wlp0s20f3': {'link': 56.0, 'level_dbm': -54.0}}
    assert read_wireless(str(tmp_path / 'missing')) == {}


def test_tracker_counts_new_positions_waits_and_gaps():
    tr = Tracker(['F', 'R'])
    assert tr.update(payload('F', '1970-01-01T00:00:01.000+00:00'), t=100.0, mono=1.0)
    # repeat: not new
    assert tr.update(payload('F', '1970-01-01T00:00:01.000+00:00'), t=100.1, mono=1.1)
    assert tr.update(payload('F', '1970-01-01T00:00:01.100+00:00'), t=100.2, mono=1.2)
    # 0.8 s wait: a gap
    assert tr.update(payload('F', '1970-01-01T00:00:02.000+00:00'), t=101.0, mono=2.0)
    assert not tr.update(payload('OTHER', 'x'), t=101.0, mono=2.0)
    s = tr.summary()
    f = s['antennas']['F']
    assert (f['messages'], f['new_positions'], f['gaps_over_0.5s']) == (4, 3, 1)
    assert abs(f['longest_wait_s'] - 0.8) < 1e-9 and f['gaps'][0][1] == 0.8
    assert s['antennas']['R']['messages'] == 0 and s['other_beacon_messages'] == 1
    assert 'state.dynamics.updated' in s['payload_keys']
    assert 'nothing yet' in tr.status(mono=2.0)
