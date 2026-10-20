"""
Test nift_sensor's beacon_node with its feed checks on the sim's beacon payloads.

The checks are gnss_hygiene. A clean feed is published and its raw deliveries marked FIX;
a swapped or stuck feed is dropped and goes silent instead of being published;
hygiene.enabled:=false publishes it as before. The node connects to the Octane emulator
like in a sim run; payloads go straight into its callback on a controlled clock,
publishers are recorded (no DDS round trips).
"""
import json
import time
from types import SimpleNamespace

from nift_carla.octane_emulator import beacon_payload, OctaneEmulator
from nift_common.vehicle import ShuttleConfig
from nift_sensor.beacon_node import BeaconNode
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import NavSatStatus

HALF = ShuttleConfig().gnss_front_x     # antennas at +-HALF from base_link
M_PER_DEG_LAT = 111_132.0


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def emu():
    e = OctaneEmulator(port=0, api_key='sim')
    e.start()
    yield e
    e.stop()


class Recorder:

    def __init__(self):
        self.items = []

    def publish(self, x):
        self.items.append(x)


class Clock:
    ns = 0

    def now(self):
        return Time(nanoseconds=self.ns)


@pytest.fixture
def make(emu, monkeypatch):
    for k, v in {'MCITY_OCTANE_SERVER': f'http://127.0.0.1:{emu.port}', 'MCITY_OCTANE_KEY': 'sim',
                 'MCITY_OCTANE_NAMESPACE': '/octane', 'MCITY_FRONT_BEACON_ID': 'F',
                 'MCITY_REAR_BEACON_ID': 'R'}.items():
        monkeypatch.setenv(k, v)
    nodes = []

    def _make(**params):
        authed = emu.clients_authed
        node = BeaconNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        t0 = time.monotonic()
        # let the joins land before teardown
        while emu.clients_authed == authed and time.monotonic() - t0 < 5.0:
            time.sleep(0.02)
        time.sleep(0.1)
        node.timer.cancel()
        node.rec = SimpleNamespace(
            fix=Recorder(),
            vel=Recorder(),
            heading=Recorder(),
            front=Recorder(),
            rear=Recorder(),
            hygiene=Recorder(),
        )
        node.pub_fix, node.pub_vel, node.pub_heading = node.rec.fix, node.rec.vel, node.rec.heading
        node.pub_raw = {'front': node.rec.front, 'rear': node.rec.rear}
        node.pub_hygiene = node.rec.hygiene
        node.clock = Clock()
        node.get_clock = lambda: node.clock
        nodes.append(node)
        return node
    yield _make
    for n in nodes:
        n.sio.disconnect()
        n.destroy_node()


def start(node, t0=9.0):
    """1 s at rest first: the antenna baseline is learned below 0.3 m/s."""
    drive(node, t0, 1.0, v=0.0)


def drive(node, t0, secs, v=2.0, swap=False, stuck_at=None):
    """
    Feed both antennas every 50 ms: base_link at the origin until 10 s, then north at v.

    The timer's 50 Hz publish runs after each tick. swap: front / rear positions exchanged;
    stuck_at: the feed repeats the positions of that time.
    """
    for k in range(round(secs / 0.05)):
        t = t0 + 0.05 * k
        node.clock.ns = round(t * 1e9)
        north = v * max(0.0, (t if stuck_at is None or t < stuck_at else stuck_at) - 10.0)
        fixes = [(42.3 + (north + s * HALF) / M_PER_DEG_LAT, -83.7) for s in (1.0, -1.0)]
        if swap:
            fixes.reverse()
        for bid, (lat, lon) in zip('FR', fixes):
            node._handle_payload(beacon_payload(bid, round(t * 1e9), lat, lon, 277.0, 0.0, v))
        node._publish_latest()


def verdicts(node):
    """
    Return the /gnss/hygiene verdicts, without no_pair.

    no_pair is a delivery whose partner is missing, e.g. the very first one.
    """
    return {json.loads(m.data)['verdict'] for m in node.rec.hygiene.items} - {'no_pair'}


def published_after(rec, node, t):
    return (
        [m for m in rec.items if Time.from_msg(m.header.stamp).nanoseconds / 1e9 > t]
        if rec.items
        else []
    )


def test_clean_feed_is_published_and_marked_fix(make):
    node = make()
    start(node)
    drive(node, 10.0, 3.0)
    # from the baseline (0.5 s at rest) on
    assert len(node.rec.fix.items) >= 65
    assert all(abs((h.data + 180) % 360 - 180) < 1.0 for h in node.rec.heading.items)     # north
    assert node.rec.vel.items[-1].twist.linear.x == pytest.approx(2.0, abs=0.01)
    raw = node.rec.front.items + node.rec.rear.items
    # every delivery, as received
    assert len(raw) == 160
    assert all(
        m.status.status == NavSatStatus.STATUS_FIX
        for m in published_after(node.rec.front, node, 11.0)
    )
    assert verdicts(node) == set()
    stat = SimpleNamespace(values={}, summary=lambda *a: None)
    stat.add = lambda k, v: stat.values.__setitem__(k, v)
    node._diagnostics_callback(stat)
    assert float(stat.values['antenna baseline (m)']) == pytest.approx(2 * HALF, abs=0.01)


def test_swapped_antennas_go_silent_instead_of_a_reversed_heading(make):
    node = make()
    start(node)
    drive(node, 10.0, 2.0)
    n_ok = len(node.rec.fix.items)
    drive(node, 12.0, 1.0, swap=True)
    # front 2 m behind itself
    assert verdicts(node) and verdicts(node) <= {'heading', 'baseline', 'older_position'}
    # never published reversed
    assert all(abs((h.data + 180) % 360 - 180) < 1.0 for h in node.rec.heading.items)
    # silent from 0.5 s in (stale gate)
    assert len(node.rec.fix.items) - n_ok <= 10
    assert not published_after(node.rec.fix, node, 12.6)
    assert any(m.status.status == NavSatStatus.STATUS_NO_FIX for m in node.rec.front.items)


def test_stuck_feed_goes_silent(make):
    node = make()
    start(node)
    drive(node, 10.0, 2.0)
    # same positions, fresh stamps
    drive(node, 12.0, 2.0, stuck_at=12.0)
    assert verdicts(node) == {'stuck'}
    # 0.3 s of repeats + the 0.5 s stale gate
    assert not published_after(node.rec.fix, node, 12.85)


def test_reanchor_is_reported(make):
    node = make()
    start(node)
    drive(node, 10.0, 2.0)
    # nothing accepted for > reset_s (2 s)
    drive(node, 12.0, 3.0, stuck_at=12.0)
    # the feed moves again, 6 m further on
    drive(node, 15.0, 1.0)
    verdicts = [json.loads(m.data)['verdict'] for m in node.rec.hygiene.items]
    assert 'reanchored' in verdicts and published_after(node.rec.fix, node, 15.2)


def test_one_antenna_out_its_partner_goes_out_as_no_pair_with_its_position(make):
    """
    Rear silent: the front's new positions have no partner.

    They are NO_FIX on the raw topic (not in the published stream) and on /gnss/hygiene as
    no_pair with lat / lon, for localization_node.
    """
    node = make()
    start(node)
    drive(node, 10.0, 2.0)
    n_raw = len(node.rec.front.items)
    for k in range(20):                                                  # 1 s, front only
        t = 12.0 + 0.05 * k
        node.clock.ns = round(t * 1e9)
        lat = 42.3 + (2.0 * (t - 10.0) + HALF) / M_PER_DEG_LAT
        node._handle_payload(beacon_payload('F', round(t * 1e9), lat, -83.7, 277.0, 0.0, 2.0))
    msgs = [
        json.loads(m.data)
        for m in node.rec.hygiene.items
        if json.loads(m.data)['verdict'] == 'no_pair'
    ]
    # once the rear's latest is > 0.5 s old
    late = [m for m in msgs if m['updated'] > '1970-01-01T00:00:12.5']
    assert len(late) >= 8 and all(m['antenna'] == 'front' for m in late)
    raw = {(m.latitude, m.longitude): m.status.status for m in node.rec.front.items[n_raw:]}
    assert all(raw[(m['lat'], m['lon'])] == NavSatStatus.STATUS_NO_FIX for m in late)


def test_checks_off_publishes_the_swap_as_before(make):
    node = make(**{'hygiene.enabled': False})
    start(node)
    drive(node, 10.0, 2.0)
    drive(node, 12.0, 1.0, swap=True)
    assert node.hygiene is None and node.rec.hygiene.items == []
    # the reversed heading goes out
    assert abs(node.rec.heading.items[-1].data - 180.0) < 1.0


def test_thresholds_are_parameters(make):
    # an int works for a float field
    node = make(**{'hygiene.stuck_s': 1, 'hygiene.antenna_baseline_m': 2.08})
    assert node.hygiene.cfg.stuck_s == 1.0 and node.hygiene.baseline_m == pytest.approx(2.08)
