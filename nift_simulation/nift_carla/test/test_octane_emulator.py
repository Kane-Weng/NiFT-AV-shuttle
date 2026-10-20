import threading

from nift_carla.octane_emulator import beacon_payload, OctaneEmulator, sim_iso8601
from nift_sensor.beacon_node import iso8601_to_ros_time, normalize
from nift_sensor.octane_client import KNOTS_TO_MPS
import pytest
import socketio

KEY = 'sim'
NS = '/octane'


@pytest.fixture
def emu():
    e = OctaneEmulator(port=0, api_key=KEY)
    e.start()
    yield e
    e.stop()


class OctaneClient:
    """The calls beacon_node's real branch makes (_init_real_mode), recorded."""

    def __init__(self, port, key=KEY, join=True):
        self.auth_ok = threading.Event()
        self.joined = threading.Event()
        self.updates = []
        self.got = threading.Event()
        self.sio = socketio.Client(reconnection=False)

        @self.sio.on('connect', namespace=NS)
        def _on_connect():
            self.sio.emit('auth', {'x-api-key': key}, namespace=NS)

        @self.sio.on('auth_ok', namespace=NS)
        def _on_auth_ok(_data):
            self.auth_ok.set()
            if join:
                self.sio.emit('join', {'channel': 'beacon'}, namespace=NS,
                              callback=lambda *a: self.joined.set())
                self.sio.emit('join', {'channel': 'v2x_obu_parsed'}, namespace=NS)

        @self.sio.on('beacon_update', namespace=NS)
        def _on_beacon_update(data):
            self.updates.append(data)
            self.got.set()

        self.sio.connect(f'http://127.0.0.1:{port}', namespaces=[NS])

    def close(self):
        self.sio.disconnect()


def test_sim_time_as_octane_iso8601():
    assert sim_iso8601(123_450_000_000) == '1970-01-01T00:02:03.450+00:00'
    # rounded, not truncated
    assert sim_iso8601(123_449_999_999) == '1970-01-01T00:02:03.450+00:00'
    assert sim_iso8601(0) == '1970-01-01T00:00:00.000+00:00'


def test_payload_parses_like_octane_in_beacon_node():
    p = beacon_payload('SIM-FRONT', 123_450_000_000, 42.30051, -83.69872, 278.5, 91.5, 3.25)
    # on the wire the speed is in knots, like the real Octane feed; normalize() gives m/s back
    assert p['state']['dynamics']['velocity'] == pytest.approx(3.25 / KNOTS_TO_MPS)
    assert KNOTS_TO_MPS == pytest.approx(0.514444, abs=1e-6)
    n = normalize(p)
    assert n['source'] == 'state_dynamics'
    assert (n['id'], n['lat'], n['lon'], n['elev'], n['heading']) == \
        ('SIM-FRONT', 42.30051, -83.69872, 278.5, 91.5)
    assert n['speed'] == pytest.approx(3.25)
    stamp = iso8601_to_ros_time(None, n['ts'])      # node only used on a parse failure
    assert stamp.sec == 123
    assert abs(stamp.nanosec - 450_000_000) < 1_000


def test_auth_join_and_beacon_updates(emu):
    c = OctaneClient(emu.port)
    try:
        assert c.auth_ok.wait(3) and c.joined.wait(3)
        assert emu.clients_authed == 1
        sent = [beacon_payload(i, 5_000_000_000, 42.3, -83.7, 270.0, 0.0, 1.0) for i in ('F', 'R')]
        emu.publish(sent)
        assert c.got.wait(3)
        for _ in range(50):
            if len(c.updates) == 2:
                break
            c.got.clear()
            c.got.wait(0.1)
        assert c.updates == sent
    finally:
        c.close()


def test_wrong_key_gets_no_auth_ok_and_no_data(emu):
    c = OctaneClient(emu.port, key='not-the-key')
    try:
        assert not c.auth_ok.wait(1.0)
        assert emu.clients_authed == 0
        emu.publish([beacon_payload('F', 0, 42.3, -83.7, 270.0, 0.0, 0.0)])
        assert not c.got.wait(0.5)
    finally:
        c.close()


def test_no_beacon_updates_before_join(emu):
    c = OctaneClient(emu.port, join=False)
    try:
        assert c.auth_ok.wait(3)
        emu.publish([beacon_payload('F', 0, 42.3, -83.7, 270.0, 0.0, 0.0)])
        assert not c.got.wait(0.5)
    finally:
        c.close()


def test_port_in_use_raises(emu):
    other = OctaneEmulator(port=emu.port, api_key=KEY)
    with pytest.raises(RuntimeError, match='cannot listen'):
        other.start()


def test_publish_before_start_and_after_stop_is_a_no_op():
    e = OctaneEmulator(port=0, api_key=KEY)
    e.publish([beacon_payload('F', 0, 0.0, 0.0, 0.0, 0.0, 0.0)])
    e.start()
    e.stop()
    e.publish([beacon_payload('F', 0, 0.0, 0.0, 0.0, 0.0, 0.0)])
    assert e.emitted == 0
