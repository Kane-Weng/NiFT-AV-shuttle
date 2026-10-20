"""The socket.io stand-in and the import stubs, without ROS: the client API beacon_node and octane_client use."""

import io
import logging
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PY = os.path.abspath(os.path.join(HERE, '..', '..', 'py'))
for sub in ('fakesocketio', 'stubs', '.'):
    p = os.path.normpath(os.path.join(PY, sub))
    if p not in sys.path:
        sys.path.insert(0, p)

import socketio  # noqa: E402
from socketio.hub import Hub, ManualScheduler  # noqa: E402


@pytest.fixture
def net():
    hub = Hub()
    sched = ManualScheduler()
    hub.scheduler = sched
    return hub, sched


def octane_like(hub, url='http://127.0.0.1:8765', ns='/octane', key='sim'):
    """A server with octane_emulator.py's handlers."""
    srv = hub.listen(url)
    seen = {'authed': set(), 'log': []}

    def on_auth(sid, data):
        if data.get('x-api-key') != key:
            seen['log'].append('rejected')
            return
        seen['authed'].add(sid)
        srv.emit('auth_ok', {}, to=sid, namespace=ns)

    def on_join(sid, data):
        if sid in seen['authed']:
            srv.enter_room(sid, data['channel'], namespace=ns)

    srv.on('connect', lambda sid, environ, auth=None: seen['log'].append(('connect', sid)), namespace=ns)
    srv.on('auth', on_auth, namespace=ns)
    srv.on('join', on_join, namespace=ns)
    srv.on('disconnect', lambda sid, reason=None: seen['log'].append(('disconnect', sid, reason)), namespace=ns)
    return srv, seen


def beacon_like(hub, events, key='sim', ns='/octane'):
    """The handlers of beacon_node._init_real_mode, verbatim in shape (the disconnect handler takes no reason)."""
    sio = socketio.Client(reconnection=True, reconnection_attempts=0, reconnection_delay=1, hub=hub)

    @sio.on('connect', namespace=ns)
    def _on_connect():
        events.append('connect')
        sio.emit('auth', {'x-api-key': key}, namespace=ns)

    @sio.on('auth_ok', namespace=ns)
    def _on_auth_ok(_data):
        events.append('auth_ok')
        sio.emit('join', {'channel': 'beacon'}, namespace=ns)
        sio.emit('join', {'channel': 'v2x_obu_parsed'}, namespace=ns)

    @sio.on('beacon_update', namespace=ns)
    def _on_beacon_update(data):
        events.append(('beacon', data['id']))

    @sio.on('disconnect', namespace=ns)
    def _on_disconnect():
        events.append('disconnect')

    return sio


def test_handshake_auth_join_and_beacon(net):
    hub, sched = net
    srv, seen = octane_like(hub)
    events = []
    sio = beacon_like(hub, events)
    sio.connect('http://127.0.0.1:8765', namespaces=['/octane'])
    assert sio.connected and events == []           # nothing is delivered inside connect()
    sched.run()
    assert events == ['connect', 'auth_ok']
    assert len(seen['authed']) == 1
    assert srv.rooms_of(sio.get_sid('/octane'), '/octane') == ['beacon', 'v2x_obu_parsed']
    n = srv.emit('beacon_update', {'id': 'SIM-FRONT'}, room='beacon', namespace='/octane')
    assert n == 1 and ('beacon', 'SIM-FRONT') not in events     # not inside emit()
    sched.run()
    assert events[-1] == ('beacon', 'SIM-FRONT')
    assert hub.errors == []


def test_wrong_key_gets_no_reply(net):
    hub, sched = net
    srv, seen = octane_like(hub)
    events = []
    sio = beacon_like(hub, events, key='wrong')
    sio.connect('http://localhost:8765/', namespaces=['/octane'])  # same server: localhost, trailing slash
    sched.run()
    assert events == ['connect'] and seen['log'][-1] == 'rejected'
    srv.emit('beacon_update', {'id': 'SIM-FRONT'}, room='beacon', namespace='/octane')
    sched.run()
    assert events == ['connect']


def test_unknown_url_raises_connection_error(net):
    hub, _ = net
    sio = socketio.Client(hub=hub)
    with pytest.raises(socketio.exceptions.ConnectionError):
        sio.connect('http://127.0.0.1:9999', namespaces=['/octane'])
    assert not sio.connected


def test_unknown_namespace_fails_and_emit_before_connect_raises(net):
    hub, _ = net
    octane_like(hub)
    sio = socketio.Client(hub=hub)
    with pytest.raises(socketio.exceptions.BadNamespaceError):
        sio.emit('auth', {}, namespace='/octane')
    with pytest.raises(socketio.exceptions.ConnectionError):
        sio.connect('http://127.0.0.1:8765', namespaces=['/nope'])


def test_already_connected(net):
    hub, sched = net
    octane_like(hub)
    sio = beacon_like(hub, [])
    sio.connect('http://127.0.0.1:8765', namespaces=['/octane'])
    with pytest.raises(socketio.exceptions.ConnectionError, match='Already connected'):
        sio.connect('http://127.0.0.1:8765', namespaces=['/octane'])


def test_server_shutdown_calls_legacy_disconnect_handler(net):
    """python-socketio 5.x passes a reason and retries without it for a handler that takes none (beacon_node's)."""
    hub, sched = net
    srv, seen = octane_like(hub)
    events = []
    sio = beacon_like(hub, events)
    sio.connect('http://127.0.0.1:8765', namespaces=['/octane'])
    sched.run()
    srv.shutdown()
    sched.run()
    assert events[-1] == 'disconnect'
    assert not sio.connected
    assert seen['log'][-1][2] == socketio.Client.reason.SERVER_DISCONNECT
    assert hub.errors == []
    assert 'http://127.0.0.1:8765' not in hub.servers     # the address is free again


def test_client_disconnect_reason_and_server_side(net):
    hub, sched = net
    srv, seen = octane_like(hub)
    reasons = []
    sio = socketio.Client(hub=hub)
    sio.on('disconnect', lambda reason: reasons.append(reason), namespace='/octane')
    sio.on('connect', lambda: None, namespace='/octane')
    sio.connect('http://127.0.0.1:8765', namespaces=['/octane'])
    sched.run()
    sio.disconnect()
    assert reasons == [socketio.Client.reason.CLIENT_DISCONNECT]     # at once, as python-socketio
    sched.run()
    assert seen['log'][-1][0] == 'disconnect' and srv.sids == []


def test_handler_exception_is_logged_and_delivery_goes_on(net, caplog):
    hub, sched = net
    srv, _ = octane_like(hub)
    events = []
    sio = beacon_like(hub, events)
    sio.on('beacon_update', lambda data: 1 / 0 if data['id'] == 'BAD' else events.append(data['id']),
           namespace='/octane')
    sio.connect('http://127.0.0.1:8765', namespaces=['/octane'])
    sched.run()
    with caplog.at_level(logging.ERROR, logger='engineio.client'):
        srv.emit('beacon_update', {'id': 'BAD'}, room='beacon', namespace='/octane')
        srv.emit('beacon_update', {'id': 'GOOD'}, room='beacon', namespace='/octane')
        sched.run()
    assert events[-1] == 'GOOD'
    assert len(hub.errors) == 1 and 'ZeroDivisionError' in hub.errors[0]['traceback']
    assert 'message handler error' in caplog.text


def test_old_connection_deliveries_are_dropped(net):
    hub, sched = net
    srv, _ = octane_like(hub)
    events = []
    sio = beacon_like(hub, events)
    sio.connect('http://127.0.0.1:8765', namespaces=['/octane'])
    sched.run()
    srv.emit('beacon_update', {'id': 'SIM-FRONT'}, room='beacon', namespace='/octane')
    sio.disconnect()
    sched.run()
    assert ('beacon', 'SIM-FRONT') not in events


def test_probe_signature(net):
    """octane_client.probe() builds Client(reconnection=False) and passes wait_timeout."""
    hub, sched = net
    octane_like(hub)
    sio = socketio.Client(reconnection=False, hub=hub)
    sio.connect('http://127.0.0.1:8765', namespaces=['/octane'], wait_timeout=2.5)
    assert sio.connected
    sio.disconnect()


def test_address_in_use(net):
    hub, _ = net
    hub.listen('http://127.0.0.1:8765')
    with pytest.raises(RuntimeError, match='in use'):
        hub.listen('http://localhost:8765')


def test_server_classes_raise():
    with pytest.raises(NotImplementedError):
        socketio.AsyncServer(async_mode='aiohttp')


# ---------------------------------------------------------------------- stubs
def test_dotenv_values(tmp_path):
    import dotenv
    p = tmp_path / '.env'
    p.write_text('# comment\nexport A=1\nB="two words"\nC=x # trailing\nD\nE=\'q\'\n')
    assert dotenv.dotenv_values(p) == {'A': '1', 'B': 'two words', 'C': 'x', 'D': None, 'E': 'q'}
    assert dotenv.dotenv_values(None) == {}
    assert dotenv.dotenv_values(stream=io.StringIO('K=v')) == {'K': 'v'}


def test_aiohttp_imports_and_raises_on_use():
    from aiohttp import web
    with pytest.raises(NotImplementedError):
        web.Application()
    assert not hasattr(web, '__wrapped__')        # introspection still works


def test_pyproj_constructs_and_raises_on_transform():
    import pyproj
    crs = pyproj.CRS.from_proj4('+proj=tmerc +lat_0=42.3 +lon_0=-83.7')
    tr = pyproj.Transformer.from_crs('EPSG:4326', crs, always_xy=True)
    with pytest.raises(NotImplementedError):
        tr.transform(-83.7, 42.3)
