"""
Test the preflight checks that need no hardware.

Registry, CAN link assessment, static_path, the generic outputs check, and the GNSS probe / check
against nift_carla's Octane emulator.
"""

from pathlib import Path
import socket
import threading

from nift_bringup import preflight
from nift_bringup.components import bag, can, Component, gnss, REGISTRY, rviz
from nift_bringup.components.common import FAIL, OK, port_free, SKIP, WARN
from nift_bringup.profiles import COMPONENTS, resolve
import pytest


def plan(mode='real', on=(), **args):
    return resolve({'p': {'description': '', 'mode': mode, 'components': list(on)}}, 'p', args)


class FakeGraph:

    def __init__(self, pubs=None):
        self.pubs = pubs or {}

    def publishers(self, topic):
        return self.pubs.get(topic, [])


def test_every_component_has_an_implementation():
    assert set(REGISTRY) | {'lateral'} == set(COMPONENTS)
    assert all(isinstance(c, Component) and c.name == name for name, c in REGISTRY.items())


def test_actions_is_abstract_and_check_outputs_default_to_nothing():
    class NoActions(Component):
        pass
    with pytest.raises(TypeError):
        NoActions()

    class OnlyActions(Component):

        def actions(self, plan):
            return []
    assert OnlyActions().check(plan(), None) == [] and OnlyActions().outputs(plan()) == []


@pytest.mark.skipif(not (Path(__file__).resolve().parents[2] / '.git').exists(),
                    reason='needs a git checkout: bag_dir() finds the repo by its .git')
def test_bags_go_into_the_repo():
    repo = Path(__file__).resolve().parents[2]
    assert bag.bag_dir() == repo / 'nift_bags'
    assert (repo / 'nift_bags' / 'COLCON_IGNORE').exists()


# ---------------------------------------------------------------- can
def can_link(flags=('UP', 'LOWER_UP'), kind='can', bitrate=500000, state='ERROR-ACTIVE'):
    info = {'info_kind': kind}
    if kind == 'can':
        info['info_data'] = {'state': state, 'bittiming': {'bitrate': bitrate}}
    return {'ifname': 'can0', 'flags': list(flags), 'link_type': 'can', 'linkinfo': info}


@pytest.mark.parametrize('link, level, words', [
    (None, FAIL, 'does not exist'),
    (can_link(), OK, '500 kbit/s'),
    (can_link(flags=('NOARP',)), FAIL, 'DOWN'),
    (can_link(bitrate=250000), FAIL, 'expected 500000'),
    (can_link(state='BUS-OFF'), FAIL, 'BUS-OFF'),
    (can_link(state='ERROR-PASSIVE'), WARN, 'ERROR-PASSIVE'),
    (can_link(kind='vxcan'), OK, 'virtual'),
    ({'flags': ['UP'], 'link_type': 'ether', 'linkinfo': None}, FAIL, 'not a CAN interface'),
])
def test_can_link_assessment(link, level, words):
    out = can.assess_link(link, 'can0')
    assert out[0][0] == level and words in out[0][1]


# ---------------------------------------------------------------- static_path
def test_static_path_needs_two_waypoints(tmp_path):
    good, bad = tmp_path / 'good.yaml', tmp_path / 'bad.yaml'
    good.write_text('waypoints:\n  - [0.0, 0.0]\n  - [10.0, 0.0]\n')
    bad.write_text('waypoints:\n  - [0.0, 0.0]\n')
    check = REGISTRY['static_path'].check
    assert check(plan(scenario_file=str(good)), None) == [(OK, '2 waypoints')]
    assert check(plan(scenario_file=str(bad)), None)[0][0] == FAIL
    assert check(plan(scenario_file=str(tmp_path / 'missing.yaml')), None)[0][0] == FAIL


def test_kinematic_backend_checks_no_carla(tmp_path, monkeypatch):
    monkeypatch.setattr(can, 'port_free', lambda port, host: True)
    scenario = tmp_path / 's.yaml'
    scenario.write_text('waypoints:\n  - [0.0, 0.0]\n  - [10.0, 0.0]\n'
                        'obstacles:\n  - {name: a}\n  - {name: b}\n')
    check = REGISTRY['can'].check
    assert check(plan(mode='sim', sim_backend='kinematic'), None) == [(OK,
                                                                       'no CARLA server needed')]
    out = check(plan(mode='sim', sim_backend='kinematic', scenario_file=str(scenario)), None)
    assert out[0][0] == WARN and "the scenario's 2 obstacle(s) are not simulated" in out[0][1]
    assert check(plan(mode='sim', sim_backend='kinematic',
                      scenario_file=str(tmp_path / 'no.yaml')), None)[0][0] == FAIL
    monkeypatch.setattr(can, 'port_free', lambda port, host: False)
    assert check(plan(mode='sim', sim_backend='kinematic'), None)[0][0] == FAIL


def test_port_free_ignores_time_wait_but_not_a_listener():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', 0))
    srv.listen()
    port = srv.getsockname()[1]
    assert not port_free(port, '127.0.0.1')             # a server listens: taken
    cli = socket.create_connection(('127.0.0.1', port))
    conn, _ = srv.accept()
    # the server closes first: its side waits in TIME_WAIT
    conn.close()
    cli.close()
    srv.close()
    # as after a sim run: the next server can bind
    assert port_free(port, '127.0.0.1')


def test_rviz_needs_a_display(monkeypatch):
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.delenv('WAYLAND_DISPLAY', raising=False)
    assert REGISTRY['rviz'].check(plan(), None)[0][0] == FAIL


def test_satellite_layer_follows_the_generated_urdf(tmp_path, monkeypatch):
    monkeypatch.setenv('DISPLAY', ':0')
    urdf = tmp_path / 'mcity_sat.urdf'
    urdf.write_text('<robot name="mcity_satellite"><link name="map"/></robot>')
    monkeypatch.setattr(rviz, 'satellite_urdf', lambda: urdf)
    out = REGISTRY['rviz'].check(plan(), None)
    assert out[1] == (OK, f'satellite layer: {tmp_path}')
    nodes = REGISTRY['rviz'].actions(plan())
    assert len(nodes) == 2      # rviz2 + the satellite robot_state_publisher
    monkeypatch.setattr(rviz, 'satellite_urdf', lambda: None)
    assert 'satellite layer off' in REGISTRY['rviz'].check(plan(), None)[1][1]
    assert len(REGISTRY['rviz'].actions(plan())) == 1
    # + the top-down window
    assert len(REGISTRY['rviz'].actions(plan(rviz_layout='split'))) == 2


# ---------------------------------------------------------------- generic outputs check
class Stub(Component):

    def __init__(self, check=lambda p, g: [], outputs=('/x',)):
        self._check, self._outputs = check, list(outputs)

    def actions(self, plan):
        return []

    def check(self, plan, graph):
        return self._check(plan, graph)

    def outputs(self, plan):
        return self._outputs


def test_output_already_published_fails(monkeypatch):
    monkeypatch.setitem(REGISTRY, 'bag', Stub())
    out = preflight._check_one('bag', plan(), FakeGraph({'/x': ['/other_launch/node']}))
    assert out == [(FAIL, '/x is already published by /other_launch/node: stop that first')]


def test_free_output_is_reported_when_nothing_else_was_checked(monkeypatch):
    monkeypatch.setitem(REGISTRY, 'bag', Stub())
    assert preflight._check_one('bag', plan(), FakeGraph()) == [(OK, 'nothing else publishes /x')]


def test_a_crashing_check_is_a_fail(monkeypatch):
    monkeypatch.setitem(REGISTRY, 'bag', Stub(check=lambda p, g: 1 / 0))
    out = preflight._check_one('bag', plan(), FakeGraph())
    assert out[0][0] == FAIL and 'ZeroDivisionError' in out[0][1]


def test_report_names_can_after_the_sim_backend():
    results = [('can', OK, 'fine'), ('gnss', SKIP, 'later')]
    assert '  OK    can            fine' in preflight.report(results, plan=plan())
    assert '  OK    CARLA bridge   fine' in preflight.report(results, plan=plan(mode='sim'))
    assert '  OK    kinematic sim  fine' in preflight.report(
        results, plan=plan(mode='sim', sim_backend='kinematic'))
    assert '  SKIP  gnss           later' in preflight.report(results, plan=plan(mode='sim'))


def test_report_marks_failures():
    results = [('can', OK, 'fine'), ('gnss', FAIL, 'silent')]
    assert preflight.failed(results) and 'checks:=false' in preflight.report(results)
    assert not preflight.failed([('can', OK, 'fine'), ('bag', WARN, 'low')])


# ---------------------------------------------------------------- gnss vs the Octane emulator
@pytest.fixture
def emulator():
    emu_mod = pytest.importorskip('nift_carla.octane_emulator')
    emu = emu_mod.OctaneEmulator(port=0, api_key='k')
    emu.start()
    stop, ids = threading.Event(), []

    def publish():      # 20 Hz, a new `updated` every time
        ns = 0
        while not stop.wait(0.05):
            ns += 50_000_000
            emu.publish([emu_mod.beacon_payload(i, ns, 42.3, -83.7, 270.0, 0.0,
                                                0.0) for i in list(ids)])
    t = threading.Thread(target=publish, daemon=True)
    t.start()
    yield emu, ids
    stop.set()
    t.join()
    emu.stop()


def octane_env(monkeypatch, emu, key='k'):
    for k, v in {'MCITY_OCTANE_SERVER': f'http://127.0.0.1:{emu.port}', 'MCITY_OCTANE_KEY': key,
                 'MCITY_OCTANE_NAMESPACE': '/octane', 'MCITY_FRONT_BEACON_ID': 'F',
                 'MCITY_REAR_BEACON_ID': 'R'}.items():
        monkeypatch.setenv(k, v)


def test_gnss_check_ok_with_both_antennas(monkeypatch, emulator):
    emu, ids = emulator
    ids += ['F', 'R']
    octane_env(monkeypatch, emu)
    out = REGISTRY['gnss'].check(plan(), None)
    assert out[0][0] == OK and 'front and rear' in out[0][1]


def test_gnss_check_names_the_silent_antenna(monkeypatch, emulator):
    emu, ids = emulator
    ids += ['F']
    octane_env(monkeypatch, emu)
    monkeypatch.setattr(gnss, 'PROBE_S', 1.5)
    out = REGISTRY['gnss'].check(plan(), None)
    assert out[0][0] == FAIL and 'rear R' in out[0][1] and 'front' not in out[0][1]


def test_gnss_check_wrong_key(monkeypatch, emulator):
    emu, ids = emulator
    ids += ['F', 'R']
    octane_env(monkeypatch, emu, key='wrong')
    monkeypatch.setattr(gnss, 'PROBE_S', 1.5)
    out = REGISTRY['gnss'].check(plan(), None)
    assert out[0][0] == FAIL and 'no auth_ok' in out[0][1]


def test_gnss_check_is_skipped_in_sim():
    assert REGISTRY['gnss'].check(plan(mode='sim'), None)[0][0] == SKIP
