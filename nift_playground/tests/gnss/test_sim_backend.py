"""
The sim backend wrapper (ARCH 2.1, 8.4): kinematic_sim_node unchanged, BrowserOctane swapped in, live faults.

The live fault proofs. A "now" preset rebuilds node.gnss from the old GnssSim's state plus the fault. The payloads it
gives must equal those of the same fault scheduled from the launch (real feed, random stream included), and a fault
that has not triggered yet must not change a single payload (ideal feed, which switches to EXACT_FEED).
"""

import inspect

import pytest

import gnss_chain as gc

STEER = (lambda t: 0.0 if t < 6.0 else 6.0)


def record_payloads(c):
    out = []
    publish = c.sim.octane.publish

    def wrapped(payloads):
        out.append((c.sim.sim_ns, [dict(p) for p in payloads]))
        publish(payloads)
    c.sim.octane.publish = wrapped
    return out


@pytest.fixture
def chain():
    made = []

    def make(**kw):
        c = gc.Chain(**kw)
        made.append(c)
        return c
    yield make
    for c in made:
        c.close()


def test_node_runs_unchanged_with_browser_octane(chain):
    from nift_carla import kinematic_sim_node as ksn
    from nift_carla.octane_emulator import OctaneEmulator
    from playground.octane_bridge import BrowserOctane
    c = chain()
    assert ksn.OctaneEmulator is OctaneEmulator                 # put back after construction
    assert type(c.sim.node) is ksn.KinematicSimNode
    assert isinstance(c.sim.node.octane, BrowserOctane)
    c.run_until(2.0)
    assert not c.sim.node._thread.is_alive()                    # the tick thread is never started
    assert c.sim.sim_ns == gc.START_NS + 2_000_000_000
    assert c.sim.octane.emitted == 2 * 41                       # both antennas, ticks 0.00 .. 2.00
    diag = c.sim.node._diagnostics_callback(_Stat())
    assert '1 client(s) authenticated' in diag.values['Octane emulator']


class _Stat:
    def __init__(self):
        self.values = {}

    def add(self, k, v):
        self.values[k] = v

    def summary(self, level, msg):
        self.level, self.msg = level, msg


def test_private_names_the_wrapper_needs():
    """ARCH 12.1: a rename in kinematic_sim_node breaks the playground, so watch the names here."""
    from nift_carla import kinematic_sim_node as ksn
    from nift_carla import sim_gnss
    assert hasattr(ksn, 'OctaneEmulator')
    for name in ('_tick_once', '_diagnostics_callback'):
        assert callable(getattr(ksn.KinematicSimNode, name))
    src = inspect.getsource(ksn.KinematicSimNode.__init__)
    for attr in ('self._sim_ns', 'self.dt', 'self.gnss', 'self.octane = OctaneEmulator(', 'self.octane.start()'):
        assert attr in src, attr
    gsrc = inspect.getsource(sim_gnss.GnssSim.__init__)
    for attr in ('self.faults', 'self._fstate', 'self.exact', 'self.feed'):
        assert attr in gsrc, attr


def test_browser_octane_has_the_emulator_interface():
    from nift_carla.octane_emulator import OctaneEmulator
    from playground.octane_bridge import BrowserOctane
    emu = list(inspect.signature(OctaneEmulator.__init__).parameters)
    ours = list(inspect.signature(BrowserOctane.__init__).parameters)
    assert ours[:len(emu)] == emu
    for name in ('start', 'publish', 'stop', 'clients_authed'):
        assert hasattr(BrowserOctane, name)


@pytest.mark.parametrize('param, value', [('time_factor', 1.0), ('lockstep', True)])
def test_refuses_pacing_parameters(param, value):
    with pytest.raises(ValueError, match=param):
        gc.Chain(sim_params={param: value})


def test_port_taken_raises_like_the_emulator(chain):
    from playground.octane_bridge import BrowserOctane
    c = chain()
    other = BrowserOctane(port=c.sim.octane.port, api_key='x')
    with pytest.raises(RuntimeError, match='cannot listen'):
        other.start()


@pytest.mark.parametrize('fault', [
    {'kind': 'dropout', 'duration_s': 1.5},
    {'kind': 'noise', 'sigma_m': 0.3, 'duration_s': 2.0},     # draws from the shared random stream
    {'kind': 'burst', 'duration_s': 1.0},
])
def test_live_fault_equals_launch_fault_on_the_real_feed(chain, fault):
    t_cmd = 6.0                     # page time of the command
    t_s = t_cmd + 1.0 + 0.05        # what the preset schedules: the next tick, in sim time
    a = chain(gnss_feed='real', steer_deg=STEER, gnss_faults=[dict(fault, t_s=t_s)])
    pa = record_payloads(a)
    a.run_until(12.0)
    b = chain(gnss_feed='real', steer_deg=STEER)
    pb = record_payloads(b)
    b.run_until(t_cmd)
    b.sim.apply_gnss_fault(fault['kind'], {k: v for k, v in fault.items() if k != 'kind'})
    b.run_until(12.0)
    assert pa == pb
    # the same /carla/gnss_fault events, except the scenario name the launch gave its list
    assert [dict(d, scenario=None) for _t, d in a.gnss_fault] == [dict(d, scenario=None) for _t, d in b.gnss_fault]
    assert a.gnss_fault


def test_pending_live_fault_does_not_change_the_trace(chain):
    """Ideal feed: adding a fault switches GnssSim to EXACT_FEED; until it triggers, every payload is the same."""
    a = chain(steer_deg=STEER)
    pa = record_payloads(a)
    a.run_until(12.0)
    b = chain(steer_deg=STEER)
    pb = record_payloads(b)
    b.run_until(4.0)
    b.sim.schedule_fault({'kind': 'dropout', 't_s': 1000.0, 'duration_s': 1.0})
    assert b.sim.node.gnss.exact
    b.run_until(12.0)
    assert pa == pb
    assert (a.fix, a.tf) == (b.fix, b.tf)


def test_two_live_faults_keep_their_events(chain):
    c = chain(steer_deg=STEER)
    c.run_until(3.0)
    c.sim.apply_gnss_fault('jump', {'east_m': 0.4})
    c.run_until(5.0)
    c.sim.apply_gnss_fault('dropout', {'duration_s': 0.5})
    c.run_until(8.0)
    edges = [(d['index'], d['edge'], d['kind'], round(d['t'] - 1.0, 2)) for _t, d in c.gnss_fault]
    assert edges == [(0, 'start', 'jump', 3.05), (0, 'end', 'jump', 3.05),
                     (1, 'start', 'dropout', 5.05), (1, 'end', 'dropout', 5.55)]


def test_presets_and_errors(chain):
    from playground.sim_backend import gnss_fault_presets
    catalogue = gc.load_yaml('nift_simulation', 'nift_carla', 'config', 'gnss_faults.yaml')
    p = gnss_fault_presets(catalogue)
    names = {s['name'] for s in p['scenarios']}
    assert 'outage_30s_straight' in names and 'control_straight' in names
    live = {x['name']: x for x in p['live']}
    assert {'outage', 'outage_front', 'outage_rear', 'repeat'} <= set(live)
    assert live['dropout']['kind'] == 'sim' and live['outage']['kind'] == 'link'
    c = chain()
    with pytest.raises(ValueError, match='not a live preset'):
        c.sim.apply_gnss_fault('outage_30s_straight')
    with pytest.raises(ValueError, match='unknown parameter'):
        c.sim.apply_gnss_fault('outage', {'sigma_m': 1.0})
    with pytest.raises(ValueError):
        c.sim.apply_gnss_fault('noise', {'sigma_m': 0.0})
    msg = c.sim.apply_gnss_fault('dropout')
    assert 'page t=' in msg
    st = c.sim.status()
    assert st['faults'][0]['kind'] == 'dropout' and st['octane']['clients_authed'] == 1
