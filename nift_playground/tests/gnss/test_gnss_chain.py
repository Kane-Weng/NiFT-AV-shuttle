"""
The GNSS chain with the repo nodes unchanged (ARCH 2.2, 10).

kinematic_sim_node -> BrowserOctane -> fake socket.io -> beacon_node -> shuttle_tf_node (direct) or localization_node (ekf). The TF map -> base_link must match the sim's true pose within the GNSS noise, and GNSS faults must trip beacon_node's stale gate and the localization status as they do on ROS (tests/reference/runs/gnss_outage30s_01: /gnss/* paused 0.55 s after the last fix).

Needs from fakeros: `world.reset`, `world.now_ns`, `world.drain`, `world.params` keyed by node name, `world.log_sink(level, node, text)`, and timers. Where wave B is not there yet, gnss_chain.py falls back on the wave A world (FAKEROS-FALLBACK).
"""

import math

import pytest

import gnss_chain as gc

STEER = (lambda t: 0.0 if t < 8.0 else (8.0 if t < 16.0 else -6.0))     # straight, left, right
SETTLED_NS = gc.START_NS + 3_000_000_000


def t_rel(ns):
    return (ns - gc.START_NS) / 1e9


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


def test_handshake_before_first_tick(chain):
    """ARCH 6.1: the runner drains until beacon_node has logged auth_ok, with no sim time passing."""
    c = chain()
    assert c.sim.octane.clients_authed == 1
    assert c.logs_matching('beacon_node', 'auth_ok, joining channels')
    assert c.logs_matching('kinematic_sim', 'joined beacon')
    assert c.sim.sim_ns == gc.START_NS - c.sim.dt_ns
    assert c.hub.errors == []


def _p(values, q):
    v = sorted(values)
    return v[min(len(v) - 1, int(q * len(v)))]


@pytest.mark.parametrize('feed, p95_tol, max_tol, yaw_tol_deg', [('ideal', 0.01, 0.01, 0.25),
                                                                  ('real', 0.2, 0.5, 1.5)])
def test_direct_tf_matches_truth(chain, feed, p95_tol, max_tol, yaw_tol_deg):
    """
    Direct mode: the TF is the front fix minus the lever arm, stamped with the fix's `updated`.

    Ideal feed: millimetres. Real-feed model: centimetres of noise, but its stamps are the relay time, about 0.1 s
    after the position was measured (median error 8 cm at 0.83 m/s, the delay localization_node's gnss_delay_s
    takes out). After a gap Octane's backlog delivers positions up to about the gap old under fresh stamps, which
    puts a few TFs some decimetres behind, as on the shuttle. Hence the 95th percentile and a loose maximum.
    """
    c = chain(localization='direct', gnss_feed=feed, steer_deg=STEER)
    c.run_until(25.0)
    assert c.sim.truth()['speed'] > 0.5, 'the driver should have the shuttle moving'
    errs = c.tf_errors(since_ns=SETTLED_NS, use='stamp')        # shuttle_tf stamps the TF with the fix's stamp
    assert len(errs) > 300
    assert _p([e[0] for e in errs], 0.95) < p95_tol
    assert max(e[0] for e in errs) < max_tol
    assert math.degrees(_p([e[1] for e in errs], 0.95)) < yaw_tol_deg
    assert c.hub.errors == []


@pytest.mark.parametrize('feed, pos_tol, yaw_tol_deg', [('ideal', 0.02, 0.5), ('real', 0.15, 1.5)])
def test_ekf_tf_matches_truth(chain, feed, pos_tol, yaw_tol_deg):
    c = chain(localization='ekf', gnss_feed=feed, steer_deg=STEER)
    c.run_until(25.0)
    errs = c.tf_errors(since_ns=SETTLED_NS, use='now')          # the EKF predicts to now and stamps now
    assert len(errs) > 300
    assert max(e[0] for e in errs) < pos_tol
    assert math.degrees(max(e[1] for e in errs)) < yaw_tol_deg
    assert c.loc_status[-1][2] == 'OK'
    assert c.hub.errors == []


def test_gnss_topics_match_truth(chain):
    """/gnss/fix is the front antenna, /gnss/heading_deg the compass heading, /gnss/vel the speed in m/s."""
    from nift_carla.carla_geo import CarlaGeo
    from nift_common.vehicle import ShuttleConfig
    c = chain(localization='direct', steer_deg=STEER)
    c.run_until(20.0)
    geo = CarlaGeo(*_origin())
    off = ShuttleConfig().gnss_front_x
    worst_pos = 0.0
    for _now, stamp, lat, lon in c.fix:
        tr = c.truth_at(stamp)
        if tr is None:
            continue
        x, y = geo.wgs_to_map(lat, lon)
        worst_pos = max(worst_pos, math.hypot(x - (tr[0] + off * math.cos(tr[2])), y - (tr[1] + off * math.sin(tr[2]))))
    assert worst_pos < 0.005
    # heading: compass, true north. Map yaw = true-north yaw - heading (shuttle_tf_node's conversion)
    north = geo.wgs_to_map(_origin()[0] + 0.001, _origin()[1])
    true_north_yaw = math.atan2(north[1], north[0])
    now, deg = c.heading[-1]
    assert abs(gc.wrap(true_north_yaw - math.radians(deg) - c.truth_at(now)[2])) < math.radians(0.25)
    assert abs(c.vel[-1][1] - abs(c.sim.truth()['speed'])) < 1e-6      # knots out, m/s back in, exactly


def _origin():
    from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
    return MAP_ORIGIN_LAT, MAP_ORIGIN_LON


def _stale_after_last_fix(c, since_ns):
    """
    (stamp of the last new position on /gnss/fix before the stale log, sim ns of the stale log).

    beacon_node republishes the last pair at its timer rate until the gate closes, so the last new position is the
    newest stamp, not the last message. With the ideal feed a stamp is the sim time of the tick that measured it.
    """
    stale = [t for t, text in c.logs_matching('beacon_node', 'GNSS stale') if t >= since_ns]
    assert stale, 'beacon_node never reported GNSS stale'
    last_fix = max(stamp for n, stamp, *_ in c.fix if n <= stale[0])
    return last_fix, stale[0]


def _status_changes(c):
    out, prev = [], None
    for t, _lv, msg in c.loc_status:
        word = msg.split()[0]
        if word != prev:
            out.append((t, word))
            prev = word
    return out


@pytest.mark.parametrize('how', ['launch', 'live_sim', 'live_link'])
def test_outage_trips_stale_gate_and_loc_lost(chain, how):
    """
    A 3 s GNSS loss three ways: a scheduled dropout from the launch, the live "dropout" preset, the live link "outage".

    As on ROS (gnss_outage30s_01): /gnss/* stops, beacon_node logs "GNSS stale" 0.55 s after the last fix, the EKF
    goes DEGRADED (ok_age_s 0.5) then LOST (bridge_s 1.5), and everything recovers when the fixes come back.
    """
    t_fault = 10.0          # page time of the command; the fault covers the ticks after it
    if how == 'launch':
        c = chain(localization='ekf', steer_deg=STEER,
                  gnss_faults=[{'kind': 'dropout', 't_s': t_fault + 1.0 + 0.05, 'duration_s': 3.0}])
        c.run_until(t_fault)
    else:
        c = chain(localization='ekf', steer_deg=STEER)
        c.run_until(t_fault)
        c.sim.apply_gnss_fault('dropout' if how == 'live_sim' else 'outage', {'duration_s': 3.0})
    c.run_until(20.0)

    last_fix, stale = _stale_after_last_fix(c, gc.START_NS + 1)
    assert t_rel(last_fix) == pytest.approx(t_fault)                # the last tick before the fault
    assert stale - last_fix == 550_000_000                          # as the reference: 0.55 s
    sent = [n for n, *_ in c.fix]
    gap = [(a, b) for a, b in zip(sent, sent[1:]) if b - a > c.sim.dt_ns]
    assert len(gap) == 1 and t_rel(gap[0][0]) == pytest.approx(t_fault + 0.5)     # the gate closes
    # the first new pair: its payload of t_fault + 3.05 arrives after beacon_node's timer of that tick (ARCH 7,
    # ROS order), so it goes out one tick later
    assert t_rel(gap[0][1]) == pytest.approx(t_fault + 3.1)
    assert not [m for m in c.fix if t_fault + 0.05 <= t_rel(m[1]) < t_fault + 3.05]    # no position from the gap
    assert c.logs_matching('beacon_node', 'GNSS fresh again')

    changes = [(round(t_rel(t), 3), w) for t, w in _status_changes(c) if t > SETTLED_NS]
    words = [w for _t, w in changes]
    assert words[:3] == ['DEGRADED', 'LOST', 'DEGRADED'] and words[-1] == 'OK'
    assert changes[0][0] == pytest.approx(t_rel(last_fix) + 0.55)   # first status tick with age > ok_age_s
    assert changes[1][0] == pytest.approx(t_rel(last_fix) + 1.55)   # age > bridge_s
    if how != 'live_link':
        assert [(round(t_rel(t), 2), d['edge']) for t, d in c.gnss_fault] == [(t_fault + 0.05, 'start'),
                                                                                (t_fault + 3.05, 'end')]
    assert c.hub.errors == []


def test_launch_and_live_dropout_give_the_same_trace(chain):
    """The live preset rebuilds node.gnss; the trace equals the one with the fault scheduled from the launch."""
    a = chain(localization='ekf', steer_deg=STEER, gnss_faults=[{'kind': 'dropout', 't_s': 11.05, 'duration_s': 3.0}])
    a.run_until(18.0)
    trace_a = (a.fix, a.tf, a.loc_status)
    b = chain(localization='ekf', steer_deg=STEER)
    b.run_until(10.0)
    b.sim.apply_gnss_fault('dropout', {'duration_s': 3.0})
    b.run_until(18.0)
    assert trace_a == (b.fix, b.tf, b.loc_status)


def test_repeat_link_fault_is_stale_not_fresh(chain):
    """A stuck relay repeats `updated`: beacon_node's checks see repeats, not new positions, and the gate closes."""
    c = chain(localization='direct', steer_deg=STEER)
    c.run_until(10.0)
    c.sim.apply_gnss_fault('repeat', {'duration_s': 2.0})
    c.run_until(15.0)
    last_fix, stale = _stale_after_last_fix(c, gc.START_NS + 1)
    assert stale - last_fix == 550_000_000
    assert c.beacon.hygiene.counts['duplicate'] >= 2 * 40 * 0.9   # both antennas, about 40 ticks
    assert c.sim.octane.emitted > 0 and c.sim.octane.dropped == 0


def test_single_antenna_outage_also_closes_the_gate(chain):
    c = chain(localization='direct', steer_deg=STEER)
    c.run_until(10.0)
    c.sim.apply_gnss_fault('outage_rear', {'duration_s': 2.0})
    c.run_until(15.0)
    stale = c.logs_matching('beacon_node', 'GNSS stale')
    # ROS order (ARCH 7): beacon_node's timer runs before the tick's payloads arrive, so front is one tick old
    assert stale and 'rear 0.55 s' in stale[0][1] and 'front 0.05 s' in stale[0][1]


def test_catalogue_scenario_outage_30s_straight(chain):
    """The reference's own scenario: route 01, a 30 s dropout of both antennas after 40 m driven."""
    c = chain(localization='direct', gnss_faults='outage_30s_straight', pedal=100)
    c.run_until(60.0)
    start = [t for t, d in c.gnss_fault if d['edge'] == 'start']
    assert start and c.gnss_fault[0][1]['scenario'] == 'outage_30s_straight'
    last_fix, stale = _stale_after_last_fix(c, gc.START_NS + 1)
    assert last_fix == start[0] - c.sim.dt_ns
    assert stale - last_fix == 550_000_000


def test_determinism(chain):
    a = chain(localization='ekf', gnss_feed='real', steer_deg=STEER)
    a.run_until(12.0)
    b = chain(localization='ekf', gnss_feed='real', steer_deg=STEER)
    b.run_until(12.0)
    assert (a.fix, a.tf, a.loc_status) == (b.fix, b.tf, b.loc_status)
