"""
Unit tests of the simulated GNSS (sim_gnss.py).

Both sim backends serve it through the Octane emulator: antenna placement, compass
heading, payload fields, the dropout schedule and seeded noise; the real-feed model
(FeedConfig): rates, relay stamps, gaps, the backlog after a gap, out-of-order
deliveries, dead zones; the backend parameter helpers; the scheduled faults (GnssFault)
of the fault-injection bench.
"""
from dataclasses import fields
from datetime import datetime
import math
import os

from nift_carla.carla_geo import CarlaGeo
from nift_carla.sim_gnss import (
    dead_zones_from_flat,
    EXACT_FEED,
    faults_from_dicts,
    faults_from_param,
    feed_from_params,
    feed_param_defaults,
    FeedConfig,
    GnssFault,
    GnssSim,
    launch_overrides,
    resolve_fault_scenario,
)
from nift_common.vehicle import ShuttleConfig
from nift_sensor.octane_client import KNOTS_TO_MPS
import pytest
import yaml

IDS = ('SIM-FRONT', 'SIM-REAR')
OFFSET = ShuttleConfig().gnss_front_x


@pytest.fixture(scope='module')
def geo():
    return CarlaGeo()


def fix(p):
    d = p['state']['dynamics']
    return d['latitude'], d['longitude']


def heading(payloads):
    return payloads[0]['state']['dynamics']['heading']


def step_map(gnss, geo, yaw, t=1.0, x=20.0, y=30.0, speed=1.5, elev=277.0):
    return gnss.step(
        round(t * 1e9), x, y, math.cos(yaw), math.sin(yaw), speed, elev, geo.map_to_wgs
    )


def test_antennas_on_the_baseline(geo):
    yaw = 0.3
    payloads = step_map(GnssSim(OFFSET, IDS), geo, yaw)
    assert [p['id'] for p in payloads] == list(IDS)
    for p, sign in zip(payloads, (1.0, -1.0)):
        x, y = geo.wgs_to_map(*fix(p))
        assert x == pytest.approx(20.0 + sign * OFFSET * math.cos(yaw), abs=1e-6)
        assert y == pytest.approx(30.0 + sign * OFFSET * math.sin(yaw), abs=1e-6)


def test_heading_follows_yaw(geo):
    north = heading(step_map(GnssSim(OFFSET, IDS), geo, math.pi / 2))
    east = heading(step_map(GnssSim(OFFSET, IDS), geo, 0.0))
    # map +y is UTM grid north, 1.82 deg west of true north at Mcity (carla_geo.py)
    assert (north + 180.0) % 360.0 - 180.0 == pytest.approx(-1.82, abs=0.05)
    # the heading formula is spherical: off by up to ~0.1 deg on the ellipsoid (beacon_node
    # computes its own)
    assert (east - north) % 360.0 == pytest.approx(90.0, abs=0.02)


def test_payload_fields(geo):
    p = step_map(GnssSim(OFFSET, IDS), geo, 0.0, t=123.45, speed=2.0, elev=277.5)[1]
    d = p['state']['dynamics']
    assert d['velocity'] == pytest.approx(2.0 / KNOTS_TO_MPS)
    assert d['elevation'] == 277.5
    assert d['updated'] == p['serviced'] == '1970-01-01T00:02:03.450+00:00'


@pytest.mark.parametrize(
    't, ok', [(0.0, False), (0.95, False), (1.0, True), (9.95, True), (10.2, False), (11.0, True)]
)
def test_dropout_schedule(geo, t, ok):
    gnss = GnssSim(OFFSET, IDS, dropout_period_s=10.0, dropout_duration_s=1.0)
    payloads = step_map(gnss, geo, 0.0, t=t)
    assert gnss.ok is ok
    assert bool(payloads) is ok


def test_no_dropout_unless_both_set(geo):
    gnss = GnssSim(OFFSET, IDS, dropout_period_s=10.0)
    assert step_map(gnss, geo, 0.0, t=0.5) and gnss.ok


def test_seeded_noise_is_reproducible(geo):
    def run(seed):
        gnss = GnssSim(OFFSET, IDS, noise_stddev_m=0.8, noise_heading=True, seed=seed)
        return [fix(p) for t in range(10) for p in step_map(gnss, geo, 0.3, t=1.0 + 0.05 * t)]
    assert run(7) == run(7)
    assert run(7) != run(8)


def test_shared_noise_keeps_the_heading(geo):
    exact = heading(step_map(GnssSim(OFFSET, IDS), geo, 0.3))
    shared = step_map(GnssSim(OFFSET, IDS, noise_stddev_m=0.8, seed=3), geo, 0.3)
    split = step_map(
        GnssSim(OFFSET, IDS, noise_stddev_m=0.8, noise_heading=True, seed=3), geo, 0.3
    )
    assert heading(shared) == pytest.approx(exact, abs=1e-4)
    # but the position moved
    assert fix(shared[0]) != fix(step_map(GnssSim(OFFSET, IDS), geo, 0.3)[0])
    assert abs(heading(split) - exact) > 1e-3


# ---------------------------------------------------------------------------------------------------------------
# FeedConfig: the real Octane feed
# (10 Hz receiver behind a jittered relay, gaps, out-of-order, dead zones)
# ---------------------------------------------------------------------------------------------------------------
QUIET = FeedConfig(
    white_m=0.0,
    antenna_m=0.0,
    wander_m=0.0,
    speed_noise_mps=0.0,
    uplink_mean_s=0.0,
    gap_rate_moving=0.0,
    gap_rate_still=0.0,
    burst_per_min=0.0,
    out_of_order_per_h=0.0,
)


def drive(gnss, geo, seconds, v=1.0, x0=20.0, dt=0.05, t0=1.0):
    """
    Drive straight along map +x at v and return [(sim_t, payload)].

    Sim time is accumulated as a float like a caller would.
    """
    out, t = [], t0
    for k in range(int(seconds / dt)):
        x = x0 + v * (t - t0)
        out += [
            (t, p) for p in gnss.step(round(t * 1e9), x, 30.0, 1.0, 0.0, v, 277.0, geo.map_to_wgs)
        ]
        t += dt
    return out


def stamp(p):
    return datetime.fromisoformat(p['state']['dynamics']['updated']).timestamp()


def test_feed_is_10_hz_behind_a_15_hz_relay(geo):
    # late sim time: a float epoch clock drifted here (13.7 Hz), the integer-ns grid does not
    out = drive(GnssSim(OFFSET, IDS, feed=QUIET, seed=1), geo, 120.0, t0=20000.0)
    front = [p for _, p in out if p['id'] == IDS[0]]
    # ~16 deliveries / s
    assert len(front) / 120.0 == pytest.approx(1.0 / QUIET.interval_s, rel=0.05)
    assert len({fix(p) for p in front}) / 120.0 == pytest.approx(10.0, abs=0.05)
    repeats = sum(fix(a) == fix(b) for a, b in zip(front, front[1:])) / len(front)
    # measured: ~38 %
    assert 0.3 < repeats < 0.45


@pytest.mark.parametrize('latency', [0.0, 0.3])
def test_feed_stamp_is_the_relay_time(geo, latency):
    """
    Check that `updated` is when Octane relayed the position.

    That is 0-100 ms (one receiver period) after it was measured.
    """
    feed = FeedConfig(**{**QUIET.__dict__, 'latency_s': latency})
    for _, p in drive(GnssSim(OFFSET, IDS, feed=feed, seed=2), geo, 30.0):
        if p['id'] != IDS[0]:
            continue
        x, _ = geo.wgs_to_map(*fix(p))
        t_measured = 1.0 + (x - OFFSET - 20.0) / 1.0
        assert latency - 1e-3 <= stamp(p) - t_measured <= latency + 0.1 + 1e-3


def test_feed_gaps_leave_ok_alone(geo):
    gnss = GnssSim(
        OFFSET, IDS, feed=FeedConfig(**{**QUIET.__dict__, 'gap_rate_moving': 1.0}), seed=3
    )
    out = drive(gnss, geo, 60.0)
    assert gnss.ok
    st = [stamp(p) for _, p in out if p['id'] == IDS[0]]
    assert max(b - a for a, b in zip(st, st[1:])) > 0.3


def test_feed_out_of_order(geo):
    # 20 %
    feed = FeedConfig(**{**QUIET.__dict__, 'out_of_order_per_h': 0.2 * 3600.0 / QUIET.interval_s})
    st = [
        stamp(p)
        for _, p in drive(GnssSim(OFFSET, IDS, feed=feed, seed=4), geo, 30.0)
        if p['id'] == IDS[0]
    ]
    back = [a - b for a, b in zip(st, st[1:]) if b < a]
    assert back and max(back) <= 0.08 + 0.15


def test_feed_seeded_is_reproducible(geo):
    def run(seed):
        return [(t, p['id'], p['state']['dynamics']['updated'], fix(p))
                for t, p in drive(GnssSim(OFFSET, IDS, feed=FeedConfig(), seed=seed), geo, 20.0)]
    assert run(5) == run(5)
    assert run(5) != run(6)


# Backlog: gaps of ~0.8 s that all hold their positions (no other randomness), backlog cap out of
# the way
BACKLOG = FeedConfig(
    **{
        **QUIET.__dict__,
        'gap_rate_moving': 0.3,
        'gap_median_moving_s': 0.8,
        'gap_sigma': 0.3,
        'hold_fraction': 1.0,
        'backlog_max_s': 10.0,
    }
)


def front_track(out, geo):
    """
    Drive a QUIET-style feed at 1 m/s.

    Returns (stamp, measurement time) of every front delivery.
    """
    track = []
    for _, p in out:
        if p['id'] == IDS[0]:
            x, _ = geo.wgs_to_map(*fix(p))
            track.append((stamp(p), 1.0 + (x - OFFSET - 20.0)))
    return track


def test_feed_backlog_after_a_gap(geo):
    """
    Check a gap that holds: the next epoch comes first, about the gap old.

    Then one epoch every backlog_interval_s until it has caught up, then the normal relay
    again (repeats, ~62 ms). Nothing is lost, nothing goes backwards.
    """
    track = front_track(drive(GnssSim(OFFSET, IDS, feed=BACKLOG, seed=11), geo, 120.0), geo)
    st, tm = [s for s, _ in track], [m for _, m in track]
    slots = {round((m - tm[0]) * 10.0) for m in tm}
    # 10 Hz epochs lost only to relay intervals > 0.1 s
    assert max(slots) + 1 - len(slots) <= 0.01 * len(slots)
    assert all(b >= a - 1e-3 for a, b in zip(tm, tm[1:]))                       # in order
    gaps, drain, normal, rep = 0, [], [], []
    for i in range(len(st) - 1):
        if st[i + 1] - st[i] <= 0.5:
            continue
        gaps += 1
        assert tm[i + 1] - tm[i] == pytest.approx(0.1, abs=1e-3)               # the next epoch ...
        # ... about the gap old
        assert st[i + 1] - tm[i + 1] >= st[i + 1] - st[i] - 0.1 - 1e-3
        j = i + 1
        # catching up
        while j + 1 < len(st) and st[j] - tm[j] > 0.1 + 1e-3 and st[j + 1] - st[j] < 0.3:
            assert tm[j + 1] - tm[j] == pytest.approx(0.1, abs=1e-3)
            drain.append(st[j + 1] - st[j])
            j += 1
        k = j
        # caught up: the normal relay
        while k + 1 < len(st) and st[k + 1] - st[k] < 0.3 and st[k] < st[j] + 3.0:
            normal.append(st[k + 1] - st[k])
            rep.append(tm[k + 1] == tm[k])
            k += 1
    assert gaps >= 10 and len(drain) >= 50 and len(normal) >= 200
    assert sum(drain) / len(drain) == pytest.approx(BACKLOG.backlog_interval_s, abs=0.005)
    assert sum(normal) / len(normal) == pytest.approx(BACKLOG.interval_s, abs=0.006)
    # repeats again (~38 %)
    assert 0.25 < sum(rep) / len(rep) < 0.5


def test_feed_without_backlog_the_newest_follows_a_gap(geo):
    """
    Check hold_fraction 0 (the model before the backlog).

    After a gap the newest position comes, and what the gap swallowed is lost.
    """
    feed = FeedConfig(**{**BACKLOG.__dict__, 'hold_fraction': 0.0})
    track = front_track(drive(GnssSim(OFFSET, IDS, feed=feed, seed=11), geo, 60.0), geo)
    after = [(b, a) for a, b in zip(track, track[1:]) if b[0] - a[0] > 0.5]
    assert len(after) >= 5
    for (s1, m1), (s0, m0) in after:
        assert s1 - m1 <= 0.1 + 1e-3 and m1 - m0 > 0.3


def test_feed_backlog_drops_what_is_older_than_backlog_max(geo):
    feed = FeedConfig(**{**BACKLOG.__dict__, 'backlog_max_s': 0.3})
    track = front_track(drive(GnssSim(OFFSET, IDS, feed=feed, seed=12), geo, 60.0), geo)
    after = [b for a, b in zip(track, track[1:]) if b[0] - a[0] > 0.6]
    assert len(after) >= 5 and all(0.2 - 1e-3 <= s - m <= 0.3 + 1e-3 for s, m in after)


@pytest.mark.parametrize('how', ['fault', 'schedule'])
def test_feed_holds_nothing_through_a_fault_dropout_or_the_schedule(geo, how):
    feed = FeedConfig(**{**QUIET.__dict__, 'hold_fraction': 1.0})
    if how == 'fault':
        g = GnssSim(
            OFFSET, IDS, feed=feed, faults=[GnssFault('dropout', t_s=3.0, duration_s=1.0)], seed=13
        )
    else:
        g = GnssSim(OFFSET, IDS, feed=feed, dropout_period_s=5.0, dropout_duration_s=1.0, seed=13)
    track = front_track(drive(g, geo, 12.0), geo)
    after = [b for a, b in zip(track, track[1:]) if b[0] - a[0] > 0.5]
    assert after and all(s - m <= 0.1 + 1e-3 for s, m in after)


def test_backlog_off_switch(geo):
    """
    Check that hold_fraction 0 makes the other backlog fields inert.

    EXACT_FEED has it off; the default holds.
    """
    assert EXACT_FEED.hold_fraction == 0.0 and FeedConfig().hold_fraction > 0.0

    def run(**kw):
        feed = FeedConfig(**{**FeedConfig().__dict__, 'gap_rate_moving': 1.0, **kw})
        return [(t, p['id'], p['state']['dynamics']['updated'], fix(p))
                for t, p in drive(GnssSim(OFFSET, IDS, feed=feed, seed=14), geo, 30.0)]
    off = run(hold_fraction=0.0)
    assert off == run(
        hold_fraction=0.0, backlog_interval_s=0.01, backlog_jitter_s=0.0, backlog_max_s=0.1
    )
    assert off != run()


@pytest.mark.parametrize('feed', [None, QUIET])
def test_dead_zone(geo, feed):
    lat, lon = geo.map_to_wgs(20.0, 30.0)
    # ~ +-10 m around base_link
    zone = (lat - 1e-4, lat + 1e-4, lon - 1e-4, lon + 1e-4)
    inside = GnssSim(OFFSET, IDS, feed=feed, dead_zones=[zone], seed=7)
    assert drive(inside, geo, 2.0, v=0.0) == [] and inside.ok is False
    outside = GnssSim(
        OFFSET,
        IDS,
        feed=feed,
        dead_zones=[(lat + 1e-3, lat + 2e-3, lon - 1e-4, lon + 1e-4)],
        seed=7,
    )
    assert drive(outside, geo, 2.0, v=0.0) and outside.ok is True


# ---------------------------------------------------------------------------------------------------------------
# Backend parameters: gps.feed.* and gps.dead_zones
# ---------------------------------------------------------------------------------------------------------------
def test_feed_param_defaults_mirror_feedconfig():
    d = feed_param_defaults()
    assert d.pop('gps.feed.enabled') is False
    assert d == {f'gps.feed.{f.name}': f.default for f in fields(FeedConfig)}


def test_feed_from_params():
    params = feed_param_defaults()
    # off by default: ideal feed
    assert feed_from_params(params.get) is None
    params.update(
        {'gps.feed.enabled': True, 'gps.feed.latency_s': 0.25, 'gps.feed.burst_per_min': 0}
    )
    feed = feed_from_params(params.get)
    assert feed == FeedConfig(latency_s=0.25, burst_per_min=0.0)
    # ints from YAML become floats
    assert isinstance(feed.burst_per_min, float)


def test_dead_zones_from_flat():
    assert dead_zones_from_flat([]) == []
    assert dead_zones_from_flat([42, 43, -84, -83.5]) == [(42.0, 43.0, -84.0, -83.5)]
    with pytest.raises(ValueError, match='4 numbers'):
        dead_zones_from_flat([42.0, 43.0, -84.0])
    with pytest.raises(ValueError, match='lat_min < lat_max'):
        dead_zones_from_flat([43.0, 42.0, -84.0, -83.0])


def test_summary_names_the_model():
    assert GnssSim(OFFSET, IDS).summary().startswith('ideal feed')
    s = GnssSim(OFFSET, IDS, feed=FeedConfig(latency_s=0.2), dead_zones=[(0, 1, 0, 1)]).summary()
    assert s.startswith('real Octane feed model') and 'latency 0.2 s' in s and '1 dead zone' in s
    assert f'antennas +-{OFFSET:g} m' in s and 'backlog after 90 % of the gaps' in s
    assert 'backlog' not in GnssSim(OFFSET, IDS, feed=FeedConfig(hold_fraction=0.0)).summary()


def test_launch_overrides():
    sets = {'north': [[42.302, 42.303, -83.70, -83.69]], 'turn': [[1, 2, 3, 4], [5, 6, 7, 8]]}
    # '' everywhere: the yaml's values
    assert launch_overrides() == {}
    assert launch_overrides('real', 'north,turn', '7', sets) == {
        'gps.feed.enabled': True, 'gps.seed': 7,
        'gps.dead_zones': [42.302, 42.303, -83.70, -83.69, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]}
    assert launch_overrides('ideal') == {'gps.feed.enabled': False}
    with pytest.raises(ValueError, match='real or ideal'):
        launch_overrides('octane')
    with pytest.raises(
        ValueError, match=r"unknown set\(s\) \['south'\]; known: \['north', 'turn'\]"
    ):
        launch_overrides(dead_zones='south', sets=sets)
    with pytest.raises(ValueError, match='integer'):
        launch_overrides(seed='x')


def test_shipped_dead_zone_sets_are_valid():
    path = os.path.join(os.path.dirname(__file__), '..', 'config', 'gnss_dead_zones.yaml')
    with open(path) as f:
        sets = yaml.safe_load(f)['sets']
    assert {'north_end', 'south_end'} <= set(sets)
    for name in sets:
        assert launch_overrides(dead_zones=name, sets=sets)['gps.dead_zones']


# ---------------------------------------------------------------------------------------------------------------
# Faults (the fault-injection bench): exact feed underneath unless a FeedConfig is given
# ---------------------------------------------------------------------------------------------------------------
def by_id(out, bid):
    return [(t, p) for t, p in out if p['id'] == bid]


def run_fault(geo, *faults, seconds=6.0, v=1.0, feed=None):
    g = GnssSim(OFFSET, IDS, feed=feed, faults=list(faults), seed=9)
    return g, drive(g, geo, seconds, v=v)


def test_fault_validation():
    with pytest.raises(ValueError, match='kind'):
        GnssFault('teleport', t_s=1.0, duration_s=1.0)
    with pytest.raises(ValueError, match='exactly one trigger'):
        GnssFault('dropout', duration_s=1.0)
    with pytest.raises(ValueError, match='exactly one trigger'):
        GnssFault('dropout', t_s=1.0, at_m=3.0, duration_s=1.0)
    with pytest.raises(ValueError, match='duration_s > 0'):
        GnssFault('dropout', t_s=1.0)
    with pytest.raises(ValueError, match='both antennas'):
        GnssFault('swap', antenna='rear', t_s=1.0, duration_s=1.0)
    with pytest.raises(ValueError, match='sigma_m'):
        GnssFault('noise', t_s=1.0, duration_s=1.0)
    with pytest.raises(ValueError, match=r'fault 1: .*unexpected keyword'):
        faults_from_dicts(
            [{'kind': 'dropout', 't_s': 1.0, 'duration_s': 1.0}, {'kind': 'jump', 'meters': 3}]
        )
    assert faults_from_dicts([{'kind': 'jump', 'at_m': 2.0, 'north_m': 5.0}])[0].north_m == 5.0


def test_faults_run_on_the_exact_feed(geo):
    """
    Check no FeedConfig + faults: every tick delivers both antennas.

    Stamped with the tick, positions exact.
    """
    g, out = run_fault(
        geo, GnssFault('clock', t_s=100.0, duration_s=1.0, offset_s=0.5), seconds=2.0
    )
    assert g.exact and len(by_id(out, IDS[0])) == len(by_id(out, IDS[1])) == 40
    for t, p in by_id(out, IDS[0]):
        assert stamp(p) == pytest.approx(t, abs=1e-3)
        x, _ = geo.wgs_to_map(*fix(p))
        assert x == pytest.approx(20.0 + (t - 1.0) + OFFSET, abs=1e-6)
    assert 'scheduled fault' in g.summary() and g.summary().startswith('ideal feed')


def test_dropout_one_antenna_by_time(geo):
    g, out = run_fault(geo, GnssFault('dropout', antenna='rear', t_s=2.0, duration_s=1.0))
    rear_t = [t for t, _ in by_id(out, IDS[1])]
    assert not any(2.0 - 1e-6 <= t < 3.0 - 1e-6 for t in rear_t) and any(t >= 3.0 for t in rear_t)
    assert any(2.0 <= t < 3.0 for t, _ in by_id(out, IDS[0]))            # the front keeps going
    assert [(e[2], e[3]) for e in g.fault_events] == [('start', 'dropout'), ('end', 'dropout')]


def test_dropout_while_inside_a_box(geo):
    lat0, lon0 = geo.map_to_wgs(22.0, 30.0)
    # base_link inside for x 22..24 (2 s)
    lat1, lon1 = geo.map_to_wgs(24.0, 30.0)
    box = (min(lat0, lat1) - 1e-5, max(lat0, lat1) + 1e-5, lon0, lon1)
    g, out = run_fault(geo, GnssFault('dropout', box=box))
    t_front = [t for t, _ in by_id(out, IDS[0])]
    assert not any(3.05 <= t <= 4.95 for t in t_front) and any(t > 5.1 for t in t_front)


def test_jump_single_epoch_by_distance(geo):
    g, out = run_fault(geo, GnssFault('jump', antenna='front', at_m=2.0, north_m=5.0))
    d = []
    for t, p in by_id(out, IDS[0]):
        x, y = geo.wgs_to_map(*fix(p))
        d.append(math.hypot(x - (20.0 + (t - 1.0) + OFFSET), y - 30.0))
    jumped = [e for e in d if e > 1e-3]
    assert len(jumped) == 1 and jumped[0] == pytest.approx(5.0, abs=0.01)
    assert stamp(next(p for (t, p), e in zip(by_id(out, IDS[0]), d) if e > 1e-3)) == pytest.approx(
        3.0, abs=0.06
    )


def test_noise_window(geo):
    g, out = run_fault(geo, GnssFault('noise', t_s=2.0, duration_s=3.0, sigma_m=0.3), seconds=6.0)
    err_in, err_out = [], []
    for t, p in by_id(out, IDS[1]):
        x, y = geo.wgs_to_map(*fix(p))
        e = math.hypot(x - (20.0 + (t - 1.0) - OFFSET), y - 30.0)
        (err_in if 2.0 <= t < 5.0 else err_out).append(e)
    assert (
        max(err_out) < 1e-3 and 0.2 < (sum(e * e for e in err_in) / len(err_in) / 2) ** 0.5 < 0.4
    )


def test_stuck_feed_repeats_with_fresh_stamps(geo):
    g, out = run_fault(geo, GnssFault('stuck', t_s=2.0, duration_s=1.0))
    win = [p for t, p in by_id(out, IDS[0]) if 2.0 <= t < 3.0 - 1e-6]
    assert len({fix(p) for p in win}) == 1 and len({stamp(p) for p in win}) == len(win) == 20
    assert len({p['state']['dynamics']['velocity'] for p in win}) == 1
    after = [fix(p) for t, p in by_id(out, IDS[0]) if t >= 3.0]
    assert after[0] != fix(win[0]) and len(set(after)) == len(after)


def test_freeze_rear_and_swap(geo):
    g, out = run_fault(geo, GnssFault('freeze', antenna='rear', t_s=2.0, duration_s=1.0))
    rear = [fix(p) for t, p in by_id(out, IDS[1]) if 2.0 <= t < 3.0 - 1e-6]
    front = [fix(p) for t, p in by_id(out, IDS[0]) if 2.0 <= t < 3.0 - 1e-6]
    assert len(set(rear)) == 1 and len(set(front)) == len(front)
    g, out = run_fault(geo, GnssFault('swap', t_s=2.0, duration_s=1.0), seconds=3.0)
    t, p = by_id(out, IDS[0])[25]                                         # t = 2.25
    x, _ = geo.wgs_to_map(*fix(p))
    # the front payload carries the rear
    assert x == pytest.approx(20.0 + (t - 1.0) - OFFSET, abs=1e-6)
    assert (
        p['state']['dynamics']['heading'] - heading(step_map(GnssSim(OFFSET, IDS), geo, 0.0))
    ) % 360.0 == pytest.approx(180.0, abs=0.2)


def test_delay_and_clock(geo):
    g, out = run_fault(geo, GnssFault('delay', t_s=2.0, duration_s=2.0, delay_s=0.3))
    for t, p in by_id(out, IDS[0]):
        x, _ = geo.wgs_to_map(*fix(p))
        # 1 m/s: metres behind = seconds old
        lag = (20.0 + (t - 1.0) + OFFSET) - x
        # older at once (history)
        assert lag == pytest.approx(0.3 if 2.0 <= t < 4.0 - 1e-6 else 0.0, abs=1e-3)
    g, out = run_fault(
        geo, GnssFault('clock', antenna='front', t_s=2.0, duration_s=1.0, offset_s=-0.5)
    )
    for t, p in by_id(out, IDS[0]):
        assert stamp(p) == pytest.approx(t - (0.5 if 2.0 <= t < 3.0 - 1e-6 else 0.0), abs=1e-3)


def test_burst_holds_then_releases_at_once(geo):
    g, out = run_fault(geo, GnssFault('burst', antenna='both', t_s=2.0, duration_s=1.0))
    rx = by_id(out, IDS[0])
    assert not any(2.0 <= t < 3.0 - 1e-6 for t, _ in rx)
    batch = [p for t, p in rx if abs(t - 3.0) < 1e-6]
    # the 20 held + the tick at 3.0
    assert len(batch) == 21 and stamp(batch[1]) == pytest.approx(2.0, abs=1e-3)


def test_faults_on_the_real_feed_and_seeded(geo):
    def run(seed):
        g = GnssSim(
            OFFSET,
            IDS,
            feed=QUIET,
            faults=[GnssFault('dropout', t_s=2.0, duration_s=1.0)],
            seed=seed,
        )
        return [(t, p['id'], stamp(p)) for t, p in drive(g, geo, 4.0)]
    a = run(4)
    assert a == run(4)
    # by relay stamp
    assert not any(2.0 <= st < 3.0 for _, _, st in a) and any(st > 3.1 for _, _, st in a)


# ---------------------------------------------------------------------------------------------------------------
# Fault scenarios: catalogue, launch overrides, the gps.faults parameter,
# events for /carla/gnss_fault
# ---------------------------------------------------------------------------------------------------------------
CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')


def shipped(name):
    with open(os.path.join(CONFIG, name)) as f:
        return yaml.safe_load(f)


def test_every_shipped_fault_scenario_resolves():
    cat, sets = shipped('gnss_faults.yaml'), shipped('gnss_dead_zones.yaml')['sets']
    # keyed by route id ('01', ...)
    routes = set(shipped('test_routes.yaml')['routes'])
    assert len(cat['scenarios']) >= 25
    for name, sc in cat['scenarios'].items():
        items, backend, got = resolve_fault_scenario(name, cat, sets)
        assert got == name and sc['route'] in routes and 'stop' in sc['target'], name
        faults = faults_from_param(
            launch_overrides(faults=name, catalogue=cat, sets=sets).get('gps.faults', '')
        )
        assert len(faults) == len(sc['faults']), name


def test_resolve_box_names_backend_params_and_files(tmp_path):
    sets = {'turn': [[42.30, 42.31, -83.70, -83.69]]}
    cat = {'scenarios': {'a': {'faults': [{'kind': 'dropout', 'box': 'turn', 'duration_s': 2.0}]},
                         'lever': {'faults': [], 'backend_params': {'antenna_offset_x': 1.44}},
                         'bad': {'faults': [], 'backend_params': {'wheelbase': 3.0}}}}
    items, backend, _ = resolve_fault_scenario('a', cat, sets)
    assert items[0]['box'] == [42.30, 42.31, -83.70, -83.69] and backend == {}
    lever = {'gps.faults': '', 'gps.faults_name': 'lever', 'antenna_offset_x': 1.44}
    assert launch_overrides(faults='lever', catalogue=cat) == lever
    with pytest.raises(ValueError, match='not allowed'):
        resolve_fault_scenario('bad', cat)
    with pytest.raises(ValueError, match=r"known: \['a', 'bad', 'lever'\]"):
        resolve_fault_scenario('nope', cat)
    with pytest.raises(ValueError, match='not a dead-zone set'):
        resolve_fault_scenario('a', cat, {})
    f = tmp_path / 'mine.yaml'
    f.write_text('faults:\n  - {kind: stuck, t_s: 5.0, duration_s: 2.0}\n')
    out = launch_overrides(faults=str(f))
    assert faults_from_param(out['gps.faults']) == [GnssFault('stuck', t_s=5.0, duration_s=2.0)]
    assert faults_from_param('') == [] and faults_from_param('  ') == []
    with pytest.raises(ValueError, match='YAML list'):
        faults_from_param('kind: stuck')


def test_new_fault_events(geo):
    g = GnssSim(
        OFFSET, IDS, faults=[GnssFault('dropout', antenna='rear', t_s=2.0, duration_s=1.0)]
    )
    drive(g, geo, 1.5)                                                    # ticks 1.0 .. 2.45
    assert g.new_fault_events() == [
        {'t': 2.0, 'index': 0, 'edge': 'start', 'kind': 'dropout', 'antenna': 'rear'}
    ]
    assert g.new_fault_events() == []
    drive(g, geo, 1.0, t0=2.5)                                            # past the end at 3.0
    assert [e['edge'] for e in g.new_fault_events()] == ['end']
