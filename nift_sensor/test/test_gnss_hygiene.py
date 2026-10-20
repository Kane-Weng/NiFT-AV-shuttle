"""
gnss_hygiene.BeaconHygiene.

The beacon feed checks, on a synthetic feed shaped like the real one (10 Hz receiver epochs relayed
at ~15 Hz per antenna with fresh millisecond stamps, positions rounded to 7 decimals).
"""
from datetime import datetime, timedelta, timezone
import math

from nift_common.vehicle import ShuttleConfig
from nift_sensor.gnss_hygiene import BeaconHygiene, enu_offset, HygieneConfig, stamp_s
import pytest

SHUTTLE = ShuttleConfig()
LAT0, LON0 = 42.3, -83.7
HALF = SHUTTLE.gnss_front_x     # antennas at +-HALF along the heading from base_link
BASE_X = SHUTTLE.base_link_x    # base_link ahead of the rear axle
R_TURN = SHUTTLE.min_turn_radius    # full lock
T0 = datetime(2026, 10, 18, 12, 0, 0, tzinfo=timezone.utc)


def to_latlon(east, north):
    """Inverse of enu_offset() around (LAT0, LON0), iterated to the 7 decimals the feed carries."""
    lat, lon = LAT0, LON0
    for _ in range(3):
        e, n = enu_offset(LAT0, LON0, lat, lon)
        lat += (north - n) / 111_000.0
        lon += (east - e) / (111_000.0 * math.cos(math.radians(LAT0)))
    return lat, lon


def iso(t):
    return (T0 + timedelta(milliseconds=round(t * 1000))).isoformat(timespec='milliseconds')


def track(segments, dt=0.1, rest=1.0):
    """
    Receiver epochs every dt.

    [(t, east, north, heading [rad, compass], speed)] of base_link along (duration, speed,
    curvature) segments, starting north after `rest` s at rest (where the antenna baseline is
    learned). Bicycle model as the kinematic sim: the rear axle moves along the heading, base_link
    is BASE_X ahead of it (so in a turn it slips sideways).
    """
    t, e, n, h, out = 0.0, 0.0, 0.0, 0.0, []
    for duration, v, kappa in ([(rest, 0.0, 0.0)] if rest else []) + list(segments):
        for _ in range(round(duration / dt)):
            out.append((t, e + BASE_X * math.sin(h), n + BASE_X * math.cos(h), h, abs(v)))
            h += v * kappa * dt * 0.5
            e += v * dt * math.sin(h)
            n += v * dt * math.cos(h)
            h += v * kappa * dt * 0.5
            t += dt
    return out


def antenna(ep, k, swap=False):
    t, e, n, h, v = ep
    sign = (1.0 if k == 0 else -1.0) * (-1.0 if swap else 1.0)
    lat, lon = to_latlon(e + sign * HALF * math.sin(h), n + sign * HALF * math.cos(h))
    return round(lat, 7), round(lon, 7)


def feed(epochs, relay=0.066, phase=(0.0, 0.031), edit=None):
    """
    Deliveries [(rx, normalize()-style dict)] of both antennas.

    Every `relay` s each antenna's latest epoch, stamped with the relay time, arriving 25 ms later.
    edit(k, t, ep, d) may change or drop (return None) a delivery.
    """
    out = []
    t_end = epochs[-1][0]
    for k, bid in enumerate(('F', 'R')):
        t = phase[k] + 0.0005
        while t < t_end:
            ep = max((x for x in epochs if x[0] <= t), key=lambda x: x[0])
            lat, lon = antenna(ep, k)
            d = {'id': bid, 'ts': iso(t), 'lat': lat, 'lon': lon, 'speed': ep[4]}
            d = edit(k, t, ep, d) if edit else d
            if d is not None:
                out.append((t + 0.025, d))
            t += relay
    return sorted(out, key=lambda x: x[0])


def run(deliveries, cfg=None):
    hyg = BeaconHygiene('F', 'R', cfg)
    return hyg, [(rx, hyg.add(d, rx)) for rx, d in deliveries]


REJECT = ('baseline', 'heading', 'step')


def rejected(verdicts, t0=-1.0, t1=1e9):
    return [(rx, v) for rx, v in verdicts if v in REJECT and t0 <= rx < t1]


def test_stamp_and_geometry_helpers():
    assert stamp_s('2026-10-08T21:28:06.080+00:00') == pytest.approx(
        datetime(2026, 10, 8, 21, 28, 6, 80000, tzinfo=timezone.utc).timestamp())
    assert stamp_s('') is None and stamp_s('garbage') is None
    e, n = enu_offset(LAT0, LON0, LAT0 + 1e-5, LON0 + 1e-5)
    assert n == pytest.approx(1.111, abs=0.002) and e == pytest.approx(0.823, abs=0.002)


@pytest.mark.parametrize('v', [0.0, 1.0, 2.0, 3.5])
def test_clean_feed_straight_and_full_lock_turns_is_never_rejected(v):
    epochs = track([(5, v, 0.0), (6, v, 1 / R_TURN), (3, v, 0.0), (6, v, -1 / R_TURN), (3, v,
                                                                                        0.0)])
    hyg, out = run(feed(epochs))
    assert rejected(out) == []
    assert hyg.baseline_m == pytest.approx(2 * HALF, abs=0.02)
    assert hyg.counts['accepted'] > 200
    if v >= 0.3:
        assert hyg.counts['repeat'] > 50                 # the ~1/3 relays of an epoch already sent
    else:
        # at rest a repeat is a new (current) position
        assert hyg.counts['repeat'] == 0
    assert hyg.counts['stuck'] == 0


def test_reversing_is_not_a_step():
    _, out = run(feed(track([(4, 0.0, 0.0), (8, -1.5, 1 / R_TURN), (4, -1.5, 0.0)])))
    assert rejected(out) == []


def test_forward_stop_reverse_is_never_an_older_position():
    hyg, out = run(feed(track([(6, 2.0, 0.0), (0.3, 0.0, 0.0), (6, -1.0, 0.0)])))
    assert rejected(out) == [] and hyg.counts['older_position'] == 0


def test_older_epoch_under_a_fresh_stamp_is_dropped():
    epochs = track([(8, 1.0, 0.0)])

    # front, at ~5 s: the position of 0.3 s ago, stamp fresh
    def late(k, t, ep, d):
        if k == 0 and 5.0 <= t < 5.07:
            d['lat'], d['lon'] = antenna(next(x for x in epochs if x[0] >= ep[0] - 0.31), 0)
        return d
    hyg, out = run(feed(epochs, edit=late))
    assert [v for rx, v in out if 5.0 < rx < 5.1 and v == 'older_position']
    assert hyg.counts['older_position'] == 1 and rejected(out) == []


def test_baseline_is_learned_at_rest_only():
    hyg, out = run(feed(track([(4, 2.0, 0.0), (3, 0.0, 0.0)], rest=0)))   # starts while driving
    assert hyg.accepted_rx > 4.0 and not [v for rx, v in out if rx < 4.0 and v in ('accepted',
                                                                                   'reanchored')]
    assert hyg.baseline_m == pytest.approx(2 * HALF, abs=0.02)


def test_learning_then_fixed_baseline():
    hyg, out = run(feed(track([(3, 0.0, 0.0)])))
    assert [v for _, v in out if v not in ('no_pair',)][:20] == ['learning'] * 19 + ['accepted']
    hyg, out = run(feed(track([(3, 0.0, 0.0)])), HygieneConfig(antenna_baseline_m=2 * HALF))
    assert 'learning' not in [v for _, v in out] and hyg.counts['accepted'] > 0
    hyg, out = run(feed(track([(3, 0.0, 0.0)])), HygieneConfig(antenna_baseline_m=3.0))
    assert hyg.accepted is None and hyg.counts['baseline'] > 0


def test_stuck_feed_while_moving_is_not_new():
    # from 5 s on, the feed repeats the epoch at 5 s
    def stuck(k, t, ep, d):
        if t >= 5.0:
            d['lat'], d['lon'] = antenna(frozen[0], k)
            d['speed'] = frozen[0][4]
        return d
    epochs = track([(10, 2.0, 0.0)])
    frozen = [next(x for x in epochs if x[0] >= 4.95)]
    ds = feed(epochs, edit=stuck)
    hyg, _ = run([x for x in ds if x[0] < 5.28])
    # the usual repeats first: still fresh, as before
    assert hyg.accepted_rx > 5.2
    hyg, out = run(ds)
    assert set(v for rx, v in out if 5.1 < rx < 5.3) == {'repeat'}
    assert set(v for rx, v in out if rx > 5.4) == {'stuck'}
    # the stale gate sees nothing fresh after 0.3 s
    assert max(hyg.new_rx) < 5.4 and hyg.accepted_rx < 5.4


def test_out_of_order_dropped_however_far_back_until_it_lasts():
    epochs = track([(6, 2.0, 0.0)])
    ds = feed(epochs)
    i = next(j for j, (rx, d) in enumerate(ds) if rx > 3.0 and d['id'] == 'F')
    old = dict(ds[i][1], ts=iso(stamp_s(ds[i][1]['ts']) - (T0.timestamp()) - 0.05))
    hyg, out = run(ds[:i + 1] + [(ds[i][0] + 0.01, old)])
    assert out[-1][1] == 'out_of_order'
    hyg, out = run(ds[:i + 1] + [(ds[i][0] + 0.01, dict(ds[i][1]))])
    assert out[-1][1] == 'duplicate'
    far = dict(ds[i][1], ts=iso(stamp_s(ds[i][1]['ts']) - T0.timestamp() - 1.0))
    hyg, out = run(ds[:i + 1] + [(ds[i][0] + 0.01, far)])
    assert out[-1][1] == 'out_of_order'                 # a late backlog, not a clock step
    # Octane's clock steps back 3 s for good: dropped for reset_s (a shorter step ends by itself),
    # then followed
    ds = feed(track([(9, 2.0, 0.0)]))
    back = [(rx, dict(d, ts=iso(stamp_s(d['ts']) - T0.timestamp() - 3.0)) if rx > 3.0 else d)
            for rx, d in ds]
    hyg, out = run(back)
    assert {v for rx, v in out if 3.1 < rx < 4.95} == {'out_of_order'}
    assert hyg.counts['time_jump'] == 2 and rejected(out, 5.2) == [] and hyg.accepted_rx > 8.8


def test_one_epoch_jump_of_both_antennas_does_not_poison_the_track():
    def jump(k, t, ep, d):
        if 5.0 <= t < 5.066:                            # one relay of each antenna, 5 m sideways
            d['lat'], d['lon'] = to_latlon(*_shift(d, 5.0, 0.0))
        return d
    hyg, out = run(feed(track([(10, 2.0, 0.0)]), edit=jump))
    assert hyg.counts['older_position'] == 0
    assert [v for rx, v in out if 5.2 < rx < 5.5].count('accepted') > 3


def test_swap_is_rejected_until_reset_then_reanchored():
    def swap(k, t, ep, d):
        if 6.0 <= t < 9.0:
            d['lat'], d['lon'] = antenna(ep, k, swap=True)
        return d
    epochs = track([(5, 2.0, 0.0), (8, 2.0, 1 / R_TURN)])
    hyg, out = run(feed(epochs, edit=swap))
    assert rejected(out, 0, 6.0) == []
    rej = rejected(out, 6.0, 9.0)
    # the antennas switch at their own relay times: the first mixed pair breaks the baseline, then
    # the heading
    assert rej and rej[0][0] < 6.15 and {v for _, v in rej} == {'baseline', 'heading'}
    # a re-delivery of a rejected position is not a usable repeat (beacon_node marks only usable
    # ones FIX)
    assert 'repeat' not in {v for rx, v in out if 6.05 < rx < 7.9}
    assert 'repeat_unused' in {v for rx, v in out if 6.05 < rx < 7.9}
    # rejected for reset_s, then the swapped pair is taken (GNSS alone cannot tell), and the real
    # one after the swap
    acc = [rx for rx, v in out if v in ('accepted', 'reanchored') and 6.0 <= rx < 8.0]
    assert acc == [] or acc[0] > 6.0 + 1.9
    assert 'reanchored' in [v for rx, v in out if rx >= 8.0]


def test_lateral_step_of_both_antennas_is_rejected():
    def step(k, t, ep, d):
        if 5.0 <= t < 10.0:
            d['lat'], d['lon'] = to_latlon(*_shift(d, 1.0, 0.0))
        return d
    hyg, out = run(feed(track([(14, 2.0, 0.0)]), edit=step))     # driving north: east is sideways
    assert rejected(out, 0, 5.0) == []
    rej = rejected(out, 5.0, 10.0)
    # first the mixed pair (one antenna shifted): heading
    assert rej[0][0] < 5.15
    # then both shifted: the midpoint jumped sideways
    assert 'step' in {v for _, v in rej}
    assert not [v for rx, v in out if 5.1 < rx < 6.9 and v in ('accepted', 'reanchored')]


def _shift(d, east, north):
    e, n = enu_offset(LAT0, LON0, d['lat'], d['lon'])
    return e + east, n + north


@pytest.mark.parametrize('east,north,why', [(0.0, 1.0, 'baseline'), (1.0, 0.0, 'heading')])
def test_one_antenna_jump(east, north, why):
    # front antenna, held for 1 s, driving north at 1 m/s
    def jump(k, t, ep, d):
        if k == 0 and 5.0 <= t < 6.0:
            d['lat'], d['lon'] = to_latlon(*_shift(d, east, north))
        return d
    hyg, out = run(feed(track([(10, 1.0, 0.0)]), edit=jump))
    rej = rejected(out, 5.0, 6.1)
    assert rej and rej[0][1] == why and rej[0][0] < 5.15


def test_rtk_float_scatter_is_mostly_rejected():
    import random
    rng = random.Random(1)

    def scatter(k, t, ep, d):
        if 5.0 <= t < 10.0:
            d['lat'], d['lon'] = to_latlon(*_shift(d, rng.gauss(0, 0.3), rng.gauss(0, 0.3)))
        return d
    hyg, out = run(feed(track([(12, 2.0, 0.0)]), edit=scatter))
    inside = [v for rx, v in out if 5.1 < rx < 10.0]
    assert sum(v not in ('accepted', 'reanchored') for v in inside) > 0.6 * len(inside)


def test_long_gap_in_a_turn_reanchors_instead_of_locking_out():
    epochs = track([(5, 3.5, 0.0), (10, 3.5, 1 / R_TURN)])
    hyg, out = run(feed(epochs, edit=lambda k, t, ep, d: None if 7.0 <= t < 10.0 else d))
    assert rejected(out) == []
    assert [v for rx, v in out if rx > 10.0 and v in ('accepted', 'reanchored')][0] == 'reanchored'


def test_one_antenna_behind_in_a_fast_turn_never_becomes_the_reference():
    # front quiet 0.45 s at 3.5 m/s, full lock
    epochs = track([(5, 3.5, 0.0), (8, 3.5, 1 / R_TURN)])
    hyg, out = run(feed(epochs, edit=lambda k, t, ep, d: None if k == 0 and 7.0 <= t < 7.45
                        else d))
    # good pairs after it: no lock-out
    assert rejected(out, 7.5) == []
    # the mixed pairs' heading was off
    assert hyg.counts['heading'] > 0


def test_clock_step_and_late_burst_are_time_jumps_not_rejections():
    clock = feed(track([(10, 3.5, 0.0)]), edit=lambda k, t, ep, d: dict(d, ts=iso(t + 0.5))
                 if t >= 5.0 else d)
    hyg, out = run(clock)
    assert hyg.counts['time_jump'] == 2 and rejected(out) == []
    burst = [(max(rx, 6.0) if 5.0 <= rx < 6.0 else rx, d) for rx, d in feed(track([(10, 3.5,
                                                                                    0.0)]))]
    hyg, out = run(sorted(burst, key=lambda x: x[0]))
    assert hyg.counts['time_jump'] >= 2 and rejected(out) == []
