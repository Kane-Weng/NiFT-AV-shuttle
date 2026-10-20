"""
localization_filter.LocalizationFilter.

The shuttle's EKF on synthetic drives with its geometry (wheelbase 2.082 m, full lock 15 deg = a
7.77 m radius, base_link mid-wheelbase, antennas over the axles as in the kinematic sim), GNSS
epochs at 10 Hz from both antennas, the steering input the truth used.
"""
import math
import random

from nift_common.vehicle import ShuttleConfig
from nift_sensor.localization_filter import FilterConfig, LocalizationFilter, StampClock
import pytest

SHUTTLE = ShuttleConfig()
L = SHUTTLE.wheel_base
# base_link ahead of the rear axle; antennas at +-HALF from base_link
HALF = SHUTTLE.base_link_x
FULL = SHUTTLE.max_steering_deg     # full lock [deg]
R_FULL = L / math.tan(math.radians(FULL))
ACC = 0.5                       # truth speed ramps [m/s^2]
DT = 0.001                      # truth table step
# the kinematic sim (and this file's feed) reports the rear axle's speed
SIM = FilterConfig(speed_point_x_m=-HALF)


class Truth:
    """
    Kinematic bicycle (rear axle, exact arcs) along (duration, speed, steer_deg[, decel]) segments.

    Speed ramps at ACC (or `decel` when given) towards the segment's, steering changes at once (the
    kinematic sim's default).
    """

    def __init__(self, segments, x=0.0, y=0.0, yaw=0.0, v=None):
        self.rows, self.changes = [], []
        v = segments[0][1] if v is None else v
        t = 0.0
        for seg in segments:
            duration, target, steer = seg[:3]
            acc = seg[3] if len(seg) > 3 else ACC
            self.changes.append((t, steer, math.copysign(1, target) if target else 1, seg[3]
                                 if len(seg) > 3 else 0.0))
            kap = math.tan(math.radians(steer)) / L
            for _ in range(round(duration / DT)):
                self.rows.append((t, x, y, yaw, v, steer))
                v1 = target if abs(target - v) <= acc * DT else v + math.copysign(acc * DT,
                                                                                  target - v)
                ds = 0.5 * (v + v1) * DT
                dpsi = kap * ds
                c = ds * (1.0 - dpsi * dpsi / 24.0)
                x += c * math.cos(yaw + 0.5 * dpsi)
                y += c * math.sin(yaw + 0.5 * dpsi)
                yaw += dpsi
                v = v1
                t += DT
        self.end = t

    def at(self, t):
        return self.rows[min(len(self.rows) - 1, max(0, round(t / DT)))]

    def point(self, t, d):
        """Map position of the centreline point d ahead of the rear axle at t."""
        _, x, y, yaw, _, _ = self.at(t)
        return x + d * math.cos(yaw), y + d * math.sin(yaw)

    def base(self, t):
        x, y = self.point(t, HALF)
        return x, y, self.at(t)[3]


def drive(segments, delay=0.0, transport=0.025, rear_lag=0.03, drop=(), drop_antenna=None,
          edit=None,
          steer_input=True, cfg=None, repeat_s=None, gear=True, decel_input=True, backlog=None,
          unpaired=(),
          backlog_antenna=None, jitter=0.0, no_speed=(), antennas=(2 * HALF, 0.0)):
    """
    Feed a truth drive into a filter and record pose and status.

    Feed a Truth's antenna epochs (10 Hz, both antennas, stamp = epoch + delay, arriving transport
    later; the rear rear_lag after the front) into a filter; check pose_at / status at 50 Hz. drop:
    (start, duration) gaps; drop_antenna: 'front' / 'rear' / None (both); edit(t, k, x, y) -> (x,
    y) corrupts an epoch; repeat_s: each position delivered again this much later with a fresh
    stamp (an Octane repeat); gear=False: direction unknown; backlog=(start, duration): nothing
    relayed for `duration`, then the held epochs in order at 15 Hz under fresh stamps until caught
    up (the shuttle feed after a gap); unpaired: (start, duration) spans in which the delivered
    positions come without a partner (gnss_hygiene no_pair); jitter: stamp noise [s, Gaussian,
    clipped at 0.12 s]; no_speed: (start, duration) spans without speed updates; antennas: where
    the antennas really are, ahead of the rear axle (the filter is configured with the vehicle.yaml
    lever arms: 2 x HALF and 0). Returns (filter, truth, rows): rows = (t, error [m], yaw error
    [deg], status, pose).
    """
    tr = Truth(segments)
    f = LocalizationFilter(cfg or SIM)
    clock = StampClock(delay_s=delay, transport_s=transport)
    events = []                                                 # (time, order, kind, payload)
    for t, steer, direction, decel in tr.changes:
        events.append((t, 0, 'input', (steer if steer_input else None, direction if gear else 0,
                                       decel if decel_input else 0.0)))
    n = int(tr.end / 0.1)
    relay = [0.0, 0.0]
    rng = random.Random(1)
    for i in range(1, n):
        te = i * 0.1
        for k, name in ((0, 'front'), (1, 'rear')):
            stamp = te + delay + (max(-0.12, min(0.12, rng.gauss(0.0, jitter))) if jitter else 0.0)
            if backlog and te >= backlog[0] and backlog_antenna in (None, name):
                stamp = max(stamp, relay[k] + 1.0 / 15.0, backlog[0] + backlog[1])
            relay[k] = stamp
            if any(a <= te < a + d for a, d in drop) and drop_antenna in (None, name):
                continue
            x, y = tr.point(te, antennas[k])
            if edit is not None:
                x, y = edit(te, k, x, y)
            rx = stamp + transport + (rear_lag if k else 0.0)
            events.append((rx, 1, 'pos', (stamp, name, x, y, abs(tr.at(te)[4]),
                                          not any(a <= te < a + d for a, d in unpaired))))
            if repeat_s:
                events.append((rx + repeat_s, 1, 'pos', (stamp + repeat_s, name, x, y, None,
                                                         True)))
    for i in range(1, int(tr.end / 0.02)):
        events.append((i * 0.02, 2, 'check', None))
    events.sort(key=lambda e: (e[0], e[1]))
    rows = []
    for t, _, kind, p in events:
        if kind == 'input':
            f.set_input(t, *p)
        elif kind == 'pos':
            stamp, name, x, y, spd, paired = p
            tm = clock.measurement_time(stamp, t)
            f.update_antenna(tm, name, x, y, paired)
            if name == 'front' and spd is not None and not any(a <= stamp < a + d for a,
                                                               d in no_speed):
                f.update_speed(tm, spd)
        else:
            pose, st = f.pose_at(t), f.status(t)
            if pose is None:
                rows.append((t, math.inf, math.inf, st, None))
                continue
            bx, by, byaw = tr.base(t)
            yerr = math.degrees((pose.yaw - byaw + math.pi) % (2 * math.pi) - math.pi)
            rows.append((t, math.hypot(pose.x - bx, pose.y - by), yerr, st, pose))
    return f, tr, rows


def level_seq(rows, t0=-1.0, t1=1e9):
    """Return the status levels in order, repeats collapsed."""
    out = []
    for r in rows:
        if t0 <= r[0] < t1 and (not out or r[3].level != out[-1]):
            out.append(r[3].level)
    return out


def worst(rows, t0=-1.0, t1=1e9):
    sel = [r for r in rows if t0 <= r[0] < t1]
    return max(r[1] for r in sel), max(abs(r[2]) for r in sel)


def levels(rows, t0=-1.0, t1=1e9):
    """Status timeline: [(t, level, reason)] at every change."""
    out = []
    for r in rows:
        if t0 <= r[0] < t1 and (not out or (r[3].level, r[3].reason) != out[-1][1:]):
            out.append((round(r[0], 2), r[3].level, r[3].reason))
    return out


# -------------------------------------------------------------------------------------------------------------
# #
def test_straight_tracks_to_cm():
    f, _, rows = drive([(12.0, 2.0, 0.0)])
    err, yerr = worst(rows, 2.0)
    assert err < 0.02 and yerr < 0.2
    assert rows[-1][4].v == pytest.approx(2.0, abs=0.02)
    assert levels(rows, 2.0) == [(2.0, 'OK', '')]
    assert f.counts['rejected'] == 0


@pytest.mark.parametrize('v', [1.0, 2.0, 3.5])
@pytest.mark.parametrize('side', [1, -1])
def test_full_lock_turn(v, side):
    f, _, rows = drive([(3.0, v, 0.0), (2 * math.pi * R_FULL / v, v, side * FULL), (3.0, v, 0.0)])
    err, yerr = worst(rows, 1.5)
    assert err < 0.03 and yerr < 0.3
    mid = rows[len(rows) // 2][4]
    assert mid.yaw_rate == pytest.approx(side * v / R_FULL, rel=0.02)
    assert f.counts['rejected'] == 0


def test_predicts_to_now():
    """
    Positions 0.1 s old at their stamp + 25 ms transport.

    The direct transform is v x 0.125 s behind, the filter predicts the pose to now.
    """
    v = 2.0
    f, tr, rows = drive([(10.0, v, 0.0), (8.0, v, FULL)], delay=0.1)
    err, _ = worst(rows, 2.0)
    assert err < 0.03
    # what the direct transform would publish at the same instants: the newest delivered epoch, as
    # is
    lag = 0.1 + 0.025
    direct = max(math.dist(tr.base(t)[:2], tr.base(t - lag)[:2]) for t, *_ in rows if t > 2.0)
    assert direct > 0.2 and err < 0.15 * direct


@pytest.mark.parametrize('gap', [0.3, 0.8, 1.5, 3.0])
@pytest.mark.parametrize('steer', [0.0, FULL])
def test_dropout_bridging_and_status(gap, steer):
    """
    Both antennas silent for `gap` at 1.5 m/s, straight or at full lock.

    DEGRADED after ok_age_s, LOST after bridge_s (time first at this speed), error < 10 cm while
    bridging <= 1.5 s, recovery after.
    """
    v, t0 = 1.5, 6.05
    f, tr, rows = drive([(4.0, v, 0.0), (12.0, v, steer)], drop=[(t0, gap)])
    last = t0 - 0.05                                            # the last epoch before the gap
    first = t0 - 0.05 + math.ceil(gap / 0.1 - 1e-9) * 0.1 + 0.1  # ... and the first after it
    seq = levels(rows, t0, first + 1.0)
    # up to the first update after the gap
    bridged, _ = worst(rows, t0, first + 0.025)
    if gap < 0.5:
        assert [s[1] for s in seq] == ['OK']
    elif gap < 1.5:
        assert [s[1:] for s in seq] == [('OK', ''), ('DEGRADED', 'bridging'), ('OK', '')]
        assert seq[1][0] == pytest.approx(last + 0.5, abs=0.03)
    else:
        assert [s[1:] for s in seq][:3] == [('OK', ''), ('DEGRADED', 'bridging'), ('LOST',
                                                                                   'outage')]
        assert seq[2][0] == pytest.approx(last + 1.5, abs=0.03)
        # restart on the next consistent pair, then settle
        if gap > 2.0:
            assert [s[1:] for s in seq][3:] == [('DEGRADED', 'settling'), ('OK', '')]
            assert f.counts['initialised'] == 2
    if gap <= 1.5:
        assert bridged < 0.10
    assert worst(rows, first + 1.0)[0] < 0.03


def test_distance_bound_before_time():
    """At 3.5 m/s, 3 m of bridging come after 0.86 s: LOST for distance, before bridge_s."""
    f, _, rows = drive([(10.0, 3.5, 0.0)], drop=[(6.05, 3.0)])
    seq = levels(rows, 6.0, 8.0)
    assert [s[1:] for s in seq][:3] == [('OK', ''), ('DEGRADED', 'bridging'), ('LOST', 'distance')]
    assert seq[2][0] == pytest.approx(6.0 + 3.0 / 3.5, abs=0.03)


@pytest.mark.parametrize('antenna', ['front', 'rear'])
def test_one_antenna_keeps_it_ok(antenna):
    """
    The other antenna's positions come without a partner (no_pair) during the dropout.

    They keep it OK.
    """
    f, _, rows = drive([(4.0, 2.0, 0.0), (10.0, 2.0, FULL)], drop=[(3.0, 6.0)],
                       drop_antenna=antenna,
                       unpaired=[(3.0, 6.2)])
    assert levels(rows, 2.0) == [(2.0, 'OK', '')]
    assert worst(rows, 2.0)[0] < 0.05
    assert f.counts['unpaired_ignored'] == 0


def test_unpaired_positions_cannot_revive_a_bridging_filter():
    """
    Unpaired positions cannot revive a bridging filter.

    Both antennas silent 0.8 s, then only unchecked (no_pair) positions for 2 s, here 2 m off along
    the travel (an antenna swap looks like that): ignored while bridging -> LOST, never dragged.
    """
    def edit(t, k, x, y):
        return (x + 2.0, y) if 6.85 <= t < 8.85 else (x, y)
    f, _, rows = drive([(12.0, 1.0, 0.0)], drop=[(6.05, 0.8)], unpaired=[(6.85, 2.0)], edit=edit)
    assert f.counts['unpaired_ignored'] > 10
    assert 'LOST' in level_seq(rows, 6.0, 8.85)
    lost = next(r[0] for r in rows if r[0] >= 6.0 and r[3].level == 'LOST')
    assert worst(rows, 6.0, lost)[0] < 0.05


def test_turn_entry_in_a_gap_follows_the_steering():
    """
    The turn starts 0.2 s into a 1.5 s gap.

    The steering input carries the pose round it; without one the filter assumes the straight goes
    on.
    """
    segs = [(6.25, 1.5, 0.0), (8.0, 1.5, FULL)]
    _, _, rows = drive(segs, drop=[(6.05, 1.5)])
    with_input = worst(rows, 6.05, 7.65)[0]
    _, _, rows = drive(segs, drop=[(6.05, 1.5)], steer_input=False)
    without = worst(rows, 6.05, 7.65)[0]
    assert with_input < 0.10 < without


@pytest.mark.parametrize('jump', [0.3, 1.0, 5.0])
def test_single_jump_rejected(jump):
    """
    One front epoch off sideways (along the travel, the stamp jitter allows ~3.7 x v x 45 ms.

    0.33 m at 2 m/s).
    """
    segs = [(4.0, 2.0, 0.0), (8.0, 2.0, FULL)]
    tr = Truth(segs)

    def edit(t, k, x, y):
        yaw = tr.at(t)[3]
        return (x - jump * math.sin(yaw),
                y + jump * math.cos(yaw)) if k == 0 and abs(t - 6.0) < 0.01 else (x, y)
    f, _, rows = drive(segs, edit=edit)
    assert f.counts['rejected'] == 1
    assert worst(rows, 2.0)[0] < 0.03
    assert levels(rows, 2.0) == [(2.0, 'OK', '')]


@pytest.mark.parametrize('v, seed', [(1.5, 7), (3.5, 9)])
def test_rtk_float_is_degraded(v, seed):
    """
    0.3 m scatter on both antennas for 5 s (an RTK float episode the upstream checks partly pass).

    Inconsistent (DEGRADED) within 1 s, then no position is trusted: LOST within 2.6 s, pose within
    0.25 m until then. (After LOST a float pair may restart it; 3.3's latched SAFE_STOP has stopped
    the shuttle by then.)
    """
    import random
    rng = random.Random(seed)

    def edit(t, k, x, y):
        return (x + rng.gauss(0.0, 0.3), y + rng.gauss(0.0, 0.3)) if 6.0 <= t < 11.0 else (x, y)
    f, _, rows = drive([(16.0, v, 0.0)], edit=edit)
    assert 'inconsistent' in {r[3].reason for r in rows if 6.0 <= r[0] < 7.0}
    lost = next(r[0] for r in rows if r[0] >= 6.0 and r[3].level == 'LOST')
    assert lost < 8.6 and level_seq(rows, 7.0, lost) == ['DEGRADED']
    assert worst(rows, 6.0, lost)[0] < 0.25


def test_persistent_offset_stops_then_restarts():
    """
    Both antennas 1 m off for 5 s.

    Rejected -> bridging -> LOST after bridge_s -> restart on the offset GNSS (it cannot tell which
    is true) -> the same when the offset ends.
    """
    def edit(t, k, x, y):
        return (x + 1.0, y) if 6.0 <= t < 11.0 else (x, y)
    f, _, rows = drive([(16.0, 1.0, 0.0)], edit=edit)
    assert level_seq(rows, 5.0, 16.0) == ['OK', 'DEGRADED', 'LOST', 'DEGRADED'] * 2 + ['OK']
    assert {s[2] for s in levels(rows, 5.0, 16.0)} == {'', 'bridging', 'inconsistent', 'outage',
                                                       'settling'}
    assert f.counts['initialised'] == 3
    # driven on the model, not on the offset
    assert worst(rows, 6.0, 7.45)[0] < 0.05


def test_out_of_sequence_rewinds_to_the_same_answer():
    segs = [(4.0, 2.0, 0.0), (6.0, 2.0, FULL)]
    fa, tr, _ = drive(segs, rear_lag=0.03)
    # each rear epoch arrives after the next front one
    fb, _, _ = drive(segs, rear_lag=0.13)
    assert fa.counts['rewind'] == 0 and fb.counts['rewind'] > 50 and fb.counts['late'] == 0
    # the same updates, in another arrival order
    p, q = fa.pose_at(tr.end + 0.2), fb.pose_at(tr.end + 0.2)
    assert (q.x, q.y, q.yaw, q.v) == pytest.approx((p.x, p.y, p.yaw, p.v), abs=1e-9)


def test_jittered_stamps_no_false_rejections():
    """
    Relay stamps +-45 ms rms off the epoch at 3.5 m/s in full-lock turns.

    No rejections (along-track noise).
    """
    import random
    rng = random.Random(3)
    segs = [(4.0, 3.5, 0.0), (8.0, 3.5, FULL), (4.0, 3.5, -FULL)]
    tr = Truth(segs)
    f = LocalizationFilter(SIM)
    for t, steer, direction, decel in tr.changes:
        f.set_input(t, steer, direction, decel)
    errs = []
    for i in range(1, int(tr.end / 0.1)):
        te = i * 0.1
        for k, name in ((0, 'front'), (1, 'rear')):
            x, y = tr.point(te, HALF + (HALF if k == 0 else -HALF))
            tm = te + max(-0.12, min(0.12, rng.gauss(0.0, 0.045)))
            f.update_antenna(tm, name, x, y)
        f.update_speed(te, 3.5)
        if te > 2.0:
            p = f.pose_at(te + 0.05)
            errs.append(math.dist((p.x, p.y), tr.base(te + 0.05)[:2]))
    assert f.counts['rejected'] <= 2
    assert max(errs) < 0.12


def test_repeats_while_moving_are_ignored():
    f, _, rows = drive([(4.0, 2.0, 0.0), (6.0, 2.0, FULL)], repeat_s=0.05)
    assert f.counts['repeat'] > 100
    assert worst(rows, 2.0)[0] < 0.03


def test_reverse_from_the_gear():
    """
    Unsigned GNSS speed, gear REVERSE.

    The speed updates fit (none gated out) and the pose moves backwards.
    """
    f, _, rows = drive([(10.0, -1.0, 0.0), (6.0, -1.0, -FULL)])
    assert rows[-1][4].v == pytest.approx(-1.0, abs=0.03)
    assert worst(rows, 2.0)[0] < 0.03
    assert f.counts['speed'] > 100 and f.counts['speed_rejected'] == 0


def test_reverse_with_the_gear_unknown():
    """
    Driven by hand (no gear on CAN).

    The positions show the direction, then the unsigned speed updates fit.
    """
    f, _, rows = drive([(10.0, -1.0, 0.0), (6.0, -1.0, -FULL)], gear=False)
    assert rows[-1][4].v == pytest.approx(-1.0, abs=0.03)
    assert worst(rows, 3.0)[0] < 0.05
    assert f.counts['speed'] > 100 and f.counts['initialised'] == 1


def test_swapped_pair_cannot_restart_it_backwards():
    """
    Antennas swapped for 3 s (heading off by 180 deg, antenna distance right).

    Rejected, LOST, and the restart waits for a pair that agrees with the dead-reckoned heading.
    """
    segs = [(14.0, 1.0, 0.0)]
    tr = Truth(segs)

    def edit(t, k, x, y):
        return tr.point(t, 0.0 if k == 0 else L) if 6.0 <= t < 9.0 else (x, y)
    f, _, rows = drive(segs, edit=edit)
    assert f.counts['init_bad_yaw'] > 5 and f.counts['initialised'] == 2
    assert max(abs(r[2]) for r in rows if r[0] > 2.0 and r[3].level != 'LOST') < 2.0
    assert levels(rows, 12.0) == [(12.0, 'OK', '')]


@pytest.mark.parametrize('steer', [0.0, FULL])
def test_backlog_after_a_gap_does_not_drag_the_pose_back(steer):
    """
    A backlog after a gap does not drag the pose back.

    1 s without deliveries at 1.5 m/s, then the held epochs under fresh stamps (1 s old at first,
    caught up ~2 s later): they correct across the travel only, the pose stays (within 0.1 m) where
    the model puts it, DEGRADED while the data is old, never LOST.
    """
    f, _, rows = drive([(4.0, 1.5, 0.0), (12.0, 1.5, steer)], backlog=(6.05, 1.0))
    # the drained tail (within 1 sigma of jitter) drags a little
    assert worst(rows, 6.0, 10.0)[0] < 0.1
    assert level_seq(rows, 6.0, 12.0) == ['OK', 'DEGRADED', 'OK']
    assert f.counts['rejected'] == 0


def test_fresh_data_after_a_gap_is_not_biased():
    """
    Jittered stamps (45 ms) and fresh data after a 1 s gap (the sim's feed).

    No forward bias from the backlog rule (a lag within the usual jitter is a normal update).
    """
    f, _, rows = drive([(4.0, 2.0, 0.0), (12.0, 2.0, FULL)], drop=[(6.05, 1.0)], jitter=0.045)
    after = [r[1] for r in rows if 7.3 <= r[0] < 9.5]
    before = [r[1] for r in rows if 3.0 <= r[0] < 6.0]
    assert sum(after) / len(after) < 0.04 \
        and sum(after) / len(after) < 3.0 * sum(before) / len(before) + 0.01


def test_stale_antenna_does_not_age_a_fresh_filter():
    """
    Only the rear antenna has a 1 s gap and backlog, the front stays fresh.

    The rear's old positions must not make the status age jump back (it counts from the newest
    measurement).
    """
    f, _, rows = drive([(4.0, 1.5, 0.0), (10.0, 1.5, FULL)], backlog=(6.05, 1.0),
                       backlog_antenna='rear')
    assert levels(rows, 2.0) == [(2.0, 'OK', '')]
    assert max(r[3].age_s for r in rows if r[0] > 2.0) < 0.2
    assert worst(rows, 2.0)[0] < 0.05


def test_backlog_while_accelerating():
    """
    The shuttle speeds up 1 -> 2 m/s through a 1 s gap and its backlog.

    The stale speeds must not pin the filter to the old speed (it would fall behind and gate out
    the caught-up positions). Along the travel it is ~0.5 m behind while the backlog drains (GNSS
    alone cannot tell; ~0.6 m), then back within 5 cm, never LOST.
    """
    f, _, rows = drive([(6.05, 1.0, 0.0), (10.0, 2.0, 0.0)], backlog=(6.05, 1.0))
    assert 'LOST' not in level_seq(rows, 6.0, 14.0)
    assert worst(rows, 6.0, 10.5)[0] < 0.7 and worst(rows, 10.5, 14.0)[0] < 0.05


def test_stale_speed_counts_less():
    """
    Right after a 1 s gap the speed is as old as the backlog.

    One reading of 2 m/s (the filter says 1) moves the estimate only part of the way (trusted like
    a fresh one it would jump to 2).
    """
    f, tr, rows = drive([(7.0, 1.0, 0.0)], drop=[(5.95, 10.0)])
    assert rows[-1][3].level == 'DEGRADED'
    assert f.update_antenna(7.0, 'front', *tr.point(6.999, L)) == 'accepted'
    assert f.update_speed(7.0, 2.0) == 'speed'
    assert 1.1 < f.pose_at(7.0).v < 1.6


def test_curvature_stays_within_full_lock():
    """
    No steering input and a path tighter than full lock (3 m radius).

    Kappa is kept within 1.15 x full lock.
    """
    f, _, rows = drive([(3.0, 1.0, 0.0), (6.0, 1.0, 35.0)], steer_input=False)
    k_max = 1.15 * math.tan(math.radians(FULL)) / L
    assert max(abs(r[4].kappa) for r in rows if r[4] is not None) <= k_max + 1e-9
    assert max(abs(r[4].kappa) for r in rows if r[4] is not None) > 0.9 * k_max


@pytest.mark.parametrize('off', [0.4, -0.4])
def test_lever_arm_off_starts_and_shifts_like_direct(off):
    """
    A wrong lever arm still starts, and shifts the pose like direct mode.

    The antennas really at +-(1.041 + off) from base_link, the filter configured with +-1.041 (the
    bench's lever_arm_+-0.4m): it learns the antenna distance (2.88 / 1.28 m) from the first pairs
    and adopts it for the rear lever arm. The front one defines base_link, so the pose is shifted
    along the heading by `off`, as the direct transform's (front antenna - 1.041 along the
    heading), not blocked.
    """
    segs = [(4.0, 2.0, 0.0), (8.0, 2.0, FULL)]
    f, tr, rows = drive(segs, antennas=(2 * HALF + off, -off))
    assert f.baseline_measured == pytest.approx(2 * HALF + 2 * off, abs=0.01)
    assert f.rear_antenna_x == pytest.approx(-HALF - 2 * off, abs=0.01)
    assert f.counts['initialised'] == 1 and f.counts['rejected'] == 0
    assert levels(rows, 2.0) == [(2.0, 'OK', '')]
    worst_shift = 0.0
    for t, _, _, _, p in rows:
        if t > 2.0:
            bx, by, yaw = tr.base(t)
            # front - 1.041 x heading
            direct = (bx + off * math.cos(yaw), by + off * math.sin(yaw))
            worst_shift = max(worst_shift, math.dist((p.x, p.y), direct))
    assert worst_shift < 0.05


def test_lever_arm_off_swapped_pair_still_refused():
    """
    After adopting the measured distance, antennas swapped for 3 s.

    LOST, the restart is refused while the pair's heading is flipped, then it restarts on the true
    pair.
    """
    off = 0.4
    segs = [(14.0, 1.0, 0.0)]
    tr = Truth(segs)
    ant = (2 * HALF + off, -off)

    def edit(t, k, x, y):
        return tr.point(t, ant[1 - k]) if 6.0 <= t < 9.0 else (x, y)
    f, _, rows = drive(segs, edit=edit, antennas=ant)
    assert f.baseline_measured == pytest.approx(2 * HALF + 2 * off, abs=0.01)
    assert f.counts['init_bad_yaw'] > 5 and f.counts['initialised'] == 2
    assert max(abs(r[2]) for r in rows if r[0] > 2.0 and r[3].level != 'LOST') < 2.0
    assert levels(rows, 12.0) == [(12.0, 'OK', '')]


def test_unstable_antenna_distance_does_not_start():
    """
    Start pairs 2.88 m apart +-0.1 m, alternating (one antenna jumping).

    Never stable, never adopted; the status says why.
    """
    off = 0.4

    def edit(t, k, x, y):
        return (x + (0.1 if round(t * 10) % 2 else -0.1), y) if k == 1 else (x, y)
    f, _, rows = drive([(6.0, 0.0, 0.0)], edit=edit, antennas=(2 * HALF + off, -off))
    assert f.counts['initialised'] == 0 and f.baseline_measured is None
    st = rows[-1][3]
    assert st.reason == 'uninitialised' and 'vs configured 2.08 m' in st.detail \
        and 'learning it' in st.detail


def test_cannot_start_says_why():
    """
    Adopting off (baseline_learn_pairs 0).

    It refuses the start, and the status names both distances.
    """
    cfg = FilterConfig(speed_point_x_m=-HALF, baseline_learn_pairs=0)
    f, _, rows = drive([(4.0, 0.0, 0.0)], antennas=(2 * HALF + 0.4, -0.4), cfg=cfg)
    st = rows[-1][3]
    assert f.counts['initialised'] == 0 and st.level == 'LOST' and st.reason == 'uninitialised'
    assert st.detail == 'antenna distance 2.88 m vs configured 2.08 m'


def test_restart_does_not_relearn_the_distance():
    """
    Started on the configured distance; after a gap the pairs are 0.5 m longer (one antenna moved.

    A fault): no restart, and the status says why.
    """
    tr = Truth([(12.0, 1.0, 0.0)])

    def edit(t, k, x, y):
        if t >= 8.0 and k == 1:
            _, _, _, yaw, _, _ = tr.at(t)
            return x - 0.5 * math.cos(yaw), y - 0.5 * math.sin(yaw)
        return x, y
    f, _, rows = drive([(12.0, 1.0, 0.0)], drop=[(6.05, 1.95)], edit=edit)
    assert f.counts['initialised'] == 1 and f.baseline_measured is None
    st = rows[-1][3]
    assert st.level == 'LOST' \
        and st.detail.startswith('antenna distance 2.58 m vs configured 2.08 m')


def test_small_rear_lever_arm_error_is_learned_too():
    """
    The rear antenna 0.10 m further back than configured.

    Inside the old start tolerance (0.15 m), but the two antennas would disagree for good (5 sigma
    at rest); the first start measures and adopts it.
    """
    f, _, rows = drive([(3.0, 0.0, 0.0), (6.0, 1.5, 0.0)], antennas=(2 * HALF, -0.10))
    assert f.baseline_measured == pytest.approx(2 * HALF + 0.10, abs=0.01)
    assert f.counts['rejected'] == 0 and worst(rows, 1.5)[0] < 0.03


def test_lost_holds_until_a_pair():
    """
    Once LOST, the filter waits for a pair.

    Once LOST, a late position from inside the gap (a held-back delivery) does not bring the filter
    back.
    """
    f, tr, rows = drive([(7.6, 1.0, 0.0)], drop=[(6.05, 5.0)])
    assert f.status(7.6).level == 'LOST'
    assert f.update_antenna(6.4, 'front', *tr.point(6.4, L)) == 'init_waiting'
    assert f.status(7.62).level == 'LOST'


def test_braking_input_stops_the_prediction():
    """
    Brakes at 0.5 m/s^2 from 2 m/s 0.3 s into a 6 s outage.

    With the braking input the dead-reckoned pose stops where the shuttle does (never reverses);
    without it, it runs on at 2 m/s.
    """
    segs = [(6.35, 2.0, 0.0), (8.0, 0.0, 0.0, 0.5)]
    _, tr, rows = drive(segs, drop=[(6.05, 6.0)])
    # dead reckoning, 1.15 s after the stop
    t, err, _, st, p = next(r for r in rows if r[0] >= 11.5)
    assert st.level == 'LOST' and p.v == 0.0 and err < 0.15
    _, tr, rows = drive(segs, drop=[(6.05, 6.0)], decel_input=False)
    assert next(r for r in rows if r[0] >= 11.5)[1] > 5.0


def test_braking_input_does_not_freeze_a_wrong_speed():
    """
    Restarted after a 1.5 s gap while rolling out of a stop (braking input on, gear unknown.

    V starts at 0, the position uncertain by the gap's backlog bound): the braking clamps the
    predicted speed at 0 but must not make it certain, or the positions can never pull it up (found
    replaying a SAFE_STOP roll-out through the node).
    """
    f = LocalizationFilter(SIM)
    f.set_input(0.0, 0.0, 0, 0.5)
    f.update_speed(0.0, 1.6)
    f.update_antenna(0.0, 'front', L, 0.0)
    # 20 Hz, the rear first: the restart pair is 50 ms apart
    for i in range(30):
        tau = 0.05 * i
        t = 1.5 + tau
        x = 1.6 * 1.5 + 1.6 * tau - 0.25 * tau * tau
        f.update_antenna(t, 'rear', x, 0.0)
        f.update_antenna(t + 0.05, 'front', x + 1.6 * 0.05 + L, 0.0)
        f.update_speed(t + 0.05, 1.6 - 0.5 * tau)
    assert f.counts['rejected'] == 0 and f.status(3.0).level != 'LOST'
    assert f.pose_at(3.0).v == pytest.approx(1.6 - 0.5 * 1.5, abs=0.1)


def test_restart_at_rest_uses_the_dead_reckoned_speed():
    """
    A restart at rest uses the dead-reckoned speed.

    GNSS gone while braking to a stop (1 m/s^2 from 2 m/s), back 4 s later with the shuttle at
    rest: the restart takes the dead-reckoned speed (0), not the last measured one (2 m/s, 4 s
    old); with that, the identical positions at rest would be taken for stale repeats and the
    restarted filter would bridge instead of settling. (beacon_node's speed only resumes once its
    checks accept a pair again: here 0.5 s after the positions.)
    """
    _, _, rows = drive([(6.35, 2.0, 0.0), (8.0, 0.0, 0.0, 1.0)], drop=[(6.05, 4.0)],
                       no_speed=[(6.05, 4.6)])
    assert [s[1:] for s in levels(rows, 10.0)][-2:] == [('DEGRADED', 'settling'), ('OK', '')]
    assert worst(rows, 11.0, 14.0)[0] < 0.03


def test_no_steering_input_still_tracks_with_gnss():
    f, _, rows = drive([(4.0, 2.0, 0.0), (8.0, 2.0, FULL), (4.0, 2.0, 0.0)], steer_input=False)
    assert worst(rows, 2.0)[0] < 0.08


def test_needs_a_consistent_pair_to_start():
    f = LocalizationFilter()
    assert f.status(0.0).level == 'LOST' and f.pose_at(0.0) is None
    assert f.update_antenna(1.0, 'front', 3.0, 0.0) == 'init_waiting'
    # 1 m apart, the lever arms say 2.082
    assert f.update_antenna(1.01, 'rear', 2.0, 0.0) == 'init_bad_pair'
    # 0.3 s apart: not synchronous
    assert f.update_antenna(1.30, 'rear', 3.0 - L, 0.0) == 'init_waiting'
    assert f.update_antenna(1.35, 'front', 3.0, 0.0) == 'initialised'
    p = f.pose_at(1.35)
    assert (p.x, p.y, p.yaw) == pytest.approx((3.0 - HALF, 0.0, 0.0), abs=1e-6)
    assert f.status(1.4).reason == 'settling'
    # at rest, the same position is current
    assert f.update_antenna(1.70, 'front', 3.0, 0.0) == 'accepted'
    assert f.status(1.82).level == 'OK'


def test_late_update_dropped():
    f, _, _ = drive([(5.0, 1.0, 0.0)])
    assert f.update_antenna(2.0, 'front', 0.0, 0.0) == 'late'


def test_stamp_clock():
    c = StampClock(delay_s=0.08)
    # synchronised: stamp - delay
    assert c.measurement_time(100.0, 100.03) == pytest.approx(100.0 - 0.08)
    c = StampClock(transport_s=0.025)
    # synchronised first
    for i in range(100):
        assert c.measurement_time(90.0 + 0.033 * i,
                                  90.025 + 0.033 * i) == pytest.approx(90.0 + 0.033 * i)
    true = [100.0 + 0.033 * i for i in range(300)]
    # Octane's clock steps 0.5 s ahead
    ts = [c.measurement_time(t + 0.5, t + 0.025) for t in true]
    # never after the arrival
    assert all(t <= tt + 0.025 for t, tt in zip(ts, true))
    # 15 in a row agree: followed
    assert c.flagged == 14 and c.offset_s == pytest.approx(-0.5)
    # arrival - transport, then the offset
    assert max(abs(t - tt) for t, tt in zip(ts, true)) < 1e-9
    true = [200.0 + 0.033 * i for i in range(30)]
    # ... and steps back
    ts = [c.measurement_time(t, t + 0.025) for t in true]
    assert c.offset_s == 0.0 and max(abs(t - tt) for t, tt in zip(ts[14:], true[14:])) < 1e-9
    # 1 s held back, released
    held = [c.measurement_time(210.0 + 0.033 * i, 211.2 + 0.025) for i in range(30)]
    # at once 0.2 s later: not a clock step
    assert c.offset_s == 0.0
    # each keeps its stamp
    assert held == pytest.approx([210.0 + 0.033 * i for i in range(30)])
