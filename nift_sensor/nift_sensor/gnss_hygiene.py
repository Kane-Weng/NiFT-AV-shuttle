"""
Checks on the Octane beacon feed before beacon_node publishes it.

File: gnss_hygiene.py

Description:
  Checks on the Octane beacon feed before beacon_node publishes it, so a feed that is
  late, repeated or inconsistent shows up downstream as silence (beacon_node's stale
  gate, then trajectory_controller's SAFE_STOP) instead of as a wrong pose. ROS-free
  (test/test_gnss_hygiene.py).

  The real feed (shuttle garage bags, 2026-10-08): Octane relays each antenna's latest
  position ~15 times a second with a fresh `updated` stamp, but the receivers make 10
  positions a second, so ~40 % of the deliveries repeat a position. Positions carry 7
  decimals (~1 cm) and at rest stay bit-identical for up to 42 s. ~30 deliveries an hour
  arrive out of order (up to 83 ms back), and ~2 per minute of driving carry an older
  position than the one before under a newer stamp (up to 0.5 m behind). RTK noise is
  ~1 cm, heading noise ~0.2 deg.

  Per delivery, per antenna:
    out of order  a stamp older than that antenna's newest is dropped (late data). Only
                  if older stamps keep coming for reset_s is it Octane's clock that
                  stepped back: followed, as a time jump
    time jump     the stamp interval and the arrival interval differ by more than
                  time_jump_s (Octane's clock stepped, or a held-back burst arrives):
                  taken, and its pair is re-anchored instead of motion-checked
    duplicate     the same stamp again
    repeat        moving (that antenna reports >= stuck_speed_mps), the same
                  position under a fresh stamp (an epoch relayed again): keeps the
                  antenna fresh, as before, and its stamp is the one published, but
                  there is nothing new to check. Only a repeat of a position in the
                  accepted pair counts (else repeat_unused: a rejected or unpaired
                  position stays unused). At rest a repeat is a new position: the
                  held position is current (bit-identical for minutes)
    stuck         moving, a repeat of a position first stamped more than stuck_s ago:
                  a stuck feed (fresh stamps, frozen position), or an old position
                  re-sent after a gap. Not fresh. Moving, a position is relayed for at
                  most ~0.14 s (p99) before the next epoch arrives
    older         moving, a position more than order_tol_m behind that antenna's
                  latest new one, along its own track over the last second (an older
                  epoch relayed late under a fresh stamp). The track restarts below
                  stuck_speed_mps, so driving off the other way is not mistaken for it
  Per pair (each new position with the other antenna's latest new one, if their stamps
  are at most pair_window_s apart):
    baseline      the front - rear distance within baseline_tol_m of the reference,
                  plus what the speed can move one antenna between the two stamps
                  (and one receiver epoch). The reference is antenna_baseline_m, or
                  the median of the last baseline_learn_pairs synchronous pairs
                  whose stamps are so close that the speed moves an antenna at most
                  LEARN_MOTION_M between them, once 3/4 of them agree with it within
                  LEARN_AGREE_M (nothing is accepted until then). At rest that is
                  every synchronous pair, and the first ones agree. Moving, it keeps
                  the pairs of one receiver epoch and skips those an epoch apart,
                  which would bias the length. Stamps cannot show a backlog after a
                  gap (old positions under fresh stamps), so its pairs are the ones
                  that disagree. So a beacon_node restarted while driving learns it
                  too. A baseline learned while moving is learned again at the next
                  stop, the same way, because one antenna that lags for the whole
                  learning would bias every moving pair alike
    heading       the rear -> front heading within heading_tol_deg of the last
                  accepted pair's, plus the yaw the distance driven allows at the
                  minimum turning radius
    step          the antenna midpoint where the last accepted pair, the reported
                  speed and the mean heading put it: across within step_tol_m, along
                  within along_tol_m, each plus a share of the distance driven
  A pair that fails is not accepted: the last accepted pair stays current and ages.
  The checks compare with the last accepted synchronous pair (stamps at most sync_s
  apart): with one antenna an epoch or more behind, the rear -> front vector is off in
  a turn (15 deg at 3.5 m/s with the front 0.3 s old), so such a pair is checked and
  may be published, but never becomes the reference.
  reset_s after the last accepted pair, the next synchronous pair that passes the
  baseline check is accepted without the heading / step checks (re-anchor), so a long
  outage or a persistent offset never locks the feed out. reset_s is longer than the stale gate
  plus the controller's watchdog, so a driving shuttle's SAFE_STOP has latched by then (it
  may still be rolling: the brake is weak). GNSS alone cannot tell which of two persistent
  states is the true one.

Usage:
  hyg = BeaconHygiene(front_id, rear_id)
  verdict = hyg.add(normalize(payload), rx_s)     # 'accepted', or why not (VERDICTS)
  if hyg.accepted is not None:
      front, rear = hyg.accepted                  # normalize() dicts of the current pair
"""

import collections
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from typing import Optional

from nift_common.vehicle import ShuttleConfig

# one receiver epoch (10 Hz) + margin: the two antennas' latest positions can be an epoch apart
EPOCH_S = 0.12
# Baseline learning takes a pair only if the speed moves an antenna at most this between its two
# stamps (localization_filter's start pairs use the same bound) ...
LEARN_MOTION_M = 0.05
# ... and adopts the median of the last baseline_learn_pairs once 3/4 of them lie within this of it
LEARN_AGREE_M = 0.05

# accepted / reanchored / learning: a new position that formed a pair; repeat: still fresh; the
# rest: not used
VERDICTS = ('accepted', 'reanchored', 'learning', 'repeat', 'repeat_unused', 'ignored', 'invalid',
            'duplicate', 'out_of_order', 'stuck', 'older_position', 'no_pair', 'baseline',
            'heading', 'step')


@dataclass
class HygieneConfig:
    # moving: at or above this reported speed (10 Hz epochs >= 3 cm apart)
    stuck_speed_mps: float = 0.3
    # moving, a repeat of a position first stamped longer ago = stuck
    stuck_s: float = 0.3
    time_jump_s: float = 0.3            # stamp interval vs arrival interval mismatch = a time jump
    # a position this far behind the latest = an older epoch (noise ~1 cm)
    order_tol_m: float = 0.05
    antenna_baseline_m: float = 0.0     # front - rear distance; 0 = learn it
    baseline_learn_pairs: int = 20
    baseline_tol_m: float = 0.15
    heading_tol_deg: float = 5.0
    min_turn_radius_m: float = ShuttleConfig().min_turn_radius  # bounds the yaw per metre driven
    step_tol_m: float = 0.3             # midpoint across the direction of travel
    # midpoint along it (stamps are relay times: +-45 ms of motion)
    along_tol_m: float = 0.5
    # no accepted pair this long -> re-anchor on the next consistent one
    reset_s: float = 2.0
    # front and rear stamps at most this far apart (beacon_node's stale gate)
    pair_window_s: float = 0.5
    # ... and this close for a pair to be a reference (one epoch + jitter)
    sync_s: float = 0.15


def stamp_s(ts) -> Optional[float]:
    """
    Convert an Octane `updated` stamp to seconds since the epoch.

    Octane `updated` ('2026-10-08T21:28:06.080+00:00') -> seconds since the epoch, None if
    unparsable.
    """
    if not ts:
        return None
    try:
        dt = datetime.fromisoformat(ts)
    except (TypeError, ValueError):
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.timestamp()


def enu_offset(lat0, lon0, lat1, lon1):
    """
    (east, north) metres from (lat0, lon0) to (lat1, lon1).

    WGS84 radii at the mean latitude (flat over metres).
    """
    a, e2 = 6378137.0, 6.69437999014e-3
    lat = math.radians(0.5 * (lat0 + lat1))
    w = 1.0 - e2 * math.sin(lat) ** 2
    east = math.radians(lon1 - lon0) * math.cos(lat) * a / math.sqrt(w)
    north = math.radians(lat1 - lat0) * a * (1.0 - e2) / w ** 1.5
    return east, north


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class BeaconHygiene:

    def __init__(self, front_id, rear_id, cfg: Optional[HygieneConfig] = None):
        self.cfg = cfg or HygieneConfig()
        self._index = {front_id: 0, rear_id: 1}
        self._newest = [None, None]     # per antenna: (stamp, arrival) of the newest delivery
        # per antenna: arrival of the first of a run of out-of-order deliveries
        self._ooo_since = [None, None]
        # per antenna: latest new position (normalize() dict + 't': its stamp [s])
        self.new = [None, None]
        # per antenna: (stamp, lat, lon), last second moving
        self._track = [collections.deque(), collections.deque()]
        self.new_rx = [None, None]      # its arrival time [s]
        self.accepted = None            # (front, rear) of the last accepted pair
        self.accepted_rx = None
        self.baseline_m = (self.cfg.antenna_baseline_m if self.cfg.antenna_baseline_m > 0.0
                           else None)
        # learned from moving pairs: learned again at the next stop
        self.baseline_moving = False
        self._learn = []                # learning pairs: (length, moving)
        # last accepted pair: (mid stamp, mid lat, mid lon, heading [rad], speed)
        self._ref = None
        # a time jump not yet consumed by a pair: re-anchor that pair
        self._jump = False
        self.counts = collections.Counter()  # verdicts, plus 'time_jump'

    def add(self, n: dict, rx_s: float) -> str:
        """
        Check one delivery and return its verdict.

        One normalize()d delivery that arrived at rx_s [s, the caller's clock] -> its verdict
        (VERDICTS).
        """
        k = self._index.get(n.get('id'))
        if k is None:
            return 'ignored'
        if n.get('lat') is None or n.get('lon') is None:
            return self._count('invalid')
        t = stamp_s(n.get('ts'))
        t = rx_s if t is None else t
        newest = self._newest[k]
        if newest is not None:
            d_stamp, d_rx = t - newest[0], rx_s - newest[1]
            if d_stamp == 0.0:
                return self._count('duplicate')
            if d_stamp < 0.0:
                if self._ooo_since[k] is None:
                    self._ooo_since[k] = rx_s
                if rx_s - self._ooo_since[k] <= self.cfg.reset_s:
                    return self._count('out_of_order')
            if d_stamp < 0.0 or abs(d_stamp - d_rx) > self.cfg.time_jump_s:
                self._jump = True
                # not a verdict: the delivery goes on through the checks
                self.counts['time_jump'] += 1
        self._newest[k] = (t, rx_s)
        self._ooo_since[k] = None
        last, track = self.new[k], self._track[k]
        moving = abs(n.get('speed') or 0.0) >= self.cfg.stuck_speed_mps
        if moving and last is not None and (last['lat'], last['lon']) == (n['lat'], n['lon']):
            if t - last['t'] > self.cfg.stuck_s:
                return self._count('stuck')
            if self.accepted is None or self.accepted[k] is not last:
                # repeats a position the pair checks did not take
                return self._count('repeat_unused')
            last['ts'] = n.get('ts') or last['ts']
            self.new_rx[k] = rx_s
            self.accepted_rx = rx_s
            return self._count('repeat')
        if moving:
            if self._behind(track, n):
                return self._count('older_position')
        else:
            track.clear()
        self.new[k] = dict(n, t=t)
        self.new_rx[k] = rx_s
        track.append((t, n['lat'], n['lon']))
        while track[0][0] < t - 1.0:
            track.popleft()
        other = self.new[1 - k]
        if other is None or abs(t - other['t']) > self.cfg.pair_window_s:
            return self._count('no_pair')
        verdict = self._check_pair(rx_s)
        if verdict in ('baseline', 'heading', 'step'):
            # a rejected position must not be the track the next one is judged by
            track.pop()
        return self._count(verdict)

    def _behind(self, track, n) -> bool:
        """
        Tell whether a position lies behind the antenna's own track.

        n lies more than order_tol_m behind the track's latest point, along the track (needs >= 0.2
        m of it).
        """
        if len(track) < 2:
            return False
        de, dn = enu_offset(track[0][1], track[0][2], track[-1][1], track[-1][2])
        d = math.hypot(de, dn)
        if d < 0.2:
            return False
        pe, pn = enu_offset(track[-1][1], track[-1][2], n['lat'], n['lon'])
        return (pe * de + pn * dn) / d < -self.cfg.order_tol_m

    def _check_pair(self, rx_s) -> str:
        c = self.cfg
        f, r = self.new
        east, north = enu_offset(r['lat'], r['lon'], f['lat'], f['lon'])
        length = math.hypot(east, north)
        heading = math.atan2(east, north)
        # either may be the newer one
        v = max(abs(f.get('speed') or 0.0), abs(r.get('speed') or 0.0))
        sync = abs(f['t'] - r['t']) <= c.sync_s
        if self.baseline_m is None:
            if not self._learn_baseline(length, v, abs(f['t'] - r['t']), sync):
                return 'learning'
        elif self.baseline_moving and v < c.stuck_speed_mps:
            # at rest: learn it again
            self._learn_baseline(length, v, abs(f['t'] - r['t']), sync)
        if abs(length - self.baseline_m) > c.baseline_tol_m + v * (abs(f['t'] - r['t']) + EPOCH_S):
            return 'baseline'
        t_mid = 0.5 * (f['t'] + r['t'])
        lat_mid, lon_mid = 0.5 * (f['lat'] + r['lat']), 0.5 * (f['lon'] + r['lon'])
        ref = self._ref
        anchor = ref is None or self._jump or rx_s - self.accepted_rx > c.reset_s
        if anchor:
            # only a synchronous pair may (re)start the reference
            if not sync:
                return 'no_pair'
            verdict = 'accepted' if ref is None else 'reanchored'
        else:
            # distance driven since the reference
            s = 0.5 * (v + ref[4]) * max(0.0, t_mid - ref[0])
            dh = _wrap(heading - ref[3])
            if abs(dh) > math.radians(c.heading_tol_deg) + 1.5 * s / c.min_turn_radius_m:
                return 'heading'
            # chord of a constant-curvature arc: the mean heading
            hm = ref[3] + 0.5 * dh
            de, dn = enu_offset(ref[1], ref[2], lat_mid, lon_mid)
            along = de * math.sin(hm) + dn * math.cos(hm)
            across = de * math.cos(hm) - dn * math.sin(hm)
            if abs(across) > c.step_tol_m + 0.15 * s \
                    or abs(abs(along) - s) > c.along_tol_m + 0.5 * s:
                # abs(along): reversing moves the midpoint backwards
                return 'step'
            verdict = 'accepted'
        self.accepted = (f, r)
        self.accepted_rx = rx_s
        if sync:
            self._jump = False
            self._ref = (t_mid, lat_mid, lon_mid, heading, v)
        return verdict

    def _learn_baseline(self, length, v, dt, sync) -> bool:
        """
        Learn the baseline from one pair's antenna distance; True once it is (re)learned.

        Takes a synchronous pair whose stamps are dt apart if the speed v moves an antenna at most
        LEARN_MOTION_M in dt. Adopts the median of the newest baseline_learn_pairs once 3/4 of
        them lie within LEARN_AGREE_M of it.
        """
        c = self.cfg
        if not sync or v * dt > LEARN_MOTION_M:
            return False
        self._learn.append((length, v >= c.stuck_speed_mps))
        if len(self._learn) < c.baseline_learn_pairs:
            return False
        # the newest baseline_learn_pairs
        del self._learn[:-c.baseline_learn_pairs]
        lengths = sorted(x for x, _ in self._learn)
        median = lengths[len(lengths) // 2]
        if sum(abs(x - median) <= LEARN_AGREE_M for x in lengths) < 0.75 * len(lengths):
            return False
        self.baseline_m = median
        self.baseline_moving = any(moving for _, moving in self._learn)
        self._learn = []
        return True

    def _count(self, verdict: str) -> str:
        self.counts[verdict] += 1
        return verdict
