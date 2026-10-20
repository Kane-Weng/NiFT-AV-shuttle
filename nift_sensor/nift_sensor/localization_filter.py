"""
EKF that fuses both GNSS antennas, the GNSS speed and our commands into the pose.

File: localization_filter.py

Description:
  The shuttle's localization filter: an EKF that fuses the two GNSS antennas, the GNSS speed and
  our own steering / gear / brake commands into the pose of base_link in the map frame. ROS-free
  (tests:
  test/test_localization_filter.py); a node wraps it in place of shuttle_tf_node's direct transform
  when selected.

  Over the direct transform it adds:
    - the pose predicted to "now", instead of the last fix (0.1-0.2 s old)
    - short GNSS gaps bridged on the vehicle model
    - implausible positions gated out
    - a status: OK / DEGRADED / LOST

  Where things are (details in each docstring):
    LocalizationFilter  the state (rear axle x, y, yaw, v, kappa), kept as a short logbook of
                        beliefs
    _step               predict: bicycle model on an arc; the uncertainty grows
    _update_pos         correct with one antenna position (incl. the backlog after a gap)
    _update_speed       correct with the GNSS speed
    _kalman             the gate, then the EKF update
    _insert             late and out-of-order messages
    _initialise         start / restart from a consistent antenna pair
    _adopt_baseline     first start: the measured antenna distance, if it is not the configured
                        lever arms'
    status              OK / DEGRADED / LOST
    StampClock          Octane stamp -> measurement time on this computer's clock
    FilterConfig        every tuning value

Usage:
  clock = StampClock(delay_s=0.0)
  f = LocalizationFilter(FilterConfig())
  f.set_input(now, steer_deg=deg, direction=+1, decel=0.0)       # CAN steering / gear / braking,
                                                                 when they change
  t = clock.measurement_time(stamp_s, rx_s)                      # each new antenna position
  f.update_antenna(t, 'front', x, y)                             # -> 'accepted', 'rejected',
                                                                 'repeat', 'late', ...
  f.update_speed(t, speed_mps)
  pose = f.pose_at(now)                                          # base_link (x, y, yaw), v, yaw
                                                                 rate, covariance
  st = f.status(now)                                             # st.level 'OK' / 'DEGRADED' /
                                                                 'LOST', st.age_s, ...
"""

import bisect
import collections
from dataclasses import dataclass
import math
from typing import Optional

from nift_common.vehicle import ShuttleConfig
import numpy as np

SHUTTLE = ShuttleConfig()
X, Y, YAW, V, K = range(5)      # state: rear axle x, y, yaw, signed speed, curvature
ANTENNAS = {'front': 0, 'rear': 1}
# a repeated position is an old epoch only while moving (at rest it stays put)
MOVING_MPS = 0.3
# pose_at() carries the posterior forward in chunks this long (bounded cost in a gap)
COMMIT_S = 0.5


@dataclass(frozen=True)
class FilterConfig:
    # Shuttle Geometry (defaults: nift_common vehicle.yaml)
    wheelbase_m: float = SHUTTLE.wheel_base             # full lock (15 deg) = a 7.77 m radius
    # full lock: kappa is kept within 1.15 x tan(this) / wheelbase
    max_steer_deg: float = SHUTTLE.max_steering_deg
    # base_link ahead of the rear axle (mid-wheelbase)
    base_link_x_m: float = SHUTTLE.base_link_x
    front_antenna_x_m: float = SHUTTLE.gnss_front_x     # ahead of base_link
    rear_antenna_x_m: float = SHUTTLE.gnss_rear_x       # ahead of base_link (negative: behind)
    antenna_y_m: float = 0.0                            # both antennas, left of the centreline
    # where the GNSS speed is measured, ahead of base_link (the front receiver; the kinematic sim
    # reports the rear axle's: -base_link_x_m)
    speed_point_x_m: float = SHUTTLE.gnss_front_x
    # How much to trust each measurement
    # antenna position error sideways (RTK ~1 cm + slow wander)
    sigma_pos_m: float = 0.02
    # timestamp error: adds (speed x this) along the travel (45 ms rms against a local fit, which
    # hides the slow latency changes; 0.08 s gave the fewest false rejections on the shuttle bags)
    sigma_stamp_s: float = 0.08
    # a gap this long between an antenna's new positions: the next ones may be
    stale_gap_s: float = 0.25
    # ... up to the gap old, draining at this many s per s (shuttle bags)
    stale_drain: float = 0.5
    # a stale speed may be off by this x its staleness [m/s^2]
    stale_accel: float = 1.0
    # a lag within this many sigma of the usual jitter is not taken as stale
    stale_band_sigma: float = 1.0
    sigma_speed_mps: float = 0.05           # GNSS speed, driving (~4 cm/s on the shuttle)

    # How much to distrust its own prediction
    # v random walk [m/s per sqrt(s)]: throttle / brake the filter is not told
    accel_noise: float = 1.0
    # (the bags have pedal steps of ~2 m/s^2; 0.3 lost track of them)
    pos_noise: float = 0.02                 # x, y [m per sqrt(m driven)]: slip, model error
    yaw_noise_deg: float = 0.3              # yaw [deg per sqrt(m driven)]
    steer_sigma_deg: float = 0.5            # steering input -> wheel angle (map, zero offset)
    steer_tau_s: float = 0.0                # wheel lag behind the steering input; 0 = none
    # kappa random walk with a steering input [1/m per sqrt(s)]
    kappa_noise: float = 0.01
    kappa_noise_free: float = 0.1           # ... without one (full lock 0.128 1/m)

    # Gates: rejecting measurements that don't fit
    gate_pos: float = 13.8                  # chi-square, 2 dof, 99.9 %
    gate_speed: float = 10.8                # chi-square, 1 dof, 99.9 %

    # Status thresholds
    # no accepted antenna position this long -> DEGRADED (bridging; OK again
    ok_age_s: float = 0.5
    #                                         below 0.8 x this)
    bridge_s: float = 1.5                   # ... this long -> LOST
    bridge_m: float = 3.0                   # ... or driven this far since -> LOST
    # DEGRADED (settling) this long after an (re)initialisation
    settle_s: float = 0.5
    inconsistent_window_s: float = 1.0
    # more antenna positions rejected than this in the window -> DEGRADED
    inconsistent_ratio: float = 0.5
    inconsistent_min: int = 4               # ... out of at least this many

    # Starting and restarting
    sync_s: float = 0.15                    # (re)initialising pair: stamps at most this far apart
    # ... and antenna distance within this of the lever arms' (+ v x dt)
    baseline_tol_m: float = 0.15
    # first start, distance not the lever arms': adopt the one this many tight
    baseline_learn_pairs: int = 5
    # ... pairs agree on within this (0 pairs = never: refuse to start)
    baseline_learn_spread_m: float = 0.06
    # a restart: pair heading within this (or 3 sigma) of the dead-reckoned one
    reinit_yaw_deg: float = 20.0
    # ... unless the last update is this old (or that heading is unknown)
    reinit_any_s: float = 30.0

    # Housekeeping
    history_s: float = 1.0                  # posteriors kept for out-of-sequence updates
    max_step_s: float = 0.02                # predict step


@dataclass(frozen=True)
class Pose:
    t: float
    x: float                    # base_link, map frame
    y: float
    yaw: float
    v: float                    # rear-axle speed [m/s, signed]
    yaw_rate: float             # [rad/s]
    kappa: float                # [1/m]
    cov: np.ndarray             # 3x3 covariance of (x, y, yaw) of base_link
    v_var: float


@dataclass(frozen=True)
class Status:
    level: str                  # 'OK', 'DEGRADED' or 'LOST'
    # '', 'bridging', 'settling', 'inconsistent', 'uninitialised', 'outage', 'distance'
    reason: str
    age_s: float                # since the newest accepted antenna position was measured
    dist_m: float               # driven (predicted) since then
    sigma_pos_m: float          # base_link position, major axis of its covariance
    sigma_yaw_deg: float
    # not started / not restarting: why (e.g. the antenna distance vs the lever arms')
    detail: str = ''


class StampClock:
    """
    Octane `updated` stamps -> measurement times on this computer's clock.

    Stamp + clock offset - delay_s.

    rx - stamp is the transport (shuttle: p1 / p50 / p99 12 / 25 / 70 ms, late bursts up to
    seconds) plus any offset between Octane's clock and this one. The offset follows rx - stamp -
    transport_s once `window` deliveries in a row (~0.5 s) agree on it within spread_s (their
    smallest value), and is applied only beyond deadband_s (synchronised clocks: stamps used as
    they are). A held-back burst never agrees (each delivery was held a different time), so its
    deliveries keep their stamps: the filter puts them where they belong, or drops them as late. A
    stamp that maps to after its own arrival takes the arrival time instead (a clock step forward,
    until the offset follows; counted in `flagged`); after a step back, positions look late for
    those ~0.5 s. delay_s: the mean age of a position at its first delivery (relay wait + uplink +
    the unknown latency; the filter ignores the repeats, which are older): 0 for the sim's ideal
    feed, 0.061 s for its real-feed model (84 ms over every delivery). On the shuttle: that plus
    the measured latency, once measured.
    """

    def __init__(self, delay_s: float = 0.0, transport_s: float = 0.015, deadband_s: float = 0.1,
                 window: int = 15,
                 spread_s: float = 0.1):
        self.delay_s = delay_s
        self.transport_s = transport_s
        self.deadband_s = deadband_s
        self.spread_s = spread_s
        self._d = collections.deque(maxlen=window)
        self.offset_s = 0.0
        self.flagged = 0

    def measurement_time(self, stamp_s: float, rx_s: float) -> float:
        """
        Map an Octane stamp to a measurement time.

        Turn Octane's `updated` stamp into 'when the antenna measured this' on computer's clock.
        """
        self._d.append(rx_s - stamp_s - self.transport_s)

        # Check the last `window` estimates agree within `spread_s`
        if len(self._d) == self._d.maxlen and max(self._d) - min(self._d) < self.spread_s:
            low = min(self._d)
            self.offset_s = low if abs(low) > self.deadband_s else 0.0
        t = stamp_s + self.offset_s
        if t > rx_s:
            self.flagged += 1
            t = rx_s - self.transport_s
        return t - self.delay_s


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _sinc(u):
    return 1.0 - u * u / 6.0 if abs(u) < 1e-4 else math.sin(u) / u


class _Rec:
    """
    One logbook entry.

    The posterior right after one event (an update, an initialisation, or a predict-only commit).
    """

    __slots__ = ('t', 'x', 'P', 'meas', 't_pos', 'dist')

    def __init__(self, t, x, P, meas, t_pos, dist):
        self.t = t              # time of this entry
        self.x = x              # (rear axle x, y, yaw, v, κ), the belief
        self.P = P              # 5x5 uncertainty
        self.meas = meas        # message that created it (kept for rewind)
        self.t_pos = t_pos      # status age (when the newest ACCEPTED pos was measured)
        self.dist = dist        # metres driven since then


class LocalizationFilter:
    """
    Localization Filter.

    State, at the rear axle (the point a kinematic bicycle moves without side slip): x, y [m, map],
    yaw [rad, CCW from +x], v [m/s, signed: < 0 reversing], kappa [1/m, + = left]. base_link sits
    base_link_x_m ahead of it (mid-wheelbase, as in the URDF and the kinematic sim).

    Structure: keeps a logbook of its beliefs, one entry per message received (_recs, the last
    history_s). The newest entry is the current belief; pose_at() and status() predict from it to
    the time asked.
    """

    def __init__(self, cfg: Optional[FilterConfig] = None):
        # Settings
        self.cfg = c = cfg or FilterConfig()
        self._d_ant = (c.base_link_x_m + c.front_antenna_x_m, c.base_link_x_m + c.rear_antenna_x_m)
        self._d_speed = c.base_link_x_m + c.speed_point_x_m
        self._baseline = self._d_ant[0] - self._d_ant[1]
        self.baseline_configured = self._baseline   # front - rear lever arm as configured
        # adopted at the first start if the pairs disagree (_adopt_baseline)
        self.baseline_measured = None
        self._k_max = 1.15 * math.tan(math.radians(c.max_steer_deg)) / c.wheelbase_m

        # Logbook: list of _Rec, time order, store the last history_s
        self._recs = []

        # Input Timeline: when steering/gear/brake cmd changed, and to what
        self._in_t, self._in = [], []

        # Per-antenna notes: [front, rear]
        self._pair = [None, None]       # (t, x, y): the latest position while (re)initialising
        self._last_xy = [None, None]    # the previous position (repeats)
        self._t_new = [None, None]      # measurement time of its previous new position
        self._stale = [(0.0, 0.0), (0.0, 0.0)]  # (since, bound [s]) how old its positions may be

        self._speed = None              # (t, unsigned speed): the latest, for initialising v
        # tight start pairs' distances
        self._lengths = collections.deque(maxlen=max(1, c.baseline_learn_pairs))
        self._init_note = ''            # why it has not (re)started yet, for the status

        # Status Memory
        self._lost = True               # LOST holds until a pair re-initialises
        # the last status() said OK (DEGRADED -> OK below 0.8 x ok_age_s)
        self._was_ok = False
        self._t_init = None
        # (t, accepted) of antenna positions, for 'inconsistent'
        self._checks = collections.deque()

        # Diagnostics
        self.counts = collections.Counter()

    # ------------------------------------------------------------------ #
    #  Inputs and measurements                                           #
    # ------------------------------------------------------------------ #
    def set_input(self, t: float, steer_deg: Optional[float] = None, direction: int = 0,
                  decel: float = 0.0):
        """
        Set the steering, gear and braking inputs from time t on.

        From t on: front-wheel angle [deg, + = left; None = unknown], gear direction (+1 forward,
        -1 reverse, 0 unknown), braking deceleration [m/s^2, >= 0, opposes the motion]. Held until
        the next call.
        """
        i = bisect.bisect_right(self._in_t, t)
        self._in_t.insert(i, t)
        self._in.insert(i, (steer_deg, int(direction), max(0.0, float(decel))))
        keep = self._recs[0].t if self._recs else t - self.cfg.history_s
        j = bisect.bisect_right(self._in_t, keep) - 1       # the input active at `keep` stays
        if j > 0:
            del self._in_t[:j], self._in[:j]

    def update_antenna(self, t: float, antenna: str, x: float, y: float,
                       paired: bool = True) -> str:
        """
        Take one new antenna position and return the verdict.

        One new position of antenna 'front' / 'rear' [m, map], measured at t (StampClock time) ->
        verdict.

        The same position as that antenna's previous one while moving is a repeat (an old epoch
        under a fresh stamp): ignored. Not tracking (LOST, or never started): it goes to
        _initialise. paired=False (no partner to check it against, gnss_hygiene's no_pair): it
        counts only while the filter is not bridging, so one antenna can carry a confident filter
        through the other's dropout, but cannot revive a bridging one (a swapped or broken antenna
        would).
        """
        k = ANTENNAS[antenna]
        prev, self._last_xy[k] = self._last_xy[k], (x, y)
        if prev == (x, y) and self._recs and not self._lost \
                and abs(self._recs[-1].x[V]) >= MOVING_MPS:
            return self._count('repeat')
        # after a gap: possibly up to this much older than its stamp
        stale = self._staleness(k, t)
        if not paired and (not self._tracking(t) or t - self._recs[-1].t_pos > self.cfg.ok_age_s):
            return self._count('unpaired_ignored')
        if not self._tracking(t):
            return self._count(self._initialise(t, k, x, y))
        return self._count(self._insert(t, ('pos', k, x, y, stale), first=True))

    def update_speed(self, t: float, speed: float) -> str:
        """GNSS ground speed [m/s, unsigned] measured at t -> verdict."""
        self._speed = (t, abs(speed))
        if self._lost or not self._recs:
            return self._count('speed_waiting')
        # the front receiver's: as stale as its positions after a gap
        since, bound = self._stale[0]
        bound = max(0.0, bound - self.cfg.stale_drain * max(0.0, t - since))
        return self._count(self._insert(t, ('speed', abs(speed), bound), first=True))

    # ------------------------------------------------------------------ #
    #  Outputs                                                           #
    # ------------------------------------------------------------------ #
    def pose_at(self, t: float) -> Optional[Pose]:
        """
        Return the pose of base_link at time t.

        base_link at time t (local clock), predicted from the newest update; None before the first.
        """
        if not self._recs:
            return None
        x, P, _ = self._at(t)
        return self._pose(t, x, P)

    def status(self, t: float) -> Status:
        """
        Return the filter status at time t.

        OK / DEGRADED / LOST, from the age of the newest accepted antenna position (its measurement
        time).

          OK        one within ok_age_s (back from DEGRADED only below 0.8 x ok_age_s: no flicker)
          DEGRADED  bridging (none for ok_age_s: the model carries the pose), inconsistent (most
                    positions of the last inconsistent_window_s failed the gate; none is applied,
                    see _apply), or settling (settle_s after a (re)start)
          LOST      never started, or bridged longer than bridge_s or further than bridge_m. The
                    pose is still predicted (for a controlled stop) but must not be driven on.
        Calling it latches LOST: it holds until _initialise restarts the filter from a consistent
        pair.
        """
        c = self.cfg

        # Never started
        if not self._recs:
            return Status('LOST', 'uninitialised', math.inf, math.inf, math.inf, math.inf,
                          self._init_note or 'no front / rear pair yet')

        x, P, dist = self._at(t)
        age = t - self._recs[-1].t_pos
        pose = self._pose(t, x, P)
        sp = math.sqrt(max(np.linalg.eigvalsh(pose.cov[:2, :2])))
        sy = math.degrees(math.sqrt(pose.cov[2, 2]))

        # Already LOST, or long age, or long driven dist without GNSS
        if self._lost or age > c.bridge_s or dist > c.bridge_m:
            self._lost = True
            self._was_ok = False
            reason = 'outage' if age > c.bridge_s else 'distance'
            return Status('LOST', reason, age, dist, sp, sy, self._init_note)

        # Degrade due to age / inconsistency / settling
        # hysteresis: no flicker at the threshold
        if age > (c.ok_age_s if self._was_ok else 0.8 * c.ok_age_s):
            level, reason = 'DEGRADED', 'inconsistent' if self._inconsistent(t) else 'bridging'
        elif self._inconsistent(t):
            level, reason = 'DEGRADED', 'inconsistent'
        elif t - self._t_init < c.settle_s:
            level, reason = 'DEGRADED', 'settling'

        # Normal ok
        else:
            level, reason = 'OK', ''
        self._was_ok = level == 'OK'
        return Status(level, reason, age, dist, sp, sy)

    def _inconsistent(self, t) -> bool:
        """
        Tell whether too many recent antenna positions failed the gate.

        More than inconsistent_ratio of the antenna positions of the last inconsistent_window_s
        failed the gate.
        """
        c = self.cfg
        recent = [ok for tc, ok in self._checks if t - c.inconsistent_window_s < tc <= t]
        return len(recent) >= c.inconsistent_min \
            and recent.count(False) > c.inconsistent_ratio * len(recent)

    @property
    def baseline(self) -> float:
        """
        Return the antenna distance in use.

        Front - rear antenna distance in use [m]: the configured lever arms', or the measured one.
        """
        return self._baseline

    @property
    def rear_antenna_x(self) -> float:
        """Rear lever arm in use [m ahead of base_link]."""
        return self._d_ant[1] - self.cfg.base_link_x_m

    def _pose(self, t, x, P) -> Pose:
        c, b = math.cos(x[YAW]), math.sin(x[YAW])
        d = self.cfg.base_link_x_m
        J = np.array([[1.0, 0.0, -d * b, 0.0, 0.0], [0.0, 1.0, d * c, 0.0, 0.0], [0.0, 0.0, 1.0,
                                                                                  0.0, 0.0]])
        return Pose(t, x[X] + d * c, x[Y] + d * b, x[YAW], x[V], x[V] * x[K], x[K], J @ P @ J.T,
                    P[V, V])

    # ------------------------------------------------------------------ #
    #  (Re)initialisation                                                #
    # ------------------------------------------------------------------ #
    def _staleness(self, k, t) -> float:
        """
        How old (beyond delay_s) antenna k's position at t may be.

        After a gap the feed delivers the backlog under fresh stamps (shuttle bags: the first
        position after a > 0.5 s gap median 0.45 s / p90 0.95 s old, back to normal within ~1 s),
        so the bound is the gap, draining at stale_drain.
        """
        c = self.cfg
        since, bound = self._stale[k]
        bound = max(0.0, bound - c.stale_drain * max(0.0, t - since))
        prev, self._t_new[k] = self._t_new[k], t
        if prev is not None and t - prev > c.stale_gap_s:
            bound = max(bound, t - prev)
        self._stale[k] = (t, bound)
        return bound

    def _tracking(self, t) -> bool:
        """Decide whether allow to bridge or not."""
        if self._lost or not self._recs:
            return False
        r = self._recs[-1]
        if t - r.t_pos > self.cfg.bridge_s \
                or r.dist + abs(r.x[V]) * max(0.0, t - r.t) > self.cfg.bridge_m:
            self._lost = True
            return False
        return True

    def _initialise(self, t, k, x, y) -> str:
        """
        Start or restart from a front / rear pair.

        Stamps within sync_s, antenna distance within baseline_tol_m of the lever arms' (+ speed x
        their time apart). The first start is stricter, baseline_learn_spread_m: a pair further off
        goes to _adopt_baseline, which measures the distance and may adopt it. Start at the older
        position, then update with the newer one.

        A restart within reinit_any_s must also agree with the dead-reckoned heading
        (reinit_yaw_deg, or 3 sigma): a swapped pair has the right length but would restart it
        backwards. Otherwise any consistent pair restarts it, so a long outage or a persistent
        offset ends in a stop and a restart, never in a lock-out (GNSS alone cannot tell which of
        two persistent states is the true one).
        """
        c = self.cfg
        self._pair[k] = (t, x, y)
        other = self._pair[1 - k]
        if other is None or abs(t - other[0]) > c.sync_s:
            self._init_note = self._init_note or 'waiting for a synchronous front / rear pair'
            return 'init_waiting'
        (tf, xf, yf), (tr, xr, yr) = self._pair
        # the latest GNSS speed (after a gap it is from before it: its uncertainty grows with its
        # age)
        spd_age = abs(t - self._speed[0]) if self._speed is not None else math.inf
        spd = self._speed[1] if spd_age < 5.0 else None
        length = math.hypot(xf - xr, yf - yr)
        # what the speed can move one antenna between the two stamps
        motion = (spd or 0.0) * abs(tf - tr)
        # first start
        learn = not self._recs and self.baseline_measured is None and c.baseline_learn_pairs > 0
        tol = (c.baseline_learn_spread_m if learn else c.baseline_tol_m) + motion
        if abs(length - self._baseline) > tol and not self._adopt_baseline(length, motion):
            return 'init_bad_pair'
        yaw = math.atan2(yf - yr, xf - xr)      # both antennas on one line along the centreline
        xp = Pp = None
        if self._recs:                              # a restart: the dead-reckoned state
            xp, Pp, _ = self._at(t)
        if xp is not None and t - self._recs[-1].t_pos < c.reinit_any_s:
            # must agree with the dead-reckoned heading while it is known: a
            sy = math.sqrt(Pp[YAW, YAW])
            if sy < math.radians(30.0) \
                    and abs(_wrap(yaw - xp[YAW])) > max(math.radians(c.reinit_yaw_deg), 3.0 * sy):
                self._init_note = (f'restart refused: pair heading {math.degrees(yaw):.0f} deg '
                                   f'vs dead-reckoned '
                                   f'{math.degrees(xp[YAW]):.0f} deg (swapped antennas?)')
                # swapped pair has the right length but a flipped heading
                return 'init_bad_yaw'
        old, new = (0, 1) if tf <= tr else (1, 0)
        t0, x0, y0 = self._pair[old]
        d, e = self._d_ant[old], c.antenna_y_m
        steer, direction, _ = self._input_at(t0)
        sp = 0.05 + (spd or 0.0) * (abs(tf - tr) + 0.5 * max(self._stale[0][1], self._stale[1][1]))
        # a restart without a fresh speed: the dead-reckoned one (it has
        if xp is not None and spd_age > 0.5:
            # braked to 0 through a stop), not the last measured one
            v0, sv2 = xp[V], min(Pp[V, V], 9.0) + 0.1 ** 2
        elif spd is None:
            v0, sv2 = 0.0, 3.0 ** 2
        elif direction:
            v0, sv2 = direction * spd, 0.1 ** 2 + c.accel_noise ** 2 * spd_age
        else:                                   # gear unknown: either way, the positions will tell
            v0, sv2 = 0.0, (spd + 0.1) ** 2
        xs = np.array([x0 - d * math.cos(yaw) + e * math.sin(yaw),
                       y0 - d * math.sin(yaw) - e * math.cos(yaw), yaw,
                       v0, math.tan(math.radians(steer)) / c.wheelbase_m if steer is not None
                       else 0.0])
        P = np.diag([sp ** 2, sp ** 2, math.radians(2.0) ** 2, sv2,
                     self._sigma_kappa_in(steer) ** 2 if steer is not None
                     else SHUTTLE.max_curvature ** 2])
        self._recs = [_Rec(t0, xs, P, None, t0, 0.0)]
        self._lost = False
        self._t_init = t0
        self._pair = [None, None]
        self._checks.clear()
        self._init_note = ''
        self.counts['init'] += 1
        tn, xn, yn = (tf, xf, yf) if new == 0 else (tr, xr, yr)
        self._insert(tn, ('pos', new, xn, yn, self._stale[new][1]), first=True)
        return 'initialised'

    def _adopt_baseline(self, length, motion) -> bool:
        """
        Learn the antenna distance when the start pair disagrees with the lever arms.

        The start pair's antenna distance is not the lever arms' (front - rear).

        At the first start only, measure it: once baseline_learn_pairs tight pairs (the speed moves
        an antenna <= 0.05 m between their stamps) agree on one within baseline_learn_spread_m,
        start (True). If that median is further than baseline_learn_spread_m from the lever arms',
        adopt it: the rear lever arm becomes front - measured. The front one stays: it defines
        base_link, as in shuttle_tf_node, so a wrong front lever arm only shifts the pose along the
        heading, as in direct mode. A restart never re-learns (the geometry cannot change during a
        run: a new distance is a fault). False: not (yet), with the reason in the status
        (_init_note).
        """
        c = self.cfg
        if self._recs or c.baseline_learn_pairs <= 0:
            kind = 'measured' if self.baseline_measured is not None else 'configured'
            self._init_note = f'antenna distance {length:.2f} m vs {kind} {self._baseline:.2f} m'
            return False
        if motion <= 0.05:
            self._lengths.append(length)
        n = len(self._lengths)
        spread = max(self._lengths) - min(self._lengths) if n else 0.0
        median = sorted(self._lengths)[n // 2] if n else length
        if n < c.baseline_learn_pairs or spread > c.baseline_learn_spread_m \
                or abs(length - median) > c.baseline_tol_m + motion:
            self._init_note = (f'antenna distance {length:.2f} m '
                               f'vs configured {self.baseline_configured:.2f} m '
                               f'(lever arms {c.front_antenna_x_m:+.3f} / '
                               f'{c.rear_antenna_x_m:+.3f}): learning it, '
                               f'{n}/{c.baseline_learn_pairs} pairs'
                               + (f', spread {spread:.2f} m' if n > 1 else ''))
            return False
        if abs(median - self.baseline_configured) > c.baseline_learn_spread_m:
            self.baseline_measured = self._baseline = median
            self._d_ant = (self._d_ant[0], self._d_ant[0] - median)
            self.counts['baseline_adopted'] += 1
        return True

    # ------------------------------------------------------------------ #
    #  Filter core                                                       #
    # ------------------------------------------------------------------ #
    def _insert(self, t, meas, first) -> str:
        """Decide where in the logbook a message belongs, then calls _apply."""
        recs = self._recs

        # Case a: older than the whole logbook
        if t < recs[0].t:
            return 'late'

        # Case b: newest
        if t >= recs[-1].t:
            verdict = self._apply(t, meas, first)

        # Case c: in between, rewind
        else:
            i = bisect.bisect_right([r.t for r in recs], t) - 1
            redo = [(r.t, r.meas) for r in recs[i + 1:] if r.meas is not None]
            del recs[i + 1:]
            verdict = self._apply(t, meas, first)
            for tr, mr in redo:
                self._apply(tr, mr, first=False)
            self.counts['rewind'] += 1
        newest = recs[-1].t
        while len(recs) > 1 and recs[1].t <= newest - self.cfg.history_s:
            recs.pop(0)
        return verdict

    def _apply(self, t, meas, first) -> str:
        # Predict from the last logbook entry to the msg's time
        r = self._recs[-1]
        x, P, dist = self._predict(r.x.copy(), r.P.copy(), r.t, t, r.dist)
        t_pos = r.t_pos

        # Correct with the message
        if meas[0] == 'pos':
            # most positions failing the gate: apply none (bridge on the model)
            trust = not self._inconsistent(t)
            ok, older = self._update_pos(x, P, meas[1], meas[2], meas[3], meas[4], apply=trust)
            # the age counts from when the newest one was measured
            if ok and trust and t - older > t_pos:
                t_pos, dist = t - older, abs(x[V]) * older
            if first:
                self._checks.append((t, ok))
                while self._checks and self._checks[0][0] < t - self.cfg.inconsistent_window_s:
                    self._checks.popleft()
            verdict = ('accepted' if trust else 'distrusted') if ok else 'rejected'
        else:
            ok = self._update_speed(x, P, t, meas[1], meas[2])
            verdict = 'speed' if ok else 'speed_rejected'

        # Write a new logbook entry
        self._recs.append(_Rec(t, x, P, meas, t_pos, dist))
        return verdict

    def _at(self, t):
        """
        Predict the state to time t.

        (x, P, dist since the last accepted position) predicted to t; commits in COMMIT_S chunks in
        a gap.
        """
        r = self._recs[-1]
        # predict-only posterior: dropped again by a rewind
        while t - r.t > COMMIT_S:
            tc = r.t + COMMIT_S
            x, P, dist = self._predict(r.x.copy(), r.P.copy(), r.t, tc, r.dist)
            r = _Rec(tc, x, P, None, r.t_pos, dist)
            self._recs.append(r)
            while len(self._recs) > 1 and self._recs[1].t <= tc - self.cfg.history_s:
                self._recs.pop(0)
        return self._predict(r.x.copy(), r.P.copy(), r.t, t, r.dist)

    def _input_at(self, t):
        i = bisect.bisect_right(self._in_t, t) - 1
        return self._in[i] if i >= 0 else (None, 0, 0.0)

    def _predict(self, x, P, t0, t1, dist):
        # Special case: a little backwards (stamp jitter); constant motion, no noise
        if t1 < t0:
            steer, _, _ = self._input_at(t1)
            return self._step(x, P, t1 - t0, steer, 0.0, noise=False) + (dist,)

        # Cut the interval at every input change
        i = bisect.bisect_right(self._in_t, t0)
        cuts = [t0] + [tc for tc in self._in_t[i:] if tc < t1] + [t1]
        for a, b in zip(cuts[:-1], cuts[1:]):
            steer, _, decel = self._input_at(a)
            # Split each piece into steps, and call action model on each
            n = max(1, math.ceil((b - a) / self.cfg.max_step_s - 1e-9))
            dt = (b - a) / n
            for _ in range(n):
                v0 = x[V]
                x, P = self._step(x, P, dt, steer, decel)
                dist += 0.5 * abs(v0 + x[V]) * dt   # add up distance driven
        return x, P, dist

    @staticmethod
    def _arc(x, ds):
        """
        Move the rear axle pose along the current curvature.

        (x, y, yaw) of the rear axle after ds [m, signed] along the current curvature (no noise, no
        state change).
        """
        dpsi = x[K] * ds
        chord = ds * _sinc(0.5 * dpsi)
        return (x[X] + chord * math.cos(x[YAW] + 0.5 * dpsi),
                x[Y] + chord * math.sin(x[YAW] + 0.5 * dpsi),
                x[YAW] + dpsi)

    def _sigma_kappa_in(self, steer):
        c = self.cfg
        return math.radians(c.steer_sigma_deg) \
            / (c.wheelbase_m * math.cos(math.radians(steer)) ** 2)

    def _step(self, x, P, dt, steer, decel, noise=True):
        """
        Predict the state and its uncertainty dt later.

        Action model: given current state and inputs, find the state dt later & uncertainty.

        Exact arc at v and kappa. kappa follows the steering input, tan(steer) / wheelbase, with
        the lag steer_tau_s (0 = at once); without a steering input it is a random walk (in a gap
        the turn goes on). v is constant unless braking (it then stops at 0, never reverses).
        Process noise per metre driven on x, y and yaw (none added at rest), per second on v.
        """
        c = self.cfg
        adt = abs(dt)

        # Curvature: towards the steering input, or a random walk
        if steer is not None:
            k_in = math.tan(math.radians(steer)) / c.wheelbase_m
            a = math.exp(-adt / c.steer_tau_s) if c.steer_tau_s > 0.0 else 0.0
            x[K] = k_in + (x[K] - k_in) * a
            if a != 1.0:
                P[K, :] *= a
                P[:, K] *= a
            if noise:
                P[K,
                  K] += (1.0 - a * a) * self._sigma_kappa_in(steer) ** 2 + c.kappa_noise ** 2 * adt
        elif noise:
            P[K, K] += c.kappa_noise_free ** 2 * adt    # no command: grow doubt fast
        self._bound_kappa(x, P)

        # Speed
        v, kap, yaw = x[V], x[K], x[YAW]
        v1 = v          # constant by default
        if decel > 0.0 and v != 0.0 and dt > 0.0:
            dv = decel * dt
            if dv >= abs(v):
                v1 = 0.0
            else:
                v1 = v - math.copysign(dv, v)

        # Move along the arc
        ds = 0.5 * (v + v1) * dt
        dpsi = kap * ds                 # heading change = curvature * distance
        chord = ds * _sinc(0.5 * dpsi)  # straight line distance start -> end of the arc
        cs, sn = math.cos(yaw + 0.5 * dpsi), math.sin(yaw + 0.5 * dpsi)
        x[X] += chord * cs
        x[Y] += chord * sn
        x[YAW] = _wrap(yaw + dpsi)
        x[V] = v1

        # Uncertainty: carry existing doubt forward + add new doubt
        F = np.eye(5)
        F[X, YAW], F[Y, YAW] = -chord * sn, chord * cs
        F[X, V] = dt * cs - chord * sn * 0.5 * kap * dt
        F[Y, V] = dt * sn + chord * cs * 0.5 * kap * dt
        F[X, K], F[Y, K] = -chord * sn * 0.5 * ds, chord * cs * 0.5 * ds
        F[YAW, V], F[YAW, K] = kap * dt, ds
        # (a braking stop clamps the mean only: its uncertainty stays, so a wrong speed
        P = F @ P @ F.T
        # estimate, e.g. right after a restart, can still be corrected by the positions)
        if noise:
            P[X, X] += c.pos_noise ** 2 * abs(ds)
            P[Y, Y] += c.pos_noise ** 2 * abs(ds)
            P[YAW, YAW] += math.radians(c.yaw_noise_deg) ** 2 * abs(ds)
            P[V, V] += c.accel_noise ** 2 * adt
        return x, P

    def _update_pos(self, x, P, k, zx, zy, stale=0.0, apply=True):
        """
        Correct the state with one antenna position.

        Correct with one antenna position -> (passed the gate, how much older than its time it
        looked [s]).

        The antenna's lever arm is in the model, so each antenna updates on its own and the two
        together give the yaw. Noise: sigma_pos_m across the antenna's travel; along it also speed
        x sigma_stamp_s (stamp jitter). stale > 0: after a gap the feed may deliver its backlog
        under fresh stamps (see _staleness):
          a  lag within the usual jitter: more timing noise, both ways (fresh data stays unbiased)
          b  lagging further, explained by `stale`: re-timed onto the predicted path; it corrects
             only across the travel (and so the heading), and the status age counts from when it
             looks measured
          c  ahead of the prediction, or lagging beyond `stale`: a normal update (beyond the bound,
             only the rest counts as an error)
        """
        c = self.cfg

        # Antenna read (h) and its Jacobian (H)
        d, e = self._d_ant[k], c.antenna_y_m
        cs, sn = math.cos(x[YAW]), math.sin(x[YAW])
        h = np.array([x[X] + d * cs - e * sn, x[Y] + d * sn + e * cs])
        H = np.zeros((2, 5))
        H[0, X] = H[1, Y] = 1.0
        H[0, YAW], H[1, YAW] = -d * sn - e * cs, d * cs - e * sn

        # Noise (trust factor as an ellipse)
        kd = x[K] * d
        # instantaneous heading of antenna's velocity
        beta = x[YAW] + math.atan(kd)
        u = abs(x[V]) * math.sqrt(1.0 + kd * kd)                # magnitude of antenna's velocity
        sa2 = c.sigma_pos_m ** 2 + (u * c.sigma_stamp_s) ** 2   # longitudinal variance
        sc2 = c.sigma_pos_m ** 2                                # lateral variance
        cb, sb = math.cos(beta), math.sin(beta)
        z = np.array([zx, zy])
        nu = z - h      # measured - expected: the innovation (surprise)

        # Counter the backlog issue
        older = 0.0
        # the direction of travel along beta (reversing: backwards)
        fwd = 1.0 if x[V] >= 0.0 else -1.0
        lag = -fwd * (nu[0] * cb + nu[1] * sb)  # how far the fix is behind the prediction

        # Case a: within normal jitter - widens the longitudinal noise
        if stale > 0.0 and u >= MOVING_MPS and abs(lag) <= c.stale_band_sigma * math.sqrt(sa2):
            sa2 += (u * 0.5 * stale) ** 2

        # Case b: behind by more than the jitter - re-time it
        elif stale > 0.0 and u >= MOVING_MPS and lag > 0.0:
            a = 0.0
            # walk back along the predicted arc to find how many seconds earlier the fix fits
            for _ in range(3):
                xb = self._arc(x, -x[V] * a)
                cs, sn = math.cos(xb[YAW]), math.sin(xb[YAW])
                beta = xb[YAW] + math.atan(kd)
                cb, sb = math.cos(beta), math.sin(beta)
                nu = z - np.array([xb[X] + d * cs - e * sn, xb[Y] + d * sn + e * cs])
                a = min(stale, max(0.0, a - fwd * (nu[0] * cb + nu[1] * sb) / u))
            older = a
            # explained by the backlog: it corrects across the travel only
            if a < stale:
                ec = np.array([-sb, cb])        # (gate_speed: the 1-dof chi-square)
                return self._kalman(x, P, np.array([nu @ ec]), (ec @ H)[None, :],
                                    np.array([[sc2]]), c.gate_speed,
                                    apply), a

        # Case c: ahead of the prediction, or bound = 0 - just normal update
        Rr = np.array([[cb, -sb], [sb, cb]])
        R = Rr @ np.diag([sa2, sc2]) @ Rr.T
        return self._kalman(x, P, nu, H, R, c.gate_pos, apply), older

    def _update_speed(self, x, P, t, speed, stale=0.0) -> bool:
        """
        Correct the state with the GNSS speed.

        Correct with the GNSS speed, measured at speed_point_x_m (in a turn faster than the rear
        axle) -> passed.

        Signed by the gear input. Gear unknown (0, e.g. driven by hand): compared as |v|, and
        skipped while starting off until the positions have shown the direction. After a gap:
        stale_accel x staleness more noise (a filter sure of an old speed falls behind, then gates
        out the caught-up positions).
        """
        c = self.cfg
        _, direction, _ = self._input_at(t)
        if direction:                           # signed by the gear
            z, hv, sgn = direction * speed, x[V], 1.0
        # gear unknown: |v|; the positions give the sign, so wait for them
        else:
            # starting off: either way until they have
            if abs(x[V]) < MOVING_MPS and speed > 0.05:
                return False
            z, hv, sgn = speed, abs(x[V]), (1.0 if x[V] >= 0.0 else -1.0)
        ks = x[K] * self._d_speed
        g = math.sqrt(1.0 + ks * ks)
        H = np.zeros((1, 5))
        H[0, V] = sgn * g
        H[0, K] = hv * x[K] * self._d_speed ** 2 / g
        nu = np.array([z - hv * g])
        # after a gap the speed is old too
        r = c.sigma_speed_mps ** 2 + (c.stale_accel * stale) ** 2
        return self._kalman(x, P, nu, H, np.array([[r]]), c.gate_speed)

    def _bound_kappa(self, x, P):
        """
        Keep kappa within full lock.

        The steering cannot exceed full lock.

        Kappa and its spread stay within it (without a steering input, at crawling speed, kappa is
        barely observable and would otherwise drift past it).
        """
        x[K] = min(max(x[K], -self._k_max), self._k_max)
        if P[K, K] > self._k_max ** 2:
            f = self._k_max / math.sqrt(P[K, K])
            P[K, :] *= f
            P[:, K] *= f

    def _kalman(self, x, P, nu, H, R, gate, apply=True) -> bool:
        """
        Apply one gated Kalman update.

        Gate, then (if apply) update x, P in place; returns whether the measurement passed the
        gate.

        The gate is a Mahalanobis test on the innovation (gate_pos / gate_speed: chi-square 99.9
        %); a rejected measurement changes nothing. Unlike gnss_hygiene's fixed, model-free
        tolerances upstream, it uses the prediction: tight (a few cm) while updates come, wider as
        a gap goes on.
        """
        S = H @ P @ H.T + R                 # Total uncertainty: existing doubt + sensor noise
        Si = np.linalg.inv(S)
        # check if surprise measured in normal sizes more than gate
        if float(nu @ Si @ nu) > gate:
            return False                    # rejected, nothing changes
        if not apply:
            return True
        # gain: how much of the surprise goes into each of the 5 numbers
        G = P @ H.T @ Si
        x += G @ nu                         # nudge the state
        x[YAW] = _wrap(x[YAW])
        A = np.eye(5) - G @ H
        P[:] = A @ P @ A.T + G @ R @ G.T    # doubt shrink (Joseph form)
        self._bound_kappa(x, P)
        return True

    def _count(self, verdict: str) -> str:
        self.counts[verdict] += 1
        return verdict
