"""
Simulated GNSS of both sim backends: pose to antenna fixes, noise, dropout and faults.

File: sim_gnss.py
Author: Kane Weng

Description:
  The simulated GNSS of both sim backends (carla_bridge_node.py and the
  kinematic sim): turns the vehicle's pose into the two antenna fixes Octane
  would report, as the beacon_update payloads the Octane emulator
  (octane_emulator.py) emits. Moved out of the CARLA bridge unchanged, so both
  backends share one GNSS model. The only backend-specific piece is to_wgs, the
  projection of the frame the pose is given in (CarlaGeo.carla_to_wgs for the
  CARLA world, CarlaGeo.map_to_wgs for the map frame); the rest is planar
  metres.

    antennas  front / rear at +-antenna_offset_x along the forward vector from
              base_link (the backends' default: gnss_front_x of nift_common vehicle.yaml)
    noise     noise_stddev_m: one Gaussian offset per fix, shared by both antennas
              (the heading stays exact); noise_heading adds an independent one per
              antenna on top (corrupts the heading)
    dropout   in every dropout_period_s of sim time, the first dropout_duration_s
              send nothing, as when Octane goes quiet; `ok` is False meanwhile
    dead zone dead_zones: (lat_min, lat_max, lon_min, lon_max) boxes, e.g. Wi-Fi
              dead spots: nothing is sent while base_link is inside one; `ok` is
              False meanwhile
    heading   compass degrees of the rear -> front baseline (true north), as
              Octane's dynamics.heading; beacon_node computes its own
    speed     ground speed as given (like an RTK receiver); beacon_payload() sends
              it in knots

  Without a FeedConfig, every step() sends both antennas' fix of that instant,
  stamped with the sim time (an ideal feed). With one, step() plays the real
  Octane feed as measured on the shuttle (2026-10-08 garage bags; FeedConfig's
  defaults are that fit):
    receiver  a new position every 1 / receiver_hz (10 Hz), both antennas
              sampled together, with cm-level noise (white per fix and per
              antenna, plus a slow common wander) and speed noise
    relay     Octane relays each antenna's latest position at jittered intervals
              (61.6 +- 16.9 ms), so ~40 % of the deliveries repeat a position,
              and stamps `updated` with the relay time, not the measurement time
    latency   each position reaches Octane latency_s (unknown on the shuttle: 0)
              plus an exponential uplink delay (mean uplink_mean_s) after it
              was measured; the relay sends the newest one that has arrived
              (except while it catches up on a backlog, below)
    gaps      per antenna, deliveries stop for a lognormal time at
              gap_rate_moving / gap_rate_still onsets per second (median
              gap_median_moving_s / gap_median_still_s); gap_shared_fraction of
              the onsets stop both antennas at once; during bad-link spells
              (burst_*) the onset rate is higher, so the gaps cluster
    backlog   after hold_fraction of the gaps Octane delivers the positions the
              gap held, oldest first, under fresh stamps: the first one after
              the gap is the next epoch, about the gap old. It then sends the
              next epoch every backlog_interval_s (faster than real time) until
              it has caught up with the newest. Positions older than
              backlog_max_s are dropped; the other gaps drop what they
              swallowed (the newest position comes next). After the dropout
              schedule, a dead zone or a fault dropout nothing is held
    order     out_of_order_per_h deliveries go out with an older stamp and
              position than the delivery before
  The random gaps leave `ok` True: it only reports the dropout schedule and
  the dead zones.

  Faults (GnssFault, the fault-injection bench): scheduled faults on top of
  either feed; without a FeedConfig they run through EXACT_FEED (every 20 Hz
  tick, exact, no random gaps), so a test sees only its fault. Trigger: a sim
  time (t_s), the distance driven (at_m) or base_link entering a lat/lon box;
  then active for duration_s (0: one epoch for a jump, while inside for a box).
    dropout  no deliveries from the antenna(s)
    jump     fixes offset by (east_m, north_m)
    noise    fixes scattered by sigma_m per antenna (an RTK float episode)
    stuck    the whole feed repeats the epoch it froze on, with fresh stamps
    freeze   the antenna's position stops moving, stamps stay fresh
    swap     front and rear positions swapped (heading off by 180 deg)
    delay    deliveries carry positions delay_s older than usual
    burst    deliveries held back, then all sent when the fault ends
    clock    stamps off by offset_s
  fault_events logs (sim time, fault index, 'start' / 'end', kind); the backends
  publish them on /carla/gnss_fault (new_fault_events()). Scenarios: named lists
  of faults in config/gnss_faults.yaml (resolve_fault_scenario(); launch arg
  gnss_faults:=<name or file>), handed to the backends as one YAML string
  parameter, gps.faults (faults_from_param()).

  Backend parameters (both backends, carla_params.yaml): gps.feed.enabled turns
  the real feed on, gps.feed.<field> overrides a FeedConfig default,
  gps.dead_zones is a flat [lat_min, lat_max, lon_min, lon_max, ...] list.
  feed_param_defaults() / feed_from_params() / dead_zones_from_flat() map them.

  No carla or ROS node code in here (test/test_sim_gnss.py).

Usage:
  gnss = GnssSim(antenna_offset_x=1.041, ids=('SIM-FRONT', 'SIM-REAR'))
  gnss = GnssSim(1.041, ids, feed=FeedConfig(), seed=1)          # the real feed
  gnss = GnssSim(1.041, ids, feed=feed_from_params(get),
                 dead_zones=dead_zones_from_flat(get('gps.dead_zones')))
  payloads = gnss.step(sim_ns, x, y, fx, fy, speed, elevation, geo.map_to_wgs)
  if payloads:                          # [] during a dropout
      octane.publish(payloads)
  gnss.ok                               # False while a dropout is active
"""

import collections
from dataclasses import dataclass, fields
import math
import random
from typing import Callable, List, Optional, Sequence, Tuple

from nift_carla.octane_emulator import beacon_payload
import yaml

# ground speed above which the moving gap rate applies (as in the fit)
MOVING_SPEED_THRESHOLD_MPS = 0.3


@dataclass(frozen=True)
class FeedConfig:
    """The real Octane feed; defaults fitted to the 2026-10-08 garage bags (front, 57 min)."""

    receiver_hz: float = 10.0           # real positions: spacing / speed clusters at 0.10 s
    interval_s: float = 0.0616          # relay: mean delivery interval per antenna
    jitter_s: float = 0.0169            # relay: its standard deviation (clipped to [0.01, 0.15] s)
    latency_s: float = 0.0              # measurement -> Octane, fixed part; unknown on the shuttle
    # plus an exponential part (beacon -> Octane over Wi-Fi); with the relay's wait: the measured
    # 45 ms rms between `updated` and the measurement
    uplink_mean_s: float = 0.034
    # Gaps, calibrated through a beacon_node emulation. Measured -> model (mean of 7 runs of 4x the
    # session): gaps > 0.3 / 0.5 / 1.0 s per minute of driving 9.1 / 4.3 / 0.96 -> 8.3 / 3.8 /
    # 0.80; both antennas in 23 -> 23 % of the stale pauses; > 0.3 s gaps followed by another
    # within 2 s 39 -> 32 %; SAFE_STOPs with today's 0.3 s watchdog 2.1 -> 2.1 per minute of
    # driving
    # gap onsets per second per antenna above MOVING_SPEED_THRESHOLD_MPS, outside a bad spell
    gap_rate_moving: float = 0.08
    gap_rate_still: float = 0.055       # ... and below it
    gap_median_moving_s: float = 0.23   # lognormal gap length: median while moving ...
    gap_median_still_s: float = 0.18    # ... and at rest
    gap_sigma: float = 0.72             # ... and sigma of its log
    gap_shared_fraction: float = 0.35   # onsets that stop both antennas at once
    burst_per_min: float = 2.1          # bad-link spells per minute (common to both antennas) ...
    burst_mean_s: float = 8.0           # ... their mean length (exponential) ...
    burst_factor: float = 15.0          # ... and the gap onset rate x this during one
    # deliveries with an older stamp than the one before (up to 80 ms)
    out_of_order_per_h: float = 30.0
    # Backlog after a gap. Bags, gaps > 0.5 s while moving: the first position after one is a
    # median 0.48 s (p90 0.97 s) older than its stamp, 35 of 44 are the next epoch (nothing lost),
    # the rest fresh; the backlog drains at ~1.2 s per s (within 0.1 s again after a median 0.32
    # s). Through the same beacon_node emulation (20 runs of 4x the session) the model gives 0.52
    # (p90 0.99) s, 78 % the next epoch, 1.1 s per s, 0.33 s
    # hold_fraction: gaps after which the held positions follow, oldest first (0: none held)
    hold_fraction: float = 0.9
    backlog_interval_s: float = 0.04    # relay interval while catching up: mean ...
    backlog_jitter_s: float = 0.015     # ... and standard deviation (clipped to [0.01, 0.15] s)
    backlog_max_s: float = 2.0          # held positions older than this are dropped
    white_m: float = 0.004              # per fix, common to both antennas
    # per fix and antenna: heading noise ~0.15 deg on the 2.082 m baseline
    antenna_m: float = 0.004
    wander_m: float = 0.01              # slow common wander (Gauss-Markov) ...
    wander_tau_s: float = 60.0          # ... and its time constant
    # at rest |v| ~ 0.01 m/s, moving ~0.04 m/s step noise at 10 Hz
    speed_noise_mps: float = 0.02


# The ideal feed as a FeedConfig, for running faults without the real feed's randomness: every 20
# Hz tick, exact
EXACT_FEED = FeedConfig(
    receiver_hz=20.0,
    interval_s=0.05,
    jitter_s=0.0,
    latency_s=0.0,
    uplink_mean_s=0.0,
    gap_rate_moving=0.0,
    gap_rate_still=0.0,
    burst_per_min=0.0,
    out_of_order_per_h=0.0,
    hold_fraction=0.0,
    white_m=0.0,
    antenna_m=0.0,
    wander_m=0.0,
    speed_noise_mps=0.0,
)

FAULT_KINDS = ('dropout', 'jump', 'noise', 'stuck', 'freeze', 'swap', 'delay', 'burst', 'clock')
_BOTH_ONLY = ('stuck', 'swap')
_ANTENNAS = {'front': (0,), 'rear': (1,), 'both': (0, 1)}


@dataclass(frozen=True)
class GnssFault:
    """One scheduled GNSS fault (see the module docstring): one trigger, t_s, at_m or box."""

    kind: str
    antenna: str = 'both'               # front | rear | both
    t_s: Optional[float] = None         # trigger: sim time [s] ...
    at_m: Optional[float] = None        # ... or distance driven since the start [m] ...
    # ... or base_link inside (lat_min, lat_max, lon_min, lon_max)
    box: Optional[Tuple[float, float, float, float]] = None
    duration_s: float = 0.0             # 0: one epoch (jump), while inside (box)
    east_m: float = 0.0                 # jump
    north_m: float = 0.0
    sigma_m: float = 0.0                # noise
    delay_s: float = 0.0                # delay
    offset_s: float = 0.0               # clock

    def __post_init__(self):
        if self.kind not in FAULT_KINDS:
            raise ValueError(f'fault kind {self.kind!r}: expected one of {FAULT_KINDS}')
        if self.antenna not in _ANTENNAS:
            raise ValueError(f'fault antenna {self.antenna!r}: expected front, rear or both')
        if self.kind in _BOTH_ONLY and self.antenna != 'both':
            raise ValueError(f'fault {self.kind}: affects both antennas (antenna: both)')
        if sum(v is not None for v in (self.t_s, self.at_m, self.box)) != 1:
            raise ValueError(f'fault {self.kind}: give exactly one trigger (t_s, at_m or box)')
        if self.box is not None:
            object.__setattr__(self, 'box', tuple(float(v) for v in self.box))
            dead_zones_from_flat(self.box)               # validates
        if self.duration_s < 0.0 or (
            self.duration_s == 0.0 and self.kind != 'jump' and self.box is None
        ):
            raise ValueError(
                f'fault {self.kind}: duration_s > 0 needed (0 only for a one-epoch jump or a box)'
            )
        need = {
            'jump': self.east_m or self.north_m,
            'noise': self.sigma_m > 0.0,
            'delay': self.delay_s > 0.0,
            'clock': self.offset_s,
        }
        if self.kind in need and not need[self.kind]:
            raise ValueError(
                f'fault {self.kind}: needs '
                + {
                    'jump': 'east_m / north_m',
                    'noise': 'sigma_m > 0',
                    'delay': 'delay_s > 0',
                    'clock': 'offset_s',
                }[self.kind]
            )


def faults_from_dicts(items: Sequence[dict]) -> List[GnssFault]:
    """Build GnssFaults from plain dicts (a YAML scenario); ValueError names the bad entry."""
    out = []
    for i, d in enumerate(items or []):
        try:
            out.append(GnssFault(**dict(d)))
        except TypeError as e:                  # unknown field
            raise ValueError(f'fault {i}: {e}') from None
        except ValueError as e:
            raise ValueError(f'fault {i}: {e}') from None
    return out


def faults_from_param(text: str) -> List[GnssFault]:
    """Build GnssFaults from the gps.faults parameter, a YAML list of fault dicts ('' = none)."""
    if not text or not text.strip():
        return []
    items = yaml.safe_load(text)
    if not isinstance(items, list):
        raise ValueError('gps.faults: expected a YAML list of faults')
    return faults_from_dicts(items)


# Scenario-level parameters a fault scenario may set on the sim backend (lever-arm cases: where the
# sim puts the antennas, while the stack keeps assuming its own gnss.offset_x)
SCENARIO_BACKEND_PARAMS = ('antenna_offset_x',)


def resolve_fault_scenario(
    spec: str, catalogue: Optional[dict] = None, zone_sets: Optional[dict] = None
):
    """
    Resolve gnss_faults:=<spec> to (fault dicts, backend parameter overrides, scenario name).

    spec is a scenario name in catalogue['scenarios'] (config/gnss_faults.yaml) or a path
    to a YAML file with `faults:` (and optionally `backend_params:`). A fault's `box` may
    name a dead-zone set (config/gnss_dead_zones.yaml): its first box. Every fault is
    validated; ValueError names what is wrong.
    """
    scenarios = (catalogue or {}).get('scenarios') or {}
    if spec in scenarios:
        sc, name = scenarios[spec], spec
    elif spec.endswith(('.yaml', '.yml')):
        try:
            with open(spec) as f:
                sc = yaml.safe_load(f) or {}
        except OSError as e:
            raise ValueError(f'gnss_faults:={spec}: {e}') from None
        name = spec
    else:
        raise ValueError(
            f'gnss_faults:={spec}: no such scenario (or .yaml file); known: {sorted(scenarios)}'
        )
    faults = []
    for i, d in enumerate(sc.get('faults') or []):
        d = dict(d)
        if isinstance(d.get('box'), str):
            boxes = (zone_sets or {}).get(d['box'])
            if not boxes:
                raise ValueError(
                    f'gnss_faults:={spec}: fault {i}: box {d["box"]!r} is not a dead-zone set; '
                    f'known: {sorted(zone_sets or {})}'
                )
            d['box'] = [float(v) for v in boxes[0]]
        faults.append(d)
    faults_from_dicts(faults)                    # validates
    backend = dict(sc.get('backend_params') or {})
    bad = sorted(set(backend) - set(SCENARIO_BACKEND_PARAMS))
    if bad:
        raise ValueError(
            f'gnss_faults:={spec}: backend_params {bad} not allowed; allowed: '
            f'{SCENARIO_BACKEND_PARAMS}'
        )
    return faults, {k: float(v) for k, v in backend.items()}, name


def _offset(fix, east_m, north_m):
    """(lat, lon) moved by east_m / north_m (WGS84 radii of curvature at that latitude)."""
    lat, lon = fix
    a, e2 = 6378137.0, 6.69437999014e-3
    w = 1.0 - e2 * math.sin(math.radians(lat)) ** 2
    m, n = a * (1.0 - e2) / w ** 1.5, a / math.sqrt(w)
    return (
        lat + math.degrees(north_m / m),
        lon + math.degrees(east_m / (n * math.cos(math.radians(lat)))),
    )


FEED_PREFIX = 'gps.feed.'


def feed_param_defaults() -> dict:
    """
    Return backend parameter names -> defaults.

    gps.feed.enabled (False = ideal feed) and one per FeedConfig field.
    """
    out = {FEED_PREFIX + 'enabled': False}
    out.update({FEED_PREFIX + f.name: f.default for f in fields(FeedConfig)})
    return out


def feed_from_params(get: Callable[[str], object]) -> Optional[FeedConfig]:
    """
    Build a FeedConfig from a backend's parameters (get(name) -> value).

    None (ideal feed) unless gps.feed.enabled.
    """
    if not get(FEED_PREFIX + 'enabled'):
        return None
    return FeedConfig(**{f.name: float(get(FEED_PREFIX + f.name)) for f in fields(FeedConfig)})


def dead_zones_from_flat(values: Sequence[float]) -> List[Tuple[float, float, float, float]]:
    """
    Turn [lat_min, lat_max, lon_min, lon_max, ...] into boxes.

    ValueError for a ragged list or an empty box.
    """
    v = [float(x) for x in values]
    if len(v) % 4:
        raise ValueError(
            'gps.dead_zones needs 4 numbers per box (lat_min, lat_max, lon_min, lon_max), got '
            f'{len(v)}'
        )
    boxes = [tuple(v[i:i + 4]) for i in range(0, len(v), 4)]
    for a, b, c, d in boxes:
        if not (a < b and c < d):
            raise ValueError(
                f'gps.dead_zones box {(a, b, c, d)}: need lat_min < lat_max and lon_min < lon_max'
            )
    return boxes


def launch_overrides(
    feed: str = '',
    dead_zones: str = '',
    seed: str = '',
    sets: Optional[dict] = None,
    faults: str = '',
    catalogue: Optional[dict] = None,
) -> dict:
    """
    Return parameter overrides for the GNSS launch arguments.

    gnss_feed (real | ideal), gnss_dead_zones (set names from config/gnss_dead_zones.yaml,
    comma-separated), gnss_seed (int) and gnss_faults (a scenario of config/gnss_faults.yaml,
    or a .yaml file); '' keeps carla_params.yaml's value. ValueError for anything else,
    naming the known sets / scenarios.
    """
    out = {}
    if faults:
        items, backend, name = resolve_fault_scenario(faults, catalogue, sets)
        out['gps.faults'] = (
            yaml.safe_dump(items, default_flow_style=True, width=1000).strip() if items else ''
        )
        out['gps.faults_name'] = name
        out.update(backend)
    if feed:
        if feed not in ('real', 'ideal'):
            raise ValueError(f"gnss_feed:={feed}: expected real or ideal ('' = carla_params.yaml)")
        out[FEED_PREFIX + 'enabled'] = feed == 'real'
    if dead_zones:
        sets = sets or {}
        names = [n.strip() for n in dead_zones.split(',') if n.strip()]
        unknown = [n for n in names if n not in sets]
        if unknown:
            raise ValueError(f'gnss_dead_zones: unknown set(s) {unknown}; known: {sorted(sets)}')
        flat = [float(v) for n in names for box in sets[n] for v in box]
        dead_zones_from_flat(flat)                   # validates
        out['gps.dead_zones'] = flat
    if seed:
        try:
            out['gps.seed'] = int(seed)
        except ValueError:
            raise ValueError(
                f"gnss_seed:={seed}: expected an integer ('' = carla_params.yaml, 0 = random)"
            )
    return out


class GnssSim:

    def __init__(
        self,
        antenna_offset_x: float,
        ids: Sequence[str],
        noise_stddev_m: float = 0.0,
        noise_heading: bool = False,
        dropout_period_s: float = 0.0,
        dropout_duration_s: float = 0.0,
        seed: int = 0,
        feed: Optional[FeedConfig] = None,
        dead_zones: Sequence[Tuple[float, float, float, float]] = (),
        faults: Sequence[GnssFault] = (),
    ):
        self.antenna_offset_x = float(antenna_offset_x)
        self.ids = tuple(ids)               # front, rear
        self.noise = float(noise_stddev_m)
        self.noise_heading = bool(noise_heading)
        self.dropout_period = float(dropout_period_s)
        self.dropout_duration = float(dropout_duration_s)
        self.faults = list(faults)
        # faults on the ideal feed: run it as EXACT_FEED
        self.exact = feed is None and bool(self.faults)
        self.feed = EXACT_FEED if self.exact else feed
        self.dead_zones = [tuple(float(v) for v in z) for z in dead_zones]
        self._rng = random.Random(int(seed) or None)
        self.ok = True
        # feed state
        self._last_t = None
        self._epoch_grid = None             # (origin ns, last slot measured)
        # (t, at Octane, ((lat, lon) front, (lat, lon) rear), heading, speed, elev)
        self._epochs = collections.deque()
        self._wander = [0.0, 0.0]
        self._next_delivery = [None, None]  # per antenna
        self._last_stamp = [None, None]
        self._gap = [(-1.0, -1.0)] * 3      # (from, until): front, rear, both
        # ... and whether that gap holds its positions (backlog)
        self._gap_holds = [False] * 3
        self._backlog = [False, False]      # per antenna: catching up on held positions
        # per antenna: measurement time of the newest epoch sent
        self._sent_t = [None, None]
        self._bad_until = -1.0              # end of the current bad-link spell
        # fault state: per fault {start, end, consumed (one-epoch jump), frozen fixes / epoch, held
        # (burst)}
        self._fstate = [
            {
                'start': None,
                'end': None,
                'ended': False,
                'consumed': False,
                'frozen': [None, None],
                'epoch': None,
                'held': [],
            }
            for _ in self.faults
        ]
        self.fault_events = []              # (sim time, fault index, 'start' | 'end', kind)
        self._odo = 0.0                     # distance driven [m] (at_m triggers)
        self._last_xy = None

    def new_fault_events(self) -> List[dict]:
        """
        Return fault starts / ends since the last call, for /carla/gnss_fault.

        As dicts (t, index, edge, kind, antenna).
        """
        n = getattr(self, '_events_sent', 0)
        self._events_sent = len(self.fault_events)
        return [{'t': t, 'index': i, 'edge': edge, 'kind': kind, 'antenna': self.faults[i].antenna}
                for t, i, edge, kind in self.fault_events[n:]]

    def summary(self) -> str:
        """One line for the backend's startup log."""
        if self.feed is None or self.exact:
            s = 'ideal feed (every tick, exact)'
        else:
            c = self.feed
            s = (
                f'real Octane feed model: {c.receiver_hz:g} Hz positions relayed every '
                f'{c.interval_s * 1000:.0f} ms, '
                f'latency {c.latency_s:g} s + uplink {c.uplink_mean_s * 1000:.0f} ms, gaps '
                f'{c.gap_rate_moving:g} / '
                f'{c.gap_rate_still:g} per s moving / still, bad-link spells {c.burst_per_min:g} '
                '/ min'
            )
            if c.hold_fraction > 0.0:
                s += f', backlog after {c.hold_fraction * 100:.0f} % of the gaps'
        extras = [f'antennas +-{self.antenna_offset_x:g} m from base_link']
        if self.noise > 0.0:
            extras.append(
                f'noise {self.noise:g} m' + (' per antenna' if self.noise_heading else '')
            )
        if self.dropout_period > 0.0 and self.dropout_duration > 0.0:
            extras.append(f'dropout {self.dropout_duration:g} s every {self.dropout_period:g} s')
        if self.dead_zones:
            extras.append(f'{len(self.dead_zones)} dead zone(s)')
        if self.faults:
            extras.append(
                f'{len(self.faults)} scheduled fault(s): ' + ', '.join(f.kind for f in self.faults)
            )
        return s + ''.join(f'; {e}' for e in extras)

    def step(
        self,
        sim_ns: int,
        x: float,
        y: float,
        fx: float,
        fy: float,
        speed: float,
        elevation: float,
        to_wgs: Callable[[float, float], Tuple[float, float]],
    ) -> List[dict]:
        """
        Run one GNSS tick and return the beacon_update payloads (front, rear).

        Inputs: base_link (x, y) and unit forward vector (fx, fy) in to_wgs's frame [m],
        ground speed [m/s], antenna elevation [m]. Returns [] during a dropout.
        """
        sim_t = sim_ns / 1e9
        if self.dropout_period > 0.0 and self.dropout_duration > 0.0:
            self.ok = sim_t % self.dropout_period >= self.dropout_duration
        else:
            self.ok = True
        if self.ok and self.dead_zones:
            lat, lon = to_wgs(x, y)
            self.ok = not any(a <= lat <= b and c <= lon <= d for a, b, c, d in self.dead_zones)
        if self.feed is not None:
            return self._step_feed(sim_ns, x, y, fx, fy, speed, elevation, to_wgs)
        if not self.ok:
            return []

        nx = ny = 0.0
        if self.noise > 0.0:
            nx, ny = self._rng.gauss(0.0, self.noise), self._rng.gauss(0.0, self.noise)
        fixes = []                          # (lat, lon): front, rear
        for sign in (+1.0, -1.0):
            ax = x + fx * self.antenna_offset_x * sign + nx
            ay = y + fy * self.antenna_offset_x * sign + ny
            if self.noise > 0.0 and self.noise_heading:
                ax += self._rng.gauss(0.0, self.noise)
                ay += self._rng.gauss(0.0, self.noise)
            fixes.append(to_wgs(ax, ay))
        heading = _compass_deg(fixes)
        return [beacon_payload(bid, sim_ns, lat, lon, elevation, heading, speed)
                for bid, (lat, lon) in zip(self.ids, fixes)]

    # ------------------------------------------------------------------ #
    #  Real feed                                                         #
    # ------------------------------------------------------------------ #
    def _step_feed(self, sim_ns, x, y, fx, fy, speed, elevation, to_wgs) -> List[dict]:
        c = self.feed
        rng = self._rng
        sim_t = sim_ns / 1e9
        dt = 0.0 if self._last_t is None else max(0.0, sim_t - self._last_t)
        self._last_t = sim_t
        if self.faults:
            self._update_faults(sim_t, x, y, to_wgs)

        # receiver: a new position pair in every 1 / receiver_hz slot (integer ns: float sums
        # drift)
        period_ns = round(1e9 / c.receiver_hz)
        if self._epoch_grid is None:
            self._epoch_grid = (sim_ns, -1)
        t0_ns, last_slot = self._epoch_grid
        slot = (sim_ns - t0_ns + 1_000_000) // period_ns    # 1 ms tolerance for callers' rounding
        if slot > last_slot:
            self._epoch_grid = (t0_ns, slot)
            ep = self._measure(sim_t, period_ns / 1e9, x, y, fx, fy, speed, elevation, to_wgs)
            self._epochs.append(self._fault_epoch(ep) if self.faults else ep)
            keep = (
                c.latency_s
                + 10.0 * c.uplink_mean_s
                + 0.2
                + max((f.delay_s for f in self.faults), default=0.0)
            )
            keep += c.backlog_max_s if c.hold_fraction > 0.0 else 0.0
            while len(self._epochs) > 1 and self._epochs[1][0] <= sim_t - keep:
                self._epochs.popleft()

        # gap onsets in this step (Poisson): own gaps per antenna, shared ones for both
        moving = abs(speed) > MOVING_SPEED_THRESHOLD_MPS
        rate = c.gap_rate_moving if moving else c.gap_rate_still
        if c.burst_per_min > 0.0:
            if sim_t >= self._bad_until and rng.random() < 1.0 - math.exp(
                -c.burst_per_min / 60.0 * dt
            ):
                self._bad_until = sim_t + rng.expovariate(1.0 / c.burst_mean_s)
            if sim_t < self._bad_until:
                rate *= c.burst_factor
        median = c.gap_median_moving_s if moving else c.gap_median_still_s
        for k, share in ((0, 1.0 - c.gap_shared_fraction), (1, 1.0 - c.gap_shared_fraction),
                         (2, c.gap_shared_fraction)):
            if (
                rate * share > 0.0
                and sim_t >= self._gap[k][1]
                and rng.random() < 1.0 - math.exp(-rate * share * dt)
            ):
                start = sim_t - rng.uniform(0.0, dt)        # somewhere inside this step
                self._gap[k] = (start, start + rng.lognormvariate(math.log(median), c.gap_sigma))
                # (no draw without a backlog: the same random stream)
                if c.hold_fraction > 0.0:
                    self._gap_holds[k] = rng.random() < c.hold_fraction

        # relay: each antenna's deliveries due by now, stamped with their relay time
        p_ooo = c.out_of_order_per_h / 3600.0 * c.interval_s
        out = []
        for k in (0, 1):
            if self._next_delivery[k] is None:          # exact feed (no jitter): on the ticks
                self._next_delivery[k] = sim_t + (
                    rng.uniform(0.0, c.interval_s) if c.jitter_s > 0.0 else 0.0
                )
            while self._next_delivery[k] <= sim_t + 1e-9:
                t_d = self._next_delivery[k]
                gaps = [g for g in (k, 2) if self._gap[g][0] <= t_d < self._gap[g][1]]
                # catching up (not inside the gap itself)
                fast = self._backlog[k] and self.ok and not gaps
                mean, sd = (
                    (c.backlog_interval_s, c.backlog_jitter_s)
                    if fast
                    else (c.interval_s, c.jitter_s)
                )
                self._next_delivery[k] = t_d + min(0.15, max(0.01, rng.gauss(mean, sd)))
                if not self.ok or gaps:
                    # a gap that holds its positions sends them when it ends; otherwise the newest
                    # comes next
                    self._backlog[k] = self.ok and any(self._gap_holds[g] for g in gaps)
                    continue
                if self.faults and self._fault_on('dropout', k, t_d) is not None:
                    self._backlog[k] = False
                    continue
                stamp = t_d
                ooo = self._last_stamp[k] is not None and rng.random() < p_ooo
                if ooo:
                    stamp = self._last_stamp[k] - rng.uniform(0.005, 0.08)
                else:
                    self._last_stamp[k] = stamp
                look, sent = stamp, stamp
                if self.faults:
                    i = self._fault_on('delay', k, t_d)
                    look -= self.faults[i].delay_s if i is not None else 0.0
                    i = self._fault_on('clock', k, t_d)
                    sent += self.faults[i].offset_s if i is not None else 0.0
                ep = self._epoch_at(look)
                if ep is None:
                    continue
                if not ooo:
                    # catching up: the next epoch after the last one sent
                    if self._backlog[k]:
                        held = self._held_epoch(self._sent_t[k], look - c.backlog_max_s, look)
                        if held is not None and held[0] < ep[0]:
                            ep = held
                        else:
                            self._backlog[k] = False        # caught up: the newest goes out
                    self._sent_t[k] = (
                        ep[0] if self._sent_t[k] is None else max(self._sent_t[k], ep[0])
                    )
                _, _, fixes, heading, v, elev = ep
                p = beacon_payload(self.ids[k], round(sent * 1e9), *fixes[k], elev, heading, v)
                i = self._fault_on('burst', k, t_d) if self.faults else None
                if i is not None:
                    self._fstate[i]['held'].append(p)
                    continue
                out.append(p)
        # a burst that has ended releases everything it held, at once
        for i, f in enumerate(self.faults):
            st = self._fstate[i]
            if f.kind == 'burst' and st['held'] and not self._active(i, sim_t):
                out.extend(st['held'])
                st['held'] = []
        return out

    # ------------------------------------------------------------------ #
    #  Faults                                                            #
    # ------------------------------------------------------------------ #
    def _update_faults(self, sim_t, x, y, to_wgs):
        """Update the odometer, triggers, and the end of time-limited faults or box stays."""
        if self._last_xy is not None:
            self._odo += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        latlon = to_wgs(x, y) if any(f.box is not None for f in self.faults) else None
        for i, f in enumerate(self.faults):
            st = self._fstate[i]
            inside = (latlon is not None and f.box is not None
                      and f.box[0] <= latlon[0] <= f.box[1] and f.box[2] <= latlon[1] <= f.box[3])
            if st['start'] is None:
                if f.t_s is not None and sim_t >= f.t_s:
                    st['start'] = f.t_s
                elif (f.at_m is not None and self._odo >= f.at_m) or inside:
                    st['start'] = sim_t
                else:
                    continue
                if f.duration_s > 0.0:
                    st['end'] = st['start'] + f.duration_s
                self.fault_events.append((round(st['start'], 3), i, 'start', f.kind))
            if f.box is not None and f.duration_s == 0.0 and st['end'] is None and not inside:
                st['end'] = sim_t                   # left the box
            if not st['ended'] and st['end'] is not None and sim_t >= st['end']:
                st['ended'] = True
                self.fault_events.append((round(st['end'], 3), i, 'end', f.kind))

    def _active(self, i, t):
        f, st = self.faults[i], self._fstate[i]
        if st['start'] is None or t < st['start'] - 1e-9:
            return False
        if f.kind == 'jump' and f.duration_s == 0.0 and f.box is None:
            return not st['consumed']
        return st['end'] is None or t < st['end'] - 1e-9

    def _fault_on(self, kind, k, t):
        """Index of the first active fault of `kind` affecting antenna k at time t, or None."""
        for i, f in enumerate(self.faults):
            if f.kind == kind and k in _ANTENNAS[f.antenna] and self._active(i, t):
                return i
        return None

    def _fault_epoch(self, ep):
        """
        Return a receiver epoch with the active measurement faults applied.

        Faults: stuck, freeze, jump, noise, swap.
        """
        t, at_octane, fixes, heading, v, elev = ep
        i = self._fault_on('stuck', 0, t)
        if i is not None:
            st = self._fstate[i]
            if st['epoch'] is None:
                st['epoch'] = (fixes, heading, v)
            fixes, heading, v = st['epoch']
            return (t, at_octane, fixes, heading, v, elev)
        fixes, changed = list(fixes), False
        for i, f in enumerate(self.faults):
            if f.kind not in ('freeze', 'jump', 'noise') or not self._active(i, t):
                continue
            st = self._fstate[i]
            for k in _ANTENNAS[f.antenna]:
                if f.kind == 'freeze':
                    if st['frozen'][k] is None:
                        st['frozen'][k] = fixes[k]
                    fixes[k] = st['frozen'][k]
                elif f.kind == 'jump':
                    fixes[k] = _offset(fixes[k], f.east_m, f.north_m)
                else:
                    fixes[k] = _offset(
                        fixes[k], self._rng.gauss(0.0, f.sigma_m), self._rng.gauss(0.0, f.sigma_m)
                    )
            changed = True
            if f.kind == 'jump' and f.duration_s == 0.0 and f.box is None:
                st['consumed'] = True
                st['ended'] = True
                self.fault_events.append((round(t, 3), i, 'end', f.kind))
        if self._fault_on('swap', 0, t) is not None:
            fixes, changed = [fixes[1], fixes[0]], True
        if changed:
            heading = _compass_deg(fixes)
        return (t, at_octane, tuple(fixes), heading, v, elev)

    def _measure(self, t, period, x, y, fx, fy, speed, elevation, to_wgs):
        """One receiver epoch: both antenna fixes with the feed's noise, heading, noisy speed."""
        c, rng = self.feed, self._rng
        a = math.exp(-period / c.wander_tau_s) if c.wander_tau_s > 0.0 else 0.0
        for i in (0, 1):
            self._wander[i] = a * self._wander[i] + c.wander_m * math.sqrt(
                1.0 - a * a
            ) * rng.gauss(0.0, 1.0)
        cx = (
            self._wander[0]
            + rng.gauss(0.0, c.white_m)
            + (rng.gauss(0.0, self.noise) if self.noise > 0.0 else 0.0)
        )
        cy = (
            self._wander[1]
            + rng.gauss(0.0, c.white_m)
            + (rng.gauss(0.0, self.noise) if self.noise > 0.0 else 0.0)
        )
        fixes = []
        for sign in (+1.0, -1.0):
            ax = x + fx * self.antenna_offset_x * sign + cx + rng.gauss(0.0, c.antenna_m)
            ay = y + fy * self.antenna_offset_x * sign + cy + rng.gauss(0.0, c.antenna_m)
            if self.noise > 0.0 and self.noise_heading:
                ax += rng.gauss(0.0, self.noise)
                ay += rng.gauss(0.0, self.noise)
            fixes.append(to_wgs(ax, ay))
        v = abs(speed + rng.gauss(0.0, c.speed_noise_mps))
        at_octane = (
            t
            + c.latency_s
            + (rng.expovariate(1.0 / c.uplink_mean_s) if c.uplink_mean_s > 0.0 else 0.0)
        )
        return (t, at_octane, tuple(fixes), _compass_deg(fixes), v, elevation)

    def _epoch_at(self, t):
        """Return the newest receiver epoch that had reached Octane by relay time t, or None."""
        found = None
        for ep in self._epochs:
            if ep[1] <= t + 1e-9:
                found = ep
        return found

    def _held_epoch(self, after_t, min_t, t):
        """
        Return the oldest epoch that had reached Octane by relay time t, or None.

        It must be measured after after_t (None: any) and not before min_t.
        """
        for ep in self._epochs:
            if (
                (after_t is None or ep[0] > after_t + 1e-9)
                and ep[0] >= min_t - 1e-9
                and ep[1] <= t + 1e-9
            ):
                return ep
        return None


def _compass_deg(fixes) -> float:
    """Compass degrees of the rear -> front baseline (true north), as Octane's dynamics.heading."""
    (lat_f, lon_f), (lat_r, lon_r) = fixes
    east = math.radians(lon_f - lon_r) * math.cos(math.radians(lat_f))
    return math.degrees(math.atan2(east, math.radians(lat_f - lat_r))) % 360.0
