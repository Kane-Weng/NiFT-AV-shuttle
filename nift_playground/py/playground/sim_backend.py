"""
The sim backend: the repo's KinematicSimNode, unchanged, ticked by the runner (ARCH sections 2.1, 7, 8.4).

Two things change around the node, and nothing inside it.

1. Before construction, `nift_carla.kinematic_sim_node.OctaneEmulator` is set to `BrowserOctane` (and put back right after), so the node's `__init__` builds and starts the in-process Octane server instead of an aiohttp one.
2. The node's tick thread is never started. The runner calls `tick()`, which calls `KinematicSimNode._tick_once()`, the method the thread would call. With `time_factor 0` and `lockstep false` it never sleeps and never waits, so the backend refuses anything else.

GNSS faults (ARCH 8.4). Scheduled sim faults (`sim_gnss.GnssFault`) normally come from the launch option `gnss_faults` and a reset. A live preset ("dropout 1.5 s now") is a fault with `t_s = now + dt`, added by `schedule_fault()`. It rebuilds `node.gnss` as a new GnssSim that carries on the old one's state (random stream, feed, odometer, earlier faults and their events) with the new fault appended. That is what the same GnssSim would have done with the fault scheduled from the start, as long as the fault triggers after now. On the ideal feed the faults run through `EXACT_FEED`, as GnssSim does whenever it has faults, which gives the same fixes. `tests/gnss/test_sim_backend.py` proves both. A run without live faults never touches `node.gnss`. Live link faults go to `BrowserOctane.add_link_fault()`.

Private names used, all read or called and never patched: `kinematic_sim_node.OctaneEmulator` (module attribute), `KinematicSimNode._tick_once`, `._sim_ns`, `.dt`, `.gnss`, plus `.car`, `.plant`, `.ecu`, `._dbw_status` and `.octane` for the read-only accessors. GnssSim's own state is copied as a whole (`__dict__`), and only `faults`, `_fstate`, `exact` and `feed` are set.
"""

import copy
import math

from .octane_bridge import BrowserOctane, LINK_FAULTS

# Live presets for the page (ARCH 8.4): name -> (kind of fault, default parameters, one line for the page)
SIM_PRESETS = {
    'dropout': ({'duration_s': 1.5, 'antenna': 'both'}, 'no fixes from the antenna(s)'),
    'jump': ({'east_m': 0.5, 'north_m': 0.0, 'antenna': 'both', 'duration_s': 0.0},
             'fixes offset by (east_m, north_m); duration 0 = one epoch'),
    'noise': ({'sigma_m': 0.5, 'duration_s': 5.0, 'antenna': 'both'}, 'fixes scattered (an RTK float episode)'),
    'stuck': ({'duration_s': 2.0}, 'the whole feed repeats one epoch with fresh stamps'),
    'freeze': ({'duration_s': 2.0, 'antenna': 'front'}, "the antenna's position stops, stamps stay fresh"),
    'swap': ({'duration_s': 1.0}, 'front and rear swapped (heading off by 180 deg)'),
    'delay': ({'delay_s': 0.5, 'duration_s': 5.0, 'antenna': 'both'}, 'positions delay_s older than usual'),
    'burst': ({'duration_s': 2.0, 'antenna': 'both'}, 'deliveries held back, then all sent at the end'),
    'clock': ({'offset_s': 0.5, 'duration_s': 5.0, 'antenna': 'both'}, 'stamps off by offset_s'),
}
LINK_PRESETS = {
    'outage': ({'duration_s': 1.5}, 'Octane link down: both antennas silent'),
    'outage_front': ({'duration_s': 1.5}, 'front antenna silent on the link'),
    'outage_rear': ({'duration_s': 1.5}, 'rear antenna silent on the link'),
    'repeat': ({'duration_s': 1.5}, 'a stuck relay: the last payloads again, same `updated`'),
}
assert set(LINK_PRESETS) == set(LINK_FAULTS)


def gnss_fault_presets(catalogue=None):
    """
    The `presets.gnss_faults` part of the `ready` message.

    scenarios: the catalogue's names (config/gnss_faults.yaml, a reset with the launch option gnss_faults).
    live: the presets `gnss_fault {preset, params}` applies at once, kind 'sim' (a GnssFault from the next tick) or
    'link' (BrowserOctane).
    """
    scenarios = []
    for name, sc in sorted(((catalogue or {}).get('scenarios') or {}).items()):
        sc = sc or {}
        scenarios.append({'name': name, 'route': sc.get('route'), 'faults': sc.get('faults') or [],
                          'target': sc.get('target')})
    live = [{'name': n, 'kind': 'sim', 'params': dict(p), 'doc': d} for n, (p, d) in SIM_PRESETS.items()]
    live += [{'name': n, 'kind': 'link', 'params': dict(p), 'doc': d} for n, (p, d) in LINK_PRESETS.items()]
    return {'scenarios': scenarios, 'live': live}


def _world():
    from fakeros._core import world
    return world


class SimBackend:
    """Build KinematicSimNode with BrowserOctane, tick it, and read its state for the page."""

    def __init__(self, reset_hub: bool = True, **node_kwargs):
        import socketio
        from nift_carla import kinematic_sim_node as ksn
        from nift_carla import sim_gnss
        self._ksn = ksn
        self._sim_gnss = sim_gnss
        if reset_hub:
            socketio.hub.reset()        # a fresh network per launch: no server of an earlier run, sids from 1
        original = ksn.OctaneEmulator
        ksn.OctaneEmulator = BrowserOctane
        try:
            node = ksn.KinematicSimNode(**node_kwargs)
        finally:
            ksn.OctaneEmulator = original
        self.node = node
        self._check_private_names()
        if not isinstance(node.octane, BrowserOctane):
            raise RuntimeError('kinematic_sim_node did not build BrowserOctane: the OctaneEmulator swap failed')
        if node.time_factor != 0.0:
            raise ValueError(f'kinematic_sim time_factor {node.time_factor}: the playground needs 0 '
                             '(the worker paces wall time, the node must never sleep)')
        if node.lockstep:
            raise ValueError('kinematic_sim lockstep true: the playground needs false (the runner order gives '
                             'what lockstep gives, and Pyodide cannot wait on a Condition)')
        self.octane = node.octane
        self.octane.antenna_ids = tuple(node.gnss.ids)
        self.live_faults = []           # (sim ns, preset, fault dict or link fault dict)

    def _check_private_names(self):
        n = self.node
        missing = [name for name in ('_tick_once', '_sim_ns', 'dt', 'gnss', 'car', 'plant', 'ecu', 'octane',
                                     '_dbw_status', '_thread') if not hasattr(n, name)]
        if not hasattr(self._ksn, 'OctaneEmulator'):
            missing.append('kinematic_sim_node.OctaneEmulator')
        if missing:
            raise RuntimeError(f'kinematic_sim_node changed: the playground needs {missing}')

    # ------------------------------------------------------------------ #
    #  Ticking                                                           #
    # ------------------------------------------------------------------ #
    @property
    def dt_ns(self) -> int:
        return round(self.node.dt * 1e9)

    @property
    def sim_ns(self) -> int:
        return self.node._sim_ns

    def tick(self, now_ns: int = None):
        """One sim step (ARCH 7 step 2). The world clock must already be at the new tick time."""
        expected = self.node._sim_ns + self.dt_ns
        if now_ns is None:
            now_ns = _world().now_ns
        if now_ns != expected:
            raise RuntimeError(f'sim tick at {now_ns} ns, but kinematic_sim expects {expected} ns '
                               '(set world.now_ns to the tick time first)')
        self.node._tick_once()
        if self.node._sim_ns != now_ns:
            raise RuntimeError(f'kinematic_sim moved to {self.node._sim_ns} ns, not {now_ns} ns')

    def close(self):
        """Stop the Octane server (its clients see a server disconnect)."""
        self.octane.stop()

    # ------------------------------------------------------------------ #
    #  GNSS faults                                                       #
    # ------------------------------------------------------------------ #
    def schedule_fault(self, fault):
        """
        Add one GnssFault (a dict as in gnss_faults.yaml, or a GnssFault) to the running GnssSim.

        Use t_s > now: a t_s in the past starts at once and is logged with its t_s. An at_m trigger counts the
        distance from the first fault the GnssSim got (from now, if it had none). Returns the GnssFault.
        """
        sg = self._sim_gnss
        f = fault if isinstance(fault, sg.GnssFault) else sg.faults_from_dicts([fault])[0]
        old = self.node.gnss
        template = sg.GnssSim(old.antenna_offset_x, old.ids, faults=[f])
        new = copy.copy(old)
        new.faults = list(old.faults) + [f]
        new._fstate = list(old._fstate) + [template._fstate[0]]
        if old.feed is None:            # the plain ideal feed: faults run through EXACT_FEED, as GnssSim does
            new.exact = True
            new.feed = sg.EXACT_FEED
        self.node.gnss = new
        return f

    def apply_gnss_fault(self, preset: str, params=None, now_ns: int = None) -> str:
        """
        Apply a live preset of gnss_fault_presets() at sim time now_ns (default: the world clock).

        Returns a line for the page's reply. ValueError for an unknown preset or bad parameters.
        """
        now = _world().now_ns if now_ns is None else now_ns
        params = dict(params or {})
        if preset in LINK_PRESETS:
            unknown = sorted(set(params) - set(LINK_PRESETS[preset][0]))
            if unknown:
                raise ValueError(f'gnss_fault {preset}: unknown parameter(s) {unknown}')
            values = dict(LINK_PRESETS[preset][0], **params)
            lf = self.octane.add_link_fault(preset, float(values['duration_s']), now)
            self.live_faults.append((now, preset, lf.as_dict()))
            return f'link fault {preset} for {lf.duration_s:g} s from the next tick'
        if preset in SIM_PRESETS:
            values = dict(SIM_PRESETS[preset][0], **params)
            for trigger in ('t_s', 'at_m', 'box'):
                values.pop(trigger, None)
            # "now": the first tick after the command (ARCH 8.4), on the sim's ms grid
            values['t_s'] = round((now + self.dt_ns) / 1e9, 6)
            d = dict(kind=preset, **values)
            if d.get('duration_s', 0.0) == 0.0 and preset != 'jump':
                raise ValueError(f'gnss_fault {preset}: duration_s > 0 needed')
            self.schedule_fault(d)
            self.live_faults.append((now, preset, d))
            return (f'GNSS fault {preset} from the next tick, page t={d["t_s"] - 1.0:.2f} s '
                    f'(sim time {d["t_s"]:.2f} s; {_describe(d)})')
        raise ValueError(f'gnss_fault {preset!r}: not a live preset ({sorted(SIM_PRESETS) + sorted(LINK_PRESETS)}); '
                         'a scenario of gnss_faults.yaml goes in the launch option gnss_faults (reset)')

    # ------------------------------------------------------------------ #
    #  Read-only state for the page                                      #
    # ------------------------------------------------------------------ #
    def truth(self) -> dict:
        x, y, yaw = self.node.car.base_link()
        return {'x': x, 'y': y, 'yaw': yaw, 'speed': self.node.car.v,
                'steer': math.radians(self.node.car.steer_deg)}

    def status(self) -> dict:
        n = self.node
        g = n.gnss
        now = n._sim_ns
        return {
            't': now / 1e9,
            'ecu': n._dbw_status.get('mode'),
            'gnss_ok': bool(g.ok),
            'faults': [{'index': i, 'kind': f.kind, 'antenna': f.antenna,
                        'active': g._active(i, now / 1e9) if hasattr(g, '_active') else None}
                       for i, f in enumerate(g.faults)],
            'link_faults': [f.as_dict() for f in self.octane.active_link_faults(now)],
            'octane': {'port': self.octane.port, 'clients_authed': self.octane.clients_authed,
                       'emitted': self.octane.emitted, 'dropped': self.octane.dropped},
        }


def _describe(d):
    return ', '.join(f'{k} {v}' for k, v in d.items() if k not in ('kind', 't_s'))
