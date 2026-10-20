"""
BrowserOctane: the Octane server of the kinematic sim, on the in-process socket.io hub (ARCH sections 2.1, 5, 8.4).

`nift_carla.octane_emulator.OctaneEmulator` runs a python-socketio AsyncServer on aiohttp in a thread. Pyodide has neither threads nor sockets, so the sim backend swaps this class in before it builds `KinematicSimNode`. It has the same constructor (`port, api_key, namespace, host, log`), the same `start()`, `publish(payloads)` and `stop()`, and the same `host`, `port`, `namespace`, `emitted` and `clients_authed` attributes the node's diagnostics read. It plays the emulator's protocol with the same log lines. A client connects, sends `auth` with the key and gets `auth_ok` (a wrong key gets no reply). It joins channels, and `beacon_update` goes to the `beacon` channel. The payloads come in already built by the repo's `sim_gnss.GnssSim` and `octane_emulator.beacon_payload`, so this class never makes one.

Live link faults model the Octane server or the Wi-Fi, outside every node. `outage` silences both antennas, `outage_front` and `outage_rear` one of them, and `repeat` delivers each antenna's last payload again with the same `updated` (a stuck relay). A fault added at sim time t covers the ticks after t up to t + duration_s, so it starts at the next tick.
"""

import copy
import logging

import socketio
from socketio.hub import normalize_url

BEACON_CHANNEL = 'beacon'           # as octane_emulator.BEACON_CHANNEL
LINK_FAULTS = ('outage', 'outage_front', 'outage_rear', 'repeat')


def _world_now_ns():
    from fakeros._core import world
    return int(world.now_ns)


class LinkFault:
    """One live link fault: kind, the sim time it was added and how long it lasts."""

    def __init__(self, kind, from_ns, duration_s):
        if kind not in LINK_FAULTS:
            raise ValueError(f'link fault {kind!r}: expected one of {LINK_FAULTS}')
        if not duration_s or float(duration_s) <= 0.0:
            raise ValueError(f'link fault {kind}: duration_s > 0 needed')
        self.kind = kind
        self.from_ns = int(from_ns)
        self.duration_s = float(duration_s)
        self.until_ns = self.from_ns + round(self.duration_s * 1e9)

    def active(self, now_ns):
        return self.from_ns < now_ns <= self.until_ns

    def as_dict(self):
        return {'kind': self.kind, 'from_s': self.from_ns / 1e9, 'until_s': self.until_ns / 1e9,
                'duration_s': self.duration_s}


class BrowserOctane:
    """OctaneEmulator's interface, served on `socketio.hub` instead of aiohttp."""

    def __init__(self, port: int, api_key: str, namespace: str = '/octane', host: str = '127.0.0.1', log=None,
                 hub=None, clock_ns=None):
        self.host = host
        self.port = port                # 0 = any free port; the one taken is set by start()
        self.namespace = namespace
        self._api_key = api_key
        self._log = log or logging.getLogger(__name__)
        self._authed = set()
        self.emitted = 0                # beacon_update events handed to socket.io
        self.hub = hub or socketio.hub
        self.clock_ns = clock_ns or _world_now_ns
        self.server = None
        # set by the sim backend from GnssSim.ids, for outage_front / outage_rear
        self.antenna_ids = None
        self.link_faults = []
        self.link_events = []           # (sim ns, 'start' | 'end', fault dict), for the page
        self.dropped = 0                # payloads a link fault swallowed
        self._last_sent = {}            # beacon id -> the last payload delivered (repeat)
        self._frozen = {}               # beacon id -> the payload a repeat fault repeats

    # ------------------------------------------------------------------ #
    @property
    def clients_authed(self) -> int:
        return len(self._authed)

    @property
    def url(self) -> str:
        return normalize_url(f'http://{self.host}:{self.port}')

    def start(self, timeout_s: float = 5.0):
        """Start serving. RuntimeError if the address is taken, as the emulator raises."""
        if not self.port:
            self.port = self.hub.free_port(self.host)
        try:
            self.server = self.hub.listen(f'http://{self.host}:{self.port}')
        except RuntimeError as exc:
            raise RuntimeError(f'Octane emulator cannot listen on {self.host}:{self.port}: {exc}') from None
        for event, handler in (('connect', self._on_connect), ('auth', self._on_auth),
                               ('join', self._on_join), ('disconnect', self._on_disconnect)):
            self.server.on(event, handler, namespace=self.namespace)
        self._log.info(f'Octane emulator listening on http://{self.host}:{self.port} '
                       f'namespace={self.namespace}')

    def publish(self, payloads):
        """Emit beacon_update payloads to the beacon channel. Delivered later (the hub's scheduler), never here."""
        if self.server is None or not self.server.open:
            return
        for p in self._apply_link_faults(list(payloads)):
            self.server.emit('beacon_update', p, room=BEACON_CHANNEL, namespace=self.namespace)
            self.emitted += 1

    def stop(self, timeout_s: float = 5.0):
        """Disconnect every client and free the address."""
        if self.server is None:
            return
        self.server.shutdown()
        self.server = None

    # ------------------------------------------------------------------ #
    #  Live link faults (ARCH 8.4)                                       #
    # ------------------------------------------------------------------ #
    def add_link_fault(self, kind: str, duration_s: float, now_ns: int = None) -> LinkFault:
        """Add a link fault at sim time now_ns (default: the clock now). It starts at the next tick."""
        fault = LinkFault(kind, self.clock_ns() if now_ns is None else now_ns, duration_s)
        if kind in ('outage_front', 'outage_rear') and self.antenna_ids is None:
            raise ValueError(f'link fault {kind}: the antenna ids are not known (antenna_ids)')
        self.link_faults.append(fault)
        self._log.warn(f'Octane link fault: {kind} for {fault.duration_s:g} s '
                       f'after sim time {fault.from_ns / 1e9:.2f} s')
        return fault

    def clear_link_faults(self):
        self.link_faults = []
        self._frozen = {}

    def active_link_faults(self, now_ns: int = None):
        now = self.clock_ns() if now_ns is None else now_ns
        return [f for f in self.link_faults if f.active(now)]

    def _apply_link_faults(self, payloads):
        if not self.link_faults:
            self._remember(payloads)
            return payloads
        now = self.clock_ns()
        active = []
        for f in self.link_faults:
            on = f.active(now)
            was = getattr(f, '_on', False)
            if on and not was:
                self.link_events.append((now, 'start', f.as_dict()))
            elif was and not on:
                self.link_events.append((now, 'end', f.as_dict()))
            f._on = on
            if on:
                active.append(f.kind)
        # finished faults go once their end has been seen
        self.link_faults = [f for f in self.link_faults if now <= f.until_ns or getattr(f, '_on', False)]
        if 'repeat' not in active:
            self._frozen = {}
        silent = set()
        if 'outage' in active:
            silent = {p.get('id') for p in payloads} | set(self.antenna_ids or ())
        if 'outage_front' in active:
            silent.add(self.antenna_ids[0])
        if 'outage_rear' in active:
            silent.add(self.antenna_ids[1])
        out = []
        for p in payloads:
            bid = p.get('id')
            if bid in silent:
                self.dropped += 1
                continue
            if 'repeat' in active and bid in self._last_sent:
                # a stuck relay: the payload delivered last before the fault, same `updated`
                frozen = self._frozen.setdefault(bid, self._last_sent[bid])
                out.append(copy.deepcopy(frozen))
                continue
            out.append(p)
        self._remember(out)
        return out

    def _remember(self, payloads):
        for p in payloads:
            self._last_sent[p.get('id')] = p

    # ------------------------------------------------------------------ #
    #  Protocol handlers (as OctaneEmulator's, synchronous)              #
    # ------------------------------------------------------------------ #
    def _on_connect(self, sid, environ, auth=None):
        self._log.info(f'Octane emulator: client {sid} connected')

    def _on_auth(self, sid, data):
        key = data.get('x-api-key') if isinstance(data, dict) else None
        if key != self._api_key:
            self._log.warning(f'Octane emulator: auth rejected for {sid} (wrong key)')
            return
        self._authed.add(sid)
        self.server.emit('auth_ok', {}, to=sid, namespace=self.namespace)
        self._log.info(f'Octane emulator: client {sid} authenticated')

    def _on_join(self, sid, data):
        channel = data.get('channel') if isinstance(data, dict) else None
        if sid not in self._authed or not channel:
            return
        self.server.enter_room(sid, channel, namespace=self.namespace)
        self._log.info(f'Octane emulator: client {sid} joined {channel}')

    def _on_disconnect(self, sid, reason=None):
        self._authed.discard(sid)
        self._log.info(f'Octane emulator: client {sid} disconnected')
