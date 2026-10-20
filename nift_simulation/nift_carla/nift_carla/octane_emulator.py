"""
Serve simulated GNSS fixes the way the Octane RTK server does, over Socket.io.

File: octane_emulator.py
Author: Kane Weng

Description:
  Plays the Mcity Octane RTK server in simulation. The bridge hands this
  module both antenna fixes every GNSS tick and it emits them as Octane
  `beacon_update` events, so beacon_node runs the same code as on the
  shuttle; only the server URL, key and beacon IDs in its environment
  differ. No carla or ROS node code in here, so it is unit-tested without
  a server.

  Protocol (namespace /octane, as in the public Octane example clients)
  ──────────────────────────────────────────────────────────────
    client → auth {"x-api-key": key}   server → auth_ok {} (key must match;
                                        a wrong key gets no reply and is
                                        never logged)
    client → join {"channel": name}    server adds the client to that channel
                                        (beacon_node joins beacon and
                                        v2x_obu_parsed)
    server → beacon_update             to channel beacon, one event per
                                        antenna per GNSS tick

  Payload = the published Octane schema (/octane/beacon_update):
    {"id", "serviced", "state": {"dynamics": {"latitude", "longitude",
     "elevation", "heading", "velocity", "updated"}}}
  heading is compass degrees, velocity is ground speed in KNOTS, as the real
  feed (measured on the shuttle 2026-10-08); beacon_node converts it back to
  m/s through octane_client.normalize(), so the conversion is exercised in sim
  too. beacon_payload() takes m/s and does the scaling. The timestamps
  carry *sim* time as ISO-8601 UTC (1970-01-01 + sim seconds, milliseconds
  like the real feed), because beacon_node stamps /gnss/fix with `updated`
  and the stack runs on /clock.

  Not emulated (beacon_node ignores all of them): v2x_BSM, acceleration, the
  server's join / channels replies, an auth error reply.

  Threading: python-socketio AsyncServer on aiohttp, in its own thread with
  its own asyncio loop. publish() is thread-safe and never blocks the caller
  (the bridge tick loop).

Usage:
  emu = OctaneEmulator(port=8765, api_key='sim', log=node.get_logger())
  emu.start()                                   # raises if the port is taken
  emu.publish([beacon_payload('SIM-FRONT', sim_ns, lat, lon, elev, hdg, v),
               beacon_payload('SIM-REAR', ...)])
  emu.stop()
"""

import asyncio
from datetime import datetime, timedelta, timezone
import logging
import threading

from aiohttp import web
from nift_sensor.octane_client import KNOTS_TO_MPS
import socketio

EPOCH = datetime(1970, 1, 1, tzinfo=timezone.utc)
BEACON_CHANNEL = 'beacon'


def sim_iso8601(sim_ns: int) -> str:
    """Sim time as Octane writes it: 123.45 s → '1970-01-01T00:02:03.450+00:00'."""
    return (EPOCH + timedelta(milliseconds=round(sim_ns / 1e6))).isoformat(timespec='milliseconds')


def beacon_payload(beacon_id: str, sim_ns: int, lat: float, lon: float, elevation: float,
                   heading_deg: float, speed_mps: float) -> dict:
    """One beacon_update event in the published Octane schema (speed_mps goes out in knots)."""
    ts = sim_iso8601(sim_ns)
    return {
        'id': beacon_id,
        'serviced': ts,
        'state': {'dynamics': {
            'latitude': lat,
            'longitude': lon,
            'elevation': elevation,
            'heading': heading_deg,
            'velocity': speed_mps / KNOTS_TO_MPS,
            'updated': ts,
        }},
    }


class OctaneEmulator:

    def __init__(self, port: int, api_key: str, namespace: str = '/octane',
                 host: str = '127.0.0.1', log=None):
        self.host = host
        self.port = port                # 0 = any free port; the bound one is set by start()
        self.namespace = namespace
        self._api_key = api_key
        self._log = log or logging.getLogger(__name__)
        self._authed = set()
        self.emitted = 0                # beacon_update events handed to socket.io

        self._sio = socketio.AsyncServer(async_mode='aiohttp', logger=False, engineio_logger=False)
        self._app = web.Application()
        self._sio.attach(self._app)
        for event, handler in (('connect', self._on_connect), ('auth', self._on_auth),
                               ('join', self._on_join), ('disconnect', self._on_disconnect)):
            self._sio.on(event, handler, namespace=namespace)

        self._loop = asyncio.new_event_loop()
        self._ready = threading.Event()
        self._error = None
        self._thread = threading.Thread(target=self._run, name='octane_emulator', daemon=True)

    # ------------------------------------------------------------------ #
    @property
    def clients_authed(self) -> int:
        return len(self._authed)

    def start(self, timeout_s: float = 5.0):
        """Start serving; returns once the port is bound, raises RuntimeError if it cannot be."""
        self._thread.start()
        if not self._ready.wait(timeout_s):
            raise RuntimeError(f'Octane emulator did not start within {timeout_s} s')
        if self._error is not None:
            raise RuntimeError(f'Octane emulator cannot listen on {self.host}:{self.port}: '
                               f'{self._error}')
        self._log.info(f'Octane emulator listening on http://{self.host}:{self.port} '
                       f'namespace={self.namespace}')

    def publish(self, payloads):
        """Emit beacon_update payloads to the beacon channel. Thread-safe, never blocks."""
        if not self._ready.is_set() or self._error is not None or not self._loop.is_running():
            return
        fut = asyncio.run_coroutine_threadsafe(self._emit(list(payloads)), self._loop)
        fut.add_done_callback(self._report)

    def stop(self, timeout_s: float = 5.0):
        """Disconnect every client and close the port."""
        if not self._thread.is_alive():
            return
        if self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._sio.shutdown(), self._loop).result(timeout_s)
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout_s)

    # ------------------------------------------------------------------ #
    def _run(self):
        asyncio.set_event_loop(self._loop)
        runner = web.AppRunner(self._app, handle_signals=False, access_log=None,
                               shutdown_timeout=1.0)
        try:
            self._loop.run_until_complete(runner.setup())
            site = web.TCPSite(runner, self.host, self.port)
            self._loop.run_until_complete(site.start())
            self.port = runner.addresses[0][1]
        except Exception as exc:        # port taken, bad host, …: start() reports it
            self._error = exc
            self._ready.set()
            self._loop.run_until_complete(runner.cleanup())
            self._loop.close()
            return
        self._ready.set()
        try:
            self._loop.run_forever()
        finally:
            self._loop.run_until_complete(runner.cleanup())
            pending = asyncio.all_tasks(self._loop)     # engine.io ping tasks of closed sockets
            for task in pending:
                task.cancel()
            self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self._loop.close()

    async def _emit(self, payloads):
        for p in payloads:
            await self._sio.emit('beacon_update', p, room=BEACON_CHANNEL, namespace=self.namespace)
            self.emitted += 1

    def _report(self, fut):
        if not fut.cancelled() and fut.exception() is not None:
            self._log.error(f'Octane emulator emit failed: {fut.exception()!r}')

    async def _on_connect(self, sid, environ, auth=None):
        self._log.info(f'Octane emulator: client {sid} connected')

    async def _on_auth(self, sid, data):
        key = data.get('x-api-key') if isinstance(data, dict) else None
        if key != self._api_key:
            self._log.warning(f'Octane emulator: auth rejected for {sid} (wrong key)')
            return
        self._authed.add(sid)
        await self._sio.emit('auth_ok', {}, to=sid, namespace=self.namespace)
        self._log.info(f'Octane emulator: client {sid} authenticated')

    async def _on_join(self, sid, data):
        channel = data.get('channel') if isinstance(data, dict) else None
        if sid not in self._authed or not channel:
            return
        await self._sio.enter_room(sid, channel, namespace=self.namespace)
        self._log.info(f'Octane emulator: client {sid} joined {channel}')

    async def _on_disconnect(self, sid, reason=None):
        self._authed.discard(sid)
        self._log.info(f'Octane emulator: client {sid} disconnected')
