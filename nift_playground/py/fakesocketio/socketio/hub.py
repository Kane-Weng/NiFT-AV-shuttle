"""
The in-process socket.io "network": a hub of servers keyed by URL, and the server end that BrowserOctane uses.

There are no sockets and no threads. A server registers at a URL (`hub.listen`). A `socketio.Client` that connects to that URL is attached to it. Every delivery in either direction (the connect handshake, an emit, a server disconnect) is handed to the hub's scheduler, which by default is the fakeros world's callback queue (`world.call_soon(fn, *args)`). So a delivery runs after the current callback returns, in order, like a ROS message. Nothing is ever delivered inside the call that emits it.

Handler exceptions are caught and logged, as python-socketio 5.x does in its read loop (`engineio.client` logger, "<event> handler error"), so one bad handler does not stop the chain. They are also kept in `hub.errors`, which tests check.

The server end mirrors the parts of python-socketio's `Server` that `octane_emulator.py` uses (`on`, `emit` with `to` or `room`, `enter_room`, `disconnect`, `shutdown`), but synchronous.
"""

import collections
import logging
import traceback
from urllib.parse import urlsplit

from .exceptions import ConnectionError  # noqa: A004

CLIENT_LOGGER = logging.getLogger('engineio.client')
SERVER_LOGGER = logging.getLogger('socketio.server')


class reason:  # noqa: N801 (python-socketio's name: Client.reason)
    """Disconnect reasons, as engineio.Client.reason in python-socketio 5.x."""

    CLIENT_DISCONNECT = 'client disconnect'
    SERVER_DISCONNECT = 'server disconnect'
    TRANSPORT_ERROR = 'transport error'


def normalize_url(url: str) -> str:
    """'http://localhost:8765/' and 'http://127.0.0.1:8765' are the same server."""
    parts = urlsplit(str(url).strip())
    scheme = (parts.scheme or 'http').lower()
    host = (parts.hostname or '').lower()
    if host == 'localhost':
        host = '127.0.0.1'
    try:
        port = parts.port
    except ValueError:
        port = None
    if port is None:
        port = 443 if scheme in ('https', 'wss') else 80
    return f'{scheme}://{host}:{port}'


def _fakeros_call_soon(fn, *args):
    """The default scheduler: the fakeros world's callback queue."""
    from fakeros._core import world
    world.call_soon(fn, *args)


class ManualScheduler:
    """A FIFO for tests without fakeros: `hub.scheduler = s`, then `s.run()` delivers everything."""

    def __init__(self):
        self.queue = collections.deque()

    def __call__(self, fn, *args):
        self.queue.append((fn, args))

    def run(self, limit: int = 100000) -> int:
        n = 0
        while self.queue:
            fn, args = self.queue.popleft()
            fn(*args)
            n += 1
            if n >= limit:
                raise RuntimeError('ManualScheduler: delivery loop did not settle')
        return n


def _args(data):
    """python-socketio's rule: a tuple is several arguments, None is none, anything else is one."""
    if isinstance(data, tuple):
        return list(data)
    if data is None:
        return []
    return [data]


class Hub:
    """All servers of the process, keyed by normalized URL."""

    def __init__(self):
        self.scheduler = None
        self.reset()

    def reset(self):
        """Forget every server and error, restart the sid counter. Call it when the world resets."""
        self.servers = {}
        self.errors = []
        self._next_sid = 0
        self.delivered = 0

    # ------------------------------------------------------------------ #
    def defer(self, fn, *args):
        (self.scheduler or _fakeros_call_soon)(self._deliver, fn, args)

    def _deliver(self, fn, args):
        self.delivered += 1
        fn(*args)

    def run_handler(self, side: str, event: str, handler, args):
        """Call a handler; log and keep an exception the way python-socketio does."""
        try:
            return handler(*args)
        except TypeError:
            # python-socketio retries a disconnect handler without the reason (legacy signature)
            if event == 'disconnect' and args:
                try:
                    return handler(*args[:-1])
                except Exception:
                    self._error(side, event)
                    return None
            self._error(side, event)
        except Exception:
            self._error(side, event)
        return None

    def _error(self, side, event):
        label = 'disconnect' if event == 'disconnect' else 'message'
        text = traceback.format_exc()
        self.errors.append({'side': side, 'event': event, 'traceback': text})
        logger = CLIENT_LOGGER if side == 'client' else SERVER_LOGGER
        logger.error(f'{label} handler error ({event})\n{text}')

    def new_sid(self) -> str:
        self._next_sid += 1
        return f'fakesio{self._next_sid:013d}'

    # ------------------------------------------------------------------ #
    def listen(self, url: str) -> 'Server':
        """Register a server at url. RuntimeError if one is already there (the port is taken)."""
        key = normalize_url(url)
        if key in self.servers:
            raise RuntimeError(f'{key}: address already in use')
        server = Server(self, key)
        self.servers[key] = server
        return server

    def free_port(self, host: str = '127.0.0.1', start: int = 40000) -> int:
        """The first free port from start on (for port 0, 'any free port')."""
        port = start
        while normalize_url(f'http://{host}:{port}') in self.servers:
            port += 1
        return port

    def lookup(self, url: str) -> 'Server':
        server = self.servers.get(normalize_url(url))
        if server is None:
            # What python-socketio raises on the first attempt when nothing listens
            raise ConnectionError('Connection error')
        return server


hub = Hub()


class Server:
    """The server end at one URL (a synchronous python-socketio Server for the parts octane_emulator uses)."""

    def __init__(self, owner: Hub, url: str):
        self.hub = owner
        self.url = url
        self.handlers = collections.defaultdict(dict)      # namespace -> event -> handler
        self.clients = {}                                   # (sid, namespace) -> Client
        self.rooms = collections.defaultdict(set)           # (namespace, room) -> sids
        self.open = True

    def on(self, event, handler=None, namespace=None):
        namespace = namespace or '/'

        def set_handler(fn):
            self.handlers[namespace][event] = fn
            return fn

        if handler is None:
            return set_handler
        set_handler(handler)
        return None

    def _trigger(self, namespace, event, *args):
        handler = self.handlers.get(namespace, {}).get(event)
        if handler is None:
            return None
        return self.hub.run_handler('server', event, handler, list(args))

    # ---- client side calls these ---- #
    def _attach(self, client, namespaces, auth):
        """Synchronous part of the handshake. Returns {namespace: sid}."""
        if not self.open:
            raise ConnectionError('Connection error')
        missing = [n for n in namespaces if n not in self.handlers]
        if missing:
            raise ConnectionError('One or more namespaces failed to connect: ' + ', '.join(missing))
        sids = {}
        for ns in namespaces:
            sid = self.hub.new_sid()
            self.clients[(sid, ns)] = client
            self.rooms[(ns, None)].add(sid)
            sids[ns] = sid
            self.hub.defer(self._handshake, sid, ns, auth, client._epoch)
        return sids

    def _handshake(self, sid, ns, auth, epoch):
        if (sid, ns) not in self.clients:
            return
        environ = {'REMOTE_ADDR': '127.0.0.1', 'HTTP_HOST': self.url.split('://', 1)[1],
                   'PATH_INFO': '/socket.io/', 'fakesocketio': True}
        handler = self.handlers.get(ns, {}).get('connect')
        ok = True
        if handler is not None:
            try:
                try:
                    r = handler(sid, environ, auth)
                except TypeError:
                    r = handler(sid, environ)       # python-socketio's legacy connect signature
                ok = r is not False
            except Exception as exc:
                from .exceptions import ConnectionRefusedError  # noqa: A004
                if not isinstance(exc, ConnectionRefusedError):
                    self.hub._error('server', 'connect')
                ok = False
                refused = getattr(exc, 'error_args', {'message': 'Connection rejected by server'})
            else:
                refused = {'message': 'Connection rejected by server'}
        client = self.clients[(sid, ns)]
        if not ok:
            self._drop(sid, ns)
            self.hub.defer(client._refused, ns, refused, epoch)
            return
        self.hub.defer(client._connected, ns, epoch)

    def _from_client(self, sid, ns, event, args):
        if (sid, ns) not in self.clients:
            return
        self._trigger(ns, event, sid, *args)

    def _client_left(self, sid, ns, why):
        if (sid, ns) not in self.clients:
            return
        self._drop(sid, ns)
        self._trigger(ns, 'disconnect', sid, why)

    def _drop(self, sid, ns):
        self.clients.pop((sid, ns), None)
        for (room_ns, _room), members in self.rooms.items():
            if room_ns == ns:
                members.discard(sid)

    # ---- the python-socketio Server API ---- #
    def emit(self, event, data=None, to=None, room=None, skip_sid=None, namespace=None, callback=None):
        """Send to one sid (to), a room, or every client of the namespace. Delivered later, never here."""
        ns = namespace or '/'
        target = to if to is not None else room
        if target is None:
            sids = sorted(self.rooms.get((ns, None), ()))
        elif (target, ns) in self.clients:
            sids = [target]
        else:
            sids = sorted(self.rooms.get((ns, target), ()))
        skip = set(skip_sid if isinstance(skip_sid, (list, tuple, set)) else [skip_sid])
        n = 0
        for sid in sids:
            if sid in skip:
                continue
            client = self.clients.get((sid, ns))
            if client is not None:
                self.hub.defer(client._receive, ns, event, _args(data), client._epoch)
                n += 1
        return n

    def enter_room(self, sid, room, namespace=None):
        ns = namespace or '/'
        if (sid, ns) in self.clients:
            self.rooms[(ns, room)].add(sid)

    def leave_room(self, sid, room, namespace=None):
        self.rooms.get((namespace or '/', room), set()).discard(sid)

    def rooms_of(self, sid, namespace=None):
        ns = namespace or '/'
        return sorted(str(r) for (n, r), m in self.rooms.items() if n == ns and r is not None and sid in m)

    def disconnect(self, sid, namespace=None):
        ns = namespace or '/'
        client = self.clients.get((sid, ns))
        if client is None:
            return
        self._drop(sid, ns)
        self._trigger(ns, 'disconnect', sid, reason.SERVER_DISCONNECT)
        self.hub.defer(client._server_disconnect, ns, client._epoch)

    def shutdown(self):
        """Disconnect every client and stop listening (the URL is free again)."""
        for sid, ns in sorted(self.clients):
            self.disconnect(sid, ns)
        self.open = False
        if self.hub.servers.get(self.url) is self:
            del self.hub.servers[self.url]

    @property
    def sids(self):
        return sorted({sid for sid, _ns in self.clients})
