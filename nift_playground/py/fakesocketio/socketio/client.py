"""
socketio.Client as python-socketio 5.x has it, for the calls beacon_node and octane_client make.

Connected through the in-process hub (hub.py) instead of a network. The differences that matter:

- `connect()` returns at once with `connected` True and the namespaces set. The server's connect handler and then the client's `connect` handler run later, from the scheduler, in that order. The real client runs its `connect` handler in its read thread before `connect(wait=True)` returns; here it runs after the current callback.
- `connect()` to a URL where nothing listens raises `socketio.exceptions.ConnectionError`, as the real client does on its first attempt.
- A server disconnect (the server's `shutdown()` or `disconnect(sid)`) calls the `disconnect` handler with `reason.SERVER_DISCONNECT` and does not reconnect, as python-socketio does for a socket.io level disconnect. Transport loss and the reconnect loop are not modelled, so `reconnection*` are kept but unused.
- `disconnect()` calls the `disconnect` handler at once with `reason.CLIENT_DISCONNECT`.
- Handlers are called with python-socketio's rules, including the retry of a disconnect handler without the reason argument when it takes none (beacon_node's does). An exception in a handler is logged and kept in `hub.errors`, and delivery goes on.
"""

from . import exceptions
from .hub import hub as _default_hub
from .hub import _args, reason as _reason


class Client:
    reason = _reason

    def __init__(self, reconnection=True, reconnection_attempts=0, reconnection_delay=1,
                 reconnection_delay_max=5, randomization_factor=0.5, logger=False, serializer='default',
                 json=None, handle_sigint=True, **kwargs):
        self.reconnection = reconnection
        self.reconnection_attempts = reconnection_attempts
        self.reconnection_delay = reconnection_delay
        self.reconnection_delay_max = reconnection_delay_max
        self.randomization_factor = randomization_factor
        self.handlers = {}                  # namespace -> event -> handler
        self.namespace_handlers = {}
        self.namespaces = {}                # namespace -> sid
        self.failed_namespaces = []
        self.connected = False
        self.connection_url = None
        self.connection_namespaces = []
        self.sid = None
        self.hub = kwargs.get('hub') or _default_hub
        self._server = None
        self._epoch = 0                     # one per connect(): drops deliveries of an old connection

    # ------------------------------------------------------------------ #
    #  Handlers                                                          #
    # ------------------------------------------------------------------ #
    def on(self, event, handler=None, namespace=None):
        namespace = namespace or '/'

        def set_handler(fn):
            self.handlers.setdefault(namespace, {})[event] = fn
            return fn

        if handler is None:
            return set_handler
        set_handler(handler)
        return None

    def event(self, *args, **kwargs):
        """@sio.event: the function name is the event name."""
        if len(args) == 1 and not kwargs and callable(args[0]):
            return self.on(args[0].__name__)(args[0])

        def set_handler(fn):
            return self.on(fn.__name__, **kwargs)(fn)
        return set_handler

    def register_namespace(self, namespace_handler):
        raise NotImplementedError('fakesocketio: class-based namespaces are not supported')

    def _trigger(self, namespace, event, *args):
        handler = self.handlers.get(namespace, {}).get(event)
        if handler is None:
            return None
        return self.hub.run_handler('client', event, handler, list(args))

    # ------------------------------------------------------------------ #
    #  Connection                                                        #
    # ------------------------------------------------------------------ #
    def connect(self, url, headers={}, auth=None, transports=None, namespaces=None,  # noqa: B006 (as python-socketio)
                socketio_path='socket.io', wait=True, wait_timeout=1, retry=False):
        if self.connected:
            raise exceptions.ConnectionError('Already connected')
        if namespaces is None:
            namespaces = sorted(self.handlers) or ['/']
        elif isinstance(namespaces, str):
            namespaces = [namespaces]
        self.connection_url = url
        self.connection_namespaces = list(namespaces)
        self.namespaces = {}
        self.failed_namespaces = []
        try:
            server = self.hub.lookup(url)
        except exceptions.ConnectionError as exc:
            for ns in self.connection_namespaces:
                self._trigger(ns, 'connect_error', exc.args[0])
            raise
        self._epoch += 1
        self.namespaces = server._attach(self, self.connection_namespaces, auth)
        self._server = server
        self.sid = next(iter(self.namespaces.values()), None)
        self.connected = True

    def disconnect(self):
        if not self.connected:
            return
        server, namespaces = self._server, list(self.namespaces.items())
        self.namespaces = {}
        self.connected = False
        self.sid = None
        self._server = None
        self._epoch += 1
        for ns, sid in namespaces:
            self.hub.defer(server._client_left, sid, ns, _reason.CLIENT_DISCONNECT)
            self._trigger(ns, 'disconnect', _reason.CLIENT_DISCONNECT)
            self._trigger(ns, '__disconnect_final')

    def shutdown(self):
        self.disconnect()

    def get_sid(self, namespace=None):
        return self.namespaces.get(namespace or '/')

    def transport(self):
        return 'websocket'

    # ------------------------------------------------------------------ #
    #  Messages                                                          #
    # ------------------------------------------------------------------ #
    def emit(self, event, data=None, namespace=None, callback=None):
        namespace = namespace or '/'
        if namespace not in self.namespaces:
            raise exceptions.BadNamespaceError(namespace + ' is not a connected namespace.')
        if callback is not None:
            raise NotImplementedError('fakesocketio: emit callbacks (acks) are not supported')
        self.hub.defer(self._server._from_client, self.namespaces[namespace], namespace, event, _args(data))

    def send(self, data, namespace=None, callback=None):
        self.emit('message', data=data, namespace=namespace, callback=callback)

    def call(self, *args, **kwargs):
        raise NotImplementedError('fakesocketio: call() would block; nothing in the stack uses it')

    def wait(self):
        raise NotImplementedError('fakesocketio: wait() would block; the runner drives deliveries')

    def sleep(self, seconds=0):
        """No wall clock here: returns at once."""
        return None

    def start_background_task(self, target, *args, **kwargs):
        raise NotImplementedError('fakesocketio: no background tasks (no threads in Pyodide)')

    # ------------------------------------------------------------------ #
    #  Called by the server end, through the scheduler                   #
    # ------------------------------------------------------------------ #
    def _connected(self, ns, epoch):
        if epoch == self._epoch and ns in self.namespaces:
            self._trigger(ns, 'connect')

    def _refused(self, ns, data, epoch):
        if epoch != self._epoch or ns not in self.namespaces:
            return
        del self.namespaces[ns]
        self.failed_namespaces.append(ns)
        self._trigger(ns, 'connect_error', data)
        if not self.namespaces:
            self.connected = False
            self._server = None
            self.sid = None

    def _receive(self, ns, event, args, epoch):
        if epoch == self._epoch and ns in self.namespaces:
            self._trigger(ns, event, *args)

    def _server_disconnect(self, ns, epoch):
        if epoch != self._epoch or ns not in self.namespaces:
            return
        del self.namespaces[ns]
        self._trigger(ns, 'disconnect', _reason.SERVER_DISCONNECT)
        self._trigger(ns, '__disconnect_final')
        if not self.namespaces:
            self.connected = False
            self._server = None
            self.sid = None
