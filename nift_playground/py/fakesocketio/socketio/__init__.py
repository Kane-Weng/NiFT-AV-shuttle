"""
python-socketio stand-in for the playground: `socketio.Client` on an in-process hub (ARCH section 5).

beacon_node and octane_client import `socketio` and use only the client. The Octane side is `playground.octane_bridge.BrowserOctane`, which listens on `socketio.hub` at the URL beacon_node is given. The server classes exist as names only and raise on use, because `octane_emulator.py` imports this module but the playground never builds its aiohttp server.
"""

from . import exceptions
from .client import Client
from .hub import hub, Hub, ManualScheduler, normalize_url, Server as HubServer

__version__ = '5.17.0+fakesocketio'     # the python-socketio this follows (nift_pub_deps:humble)


class _NotHere:
    """A python-socketio class the playground does not provide. Constructing it raises."""

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(
            f'fakesocketio: socketio.{type(self).__name__} is not available in the playground '
            '(use playground.octane_bridge.BrowserOctane for the Octane server)')


class AsyncServer(_NotHere):
    pass


class Server(_NotHere):
    pass


class AsyncClient(_NotHere):
    pass


class SimpleClient(_NotHere):
    pass


class AsyncSimpleClient(_NotHere):
    pass


class ASGIApp(_NotHere):
    pass


class WSGIApp(_NotHere):
    pass


__all__ = ['Client', 'AsyncClient', 'Server', 'AsyncServer', 'SimpleClient', 'AsyncSimpleClient',
           'ASGIApp', 'WSGIApp', 'exceptions', 'hub', 'Hub', 'HubServer', 'ManualScheduler', 'normalize_url']
