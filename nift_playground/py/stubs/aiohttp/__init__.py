"""
aiohttp stand-in (ARCH section 5). `octane_emulator.py` runs `from aiohttp import web` at import time, and the playground never builds that server (BrowserOctane replaces OctaneEmulator). So the import works and any use raises NotImplementedError.
"""

from . import web  # noqa: F401


def __getattr__(name):
    if name.startswith('__'):
        raise AttributeError(name)
    raise NotImplementedError(f'fakes: aiohttp.{name} is not available in the playground')
