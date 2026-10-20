"""aiohttp.web stand-in: importable, and every name raises NotImplementedError on use (see the package docstring)."""


def __getattr__(name):
    if name.startswith('__'):
        raise AttributeError(name)
    raise NotImplementedError(
        f'aiohttp.web.{name}: no aiohttp in the playground (BrowserOctane serves Octane instead)')
