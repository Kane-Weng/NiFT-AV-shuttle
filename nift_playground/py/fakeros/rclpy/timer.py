from fakeros._node import Timer  # noqa: F401


class Rate:
    def __init__(self, *a, **kw):
        raise RuntimeError('fakeros: Rate sleeps, and nothing may sleep here')
