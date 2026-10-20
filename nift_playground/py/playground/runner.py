"""
The deterministic runner: one process, one sim clock, one thread (ARCH section 7).

`Runner(launch_set).build()` constructs the nodes in the launch's order of readiness on a fresh fakeros world. `step()` runs one tick:

  1. world clock = t = previous + dt
  2. the sim node's `_tick_once()` (ECU, plant, bicycle, GNSS to BrowserOctane, odom, /clock); what it sends is held
  3. fire the timers due at or before t (rcl's catch-up rule), with nothing delivered in between
  4. drain: deliver everything, the sim's messages and socket.io events first, then the timers' output, cascading
  5. the observer has seen everything

This is the ROS order (fakeros World.step): on ROS each node is its own process, so when /clock reaches t every due timer fires on what had already arrived. The controller acts on the GNSS fix of t - 2 dt, as in the reference runs (tests/validation/README.md).

Commands from the page run between ticks, at the current t, through `apply()`, and are followed by a drain. Nothing here reads the wall clock except the real-time factor the page shows.
"""

import importlib
import os
import time

START_NS = 1_000_000_000        # kinematic_sim_node.START_NS: sim time of the first tick
LEVEL_NAMES = {10: 'debug', 20: 'info', 30: 'warn', 40: 'error', 50: 'fatal'}
LEVELS = {'debug': 10, 'info': 20, 'warn': 30, 'warning': 30, 'error': 40, 'fatal': 50}


class RunnerError(RuntimeError):
    pass


# ================================================================ fakeros and the sim node
#
# fakeros owns the world: reset(start_ns), step(t, pre), configure_node(own, name=, parameters=[...]).
# playground.sim_backend (sim-gnss agent) constructs the repo's KinematicSimNode with BrowserOctane in place of
# the aiohttp Octane server and ticks it (ARCH 2.1). It also resets the socket.io hub, so every build starts on
# a fresh in-process network.

def fakeros_source(src):
    """A launch_set.Source as fakeros takes it: a path, a launch dict, or a node-scoped ParamsFile."""
    raw = src.raw
    if isinstance(raw, tuple) and raw and raw[0] == 'scoped':
        import fakeros
        return fakeros.node_scoped(raw[1], raw[2], src.label)
    return raw


# ================================================================ runner

class Runner:

    def __init__(self, launch, log_level='info', record=False):
        self.launch = launch
        self.log_level = LEVELS[log_level] if isinstance(log_level, str) else int(log_level)
        self.record = record
        import fakeros
        self.world = fakeros.world
        self.nodes = {}                 # launch name -> node
        self.logs = []                  # (t, level, node, text), taken by the session
        self.obstacles = {}             # oid -> [(x, y)] map frame
        self.obstacles_dirty = False
        self.tick = 0
        self.dt_ns = None
        self.sim = None
        self.backend = None
        self.observer = None
        self.operator = None
        self.auth_ok = False
        self.wall_s = 0.0               # wall time spent in step() (for the real-time factor only)
        self._remap = launch.remaps

    # ------------------------------------------------------------ build
    @property
    def now_ns(self) -> int:
        return self.world.now_ns

    @property
    def t(self) -> float:
        """Page time: sim time minus the first tick's 1 s."""
        return (self.world.now_ns - START_NS) / 1e9

    def _log_sink(self, lv, name, text):
        """fakeros' log sink: (level 10..50, logger name, text)."""
        level = LEVEL_NAMES.get(lv, lv) if isinstance(lv, int) else str(lv).lower()
        level = 'warn' if level == 'warning' else level
        node = self._remap.get(name, name)
        t = round(self.t, 6)
        if LEVELS.get(level, 20) >= self.log_level:
            self.logs.append((t, level, node, text))
        if node == 'beacon_node' and 'auth_ok' in text:
            self.auth_ok = True
        if self.observer is not None:
            self.observer.on_log(t, level, node, text)

    def build(self):
        """Fresh world, then the nodes in the launch's order (ARCH 6.1)."""
        self.teardown()
        w = self.world
        dt_s = float(self.launch.node('kinematic_sim').params.get('fixed_delta_seconds', 0.05))
        self.dt_ns = round(dt_s * 1e9)
        w.reset(START_NS - self.dt_ns)
        w.log_sink, w.log_level = self._log_sink, 10
        for spec in self.launch.nodes:
            # what a launch Node action gives: the launch name and the raw parameter sources (fakeros merges
            # them with rcl's YAML typing and launch_ros's dict rules, ARCH 6)
            w.configure_node(spec.own_name, name=spec.name, parameters=[fakeros_source(s) for s in spec.sources])

        for spec in self.launch.nodes:
            if spec.after_auth and not self.auth_ok:
                self._drain_until_auth()
            self.nodes[spec.name] = self._construct(spec)
        self.sim = self.backend.node
        self.observer = self.nodes['playground_observer']
        self.operator = self.nodes['playground_operator']
        if self.record:
            from .observer import Trace
            self.observer.trace = Trace()
        assert self.sim._sim_ns == w.now_ns, (self.sim._sim_ns, w.now_ns)
        assert round(self.sim.dt * 1e9) == self.dt_ns
        w.drain()
        return self

    def _construct(self, spec):
        if spec.name == 'kinematic_sim':
            from .sim_backend import SimBackend
            self.backend = SimBackend()
            return self.backend.node
        cls = getattr(importlib.import_module(spec.module), spec.cls)
        if not spec.repo:
            return cls(runner=self)
        # the node's environment (beacon_node: MCITY_OCTANE_*) only while it is built, which is when it reads it
        saved = {k: os.environ.get(k) for k in spec.env}
        os.environ.update(spec.env)
        try:
            return cls()
        finally:
            for k, v in saved.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v

    def _drain_until_auth(self):
        """bringup starts planning and control once beacon_node logs auth_ok. Here no sim time passes."""
        self.world.drain()
        if not self.auth_ok:
            raise RunnerError('beacon_node logged no auth_ok after the Octane handshake '
                              '(is the socket.io stand-in feeding the world queue?)')

    def teardown(self):
        """Stop what holds resources outside the world (the Octane hub entry, the socket.io client)."""
        if self.backend is not None:
            try:
                self.backend.close()
            except Exception:
                pass
        for name, node in list(self.nodes.items()):
            try:
                if name == 'beacon_node' and getattr(node, 'sio', None) is not None and node.sio.connected:
                    node.sio.disconnect()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        self.nodes = {}
        self.sim = self.observer = self.operator = self.backend = None
        self.auth_ok = False
        self.tick = 0
        self.wall_s = 0.0
        self.logs = []

    # ------------------------------------------------------------ run
    def step(self):
        """One tick (ARCH 7)."""
        w0 = time.perf_counter()
        w = self.world
        t = w.now_ns + self.dt_ns
        # 1 clock, 2 the sim (asserts its _sim_ns == t, its output held), 3 timers, 4 drain (ROS order)
        w.step(t, lambda: self.backend.tick(t))
        self.tick += 1                                  # 5 the observer has seen everything
        self.wall_s += time.perf_counter() - w0

    def apply(self, fn, *args, **kwargs):
        """Run a page command at the current t, then deliver what it sent."""
        result = fn(*args, **kwargs)
        self.world.drain()
        return result

    def call_service(self, name):
        return self.apply(self.operator.call, name, self.world.drain)

    # ------------------------------------------------------------ obstacles
    def set_obstacle(self, oid, poly):
        self.obstacles[str(oid)] = [(float(x), float(y)) for x, y in poly]
        self.obstacles_changed()

    def remove_obstacle(self, oid):
        if self.obstacles.pop(str(oid), None) is not None:
            self.obstacles_changed()

    def obstacles_changed(self):
        self.obstacles_dirty = True

    def obstacles_message(self) -> dict:
        perceived = self.observer.perceived if 'playground_perception' in self.nodes else None
        return {'type': 'obstacles',
                'polygons': [{'oid': k, 'poly': [list(p) for p in v]} for k, v in self.obstacles.items()],
                'perceived': perceived}

    # ------------------------------------------------------------ outputs
    def take_logs(self) -> list:
        out, self.logs = self.logs, []
        return out
