"""
The session: the Python end of the worker protocol (ARCH section 8).

    s = Session(repo_root)                  # '/nift' in Pyodide, the checkout natively
    s.handle(json_text) -> json_text        # one page message in, a list of worker messages out
    s.advance(n) -> json_text               # run n ticks, return what they produced

The worker's JavaScript loop owns wall-clock pacing (pause, speed). Native tests call these two methods with no pacing, so they speak exactly what the worker speaks. `handle_obj` and `advance_obj` do the same with dicts.

Every applied page message that changes the sim is logged with the tick it was applied after. `scenario_save` returns that log with the launch options, and `scenario_load` replays it from a reset, message by message at the same ticks.
"""

import copy
import json
import math
import platform
import sys
import time
import traceback

from . import paths

STATE_HZ = 10
SPEED_RANGE = (0.1, 100.0)


def _finite(obj):
    """Replace inf and nan (not JSON) by None, recursively."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _finite(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_finite(v) for v in obj]
    return obj


def dumps(msgs) -> str:
    try:
        return json.dumps(msgs, allow_nan=False, separators=(',', ':'))
    except ValueError:
        return json.dumps(_finite(msgs), separators=(',', ':'))


def box_polygon(box: dict) -> list:
    x, y, yaw = float(box['x']), float(box['y']), float(box.get('yaw', 0.0))
    hl, hw = float(box['length']) / 2.0, float(box['width']) / 2.0
    c, s = math.cos(yaw), math.sin(yaw)
    return [[x + c * dx - s * dy, y + s * dx + c * dy] for dx, dy in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


def centroid(poly) -> tuple:
    n = len(poly)
    return sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n


class CommandError(ValueError):
    """A page message that cannot be applied (the reply carries the text).

    `reported`: an `error` message with the details is already in the output, so without a reply id no second one is added."""

    def __init__(self, text, reported=False):
        super().__init__(text)
        self.reported = reported


class Session:

    def __init__(self, repo_root=None):
        self.root = paths.setup(repo_root)
        self.runner = None
        self.launch = None              # the LaunchSet in force
        self.options = None             # its options (ARCH 6.4)
        self.staged = {}                # set_param values for the next reset: {node: {name: value}}
        self.events = []                # applied page messages: [{tick, msg}]
        self.pending = []               # scenario events still to replay
        self.state_hz = STATE_HZ
        self.paused = False
        self.factor = 1.0
        self.rtf = 0.0
        self.obstacle_yaw = {}          # oid -> yaw the shape was given in
        self.record = False             # keep a Trace (CLI, tests)
        self.workdir = None             # this session's temp folder (a list of GNSS faults goes there as a file)

    # ================================================================ entry points
    def handle(self, text: str) -> str:
        return dumps(self.handle_obj(json.loads(text)))

    def advance(self, n: int) -> str:
        return dumps(self.advance_obj(n))

    def handle_obj(self, msg: dict) -> list:
        out = []
        mid = msg.get('id') if isinstance(msg, dict) else None
        try:
            if not isinstance(msg, dict) or 'type' not in msg:
                raise CommandError('expected an object with a type')
            fn = getattr(self, '_on_' + str(msg['type']), None)
            if fn is None:
                raise CommandError(f'unknown message type {msg["type"]!r}')
            if self.runner is None and msg['type'] not in ('init', 'scenario_load'):
                raise CommandError('not initialised: send init first')
            text = fn(msg, out)
            if mid is not None:
                out.append({'type': 'reply', 'id': mid, 'ok': True, 'message': text or ''})
        except CommandError as e:
            if mid is not None:
                out.append({'type': 'reply', 'id': mid, 'ok': False, 'message': str(e)})
            elif not e.reported:
                out.append({'type': 'error', 'message': str(e), 'traceback': '', 'fatal': False})
        except Exception as e:      # a Python error: the page shows it
            fatal = self.runner is None or msg.get('type') in ('init', 'reset', 'scenario_load')
            out.append({'type': 'error', 'message': f'{type(e).__name__}: {e}',
                        'traceback': traceback.format_exc(), 'fatal': fatal})
            if mid is not None:
                out.append({'type': 'reply', 'id': mid, 'ok': False, 'message': f'{type(e).__name__}: {e}'})
        return out

    def advance_obj(self, n: int) -> list:
        out = []
        if self.runner is None:
            return out
        try:
            self._run_ticks(int(n), out, state_at_end=False)
        except Exception as e:
            out += self.collect()
            out.append({'type': 'error', 'message': f'{type(e).__name__}: {e}',
                        'traceback': traceback.format_exc(), 'fatal': True})
        return out

    # ================================================================ build
    def _build(self, options, out, scenario_events=None, obstacles=None, restoring=False):
        """
        Build the stack for these options. The page's obstacles survive (they are the page's world).

        If the build fails (a node raises, for example on a parameter of the wrong type), the previous launch is built again, the error goes out with `fatal: false`, and the command is refused.
        """
        from . import launch_set as ls
        from .runner import Runner
        if self.workdir is None:
            import tempfile
            self.workdir = tempfile.mkdtemp(prefix='nift_playground_')
        try:
            launch = ls.build(options, workdir=self.workdir)
        except ls.LaunchError as e:         # bad options: the stack in force keeps running
            raise CommandError(str(e)) from None
        out.append({'type': 'progress', 'stage': 'nodes', 'pct': 90})
        runner = Runner(launch, record=self.record)
        old, prev_options = self.runner, self.options
        if obstacles is None:
            obstacles = dict(old.obstacles) if old is not None else {}
        if old is not None:
            old.teardown()
        self.runner = None
        try:
            runner.build()
        except Exception as e:
            text = f'{type(e).__name__}: {e}'
            try:
                runner.teardown()
            except Exception:
                pass
            if prev_options is None or restoring:
                raise
            out.append({'type': 'error', 'message': f'the new launch failed to start, the previous one is back: {text}',
                        'traceback': traceback.format_exc(), 'fatal': False})
            self.options = None
            self._build(prev_options, out, obstacles=obstacles, restoring=True)
            raise CommandError(f'{text} (the previous launch was restored)', reported=True) from None
        self.runner, self.launch, self.options = runner, launch, launch.options
        self.events, self.pending = [], list(scenario_events or [])
        if scenario_events is not None:
            self.obstacle_yaw = {}
        elif obstacles:
            # obstacles live in the page's world and survive a reset: log them, so a saved scenario has them
            msg = {'type': 'set_obstacles', 'obstacles': [{'oid': k, 'poly': [list(p) for p in v]}
                                                         for k, v in obstacles.items()]}
            self._apply(msg)
        self._replay_due(out)
        out.append(self._ready())
        out += self.collect()
        out.append(self.state())

    def _ready(self) -> dict:
        from . import launch_set as ls
        try:
            import pyodide
            pyv = getattr(pyodide, '__version__', None)
        except ImportError:
            pyv = None
        from .sim_backend import gnss_fault_presets
        presets = ls.presets()
        gf = gnss_fault_presets(ls.load_yaml(ls.GNSS_FAULTS))
        if isinstance(gf, dict):        # ARCH 12: one flat list, kind scheduled | sim | link
            gf = [dict(p, kind='scheduled') for p in gf.get('scenarios', [])] + list(gf.get('live', []))
        presets['gnss_faults'] = gf
        return {'type': 'ready', 'pyodide': pyv, 'python': platform.python_version(),
                'launch': self.launch.describe(), 'presets': presets,
                'dt': self.runner.dt_ns / 1e9, 'stateHz': self.state_hz}

    # ================================================================ ticks
    def _run_ticks(self, n, out, state_at_end=True):
        r = self.runner
        every = max(1, round(1.0 / (self.state_hz * r.dt_ns / 1e9))) if self.state_hz > 0 else 0
        w0, t0 = time.perf_counter(), r.now_ns
        for _ in range(max(0, n)):
            r.step()
            self._replay_due(out)
            if every and r.tick % every == 0 and not (state_at_end and _ == n - 1):
                out += self.collect()
                out.append(self.state())
        wall = time.perf_counter() - w0
        if n > 0 and wall > 0:
            self.rtf = (r.now_ns - t0) / 1e9 / wall
        out += self.collect()
        if state_at_end:
            out.append(self.state())

    def _replay_due(self, out):
        while self.pending and self.pending[0]['tick'] <= self.runner.tick:
            ev = self.pending.pop(0)
            try:
                self._apply(copy.deepcopy(ev['msg']))
            except CommandError as e:     # refused then too (e.g. a start without a path): it is in the log
                out.append({'type': 'log', 't': round(self.runner.t, 6), 'level': 'warn', 'node': 'playground',
                            'text': f'replay of tick {ev["tick"]} {ev["msg"].get("type")}: {e}'})

    def collect(self) -> list:
        """
        The worker messages queued since the last call: logs, the observer's messages (one diag per batch), obstacles.

        Public for the worker (web/src/worker/core.ts worker_state), which posts `collect() + [state()]` on demand.
        """
        r = self.runner
        out = [{'type': 'log', 't': t, 'level': lv, 'node': node, 'text': text} for t, lv, node, text in r.take_logs()]
        diag = None
        for m in r.observer.take():
            if m['type'] == 'diag':
                diag = m
            else:
                out.append(m)
        if diag is not None:
            out.append(diag)
        if r.obstacles_dirty:
            r.obstacles_dirty = False
            out.append(r.obstacles_message())
        return out

    def state(self) -> dict:
        """The `state` message now (ARCH 8.2). Public for the worker; no tick runs and nothing is logged."""
        r = self.runner
        st = {'type': 'state', 't': round(r.t, 6), 'tick': r.tick, 'paused': self.paused, 'factor': self.factor,
              'rtf': round(self.rtf, 2)}
        st.update(r.observer.snapshot())
        return st

    # ================================================================ applying page messages
    def _apply(self, msg) -> str:
        """Apply a sim-changing message at the current tick and log it (ARCH 8.5)."""
        r = self.runner
        kind = msg['type']
        text = ''
        if kind == 'goal':
            x, y = float(msg['x']), float(msg['y'])
            r.apply(r.operator.goal, x, y, msg.get('yaw'))
            text = f'goal ({x:.2f}, {y:.2f}) published on /goal_pose'
        elif kind == 'cmd':
            name = msg.get('name')
            if name == 'dbw_estop':
                r.apply(r.operator.dbw_estop)
                text = '0x100 E-stop frame sent on /to_can_bus'
            elif name in ('start', 'stop', 'estop', 'resume', 'manual', 'power_cycle'):
                ok, text = r.call_service(name)
                self._log_event(msg)
                if not ok:
                    raise CommandError(text)
                return text
            else:
                raise CommandError(f'cmd {name!r}: expected start, stop, estop, resume, manual, dbw_estop '
                                   'or power_cycle')
        elif kind == 'teleop':
            r.apply(r.operator.teleop, float(msg['v']), float(msg['steer']))
        elif kind == 'initialpose':
            r.apply(r.operator.initialpose, float(msg['x']), float(msg['y']), float(msg['yaw']))
            text = 'published on /initialpose'
        elif kind == 'obstacle_add':
            oid = self._oid(msg)
            if 'box' in msg:
                poly = box_polygon(msg['box'])
                self.obstacle_yaw[oid] = float(msg['box'].get('yaw', 0.0))
            elif 'poly' in msg and len(msg['poly']) >= 3:
                poly = msg['poly']
                self.obstacle_yaw[oid] = 0.0
            else:
                raise CommandError('obstacle_add needs poly (3 or more [x, y]) or box {x, y, yaw, length, width}')
            r.set_obstacle(oid, poly)
        elif kind == 'obstacle_move':
            oid = self._oid(msg)
            if oid not in r.obstacles:
                raise CommandError(f'no obstacle {oid!r}')
            poly = r.obstacles[oid]
            cx, cy = centroid(poly)
            yaw0 = self.obstacle_yaw.get(oid, 0.0)
            dyaw = float(msg['yaw']) - yaw0 if msg.get('yaw') is not None else 0.0
            c, s = math.cos(dyaw), math.sin(dyaw)
            x, y = float(msg['x']), float(msg['y'])
            r.set_obstacle(oid, [(x + c * (px - cx) - s * (py - cy), y + s * (px - cx) + c * (py - cy))
                                 for px, py in poly])
            self.obstacle_yaw[oid] = yaw0 + dyaw
        elif kind == 'obstacle_remove':
            oid = self._oid(msg)
            if oid not in r.obstacles:
                raise CommandError(f'no obstacle {oid!r}')
            r.remove_obstacle(oid)
            self.obstacle_yaw.pop(oid, None)
        elif kind == 'set_obstacles':
            r.obstacles.clear()
            for ob in msg.get('obstacles') or []:
                oid = self._oid(ob)
                r.obstacles[oid] = [(float(x), float(y)) for x, y in ob['poly']]
                self.obstacle_yaw[oid] = 0.0
            r.obstacles_changed()
        elif kind == 'gnss_fault':
            text = self._gnss_fault(msg)
        else:
            raise CommandError(f'{kind} is not a sim message')
        self._log_event(msg)
        return text

    @staticmethod
    def _oid(msg) -> str:
        """The obstacle id (ARCH 8.1: `oid`; `id` is the reply id of any page message)."""
        if msg.get('oid') in (None, ''):
            raise CommandError('obstacle messages need oid (the obstacle id)')
        return str(msg['oid'])

    def _log_event(self, msg):
        from .scenario import strip
        self.events.append({'tick': self.runner.tick, 'msg': strip(msg)})

    def _gnss_fault(self, msg) -> str:
        """
        ARCH 8.4: a live preset, applied by the sim backend at the current t (it starts at the next tick).

        Live link faults go to BrowserOctane, "now" sim faults become a GnssFault with t_s = now + dt. Scheduled faults of the catalogue are the launch option gnss_faults and need a reset.
        """
        from .sim_backend import SIM_PRESETS
        preset, params = msg.get('preset'), msg.get('params') or {}
        if not isinstance(params, dict):
            raise CommandError('gnss_fault: params must be an object')
        params = dict(params)
        if preset in SIM_PRESETS:
            unknown = sorted(set(params) - set(SIM_PRESETS[preset][0]))
            if unknown:
                raise CommandError(f'gnss_fault {preset}: unknown parameter(s) {unknown}; '
                                   f'known: {sorted(SIM_PRESETS[preset][0])}')
        r = self.runner
        try:
            return r.apply(r.backend.apply_gnss_fault, preset, params, r.now_ns)
        except (ValueError, TypeError) as e:
            raise CommandError(f'gnss_fault {preset}: {e}') from None

    # ================================================================ handlers
    def _on_init(self, msg, out):
        self.state_hz = float(msg.get('stateHz') or STATE_HZ)
        sc = msg.get('scenario')
        if sc:
            from . import launch_set as ls
            from .scenario import check, ScenarioError
            try:
                sc = check(sc)
                opts = ls.normalize_options(sc['launch'])
            except (ScenarioError, ls.LaunchError) as e:
                # a broken scenario (a bad URL hash): start without it and refuse it
                self._build(msg.get('launch'), out)
                raise CommandError(f'scenario refused, started without it: {e}') from None
            self._build(opts, out, scenario_events=sc['events'])
        else:
            self._build(msg.get('launch'), out)
        return 'ready'

    def _sim_msg(self, msg, out):
        text = self._apply(msg)
        out += self.collect()
        out.append(self.state())
        return text

    _on_goal = _on_cmd = _on_teleop = _on_initialpose = _sim_msg
    _on_obstacle_add = _on_obstacle_move = _on_obstacle_remove = _on_set_obstacles = _on_gnss_fault = _sim_msg

    def _on_pause(self, msg, out):
        self.paused = True
        out.append(self.state())
        return 'paused'

    def _on_resume_sim(self, msg, out):
        self.paused = False
        out.append(self.state())
        return 'running'

    def _on_step(self, msg, out):
        n = int(msg.get('n') or 1)
        if n < 1:
            raise CommandError('step: n must be 1 or more')
        self._run_ticks(n, out, state_at_end=True)
        return f'{n} tick(s)'

    def _on_speed(self, msg, out):
        f = msg.get('factor')
        if f == 'max':
            self.factor = 'max'
        else:
            try:
                f = float(f)
            except (TypeError, ValueError):
                raise CommandError("speed: factor must be a number or 'max'") from None
            if not SPEED_RANGE[0] <= f <= SPEED_RANGE[1]:
                raise CommandError(f'speed: factor must be within {SPEED_RANGE[0]} and {SPEED_RANGE[1]}, or max')
            self.factor = f
        out.append(self.state())
        return f'speed {self.factor}'

    def _on_set_param(self, msg, out):
        from .launch_set import PAGE_PARAMS
        node, name, value = str(msg.get('node')), str(msg.get('name')), msg.get('value')
        if (node, name) not in PAGE_PARAMS:
            raise CommandError(f'set_param: {node} {name} is not offered; the page may set '
                               f'{", ".join(n for _, n in PAGE_PARAMS)} of trajectory_controller')
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise CommandError(f'set_param: {name} takes a number, not {value!r}')
        lo, hi = PAGE_PARAMS[(node, name)]
        if not lo <= value <= hi:
            raise CommandError(f'set_param: {name} must be within {lo:g} and {hi:g}')
        value = float(value)            # every offered parameter is a double: an int would be the wrong type
        live = self.runner.nodes.get(node)
        if live is not None and live.has_parameter(name):
            declared = live.get_parameter(name).value
            if isinstance(declared, bool) or not isinstance(declared, float):
                raise CommandError(f'set_param: {node} declares {name} as {type(declared).__name__}')
        self.staged.setdefault(node, {})[name] = value
        return (f'{node} {name} = {value:g} staged: the nodes read parameters only at start-up, '
                'so it takes effect on the next reset')

    def _on_reset(self, msg, out):
        from . import launch_set as ls
        try:
            opts = ls.normalize_options(msg.get('launch'), base=self.options)
        except ls.LaunchError as e:
            raise CommandError(str(e)) from None
        if self.staged:
            params = copy.deepcopy(opts.get('params') or {})
            for node, values in self.staged.items():
                params.setdefault(node, {}).update(values)
            opts['params'] = params
        self.staged = {}                # applied now, or dropped with the launch that could not start
        self._build(opts, out)
        return 'reset'

    def _on_scenario_save(self, msg, out):
        from .scenario import make
        sc = make(copy.deepcopy(self.options), copy.deepcopy(self.events), msg.get('note'))
        out.append({'type': 'scenario', 'scenario': sc})
        return f'{len(sc["events"])} event(s) up to tick {self.runner.tick}'

    def _on_scenario_load(self, msg, out):
        from . import launch_set as ls
        from .scenario import check, ScenarioError
        try:
            sc = check(msg.get('scenario'))
            opts = ls.normalize_options(sc['launch'])
        except (ScenarioError, ls.LaunchError) as e:
            raise CommandError(f'scenario refused: {e}') from None
        self._build(opts, out, scenario_events=sc['events'], obstacles={})
        return f'replaying {len(sc["events"])} event(s)'

    # ================================================================ native helpers
    def scenario(self, note=None) -> dict:
        from .scenario import make
        return make(copy.deepcopy(self.options), copy.deepcopy(self.events), note)


def python_info() -> str:
    return f'{platform.python_implementation()} {sys.version.split()[0]}'
