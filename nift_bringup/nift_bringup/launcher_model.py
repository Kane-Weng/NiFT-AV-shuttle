"""
State and rules behind the launcher window, without Qt.

File: launcher_model.py

Description:
  The state behind the launcher window (launcher.py), without Qt: a preset from profiles.yaml, the
  mode, the components switched on, and the launch arguments typed in. Whether something may change
  is always asked of profiles.resolve(); REQUIRES is only used to say what is missing and to find
  what has to go off together with a component. Tested in test/test_launcher_model.py.

  Node states on the map
    ON         in the plan
    LOCKED     can in sim: the sim backend (CARLA bridge or kinematic sim) is the vehicle, always
               on
    AVAILABLE  off, and switching it on alone resolves (the map pulses it)
    BLOCKED    off, and needs something else first (or is sim only in real mode)

  Clicks
    AVAILABLE -> on.  ON -> off, with everything that needed it.  BLOCKED -> nothing changes; the
    Change says what it needs.  turn_on_with_prereqs() switches on a node and everything it needs
    (the first alternative of an "any one of": planning before static_path, localization before
    filter). A click that changes the localization mode (localization alone = direct, filter alone
    = ekf, both = shadow) says so (not for direct from nothing: today's default).
  A change that breaks a rule the current selection does not already break is refused; with valid
  arguments that is exactly "resolve() fails".

  The command is the preset plus only what differs from it: mode, component switches, and the
  arguments that were typed in (or set by set_backend(), which picks the sim backend; kinematic has
  no LiDAR, so it switches perception off).

Usage:
  from nift_bringup.launcher_model import LauncherModel
  m = LauncherModel(load(path), 'none')
  m.click('gnss'); m.plan(); m.command()
"""

from dataclasses import dataclass, field
from enum import Enum

import yaml

from .components.common import share
from .profiles import (
    ARGS,
    COMPONENTS,
    localization_mode,
    ProfileError,
    REQUIRES,
    resolve,
    SIM_ONLY,
)

LAUNCH = ['ros2', 'launch', 'nift_bringup', 'system.launch.py']
# the localization modes in a few words (side panel, status line)
MODE_NOTES = {'direct': 'shuttle_tf_node publishes map → base_link',
              'ekf': 'the filter publishes map → base_link; the controller follows its status',
              'shadow': 'shuttle_tf_node publishes map → base_link; the filter runs beside it, '
                        'the controller '
                        'ignores it'}


class NodeState(Enum):
    ON = 'on'
    LOCKED = 'locked'
    AVAILABLE = 'available'
    BLOCKED = 'blocked'


@dataclass
class Change:
    """What a click did, for the status line and the map."""

    message: str
    turned_on: list = field(default_factory=list)
    turned_off: list = field(default_factory=list)
    # what a blocked node needs, to point at on the map
    highlight: list = field(default_factory=list)


def alternatives(need):
    """Return a REQUIRES entry as a tuple: 'gnss' -> ('gnss',); a tuple stays."""
    return need if isinstance(need, tuple) else (need,)


def rule_errors(e: ProfileError) -> list:
    """
    Return the broken rules in a resolve() error.

    The error is 'profile <name> (<mode>):', then one indented line each.
    """
    lines = str(e).split('\n')
    if len(lines) > 1 and lines[0].startswith('profile '):
        return [line.strip() for line in lines[1:]]
    return [str(e)]


def in_order(names) -> list:
    return [c for c in COMPONENTS if c in names]


class LauncherModel:

    def __init__(self, profiles: dict, preset: str = 'none'):
        self.profiles = profiles
        # '' = not given: the preset's value, else the default
        self.args = {k: '' for k in ARGS}
        self.select(preset)

    @property
    def sim(self) -> bool:
        return self.mode == 'sim'

    @property
    def backend(self) -> str:
        """
        Return the sim backend the command gives.

        Typed in, else the preset's, else the default (carla).
        """
        return self.args['sim_backend'] or self._preset_backend()

    @property
    def kinematic(self) -> bool:
        return self.sim and self.backend == 'kinematic'

    @property
    def vehicle(self) -> str:
        """Name what `can` is in sim."""
        return 'kinematic sim' if self.backend == 'kinematic' else 'CARLA bridge'

    def _preset_backend(self) -> str:
        return str((self.profiles[self.preset].get('args') or {}).get('sim_backend',
                                                                      ARGS['sim_backend'][0]))

    # ===================================
    #            selection
    # ===================================
    def select(self, preset: str):
        """Take a preset's components, mode and sim backend. The other typed-in arguments stay."""
        p = self.profiles[preset]
        self.preset, self.mode = preset, p['mode']
        # part of the mode: the preset's, until a mode button says else
        self.args['sim_backend'] = ''
        self.on = set(p.get('components') or [])
        if self.sim:
            self.on.add('can')

    def set_arg(self, key: str, value: str):
        self.args[key] = value.strip()

    def set_mode(self, mode: str) -> Change:
        """
        Switch between sim and real.

        Sim switches can on (the sim backend); real switches off the sim-only nodes and what
        needs them.
        """
        if mode == self.mode:
            return Change('')
        if mode == 'sim':
            added = [] if 'can' in self.on else ['can']
            self.mode = 'sim'
            self.on.add('can')
            return Change(f'Sim: can is the {self.vehicle}, always on', turned_on=added)
        on = self._without_broken(self.on - SIM_ONLY)
        off = in_order(self.on - on)
        self.mode, self.on = 'real', on
        # no sim backend on the real vehicle: keep it out of the command
        self.args['sim_backend'] = ''
        return Change('REAL VEHICLE' + (f': switched off {", ".join(off)}' if off else ''),
                      turned_off=off)

    def set_backend(self, backend: str) -> Change:
        """Sim on `backend` (carla | kinematic). Kinematic has no LiDAR: perception goes off."""
        added = self.set_mode('sim').turned_on
        self.args['sim_backend'] = '' if backend == self._preset_backend() else backend
        off = []
        if backend == 'kinematic' and 'perception' in self.on:
            self.on.discard('perception')       # nothing REQUIRES perception
            off = ['perception']
        note = ': no LiDAR, perception off' if off else ''
        return Change(f'Sim ({backend}): can is the {self.vehicle}, always on{note}',
                      turned_on=added, turned_off=off)

    # ===================================
    #            rules
    # ===================================
    def cli(self, on=None) -> dict:
        """Command-line overrides of the preset that give `on` (default: the current selection)."""
        on = self.on if on is None else on
        p = self.profiles[self.preset]
        base = set(p.get('components') or [])
        cli = {}
        if self.mode != p['mode']:
            cli['mode'] = self.mode
        for c in COMPONENTS:
            if c == 'can' and self.sim:          # the sim backend: always on, no switch
                continue
            if (c in on) != (c in base):
                cli[c] = 'true' if c in on else 'false'
        cli.update({k: v for k, v in self.args.items() if v})
        return cli

    def plan(self, on=None):
        """profiles.resolve() of the current selection (or of `on`); raises ProfileError."""
        try:
            return resolve(self.profiles, self.preset, self.cli(on))
        # package_path() on a half-typed package://<pkg> while it is typed in
        except ValueError as e:
            raise ProfileError(f'package path: {e}') from None

    def errors(self, on=None) -> list:
        """
        Return the rules `on` breaks, from resolve(); [] = it resolves.

        `on` defaults to the current selection.
        """
        try:
            self.plan(on)
            return []
        except ProfileError as e:
            return rule_errors(e)

    def _new_errors(self, on, now=None) -> list:
        """
        Return the errors of `on` that the current selection does not have.

        Typed-in arguments can break rules too.
        """
        now = self.errors() if now is None else now
        return [e for e in self.errors(on) if e not in now]

    def _without_broken(self, on) -> set:
        """
        Drop from `on` every component whose REQUIRES it no longer meets.

        Repeated until nothing changes.
        """
        on = set(on)
        while True:
            broken = {c for c in on if self.needs(c, on)}
            if not broken:
                return on
            on -= broken

    def _allowed(self, names) -> list:
        return [c for c in names if self.sim or c not in SIM_ONLY]

    # ===================================
    #            node states
    # ===================================
    def state(self, name: str, now=None) -> NodeState:
        if name == 'can' and self.sim:
            return NodeState.LOCKED
        if name in self.on:
            return NodeState.ON
        return NodeState.BLOCKED if self._new_errors(self.on | {name},
                                                     now) else NodeState.AVAILABLE

    def states(self) -> dict:
        now = self.errors()
        return {c: self.state(c, now) for c in COMPONENTS}

    def activatable(self, name: str) -> bool:
        """Tell whether `name` can go on without changing anything else."""
        return self.state(name) == NodeState.AVAILABLE

    def needs(self, name: str, on=None) -> list:
        """
        Return name's REQUIRES entries that `on` does not meet.

        `on` defaults to the current selection.
        """
        on = self.on if on is None else on
        return [need for need in REQUIRES.get(name, []) if not on & set(alternatives(need))]

    def missing(self, name: str) -> list:
        """
        Return every off component `name` needs, through all levels.

        All alternatives of an "any one of" that are allowed in this mode. What the map points
        at for a blocked node.
        """
        found, todo = set(), [name]
        while todo:
            for need in self.needs(todo.pop()):
                new = set(self._allowed(alternatives(need))) - found
                found |= new
                todo += new
        return in_order(found)

    def with_prereqs(self, name: str) -> set:
        """
        Return the selection plus `name` and everything it needs, through all levels.

        An "any one of" takes its first alternative allowed in this mode (planning before
        static_path).
        """
        on, todo = set(self.on), [name]
        while todo:
            c = todo.pop()
            on.add(c)
            for need in self.needs(c, on):
                todo.append((self._allowed(alternatives(need)) or alternatives(need))[0])
        return on

    def hint(self, name: str) -> str:
        """
        Say why `name` cannot go on now.

        E.g. 'lateral needs: localization, planning or static_path'.
        """
        if not self.sim and name in SIM_ONLY:
            return f'{name} is sim only: switch to Sim'
        needs = [' or '.join(self._allowed(alternatives(n)) or alternatives(n))
                 for n in self.needs(name)]
        # what the prerequisites do not fix
        others = self._new_errors(self.with_prereqs(name))
        parts = [f'{name} needs: {", ".join(needs)}'] if needs else []
        parts += others
        if needs and not others:
            parts.append('double-click switches them on too')
        return '; '.join(parts) or f'{name} cannot go on now'

    # ===================================
    #            clicks
    # ===================================
    def click(self, name: str) -> Change:
        state, before = self.state(name), localization_mode(self.on)
        if state == NodeState.LOCKED:
            return Change(f'can is the {self.vehicle} in sim: always on')
        if state == NodeState.ON:
            return self._mode_changed(self._turn_off(name), before)
        if state == NodeState.AVAILABLE:
            self.on.add(name)
            return self._mode_changed(Change(f'{name} on', turned_on=[name]), before)
        return Change(self.hint(name), highlight=self.missing(name))

    def turn_on_with_prereqs(self, name: str) -> Change:
        if self.state(name) in (NodeState.ON, NodeState.LOCKED):
            return Change('')
        on, before = self.with_prereqs(name), localization_mode(self.on)
        if self._new_errors(on):
            return Change(self.hint(name), highlight=self.missing(name))
        added = in_order(on - self.on)
        self.on = on
        others = [c for c in added if c != name]
        change = Change(f'{name} on' + (f', with {", ".join(others)}' if others else ''),
                        turned_on=added)
        return self._mode_changed(change, before)

    def _mode_changed(self, change: Change, before: str) -> Change:
        """
        Add the new localization mode to the message when the change switched it.

        Except to the default from nothing (localization on: direct, as always) and to nothing
        (both off).
        """
        mode = localization_mode(self.on)
        if mode and mode != before and (before or mode != 'direct'):
            change.message += f'. Localization {mode}: {MODE_NOTES[mode]}'
        return change

    def _turn_off(self, name: str) -> Change:
        on = self._without_broken(self.on - {name})
        blocking = self._new_errors(on)
        if blocking:
            return Change(f'{name} stays on: ' + '; '.join(blocking))
        off = in_order(self.on - on)
        self.on = on
        others = [c for c in off if c != name]
        return Change(f'{name} off' + (f', and with it {", ".join(others)}' if others else ''),
                      turned_off=off)

    # ===================================
    #            command
    # ===================================
    def command(self, checks: bool = True, dry_run: bool = False) -> list:
        """Return the argv of the launch: the preset plus what differs from it."""
        cmd = LAUNCH + [f'profile:={self.preset}'] + [f'{k}:={v}' for k, v in self.cli().items()]
        if not checks:
            cmd.append('checks:=false')
        if dry_run:
            cmd.append('dry_run:=true')
        return cmd


def localization(plan) -> str:
    """
    Describe the localization mode and what it runs, for the side panel.

    '' = neither localization nor filter is on.
    """
    mode = plan.localization_mode
    return f'{mode}: {MODE_NOTES[mode]}' if mode else ''


def safety_filter(plan) -> str:
    """Name the safety filter trajectory_controller will run with, for the side panel."""
    if 'longitudinal' not in plan.on:
        return '— (longitudinal off)'
    if 'perception' not in plan.on:
        return 'none (perception off)'
    if plan.args['filter_type']:
        return plan.args['filter_type']
    try:
        with open(share('nift_control', 'config', 'planning.yaml')) as f:
            ft = yaml.safe_load(f)['trajectory_controller']['ros__parameters']['filter_type']
        return f'{ft} (planning.yaml)'
    except Exception:       # only a label: never stop the launcher over it
        return "planning.yaml's filter_type"
