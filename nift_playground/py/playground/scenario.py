"""
Scenarios: the launch options plus the log of applied page messages, with the tick each was applied after (ARCH 8.5).

    { v: 1, launch: {...}, events: [ {tick, msg}, ... ], note?, ticks? }

`ticks` is optional. It says how many ticks the recording ran, so a replay can run as long.

Loading one resets the stack with its launch options and replays each message before tick `tick + 1`, which gives the same trace. In the URL hash a scenario is base64url (no padding) of the raw-deflated (RFC 1951) JSON. `decode()` also takes zlib-wrapped data, padded base64 and plain JSON.
"""

import base64
import json
import zlib

VERSION = 1
# page messages that change the sim and so go into the log (pacing and queries do not)
LOGGED = ('goal', 'cmd', 'teleop', 'initialpose', 'obstacle_add', 'obstacle_move', 'obstacle_remove',
          'set_obstacles', 'gnss_fault')


class ScenarioError(ValueError):
    pass


def make(launch: dict, events: list, note=None) -> dict:
    sc = {'v': VERSION, 'launch': launch, 'events': events}
    if note:
        sc['note'] = note
    return sc


def strip(msg: dict) -> dict:
    """The part of a page message that is replayed: without its reply `id` (obstacles are named by `oid`)."""
    return {k: v for k, v in msg.items() if k != 'id'}


def check(sc) -> dict:
    """A valid scenario from a dict or its URL text; ScenarioError for anything else."""
    if isinstance(sc, str):
        sc = decode(sc)
    if not isinstance(sc, dict) or sc.get('v') != VERSION:
        raise ScenarioError(f'not a version {VERSION} scenario')
    if not isinstance(sc.get('launch', {}), dict):
        raise ScenarioError('scenario.launch must be an object')
    events = sc.get('events') or []
    if not isinstance(events, list):
        raise ScenarioError('scenario.events must be a list')
    last = 0
    for i, ev in enumerate(events):
        if (not isinstance(ev, dict) or not isinstance(ev.get('tick'), int) or isinstance(ev.get('tick'), bool)
                or ev['tick'] < 0 or not isinstance(ev.get('msg'), dict)):
            raise ScenarioError(f'scenario.events[{i}]: expected {{tick: int >= 0, msg: {{type, ...}}}}')
        if ev['tick'] < last:
            raise ScenarioError(f'scenario.events[{i}]: ticks must not go back ({ev["tick"]} < {last})')
        if ev['msg'].get('type') not in LOGGED:
            raise ScenarioError(f'scenario.events[{i}]: {ev["msg"].get("type")!r} is not a replayable message')
        last = ev['tick']
    out = {'v': VERSION, 'launch': dict(sc.get('launch') or {}), 'events': list(events)}
    if sc.get('note'):
        out['note'] = sc['note']
    if isinstance(sc.get('ticks'), int):
        out['ticks'] = sc['ticks']      # optional: how long the recording ran (CLI, tests/pyodide)
    return out


def encode(sc: dict) -> str:
    raw = json.dumps(sc, separators=(',', ':'), sort_keys=True).encode()
    c = zlib.compressobj(9, zlib.DEFLATED, -15)
    data = c.compress(raw) + c.flush()
    return base64.urlsafe_b64encode(data).decode().rstrip('=')


def decode(text: str) -> dict:
    text = text.strip()
    if text.startswith('#'):
        text = text[1:]
    if text.startswith('{'):
        return _json(text)
    pad = '=' * (-len(text) % 4)
    try:
        data = base64.urlsafe_b64decode(text + pad)
    except ValueError as e:
        raise ScenarioError(f'scenario: not base64url ({e})') from None
    for wbits in (-15, 15, 31):
        try:
            raw = zlib.decompress(data, wbits).decode()
        except (zlib.error, ValueError):
            continue
        return _json(raw)
    raise ScenarioError('scenario: not deflated JSON')


def _json(text: str) -> dict:
    try:
        sc = json.loads(text)
    except ValueError as e:
        raise ScenarioError(f'scenario: not JSON ({e})') from None
    if not isinstance(sc, dict):
        raise ScenarioError('scenario: expected a JSON object')
    return sc


def save(path: str, sc: dict):
    with open(path, 'w') as f:
        json.dump(sc, f, indent=1)


def load(path: str) -> dict:
    with open(path) as f:
        return check(decode(f.read()))
