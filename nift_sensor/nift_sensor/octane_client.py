"""
Octane client helpers shared by beacon_node, octane_logger and the preflight check.

File: octane_client.py

Description:
  The Mcity Octane side of beacon_node without ROS, shared with the system.launch.py
  preflight check (nift_bringup/components/gnss.py):

    octane_config()  MCITY_OCTANE_* from the environment, else from nift_sensor/.env (the
                     same .env beacon_node loads; the environment wins, as with load_dotenv)
    normalize()      one beacon payload (either observed schema) -> a flat dict; speed in m/s
                     (Octane's `state.dynamics.velocity` is in KNOTS: garage session 2026-10-08,
                     reported speed / position-derived speed = 1.936 over 1524 samples, knots =
                     1.944)
    probe()          connect, auth, join `beacon`, and wait until both antennas deliver a
                     new position (a changed `updated`, as beacon_node's stale gate counts it)

Usage:
  from nift_sensor.octane_client import octane_config, probe
  report = probe(octane_config(), timeout_s=5.0)
"""

import os
from pathlib import Path
import re
import threading
import time

ENV_KEYS = ('MCITY_OCTANE_SERVER', 'MCITY_OCTANE_KEY', 'MCITY_OCTANE_NAMESPACE',
            'MCITY_FRONT_BEACON_ID', 'MCITY_REAR_BEACON_ID')
DEFAULTS = {'MCITY_OCTANE_SERVER': 'https://octane.invalid', 'MCITY_OCTANE_NAMESPACE': '/octane',
            'MCITY_FRONT_BEACON_ID': '', 'MCITY_REAR_BEACON_ID': ''}
# Octane `velocity` is knots (measured 2026-10-08); /gnss/vel is m/s
KNOTS_TO_MPS = 1852.0 / 3600.0


def find_env_file():
    """
    Find the .env file that beacon_node uses.

    The first one walking up from this source file, or None.
    """
    here = Path(__file__).resolve()
    for parent in [here.parent, *here.parents]:
        if (parent / '.env').exists():
            return parent / '.env'
    return None


def octane_config() -> dict:
    """
    Read the MCITY_OCTANE_* settings.

    Values already in the environment win over .env (load_dotenv semantics). Does not modify
    os.environ.
    """
    from dotenv import dotenv_values
    path = find_env_file()
    values = dict(DEFAULTS)
    values.update({k: v for k, v in (dotenv_values(path) if path else {}).items()
                   if v is not None})
    values.update({k: os.environ[k] for k in ENV_KEYS if k in os.environ})
    values['env_file'] = str(path) if path else None
    return values


def normalize(payload: dict) -> dict:
    """
    Normalize two observed schemas into one dict.

    Returns keys: id, ts, lat, lon, alt, elev, speed, heading, source speed is in m/s. Schema A's
    `velocity` arrives in knots and is converted here; schema B's `speed` has never been seen on
    the shuttle and is passed through unconverted.
    """
    beacon_id = payload.get('id') or payload.get('idFixed') or payload.get('idTemporary')

    # Schema A: state.dynamics
    if isinstance(payload.get('state'), dict) and isinstance(payload['state'].get('dynamics'),
                                                             dict):
        dyn = payload['state']['dynamics']
        return {
            'id': beacon_id,
            'ts': dyn.get('updated') or payload.get('updated'),
            'lat': dyn.get('latitude'),
            'lon': dyn.get('longitude'),
            'alt': dyn.get('altitude', 0.0),
            'elev': dyn.get('elevation'),
            'speed': None if dyn.get('velocity') is None
            else float(dyn['velocity']) * KNOTS_TO_MPS,
            'heading': dyn.get('heading'),
            'source': 'state_dynamics',
        }

    # Schema B: flat J2735
    return {
        'id': beacon_id,
        'ts': payload.get('updated'),
        'lat': payload.get('latitude'),
        'lon': payload.get('longitude'),
        'alt': 0.0,
        'elev': payload.get('elevation'),
        'speed': payload.get('speed'),      # unit unverified (never observed on the shuttle)
        'heading': payload.get('heading'),
        'source': 'j2735_bsm',
    }


def _short_error(e: Exception) -> str:
    """
    Return the root cause of a connection error.

    Strip the root cause of a connection error ('[Errno 111] Connection refused') out from the
    requests' whole chain.
    """
    text = str(e) or type(e).__name__
    errno = re.findall(r'\[Errno -?\d+\][^)"\']*', text)
    return errno[-1].strip() if errno else text[:160]


def probe(cfg: dict, timeout_s: float = 5.0) -> dict:
    """
    One-shot health check for the Octane beacon server within timeout_s.

    {'connected': bool, 'auth_ok': bool, 'error': str | None, 'antennas': {beacon_id: number of new
    positions (changed `updated`)}}

    Returns as soon as both antennas have delivered a new position.
    """
    import socketio
    ns = cfg['MCITY_OCTANE_NAMESPACE']
    ids = [cfg['MCITY_FRONT_BEACON_ID'], cfg['MCITY_REAR_BEACON_ID']]
    report = {'connected': False, 'auth_ok': False, 'error': None, 'antennas': {i: 0 for i in ids}}
    last_ts, done = {}, threading.Event()
    sio = socketio.Client(reconnection=False)   # probe fail fast; no retry in background

    # =======================================
    #          HANDSHAKE HANDLERS
    # =======================================
    @sio.on('connect', namespace=ns)
    def _on_connect():
        report['connected'] = True
        sio.emit('auth', {'x-api-key': cfg['MCITY_OCTANE_KEY']}, namespace=ns)

    @sio.on('auth_ok', namespace=ns)
    def _on_auth_ok(_data):
        report['auth_ok'] = True
        sio.emit('join', {'channel': 'beacon'}, namespace=ns)

    # =======================================
    #            DATA HANDLERS
    # =======================================
    @sio.on('beacon_update', namespace=ns)
    def _on_update(data):
        n = normalize(data)

        # Ignore any beacon that isn't the front / rear one
        if n['id'] not in report['antennas']:
            return

        # Message and timestamp: msg with different `ts` counts as a new position
        if n['id'] in last_ts and n['ts'] != last_ts[n['id']]:
            report['antennas'][n['id']] += 1
        last_ts[n['id']] = n['ts']

        # True once both counts are at least 1
        if all(report['antennas'].values()):
            done.set()

    # Run and wait
    deadline = time.monotonic() + timeout_s
    try:
        sio.connect(cfg['MCITY_OCTANE_SERVER'], namespaces=[ns], wait_timeout=max(1.0,
                                                                                  timeout_s / 2))
        done.wait(max(0.0, deadline - time.monotonic()))
    except Exception as e:     # socketio.exceptions.ConnectionError and friends
        report['error'] = _short_error(e)
    finally:
        try:
            sio.disconnect()
        except Exception:
            pass
    return report
