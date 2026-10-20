#!/usr/bin/env python3
"""
Command line logger that records the raw Octane beacon feed.

File: octane_logger.py

Description:
  Records the Mcity Octane feed exactly as it arrives, for mapping where GNSS
  drops out on the track and for reading fields beacon_node does not use. No
  ROS: it runs next to the stack or on its own (a manual drive needs nothing
  else). Same server, key and beacon IDs as beacon_node (octane_client:
  MCITY_OCTANE_* from the environment, else nift_sensor/.env).

  Output: one JSON object per line in <out>/octane_<YYYYmmdd_HHMMSS>.jsonl,
  flushed per line:
    {"t": <wall s>, "mono": <monotonic s>, "ev": <event>, "data": <payload>}
      ev = connect | auth_ok | disconnect | join | beacon_update | v2x_BSM |
           <any other event the server sends> | net | note
    net   every --net-period s: TCP connect time to the Octane server
          (ms or error) and the Wi-Fi link from /proc/net/wireless, to tell a
          dead spot of the beacons' uplink from one of this computer's link
  Only the front / rear beacon IDs are logged unless --all (the account may
  see other vehicles' beacons); the others are only counted.

  Every --status-period s a status line: per antenna messages, new positions
  (changed `updated`) and the longest wait for one; Octane link; Wi-Fi.
  At the end (Ctrl-C or --minutes): <file>.summary.json with counts, gaps
  and every payload key seen.

Usage:
  ros2 run nift_sensor octane_logger --out octane_logs
  python3 -m nift_sensor.octane_logger --out /tmp/oct --minutes 2 --all
"""

import argparse
from datetime import datetime
import json
import os
import socket
import threading
import time
from urllib.parse import urlparse

from .octane_client import normalize, octane_config

CHANNELS = ('beacon', 'v2x_obu_parsed')     # as beacon_node joins


def read_wireless(path='/proc/net/wireless'):
    """{iface: {'link': float, 'level_dbm': float}} from /proc/net/wireless ({} without Wi-Fi)."""
    out = {}
    try:
        with open(path) as f:
            lines = f.readlines()[2:]
    except OSError:
        return out
    for line in lines:
        if ':' not in line:
            continue
        iface, rest = line.split(':', 1)
        cols = rest.split()
        try:
            out[iface.strip()] = {'link': float(cols[1].rstrip('.')),
                                  'level_dbm': float(cols[2].rstrip('.'))}
        except (IndexError, ValueError):
            continue
    return out


def tcp_rtt_ms(host, port, timeout_s=2.0):
    """TCP connect time to host:port in ms, or the error text."""
    t0 = time.monotonic()
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return round((time.monotonic() - t0) * 1000.0, 1), None
    except OSError as e:
        return None, str(e) or type(e).__name__


class Tracker:
    """Per-antenna counters for the status lines and the summary (thread-safe)."""

    def __init__(self, ids):
        self.ids = list(ids)
        self.lock = threading.Lock()
        self.msgs = {i: 0 for i in ids}
        self.new = {i: 0 for i in ids}
        self.last_ts = {}
        self.last_new_mono = {}
        # longest time without a new position, whole run
        self.max_wait = {i: 0.0 for i in ids}
        self.window_wait = {i: 0.0 for i in ids}         # same, since the last status line
        # waits > GAP_S: (wall t at the end, seconds)
        self.gaps = {i: [] for i in ids}
        self.others = 0
        self.keys = set()

    GAP_S = 0.5

    def update(self, payload, t, mono):
        n = normalize(payload)
        with self.lock:
            self.keys |= _keys(payload)
            if n['id'] not in self.msgs:
                self.others += 1
                return False
            i = n['id']
            self.msgs[i] += 1
            if n['ts'] is not None and n['ts'] != self.last_ts.get(i):
                self.new[i] += 1
                if i in self.last_new_mono:
                    wait = mono - self.last_new_mono[i]
                    self.max_wait[i] = max(self.max_wait[i], wait)
                    self.window_wait[i] = max(self.window_wait[i], wait)
                    if wait > self.GAP_S:
                        self.gaps[i].append((round(t, 3), round(wait, 3)))
                self.last_new_mono[i] = mono
                self.last_ts[i] = n['ts']
            return True

    def status(self, mono):
        with self.lock:
            parts = []
            for i in self.ids:
                since = mono - self.last_new_mono[i] if i in self.last_new_mono else None
                w = max(self.window_wait[i], since or 0.0)
                parts.append(f'{i}: {self.msgs[i]} msgs, {self.new[i]} new, longest wait {w:.2f} s'
                             + ('' if since is not None else ' (nothing yet)'))
                self.window_wait[i] = 0.0
            return ' | '.join(parts)

    def summary(self):
        with self.lock:
            return {'antennas': {i: {'messages': self.msgs[i], 'new_positions': self.new[i],
                                     'longest_wait_s': round(self.max_wait[i], 3),
                                     'gaps_over_0.5s': len(self.gaps[i]), 'gaps': self.gaps[i]}
                                 for i in self.ids},
                    'other_beacon_messages': self.others, 'payload_keys': sorted(self.keys)}


def _keys(obj, prefix=''):
    """Dotted key paths of a nested dict, e.g. state.dynamics.latitude."""
    out = set()
    if isinstance(obj, dict):
        for k, v in obj.items():
            p = f'{prefix}{k}'
            out.add(p)
            out |= _keys(v, p + '.')
    return out


class OctaneLogger:
    """
    One Octane connection, logged to JSONL.

    run() blocks until stop_event, --minutes or Ctrl-C.
    """

    def __init__(self, cfg, out_dir, log_all=False, net_period=1.0, status_period=10.0,
                 printer=print):
        self.cfg = cfg
        self.ns = cfg['MCITY_OCTANE_NAMESPACE']
        self.ids = [cfg['MCITY_FRONT_BEACON_ID'], cfg['MCITY_REAR_BEACON_ID']]
        self.log_all = log_all
        self.net_period, self.status_period = net_period, status_period
        self.print = printer
        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(out_dir,
                                 f'octane_{datetime.now().strftime("%Y%m%d_%H%M%S")}.jsonl')
        self._f = open(self.path, 'a', buffering=1)
        self._wlock = threading.Lock()
        self.tracker = Tracker(self.ids)
        self.stop_event = threading.Event()
        url = urlparse(cfg['MCITY_OCTANE_SERVER'])
        self.host = url.hostname
        self.port = url.port or (443 if url.scheme == 'https' else 80)
        self.connected = False
        self.last_net = {}

    # ------------------------------------------------------------------ #
    def write(self, ev, data):
        line = json.dumps({'t': round(time.time(), 6), 'mono': round(time.monotonic(), 6),
                           'ev': ev, 'data': data},
                          separators=(',', ':'), default=str)
        with self._wlock:
            self._f.write(line + '\n')

    def on_event(self, ev, data):
        """
        Every server event.

        Beacon payloads go through the tracker (and the ID filter), the rest is logged.
        """
        t, mono = time.time(), time.monotonic()
        if ev in ('beacon_update', 'v2x_BSM') and isinstance(data, dict):
            ours = self.tracker.update(data, t, mono)
            if not ours and not self.log_all:
                return
        elif ev == 'auth_ok':
            for ch in CHANNELS:
                self.sio.emit('join', {'channel': ch}, namespace=self.ns)
            self.write('join', {'channels': list(CHANNELS)})
        self.write(ev, data)

    def run(self, minutes=None):
        import socketio
        self.sio = socketio.Client(reconnection=True, reconnection_attempts=0,
                                   reconnection_delay=1)

        @self.sio.on('connect', namespace=self.ns)
        def _connect():
            self.connected = True
            self.write('connect', {'server': self.cfg['MCITY_OCTANE_SERVER'],
                                   'namespace': self.ns})
            self.sio.emit('auth', {'x-api-key': self.cfg['MCITY_OCTANE_KEY']}, namespace=self.ns)

        @self.sio.on('disconnect', namespace=self.ns)
        def _disconnect(*_args):
            self.connected = False
            self.write('disconnect', {})

        @self.sio.on('*', namespace=self.ns)
        def _any(event, data=None):
            self.on_event(event, data)

        self.write('note', {'logger': 'octane_logger', 'ids': self.ids, 'log_all': self.log_all,
                            'host': socket.gethostname()})
        threads = [threading.Thread(target=self._net_loop, daemon=True),
                   threading.Thread(target=self._status_loop, daemon=True)]
        for th in threads:
            th.start()
        self.print(f'octane_logger: {self.cfg["MCITY_OCTANE_SERVER"]} -> {self.path}')
        deadline = None if not minutes else time.monotonic() + minutes * 60.0
        try:
            # first connection: retry here (the server may be out of reach where the drive starts);
            # after that the client's own reconnection takes over (calling connect() again would
            # fight it)
            while not self.stop_event.is_set() and not (deadline and time.monotonic() >= deadline):
                try:
                    self.sio.connect(self.cfg['MCITY_OCTANE_SERVER'], namespaces=[self.ns],
                                     wait_timeout=5)
                    break
                except Exception as e:
                    self.write('note', {'connect_error': str(e)[:200]})
                    self.print(f'octane_logger: cannot connect yet ({str(e)[:120]}), retrying')
                    self.stop_event.wait(2.0)
            while not self.stop_event.is_set() and not (deadline and time.monotonic() >= deadline):
                self.stop_event.wait(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            try:
                self.sio.disconnect()
            except Exception:
                pass
            summary = dict(self.tracker.summary(), file=self.path)
            with open(self.path + '.summary.json', 'w') as f:
                json.dump(summary, f, indent=1)
            self._f.close()
            self.print(json.dumps({k: v for k, v in summary.items() if k != 'payload_keys'},
                                  default=str)[:2000])
            self.print(f'payload keys: {summary["payload_keys"]}')
        return summary

    def _net_loop(self):
        while not self.stop_event.wait(self.net_period):
            rtt, err = tcp_rtt_ms(self.host, self.port)
            wifi = read_wireless()
            self.last_net = {'rtt_ms': rtt, 'err': err, 'wifi': wifi}
            self.write('net', {'host': self.host, 'port': self.port, 'rtt_ms': rtt, 'err': err,
                               'wifi': wifi, 'socket_connected': self.connected})

    def _status_loop(self):
        while not self.stop_event.wait(self.status_period):
            n = self.last_net
            wifi = ', '.join(f'{k} {v["level_dbm"]:.0f} dBm'
                             for k, v in (n.get('wifi') or {}).items()) or 'no Wi-Fi'
            link = f'{n["rtt_ms"]} ms' if n.get('rtt_ms') is not None else f'down ({n.get("err")})'
            self.print(f'{time.strftime("%H:%M:%S")} {self.tracker.status(time.monotonic())} | '
                       f'Octane {"connected" if self.connected else "NOT connected"}, '
                       f'TCP {link} | {wifi}')


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Record the raw Mcity Octane feed (JSONL) for GNSS coverage mapping.')
    ap.add_argument('--out', default='octane_logs',
                    help='output directory (default: octane_logs in the current directory)')
    ap.add_argument('--minutes', type=float, help='stop after this long (default: until Ctrl-C)')
    ap.add_argument('--all', action='store_true', help='also log beacons other than front / rear')
    ap.add_argument('--net-period', type=float, default=1.0, help='network probe period [s]')
    ap.add_argument('--status-period', type=float, default=10.0, help='status line period [s]')
    a = ap.parse_args(argv)
    cfg = octane_config()
    missing = [k for k in ('MCITY_OCTANE_KEY', 'MCITY_FRONT_BEACON_ID', 'MCITY_REAR_BEACON_ID')
               if not cfg.get(k)]
    if missing:
        raise SystemExit(f'octane_logger: {", ".join(missing)} not set '
                         f'(nift_sensor/.env or the environment; '
                         f'found: {cfg["env_file"] or "no .env"})')
    OctaneLogger(cfg, a.out, log_all=a.all, net_period=a.net_period,
                 status_period=a.status_period).run(a.minutes)


if __name__ == '__main__':
    main()
