"""
ROS side of gap_probe.py: with PYTHONPATH=<this dir> and GNSS_PROBE=<file.jsonl>, kinematic_sim_node logs the
real-feed model's gap onsets, bad-link spells and deliveries, one JSON line each, flushed at once (the node is
stopped with SIGINT, so nothing waits for exit). Other processes are left alone. The repo file is not changed.
"""
import json
import os
import sys

_out = os.environ.get('GNSS_PROBE')
if _out and any('kinematic_sim' in a for a in sys.argv[:1]):
    from nift_carla import sim_gnss

    _f = open(_out, 'a', buffering=1)
    _orig = sim_gnss.GnssSim._step_feed
    _names = ('front', 'rear', 'both')

    def _step_feed(self, sim_ns, x, y, fx, fy, speed, elevation, to_wgs):
        gaps0, bad0 = list(self._gap), self._bad_until
        out = _orig(self, sim_ns, x, y, fx, fy, speed, elevation, to_wgs)
        t = sim_ns / 1e9
        moving = abs(speed) > sim_gnss.MOVING_SPEED_THRESHOLD_MPS
        for k in range(3):
            if self._gap[k] != gaps0[k]:
                a, b = self._gap[k]
                _f.write(json.dumps({'ev': 'gap', 't': round(t, 3), 'antenna': _names[k], 'start': round(a, 4),
                                     'end': round(b, 4), 'len': round(b - a, 4),
                                     'holds': bool(self._gap_holds[k]), 'moving': moving}) + '\n')
        if self._bad_until != bad0:
            _f.write(json.dumps({'ev': 'spell', 'start': round(t, 3), 'end': round(self._bad_until, 3)}) + '\n')
        for p in out:
            k = list(self.ids).index(p['id']) if p['id'] in self.ids else -1
            _f.write(json.dumps({'ev': 'delivery', 't': round(t, 3),
                                 'antenna': _names[k] if k >= 0 else p['id'], 'stamp': p['serviced']}) + '\n')
        return out

    sim_gnss.GnssSim._step_feed = _step_feed
