"""
Run playground.cli and log what the real-feed model of sim_gnss does: gap onsets, bad-link spells, deliveries.

    cd nift_playground/py
    python3 ../tests/investigations/ekf_realfeed/gap_probe.py <probe.json> [playground.cli args ...]

Wraps GnssSim._step_feed at run time (the repo file is not changed). It writes <probe.json> with
  gaps        every gap onset: antenna (front, rear, both), start, end, holds (backlog), moving
  spells      every bad-link spell: start, end
  deliveries  every payload: sim time of the step, antenna, stamp, measurement time of its epoch
START_DELAY_S=<s> waits that long (sim) before /start_navigation, to start at a reference run's sim time.
ros_hook/sitecustomize.py does the same logging inside kinematic_sim_node on ROS (see README.md).
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def install(out_path):
    """Wrap GnssSim._step_feed; the log is written at exit."""
    import atexit
    from nift_carla import sim_gnss
    log = {'gaps': [], 'spells': [], 'deliveries': [], 'steps': 0}
    orig = sim_gnss.GnssSim._step_feed
    names = ('front', 'rear', 'both')

    def step_feed(self, sim_ns, x, y, fx, fy, speed, elevation, to_wgs):
        gaps0 = list(self._gap)
        bad0 = self._bad_until
        out = orig(self, sim_ns, x, y, fx, fy, speed, elevation, to_wgs)
        t = sim_ns / 1e9
        log['steps'] += 1
        moving = abs(speed) > sim_gnss.MOVING_SPEED_THRESHOLD_MPS
        for k in range(3):
            if self._gap[k] != gaps0[k]:
                a, b = self._gap[k]
                log['gaps'].append({'t': round(t, 3), 'antenna': names[k], 'start': round(a, 4),
                                    'end': round(b, 4), 'len': round(b - a, 4),
                                    'holds': bool(self._gap_holds[k]), 'moving': moving})
        if self._bad_until != bad0:
            log['spells'].append({'start': round(t, 3), 'end': round(self._bad_until, 3)})
        for p in out:
            k = list(self.ids).index(p['id']) if p['id'] in self.ids else -1
            log['deliveries'].append({'t': round(t, 3), 'antenna': names[k] if k >= 0 else p['id'],
                                      'stamp': p['serviced']})
        return out

    sim_gnss.GnssSim._step_feed = step_feed

    def dump():
        with open(out_path, 'w') as f:
            json.dump(log, f, indent=0)
    atexit.register(dump)
    return log


def main():
    out_path = sys.argv[1]
    sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'py'))
    from playground import paths
    paths.setup(None)
    install(out_path)
    from playground import cli
    # START_DELAY_S: wait this long (sim) before /start_navigation, to start at the reference's sim time
    delay = float(os.environ.get('START_DELAY_S', '0') or 0)
    if delay > 0.0:
        start = cli.RouteRun.start

        def delayed_start(self, *a, **kw):
            self.run_for(delay)
            return start(self, *a, **kw)
        cli.RouteRun.start = delayed_start
    return cli.main(sys.argv[2:])


if __name__ == '__main__':
    sys.exit(main())
