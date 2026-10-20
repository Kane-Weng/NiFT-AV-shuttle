#!/usr/bin/env python3
"""
Pure helpers for the steering passes of longitudinal_test.

File: steer_util.py
Date: Oct 17, 2026

Description:
  Pure helpers for the steering passes of longitudinal_test (no ROS): the steering sequence, the
  turn the heading guard predicts, and how the shuttle's 0x301 acknowledgement follows each
  steering command.

  parse_seq("10:2,-10:2,0:3")  -> [(10.0, 2.0), (-10.0, 2.0), (0.0, 3.0)]
                                  angle [deg, + = left], seconds
  seq_angle(seq, t)            -> the angle commanded t seconds into the sequence (the last
                                  one after the end)
  wrap_deg(a)                  -> a wrapped to (-180, 180]
  turn_after_stop_deg(...)     -> heading change still to come if the shuttle brakes now with
                                  the wheel held
  ack_steps(events, acks)      -> per steering step: when the 0x301 steering value starts to
                                  move and when it reaches the new command, and whether it
                                  jumps there (an echo of the command) or moves there over
                                  several frames (a measured wheel angle)
"""

import math

from nift_common.vehicle import ShuttleConfig

STEER_LIMIT_DEG = ShuttleConfig().max_steering_deg   # full lock
ACK_TOL_DEG = 0.2          # the ack has reached the command
ACK_MOVE_DEG = 0.05        # the ack has started to move


def parse_seq(text):
    """'angle:secs,angle:secs,...' -> list of (angle_deg, secs). Raises ValueError on bad input."""
    seq = []
    for item in str(text).split(','):
        item = item.strip()
        if not item:
            continue
        try:
            angle, secs = (float(x) for x in item.split(':'))
        except ValueError:
            raise ValueError(f"'{item}': expected angle:seconds, e.g. 10:2") from None
        if not abs(angle) <= STEER_LIMIT_DEG:
            raise ValueError(f"'{item}': angle must be within ±{STEER_LIMIT_DEG:g} deg")
        if not secs > 0:
            raise ValueError(f"'{item}': seconds must be > 0")
        seq.append((angle, secs))
    if not seq:
        raise ValueError('empty steering sequence')
    return seq


def seq_duration(seq):
    return sum(secs for _, secs in seq)


def seq_angle(seq, t):
    """(index, angle) commanded t seconds into the sequence; after the end, the last step."""
    end = 0.0
    for i, (angle, secs) in enumerate(seq):
        end += secs
        if t < end:
            return i, angle
    return len(seq) - 1, seq[-1][0]


def wrap_deg(a):
    a = math.fmod(a + 180.0, 360.0)
    if a <= 0.0:
        a += 360.0
    return a - 180.0


def turn_after_stop_deg(v, steer_deg, wheelbase, blind_s, decel):
    """
    Return the heading change [deg, >= 0] while stopping from speed v, wheel held at steer_deg.

    blind_s of travel at v (GNSS age + reaction), then v² / (2 decel) of braking, on the
    circle of curvature tan(δ) / L.
    """
    kappa = abs(math.tan(math.radians(steer_deg))) / wheelbase
    return math.degrees(kappa * (v * blind_s + v * v / (2.0 * decel)))


def ack_steps(events, acks):
    """
    Show how the 0x301 steering value follows each steering command.

    events: [(t, new_deg, prev_deg)] at every change of the commanded angle;
    acks: [(t, ack_deg)] in time order.
    Returns one dict per event: start [s after the command] when the ack first left the
    previous value, reach [s] when it came within ACK_TOL_DEG of the new one, frames between
    the two, and kind: 'echo' (it jumped to the new value in one frame), 'moves' (it took
    several frames: rate = change / time), or 'none'.
    """
    out = []
    for k, (t_cmd, new, prev) in enumerate(events):
        t_next = events[k + 1][0] if k + 1 < len(events) else math.inf
        window = [(t, a) for t, a in acks if t_cmd <= t < t_next]
        first = next(((t, a) for t, a in window if abs(a - prev) > ACK_MOVE_DEG), None)
        reach = next(((t, a) for t, a in window if abs(a - new) <= ACK_TOL_DEG), None)
        step = {
            't': round(t_cmd, 3),
            'from': prev,
            'to': new,
            'start': None,
            'reach': None,
            'frames': None,
            'kind': 'none',
            'rate_dps': None,
        }
        if first is not None:
            step['start'] = round(first[0] - t_cmd, 3)
        if first is not None and reach is not None:
            step['reach'] = round(reach[0] - t_cmd, 3)
            frames = sum(1 for t, _ in window if first[0] <= t <= reach[0])
            step['frames'] = frames
            if frames <= 1:
                step['kind'] = 'echo'
            else:
                step['kind'] = 'moves'
                dt = reach[0] - first[0]
                if dt > 0:
                    step['rate_dps'] = round(abs(new - prev) / dt, 1)
        out.append(step)
    return out


def ack_summary(steps):
    """
    Summarize the steps as 'echo', 'moves' or 'none'.

    Only steps that reached their command count; the median rate is added if it moves.
    """
    kinds = [s['kind'] for s in steps if s['kind'] != 'none']
    if not kinds:
        return {'kind': 'none'}
    if all(k == 'echo' for k in kinds):
        return {'kind': 'echo'}
    rates = sorted(s['rate_dps'] for s in steps if s['rate_dps'])
    return {'kind': 'moves' if all(k == 'moves' for k in kinds) else 'mixed',
            'rate_dps_median': rates[len(rates) // 2] if rates else None}
