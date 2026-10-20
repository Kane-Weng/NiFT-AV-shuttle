"""
The playground against the real ROS 2 stack (tests/reference/runs, README there).

Each case runs the CLI twice (conftest.py). 'shipped' is the CLI as it is. 'ros_order' is the same CLI with the tick order the ROS runs show (ros_order.py, README.md here): every node's timer fires on what had arrived before the tick, so trajectory_controller sees the GNSS fix of t - 2 dt, as on ROS. Run with

    python3 -m pytest nift_playground/tests/validation                              # fast: route 03, GNSS outage
    NIFT_PLAYGROUND_SLOW=1 python3 -m pytest nift_playground/tests/validation       # all 16 cases

Time axis. The runs are lined up on the first tick the shuttle moves (reference_compare.py). On ROS the first 0x101 frame sometimes reaches the virtual ECU one tick late, and that would shift a whole run by 10 cm.

Tolerances (README.md here has the numbers behind them):
- Both modes: the same outcome and status sequence, the same planned path (the Lanelet2 stand-in is exact), and on GNSS loss the "GNSS stale" and SAFE_STOP lines and the stop time within 0.15 s (one tick plus 0.1 s).
- ros_order: position at the same time since the onset within 10 cm while both navigate, track shape within 5 cm, nav duration within 0.2 s, final position within 10 cm. The final position and the duration are quantised by the controller's 10 Hz goal check (up to 0.1 s and 1.8 m/s x 0.1 s = 0.18 m). The worst seen is 8 cm and 0.1 s (route 03). Route 12 by shape and final position only.
- shipped: since 2026-10-20 fakeros' World.step uses the ROS order itself (ARCH section 7), so shipped and ros_order give the same runs and share the ros_order bounds. (Before, the shipped gap was up to 32 cm at the same time and 0.28 s.)
"""

import os

import pytest

from cases import CASES, FAST, SLOW
from reference_compare import REFERENCE_RUNS, compare_dirs

TICK = 0.05

# metric: (ros_order bound, shipped bound); None = not checked
BOUNDS = {
    'pos_nav_max': (0.10, 0.10),
    'shape_max': (0.05, 0.05),
    'dur': (0.20, 0.20),
    'final': (0.10, 0.10),
}
# per-route exceptions
ROUTE_BOUNDS = {
    # 1.1 m corner: compare the shape, not the timing (reference README). ROS direct vs ekf differ by 0.6 s.
    '12': {'pos_nav_max': (None, None), 'dur': (None, None), 'shape_max': (0.05, 0.05), 'final': (0.10, 0.10)},
    # the stall: no duration; where it stalls is `final`
    '11': {'shape_max': (0.05, 0.05)},
}
FAULT_DT = TICK + 0.10


def _params(labels, slow):
    marks = [pytest.mark.slow] if slow else []
    return [pytest.param(lab, m, id=f'{m}-{lab}', marks=marks) for lab in labels for m in ('shipped', 'ros_order')]


@pytest.mark.parametrize('label,mode', _params(FAST, False) + _params(SLOW, True))
def test_against_reference(playground_runs, label, mode):
    out = playground_runs.get(label, mode)
    c = compare_dirs(os.path.join(REFERENCE_RUNS, label), out)
    route = CASES[label][0]
    fault = label.startswith('gnss_')
    report = ', '.join(f'{k} {v:.3f}' if isinstance(v, float) else f'{k} {v}' for k, v in c.items())

    assert c['outcome'][0] == c['outcome'][1], report
    assert c['status_sequence'][0] == c['status_sequence'][1], report
    assert c['path_points'][0] == c['path_points'][1] and c['path_max'] <= 1e-6, report

    if fault:
        for key in ('stale_dt', 'safe_stop_dt', 'dur'):
            assert c[key] is not None and abs(c[key]) <= FAULT_DT, f'{key}: {report}'

    i = 0 if mode == 'ros_order' else 1
    for metric, bounds in BOUNDS.items():
        bound = ROUTE_BOUNDS.get(route, {}).get(metric, bounds)[i]
        value = c[metric]
        if bound is None or value is None:
            continue
        assert abs(value) <= bound, f'{metric} {value:.3f} > {bound}: {report}'
