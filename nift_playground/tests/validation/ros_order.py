"""
Run `playground.cli` with the tick order the ROS reference runs show (validation only, see README.md here).

    cd nift_playground/py && python3 ../tests/validation/ros_order.py --route 03 --out DIR

On ROS each node is its own process. When `/clock` reaches t, every node's timer fires at once, on what had arrived by then. The sim's GNSS payloads of tick t reach beacon_node over the websocket after its timer has fired, and beacon_node -> shuttle_tf_node -> TF reaches trajectory_controller after the controller's own timer has fired. So the controller works on the fix of t - 2 dt. The runner of ARCH section 7 instead delivers the sim's messages before the timers and drains after each timer, so the controller works on the fix of t itself.

This script swaps `fakeros` World.step for that ROS order at run time and then calls the CLI unchanged.
1. clock to t, the sim ticks; what it sends is held
2. every timer due at or before t fires, in (due, creation) order, without any delivery in between
3. everything is delivered, the sim's messages first, then the timers' output, cascading

The controller's 0x101 for t still reaches the virtual ECU before tick t + dt (lockstep). Nothing in fakeros, the runner or a node is edited.
"""

import os
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
PY = os.path.normpath(os.path.join(HERE, '..', '..', 'py'))


def ros_order_step(self, t_ns, pre=None):
    t_ns = int(t_ns)
    if t_ns < self.now_ns:
        raise ValueError(f'fakeros: time goes backwards ({t_ns} < {self.now_ns})')
    self.now_ns = t_ns
    if pre is not None:
        pre()
    held, self._queue = self._queue, deque()
    again = []                                  # period 0: once per tick, as World.fire_timers
    while self.running:
        hit = self._pop_due(t_ns)
        if hit is None:
            break
        due, tm = hit
        tm._last_call_ns = t_ns
        period = tm._period_ns
        nxt = due + period if period else t_ns
        if period and nxt < t_ns:
            nxt += (1 + (t_ns - nxt - 1) // period) * period
        tm._due = nxt
        tm._version += 1
        version = tm._version
        self.stats['timers'] += 1
        tm.callback()
        if tm._alive and not tm._canceled and tm._version == version:
            (self._schedule if period else again.append)(tm)
    for tm in again:
        if tm._alive and not tm._canceled:
            self._schedule(tm)
    held.extend(self._queue)
    self._queue = held
    self.drain()


def install():
    if PY not in sys.path:
        sys.path.insert(0, PY)
    from fakeros import _world
    _world.World.step = ros_order_step


if __name__ == '__main__':
    install()
    from playground import cli
    sys.exit(cli.main(sys.argv[1:]))
