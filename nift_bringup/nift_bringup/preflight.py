"""
Preflight checks of a launch plan before anything starts.

File: preflight.py

Description:
  Checks a resolved plan before anything starts (system.launch.py, on by default; checks:=false
  skips them). Every enabled component's check() from nift_bringup/components runs in parallel,
  so the whole preflight takes as long as the slowest one (the GNSS probe, up to 5 s). On top of
  the component checks, one generic check: nothing may already publish a component's outputs()
  (a second launch or a forgotten `ros2 run` would fight it, e.g. two writers on /to_can_bus).

  Results: [(component, level, message)], level OK / WARN / FAIL / SKIP; any FAIL stops the launch.

Usage:
  from nift_bringup.preflight import run, report, failed
  results = run(plan); print(report(results)); failed(results)
"""

from concurrent.futures import ThreadPoolExecutor
import threading
import time

from .components import REGISTRY
from .components.common import FAIL, Level, OK
from .profiles import COMPONENTS

GRAPH_WAIT_S = 1.5      # DDS discovery of nodes that are already running


class Graph:
    """Who publishes which topic right now, from one ROS graph snapshot taken on first use."""

    def __init__(self, wait_s=GRAPH_WAIT_S):
        self._wait_s, self._lock, self._snapshot = wait_s, threading.Lock(), None

    def publishers(self, topic):
        with self._lock:
            if self._snapshot is None:
                self._snapshot = self._take()
        return self._snapshot.get(topic, [])

    def _take(self):
        import rclpy
        context = rclpy.Context()       # our own, so it never touches launch_ros's
        rclpy.init(context=context)
        node = rclpy.create_node('nift_preflight', context=context)
        try:
            time.sleep(self._wait_s)
            snapshot = {}
            for topic, _types in node.get_topic_names_and_types():
                pubs = [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                        for i in node.get_publishers_info_by_topic(topic)
                        if i.node_name != 'nift_preflight']
                if pubs:
                    snapshot[topic] = sorted(set(pubs))
            return snapshot
        finally:
            node.destroy_node()
            rclpy.shutdown(context=context)


def _check_one(component, plan, graph):
    comp = REGISTRY[component]
    try:
        results = list(comp.check(plan, graph))
        taken = [(t, graph.publishers(t)) for t in comp.outputs(plan)]
    except Exception as e:      # a broken check must not look like a pass
        return [(FAIL, f'check crashed: {type(e).__name__}: {e}')]
    busy = [(t, pubs) for t, pubs in taken if pubs]
    results += [(FAIL, f'{t} is already published by {", ".join(pubs)}: stop that first') for t,
                pubs in busy]
    if taken and not busy and not results:
        results.append((OK, f'nothing else publishes {", ".join(t for t, _ in taken)}'))
    return results


def run(plan):
    """
    Check every enabled component of `plan`.

    Return [(component, level, message)] in COMPONENTS order.
    """
    names = [c for c in COMPONENTS if c in plan.on and c in REGISTRY]
    graph = Graph()
    with ThreadPoolExecutor(max_workers=max(1, len(names))) as pool:
        futures = {c: pool.submit(_check_one, c, plan, graph) for c in names}
    return [(c, level, msg) for c in names
            for level, msg in futures[c].result() or [(Level.NONE, 'nothing to check')]]


def failed(results):
    return any(level == FAIL for _, level, _ in results)


def label(component, plan=None):
    """
    Return the name a report shows for a component.

    In sim, `can` is the sim backend (the vehicle), so its rows are named after it, as the
    launcher map names that node.
    """
    if component == 'can' and plan is not None and plan.sim:
        return 'kinematic sim' if plan.kinematic else 'CARLA bridge'
    return component


def report(results, seconds=None, plan=None):
    took = f' ({seconds:.1f} s)' if seconds is not None else ''
    lines = [f'preflight{took}:'] + [f'  {level.value:<4}  {label(c, plan):<13}  {msg}' for c,
                                     level, msg in results]
    if failed(results):
        lines.append('  -> FAIL: fix the lines above, or start anyway with checks:=false')
    return '\n'.join(lines)
