"""
The fake ROS graph: one process, one sim clock, one thread, deterministic.

File: fakeros/_world.py

Description:
  World is the executor and the middleware at once (ARCH 7).
  - A callback queue, FIFO. publish() queues one delivery per matching subscription (a deep copy
    each, as every subscriber deserializes its own). call_soon(fn, *args) queues any callable
    (socket.io deliveries, service requests and replies). drain() runs the queue, cascading,
    each callback to completion. Nothing is delivered inside the call that queues it.
  - Timers on the clock grid, in the ROS order (ARCH 7). On ROS every node is its own process:
    when /clock reaches t, every due timer fires on what had already arrived, and what the sim
    and the timers send at t arrives afterwards. So fire_timers(t) fires, while a timer is due at
    or before t, the one with the smallest (due, creation order), with nothing delivered in
    between, and then drains: the messages queued before (the sim's) first, then the timers'
    output, cascading. Each timer is rescheduled by rcl's rule: next = due + period, and if
    next < t, skip ahead by whole periods to the first next >= t. A timer whose next due time
    equals t fires again in the same call, as rcl would (beacon_node's 50 Hz timer fires 1 or 2
    times per 20 Hz tick, 30 Hz on average, as in the ROS reference runs). Every callback sees
    now() == t. A period-0 timer fires once per call (rcl would spin on it).
  - step(t, pre) is one tick: set the clock to t, call pre() (the sim, its messages held), then
    fire_timers(t). tests/validation/README.md shows this order matches the ROS reference runs
    (the controller acts on the GNSS fix of t - 2 dt); delivering the sim's messages before the
    timers made the controller see the fix of t itself.
  The world is one object for the whole interpreter. reset() empties it in place, so module-level
  objects (can_interface.logger) keep working, and entities made before a reset are dead: using
  them raises.

  Node configuration, set by the launch set before it builds a node (ARCH 6).
  - remaps {own name: launch name}: the launch name replaces the node's own name everywhere.
  - configure_node(own name, name=, namespace=, parameters=[sources]) gives a node its launch
    name and its parameter sources (paths, launch dicts, ParamsFile), merged by rcl's rule.
  - set_node_params(launch name, {param: value}) hands over parameters already merged.

  Legacy, for the Phase 0 spike (tests/spike) only. New code must not use them.
  - run_until(t, stop): fires each timer at its own due time, not on the clock grid.
  - params {node name: {param: value}}: applied like parameter_overrides, after every source.
  - rclpy Client.call(): drains the whole world from inside the caller (ROS would deadlock).
"""
import copy
import heapq
import itertools
from collections import defaultdict, deque

from . import _qos

START_NS = 1_000_000_000
MAX_CALLBACKS_PER_DRAIN = 10_000_000


class StaleEntityError(RuntimeError):
    pass


class _Topic:
    __slots__ = ('name', 'msg_type', 'pubs', 'subs')

    def __init__(self, name):
        self.name = name
        self.msg_type = None
        self.pubs = []
        self.subs = []


class World:
    def __init__(self):
        self.epoch = 0
        self.reset()

    # ------------------------------------------------------------ lifecycle
    def reset(self, start_ns=START_NS):
        """Forget everything (nodes, topics, timers, queue, configuration) and set the clock."""
        self.epoch += 1
        from ._logging import clear_config
        clear_config()                    # logger levels set with rclpy.logging.set_logger_level
        self.now_ns = int(start_ns)
        self.running = True
        self._timers = []                 # heap of (due, creation seq, version, timer)
        self._timer_seq = itertools.count()
        self._queue = deque()
        self.topics = {}
        self.services = {}
        self.nodes = []
        self.remaps = {}
        self.namespaces = {}              # launch name -> namespace
        self.node_sources = {}            # launch name -> [parameter sources]
        self.node_params = {}             # launch name -> {param: value}, already merged
        self.params = {}                  # LEGACY (spike): node name -> {param: value}
        self.log_sink = None              # callable(level 10..50, logger name, text)
        self.log_level = 20
        self.stats = defaultdict(int)

    def configure_node(self, own_name, *, name=None, namespace=None, parameters=()):
        """What a launch Node action gives one node: its launch name, namespace and parameter files."""
        launch_name = name or own_name
        if launch_name != own_name:
            self.remaps[own_name] = launch_name
        if namespace is not None:
            self.namespaces[launch_name] = namespace
        self.node_sources[launch_name] = list(parameters)

    def set_node_params(self, name, params):
        """Parameters for the node with this launch (or own) name, already merged (ARCH 6.2)."""
        self.node_params[name] = dict(params)

    # ------------------------------------------------------------ callback queue
    def call_soon(self, fn, *args):
        """Run fn(*args) after the current callback, in order, like a message delivery."""
        self._queue.append((fn, args, None))

    def _enqueue(self, owner, fn, args):
        self._queue.append((fn, args, owner))

    def drain(self):
        """Run queued callbacks, FIFO, cascading, each to completion. Return how many ran."""
        n = 0
        q = self._queue
        while q:
            fn, args, owner = q.popleft()
            if owner is not None and not owner._alive:
                continue
            n += 1
            if n > MAX_CALLBACKS_PER_DRAIN:
                raise RuntimeError('fakeros: drain did not settle (a callback loop?)')
            self.stats['callbacks'] += 1
            fn(*args)
        return n

    def pending(self):
        return len(self._queue)

    # ------------------------------------------------------------ timers
    def _schedule(self, timer):
        heapq.heappush(self._timers, (timer._due, timer._seq, timer._version, timer))

    def _pop_due(self, t_ns):
        """The next live timer entry due at or before t_ns, popped, else None."""
        h = self._timers
        while h and h[0][0] <= t_ns:
            due, _, version, tm = heapq.heappop(h)
            if tm._alive and not tm._canceled and version == tm._version:
                return due, tm
        return None

    def _fire(self, tm, due, now, drain):
        """rcl_timer_call: move the next call time first, then run the callback (then drain)."""
        tm._last_call_ns = now
        period = tm._period_ns
        if period == 0:
            nxt = now
        else:
            nxt = due + period
            if nxt < now:
                nxt += (1 + (now - nxt - 1) // period) * period
        tm._due = nxt
        tm._version += 1
        version = tm._version
        self.stats['timers'] += 1
        tm.callback()
        if drain:
            self.drain()
        # reschedule unless the callback cancelled, reset (which schedules itself) or destroyed it
        return tm._alive and not tm._canceled and tm._version == version

    def fire_timers(self, t_ns=None):
        """Fire every timer due at or before t_ns (nothing delivered in between), then drain."""
        t_ns = self.now_ns if t_ns is None else int(t_ns)
        if t_ns < self.now_ns:
            raise ValueError(f'fakeros: time goes backwards ({t_ns} < {self.now_ns})')
        self.now_ns = t_ns
        again = []
        while self.running:
            hit = self._pop_due(t_ns)
            if hit is None:
                break
            due, tm = hit
            self.now_ns = t_ns
            if self._fire(tm, due, t_ns, drain=False):
                if tm._period_ns == 0:
                    again.append(tm)                 # period 0: once per call
                else:
                    self._schedule(tm)
        for tm in again:
            if tm._alive and not tm._canceled:
                self._schedule(tm)
        self.now_ns = t_ns
        self.drain()

    def step(self, t_ns, pre=None):
        """One tick (ARCH 7): clock to t_ns, pre() (the sim; what it sends is held), fire_timers(t_ns)."""
        t_ns = int(t_ns)
        if t_ns < self.now_ns:
            raise ValueError(f'fakeros: time goes backwards ({t_ns} < {self.now_ns})')
        self.now_ns = t_ns
        if pre is not None:
            pre()
        self.fire_timers(t_ns)

    def next_timer_ns(self):
        """When the next live timer is due, or None."""
        for due, _, version, tm in sorted(self._timers):
            if tm._alive and not tm._canceled and version == tm._version:
                return due
        return None

    def run_until(self, t_ns, stop=None):
        """LEGACY (spike only): fire each timer at its own due time up to t_ns, stop() after each.

        Not the ARCH 7 clock grid. The runner uses step()."""
        self.drain()
        while self.running:
            h = self._timers
            while h and not (h[0][3]._alive and not h[0][3]._canceled and h[0][2] == h[0][3]._version):
                heapq.heappop(h)
            if not h or h[0][0] > t_ns:
                break
            due, _, _, tm = heapq.heappop(h)
            self.now_ns = max(self.now_ns, due)
            if self._fire(tm, due, self.now_ns, drain=True):
                self._schedule(tm)
            if stop is not None and stop():
                return
        self.now_ns = max(self.now_ns, int(t_ns))

    # ------------------------------------------------------------ topics
    def _topic(self, name, msg_type):
        tp = self.topics.get(name)
        if tp is None:
            tp = self.topics[name] = _Topic(name)
        if tp.msg_type is None:
            tp.msg_type = msg_type
        elif tp.msg_type is not msg_type:
            raise TypeError(f"fakeros: topic '{name}' carries {_tname(tp.msg_type)}, "
                            f'not {_tname(msg_type)}')
        return tp

    def _add_publisher(self, pub):
        tp = self._topic(pub.topic, pub.msg_type)
        tp.pubs.append(pub)
        for sub in tp.subs:
            _warn_incompatible(pub, sub)

    def _add_subscription(self, sub):
        tp = self._topic(sub.topic, sub.msg_type)
        tp.subs.append(sub)
        latched = _qos.is_transient_local(sub.qos_profile)
        for pub in tp.pubs:
            if _warn_incompatible(pub, sub):
                continue
            if latched and pub._history is not None:
                for msg in pub._history:
                    self._enqueue(sub, sub._deliver, (copy.deepcopy(msg),))

    def _remove(self, entity, attr):
        tp = self.topics.get(entity.topic)
        if tp is not None:
            lst = getattr(tp, attr)
            if entity in lst:
                lst.remove(entity)

    def _publish(self, pub, msg):
        self.stats['publish'] += 1
        tp = self.topics[pub.topic]
        pq = pub.qos_profile
        for sub in tp.subs:
            if _qos.incompatible_policy(pq, sub.qos_profile) is None:
                self._enqueue(sub, sub._deliver, (copy.deepcopy(msg),))
        if pub._history is not None:
            pub._history.append(copy.deepcopy(msg))
            if pub._history_depth is not None and len(pub._history) > pub._history_depth:
                del pub._history[0]

    def count_subscribers(self, topic):
        tp = self.topics.get(topic)
        return len(tp.subs) if tp else 0

    def count_publishers(self, topic):
        tp = self.topics.get(topic)
        return len(tp.pubs) if tp else 0


def _tname(t):
    return f'{t.__module__}.{t.__name__}'


def _warn_incompatible(pub, sub):
    """Log rclpy's incompatible-QoS warnings on both sides once. True if they do not match."""
    policy = _qos.incompatible_policy(pub.qos_profile, sub.qos_profile)
    if policy is None:
        return False
    sub._node.get_logger().warning(
        f"New publisher discovered on topic '{sub.topic}', offering incompatible QoS. No messages "
        f'will be received from it. Last incompatible policy: {policy}')
    pub._node.get_logger().warning(
        f"New subscription discovered on topic '{pub.topic}', requesting incompatible QoS. No "
        f'messages will be sent to it. Last incompatible policy: {policy}')
    return True


world = World()
