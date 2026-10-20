"""The world as executor (ARCH 7): clock-grid timers with rcl's catch-up rule, the callback queue."""
import pytest

START_NS = 1_000_000_000
DT_NS = 50_000_000


def ticks(world, n, pre=None):
    for k in range(1, n + 1):
        world.step(START_NS - DT_NS + k * DT_NS, pre)


# ------------------------------------------------------------------ timers

def test_timer_first_due_one_period_after_creation(world, node):
    seen = []
    node.create_timer(0.1, lambda: seen.append(world.now_ns))
    ticks(world, 6)
    # created at 0.95 s: due 1.05, 1.15, ...; every callback sees the tick time
    assert seen == [1_050_000_000, 1_150_000_000, 1_250_000_000]


def test_50hz_timer_under_20hz_clock_fires_1_or_2_times_per_tick(world, node):
    """rcl's rule: next = due + period, skipped ahead to the first next >= now. Due == now fires again.

    The reference ROS runs show the same: /gnss/fix at 30 Hz, alternating 1 and 2 per 0.05 s tick."""
    seen = []
    node.create_timer(0.02, lambda: seen.append(world.now_ns))
    ticks(world, 8)
    per_tick = [seen.count(START_NS - DT_NS + k * DT_NS) for k in range(1, 9)]
    assert per_tick == [1, 2, 1, 2, 1, 2, 1, 2]


def test_period_is_truncated_like_rclpy(world, node):
    tm = node.create_timer(1.0 / 3.0, lambda: None)
    assert tm.timer_period_ns == int(float(1.0 / 3.0) * 1e9) == 333333333
    assert node.create_timer(1 / 50, lambda: None).timer_period_ns == 20_000_000


def test_catch_up_skips_whole_periods(world, node):
    seen = []
    node.create_timer(0.1, lambda: seen.append(world.now_ns))
    world.fire_timers(START_NS + 1_000_000_000)        # 1 s later in one jump
    # fires once (due 1.05), then next = 1.15 < 2.0 -> skip to the first >= 2.0, which is 2.05
    assert seen == [START_NS + 1_000_000_000]
    world.fire_timers(START_NS + 1_049_999_999)
    assert len(seen) == 1
    world.fire_timers(START_NS + 1_050_000_000)
    assert len(seen) == 2


def test_ties_fire_in_creation_order_with_nothing_delivered_between(world, node):
    from std_msgs.msg import String
    order = []
    pub = node.create_publisher(String, '/x', 10)
    node.create_subscription(String, '/x', lambda m: order.append('msg ' + m.data), 10)
    node.create_timer(0.05, lambda: (order.append('a'), pub.publish(String(data='from a'))))
    node.create_timer(0.05, lambda: order.append('b'))
    ticks(world, 1)
    assert order == ['a', 'b', 'msg from a']


def test_earlier_due_fires_first_regardless_of_creation(world, node):
    order = []
    node.create_timer(0.05, lambda: order.append('slow'))     # due 1.00
    node.create_timer(0.02, lambda: order.append('fast'))     # due 0.97 and 0.99, 1.01
    ticks(world, 1)
    assert order == ['fast', 'slow']


def test_step_ros_order_sim_held_timers_then_delivery(world, node):
    """ARCH 7: the sim ticks with its output held, every due timer fires, then all is delivered."""
    from std_msgs.msg import String
    order = []
    pub = node.create_publisher(String, '/x', 10)
    node.create_subscription(String, '/x', lambda m: order.append(('msg', m.data, world.now_ns)), 10)
    node.create_timer(0.05, lambda: (order.append(('timer', world.now_ns)), pub.publish(String(data='timer'))))

    def pre():
        order.append(('pre', world.now_ns))
        pub.publish(String(data='sim'))
    ticks(world, 1, pre)
    assert order == [('pre', START_NS), ('timer', START_NS), ('msg', 'sim', START_NS), ('msg', 'timer', START_NS)]


def test_ros_order_gives_a_two_tick_relay_latency(world):
    """sim -> relay (timer) -> controller (timer): the controller acts on the sim value of t - 2 dt,
    as trajectory_controller does on the GNSS fix in the ROS reference runs."""
    from rclpy.node import Node
    from std_msgs.msg import Int64
    relay, ctl = Node('relay'), Node('ctl')
    sim_pub = relay.create_publisher(Int64, '/sim', 10)
    latest = {'relay': None, 'ctl': None}
    relay.create_subscription(Int64, '/sim', lambda m: latest.__setitem__('relay', m.data), 10)
    out = relay.create_publisher(Int64, '/relayed', 10)
    relay.create_timer(0.05, lambda: latest['relay'] is not None and out.publish(Int64(data=latest['relay'])))
    ctl.create_subscription(Int64, '/relayed', lambda m: latest.__setitem__('ctl', m.data), 10)
    seen = []
    ctl.create_timer(0.05, lambda: seen.append((world.now_ns, latest['ctl'])))
    ticks(world, 5, lambda: sim_pub.publish(Int64(data=world.now_ns)))
    assert [(t, v) for t, v in seen if v is not None] == [(t, t - 2 * DT_NS) for t, _ in seen[2:]]


def test_cancel_reset_inside_callback(world, node):
    seen = []
    tm = None

    def cb():
        seen.append(world.now_ns)
        if len(seen) == 1:
            tm.reset()           # next one period from now
        elif len(seen) == 2:
            tm.cancel()
    tm = node.create_timer(0.1, cb)
    ticks(world, 10)
    assert seen == [1_050_000_000, 1_150_000_000]
    assert tm.is_canceled()
    tm.reset()
    ticks_from = world.now_ns
    world.fire_timers(ticks_from + 100_000_000)
    assert len(seen) == 3


def test_destroy_timer_and_destroy_node(world, node):
    seen = []
    tm = node.create_timer(0.05, lambda: seen.append(1))
    node.destroy_timer(tm)
    node.create_timer(0.05, lambda: seen.append(2))
    ticks(world, 1)
    assert seen == [2]
    node.destroy_node()
    ticks(world, 3)
    assert seen == [2]


def test_timer_created_in_callback_due_one_period_later(world, node):
    seen = []
    node.create_timer(0.05, lambda: seen.append('outer') if seen else (
        seen.append('outer'), node.create_timer(0.1, lambda: seen.append(('inner', world.now_ns)))))
    ticks(world, 4)
    assert ('inner', START_NS + 100_000_000) in seen


def test_period_zero_fires_once_per_call(world, node):
    seen = []
    node.create_timer(0.0, lambda: seen.append(world.now_ns))
    world.fire_timers(START_NS)
    world.fire_timers(START_NS)
    assert seen == [START_NS, START_NS]


def test_time_never_goes_back(world):
    with pytest.raises(ValueError):
        world.step(START_NS - 2 * DT_NS)
    with pytest.raises(ValueError):
        world.fire_timers(0)


def test_timer_introspection(world, node):
    tm = node.create_timer(0.1, lambda: None)
    assert tm.time_until_next_call() == 100_000_000
    assert not tm.is_ready()
    world.now_ns += 100_000_000
    assert tm.is_ready()
    assert tm.clock is node.get_clock()


def test_run_until_legacy_fires_at_each_due_time(world, node):
    seen = []
    node.create_timer(0.02, lambda: seen.append(world.now_ns))
    world.run_until(START_NS)
    assert seen == [970_000_000, 990_000_000]
    assert world.now_ns == START_NS


# ------------------------------------------------------------------ queue

def test_call_soon_runs_after_current_callback_in_order(world):
    order = []

    def first():
        world.call_soon(order.append, 'queued by first')
        order.append('first done')
    world.call_soon(first)
    world.call_soon(order.append, 'second')
    assert order == []
    assert world.drain() == 3
    assert order == ['first done', 'second', 'queued by first']


def test_publish_is_delivered_later_as_a_copy(world, node):
    from geometry_msgs.msg import Point
    got = []
    node.create_subscription(Point, '/p', got.append, 10)
    node.create_subscription(Point, '/p', got.append, 10)
    pub = node.create_publisher(Point, '/p', 10)
    msg = Point(x=1.0)
    pub.publish(msg)
    assert got == []
    msg.x = 99.0                     # the publisher keeps its object; subscribers saw the old value
    world.drain()
    assert [m.x for m in got] == [1.0, 1.0]
    assert got[0] is not got[1] and got[0] is not msg


def test_publish_checks_type(world, node):
    from std_msgs.msg import Bool, String
    pub = node.create_publisher(String, '/s', 10)
    with pytest.raises(TypeError):
        pub.publish(Bool(data=True))


def test_topic_with_two_types_is_refused(world, node):
    from std_msgs.msg import Bool, String
    node.create_publisher(String, '/s', 10)
    with pytest.raises(TypeError):
        node.create_subscription(Bool, '/s', lambda m: None, 10)


def test_destroyed_subscription_gets_nothing_queued(world, node):
    from std_msgs.msg import String
    got = []
    sub = node.create_subscription(String, '/s', got.append, 10)
    pub = node.create_publisher(String, '/s', 10)
    pub.publish(String(data='a'))
    node.destroy_subscription(sub)
    world.drain()
    assert got == []


def test_relative_and_private_names(world):
    from rclpy.node import Node
    from std_msgs.msg import String
    n = Node('n', namespace='ns')
    assert n.create_publisher(String, 'to_can_bus', 1).topic_name == '/ns/to_can_bus'
    assert n.create_publisher(String, '~/out', 1).topic_name == '/ns/n/out'
    root = Node('r')
    assert root.create_publisher(String, 'to_can_bus', 1).topic_name == '/to_can_bus'


def test_reset_kills_old_entities(world, node):
    from fakeros._world import StaleEntityError
    from std_msgs.msg import String
    pub = node.create_publisher(String, '/s', 10)
    seen = []
    node.create_timer(0.05, lambda: seen.append(1))
    world.reset(start_ns=START_NS - DT_NS)
    with pytest.raises(StaleEntityError):
        pub.publish(String())
    ticks(world, 2)
    assert seen == [] and world.nodes == [] and world.topics == {}


def test_callback_exception_propagates(world, node):
    from std_msgs.msg import String
    node.create_subscription(String, '/s', lambda m: 1 / 0, 10)
    node.create_publisher(String, '/s', 10).publish(String())
    with pytest.raises(ZeroDivisionError):
        world.drain()


def test_rclpy_module_api(world):
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor  # noqa: F401
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    assert rclpy.ok()
    rclpy.try_shutdown()
    assert not rclpy.ok()
    rclpy.init()
    with pytest.raises(RuntimeError):
        rclpy.spin(None)
    assert rclpy.qos.qos_profile_sensor_data.depth == 5
    assert rclpy.time.Time().nanoseconds == 0
