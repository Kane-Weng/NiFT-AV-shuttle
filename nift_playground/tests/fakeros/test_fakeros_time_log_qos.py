"""Time, Duration, clocks, the logger, QoS matching and latching, services."""
import pytest


# ------------------------------------------------------------------ time

def test_time_rules():
    from rclpy.clock import ClockType
    from rclpy.duration import Duration
    from rclpy.time import Time
    assert Time().clock_type == ClockType.SYSTEM_TIME
    assert Time(seconds=1.9999999999).nanoseconds == 1_999_999_999       # truncated, not rounded
    assert Time(seconds=1, nanoseconds=5).nanoseconds == 1_000_000_005
    with pytest.raises(ValueError):
        Time(seconds=-1)
    t = Time(nanoseconds=3_500_000_000, clock_type=ClockType.ROS_TIME)
    assert t.seconds_nanoseconds() == (3, 500_000_000)
    assert (t - Time(seconds=1, clock_type=ClockType.ROS_TIME)).nanoseconds == 2_500_000_000
    assert (t + Duration(seconds=0.5)).nanoseconds == 4_000_000_000
    with pytest.raises(TypeError):
        t - Time(seconds=1)                       # SYSTEM_TIME vs ROS_TIME
    with pytest.raises(TypeError):
        hash(t)
    msg = t.to_msg()
    assert (msg.sec, msg.nanosec) == (3, 500_000_000)
    assert Time.from_msg(msg) == t
    with pytest.raises(TypeError):
        Time.from_msg(object())
    assert Duration(seconds=-1).nanoseconds == -1_000_000_000
    assert Duration(seconds=1) > Duration(nanoseconds=5)


def test_node_clock_is_sim_time(world, node):
    from rclpy.clock import ClockType
    now = node.get_clock().now()
    assert now.clock_type == ClockType.ROS_TIME and now.nanoseconds == world.now_ns
    world.now_ns += 7
    assert (node.get_clock().now() - now).nanoseconds == 7


# ------------------------------------------------------------------ logging

def test_logger_levels_and_sink(world, node):
    lg = node.get_logger()
    assert lg.info('hello') is True
    assert lg.debug('hidden') is False
    lg.warn('w')
    lg.warning('w2')
    assert world.logs == [(20, 'test_node', 'hello'), (30, 'test_node', 'w'), (30, 'test_node', 'w2')]
    lg.set_level(10)
    assert lg.debug('shown') is True


def test_throttle_per_call_site_on_sim_clock(world, node):
    lg = node.get_logger()
    out = []
    for k in range(25):                       # 0.1 s apart, 2.4 s in all
        out.append(lg.warn('t', throttle_duration_sec=1.0))
        world.now_ns += 100_000_000
    assert [k for k, v in enumerate(out) if v] == [0, 10, 20]
    # two call sites on one line are two sites (rclpy keys on the bytecode offset too)
    assert [lg.info('a', throttle_duration_sec=5.0), lg.info('b', throttle_duration_sec=5.0)] == [True, True]


def test_once_and_skip_first(world, node):
    lg = node.get_logger()
    assert [lg.info('o', once=True) for _ in range(3)] == [True, False, False]
    assert [lg.info('s', skip_first=True) for _ in range(3)] == [False, True, True]


def test_filter_state_forgotten_on_reset(world):
    import rclpy.logging
    lg = rclpy.logging.get_logger('module_level')          # like can_interface.logger

    def site():
        return lg.info('y', once=True)
    assert [site(), site()] == [True, False]
    world.reset(start_ns=0)
    world.log_sink = lambda *a: None
    assert [site(), site()] == [True, False]


# ------------------------------------------------------------------ QoS

def test_qos_profile_rules():
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, InvalidQoSProfileException, QoSProfile,
                           ReliabilityPolicy, qos_profile_sensor_data)
    with pytest.raises(InvalidQoSProfileException):
        QoSProfile()
    q = QoSProfile(depth=3)
    assert (q.history, q.reliability, q.durability) == (HistoryPolicy.KEEP_LAST, ReliabilityPolicy.RELIABLE,
                                                        DurabilityPolicy.VOLATILE)
    assert (qos_profile_sensor_data.depth, qos_profile_sensor_data.reliability) == (5, ReliabilityPolicy.BEST_EFFORT)
    assert int(DurabilityPolicy.TRANSIENT_LOCAL) == 1 and int(ReliabilityPolicy.BEST_EFFORT) == 2


def _latched(depth=1):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def test_latched_topic_reaches_late_transient_local_subscriber(world, node):
    from std_msgs.msg import String
    pub = node.create_publisher(String, '/planned_path', _latched(2))
    for s in 'abc':
        pub.publish(String(data=s))
    world.drain()
    late, volatile = [], []
    node.create_subscription(String, '/planned_path', lambda m: late.append(m.data), _latched())
    node.create_subscription(String, '/planned_path', lambda m: volatile.append(m.data), 10)
    assert late == []                         # queued, not delivered inside create_subscription
    world.drain()
    assert late == ['b', 'c'] and volatile == []


def test_incompatible_qos_gets_nothing_and_warns(world, node):
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    reliable, best = [], []
    pub = node.create_publisher(String, '/gnss/fix_like', qos_profile_sensor_data)
    node.create_subscription(String, '/gnss/fix_like', reliable.append, 10)
    node.create_subscription(String, '/gnss/fix_like', best.append, qos_profile_sensor_data)
    pub.publish(String(data='x'))
    world.drain()
    assert reliable == [] and len(best) == 1
    assert any('incompatible QoS' in text and 'RELIABILITY' in text for _, _, text in world.logs)
    # a TRANSIENT_LOCAL subscription does not match a VOLATILE publisher
    vol = node.create_publisher(String, '/v', 10)
    got = []
    node.create_subscription(String, '/v', got.append, _latched())
    vol.publish(String())
    world.drain()
    assert got == []


# ------------------------------------------------------------------ services

def test_trigger_service_relative_name_and_async_reply(world, node):
    from std_srvs.srv import Trigger
    calls = []

    def cb(req, resp):
        calls.append(world.now_ns)
        resp.success, resp.message = True, 'ok'
        return resp
    svc = node.create_service(Trigger, 'start_navigation', cb)
    assert svc.srv_name == 'start_navigation'
    cli = node.create_client(Trigger, '/start_navigation')
    assert cli.service_is_ready()
    fut = cli.call_async(Trigger.Request())
    assert not fut.done() and calls == []
    world.drain()
    assert fut.done() and (fut.result().success, fut.result().message) == (True, 'ok')
    assert cli.call(Trigger.Request()).success


def test_service_must_return_its_response(world, node):
    from std_srvs.srv import Trigger
    node.create_service(Trigger, 'bad', lambda req, resp: None)
    fut = node.create_client(Trigger, 'bad').call_async(Trigger.Request())
    with pytest.raises(TypeError):
        world.drain()
    assert not fut.done()


def test_missing_service(world, node):
    from std_srvs.srv import Trigger
    cli = node.create_client(Trigger, '/nobody')
    assert not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=1.0)
    fut = cli.call_async(Trigger.Request())
    world.drain()
    assert not fut.done()
    with pytest.raises(RuntimeError):
        cli.call(Trigger.Request())
