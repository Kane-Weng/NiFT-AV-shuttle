"""
Review fixes (2026-10-20) and the reset contract: world.reset() leaves fakeros as a fresh process would.

M1 cached params files are not shared, M2 a call site keeps its severity, L1 uninitialized static
parameters, L2 KEEP_LAST needs a depth, L4 filter order, L5 logger levels reset, L6 destroyed node,
L7 same-frame TF stamp.
"""
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
START_NS = 1_000_000_000
DT_NS = 50_000_000


def _paths():
    py = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'py')
    for p in (py, os.path.join(py, 'fakeros')):
        if p not in sys.path:
            sys.path.insert(0, p)
    import fakeros
    fakeros.install()


# ------------------------------------------------------------------ M1

def test_params_file_cache_is_not_shared(world, tmp_path):
    from fakeros import effective_params
    p = tmp_path / 'p.yaml'
    p.write_text('n:\n  ros__parameters:\n    arr: [1.0, 2.0]\n')
    first = effective_params([str(p)], '/n')
    first['arr'].append(99.0)
    world.reset()
    assert effective_params([str(p)], '/n')['arr'] == [1.0, 2.0]


# ------------------------------------------------------------------ M2, L4

def test_call_site_keeps_its_severity(node):
    lg = node.get_logger()

    def site(level):
        return getattr(lg, level)('x')
    assert site('info')
    with pytest.raises(ValueError, match='Logger severity cannot be changed between calls.'):
        site('error')


def test_call_site_keeps_its_filters(node):
    lg = node.get_logger()

    def site(**kw):
        return lg.info('x', **kw)
    site(throttle_duration_sec=1.0)
    with pytest.raises(ValueError, match='filter parameters cannot be changed'):
        site(throttle_duration_sec=2.0)
    with pytest.raises(ValueError, match='filters cannot be changed'):
        site()


def test_disabled_severity_records_nothing(node):
    lg = node.get_logger()

    def site(level):
        return getattr(lg, level)('x')
    assert site('debug') is False             # below INFO: no context made
    assert site('info') is True


def test_unknown_filter_keyword_is_a_type_error(node):
    with pytest.raises(TypeError):
        node.get_logger().info('x', bogus=1)


def test_filter_order_throttle_skip_first_once(world, node):
    lg = node.get_logger()

    def site():
        return lg.info('x', throttle_duration_sec=1.0, skip_first=True, once=True)
    out = []
    for _ in range(4):
        out.append(site())
        world.now_ns += 1_000_000_000
    # throttle passes, skip_first swallows the first, once lets the next one through, then nothing
    assert out == [False, True, False, False]


# ------------------------------------------------------------------ L1

def test_uninitialized_static_parameter(world):
    from fakeros import ParamsFile
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    world.configure_node('n', parameters=[ParamsFile('n:\n  ros__parameters:\n    sz: []\n')])
    n = Node('n', parameter_overrides=[Parameter('ov', Parameter.Type.NOT_SET, None)])
    assert n.declare_parameter('sz', [0.5]).value is None       # declare returns it unchecked
    with pytest.raises(ParameterUninitializedException, match="The parameter 'sz' is not initialized"):
        n.get_parameter('sz')
    n.declare_parameter('ov', 1.0)
    with pytest.raises(ParameterUninitializedException):
        n.get_parameter('ov')
    assert n.get_parameter_or('ov').type_ == Parameter.Type.NOT_SET


# ------------------------------------------------------------------ L2

def test_keep_last_needs_a_depth():
    from rclpy.qos import HistoryPolicy, InvalidQoSProfileException, QoSProfile
    with pytest.raises(InvalidQoSProfileException):
        QoSProfile(history=HistoryPolicy.KEEP_LAST)
    assert QoSProfile(history=HistoryPolicy.KEEP_ALL).history == HistoryPolicy.KEEP_ALL


# ------------------------------------------------------------------ L5

def test_reset_clears_logger_levels(world):
    import rclpy.logging
    rclpy.logging.set_logger_level('quiet', rclpy.logging.LoggingSeverity.ERROR)
    lg = rclpy.logging.get_logger('quiet')
    assert lg.info('x') is False
    world.reset()
    world.log_sink = lambda *a: None
    assert lg.info('x') is True


# ------------------------------------------------------------------ L6

def test_destroyed_node_refuses_new_entities(world):
    from rclpy.exceptions import InvalidHandle
    from rclpy.node import Node
    from std_msgs.msg import String
    n = Node('gone')
    n.destroy_node()
    with pytest.raises(InvalidHandle):
        n.create_publisher(String, '/x', 1)
    with pytest.raises(InvalidHandle):
        n.create_timer(1.0, lambda: None)


# ------------------------------------------------------------------ L7

def test_same_frame_latest_stamp(world):
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time

    def tf(sec, child, parent):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.header.stamp.sec = sec
        t.transform.rotation.w = 1.0
        return t
    buf = tf2_ros.Buffer()
    buf.set_transform(tf(5, 'base_link', 'map'), 'a')
    buf.set_transform(tf(6, 'base_link', 'map'), 'a')
    buf.set_transform_static(tf(2, 'gnss', 'base_link'), 'a')
    stamp = lambda f, s=0: buf.lookup_transform(f, f, Time(seconds=s)).header.stamp  # noqa: E731
    assert (stamp('base_link').sec, stamp('map').sec, stamp('gnss').sec, stamp('zz').sec) == (6, 0, 0, 0)
    assert (stamp('base_link', 5.5).sec, stamp('base_link', 5.5).nanosec) == (5, 500_000_000)


# ------------------------------------------------------------------ reset == fresh process

def scenario():
    """Timers, messages, latching, services, TF, diagnostics, parameters and filtered logs."""
    _paths()
    import diagnostic_updater
    import rclpy.logging
    import tf2_ros
    from fakeros import ParamsFile
    from fakeros._core import world
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    world.reset(start_ns=START_NS - DT_NS)
    trace = []
    world.log_sink = lambda lv, name, text: trace.append(['log', world.now_ns, lv, name, text])
    module_logger = rclpy.logging.get_logger('module_level')
    world.configure_node('own', name='launch', parameters=[
        ParamsFile('launch:\n  ros__parameters:\n    rate: 0.02\n    names: [a, b]\n'), {'use_sim_time': True}])
    a = Node('own')
    b = Node('other')
    rate = a.declare_parameter('rate', 0.1).value
    trace.append(['params', a.get_name(), rate, a.declare_parameter('names', ['']).value,
                  a.get_parameter('use_sim_time').value])
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = a.create_publisher(String, '/s', latched)
    tf_out = tf2_ros.TransformBroadcaster(a)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, b)
    up = diagnostic_updater.Updater(a)
    up.setHardwareID('hw')
    up.add('task', lambda st: (st.summary(diagnostic_updater.OK, f'n={world.now_ns}'), st)[1])
    b.create_subscription(String, '/s', lambda m: trace.append(['msg', world.now_ns, m.data]), latched)
    a.create_service(Trigger, 'svc', lambda req, resp: (setattr(resp, 'success', True), resp)[1])
    cli = b.create_client(Trigger, '/svc')
    count = [0]

    def tick():
        count[0] += 1
        pub.publish(String(data=str(count[0])))
        t = TransformStamped()
        t.header.stamp = a.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.transform.translation.x = float(count[0])
        t.transform.rotation.w = 1.0
        tf_out.sendTransform(t)
        a.get_logger().warn(f'tick {count[0]}', throttle_duration_sec=0.3)
        module_logger.info('once', once=True)
        if count[0] % 7 == 0:
            fut = cli.call_async(Trigger.Request())
            fut.add_done_callback(lambda f: trace.append(['reply', world.now_ns, f.result().success]))
    a.create_timer(rate, tick)
    b.create_timer(0.1, lambda: trace.append(['tf', world.now_ns, buf.lookup_transform(
        'map', 'base_link', rclpy.time.Time()).transform.translation.x]))
    for k in range(1, 41):
        world.step(START_NS - DT_NS + k * DT_NS)
    return trace


def test_reset_equals_fresh_process():
    _paths()
    from fakeros._core import world
    dirty = scenario()                  # leaves loggers, caches and module state behind
    import rclpy.logging
    rclpy.logging.set_logger_level('module_level', rclpy.logging.LoggingSeverity.FATAL)
    again = scenario()                  # reset inside: must not see any of it
    world.reset()
    assert len(dirty) > 50
    assert again == dirty
    if sys.platform == 'emscripten':                  # Pyodide has no processes
        return
    fresh = json.loads(subprocess.run([sys.executable, __file__], check=True, capture_output=True,
                                      text=True, env={**os.environ, 'PYTHONPATH': ''}).stdout)
    assert fresh == dirty


if __name__ == '__main__':
    print(json.dumps(scenario()))
