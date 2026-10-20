"""tf2_ros (Buffer, listener, broadcasters) and diagnostic_updater, against Humble's outputs."""
import math

import pytest


def tf(sec, x, child='base_link', parent='map', yaw=0.0):
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp.sec = int(sec)
    t.header.stamp.nanosec = int(round((sec - int(sec)) * 1e9))
    t.transform.translation.x = float(x)
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t


def T(s):
    from rclpy.time import Time
    return Time(seconds=s)


def x_at(buf, s, target='map', source='base_link'):
    return buf.lookup_transform(target, source, T(s)).transform.translation.x


# ------------------------------------------------------------------ Buffer

def test_empty_and_missing_frames(world):
    import tf2_ros
    buf = tf2_ros.Buffer()
    with pytest.raises(tf2_ros.LookupException) as e:
        buf.lookup_transform('map', 'base_link', T(0))
    assert str(e.value) == '"map" passed to lookupTransform argument target_frame does not exist. '
    assert isinstance(e.value, tf2_ros.TransformException)
    buf.set_transform(tf(5, 1), 'a')
    with pytest.raises(tf2_ros.LookupException, match='"odom" passed to lookupTransform argument source_frame'):
        buf.lookup_transform('map', 'odom', T(0))
    buf.set_transform(tf(6, 0, 'x', 'odom'), 'a')
    with pytest.raises(tf2_ros.ConnectivityException, match="between 'map' and 'x'"):
        buf.lookup_transform('map', 'x', T(0))
    assert buf.can_transform('map', 'base_link', T(0)) and not buf.can_transform('map', 'x', T(0))


def test_latest_interpolation_inverse_and_extrapolation(world):
    import tf2_ros
    buf = tf2_ros.Buffer()
    buf.set_transform(tf(5, 1), 'a')
    buf.set_transform(tf(6, 3), 'a')
    r = buf.lookup_transform('map', 'base_link', T(0))
    assert (r.header.stamp.sec, r.transform.translation.x) == (6, 3.0)
    assert (r.header.frame_id, r.child_frame_id) == ('map', 'base_link')
    r = buf.lookup_transform('map', 'base_link', T(5.5))
    assert (r.header.stamp.sec, r.header.stamp.nanosec, r.transform.translation.x) == (5, 500_000_000, 2.0)
    assert x_at(buf, 0, 'base_link', 'map') == -3.0
    with pytest.raises(tf2_ros.ExtrapolationException) as e:
        x_at(buf, 7)
    assert str(e.value) == ('Lookup would require extrapolation into the future.  Requested time 7.000000 but '
                            'the latest data is at time 6.000000, when looking up transform from frame '
                            '[base_link] to frame [map]')
    with pytest.raises(tf2_ros.ExtrapolationException, match='into the past.  Requested time 4.000000 but the '
                                                             'earliest data is at time 5.000000'):
        buf.lookup_transform('map', 'base_link', T(4), timeout=__import__('rclpy').duration.Duration(seconds=1))


def test_single_sample(world):
    import tf2_ros
    buf = tf2_ros.Buffer()
    buf.set_transform(tf(5, 1), 'a')
    assert x_at(buf, 5) == 1.0 and x_at(buf, 0) == 1.0
    with pytest.raises(tf2_ros.ExtrapolationException, match='only time 5.000000 is in the buffer'):
        x_at(buf, 6)


def test_cache_window_and_old_data(world):
    import tf2_ros
    buf = tf2_ros.Buffer()
    for sec, x in ((5, 1), (6, 3), (30, 0)):
        buf.set_transform(tf(sec, x), 'a')
    assert x_at(buf, 6) == 3.0                     # pruned only before the next insert
    assert x_at(buf, 25) == pytest.approx(0.625)
    buf.set_transform(tf(31, 10), 'a')
    with pytest.raises(tf2_ros.ExtrapolationException, match='earliest data is at time 30.000000'):
        x_at(buf, 6)
    assert x_at(buf, 30.5) == 5.0
    assert not buf.set_transform(tf(10, 9), 'a')   # TF_OLD_DATA
    assert any('TF_OLD_DATA' in text for _, _, text in world.logs)
    assert x_at(buf, 0) == 10.0


def test_same_stamp_replaces(world):
    import tf2_ros
    buf = tf2_ros.Buffer()
    buf.set_transform(tf(5, 1), 'a')
    buf.set_transform(tf(5, 2), 'a')
    assert x_at(buf, 0) == 2.0


def test_static_and_chains(world):
    import tf2_ros
    buf = tf2_ros.Buffer()
    buf.set_transform(tf(5, 1), 'a')
    buf.set_transform(tf(6, 3), 'a')
    buf.set_transform_static(tf(1, 0.5, 'gnss', 'base_link'), 'a')
    r = buf.lookup_transform('map', 'gnss', T(0))
    assert (r.header.stamp.sec, r.transform.translation.x) == (6, 3.5)
    r = buf.lookup_transform('map', 'gnss', T(5.5))
    assert r.transform.translation.x == 2.5
    for s in (0, 5, 100):                          # a static link holds at any time
        r = buf.lookup_transform('base_link', 'gnss', T(s))
        assert (r.header.stamp.sec, r.transform.translation.x) == (s, 0.5)


def test_same_frame_is_identity_at_requested_time(world):
    import tf2_ros
    buf = tf2_ros.Buffer()
    r = buf.lookup_transform('nowhere', 'nowhere', T(0))
    assert r.header.stamp.sec == 0 and r.transform.rotation.w == 1.0


def test_rotation_slerp_and_composition(world):
    import tf2_ros
    buf = tf2_ros.Buffer()
    buf.set_transform(tf(1, 0, yaw=0.0), 'a')
    buf.set_transform(tf(2, 0, yaw=1.0), 'a')
    q = buf.lookup_transform('map', 'base_link', T(1.25)).transform.rotation
    assert q.z == pytest.approx(math.sin(0.125), abs=1e-12) and q.w == pytest.approx(math.cos(0.125), abs=1e-12)
    buf2 = tf2_ros.Buffer()
    buf2.set_transform(tf(1, 2.0, yaw=math.pi / 2), 'a')
    buf2.set_transform_static(tf(0, 1.0, 'gnss', 'base_link'), 'a')
    r = buf2.lookup_transform('map', 'gnss', T(0)).transform.translation
    assert (r.x, r.y) == (pytest.approx(2.0), pytest.approx(1.0))


def test_leading_slash_is_stripped(world):
    import tf2_ros
    buf = tf2_ros.Buffer()
    buf.set_transform(tf(1, 1, 'base_link', '/map'), 'a')
    assert buf.lookup_transform('map', 'base_link', T(0)).header.frame_id == 'map'


def test_broadcaster_listener_on_the_graph(world, node):
    import tf2_ros
    from rclpy.node import Node
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    other = Node('broadcaster')
    tf2_ros.TransformBroadcaster(other).sendTransform(tf(1, 4))
    with pytest.raises(tf2_ros.TransformException):
        buf.lookup_transform('map', 'base_link', T(0))           # not delivered yet
    world.drain()
    assert x_at(buf, 0) == 4.0
    st = tf2_ros.StaticTransformBroadcaster(other)
    st.sendTransform(tf(0, 0.5, 'gnss', 'base_link'))
    st.sendTransform(tf(0, 0.7, 'gnss', 'base_link'))         # replaces by child frame
    world.drain()
    late = tf2_ros.Buffer()
    tf2_ros.TransformListener(late, Node('late'))               # latched /tf_static
    world.drain()
    assert x_at(late, 0, 'base_link', 'gnss') == 0.7


# ------------------------------------------------------------------ diagnostic_updater

def test_updater_like_humble(world):
    import diagnostic_updater
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from fakeros import ParamsFile
    from rclpy.node import Node
    world.configure_node('nift_beacon_node', name='beacon_node', parameters=[
        ParamsFile('beacon_node:\n  ros__parameters:\n    diagnostic_updater:\n      period: 0.5\n')])
    n = Node('nift_beacon_node')
    got = []
    n.create_subscription(DiagnosticArray, '/diagnostics', got.append, 10)
    up = diagnostic_updater.Updater(n)
    assert up.period == 0.5 and n.get_parameter('diagnostic_updater.use_fqn').value is False
    up.setHardwareID('gnss_beacon')

    def task(stat):
        stat.summary(DiagnosticStatus.WARN, 'careful')
        stat.add('k', 'v')
        return stat
    up.add('GNSS Interface', task)
    up.add('Silent', lambda stat: stat)
    world.drain()
    start = [s for a in got for s in a.status]
    assert [(s.name, s.level, s.message) for s in start] == [
        ('beacon_node: GNSS Interface', DiagnosticStatus.OK, 'Node starting up'),
        ('beacon_node: Silent', DiagnosticStatus.OK, 'Node starting up')]
    got.clear()
    world.fire_timers(world.now_ns + 500_000_000)
    world.drain()
    (arr,) = got
    a, b = arr.status
    assert (a.name, a.level, a.message, a.hardware_id) == ('beacon_node: GNSS Interface', b'\x01', 'careful',
                                                           'gnss_beacon')
    assert [(kv.key, kv.value) for kv in a.values] == [('k', 'v')]
    assert (b.level, b.message) == (DiagnosticStatus.ERROR, 'No message was set')
    assert type(a) is DiagnosticStatus
    assert (arr.header.stamp.sec, arr.header.stamp.nanosec) == divmod(world.now_ns, 1_000_000_000)


def test_wrapper_add_keeps_message_types_strict(world):
    import diagnostic_updater
    st = diagnostic_updater.DiagnosticStatusWrapper()
    with pytest.raises(AssertionError):
        st.add('k', 1.5)


def test_missing_hardware_id_warns_once(world, node):
    import diagnostic_updater
    up = diagnostic_updater.Updater(node)
    up.add('t', lambda s: (s.summary(diagnostic_updater.OK, 'fine'), s)[1])
    up.update()
    up.update()
    assert sum('No HW_ID was set' in text for _, _, text in world.logs) == 1


def test_merge_summary():
    import diagnostic_updater
    from diagnostic_msgs.msg import DiagnosticStatus
    st = diagnostic_updater.DiagnosticStatusWrapper()
    st.summary(DiagnosticStatus.OK, 'a')
    st.mergeSummary(DiagnosticStatus.WARN, 'b')
    st.mergeSummary(DiagnosticStatus.ERROR, 'c')
    assert (st.level, st.message) == (DiagnosticStatus.ERROR, 'b; c')
