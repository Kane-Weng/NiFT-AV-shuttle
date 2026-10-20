"""
Tests of the generated message, service and action classes (py/fakeros/_generated).

tools/gen_msgs/golden.json holds what the real Humble rclpy classes give (field types, constants,
defaults, repr). These tests compare the fake classes with it natively, without Docker or ROS.
"""
import array
import copy
import json
import math
import pickle
import sys
from pathlib import Path

import pytest

PLAYGROUND = Path(__file__).resolve().parents[1]
REPO = PLAYGROUND.parent
sys.path.insert(0, str(PLAYGROUND / 'py'))
sys.path.insert(0, str(REPO / 'nift_common'))
sys.path.insert(0, str(PLAYGROUND / 'tools' / 'gen_msgs'))

from fakeros import _generated as gen  # noqa: E402
import gen_msgs  # noqa: E402   (only its USED list and probes; it imports no ROS at module level)

gen.install()

_GOLD = json.loads((PLAYGROUND / 'tools' / 'gen_msgs' / 'golden.json').read_text())
PROBES = _GOLD.pop('_probes')
GOLDEN = _GOLD

USED = gen_msgs.USED


def _class_of(golden_key):
    key, _, part = golden_key.partition('.')
    kind = key.split('/')[1]
    if kind == 'msg':
        return gen.get_message(key)
    top = gen.get_service(key) if kind == 'srv' else gen.get_action(key)
    return getattr(top, part)


_DTYPE = {'char': 'uint8'}


def _norm(v):
    """The same JSON form as gen_msgs.py's _norm of a real rclpy value."""
    if hasattr(v, 'get_fields_and_field_types'):
        return {'msg': type(v).__name__,
                'v': {k: _norm(getattr(v, k)) for k in v.get_fields_and_field_types()}}
    if isinstance(v, gen.FixedArray):
        return {'py': 'fixed', 'dtype': _DTYPE.get(v.dtype, v.dtype), 'v': v.tolist()}
    if isinstance(v, array.array):
        return {'py': 'array', 'tc': v.typecode, 'v': v.tolist()}
    if isinstance(v, (bytes, bytearray)):
        return {'py': 'bytes', 'v': list(v)}
    if isinstance(v, list):
        return {'py': 'list', 'v': [_norm(x) for x in v]}
    if isinstance(v, float) and v != v:
        return {'py': 'float', 'v': 'nan'}
    return {'py': type(v).__name__, 'v': v}


def _json(x):
    return json.loads(json.dumps(x))


# ---------------------------------------------------------------------------------------------
# Every type against the real rclpy
# ---------------------------------------------------------------------------------------------

def test_spec_and_golden_cover_the_same_types():
    keys = set()
    for k in gen.type_names():
        kind = k.split('/')[1]
        if kind == 'msg':
            keys.add(k)
        elif kind == 'srv':
            keys |= {f'{k}.Request', f'{k}.Response'}
        else:
            keys |= {f'{k}.Goal', f'{k}.Result', f'{k}.Feedback'}
    assert keys == set(GOLDEN)


def test_used_types_are_covered():
    for k in USED:
        assert k in gen.type_names(), k


@pytest.mark.parametrize('key', sorted(GOLDEN))
def test_type_matches_rclpy(key):
    g = GOLDEN[key]
    cls = _class_of(key)
    assert cls.__name__ == g['class']
    assert cls.__module__ == g['module']
    # field names, order and IDL type strings
    assert list(cls.get_fields_and_field_types().items()) == list(g['fields'].items())
    # constants: value and Python type
    for name, val in g['consts'].items():
        assert _json(_norm(getattr(cls, name))) == val, name
    # default construction
    inst = cls()
    assert _json(_norm(inst)) == g['default']
    # repr, where no numpy array is involved (FixedArray's repr is only close to numpy's)
    if 'array([' not in g['repr']:
        assert repr(inst) == g['repr']


@pytest.mark.parametrize('key', sorted(GOLDEN))
def test_type_copies_and_compares(key):
    cls = _class_of(key)
    a = cls()
    b = copy.deepcopy(a)
    assert a == b and not (a != b)
    assert pickle.loads(pickle.dumps(a)) == a
    assert copy.copy(a) == a


def _fake_class(t):
    key, _, part = t.partition('.')
    bits = key.split('/')
    if len(bits) == 2 or bits[1] == 'msg':
        return gen.get_message(key)
    return getattr(gen.get_service(key), part)


def _probe_cases():
    spec = json.loads((PLAYGROUND / 'py' / 'fakeros' / '_generated' / 'spec.json').read_text())['types']
    out = {}
    for k, want in PROBES.items():
        type_key, fname, probe, how = k.rsplit('.', 3)
        base, _, part = type_key.partition('.')
        fields = spec[base]['fields'] if not part else spec[base][part.lower()]['fields']
        field = next(f for f in fields if f['n'] == fname)
        out[k] = (type_key, field, probe, how, want)
    return out


_CASES = _probe_cases()


@pytest.mark.parametrize('case', sorted(_CASES))
def test_setter_and_constructor_match_rclpy(case):
    """The same value is accepted or refused, and stored as the same type, as by the real rclpy."""
    type_key, field, probe, how, want = _CASES[case]
    ok, value = gen_msgs.probe_value(probe, field, _fake_class)
    assert ok
    got = gen_msgs.probe_outcome(_fake_class(type_key), field['n'], value, how, _norm)
    assert got == want


def test_import_forms():
    import diagnostic_msgs.msg
    from geometry_msgs.msg import Point
    from std_srvs.srv import Trigger
    from tf2_msgs.action import LookupTransform
    assert diagnostic_msgs.msg.DiagnosticStatus is gen.get_message('diagnostic_msgs/DiagnosticStatus')
    assert Point is gen.get_message('geometry_msgs/msg/Point')
    assert Trigger.Request.__name__ == 'Trigger_Request'
    assert LookupTransform.Goal().target_frame == ''
    with pytest.raises(ImportError):
        from geometry_msgs.msg import NoSuchType  # noqa: F401
    with pytest.raises(ImportError):
        import geometry_msgs.srv  # noqa: F401   (geometry_msgs has no services)


# ---------------------------------------------------------------------------------------------
# rclpy behaviour that our nodes rely on, or that should make their bugs show
# ---------------------------------------------------------------------------------------------

def test_constants():
    from diagnostic_msgs.msg import DiagnosticStatus
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from visualization_msgs.msg import Marker
    assert NavSatStatus.STATUS_FIX == 0 and NavSatStatus.STATUS_NO_FIX == -1
    assert NavSatStatus.STATUS_GBAS_FIX == 2 and NavSatStatus.SERVICE_GPS == 1
    assert NavSatFix.COVARIANCE_TYPE_UNKNOWN == 0 and NavSatFix.COVARIANCE_TYPE_KNOWN == 3
    # byte constants are bytes in rclpy
    assert (DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR, DiagnosticStatus.STALE) == \
        (b'\x00', b'\x01', b'\x02', b'\x03')
    assert max(DiagnosticStatus.OK, DiagnosticStatus.ERROR) == DiagnosticStatus.ERROR
    assert Marker.LINE_STRIP == 4 and Marker.ADD == 0 and Marker.DELETEALL == 3


def test_defaults():
    from geometry_msgs.msg import Quaternion
    from rcl_interfaces.msg import ParameterDescriptor
    from sensor_msgs.msg import NavSatFix
    assert Quaternion().w == 1.0 and Quaternion().x == 0.0
    d = ParameterDescriptor()
    assert d.read_only is False and d.floating_point_range == []
    n = NavSatFix()
    assert n.header.stamp.sec == 0 and n.header.frame_id == ''
    assert len(n.position_covariance) == 9 and all(c == 0.0 for c in n.position_covariance)
    # each instance owns its sub messages
    assert NavSatFix().header is not n.header


def test_typos_and_wrong_types_raise():
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import Point
    from std_msgs.msg import Header, String
    p = Point()
    with pytest.raises(AttributeError):
        p.q = 1.0
    with pytest.raises(AttributeError):
        p.__dict__  # noqa: B018   (slots, no instance dict)
    with pytest.raises(AssertionError, match='Invalid arguments passed to constructor: q'):
        Point(q=1.0)
    with pytest.raises(AssertionError, match="The 'x' field must be of type 'float'"):
        Point(x=1)
    with pytest.raises(AssertionError, match="The 'x' field must be of type 'float'"):
        p.x = '1.0'
    with pytest.raises(AssertionError, match="must be a sub message of type 'Time'"):
        Header(stamp=None)
    with pytest.raises(AssertionError):
        String(data=b'abc')
    with pytest.raises(AssertionError, match='length 1'):
        DiagnosticStatus().level = 0                                   # must be bytes, as in rclpy
    s = DiagnosticStatus(level=DiagnosticStatus.WARN)
    assert s.level == b'\x01'


def test_integer_ranges():
    from can_msgs.msg import Frame
    from std_msgs.msg import Char
    f = Frame()
    f.id = 0x101
    f.dlc = 8
    with pytest.raises(AssertionError, match=r'unsigned integer in \[0, 255\]'):
        f.dlc = 256
    with pytest.raises(AssertionError):
        f.id = -1
    with pytest.raises(AssertionError, match="of type 'int'"):
        f.id = 1.0
    f.dlc = True                                                       # bool is an int, as in rclpy
    assert Char(data=65).data == 65


def test_float_ranges():
    from geometry_msgs.msg import Point32
    from std_msgs.msg import Float64
    Float64(data=math.inf)
    Float64(data=1e300)
    Point32(x=math.inf)
    with pytest.raises(AssertionError):
        Point32(x=1e300)                                               # beyond float32


def test_can_frame_data_like_numpy():
    from can_msgs.msg import Frame
    f = Frame(is_rtr=False, is_error=False, is_extended=False)
    assert len(f.data) == 8 and list(f.data) == [0] * 8
    f.data[0] = 1
    f.data[0:5] = [1, 2, 3, 4, 5]
    assert f.data[:5] == [1, 2, 3, 4, 5]
    assert [int(b) for b in f.data[:5]] == [1, 2, 3, 4, 5]
    assert ' '.join(f'{x:02X}' for x in f.data) == '01 02 03 04 05 00 00 00'
    with pytest.raises(ValueError):
        f.data[0:5] = [1, 2]                                           # cannot change the length
    with pytest.raises(OverflowError):
        f.data[0] = 256
    with pytest.raises(AttributeError):
        f.data.append(0)
    f.data[5:] = 7                                                     # a scalar broadcasts
    assert f.data.tolist() == [1, 2, 3, 4, 5, 7, 7, 7]
    f.data = [9] * 8
    assert f.data == [9] * 8 and isinstance(f.data, gen.FixedArray)
    with pytest.raises(AssertionError):
        f.data = [0] * 7
    g = copy.deepcopy(f)
    g.data[0] = 0
    assert f.data[0] == 9                                              # deep copy owns its array
    assert Frame(data=[1, 2, 3, 4, 5, 6, 7, 8]).data[7] == 8


def test_can_round_trip_through_nift_common():
    """Build frames the way nift_control's can_interface does and decode them with nift_common."""
    from can_msgs.msg import Frame
    from nift_common.can_protocol import (CAN_ID_COMMAND, CAN_ID_ESTOP, compute_steering, decode_command_frame,
                                          decode_estop_frame, DLC_COMMAND, DLC_ESTOP, Gear)
    msg = Frame(is_rtr=False, is_error=False, is_extended=False)
    msg.id = CAN_ID_COMMAND
    msg.data[0:5] = [Gear.FORWARD, 0, 120, *compute_steering(-7.25)]     # Gear is an IntEnum
    msg.dlc = DLC_COMMAND
    cmd = decode_command_frame(copy.deepcopy(msg))
    assert (cmd.gear, cmd.brake, cmd.pedal, cmd.steering_angle_deg) == (Gear.FORWARD, 0, 120, -7.25)
    stop = Frame(is_rtr=False, is_error=False, is_extended=False)
    stop.id = CAN_ID_ESTOP
    stop.data[0] = int(True)
    stop.dlc = DLC_ESTOP
    assert decode_estop_frame(stop) is True


def test_covariance_assign_list():
    from nav_msgs.msg import Odometry
    o = Odometry()
    cov = [0.0] * 36
    cov[35] = 0.5
    o.pose.covariance = cov
    assert o.pose.covariance[35] == 0.5 and len(o.pose.covariance) == 36
    o.pose.covariance[0] = 1                                           # numpy converts an int element
    assert o.pose.covariance[0] == 1.0 and isinstance(o.pose.covariance[0], float)
    with pytest.raises(AssertionError):
        o.pose.covariance = [0] * 36                                   # the setter wants floats, as in rclpy
    with pytest.raises(AssertionError):
        o.pose.covariance = [0.0] * 35


def test_unbounded_numeric_arrays_are_array_array():
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float32MultiArray
    p = PointCloud2()
    assert isinstance(p.data, array.array) and p.data.typecode == 'B'
    p.data = b'\x01\x02\x03'
    assert p.data.tolist() == [1, 2, 3] and isinstance(p.data, array.array)
    m = Float32MultiArray(data=[1.0, 2.5])
    assert m.data.typecode == 'f' and m.data[1] == 2.5
    with pytest.raises(AssertionError):
        m.data = [1, 2]                                                # ints in a float sequence
    with pytest.raises(AssertionError):
        m.data = array.array('d', [1.0])                               # wrong type code


def test_bounded_sequence():
    from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
    d = ParameterDescriptor(floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.0)])
    assert d.floating_point_range[0].to_value == 1.0
    with pytest.raises(AssertionError, match='length <= 1'):
        d.floating_point_range = [FloatingPointRange(), FloatingPointRange()]


def test_message_lists_and_nesting():
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    path = Path()
    path.header.frame_id = 'map'
    for i in range(3):
        ps = PoseStamped()
        ps.pose.position.x = float(i)
        path.poses.append(ps)
    assert [p.pose.position.x for p in path.poses] == [0.0, 1.0, 2.0]
    with pytest.raises(AssertionError, match="of type 'PoseStamped'"):
        path.poses = [1.0]
    c = copy.deepcopy(path)
    c.poses[0].pose.position.x = 9.0
    assert path.poses[0].pose.position.x == 0.0
    assert c != path


def test_shallow_copy_shares_sub_messages():
    from sensor_msgs.msg import NavSatFix
    n = NavSatFix()
    assert copy.copy(n).status is n.status                             # as in rclpy


def test_header_stamp_round_trip():
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    t = TransformStamped()
    t.header.stamp = Time(sec=12, nanosec=500_000_000)
    t.header.frame_id, t.child_frame_id = 'map', 'base_link'
    t.transform.translation.x = 1.5
    t.transform.rotation.z, t.transform.rotation.w = 0.0, 1.0
    tf = TFMessage(transforms=[t])
    back = pickle.loads(pickle.dumps(copy.deepcopy(tf)))
    assert back == tf
    assert back.transforms[0].header.stamp.nanosec == 500_000_000
    assert back.transforms[0].header.stamp is not t.header.stamp


def test_round_trip_used_types():
    """The messages our nodes publish most, filled the way the nodes fill them."""
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Point, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from std_msgs.msg import ColorRGBA, String
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker, MarkerArray

    fix = NavSatFix()
    fix.status.status = NavSatStatus.STATUS_GBAS_FIX
    fix.status.service = NavSatStatus.SERVICE_GPS
    fix.latitude, fix.longitude, fix.altitude = 42.3, -83.7, 260.0
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    st = DiagnosticStatus(level=DiagnosticStatus.OK, name='beacon', message='ok')
    st.values.append(KeyValue(key='hz', value='10.0'))
    diag = DiagnosticArray(status=[st])

    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'map', 'base_link'
    odom.twist.twist.linear.x = 2.0

    tw = TwistStamped()
    tw.twist.angular.z = -0.1

    mk = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns='path', id=3)
    mk.points = [Point(x=1.0, y=2.0), Point(x=3.0)]
    mk.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
    mk.scale.x = 0.2
    arr = MarkerArray(markers=[mk])

    resp = Trigger.Response(success=True, message='started')
    js = String(data=json.dumps({'mode': 'AUTO'}))

    for m in (fix, diag, odom, tw, arr, resp, js):
        c = copy.deepcopy(m)
        assert c == m and c is not m
        assert pickle.loads(pickle.dumps(m)) == m
        assert repr(c) == repr(m)
    assert json.loads(js.data)['mode'] == 'AUTO'
    assert arr.markers[0].points[1].x == 3.0
    assert diag.status[0].values[0].value == '10.0'


def test_shapes_not_in_our_packages(monkeypatch):
    """Field shapes no Humble type of ours has (fixed string, bool and message arrays, bounds, array defaults)."""
    key = 'test_msgs/msg/Shapes'
    monkeypatch.setitem(gen._SPEC, key, {'consts': [{'n': 'NAME', 't': 'string', 'v': 'x#y'}], 'fields': [
        {'n': 'names', 't': 'string', 'a': 3},
        {'n': 'flags', 't': 'bool', 'a': 2},
        {'n': 'raw', 't': 'byte', 'a': 2},
        {'n': 'points', 't': 'geometry_msgs/Point', 'a': 2},
        {'n': 'label', 't': 'string', 'sb': 4},
        {'n': 'tags', 't': 'string', 'sb': 2, 'a': -1, 'ub': 3},
        {'n': 'gains', 't': 'float64', 'a': 3, 'd': [1.0, 2.0, 3.0]},
        {'n': 'ids', 't': 'int32', 'a': -1, 'd': [4, 5]},
        {'n': 'mode', 't': 'string', 'd': 'auto'},
    ]})
    cls = gen.get_message(key)
    m = cls()
    assert cls.NAME == 'x#y'
    assert m.names == ['', '', ''] and m.flags == [False, False] and m.raw == [b'\x00', b'\x00']
    assert len(m.points) == 2 and m.points[0] is not m.points[1]
    assert m.gains.tolist() == [1.0, 2.0, 3.0] and m.ids.tolist() == [4, 5] and m.ids.typecode == 'i'
    assert m.mode == 'auto'
    assert cls.get_fields_and_field_types() == {
        'names': 'string[3]', 'flags': 'boolean[2]', 'raw': 'octet[2]', 'points': 'geometry_msgs/Point[2]',
        'label': 'string<4>', 'tags': 'sequence<string<2>, 3>', 'gains': 'double[3]', 'ids': 'sequence<int32>',
        'mode': 'string'}
    m.label = 'abcd'
    with pytest.raises(AssertionError):
        m.label = 'abcde'
    m.tags = ['ab', 'c']
    with pytest.raises(AssertionError):
        m.tags = ['abc']
    with pytest.raises(AssertionError):
        m.tags = ['a', 'b', 'c', 'd']
    with pytest.raises(AssertionError):
        m.names = ['a', 'b']
    with pytest.raises(AssertionError):
        m.flags = [1, 0]
    assert copy.deepcopy(m) == m
    # a second class build would give a new class; the cache must hand back the same one
    assert gen.get_message(key) is cls
    gen._CLASSES.pop(key)
