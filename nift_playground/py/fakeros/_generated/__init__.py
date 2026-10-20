"""
ROS 2 message, service and action classes for the playground, built from spec.json.

File: _generated/__init__.py

Description:
  spec.json is written by tools/gen_msgs/gen_msgs.py from the real Humble .msg, .srv and .action
  files. This module turns it into classes at import time and makes them importable under their
  usual names (`from std_msgs.msg import Header`, `from std_srvs.srv import Trigger`,
  `import diagnostic_msgs.msg`). Call install() once before importing a node.

  The classes behave like the rclpy ones for what our nodes do.
  - Keyword construction, with the .msg defaults (Quaternion w = 1.0) and nested messages.
  - The rclpy setter checks: a float field takes only a float, an int field only an int in range,
    a sub message only its own class. They raise AssertionError with the rclpy wording.
  - __slots__, so a typo like `msg.heder = ...` raises AttributeError. An unknown constructor
    keyword raises AssertionError, as in rclpy.
  - Constants on the class (NavSatStatus.STATUS_FIX, DiagnosticStatus.OK). A `byte` constant
    or field is a 1-byte bytes object (DiagnosticStatus.OK == b'\\x00'), as in rclpy.
  - Unbounded numeric arrays are array.array with the rclpy type code (PointCloud2.data is
    array('B'), so `msg.data = buf.tobytes()` works). Other unbounded arrays are lists.
  - Fixed numeric arrays (Frame.data, covariance) are FixedArray, a numpy-free stand-in for the
    numpy array rclpy uses. Indexing, slicing and slice assignment of the same length work. It
    cannot grow or shrink. Assigning a value out of range raises OverflowError (numpy >= 2 does the
    same; the numpy 1.21 in Humble wraps silently). Writing a float with a fractional part into an
    integer FixedArray (Frame.data) raises TypeError, where numpy truncates it silently (1.5 -> 1);
    a whole float (2.0) is stored as the int 2, as numpy does.
  - ==, copy.copy, copy.deepcopy and repr as in rclpy (repr of a FixedArray is close to numpy's,
    not equal).
"""
import array
import copy
import importlib.abc
import importlib.machinery
import json
import math
import operator
import re
import sys
from collections import UserList, UserString
from collections.abc import Sequence, Set
from pathlib import Path

__all__ = ['FixedArray', 'PACKAGES', 'get_action', 'get_message', 'get_service', 'install', 'type_names']

_SPEC_DOC = json.loads(Path(__file__).with_name('spec.json').read_text())
_SPEC = _SPEC_DOC['types']
PACKAGES = tuple(_SPEC_DOC['packages'])

_INT_RANGE = {
    'char': (0, 255), 'int8': (-2 ** 7, 2 ** 7 - 1), 'uint8': (0, 2 ** 8 - 1),
    'int16': (-2 ** 15, 2 ** 15 - 1), 'uint16': (0, 2 ** 16 - 1),
    'int32': (-2 ** 31, 2 ** 31 - 1), 'uint32': (0, 2 ** 32 - 1),
    'int64': (-2 ** 63, 2 ** 63 - 1), 'uint64': (0, 2 ** 64 - 1),
}
_FLOAT_MAX = {'float32': 3.4028234663852886e+38, 'float64': 1.7976931348623157e+308}
# rclpy keeps these in array.array (unbounded) or numpy (fixed size); char counts as uint8
_TYPECODE = {'float32': 'f', 'float64': 'd', 'int8': 'b', 'uint8': 'B', 'char': 'B', 'int16': 'h',
             'uint16': 'H', 'int32': 'i', 'uint32': 'I', 'int64': 'q', 'uint64': 'Q'}
_IDL = {'float32': 'float', 'float64': 'double', 'bool': 'boolean', 'byte': 'octet', 'char': 'uint8'}
_PY_NAME = {'bool': 'bool', 'byte': 'bytes', 'string': 'str', 'wstring': 'str'}

_CLASSES = {}


# ---------------------------------------------------------------------------------------------
# Scalars
# ---------------------------------------------------------------------------------------------

def _scalar_default(t):
    if t in _INT_RANGE:
        return 0
    if t in _FLOAT_MAX:
        return 0.0
    if t == 'bool':
        return False
    if t == 'byte':
        return b'\x00'
    return ''                                                   # string, wstring


def _from_spec_value(t, v):
    """A default or constant value from spec.json in its rclpy Python type."""
    if t == 'byte':
        return bytes([v])
    if t in _FLOAT_MAX:
        return float(v)
    return v


def _scalar_ok(t, v, sbound=None):
    """None when v is a valid value for the primitive type t, else the reason (rclpy wording)."""
    if t in _FLOAT_MAX:
        if not isinstance(v, float):
            return "of type 'float'"
        m = _FLOAT_MAX[t]
        if (v < -m or v > m) and not math.isinf(v):
            return f'a {"float" if t == "float32" else "double"} in [{-m:f}, {m:f}]'
        return None
    if t in _INT_RANGE:
        if not isinstance(v, int):
            return "of type 'int'"
        lo, hi = _INT_RANGE[t]
        if not lo <= v <= hi:
            return f'{"an unsigned integer" if lo == 0 else "an integer"} in [{lo}, {hi}]'
        return None
    if t == 'bool':
        return None if isinstance(v, bool) else "of type 'bool'"
    if t == 'byte':
        ok = isinstance(v, (bytes, bytearray)) and len(v) == 1
        return None if ok else "of type 'bytes' or 'ByteString' with length 1"
    if not isinstance(v, str):
        return "of type 'str'"
    if sbound is not None and len(v) > sbound:
        return f"of type 'str' and length <= {sbound}"
    return None


def _element(t, v):
    """numpy-like conversion of one element written into a FixedArray."""
    if t in _FLOAT_MAX:
        if isinstance(v, (bool, str, bytes)) or not hasattr(v, '__float__'):
            raise TypeError(f'cannot store {v!r} in a {t} array')
        return float(v)
    if isinstance(v, float) and not v.is_integer():
        raise TypeError(f'cannot store the float {v!r} in a {t} array without losing data')
    if not hasattr(v, '__index__') and not isinstance(v, float):
        raise TypeError(f'cannot store {v!r} in a {t} array')
    i = int(v)
    lo, hi = _INT_RANGE[t]
    if not lo <= i <= hi:
        raise OverflowError(f'Python integer {i} out of bounds for {t}')
    return i


# ---------------------------------------------------------------------------------------------
# FixedArray: the numpy-free stand-in for rclpy's fixed-size numeric arrays
# ---------------------------------------------------------------------------------------------

class FixedArray:
    """A fixed-length numeric array, like the numpy array rclpy gives for `float64[9]` or `uint8[8]`."""

    __slots__ = ('_t', '_v')
    __hash__ = None

    def __init__(self, t, values):
        self._t = t
        self._v = [_element(t, x) for x in values]

    @property
    def dtype(self):
        return self._t

    @property
    def shape(self):
        return (len(self._v),)

    @property
    def size(self):
        return len(self._v)

    def __len__(self):
        return len(self._v)

    def __iter__(self):
        return iter(self._v)

    def __reversed__(self):
        return reversed(self._v)

    def __contains__(self, x):
        return x in self._v

    def __getitem__(self, i):
        return self._v[i]                   # a slice gives a list, not a view as in numpy

    def __setitem__(self, i, value):
        if isinstance(i, slice):
            idx = range(*i.indices(len(self._v)))
            if isinstance(value, (str, bytes)) or not hasattr(value, '__len__'):
                vals = [value] * len(idx)                         # numpy broadcasts a scalar
            else:
                vals = list(value)
            if len(vals) != len(idx):
                raise ValueError(f'could not broadcast input array from shape ({len(vals)},) '
                                 f'into shape ({len(idx)},)')
            for k, v in zip(idx, vals):
                self._v[k] = _element(self._t, v)
        else:
            self._v[i] = _element(self._t, value)

    def __delitem__(self, i):
        raise ValueError('cannot delete an element of a fixed-size array')

    def __eq__(self, other):
        if isinstance(other, FixedArray):
            return self._v == other._v
        if isinstance(other, (list, tuple)):
            return self._v == list(other)
        return NotImplemented

    def __ne__(self, other):
        r = self.__eq__(other)
        return r if r is NotImplemented else not r

    def __repr__(self):
        body = ', '.join(repr(x) for x in self._v)
        return f'array([{body}])' if self._t in _FLOAT_MAX else f'array([{body}], dtype={self._t})'

    def tolist(self):
        return list(self._v)

    def copy(self):
        return self.__copy__()

    def __copy__(self):
        new = FixedArray.__new__(FixedArray)
        new._t, new._v = self._t, list(self._v)
        return new

    def __deepcopy__(self, memo):
        return self.__copy__()

    def __reduce__(self):
        return (FixedArray, (self._t, self._v))


# ---------------------------------------------------------------------------------------------
# Message base and class builder
# ---------------------------------------------------------------------------------------------

class _Msg:
    __slots__ = ()
    __hash__ = None
    _m_fields = ()                    # tuple of _Field
    _fields_and_field_types = {}

    def __init__(self, **kwargs):
        bad = [k for k in kwargs if k not in self._m_names]
        if bad:
            raise AssertionError('Invalid arguments passed to constructor: %s' % ', '.join(sorted(bad)))
        for f in self._m_fields:
            if f.name in kwargs:
                f.init(self, kwargs[f.name])
            else:
                object.__setattr__(self, f.slot, f.default())

    @classmethod
    def get_fields_and_field_types(cls):
        return copy.copy(cls._fields_and_field_types)

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        for s in self.__slots__:
            if getattr(self, s) != getattr(other, s):
                return False
        return True

    def __repr__(self):
        args = []
        for f in self._m_fields:
            v = getattr(self, f.slot)
            if isinstance(v, array.array):
                s = '[]' if len(v) == 0 else repr(v)[len("array('X', "):-1]
            else:
                s = repr(v)
            args.append(f'{f.name}={s}')
        return f'{self._m_typename}({", ".join(args)})'

    def __copy__(self):
        new = object.__new__(type(self))
        for s in self.__slots__:
            object.__setattr__(new, s, getattr(self, s))
        return new

    def __deepcopy__(self, memo):
        new = object.__new__(type(self))
        memo[id(self)] = new
        for s in self.__slots__:
            v = getattr(self, s)
            if type(v) not in (int, float, bool, str, bytes):
                v = copy.deepcopy(v, memo)
            object.__setattr__(new, s, v)
        return new

    def __reduce__(self):
        return (_rebuild, (self._m_key, {f.name: getattr(self, f.slot) for f in self._m_fields}))


def _rebuild(key, values):
    pkg, kind, name = key.split('/', 2)
    if kind == 'msg':
        cls = get_message(key)
    else:
        top, part = name.rsplit('_', 1)
        parent = get_service(f'{pkg}/srv/{top}') if kind == 'srv' else get_action(f'{pkg}/action/{top}')
        cls = getattr(parent, part)
    msg = cls.__new__(cls)
    for f in cls._m_fields:
        object.__setattr__(msg, f.slot, values[f.name])
    return msg


class _Field:
    """One field: its default, its constructor conversion and its setter check."""

    def __init__(self, spec):
        self.name = spec['n']
        self.slot = '_' + self.name
        self.t = spec['t']
        self.size = spec.get('a')                   # None scalar, -1 sequence, N fixed
        self.ub = spec.get('ub')                    # bound of a bounded sequence
        self.sb = spec.get('sb')                    # bound of a bounded string
        self.has_default = 'd' in spec
        self.raw_default = spec.get('d')
        self.nested = '/' in self.t
        self.numeric = self.t in _TYPECODE
        self.short = self.t.split('/')[-1]

    # the class of a nested field is resolved late, so that types can refer to each other
    @property
    def cls(self):
        return _message_class(self.t)

    def idl(self):
        base = self.t if self.nested else _IDL.get(self.t, self.t)
        if self.sb:
            base = f'{base}<{self.sb}>'
        if self.size is None:
            return base
        if self.size >= 0:
            return f'{base}[{self.size}]'
        return f'sequence<{base}, {self.ub}>' if self.ub else f'sequence<{base}>'

    def default(self):
        t = self.t
        if self.size is None:
            if self.nested:
                return self.cls()
            return _from_spec_value(t, self.raw_default) if self.has_default else _scalar_default(t)
        if self.has_default:
            vals = [_from_spec_value(t, v) for v in self.raw_default]
        elif self.size > 0:
            vals = None
        else:
            vals = []
        if self.size == -1:
            if self.numeric:
                return array.array(_TYPECODE[t], vals)
            return vals
        if vals is None:
            if self.nested:
                return [self.cls() for _ in range(self.size)]
            vals = [_scalar_default(t)] * self.size
        return FixedArray(t, vals) if self.numeric else vals

    def init(self, msg, value):
        """Constructor path. rclpy builds an unbounded numeric array with array.array() first."""
        if self.numeric and self.size == -1:
            value = array.array(_TYPECODE[self.t], value)
        self.set(msg, value)

    def set(self, msg, value):
        err = self.check(value)
        if err:
            raise AssertionError(f"The '{self.name}' field must be {err}")
        if self.numeric and self.size is not None:
            if self.size == -1 and not isinstance(value, array.array):
                value = array.array(_TYPECODE[self.t], value)
            elif self.size >= 0 and not isinstance(value, FixedArray):
                if isinstance(value, (bytes, bytearray)):
                    # rclpy passes the checks, then numpy.array(b'...', dtype=uint8) fails
                    raise ValueError(f"invalid literal for int() with base 10: {bytes(value)!r}")
                value = FixedArray(self.t, _tolist(value))
        object.__setattr__(msg, self.slot, value)

    def check(self, value):
        """None when value is valid for this field, else the end of the rclpy error message."""
        if self.size is None:
            if self.nested:
                return None if isinstance(value, self.cls) else f"a sub message of type '{self.short}'"
            return _scalar_ok(self.t, value, self.sb)
        # arrays
        if self.numeric and self.size == -1 and isinstance(value, array.array):
            tc = _TYPECODE[self.t]
            if value.typecode != tc:
                return f"array.array() with the type code of '{tc}'"
            return self._len_err(len(value))
        if self.numeric and self.size >= 0 and (isinstance(value, FixedArray) or hasattr(value, 'dtype')):
            if len(value) != self.size:
                return f'an array with {self.size} elements'
            if isinstance(value, FixedArray) and value.dtype != self.t:
                return f"an array of dtype '{self.t}'"
            return None
        if not isinstance(value, (Sequence, Set, UserList)) or isinstance(value, (str, UserString)):
            return f"a set or sequence of {self._elem_desc()}"
        n = len(value)
        len_err = self._len_err(n)
        if len_err:
            return len_err
        if self.nested:
            cls = self.cls
            bad = any(not isinstance(v, cls) for v in value)
        else:
            bad = any(_scalar_ok(self.t, v, self.sb) for v in value)
        return f'a set or sequence of {self._elem_desc()}' if bad else None

    def _len_err(self, n):
        if self.size >= 0 and n != self.size:
            return f'a set or sequence with length {self.size} and each value {self._elem_desc()}'
        if self.ub is not None and n > self.ub:
            return f'a set or sequence with length <= {self.ub} and each value {self._elem_desc()}'
        return None

    def _elem_desc(self):
        if self.nested:
            return f"of type '{self.short}'"
        if self.t in _FLOAT_MAX:
            return "of type 'float'"
        if self.t in _INT_RANGE:
            return "of type 'int'"
        return f"of type '{_PY_NAME.get(self.t, self.t)}'"


def _tolist(value):
    return value.tolist() if hasattr(value, 'tolist') else list(value)


def _make_property(f):
    def put(self, value):
        f.set(self, value)
    return property(operator.attrgetter(f.slot), put)


def _build(key, spec, class_name, module):
    fields = tuple(_Field(s) for s in spec['fields'])
    pkg = key.split('/')[0]
    kind = key.split('/')[1]
    attrs = {
        '__slots__': tuple(f.slot for f in fields),
        '__module__': module,
        '__qualname__': class_name,
        '_m_fields': fields,
        '_m_names': frozenset(f.name for f in fields),
        '_fields_and_field_types': {f.name: f.idl() for f in fields},
        '_m_key': f'{pkg}/{kind}/{class_name}',
        '_m_typename': f'{pkg}.{kind}.{class_name}',
    }
    for f in fields:
        attrs[f.name] = _make_property(f)
    for c in spec['consts']:
        attrs[c['n']] = _from_spec_value(c['t'], c['v'])
    return type(class_name, (_Msg,), attrs)


def _snake(name):
    """rclpy module name of a type: NavSatFix -> _nav_sat_fix."""
    s = re.sub(r'(.)([A-Z][a-z]+)', r'\1_\2', name)
    return '_' + re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s).lower()


def _message_class(t):
    """t = 'pkg/Name' or 'pkg/msg/Name'."""
    parts = t.split('/')
    key = f'{parts[0]}/msg/{parts[-1]}'
    cls = _CLASSES.get(key)
    if cls is None:
        if key not in _SPEC:
            raise ImportError(f'no message type {key} in spec.json')
        name = parts[-1]
        cls = _CLASSES[key] = _build(key, _SPEC[key], name, f'{parts[0]}.msg.{_snake(name)}')
    return cls


def get_message(key):
    """The class of a message, from 'pkg/msg/Name' or 'pkg/Name'."""
    return _message_class(key)


def _composite(key, kind, parts):
    cls = _CLASSES.get(key)
    if cls is None:
        if key not in _SPEC:
            raise ImportError(f'no {kind} type {key} in spec.json')
        pkg, _, name = key.split('/')
        module = f'{pkg}.{kind}.{_snake(name)}'
        attrs = {'__module__': module, '__qualname__': name, '_m_key': key}
        for attr, part in parts:
            attrs[attr] = _build(key, _SPEC[key][part], f'{name}_{attr}', module)
        cls = _CLASSES[key] = type(name, (), attrs)
    return cls


def get_service(key):
    """The class of a service ('pkg/srv/Name'), with .Request and .Response."""
    return _composite(key, 'srv', (('Request', 'request'), ('Response', 'response')))


def get_action(key):
    """The class of an action ('pkg/action/Name'), with .Goal, .Result and .Feedback."""
    return _composite(key, 'action', (('Goal', 'goal'), ('Result', 'result'), ('Feedback', 'feedback')))


def type_names(kind=None):
    """Every type key in spec.json, optionally only one kind ('msg', 'srv', 'action')."""
    return [k for k in _SPEC if kind is None or k.split('/')[1] == kind]


# ---------------------------------------------------------------------------------------------
# Import hook: `std_msgs`, `std_msgs.msg`, `std_srvs.srv`, `tf2_msgs.action`
# ---------------------------------------------------------------------------------------------

_KINDS = {'msg': get_message, 'srv': get_service, 'action': get_action}
_BY_MODULE = {}
for _k in _SPEC:
    _p, _kind, _n = _k.split('/')
    _BY_MODULE.setdefault(f'{_p}.{_kind}', []).append((_n, _k))


class _Loader(importlib.abc.Loader):
    def create_module(self, spec):
        return None

    def exec_module(self, module):
        parts = module.__name__.split('.')
        if len(parts) == 1:
            module.__path__ = []
            return
        get = _KINDS[parts[1]]
        for name, key in _BY_MODULE[module.__name__]:
            setattr(module, name, get(key))


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        parts = fullname.split('.')
        if parts[0] not in PACKAGES or len(parts) > 2:
            return None
        if len(parts) == 2 and fullname not in _BY_MODULE:
            return None
        return importlib.machinery.ModuleSpec(fullname, _Loader(), is_package=len(parts) == 1)


def install():
    """Make the message packages importable. Safe to call more than once."""
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())
