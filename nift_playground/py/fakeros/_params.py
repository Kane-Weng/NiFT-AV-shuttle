"""
Parameters: rclpy's Parameter, the rcl params-file parser and the launch precedence rule.

File: fakeros/_params.py

Description:
  Parameter is rclpy's (Parameter.Type, from_parameter_value, check, the ParameterValue
  conversions).

  A parameter source is one of these, as a launch file gives them to a node.
  - A path (str or os.PathLike): a params YAML file, parsed with rcl's rules.
  - A dict: a launch_ros parameter dict. launch_ros flattens it, normalizes its values and
    writes it as a `/**` params file, which rcl then parses. fakeros does the same steps.
  - ParamsFile(content): a params file given as its YAML text or as its parsed dict. A
    node-scoped override file (bringup's components/common.py node_params) is
    node_scoped(name, params).

  rcl's params-file rules (checked in nift_pub_deps:humble).
  - The top-level keys name nodes (`/**`, `/*`, `name`, `/name`, nested keys as a namespace)
    down to `ros__parameters`. Nested maps below it flatten to dotted names.
  - A quoted scalar is a string. Otherwise y/yes/true/on (any case form rcl lists) is a bool,
    then strtol(base 0) is an int (010 is 8, 0x1A is 26), then strtod is a double (1e3, .5, inf),
    else a string ('~' and 'null' are strings). An empty value is an error.
  - A sequence takes the type of its items and all items must have the same type (`[0, 1.5]` is
    an error). An empty sequence gives no value (None, type NOT_SET).
  How the sources combine (ARCH 6.2). rcl merges all files into one table of node keys in the
  order each key first appears, a later file winning within a key. `name` and `/name` are two
  keys. rclpy then applies every key that matches the node, in table order, so a later key wins.
"""
import array
import copy
import enum
import math
import os
import re

import yaml


# ------------------------------------------------------------------ Parameter

class Parameter:
    class Type(enum.IntEnum):
        NOT_SET = 0
        BOOL = 1
        INTEGER = 2
        DOUBLE = 3
        STRING = 4
        BYTE_ARRAY = 5
        BOOL_ARRAY = 6
        INTEGER_ARRAY = 7
        DOUBLE_ARRAY = 8
        STRING_ARRAY = 9

        @classmethod
        def from_parameter_value(cls, parameter_value):
            if parameter_value is None:
                return Parameter.Type.NOT_SET
            if isinstance(parameter_value, bool):
                return Parameter.Type.BOOL
            if isinstance(parameter_value, int):
                return Parameter.Type.INTEGER
            if isinstance(parameter_value, float):
                return Parameter.Type.DOUBLE
            if isinstance(parameter_value, str):
                return Parameter.Type.STRING
            if isinstance(parameter_value, (list, tuple, array.array)):
                if all(isinstance(v, bytes) for v in parameter_value):
                    return Parameter.Type.BYTE_ARRAY
                if all(isinstance(v, bool) for v in parameter_value):
                    return Parameter.Type.BOOL_ARRAY
                if all(isinstance(v, int) for v in parameter_value):
                    return Parameter.Type.INTEGER_ARRAY
                if all(isinstance(v, float) for v in parameter_value):
                    return Parameter.Type.DOUBLE_ARRAY
                if all(isinstance(v, str) for v in parameter_value):
                    return Parameter.Type.STRING_ARRAY
                raise TypeError('The given value is not a list of one of the allowed types'
                                f" '{parameter_value}'.")
            raise TypeError(f"The given value is not one of the allowed types '{parameter_value}'.")

        def check(self, parameter_value):
            T = Parameter.Type
            seq = (list, tuple, array.array)
            if self == T.NOT_SET:
                return parameter_value is None
            if self == T.BOOL:
                return isinstance(parameter_value, bool)
            if self == T.INTEGER:
                return isinstance(parameter_value, int)
            if self == T.DOUBLE:
                return isinstance(parameter_value, float)
            if self == T.STRING:
                return isinstance(parameter_value, str)
            if self == T.BYTE_ARRAY:
                return isinstance(parameter_value, seq) and \
                    all(isinstance(v, bytes) and len(v) == 1 for v in parameter_value)
            if self == T.BOOL_ARRAY:
                return isinstance(parameter_value, seq) and \
                    all(isinstance(v, bool) for v in parameter_value)
            if self == T.INTEGER_ARRAY:
                return isinstance(parameter_value, seq) and \
                    all(isinstance(v, int) for v in parameter_value)
            if self == T.DOUBLE_ARRAY:
                return isinstance(parameter_value, seq) and \
                    all(isinstance(v, float) for v in parameter_value)
            if self == T.STRING_ARRAY:
                return isinstance(parameter_value, seq) and \
                    all(isinstance(v, str) for v in parameter_value)
            return False

    def __init__(self, name, type_=None, value=None):
        if type_ is None:
            type_ = Parameter.Type.from_parameter_value(value)
        if not isinstance(type_, Parameter.Type):
            raise TypeError("type must be an instance of '{}'".format(repr(Parameter.Type)))
        if not type_.check(value):
            raise ValueError("Type '{}' and value '{}' do not agree".format(type_, value))
        self._type_ = type_
        self._name = name
        self._value = value

    @property
    def name(self):
        return self._name

    @property
    def type_(self):
        return self._type_

    @property
    def value(self):
        return self._value

    def __repr__(self):
        return f'Parameter(name={self._name!r}, type_={self._type_}, value={self._value!r})'

    @classmethod
    def from_parameter_msg(cls, param_msg):
        value = parameter_value_to_python(param_msg.value)
        return cls(param_msg.name, Parameter.Type(param_msg.value.type), value)

    def get_parameter_value(self):
        from rcl_interfaces.msg import ParameterValue
        T = Parameter.Type
        pv = ParameterValue(type=int(self._type_))
        v = self._value
        if self._type_ == T.BOOL:
            pv.bool_value = v
        elif self._type_ == T.INTEGER:
            pv.integer_value = v
        elif self._type_ == T.DOUBLE:
            pv.double_value = v
        elif self._type_ == T.STRING:
            pv.string_value = v
        elif self._type_ == T.BYTE_ARRAY:
            pv.byte_array_value = list(v)
        elif self._type_ == T.BOOL_ARRAY:
            pv.bool_array_value = list(v)
        elif self._type_ == T.INTEGER_ARRAY:
            pv.integer_array_value = list(v)
        elif self._type_ == T.DOUBLE_ARRAY:
            pv.double_array_value = list(v)
        elif self._type_ == T.STRING_ARRAY:
            pv.string_array_value = list(v)
        return pv

    def to_parameter_msg(self):
        from rcl_interfaces.msg import Parameter as ParameterMsg
        return ParameterMsg(name=self._name, value=self.get_parameter_value())


def parameter_value_to_python(parameter_value):
    T = Parameter.Type
    t = T(parameter_value.type)
    if t == T.NOT_SET:
        return None
    attr = {T.BOOL: 'bool_value', T.INTEGER: 'integer_value', T.DOUBLE: 'double_value',
            T.STRING: 'string_value', T.BYTE_ARRAY: 'byte_array_value',
            T.BOOL_ARRAY: 'bool_array_value', T.INTEGER_ARRAY: 'integer_array_value',
            T.DOUBLE_ARRAY: 'double_array_value', T.STRING_ARRAY: 'string_array_value'}[t]
    return getattr(parameter_value, attr)


# ------------------------------------------------------------------ rcl scalar rules

_TRUE = frozenset(('Y', 'y', 'yes', 'Yes', 'YES', 'true', 'True', 'TRUE', 'on', 'On', 'ON'))
_FALSE = frozenset(('N', 'n', 'no', 'No', 'NO', 'false', 'False', 'FALSE', 'off', 'Off', 'OFF'))
_STRTOL = re.compile(r'[ \t\n\v\f\r]*([+-]?)(0[xX][0-9a-fA-F]+|0[0-7]*|[1-9][0-9]*)\Z')
_STRTOD_DEC = re.compile(r'[ \t\n\v\f\r]*[+-]?(?:[0-9]+\.?[0-9]*|\.[0-9]+)(?:[eE][+-]?[0-9]+)?\Z')
_STRTOD_HEX = re.compile(r'[ \t\n\v\f\r]*[+-]?0[xX](?:[0-9a-fA-F]+\.?[0-9a-fA-F]*|\.[0-9a-fA-F]+)'
                         r'(?:[pP][+-]?[0-9]+)?\Z')
_STRTOD_SPECIAL = re.compile(r'[ \t\n\v\f\r]*[+-]?(?:inf|infinity|nan(?:\([0-9A-Za-z_]*\))?)\Z',
                             re.IGNORECASE)
_NAN = ('.nan', '.NaN', '.NAN')
_INF = ('.inf', '.Inf', '.INF')
_NINF = ('-.inf', '-.Inf', '-.INF')


def _strtol(text):
    m = _STRTOL.match(text)
    if not m:
        return None
    digits = m.group(2)
    if digits[:2] in ('0x', '0X'):
        v = int(digits, 16)
    elif digits.startswith('0') and len(digits) > 1:
        v = int(digits, 8)
    else:
        v = int(digits)
    if m.group(1) == '-':
        v = -v
    if not (-2 ** 63 <= v < 2 ** 63):            # ERANGE: rcl moves on to strtod
        return None
    return v


def _strtod(text):
    if _STRTOD_DEC.match(text):
        v = float(text.strip())
    elif _STRTOD_HEX.match(text):
        t = text.strip()
        sign = -1.0 if t.startswith('-') else 1.0
        t = t.lstrip('+-')
        if 'p' not in t.lower():
            t += 'p0'
        v = sign * float.fromhex(t)
    elif _STRTOD_SPECIAL.match(text):
        t = text.strip().lower()
        v = float('nan') if 'nan' in t else (float('-inf') if t.startswith('-') else float('inf'))
        return v
    else:
        return None
    # ERANGE (overflow, or underflow to zero or a subnormal): rcl keeps the text as a string
    if math.isinf(v):
        return None
    if abs(v) < 2.2250738585072014e-308:
        mantissa = re.split(r'[eEpP]', text.strip().lstrip('+-'))[0]
        if mantissa[:2] in ('0x', '0X'):
            mantissa = mantissa[2:]
        if any(c not in '0.' for c in mantissa):
            return None
    return v


def rcl_scalar(text, quoted, where=''):
    """A YAML scalar as rcl_yaml_param_parser types it."""
    if quoted:
        return text
    if text == '':
        raise ParamsFileError(f'No value{where}')
    if text in _TRUE:
        return True
    if text in _FALSE:
        return False
    v = _strtol(text)
    if v is not None:
        return v
    if text in _NAN:
        return float('nan')
    if text in _INF:
        return float('inf')
    if text in _NINF:
        return float('-inf')
    v = _strtod(text)
    if v is not None:
        return v
    return text


# ------------------------------------------------------------------ params file parser

class ParamsFileError(ValueError):
    pass


def _events_to_tree(events):
    """('s', text, quoted, line) | ('seq', [nodes], line) | ('map', [(knode, vnode)], line)."""
    ev = next(events)
    line = ev.start_mark.line + 1 if ev.start_mark else 0
    if isinstance(ev, yaml.ScalarEvent):
        return ('s', ev.value, ev.style in ("'", '"'), line)
    if isinstance(ev, yaml.SequenceStartEvent):
        items = []
        while True:
            peek = next(events)
            if isinstance(peek, yaml.SequenceEndEvent):
                return ('seq', items, line)
            items.append(_events_to_tree(_chain(peek, events)))
    if isinstance(ev, yaml.MappingStartEvent):
        pairs = []
        while True:
            peek = next(events)
            if isinstance(peek, yaml.MappingEndEvent):
                return ('map', pairs, line)
            k = _events_to_tree(_chain(peek, events))
            v = _events_to_tree(events)
            pairs.append((k, v))
    if isinstance(ev, yaml.AliasEvent):
        raise ParamsFileError(f'YAML aliases are not supported (line {line})')
    raise ParamsFileError(f'unexpected YAML event {ev!r}')


def _chain(first, rest):
    yield first
    yield from rest


def parse_params_yaml(text, label='<params>'):
    """A params file -> [(node key, {param: value})], keys in first-appearance order."""
    events = iter(yaml.parse(text, Loader=yaml.SafeLoader))
    out = {}
    for ev in events:
        if isinstance(ev, (yaml.StreamStartEvent, yaml.DocumentStartEvent)):
            continue
        if isinstance(ev, (yaml.StreamEndEvent, yaml.DocumentEndEvent)):
            continue
        node = _events_to_tree(_chain(ev, events))
        if node[0] == 's' and node[1] == '' and not node[2]:
            continue                               # an empty document
        if node[0] != 'map':
            raise ParamsFileError(f'{label}: a params file is a map of node names')
        _walk_nodes(node, [], out, label)
    return list(out.items())


def _key_text(knode, label):
    if knode[0] != 's':
        raise ParamsFileError(f'{label}: line {knode[-1]}: a key must be a scalar')
    return knode[1]


def _walk_nodes(mnode, ns, out, label):
    for knode, vnode in mnode[1]:
        key = _key_text(knode, label)
        if key == 'ros__parameters':
            if not ns:
                raise ParamsFileError(f'{label}: line {knode[-1]}: ros__parameters needs a node name')
            if vnode[0] != 'map':
                raise ParamsFileError(f'{label}: line {vnode[-1]}: ros__parameters must be a map')
            params = out.setdefault('/'.join(ns), {})
            _walk_params(vnode, '', params, label)
        elif vnode[0] == 'map':
            _walk_nodes(vnode, ns + [key], out, label)
        else:
            raise ParamsFileError(f'{label}: line {vnode[-1]}: Cannot have a value before '
                                  'ros__parameters')


def _walk_params(mnode, prefix, params, label):
    for knode, vnode in mnode[1]:
        key = _key_text(knode, label)
        name = f'{prefix}.{key}' if prefix else key
        kind = vnode[0]
        where = f' at line {vnode[-1]} ({label})'
        if kind == 'map':
            _walk_params(vnode, name, params, label)
        elif kind == 's':
            params[name] = rcl_scalar(vnode[1], vnode[2], where)
        else:
            params[name] = _sequence(vnode, where)


def _rcl_type(v):
    return type(v) if not isinstance(v, bool) else bool


def _sequence(snode, where):
    items = snode[1]
    if not items:
        return None
    values = []
    for item in items:
        if item[0] != 's':
            raise ParamsFileError(f'Sequences can only have scalars{where}')
        values.append(rcl_scalar(item[1], item[2], where))
    t = _rcl_type(values[0])
    for v in values[1:]:
        if _rcl_type(v) is not t:
            raise ParamsFileError(f"Sequence should be of same type. Value type '{_rcl_type(v).__name__}'"
                                  f' do not belong{where}')
    return values


# ------------------------------------------------------------------ launch_ros dicts

def _flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        name = f'{prefix}.{k}' if prefix else str(k)
        if isinstance(v, dict):
            out.update(_flatten(v, name))
        else:
            out[name] = v
    return out


def _launch_array(name, value):
    kinds = set()
    for v in value:
        if isinstance(v, str):
            kinds.add(str)
        elif isinstance(v, bool):
            kinds.add(bool)
        elif isinstance(v, int):
            kinds.add(int)
        elif isinstance(v, float):
            kinds.add(float)
        else:
            raise TypeError(f'parameter {name}: unexpected list item {v!r}')
    if not kinds:
        raise TypeError(f'parameter {name}: launch_ros refuses an empty list')
    if kinds == {int}:
        return [int(v) for v in value]
    if kinds in ({float}, {int, float}):
        return [float(v) for v in value]
    if kinds == {bool}:
        return [bool(v) for v in value]
    # strings or mixed: each item goes through yaml, then the items must agree
    loaded = [yaml.safe_load(yaml.dump(v)) for v in value]
    sub = type(loaded[0])
    if sub not in (bool, int, float, str) or not all(isinstance(v, sub) for v in loaded):
        raise TypeError(f'parameter {name}: Expected a non-empty sequence, with items of uniform '
                        'type. Allowed sequence item types are bool, int, float, str.')
    return loaded


def launch_dict_yaml(params):
    """A launch_ros parameter dict as the `/**` params file launch_ros writes for it."""
    flat = {}
    for name, value in _flatten(params).items():
        if isinstance(value, (bool, int, float, str)):
            flat[name] = value
        elif isinstance(value, (list, tuple)):
            flat[name] = _launch_array(name, value)
        else:
            raise TypeError(f'parameter {name}: launch_ros does not take {type(value).__name__} '
                            f'({value!r})')
    return yaml.dump({'/**': {'ros__parameters': flat}}, default_flow_style=False)


# ------------------------------------------------------------------ sources

class ParamsFile:
    """A params file given by its content: YAML text, or the dict a YAML file would hold."""

    def __init__(self, content, label=None):
        if not isinstance(content, (str, dict)):
            raise TypeError('ParamsFile takes YAML text or a dict')
        self.content = content
        self.label = label or '<ParamsFile>'

    def text(self):
        if isinstance(self.content, str):
            return self.content
        return yaml.safe_dump(self.content)

    def __repr__(self):
        return f'ParamsFile({self.label})'


def node_scoped(node_name, params, label=None):
    """A params file scoped to one node, as bringup's components/common.py node_params writes."""
    return ParamsFile({node_name: {'ros__parameters': dict(params)}},
                      label or f'<{node_name} node-scoped>')


_FILE_CACHE = {}


def source_entries(src):
    """One parameter source -> [(node key, {param: value})]."""
    if isinstance(src, ParamsFile):
        return parse_params_yaml(src.text(), src.label)
    if isinstance(src, dict):
        return parse_params_yaml(launch_dict_yaml(src), '<launch dict>')
    if isinstance(src, (str, os.PathLike)):
        path = os.fspath(src)
        st = os.stat(path)
        key = (path, st.st_mtime_ns, st.st_size)
        hit = _FILE_CACHE.get(key)
        if hit is None:
            with open(path, encoding='utf-8') as f:
                hit = parse_params_yaml(f.read(), path)
            _FILE_CACHE[key] = hit
        return copy.deepcopy(hit)        # callers may change the lists; the cache must not see it
    raise TypeError(f'unknown parameter source {src!r} (a path, a dict or a ParamsFile)')


def node_key_matches(key, fqn):
    """Does params-file node key `key` apply to the node with fully qualified name `fqn`?"""
    k = key if key.startswith('/') else '/' + key
    if '*' not in k:
        return k == fqn
    rx = ''
    i = 0
    while i < len(k):
        if k.startswith('**', i):
            rx += '.*'
            i += 2
        elif k[i] == '*':
            rx += '[^/]*'
            i += 1
        else:
            rx += re.escape(k[i])
            i += 1
    return re.fullmatch(rx, fqn) is not None


def merge_table(sources):
    """rcl's table: node key -> params, keys in first-appearance order, later files win in a key."""
    table = {}
    for src in sources:
        for key, params in source_entries(src):
            table.setdefault(key, {}).update(params)
    return table


def effective_params(sources, fqn):
    """The parameter overrides a node with fully qualified name `fqn` gets from `sources`."""
    if not fqn.startswith('/'):
        fqn = '/' + fqn
    out = {}
    for key, params in merge_table(sources).items():
        if node_key_matches(key, fqn):
            out.update(params)
    return out
