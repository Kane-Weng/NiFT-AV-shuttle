#!/usr/bin/env python3
"""
Generate the playground's ROS message, service and action definitions from the Humble interface files.

File: gen_msgs.py

Description:
  The browser has no /opt/ros. So the fake ROS builds its message classes at import time from a JSON
  spec (py/fakeros/_generated/spec.json, read by py/fakeros/_generated/__init__.py). This script
  writes that spec from the real Humble .msg, .srv and .action files, parsed with rosidl_adapter (the
  parser the ROS build itself uses).

  It also writes tools/gen_msgs/golden.json from the real rclpy classes: field types, constants and
  default values of every type. tests/test_msgs.py checks the fake classes against it natively, so
  the tests need no Docker.

Usage (one command, from anywhere):
  python3 nift_playground/tools/gen_msgs/gen_msgs.py

  Outside ROS it runs itself again inside the nift_pub_deps:humble image (it has can_msgs, which
  ros:humble lacks). Pass --image to pick another image. Inside a sourced Humble shell it runs
  directly.

  nift_pub_deps:humble is built locally from this repo's Docker setup. Where it is missing, the
  script falls back on the public ros:humble image and installs ros-humble-can-msgs with apt in
  the throwaway container first (needs network; the outputs are then given back to your user).
  The same fallback runs with --image ros:humble.
"""
import argparse
import array
import importlib
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLAYGROUND = HERE.parents[1]
SPEC_OUT = PLAYGROUND / 'py' / 'fakeros' / '_generated' / 'spec.json'
GOLDEN_OUT = HERE / 'golden.json'
SHARE = Path('/opt/ros/humble/share')
DEFAULT_IMAGE = 'nift_pub_deps:humble'

# Every package a playground node imports, plus what the fake rclpy, tf2_ros and diagnostic_updater
# need (rcl_interfaces, rosgraph_msgs, tf2_msgs) and their dependencies. Whole packages are kept so
# that a node can import any type of them, as on a real system.
PACKAGES = [
    'builtin_interfaces', 'std_msgs', 'std_srvs', 'geometry_msgs', 'nav_msgs', 'sensor_msgs',
    'visualization_msgs', 'diagnostic_msgs', 'tf2_msgs', 'rosgraph_msgs', 'can_msgs',
    'rcl_interfaces', 'unique_identifier_msgs', 'action_msgs',
]

# Every type our nodes import (grep of nift_common, nift_sensor, nift_waypoint, nift_control and
# nift_simulation/nift_carla), plus what the fake rclpy, tf2_ros and diagnostic_updater publish.
# The golden file records setter and constructor behaviour of each of their fields (probes below).
USED = [
    'builtin_interfaces/msg/Time', 'builtin_interfaces/msg/Duration',
    'can_msgs/msg/Frame',
    'diagnostic_msgs/msg/DiagnosticArray', 'diagnostic_msgs/msg/DiagnosticStatus', 'diagnostic_msgs/msg/KeyValue',
    'geometry_msgs/msg/Point', 'geometry_msgs/msg/Point32', 'geometry_msgs/msg/PolygonStamped',
    'geometry_msgs/msg/PoseStamped', 'geometry_msgs/msg/PoseWithCovarianceStamped', 'geometry_msgs/msg/Quaternion',
    'geometry_msgs/msg/TransformStamped', 'geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped',
    'nav_msgs/msg/Odometry', 'nav_msgs/msg/Path',
    'rcl_interfaces/msg/ParameterDescriptor', 'rcl_interfaces/msg/SetParametersResult',
    'rosgraph_msgs/msg/Clock',
    'sensor_msgs/msg/Image', 'sensor_msgs/msg/NavSatFix', 'sensor_msgs/msg/NavSatStatus',
    'sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/PointField',
    'std_msgs/msg/Bool', 'std_msgs/msg/ColorRGBA', 'std_msgs/msg/Float64', 'std_msgs/msg/Header',
    'std_msgs/msg/String',
    'tf2_msgs/msg/TFMessage',
    'visualization_msgs/msg/Marker', 'visualization_msgs/msg/MarkerArray',
    'std_srvs/srv/Trigger',
]

PROBES = ['int', 'float', 'bool', 'str', 'bytes1', 'bytes2', 'none', 'big', 'neg', 'huge', 'inf',
          'list0', 'listf', 'listi', 'listf_long', 'tuplef', 'bytesn', 'own', 'other', 'list_own']


def probe_value(name, field, get_class):
    """
    The probe value `name` for a field of spec.json. get_class('pkg/Name') gives a message class.

    Shared by this script (real rclpy) and tests/test_msgs.py (fake classes), so both try the same
    values. Returns (False, None) when the probe does not apply to the field.
    """
    n = field['a'] if field.get('a', -1) > 0 else 2
    nested = '/' in field['t']
    simple = {
        'int': 1, 'float': 1.0, 'bool': True, 'str': 'a', 'bytes1': b'a', 'bytes2': b'ab', 'none': None,
        'big': 2 ** 64, 'neg': -1, 'huge': 1e300, 'inf': float('inf'), 'list0': [], 'listf': [1.0] * n,
        'listi': [1] * n, 'listf_long': [1.0] * (n + 1), 'tuplef': tuple([1.0] * n), 'bytesn': bytes(n),
    }
    if name in simple:
        return True, simple[name]
    if name == 'other':
        return True, get_class('builtin_interfaces/Duration')()
    if not nested:
        return False, None
    cls = get_class(field['t'])
    return True, (cls() if name == 'own' else [cls() for _ in range(n)])


def probe_outcome(cls, fname, value, how, norm):
    """'ok:<json of the stored value>' or the exception class name."""
    try:
        if how == 'ctor':
            inst = cls(**{fname: value})
        else:
            inst = cls()
            setattr(inst, fname, value)
    except Exception as e:  # noqa: BLE001   (the class of any error is the result)
        return type(e).__name__
    return 'ok:' + json.dumps(norm(getattr(inst, fname)), sort_keys=True)


def probes(types, get_class, norm):
    """{'pkg/msg/Name.field.probe.how': outcome} for every field of the USED types."""
    out = {}
    for key in USED:
        pkg, kind, name = key.split('/')
        parts = {'': types[key]} if kind == 'msg' else {
            '.Request': types[key]['request'], '.Response': types[key]['response']}
        for suffix, part in parts.items():
            cls = get_class(key + suffix)
            for f in part['fields']:
                for p in PROBES:
                    ok, value = probe_value(p, f, get_class)
                    if not ok:
                        continue
                    for how in ('set', 'ctor'):
                        out[f'{key}{suffix}.{f["n"]}.{p}.{how}'] = probe_outcome(cls, f['n'], value, how, norm)
    return out


# ---------------------------------------------------------------------------------------------
# Parsing (inside ROS)
# ---------------------------------------------------------------------------------------------

def _field(f):
    t = f.type
    out = {'n': f.name}
    if t.is_primitive_type():
        out['t'] = t.type
    else:
        out['t'] = f'{t.pkg_name}/{t.type}'
    if t.string_upper_bound:
        out['sb'] = t.string_upper_bound
    if t.is_array:
        if t.array_size is None:
            out['a'] = -1
        elif t.is_upper_bound:
            out['a'] = -1
            out['ub'] = t.array_size
        else:
            out['a'] = t.array_size
    if f.default_value is not None:
        out['d'] = f.default_value
    return out


def _spec(msg_spec):
    return {
        'fields': [_field(f) for f in msg_spec.fields],
        'consts': [{'n': c.name, 't': c.type, 'v': c.value} for c in msg_spec.constants],
    }


def parse_all():
    from rosidl_adapter.parser import parse_action_file, parse_message_file, parse_service_file

    types = {}
    for pkg in PACKAGES:
        root = SHARE / pkg
        if not root.is_dir():
            raise SystemExit(f'package {pkg} is not installed in this image')
        for f in sorted((root / 'msg').glob('*.msg')):
            types[f'{pkg}/msg/{f.stem}'] = _spec(parse_message_file(pkg, str(f)))
        for f in sorted((root / 'srv').glob('*.srv')):
            s = parse_service_file(pkg, str(f))
            types[f'{pkg}/srv/{f.stem}'] = {'request': _spec(s.request), 'response': _spec(s.response)}
        for f in sorted((root / 'action').glob('*.action')):
            a = parse_action_file(pkg, str(f))
            types[f'{pkg}/action/{f.stem}'] = {
                'goal': _spec(a.goal), 'result': _spec(a.result), 'feedback': _spec(a.feedback)}
    _check_closure(types)
    return types


def _check_closure(types):
    missing = set()
    for spec in types.values():
        for part in (spec,) if 'fields' in spec else spec.values():
            for f in part['fields']:
                if '/' in f['t']:
                    pkg, name = f['t'].split('/')
                    if f'{pkg}/msg/{name}' not in types:
                        missing.add(f['t'])
    if missing:
        raise SystemExit(f'field types outside PACKAGES: {sorted(missing)}')


# ---------------------------------------------------------------------------------------------
# Golden values from the real rclpy classes (inside ROS)
# ---------------------------------------------------------------------------------------------

def _norm(v):
    """A JSON form of a real rclpy value that keeps its Python type."""
    import numpy
    if hasattr(v, 'get_fields_and_field_types'):
        return {'msg': type(v).__name__,
                'v': {k: _norm(getattr(v, k)) for k in v.get_fields_and_field_types()}}
    if isinstance(v, numpy.ndarray):
        return {'py': 'fixed', 'dtype': str(v.dtype), 'v': v.tolist()}
    if isinstance(v, array.array):
        return {'py': 'array', 'tc': v.typecode, 'v': v.tolist()}
    if isinstance(v, (bytes, bytearray)):
        return {'py': 'bytes', 'v': list(v)}
    if isinstance(v, list):
        return {'py': 'list', 'v': [_norm(x) for x in v]}
    if isinstance(v, float) and v != v:
        return {'py': 'float', 'v': 'nan'}
    return {'py': type(v).__name__, 'v': v}


def golden(types):
    out = {}
    for key, spec in types.items():
        pkg, kind, name = key.split('/')
        mod = importlib.import_module(f'{pkg}.{kind}')
        top = getattr(mod, name)
        if kind == 'msg':
            parts = {'': (top, spec)}
        elif kind == 'srv':
            parts = {'Request': (top.Request, spec['request']), 'Response': (top.Response, spec['response'])}
        else:
            parts = {'Goal': (top.Goal, spec['goal']), 'Result': (top.Result, spec['result']),
                     'Feedback': (top.Feedback, spec['feedback'])}
        for suffix, (cls, part) in parts.items():
            inst = cls()
            out[key + (f'.{suffix}' if suffix else '')] = {
                'class': cls.__name__,
                'module': cls.__module__,
                'fields': cls.get_fields_and_field_types(),
                'consts': {c['n']: _norm(getattr(cls, c['n'])) for c in part['consts']},
                'default': _norm(inst),
                'repr': repr(inst),
            }
    return out


def _real_class(t):
    """The real rclpy class of 'pkg/Name', 'pkg/msg/Name' or 'pkg/srv/Name.Request'."""
    key, _, part = t.partition('.')
    bits = key.split('/')
    kind = bits[1] if len(bits) == 3 else 'msg'
    cls = getattr(importlib.import_module(f'{bits[0]}.{kind}'), bits[-1])
    return getattr(cls, part) if part else cls


# ---------------------------------------------------------------------------------------------
# Entry
# ---------------------------------------------------------------------------------------------

def _write(path, data, indent):
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(data, indent=indent, separators=(',', ':') if indent is None else None)
    path.write_text(text + '\n')
    return path.stat().st_size


def run_inside():
    types = parse_all()
    n = _write(SPEC_OUT, {'distro': 'humble', 'packages': PACKAGES, 'types': types}, None)
    gold = golden(types)
    gold['_probes'] = probes(types, _real_class, _norm)
    g = _write(GOLDEN_OUT, gold, 1)
    print(f'{len(types)} types from {len(PACKAGES)} packages')
    for path, size in ((SPEC_OUT, n), (GOLDEN_OUT, g)):
        print(f'  {path.relative_to(PLAYGROUND)} ({size // 1024} KiB)')


FALLBACK_IMAGE = 'ros:humble'
FALLBACK_APT = ('ros-humble-can-msgs',)       # the only interface package ros:humble lacks


def _have_image(image):
    return subprocess.call(['docker', 'image', 'inspect', image], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL) == 0


def run_docker(image):
    repo = PLAYGROUND.parent
    rel = Path(__file__).resolve().relative_to(repo)
    inner = f'source /opt/ros/humble/setup.bash && python3 /repo/{rel} --inside'
    if image == DEFAULT_IMAGE and not _have_image(image):
        print(f'{image} not found; using {FALLBACK_IMAGE} with {" ".join(FALLBACK_APT)}',
              file=sys.stderr)
        image = FALLBACK_IMAGE
    if image == FALLBACK_IMAGE:
        # root in the container for apt, then hand the two outputs back to the caller's user
        outs = ' '.join(f'/repo/{p.relative_to(repo)}' for p in (SPEC_OUT, GOLDEN_OUT))
        script = (f'apt-get update -qq && apt-get install -y -qq {" ".join(FALLBACK_APT)} > /dev/null'
                  f' && {inner}; rc=$?; chown {os.getuid()}:{os.getgid()} {outs}; exit $rc')
        cmd = ['docker', 'run', '--rm', '-e', 'HOME=/tmp', '-v', f'{repo}:/repo', '-w', '/repo', image,
               'bash', '-c', script]
    else:
        cmd = ['docker', 'run', '--rm', '-u', f'{os.getuid()}:{os.getgid()}', '-e', 'HOME=/tmp',
               '-v', f'{repo}:/repo', '-w', '/repo', image, 'bash', '-c', inner]
    return subprocess.call(cmd)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--inside', action='store_true', help='run here (needs a sourced Humble)')
    ap.add_argument('--image', default=DEFAULT_IMAGE, help='Docker image with Humble and can_msgs')
    args = ap.parse_args()
    if args.inside or SHARE.is_dir():
        run_inside()
        return 0
    return run_docker(args.image)


if __name__ == '__main__':
    sys.exit(main())
