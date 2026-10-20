"""Parameters: rclpy typing, rcl params-file rules, launch_ros dicts, precedence and remaps (ARCH 6.2)."""
import math

import pytest

from fakeros import ParamsFile, effective_params, node_scoped


def make(world, own='tc', name=None, sources=(), **kw):
    from rclpy.node import Node
    world.configure_node(own, name=name, parameters=list(sources))
    return Node(own, **kw)


def pf(text):
    return ParamsFile(text)


# ------------------------------------------------------------------ typing

def test_default_types_the_parameter(node):
    assert node.declare_parameter('f', 1.5).value == 1.5
    assert node.declare_parameter('i', 3).value == 3
    assert node.declare_parameter('b', True).value is True
    assert node.declare_parameter('s', 'x').value == 'x'
    assert node.declare_parameter('fl', [1.0, 2.0]).value == [1.0, 2.0]
    assert node.declare_parameter('sl', ['a']).value == ['a']
    assert node.declare_parameter('tup', (1.0, 2.0)).value == (1.0, 2.0)
    from rclpy.parameter import Parameter
    assert node.declare_parameter('e', []).type_ == Parameter.Type.BYTE_ARRAY
    with pytest.raises(TypeError):
        node.declare_parameter('mixed', [1, 2.0])
    with pytest.raises(TypeError):
        node.declare_parameter('d', {'a': 1})


def test_override_of_wrong_type_raises_like_rclpy(world):
    from rclpy.exceptions import InvalidParameterTypeException
    n = make(world, sources=[pf('/**:\n  ros__parameters:\n    wrong: 3\n')])
    with pytest.raises(InvalidParameterTypeException) as e:
        n.declare_parameter('wrong', 1.0)
    assert str(e.value) == ("Trying to set parameter 'wrong' to '3' of type 'INTEGER', "
                            "expecting type 'DOUBLE'")


def test_dynamic_typing_takes_the_override_type(world):
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.parameter import Parameter
    dyn = ParameterDescriptor(dynamic_typing=True)
    n = make(world, sources=[pf('/**:\n  ros__parameters:\n    dyn: 3\n    empty: []\n')])
    assert n.declare_parameter('dyn', 1.0, dyn).value == 3
    p = n.declare_parameter('empty', [], dyn)          # rcl gives [] no value
    assert p.value is None and p.type_ == Parameter.Type.NOT_SET
    p = n.declare_parameter('none_given', [], dyn)
    assert p.value == [] and p.type_ == Parameter.Type.BYTE_ARRAY


def test_static_parameter_with_empty_list_override_is_none(world):
    n = make(world, sources=[pf('tc:\n  ros__parameters:\n    sz: []\n')])
    assert n.declare_parameter('sz', [0.5]).value is None


def test_sequences_are_lists(world):
    n = make(world, sources=[pf('tc:\n  ros__parameters:\n    darr: [1.0, 2.0]\n    iarr: [1, 2]\n'
                                '    sarr: [p, q]\n    barr: [true, false]\n')])
    for name, default, want in (('darr', [0.5], [1.0, 2.0]), ('iarr', [0], [1, 2]),
                                ('sarr', [''], ['p', 'q']), ('barr', [False], [True, False])):
        v = n.declare_parameter(name, default).value
        assert v == want and type(v) is list


def test_declare_twice_and_undeclared(node):
    from rclpy.exceptions import ParameterAlreadyDeclaredException, ParameterNotDeclaredException
    node.declare_parameter('x', 0)
    with pytest.raises(ParameterAlreadyDeclaredException):
        node.declare_parameter('x', 0)
    with pytest.raises(ParameterNotDeclaredException):
        node.get_parameter('nope')


def test_declare_parameters_with_namespace_and_tuples(world):
    n = make(world, sources=[pf('tc:\n  ros__parameters:\n    a: 5\n')])
    ps = n.declare_parameters('', [('a', 1), ('b', 2.0)])
    assert [p.value for p in ps] == [5, 2.0]
    ps = n.declare_parameters('adrc', [('b0', 0.5)])
    assert ps[0].name == 'adrc.b0'
    assert n.get_parameters_by_prefix('adrc') == {'b0': ps[0]}


def test_use_sim_time_declared_false_unless_overridden(world):
    n = make(world)
    assert n.get_parameter('use_sim_time').value is False
    world.reset()
    n = make(world, sources=[{'use_sim_time': True}])
    assert n.get_parameter('use_sim_time').value is True


def test_undeclared_overrides_are_ignored(world):
    n = make(world, sources=[pf('tc:\n  ros__parameters:\n    only_a: a\n')])
    assert not n.has_parameter('only_a')


def test_set_parameters_checks_type(node):
    from rclpy.parameter import Parameter
    node.declare_parameter('x', 1.0)
    r = node.set_parameters([Parameter('x', value=2)])
    assert not r[0].successful and r[0].reason.startswith('Wrong parameter type')
    assert node.set_parameters([Parameter('x', value=2.0)])[0].successful
    assert node.get_parameter('x').value == 2.0


# ------------------------------------------------------------------ rcl scalars

RCL_SCALARS = {
    'y': True, 'Y': True, 'n': False, 'yes': True, 'On': True, '010': 8, '0x1A': 26, '1e3': 1000.0,
    '1_000': '1_000', '+5': 5, '.5': 0.5, '5.': 5.0, '.inf': math.inf, 'inf': math.inf,
    '~': '~', 'null': 'null', "'5'": '5', '!!str 5': 5, '99999999999999999999': 1e20,
    '1e400': '1e400', '1e-400': '1e-400', '0x1p3': 8.0, '-3': -3, '-.5': -0.5, 'Infinity': math.inf,
    "''": '',
}


@pytest.mark.parametrize('text,want', list(RCL_SCALARS.items()))
def test_rcl_scalar_rules(text, want):
    got = effective_params([pf(f'/**:\n  ros__parameters:\n    v: {text}\n')], '/t')['v']
    assert got == want and type(got) is type(want)


def test_rcl_nan_and_literal_block():
    got = effective_params([pf('/**:\n  ros__parameters:\n    a: .nan\n    b: |\n      42\n')], '/t')
    assert math.isnan(got['a']) and got['b'] == '42\n'


def test_rcl_sequence_rules():
    from fakeros._params import ParamsFileError
    with pytest.raises(ParamsFileError, match='Sequence should be of same type'):
        effective_params([pf('/**:\n  ros__parameters:\n    m: [0, 1.5]\n')], '/t')
    got = effective_params([pf('/**:\n  ros__parameters:\n    y: [y, n]\n')], '/t')
    assert got['y'] == [True, False]


def test_rcl_empty_value_is_an_error():
    from fakeros._params import ParamsFileError
    with pytest.raises(ParamsFileError, match='No value'):
        effective_params([pf('/**:\n  ros__parameters:\n    e:\n')], '/t')


def test_nested_maps_flatten():
    got = effective_params([pf('tc:\n  ros__parameters:\n    nested:\n      k: 5\n      d:\n        x: 1.0\n')],
                           '/tc')
    assert got == {'nested.k': 5, 'nested.d.x': 1.0}


# ------------------------------------------------------------------ precedence (table order)

A = pf('tc:\n  ros__parameters:\n    x: 1\n')
B = pf('/**:\n  ros__parameters:\n    x: 2\n    y: 20\n')
C = pf('/tc:\n  ros__parameters:\n    y: 30\n')
D = pf('/**:\n  ros__parameters:\n    y: 40\n')
E3 = pf('/tc:\n  ros__parameters:\n    x: 3\n')
E4 = pf('/*:\n  ros__parameters:\n    x: 4\n')


@pytest.mark.parametrize('sources,name,want', [
    ((A, B, C), 'x', 2),        # tc, then /** (later key wins)
    ((A, B, C), 'y', 30),
    ((B, A), 'x', 1),
    ((C, D), 'y', 40),
    ((A, B, E3), 'x', 3),       # 'tc' and '/tc' are two keys
    ((E3, B, A), 'x', 1),
    ((A, E4), 'x', 4),
    ((E4, A), 'x', 1),
])
def test_precedence_matches_humble(sources, name, want):
    assert effective_params(list(sources), '/tc')[name] == want


def test_arch_example_dict_after_node_scoped_file_loses(world):
    """{use_sim_time} makes the /** key first, so a later dict's filter_type loses to planning.yaml."""
    planning = pf('trajectory_controller:\n  ros__parameters:\n    filter_type: cbf\n')
    srcs = [{'use_sim_time': True}, planning, {'filter_type': 'none'}]
    assert effective_params(srcs, '/trajectory_controller')['filter_type'] == 'cbf'
    srcs.append(node_scoped('trajectory_controller', {'filter_type': 'none'}))
    assert effective_params(srcs, '/trajectory_controller')['filter_type'] == 'none'


def test_other_nodes_keys_do_not_apply():
    assert effective_params([pf('other:\n  ros__parameters:\n    x: 1\n')], '/tc') == {}


# ------------------------------------------------------------------ remaps and the other channels

def test_launch_name_replaces_own_name(world):
    own_yaml = pf('own_name:\n  ros__parameters:\n    z: 7\nrenamed:\n  ros__parameters:\n    w: 8\n')
    n = make(world, own='own_name', name='renamed', sources=[own_yaml])
    assert n.get_name() == 'renamed' and n.get_logger().name == 'renamed'
    assert n.get_fully_qualified_name() == '/renamed'
    assert n.declare_parameter('z', 0).value == 0
    assert n.declare_parameter('w', 0).value == 8


def test_remaps_table_alone(world):
    from rclpy.node import Node
    world.remaps['nift_beacon_node'] = 'beacon_node'
    world.set_node_params('beacon_node', {'gnss_timeout_s': 0.7})
    n = Node('nift_beacon_node')
    assert n.get_name() == 'beacon_node'
    assert n.declare_parameter('gnss_timeout_s', 0.5).value == 0.7


def test_parameter_overrides_win_over_files(world):
    from rclpy.parameter import Parameter
    n = make(world, sources=[A], parameter_overrides=[Parameter('x', value=99)])
    assert n.declare_parameter('x', 0).value == 99


def test_spike_params_still_apply(world):
    from rclpy.node import Node
    world.params['tc'] = {'x': 5, 'empty': []}
    n = Node('tc')
    assert n.declare_parameter('x', 0).value == 5
    assert n.declare_parameter('empty', [1.0]).value is None


def test_cli_args(world):
    from rclpy.node import Node
    n = Node('tc', cli_args=['--ros-args', '-r', '__node:=other', '-p', 'x:=3'])
    assert n.get_name() == 'other' and n.declare_parameter('x', 0).value == 3


def test_yaml_file_on_disk(world, tmp_path):
    p = tmp_path / 'p.yaml'
    p.write_text('tc:\n  ros__parameters:\n    x: 11\n')
    n = make(world, sources=[str(p)])
    assert n.declare_parameter('x', 0).value == 11


# ------------------------------------------------------------------ launch_ros dicts

def test_launch_dict_rules():
    got = effective_params([{'s': 'abc', 'num_text': '1.0', 'f': 0.061, 'i': 3, 'b': True,
                             'ints': [1, 2], 'mix': [1, 2.5], 'strs': ['NAVIGATING', 'MANUAL'],
                             'nested': {'a': 1}, 'empty_text': ''}], '/any')
    assert got == {'s': 'abc', 'num_text': '1.0', 'f': 0.061, 'i': 3, 'b': True, 'ints': [1, 2],
                   'mix': [1.0, 2.5], 'strs': ['NAVIGATING', 'MANUAL'], 'nested.a': 1, 'empty_text': ''}


def test_launch_dict_y_becomes_bool_through_rcl():
    """launch_ros dumps 'y' unquoted (YAML 1.1 has no bool y) and rcl reads it as true."""
    assert effective_params([{'v': 'y'}], '/any')['v'] is True


@pytest.mark.parametrize('bad', [[], [True, 1], ['a', 1], None])
def test_launch_dict_refuses_what_launch_ros_refuses(bad):
    with pytest.raises(TypeError):
        effective_params([{'v': bad}], '/any')


def test_launch_dict_is_wildcard_scoped():
    from fakeros import merge_table
    assert list(merge_table([{'a': 1}])) == ['/**']
    assert list(merge_table([node_scoped('n', {'a': 1})])) == ['n']
