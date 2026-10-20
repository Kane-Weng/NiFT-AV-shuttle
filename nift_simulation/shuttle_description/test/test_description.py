"""
The shuttle model and its views: the TF tree, the RViz frames and robot_view.launch.py.

The URDF is built from this package's sources. nift_common, which holds vehicle.yaml, comes from
the install (it is a dependency, so colcon builds it first).
"""

import importlib.util
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
FRAME_LINE = re.compile(r'^\s*(?:Fixed|Target|Reference) Frame: (\S+)\s*$', re.M)


@pytest.fixture(scope='module')
def urdf():
    import xacro
    from xacro import substitution_args

    find = substitution_args._eval_find

    def eval_find(pkg):
        return str(PKG) if pkg == 'shuttle_description' else find(pkg)

    substitution_args._eval_find = eval_find
    try:
        doc = xacro.process_file(str(PKG / 'description' / 'vehicle.urdf.xacro'))
    finally:
        substitution_args._eval_find = find
    return ET.fromstring(doc.toxml())


def _links(urdf):
    return {link.get('name') for link in urdf.findall('link')}


def _rviz(name):
    return (PKG / 'rviz' / name).read_text()


def test_base_link_is_the_only_root(urdf):
    """
    Keep base_link without a parent: the localization publishes map -> base_link.

    A parent in the URDF as well would give base_link two parents.
    """
    children = {joint.find('child').get('link') for joint in urdf.findall('joint')}
    assert _links(urdf) - children == {'base_link'}
    assert 'map' not in _links(urdf)


def test_robot_view_uses_a_frame_of_the_model(urdf):
    """robot_view.launch.py starts no localization, so nothing publishes map there."""
    assert set(FRAME_LINE.findall(_rviz('robot_view.rviz'))) <= _links(urdf) | {'<Fixed Frame>'}


@pytest.mark.parametrize('name', ['navigation.rviz', 'navigation_topdown.rviz'])
def test_navigation_views_use_frames_the_stack_publishes(urdf, name):
    known = _links(urdf) | {'map', '<Fixed Frame>'}
    text = _rviz(name)
    assert set(FRAME_LINE.findall(text)) <= known

    def tf_frames(node):
        if isinstance(node, dict):
            for key, value in node.items():
                yield key
                yield from tf_frames(value)

    for display in yaml.safe_load(text)['Visualization Manager']['Displays']:
        if display.get('Class') == 'rviz_default_plugins/TF':
            listed = set(display['Frames']) - {'All Enabled'}
            listed |= set(tf_frames(display.get('Tree', {})))
            assert listed <= known, f'{name}: frames nothing publishes: {listed - known}'


def test_robot_view_defaults_to_wall_time():
    """use_sim_time defaults to false, as on the shuttle. A sim passes use_sim_time:=true."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    spec = importlib.util.spec_from_file_location(
        'robot_view_launch', PKG / 'launch' / 'robot_view.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    args = {a.name: a for a in module.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)}
    assert perform_substitutions(LaunchContext(), args['use_sim_time'].default_value) == 'false'
