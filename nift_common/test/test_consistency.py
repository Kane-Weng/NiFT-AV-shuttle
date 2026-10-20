"""
Copies of the shared facts in files that cannot import nift_common still match it.

Each copy (C++, TypeScript, the Blender / Unreal build scripts) carries an `owner: nift_common/...`
comment on its line. The URDF loads vehicle.yaml itself and must keep no copies. The sources are
read from the repository, so these tests skip where only the install is present.
"""

import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from nift_common import site
from nift_common.vehicle import YAML_PATH
import pytest
import yaml

VEH = yaml.safe_load(YAML_PATH.read_text())
NUM = r'(-?\d+(?:\.\d+)?)'
XACRO_DIR = 'nift_simulation/shuttle_description/description/'
BUILD_SCRIPTS = 'nift_simulation/nift_carla/source_build/scripts/'


def _repo_root():
    for parent in Path(__file__).resolve().parents:
        if all((parent / d).is_dir() for d in ('nift_common', 'nift_simulation',
                                               'nift_web_frontend')):
            return parent
    return None


ROOT = _repo_root()


def _source(rel):
    """Return a repository file's text; skip when the sources are not here."""
    if ROOT is None or not (ROOT / rel).is_file():
        pytest.skip(f'{rel}: sources not found')
    return (ROOT / rel).read_text()


def _owned(rel, pattern):
    """Return the numbers pattern captures on a line marked `owner: nift_common/`."""
    for line in _source(rel).splitlines():
        match = re.search(pattern, line) if 'owner: nift_common/' in line else None
        if match:
            return [float(g) for g in match.groups()]
    pytest.fail(f'{rel}: no line marked "owner: nift_common/" matches {pattern!r}')


@pytest.mark.parametrize('rel, pattern, keys', [
    pytest.param('nift_simulation/nift_rviz_plugins/src/nav_panel.cpp',
                 rf'max_steer_deg\s*=\s*{NUM}', ['max_steering_deg'], id='nav_panel'),
    pytest.param('nift_web_frontend/src/components/ControlPanel.tsx',
                 rf'MAX_STEER_DEG\s*=\s*{NUM}', ['max_steering_deg'], id='ControlPanel'),
    pytest.param('nift_web_frontend/src/components/ObstacleRadar.tsx',
                 rf'V_LEN\s*=\s*{NUM}', ['length'], id='ObstacleRadar_len'),
    pytest.param('nift_web_frontend/src/components/ObstacleRadar.tsx',
                 rf'V_WID\s*=\s*{NUM}', ['width'], id='ObstacleRadar_wid'),
    pytest.param(BUILD_SCRIPTS + '04_build_vehicle.py',
                 rf'STEER_DEG, MASS_KG\s*=\s*{NUM},\s*{NUM}', ['max_steering_deg', 'mass'],
                 id='04_build_vehicle'),
    pytest.param(BUILD_SCRIPTS + '03_rig_shuttle.py',
                 rf'STEER_DEG, MASS_KG, COG_HEIGHT_M\s*=\s*{NUM},\s*{NUM},\s*{NUM}',
                 ['max_steering_deg', 'mass', 'cog_height'], id='03_rig_shuttle'),
])
def test_vehicle_copies(rel, pattern, keys):
    assert _owned(rel, pattern) == [VEH[k] for k in keys], f'{rel}: {keys}'


def test_site_copies():
    rel = 'nift_web_frontend/src/utils/coordinates.ts'
    assert _owned(rel, rf'MAP_ORIGIN_LAT\s*=\s*{NUM}') == [site.MAP_ORIGIN_LAT]
    assert _owned(rel, rf'MAP_ORIGIN_LON\s*=\s*{NUM}') == [site.MAP_ORIGIN_LON]
    # Lanelet2's UtmProjector takes the zone from the origin: the frontend's fixed zone must match
    zone = re.search(r'\+zone=(\d+)', _source(rel))
    assert zone and int(zone.group(1)) == int((site.MAP_ORIGIN_LON + 180) // 6) + 1


def test_urdf_loads_vehicle_yaml():
    text = _source(XACRO_DIR + 'vehicle.xacro')
    assert "xacro.load_yaml('$(find nift_common)/config/vehicle.yaml')" in text


# vehicle.yaml values and the URDF's old derived copies of them
STRAYS = ['2.082', '1.041', '1.04', '0.474165', '0.2618', '0.9525', '0.7245', '0.72', '0.228',
          '0.213', '3.055', '1.588', '1.905', '1.53', '0.96', '0.658', '0.2945', '500']


@pytest.mark.parametrize('name', ['vehicle.xacro', 'lidar.xacro', 'gps.xacro'])
def test_urdf_keeps_no_copies(name):
    text = _source(XACRO_DIR + name)
    for value in STRAYS:
        assert not re.search(rf'(?<![\d.]){re.escape(value)}(?!\d)', text), f'{name}: {value}'


def test_urdf_expands_to_vehicle_yaml():
    xacro = pytest.importorskip('xacro')
    ament = pytest.importorskip('ament_index_python.packages')
    for pkg in ('nift_common', 'shuttle_description'):     # $(find ...) in the xacro files
        try:
            ament.get_package_share_directory(pkg)
        except ament.PackageNotFoundError:
            pytest.skip(f'{pkg} not installed')
    xacro_file = ROOT / XACRO_DIR / 'vehicle.urdf.xacro' if ROOT else None
    if xacro_file is None or not xacro_file.is_file():
        pytest.skip('URDF sources not found')
    urdf = ET.fromstring(xacro.process_file(str(xacro_file)).toxml())

    def joint(name, tag='origin', attr='xyz'):
        elem = urdf.find(f"joint[@name='{name}']/{tag}")
        return [float(v) for v in elem.get(attr).split()]

    assert joint('gps_joint_1') == [VEH['gnss_front_x'], 0.0, VEH['gnss_z']]
    assert joint('gps_joint_2') == [VEH['gnss_rear_x'], 0.0, VEH['gnss_z']]
    assert joint('laser_joint') == [VEH['lidar_x'], VEH['lidar_y'], VEH['lidar_z']]
    assert joint('laser_joint', attr='rpy')[1] == math.radians(VEH['lidar_pitch_deg'])
    assert joint('base_footprint_joint') == [0.0, 0.0, -VEH['base_link_height']]
    assert joint('front_left_steering_joint') == [
        VEH['wheel_base'] / 2, (VEH['wheel_tread'] + VEH['wheel_width']) / 2,
        -(VEH['base_link_height'] - VEH['wheel_radius'])]
    assert joint('front_left_steering_joint', 'limit', 'upper') == [
        math.radians(VEH['max_steering_deg'])]
