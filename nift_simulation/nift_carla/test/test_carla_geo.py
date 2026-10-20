import math

from nift_carla.carla_geo import CarlaGeo, MCITY_TMERC, parse_georeference, sanitize_proj4
import pytest

# Lanelet2 map bbox corners in the map frame (from an earlier, unpublished map)
MAP_CORNERS = [(-50.0, -150.0), (169.0, -150.0), (169.0, 239.0), (-50.0, 239.0), (0.0, 0.0)]


@pytest.fixture(scope='module')
def geo():
    return CarlaGeo()


def test_parse_georeference_cdata():
    xodr = ('<OpenDRIVE><header><geoReference><![CDATA[+proj=tmerc +lat_0=1 +lon_0=2 '
            '+datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs ]]>'
            '</geoReference></header></OpenDRIVE>')
    s = parse_georeference(xodr)
    assert s == '+proj=tmerc +lat_0=1 +lon_0=2 +datum=WGS84 +units=m +no_defs'
    assert parse_georeference('<OpenDRIVE/>') is None
    assert 'geoidgrids' not in sanitize_proj4(MCITY_TMERC + ' +geoidgrids=x.gtx')


def test_datum_lands_at_expected_carla_location(geo):
    cx, cy, cz = geo.map_to_carla(0.0, 0.0)
    assert cx == pytest.approx(48.090, abs=1e-3)
    assert cy == pytest.approx(10.376, abs=1e-3)   # y flipped: tmerc -10.376
    assert cz == 0.0


def test_round_trips(geo):
    for x, y in MAP_CORNERS:
        cx, cy, _ = geo.map_to_carla(x, y)
        bx, by = geo.carla_to_map(cx, cy)
        assert bx == pytest.approx(x, abs=1e-6)
        assert by == pytest.approx(y, abs=1e-6)
        lat, lon = geo.map_to_wgs(x, y)
        mx, my = geo.wgs_to_map(lat, lon)
        assert mx == pytest.approx(x, abs=1e-6)
        assert my == pytest.approx(y, abs=1e-6)


def test_grid_convergence_and_yaw(geo):
    conv = geo.grid_convergence_deg()
    assert conv == pytest.approx(1.82, abs=0.05)
    # map north -> CARLA yaw about -90 - convergence (CW-positive UE yaw)
    assert geo.map_yaw_to_carla_yaw_deg(0.0, 0.0, math.pi / 2) == pytest.approx(-91.82, abs=0.05)
    assert geo.map_yaw_to_carla_yaw_deg(0.0, 0.0, 0.0) == pytest.approx(-1.82, abs=0.05)
    # inverse via forward vector
    for yaw in (0.0, 0.7, math.pi / 2, -2.0):
        cyaw = geo.map_yaw_to_carla_yaw_deg(10.0, 20.0, yaw)
        cx, cy, _ = geo.map_to_carla(10.0, 20.0)
        back = geo.carla_yaw_to_map_yaw(cx, cy, cyaw)
        assert math.atan2(math.sin(back - yaw), math.cos(back - yaw)) == pytest.approx(
            0.0, abs=1e-6
        )


def test_metric_scale_preserved(geo):
    # 100 m in the map frame is ~100 m in CARLA (tmerc k=1, small area)
    a = geo.map_to_carla(0.0, 0.0)
    b = geo.map_to_carla(100.0, 0.0)
    assert math.hypot(b[0] - a[0], b[1] - a[1]) == pytest.approx(100.0, abs=0.05)


def test_pyproj_projector_matches_lanelet2(geo):
    # the ROS-free fallback used on the GPU host must agree with lanelet2's UtmProjector
    alt = CarlaGeo(force_pyproj=True)
    assert not alt.using_lanelet2
    for x, y in MAP_CORNERS:
        lat, lon = geo.map_to_wgs(x, y)
        ax, ay = alt.wgs_to_map(lat, lon)
        assert ax == pytest.approx(x, abs=1e-3)
        assert ay == pytest.approx(y, abs=1e-3)
        c1, c2 = geo.map_to_carla(x, y), alt.map_to_carla(x, y)
        assert c1[0] == pytest.approx(c2[0], abs=1e-3)
        assert c1[1] == pytest.approx(c2[1], abs=1e-3)


def test_fallback_flag():
    assert CarlaGeo().georef_is_fallback
    assert not CarlaGeo.from_opendrive(
        '<geoReference><![CDATA[' + MCITY_TMERC + ']]></geoReference>').georef_is_fallback
