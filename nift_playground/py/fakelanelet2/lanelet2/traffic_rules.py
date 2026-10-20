"""Traffic rules of the Lanelet2 stand-in (lanelet2.traffic_rules), ported from GenericTrafficRules.cpp and
GermanTrafficRules.cpp in Lanelet2 1.2.3.

Third-party portions: translated from Lanelet2 1.2.3 (https://github.com/fzi-forschungszentrum-informatik/Lanelet2),
BSD-3-Clause, Copyright 2018 FZI Forschungszentrum Informatik. See ../LICENSES/BSD-3-Clause-Lanelet2.txt and
../NOTICE.

Rules that matter for routing:
- canPass(lanelet): the inverted direction only when one_way is false (or a one_way:<participant> override says so);
  participant:<name> overrides; otherwise the subtype decides (road, highway, play_street, exit, ... for vehicles).
- Areas are never passable for German vehicles.
- canChangeLane: the shared bound's lane_change / lane_change:left / lane_change:right tags win, otherwise
  line_thin or line_thick with dashed (both ways), dashed_solid (to the right) or solid_dashed (to the left).
- speedLimit: speed limit regulatory elements, then speed_limit tags, then the German defaults by location/subtype.
"""
from . import geometry
from .core import Area, SpeedLimit, _as_bool, _as_double

KMH = 1.0 / 3.6  # m/s per km/h


class Locations:
    Germany = 'de'


class Participants:
    Vehicle = 'vehicle'
    VehicleBus = 'vehicle:bus'
    VehicleCar = 'vehicle:car'
    VehicleCarElectric = 'vehicle:car:electric'
    VehicleCarCombustion = 'vehicle:car:combustion'
    VehicleTruck = 'vehicle:truck'
    VehicleMotorcycle = 'vehicle:motorcycle'
    VehicleTaxi = 'vehicle:taxi'
    VehicleEmergency = 'vehicle:emergency'
    Bicycle = 'bicycle'
    Pedestrian = 'pedestrian'
    Train = 'train'

    @staticmethod
    def tag(participant):
        return 'participant:' + participant


class LaneChangeType:
    ToRight = 1
    ToLeft = 2
    Both = 3
    None_ = 0


class SpeedLimitInformation:
    """speedLimit is in km/h, as the Lanelet2 Python bindings return it. mps holds the same in m/s."""

    def __init__(self, mps=0.0, isMandatory=True):
        self.mps = mps
        self.speedLimit = mps * 3.6
        self.isMandatory = isMandatory

    def __repr__(self):
        return 'speedLimit: %gkm/h, mandatory: %s' % (self.speedLimit, 'yes' if self.isMandatory else 'no')


_VEHICLE_CHANGE = {
    ('line_thin', 'dashed'): LaneChangeType.Both,
    ('line_thick', 'dashed'): LaneChangeType.Both,
    ('line_thin', 'dashed_solid'): LaneChangeType.ToRight,
    ('line_thick', 'dashed_solid'): LaneChangeType.ToRight,
    ('line_thin', 'solid_dashed'): LaneChangeType.ToLeft,
    ('line_thick', 'solid_dashed'): LaneChangeType.ToLeft,
}
_PEDESTRIAN_CHANGE = {('curbstone', 'low'): LaneChangeType.Both}

_PARTICIPANT_MAP = {
    '': ['vehicle'],
    'road': ['vehicle', 'bicycle'],
    'highway': ['vehicle'],
    'bicycle_lane': ['bicycle'],
    'play_street': ['pedestrian', 'bicycle', 'vehicle'],
    'emergency_lane': ['vehicle:emergency'],
    'bus_lane': ['vehicle:bus', 'vehicle:emergency', 'vehicle:taxi'],
    'exit': ['pedestrian', 'bicycle', 'vehicle'],
    'walkway': ['pedestrian'],
    'crosswalk': ['pedestrian'],
    'stairs': ['pedestrian'],
    'shared_walkway': ['pedestrian', 'bicycle'],
}

# CountrySpeedLimits for Germany: vehicleUrbanRoad, vehicleNonurbanRoad, vehicleUrbanHighway,
# vehicleNonurbanHighway, playStreet, pedestrian, bicycle
_GERMAN_LIMITS = {
    'vehicleUrbanRoad': SpeedLimitInformation(50 * KMH, True),
    'vehicleNonurbanRoad': SpeedLimitInformation(100 * KMH, True),
    'vehicleUrbanHighway': SpeedLimitInformation(130 * KMH, False),
    'vehicleNonurbanHighway': SpeedLimitInformation(130 * KMH, False),
    'playStreet': SpeedLimitInformation(7 * KMH, True),
    'pedestrian': SpeedLimitInformation(5 * KMH, True),
    'bicycle': SpeedLimitInformation(20 * KMH, True),
}
_SPEED_LOOKUP = {
    ('urban', 'road'): 'vehicleUrbanRoad',
    ('nonurban', 'road'): 'vehicleNonurbanRoad',
    ('urban', 'highway'): 'vehicleUrbanHighway',
    ('nonurban', 'highway'): 'vehicleNonurbanHighway',
    ('urban', 'play_street'): 'playStreet',
    ('nonurban', 'play_street'): 'playStreet',
    ('urban', 'bus_lane'): 'vehicleUrbanRoad',
    ('nonurban', 'bus_lane'): 'vehicleNonurbanRoad',
    ('urban', 'exit'): 'vehicleUrbanRoad',
}
_SIGN_TO_KMH = {
    'de274': 30, 'de274-5': 5, 'de274-10': 10, 'de274-15': 15, 'de274-20': 20, 'de274-30': 30, 'de274-40': 40,
    'de274-50': 50, 'de274-60': 60, 'de274-70': 70, 'de274-80': 80, 'de274-90': 90, 'de274-100': 100,
    'de274-110': 110, 'de274-120': 120, 'de274-130': 130, 'de274_1': 30, 'de274_1-20': 20, 'de310': 50,
}
_UNITS = {'kmh': KMH, 'km/h': KMH, 'mps': 1.0, 'm/s': 1.0, 'mph': 0.44704}


def _as_velocity(value):
    """Attribute::asVelocity: a plain number is km/h; otherwise number plus unit ('2 mps')."""
    d = _as_double(value)
    if d is not None:
        return d * KMH
    s = value.strip()
    i = 0
    while i < len(s) and (s[i].isdigit() or s[i] in '+-.eE'):
        i += 1
    try:
        num = float(s[:i])
    except ValueError:
        return None
    unit = _UNITS.get(s[i:].strip())
    return num * unit if unit is not None else None


def _has_override(attrs, prefix):
    return any(k.startswith(prefix) for k in attrs)


def _get_override(attrs, prefix, override, default, conv):
    # AttributeMap iterates in key order; the first key that `override` starts with wins
    for k in sorted(attrs):
        if len(k) < len(prefix):
            continue
        if override.startswith(k):
            v = conv(attrs[k])
            return default if v is None else v
    return default


def _hardcoded_change_type(boundary):
    a = boundary.attributes
    if 'lane_change' in a:
        return LaneChangeType.Both if _as_bool(a['lane_change']) else LaneChangeType.None_
    if 'lane_change:left' in a:
        if _as_bool(a['lane_change:left']):
            if _as_bool(a.get('lane_change:right', 'no')):
                return LaneChangeType.Both
            return LaneChangeType.ToLeft
    if 'lane_change:right' in a:
        if _as_bool(a['lane_change:right']):
            return LaneChangeType.ToRight
        return LaneChangeType.None_
    return None


def _can_change_to_left(t):
    return t in (LaneChangeType.Both, LaneChangeType.ToLeft)


def _can_change_to_right(t):
    return t in (LaneChangeType.Both, LaneChangeType.ToRight)


class TrafficRules:
    """GenericTrafficRules with the German participant specifics."""

    def __init__(self, location, participant):
        self._location = location
        self._participant = participant

    def location(self):
        return self._location

    def participant(self):
        return self._participant

    def __repr__(self):
        return 'location: %s, participant: %s' % (self._location, self._participant)

    # -- passability
    def _is_driving_dir(self, ll):
        if not ll.inverted():
            return True
        a = ll.attributes
        if 'one_way' in a:
            one_way = _as_bool(a['one_way'])
            if one_way is not None:
                return not one_way
        if _has_override(a, 'one_way'):
            return not _get_override(a, 'one_way', 'one_way:' + self._participant, True, _as_bool)
        return self._participant == Participants.Pedestrian

    def _can_pass_regelems(self, regelems):
        return None  # German vehicle, pedestrian and bicycle rules return "no rule" here

    def _can_pass_type(self, subtype):
        parts = _PARTICIPANT_MAP.get(subtype)
        if parts is None:
            return None
        return any(self._participant.startswith(p) for p in parts)

    def _can_pass_one(self, prim):
        if isinstance(prim, Area):
            if self._participant.startswith(Participants.Vehicle):
                return False  # GermanVehicle: areas are not for normal driving
            by_rule = self._can_pass_regelems(prim.regulatoryElements)
            if by_rule is not None:
                return by_rule
            a = prim.attributes
            if _has_override(a, 'participant'):
                return _get_override(a, 'participant', Participants.tag(self._participant), False, _as_bool)
            r = self._can_pass_type(a.get('subtype', ''))
            return bool(r)
        if not self._is_driving_dir(prim):
            return False
        by_rule = self._can_pass_regelems(prim.regulatoryElements)
        if by_rule is not None:
            return by_rule
        a = prim.attributes
        if _has_override(a, 'participant'):
            return _get_override(a, 'participant', Participants.tag(self._participant), False, _as_bool)
        r = self._can_pass_type(a.get('subtype', ''))
        return bool(r)

    def canPass(self, a, b=None):
        if b is None:
            return self._can_pass_one(a)
        if isinstance(a, Area) or isinstance(b, Area):
            return False  # only reachable for areas, which German vehicles cannot use; others not needed
        return geometry.follows(a, b) and self._can_pass_one(a) and self._can_pass_one(b)

    # -- lane changes
    def laneChangeType(self, boundary, virtualIsPassable=False):
        t = _hardcoded_change_type(boundary)
        if t is None:
            typ = boundary.attributes.get('type', '')
            if virtualIsPassable and typ == 'virtual':
                return LaneChangeType.Both
            key = (typ, boundary.attributes.get('subtype', ''))
            p = self._participant
            if p.startswith(Participants.Vehicle):
                t = _VEHICLE_CHANGE.get(key, LaneChangeType.None_)
            elif p == Participants.Pedestrian:
                t = _PEDESTRIAN_CHANGE.get(key, LaneChangeType.None_)
            elif p == Participants.Bicycle:
                t = _VEHICLE_CHANGE.get(key, LaneChangeType.None_)
                if t == LaneChangeType.None_:
                    t = _PEDESTRIAN_CHANGE.get(key, LaneChangeType.None_)
            else:
                t = LaneChangeType.None_
        if boundary.inverted():
            if t == LaneChangeType.ToLeft:
                return LaneChangeType.ToRight
            if t == LaneChangeType.ToRight:
                return LaneChangeType.ToLeft
        return t

    def canChangeLane(self, frm, to):
        if not self.canPass(frm) or not self.canPass(to):
            return False
        if geometry.leftOf(frm, to):
            is_left = True
        elif geometry.rightOf(frm, to):
            is_left = False
        else:
            return False
        t = self.laneChangeType(frm.rightBound if is_left else frm.leftBound)
        return _can_change_to_right(t) if is_left else _can_change_to_left(t)

    def isOneWay(self, ll):
        return self._is_driving_dir(ll) != self._is_driving_dir(ll.invert())

    def hasDynamicRules(self, ll):
        return any(_as_bool(r.attributes.get('dynamic', 'no')) for r in ll.regulatoryElements)

    # -- speed limits
    def _regelem_speed_limit(self, regelems):
        if not self._participant.startswith(Participants.Vehicle):
            return None
        for r in regelems:
            if isinstance(r, SpeedLimit):
                typ = r.type()
                kmh = _SIGN_TO_KMH.get(typ)
                if kmh is not None:
                    return SpeedLimitInformation(kmh * KMH, True)
                v = _as_velocity(typ)
                if v is None:
                    raise ValueError('Unable to interpret the velocity information from ' + typ)
                return SpeedLimitInformation(v, True)
        return None

    def speedLimit(self, prim):
        regelems = prim.regulatoryElements
        info = self._regelem_speed_limit(regelems)
        if info is not None:
            return info
        a = prim.attributes
        if _has_override(a, 'speed_limit') or _has_override(a, 'speed_limit_mandatory'):
            default = 0.0
            if 'speed_limit' in a:
                v = _as_velocity(a['speed_limit'])
                default = v if v is not None else 0.0
            limit = _get_override(a, 'speed_limit:', 'speed_limit:' + self._participant, default, _as_velocity)
            mandatory = _get_override(a, 'speed_limit_mandatory', 'speed_limit_mandatory:' + self._participant,
                                      True, _as_bool)
            return SpeedLimitInformation(limit, mandatory)
        p = self._participant
        if p == Participants.Pedestrian:
            return _GERMAN_LIMITS['pedestrian']
        if p == Participants.Bicycle:
            return _GERMAN_LIMITS['bicycle']
        if p.startswith(Participants.Vehicle):
            key = _SPEED_LOOKUP.get((a.get('location', 'urban'), a.get('subtype', 'road')))
            if key is not None:
                return _GERMAN_LIMITS[key]
        return SpeedLimitInformation(0.0, True)


def create(location, participant):
    if location != Locations.Germany or participant not in (Participants.Vehicle, Participants.Pedestrian,
                                                            Participants.Bicycle):
        raise ValueError('No traffic rules registered for location %r and participant %r' % (location, participant))
    return TrafficRules(location, participant)
