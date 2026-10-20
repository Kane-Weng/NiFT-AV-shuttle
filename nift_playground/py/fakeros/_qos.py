"""
rclpy.qos: policies, QoSProfile, the preset profiles and the matching rule.

File: fakeros/_qos.py

Description:
  The policy enums have rclpy's values. QoSProfile needs a history or a depth, as in rclpy
  (QoSProfile() raises InvalidQoSProfileException). A depth alone means KEEP_LAST, and KEEP_LAST
  without a depth raises too.

  What fakeros does with QoS, as DDS does.
  - A subscription gets nothing from a publisher it does not match: a RELIABLE subscription and a
    BEST_EFFORT publisher, or a TRANSIENT_LOCAL subscription and a VOLATILE publisher. Both sides
    log the rclpy warning once.
  - A TRANSIENT_LOCAL publisher keeps its last `depth` messages (all with KEEP_ALL). A matching
    TRANSIENT_LOCAL subscription created later gets them (a latched topic).
  - Every other message reaches every matching subscription. Queue depths never drop anything,
    because the subscriber always keeps up here (ARCH 12.7).
"""
import enum

from ._time import Duration


class InvalidQoSProfileException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, 'Invalid QoSProfile', *args)


class HistoryPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    KEEP_LAST = 1
    KEEP_ALL = 2
    UNKNOWN = 3


class ReliabilityPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2
    UNKNOWN = 3


class DurabilityPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2
    UNKNOWN = 3


class LivelinessPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    AUTOMATIC = 1
    MANUAL_BY_TOPIC = 3
    UNKNOWN = 4


# the deprecated aliases rclpy still exports
QoSHistoryPolicy = HistoryPolicy
QoSReliabilityPolicy = ReliabilityPolicy
QoSDurabilityPolicy = DurabilityPolicy
QoSLivelinessPolicy = LivelinessPolicy


class QoSPolicyKind(enum.IntEnum):
    INVALID = 1 << 0
    DURABILITY = 1 << 1
    DEADLINE = 1 << 2
    LIVELINESS = 1 << 3
    RELIABILITY = 1 << 4
    HISTORY = 1 << 5
    LIFESPAN = 1 << 6
    DEPTH = 1 << 7
    LIVELINESS_LEASE_DURATION = 1 << 8
    AVOID_ROS_NAMESPACE_CONVENTIONS = 1 << 9


def _enum(cls, value):
    if isinstance(value, str):
        return cls[value.upper()]
    return cls(value)


class QoSProfile:
    __slots__ = ('_history', '_depth', '_reliability', '_durability', '_lifespan', '_deadline',
                 '_liveliness', '_liveliness_lease_duration', '_avoid_ros_namespace_conventions')

    def __init__(self, **kwargs):
        if 'history' not in kwargs:
            if 'depth' not in kwargs:
                raise InvalidQoSProfileException('History and/or depth settings are required.')
            kwargs['history'] = HistoryPolicy.KEEP_LAST
        elif _enum(HistoryPolicy, kwargs['history']) == HistoryPolicy.KEEP_LAST and 'depth' not in kwargs:
            raise InvalidQoSProfileException('History set to KEEP_LAST without a depth setting.')
        self._history = _enum(HistoryPolicy, kwargs['history'])
        self._depth = int(kwargs.get('depth', 0))
        self._reliability = _enum(ReliabilityPolicy, kwargs.get('reliability',
                                                                 ReliabilityPolicy.RELIABLE))
        self._durability = _enum(DurabilityPolicy, kwargs.get('durability',
                                                               DurabilityPolicy.VOLATILE))
        self._lifespan = kwargs.get('lifespan', Duration())
        self._deadline = kwargs.get('deadline', Duration())
        self._liveliness = _enum(LivelinessPolicy, kwargs.get('liveliness',
                                                               LivelinessPolicy.SYSTEM_DEFAULT))
        self._liveliness_lease_duration = kwargs.get('liveliness_lease_duration', Duration())
        self._avoid_ros_namespace_conventions = bool(
            kwargs.get('avoid_ros_namespace_conventions', False))

    def _prop(name, cls=None):     # noqa: N805  (property factory)
        def get(self):
            return getattr(self, '_' + name)

        def set_(self, value):
            setattr(self, '_' + name, _enum(cls, value) if cls else value)
        return property(get, set_)

    history = _prop('history', HistoryPolicy)
    depth = _prop('depth')
    reliability = _prop('reliability', ReliabilityPolicy)
    durability = _prop('durability', DurabilityPolicy)
    lifespan = _prop('lifespan')
    deadline = _prop('deadline')
    liveliness = _prop('liveliness', LivelinessPolicy)
    liveliness_lease_duration = _prop('liveliness_lease_duration')
    avoid_ros_namespace_conventions = _prop('avoid_ros_namespace_conventions')
    del _prop

    def __eq__(self, other):
        if not isinstance(other, QoSProfile):
            return False
        return all(getattr(self, s) == getattr(other, s) for s in self.__slots__)

    __hash__ = None

    def __repr__(self):
        return ('QoSProfile(history={0}, depth={1}, reliability={2}, durability={3})'.format(
            self._history, self._depth, self._reliability, self._durability))


qos_profile_sensor_data = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                     reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
qos_profile_system_default = QoSProfile(history=HistoryPolicy.SYSTEM_DEFAULT, depth=0,
                                        reliability=ReliabilityPolicy.SYSTEM_DEFAULT,
                                        durability=DurabilityPolicy.SYSTEM_DEFAULT)
qos_profile_services_default = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                          reliability=ReliabilityPolicy.RELIABLE,
                                          durability=DurabilityPolicy.VOLATILE)
qos_profile_parameters = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000,
                                    reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.VOLATILE)
qos_profile_parameter_events = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000,
                                          reliability=ReliabilityPolicy.RELIABLE,
                                          durability=DurabilityPolicy.VOLATILE)
qos_profile_action_status_default = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                               reliability=ReliabilityPolicy.RELIABLE,
                                               durability=DurabilityPolicy.TRANSIENT_LOCAL)


class QoSPresetProfiles(enum.Enum):
    SYSTEM_DEFAULT = qos_profile_system_default
    SENSOR_DATA = qos_profile_sensor_data
    SERVICES_DEFAULT = qos_profile_services_default
    PARAMETERS = qos_profile_parameters
    PARAMETER_EVENTS = qos_profile_parameter_events
    ACTION_STATUS_DEFAULT = qos_profile_action_status_default

    @classmethod
    def get_from_short_key(cls, name):
        return cls[name.upper()].value


def as_profile(qos):
    """create_publisher / create_subscription take a profile or an int depth (KEEP_LAST)."""
    if isinstance(qos, QoSProfile):
        return qos
    if isinstance(qos, int) and not isinstance(qos, bool):
        if qos < 0:
            raise ValueError('history depth must be greater than or equal to zero')
        return QoSProfile(depth=qos)
    raise TypeError('Expected QoSProfile or int, but received {!r}'.format(type(qos)))


def _rel(q):
    return ReliabilityPolicy.RELIABLE if q.reliability == ReliabilityPolicy.SYSTEM_DEFAULT \
        else q.reliability


def _dur(q):
    return DurabilityPolicy.VOLATILE if q.durability == DurabilityPolicy.SYSTEM_DEFAULT \
        else q.durability


def incompatible_policy(pub_qos, sub_qos):
    """None if a publisher with pub_qos reaches a subscription with sub_qos, else the policy name."""
    if _rel(pub_qos) == ReliabilityPolicy.BEST_EFFORT and _rel(sub_qos) == ReliabilityPolicy.RELIABLE:
        return 'RELIABILITY'
    if _dur(pub_qos) == DurabilityPolicy.VOLATILE and _dur(sub_qos) == DurabilityPolicy.TRANSIENT_LOCAL:
        return 'DURABILITY'
    return None


def is_transient_local(qos):
    return _dur(qos) == DurabilityPolicy.TRANSIENT_LOCAL
