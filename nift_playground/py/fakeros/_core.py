"""
The fakeros core in one namespace (the spike's module, kept for the code that imports it).

File: fakeros/_core.py

Description:
  `from fakeros._core import world` is how the playground reaches the one world object. The
  code lives in _world (executor and middleware), _node (Node and entities), _params, _qos,
  _time and _logging. This module only re-exports it.
"""
from ._world import START_NS, StaleEntityError, World, world  # noqa: F401
from ._time import ClockType, Clock, Duration, ROSClock, Time  # noqa: F401
from ._logging import LoggingSeverity, RcutilsLogger, get_logger  # noqa: F401
from ._logging import RcutilsLogger as Logger  # noqa: F401
from ._exceptions import (ParameterAlreadyDeclaredException,  # noqa: F401
                          ParameterNotDeclaredException, InvalidParameterTypeException)
from ._params import Parameter, ParamsFile, effective_params, node_scoped  # noqa: F401
from ._qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,  # noqa: F401
                   qos_profile_sensor_data, qos_profile_system_default)
from ._node import Client, Future, Node, Publisher, Service, Subscription, Timer  # noqa: F401
