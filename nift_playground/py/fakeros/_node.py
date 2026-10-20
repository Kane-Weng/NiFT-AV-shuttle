"""
rclpy's Node and its entities on the fakeros world.

File: fakeros/_node.py

Description:
  Node follows Humble's rclpy for what the five nodes use (ARCH 3.1).
  - The launch name (world.remaps or world.configure_node) replaces the node's own name for
    get_name(), the logger, parameter matching, private names and diagnostics.
  - Parameter overrides, lowest to highest: the node's sources from world.configure_node (rcl's
    merge, ARCH 6.2), world.set_node_params, the node's cli_args (--params-file, -p), the
    parameter_overrides argument, then world.params (the spike's form).
  - declare_parameter(s) types a parameter from its default unless the descriptor has
    dynamic_typing. An override of another type raises InvalidParameterTypeException, as on ROS.
    An override always beats the default. Overrides a node never declares are ignored.
  - Names: relative topic and service names resolve against the namespace, `~/x` against the
    node. Publishing checks the message type (TypeError) and delivers deep copies.
  - Services answer on the callback queue. A client's call_async returns a Future that completes
    when the queue has run the service and the reply. call() drains the world until it has.
"""
import copy
import re

from . import _qos
from ._exceptions import (InvalidHandle, InvalidParameterTypeException,
                          InvalidParameterValueException, ParameterAlreadyDeclaredException,
                          ParameterImmutableException, ParameterNotDeclaredException,
                          ParameterUninitializedException)
from ._logging import RcutilsLogger
from ._params import Parameter, ParamsFile, effective_params, rcl_scalar
from ._time import Duration, ROSClock, S_TO_NS
from ._world import StaleEntityError, world


def _descriptor_cls():
    from rcl_interfaces.msg import ParameterDescriptor
    return ParameterDescriptor


def _set_result(successful=True, reason=''):
    from rcl_interfaces.msg import SetParametersResult
    return SetParametersResult(successful=successful, reason=reason)


_NAME_OK = re.compile(r'[A-Za-z_][A-Za-z0-9_]*\Z')


def resolve_name(name, namespace, node_name):
    """rcl's expansion: absolute stays, `~` is the node, anything else is under the namespace."""
    if name.startswith('/'):
        return name
    ns = namespace.rstrip('/')
    if name == '~':
        return f'{ns}/{node_name}'
    if name.startswith('~/'):
        return f'{ns}/{node_name}/{name[2:]}'
    return f'{ns}/{name}'


class _Entity:
    def _check_alive(self):
        if self._epoch != world.epoch:
            raise StaleEntityError(f'fakeros: {type(self).__name__} from before world.reset()')
        if not self._alive:
            raise RuntimeError(f'fakeros: {type(self).__name__} was destroyed')


# ------------------------------------------------------------------ pub / sub

class Publisher(_Entity):
    def __init__(self, node, msg_type, topic, qos_profile):
        self._node = node
        self.msg_type = msg_type
        self.topic = topic
        self.qos_profile = qos_profile
        self._epoch = world.epoch
        self._alive = True
        if _qos.is_transient_local(qos_profile):
            self._history = []
            self._history_depth = None if qos_profile.history == _qos.HistoryPolicy.KEEP_ALL \
                else max(1, qos_profile.depth)
        else:
            self._history = None
            self._history_depth = None
        world._add_publisher(self)

    @property
    def topic_name(self):
        return self.topic

    def publish(self, msg):
        self._check_alive()
        if not isinstance(msg, self.msg_type):
            raise TypeError(f'Expected {self.msg_type}, got {type(msg)}')
        world._publish(self, msg)

    def get_subscription_count(self):
        return world.count_subscribers(self.topic)

    def assert_liveliness(self):
        pass

    def destroy(self):
        if self._alive and self._epoch == world.epoch:
            world._remove(self, 'pubs')
        self._alive = False


class Subscription(_Entity):
    def __init__(self, node, msg_type, topic, callback, qos_profile, raw=False):
        self._node = node
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos_profile = qos_profile
        self.raw = raw
        self._epoch = world.epoch
        self._alive = True
        world._add_subscription(self)

    @property
    def topic_name(self):
        return self.topic

    def _deliver(self, msg):
        self.callback(msg)

    def destroy(self):
        if self._alive and self._epoch == world.epoch:
            world._remove(self, 'subs')
        self._alive = False


# ------------------------------------------------------------------ timers

class Timer(_Entity):
    def __init__(self, callback, callback_group, timer_period_ns, clock, *, context=None):
        self.callback = callback
        self.callback_group = callback_group
        self._clock = clock
        self._period_ns = int(timer_period_ns)
        self._seq = next(world._timer_seq)
        self._version = 0
        self._epoch = world.epoch
        self._alive = True
        self._canceled = False
        self._last_call_ns = world.now_ns
        self._due = world.now_ns + self._period_ns
        world._schedule(self)

    @property
    def clock(self):
        return self._clock

    @property
    def timer_period_ns(self):
        return self._period_ns

    @timer_period_ns.setter
    def timer_period_ns(self, value):
        self._period_ns = int(value)
        self.reset()

    def is_ready(self):
        return self._alive and not self._canceled and world.now_ns >= self._due

    def is_canceled(self):
        return self._canceled

    def cancel(self):
        self._canceled = True
        self._version += 1

    def reset(self):
        """rcl_timer_reset: next call one period from now, cancelled or not."""
        if self._epoch != world.epoch or not self._alive:
            return
        self._canceled = False
        self._version += 1
        self._due = world.now_ns + self._period_ns
        world._schedule(self)

    def time_since_last_call(self):
        return world.now_ns - self._last_call_ns

    def time_until_next_call(self):
        if self._canceled:
            return None
        return self._due - world.now_ns

    def destroy(self):
        self._alive = False
        self._version += 1


# ------------------------------------------------------------------ services

class Future:
    """rclpy.task.Future, completed on the world's queue."""

    def __init__(self, *, executor=None):
        self._state = 'PENDING'
        self._result = None
        self._exception = None
        self._callbacks = []

    def cancel(self):
        if self._state == 'PENDING':
            self._state = 'CANCELLED'
            self._run_callbacks()

    def cancelled(self):
        return self._state == 'CANCELLED'

    def done(self):
        return self._state in ('FINISHED', 'CANCELLED')

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result

    def exception(self):
        return self._exception

    def set_result(self, result):
        self._result = result
        self._state = 'FINISHED'
        self._run_callbacks()

    def set_exception(self, exception):
        self._exception = exception
        self._state = 'FINISHED'
        self._run_callbacks()

    def _run_callbacks(self):
        cbs, self._callbacks = self._callbacks, []
        for cb in cbs:
            cb(self)

    def add_done_callback(self, callback):
        if self.done():
            callback(self)
        else:
            self._callbacks.append(callback)

    def __await__(self):
        while not self.done():
            yield
        return self.result()


class Service(_Entity):
    def __init__(self, node, srv_type, srv_name, resolved, callback, qos_profile):
        self._node = node
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.resolved_name = resolved
        self.callback = callback
        self.qos_profile = qos_profile
        self._epoch = world.epoch
        self._alive = True
        if resolved in world.services and world.services[resolved]._alive:
            raise RuntimeError(f"fakeros: service '{resolved}' is already offered")
        world.services[resolved] = self

    def _handle(self, request, future):
        response = self.callback(request, self.srv_type.Response())
        if not isinstance(response, self.srv_type.Response):
            raise TypeError(f"fakeros: service '{self.resolved_name}' callback returned "
                            f'{type(response).__name__}, not {self.srv_type.Response.__name__}')
        world.call_soon(future.set_result, copy.deepcopy(response))

    def destroy(self):
        if self._alive and world.services.get(self.resolved_name) is self:
            del world.services[self.resolved_name]
        self._alive = False


class Client(_Entity):
    def __init__(self, node, srv_type, srv_name, resolved, qos_profile):
        self._node = node
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.resolved_name = resolved
        self.qos_profile = qos_profile
        self._epoch = world.epoch
        self._alive = True

    def _service(self):
        svc = world.services.get(self.resolved_name)
        return svc if svc is not None and svc._alive and svc._epoch == world.epoch else None

    def service_is_ready(self):
        return self._service() is not None

    def wait_for_service(self, timeout_sec=None):
        """Nothing can appear while we wait (one thread), so this only says whether it is there."""
        return self.service_is_ready()

    def call_async(self, request):
        self._check_alive()
        if not isinstance(request, self.srv_type.Request):
            raise TypeError(f'Expected {self.srv_type.Request}, got {type(request)}')
        future = Future()
        svc = self._service()
        if svc is not None:                       # else the request is lost, as in DDS
            world._enqueue(svc, svc._handle, (copy.deepcopy(request), future))
        return future

    def call(self, request, timeout_sec=None):
        """LEGACY (spike only): send, then drain the whole world until the reply is in.

        On ROS this blocks and deadlocks inside a callback. New code uses call_async and lets the
        runner drain."""
        future = self.call_async(request)
        world.drain()
        if not future.done():
            raise RuntimeError(f"fakeros: no reply from service '{self.resolved_name}' "
                               '(is it offered?)')
        return future.result()

    def remove_pending_request(self, future):
        future.cancel()

    def destroy(self):
        self._alive = False


# ------------------------------------------------------------------ node

def _parse_cli_args(args):
    """--ros-args -r __node:=x -r __ns:=/y --params-file f -p name:=value -> (name, ns, sources, params)."""
    name = ns = None
    sources, params = [], {}
    if not args:
        return name, ns, sources, params
    in_ros = False
    it = iter(args)
    for a in it:
        if a == '--ros-args':
            in_ros = True
            continue
        if a == '--':
            in_ros = False
            continue
        if not in_ros:
            continue
        if a in ('-r', '--remap'):
            rule = next(it)
            lhs, rhs = rule.split(':=', 1)
            if lhs == '__node' or lhs == '__name':
                name = rhs
            elif lhs == '__ns':
                ns = rhs
        elif a == '--params-file':
            sources.append(next(it))
        elif a in ('-p', '--param'):
            rule = next(it)
            k, v = rule.split(':=', 1)
            params[k] = rcl_scalar(v, False) if not v.startswith('[') else \
                [rcl_scalar(x.strip(), False) for x in v.strip('[]').split(',') if x.strip()]
    return name, ns, sources, params


class Node:
    def __init__(self, node_name, *, context=None, cli_args=None, namespace=None,
                 use_global_arguments=True, enable_rosout=True, start_parameter_services=True,
                 parameter_overrides=None, allow_undeclared_parameters=False,
                 automatically_declare_parameters_from_overrides=False):
        if not _NAME_OK.match(node_name or ''):
            from ._exceptions import InvalidNodeNameException
            raise InvalidNodeNameException(node_name, 'node name must be alphanumeric or _', 0)
        cli_name, cli_ns, cli_sources, cli_params = _parse_cli_args(cli_args)
        self._own_name = node_name
        name = world.remaps.get(node_name, node_name) if use_global_arguments else node_name
        name = cli_name or name
        ns = cli_ns or (world.namespaces.get(name) if use_global_arguments else None) \
            or namespace or '/'
        if not ns.startswith('/'):
            ns = '/' + ns
        self._name = name
        self._ns = ns
        self._epoch = world.epoch
        self._alive = True
        self._allow_undeclared = allow_undeclared_parameters

        overrides = {}
        if use_global_arguments:
            sources = world.node_sources.get(name)
            if sources is None and name != node_name:
                sources = world.node_sources.get(node_name)
            if sources:
                overrides.update(effective_params(sources, self.get_fully_qualified_name()))
            merged = world.node_params.get(name)
            if merged is None and name != node_name:
                merged = world.node_params.get(node_name)
            overrides.update(_python_params(merged or {}))
        if cli_sources:
            overrides.update(effective_params(cli_sources, self.get_fully_qualified_name()))
        overrides.update(cli_params)
        self._parameter_overrides = {k: Parameter(k, value=v) for k, v in overrides.items()}
        for p in parameter_overrides or ():
            self._parameter_overrides[p.name] = p
        legacy = world.params.get(name)
        if legacy is None and name != node_name:
            legacy = world.params.get(node_name)
        for k, v in (legacy or {}).items():
            self._parameter_overrides[k] = Parameter(k, value=_python_value(v))

        self._parameters = {}
        self._descriptors = {}
        self._parameter_callbacks = []
        self._publishers, self._subscriptions, self._timers = [], [], []
        self._services, self._clients = [], []
        logger_name = (ns.strip('/').replace('/', '.') + '.' + name) if ns != '/' else name
        self._logger = RcutilsLogger(logger_name)
        self._clock = ROSClock()
        self.default_callback_group = None
        world.nodes.append(self)

        if automatically_declare_parameters_from_overrides:
            self.declare_parameters('', [
                (p.name, p.value, _descriptor_cls()(dynamic_typing=True))
                for p in self._parameter_overrides.values() if not self.has_parameter(p.name)])
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

    # ------------------------------------------------------------ identity
    def get_name(self):
        return self._name

    def get_namespace(self):
        return self._ns

    def get_fully_qualified_name(self):
        return f'{self._ns.rstrip("/")}/{self._name}'

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    @property
    def context(self):
        return None

    @property
    def executor(self):
        return None

    @property
    def publishers(self):
        yield from self._publishers

    @property
    def subscriptions(self):
        yield from self._subscriptions

    @property
    def timers(self):
        yield from self._timers

    @property
    def services(self):
        yield from self._services

    @property
    def clients(self):
        yield from self._clients

    def _resolve(self, name):
        return resolve_name(name, self._ns, self._name)

    def _check_alive(self):
        if self._epoch != world.epoch:
            raise StaleEntityError('fakeros: node from before world.reset()')
        if not self._alive:
            raise InvalidHandle('Tried to use a handle that has been destroyed.')

    # ------------------------------------------------------------ parameters
    def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
        if value is None and descriptor is None:
            args = (name,)
        else:
            args = (name, value, _descriptor_cls()() if descriptor is None else descriptor)
        return self.declare_parameters('', [args], ignore_override)[0]

    def declare_parameters(self, namespace, parameters, ignore_override=False):
        Descriptor = _descriptor_cls()
        plist, descriptors = [], {}
        for index, tup in enumerate(parameters):
            if len(tup) < 1 or len(tup) > 3:
                raise TypeError(f'Invalid parameter tuple length at index {index} in parameters list: '
                                f'{tup}; expecting length between 1 and 3')
            name = tup[0]
            second = tup[1] if len(tup) > 1 else None
            descriptor = tup[2] if len(tup) > 2 else Descriptor()
            if not isinstance(name, str):
                raise TypeError(f'First element {name} at index {index} in parameters list is not a str.')
            if not isinstance(descriptor, Descriptor):
                raise TypeError(f'Third element {descriptor} at index {index} in parameters list is '
                                'not a ParameterDescriptor.')
            descriptor = copy.copy(descriptor)
            value = None
            if len(tup) == 1:
                descriptor.dynamic_typing = True
            if isinstance(second, Parameter.Type):
                if second == Parameter.Type.NOT_SET:
                    raise ValueError(f'Cannot declare parameter {{{name}}} as statically typed of type '
                                     'NOT_SET')
                if descriptor.dynamic_typing:
                    raise ValueError(f'When declaring parameter {{{name}}} passing a descriptor with'
                                     '`dynamic_typing=True` is not allowed when the parameter type is '
                                     'provided')
                descriptor.type = int(second)
            else:
                value = second
                if not descriptor.dynamic_typing and value is not None:
                    descriptor.type = int(Parameter.Type.from_parameter_value(value))
            # Humble looks the override up before adding the namespace (a quirk kept as is)
            if not ignore_override and name in self._parameter_overrides:
                value = self._parameter_overrides[name].value
            if namespace:
                name = f'{namespace}.{name}'
            plist.append(Parameter(name, value=value))
            descriptors[name] = descriptor
        already = [p.name for p in plist if p.name in self._parameters]
        if already:
            raise ParameterAlreadyDeclaredException(already)
        for p in plist:
            result = self._set_one(p, descriptors[p.name])
            if not result.successful:
                if result.reason.startswith('Wrong parameter type'):
                    raise InvalidParameterTypeException(
                        p, Parameter.Type(descriptors[p.name].type).name)
                raise InvalidParameterValueException(p.name, p.value, result.reason)
        return [self._parameters[p.name] for p in plist]

    def _apply_descriptor(self, parameter, descriptor):
        descriptor.name = parameter.name
        if descriptor.dynamic_typing:
            descriptor.type = int(parameter.type_)
        elif parameter.name in self._parameters and parameter.type_ == Parameter.Type.NOT_SET:
            return _set_result(False, 'Static parameter cannot be undeclared')
        elif parameter.type_ != Parameter.Type.NOT_SET and int(parameter.type_) != descriptor.type:
            return _set_result(False, "Wrong parameter type, expected '{}' got '{}'".format(
                Parameter.Type(descriptor.type), parameter.type_))
        return _set_result(True)

    def _set_one(self, parameter, descriptor):
        result = self._apply_descriptor(parameter, descriptor)
        if not result.successful:
            return result
        for cb in self._parameter_callbacks:
            result = cb([parameter])
            if not result.successful:
                return result
        self._parameters[parameter.name] = parameter
        self._descriptors[parameter.name] = descriptor
        return _set_result(True)

    def undeclare_parameter(self, name):
        if name not in self._parameters:
            raise ParameterNotDeclaredException(name)
        if self._descriptors[name].read_only:
            raise ParameterImmutableException(name)
        del self._parameters[name]
        del self._descriptors[name]

    def has_parameter(self, name):
        return name in self._parameters

    def get_parameter(self, name):
        if name in self._parameters:
            p = self._parameters[name]
            if p.type_ != Parameter.Type.NOT_SET or self._descriptors[name].dynamic_typing:
                return p
            raise ParameterUninitializedException(name)     # statically typed, no value
        if self._allow_undeclared:
            return Parameter(name, Parameter.Type.NOT_SET, None)
        raise ParameterNotDeclaredException(name)

    def get_parameters(self, names):
        return [self.get_parameter(n) for n in names]

    def get_parameter_or(self, name, alternative_value=None):
        if alternative_value is None:
            alternative_value = Parameter(name, Parameter.Type.NOT_SET)
        p = self._parameters.get(name)
        if p is None or p.type_ == Parameter.Type.NOT_SET:
            return alternative_value
        return p

    def get_parameters_by_prefix(self, prefix):
        pre = prefix + '.' if prefix else prefix
        return {k[len(pre):]: v for k, v in self._parameters.items() if k.startswith(pre)}

    def describe_parameter(self, name):
        if name not in self._descriptors:
            raise ParameterNotDeclaredException(name)
        return self._descriptors[name]

    def describe_parameters(self, names):
        return [self.describe_parameter(n) for n in names]

    def set_parameters(self, parameter_list):
        results = []
        for p in parameter_list:
            if p.name not in self._parameters and not self._allow_undeclared:
                raise ParameterNotDeclaredException([p.name])
            if p.name in self._descriptors and self._descriptors[p.name].read_only:
                results.append(_set_result(False, f'Trying to set a read-only parameter: {p.name}.'))
                continue
            descriptor = copy.copy(self._descriptors.get(p.name) or
                                   _descriptor_cls()(dynamic_typing=True))
            results.append(self._set_one(p, descriptor))
        return results

    def set_parameters_atomically(self, parameter_list):
        results = self.set_parameters(parameter_list)
        bad = [r for r in results if not r.successful]
        return bad[0] if bad else _set_result(True)

    def add_on_set_parameters_callback(self, callback):
        self._parameter_callbacks.insert(0, callback)

    def remove_on_set_parameters_callback(self, callback):
        self._parameter_callbacks.remove(callback)

    # ------------------------------------------------------------ entities
    def create_publisher(self, msg_type, topic, qos_profile, *, callback_group=None,
                         event_callbacks=None, qos_overriding_options=None, publisher_class=None):
        self._check_alive()
        pub = Publisher(self, msg_type, self._resolve(topic), _qos.as_profile(qos_profile))
        self._publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos_profile, *, callback_group=None,
                            event_callbacks=None, qos_overriding_options=None, raw=False):
        self._check_alive()
        sub = Subscription(self, msg_type, self._resolve(topic), callback,
                           _qos.as_profile(qos_profile), raw)
        self._subscriptions.append(sub)
        return sub

    def create_timer(self, timer_period_sec, callback, callback_group=None, clock=None):
        self._check_alive()
        period_ns = int(float(timer_period_sec) * S_TO_NS)
        tm = Timer(callback, callback_group, period_ns, clock or self._clock)
        self._timers.append(tm)
        return tm

    def create_rate(self, frequency, clock=None):
        raise RuntimeError('fakeros: Rate sleeps, and nothing may sleep here')

    def create_service(self, srv_type, srv_name, callback, *, qos_profile=None, callback_group=None):
        self._check_alive()
        svc = Service(self, srv_type, srv_name, self._resolve(srv_name), callback,
                      qos_profile or _qos.qos_profile_services_default)
        self._services.append(svc)
        return svc

    def create_client(self, srv_type, srv_name, *, qos_profile=None, callback_group=None):
        self._check_alive()
        cli = Client(self, srv_type, srv_name, self._resolve(srv_name),
                     qos_profile or _qos.qos_profile_services_default)
        self._clients.append(cli)
        return cli

    def destroy_publisher(self, publisher):
        publisher.destroy()
        if publisher in self._publishers:
            self._publishers.remove(publisher)
        return True

    def destroy_subscription(self, subscription):
        subscription.destroy()
        if subscription in self._subscriptions:
            self._subscriptions.remove(subscription)
        return True

    def destroy_timer(self, timer):
        timer.destroy()
        if timer in self._timers:
            self._timers.remove(timer)
        return True

    def destroy_service(self, service):
        service.destroy()
        if service in self._services:
            self._services.remove(service)
        return True

    def destroy_client(self, client):
        client.destroy()
        if client in self._clients:
            self._clients.remove(client)
        return True

    def destroy_node(self):
        for e in self._publishers + self._subscriptions + self._timers + self._services + \
                self._clients:
            e.destroy()
        self._publishers, self._subscriptions, self._timers = [], [], []
        self._services, self._clients = [], []
        self._alive = False
        if self in world.nodes:
            world.nodes.remove(self)

    # ------------------------------------------------------------ graph
    def count_publishers(self, topic_name):
        return world.count_publishers(self._resolve(topic_name))

    def count_subscribers(self, topic_name):
        return world.count_subscribers(self._resolve(topic_name))

    def get_topic_names_and_types(self, no_demangle=False):
        out = []
        for name, tp in sorted(world.topics.items()):
            if tp.msg_type is not None and (tp.pubs or tp.subs):
                mod = tp.msg_type.__module__.split('.')[0]
                out.append((name, [f'{mod}/msg/{tp.msg_type.__name__}']))
        return out

    def get_node_names(self):
        return [n.get_name() for n in world.nodes]

    def get_node_names_and_namespaces(self):
        return [(n.get_name(), n.get_namespace()) for n in world.nodes]

    def get_service_names_and_types(self):
        return sorted((name, [f'{s.srv_type.__module__.split(".")[0]}/srv/{s.srv_type.__name__}'])
                      for name, s in world.services.items() if s._alive)


def _python_value(v):
    """A pre-merged override value: rcl gives a list for a sequence, and no value for []."""
    if isinstance(v, tuple):
        v = list(v)
    if isinstance(v, list) and not v:
        return None
    return v


def _python_params(params):
    return {k: _python_value(v) for k, v in params.items()}


__all__ = ['Node', 'Publisher', 'Subscription', 'Timer', 'Service', 'Client', 'Future',
           'ParamsFile', 'Duration']
