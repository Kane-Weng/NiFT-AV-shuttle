"""
rclpy stand-in: the nodes construct and run on the fakeros world. Time is the runner's job.

File: rclpy/__init__.py

Description:
  init / ok / shutdown / try_shutdown work on the world's running flag. spin() raises, because
  the runner drives the world (ARCH 7). spin_once and spin_until_future_complete drain the queue.
  `import rclpy` also makes rclpy.qos, rclpy.time and the other submodules reachable as
  attributes, as the real package does (shuttle_tf_node uses rclpy.qos.qos_profile_sensor_data).
"""
import fakeros
fakeros.install()
from fakeros._world import world  # noqa: E402
from . import (callback_groups, clock, duration, exceptions, executors, logging, node,  # noqa: E402,F401
               parameter, publisher, qos, service, client, signals, subscription, task, time,
               timer, utilities)
from .utilities import get_default_context  # noqa: E402,F401


def init(*, args=None, context=None, domain_id=None, signal_handler_options=None):
    world.running = True


def ok(*, context=None):
    return world.running


def shutdown(*, context=None, uninstall_handlers=None):
    world.running = False


def try_shutdown(*, context=None, uninstall_handlers=None):
    world.running = False


def create_node(node_name, *, context=None, cli_args=None, namespace=None, use_global_arguments=True,
                enable_rosout=True, start_parameter_services=True, parameter_overrides=None,
                allow_undeclared_parameters=False,
                automatically_declare_parameters_from_overrides=False):
    return node.Node(node_name, context=context, cli_args=cli_args, namespace=namespace,
                     use_global_arguments=use_global_arguments, enable_rosout=enable_rosout,
                     start_parameter_services=start_parameter_services,
                     parameter_overrides=parameter_overrides,
                     allow_undeclared_parameters=allow_undeclared_parameters,
                     automatically_declare_parameters_from_overrides=(
                         automatically_declare_parameters_from_overrides))


def spin(node, executor=None):
    raise RuntimeError('fakeros: the runner drives time (world.step), not rclpy.spin')


def spin_once(node, *, executor=None, timeout_sec=None):
    world.drain()


def spin_until_future_complete(node, future, executor=None, timeout_sec=None):
    world.drain()
