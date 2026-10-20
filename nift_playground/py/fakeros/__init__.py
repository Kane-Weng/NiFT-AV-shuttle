"""
fakeros: the rclpy, tf2_ros and diagnostic_updater our nodes use, on one deterministic sim clock.

File: fakeros/__init__.py

Description:
  Put nift_playground/py and nift_playground/py/fakeros first on sys.path (playground.paths does
  it) and call fakeros.install() before importing a node module. Then `import rclpy`, `tf2_ros`,
  `diagnostic_updater` and the message packages (from _generated/) resolve to the stand-ins.

  The world (fakeros.world, also fakeros._core.world) is the executor and the middleware. The
  runner drives it with world.step(t, pre) or world.drain() and world.fire_timers(t) (ARCH 7).
  The launch set configures nodes with world.configure_node(...) or world.set_node_params(...)
  and world.remaps (ARCH 6).
"""
from . import _generated
from ._world import world  # noqa: F401
from ._params import ParamsFile, effective_params, merge_table, node_key_matches, node_scoped  # noqa: F401


def install():
    """Make the message packages importable. Safe to call more than once."""
    _generated.install()
