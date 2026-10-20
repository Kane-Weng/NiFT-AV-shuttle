"""
Components of system.launch.py, one module each.

Each module holds what the component starts and how it is
checked before starting, side by side, as a Component subclass (base.py). Used by
system.launch.py and nift_bringup/preflight.py.

REGISTRY maps each profiles.yaml component name to its instance. `lateral` has no entry: it is
trajectory_controller's lateral_enabled (control.py). `localization` and `filter`, the two
providers of map -> base_link, share localization.py. ALWAYS (robot_state_publisher) is not a
component: it always runs.
"""

from .bag import Bag
from .base import Component
from .can import Can
from .control import Control
from .description import Description
from .gnss import Gnss
from .localization import Filter, Localization
from .perception import Perception
from .planning import Planning
from .rviz import Rviz
from .static_path import StaticPath
from .web import Web

REGISTRY = {c.name: c for c in (Can(), Gnss(), Localization(), Filter(), Perception(), Planning(),
                                StaticPath(),
                                Control(), Bag(), Rviz(), Web())}

# `robot_state_publisher` always runs.
ALWAYS = Description()

__all__ = ['ALWAYS', 'REGISTRY', 'Component']
