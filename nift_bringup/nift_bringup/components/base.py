"""
Base class of the system.launch.py components.

File: base.py

Description:
  Abstract base class for the system.launch.py components (one subclass per module).

  Component
  ─────────
    name                 profiles.yaml component name ('longitudinal' for trajectory_controller)
    actions(plan)        abstract: launch actions that start the component (system.launch.py)
    check(plan, graph)   [(level, message)] before anything starts (preflight.py); level from
                         common (OK / WARN / FAIL / SKIP). Default: nothing to check
    outputs(plan)        topics only this component may publish: preflight FAILs if something
                         already publishes one (a second launch, a forgotten `ros2 run`). Default:
                         none

  plan  = the resolved launch configuration (profiles.Plan): plan.mode, plan.on, plan.args
  graph = a snapshot of the live ROS graph (preflight.Graph);
  graph.publishers(topic) lists who publishes a topic right now (preflight.Graph).

Usage:
  from .base import Component
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from ..profiles import Plan

if TYPE_CHECKING:       # preflight imports this package, so a real import would be circular
    from ..preflight import Graph


class Component(ABC):
    name = ''

    @abstractmethod
    def actions(self, plan: Plan) -> list:
        """Launch actions that start this component."""

    def check(self, plan: Plan, graph: 'Graph') -> list:
        """[(level, message)] before anything starts; [] = nothing to check."""
        return []

    def outputs(self, plan: Plan) -> list:
        """Topics nothing else may already publish."""
        return []
