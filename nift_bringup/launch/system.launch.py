"""
Start the NiFT shuttle stack from a profile, on the vehicle or in simulation.

File: system.launch.py
Author: Kane Weng
Date: April 11, 2026

Description:
  The one launch file for the NiFT shuttle, on the vehicle and in CARLA.
    1. A profile (config/profiles.yaml) + per-run switches say which components start;
       nift_bringup/profiles.py checks the rules between them.
    2. The preflight (nift_bringup/preflight.py) checks every enabled component — CAN interface
       up, Octane reachable and both antennas reporting, CARLA answering, map loads, nobody else
       already on /to_can_bus, ... — and stops the launch on any FAIL (checks:=false skips it).
    3. Each component's nodes come from its module in nift_bringup/components/. planning and
       longitudinal/lateral are withheld until beacon_node reports auth_ok.
  The plan and the check results are printed first; dry_run:=true prints them and starts nothing.

Usage:
  ros2 launch nift_bringup system.launch.py                                  # lists the profiles
  ros2 launch nift_bringup system.launch.py profile:=shuttle
  ros2 launch nift_bringup system.launch.py profile:=shuttle perception:=false rviz:=false
  ros2 launch nift_bringup system.launch.py profile:=shuttle filter:=true    # + the filter beside
  it: shadow ros2 launch nift_bringup system.launch.py profile:=none gnss:=true          # one
  component alone ros2 launch nift_bringup system.launch.py profile:=sim [carla_host:=<ip>]
  [scenario_file:=<yaml>] ros2 launch nift_bringup system.launch.py profile:=<name> dry_run:=true
  # plan + checks, start nothing ros2 launch nift_bringup system.launch.py --show-args
  # every argument
"""

import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration
from nift_bringup import preflight
from nift_bringup.components import ALWAYS, REGISTRY
from nift_bringup.components.common import once, share
from nift_bringup.profiles import (
    AFTER_AUTH,
    ARGS,
    COMPONENTS,
    load,
    ProfileError,
    resolve,
    summary,
    SWITCH_HELP,
)

LAUNCH_ARGS = ('profile', 'mode', 'dry_run', 'checks', *COMPONENTS, *ARGS)


def _is_beacon(action):
    return getattr(action, 'node_executable', None) == 'beacon_node'


def _launch(context):
    # —— 1. Reject bad arguments ——————————————————————————————————————
    configs = context.launch_configurations
    if 'sim_mode' in configs:
        raise ProfileError('sim_mode was replaced by profiles: profile:=sim (CARLA) '
                           'or profile:=shuttle '
                           '(real vehicle); run without arguments for the list')
    unknown = sorted(set(configs) - set(LAUNCH_ARGS))
    if unknown:
        raise ProfileError(f'unknown argument(s) {unknown}; see --show-args')

    # —— 2. Build the plan ————————————————————————————————————————————
    cfg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    plan = resolve(load(share('nift_bringup', 'config', 'profiles.yaml')), cfg('profile'),
                   {k: cfg(k) for k in ('mode', *COMPONENTS, *ARGS)})
    text = '\n' + summary(plan)

    # —— 3. Preflight checks ——————————————————————————————————————————
    if cfg('checks').lower() in ('true', '1'):
        t0 = time.monotonic()
        results = preflight.run(plan)
        text += '\n' + preflight.report(results, time.monotonic() - t0, plan)
        if preflight.failed(results) and cfg('dry_run').lower() not in ('true', '1'):
            raise ProfileError(text)
    else:
        text += '\npreflight: skipped (checks:=false)'

    # —— 4. Dry run ————————————————————————————————————————————————————
    if cfg('dry_run').lower() in ('true', '1'):
        return [LogInfo(msg=text + '\n(dry run: nothing started)')]

    # —— 5. Collect actions ————————————————————————————————————————————
    actions, after_auth = list(ALWAYS.actions(plan)), []
    for c in COMPONENTS:
        if c in plan.on and c in REGISTRY:
            (after_auth if c in AFTER_AUTH else actions).extend(REGISTRY[c].actions(plan))
    if after_auth:      # gnss is on whenever these are (profiles.py rules)
        cb = once('auth_ok', [
            LogInfo(msg='[CHECKPOINT CLEARED] Beacon authenticated: starting planning / control'),
            *after_auth])
        actions.append(RegisterEventHandler(OnProcessIO(target_action=_is_beacon, on_stdout=cb,
                                                        on_stderr=cb)))

    # event handlers first: registered before any process they watch can print
    handlers = [a for a in actions if isinstance(a, RegisterEventHandler)]
    return [LogInfo(msg=text), *handlers, *[a for a in actions if a not in handlers]]


def generate_launch_description():
    # Default '' means "use whatever the profile says".
    declarations = [
        DeclareLaunchArgument('profile', default_value='',
                              description='profile from nift_bringup/config/profiles.yaml; '
                                          'none given = list them'),
        DeclareLaunchArgument('mode', default_value='',
                              description="real | sim; '' = the profile's"),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true = print the plan and the check results, '
                                          'then exit'),
        DeclareLaunchArgument('checks', default_value='true',
                              description='false = skip the preflight checks and start anyway'),
    ]

    # one on/off switch per component (localization: also a mode, SWITCH_HELP)
    for name, description in COMPONENTS.items():
        if name in SWITCH_HELP:
            description += f'. {SWITCH_HELP[name]}'
        declarations.append(DeclareLaunchArgument(
            name, default_value='',
            description=f"true | false; '' = the profile's. {description}"))

    # profile-overridable values (ports, paths, ...)
    for name, (default, description) in ARGS.items():
        declarations.append(DeclareLaunchArgument(
            name, default_value='',
            description=f"{description} [default: {default or 'none'}]"))

    return LaunchDescription([*declarations, OpaqueFunction(function=_launch)])
