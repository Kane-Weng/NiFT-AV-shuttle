"""
Launch component for beacon_node, the GNSS input.

File: gnss.py

Description:
  Component `gnss`: beacon_node. Planning and control wait for its auth_ok (system.launch.py).
  Settings: nift_sensor/config/beacon_node.yaml (the feed checks' limits). gnss_checks:=false turns
  the checks off (parameter hygiene.enabled).
    real  Mcity Octane, credentials from nift_sensor/.env
    sim   the sim backend's Octane emulator (CARLA bridge or kinematic_sim_node, same settings from
          nift_carla's carla_params.yaml): all five MCITY_OCTANE_* are set here (load_dotenv does
          not override them, so no sim run can reach the real Octane from .env), and beacon_node is
          started once the emulator logs that it listens (beacon_node connects once, no retry)

  Check
    real  the .env settings exist, then nift_sensor.octane_client.probe(): connect, auth_ok, and
          both antennas deliver a new position within PROBE_S
    sim   SKIP: the emulator only exists once the bridge runs
"""

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch_ros.actions import Node

from .base import Component
from .common import carla_octane, FAIL, OK, once, share, sim_time, SKIP

PROBE_S = 5.0


class Gnss(Component):
    name = 'gnss'

    def outputs(self, plan):
        return ['/gnss/fix']

    def check(self, plan, graph):
        if plan.sim:
            return [(SKIP, "served by the sim backend's Octane emulator once it starts")]
        from nift_sensor.octane_client import octane_config, probe
        cfg = octane_config()
        missing = [k for k in ('MCITY_OCTANE_KEY', 'MCITY_FRONT_BEACON_ID', 'MCITY_REAR_BEACON_ID')
                   if not cfg.get(k)]
        if missing:
            return [(FAIL, f'{", ".join(missing)} not set (nift_sensor/.env, see .env_example; '
                           f'found: {cfg["env_file"] or "no .env"})')]
        server = cfg['MCITY_OCTANE_SERVER']
        r = probe(cfg, PROBE_S)
        if not r['connected']:
            return [(FAIL,
                     f'cannot reach Octane at {server}: {r["error"] or "no connection"} '
                     '(network?)')]
        if not r['auth_ok']:
            return [(FAIL,
                     f'Octane at {server} gave no auth_ok in {PROBE_S:.0f} s '
                     '(wrong MCITY_OCTANE_KEY?)')]
        silent = [f'{side} {cfg[key]}' for side, key in (('front', 'MCITY_FRONT_BEACON_ID'),
                                                         ('rear', 'MCITY_REAR_BEACON_ID'))
                  if not r['antennas'][cfg[key]]]
        if silent:
            return [(FAIL,
                     f'auth_ok, but no new position in {PROBE_S:.0f} s from: {", ".join(silent)} '
                     '(beacon powered? RTK fix?)')]
        return [(OK, f'Octane {server}: auth_ok, front and rear antennas reporting')]

    def actions(self, plan):
        env = None
        if plan.sim:
            octane = carla_octane()
            env = {
                'MCITY_OCTANE_SERVER': f"http://127.0.0.1:{octane['port']}",
                'MCITY_OCTANE_KEY': str(octane['api_key']),
                'MCITY_OCTANE_NAMESPACE': str(octane['namespace']),
                'MCITY_FRONT_BEACON_ID': str(octane['front_id']),
                'MCITY_REAR_BEACON_ID': str(octane['rear_id']),
            }
        beacon = Node(package='nift_sensor', executable='beacon_node', name='beacon_node',
                      output='screen',
                      emulate_tty=True, additional_env=env,
                      parameters=[share('nift_sensor', 'config', 'beacon_node.yaml'),
                                  sim_time(plan),
                                  {'hygiene.enabled': plan.args['gnss_checks'] == 'true'}])
        if not plan.sim:
            return [beacon]
        cb = once('Octane emulator listening', [beacon])
        return [RegisterEventHandler(OnProcessIO(on_stdout=cb, on_stderr=cb))]
