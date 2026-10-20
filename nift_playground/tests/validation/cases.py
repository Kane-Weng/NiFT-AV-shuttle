"""The reference cases the validation runs: labels of tests/reference/runs and how the CLI runs each."""

ROUTES = ('01', '02', '03', '04', '05', '11', '12')
# label of tests/reference/runs -> (route, extra CLI arguments)
CASES = {}
for _r in ROUTES:
    CASES[f'direct_{_r}'] = (_r, [])
    CASES[f'ekf_{_r}'] = (_r, ['--localization', 'ekf'])
CASES['gnss_outage30s_01'] = ('01', ['--gnss-faults', 'outage_30s_straight'])
CASES['gnss_deadzone_04'] = ('04', ['--gnss-dead-zones', 'route04_turn'])

FAST = ('direct_03', 'gnss_outage30s_01')
SLOW = tuple(c for c in CASES if c not in FAST)
# 'shipped': the CLI as it is. 'ros_order': the same CLI with the ROS tick order (ros_order.py)
MODES = ('shipped', 'ros_order')
