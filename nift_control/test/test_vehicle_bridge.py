"""
Test the vehicle bridge FSM (NiftVehicleBridge) and the speed sign it gives ADRC.

A change of direction passes through a braking state, the braking states and STANDBY hold a
brake floor, and pedal and brake never mix. BRAKING_TO_FORWARD still rolls backwards, so the
speed it reports is negative.
"""

from nift_control.dbw_interface import Gear, NiftVehicleBridge, REVERSE_GEAR_STATES, VehicleMode
import pytest


def drive(bridge, steps):
    """Tick the bridge through (u, target, current) steps and return the states it passed."""
    states = []
    for u, target, current in steps:
        bridge.map_signals(u, target, current)
        states.append(bridge.state)
    return states


@pytest.mark.parametrize('state, sign', [
    (VehicleMode.STANDBY, 1.0),
    (VehicleMode.FORWARD, 1.0),
    (VehicleMode.BRAKING_TO_REVERSE, 1.0),      # reverse asked for, still rolling forward
    (VehicleMode.REVERSE, -1.0),
    (VehicleMode.BRAKING_TO_FORWARD, -1.0),     # forward asked for, still rolling backwards
])
def test_signed_speed_follows_the_way_the_shuttle_rolls(state, sign):
    bridge = NiftVehicleBridge()
    bridge.state = state
    assert bridge.signed_speed(1.3) == sign * 1.3
    assert bridge.signed_speed(-1.3) == sign * 1.3      # the input's own sign is ignored
    assert (state in REVERSE_GEAR_STATES) == (sign < 0)


def test_speed_sign_stays_negative_from_reverse_through_braking_to_forward():
    """
    Reverse at 1 m/s, then flip the target forward.

    The shuttle brakes while it still rolls back, and the speed ADRC sees keeps its sign until
    the stop. It used to jump to +1 m/s here.
    """
    bridge = NiftVehicleBridge()
    states = drive(bridge, [(-20.0, -1.0, 0.0), (-20.0, -1.0, -1.0), (40.0, 1.0, -1.0)])
    assert states == [VehicleMode.REVERSE, VehicleMode.REVERSE, VehicleMode.BRAKING_TO_FORWARD]
    assert [bridge.signed_speed(v) for v in (1.0, 0.6, 0.2)] == [-1.0, -0.6, -0.2]
    gear, _, _ = bridge.map_signals(40.0, 1.0, -0.05)                # stopped: now forward
    assert bridge.state == VehicleMode.FORWARD and gear == Gear.FORWARD
    assert bridge.signed_speed(0.05) == 0.05


def test_no_direct_edge_between_forward_and_reverse():
    bridge = NiftVehicleBridge()
    assert drive(bridge, [(30.0, 1.0, 0.0), (30.0, 1.0, 1.0)]) == [VehicleMode.FORWARD] * 2
    gear, brake, throttle = bridge.map_signals(-80.0, -1.0, 1.0)     # reverse while rolling on
    assert bridge.state == VehicleMode.BRAKING_TO_REVERSE
    assert (gear, brake, throttle) == (Gear.FORWARD, 80, 0)
    gear, brake, throttle = bridge.map_signals(-5.0, -1.0, 0.5)      # small u: the floor of 50
    assert (gear, brake, throttle) == (Gear.FORWARD, 50, 0)
    bridge.map_signals(-5.0, -1.0, 0.05)                             # stopped: now reverse
    assert bridge.state == VehicleMode.REVERSE


def test_braking_to_forward_holds_reverse_gear_and_brakes():
    bridge = NiftVehicleBridge()
    bridge.state = VehicleMode.REVERSE
    gear, brake, throttle = bridge.map_signals(10.0, 1.0, -1.0)
    assert bridge.state == VehicleMode.BRAKING_TO_FORWARD
    assert (gear, brake, throttle) == (Gear.REVERSE, 50, 0)
    gear, brake, throttle = bridge.map_signals(120.0, 1.0, -0.8)
    assert (gear, brake, throttle) == (Gear.REVERSE, 120, 0)
    bridge.map_signals(-20.0, -1.0, -0.8)                            # planner back to reverse
    assert bridge.state == VehicleMode.REVERSE


def test_standby_holds_a_brake_and_pedal_and_brake_never_mix():
    bridge = NiftVehicleBridge()
    assert bridge.map_signals(0.0, 0.0, 0.0) == (Gear.NEUTRAL, 50, 0)
    assert bridge.map_signals(300.0, 0.0, 0.0) == (Gear.NEUTRAL, 255, 0)
    for u in (-300.0, -40.0, 0.0, 40.0, 300.0):
        for target in (-1.0, 0.0, 1.0):
            for current in (-1.0, 0.0, 1.0):
                _, brake, throttle = bridge.map_signals(u, target, current)
                assert brake == 0 or throttle == 0
                assert 0 <= brake <= 255 and 0 <= throttle <= 255
