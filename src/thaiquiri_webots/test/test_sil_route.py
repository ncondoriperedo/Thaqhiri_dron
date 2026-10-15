"""Check route feedback frames, braking, and localization-loss behavior."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from thaiquiri_webots.sil_route import RouteController


def pose(t, xyz=(0, 0, 1), yaw=0):
    return [t, *xyz, *Rotation.from_euler('z', yaw).as_quat()]


def test_forward_acceleration_and_overspeed_braking():
    route = RouteController(3)
    route.observe(pose(15), [0, 0, 0])
    route.observe(pose(20), [0, 0, 0])
    pitch, roll = route.command(20)
    assert pitch < 0
    assert roll == 0
    # Measured speed can exceed the desired limit; the law must brake it.
    route.observe(pose(20.1), [1.1, 0, 0])
    assert route.command(20.1)[0] > 0


@pytest.mark.parametrize('yaw', [0, np.pi/2, -np.pi/2, np.pi])
def test_same_body_command_under_world_heading_changes(yaw):
    route = RouteController(3)
    route.observe(pose(15, yaw=yaw), [0, 0, 0])
    route.observe(pose(20, yaw=yaw), [0, 0, 0])
    assert route.command(20) == pytest.approx((-180*.24/9.81, 0))


def test_stale_and_future_feedback_cancel_commands_and_fresh_data_resumes():
    route = RouteController(3)
    assert route.command(20) == (0, 0)
    assert route.status == 'waiting_for_feedback'
    route.observe(pose(20), [0, 0, 0])
    assert route.command(20.49)[0] < 0
    assert route.command(20.5) == (0, 0)
    assert route.status == 'stale_feedback'
    assert route.command(19.99) == (0, 0)
    route.observe(pose(21), [0, 0, 0])
    assert route.command(20.9) == (0, 0)
    assert route.command(21)[0] < 0
    assert route.status == 'tracking'


def test_return_uses_original_anchor_after_recovery():
    route = RouteController(3)
    route.observe(pose(15, xyz=(1, 2, 1)), [0, 0, 0])
    route.observe(pose(69.9, xyz=(3, 2, 1)), [0, 0, 0])
    assert route.command(69.9)[0] < 0
    assert route.command(70)[0] > 0
    assert route.command(71) == (0, 0)
    route.observe(pose(72, xyz=(3, 2, 1)), [0, 0, 0])
    assert route.command(72)[0] > 0
    assert route.origin == pytest.approx([1, 2, 1])


def test_return_settles_only_when_slow_and_close_and_reengages_on_drift():
    route = RouteController(3)
    route.observe(pose(15), [0, 0, 0])
    route.observe(pose(69, xyz=(0, 0, 1)), [0, 0, 0])
    assert route.command(69)[0] < 0  # Do not settle at the outbound origin.
    route.observe(pose(90, xyz=(.05, 0, 1)), [.16, 0, 0])
    assert route.command(90)[0] != 0  # Arriving too fast must still brake.
    route.observe(pose(91, xyz=(.05, 0, 1)), [.08, 0, 0])
    assert route.command(91) == (0, 0)
    assert route.status == 'settled'
    route.observe(pose(92, xyz=(.14, 0, 1)), [0, 0, 0])
    assert route.command(92) == (0, 0)  # Hysteresis avoids toggling.
    assert route.command(92.5) == (0, 0)
    assert route.status == 'stale_feedback'
    route.observe(pose(93, xyz=(.21, 0, 1)), [0, 0, 0])
    assert route.command(93)[0] > 0
    assert route.status == 'tracking'
    route.observe(pose(94, xyz=(.05, 0, 1)), [0, 0, 0])
    assert route.command(94) == (0, 0)


def test_hover_has_no_horizontal_commands_with_fresh_feedback():
    route = RouteController(3, enabled=False)
    route.observe(pose(20), [0, 0, 0])
    assert route.command(20) == (0, 0)
    assert route.status == 'hover'


def test_invalid_or_out_of_order_feedback_does_not_refresh_estimate():
    route = RouteController(3)
    assert route.observe(pose(20), [0, 0, 0])
    assert not route.observe(pose(19), [0, 0, 0])
    assert not route.observe(pose(21), [np.nan, 0, 0])
    assert not route.observe([21, 0, 0, 1, 0, 0, 0, 0], [0, 0, 0])
    assert route.command(20.5) == (0, 0)


@pytest.mark.parametrize('limit', [0, -1, np.inf, np.nan])
def test_invalid_requested_speed_limit(limit):
    with pytest.raises(ValueError):
        RouteController(3, desired_speed_limit=limit)
