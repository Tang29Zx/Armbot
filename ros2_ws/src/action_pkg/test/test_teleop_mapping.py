"""Unit tests for the ROS-independent Xbox mapping layer."""

import math

from action_pkg.teleop_mapping import (
    apply_deadzone,
    controls_neutral,
    integrate_target,
    joints_near_home,
    MODE_ARM,
    MODE_GRIPPER,
    MODE_WRIST_ROLL,
    rising_edge,
    Target,
    trigger_pressed,
    valid_joy,
)
import pytest


BOUNDS = {
    'pitch': (-90.0, 90.0),
    'wrist_roll': (-math.pi / 2.0, math.pi / 2.0),
}


def _integrate(target, axes, dt=1.0, pitch_modifier=False):
    return integrate_target(
        target,
        axes,
        dt,
        deadzone=0.12,
        translation_speed=1.0,
        pitch_speed=10.0,
        wrist_roll_speed=math.radians(20.0),
        gripper_speed=0.5,
        bounds=BOUNDS,
        pitch_modifier=pitch_modifier,
    )


def test_deadzone_removes_drift_and_rescales():
    assert apply_deadzone(0.1, 0.12) == 0.0
    assert apply_deadzone(-0.12, 0.12) == 0.0
    assert apply_deadzone(1.0, 0.12) == 1.0
    assert apply_deadzone(-1.0, 0.12) == -1.0


def test_trigger_normalization():
    assert trigger_pressed(1.0) == 0.0
    assert trigger_pressed(0.0) == 0.5
    assert trigger_pressed(-1.0) == 1.0


def test_cartesian_axis_directions_and_modified_pitch():
    home = Target()
    x_target, mode = _integrate(home, [0, 1, 0, 0, 1, 1])
    assert mode == MODE_ARM and x_target.x == 16.0
    y_target, _ = _integrate(home, [1, 0, 0, 0, 1, 1])
    assert y_target.y == 1.0
    z_target, _ = _integrate(home, [0, 0, 0, 1, 1, 1])
    assert z_target.z == 3.0
    pitch_target, mode = _integrate(
        home, [0, 0, 1, 0, 1, 1], pitch_modifier=True)
    assert mode == MODE_ARM
    assert pitch_target.pitch == -44.48


def test_right_stick_horizontal_controls_wrist_roll_by_default():
    home = Target()
    left, mode = _integrate(home, [0, 0, 1, 0, 1, 1])
    assert mode == MODE_WRIST_ROLL
    assert left.wrist_roll == pytest.approx(math.radians(20.0))

    right, mode = _integrate(home, [0, 0, -1, 0, 1, 1])
    assert mode == MODE_WRIST_ROLL
    assert right.wrist_roll == pytest.approx(math.radians(-20.0))


def test_gripper_has_priority_over_cartesian():
    target, mode = _integrate(Target(), [0, 1, 0, 0, -1, 1])
    assert mode == MODE_GRIPPER
    assert target.gripper == 0.5


def test_rt_closes_and_lt_opens_gripper():
    half = Target(gripper=0.5)
    closed, mode = _integrate(half, [0, 0, 0, 0, -1, 1])
    assert mode == MODE_GRIPPER and closed.gripper == 1.0
    opened, mode = _integrate(half, [0, 0, 0, 0, 1, -1])
    assert mode == MODE_GRIPPER and opened.gripper == 0.0


def test_xyz_is_unbounded_but_pitch_is_clamped():
    target = Target(x=20.0, y=10.0, z=25.0, pitch=90.0)
    updated, mode = _integrate(target, [1, 1, 1, 1, 1, 1])
    assert mode == MODE_ARM
    assert updated == Target(x=21.0, y=11.0, z=26.0, pitch=90.0)

    target = Target(x=10.0, y=-10.0, z=0.0, pitch=-90.0)
    updated, mode = _integrate(target, [-1, -1, -1, -1, 1, 1])
    assert mode == MODE_ARM
    assert updated == Target(x=9.0, y=-11.0, z=-1.0, pitch=-90.0)


def test_wrist_roll_is_clamped_to_ninety_degrees():
    target = Target(wrist_roll=math.pi / 2.0)
    updated, mode = _integrate(target, [0, 0, 1, 0, 1, 1])
    assert mode == MODE_WRIST_ROLL
    assert updated.wrist_roll == math.pi / 2.0


def test_joy_validation_neutral_and_button_edge():
    neutral = [0, 0, 0, 0, 1, 1]
    assert valid_joy(neutral, [0] * 8)
    assert not valid_joy(neutral[:5], [0] * 8)
    assert not valid_joy([0, 0, math.nan, 0, 1, 1], [0] * 8)
    assert controls_neutral(neutral, 0.12)
    assert not controls_neutral([0, 0.2, 0, 0, 1, 1], 0.12)
    assert rising_edge([1] + [0] * 7, [0] * 8, 0)
    assert not rising_edge([1] + [0] * 7, [1] + [0] * 7, 0)


def test_home_joint_check_rejects_invalid_or_distant_feedback():
    expected = [0.0, 1.0, -1.0, -0.5, 0.0]
    assert joints_near_home(expected, expected, 0.01)
    assert not joints_near_home([0.0] * 5, expected, 0.01)
    assert not joints_near_home([math.nan] * 5, expected, 0.01)
