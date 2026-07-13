"""State-machine tests for Xbox teleoperation without hardware."""

import math
import time
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

from action_interfaces.msg import ArmState
from action_pkg.arm_teleop_node import ArmTeleopNode


def _joy(*, axes=None, buttons=None):
    msg = Joy()
    msg.axes = axes or [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]
    msg.buttons = buttons or [0] * 16
    return msg


@pytest.fixture
def shadow_node(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmTeleopNode(parameter_overrides=[
        Parameter('shadow_mode', value=True),
    ])
    node._command_pub = MagicMock()
    node._estop_pub = MagicMock()
    node._enabled_pub = MagicMock()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_a_is_edge_triggered_and_requires_neutral(shadow_node):
    pressed = [0] * 16
    pressed[0] = 1
    shadow_node._joy_callback(_joy(buttons=pressed))
    assert shadow_node._enabled

    shadow_node._joy_callback(_joy(buttons=pressed))
    assert shadow_node._enabled

    shadow_node._joy_callback(_joy())
    deflected = [0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]
    shadow_node._joy_callback(_joy(axes=deflected, buttons=pressed))
    assert shadow_node._enabled is False
    assert shadow_node._synced
    shadow_node._joy_callback(_joy(axes=deflected))
    shadow_node._joy_callback(_joy(axes=deflected, buttons=pressed))
    assert shadow_node._enabled is False


def test_reset_chord_requires_full_hold(shadow_node):
    buttons = [0] * 16
    buttons[3] = 1
    buttons[6] = 1
    buttons[7] = 1
    shadow_node._joy_callback(_joy(buttons=buttons))
    shadow_node._request_reset = MagicMock()
    started = time.monotonic()
    shadow_node._update_chords(started)
    shadow_node._update_chords(started + 0.9)
    shadow_node._request_reset.assert_not_called()
    shadow_node._update_chords(started + 1.1)
    shadow_node._request_reset.assert_called_once_with()


def test_invalid_joy_disables_and_loses_sync(shadow_node):
    pressed = [0] * 16
    pressed[0] = 1
    shadow_node._joy_callback(_joy(buttons=pressed))
    assert shadow_node._enabled
    shadow_node._joy_callback(_joy(axes=[0.0] * 5))
    assert shadow_node._enabled is False
    assert shadow_node._synced is False


def test_b_latches_shadow_estop_until_reset_then_home(shadow_node):
    pressed = [0] * 16
    pressed[1] = 1
    shadow_node._joy_callback(_joy(buttons=pressed))
    assert shadow_node._shadow_estop_latched
    assert shadow_node._synced is False

    shadow_node._joy_callback(_joy())
    assert shadow_node._shadow_estop_latched
    shadow_node._request_reset()
    assert shadow_node._shadow_estop_latched is False
    assert shadow_node._synced is False

    shadow_node._request_home()
    assert shadow_node._synced
    published = shadow_node._command_pub.publish.call_args.args[0]
    assert published.x == 15.0
    assert published.pitch == pytest.approx(-54.48)


def test_joy_timeout_disables_and_requires_home(shadow_node):
    pressed = [0] * 16
    pressed[0] = 1
    shadow_node._joy_callback(_joy(buttons=pressed))
    assert shadow_node._enabled
    shadow_node._last_joy_time = time.monotonic() - 1.0
    shadow_node._check_timeouts(time.monotonic())
    assert shadow_node._enabled is False
    assert shadow_node._synced is False


def test_real_startup_syncs_after_three_home_samples(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmTeleopNode()
    state = ArmState()
    state.state = ArmState.STATE_IDLE
    state.position_valid = True
    state.joint_position = [
        0.0,
        math.radians(112.08),
        math.radians(-89.04),
        math.radians(-77.52),
        0.0,
    ]
    state.gripper_position = 0.1
    for _ in range(3):
        node._state_callback(state)
    assert node._synced
    assert node._target.gripper == pytest.approx(0.1)
    node.destroy_node()
    rclpy.shutdown()
