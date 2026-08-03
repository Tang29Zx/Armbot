"""State-machine tests for Xbox teleoperation without hardware."""

from dataclasses import replace
import math
import time
from unittest.mock import MagicMock

from action_interfaces.msg import ArmCommand, ArmState
from action_pkg.arm_teleop_node import ArmTeleopNode, ERR_FW_NO_SOLVE
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy


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
    published = [call.args[0]
                 for call in shadow_node._command_pub.publish.call_args_list]
    assert [msg.mode for msg in published] == [
        ArmCommand.MODE_GRIPPER,
        ArmCommand.MODE_WRIST_ROLL,
        ArmCommand.MODE_END_EFFECTOR,
    ]
    assert published[0].gripper_position == 0.0
    assert published[1].x == 15.0
    assert published[1].joint_position[4] == pytest.approx(0.0)
    assert published[2].pitch == pytest.approx(-54.48)


def test_joy_timeout_disables_and_requires_home(shadow_node):
    pressed = [0] * 16
    pressed[0] = 1
    shadow_node._joy_callback(_joy(buttons=pressed))
    assert shadow_node._enabled
    shadow_node._last_joy_time = time.monotonic() - 1.0
    shadow_node._check_timeouts(time.monotonic())
    assert shadow_node._enabled is False
    assert shadow_node._synced is False


def test_arm_stream_sends_one_end_when_stick_returns_to_center(shadow_node):
    shadow_node._enabled = True
    shadow_node._joy_valid = True
    shadow_node._axes = [0.0, 1.0, 0.0, 0.0, 1.0, 1.0]
    shadow_node._last_tick = time.monotonic() - 0.1
    shadow_node._control_tick()

    shadow_node._axes = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0]
    shadow_node._last_tick = time.monotonic() - 0.1
    shadow_node._control_tick()
    shadow_node._last_tick = time.monotonic() - 0.1
    shadow_node._control_tick()

    modes = [call.args[0].mode
             for call in shadow_node._command_pub.publish.call_args_list]
    assert modes == [
        ArmCommand.MODE_CARTESIAN_SERVO,
        ArmCommand.MODE_CARTESIAN_SERVO_END,
    ]
    stream = shadow_node._command_pub.publish.call_args_list[0].args[0]
    assert stream.duration_sec == pytest.approx(0.30)


def test_right_stick_horizontal_publishes_wrist_roll(shadow_node):
    shadow_node._enabled = True
    shadow_node._joy_valid = True
    shadow_node._axes = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0]
    shadow_node._last_tick = time.monotonic() - 0.1

    shadow_node._control_tick()

    command = shadow_node._command_pub.publish.call_args.args[0]
    assert command.mode == ArmCommand.MODE_WRIST_ROLL_SERVO
    assert command.duration_sec == pytest.approx(0.30)
    assert command.joint_position[4] == pytest.approx(
        math.radians(2.0), abs=0.05)
    assert command.pitch == pytest.approx(-54.48)


def test_wrist_stream_sends_one_end_at_center(shadow_node):
    shadow_node._enabled = True
    shadow_node._joy_valid = True
    shadow_node._axes = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0]
    shadow_node._last_tick = time.monotonic() - 0.1
    shadow_node._control_tick()

    shadow_node._axes = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0]
    shadow_node._last_tick = time.monotonic() - 0.1
    shadow_node._control_tick()
    shadow_node._last_tick = time.monotonic() - 0.1
    shadow_node._control_tick()

    modes = [call.args[0].mode
             for call in shadow_node._command_pub.publish.call_args_list]
    assert modes == [
        ArmCommand.MODE_WRIST_ROLL_SERVO,
        ArmCommand.MODE_WRIST_ROLL_SERVO_END,
    ]


def test_rb_modifies_right_stick_horizontal_to_cartesian_pitch(shadow_node):
    shadow_node._enabled = True
    shadow_node._joy_valid = True
    shadow_node._axes = [0.0, 0.0, 1.0, 0.0, 1.0, 1.0]
    shadow_node._buttons[7] = 1
    shadow_node._last_tick = time.monotonic() - 0.1

    shadow_node._control_tick()

    command = shadow_node._command_pub.publish.call_args.args[0]
    assert command.mode == ArmCommand.MODE_CARTESIAN_SERVO
    assert command.pitch == pytest.approx(-53.98, abs=0.05)
    assert command.joint_position[4] == pytest.approx(0.0)


def test_a_pause_ends_stream_but_joy_timeout_does_not(shadow_node):
    shadow_node._enabled = True
    shadow_node._joy_valid = True
    shadow_node._axes = [0.0, 1.0, 0.0, 0.0, 1.0, 1.0]
    shadow_node._last_tick = time.monotonic() - 0.1
    shadow_node._control_tick()

    pressed = [0] * 16
    pressed[0] = 1
    shadow_node._joy_callback(_joy(buttons=pressed))
    assert shadow_node._command_pub.publish.call_args.args[0].mode == (
        ArmCommand.MODE_CARTESIAN_SERVO_END)

    shadow_node._enabled = True
    shadow_node._synced = True
    shadow_node._joy_valid = True
    shadow_node._axes = [0.0, 1.0, 0.0, 0.0, 1.0, 1.0]
    shadow_node._last_tick = time.monotonic() - 0.1
    shadow_node._control_tick()
    shadow_node._command_pub.reset_mock()
    shadow_node._last_joy_time = time.monotonic() - 1.0
    shadow_node._check_timeouts(time.monotonic())
    shadow_node._command_pub.publish.assert_not_called()


def test_no_ik_rolls_back_and_resumes_after_neutral(shadow_node):
    deflected = [0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]
    shadow_node._enabled = True
    shadow_node._joy_valid = True
    shadow_node._axes = deflected

    shadow_node._last_tick = time.monotonic() - 0.1
    shadow_node._control_tick()
    accepted = shadow_node._command_pub.publish.call_args.args[0]
    accepted_target = shadow_node._target

    state = ArmState()
    state.state = ArmState.STATE_MOVING
    state.command_phase = ArmState.PHASE_EXECUTING
    state.sequence_id = accepted.sequence_id
    state.position_valid = True
    state.gripper_position = 0.0
    shadow_node._state_callback(state)
    assert shadow_node._last_successful_arm_target == accepted_target

    shadow_node._last_tick = time.monotonic() - 0.1
    shadow_node._control_tick()
    rejected = shadow_node._command_pub.publish.call_args.args[0]
    assert shadow_node._target.x > accepted_target.x

    state.state = ArmState.STATE_IDLE
    state.command_phase = ArmState.PHASE_FAILED
    state.sequence_id = rejected.sequence_id
    state.error_code = ERR_FW_NO_SOLVE
    shadow_node._state_callback(state)

    assert shadow_node._enabled is False
    assert shadow_node._synced
    assert shadow_node._target == accepted_target
    assert shadow_node._no_ik_waiting_neutral

    shadow_node._joy_callback(_joy(axes=deflected))
    assert shadow_node._enabled is False
    shadow_node._joy_callback(_joy())
    assert shadow_node._enabled
    assert shadow_node._synced
    assert not shadow_node._no_ik_waiting_neutral


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


def test_enabled_teleop_tracks_gripper_feedback_when_triggers_are_released(
        tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmTeleopNode()
    node._enabled = True
    node._target = replace(node._target, gripper=1.0)

    state = ArmState()
    state.state = ArmState.STATE_MOVING
    state.gripper_position = 0.05
    node._state_callback(state)

    assert node._target.gripper == pytest.approx(0.05)

    node._target = replace(node._target, gripper=1.0)
    node._axes[4] = -1.0
    state.gripper_position = 0.2
    node._state_callback(state)
    assert node._target.gripper == pytest.approx(1.0)
    node.destroy_node()
    rclpy.shutdown()


def test_gripper_contact_holds_feedback_and_stops_closing(
        tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmTeleopNode()
    node._command_pub = MagicMock()
    node._enabled = True
    node._joy_valid = True
    node._axes[4] = -1.0
    node._target = replace(node._target, gripper=0.8)

    state = ArmState()
    state.state = ArmState.STATE_MOVING
    state.position_valid = True
    for actual in (0.20, 0.30, 0.31, 0.311, 0.310, 0.311):
        state.gripper_position = actual
        node._state_callback(state)

    hold = node._command_pub.publish.call_args.args[0]
    assert hold.mode == ArmCommand.MODE_GRIPPER_STOP
    assert hold.gripper_position == pytest.approx(0.311)
    assert node._target.gripper == pytest.approx(0.311)
    assert node._gripper_contact_latched

    node._command_pub.reset_mock()
    node._axes[1] = 1.0
    node._last_tick = time.monotonic() - 0.1
    node._control_tick()
    node._command_pub.publish.assert_not_called()

    state.state = ArmState.STATE_SUCCEEDED
    state.sequence_id = hold.sequence_id
    node._state_callback(state)
    node._last_tick = time.monotonic() - 0.1
    node._control_tick()
    moved = node._command_pub.publish.call_args.args[0]
    assert moved.mode == ArmCommand.MODE_CARTESIAN_SERVO

    node._axes[1] = 0.0
    node._axes[4] = 1.0
    node._state_callback(state)
    assert not node._gripper_contact_latched
    node.destroy_node()
    rclpy.shutdown()


def test_stationary_gripper_without_progress_is_not_contact(
        tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmTeleopNode()
    node._command_pub = MagicMock()
    node._enabled = True
    node._axes[4] = -1.0
    node._target = replace(node._target, gripper=0.8)

    state = ArmState()
    state.state = ArmState.STATE_MOVING
    state.position_valid = True
    state.gripper_position = 0.3
    for _ in range(6):
        node._state_callback(state)

    node._command_pub.publish.assert_not_called()
    assert not node._gripper_contact_latched
    node.destroy_node()
    rclpy.shutdown()


def test_releasing_close_trigger_stops_without_feedback_hold(
        tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmTeleopNode()
    node._command_pub = MagicMock()
    node._enabled = True
    node._joy_valid = True
    node._axes[4] = -1.0
    node._target = replace(node._target, gripper=0.7)

    node._last_tick = time.monotonic() - 0.1
    node._control_tick()
    close = node._command_pub.publish.call_args.args[0]
    assert close.mode == ArmCommand.MODE_GRIPPER_SERVO
    assert close.duration_sec == pytest.approx(0.30)
    assert node._gripper_close_command_active
    close_target = node._target.gripper
    node._command_pub.reset_mock()

    state = ArmState()
    state.state = ArmState.STATE_MOVING
    state.position_valid = True
    state.gripper_position = 0.3
    node._state_callback(state)

    node._joy_callback(_joy())
    end = node._command_pub.publish.call_args.args[0]
    assert end.mode == ArmCommand.MODE_GRIPPER_SERVO_END
    assert node._target.gripper == pytest.approx(close_target)
    assert node._gripper_stop_pending_seq is None

    node._command_pub.reset_mock()
    state.gripper_position = 0.32
    node._state_callback(state)
    node._state_callback(state)

    node._command_pub.publish.assert_not_called()
    assert node._target.gripper == pytest.approx(0.32)

    node._axes[1] = 1.0
    node._last_tick = time.monotonic() - 0.1
    node._control_tick()
    moved = node._command_pub.publish.call_args.args[0]
    assert moved.mode == ArmCommand.MODE_CARTESIAN_SERVO
    node.destroy_node()
    rclpy.shutdown()


def test_explicit_home_requires_three_valid_feedback_samples(
        tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmTeleopNode()
    node._command_pub = MagicMock()

    state = ArmState()
    state.state = ArmState.STATE_IDLE
    state.position_valid = False
    node._state_callback(state)
    node._request_home()

    opened = node._command_pub.publish.call_args.args[0]
    assert opened.mode == ArmCommand.MODE_GRIPPER
    assert opened.gripper_position == 0.0
    assert node._command_pub.publish.call_count == 1
    state.state = ArmState.STATE_SUCCEEDED
    state.sequence_id = opened.sequence_id
    node._state_callback(state)
    wrist = node._command_pub.publish.call_args.args[0]
    assert wrist.mode == ArmCommand.MODE_WRIST_ROLL
    assert node._command_pub.publish.call_count == 2
    state.sequence_id = wrist.sequence_id
    node._state_callback(state)
    published = node._command_pub.publish.call_args.args[0]
    assert published.mode == ArmCommand.MODE_END_EFFECTOR
    assert node._command_pub.publish.call_count == 3
    state.sequence_id = published.sequence_id

    node._joy_valid = True
    node._last_joy_time = time.monotonic()
    assert node._synced is False
    assert node._enable_block_reason(time.monotonic()) != ''

    state.position_valid = True
    state.joint_position = [
        0.0,
        math.radians(112.08),
        math.radians(-89.04),
        math.radians(-77.52),
        0.0,
    ]
    for _ in range(2):
        node._state_callback(state)
        assert node._synced is False
    node._state_callback(state)

    assert node._synced is True
    assert node._enable_block_reason(time.monotonic()) == ''
    node.destroy_node()
    rclpy.shutdown()


def test_home_near_open_feedback_stops_gripper_before_wrist(
        tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmTeleopNode()
    node._command_pub = MagicMock()

    state = ArmState()
    state.state = ArmState.STATE_IDLE
    state.position_valid = True
    state.gripper_position = 0.09
    node._state_callback(state)
    node._request_home()

    opened = node._command_pub.publish.call_args.args[0]
    state.state = ArmState.STATE_MOVING
    state.sequence_id = opened.sequence_id
    for _ in range(2):
        node._state_callback(state)
    assert node._command_pub.publish.call_count == 1

    node._state_callback(state)
    stopped = node._command_pub.publish.call_args.args[0]
    assert stopped.mode == ArmCommand.MODE_GRIPPER_STOP
    assert node._command_pub.publish.call_count == 2
    assert node._home_open_pending_seq is None
    assert node._home_open_stop_pending_seq == stopped.sequence_id

    state.state = ArmState.STATE_SUCCEEDED
    state.sequence_id = stopped.sequence_id
    node._state_callback(state)
    wrist = node._command_pub.publish.call_args.args[0]
    assert wrist.mode == ArmCommand.MODE_WRIST_ROLL
    assert node._command_pub.publish.call_count == 3
    assert node._home_open_stop_pending_seq is None
    node.destroy_node()
    rclpy.shutdown()


def test_home_stops_if_gripper_open_fails(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmTeleopNode()
    node._command_pub = MagicMock()

    state = ArmState()
    state.state = ArmState.STATE_IDLE
    node._state_callback(state)
    node._request_home()
    opened = node._command_pub.publish.call_args.args[0]

    state.state = ArmState.STATE_ERROR
    state.sequence_id = opened.sequence_id
    node._state_callback(state)

    assert node._command_pub.publish.call_count == 1
    assert node._home_open_pending_seq is None
    assert node._home_open_stop_pending_seq is None
    assert node._home_roll_pending_seq is None
    assert node._home_pending_seq is None
    assert node._synced is False
    node.destroy_node()
    rclpy.shutdown()


def test_home_feedback_outside_tolerance_does_not_sync(
        tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmTeleopNode()
    node._command_pub = MagicMock()
    state = ArmState()
    state.state = ArmState.STATE_IDLE
    node._state_callback(state)
    node._request_home()
    opened = node._command_pub.publish.call_args.args[0]
    state.state = ArmState.STATE_SUCCEEDED
    state.sequence_id = opened.sequence_id
    node._state_callback(state)
    state.sequence_id = node._home_roll_pending_seq
    node._state_callback(state)
    state.sequence_id = node._home_pending_seq
    state.position_valid = True
    state.joint_position = [1.0] * 5
    for _ in range(5):
        node._state_callback(state)
    assert node._synced is False
    node.destroy_node()
    rclpy.shutdown()


def test_invalid_stream_watchdog_is_rejected(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    with pytest.raises(ValueError, match='stream_watchdog_sec'):
        ArmTeleopNode(parameter_overrides=[
            Parameter('stream_watchdog_sec', value=0.05),
        ])
    rclpy.shutdown()
