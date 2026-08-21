"""Safety and source-selection tests for the RDK command mux."""

from types import SimpleNamespace
from unittest.mock import MagicMock
import time

from action_interfaces.msg import ArmCommand, ArmState
from action_pkg.arm_command_mux_node import ArmCommandMuxNode
import pytest
import rclpy
from std_msgs.msg import Bool, Empty


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    instance = ArmCommandMuxNode()
    instance._command_pub = MagicMock()
    instance._vla_enabled_pub = MagicMock()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def _healthy_state(sequence=20):
    state = ArmState()
    state.state = ArmState.STATE_SUCCEEDED
    state.command_phase = ArmState.PHASE_COMPLETED
    state.sequence_id = sequence
    state.position_valid = True
    state.error_code = 0
    return state


def _prepare_enable(node):
    node._on_state(_healthy_state())
    node._on_heartbeat(Empty())
    node._on_teleop_enabled(Bool(data=False))
    node._on_teleop_synced(Bool(data=True))


def _set_enabled(node, enabled):
    request = SimpleNamespace(data=enabled)
    response = SimpleNamespace(success=False, message="")
    return node._set_vla_enabled(request, response)


def test_enable_requires_verified_home(node):
    node._on_state(_healthy_state())
    node._on_heartbeat(Empty())

    response = _set_enabled(node, True)

    assert not response.success
    assert "Home has not been verified" in response.message


def test_only_selected_source_is_forwarded(node):
    _prepare_enable(node)
    response = _set_enabled(node, True)
    assert response.success

    teleop = ArmCommand(mode=ArmCommand.MODE_CARTESIAN_SERVO, sequence_id=21)
    vla = ArmCommand(mode=ArmCommand.MODE_CARTESIAN_SERVO, sequence_id=22)
    node._on_teleop_command(teleop)
    node._on_vla_command(vla)

    node._command_pub.publish.assert_called_once_with(vla)


def test_post_vla_teleop_motion_is_blocked_until_home(node):
    _prepare_enable(node)
    assert _set_enabled(node, True).success
    assert _set_enabled(node, False).success
    node._command_pub.reset_mock()
    node._on_teleop_enabled(Bool(data=True))

    node._on_teleop_command(
        ArmCommand(mode=ArmCommand.MODE_CARTESIAN_SERVO, sequence_id=30)
    )

    node._command_pub.publish.assert_not_called()


def test_heartbeat_timeout_disables_vla_and_sends_stop(node):
    _prepare_enable(node)
    assert _set_enabled(node, True).success
    node._command_pub.reset_mock()
    node._heartbeat_received_at = time.monotonic() - 1.0

    node._watchdog_tick()

    assert not node._vla_enabled
    stop = node._command_pub.publish.call_args.args[0]
    assert stop.mode == ArmCommand.MODE_STOP
    assert stop.sequence_id > 20


def test_untracked_vla_sequence_disables_and_stops(node):
    _prepare_enable(node)
    assert _set_enabled(node, True).success
    node._command_pub.reset_mock()

    node._on_vla_command(ArmCommand(sequence_id=0))

    assert not node._vla_enabled
    assert node._command_pub.publish.call_args.args[0].mode == ArmCommand.MODE_STOP


def test_stale_vla_sequence_disables_and_stops(node):
    _prepare_enable(node)
    assert _set_enabled(node, True).success
    node._last_forwarded_sequence = 25
    node._command_pub.reset_mock()

    node._on_vla_command(ArmCommand(sequence_id=25))

    assert not node._vla_enabled
    stop = node._command_pub.publish.call_args.args[0]
    assert stop.mode == ArmCommand.MODE_STOP
    assert stop.sequence_id == 26


def test_operator_disable_sends_stop(node):
    _prepare_enable(node)
    assert _set_enabled(node, True).success
    node._command_pub.reset_mock()

    response = _set_enabled(node, False)

    assert response.success
    assert node._command_pub.publish.call_args.args[0].mode == ArmCommand.MODE_STOP


def _state_with_position_valid(node, valid):
    state = _healthy_state()
    state.position_valid = valid
    node._on_state(state)


def _state_with_error_code(node, error_code):
    state = _healthy_state()
    state.error_code = error_code
    node._on_state(state)


def test_single_invalid_feedback_frame_does_not_disable(node):
    _prepare_enable(node)
    assert _set_enabled(node, True).success
    node._command_pub.reset_mock()

    _state_with_position_valid(node, False)

    assert node._vla_enabled
    node._command_pub.publish.assert_not_called()


def test_invalid_feedback_streak_disables_and_sends_stop(node):
    _prepare_enable(node)
    assert _set_enabled(node, True).success
    node._command_pub.reset_mock()

    streak = int(node._cfg("feedback_invalid_streak"))
    for _ in range(streak):
        _state_with_position_valid(node, False)

    node._watchdog_tick()

    assert not node._vla_enabled
    stop = node._command_pub.publish.call_args.args[0]
    assert stop.mode == ArmCommand.MODE_STOP


def test_recoverable_error_code_does_not_disable(node):
    _prepare_enable(node)
    assert _set_enabled(node, True).success
    node._command_pub.reset_mock()

    _state_with_error_code(node, 0x21)

    node._watchdog_tick()

    assert node._vla_enabled
    node._command_pub.publish.assert_not_called()


def test_non_recoverable_error_code_disables(node):
    _prepare_enable(node)
    assert _set_enabled(node, True).success
    node._command_pub.reset_mock()

    _state_with_error_code(node, 0x42)

    node._watchdog_tick()

    assert not node._vla_enabled
    stop = node._command_pub.publish.call_args.args[0]
    assert stop.mode == ArmCommand.MODE_STOP


def test_recovery_resets_invalid_streak(node):
    _prepare_enable(node)
    assert _set_enabled(node, True).success
    node._command_pub.reset_mock()

    streak = int(node._cfg("feedback_invalid_streak"))
    _state_with_position_valid(node, False)
    _state_with_position_valid(node, True)
    for _ in range(streak - 1):
        _state_with_position_valid(node, False)

    node._watchdog_tick()

    assert node._vla_enabled
    node._command_pub.publish.assert_not_called()
