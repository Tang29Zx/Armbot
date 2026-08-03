"""Tests for the VLA-only filtered ArmState topic."""

from unittest.mock import MagicMock

from action_interfaces.msg import ArmState
from action_pkg.arm_state_filter_node import ArmStateFilterNode
from action_pkg.state_filter import MedianOneEuroFilter
import pytest
import rclpy


def _state(value=0.0, *, timestamp=42.0, valid=True,
           state=ArmState.STATE_IDLE):
    msg = ArmState()
    msg.header.stamp.sec = int(timestamp)
    msg.header.stamp.nanosec = int(
        round((timestamp - int(timestamp)) * 1_000_000_000))
    msg.state = state
    msg.command_phase = ArmState.PHASE_EXECUTING
    msg.sequence_id = 7
    msg.joint_position = [value] * 5
    msg.gripper_position = value
    msg.position_valid = valid
    return msg


def test_median_rejects_an_isolated_spike():
    state_filter = MedianOneEuroFilter(1)

    assert state_filter.update([0.0], 0.0) == pytest.approx([0.0])
    assert state_filter.update([0.0], 0.1) == pytest.approx([0.0])
    assert state_filter.update([10.0], 0.2) == pytest.approx([0.0])
    assert state_filter.update([0.0], 0.3) == pytest.approx([0.0])


def test_one_euro_reduces_lag_for_fast_motion():
    fixed = MedianOneEuroFilter(1, beta=0.0)
    adaptive = MedianOneEuroFilter(1, beta=1.5)
    for timestamp in (0.0, 0.1, 0.2):
        fixed.update([0.0], timestamp)
        adaptive.update([0.0], timestamp)

    assert fixed.update([1.0], 0.3) == pytest.approx([0.0])
    assert adaptive.update([1.0], 0.3) == pytest.approx([0.0])
    fixed_step = fixed.update([1.0], 0.4)[0]
    adaptive_step = adaptive.update([1.0], 0.4)[0]

    assert 0.0 < fixed_step < adaptive_step < 1.0


@pytest.mark.parametrize(
    'kwargs',
    [
        {'dimensions': 0},
        {'dimensions': 1, 'window_size': 2},
        {'dimensions': 1, 'min_cutoff_hz': 0.0},
        {'dimensions': 1, 'beta': -0.1},
        {'dimensions': 1, 'derivative_cutoff_hz': 0.0},
        {'dimensions': 1, 'reset_gap_sec': 0.0},
    ],
)
def test_invalid_filter_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        MedianOneEuroFilter(**kwargs)


def test_reset_discards_previous_history():
    state_filter = MedianOneEuroFilter(1)
    for index, value in enumerate((0.0, 0.0, 0.0, 1.0, 1.0)):
        state_filter.update([value], index * 0.1)

    state_filter.reset()

    assert state_filter.update([1.0], 1.0) == pytest.approx([1.0])


@pytest.mark.parametrize('timestamp', [0.2, 1.0])
def test_bad_or_stale_timestamp_reinitializes(timestamp):
    state_filter = MedianOneEuroFilter(1, reset_gap_sec=0.5)
    state_filter.update([0.0], 0.1)
    state_filter.update([0.0], 0.2)

    assert state_filter.update([2.0], timestamp) == pytest.approx([2.0])


def test_node_preserves_metadata_and_does_not_mutate_raw_state(
        tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmStateFilterNode()
    node._publisher = MagicMock()

    for timestamp in (42.0, 42.1, 42.2):
        node._state_callback(_state(0.0, timestamp=timestamp))
    raw = _state(1.0, timestamp=42.3)
    node._state_callback(raw)
    raw.header.stamp.nanosec = 400_000_000
    node._state_callback(raw)
    filtered = node._publisher.publish.call_args.args[0]

    assert filtered.header.stamp.sec == 42
    assert filtered.header.stamp.nanosec == 400_000_000
    assert filtered.sequence_id == 7
    assert filtered.command_phase == ArmState.PHASE_EXECUTING
    assert all(0.5 < value < 1.0 for value in filtered.joint_position)
    assert 0.5 < filtered.gripper_position < 1.0
    assert raw.joint_position == pytest.approx([1.0] * 5)
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize(
    'bad_state',
    [
        _state(0.0, valid=False),
        _state(float('nan')),
        _state(0.0, state=ArmState.STATE_ERROR),
        _state(0.0, state=ArmState.STATE_ESTOP),
    ],
)
def test_invalid_or_unsafe_state_resets_filter(
        bad_state, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    node = ArmStateFilterNode()
    node._publisher = MagicMock()
    node._state_callback(_state(0.0, timestamp=42.0))

    node._state_callback(bad_state)
    invalid = node._publisher.publish.call_args.args[0]
    assert invalid.position_valid is False

    node._state_callback(_state(1.0, timestamp=42.1))
    restarted = node._publisher.publish.call_args.args[0]
    assert restarted.joint_position == pytest.approx([1.0] * 5)
    node.destroy_node()
    rclpy.shutdown()
