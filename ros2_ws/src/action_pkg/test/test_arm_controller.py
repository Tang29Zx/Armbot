"""
Acceptance tests for the action_pkg arm controller contract.

These exercise the typed-interface logic WITHOUT hardware: the I2C layer is
monkeypatched, so they run wherever ROS2 + this package are built/installed.

Run (in a ROS2 shell, after colcon build):
    source install/setup.bash
    pytest ACTION_PKG_SOURCE/test/test_arm_controller.py -v
or, from the workspace:
    pytest ros2_ws/src/action_pkg/test/test_arm_controller.py -v

Missing ROS2 dependencies are collection errors; CI cannot skip the suite.
"""

import errno
import math
import struct
from types import SimpleNamespace
from unittest.mock import MagicMock

from action_interfaces.msg import ArmCommand, ArmState
from action_pkg.arm_controller_node import (
    ArmControllerNode,
    ERR_CMD_TIMEOUT,
    ERR_DURATION_RANGE,
    ERR_ESTOP_LATCHED,
    ERR_FW_MOTION_TIMEOUT,
    ERR_FW_NO_SOLVE,
    ERR_FW_NOT_READY,
    ERR_FW_PROTOCOL,
    ERR_FW_RESTARTED,
    ERR_FW_SERVO_DEADLINE,
    ERR_FW_STREAM_STEP_TOO_LARGE,
    ERR_FW_STREAM_TIMEOUT,
    ERR_GRIPPER_STOP_WRITE,
    ERR_FW_SERVO_FEEDBACK,
    ERR_GRIPPER_RANGE,
    ERR_GRIPPER_UNMAPPED,
    ERR_I2C_LOST,
    ERR_JOINT_DISABLED,
    ERR_NONFINITE_FIELD,
    ERR_STALE_CMD,
    ERR_WRIST_ROLL_RANGE,
    FW_ERROR_MOTION_TIMEOUT,
    FW_ERROR_ARM_NOT_READY,
    FW_ERROR_NO_IK_SOLUTION,
    FW_ERROR_SERVO_FEEDBACK_FAILED,
    FW_ERROR_SERVO_DEADLINE_MISSED,
    FW_ERROR_STREAM_STEP_TOO_LARGE,
    FW_ERROR_STREAM_TIMEOUT,
    FW_LIFECYCLE_ACCEPTED,
    FW_LIFECYCLE_COMPLETED,
    FW_LIFECYCLE_EXECUTING,
    FW_LIFECYCLE_FAILED,
    FW_LIFECYCLE_READY,
    FW_LIFECYCLE_STOPPING,
    I2C_FAIL_THRESHOLD,
    I2C_PROTOCOL_MAGIC,
    I2C_PROTOCOL_VERSION,
    I2C_READ_ATTEMPTS,
    I2C_READ_RETRY_DELAY_SEC,
)
import action_pkg.arm_controller_node as controller_module
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    rclpy.init()
    n = ArmControllerNode()
    # No real I2C in the test environment: stub the bus so writes "succeed"
    # and reads return nothing (firmware silent by default).
    n._i2c_write = lambda data: True
    n._i2c_read_status = lambda: None
    n.i2c_ok = True
    n._firmware_protocol_ok = True
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _cmd(mode, seq=0, **kw):
    c = ArmCommand()
    c.mode = mode
    c.sequence_id = seq
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def _status(lifecycle, wire_id=0, error=0, positions=None):
    packet = bytearray(32)
    packet[0] = I2C_PROTOCOL_MAGIC
    packet[1] = I2C_PROTOCOL_VERSION
    packet[2] = lifecycle
    packet[3] = error
    struct.pack_into('<I', packet, 4, wire_id)
    for index, value in enumerate(positions or [0.0] * 6):
        struct.pack_into('<f', packet, 8 + index * 4, value)
    return list(packet)


# ===================== sec 5.3 validation =====================
def test_mode_joint_disabled(node):
    node.handle_command(_cmd(ArmCommand.MODE_JOINT, seq=1))
    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_JOINT_DISABLED


def test_nonfinite_rejected(node):
    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=1, x=float('nan')))
    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_NONFINITE_FIELD


def test_duration_range_rejected(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0,
                             duration_sec=999.0))
    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_DURATION_RANGE


def test_valid_end_effector_moves(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0, duration_sec=1.0))
    assert node._state == ArmState.STATE_MOVING
    assert node._last_applied_seq == 1


def test_wrist_roll_uses_servo_two_position_packet(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    cmd = _cmd(
        ArmCommand.MODE_WRIST_ROLL, seq=1, duration_sec=0.1)
    cmd.joint_position[4] = math.pi / 2.0

    node.handle_command(cmd)

    packet = writes[-1]
    assert packet[0] == ord('P')
    assert packet[1] == I2C_PROTOCOL_VERSION
    assert struct.unpack_from('<H', packet, 6)[0] == 100
    assert packet[8] == 2
    assert struct.unpack_from('<f', packet, 12)[0] == pytest.approx(125.0)
    assert node._active_mode == ArmCommand.MODE_WRIST_ROLL


def test_wrist_roll_rejects_target_outside_configured_limit(node):
    cmd = _cmd(
        ArmCommand.MODE_WRIST_ROLL, seq=1, duration_sec=0.1)
    cmd.joint_position[4] = math.radians(91.0)

    node.handle_command(cmd)

    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_WRIST_ROLL_RANGE


def test_wrist_roll_rejects_nonfinite_target(node):
    cmd = _cmd(
        ArmCommand.MODE_WRIST_ROLL, seq=1, duration_sec=0.1)
    cmd.joint_position[4] = float('nan')

    node.handle_command(cmd)

    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_NONFINITE_FIELD


def test_wrist_roll_stream_uses_u_packet_with_raw_limits(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    cmd = _cmd(
        ArmCommand.MODE_WRIST_ROLL_SERVO,
        seq=1,
        duration_sec=0.3,
    )
    cmd.joint_position[4] = math.pi / 2.0

    node.handle_command(cmd)

    packet = writes[-1]
    assert packet[0] == ord('U')
    assert packet[8] == 2
    assert struct.unpack_from('<H', packet, 6)[0] == 300
    assert struct.unpack_from('<f', packet, 12)[0] == pytest.approx(125.0)
    assert struct.unpack_from('<f', packet, 16)[0] == pytest.approx(250.0)
    assert struct.unpack_from('<f', packet, 20)[0] == pytest.approx(1000.0)


def test_gripper_stream_sends_latest_target_before_g_end(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_SERVO, seq=1,
        gripper_position=0.2, duration_sec=0.3))
    first_wire = node._active_wire_id
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_SERVO, seq=2,
        gripper_position=0.3, duration_sec=0.3))
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_SERVO_END, seq=3))
    assert [packet[0] for packet in writes] == [ord('U')]

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, first_wire)
    node.poll_status()
    latest_wire = node._active_wire_id
    assert node._active_seq == 2
    assert [packet[0] for packet in writes] == [ord('U'), ord('U')]
    assert writes[-1][8] == 1
    assert struct.unpack_from('<f', writes[-1], 12)[0] == pytest.approx(350.0)
    assert struct.unpack_from('<f', writes[-1], 16)[0] == pytest.approx(300.0)
    assert struct.unpack_from('<f', writes[-1], 20)[0] == pytest.approx(1200.0)

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, latest_wire)
    node.poll_status()
    end_wire = node._active_wire_id
    assert node._active_seq == 3
    assert [packet[0] for packet in writes] == [
        ord('U'), ord('U'), ord('G')]
    assert writes[-1][8] == 1
    assert writes[-1][6:8] == bytes(2)

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, end_wire)
    node.poll_status()
    assert node._stream_open is False
    assert node._stream_open_family is None


def test_wrist_stream_waits_for_gripper_end_completion(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_SERVO, seq=1,
        gripper_position=0.2, duration_sec=0.3))
    gripper_wire = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, gripper_wire)
    node.poll_status()

    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_SERVO_END, seq=2))
    end_wire = node._active_wire_id
    wrist = _cmd(
        ArmCommand.MODE_WRIST_ROLL_SERVO, seq=3,
        duration_sec=0.3)
    wrist.joint_position[4] = 0.1
    node.handle_command(wrist)
    assert [packet[0] for packet in writes] == [ord('U'), ord('G')]

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, end_wire)
    node.poll_status()
    assert [packet[0] for packet in writes] == [ord('U'), ord('G')]
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, end_wire)
    node.poll_status()
    assert [packet[0] for packet in writes] == [
        ord('U'), ord('G'), ord('U')]
    assert writes[-1][8] == 2


def test_motion_rejected_until_v3_status_is_verified(node):
    node._firmware_protocol_ok = False
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0,
                             duration_sec=1.0))
    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_FW_PROTOCOL


def test_end_effector_uses_v3_packet_layout(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=7,
        x=15.0, y=1.0, z=2.0, pitch=-20.0, duration_sec=0.09))

    packet = writes[-1]
    assert packet[0] == ord('A')
    assert packet[1] == I2C_PROTOCOL_VERSION
    assert struct.unpack_from('<I', packet, 2)[0] == node._active_wire_id
    assert struct.unpack_from('<H', packet, 6)[0] == 90
    assert struct.unpack_from('<6f', packet, 8) == pytest.approx(
        [15.0, 1.0, 2.0, -20.0, -90.0, 90.0])


def test_cartesian_stream_waits_for_executing_and_uses_v3_tag(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO, seq=8,
        x=15.0, y=0.1, z=2.0, pitch=-54.48, duration_sec=0.2))

    packet = writes[-1]
    wire_id = node._active_wire_id
    assert packet[0] == ord('T')
    assert packet[1] == 3
    assert struct.unpack_from('<H', packet, 6)[0] == 200

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, wire_id)
    node.poll_status()
    assert node._pending_motion is True
    assert node._command_phase == ArmState.PHASE_ACCEPTED

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, wire_id)
    node.poll_status()
    assert node._pending_motion is False
    assert node._stream_open is True
    assert node._command_phase == ArmState.PHASE_EXECUTING


def test_latest_stream_target_is_sent_before_pending_end(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    base = dict(x=15.0, y=0.1, z=2.0, pitch=-54.48,
                duration_sec=0.2)
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO, seq=1, **base))
    first_wire = node._active_wire_id
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO, seq=2, **base))
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO, seq=3, **base))
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO_END, seq=4))
    assert [packet[0] for packet in writes] == [ord('T')]

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, first_wire)
    node.poll_status()
    latest_wire = node._active_wire_id
    assert node._active_seq == 3
    assert [packet[0] for packet in writes] == [ord('T'), ord('T')]

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, latest_wire)
    node.poll_status()
    end_wire = node._active_wire_id
    assert node._active_seq == 4
    assert [packet[0] for packet in writes] == [
        ord('T'), ord('T'), ord('F')]
    assert writes[-1][6:] == bytes(26)

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, end_wire)
    node.poll_status()
    assert node._state == ArmState.STATE_SUCCEEDED
    assert node._command_phase == ArmState.PHASE_COMPLETED
    assert node._stream_open is False


def test_stream_step_rejection_keeps_stream_open_for_end(node):
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO, seq=1,
        x=15.0, y=0.1, z=2.0, pitch=-54.48, duration_sec=0.2))
    wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, wire_id)
    node.poll_status()
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_FAILED, wire_id, FW_ERROR_STREAM_STEP_TOO_LARGE)
    node.poll_status()

    assert node._state == ArmState.STATE_IDLE
    assert node._error_code == ERR_FW_STREAM_STEP_TOO_LARGE
    assert node._stream_open is True
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO_END, seq=2))
    assert node._active_mode == ArmCommand.MODE_CARTESIAN_SERVO_END


def test_stream_step_rejection_is_published_before_queued_end(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.state_pub = MagicMock()

    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_SERVO, seq=1,
        gripper_position=0.1, duration_sec=0.3))
    first_wire = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, first_wire)
    node.poll_status()

    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_SERVO, seq=2,
        gripper_position=0.2, duration_sec=0.3))
    rejected_wire = node._active_wire_id
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_SERVO, seq=3,
        gripper_position=0.3, duration_sec=0.3))
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_SERVO_END, seq=4))
    assert node._queued_stream_end.sequence_id == 4
    node.state_pub.reset_mock()

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_FAILED, rejected_wire,
        FW_ERROR_STREAM_STEP_TOO_LARGE)
    node.poll_status()

    assert node.state_pub.publish.call_count == 1
    rejected = node.state_pub.publish.call_args.args[0]
    assert rejected.state == ArmState.STATE_IDLE
    assert rejected.command_phase == ArmState.PHASE_FAILED
    assert rejected.sequence_id == 2
    assert rejected.error_code == ERR_FW_STREAM_STEP_TOO_LARGE
    assert node._active_mode == ArmCommand.MODE_GRIPPER_SERVO_END
    assert node._active_seq == 4
    assert [packet[0] for packet in writes] == [
        ord('U'), ord('U'), ord('G')]


@pytest.mark.parametrize('firmware_error, expected_error', [
    (FW_ERROR_STREAM_TIMEOUT, ERR_FW_STREAM_TIMEOUT),
    (FW_ERROR_SERVO_DEADLINE_MISSED, ERR_FW_SERVO_DEADLINE),
])
def test_stream_stopping_requires_home(node, firmware_error, expected_error):
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO, seq=1,
        x=15.0, y=0.1, z=2.0, pitch=-54.48, duration_sec=0.2))
    wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_STOPPING, wire_id, firmware_error)
    node.poll_status()

    assert node._state == ArmState.STATE_ERROR
    assert node._command_phase == ArmState.PHASE_STOPPING
    assert node._error_code == expected_error
    assert node._stream_open is False


def test_arm_not_ready_error_is_published_immediately(node):
    node.state_pub = MagicMock()
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO, seq=1,
        x=15.0, y=0.1, z=2.0, pitch=-54.48, duration_sec=0.3))
    wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_FAILED, wire_id, FW_ERROR_ARM_NOT_READY)

    node.poll_status()

    published = node.state_pub.publish.call_args_list[-1].args[0]
    assert published.state == ArmState.STATE_IDLE
    assert published.command_phase == ArmState.PHASE_FAILED
    assert published.sequence_id == 1
    assert published.error_code == ERR_FW_NOT_READY


def test_arm_not_ready_non_stream_still_latches(node):
    node.state_pub = MagicMock()
    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=1,
        x=15.0, y=0.0, z=2.0, pitch=-54.48, duration_sec=0.3))
    wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_FAILED, wire_id, FW_ERROR_ARM_NOT_READY)

    node.poll_status()

    published = node.state_pub.publish.call_args_list[-1].args[0]
    assert published.state == ArmState.STATE_ERROR
    assert published.error_code == ERR_FW_NOT_READY


def test_end_effector_units_are_centimeters(node):
    assert node._cfg('end_effector_units') == 'cm'


def test_gripper_range_rejected(node):
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=1, gripper_position=1.5))
    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_GRIPPER_RANGE


@pytest.mark.parametrize('position, expected_raw', [
    (0.0, 200.0),
    (0.5, 450.0),
    (1.0, 700.0),
])
def test_gripper_open_close_contract(node, position, expected_raw):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER,
        seq=1,
        gripper_position=position,
        duration_sec=0.12,
    ))
    assert writes[-1][0] == ord('P')
    assert writes[-1][1] == I2C_PROTOCOL_VERSION
    assert writes[-1][8] == 1
    assert struct.unpack_from('<f', writes[-1], 12)[0] == expected_raw
    assert struct.unpack_from('<H', writes[-1], 6)[0] == 120


def test_gripper_duration_defaults_to_one_second_for_legacy_callers(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER,
        seq=1,
        gripper_position=0.5,
    ))
    assert struct.unpack_from('<H', writes[-1], 6)[0] == 1000


def test_gripper_command_does_not_replace_feedback(node):
    node._gripper_position = 0.25
    node._i2c_write = lambda data: True

    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER,
        seq=1,
        gripper_position=0.8,
        duration_sec=0.12,
    ))

    assert node._gripper_position == pytest.approx(0.25)


def test_gripper_stop_preempts_position_and_ignores_old_completion(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True

    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER,
        seq=1,
        gripper_position=0.8,
        duration_sec=0.09,
    ))
    old_wire_id = node._active_wire_id

    node.handle_command(_cmd(ArmCommand.MODE_GRIPPER_STOP, seq=2))
    stop_wire_id = node._active_wire_id

    assert [packet[0] for packet in writes] == [ord('P'), ord('H')]
    assert writes[-1][1] == I2C_PROTOCOL_VERSION
    assert struct.unpack_from('<I', writes[-1], 2)[0] == stop_wire_id
    assert struct.unpack_from('<H', writes[-1], 6)[0] == 0
    assert writes[-1][8] == 1
    assert writes[-1][9:] == bytes(23)
    assert node._active_seq == 2
    assert node._queued_command is None

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, old_wire_id)
    node.poll_status()
    assert node._state == ArmState.STATE_MOVING
    assert node._active_wire_id == stop_wire_id

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, stop_wire_id)
    node.poll_status()
    assert node._state == ArmState.STATE_SUCCEEDED
    assert node._last_sequence_id == 2


def test_gripper_stop_closes_direct_stream_before_arm_motion(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True

    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_SERVO,
        seq=1,
        gripper_position=0.8,
        duration_sec=0.3,
    ))
    stream_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, stream_wire_id)
    node.poll_status()
    assert node._stream_open
    assert node._stream_open_family == 'gripper'

    node.handle_command(_cmd(ArmCommand.MODE_GRIPPER_STOP, seq=2))
    stop_wire_id = node._active_wire_id
    assert [packet[0] for packet in writes] == [ord('U'), ord('H')]
    assert node._stream_open is False
    assert node._stream_open_family is None

    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR,
        seq=3,
        x=15.0,
        y=0.0,
        z=2.0,
        pitch=-54.48,
        duration_sec=0.09,
    ))
    assert node._queued_command.sequence_id == 3
    assert [packet[0] for packet in writes] == [ord('U'), ord('H')]

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, stop_wire_id)
    node.poll_status()

    assert [packet[0] for packet in writes] == [
        ord('U'), ord('H'), ord('A')]
    assert node._active_seq == 3


def test_gripper_stop_waits_for_active_arm_completion(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR,
        seq=1,
        x=15.0,
        y=0.0,
        z=2.0,
        pitch=-54.48,
        duration_sec=0.09,
    ))
    arm_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, arm_wire_id)
    node.poll_status()

    node.handle_command(_cmd(ArmCommand.MODE_GRIPPER_STOP, seq=2))

    assert [packet[0] for packet in writes] == [ord('A')]
    assert node._queued_command.mode == ArmCommand.MODE_GRIPPER_STOP

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, arm_wire_id)
    node.poll_status()

    assert [packet[0] for packet in writes] == [ord('A'), ord('H')]
    assert node._active_seq == 2


def test_failed_gripper_stop_write_enters_error(node):
    writes = []

    def write(data):
        writes.append(bytes(data))
        return len(writes) == 1

    node._i2c_write = write
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER,
        seq=1,
        gripper_position=0.8,
        duration_sec=0.09,
    ))
    old_wire_id = node._active_wire_id

    node.handle_command(_cmd(ArmCommand.MODE_GRIPPER_STOP, seq=2))

    assert [packet[0] for packet in writes] == [ord('P'), ord('H')]
    assert old_wire_id != 0
    assert node._active_wire_id == 0
    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_GRIPPER_STOP_WRITE
    assert node._last_sequence_id == 2

    node._i2c_read_status = lambda: _status(FW_LIFECYCLE_READY)
    node.poll_status()

    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_GRIPPER_STOP_WRITE


def test_unmapped_gripper_stop_keeps_mapping_error(node):
    node.set_parameters([Parameter(
        'servo_id_map', Parameter.Type.STRING_ARRAY,
        ['joint_1_base:6', 'joint_2_shoulder:5', 'joint_3_elbow:4',
         'joint_4_wrist_pitch:3', 'joint_5_wrist_roll:2'],
    )])

    node.handle_command(_cmd(ArmCommand.MODE_GRIPPER_STOP, seq=1))

    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_GRIPPER_UNMAPPED
    assert node._active_wire_id == 0


def test_global_stop_packet_remains_payload_free(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True

    node.handle_command(_cmd(ArmCommand.MODE_STOP, seq=1))

    assert writes[-1][0] == ord('S')
    assert writes[-1][1] == I2C_PROTOCOL_VERSION
    assert writes[-1][8:] == bytes(24)


def test_real_gripper_and_home_joint_feedback(node):
    packet = bytearray(32)
    reset_raw = [200.0, 500.0, 177.0, 129.0, 408.0, 500.0]
    for index, value in enumerate(reset_raw):
        struct.pack_into('<f', packet, 8 + index * 4, value)

    node._update_joint_feedback(packet)

    assert node._gripper_position == pytest.approx(0.0)
    assert node._joint_position == pytest.approx([
        0.0,
        1.95616,
        -1.55404,
        -1.35298,
        0.0,
    ], abs=1e-4)

    struct.pack_into('<f', packet, 8, 450.0)
    node._update_joint_feedback(packet)
    assert node._gripper_position == pytest.approx(0.5)


def test_missing_arm_servo_feedback_is_invalid(node):
    packet = bytearray(32)
    struct.pack_into('<f', packet, 8, 497.0)

    node._update_joint_feedback(packet)

    assert node._gripper_position == pytest.approx(0.594)
    assert node._position_valid is False


def test_single_transient_zero_servo_is_held_valid(node):
    packet = bytearray(32)
    reset_raw = [200.0, 500.0, 177.0, 129.0, 408.0, 500.0]
    for index, value in enumerate(reset_raw):
        struct.pack_into('<f', packet, 8 + index * 4, value)
    node._update_joint_feedback(packet)
    assert node._position_valid is True

    struct.pack_into('<f', packet, 8 + 5 * 4, 0.0)
    node._update_joint_feedback(packet)

    assert node._position_valid is True
    assert node._servo_raw_positions[5] == pytest.approx(500.0)
    assert node._joint_position == pytest.approx([
        0.0,
        1.95616,
        -1.55404,
        -1.35298,
        0.0,
    ], abs=1e-4)


def test_two_transient_zero_servos_are_invalid(node):
    packet = bytearray(32)
    reset_raw = [200.0, 500.0, 177.0, 129.0, 408.0, 500.0]
    for index, value in enumerate(reset_raw):
        struct.pack_into('<f', packet, 8 + index * 4, value)
    node._update_joint_feedback(packet)
    assert node._position_valid is True

    struct.pack_into('<f', packet, 8 + 4 * 4, 0.0)
    struct.pack_into('<f', packet, 8 + 5 * 4, 0.0)
    node._update_joint_feedback(packet)

    assert node._position_valid is False


# ===================== sec 3.2 / 5.4 emergency stop =====================
def test_estop_latch_rejects_motion(node):
    node.emergency_stop_callback(Bool(data=True))
    assert node._estop_latched
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0, duration_sec=1.0))
    assert node._state == ArmState.STATE_ESTOP
    assert node._error_code == ERR_ESTOP_LATCHED


def test_estop_sets_estop_state(node):
    node.emergency_stop_callback(Bool(data=True))
    assert node._state == ArmState.STATE_ESTOP


def test_estop_false_does_not_release(node):
    node.emergency_stop_callback(Bool(data=True))
    assert node._estop_latched
    node.emergency_stop_callback(Bool(data=False))
    assert node._estop_latched  # latch stays until /arm/reset_error


def test_stop_does_not_clear_latched_estop_state(node):
    node.emergency_stop_callback(Bool(data=True))
    node.handle_command(_cmd(ArmCommand.MODE_STOP, seq=1))

    assert node._estop_latched
    assert node._state == ArmState.STATE_ESTOP


def test_reset_error_clears_estop_after_request_is_released(node):
    node.emergency_stop_callback(Bool(data=True))
    node.emergency_stop_callback(Bool(data=False))

    resp = node.reset_error_callback(Trigger.Request(), Trigger.Response())

    assert resp.success is True
    assert node._estop_latched is False
    assert node._state == ArmState.STATE_IDLE


def test_reset_error_clears_joint_disabled(node):
    node.handle_command(_cmd(ArmCommand.MODE_JOINT, seq=1))
    assert node._state == ArmState.STATE_ERROR
    resp = node.reset_error_callback(Trigger.Request(), Trigger.Response())
    assert resp.success is True
    assert node._state == ArmState.STATE_IDLE
    assert node._error_code == 0


def test_reset_error_blocked_while_estop(node):
    node.emergency_stop_callback(Bool(data=True))
    resp = node.reset_error_callback(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'estop' in resp.message


# ===================== sec 5.3 stale / duplicate =====================
def test_stale_and_duplicate_sequence_rejected(node):
    base = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'pitch': 0.0,
            'duration_sec': 1.0}
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=5, **base))
    assert node._last_applied_seq == 5

    # exact duplicate
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=5, **base))
    assert node._error_code == ERR_STALE_CMD

    # out-of-order (lower seq)
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=3, **base))
    assert node._error_code == ERR_STALE_CMD

    # forward progress is accepted
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=6, **base))
    assert node._last_applied_seq == 6


def test_legacy_seq_zero_bypasses_stale_check(node):
    base = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'pitch': 0.0,
            'duration_sec': 1.0}
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=5, **base))
    # seq 0 is the legacy sentinel and must NOT be rejected as stale
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=0, **base))
    assert node._state == ArmState.STATE_MOVING


# ===================== firmware status mapping (sec 3.3) =====================
def test_matching_ack_clears_command_watchdog(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0,
                             duration_sec=1.0))
    wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, wire_id)
    node.poll_status()

    assert node._state == ArmState.STATE_MOVING
    assert node._pending_motion is False


def test_nonmatching_status_does_not_clear_command_watchdog(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0,
                             duration_sec=1.0))
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, node._active_wire_id + 1)
    node.poll_status()

    assert node._pending_motion is True
    assert node._state == ArmState.STATE_MOVING


def test_no_ik_is_recoverable_without_reset(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=3,
                             x=0.0, y=0.0, z=0.0, pitch=0.0,
                             duration_sec=1.0))
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=4,
        gripper_position=1.0, duration_sec=0.09))
    assert node._queued_command is not None

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_FAILED, node._active_wire_id,
        FW_ERROR_NO_IK_SOLUTION)
    node.poll_status()

    assert node._state == ArmState.STATE_IDLE
    assert node._error_code == ERR_FW_NO_SOLVE
    assert node._queued_command is None
    assert node._last_sequence_id == 3

    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=5,
        x=15.0, y=0.0, z=2.0, pitch=-54.48, duration_sec=0.09))
    assert node._state == ArmState.STATE_MOVING
    assert node._error_code == 0


def test_matching_motion_timeout_is_never_success(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=4,
                             x=0.0, y=0.0, z=0.0, pitch=0.0,
                             duration_sec=1.0))
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_FAILED, node._active_wire_id,
        FW_ERROR_MOTION_TIMEOUT)
    node.poll_status()

    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_FW_MOTION_TIMEOUT
    assert node._last_sequence_id == 4


def test_feedback_failure_reports_invalid_servo_ids(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=5,
                             x=0.0, y=0.0, z=0.0, pitch=0.0,
                             duration_sec=1.0))
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_FAILED, node._active_wire_id,
        FW_ERROR_SERVO_FEEDBACK_FAILED,
        positions=[500.0, 0.0, 500.0, 500.0, 500.0, 500.0])
    node.poll_status()

    assert node._error_code == ERR_FW_SERVO_FEEDBACK
    assert 'invalid_servo_ids=2' in node._error_message


def test_completion_requires_matching_wire_id(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0,
                             duration_sec=1.0))
    wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, wire_id + 1)
    node.poll_status()
    assert node._state == ArmState.STATE_MOVING

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, wire_id)
    node.poll_status()
    assert node._state == ArmState.STATE_SUCCEEDED


def test_old_text_status_fails_protocol_instead_of_acknowledging(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0,
                             duration_sec=1.0))
    node._i2c_read_status = lambda: list(b'ARM_DONE') + [0] * 24
    node.poll_status()

    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_FW_PROTOCOL
    assert node._firmware_protocol_ok is False


def test_firmware_restart_is_latched(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0, duration_sec=1.0))
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, node._active_wire_id)
    node.poll_status()
    node._i2c_read_status = lambda: _status(FW_LIFECYCLE_READY)
    node.poll_status()

    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_FW_RESTARTED
    assert node._firmware_restart_latched is True


def test_waiting_for_ack_keeps_only_latest_command(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    base = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'pitch': 0.0,
            'duration_sec': 1.0}
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1, **base))
    first_wire_id = node._active_wire_id
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=2, **base))
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=3, **base))

    assert len(writes) == 1
    assert node._queued_command.sequence_id == 3

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, first_wire_id)
    node.poll_status()

    assert len(writes) == 2
    assert node._active_seq == 3
    assert node._pending_motion is True


def test_different_motion_modes_wait_for_completion(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=1,
        gripper_position=1.0, duration_sec=0.09))
    gripper_wire_id = node._active_wire_id

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, gripper_wire_id)
    node.poll_status()
    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=2,
        x=15.0, y=0.1, z=2.0, pitch=-54.48, duration_sec=0.09))

    assert [packet[0] for packet in writes] == [ord('P')]
    assert node._queued_command.sequence_id == 2

    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, gripper_wire_id)
    node.poll_status()

    assert [packet[0] for packet in writes] == [ord('P'), ord('A')]
    assert node._active_seq == 2


def test_queued_command_preserves_active_public_lifecycle(node):
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=1,
        gripper_position=1.0, duration_sec=0.09))
    active_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, active_wire_id)
    node.poll_status()

    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=2,
        x=15.0, y=0.0, z=2.0, pitch=-54.48,
        duration_sec=0.09))

    assert node._queued_command.sequence_id == 2
    assert node._state == ArmState.STATE_MOVING
    assert node._command_phase == ArmState.PHASE_ACCEPTED
    assert node._last_sequence_id == 1


def test_queued_stream_end_preserves_active_public_lifecycle(node):
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO, seq=1,
        x=15.0, y=0.0, z=2.0, pitch=-54.48,
        duration_sec=0.3))

    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO_END, seq=2))

    assert node._queued_stream_end.sequence_id == 2
    assert node._state == ArmState.STATE_MOVING
    assert node._command_phase == ArmState.PHASE_NONE
    assert node._last_sequence_id == 1


def test_completed_state_is_published_before_queued_motion(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.state_pub = MagicMock()

    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER_STOP, seq=1))
    stop_wire_id = node._active_wire_id
    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=2,
        x=15.0, y=0.0, z=2.0, pitch=-54.48,
        duration_sec=0.09))

    assert node._queued_command.sequence_id == 2
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, stop_wire_id)
    node.poll_status()

    completed = node.state_pub.publish.call_args_list[-1].args[0]
    assert completed.state == ArmState.STATE_SUCCEEDED
    assert completed.command_phase == ArmState.PHASE_COMPLETED
    assert completed.sequence_id == 1
    assert node._active_seq == 2
    assert [packet[0] for packet in writes] == [ord('H'), ord('A')]


def test_newer_arm_command_discards_queued_gripper(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    arm = {'x': 15.0, 'y': 0.1, 'z': 2.0, 'pitch': -54.48,
           'duration_sec': 0.09}

    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1, **arm))
    first_arm_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, first_arm_wire_id)
    node.poll_status()

    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=2,
        gripper_position=0.0, duration_sec=0.09))
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=3, **arm))
    latest_arm_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, latest_arm_wire_id)
    node.poll_status()

    assert [packet[0] for packet in writes] == [ord('A'), ord('A')]
    assert node._queued_command is None


def test_newer_gripper_command_discards_queued_arm(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True

    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=1,
        gripper_position=1.0, duration_sec=0.09))
    first_gripper_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, first_gripper_wire_id)
    node.poll_status()

    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=2,
        x=15.0, y=0.1, z=2.0, pitch=-54.48, duration_sec=0.09))
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=3,
        gripper_position=0.0, duration_sec=0.09))
    latest_gripper_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_COMPLETED, latest_gripper_wire_id)
    node.poll_status()

    assert [packet[0] for packet in writes] == [ord('P'), ord('P')]
    assert node._queued_command is None


def test_stop_clears_queued_command(node):
    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=1,
        x=15.0, y=0.1, z=2.0, pitch=-54.48, duration_sec=0.09))
    active_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, active_wire_id)
    node.poll_status()
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=2,
        gripper_position=1.0, duration_sec=0.09))
    assert node._queued_command is not None

    node.handle_command(_cmd(ArmCommand.MODE_STOP, seq=3))

    assert node._queued_command is None


# ===================== sec 5.5 I2C failure threshold =====================
def test_i2c_read_retries_remote_io_within_same_poll(node, monkeypatch):
    packet = _status(FW_LIFECYCLE_EXECUTING, wire_id=7)
    bus = MagicMock()
    bus.i2c_rdwr.side_effect = [
        OSError(errno.EREMOTEIO, 'lost arbitration'),
        None,
    ]
    read = MagicMock(side_effect=[list(packet), list(packet)])
    monkeypatch.setattr(
        controller_module, 'i2c_msg', SimpleNamespace(read=read))
    delays = []
    monkeypatch.setattr(
        controller_module.time, 'sleep', delays.append)
    node.bus = bus

    result = ArmControllerNode._i2c_read_status(node)

    assert result == packet
    assert bus.i2c_rdwr.call_count == 2
    assert delays == [I2C_READ_RETRY_DELAY_SEC]
    assert node._i2c_fail_count == 0
    assert node._i2c_failure_total == 0
    assert node._i2c_read_retry_total == 1


def test_i2c_read_counts_exhausted_retries_as_one_poll_failure(
        node, monkeypatch):
    bus = MagicMock()
    bus.i2c_rdwr.side_effect = OSError(
        errno.EREMOTEIO, 'lost arbitration')
    read = MagicMock(side_effect=[bytearray(32)] * I2C_READ_ATTEMPTS)
    monkeypatch.setattr(
        controller_module, 'i2c_msg', SimpleNamespace(read=read))
    delays = []
    monkeypatch.setattr(
        controller_module.time, 'sleep', delays.append)
    node.bus = bus

    result = ArmControllerNode._i2c_read_status(node)

    assert result is None
    assert bus.i2c_rdwr.call_count == I2C_READ_ATTEMPTS
    assert delays == [I2C_READ_RETRY_DELAY_SEC] * (
        I2C_READ_ATTEMPTS - 1)
    assert node._i2c_fail_count == 1
    assert node._i2c_failure_total == 1


def test_i2c_read_does_not_retry_kernel_timeout(node, monkeypatch):
    bus = MagicMock()
    bus.i2c_rdwr.side_effect = OSError(errno.ETIMEDOUT, 'timed out')
    read = MagicMock(return_value=bytearray(32))
    monkeypatch.setattr(
        controller_module, 'i2c_msg', SimpleNamespace(read=read))
    delays = []
    monkeypatch.setattr(
        controller_module.time, 'sleep', delays.append)
    node.bus = bus

    result = ArmControllerNode._i2c_read_status(node)

    assert result is None
    assert bus.i2c_rdwr.call_count == 1
    assert delays == []
    assert node._i2c_fail_count == 1


def test_i2c_failure_threshold_latches_error(node):
    bus = __import__('unittest.mock', fromlist=['MagicMock']).MagicMock()
    bus.i2c_rdwr.side_effect = OSError('simulated bus error')
    node.bus = bus
    for _ in range(I2C_FAIL_THRESHOLD - 1):
        ArmControllerNode._i2c_write(node, b'\x00' * 32)
    assert node._error_code != ERR_I2C_LOST

    ArmControllerNode._i2c_write(node, b'\x00' * 32)
    assert node._error_code == ERR_I2C_LOST


def test_corrupt_v3_lifecycle_is_a_protocol_error(node):
    node._i2c_read_status = lambda: _status(0xFF)
    node.poll_status()

    assert node._firmware_protocol_ok is False
    assert node._error_code == ERR_FW_PROTOCOL


def test_v2_status_is_rejected_after_v3_upgrade(node):
    packet = _status(FW_LIFECYCLE_READY)
    packet[1] = 2
    node._i2c_read_status = lambda: packet
    node.poll_status()

    assert node._firmware_protocol_ok is False
    assert node._error_code == ERR_FW_PROTOCOL


# ===================== sec 5.3 command timeout =====================
def test_command_timeout_reports_error(node):
    import time
    node.set_parameters(
        [Parameter('command_timeout_sec', Parameter.Type.DOUBLE, 0.05)])
    # firmware never responds
    node._i2c_read_status = lambda: None
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0, duration_sec=1.0))
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=2,
        gripper_position=1.0, duration_sec=0.09))
    assert node._queued_command is not None
    assert node._state == ArmState.STATE_MOVING
    time.sleep(0.15)
    node.poll_status()  # triggers the watchdog
    assert node._error_code == ERR_CMD_TIMEOUT
    assert node._state == ArmState.STATE_ERROR
    assert node._queued_command is None


def test_missing_terminal_lifecycle_times_out_and_clears_queue(node):
    import time
    node.set_parameters(
        [Parameter('command_timeout_sec', Parameter.Type.DOUBLE, 0.05)])
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=1,
        gripper_position=1.0, duration_sec=0.05))
    active_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, active_wire_id)
    node.poll_status()
    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=2,
        x=15.0, y=0.0, z=2.0, pitch=-54.48,
        duration_sec=0.05))

    time.sleep(0.15)
    node._i2c_read_status = lambda: None
    node.poll_status()

    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_CMD_TIMEOUT
    assert 'terminal firmware lifecycle' in node._error_message
    assert node._queued_command is None
    assert node._active_wire_id == 0


def test_stream_end_uses_full_motion_window_then_locks_error(node):
    import time
    node.set_parameters([
        Parameter('command_timeout_sec', Parameter.Type.DOUBLE, 0.05),
        Parameter('max_duration_sec', Parameter.Type.DOUBLE, 0.20),
    ])
    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO, seq=1,
        x=15.0, y=0.0, z=2.0, pitch=-54.48,
        duration_sec=0.10))
    stream_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, stream_wire_id)
    node.poll_status()

    node.handle_command(_cmd(
        ArmCommand.MODE_CARTESIAN_SERVO_END, seq=2))
    end_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_EXECUTING, end_wire_id)
    node.poll_status()

    time.sleep(0.10)
    node._i2c_read_status = lambda: None
    node.poll_status()
    assert node._state == ArmState.STATE_MOVING
    assert node._active_wire_id == end_wire_id

    time.sleep(0.20)
    node.poll_status()
    assert node._state == ArmState.STATE_ERROR
    assert node._command_phase == ArmState.PHASE_FAILED
    assert node._error_code == ERR_CMD_TIMEOUT
    assert 'terminal firmware lifecycle' in node._error_message
    assert node._active_wire_id == 0


def test_non_stream_command_timeout_still_locks_error(node):
    import time
    node.set_parameters(
        [Parameter('command_timeout_sec', Parameter.Type.DOUBLE, 0.05)])
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER, seq=1,
        gripper_position=1.0, duration_sec=0.05))
    active_wire_id = node._active_wire_id
    node._i2c_read_status = lambda: _status(
        FW_LIFECYCLE_ACCEPTED, active_wire_id)
    node.poll_status()

    time.sleep(0.15)
    node._i2c_read_status = lambda: None
    node.poll_status()

    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_CMD_TIMEOUT
    assert node._active_wire_id == 0


def test_reset_recovers_orphan_moving_without_active_command(node):
    node._set_state(
        ArmState.STATE_MOVING, seq=9, phase=ArmState.PHASE_NONE)

    response = node.reset_error_callback(
        Trigger.Request(), Trigger.Response())

    assert response.success is True
    assert node._state == ArmState.STATE_IDLE
    assert node._command_phase == ArmState.PHASE_NONE


def test_poll_reconciles_orphan_moving_without_active_command(node):
    node._set_state(
        ArmState.STATE_MOVING, seq=9, phase=ArmState.PHASE_NONE)

    node.poll_status()

    assert node._state == ArmState.STATE_IDLE
    assert node._command_phase == ArmState.PHASE_NONE


def test_reset_remains_blocked_while_command_is_active(node):
    node.handle_command(_cmd(
        ArmCommand.MODE_END_EFFECTOR, seq=1,
        x=15.0, y=0.0, z=2.0, pitch=-54.48,
        duration_sec=0.05))

    response = node.reset_error_callback(
        Trigger.Request(), Trigger.Response())

    assert response.success is False
    assert 'command active' in response.message


# ===================== legacy compatibility layer =====================
def test_legacy_arm_command(node):
    node.legacy_command_callback(String(data='ARM 0 0 0 0 0 0 1'))
    assert node._state == ArmState.STATE_MOVING


def test_legacy_stop_command(node):
    node.legacy_command_callback(String(data='STOP'))
    assert node._state == ArmState.STATE_IDLE
