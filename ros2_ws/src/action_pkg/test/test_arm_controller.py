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

import struct

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from action_interfaces.msg import ArmCommand, ArmState
from action_pkg.arm_controller_node import (
    ArmControllerNode,
    I2C_FAIL_THRESHOLD,
    ERR_JOINT_DISABLED,
    ERR_NONFINITE_FIELD,
    ERR_DURATION_RANGE,
    ERR_GRIPPER_RANGE,
    ERR_ESTOP_LATCHED,
    ERR_STALE_CMD,
    ERR_I2C_LOST,
    ERR_CMD_TIMEOUT,
    ERR_FW_NO_SOLVE,
)


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
    assert struct.unpack_from('<f', writes[-1], 8)[0] == expected_raw
    assert struct.unpack_from('<I', writes[-1], 12)[0] == 120


def test_gripper_duration_defaults_to_one_second_for_legacy_callers(node):
    writes = []
    node._i2c_write = lambda data: writes.append(bytes(data)) or True
    node.handle_command(_cmd(
        ArmCommand.MODE_GRIPPER,
        seq=1,
        gripper_position=0.5,
    ))
    assert struct.unpack_from('<I', writes[-1], 12)[0] == 1000


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
    base = dict(x=0.0, y=0.0, z=0.0, pitch=0.0, duration_sec=1.0)
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
    base = dict(x=0.0, y=0.0, z=0.0, pitch=0.0, duration_sec=1.0)
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=5, **base))
    # seq 0 is the legacy sentinel and must NOT be rejected as stale
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=0, **base))
    assert node._state == ArmState.STATE_MOVING


# ===================== firmware status mapping (sec 3.3) =====================
def test_firmware_arm_ok_maps_to_moving(node):
    node._i2c_read_status = lambda: list(b'ARM_OK__') + [0] * 24
    node.poll_status()
    assert node._state == ArmState.STATE_MOVING


def test_firmware_no_solve_maps_to_error(node):
    node._i2c_read_status = lambda: list(b'NO_SOLVE') + [0] * 24
    node.poll_status()
    assert node._state == ArmState.STATE_ERROR
    assert node._error_code == ERR_FW_NO_SOLVE


def test_arm_done_only_from_moving(node):
    # At startup (IDLE) a stale ARM_DONE must NOT fabricate success.
    node._state = ArmState.STATE_IDLE
    node._i2c_read_status = lambda: list(b'ARM_DONE') + [0] * 24
    node.poll_status()
    assert node._state == ArmState.STATE_IDLE

    # While MOVING, ARM_DONE -> SUCCEEDED.
    node._state = ArmState.STATE_MOVING
    node._last_firmware_status = ''  # force a transition
    node._i2c_read_status = lambda: list(b'ARM_DONE') + [0] * 24
    node.poll_status()
    assert node._state == ArmState.STATE_SUCCEEDED


def test_firmware_status_clears_pending_motion(node):
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0, duration_sec=1.0))
    assert node._pending_motion
    node._i2c_read_status = lambda: list(b'ARM_OK__') + [0] * 24
    node.poll_status()
    assert node._pending_motion is False


# ===================== sec 5.5 I2C failure threshold =====================
def test_i2c_failure_threshold_latches_error(node):
    bus = __import__('unittest.mock', fromlist=['MagicMock']).MagicMock()
    bus.i2c_rdwr.side_effect = OSError('simulated bus error')
    node.bus = bus
    for _ in range(I2C_FAIL_THRESHOLD + 1):
        ArmControllerNode._i2c_write(node, b'\x00' * 32)
    assert node._error_code == ERR_I2C_LOST


# ===================== sec 5.3 command timeout =====================
def test_command_timeout_reports_error(node):
    import time
    node.set_parameters(
        [Parameter('command_timeout_sec', Parameter.Type.DOUBLE, 0.05)])
    # firmware never responds
    node._i2c_read_status = lambda: None
    node.handle_command(_cmd(ArmCommand.MODE_END_EFFECTOR, seq=1,
                             x=0.0, y=0.0, z=0.0, pitch=0.0, duration_sec=1.0))
    assert node._state == ArmState.STATE_MOVING
    time.sleep(0.15)
    node.poll_status()  # triggers the watchdog
    assert node._error_code == ERR_CMD_TIMEOUT
    assert node._state == ArmState.STATE_ERROR


# ===================== legacy compatibility layer =====================
def test_legacy_arm_command(node):
    node.legacy_command_callback(String(data='ARM 0 0 0 0 0 0 1'))
    assert node._state == ArmState.STATE_MOVING


def test_legacy_stop_command(node):
    node.legacy_command_callback(String(data='STOP'))
    assert node._state == ArmState.STATE_IDLE
