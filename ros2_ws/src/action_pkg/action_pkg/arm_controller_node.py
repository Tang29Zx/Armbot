#!/usr/bin/env python3
"""
Arm control node implementing the stable ROS2 interface contract.

Stable ROS2 interface (docs/arm-control-interface.md):
  /arm/command        action_interfaces/msg/ArmCommand
  /arm/emergency_stop std_msgs/msg/Bool            (latched)
  /arm/state          action_interfaces/msg/ArmState
  /joint_states       sensor_msgs/msg/JointState   (gated by config)
  /arm/reset_error    std_srvs/srv/Trigger

Deprecated compatibility layer (routes through the SAME safety path):
  /command_topic      std_msgs/msg/String          (ARM / SERVO / STOP)
  /status_topic       std_msgs/msg/String          (8-byte STM32 status text)

Only this node owns the target I2C address (contract sec 5.2).
"""

import copy
import errno
import math
import struct
import time

from action_interfaces.msg import ArmCommand, ArmState
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:  # pragma: no cover - runtime dep, keep node importable for inspection
    SMBus = None
    i2c_msg = None


CMD_PACKET_SIZE = 32
STATUS_PACKET_SIZE = 32
I2C_JOINT_COUNT = 6
I2C_PROTOCOL_MAGIC = 0xA5
I2C_PROTOCOL_VERSION = 3
I2C_READ_ATTEMPTS = 3
I2C_READ_RETRY_DELAY_SEC = 0.005
I2C_READ_RETRY_ERRNOS = (errno.EAGAIN, errno.EREMOTEIO)

FW_LIFECYCLE_READY = 0
FW_LIFECYCLE_ACCEPTED = 1
FW_LIFECYCLE_EXECUTING = 2
FW_LIFECYCLE_COMPLETED = 3
FW_LIFECYCLE_FAILED = 4
FW_LIFECYCLE_STOPPING = 5

FW_ERROR_NONE = 0
FW_ERROR_BAD_PROTOCOL = 1
FW_ERROR_BAD_COMMAND = 2
FW_ERROR_ARM_NOT_READY = 3
FW_ERROR_NO_IK_SOLUTION = 4
FW_ERROR_SERVO_WRITE_FAILED = 5
FW_ERROR_SERVO_FEEDBACK_FAILED = 6
FW_ERROR_MOTION_TIMEOUT = 7
FW_ERROR_STREAM_STEP_TOO_LARGE = 8
FW_ERROR_STREAM_TIMEOUT = 9
FW_ERROR_SERVO_DEADLINE_MISSED = 10

# I2C command byte tags (must match STM32 firmware; contract sec 2.3)
TAG_ARM = ord('A')
TAG_ARM_STREAM = ord('T')
TAG_ARM_STREAM_END = ord('F')
TAG_DIRECT_SERVO = ord('U')
TAG_DIRECT_SERVO_END = ord('G')
TAG_SERVO_STOP = ord('H')
TAG_SERVO = ord('P')
TAG_STOP = ord('S')

# Stable error codes (contract sec 3.3: error_code is the machine interface)
ERR_I2C_LOST = 0x0001
ERR_ESTOP_LATCHED = 0x0002
ERR_JOINT_DISABLED = 0x0003
ERR_UNKNOWN_MODE = 0x0004
ERR_NONFINITE_FIELD = 0x0010
ERR_DURATION_FINITE = 0x0011
ERR_DURATION_RANGE = 0x0012
ERR_GRIPPER_NONFINITE = 0x0013
ERR_GRIPPER_RANGE = 0x0014
ERR_GRIPPER_UNMAPPED = 0x0015

# Command lifecycle (contract sec 5.3 / command_timeout_sec)
ERR_CMD_TIMEOUT = 0x0016
ERR_STALE_CMD = 0x0017
ERR_FW_PROTOCOL = 0x0018
ERR_FW_RESTARTED = 0x0019
ERR_GRIPPER_STOP_WRITE = 0x001A
ERR_WRIST_ROLL_RANGE = 0x001B
ERR_WRIST_ROLL_UNMAPPED = 0x001C

# Firmware-reported errors (distinct from node-side validation errors).
ERR_FW_NO_SOLVE = 0x0020
ERR_FW_NOT_READY = 0x0021
ERR_FW_BAD_CMD = 0x0022
ERR_FW_SERVO_WRITE = 0x0023
ERR_FW_SERVO_FEEDBACK = 0x0024
ERR_FW_MOTION_TIMEOUT = 0x0025
ERR_FW_STREAM_STEP_TOO_LARGE = 0x0026
ERR_FW_STREAM_TIMEOUT = 0x0027
ERR_FW_SERVO_DEADLINE = 0x0028


def _decode_firmware_packet(data):
    """Return (lifecycle, error, wire_id) for a valid v3 status packet."""
    raw = bytes(data)
    if (len(raw) != STATUS_PACKET_SIZE
            or raw[0] != I2C_PROTOCOL_MAGIC
            or raw[1] != I2C_PROTOCOL_VERSION
            or raw[2] > FW_LIFECYCLE_STOPPING
            or raw[3] > FW_ERROR_SERVO_DEADLINE_MISSED):
        return None
    return raw[2], raw[3], struct.unpack_from('<I', raw, 4)[0]


def _firmware_error_details(error):
    details = {
        FW_ERROR_BAD_PROTOCOL: (
            ERR_FW_PROTOCOL, 'firmware: bad I2C protocol version'),
        FW_ERROR_BAD_COMMAND: (
            ERR_FW_BAD_CMD, 'firmware: invalid command or payload'),
        FW_ERROR_ARM_NOT_READY: (
            ERR_FW_NOT_READY, 'firmware: arm is not ready'),
        FW_ERROR_NO_IK_SOLUTION: (
            ERR_FW_NO_SOLVE, 'firmware: inverse kinematics has no solution'),
        FW_ERROR_SERVO_WRITE_FAILED: (
            ERR_FW_SERVO_WRITE, 'firmware: servo UART write failed'),
        FW_ERROR_SERVO_FEEDBACK_FAILED: (
            ERR_FW_SERVO_FEEDBACK, 'firmware: target servo feedback failed'),
        FW_ERROR_MOTION_TIMEOUT: (
            ERR_FW_MOTION_TIMEOUT, 'firmware: motion timed out'),
        FW_ERROR_STREAM_STEP_TOO_LARGE: (
            ERR_FW_STREAM_STEP_TOO_LARGE,
            'firmware: stream target exceeds the 12 degree joint step'),
        FW_ERROR_STREAM_TIMEOUT: (
            ERR_FW_STREAM_TIMEOUT,
            'firmware: servo stream timed out and is braking'),
        FW_ERROR_SERVO_DEADLINE_MISSED: (
            ERR_FW_SERVO_DEADLINE,
            'firmware: 40 ms servo deadline was missed'),
    }
    return details.get(
        error, (ERR_FW_BAD_CMD, 'firmware: unknown error %d' % error))


def _legacy_status_text(lifecycle, error):
    if lifecycle == FW_LIFECYCLE_READY:
        return 'ARM_RDY_'
    if lifecycle in (FW_LIFECYCLE_ACCEPTED, FW_LIFECYCLE_EXECUTING):
        return 'ARM_OK__'
    if lifecycle == FW_LIFECYCLE_COMPLETED:
        return 'ARM_DONE'
    if error == FW_ERROR_NO_IK_SOLUTION:
        return 'NO_SOLVE'
    return 'ARM_ERR_'


def _motion_mode_tag(mode):
    tags = {
        ArmCommand.MODE_END_EFFECTOR: 'A',
        ArmCommand.MODE_CARTESIAN_SERVO: 'T',
        ArmCommand.MODE_CARTESIAN_SERVO_END: 'F',
        ArmCommand.MODE_GRIPPER: 'P',
        ArmCommand.MODE_GRIPPER_STOP: 'H',
        ArmCommand.MODE_WRIST_ROLL: 'P(wrist)',
        ArmCommand.MODE_GRIPPER_SERVO: 'U(gripper)',
        ArmCommand.MODE_GRIPPER_SERVO_END: 'G(gripper)',
        ArmCommand.MODE_WRIST_ROLL_SERVO: 'U(wrist)',
        ArmCommand.MODE_WRIST_ROLL_SERVO_END: 'G(wrist)',
    }
    return tags.get(mode, '?')


# I2C failures before we latch into STATE_ERROR (contract sec 5.5)
I2C_FAIL_THRESHOLD = 5


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('arm_controller_node')

        self._declare_config_parameters()
        self._init_i2c()

        # --- runtime state (contract sec 3.3) ---
        self._state = ArmState.STATE_IDLE
        self._command_phase = ArmState.PHASE_NONE
        self._last_sequence_id = 0
        self._error_code = 0
        self._error_message = ''
        self._position_valid = False
        self._last_position_valid = False
        self._joint_position = [0.0] * 5
        self._servo_raw_positions = [0.0] * I2C_JOINT_COUNT  # servo raw (0..1000)
        self._gripper_position = 0.0

        # --- emergency stop (contract sec 3.2 / 5.4): latched ---
        self._estop_request = False
        self._estop_latched = False

        # --- I2C failure tracking (contract sec 5.5) ---
        self._i2c_fail_count = 0
        self._i2c_failure_total = 0
        self._i2c_read_retry_total = 0

        # --- firmware protocol / restart detection ---
        self._firmware_protocol_ok = False
        self._firmware_seen_nonzero_id = False
        self._firmware_restart_latched = False
        self._last_logged_raw = ''

        # --- command lifecycle / watchdog (contract sec 5.3) ---
        self._pending_motion = False      # a motion command is awaiting firmware ack
        self._pending_seq = 0
        self._pending_wire_id = 0
        self._pending_sent_ns = 0
        self._active_wire_id = 0
        self._active_seq = 0
        self._active_mode = None
        self._active_sent_ns = 0
        self._active_expected_duration_sec = 0.0
        self._wire_id_counter = 0
        self._queued_command = None
        self._queued_stream_end = None
        self._stream_open = False
        self._stream_open_family = None
        self._last_fw_contact_ns = 0      # any successfully read status packet
        # Highest nonzero sequence_id accepted, used for stale/duplicate checks.
        self._last_applied_seq = 0

        # --- publishers ---
        self.state_pub = self.create_publisher(ArmState, '/arm/state', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.legacy_status_pub = self.create_publisher(String, '/status_topic', 10)

        # --- subscriptions ---
        self.cmd_sub = self.create_subscription(
            ArmCommand, '/arm/command', self.command_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, '/arm/emergency_stop', self.emergency_stop_callback, 10)
        self.legacy_sub = self.create_subscription(
            String, '/command_topic', self.legacy_command_callback, 10)

        # --- service ---
        self.reset_srv = self.create_service(
            Trigger, '/arm/reset_error', self.reset_error_callback)

        # --- timers ---
        self.state_timer = self.create_timer(0.1, self.publish_state)
        self.status_timer = self.create_timer(0.1, self.poll_status)

        self.get_logger().info('arm_controller_node started; I2C ok=%s' % self.i2c_ok)

    # ===================== config (contract sec 4) =====================
    def _declare_config_parameters(self):
        defaults = {
            'i2c_bus': 5,
            'i2c_address': 0x30,
            'command_timeout_sec': 1.0,
            'min_duration_sec': 0.05,
            'max_duration_sec': 30.0,
            'stream_watchdog_min_sec': 0.10,
            'stream_watchdog_max_sec': 1.00,
            'gripper_stream_max_velocity_raw_sec': 300.0,
            'gripper_stream_max_acceleration_raw_sec2': 1200.0,
            'wrist_stream_max_velocity_deg_sec': 60.0,
            'wrist_stream_max_acceleration_deg_sec2': 240.0,
            'joint_names': [
                'joint_1_base', 'joint_2_shoulder', 'joint_3_elbow',
                'joint_4_wrist_pitch', 'joint_5_wrist_roll',
            ],
            'servo_id_map': [
                'joint_1_base:6', 'joint_2_shoulder:5', 'joint_3_elbow:4',
                'joint_4_wrist_pitch:3', 'joint_5_wrist_roll:2', 'gripper:1',
            ],
            'joint_zero_offsets': [0.0, math.pi / 2.0, 0.0, 0.0, 0.0],
            'joint_directions': [1, -1, 1, 1, -1],
            'joint_lower_limits': [
                -3.14159, -3.14159, -3.14159, -3.14159, -math.pi / 2.0],
            'joint_upper_limits': [
                3.14159, 3.14159, 3.14159, 3.14159, math.pi / 2.0],
            'gripper_closed_raw': 700.0,
            'gripper_open_raw': 200.0,
            'pitch_min_deg': -90.0,
            'pitch_max_deg': 90.0,
            'end_effector_frame': 'base',
            'end_effector_units': 'cm',
            'joint_feedback_enabled': True,
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)

    def _cfg(self, name):
        return self.get_parameter(name).value

    # ===================== I2C =====================
    def _init_i2c(self):
        self.bus = None
        self.i2c_ok = False
        if SMBus is None:
            self.get_logger().error(
                'smbus2 not available; running WITHOUT I2C '
                '(install: pip install smbus2)')
            return
        try:
            self.bus = SMBus(self._cfg('i2c_bus'))
            self.i2c_ok = True
        except Exception as e:
            self.get_logger().error('I2C init failed: %s' % e)

    def _i2c_write(self, data_bytes):
        if self.bus is None:
            return False
        last_err = None
        # Transient I2C NAKs happen when the firmware is mid-transaction or the
        # bus is briefly busy; a short retry makes a single dropped write rare
        # instead of an intermittent "command did nothing" failure.
        for attempt in range(3):
            try:
                msg = i2c_msg.write(self._cfg('i2c_address'), list(data_bytes))
                self.bus.i2c_rdwr(msg)
                self._i2c_fail_count = 0
                self.i2c_ok = True
                return True
            except Exception as e:
                last_err = e
                time.sleep(0.01)
        self._on_i2c_failure('write', last_err)
        return False

    def _i2c_read_status(self):
        if self.bus is None:
            return None
        last_err = None
        for attempt in range(I2C_READ_ATTEMPTS):
            try:
                msg = i2c_msg.read(
                    self._cfg('i2c_address'), STATUS_PACKET_SIZE)
                self.bus.i2c_rdwr(msg)
                if attempt:
                    self.get_logger().warn(
                        'I2C read recovered after %d retry '
                        '(retry_total=%d)'
                        % (attempt, self._i2c_read_retry_total))
                self._i2c_fail_count = 0
                self.i2c_ok = True
                # Communication restored: clear a stale latched I2C-lost error
                # so the typed state machine reflects live firmware status.
                if self._error_code == ERR_I2C_LOST:
                    self._error_code = 0
                    self._error_message = ''
                    if self._state == ArmState.STATE_ERROR:
                        self._state = ArmState.STATE_IDLE
                return list(msg)
            except Exception as e:
                last_err = e
                retryable = (
                    isinstance(e, OSError)
                    and e.errno in I2C_READ_RETRY_ERRNOS)
                if not retryable or attempt + 1 >= I2C_READ_ATTEMPTS:
                    break
                self._i2c_read_retry_total += 1
                time.sleep(I2C_READ_RETRY_DELAY_SEC)
        self._on_i2c_failure('read', last_err)
        return None

    def _on_i2c_failure(self, op, exc):
        self._i2c_fail_count += 1
        self._i2c_failure_total += 1
        self.i2c_ok = False
        message = ('I2C %s failed (seq=%d, consecutive=%d, total=%d): %s'
                   % (op, self._last_sequence_id,
                      self._i2c_fail_count, self._i2c_failure_total, exc))
        if self._i2c_fail_count < I2C_FAIL_THRESHOLD:
            self.get_logger().warn(message)
            return

        # Link liveness is independent from command acknowledgement. A command
        # can time out while reads still work, while repeated read/write errors
        # stop all motion through this separate threshold.
        self._position_valid = False
        self._clear_active_motion()
        self._stream_open = False
        self._stream_open_family = None
        self._set_error(ERR_I2C_LOST, message)

    # ===================== command entry points =====================
    def command_callback(self, msg):
        self.handle_command(msg)

    def handle_command(self, cmd):
        seq = cmd.sequence_id

        # Emergency-stop latch rejects motion (contract sec 3.2 / 5.4).
        if self._estop_latched and cmd.mode != ArmCommand.MODE_STOP:
            self._set_error(ERR_ESTOP_LATCHED,
                            'estop latched; motion rejected (seq=%d)' % seq)
            return

        # Stale / duplicate / out-of-order command rejection (contract sec 5.3).
        # sequence_id == 0 is the legacy compatibility sentinel: no tracking.
        if seq > 0 and seq <= self._last_applied_seq:
            self._set_error(ERR_STALE_CMD,
                            'stale or duplicate sequence_id %d (last applied %d)'
                            % (seq, self._last_applied_seq))
            return

        if cmd.mode == ArmCommand.MODE_STOP:
            if seq > 0:
                self._last_applied_seq = seq
            self._send_stop(seq)
            return

        if not self._firmware_protocol_ok:
            self._set_error(
                ERR_FW_PROTOCOL,
                'firmware I2C v3 status not verified; motion rejected (seq=%d)'
                % seq)
            return

        if cmd.mode == ArmCommand.MODE_JOINT:
            # Mapping unconfirmed -> disabled (contract sec 4 note).
            self._set_error(ERR_JOINT_DISABLED,
                            'MODE_JOINT disabled until servo mapping confirmed (seq=%d)' % seq)
            return

        if cmd.mode == ArmCommand.MODE_END_EFFECTOR:
            if not self._validate_end_effector(cmd, seq):
                return
            self._accept_motion(cmd)
            return

        if cmd.mode == ArmCommand.MODE_CARTESIAN_SERVO:
            if not self._validate_cartesian_servo(cmd, seq):
                return
            self._accept_motion(cmd)
            return

        if cmd.mode == ArmCommand.MODE_CARTESIAN_SERVO_END:
            self._accept_motion(cmd)
            return

        if cmd.mode == ArmCommand.MODE_GRIPPER_SERVO:
            if (not self._validate_gripper(cmd, seq)
                    or not self._validate_stream_watchdog(cmd, seq)):
                return
            self._accept_motion(cmd)
            return

        if cmd.mode == ArmCommand.MODE_WRIST_ROLL_SERVO:
            if (not self._validate_wrist_roll_target(cmd, seq)
                    or not self._validate_stream_watchdog(cmd, seq)):
                return
            self._accept_motion(cmd)
            return

        if cmd.mode in (
                ArmCommand.MODE_GRIPPER_SERVO_END,
                ArmCommand.MODE_WRIST_ROLL_SERVO_END):
            self._accept_motion(cmd)
            return

        if cmd.mode == ArmCommand.MODE_GRIPPER:
            if not self._validate_gripper(cmd, seq):
                return
            self._accept_motion(cmd)
            return

        if cmd.mode == ArmCommand.MODE_WRIST_ROLL:
            if not self._validate_wrist_roll(cmd, seq):
                return
            self._accept_motion(cmd)
            return

        if cmd.mode == ArmCommand.MODE_GRIPPER_STOP:
            self._accept_gripper_stop(cmd)
            return

        self._set_error(ERR_UNKNOWN_MODE, 'unknown mode %d (seq=%d)' % (cmd.mode, seq))

    @staticmethod
    def _stream_family(mode):
        families = {
            ArmCommand.MODE_CARTESIAN_SERVO: 'cartesian',
            ArmCommand.MODE_CARTESIAN_SERVO_END: 'cartesian',
            ArmCommand.MODE_GRIPPER_SERVO: 'gripper',
            ArmCommand.MODE_GRIPPER_SERVO_END: 'gripper',
            ArmCommand.MODE_WRIST_ROLL_SERVO: 'wrist',
            ArmCommand.MODE_WRIST_ROLL_SERVO_END: 'wrist',
        }
        return families.get(mode)

    @classmethod
    def _is_stream_mode(cls, mode):
        return cls._stream_family(mode) is not None

    @staticmethod
    def _is_stream_target(mode):
        return mode in (
            ArmCommand.MODE_CARTESIAN_SERVO,
            ArmCommand.MODE_GRIPPER_SERVO,
            ArmCommand.MODE_WRIST_ROLL_SERVO,
        )

    @staticmethod
    def _is_stream_end(mode):
        return mode in (
            ArmCommand.MODE_CARTESIAN_SERVO_END,
            ArmCommand.MODE_GRIPPER_SERVO_END,
            ArmCommand.MODE_WRIST_ROLL_SERVO_END,
        )

    @classmethod
    def _modes_compatible(cls, active_mode, requested_mode):
        return (active_mode == requested_mode
                or (cls._stream_family(active_mode) is not None
                    and cls._stream_family(active_mode)
                    == cls._stream_family(requested_mode)))

    def _accept_motion(self, cmd):
        seq = int(cmd.sequence_id)
        if seq > 0:
            self._last_applied_seq = seq
        family = self._stream_family(cmd.mode)
        if self._is_stream_target(cmd.mode):
            if (self._queued_stream_end is not None
                    and self._stream_family(self._queued_stream_end.mode)
                    == family):
                self._queued_stream_end = None
        elif self._is_stream_end(cmd.mode):
            stream_expected = (
                (self._stream_open and self._stream_open_family == family)
                or self._stream_family(self._active_mode) == family
                or (self._queued_command is not None
                    and self._is_stream_target(self._queued_command.mode)
                    and self._stream_family(self._queued_command.mode)
                    == family)
            )
            if not stream_expected:
                self._last_sequence_id = seq
                self._set_recoverable_rejection(
                    ERR_FW_BAD_CMD,
                    'stream END rejected without an active stream '
                    '(seq=%d)' % seq)
                return
            if (self._pending_motion
                    or (self._queued_command is not None
                        and self._is_stream_target(
                            self._queued_command.mode)
                        and self._stream_family(
                            self._queued_command.mode) == family)):
                self._queued_stream_end = copy.deepcopy(cmd)
                return
        blocked_by_ack = self._pending_motion
        blocked_by_mode = (
            self._active_wire_id != 0
            and not self._modes_compatible(self._active_mode, cmd.mode))
        if blocked_by_ack or blocked_by_mode:
            previous = self._queued_command
            self._queued_command = copy.deepcopy(cmd)
            reason = 'awaiting_ack' if blocked_by_ack else 'active_mode'
            if previous is None:
                self.get_logger().debug(
                    '[motion queue] queued mode=%s seq=%d reason=%s'
                    % (_motion_mode_tag(cmd.mode), seq, reason))
            else:
                self.get_logger().debug(
                    '[motion queue] replaced mode=%s seq=%d with mode=%s '
                    'seq=%d reason=%s'
                    % (_motion_mode_tag(previous.mode),
                       int(previous.sequence_id),
                       _motion_mode_tag(cmd.mode), seq, reason))
            return

        # A command that can run now is the newest operator intent. Do not let
        # an older cross-mode command survive and replay after this one finishes.
        if self._queued_command is not None:
            previous = self._queued_command
            self._queued_command = None
            self.get_logger().debug(
                '[motion queue] canceled mode=%s seq=%d by mode=%s seq=%d'
                % (_motion_mode_tag(previous.mode),
                   int(previous.sequence_id),
                   _motion_mode_tag(cmd.mode), seq))
        self._dispatch_motion(cmd)

    def _accept_gripper_stop(self, cmd):
        # A gripper halt must preempt an in-flight gripper position command;
        # queueing it behind that unreachable target defeats the halt. Never
        # abandon an active ARM lifecycle, because the firmware exposes only
        # one current wire id and the arm may still be moving physically.
        if (self._active_wire_id != 0
                and self._active_mode in (
                    ArmCommand.MODE_END_EFFECTOR,
                    ArmCommand.MODE_CARTESIAN_SERVO,
                    ArmCommand.MODE_CARTESIAN_SERVO_END)):
            self._accept_motion(cmd)
            return

        seq = int(cmd.sequence_id)
        if seq > 0:
            self._last_applied_seq = seq
        self._queued_command = None
        self._queued_stream_end = None
        if (self._stream_open_family == 'gripper'
                or self._stream_family(self._active_mode) == 'gripper'):
            # H invalidates an active U/G stream immediately.  Do not leave a
            # stale gripper family that can make a later END look valid.
            self._stream_open = False
            self._stream_open_family = None
        if self._dispatch_motion(cmd):
            return

        # The halt is safety-critical: after all bounded I2C retries fail,
        # never leave teleop waiting for an acknowledgement that cannot exist.
        self._clear_active_motion()
        self._last_sequence_id = seq
        if self._error_code != ERR_GRIPPER_UNMAPPED:
            self._set_error(
                ERR_GRIPPER_STOP_WRITE,
                'gripper stop I2C write failed after retries (seq=%d)' % seq)

    def _dispatch_motion(self, cmd):
        wire_id = self._take_wire_id()
        if cmd.mode == ArmCommand.MODE_END_EFFECTOR:
            sent = self._send_end_effector(cmd, wire_id)
        elif cmd.mode == ArmCommand.MODE_CARTESIAN_SERVO:
            sent = self._send_cartesian_servo(cmd, wire_id)
        elif cmd.mode == ArmCommand.MODE_CARTESIAN_SERVO_END:
            sent = self._send_cartesian_servo_end(wire_id)
        elif cmd.mode == ArmCommand.MODE_GRIPPER_SERVO:
            sent = self._send_gripper_servo(cmd, wire_id)
        elif cmd.mode == ArmCommand.MODE_WRIST_ROLL_SERVO:
            sent = self._send_wrist_roll_servo(cmd, wire_id)
        elif cmd.mode in (
                ArmCommand.MODE_GRIPPER_SERVO_END,
                ArmCommand.MODE_WRIST_ROLL_SERVO_END):
            sent = self._send_direct_servo_end(cmd, wire_id)
        elif cmd.mode == ArmCommand.MODE_GRIPPER_STOP:
            sent = self._send_gripper_stop(cmd, wire_id)
        elif cmd.mode == ArmCommand.MODE_WRIST_ROLL:
            sent = self._send_wrist_roll(cmd, wire_id)
        else:
            sent = self._send_gripper(cmd, wire_id)
        if not sent:
            return False
        seq = int(cmd.sequence_id)
        self._active_wire_id = wire_id
        self._active_seq = seq
        self._active_mode = cmd.mode
        self._active_sent_ns = self.get_clock().now().nanoseconds
        duration_sec = float(cmd.duration_sec)
        if cmd.mode == ArmCommand.MODE_GRIPPER and (
                not math.isfinite(duration_sec) or duration_sec <= 0.0):
            # _send_gripper uses the same one-second fallback on the wire.
            duration_sec = 1.0
        self._active_expected_duration_sec = (
            duration_sec
            if math.isfinite(duration_sec) and duration_sec > 0.0
            else 0.0)
        self._set_state(
            ArmState.STATE_MOVING, seq, ArmState.PHASE_NONE)
        self._arm_pending_motion(seq, wire_id)
        self.get_logger().debug(
            '[motion dispatch] mode=%s seq=%d wire_id=%d'
            % (_motion_mode_tag(cmd.mode), seq, wire_id))
        return True

    def _flush_queued_command(self):
        if self._pending_motion:
            return
        if self._queued_command is not None:
            if (self._active_wire_id != 0
                    and not self._modes_compatible(
                        self._active_mode, self._queued_command.mode)):
                if (self._stream_open
                        and self._queued_stream_end is not None
                        and self._stream_family(
                            self._queued_stream_end.mode)
                        == self._stream_open_family):
                    cmd = self._queued_stream_end
                    self._queued_stream_end = None
                    self._dispatch_motion(cmd)
                return
            cmd = self._queued_command
            self._queued_command = None
            self._dispatch_motion(cmd)
            return
        if self._queued_stream_end is not None:
            if (not self._stream_open
                    or self._stream_family(self._queued_stream_end.mode)
                    != self._stream_open_family):
                self._queued_stream_end = None
                return
            cmd = self._queued_stream_end
            self._queued_stream_end = None
            self._dispatch_motion(cmd)

    def _clear_active_motion(self, clear_queue=True):
        self._pending_motion = False
        self._pending_seq = 0
        self._pending_wire_id = 0
        self._pending_sent_ns = 0
        self._active_wire_id = 0
        self._active_seq = 0
        self._active_mode = None
        self._active_sent_ns = 0
        self._active_expected_duration_sec = 0.0
        if clear_queue:
            self._queued_command = None
            self._queued_stream_end = None

    def _take_wire_id(self):
        self._wire_id_counter = (self._wire_id_counter + 1) & 0xFFFFFFFF
        if self._wire_id_counter == 0:
            self._wire_id_counter = 1
        return self._wire_id_counter

    # ===================== validation (contract sec 5.3) =====================
    def _validate_end_effector(self, cmd, seq):
        for field in ('x', 'y', 'z', 'pitch'):
            val = getattr(cmd, field)
            if not math.isfinite(val):
                self._set_error(ERR_NONFINITE_FIELD,
                                'non-finite %s (seq=%d)' % (field, seq))
                return False
        if not (math.isfinite(cmd.duration_sec) and cmd.duration_sec > 0.0):
            self._set_error(ERR_DURATION_FINITE,
                            'duration_sec must be finite positive (seq=%d)' % seq)
            return False
        lo = self._cfg('min_duration_sec')
        hi = self._cfg('max_duration_sec')
        if not (lo <= cmd.duration_sec <= hi):
            self._set_error(ERR_DURATION_RANGE,
                            'duration_sec %s outside [%s,%s] (seq=%d)'
                            % (cmd.duration_sec, lo, hi, seq))
            return False
        return True

    def _validate_cartesian_servo(self, cmd, seq):
        if not self._validate_end_effector(cmd, seq):
            return False
        return self._validate_stream_watchdog(cmd, seq)

    def _validate_stream_watchdog(self, cmd, seq):
        if not (math.isfinite(cmd.duration_sec) and cmd.duration_sec > 0.0):
            self._set_error(
                ERR_DURATION_FINITE,
                'stream watchdog must be finite positive (seq=%d)' % seq)
            return False
        lo = self._cfg('stream_watchdog_min_sec')
        hi = self._cfg('stream_watchdog_max_sec')
        if not (lo <= cmd.duration_sec <= hi):
            self._set_error(
                ERR_DURATION_RANGE,
                'stream watchdog %.3fs outside [%.3f,%.3f] (seq=%d)'
                % (cmd.duration_sec, lo, hi, seq))
            return False
        return True

    def _validate_gripper(self, cmd, seq):
        if not math.isfinite(cmd.gripper_position):
            self._set_error(ERR_GRIPPER_NONFINITE,
                            'non-finite gripper_position (seq=%d)' % seq)
            return False
        if not (0.0 <= cmd.gripper_position <= 1.0):
            self._set_error(ERR_GRIPPER_RANGE,
                            'gripper_position %s outside [0,1] (seq=%d)'
                            % (cmd.gripper_position, seq))
            return False
        return True

    def _validate_wrist_roll_target(self, cmd, seq):
        index = self._joint_index('joint_5_wrist_roll')
        if index is None or self._servo_id_for('joint_5_wrist_roll') is None:
            self._set_error(
                ERR_WRIST_ROLL_UNMAPPED,
                'wrist roll joint or servo mapping is missing (seq=%d)' % seq)
            return False
        target = cmd.joint_position[index]
        if not math.isfinite(target):
            self._set_error(
                ERR_NONFINITE_FIELD,
                'non-finite wrist roll target (seq=%d)' % seq)
            return False
        lower = self._cfg('joint_lower_limits')
        upper = self._cfg('joint_upper_limits')
        if index >= len(lower) or index >= len(upper):
            self._set_error(
                ERR_WRIST_ROLL_UNMAPPED,
                'wrist roll limits are missing (seq=%d)' % seq)
            return False
        limit_epsilon = 1e-6
        if (target < lower[index] - limit_epsilon
                or target > upper[index] + limit_epsilon):
            self._set_error(
                ERR_WRIST_ROLL_RANGE,
                'wrist roll %.4f rad outside [%.4f,%.4f] (seq=%d)'
                % (target, lower[index], upper[index], seq))
            return False
        return True

    def _validate_wrist_roll(self, cmd, seq):
        if not self._validate_wrist_roll_target(cmd, seq):
            return False
        if not (math.isfinite(cmd.duration_sec) and cmd.duration_sec > 0.0):
            self._set_error(
                ERR_DURATION_FINITE,
                'duration_sec must be finite positive (seq=%d)' % seq)
            return False
        lo = self._cfg('min_duration_sec')
        hi = self._cfg('max_duration_sec')
        if not lo <= cmd.duration_sec <= hi:
            self._set_error(
                ERR_DURATION_RANGE,
                'duration_sec %s outside [%s,%s] (seq=%d)'
                % (cmd.duration_sec, lo, hi, seq))
            return False
        return True

    # ===================== I2C packet builders (sec 2.3 layout) =====================
    def _send_stop(self, seq):
        wire_id = self._take_wire_id()
        buf = bytearray(CMD_PACKET_SIZE)
        self._pack_command_header(buf, TAG_STOP, wire_id, 0)
        self._i2c_write(buf)
        self._clear_active_motion()
        self._stream_open = False
        self._stream_open_family = None
        # A STOP command must never make a latched emergency stop look cleared.
        state = ArmState.STATE_ESTOP if self._estop_latched else ArmState.STATE_IDLE
        self._set_state(state, seq, ArmState.PHASE_NONE)

    def _pack_command_header(self, buf, tag, wire_id, duration_ms):
        buf[0] = tag
        buf[1] = I2C_PROTOCOL_VERSION
        struct.pack_into('<I', buf, 2, wire_id)
        struct.pack_into('<H', buf, 6, duration_ms)

    def _duration_ms(self, duration_sec):
        return max(1, min(30000, int(round(duration_sec * 1000.0))))

    def _send_end_effector(self, cmd, wire_id):
        return self._send_cartesian(cmd, wire_id, TAG_ARM)

    def _send_cartesian_servo(self, cmd, wire_id):
        return self._send_cartesian(cmd, wire_id, TAG_ARM_STREAM)

    def _send_cartesian_servo_end(self, wire_id):
        buf = bytearray(CMD_PACKET_SIZE)
        self._pack_command_header(buf, TAG_ARM_STREAM_END, wire_id, 0)
        return self._i2c_write(buf)

    def _send_cartesian(self, cmd, wire_id, tag):
        buf = bytearray(CMD_PACKET_SIZE)
        self._pack_command_header(
            buf, tag, wire_id, self._duration_ms(cmd.duration_sec))
        struct.pack_into('<f', buf, 8, cmd.x)
        struct.pack_into('<f', buf, 12, cmd.y)
        struct.pack_into('<f', buf, 16, cmd.z)
        struct.pack_into('<f', buf, 20, cmd.pitch)
        # min/max pitch form the IK's allowed end-effector roll window
        # (firmware calls set_pitch_range(min,max) inside robot_arm_coordinate_set).
        # They are NOT part of ArmCommand, so we take them from config. The
        # reference firmware examples use [-90, 90]; do NOT set both to cmd.pitch
        # (i.e. a degenerate [0,0] window) or the IK is over-constrained and
        # returns NO_SOLVE even for reachable points.
        struct.pack_into('<f', buf, 24, self._cfg('pitch_min_deg'))
        struct.pack_into('<f', buf, 28, self._cfg('pitch_max_deg'))
        return self._i2c_write(buf)

    def _send_gripper(self, cmd, wire_id):
        sid = self._servo_id_for('gripper')
        if sid is None:
            self._set_error(ERR_GRIPPER_UNMAPPED,
                            'gripper servo id not in servo_id_map (seq=%d)'
                            % cmd.sequence_id)
            return False
        opened = float(self._cfg('gripper_open_raw'))
        closed = float(self._cfg('gripper_closed_raw'))
        raw = opened + cmd.gripper_position * (closed - opened)
        buf = bytearray(CMD_PACKET_SIZE)
        duration = (cmd.duration_sec
                    if math.isfinite(cmd.duration_sec) and cmd.duration_sec > 0
                    else 1.0)
        self._pack_command_header(
            buf, TAG_SERVO, wire_id, self._duration_ms(duration))
        buf[8] = sid & 0xFF
        struct.pack_into('<f', buf, 12, raw)
        return self._i2c_write(buf)

    def _send_gripper_servo(self, cmd, wire_id):
        sid = self._servo_id_for('gripper')
        if sid is None:
            self._set_error(ERR_GRIPPER_UNMAPPED,
                            'gripper servo id not in servo_id_map (seq=%d)'
                            % cmd.sequence_id)
            return False
        opened = float(self._cfg('gripper_open_raw'))
        closed = float(self._cfg('gripper_closed_raw'))
        raw = opened + cmd.gripper_position * (closed - opened)
        return self._send_direct_servo(
            sid, raw,
            float(self._cfg('gripper_stream_max_velocity_raw_sec')),
            float(self._cfg('gripper_stream_max_acceleration_raw_sec2')),
            cmd.duration_sec, wire_id, cmd.sequence_id)

    def _send_wrist_roll(self, cmd, wire_id):
        joint_name = 'joint_5_wrist_roll'
        index = self._joint_index(joint_name)
        sid = self._servo_id_for(joint_name)
        if index is None or sid is None:
            self._set_error(
                ERR_WRIST_ROLL_UNMAPPED,
                'wrist roll joint or servo mapping is missing (seq=%d)'
                % cmd.sequence_id)
            return False
        zeros = self._cfg('joint_zero_offsets')
        directions = self._cfg('joint_directions')
        if index >= len(zeros) or index >= len(directions):
            self._set_error(
                ERR_WRIST_ROLL_UNMAPPED,
                'wrist roll calibration is missing (seq=%d)'
                % cmd.sequence_id)
            return False
        direction = float(directions[index])
        if direction == 0.0:
            self._set_error(
                ERR_WRIST_ROLL_UNMAPPED,
                'wrist roll direction cannot be zero (seq=%d)'
                % cmd.sequence_id)
            return False
        rad_per_raw = math.radians(240.0) / 1000.0
        raw = (500.0 + (cmd.joint_position[index] - zeros[index])
               / (direction * rad_per_raw))
        if not 0.0 <= raw <= 1000.0:
            self._set_error(
                ERR_WRIST_ROLL_RANGE,
                'wrist roll target maps outside servo raw range (seq=%d)'
                % cmd.sequence_id)
            return False
        buf = bytearray(CMD_PACKET_SIZE)
        self._pack_command_header(
            buf, TAG_SERVO, wire_id, self._duration_ms(cmd.duration_sec))
        buf[8] = sid & 0xFF
        struct.pack_into('<f', buf, 12, raw)
        return self._i2c_write(buf)

    def _send_wrist_roll_servo(self, cmd, wire_id):
        joint_name = 'joint_5_wrist_roll'
        index = self._joint_index(joint_name)
        sid = self._servo_id_for(joint_name)
        if index is None or sid is None:
            self._set_error(
                ERR_WRIST_ROLL_UNMAPPED,
                'wrist roll joint or servo mapping is missing (seq=%d)'
                % cmd.sequence_id)
            return False
        zeros = self._cfg('joint_zero_offsets')
        directions = self._cfg('joint_directions')
        direction = float(directions[index])
        rad_per_raw = math.radians(240.0) / 1000.0
        raw = (500.0 + (cmd.joint_position[index] - zeros[index])
               / (direction * rad_per_raw))
        raw_per_degree = 1000.0 / 240.0
        return self._send_direct_servo(
            sid, raw,
            float(self._cfg('wrist_stream_max_velocity_deg_sec'))
            * raw_per_degree,
            float(self._cfg('wrist_stream_max_acceleration_deg_sec2'))
            * raw_per_degree,
            cmd.duration_sec, wire_id, cmd.sequence_id)

    def _send_direct_servo(self, sid, raw, max_velocity,
                           max_acceleration, watchdog_sec,
                           wire_id, sequence_id):
        if (sid not in (1, 2) or not math.isfinite(raw)
                or not 0.0 <= raw <= 1000.0
                or not math.isfinite(max_velocity) or max_velocity <= 0.0
                or not math.isfinite(max_acceleration)
                or max_acceleration <= 0.0):
            self._set_error(
                ERR_FW_BAD_CMD,
                'invalid direct servo mapping or limits (seq=%d)'
                % sequence_id)
            return False
        buf = bytearray(CMD_PACKET_SIZE)
        self._pack_command_header(
            buf, TAG_DIRECT_SERVO, wire_id,
            self._duration_ms(watchdog_sec))
        buf[8] = sid & 0xFF
        struct.pack_into('<f', buf, 12, raw)
        struct.pack_into('<f', buf, 16, max_velocity)
        struct.pack_into('<f', buf, 20, max_acceleration)
        return self._i2c_write(buf)

    def _send_direct_servo_end(self, cmd, wire_id):
        joint_name = ('gripper'
                      if cmd.mode == ArmCommand.MODE_GRIPPER_SERVO_END
                      else 'joint_5_wrist_roll')
        sid = self._servo_id_for(joint_name)
        if sid not in (1, 2):
            code = (ERR_GRIPPER_UNMAPPED if joint_name == 'gripper'
                    else ERR_WRIST_ROLL_UNMAPPED)
            self._set_error(
                code, 'direct servo END mapping is missing (seq=%d)'
                % cmd.sequence_id)
            return False
        buf = bytearray(CMD_PACKET_SIZE)
        self._pack_command_header(buf, TAG_DIRECT_SERVO_END, wire_id, 0)
        buf[8] = sid & 0xFF
        return self._i2c_write(buf)

    def _send_gripper_stop(self, cmd, wire_id):
        sid = self._servo_id_for('gripper')
        if sid is None:
            self._set_error(ERR_GRIPPER_UNMAPPED,
                            'gripper servo id not in servo_id_map (seq=%d)'
                            % cmd.sequence_id)
            return False
        buf = bytearray(CMD_PACKET_SIZE)
        self._pack_command_header(buf, TAG_SERVO_STOP, wire_id, 0)
        buf[8] = sid & 0xFF
        return self._i2c_write(buf)

    def _servo_id_for(self, joint_name):
        for entry in self._cfg('servo_id_map'):
            if ':' not in entry:
                continue
            name, sid = entry.split(':', 1)
            if name.strip() == joint_name:
                try:
                    return int(sid)
                except ValueError:
                    return None
        return None

    def _joint_index(self, joint_name):
        try:
            return list(self._cfg('joint_names')).index(joint_name)
        except ValueError:
            return None

    # ===================== state helpers =====================
    def _set_error(self, code, message):
        # ESTOP is the authoritative top-level state while latched. Keep the
        # detailed error code/message for diagnostics without downgrading the
        # published safety state to ERROR.
        self._state = (ArmState.STATE_ESTOP
                       if self._estop_latched else ArmState.STATE_ERROR)
        self._error_code = code
        self._error_message = message
        self._command_phase = ArmState.PHASE_FAILED
        self.get_logger().error('[seq=%d] err=0x%04X %s'
                                % (self._last_sequence_id, code, message))

    def _set_recoverable_rejection(self, code, message):
        """Report a rejected target without latching the controller in ERROR."""
        self._state = ArmState.STATE_IDLE
        self._error_code = code
        self._error_message = message
        self._command_phase = ArmState.PHASE_FAILED
        self.get_logger().warn('[seq=%d] err=0x%04X %s'
                               % (self._last_sequence_id, code, message))

    def _set_state(self, state, seq=None, phase=None):
        if seq is None:
            seq = self._last_sequence_id
        else:
            self._last_sequence_id = seq
        # Clear transient errors on a normal transition, but keep the latched
        # I2C-lost error until communication is actually restored.
        if state != ArmState.STATE_ERROR and self._error_code != ERR_I2C_LOST:
            self._error_code = 0
            self._error_message = ''
        self._state = state
        if phase is not None:
            self._command_phase = phase

    # ===================== command watchdog (sec 5.3) =====================
    def _arm_pending_motion(self, seq, wire_id):
        self._pending_motion = True
        self._pending_seq = seq
        self._pending_wire_id = wire_id
        self._pending_sent_ns = self.get_clock().now().nanoseconds

    def _check_command_timeout(self):
        # A readable packet for another command proves liveness but is not an
        # acknowledgement. Only a matching wire_command_id stops this timer.
        if not self._pending_motion or self._estop_latched:
            return
        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = int(self._cfg('command_timeout_sec') * 1e9)
        if now_ns - self._pending_sent_ns > timeout_ns:
            seq = self._pending_seq
            wire_id = self._pending_wire_id
            self._clear_active_motion()
            self._stream_open = False
            self._stream_open_family = None
            self._set_error(
                ERR_CMD_TIMEOUT,
                'no matching firmware ack within %.2fs '
                '(seq=%d, wire_id=%d)'
                % (self._cfg('command_timeout_sec'), seq, wire_id))

    def _check_terminal_timeout(self):
        """
        Require every acknowledged command to reach a terminal state.

        The ACK watchdog ends after ACCEPTED/EXECUTING.  Keep a second,
        duration-aware deadline so a lost COMPLETED/FAILED packet cannot leave
        an active command and cross-mode queue stuck forever.
        """
        if (self._pending_motion or self._estop_latched
                or self._active_wire_id == 0
                or self._active_sent_ns == 0):
            return
        expected_duration_sec = max(
            0.0, self._active_expected_duration_sec)
        if self._is_stream_end(self._active_mode):
            # F/G have no duration field on the wire.  They decelerate from
            # the current planned velocity and may legitimately need the
            # firmware's full 30-second motion window before reaching their
            # stable completion criteria.  Do not treat zero wire duration as
            # an immediate command and release the cross-mode queue early.
            expected_duration_sec = max(
                expected_duration_sec,
                max(0.0, float(self._cfg('max_duration_sec'))))
        timeout_sec = expected_duration_sec + max(
            0.0, float(self._cfg('command_timeout_sec')))
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._active_sent_ns <= int(timeout_sec * 1e9):
            return
        seq = self._active_seq
        wire_id = self._active_wire_id
        mode = self._active_mode
        self._clear_active_motion()
        self._stream_open = False
        self._stream_open_family = None
        if self._is_stream_end(mode):
            # F/G carry no wire duration and the firmware may not confirm the
            # terminal state after a large deceleration window.  Treat the
            # stream as ended instead of locking the control stack with
            # ERR_CMD_TIMEOUT, so the VLA scheduler can release its pending
            # stream-end and continue with the next action family.
            self.get_logger().warn(
                'stream end %s acknowledged but no terminal firmware '
                'lifecycle within %.2fs (seq=%d, wire_id=%d); '
                'treating stream as ended'
                % (_motion_mode_tag(mode), timeout_sec, seq, wire_id))
            self._set_state(
                ArmState.STATE_SUCCEEDED, seq, ArmState.PHASE_COMPLETED)
            self.publish_state()
            return
        self._set_error(
            ERR_CMD_TIMEOUT,
            'no terminal firmware lifecycle within %.2fs '
            '(seq=%d, wire_id=%d, mode=%s)'
            % (timeout_sec, seq, wire_id, _motion_mode_tag(mode)))

    def _reconcile_motion_state(self):
        """Repair a public MOVING state that has no hardware lifecycle."""
        if self._pending_motion or self._active_wire_id != 0:
            return
        if (self._queued_command is not None
                or self._queued_stream_end is not None):
            self._flush_queued_command()
            if self._pending_motion or self._active_wire_id != 0:
                return
        if self._state == ArmState.STATE_MOVING:
            self._set_state(
                ArmState.STATE_IDLE, phase=ArmState.PHASE_NONE)

    # ===================== publishers =====================
    def publish_state(self):
        msg = ArmState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self._cfg('end_effector_frame'))
        msg.state = self._state
        msg.command_phase = self._command_phase
        msg.sequence_id = self._last_sequence_id
        msg.joint_position = list(self._joint_position)
        msg.gripper_position = self._gripper_position
        msg.position_valid = self._position_valid
        msg.error_code = self._error_code
        msg.error_message = self._error_message
        self.state_pub.publish(msg)

        if self._cfg('joint_feedback_enabled') and self._position_valid:
            self._publish_joint_states()
        # Never publish /joint_states when feedback is disabled or unverified.

    def _update_joint_feedback(self, data):
        """
        Decode joint positions from the 32-byte status packet.

        Bytes 8..31 contain float32[6] servo raw positions for ids 1..6.
        Refresh the five arm joints using servo_id_map and mark them valid.
        """
        raw = bytes(data)
        for i in range(I2C_JOINT_COUNT):
            off = 8 + i * 4
            if off + 4 <= len(raw):
                self._servo_raw_positions[i] = struct.unpack_from('<f', raw, off)[0]
        # Map raw servo positions to arm joints (rad) for ArmState + /joint_states.
        zeros = self._cfg('joint_zero_offsets')
        dirs = self._cfg('joint_directions')
        rad_per_raw = math.pi * 240.0 / 180.0 / 1000.0  # Lobot: 0..1000 -> 0..240 deg
        arm_raw_positions = []
        ji = 0
        for entry in self._cfg('servo_id_map'):
            if ':' not in entry:
                continue
            name, sid_s = entry.split(':', 1)
            name = name.strip()
            if name == 'gripper':
                continue  # gripper state is carried by ArmState.gripper_position
            sid = int(sid_s)
            if 1 <= sid <= len(self._servo_raw_positions):
                pos = self._servo_raw_positions[sid - 1]
                arm_raw_positions.append(pos)
                self._joint_position[ji] = dirs[ji] * (pos - 500.0) * rad_per_raw + zeros[ji]
            ji += 1
        # Servo id 1 is the gripper. Its canonical contract is 0=open,
        # 1=closed, so decode real feedback instead of echoing the last command.
        opened = float(self._cfg('gripper_open_raw'))
        closed = float(self._cfg('gripper_closed_raw'))
        span = closed - opened
        if abs(span) > 1e-9:
            actual = (self._servo_raw_positions[0] - opened) / span
            self._gripper_position = max(0.0, min(1.0, actual))
        expected_joint_count = len(self._cfg('joint_names'))
        self._position_valid = (
            len(arm_raw_positions) == expected_joint_count
            and all(math.isfinite(pos) and 0.0 < pos <= 1000.0
                    for pos in arm_raw_positions)
        )

    def _publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        zeros = self._cfg('joint_zero_offsets')
        dirs = self._cfg('joint_directions')
        rad_per_raw = math.pi * 240.0 / 180.0 / 1000.0
        names = []
        positions = []
        ji = 0
        for entry in self._cfg('servo_id_map'):
            if ':' not in entry:
                continue
            name, sid_s = entry.split(':', 1)
            name = name.strip()
            if name == 'gripper':
                continue  # gripper carried by ArmState.gripper_position
            sid = int(sid_s)
            if 1 <= sid <= len(self._servo_raw_positions):
                pos = self._servo_raw_positions[sid - 1]
                names.append(name)
                positions.append(dirs[ji] * (pos - 500.0) * rad_per_raw + zeros[ji])
            ji += 1
        js.name = names
        js.position = positions
        js.velocity = []
        js.effort = []
        self.joint_state_pub.publish(js)

    def poll_status(self):
        self._check_command_timeout()
        self._check_terminal_timeout()
        self._reconcile_motion_state()

        data = self._i2c_read_status()
        if data is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        self._last_fw_contact_ns = now_ns
        decoded = _decode_firmware_packet(data)
        if decoded is None:
            self._firmware_protocol_ok = False
            self._position_valid = False
            self._clear_active_motion()
            self._stream_open = False
            self._stream_open_family = None
            raw_header = bytes(data[:8]).hex()
            if self._error_code != ERR_FW_PROTOCOL:
                self._set_error(
                    ERR_FW_PROTOCOL,
                    'firmware I2C v3 status invalid (header=%s)' % raw_header)
            return

        lifecycle, error, wire_id = decoded
        self._firmware_protocol_ok = True
        self._update_joint_feedback(data)
        if self._position_valid != self._last_position_valid:
            self.get_logger().warn(
                '[joint feedback] position_valid %s -> %s (lifecycle=%d '
                'error=%d wire_id=%d, servo_raw=[%s])'
                % (self._last_position_valid, self._position_valid, lifecycle,
                   error, wire_id,
                   ', '.join('%.1f' % v for v in self._servo_raw_positions)))
            self._last_position_valid = self._position_valid
        status_key = (lifecycle, error, wire_id)
        if status_key != self._last_logged_raw:
            self.get_logger().info(
                '[firmware status] lifecycle=%d error=%d wire_id=%d '
                '(servo1_raw=%.1f, servo6_base_raw=%.1f)'
                % (lifecycle, error, wire_id,
                   self._servo_raw_positions[0], self._servo_raw_positions[5]))
            self._last_logged_raw = status_key
        self.legacy_status_pub.publish(
            String(data=_legacy_status_text(lifecycle, error)))

        if lifecycle == FW_LIFECYCLE_READY and wire_id == 0:
            restarted = (self._firmware_seen_nonzero_id
                         or self._active_wire_id != 0
                         or self._pending_motion)
            if restarted:
                self._firmware_restart_latched = True
                self._firmware_seen_nonzero_id = False
                self._clear_active_motion()
                self._stream_open = False
                self._stream_open_family = None
                self._set_error(
                    ERR_FW_RESTARTED,
                    'firmware restarted; reset error and run home again')
            elif (not self._firmware_restart_latched
                  and not self._estop_latched
                  and self._error_code == 0):
                self._set_state(
                    ArmState.STATE_IDLE, phase=ArmState.PHASE_NONE)
            return

        if wire_id != 0:
            self._firmware_seen_nonzero_id = True

        if self._estop_latched:
            self._state = ArmState.STATE_ESTOP
            return

        # A valid but non-matching packet proves I2C liveness only.
        if wire_id == 0 or wire_id != self._active_wire_id:
            return

        if lifecycle == FW_LIFECYCLE_ACCEPTED:
            if (self._pending_motion
                    and wire_id == self._pending_wire_id):
                self._set_state(
                    ArmState.STATE_MOVING, self._active_seq,
                    ArmState.PHASE_ACCEPTED)
                if not self._is_stream_mode(self._active_mode):
                    self._pending_motion = False
                    self._flush_queued_command()
            return

        if lifecycle == FW_LIFECYCLE_EXECUTING:
            if (self._pending_motion
                    and wire_id == self._pending_wire_id):
                self._pending_motion = False
                if self._is_stream_target(self._active_mode):
                    self._stream_open = True
                    self._stream_open_family = self._stream_family(
                        self._active_mode)
                self._set_state(
                    ArmState.STATE_MOVING, self._active_seq,
                    ArmState.PHASE_EXECUTING)
                # Publish the installed target before dispatching a queued
                # replacement, otherwise the 10 Hz timer can miss this phase.
                self.publish_state()
                self._flush_queued_command()
            return

        if lifecycle == FW_LIFECYCLE_COMPLETED:
            completed_seq = self._active_seq
            completed_mode = self._active_mode
            self._clear_active_motion(clear_queue=False)
            if self._is_stream_end(completed_mode):
                self._stream_open = False
                self._stream_open_family = None
            self._set_state(
                ArmState.STATE_SUCCEEDED, completed_seq,
                ArmState.PHASE_COMPLETED)
            # Preserve the completed lifecycle edge before a queued command
            # replaces the public state in this same polling callback.
            self.publish_state()
            self._flush_queued_command()
            return

        if lifecycle == FW_LIFECYCLE_STOPPING:
            stopping_seq = self._active_seq
            self._pending_motion = False
            self._queued_command = None
            self._queued_stream_end = None
            self._stream_open = False
            self._stream_open_family = None
            self._last_sequence_id = stopping_seq
            code, message = _firmware_error_details(error)
            self._set_error(code, '%s (wire_id=%d)' % (message, wire_id))
            self._command_phase = ArmState.PHASE_STOPPING
            self.publish_state()
            return

        if lifecycle == FW_LIFECYCLE_FAILED:
            failed_seq = self._active_seq
            failed_mode = self._active_mode
            queued_stream_end = self._queued_stream_end
            self._clear_active_motion()
            self._last_sequence_id = failed_seq
            code, message = _firmware_error_details(error)
            if error == FW_ERROR_SERVO_FEEDBACK_FAILED:
                invalid_ids = [
                    str(index + 1)
                    for index, value in enumerate(self._servo_raw_positions)
                    if not math.isfinite(value) or not 0.0 < value <= 1000.0
                ]
                message += '; invalid_servo_ids=%s; servo_raw=%s' % (
                    ','.join(invalid_ids) or 'unknown',
                    ','.join('%.0f' % value
                             for value in self._servo_raw_positions))
            message = '%s (wire_id=%d)' % (message, wire_id)
            recoverable_rejection = (
                (error == FW_ERROR_NO_IK_SOLUTION
                 and failed_mode in (
                     ArmCommand.MODE_END_EFFECTOR,
                     ArmCommand.MODE_CARTESIAN_SERVO))
                or (error == FW_ERROR_STREAM_STEP_TOO_LARGE
                    and self._is_stream_target(failed_mode)))
            if recoverable_rejection:
                self._set_recoverable_rejection(code, message)
                # Preserve the rejected target edge before a queued END
                # dispatch replaces the public lifecycle in this callback.
                self.publish_state()
                if (self._is_stream_target(failed_mode)
                        and queued_stream_end is not None):
                    self._queued_stream_end = queued_stream_end
                    self._flush_queued_command()
            else:
                self._stream_open = False
                self._stream_open_family = None
                self._set_error(code, message)
                # Preserve the safety edge before a queued 10 Hz command can
                # replace ERROR with MOVING in the next executor callback.
                self.publish_state()

    # ===================== emergency stop (sec 3.2 / 5.4) =====================
    def emergency_stop_callback(self, msg):
        if msg.data:
            self._estop_request = True
            if not self._estop_latched:
                self._estop_latched = True
                # Best-effort safety stop to hardware.
                self._send_stop(self._last_sequence_id)
                self._set_state(
                    ArmState.STATE_ESTOP, phase=ArmState.PHASE_NONE)
                self.get_logger().warn('EMERGENCY STOP LATCHED (seq=%d)'
                                       % self._last_sequence_id)
        else:
            self._estop_request = False
            # false does NOT auto-release the latch (contract sec 3.2).
            self.get_logger().info('estop request cleared; latch remains until /arm/reset_error')

    # ===================== reset_error service (sec 3.5) =====================
    def reset_error_callback(self, request, response):
        problems = []
        # The latch itself is what this explicit service clears. Reset is only
        # blocked while the external estop request is still asserted.
        if self._estop_request:
            problems.append('estop request active')
        if not self.i2c_ok:
            problems.append('i2c not ok')
        if not self._firmware_protocol_ok:
            problems.append('firmware protocol v3 not verified')
        if self._pending_motion or self._active_wire_id != 0:
            problems.append('arm command active')
        if problems:
            response.success = False
            response.message = 'reset blocked: ' + ', '.join(problems)
            self.get_logger().warn(response.message)
            return response

        self._estop_latched = False
        self._estop_request = False
        self._error_code = 0
        self._error_message = ''
        self._clear_active_motion()
        self._stream_open = False
        self._stream_open_family = None
        self._firmware_restart_latched = False
        self._set_state(
            ArmState.STATE_IDLE, phase=ArmState.PHASE_NONE)
        response.success = True
        response.message = 'error reset; estop cleared'
        self.get_logger().info(response.message)
        return response

    # ===================== legacy /command_topic (deprecated, safe) =====================
    def legacy_command_callback(self, msg):
        data = msg.data.strip()
        parts = data.split()
        if not parts:
            return
        ctype = parts[0].upper()
        try:
            if ctype == 'ARM':
                if len(parts) < 8:
                    self.get_logger().error(
                        'legacy ARM needs 7 params '
                        '(x y z pitch min_pitch max_pitch time)')
                    return
                x, y, z, pitch, _min, _max, t = map(float, parts[1:8])
                cmd = ArmCommand()
                cmd.mode = ArmCommand.MODE_END_EFFECTOR
                cmd.x, cmd.y, cmd.z, cmd.pitch = x, y, z, pitch
                cmd.duration_sec = float(t)
                cmd.sequence_id = 0
                self.handle_command(cmd)

            elif ctype == 'SERVO':
                if len(parts) < 3:
                    self.get_logger().error('legacy SERVO needs id and angle')
                    return
                sid = int(float(parts[1]))
                angle = float(parts[2])
                gid = self._servo_id_for('gripper')
                if gid is not None and sid == gid:
                    cmd = ArmCommand()
                    cmd.mode = ArmCommand.MODE_GRIPPER
                    opened = float(self._cfg('gripper_open_raw'))
                    closed = float(self._cfg('gripper_closed_raw'))
                    span = closed - opened
                    pos = 0.0 if abs(span) < 1e-9 else (angle - opened) / span
                    cmd.gripper_position = max(0.0, min(1.0, pos))
                    cmd.sequence_id = 0
                    self.handle_command(cmd)
                else:
                    self.get_logger().warn(
                        'legacy SERVO id=%d not mapped to gripper; ignored '
                        '(use typed /arm/command modes)' % sid)

            elif ctype == 'STOP':
                cmd = ArmCommand()
                cmd.mode = ArmCommand.MODE_STOP
                cmd.sequence_id = 0
                self.handle_command(cmd)

            else:
                self.get_logger().warn('legacy unknown command: %s' % ctype)
        except Exception as e:
            self._set_error(ERR_NONFINITE_FIELD, 'legacy parse failed: %s' % e)

    # ===================== cleanup (sec 5.6) =====================
    def destroy_node(self):
        try:
            self._send_stop(self._last_sequence_id)
        except Exception:
            pass
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
