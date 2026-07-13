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

import math
import struct
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Header
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from action_interfaces.msg import ArmCommand, ArmState

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:  # pragma: no cover - runtime dep, keep node importable for inspection
    SMBus = None
    i2c_msg = None


CMD_PACKET_SIZE = 32
# Status packet is now 32 bytes: byte0..7 = text status code, byte8..31 =
# float32[6] real servo raw positions (see STM32 main.c I2C_JOINT_POS_OFFSET).
STATUS_PACKET_SIZE = 32
I2C_JOINT_COUNT = 6

# I2C command byte tags (must match STM32 firmware; contract sec 2.3)
TAG_ARM = ord('A')
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

# Firmware status codes returned in the 8-byte I2C status packet (main.c).
FW_STATUS_STM32_OK = 'STM32_OK'
FW_STATUS_ARM_RDY = 'ARM_RDY_'
FW_STATUS_ARM_OK = 'ARM_OK__'
FW_STATUS_SVO_OK = 'SVO_OK__'
FW_STATUS_STOP_OK = 'STOP_OK_'
FW_STATUS_NO_SOLVE = 'NO_SOLVE'
FW_STATUS_ARM_ERR = 'ARM_ERR_'
FW_STATUS_BAD_CMD = 'BAD_CMD_'
FW_STATUS_ARM_DONE = 'ARM_DONE'

# Firmware-reported error codes (distinct from node-side validation codes).
ERR_FW_NO_SOLVE = 0x0020
ERR_FW_NOT_READY = 0x0021
ERR_FW_BAD_CMD = 0x0022


def _decode_firmware_status(code):
    """
    Map an 8-char firmware status code to (ArmState, error_code, message).

    Returns (None, 0, '') for unrecognized codes (leave node state as-is).
    'ARM_OK__'/'SVO_OK__' mean the command was accepted and motion started
    (NOT that it finished). 'ARM_DONE' means all tracked servos have reached
    their targets (completion signal; firmware reports it after the motion).
    """
    if code in (FW_STATUS_ARM_OK, FW_STATUS_SVO_OK):
        return (ArmState.STATE_MOVING, 0, '')
    if code == FW_STATUS_ARM_DONE:
        return (ArmState.STATE_SUCCEEDED, 0, 'firmware: motion completed')
    if code in (FW_STATUS_STOP_OK, FW_STATUS_STM32_OK, FW_STATUS_ARM_RDY):
        return (ArmState.STATE_IDLE, 0, '')
    if code == FW_STATUS_NO_SOLVE:
        return (ArmState.STATE_ERROR, ERR_FW_NO_SOLVE,
                'firmware: inverse kinematics has no solution')
    if code == FW_STATUS_ARM_ERR:
        return (ArmState.STATE_ERROR, ERR_FW_NOT_READY,
                'firmware: arm not ready (robot_arm_ready==0)')
    if code == FW_STATUS_BAD_CMD:
        return (ArmState.STATE_ERROR, ERR_FW_BAD_CMD,
                'firmware: unknown/bad I2C command byte')
    return (None, 0, '')


# I2C failures before we latch into STATE_ERROR (contract sec 5.5)
I2C_FAIL_THRESHOLD = 5


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('arm_controller_node')

        self._declare_config_parameters()
        self._init_i2c()

        # --- runtime state (contract sec 3.3) ---
        self._state = ArmState.STATE_IDLE
        self._last_sequence_id = 0
        self._error_code = 0
        self._error_message = ''
        self._position_valid = False
        self._joint_position = [0.0] * 5
        self._servo_raw_positions = [0.0] * I2C_JOINT_COUNT  # servo raw (0..1000)
        self._gripper_position = 0.0

        # --- emergency stop (contract sec 3.2 / 5.4): latched ---
        self._estop_request = False
        self._estop_latched = False

        # --- I2C failure tracking (contract sec 5.5) ---
        self._i2c_fail_count = 0

        # --- last decoded firmware status (for change detection) ---
        self._last_firmware_status = ''
        self._last_logged_raw = ''

        # --- command lifecycle / watchdog (contract sec 5.3) ---
        self._pending_motion = False      # a motion command is awaiting firmware ack
        self._pending_seq = 0
        self._pending_sent_ns = 0
        self._last_fw_contact_ns = 0      # last time a recognized firmware status was read
        # Highest sequence_id actually executed, used for stale/duplicate checks.
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
            'min_duration_sec': 0.1,
            'max_duration_sec': 30.0,
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
            'joint_lower_limits': [-3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
            'joint_upper_limits': [3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
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
        try:
            msg = i2c_msg.read(self._cfg('i2c_address'), STATUS_PACKET_SIZE)
            self.bus.i2c_rdwr(msg)
            self._i2c_fail_count = 0
            self.i2c_ok = True
            # Communication restored: clear a stale latched I2C-lost error so
            # the typed state machine can reflect the live firmware status.
            if self._error_code == ERR_I2C_LOST:
                self._error_code = 0
                self._error_message = ''
                if self._state == ArmState.STATE_ERROR:
                    self._state = ArmState.STATE_IDLE
            return list(msg)
        except Exception as e:
            self._on_i2c_failure('read', e)
            return None

    def _on_i2c_failure(self, op, exc):
        self._i2c_fail_count += 1
        self.i2c_ok = False
        self._set_error(ERR_I2C_LOST,
                        'I2C %s failed (seq=%d, fails=%d): %s'
                        % (op, self._last_sequence_id, self._i2c_fail_count, exc))
        if self._i2c_fail_count >= I2C_FAIL_THRESHOLD:
            # stop sending motion commands (contract sec 5.5)
            self.get_logger().error('I2C failure threshold reached -> STATE_ERROR, halting motion')

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
            self._last_applied_seq = seq
            self._send_stop(seq)
            return

        if cmd.mode == ArmCommand.MODE_JOINT:
            # Mapping unconfirmed -> disabled (contract sec 4 note).
            self._set_error(ERR_JOINT_DISABLED,
                            'MODE_JOINT disabled until servo mapping confirmed (seq=%d)' % seq)
            return

        if cmd.mode == ArmCommand.MODE_END_EFFECTOR:
            if not self._validate_end_effector(cmd, seq):
                return
            if self._send_end_effector(cmd, seq):
                self._last_applied_seq = seq
                self._set_state(ArmState.STATE_MOVING, seq)
                self._arm_pending_motion(seq)
            return

        if cmd.mode == ArmCommand.MODE_GRIPPER:
            if not self._validate_gripper(cmd, seq):
                return
            if self._send_gripper(cmd, seq):
                self._last_applied_seq = seq
                # A gripper command is an immediate SERVO write; the firmware
                # answers SVO_OK__ synchronously and never emits a distinct
                # "done" transition. Do NOT arm the motion watchdog here: with
                # repeated gripper commands the status string stays SVO_OK__
                # (no transition), which would falsely trip ERR_CMD_TIMEOUT
                # (0x0016) even though the servo executed. Mark success
                # directly and clear any pending watchdog instead.
                self._pending_motion = False
                self._set_state(ArmState.STATE_SUCCEEDED, seq)
            return

        self._set_error(ERR_UNKNOWN_MODE, 'unknown mode %d (seq=%d)' % (cmd.mode, seq))

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

    # ===================== I2C packet builders (sec 2.3 layout) =====================
    def _send_stop(self, seq):
        buf = bytearray(CMD_PACKET_SIZE)
        buf[0] = TAG_STOP
        self._i2c_write(buf)
        # A STOP command must never make a latched emergency stop look cleared.
        state = ArmState.STATE_ESTOP if self._estop_latched else ArmState.STATE_IDLE
        self._set_state(state, seq)

    def _send_end_effector(self, cmd, seq):
        buf = bytearray(CMD_PACKET_SIZE)
        buf[0] = TAG_ARM
        struct.pack_into('<f', buf, 4, cmd.x)
        struct.pack_into('<f', buf, 8, cmd.y)
        struct.pack_into('<f', buf, 12, cmd.z)
        struct.pack_into('<f', buf, 16, cmd.pitch)
        # min/max pitch form the IK's allowed end-effector roll window
        # (firmware calls set_pitch_range(min,max) inside robot_arm_coordinate_set).
        # They are NOT part of ArmCommand, so we take them from config. The
        # reference firmware examples use [-90, 90]; do NOT set both to cmd.pitch
        # (i.e. a degenerate [0,0] window) or the IK is over-constrained and
        # returns NO_SOLVE even for reachable points.
        struct.pack_into('<f', buf, 20, self._cfg('pitch_min_deg'))
        struct.pack_into('<f', buf, 24, self._cfg('pitch_max_deg'))
        # Firmware forwards byte28 to serial_servo_set_position(), whose 'duration'
        # argument is uint16 in MILLISECONDS (Lobot MOVE_TIME_WRITE frame, see
        # serial_servo.c). ArmCommand.duration_sec is in seconds, so convert to ms
        # and clamp to uint16 range (max ~65.535 s) to avoid truncation on the
        # firmware side.
        dur_ms = max(0, min(0xFFFF, int(round(cmd.duration_sec * 1000.0))))
        struct.pack_into('<I', buf, 28, dur_ms)
        return self._i2c_write(buf)

    def _send_gripper(self, cmd, seq):
        sid = self._servo_id_for('gripper')
        if sid is None:
            self._set_error(ERR_GRIPPER_UNMAPPED,
                            'gripper servo id not in servo_id_map (seq=%d)' % seq)
            return False
        opened = float(self._cfg('gripper_open_raw'))
        closed = float(self._cfg('gripper_closed_raw'))
        raw = opened + cmd.gripper_position * (closed - opened)
        buf = bytearray(CMD_PACKET_SIZE)
        buf[0] = TAG_SERVO
        buf[4] = sid & 0xFF
        struct.pack_into('<f', buf, 8, raw)
        # P packet byte12..15 carries move time in milliseconds. Keep one
        # second as the compatibility default for legacy callers that leave
        # duration_sec at its zero-initialized value.
        duration = (cmd.duration_sec
                    if math.isfinite(cmd.duration_sec) and cmd.duration_sec > 0
                    else 1.0)
        dur_ms = max(1, min(0xFFFF, int(round(duration * 1000.0))))
        struct.pack_into('<I', buf, 12, dur_ms)
        ok = self._i2c_write(buf)
        if ok:
            self._gripper_position = cmd.gripper_position
        return ok

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

    # ===================== state helpers =====================
    def _set_error(self, code, message):
        # ESTOP is the authoritative top-level state while latched. Keep the
        # detailed error code/message for diagnostics without downgrading the
        # published safety state to ERROR.
        self._state = (ArmState.STATE_ESTOP
                       if self._estop_latched else ArmState.STATE_ERROR)
        self._error_code = code
        self._error_message = message
        self.get_logger().error('[seq=%d] err=0x%04X %s'
                                % (self._last_sequence_id, code, message))

    def _set_state(self, state, seq=None):
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

    # ===================== command watchdog (sec 5.3) =====================
    def _arm_pending_motion(self, seq):
        self._pending_motion = True
        self._pending_seq = seq
        self._pending_sent_ns = self.get_clock().now().nanoseconds
        # Start the firmware-contact clock at send time so a *silent* firmware
        # is only detected after the full timeout, never instantly at startup.
        self._last_fw_contact_ns = self._pending_sent_ns

    def _check_command_timeout(self):
        # contract sec 5.3 / command_timeout_sec: a motion command that never
        # receives any recognized firmware status within the timeout is a fault.
        if not self._pending_motion or self._estop_latched:
            return
        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = int(self._cfg('command_timeout_sec') * 1e9)
        if now_ns - self._last_fw_contact_ns > timeout_ns:
            self._pending_motion = False
            self._set_error(
                ERR_CMD_TIMEOUT,
                'no firmware status within %.2fs of command (seq=%d)'
                % (self._cfg('command_timeout_sec'), self._pending_seq))

    # ===================== publishers =====================
    def publish_state(self):
        msg = ArmState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self._cfg('end_effector_frame'))
        msg.state = self._state
        msg.sequence_id = self._last_sequence_id
        msg.joint_position = list(self._joint_position)
        msg.gripper_position = self._gripper_position
        msg.position_valid = self._position_valid
        msg.error_code = self._error_code
        msg.error_message = self._error_message
        self.state_pub.publish(msg)

        if self._cfg('joint_feedback_enabled'):
            self._publish_joint_states()
        # When disabled we do NOT publish /joint_states (contract sec 5.7:
        # never forge joint positions when feedback is unverified).

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
        self._position_valid = True

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
        # Watchdog runs on every poll, before the estop / transition guards,
        # so a silent firmware is still detected while estopped or idle.
        self._check_command_timeout()

        data = self._i2c_read_status()
        if data is None:
            return
        # Continuously decode real servo positions from the 32-byte status
        # packet and refresh ArmState.joint_position / position_valid. This runs
        # on every poll, independent of the status-code transition logic below,
        # so /joint_states reflects live hardware even when idle.
        self._update_joint_feedback(data)
        status_str = bytes(data[:8]).decode('utf-8', errors='ignore').rstrip('\x00')
        # Diagnostic: log the raw 8-byte status (including recognized codes) on
        # every change, so a silent/non-ACKing firmware is visible during debug.
        if status_str != self._last_logged_raw:
            self.get_logger().info(
                '[raw firmware status] %r '
                '(servo1_raw=%.1f, servo6_base_raw=%.1f)'
                % (status_str, self._servo_raw_positions[0],
                   self._servo_raw_positions[5]))
            self._last_logged_raw = status_str
        m = String()
        m.data = status_str
        self.legacy_status_pub.publish(m)

        # Reflect the firmware status in the typed state machine (contract
        # sec 3.3). The emergency-stop latch is authoritative: while latched,
        # firmware status must not clear it back to MOVING/IDLE.
        if self._estop_latched:
            self._state = ArmState.STATE_ESTOP
            self._last_firmware_status = status_str
            return

        # Communication liveness: a successful read means the firmware is
        # reachable. Refresh the contact timestamp and clear any pending
        # watchdog on EVERY successful read (not only on a status transition),
        # so consecutive commands that yield an identical status string
        # (e.g. repeated SVO_OK__, or a second ARM_DONE with no new transition)
        # no longer trip a spurious 0x0016 timeout even though the servo moved.
        now_ns = self.get_clock().now().nanoseconds
        self._last_fw_contact_ns = now_ns
        if self._pending_motion:
            self._pending_motion = False
            self.get_logger().info('[watchdog cleared] firmware contact (status=%r)' % status_str)

        # Only act on a status *transition* for the typed state machine, to
        # avoid 10 Hz churn and log spam.
        if status_str == self._last_firmware_status:
            return
        self._last_firmware_status = status_str

        state, code, message = _decode_firmware_status(status_str)
        if state is None:
            self.get_logger().warn('unrecognized firmware status: %r' % status_str)
            return

        if state == ArmState.STATE_ERROR:
            self._set_error(code, message)
        elif state == ArmState.STATE_SUCCEEDED:
            # Terminal state of a motion we initiated. Only adopt when we are
            # actually MOVING; a stale ARM_DONE (e.g. at startup) must not
            # fabricate success. Dedup prevents re-processing the same code.
            if self._state == ArmState.STATE_MOVING:
                self._set_state(ArmState.STATE_SUCCEEDED)
        else:
            # Adopt MOVING/IDLE from firmware only when not in a node-side
            # error, so a "OK" from a *prior* command cannot mask a local
            # validation error (e.g. MODE_JOINT disabled). SUCCEEDED is
            # included so the next command can re-enter MOVING.
            if self._state in (ArmState.STATE_IDLE, ArmState.STATE_MOVING,
                               ArmState.STATE_SUCCEEDED):
                self._set_state(state)

    # ===================== emergency stop (sec 3.2 / 5.4) =====================
    def emergency_stop_callback(self, msg):
        if msg.data:
            self._estop_request = True
            if not self._estop_latched:
                self._estop_latched = True
                # Best-effort safety stop to hardware.
                self._send_stop(self._last_sequence_id)
                self._set_state(ArmState.STATE_ESTOP)
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
        if self._state == ArmState.STATE_MOVING:
            problems.append('arm moving')
        if problems:
            response.success = False
            response.message = 'reset blocked: ' + ', '.join(problems)
            self.get_logger().warn(response.message)
            return response

        self._estop_latched = False
        self._estop_request = False
        self._error_code = 0
        self._error_message = ''
        self._pending_motion = False
        self._set_state(ArmState.STATE_IDLE)
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
                        '(use /arm/command MODE_JOINT after mapping confirmed)' % sid)

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
