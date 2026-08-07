#!/usr/bin/env python3
"""Xbox Cartesian teleoperation with synchronization and safety gates."""

from dataclasses import replace
import math
import time

from action_interfaces.msg import ArmCommand, ArmState
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
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 3
BUTTON_Y = 4
BUTTON_LB = 6
BUTTON_RB = 7

ERR_FW_NO_SOLVE = 0x0020
ERR_FW_STREAM_STEP_TOO_LARGE = 0x0026
ARM_TARGET_HISTORY_LIMIT = 64


class ArmTeleopNode(Node):
    """Maintain an absolute target while Xbox requests small increments."""

    def __init__(self, parameter_overrides=None):
        super().__init__(
            'arm_teleop_node', parameter_overrides=parameter_overrides)
        self._declare_parameters()

        self._shadow = bool(self._cfg('shadow_mode'))
        home = list(self._cfg('home_target'))
        expected_deg = list(self._cfg('home_joint_deg'))
        if len(home) != 4 or len(expected_deg) != 5:
            raise ValueError(
                'home_target needs 4 values and home_joint_deg needs 5')

        self._home_target = Target(*map(float, home), gripper=0.0)
        self._target = self._home_target
        self._last_successful_arm_target = self._home_target
        self._last_successful_direct_target = self._home_target
        self._arm_targets_by_seq = {}
        self._direct_targets_by_seq = {}
        self._no_ik_waiting_neutral = False
        self._last_no_ik_seq = None
        self._expected_home_joints = [math.radians(v) for v in expected_deg]
        self._home_tolerance = math.radians(
            float(self._cfg('home_joint_tolerance_deg')))
        self._bounds = {
            'pitch': tuple(self._cfg('pitch_limits_deg')),
            'wrist_roll': tuple(
                math.radians(value)
                for value in self._cfg('wrist_roll_limits_deg')),
        }

        command_topic = ('/arm/teleop_command'
                         if self._shadow else '/arm/command')
        estop_topic = ('/arm/teleop_emergency_stop'
                       if self._shadow else '/arm/emergency_stop')
        joy_topic = '/joy_sim' if self._shadow else '/joy'

        self._command_pub = self.create_publisher(
            ArmCommand, command_topic, 10)
        self._estop_pub = self.create_publisher(Bool, estop_topic, 10)
        enabled_qos = QoSProfile(depth=1)
        enabled_qos.reliability = ReliabilityPolicy.RELIABLE
        enabled_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._enabled_pub = self.create_publisher(
            Bool, '/arm/teleop_enabled', enabled_qos)
        self._synced_pub = self.create_publisher(
            Bool, '/arm/teleop_synced', enabled_qos)
        self._joy_sub = self.create_subscription(
            Joy, joy_topic, self._joy_callback, 10)

        self._state_sub = None
        self._vla_enabled_sub = None
        self._reset_client = None
        if not self._shadow:
            self._state_sub = self.create_subscription(
                ArmState, '/arm/state', self._state_callback, 10)
            self._vla_enabled_sub = self.create_subscription(
                Bool, '/arm/vla_enabled', self._vla_enabled_callback,
                enabled_qos)
            self._reset_client = self.create_client(
                Trigger, '/arm/reset_error')

        self._enabled = False
        self._synced = self._shadow
        self._axes = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0]
        # joy_linux initializes untouched absolute trigger axes to 0.0 even
        # though this controller reports +1.0 when physically released.  Arm
        # each trigger independently only after its released endpoint has
        # actually been observed, so startup 0.0 cannot become a false
        # half-press.
        self._trigger_ready = [False, False]
        self._buttons = [0] * 16
        self._joy_valid = False
        self._last_joy_time = None
        self._last_state_time = None
        self._latest_state = None
        self._joy_timed_out = False
        self._state_timed_out = False
        self._home_samples = 0
        self._next_sequence = 1
        self._home_open_pending_seq = None
        self._home_open_stop_pending_seq = None
        self._home_open_samples = 0
        self._home_roll_pending_seq = None
        self._home_pending_seq = None
        self._home_completion_seen = False
        self._home_completion_time = None
        self._home_failure_reason = ''
        self._reset_future = None
        self._gripper_close_origin = None
        self._gripper_last_feedback = None
        self._gripper_close_progress = False
        self._gripper_contact_samples = 0
        self._gripper_contact_latched = False
        self._gripper_close_command_active = False
        self._gripper_hold_pending_seq = None
        self._gripper_stop_pending_seq = None
        self._shadow_estop_latched = False
        self._arm_stream_active = False
        self._direct_stream_mode = None
        self._chord_started = {'reset': None, 'home': None}
        self._chord_triggered = {'reset': False, 'home': False}

        rate = float(self._cfg('control_rate_hz'))
        if rate <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        command_duration = float(self._cfg('command_duration_sec'))
        if command_duration <= 0.0:
            raise ValueError('command_duration_sec must be positive')
        stream_watchdog = float(self._cfg('stream_watchdog_sec'))
        if not 0.1 <= stream_watchdog <= 1.0:
            raise ValueError('stream_watchdog_sec must be within [0.1, 1.0]')
        gripper_stream_max_step = float(
            self._cfg('gripper_stream_max_target_step'))
        if not 0.0 < gripper_stream_max_step < 0.1:
            raise ValueError(
                'gripper_stream_max_target_step must be within (0.0, 0.1)')
        wrist_stream_max_step = float(
            self._cfg('wrist_stream_max_target_step_deg'))
        if not 0.0 < wrist_stream_max_step < 12.0:
            raise ValueError(
                'wrist_stream_max_target_step_deg must be within '
                '(0.0, 12.0)')
        home_open_tolerance = float(
            self._cfg('home_gripper_open_tolerance'))
        if not 0.0 <= home_open_tolerance <= 1.0:
            raise ValueError(
                'home_gripper_open_tolerance must be within [0.0, 1.0]')
        home_feedback_timeout = float(self._cfg('home_feedback_timeout_sec'))
        if home_feedback_timeout <= 0.0:
            raise ValueError('home_feedback_timeout_sec must be positive')
        self._last_tick = time.monotonic()
        self._timer = self.create_timer(1.0 / rate, self._control_tick)
        self._publish_enabled()
        self._publish_synced()
        if self._shadow:
            self.get_logger().info(
                'arm teleop started in shadow mode; waiting for A enable')
        else:
            self.get_logger().info(
                'arm teleop started in real mode; explicit home required '
                'before A enable')

    def _declare_parameters(self):
        defaults = {
            'shadow_mode': False,
            'control_rate_hz': 10.0,
            'joy_timeout_sec': 0.5,
            'state_timeout_sec': 0.5,
            'deadzone': 0.12,
            'trigger_deadzone': 0.05,
            'translation_speed_cm_sec': 1.5,
            'pitch_speed_deg_sec': 5.0,
            'wrist_roll_speed_deg_sec': 20.0,
            'gripper_speed_sec': 0.5,
            'gripper_stream_max_target_step': 0.075,
            'wrist_stream_max_target_step_deg': 9.0,
            'gripper_contact_min_progress': 0.02,
            'gripper_contact_min_gap': 0.04,
            'gripper_contact_stable_delta': 0.006,
            'gripper_contact_stable_samples': 3,
            'command_duration_sec': 0.09,
            'stream_watchdog_sec': 0.30,
            'home_gripper_duration_sec': 1.0,
            'home_gripper_open_tolerance': 0.10,
            'home_wrist_roll_duration_sec': 1.0,
            'home_duration_sec': 2.0,
            'home_target': [15.0, 0.0, 2.0, -54.48],
            'home_joint_deg': [0.0, 112.08, -89.04, -77.52, 0.0],
            'home_joint_tolerance_deg': 5.25,
            'home_stable_samples': 3,
            'home_feedback_timeout_sec': 3.0,
            'chord_hold_sec': 1.0,
            'pitch_limits_deg': [-90.0, 90.0],
            'wrist_roll_limits_deg': [-90.0, 90.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _cfg(self, name):
        return self.get_parameter(name).value

    def _joy_callback(self, msg):
        now = time.monotonic()
        axes = list(msg.axes)
        buttons = list(msg.buttons)
        if not valid_joy(axes, buttons):
            self._trigger_ready = [False, False]
            self._joy_valid = False
            self._lose_sync('invalid Joy shape or value')
            return

        axes = self._normalize_startup_triggers(axes)

        previous_axes = self._axes
        previous = self._buttons
        self._axes = axes
        self._buttons = buttons
        self._joy_valid = True
        self._joy_timed_out = False
        self._last_joy_time = now

        b_pressed = bool(buttons[BUTTON_B])
        b_was_pressed = bool(previous[BUTTON_B])
        if b_pressed != b_was_pressed:
            self._estop_pub.publish(Bool(data=b_pressed))
            if b_pressed:
                self._shadow_estop_latched = self._shadow
                self._home_open_pending_seq = None
                self._home_open_stop_pending_seq = None
                self._home_open_samples = 0
                self._home_roll_pending_seq = None
                self._home_pending_seq = None
                self._home_completion_seen = False
                self._home_completion_time = None
                self._lose_sync('B emergency stop')

        if rising_edge(buttons, previous, BUTTON_A):
            if self._enabled:
                self._finish_arm_stream()
                self._finish_direct_stream()
                self._set_enabled(False, 'A pause')
            else:
                reason = self._enable_block_reason(now)
                if reason:
                    self.get_logger().warn('enable rejected: %s' % reason)
                else:
                    self._set_enabled(True, 'A enable')

        trigger_deadzone = float(self._cfg('trigger_deadzone'))
        was_gripper_active = max(
            trigger_pressed(float(previous_axes[4])),
            trigger_pressed(float(previous_axes[5]))) > trigger_deadzone
        is_gripper_active = max(
            trigger_pressed(float(axes[4])),
            trigger_pressed(float(axes[5]))) > trigger_deadzone
        if (self._enabled and was_gripper_active and not is_gripper_active
                and self._direct_stream_mode
                == ArmCommand.MODE_GRIPPER_SERVO):
            self._finish_direct_stream()
        self._resume_after_no_ik_if_neutral(now)

    def _normalize_startup_triggers(self, axes):
        """Keep untouched joy_linux trigger axes neutral until released."""
        normalized = list(axes)
        trigger_deadzone = float(self._cfg('trigger_deadzone'))
        for slot, axis_index in enumerate((4, 5)):
            if self._trigger_ready[slot]:
                continue
            if (trigger_pressed(float(axes[axis_index]))
                    <= trigger_deadzone):
                self._trigger_ready[slot] = True
            else:
                normalized[axis_index] = 1.0
        return normalized

    def _state_callback(self, msg):
        self._latest_state = msg
        self._last_state_time = time.monotonic()
        self._state_timed_out = False
        self._next_sequence = max(
            self._next_sequence, (int(msg.sequence_id) + 1) & 0xFFFFFFFF)
        if self._next_sequence == 0:
            self._next_sequence = 1
        if msg.command_phase == ArmState.PHASE_EXECUTING:
            self._record_successful_arm_target(int(msg.sequence_id))
            self._record_successful_direct_target(int(msg.sequence_id))

        actual_wrist_roll = None
        if (msg.position_valid and len(msg.joint_position) >= 5
                and math.isfinite(msg.joint_position[4])):
            actual_wrist_roll = max(
                self._bounds['wrist_roll'][0],
                min(self._bounds['wrist_roll'][1], msg.joint_position[4]))
        try:
            wrist_input_neutral = apply_deadzone(
                float(self._axes[2]), float(self._cfg('deadzone'))) == 0.0
        except ValueError:
            wrist_input_neutral = False
        if (actual_wrist_roll is not None
                and (not self._enabled or wrist_input_neutral)):
            self._target = replace(
                self._target, wrist_roll=actual_wrist_roll)

        close_amount = trigger_pressed(float(self._axes[4]))
        open_amount = trigger_pressed(float(self._axes[5]))
        trigger_deadzone = float(self._cfg('trigger_deadzone'))
        triggers_released = (
            close_amount <= trigger_deadzone
            and open_amount <= trigger_deadzone
        )
        close_requested = close_amount - open_amount > trigger_deadzone
        actual_gripper = None
        if math.isfinite(msg.gripper_position):
            actual_gripper = max(0.0, min(1.0, msg.gripper_position))
        healthy_gripper_feedback = (
            msg.position_valid
            and msg.state not in (ArmState.STATE_ERROR, ArmState.STATE_ESTOP)
            and actual_gripper is not None
        )
        if (actual_gripper is not None
                and (not self._enabled or triggers_released)):
            self._target = replace(
                self._target,
                gripper=actual_gripper,
            )
        if (self._enabled and close_requested
                and healthy_gripper_feedback):
            self._observe_gripper_contact(actual_gripper)
        else:
            self._reset_gripper_contact_tracking(
                clear_latch=not self._enabled or not close_requested)

        if msg.error_code in (ERR_FW_NO_SOLVE,
                              ERR_FW_STREAM_STEP_TOO_LARGE):
            sequence = int(msg.sequence_id)
            if sequence in self._direct_targets_by_seq:
                self._handle_direct_rejection(sequence)
            else:
                self._handle_no_ik_rejection(sequence)
            return

        if msg.state in (ArmState.STATE_ERROR, ArmState.STATE_ESTOP):
            self._gripper_hold_pending_seq = None
            self._gripper_stop_pending_seq = None
            self._gripper_close_command_active = False
            self._home_open_pending_seq = None
            self._home_open_stop_pending_seq = None
            self._home_open_samples = 0
            self._home_roll_pending_seq = None
            self._home_pending_seq = None
            self._home_completion_seen = False
            self._home_completion_time = None
            self._lose_sync('arm state is ERROR/ESTOP')
            return

        if (self._gripper_hold_pending_seq is not None
                and msg.sequence_id == self._gripper_hold_pending_seq
                and msg.state == ArmState.STATE_SUCCEEDED):
            self._gripper_hold_pending_seq = None

        if (self._gripper_stop_pending_seq is not None
                and msg.sequence_id == self._gripper_stop_pending_seq
                and msg.state == ArmState.STATE_SUCCEEDED):
            self._gripper_stop_pending_seq = None

        if self._home_open_pending_seq is not None:
            if msg.sequence_id != self._home_open_pending_seq:
                return
            if msg.state == ArmState.STATE_SUCCEEDED:
                self._home_open_pending_seq = None
                self._home_open_samples = 0
                self._continue_home_after_open()
                return
            if (healthy_gripper_feedback
                    and actual_gripper <= float(
                        self._cfg('home_gripper_open_tolerance'))):
                self._home_open_samples += 1
            else:
                self._home_open_samples = 0
            if self._home_open_samples >= int(
                    self._cfg('home_stable_samples')):
                self._home_open_pending_seq = None
                self._home_open_samples = 0
                self._home_open_stop_pending_seq = (
                    self._request_gripper_stop())
                self.get_logger().info(
                    'gripper feedback is safely open; stop command sent')
            return

        if self._home_open_stop_pending_seq is not None:
            if msg.sequence_id != self._home_open_stop_pending_seq:
                return
            if msg.state == ArmState.STATE_SUCCEEDED:
                self._home_open_stop_pending_seq = None
                self._continue_home_after_open()
            return

        if self._home_roll_pending_seq is not None:
            if msg.sequence_id != self._home_roll_pending_seq:
                return
            if msg.state == ArmState.STATE_SUCCEEDED:
                self._home_roll_pending_seq = None
                self._target = replace(self._target, wrist_roll=0.0)
                self._start_home_arm()
                self.get_logger().info(
                    'wrist home completed; Cartesian home command sent')
            return

        if self._home_pending_seq is not None:
            if msg.sequence_id != self._home_pending_seq:
                return
            if (msg.state == ArmState.STATE_SUCCEEDED
                    and not self._home_completion_seen):
                self._home_completion_seen = True
                self._home_completion_time = time.monotonic()
            if (self._home_completion_seen and msg.position_valid
                    and joints_near_home(
                        msg.joint_position,
                        self._expected_home_joints,
                        self._home_tolerance)):
                self._home_samples += 1
                if self._home_samples >= int(
                        self._cfg('home_stable_samples')):
                    self._target = replace(
                        self._home_target, gripper=self._target.gripper)
                    self._last_successful_arm_target = self._target
                    self._home_pending_seq = None
                    self._home_completion_seen = False
                    self._home_completion_time = None
                    self._home_failure_reason = ''
                    self._synced = True
                    self._publish_synced()
                    self.get_logger().info(
                        'home feedback verified; target synchronized')
            elif self._home_completion_seen:
                self._home_samples = 0
                elapsed = time.monotonic() - self._home_completion_time
                if elapsed > float(self._cfg('home_feedback_timeout_sec')):
                    actual_deg = [
                        round(math.degrees(float(value)), 2)
                        for value in msg.joint_position
                    ] if msg.position_valid else 'invalid'
                    self._home_pending_seq = None
                    self._home_completion_seen = False
                    self._home_completion_time = None
                    self._home_failure_reason = (
                        'last home pose verification failed; run home again')
                    self.get_logger().warn(
                        'home pose verification failed after %.2fs; '
                        'position_valid=%s actual_joint_deg=%s '
                        'expected_joint_deg=%s; run home again'
                        % (elapsed, msg.position_valid, actual_deg,
                           [round(math.degrees(value), 2)
                            for value in self._expected_home_joints]))
            return

    def _control_tick(self):
        now = time.monotonic()
        rate = float(self._cfg('control_rate_hz'))
        dt = min(now - self._last_tick, 2.0 / rate)
        self._last_tick = now

        self._check_timeouts(now)
        self._update_chords(now)
        if not self._enabled or not self._joy_valid:
            return

        mapping_axes = self._axes
        trigger_deadzone = float(self._cfg('trigger_deadzone'))
        if self._gripper_contact_latched:
            close_amount = trigger_pressed(float(self._axes[4]))
            open_amount = trigger_pressed(float(self._axes[5]))
            if close_amount - open_amount > trigger_deadzone:
                # Contact already stopped the gripper. Ignore a still-held RT
                # so the operator can move the arm without first releasing it.
                mapping_axes = list(self._axes)
                mapping_axes[4] = 1.0
                mapping_axes[5] = 1.0

        updated, mode = integrate_target(
            self._target,
            mapping_axes,
            dt,
            deadzone=float(self._cfg('deadzone')),
            translation_speed=float(self._cfg('translation_speed_cm_sec')),
            pitch_speed=float(self._cfg('pitch_speed_deg_sec')),
            wrist_roll_speed=math.radians(
                float(self._cfg('wrist_roll_speed_deg_sec'))),
            gripper_speed=float(self._cfg('gripper_speed_sec')),
            bounds=self._bounds,
            pitch_modifier=bool(self._buttons[BUTTON_RB]),
            trigger_deadzone=trigger_deadzone,
        )
        direct_mode = None
        if mode == MODE_WRIST_ROLL:
            direct_mode = ArmCommand.MODE_WRIST_ROLL_SERVO
        elif mode == MODE_GRIPPER:
            direct_mode = ArmCommand.MODE_GRIPPER_SERVO
        if mode != MODE_ARM and self._arm_stream_active:
            self._finish_arm_stream()
            return
        if mode == MODE_ARM and self._direct_stream_mode is not None:
            self._finish_direct_stream()
            return
        if (self._direct_stream_mode is not None
                and direct_mode != self._direct_stream_mode):
            self._finish_direct_stream()
            return
        if mode is None:
            return
        if direct_mode is not None:
            if self._direct_stream_mode is None:
                # Start each direct stream from the latest synchronized target.
                # During a stream this baseline advances only after firmware
                # publishes a matching EXECUTING lifecycle.
                self._last_successful_direct_target = self._target
            updated = self._limit_direct_target(updated, mode)
        if (mode == MODE_ARM
                and (self._gripper_hold_pending_seq is not None
                     or self._gripper_stop_pending_seq is not None)):
            return
        if (mode == MODE_GRIPPER and self._gripper_contact_latched
                and updated.gripper > self._target.gripper):
            return
        closing = (mode == MODE_GRIPPER
                   and updated.gripper > self._target.gripper)
        if mode == MODE_GRIPPER and updated.gripper < self._target.gripper:
            self._gripper_close_command_active = False
            self._gripper_hold_pending_seq = None
            self._reset_gripper_contact_tracking(clear_latch=True)
        self._target = updated
        if mode == MODE_ARM:
            ros_mode = ArmCommand.MODE_CARTESIAN_SERVO
        elif mode == MODE_WRIST_ROLL:
            ros_mode = ArmCommand.MODE_WRIST_ROLL_SERVO
        else:
            ros_mode = ArmCommand.MODE_GRIPPER_SERVO
        duration = float(self._cfg('stream_watchdog_sec'))
        self._publish_command(
            ros_mode, self._target, duration)
        if closing:
            self._gripper_close_command_active = True

    def _limit_direct_target(self, target, mode):
        """Bound U targets from the last firmware-confirmed target."""
        baseline = self._last_successful_direct_target
        if mode == MODE_GRIPPER:
            step = float(self._cfg('gripper_stream_max_target_step'))
            return replace(
                target,
                gripper=max(
                    baseline.gripper - step,
                    min(baseline.gripper + step, target.gripper)),
            )
        if mode == MODE_WRIST_ROLL:
            step = math.radians(
                float(self._cfg('wrist_stream_max_target_step_deg')))
            return replace(
                target,
                wrist_roll=max(
                    baseline.wrist_roll - step,
                    min(baseline.wrist_roll + step, target.wrist_roll)),
            )
        return target

    def _check_timeouts(self, now):
        joy_timeout = float(self._cfg('joy_timeout_sec'))
        if (self._last_joy_time is not None
                and now - self._last_joy_time > joy_timeout
                and not self._joy_timed_out):
            self._joy_timed_out = True
            self._trigger_ready = [False, False]
            self._joy_valid = False
            self._lose_sync('Joy timeout')

        if self._shadow or self._last_state_time is None:
            return
        state_timeout = float(self._cfg('state_timeout_sec'))
        if (now - self._last_state_time > state_timeout
                and not self._state_timed_out):
            self._state_timed_out = True
            self._lose_sync('ArmState timeout')

    def _observe_gripper_contact(self, actual):
        if self._gripper_contact_latched:
            return
        if self._gripper_close_origin is None:
            self._gripper_close_origin = actual
            self._gripper_last_feedback = actual
            return

        if (actual - self._gripper_close_origin
                >= float(self._cfg('gripper_contact_min_progress'))):
            self._gripper_close_progress = True
        stable = abs(actual - self._gripper_last_feedback) <= float(
            self._cfg('gripper_contact_stable_delta'))
        target_gap = self._target.gripper - actual
        if (self._gripper_close_progress and stable
                and target_gap >= float(
                    self._cfg('gripper_contact_min_gap'))):
            self._gripper_contact_samples += 1
        else:
            self._gripper_contact_samples = 0
        self._gripper_last_feedback = actual

        if self._gripper_contact_samples < int(
                self._cfg('gripper_contact_stable_samples')):
            return
        self._hold_gripper_contact(actual)

    def _hold_gripper_contact(self, actual):
        self._gripper_contact_latched = True
        self._gripper_close_command_active = False
        self._target = replace(self._target, gripper=actual)
        self._gripper_hold_pending_seq = self._request_gripper_stop()
        self.get_logger().info(
            'gripper contact detected; holding at feedback %.3f' % actual)

    def _request_gripper_stop(self):
        if self._gripper_stop_pending_seq is not None:
            return self._gripper_stop_pending_seq
        self._gripper_close_command_active = False
        if self._direct_stream_mode == ArmCommand.MODE_GRIPPER_SERVO:
            self._direct_stream_mode = None
        seq = self._publish_command(
            ArmCommand.MODE_GRIPPER_STOP, self._target, 0.0)
        if not self._shadow:
            self._gripper_stop_pending_seq = seq
        self.get_logger().info('gripper stop command sent')
        return seq

    def _finish_arm_stream(self):
        if not self._arm_stream_active:
            return
        self._publish_command(
            ArmCommand.MODE_CARTESIAN_SERVO_END, self._target, 0.0)
        self._arm_stream_active = False

    def _finish_direct_stream(self):
        if self._direct_stream_mode is None:
            return False
        if self._direct_stream_mode == ArmCommand.MODE_GRIPPER_SERVO:
            # The deployed firmware can keep G(gripper) EXECUTING forever when
            # its feedback-based stable-target condition is not reached.  H is
            # the bounded single-servo stop path and lets arm motion continue
            # after its matching COMPLETED lifecycle.
            self._request_gripper_stop()
            return True
        end_mode = ArmCommand.MODE_WRIST_ROLL_SERVO_END
        self._publish_command(end_mode, self._target, 0.0)
        self._direct_stream_mode = None
        self._gripper_close_command_active = False
        return True

    def _reset_gripper_contact_tracking(self, clear_latch=False):
        self._gripper_close_origin = None
        self._gripper_last_feedback = None
        self._gripper_close_progress = False
        self._gripper_contact_samples = 0
        if clear_latch:
            self._gripper_contact_latched = False

    def _update_chords(self, now):
        if not self._joy_valid:
            return
        lb = bool(self._buttons[BUTTON_LB])
        rb = bool(self._buttons[BUTTON_RB])
        x = bool(self._buttons[BUTTON_X])
        y = bool(self._buttons[BUTTON_Y])
        self._update_chord('reset', lb and rb and x and not y,
                           now, self._request_reset)
        self._update_chord('home', lb and rb and y and not x,
                           now, self._request_home)

    def _update_chord(self, name, active, now, action):
        if not active:
            self._chord_started[name] = None
            self._chord_triggered[name] = False
            return
        if self._chord_started[name] is None:
            self._chord_started[name] = now
            return
        held = now - self._chord_started[name]
        if (not self._chord_triggered[name]
                and held >= float(self._cfg('chord_hold_sec'))):
            self._chord_triggered[name] = True
            action()

    def _request_reset(self):
        if self._enabled or bool(self._buttons[BUTTON_B]):
            self.get_logger().warn(
                'reset rejected: disable teleop and release B')
            return
        self._lose_sync('reset requested')
        if self._shadow:
            self._shadow_estop_latched = False
            self.get_logger().info(
                'shadow reset completed; home is still required')
            return
        if self._reset_future is not None:
            return
        if not self._reset_client.service_is_ready():
            self.get_logger().warn('/arm/reset_error is not ready')
            return
        self._reset_future = self._reset_client.call_async(Trigger.Request())
        self._reset_future.add_done_callback(self._reset_done)

    def _reset_done(self, future):
        self._reset_future = None
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - ROS transport failure
            self.get_logger().error('reset service failed: %s' % exc)
            return
        if response.success:
            self.get_logger().info('reset completed; home is still required')
        else:
            self.get_logger().warn('reset rejected: %s' % response.message)

    def _request_home(self):
        if self._enabled or bool(self._buttons[BUTTON_B]):
            self.get_logger().warn(
                'home rejected: disable teleop and release B')
            return
        if self._shadow_estop_latched:
            self.get_logger().warn(
                'home rejected: shadow estop is still latched')
            return
        if (self._reset_future is not None
                or self._home_open_pending_seq is not None
                or self._home_open_stop_pending_seq is not None
                or self._home_roll_pending_seq is not None
                or self._home_pending_seq is not None):
            return
        if not self._shadow:
            reason = self._state_block_reason(
                time.monotonic(), require_position=False)
            if reason:
                self.get_logger().warn('home rejected: %s' % reason)
                return

        self._home_failure_reason = ''
        target = replace(self._home_target, gripper=0.0)
        seq = self._publish_command(
            ArmCommand.MODE_GRIPPER,
            target,
            float(self._cfg('home_gripper_duration_sec')))
        if self._shadow:
            self._start_home_wrist()
            self._home_roll_pending_seq = None
            self._start_home_arm()
            self._home_pending_seq = None
            self._target = target
            self._last_successful_arm_target = target
            self._synced = True
            self._publish_synced()
            self.get_logger().info(
                'shadow home completed; target synchronized')
        else:
            self._home_open_pending_seq = seq
            self._home_open_samples = 0
            self.get_logger().info(
                'gripper open command sent; waiting before home arm motion')

    def _continue_home_after_open(self):
        self._target = replace(self._target, gripper=0.0)
        self._start_home_wrist()
        self.get_logger().info(
            'gripper open completed; wrist home command sent')

    def _start_home_wrist(self):
        target = replace(self._home_target, gripper=0.0)
        self._home_roll_pending_seq = self._publish_command(
            ArmCommand.MODE_WRIST_ROLL,
            target,
            float(self._cfg('home_wrist_roll_duration_sec')))

    def _start_home_arm(self):
        target = replace(self._home_target, gripper=0.0)
        self._home_pending_seq = self._publish_command(
            ArmCommand.MODE_END_EFFECTOR,
            target,
            float(self._cfg('home_duration_sec')))
        self._home_completion_seen = False
        self._home_completion_time = None
        self._home_samples = 0

    def _publish_command(self, mode, target, duration):
        msg = ArmCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = mode
        msg.x = target.x
        msg.y = target.y
        msg.z = target.z
        msg.pitch = target.pitch
        msg.joint_position[4] = target.wrist_roll
        msg.gripper_position = target.gripper
        msg.duration_sec = duration
        msg.sequence_id = self._take_sequence()
        self._command_pub.publish(msg)
        if mode in (ArmCommand.MODE_END_EFFECTOR,
                    ArmCommand.MODE_CARTESIAN_SERVO):
            self._arm_targets_by_seq[msg.sequence_id] = target
            while len(self._arm_targets_by_seq) > ARM_TARGET_HISTORY_LIMIT:
                oldest = next(iter(self._arm_targets_by_seq))
                del self._arm_targets_by_seq[oldest]
        if mode == ArmCommand.MODE_CARTESIAN_SERVO:
            self._arm_stream_active = True
        elif mode == ArmCommand.MODE_CARTESIAN_SERVO_END:
            self._arm_stream_active = False
        if mode in (ArmCommand.MODE_GRIPPER_SERVO,
                    ArmCommand.MODE_WRIST_ROLL_SERVO):
            self._direct_stream_mode = mode
            self._direct_targets_by_seq[msg.sequence_id] = (mode, target)
            while len(self._direct_targets_by_seq) > ARM_TARGET_HISTORY_LIMIT:
                oldest = next(iter(self._direct_targets_by_seq))
                del self._direct_targets_by_seq[oldest]
        elif mode in (ArmCommand.MODE_GRIPPER_SERVO_END,
                      ArmCommand.MODE_WRIST_ROLL_SERVO_END):
            self._direct_stream_mode = None
        return msg.sequence_id

    def _record_successful_arm_target(self, sequence):
        target = self._arm_targets_by_seq.get(sequence)
        if target is None:
            return
        self._last_successful_arm_target = target
        for pending_sequence in list(self._arm_targets_by_seq):
            del self._arm_targets_by_seq[pending_sequence]
            if pending_sequence == sequence:
                break

    def _record_successful_direct_target(self, sequence):
        installed = self._direct_targets_by_seq.get(sequence)
        if installed is None:
            return
        mode, target = installed
        if mode == ArmCommand.MODE_GRIPPER_SERVO:
            self._last_successful_direct_target = replace(
                self._last_successful_direct_target,
                gripper=target.gripper)
            self._target = replace(self._target, gripper=target.gripper)
        elif mode == ArmCommand.MODE_WRIST_ROLL_SERVO:
            self._last_successful_direct_target = replace(
                self._last_successful_direct_target,
                wrist_roll=target.wrist_roll)
            self._target = replace(
                self._target, wrist_roll=target.wrist_roll)
        for pending_sequence in list(self._direct_targets_by_seq):
            del self._direct_targets_by_seq[pending_sequence]
            if pending_sequence == sequence:
                break

    def _handle_direct_rejection(self, sequence):
        if self._last_no_ik_seq == sequence:
            return
        self._last_no_ik_seq = sequence
        installed = self._direct_targets_by_seq.get(sequence)
        if installed is not None:
            mode, _ = installed
            if mode == ArmCommand.MODE_GRIPPER_SERVO:
                self._target = replace(
                    self._target,
                    gripper=self._last_successful_direct_target.gripper)
            elif mode == ArmCommand.MODE_WRIST_ROLL_SERVO:
                self._target = replace(
                    self._target,
                    wrist_roll=(
                        self._last_successful_direct_target.wrist_roll))
        self._direct_targets_by_seq.clear()
        self._finish_direct_stream()
        self._no_ik_waiting_neutral = True
        self._set_enabled(False, 'direct servo target rejected')
        self.get_logger().warn(
            'direct servo target rejected; ending the last valid stream; '
            'center controls to resume')

    def _handle_no_ik_rejection(self, sequence):
        if self._last_no_ik_seq == sequence:
            return
        self._last_no_ik_seq = sequence
        self._target = replace(
            self._last_successful_arm_target,
            wrist_roll=self._target.wrist_roll,
            gripper=self._target.gripper,
        )
        self._arm_targets_by_seq.clear()
        self._no_ik_waiting_neutral = True
        self._finish_arm_stream()
        self._set_enabled(False, 'NO_IK target rejected')
        self.get_logger().warn(
            'NO_IK target rejected; rolled back to last successful target; '
            'center controls to resume')

    def _resume_after_no_ik_if_neutral(self, now):
        if not self._no_ik_waiting_neutral:
            return
        reason = self._enable_block_reason(now, require_neutral=True)
        if reason:
            return
        self._no_ik_waiting_neutral = False
        self._set_enabled(True, 'controls centered after NO_IK')

    def _take_sequence(self):
        sequence = self._next_sequence
        self._next_sequence = (sequence + 1) & 0xFFFFFFFF
        if self._next_sequence == 0:
            self._next_sequence = 1
        return sequence

    def _enable_block_reason(self, now, require_neutral=False):
        if (self._home_open_pending_seq is not None
                or self._home_open_stop_pending_seq is not None
                or self._home_roll_pending_seq is not None
                or self._home_pending_seq is not None
                or self._reset_future is not None):
            return 'home/reset operation is in progress'
        if self._home_failure_reason:
            return self._home_failure_reason
        if not self._synced:
            return 'target is not synchronized; run home first'
        if self._shadow_estop_latched:
            return 'shadow estop is latched'
        if not self._joy_valid or self._last_joy_time is None:
            return 'no valid Joy input'
        if now - self._last_joy_time > float(self._cfg('joy_timeout_sec')):
            return 'Joy input is stale'
        if require_neutral:
            try:
                neutral = controls_neutral(
                    self._axes,
                    float(self._cfg('deadzone')),
                    float(self._cfg('trigger_deadzone')))
            except ValueError:
                neutral = False
            if not neutral:
                return 'sticks and triggers must be neutral'
        if not self._shadow:
            return self._state_block_reason(now)
        return ''

    def _state_block_reason(self, now, require_position=True):
        if self._latest_state is None or self._last_state_time is None:
            return 'no ArmState received'
        if now - self._last_state_time > float(self._cfg('state_timeout_sec')):
            return 'ArmState is stale'
        if require_position and not self._latest_state.position_valid:
            return 'joint feedback is not valid'
        if self._latest_state.state == ArmState.STATE_MOVING:
            return 'arm is moving'
        if self._latest_state.state == ArmState.STATE_ERROR:
            return 'arm is in ERROR'
        if self._latest_state.state == ArmState.STATE_ESTOP:
            return 'arm is in ESTOP'
        if self._latest_state.state not in (ArmState.STATE_IDLE,
                                            ArmState.STATE_SUCCEEDED):
            return 'arm state is not recoverable'
        return ''

    def _lose_sync(self, reason):
        was_synced = self._synced
        self._synced = False
        self._publish_synced()
        self._no_ik_waiting_neutral = False
        self._arm_targets_by_seq.clear()
        self._direct_targets_by_seq.clear()
        self._arm_stream_active = False
        self._direct_stream_mode = None
        self._home_samples = 0
        self._gripper_hold_pending_seq = None
        self._gripper_stop_pending_seq = None
        self._gripper_close_command_active = False
        self._reset_gripper_contact_tracking(clear_latch=True)
        self._set_enabled(False, reason)
        if was_synced:
            self.get_logger().warn('%s; home is required' % reason)

    def _set_enabled(self, enabled, reason):
        if self._enabled == enabled:
            return
        self._enabled = enabled
        self._publish_enabled()
        self.get_logger().info(
            'teleop %s: %s' % ('enabled' if enabled else 'disabled', reason))

    def _publish_enabled(self):
        self._enabled_pub.publish(Bool(data=self._enabled))

    def _publish_synced(self):
        self._synced_pub.publish(Bool(data=self._synced))

    def _vla_enabled_callback(self, message):
        if message.data:
            self._lose_sync(
                'VLA acquired control; Home is required before teleop resumes')


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
