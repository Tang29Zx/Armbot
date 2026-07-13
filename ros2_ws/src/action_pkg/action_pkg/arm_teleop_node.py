#!/usr/bin/env python3
"""Xbox Cartesian teleoperation with synchronization and safety gates."""

from dataclasses import replace
import math
import time

from action_interfaces.msg import ArmCommand, ArmState
from action_pkg.teleop_mapping import (
    controls_neutral,
    integrate_target,
    joints_near_home,
    MODE_ARM,
    rising_edge,
    Target,
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
        self._expected_home_joints = [math.radians(v) for v in expected_deg]
        self._home_tolerance = math.radians(
            float(self._cfg('home_joint_tolerance_deg')))
        self._bounds = {
            'x': tuple(self._cfg('x_limits_cm')),
            'y': tuple(self._cfg('y_limits_cm')),
            'z': tuple(self._cfg('z_limits_cm')),
            'pitch': tuple(self._cfg('pitch_limits_deg')),
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
        self._joy_sub = self.create_subscription(
            Joy, joy_topic, self._joy_callback, 10)

        self._state_sub = None
        self._reset_client = None
        if not self._shadow:
            self._state_sub = self.create_subscription(
                ArmState, '/arm/state', self._state_callback, 10)
            self._reset_client = self.create_client(
                Trigger, '/arm/reset_error')

        self._enabled = False
        self._synced = self._shadow
        self._startup_sync_allowed = not self._shadow
        self._axes = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0]
        self._buttons = [0] * 16
        self._joy_valid = False
        self._last_joy_time = None
        self._last_state_time = None
        self._latest_state = None
        self._joy_timed_out = False
        self._state_timed_out = False
        self._home_samples = 0
        self._next_sequence = 1
        self._home_pending_seq = None
        self._reset_future = None
        self._shadow_estop_latched = False
        self._chord_started = {'reset': None, 'home': None}
        self._chord_triggered = {'reset': False, 'home': False}

        rate = float(self._cfg('control_rate_hz'))
        if rate <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        self._last_tick = time.monotonic()
        self._timer = self.create_timer(1.0 / rate, self._control_tick)
        self._publish_enabled()
        self.get_logger().info(
            'arm teleop started in %s mode; waiting for A enable'
            % ('shadow' if self._shadow else 'real'))

    def _declare_parameters(self):
        defaults = {
            'shadow_mode': False,
            'control_rate_hz': 10.0,
            'joy_timeout_sec': 0.5,
            'state_timeout_sec': 0.5,
            'deadzone': 0.12,
            'trigger_deadzone': 0.05,
            'translation_speed_cm_sec': 1.0,
            'pitch_speed_deg_sec': 10.0,
            'gripper_speed_sec': 0.5,
            'command_duration_sec': 0.12,
            'home_duration_sec': 2.0,
            'home_target': [15.0, 0.0, 2.0, -54.48],
            'home_joint_deg': [0.0, 112.08, -89.04, -77.52, 0.0],
            'home_joint_tolerance_deg': 5.0,
            'home_stable_samples': 3,
            'chord_hold_sec': 1.0,
            'x_limits_cm': [10.0, 20.0],
            'y_limits_cm': [-10.0, 10.0],
            'z_limits_cm': [0.0, 25.0],
            'pitch_limits_deg': [-90.0, 10.0],
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
            self._joy_valid = False
            self._lose_sync('invalid Joy shape or value')
            return

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
                self._startup_sync_allowed = False
                self._home_pending_seq = None
                self._lose_sync('B emergency stop')

        if rising_edge(buttons, previous, BUTTON_A):
            if self._enabled:
                self._set_enabled(False, 'A pause')
            else:
                reason = self._enable_block_reason(now)
                if reason:
                    self.get_logger().warn('enable rejected: %s' % reason)
                else:
                    self._set_enabled(True, 'A enable')

    def _state_callback(self, msg):
        self._latest_state = msg
        self._last_state_time = time.monotonic()
        self._state_timed_out = False
        self._next_sequence = max(
            self._next_sequence, (int(msg.sequence_id) + 1) & 0xFFFFFFFF)
        if self._next_sequence == 0:
            self._next_sequence = 1

        if math.isfinite(msg.gripper_position) and not self._enabled:
            self._target = replace(
                self._target,
                gripper=max(0.0, min(1.0, msg.gripper_position)),
            )

        if msg.state in (ArmState.STATE_ERROR, ArmState.STATE_ESTOP):
            self._startup_sync_allowed = False
            self._home_pending_seq = None
            self._lose_sync('arm state is ERROR/ESTOP')
            return

        if self._home_pending_seq is not None:
            if (msg.sequence_id == self._home_pending_seq
                    and msg.state == ArmState.STATE_SUCCEEDED):
                self._target = replace(
                    self._home_target, gripper=self._target.gripper)
                self._home_pending_seq = None
                self._synced = True
                self.get_logger().info('home completed; target synchronized')
            return

        healthy = msg.state in (ArmState.STATE_IDLE,
                                ArmState.STATE_SUCCEEDED)
        if (self._startup_sync_allowed and not self._synced
                and healthy and msg.position_valid
                and joints_near_home(msg.joint_position,
                                     self._expected_home_joints,
                                     self._home_tolerance)):
            self._home_samples += 1
            if self._home_samples >= int(self._cfg('home_stable_samples')):
                self._target = replace(
                    self._home_target, gripper=self._target.gripper)
                self._synced = True
                self.get_logger().info(
                    'firmware reset pose verified; target synchronized')
        else:
            self._home_samples = 0

    def _control_tick(self):
        now = time.monotonic()
        rate = float(self._cfg('control_rate_hz'))
        dt = min(now - self._last_tick, 2.0 / rate)
        self._last_tick = now

        self._check_timeouts(now)
        self._update_chords(now)
        if not self._enabled or not self._joy_valid:
            return

        updated, mode = integrate_target(
            self._target,
            self._axes,
            dt,
            deadzone=float(self._cfg('deadzone')),
            translation_speed=float(self._cfg('translation_speed_cm_sec')),
            pitch_speed=float(self._cfg('pitch_speed_deg_sec')),
            gripper_speed=float(self._cfg('gripper_speed_sec')),
            bounds=self._bounds,
            trigger_deadzone=float(self._cfg('trigger_deadzone')),
        )
        if mode is None or updated == self._target:
            return
        self._target = updated
        ros_mode = (ArmCommand.MODE_END_EFFECTOR
                    if mode == MODE_ARM else ArmCommand.MODE_GRIPPER)
        self._publish_command(
            ros_mode, self._target,
            float(self._cfg('command_duration_sec')))

    def _check_timeouts(self, now):
        joy_timeout = float(self._cfg('joy_timeout_sec'))
        if (self._last_joy_time is not None
                and now - self._last_joy_time > joy_timeout
                and not self._joy_timed_out):
            self._joy_timed_out = True
            self._joy_valid = False
            self._startup_sync_allowed = False
            self._lose_sync('Joy timeout')

        if self._shadow or self._last_state_time is None:
            return
        state_timeout = float(self._cfg('state_timeout_sec'))
        if (now - self._last_state_time > state_timeout
                and not self._state_timed_out):
            self._state_timed_out = True
            self._startup_sync_allowed = False
            self._lose_sync('ArmState timeout')

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
        self._startup_sync_allowed = False
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
                or self._home_pending_seq is not None):
            return
        if not self._shadow:
            reason = self._state_block_reason(time.monotonic())
            if reason:
                self.get_logger().warn('home rejected: %s' % reason)
                return

        target = replace(self._home_target, gripper=self._target.gripper)
        seq = self._publish_command(
            ArmCommand.MODE_END_EFFECTOR,
            target,
            float(self._cfg('home_duration_sec')))
        if self._shadow:
            self._target = target
            self._synced = True
            self.get_logger().info(
                'shadow home completed; target synchronized')
        else:
            self._home_pending_seq = seq
            self.get_logger().info('home command sent; waiting for completion')

    def _publish_command(self, mode, target, duration):
        msg = ArmCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = mode
        msg.x = target.x
        msg.y = target.y
        msg.z = target.z
        msg.pitch = target.pitch
        msg.gripper_position = target.gripper
        msg.duration_sec = duration
        msg.sequence_id = self._take_sequence()
        self._command_pub.publish(msg)
        return msg.sequence_id

    def _take_sequence(self):
        sequence = self._next_sequence
        self._next_sequence = (sequence + 1) & 0xFFFFFFFF
        if self._next_sequence == 0:
            self._next_sequence = 1
        return sequence

    def _enable_block_reason(self, now):
        if not self._synced:
            return 'target is not synchronized; run home first'
        if self._shadow_estop_latched:
            return 'shadow estop is latched'
        if not self._joy_valid or self._last_joy_time is None:
            return 'no valid Joy input'
        if now - self._last_joy_time > float(self._cfg('joy_timeout_sec')):
            return 'Joy input is stale'
        try:
            neutral = controls_neutral(
                self._axes,
                float(self._cfg('deadzone')),
                float(self._cfg('trigger_deadzone')))
        except ValueError:
            neutral = False
        if not neutral:
            return 'sticks and triggers must be neutral'
        if (self._home_pending_seq is not None
                or self._reset_future is not None):
            return 'home/reset operation is in progress'
        if not self._shadow:
            return self._state_block_reason(now)
        return ''

    def _state_block_reason(self, now):
        if self._latest_state is None or self._last_state_time is None:
            return 'no ArmState received'
        if now - self._last_state_time > float(self._cfg('state_timeout_sec')):
            return 'ArmState is stale'
        if not self._latest_state.position_valid:
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
        self._home_samples = 0
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
