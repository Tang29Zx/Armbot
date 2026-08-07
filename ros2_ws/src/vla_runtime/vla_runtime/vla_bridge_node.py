"""ROS 2 bridge between RDK DDS topics and an OpenPI policy server."""

from __future__ import annotations

from collections import deque
import queue
import threading
import time

from action_interfaces.msg import ArmCommand, ArmState
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Empty

from .action_scheduler import ActionScheduler, SchedulerConfig
from .image_tools import decode_resize_with_pad
from .policy_client import PolicyClient


class _PolicyWorker:
    def __init__(self, client):
        self._client = client
        self._requests = queue.Queue(maxsize=1)
        self.results = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="openpi-policy", daemon=True
        )
        self._thread.start()

    def submit(self, request):
        try:
            self._requests.put_nowait(request)
            return True
        except queue.Full:
            return False

    def _put_latest(self, result):
        try:
            self.results.put_nowait(result)
        except queue.Full:
            try:
                self.results.get_nowait()
            except queue.Empty:
                pass
            self.results.put_nowait(result)

    def _run(self):
        while not self._stop.is_set():
            try:
                request = self._requests.get(timeout=0.1)
            except queue.Empty:
                continue
            started = time.monotonic()
            try:
                image = decode_resize_with_pad(request["jpeg"])
                response = self._client.infer(
                    {
                        "observation/image": image,
                        "observation/state": request["state"],
                        "prompt": request["prompt"],
                    }
                )
                actions = np.asarray(response["actions"], dtype=np.float32)
                if actions.ndim != 2 or actions.shape[0] == 0 or actions.shape[1] < 6:
                    raise ValueError(
                        "expected policy actions [horizon, >=6], got %s"
                        % (actions.shape,)
                    )
                if not np.isfinite(actions).all():
                    raise ValueError("policy returned non-finite actions")
                self._put_latest(
                    {
                        "actions": actions[:, :6].copy(),
                        "observation_time": request["observation_time"],
                        "latency_sec": time.monotonic() - started,
                        "error": None,
                    }
                )
            except Exception as exc:
                self._put_latest(
                    {
                        "actions": None,
                        "observation_time": request["observation_time"],
                        "latency_sec": time.monotonic() - started,
                        "error": str(exc),
                    }
                )

    def close(self):
        self._stop.set()
        self._client.close()
        self._thread.join(timeout=2.0)


class VlaBridgeNode(Node):
    def __init__(self):
        super().__init__("vla_bridge_node")
        self._declare_parameters()

        self._shadow = bool(self._cfg("shadow_mode"))
        self._prompt = str(self._cfg("prompt"))
        self._max_actions = int(self._cfg("max_actions_per_chunk"))
        self._latest_jpeg = None
        self._image_received_at = None
        self._latest_state = None
        self._state_received_at = None
        self._latest_raw_state = None
        self._raw_state_received_at = None
        self._last_policy_success = None
        self._last_policy_error = None
        self._last_shadow_log = 0.0
        self._request_outstanding = False
        self._vla_enabled = False
        self._control_fault_latched = False
        self._queued_actions = deque()
        self._staged_actions = None
        self._gripper_contact_latched = False
        self._gripper_close_origin = None
        self._gripper_last_feedback = None
        self._gripper_close_progress = False
        self._gripper_contact_samples = 0

        scheduler_config = SchedulerConfig(
            action_scale=float(self._cfg("action_scale")),
            action_abs_limits=tuple(float(v) for v in self._cfg("action_abs_limits")),
            action_deadbands=tuple(float(v) for v in self._cfg("action_deadbands")),
            gripper_deadband=float(self._cfg("gripper_deadband")),
            gripper_max_step=float(self._cfg("gripper_max_step")),
            pitch_limits_deg=tuple(float(v) for v in self._cfg("pitch_limits_deg")),
            wrist_roll_limits_rad=tuple(
                float(v) for v in self._cfg("wrist_roll_limits_rad")
            ),
            stream_watchdog_sec=float(self._cfg("stream_watchdog_sec")),
        )
        self._scheduler = ActionScheduler(scheduler_config)

        self._command_pub = self.create_publisher(
            ArmCommand, str(self._cfg("command_topic")), 10
        )
        heartbeat_qos = QoSProfile(depth=1)
        heartbeat_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._heartbeat_pub = self.create_publisher(
            Empty, str(self._cfg("heartbeat_topic")), heartbeat_qos
        )

        enabled_qos = QoSProfile(depth=1)
        enabled_qos.reliability = ReliabilityPolicy.RELIABLE
        enabled_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._enabled_sub = self.create_subscription(
            Bool,
            str(self._cfg("vla_enabled_topic")),
            self._on_vla_enabled,
            enabled_qos,
        )
        self._image_sub = self.create_subscription(
            CompressedImage,
            str(self._cfg("image_topic")),
            self._on_image,
            qos_profile_sensor_data,
        )
        self._state_sub = self.create_subscription(
            ArmState,
            str(self._cfg("state_topic")),
            self._on_state,
            10,
        )
        self._raw_state_sub = self.create_subscription(
            ArmState,
            str(self._cfg("raw_state_topic")),
            self._on_raw_state,
            10,
        )

        client = PolicyClient(
            str(self._cfg("policy_host")),
            int(self._cfg("policy_port")),
            float(self._cfg("policy_timeout_sec")),
        )
        self._worker = _PolicyWorker(client)
        self._policy_timer = self.create_timer(
            1.0 / float(self._cfg("policy_rate_hz")),
            self._policy_tick,
        )
        self._control_timer = self.create_timer(
            1.0 / float(self._cfg("control_rate_hz")),
            self._control_tick,
        )
        self._heartbeat_timer = self.create_timer(0.05, self._heartbeat_tick)
        self.get_logger().info(
            "VLA bridge started in %s mode; policy=%s"
            % ("SHADOW" if self._shadow else "COMMAND", client.uri)
        )

    def _declare_parameters(self):
        defaults = {
            "policy_host": "127.0.0.1",
            "policy_port": 8000,
            "prompt": "抓取药盒",
            "shadow_mode": True,
            "image_topic": "/vla/image",
            "state_topic": "/arm/state_filtered",
            "raw_state_topic": "/arm/state",
            "command_topic": "/arm/command/vla",
            "heartbeat_topic": "/vla/heartbeat",
            "vla_enabled_topic": "/arm/vla_enabled",
            "control_rate_hz": 10.0,
            "policy_rate_hz": 2.0,
            "max_actions_per_chunk": 4,
            "max_image_age_sec": 0.5,
            "max_state_age_sec": 0.25,
            "max_policy_age_sec": 1.5,
            "policy_timeout_sec": 2.0,
            "stream_watchdog_sec": 0.3,
            "action_scale": 0.5,
            "action_abs_limits": [0.31, 0.31, 0.31, 0.0, 0.035],
            "action_deadbands": [0.015, 0.015, 0.015, 0.01, 0.003],
            "gripper_deadband": 0.02,
            "gripper_max_step": 0.075,
            "gripper_contact_min_progress": 0.02,
            "gripper_contact_min_gap": 0.04,
            "gripper_contact_stable_delta": 0.006,
            "gripper_contact_stable_samples": 3,
            "home_target": [15.0, 0.0, 2.0, -54.48],
            "pitch_limits_deg": [-90.0, 90.0],
            "wrist_roll_limits_rad": [-1.5707963268, 1.5707963268],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _cfg(self, name):
        return self.get_parameter(name).value

    def _on_image(self, message):
        self._latest_jpeg = bytes(message.data)
        self._image_received_at = time.monotonic()

    def _on_state(self, message):
        self._latest_state = message
        self._state_received_at = time.monotonic()

    def _on_raw_state(self, message):
        self._latest_raw_state = message
        self._raw_state_received_at = time.monotonic()
        result = self._scheduler.observe_lifecycle(
            message.sequence_id,
            message.command_phase,
            message.state,
            message.error_code,
        )
        if result.failed:
            self._control_fault_latched = True
            self._queued_actions.clear()
            self._staged_actions = None
            self.get_logger().error(
                "VLA command seq=%d failed; control heartbeat will stop"
                % message.sequence_id
            )
        elif result.consume_action and self._queued_actions:
            self._queued_actions.popleft()
        self._observe_gripper_contact(message)

    def _on_vla_enabled(self, message):
        enabled = bool(message.data) and not self._shadow
        if enabled == self._vla_enabled:
            return
        self._vla_enabled = enabled
        self._queued_actions.clear()
        self._staged_actions = None
        self._reset_gripper_contact(clear_latch=True)
        if not enabled:
            self._scheduler.cancel()
            self.get_logger().info("VLA command execution disabled")
            return
        state = self._latest_raw_state
        if state is None or not state.position_valid:
            self.get_logger().error(
                "VLA enabled without valid ArmState; withholding heartbeat"
            )
            return
        self._control_fault_latched = False
        home = tuple(float(v) for v in self._cfg("home_target"))
        self._scheduler.reset(
            home,
            float(state.joint_position[4]),
            float(state.gripper_position),
            int(state.sequence_id) + 1,
        )
        self.get_logger().warn(
            "VLA command execution enabled at half-scale safety defaults"
        )

    def _observation_fresh(self, now):
        if self._latest_jpeg is None or self._latest_state is None:
            return False
        if self._image_received_at is None or self._state_received_at is None:
            return False
        if now - self._image_received_at > float(self._cfg("max_image_age_sec")):
            return False
        if now - self._state_received_at > float(self._cfg("max_state_age_sec")):
            return False
        state = self._latest_state
        return (
            state.position_valid
            and state.state not in (ArmState.STATE_ERROR, ArmState.STATE_ESTOP)
            and state.error_code == 0
        )

    def _control_healthy(self, now):
        if not self._observation_fresh(now):
            return False
        if self._vla_enabled and (
            self._control_fault_latched or self._scheduler.target is None
        ):
            return False
        raw = self._latest_raw_state
        if raw is None or self._raw_state_received_at is None:
            return False
        if now - self._raw_state_received_at > float(self._cfg("max_state_age_sec")):
            return False
        if (
            not raw.position_valid
            or raw.state in (ArmState.STATE_ERROR, ArmState.STATE_ESTOP)
            or raw.error_code != 0
        ):
            return False
        if self._last_policy_success is None:
            return False
        if self._last_policy_error is not None:
            return False
        return now - self._last_policy_success <= float(self._cfg("max_policy_age_sec"))

    def _policy_tick(self):
        now = time.monotonic()
        if self._request_outstanding or not self._observation_fresh(now):
            return
        state = self._latest_state
        observation_state = np.asarray(
            [*state.joint_position[:5], state.gripper_position],
            dtype=np.float32,
        )
        submitted = self._worker.submit(
            {
                "jpeg": self._latest_jpeg,
                "state": observation_state,
                "prompt": self._prompt,
                "observation_time": min(
                    self._image_received_at, self._state_received_at
                ),
            }
        )
        if submitted:
            self._request_outstanding = True

    def _drain_policy_result(self):
        result = None
        while True:
            try:
                result = self._worker.results.get_nowait()
            except queue.Empty:
                break
        if result is None:
            return
        self._request_outstanding = False
        if result["error"] is not None:
            self._queued_actions.clear()
            self._staged_actions = None
            if result["error"] != self._last_policy_error:
                self.get_logger().error("OpenPI inference failed: %s" % result["error"])
                self._last_policy_error = result["error"]
            return

        self._last_policy_error = None
        self._last_policy_success = result["observation_time"]
        actions = result["actions"][: self._max_actions]
        if self._shadow:
            now = time.monotonic()
            if now - self._last_shadow_log >= 1.0:
                self.get_logger().info(
                    "shadow inference %.0f ms first_action=%s"
                    % (
                        result["latency_sec"] * 1000.0,
                        np.array2string(actions[0], precision=4),
                    )
                )
                self._last_shadow_log = now
            return
        staged = deque(action.copy() for action in actions)
        if self._scheduler.pending is None:
            self._queued_actions = staged
        else:
            self._staged_actions = staged

    def _control_tick(self):
        self._drain_policy_result()
        if self._shadow or not self._vla_enabled:
            return
        if not self._control_healthy(time.monotonic()):
            return
        if self._scheduler.pending is not None:
            return
        if self._staged_actions is not None:
            self._queued_actions = self._staged_actions
            self._staged_actions = None

        if self._queued_actions:
            action = self._filter_contact_action(self._queued_actions[0])
        else:
            target = self._scheduler.target
            gripper = target.gripper if target is not None else 0.0
            action = np.asarray([0, 0, 0, 0, 0, gripper], dtype=np.float32)
        try:
            result = self._scheduler.plan(action)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error("VLA action rejected: %s" % exc)
            self._queued_actions.clear()
            return
        if result.consume_action and self._queued_actions:
            self._queued_actions.popleft()
        if result.command is not None:
            self._command_pub.publish(self._make_command(result.command))

    def _observe_gripper_contact(self, state):
        if self._scheduler.active_family != "gripper":
            self._reset_gripper_contact(clear_latch=False)
            return
        if (
            self._shadow
            or not self._vla_enabled
            or not state.position_valid
            or self._scheduler.pending is not None
            or self._scheduler.target is None
        ):
            return
        actual = float(np.clip(state.gripper_position, 0.0, 1.0))
        if not np.isfinite(actual):
            self._reset_gripper_contact(clear_latch=False)
            return
        if self._gripper_contact_latched:
            return
        if self._gripper_close_origin is None:
            self._gripper_close_origin = actual
            self._gripper_last_feedback = actual
            return
        if actual - self._gripper_close_origin >= float(
            self._cfg("gripper_contact_min_progress")
        ):
            self._gripper_close_progress = True
        stable = abs(actual - self._gripper_last_feedback) <= float(
            self._cfg("gripper_contact_stable_delta")
        )
        gap = self._scheduler.target.gripper - actual
        if (
            self._gripper_close_progress
            and stable
            and gap >= float(self._cfg("gripper_contact_min_gap"))
        ):
            self._gripper_contact_samples += 1
        else:
            self._gripper_contact_samples = 0
        self._gripper_last_feedback = actual
        if self._gripper_contact_samples < int(
            self._cfg("gripper_contact_stable_samples")
        ):
            return
        result = self._scheduler.hold_gripper(actual)
        if result.command is None:
            return
        self._gripper_contact_latched = True
        self._command_pub.publish(self._make_command(result.command))
        self.get_logger().info(
            "gripper contact detected; bounded stop at feedback %.3f" % actual
        )

    def _filter_contact_action(self, action):
        filtered = np.asarray(action, dtype=np.float32).copy()
        if not self._gripper_contact_latched or self._scheduler.target is None:
            return filtered
        held = self._scheduler.target.gripper
        if filtered[5] >= held - float(self._cfg("gripper_deadband")):
            filtered[5] = held
        else:
            self._reset_gripper_contact(clear_latch=True)
        return filtered

    def _reset_gripper_contact(self, clear_latch):
        self._gripper_close_origin = None
        self._gripper_last_feedback = None
        self._gripper_close_progress = False
        self._gripper_contact_samples = 0
        if clear_latch:
            self._gripper_contact_latched = False

    @staticmethod
    def _make_command(spec):
        message = ArmCommand()
        message.mode = spec.mode
        message.x = spec.target.x
        message.y = spec.target.y
        message.z = spec.target.z
        message.pitch = spec.target.pitch
        message.joint_position[4] = spec.target.wrist_roll
        message.gripper_position = spec.target.gripper
        message.duration_sec = spec.duration_sec
        message.sequence_id = spec.sequence_id
        return message

    def _heartbeat_tick(self):
        # Shadow mode must be impossible to acquire at the RDK mux. Publishing
        # a healthy heartbeat here would let the enable service succeed even
        # though this process intentionally never emits commands.
        if self._shadow:
            return
        now = time.monotonic()
        healthy = self._control_healthy(now)
        if healthy:
            self._heartbeat_pub.publish(Empty())

    def destroy_node(self):
        self._worker.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VlaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
