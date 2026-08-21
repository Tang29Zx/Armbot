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

from .action_scheduler import (
    RECOVERABLE_ERROR_CODES,
    ActionScheduler,
    SchedulerConfig,
)
from .gripper_guard import (
    EVENT_CONTACT,
    EVENT_NO_PROGRESS,
    EVENT_TIMEOUT,
    GripperGuardConfig,
    GripperTransaction,
)
from .image_tools import decode_resize_with_pad
from .inference_logging import InferenceJsonlLogger
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
                        "log_context": request["log_context"],
                    }
                )
            except Exception as exc:
                self._put_latest(
                    {
                        "actions": None,
                        "observation_time": request["observation_time"],
                        "latency_sec": time.monotonic() - started,
                        "error": str(exc),
                        "log_context": request["log_context"],
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
        self._inference_log = None
        self._inference_log_error_reported = False
        self._last_lifecycle_audit_key = None
        self._recoverable_error_streak = 0
        self._published_command_audit = {}
        if bool(self._cfg("inference_logging_enabled")):
            self._inference_log = InferenceJsonlLogger(
                str(self._cfg("inference_log_path")),
                int(self._cfg("inference_log_max_bytes")),
                int(self._cfg("inference_log_backup_count")),
            )
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
        self._gripper_transaction = None

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
        self._gripper_guard_config = GripperGuardConfig(
            target_tolerance=float(self._cfg("gripper_target_tolerance")),
            min_progress=float(self._cfg("gripper_contact_min_progress")),
            stable_delta=float(self._cfg("gripper_contact_stable_delta")),
            contact_stable_sec=float(
                self._cfg("gripper_contact_stable_sec")
            ),
            keepalive_interval_sec=float(
                self._cfg("gripper_keepalive_interval_sec")
            ),
            no_progress_timeout_sec=float(
                self._cfg("gripper_no_progress_timeout_sec")
            ),
            transaction_timeout_sec=float(
                self._cfg("gripper_transaction_timeout_sec")
            ),
        )
        self._gripper_guard_config.validate(
            float(self._cfg("stream_watchdog_sec"))
        )

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
        if self._inference_log is not None:
            self._write_inference_log(
                {
                    "event": "session_start",
                    "mode": self._mode_name(),
                    "prompt": self._prompt,
                    "max_actions_per_chunk": self._max_actions,
                }
            )
            self.get_logger().info(
                "inference JSONL logging enabled at %s"
                % self._inference_log.path
            )

    def _declare_parameters(self):
        defaults = {
            "policy_host": "127.0.0.1",
            "policy_port": 8000,
            "prompt": "抓取药盒",
            "shadow_mode": True,
            "inference_logging_enabled": False,
            "inference_log_path": "/var/log/armbot/inference.jsonl",
            "inference_log_max_bytes": 52428800,
            "inference_log_backup_count": 5,
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
            "gripper_contact_stable_delta": 0.006,
            "gripper_contact_stable_sec": 2.0,
            "gripper_target_tolerance": 0.03,
            "gripper_keepalive_interval_sec": 0.05,
            "gripper_no_progress_timeout_sec": 0.6,
            "gripper_transaction_timeout_sec": 1.5,
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
        pending = self._scheduler.pending
        target_before = self._scheduler.target
        result = self._scheduler.observe_lifecycle(
            message.sequence_id,
            message.command_phase,
            message.state,
            message.error_code,
        )
        self._audit_lifecycle(message, result, pending, target_before)
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
        if pending is not None and pending.family == "gripper":
            if result.rejected:
                self._gripper_transaction = None
            elif result.committed:
                self._begin_gripper_transaction(message, pending)
        self._observe_gripper_transaction(message)

    def _on_vla_enabled(self, message):
        enabled = bool(message.data) and not self._shadow
        if enabled == self._vla_enabled:
            return
        self._vla_enabled = enabled
        self._queued_actions.clear()
        self._staged_actions = None
        self._published_command_audit.clear()
        self._last_lifecycle_audit_key = None
        self._recoverable_error_streak = 0
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
            and (state.error_code == 0
                 or state.error_code in RECOVERABLE_ERROR_CODES)
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
            or (raw.error_code != 0
                and raw.error_code not in RECOVERABLE_ERROR_CODES)
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
                "log_context": {
                    "submitted_at_unix_ns": time.time_ns(),
                    "image_bytes": len(self._latest_jpeg),
                    "observation_state": observation_state.tolist(),
                    "arm_state": int(state.state),
                    "command_phase": int(state.command_phase),
                    "sequence_id": int(state.sequence_id),
                    "position_valid": bool(state.position_valid),
                    "error_code": int(state.error_code),
                },
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
        self._write_inference_result(result)
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
        if (
            self._scheduler.pending is None
            and self._gripper_transaction is None
        ):
            self._queued_actions = staged
        else:
            self._staged_actions = staged

    def _mode_name(self):
        return "shadow" if self._shadow else "command"

    def _write_inference_result(self, result):
        if self._inference_log is None:
            return
        context = result["log_context"]
        record = {
            "event": (
                "inference_error"
                if result["error"] is not None
                else "inference_result"
            ),
            "mode": self._mode_name(),
            "vla_enabled": bool(self._vla_enabled),
            "prompt": self._prompt,
            "latency_ms": float(result["latency_sec"] * 1000.0),
            "observation_age_ms": float(
                (time.monotonic() - result["observation_time"]) * 1000.0
            ),
            **context,
        }
        if result["error"] is not None:
            record["error"] = result["error"]
        else:
            actions = result["actions"]
            record.update(
                {
                    "action_shape": list(actions.shape),
                    "selected_action_count": min(
                        self._max_actions, int(actions.shape[0])
                    ),
                    "action_abs_max": np.max(np.abs(actions), axis=0).tolist(),
                    "action_mean_abs": np.mean(np.abs(actions), axis=0).tolist(),
                    "actions": actions.tolist(),
                }
            )
        self._write_inference_log(record)

    def _write_inference_log(self, record):
        if self._inference_log is None:
            return
        try:
            self._inference_log.write(record)
        except (OSError, TypeError, ValueError) as exc:
            # Audit logging must never destabilize the safety/control loop.
            if not self._inference_log_error_reported:
                self.get_logger().error(
                    "inference JSONL write failed; control continues: %s" % exc
                )
                self._inference_log_error_reported = True

    @staticmethod
    def _target_audit_record(target):
        if target is None:
            return None
        return {
            "x": float(target.x),
            "y": float(target.y),
            "z": float(target.z),
            "pitch": float(target.pitch),
            "wrist_roll": float(target.wrist_roll),
            "gripper": float(target.gripper),
        }

    @staticmethod
    def _action_audit_record(action):
        if action is None:
            return None
        return np.asarray(action, dtype=np.float64).reshape(-1)[:6].tolist()

    def _publish_command(
        self,
        spec,
        *,
        source,
        target_before,
        active_family_before,
        policy_action=None,
        effective_action=None,
    ):
        self._command_pub.publish(self._make_command(spec))
        audit = {
            "command_mode": int(spec.mode),
            "source": str(source),
            "keepalive": bool(spec.keepalive),
        }
        self._published_command_audit[int(spec.sequence_id)] = audit
        self._write_inference_log(
            {
                "event": "command_published",
                "mode": self._mode_name(),
                "sequence_id": int(spec.sequence_id),
                "command_mode": int(spec.mode),
                "source": str(source),
                "keepalive": bool(spec.keepalive),
                "duration_sec": float(spec.duration_sec),
                "active_family_before": active_family_before,
                "policy_action": self._action_audit_record(policy_action),
                "effective_action": self._action_audit_record(
                    effective_action
                ),
                "target_before": self._target_audit_record(target_before),
                "target_candidate": self._target_audit_record(spec.target),
            }
        )

    def _audit_lifecycle(self, message, result, pending, target_before):
        if not self._vla_enabled and not result.matched:
            return
        pending_sequence = (
            int(pending.sequence_id) if pending is not None else None
        )
        command_audit = self._published_command_audit.get(
            int(message.sequence_id), {}
        )
        audit_key = (
            int(message.sequence_id),
            int(message.command_phase),
            int(message.state),
            int(message.error_code),
            pending_sequence,
        )
        if audit_key != self._last_lifecycle_audit_key:
            self._last_lifecycle_audit_key = audit_key
            self._write_inference_log(
                {
                    "event": "lifecycle_observed",
                    "mode": self._mode_name(),
                    "sequence_id": int(message.sequence_id),
                    "command_mode": command_audit.get("command_mode"),
                    "command_phase": int(message.command_phase),
                    "arm_state": int(message.state),
                    "error_code": int(message.error_code),
                    "matched_pending": bool(result.matched),
                    "pending_sequence_id": pending_sequence,
                }
            )

        if not result.matched or pending is None:
            return

        event_common = {
            "mode": self._mode_name(),
            "sequence_id": int(message.sequence_id),
            "command_mode": command_audit.get("command_mode"),
            "source": command_audit.get("source"),
            "keepalive": bool(pending.keepalive),
            "error_code": int(message.error_code),
        }
        if result.rejected:
            self._write_inference_log(
                {
                    "event": "target_rejected",
                    **event_common,
                    "recoverable": bool(result.recoverable),
                    "target_candidate": self._target_audit_record(
                        pending.target
                    ),
                    "target_retained": self._target_audit_record(
                        self._scheduler.target
                    ),
                }
            )
            if result.recoverable:
                self._recoverable_error_streak += 1
                self._write_inference_log(
                    {
                        "event": "recoverable_error",
                        **event_common,
                        "consecutive_count": self._recoverable_error_streak,
                    }
                )
            else:
                self._recoverable_error_streak = 0
        elif result.committed:
            self._recoverable_error_streak = 0
            self._write_inference_log(
                {
                    "event": "target_committed",
                    **event_common,
                    "target_before": self._target_audit_record(target_before),
                    "target_committed": self._target_audit_record(
                        self._scheduler.target
                    ),
                }
            )
        elif int(message.error_code) == 0:
            self._recoverable_error_streak = 0

        if self._scheduler.pending is None:
            self._published_command_audit.pop(int(message.sequence_id), None)

    def _control_tick(self):
        self._drain_policy_result()
        if self._shadow or not self._vla_enabled:
            return
        now = time.monotonic()
        if not self._control_healthy(now):
            return
        if self._scheduler.pending is not None:
            return
        if self._gripper_transaction is not None:
            self._gripper_transaction_tick(now)
            return
        if self._staged_actions is not None:
            self._queued_actions = self._staged_actions
            self._staged_actions = None

        target_before = self._scheduler.target
        active_family_before = self._scheduler.active_family
        if self._queued_actions:
            policy_action = np.asarray(
                self._queued_actions[0], dtype=np.float32
            ).copy()
            action = self._filter_contact_action(policy_action)
            source = "policy"
        else:
            target = self._scheduler.target
            gripper = target.gripper if target is not None else 0.0
            action = np.asarray([0, 0, 0, 0, 0, gripper], dtype=np.float32)
            policy_action = None
            source = "idle_hold"
        try:
            result = self._scheduler.plan(action)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error("VLA action rejected: %s" % exc)
            self._queued_actions.clear()
            return
        if result.consume_action and self._queued_actions:
            self._queued_actions.popleft()
        if result.command is not None:
            self._publish_command(
                result.command,
                source=source,
                target_before=target_before,
                active_family_before=active_family_before,
                policy_action=policy_action,
                effective_action=action,
            )

    def _begin_gripper_transaction(self, state, pending):
        if pending.keepalive or self._gripper_transaction is not None:
            return
        target = self._scheduler.target
        if target is None or not state.position_valid:
            return
        actual = float(np.clip(state.gripper_position, 0.0, 1.0))
        if not np.isfinite(actual):
            return
        now = self._raw_state_received_at
        if now is None:
            now = time.monotonic()
        self._gripper_transaction = GripperTransaction(
            target.gripper,
            actual,
            now,
            self._gripper_guard_config,
        )
        self._write_inference_log(
            {
                "event": "gripper_transaction_started",
                "mode": self._mode_name(),
                "sequence_id": int(state.sequence_id),
                "target_gripper": float(target.gripper),
                "actual_gripper": actual,
                "direction": (
                    "close"
                    if self._gripper_transaction.direction > 0.0
                    else "open"
                ),
            }
        )

    def _observe_gripper_transaction(self, state):
        transaction = self._gripper_transaction
        if transaction is None:
            return
        if (
            self._shadow
            or not self._vla_enabled
            or not state.position_valid
            or self._scheduler.target is None
        ):
            return
        actual = float(np.clip(state.gripper_position, 0.0, 1.0))
        if not np.isfinite(actual):
            return
        now = self._raw_state_received_at
        if now is None:
            now = time.monotonic()
        observation = transaction.observe(actual, now)
        if observation.event is None or self._scheduler.pending is not None:
            return
        self._finish_gripper_transaction(observation)

    def _gripper_transaction_tick(self, now):
        transaction = self._gripper_transaction
        if transaction is None or not transaction.keepalive_due(now):
            return
        target_before = self._scheduler.target
        active_family_before = self._scheduler.active_family
        result = self._scheduler.keep_gripper_stream_open()
        if result.command is None:
            return
        transaction.mark_keepalive(now)
        self._publish_command(
            result.command,
            source="gripper_keepalive",
            target_before=target_before,
            active_family_before=active_family_before,
        )

    def _finish_gripper_transaction(self, observation):
        transaction = self._gripper_transaction
        if transaction is None:
            return
        target_before = self._scheduler.target
        active_family_before = self._scheduler.active_family
        result = self._scheduler.stop_gripper()
        if result.command is None:
            return
        self._gripper_transaction = None
        is_contact = observation.event == EVENT_CONTACT
        is_fault = observation.event in (EVENT_NO_PROGRESS, EVENT_TIMEOUT)
        if is_contact:
            self._gripper_contact_latched = True
        self._publish_command(
            result.command,
            source="gripper_%s" % observation.event,
            target_before=target_before,
            active_family_before=active_family_before,
        )
        self._write_inference_log(
            {
                "event": "target_retained",
                "mode": self._mode_name(),
                "sequence_id": int(result.command.sequence_id),
                "reason": "gripper_%s" % observation.event,
                "target_before": self._target_audit_record(target_before),
                "target_retained": self._target_audit_record(
                    self._scheduler.target
                ),
                "requested_gripper": float(transaction.target),
                "actual_gripper": float(observation.actual),
                "gap": float(observation.gap),
                "progress": float(observation.progress),
                "stable_elapsed_sec": float(
                    observation.stable_elapsed_sec
                ),
                "elapsed_sec": float(observation.elapsed_sec),
            }
        )
        if is_fault:
            self._write_inference_log(
                {
                    "event": "no_progress_fault",
                    "mode": self._mode_name(),
                    "sequence_id": int(result.command.sequence_id),
                    "reason": "gripper_%s" % observation.event,
                    "requested_gripper": float(transaction.target),
                    "actual_gripper": float(observation.actual),
                    "origin_gripper": float(transaction.origin),
                    "progress": float(observation.progress),
                    "elapsed_sec": float(observation.elapsed_sec),
                }
            )
            self._control_fault_latched = True
            self._queued_actions.clear()
            self._staged_actions = None
            self.get_logger().error(
                "gripper %s at feedback %.3f; bounded stop sent"
                % (observation.event, observation.actual)
            )
        elif is_contact:
            self.get_logger().info(
                "gripper contact detected; bounded stop at feedback %.3f"
                % observation.actual
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
        self._gripper_transaction = None
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
        if self._inference_log is not None:
            self._write_inference_log(
                {"event": "session_end", "mode": self._mode_name()}
            )
            self._inference_log.close()
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
