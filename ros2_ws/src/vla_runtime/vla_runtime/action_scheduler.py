"""Pure action-chunk to ArmCommand scheduling state machine."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np


MODE_STOP = 0
MODE_CARTESIAN_SERVO = 5
MODE_CARTESIAN_SERVO_END = 6
MODE_GRIPPER_STOP = 4
MODE_GRIPPER_SERVO = 8
MODE_WRIST_ROLL_SERVO = 10
MODE_WRIST_ROLL_SERVO_END = 11

PHASE_EXECUTING = 2
PHASE_COMPLETED = 3
PHASE_FAILED = 4

# Controller-mapped firmware error codes that are transient target rejections
# (NO_IK_SOLUTION=0x20, ARM_NOT_READY=0x21, STREAM_STEP_TOO_LARGE=0x26).  The
# controller reports these without latching ERROR; the VLA scheduler must drop
# the rejected target and keep going instead of stopping the heartbeat.
RECOVERABLE_ERROR_CODES = frozenset((0x20, 0x21, 0x26))


@dataclass(frozen=True)
class SchedulerConfig:
    action_scale: float
    action_abs_limits: tuple[float, float, float, float, float]
    action_deadbands: tuple[float, float, float, float, float]
    gripper_deadband: float
    gripper_max_step: float
    pitch_limits_deg: tuple[float, float]
    wrist_roll_limits_rad: tuple[float, float]
    stream_watchdog_sec: float


@dataclass(frozen=True)
class Target:
    x: float
    y: float
    z: float
    pitch: float
    wrist_roll: float
    gripper: float


@dataclass(frozen=True)
class CommandSpec:
    sequence_id: int
    mode: int
    target: Target
    duration_sec: float
    keepalive: bool = False


@dataclass(frozen=True)
class PlanResult:
    command: CommandSpec | None
    consume_action: bool


@dataclass(frozen=True)
class AckResult:
    matched: bool = False
    consume_action: bool = False
    failed: bool = False
    committed: bool = False
    rejected: bool = False
    recoverable: bool = False


@dataclass
class _Pending:
    sequence_id: int
    target: Target
    family: str | None
    consume_action: bool
    end_stream: bool
    keepalive: bool = False


class ActionScheduler:
    """Execute one dominant command family and commit only firmware ACKs."""

    def __init__(self, config):
        self.config = config
        self.target = None
        self.active_family = None
        self.pending = None
        self._next_sequence = 1

    def reset(self, home_target, wrist_roll, gripper, next_sequence=1):
        if len(home_target) != 4:
            raise ValueError("home_target must contain x, y, z, pitch")
        values = tuple(float(value) for value in home_target)
        values += (float(wrist_roll), float(gripper))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("initial target must be finite")
        self.target = Target(*values)
        self.active_family = None
        self.pending = None
        self._next_sequence = max(1, int(next_sequence) & 0xFFFFFFFF)

    def cancel(self):
        self.target = None
        self.active_family = None
        self.pending = None

    def stop_gripper(self):
        """Stop an acknowledged gripper stream without rewriting its target."""
        if self.target is None:
            raise RuntimeError("scheduler must be reset from Home first")
        if self.pending is not None or self.active_family != "gripper":
            return PlanResult(None, False)
        return self._end_stream()

    def keep_gripper_stream_open(self):
        """Refresh the installed gripper target without consuming an action."""
        if self.target is None:
            raise RuntimeError("scheduler must be reset from Home first")
        if self.pending is not None or self.active_family != "gripper":
            return PlanResult(None, False)
        command = CommandSpec(
            self._take_sequence(),
            MODE_GRIPPER_SERVO,
            self.target,
            self.config.stream_watchdog_sec,
            keepalive=True,
        )
        self.pending = _Pending(
            command.sequence_id,
            self.target,
            "gripper",
            False,
            False,
            keepalive=True,
        )
        return PlanResult(command, False)

    def _take_sequence(self):
        sequence = self._next_sequence
        self._next_sequence = (sequence + 1) & 0xFFFFFFFF
        if self._next_sequence == 0:
            self._next_sequence = 1
        return sequence

    def observe_sequence(self, sequence_id):
        sequence_id = int(sequence_id)
        if sequence_id <= 0:
            return
        candidate = (sequence_id + 1) & 0xFFFFFFFF
        if candidate == 0:
            candidate = 1
        self._next_sequence = max(self._next_sequence, candidate)

    def _scaled_action(self, action):
        values = np.asarray(action, dtype=np.float64).reshape(-1)
        if values.size < 6:
            raise ValueError("policy action must have at least six values")
        values = values[:6]
        if not np.isfinite(values).all():
            raise ValueError("policy action contains non-finite values")
        deltas = values[:5] * float(self.config.action_scale)
        limits = np.asarray(self.config.action_abs_limits, dtype=np.float64)
        deltas = np.clip(deltas, -limits, limits)
        return deltas, float(np.clip(values[5], 0.0, 1.0))

    def _desired_family(self, deltas, gripper):
        deadbands = self.config.action_deadbands
        cart_score = max(
            abs(float(deltas[index])) / max(deadbands[index], 1e-9)
            for index in range(4)
        )
        wrist_score = abs(float(deltas[4])) / max(deadbands[4], 1e-9)
        gripper_score = abs(gripper - self.target.gripper) / max(
            self.config.gripper_deadband, 1e-9
        )
        scores = {
            "cartesian": cart_score,
            "wrist": wrist_score,
            "gripper": gripper_score,
        }
        family = max(scores, key=scores.get)
        return family if scores[family] > 1.0 else None

    def _end_stream(self):
        modes = {
            "cartesian": MODE_CARTESIAN_SERVO_END,
            "gripper": MODE_GRIPPER_STOP,
            "wrist": MODE_WRIST_ROLL_SERVO_END,
        }
        mode = modes[self.active_family]
        command = CommandSpec(self._take_sequence(), mode, self.target, 0.0)
        self.pending = _Pending(
            command.sequence_id,
            self.target,
            self.active_family,
            False,
            True,
        )
        return PlanResult(command, False)

    def _keep_cartesian_stream_open(self):
        """Refresh an idle Cartesian stream without changing its target."""
        command = CommandSpec(
            self._take_sequence(),
            MODE_CARTESIAN_SERVO,
            self.target,
            self.config.stream_watchdog_sec,
            keepalive=True,
        )
        self.pending = _Pending(
            command.sequence_id,
            self.target,
            "cartesian",
            True,
            False,
            keepalive=True,
        )
        return PlanResult(command, False)

    def plan(self, action):
        if self.target is None:
            raise RuntimeError("scheduler must be reset from Home first")
        if self.pending is not None:
            return PlanResult(None, False)

        deltas, gripper = self._scaled_action(action)
        family = self._desired_family(deltas, gripper)
        if self.active_family == "cartesian" and family is None:
            # A below-deadband policy frame means "hold here", not "leave
            # Cartesian control".  Refreshing the same installed target keeps
            # the firmware watchdog alive without accumulating XYZ or forcing
            # an F transition that may take the full terminal window.
            return self._keep_cartesian_stream_open()
        if self.active_family is not None and family != self.active_family:
            return self._end_stream()
        if family is None:
            return PlanResult(None, True)

        target = self.target
        if family == "cartesian":
            target = Target(
                target.x + float(deltas[0]),
                target.y + float(deltas[1]),
                target.z + float(deltas[2]),
                float(
                    np.clip(
                        target.pitch + float(deltas[3]),
                        *self.config.pitch_limits_deg,
                    )
                ),
                target.wrist_roll,
                target.gripper,
            )
            mode = MODE_CARTESIAN_SERVO
        elif family == "wrist":
            target = Target(
                target.x,
                target.y,
                target.z,
                target.pitch,
                float(
                    np.clip(
                        target.wrist_roll + float(deltas[4]),
                        *self.config.wrist_roll_limits_rad,
                    )
                ),
                target.gripper,
            )
            mode = MODE_WRIST_ROLL_SERVO
        else:
            limited = float(
                np.clip(
                    gripper,
                    target.gripper - self.config.gripper_max_step,
                    target.gripper + self.config.gripper_max_step,
                )
            )
            target = Target(
                target.x,
                target.y,
                target.z,
                target.pitch,
                target.wrist_roll,
                limited,
            )
            mode = MODE_GRIPPER_SERVO

        command = CommandSpec(
            self._take_sequence(),
            mode,
            target,
            self.config.stream_watchdog_sec,
        )
        self.pending = _Pending(command.sequence_id, target, family, True, False)
        return PlanResult(command, False)

    def observe_lifecycle(self, sequence_id, phase, state, error_code):
        self.observe_sequence(sequence_id)
        if self.pending is None or int(sequence_id) != self.pending.sequence_id:
            return AckResult()
        if int(error_code) != 0 or int(phase) == PHASE_FAILED:
            self.pending = None
            self.active_family = None
            recoverable = int(error_code) in RECOVERABLE_ERROR_CODES
            return AckResult(
                matched=True,
                failed=not recoverable,
                rejected=True,
                recoverable=recoverable,
            )

        completed = int(phase) == PHASE_COMPLETED
        installed = int(phase) == PHASE_EXECUTING or completed
        committed = False
        if self.pending.end_stream:
            if not completed:
                return AckResult(matched=True)
            self.active_family = None
        elif not installed:
            return AckResult(matched=True)
        else:
            self.target = self.pending.target
            self.active_family = self.pending.family
            committed = True

        consume = self.pending.consume_action
        self.pending = None
        return AckResult(
            matched=True,
            consume_action=consume,
            committed=committed,
        )
