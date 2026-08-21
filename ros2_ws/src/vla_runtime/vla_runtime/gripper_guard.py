"""Feedback-driven completion guard for bounded gripper stream targets."""

from __future__ import annotations

from dataclasses import dataclass
import math


EVENT_REACHED = "target_reached"
EVENT_CONTACT = "contact"
EVENT_NO_PROGRESS = "no_progress"
EVENT_TIMEOUT = "transaction_timeout"


@dataclass(frozen=True)
class GripperGuardConfig:
    target_tolerance: float
    min_progress: float
    stable_delta: float
    contact_stable_sec: float
    keepalive_interval_sec: float
    no_progress_timeout_sec: float
    transaction_timeout_sec: float

    def validate(self, stream_watchdog_sec):
        if self.target_tolerance < 0.0:
            raise ValueError("gripper target tolerance must be non-negative")
        if self.min_progress <= 0.0:
            raise ValueError("gripper minimum progress must be positive")
        if self.stable_delta < 0.0 or self.contact_stable_sec <= 0.0:
            raise ValueError("gripper stability settings must be positive")
        if not 0.0 < self.keepalive_interval_sec < stream_watchdog_sec:
            raise ValueError(
                "gripper keepalive interval must be within the watchdog"
            )
        if self.no_progress_timeout_sec <= self.keepalive_interval_sec:
            raise ValueError(
                "gripper no-progress timeout must exceed keepalive interval"
            )
        if self.transaction_timeout_sec < self.no_progress_timeout_sec:
            raise ValueError(
                "gripper transaction timeout must cover no-progress timeout"
            )


@dataclass(frozen=True)
class GripperObservation:
    event: str | None
    actual: float
    gap: float
    progress: float
    elapsed_sec: float
    progressed: bool
    stable_elapsed_sec: float
    stable_samples: int


class GripperTransaction:
    """Track one installed gripper target until feedback resolves it."""

    def __init__(self, target, actual, started_at, config):
        values = (float(target), float(actual), float(started_at))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("gripper transaction values must be finite")
        self.target = values[0]
        self.origin = values[1]
        self.started_at = values[2]
        self.direction = 1.0 if self.target >= self.origin else -1.0
        self.config = config
        self.last_feedback = self.origin
        self.stable_anchor = self.origin
        self.stable_since = self.started_at
        self.progressed = False
        self.stable_samples = 0
        # The firmware target was installed before its EXECUTING lifecycle
        # reached the bridge.  Make the first refresh due immediately so the
        # next control tick does not spend another interval of the 300 ms
        # firmware watchdog budget.
        self.last_keepalive_at = (
            self.started_at - self.config.keepalive_interval_sec
        )

    def observe(self, actual, now):
        actual = float(actual)
        now = float(now)
        if not math.isfinite(actual) or not math.isfinite(now):
            raise ValueError("gripper feedback and time must be finite")

        progress = self.direction * (actual - self.origin)
        gap = self.direction * (self.target - actual)
        elapsed = max(0.0, now - self.started_at)
        if progress >= self.config.min_progress:
            self.progressed = True

        stable = abs(actual - self.stable_anchor) <= self.config.stable_delta
        if stable:
            self.stable_samples += 1
        else:
            self.stable_anchor = actual
            self.stable_since = now
            self.stable_samples = 0
        self.last_feedback = actual
        stable_elapsed = max(0.0, now - self.stable_since)

        event = None
        if gap <= self.config.target_tolerance:
            event = EVENT_REACHED
        elif (
            self.direction > 0.0
            and stable_elapsed >= self.config.contact_stable_sec
        ):
            event = EVENT_CONTACT

        if (
            event is None
            and self.direction < 0.0
            and not self.progressed
            and elapsed >= self.config.no_progress_timeout_sec
        ):
            event = EVENT_NO_PROGRESS
        if (
            event is None
            and self.direction < 0.0
            and elapsed >= self.config.transaction_timeout_sec
        ):
            event = EVENT_TIMEOUT

        return GripperObservation(
            event=event,
            actual=actual,
            gap=gap,
            progress=progress,
            elapsed_sec=elapsed,
            progressed=self.progressed,
            stable_elapsed_sec=stable_elapsed,
            stable_samples=self.stable_samples,
        )

    def keepalive_due(self, now):
        return (
            float(now) - self.last_keepalive_at
            >= self.config.keepalive_interval_sec
        )

    def mark_keepalive(self, now):
        self.last_keepalive_at = float(now)
