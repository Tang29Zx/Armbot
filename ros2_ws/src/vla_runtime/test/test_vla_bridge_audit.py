from types import SimpleNamespace

import numpy as np

from vla_runtime.action_scheduler import (
    ActionScheduler,
    MODE_CARTESIAN_SERVO,
    MODE_GRIPPER_SERVO,
    MODE_GRIPPER_STOP,
    PHASE_EXECUTING,
    SchedulerConfig,
)
from vla_runtime.gripper_guard import (
    EVENT_CONTACT,
    EVENT_NO_PROGRESS,
    GripperGuardConfig,
    GripperTransaction,
)
from vla_runtime.vla_bridge_node import VlaBridgeNode


def _scheduler():
    scheduler = ActionScheduler(
        SchedulerConfig(
            action_scale=0.5,
            action_abs_limits=(0.31, 0.31, 0.31, 0.0, 0.035),
            action_deadbands=(0.015, 0.015, 0.015, 0.01, 0.003),
            gripper_deadband=0.02,
            gripper_max_step=0.075,
            pitch_limits_deg=(-90.0, 90.0),
            wrist_roll_limits_rad=(-1.57, 1.57),
            stream_watchdog_sec=0.3,
        )
    )
    scheduler.reset((15.0, 0.0, 2.0, -54.48), 0.0, 0.0, 8)
    return scheduler


class _AuditHarness:
    _target_audit_record = staticmethod(VlaBridgeNode._target_audit_record)
    _action_audit_record = staticmethod(VlaBridgeNode._action_audit_record)
    _make_command = staticmethod(lambda spec: spec)
    _publish_command = VlaBridgeNode._publish_command

    def __init__(self, scheduler):
        self._scheduler = scheduler
        self._vla_enabled = True
        self._last_lifecycle_audit_key = None
        self._recoverable_error_streak = 0
        self._published_command_audit = {}
        self.records = []
        self.published = []
        self._command_pub = SimpleNamespace(publish=self.published.append)
        self._gripper_transaction = None
        self._gripper_contact_latched = False
        self._control_fault_latched = False
        self._queued_actions = []
        self._staged_actions = None
        self.log_messages = []

    @staticmethod
    def _mode_name():
        return "command"

    def _write_inference_log(self, record):
        self.records.append(record)

    def get_logger(self):
        return SimpleNamespace(
            info=lambda message: self.log_messages.append(("info", message)),
            error=lambda message: self.log_messages.append(("error", message)),
        )

    @staticmethod
    def _cfg(name):
        assert name == "gripper_deadband"
        return 0.02


def _message(sequence_id, error_code=0):
    return SimpleNamespace(
        sequence_id=sequence_id,
        command_phase=PHASE_EXECUTING if error_code == 0 else 4,
        state=1 if error_code == 0 else 0,
        error_code=error_code,
    )


def _guard_config():
    return GripperGuardConfig(
        target_tolerance=0.03,
        min_progress=0.02,
        stable_delta=0.006,
        contact_stable_sec=2.0,
        keepalive_interval_sec=0.05,
        no_progress_timeout_sec=0.60,
        transaction_timeout_sec=1.50,
    )


def test_command_publish_audit_contains_action_and_candidate_target():
    scheduler = _scheduler()
    harness = _AuditHarness(scheduler)
    planned = scheduler.plan([0.2, 0, 0, 0, 0, 0])

    VlaBridgeNode._publish_command(
        harness,
        planned.command,
        source="policy",
        target_before=scheduler.target,
        active_family_before=scheduler.active_family,
        policy_action=[0.2, 0, 0, 0, 0, 0],
        effective_action=[0.2, 0, 0, 0, 0, 0],
    )

    assert harness.published == [planned.command]
    record = harness.records[-1]
    assert record["event"] == "command_published"
    assert record["command_mode"] == planned.command.mode
    assert record["policy_action"] == [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert record["target_before"]["x"] == 15.0
    assert record["target_candidate"]["x"] == 15.1


def test_audit_records_committed_target_after_matching_ack():
    scheduler = _scheduler()
    harness = _AuditHarness(scheduler)
    planned = scheduler.plan([0.2, 0, 0, 0, 0, 0])
    sequence_id = planned.command.sequence_id
    harness._published_command_audit[sequence_id] = {
        "command_mode": planned.command.mode,
        "source": "policy",
        "keepalive": False,
    }
    pending = scheduler.pending
    target_before = scheduler.target
    result = scheduler.observe_lifecycle(
        sequence_id, PHASE_EXECUTING, 1, 0)

    VlaBridgeNode._audit_lifecycle(
        harness, _message(sequence_id), result, pending, target_before)

    events = [record["event"] for record in harness.records]
    assert events == ["lifecycle_observed", "target_committed"]
    committed = harness.records[-1]
    assert committed["target_before"]["x"] == 15.0
    assert committed["target_committed"]["x"] == 15.1
    assert sequence_id not in harness._published_command_audit


def test_audit_records_recoverable_rejection_and_consecutive_count():
    scheduler = _scheduler()
    harness = _AuditHarness(scheduler)
    planned = scheduler.plan([0.2, 0, 0, 0, 0, 0])
    sequence_id = planned.command.sequence_id
    harness._published_command_audit[sequence_id] = {
        "command_mode": planned.command.mode,
        "source": "policy",
        "keepalive": False,
    }
    pending = scheduler.pending
    target_before = scheduler.target
    result = scheduler.observe_lifecycle(sequence_id, 4, 0, 0x21)

    VlaBridgeNode._audit_lifecycle(
        harness,
        _message(sequence_id, error_code=0x21),
        result,
        pending,
        target_before,
    )

    events = [record["event"] for record in harness.records]
    assert events == [
        "lifecycle_observed",
        "target_rejected",
        "recoverable_error",
    ]
    assert harness.records[-1]["error_code"] == 0x21
    assert harness.records[-1]["consecutive_count"] == 1
    assert harness.records[-2]["target_retained"]["x"] == 15.0


def test_contact_latch_turns_redundant_close_into_cartesian_keepalive():
    scheduler = _scheduler()
    scheduler.reset((15.0, 0.0, 2.0, -54.48), 0.0, 0.816, 8)
    move = scheduler.plan([0.2, 0, 0, 0, 0, 0.816])
    scheduler.observe_lifecycle(
        move.command.sequence_id, PHASE_EXECUTING, 1, 0)
    harness = _AuditHarness(scheduler)
    harness._gripper_contact_latched = True

    filtered = VlaBridgeNode._filter_contact_action(
        harness,
        np.asarray([0, 0, 0, 0, 0, 1.0], dtype=np.float32),
    )
    planned = scheduler.plan(filtered)

    assert filtered[5] == np.float32(0.816)
    assert planned.command.mode == MODE_CARTESIAN_SERVO
    assert planned.command.keepalive
    assert planned.command.target.gripper == scheduler.target.gripper


def test_gripper_transaction_refreshes_on_first_control_tick():
    scheduler = _scheduler()
    close = scheduler.plan([0, 0, 0, 0, 0, 1.0])
    scheduler.observe_lifecycle(
        close.command.sequence_id, PHASE_EXECUTING, 1, 0)
    harness = _AuditHarness(scheduler)
    harness._gripper_transaction = GripperTransaction(
        scheduler.target.gripper, 0.0, 10.0, _guard_config())

    VlaBridgeNode._gripper_transaction_tick(harness, 10.0)

    assert harness.published[-1].mode == MODE_GRIPPER_SERVO
    assert harness.published[-1].keepalive
    assert scheduler.pending.keepalive
    assert harness.records[-1]["source"] == "gripper_keepalive"


def test_two_second_stall_retains_target_and_publishes_bounded_stop():
    scheduler = _scheduler()
    scheduler.reset((15.0, 0.0, 2.0, -54.48), 0.0, 0.89, 8)
    close = scheduler.plan([0, 0, 0, 0, 0, 1.0])
    scheduler.observe_lifecycle(
        close.command.sequence_id, PHASE_EXECUTING, 1, 0)
    harness = _AuditHarness(scheduler)
    transaction = GripperTransaction(
        scheduler.target.gripper, 0.741, 10.0, _guard_config())
    transaction.observe(0.780, 10.10)
    transaction.observe(0.816, 10.20)
    transaction.observe(0.816, 12.19)
    observation = transaction.observe(0.816, 12.21)
    harness._gripper_transaction = transaction

    VlaBridgeNode._finish_gripper_transaction(harness, observation)

    assert observation.event == EVENT_CONTACT
    assert harness._gripper_transaction is None
    assert harness._gripper_contact_latched
    assert scheduler.target.gripper == 0.965
    assert harness.published[-1].mode == MODE_GRIPPER_STOP
    events = [record["event"] for record in harness.records]
    assert events == ["command_published", "target_retained"]
    assert harness.records[-1]["reason"] == "gripper_contact"
    assert harness.records[-1]["requested_gripper"] == 0.965
    assert harness.records[-1]["target_retained"]["gripper"] == 0.965


def test_no_progress_sends_stop_and_latches_control_fault():
    scheduler = _scheduler()
    scheduler.reset((15.0, 0.0, 2.0, -54.48), 0.0, 0.89, 8)
    opening = scheduler.plan([0, 0, 0, 0, 0, 0.0])
    scheduler.observe_lifecycle(
        opening.command.sequence_id, PHASE_EXECUTING, 1, 0)
    harness = _AuditHarness(scheduler)
    harness._queued_actions = [np.zeros(6)]
    transaction = GripperTransaction(
        scheduler.target.gripper, 0.89, 10.0, _guard_config())
    observation = transaction.observe(0.89, 10.61)
    harness._gripper_transaction = transaction

    VlaBridgeNode._finish_gripper_transaction(harness, observation)

    assert observation.event == EVENT_NO_PROGRESS
    assert harness._control_fault_latched
    assert not harness._gripper_contact_latched
    assert harness._queued_actions == []
    assert harness.published[-1].mode == MODE_GRIPPER_STOP
    events = [record["event"] for record in harness.records]
    assert events == [
        "command_published",
        "target_retained",
        "no_progress_fault",
    ]
