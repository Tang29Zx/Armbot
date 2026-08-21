import numpy as np

from vla_runtime.action_scheduler import (
    ActionScheduler,
    MODE_CARTESIAN_SERVO,
    MODE_CARTESIAN_SERVO_END,
    MODE_GRIPPER_SERVO,
    MODE_GRIPPER_STOP,
    PHASE_COMPLETED,
    PHASE_EXECUTING,
    SchedulerConfig,
)


def _scheduler():
    config = SchedulerConfig(
        action_scale=0.5,
        action_abs_limits=(0.31, 0.31, 0.31, 0.0, 0.035),
        action_deadbands=(0.015, 0.015, 0.015, 0.01, 0.003),
        gripper_deadband=0.02,
        gripper_max_step=0.075,
        pitch_limits_deg=(-90.0, 90.0),
        wrist_roll_limits_rad=(-1.57, 1.57),
        stream_watchdog_sec=0.3,
    )
    scheduler = ActionScheduler(config)
    scheduler.reset((15.0, 0.0, 2.0, -54.48), 0.0, 0.0, 8)
    return scheduler


def test_cartesian_target_commits_only_after_executing_ack():
    scheduler = _scheduler()
    planned = scheduler.plan([0.2, 0.0, -0.1, 0.0, 0.0, 0.0])

    assert planned.command.mode == MODE_CARTESIAN_SERVO
    assert planned.command.sequence_id == 8
    assert planned.command.target.x == 15.1
    assert planned.command.target.z == 1.95
    assert scheduler.target.x == 15.0

    ack = scheduler.observe_lifecycle(8, PHASE_EXECUTING, 1, 0)
    assert ack.consume_action
    assert scheduler.target.x == 15.1
    assert scheduler.active_family == "cartesian"


def test_family_switch_ends_cartesian_before_gripper_action():
    scheduler = _scheduler()
    first = scheduler.plan([0.2, 0, 0, 0, 0, 0])
    scheduler.observe_lifecycle(first.command.sequence_id, PHASE_EXECUTING, 1, 0)

    switch = scheduler.plan([0, 0, 0, 0, 0, 1])
    assert switch.command.mode == MODE_CARTESIAN_SERVO_END
    assert not switch.consume_action
    scheduler.observe_lifecycle(switch.command.sequence_id, PHASE_COMPLETED, 2, 0)

    gripper = scheduler.plan([0, 0, 0, 0, 0, 1])
    assert gripper.command.mode == MODE_GRIPPER_SERVO
    assert gripper.command.target.gripper == 0.075


def test_idle_cartesian_action_refreshes_same_target_without_ending_stream():
    scheduler = _scheduler()
    first = scheduler.plan([0.2, 0, 0, 0, 0, 0])
    first_ack = scheduler.observe_lifecycle(
        first.command.sequence_id, PHASE_EXECUTING, 1, 0)
    installed = scheduler.target

    keepalive = scheduler.plan([0, 0, 0, 0, 0, 0])

    assert first_ack.committed
    assert keepalive.command.mode == MODE_CARTESIAN_SERVO
    assert keepalive.command.keepalive
    assert keepalive.command.target == installed
    assert scheduler.target == installed
    assert scheduler.active_family == "cartesian"

    keepalive_ack = scheduler.observe_lifecycle(
        keepalive.command.sequence_id, PHASE_EXECUTING, 1, 0)

    assert keepalive_ack.consume_action
    assert keepalive_ack.committed
    assert scheduler.target == installed
    assert scheduler.active_family == "cartesian"


def test_gripper_stream_uses_bounded_stop_when_action_goes_idle():
    scheduler = _scheduler()
    close = scheduler.plan([0, 0, 0, 0, 0, 1])
    scheduler.observe_lifecycle(close.command.sequence_id, PHASE_EXECUTING, 1, 0)

    stop = scheduler.plan([0, 0, 0, 0, 0, 0.075])
    assert stop.command.mode == MODE_GRIPPER_STOP
    scheduler.observe_lifecycle(stop.command.sequence_id, PHASE_COMPLETED, 2, 0)
    assert scheduler.active_family is None


def test_gripper_stop_preserves_installed_target():
    scheduler = _scheduler()
    close = scheduler.plan([0, 0, 0, 0, 0, 1])
    scheduler.observe_lifecycle(close.command.sequence_id, PHASE_EXECUTING, 1, 0)
    installed = scheduler.target

    stop = scheduler.stop_gripper()

    assert stop.command.mode == MODE_GRIPPER_STOP
    assert scheduler.target == installed


def test_gripper_keepalive_reuses_target_without_consuming_action():
    scheduler = _scheduler()
    close = scheduler.plan([0, 0, 0, 0, 0, 1])
    scheduler.observe_lifecycle(
        close.command.sequence_id, PHASE_EXECUTING, 1, 0)
    installed = scheduler.target

    keepalive = scheduler.keep_gripper_stream_open()

    assert keepalive.command.mode == MODE_GRIPPER_SERVO
    assert keepalive.command.keepalive
    assert keepalive.command.target == installed
    assert not scheduler.pending.consume_action

    ack = scheduler.observe_lifecycle(
        keepalive.command.sequence_id, PHASE_EXECUTING, 1, 0)

    assert ack.committed
    assert not ack.consume_action
    assert scheduler.target == installed
    assert scheduler.active_family == "gripper"


def test_nonfinite_action_is_rejected():
    scheduler = _scheduler()
    with np.testing.assert_raises_regex(ValueError, "non-finite"):
        scheduler.plan([np.nan, 0, 0, 0, 0, 0])


def test_failed_command_rolls_back_and_closes_family():
    scheduler = _scheduler()
    planned = scheduler.plan([0.2, 0, 0, 0, 0, 0])
    failed = scheduler.observe_lifecycle(planned.command.sequence_id, 4, 0, 0xFF)

    assert failed.failed
    assert scheduler.target.x == 15.0
    assert scheduler.active_family is None


def test_recoverable_error_drops_target_without_latching():
    scheduler = _scheduler()
    planned = scheduler.plan([0.2, 0, 0, 0, 0, 0])
    result = scheduler.observe_lifecycle(
        planned.command.sequence_id, 4, 0, 0x21)

    assert not result.failed
    assert result.rejected
    assert result.recoverable
    assert scheduler.active_family is None
    assert scheduler.target.x == 15.0


def test_cancel_requires_a_fresh_home_reset():
    scheduler = _scheduler()

    scheduler.cancel()

    with np.testing.assert_raises_regex(RuntimeError, "reset from Home"):
        scheduler.plan([0, 0, 0, 0, 0, 0])
