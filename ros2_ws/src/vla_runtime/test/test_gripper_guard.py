from vla_runtime.gripper_guard import (
    EVENT_CONTACT,
    EVENT_NO_PROGRESS,
    EVENT_REACHED,
    EVENT_TIMEOUT,
    GripperGuardConfig,
    GripperTransaction,
)


def _config():
    return GripperGuardConfig(
        target_tolerance=0.03,
        min_progress=0.02,
        stable_delta=0.006,
        contact_stable_sec=2.0,
        keepalive_interval_sec=0.05,
        no_progress_timeout_sec=0.60,
        transaction_timeout_sec=1.50,
    )


def test_feedback_reaching_target_completes_without_contact():
    transaction = GripperTransaction(0.50, 0.40, 10.0, _config())

    observation = transaction.observe(0.49, 10.2)

    assert observation.event == EVENT_REACHED
    assert observation.progressed


def test_sub_deadzone_close_step_is_reached_instead_of_no_progress_fault():
    # Deployed mapping is 500 raw/full-scale.  This 0.026 gap is about 13 raw:
    # larger than the firmware's nominal 10-raw tolerance, but still inside
    # the measured servo deadband seen on hardware.  It must not block the
    # policy from issuing the next, larger close target.
    transaction = GripperTransaction(0.088, 0.062, 10.0, _config())

    observation = transaction.observe(0.062, 10.61)

    assert observation.event == EVENT_REACHED
    assert not observation.progressed


def test_close_stable_for_two_seconds_is_contact_without_target_rewrite():
    transaction = GripperTransaction(0.965, 0.816, 10.0, _config())

    assert transaction.observe(0.816, 11.99).event is None
    observation = transaction.observe(0.816, 12.0)

    assert observation.event == EVENT_CONTACT
    assert not observation.progressed
    assert observation.stable_elapsed_sec == 2.0


def test_close_motion_resets_the_two_second_stability_window():
    transaction = GripperTransaction(0.95, 0.77, 10.0, _config())

    assert transaction.observe(0.79, 10.5).event is None
    assert transaction.observe(0.816, 11.0).event is None
    assert transaction.observe(0.816, 12.99).event is None
    observation = transaction.observe(0.816, 13.0)

    assert observation.event == EVENT_CONTACT
    assert observation.stable_elapsed_sec == 2.0


def test_hardware_timeout_replay_becomes_contact_after_two_second_stall():
    transaction = GripperTransaction(0.851, 0.776, 10.0, _config())

    assert transaction.observe(0.816, 10.4).event is None
    assert transaction.observe(0.816, 12.39).event is None
    observation = transaction.observe(0.816, 12.41)

    assert observation.event == EVENT_CONTACT
    assert abs(observation.gap - 0.035) < 1e-9
    assert observation.progressed


def test_open_stationary_feedback_remains_no_progress_fault():
    transaction = GripperTransaction(0.20, 0.80, 10.0, _config())

    observation = transaction.observe(0.80, 10.61)

    assert observation.event == EVENT_NO_PROGRESS


def test_slow_open_progress_eventually_times_out():
    transaction = GripperTransaction(0.20, 0.80, 10.0, _config())

    assert transaction.observe(0.775, 10.7).event is None
    observation = transaction.observe(0.770, 11.51)

    assert observation.event == EVENT_TIMEOUT
    assert observation.progressed


def test_closing_does_not_take_the_opening_transaction_timeout():
    transaction = GripperTransaction(0.95, 0.80, 10.0, _config())

    observation = transaction.observe(0.82, 11.6)

    assert observation.event is None


def test_first_keepalive_is_due_immediately_then_deadline_advances():
    transaction = GripperTransaction(0.50, 0.40, 10.0, _config())

    # EXECUTING is delayed feedback: the next control tick must refresh the
    # stream without waiting another interval.
    assert transaction.keepalive_due(10.0)

    transaction.mark_keepalive(10.0)

    assert not transaction.keepalive_due(10.049)
    assert transaction.keepalive_due(10.05)
