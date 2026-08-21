from dataclasses import replace
from hashlib import sha256
import json

import pytest
from vla_dataset.lerobot_export import (
    MODE_CARTESIAN_SERVO,
    MODE_GRIPPER_STOP,
    MODE_GRIPPER_SERVO,
    PHASE_EXECUTING,
    AlignmentConfig,
    CommandRecord,
    ConversionError,
    EpisodeStreams,
    ImageRecord,
    StateRecord,
    apply_derivation,
    align_episode,
    load_processing_plan,
    validate_manifest_for_export,
)
from vla_dataset.review_episode import review_episode


def _image(timestamp):
    return ImageRecord(timestamp, timestamp, timestamp, 'jpeg', b'image')


def _state(timestamp, *, sequence=1, valid=True, phase=PHASE_EXECUTING):
    return StateRecord(
        timestamp=timestamp,
        bag_timestamp=timestamp,
        header_timestamp=timestamp,
        state=1,
        command_phase=phase,
        sequence_id=sequence,
        joint_position=(0.0, 0.1, 0.2, 0.3, 0.4),
        gripper_position=0.2,
        position_valid=valid,
        error_code=0,
    )


def _command(timestamp, sequence, mode, *, x=15.0, gripper=0.2):
    return CommandRecord(
        timestamp=timestamp,
        bag_timestamp=timestamp,
        header_timestamp=timestamp,
        mode=mode,
        sequence_id=sequence,
        target=(x, 0.0, 2.0, -54.48, 0.0, gripper),
    )


def _streams():
    images = tuple(_image(index / 10) for index in range(10))
    states = tuple(_state(index / 10) for index in range(10))
    commands = (
        _command(0.2, 1, MODE_CARTESIAN_SERVO),
        _command(0.35, 2, MODE_CARTESIAN_SERVO, x=15.5),
        _command(0.45, 3, MODE_GRIPPER_SERVO, x=15.5, gripper=0.8),
    )
    lifecycle = (
        _state(0.21, sequence=1),
        _state(0.36, sequence=2),
        _state(0.46, sequence=3),
    )
    return EpisodeStreams(images, states, lifecycle, commands)


def test_aligns_future_target_delta_and_absolute_gripper():
    frames, report = align_episode(
        _streams(),
        AlignmentConfig(fps=10, pre_roll_sec=0, post_roll_sec=0),
    )

    assert len(frames) == 3
    assert frames[0].source_timestamp == pytest.approx(0.2)
    assert frames[1].action == pytest.approx((0.5, 0, 0, 0, 0, 0.2))
    assert frames[2].action == pytest.approx((0, 0, 0, 0, 0, 0.8))
    assert frames[0].state == pytest.approx((0, 0.1, 0.2, 0.3, 0.4, 0.2))
    assert report['acknowledged_commands'] == 3


def test_uses_recent_valid_state_across_one_invalid_sample():
    streams = _streams()
    states = tuple(
        replace(state, position_valid=False)
        if state.timestamp == pytest.approx(0.3) else state
        for state in streams.states
    )

    _, report = align_episode(
        replace(streams, states=states),
        AlignmentConfig(fps=10, pre_roll_sec=0, post_roll_sec=0),
    )

    assert report['invalid_state_messages_in_crop'] == 1
    assert report['state_fallback_frames'] == 1


def test_rejects_invalid_state_run_when_last_valid_state_becomes_stale():
    streams = _streams()
    states = tuple(
        replace(state, position_valid=False)
        if any(state.timestamp == pytest.approx(value)
               for value in (0.3, 0.4)) else state
        for state in streams.states
    )

    with pytest.raises(ConversionError, match='consecutive position_valid=false'):
        align_episode(
            replace(streams, states=states),
            AlignmentConfig(fps=10, pre_roll_sec=0, post_roll_sec=0),
        )


def test_rejects_command_without_firmware_acknowledgement():
    streams = _streams()
    lifecycle = tuple(
        state for state in streams.lifecycle_states if state.sequence_id != 2)
    with pytest.raises(ConversionError, match='no EXECUTING/COMPLETED'):
        align_episode(
            replace(streams, lifecycle_states=lifecycle),
            AlignmentConfig(fps=10, pre_roll_sec=0, post_roll_sec=0),
        )


def test_allows_bounded_stream_target_superseded_by_acknowledged_target():
    streams = _streams()
    commands = (
        streams.commands[0],
        replace(streams.commands[1], mode=MODE_GRIPPER_SERVO),
        replace(streams.commands[2], timestamp=0.45),
    )
    lifecycle = tuple(
        state for state in streams.lifecycle_states if state.sequence_id != 2)

    _, report = align_episode(
        replace(streams, commands=commands, lifecycle_states=lifecycle),
        AlignmentConfig(fps=10, pre_roll_sec=0, post_roll_sec=0),
    )

    assert report['commands_in_crop'] == 3
    assert report['acknowledged_commands'] == 2
    assert report['superseded_stream_commands'] == 1
    assert report['superseded_stream_sequence_ids'] == [2]


def test_allows_unacknowledged_gripper_stream_target_preempted_by_stop():
    streams = _streams()
    commands = (
        streams.commands[0],
        streams.commands[1],
        replace(streams.commands[2], timestamp=0.45),
        _command(0.48, 4, MODE_GRIPPER_STOP, gripper=0.8),
    )
    lifecycle = streams.lifecycle_states[:2] + (_state(0.49, sequence=4),)

    _, report = align_episode(
        replace(streams, commands=commands, lifecycle_states=lifecycle),
        AlignmentConfig(fps=10, pre_roll_sec=0, post_roll_sec=0),
    )

    assert report['acknowledged_commands'] == 3
    assert report['superseded_stream_sequence_ids'] == [3]


def test_rejects_unacknowledged_stream_target_without_bounded_successor():
    streams = _streams()
    commands = (
        streams.commands[0],
        replace(streams.commands[1], mode=MODE_GRIPPER_SERVO),
        replace(streams.commands[2], timestamp=0.75),
    )
    lifecycle = tuple(
        state for state in streams.lifecycle_states if state.sequence_id != 2)

    with pytest.raises(ConversionError, match='no EXECUTING/COMPLETED'):
        align_episode(
            replace(streams, commands=commands, lifecycle_states=lifecycle),
            AlignmentConfig(fps=10, pre_roll_sec=0, post_roll_sec=0),
        )


def test_rejects_duplicate_command_sequence_id():
    streams = _streams()
    commands = (
        streams.commands[0],
        replace(streams.commands[1], sequence_id=1),
        streams.commands[2],
    )
    with pytest.raises(ConversionError, match='duplicate command sequence_id'):
        align_episode(
            replace(streams, commands=commands),
            AlignmentConfig(fps=10, pre_roll_sec=0, post_roll_sec=0),
        )


def test_requires_review_unless_pipeline_override_is_explicit():
    with pytest.raises(ConversionError, match='unreviewed'):
        validate_manifest_for_export({'status': 'unreviewed'})
    validate_manifest_for_export(
        {'status': 'unreviewed'}, allow_unreviewed=True)
    validate_manifest_for_export({'status': 'success'})
    with pytest.raises(ConversionError, match='not exportable'):
        validate_manifest_for_export({'status': 'failed'}, allow_unreviewed=True)


def test_loads_audited_processing_plan_and_resolves_source_root(tmp_path):
    source_root = tmp_path / 'raw'
    source_root.mkdir()
    plan_path = tmp_path / 'plan.json'
    plan_path.write_text(json.dumps({
        'schema_version': 1,
        'source_root': 'raw',
        'reviewed_at_utc': '2026-08-07T00:00:00Z',
        'review_method': 'manifest, bag integrity, lifecycle and visual QC',
        'episodes': [
            {
                'episode_id': 'episode_direct',
                'decision': 'success_usable',
                'reason': 'complete task and direct validation pass',
            },
            {
                'episode_id': 'episode_crop',
                'decision': 'success_crop',
                'reason': 'remove one rejected target',
                'derivation': {
                    'drop_command_sequence_ids': [2],
                    'drop_lifecycle_sequence_ids': [2],
                    'drop_error_state_sequence_ids': [2],
                },
            },
        ],
    }), encoding='utf-8')

    plan = load_processing_plan(plan_path)

    assert plan['source_root'] == source_root.resolve()
    assert plan['counts']['success_usable'] == 1
    assert plan['counts']['success_crop'] == 1
    assert len(plan['sha256']) == 64
    assert plan['by_id']['episode_crop']['derivation'][
        'drop_command_sequence_ids'] == [2]


def test_rejects_empty_success_crop_derivation(tmp_path):
    (tmp_path / 'raw').mkdir()
    plan_path = tmp_path / 'plan.json'
    plan_path.write_text(json.dumps({
        'schema_version': 1,
        'source_root': 'raw',
        'reviewed_at_utc': '2026-08-07T00:00:00Z',
        'review_method': 'audited QC',
        'episodes': [{
            'episode_id': 'episode_crop',
            'decision': 'success_crop',
            'reason': 'needs derivation',
            'derivation': {},
        }],
    }), encoding='utf-8')

    with pytest.raises(ConversionError, match='non-empty derivation'):
        load_processing_plan(plan_path)


def test_applies_audited_derivation_without_mutating_source_streams():
    streams = _streams()
    failed_state = replace(
        _state(0.37, sequence=2), state=3, command_phase=4,
        position_valid=False, error_code=0x21)
    streams = replace(
        streams,
        states=streams.states + (failed_state,),
        lifecycle_states=streams.lifecycle_states + (failed_state,),
    )
    derivation = {
        'min_command_sequence_id': None,
        'keep_acknowledged_only': False,
        'drop_command_sequence_ids': [2],
        'drop_lifecycle_sequence_ids': [2],
        'drop_error_state_sequence_ids': [2],
    }

    derived, audit = apply_derivation(streams, derivation)

    assert [item.sequence_id for item in streams.commands] == [1, 2, 3]
    assert [item.sequence_id for item in derived.commands] == [1, 3]
    assert all(item.sequence_id != 2 for item in derived.lifecycle_states)
    assert failed_state not in derived.states
    assert audit['removed_command_sequence_ids'] == [2]
    assert audit['removed_lifecycle_messages'] == 2
    assert audit['removed_filtered_state_messages'] == 1


def test_drops_one_audited_image_by_content_hash():
    streams = _streams()
    unique_image = replace(streams.images[3], data=b'corrupt-jpeg')
    streams = replace(
        streams,
        images=streams.images[:3] + (unique_image,) + streams.images[4:],
    )
    digest = sha256(unique_image.data).hexdigest()
    derivation = {
        'min_command_sequence_id': None,
        'keep_acknowledged_only': False,
        'drop_command_sequence_ids': [],
        'drop_lifecycle_sequence_ids': [],
        'drop_error_state_sequence_ids': [],
        'drop_image_sha256': [digest],
    }

    derived, audit = apply_derivation(streams, derivation)

    assert len(derived.images) == len(streams.images) - 1
    assert all(image.data != b'corrupt-jpeg' for image in derived.images)
    assert audit['removed_command_sequence_ids'] == []
    assert audit['removed_image_sha256'] == [digest]


def test_records_review_without_overwriting_raw_bag(tmp_path):
    episode_dir = tmp_path / 'episode_test'
    episode_dir.mkdir()
    manifest_path = episode_dir / 'manifest.json'
    manifest_path.write_text(
        '{"schema_version": 1, "episode_id": "episode_test", '
        '"status": "unreviewed"}', encoding='utf-8')

    manifest = review_episode(episode_dir, 'success', 'grasp verified')

    assert manifest['status'] == 'success'
    assert manifest['review']['notes'] == 'grasp verified'
    assert manifest['review']['reviewed_at_utc']
    with pytest.raises(ConversionError, match='only an unreviewed'):
        review_episode(episode_dir, 'failure')
