"""Convert immutable Armbot rosbag2 episodes to an OpenPI LeRobot dataset.

The ROS recorder intentionally stores event-oriented commands.  This module
reconstructs the held teleoperation target, causally resamples observations at
a fixed rate, and exports the next-step target change expected by the policy.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import sys
import uuid
from bisect import bisect_right
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path

IMAGE_KEY = 'observation.images.front'
STATE_KEY = 'observation.state'
ACTION_KEY = 'action'

STATE_NAMES = (
    'joint_1_rad',
    'joint_2_rad',
    'joint_3_rad',
    'joint_4_rad',
    'joint_5_rad',
    'gripper_absolute',
)
ACTION_NAMES = (
    'delta_x_cm',
    'delta_y_cm',
    'delta_z_cm',
    'delta_pitch_deg',
    'delta_wrist_roll_rad',
    'gripper_absolute',
)

MODE_STOP = 0
MODE_END_EFFECTOR = 1
MODE_JOINT = 2
MODE_GRIPPER = 3
MODE_GRIPPER_STOP = 4
MODE_CARTESIAN_SERVO = 5
MODE_CARTESIAN_SERVO_END = 6
MODE_WRIST_ROLL = 7
MODE_GRIPPER_SERVO = 8
MODE_GRIPPER_SERVO_END = 9
MODE_WRIST_ROLL_SERVO = 10
MODE_WRIST_ROLL_SERVO_END = 11

STATE_ERROR = 3
STATE_ESTOP = 4
PHASE_EXECUTING = 2
PHASE_COMPLETED = 3

# The controller retains only the newest pending target in a streaming family.
# At the 10 Hz teleop rate an older target can therefore be intentionally
# replaced before it receives its own firmware lifecycle response.  Keep the
# allowance bounded so a genuinely lost command cannot be hidden by an
# unrelated command much later in the episode.
MAX_STREAM_SUPERSESSION_GAP_SEC = 0.25
MAX_STREAM_SUPERSESSION_CHAIN_SEC = 1.0

ARM_TARGET_MODES = frozenset((MODE_END_EFFECTOR, MODE_CARTESIAN_SERVO))
GRIPPER_TARGET_MODES = frozenset(
    (MODE_GRIPPER, MODE_GRIPPER_STOP, MODE_GRIPPER_SERVO))
WRIST_TARGET_MODES = frozenset((MODE_WRIST_ROLL, MODE_WRIST_ROLL_SERVO))
ACTIVE_TARGET_MODES = ARM_TARGET_MODES | GRIPPER_TARGET_MODES | WRIST_TARGET_MODES
SUPPORTED_MODES = frozenset(range(MODE_WRIST_ROLL_SERVO_END + 1))

STREAM_TARGET_FAMILIES = {
    MODE_CARTESIAN_SERVO: 'cartesian',
    MODE_GRIPPER_SERVO: 'gripper',
    MODE_WRIST_ROLL_SERVO: 'wrist',
}
STREAM_END_FAMILIES = {
    MODE_CARTESIAN_SERVO_END: 'cartesian',
    MODE_GRIPPER_SERVO_END: 'gripper',
    MODE_WRIST_ROLL_SERVO_END: 'wrist',
}

PROCESSING_DECISIONS = (
    'success_usable', 'success_crop', 'discard', 'out_of_scope')
DERIVATION_KEYS = frozenset((
    'min_command_sequence_id',
    'keep_acknowledged_only',
    'drop_command_sequence_ids',
    'drop_lifecycle_sequence_ids',
    'drop_error_state_sequence_ids',
    'drop_image_sha256',
))


class ConversionError(RuntimeError):
    """Raised when an episode cannot safely become a training episode."""


@dataclass(frozen=True)
class ImageRecord:
    timestamp: float
    bag_timestamp: float
    header_timestamp: float
    format: str
    data: bytes


@dataclass(frozen=True)
class StateRecord:
    timestamp: float
    bag_timestamp: float
    header_timestamp: float
    state: int
    command_phase: int
    sequence_id: int
    joint_position: tuple[float, ...]
    gripper_position: float
    position_valid: bool
    error_code: int


@dataclass(frozen=True)
class CommandRecord:
    timestamp: float
    bag_timestamp: float
    header_timestamp: float
    mode: int
    sequence_id: int
    target: tuple[float, ...]


@dataclass(frozen=True)
class EpisodeStreams:
    images: tuple[ImageRecord, ...]
    states: tuple[StateRecord, ...]
    lifecycle_states: tuple[StateRecord, ...]
    commands: tuple[CommandRecord, ...]


@dataclass(frozen=True)
class AlignedFrame:
    source_timestamp: float
    image: ImageRecord
    state: tuple[float, ...]
    action: tuple[float, ...]
    state_age_sec: float
    image_age_sec: float


@dataclass(frozen=True)
class AlignmentConfig:
    fps: int = 10
    pre_roll_sec: float = 0.2
    post_roll_sec: float = 0.5
    max_image_age_sec: float = 0.12
    max_state_age_sec: float = 0.15

    def validate(self) -> None:
        if self.fps <= 0:
            raise ConversionError('fps must be positive')
        for name in (
                'pre_roll_sec', 'post_roll_sec', 'max_image_age_sec',
                'max_state_age_sec'):
            if getattr(self, name) < 0:
                raise ConversionError('%s must not be negative' % name)


def _message_timestamp(message, bag_timestamp_ns, clock):
    bag_timestamp = bag_timestamp_ns / 1_000_000_000.0
    stamp = message.header.stamp
    header_timestamp = float(stamp.sec) + float(stamp.nanosec) / 1e9
    if clock == 'header' and header_timestamp > 0:
        return header_timestamp, bag_timestamp, header_timestamp
    return bag_timestamp, bag_timestamp, header_timestamp


def _find_message_dir(explicit=None):
    candidates = []
    if explicit is not None:
        candidates.append(Path(explicit).expanduser())
    candidates.append(
        Path.cwd() / 'ros2_ws' / 'src' / 'action_interfaces' / 'msg')
    for parent in Path(__file__).resolve().parents:
        candidates.append(
            parent / 'ros2_ws' / 'src' / 'action_interfaces' / 'msg')
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / 'share' / 'action_interfaces' / 'msg')

    for candidate in candidates:
        if ((candidate / 'ArmCommand.msg').is_file()
                and (candidate / 'ArmState.msg').is_file()):
            return candidate.resolve()
    raise ConversionError(
        'cannot find ArmCommand.msg and ArmState.msg; pass --message-dir '
        '/path/to/Armbot/ros2_ws/src/action_interfaces/msg')


def _make_typestore(message_dir):
    try:
        from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    except ImportError as exc:
        raise ConversionError(
            'rosbags is required; install it in the OpenPI environment') from exc

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    for name in ('ArmCommand', 'ArmState'):
        definition = (message_dir / ('%s.msg' % name)).read_text(
            encoding='utf-8')
        typestore.register(get_types_from_msg(
            definition, 'action_interfaces/msg/%s' % name))
    return typestore


def read_episode_streams(
        episode_dir, *, image_topic='/image',
        state_topic='/arm/state_filtered', clock='bag', message_dir=None):
    """Read the four streams required by the offline conversion."""
    if clock not in ('bag', 'header'):
        raise ConversionError('clock must be either bag or header')
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise ConversionError(
            'rosbags is required; install it in the OpenPI environment') from exc

    episode_dir = Path(episode_dir).expanduser().resolve()
    manifest = load_manifest(episode_dir)
    bag_dir = (episode_dir / manifest.get('bag_path', 'bag')).resolve()
    if not bag_dir.is_relative_to(episode_dir):
        raise ConversionError('manifest bag_path escapes the episode directory')
    if not (bag_dir / 'metadata.yaml').is_file():
        raise ConversionError('rosbag2 metadata.yaml is missing: %s' % bag_dir)

    typestore = _make_typestore(_find_message_dir(message_dir))
    required_topics = tuple(dict.fromkeys(
        (image_topic, state_topic, '/arm/state', '/arm/command')))
    images = []
    states = []
    lifecycle_states = []
    commands = []

    try:
        with AnyReader([bag_dir], default_typestore=typestore) as reader:
            by_topic = {connection.topic: connection
                        for connection in reader.connections}
            missing = [topic for topic in required_topics
                       if topic not in by_topic]
            if missing:
                raise ConversionError(
                    'rosbag2 is missing required topics: %s'
                    % ', '.join(missing))
            connections = [by_topic[topic] for topic in required_topics]
            for connection, bag_timestamp_ns, raw in reader.messages(
                    connections=connections):
                message = reader.deserialize(raw, connection.msgtype)
                timestamp, bag_timestamp, header_timestamp = (
                    _message_timestamp(message, bag_timestamp_ns, clock))
                if connection.topic == image_topic:
                    images.append(ImageRecord(
                        timestamp=timestamp,
                        bag_timestamp=bag_timestamp,
                        header_timestamp=header_timestamp,
                        format=str(message.format),
                        data=bytes(message.data),
                    ))
                elif connection.topic in (state_topic, '/arm/state'):
                    record = StateRecord(
                        timestamp=timestamp,
                        bag_timestamp=bag_timestamp,
                        header_timestamp=header_timestamp,
                        state=int(message.state),
                        command_phase=int(message.command_phase),
                        sequence_id=int(message.sequence_id),
                        joint_position=tuple(
                            float(value) for value in message.joint_position),
                        gripper_position=float(message.gripper_position),
                        position_valid=bool(message.position_valid),
                        error_code=int(message.error_code),
                    )
                    if connection.topic == state_topic:
                        states.append(record)
                    if connection.topic == '/arm/state':
                        lifecycle_states.append(record)
                elif connection.topic == '/arm/command':
                    joint_position = tuple(
                        float(value) for value in message.joint_position)
                    if len(joint_position) < 5:
                        raise ConversionError(
                            'ArmCommand joint_position has fewer than 5 values')
                    commands.append(CommandRecord(
                        timestamp=timestamp,
                        bag_timestamp=bag_timestamp,
                        header_timestamp=header_timestamp,
                        mode=int(message.mode),
                        sequence_id=int(message.sequence_id),
                        target=(
                            float(message.x),
                            float(message.y),
                            float(message.z),
                            float(message.pitch),
                            joint_position[4],
                            float(message.gripper_position),
                        ),
                    ))
    except ConversionError:
        raise
    except Exception as exc:
        raise ConversionError('failed to read rosbag2: %s' % exc) from exc

    streams = EpisodeStreams(
        images=tuple(sorted(images, key=lambda item: item.timestamp)),
        states=tuple(sorted(states, key=lambda item: item.timestamp)),
        lifecycle_states=tuple(sorted(
            lifecycle_states, key=lambda item: item.timestamp)),
        commands=tuple(sorted(commands, key=lambda item: item.timestamp)),
    )
    for name in ('images', 'states', 'lifecycle_states', 'commands'):
        if not getattr(streams, name):
            raise ConversionError('episode has no %s' % name)
    return streams


def load_manifest(episode_dir):
    path = Path(episode_dir) / 'manifest.json'
    try:
        manifest = json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError as exc:
        raise ConversionError('manifest.json is missing: %s' % path) from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise ConversionError('cannot read manifest.json: %s' % exc) from exc
    if manifest.get('schema_version') != 1:
        raise ConversionError(
            'unsupported manifest schema_version: %r'
            % manifest.get('schema_version'))
    return manifest


def _sequence_id_set(value, label):
    if value is None:
        return frozenset()
    if not isinstance(value, list):
        raise ConversionError('%s must be a list' % label)
    if any(not isinstance(item, int) or item < 0 for item in value):
        raise ConversionError('%s must contain non-negative integers' % label)
    if len(value) != len(set(value)):
        raise ConversionError('%s contains duplicate sequence IDs' % label)
    return frozenset(value)


def _sha256_set(value, label):
    if value is None:
        return frozenset()
    if not isinstance(value, list):
        raise ConversionError('%s must be a list' % label)
    normalized = []
    for item in value:
        if not isinstance(item, str):
            raise ConversionError('%s must contain strings' % label)
        item = item.lower()
        if (len(item) != 64
                or any(character not in '0123456789abcdef'
                       for character in item)):
            raise ConversionError('%s contains an invalid SHA-256' % label)
        normalized.append(item)
    if len(normalized) != len(set(normalized)):
        raise ConversionError('%s contains duplicate SHA-256 values' % label)
    return frozenset(normalized)


def _validate_derivation(value, episode_id):
    if not isinstance(value, dict):
        raise ConversionError(
            'processing plan derivation must be an object: %s' % episode_id)
    unknown = sorted(set(value) - DERIVATION_KEYS)
    if unknown:
        raise ConversionError(
            'processing plan has unknown derivation keys for %s: %s'
            % (episode_id, ', '.join(unknown)))
    minimum = value.get('min_command_sequence_id')
    if minimum is not None and (
            not isinstance(minimum, int) or minimum < 0):
        raise ConversionError(
            'min_command_sequence_id must be a non-negative integer: %s'
            % episode_id)
    acknowledged_only = value.get('keep_acknowledged_only', False)
    if not isinstance(acknowledged_only, bool):
        raise ConversionError(
            'keep_acknowledged_only must be boolean: %s' % episode_id)
    normalized = {
        'min_command_sequence_id': minimum,
        'keep_acknowledged_only': acknowledged_only,
        'drop_command_sequence_ids': sorted(_sequence_id_set(
            value.get('drop_command_sequence_ids'),
            '%s.drop_command_sequence_ids' % episode_id)),
        'drop_lifecycle_sequence_ids': sorted(_sequence_id_set(
            value.get('drop_lifecycle_sequence_ids'),
            '%s.drop_lifecycle_sequence_ids' % episode_id)),
        'drop_error_state_sequence_ids': sorted(_sequence_id_set(
            value.get('drop_error_state_sequence_ids'),
            '%s.drop_error_state_sequence_ids' % episode_id)),
        'drop_image_sha256': sorted(_sha256_set(
            value.get('drop_image_sha256'),
            '%s.drop_image_sha256' % episode_id)),
    }
    if (minimum is None and not acknowledged_only
            and not normalized['drop_command_sequence_ids']
            and not normalized['drop_lifecycle_sequence_ids']
            and not normalized['drop_error_state_sequence_ids']
            and not normalized['drop_image_sha256']):
        raise ConversionError(
            'success_crop requires a non-empty derivation: %s' % episode_id)
    return normalized


def load_processing_plan(path):
    """Load an immutable external QC plan without editing raw manifests."""
    path = Path(path).expanduser().resolve()
    try:
        raw = path.read_bytes()
        plan = json.loads(raw)
    except FileNotFoundError as exc:
        raise ConversionError(
            'processing plan is missing: %s' % path) from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise ConversionError(
            'cannot read processing plan: %s' % exc) from exc
    if not isinstance(plan, dict):
        raise ConversionError('processing plan root must be an object')
    if plan.get('schema_version') != 1:
        raise ConversionError(
            'unsupported processing plan schema_version: %r'
            % plan.get('schema_version'))
    entries = plan.get('episodes')
    if not isinstance(entries, list) or not entries:
        raise ConversionError('processing plan episodes must be a non-empty list')
    source_root_value = plan.get('source_root')
    if not isinstance(source_root_value, str) or not source_root_value.strip():
        raise ConversionError('processing plan source_root must be a string')
    reviewed_at_utc = plan.get('reviewed_at_utc')
    if not isinstance(reviewed_at_utc, str) or not reviewed_at_utc.strip():
        raise ConversionError(
            'processing plan reviewed_at_utc must be a non-empty string')
    review_method = plan.get('review_method')
    if not isinstance(review_method, str) or not review_method.strip():
        raise ConversionError(
            'processing plan review_method must be a non-empty string')
    source_root = Path(source_root_value).expanduser()
    if not source_root.is_absolute():
        source_root = path.parent / source_root
    source_root = source_root.resolve()
    if not source_root.is_dir():
        raise ConversionError(
            'processing plan source_root is not a directory: %s' % source_root)

    by_id = {}
    counts = {decision: 0 for decision in PROCESSING_DECISIONS}
    normalized_entries = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ConversionError('processing plan episode entry must be an object')
        unknown_entry_keys = sorted(
            set(entry) - {'episode_id', 'decision', 'reason', 'derivation'})
        if unknown_entry_keys:
            raise ConversionError(
                'processing plan has unknown keys: %s'
                % ', '.join(unknown_entry_keys))
        episode_id = entry.get('episode_id')
        if (not isinstance(episode_id, str)
                or not episode_id.startswith('episode_')
                or '/' in episode_id or '\\' in episode_id):
            raise ConversionError(
                'invalid processing plan episode_id: %r' % episode_id)
        if episode_id in by_id:
            raise ConversionError(
                'duplicate processing plan episode_id: %s' % episode_id)
        decision = entry.get('decision')
        if decision not in PROCESSING_DECISIONS:
            raise ConversionError(
                'invalid processing decision for %s: %r'
                % (episode_id, decision))
        reason = entry.get('reason', '')
        if not isinstance(reason, str) or not reason.strip():
            raise ConversionError(
                'processing plan reason must be a non-empty string: %s'
                % episode_id)
        derivation = entry.get('derivation')
        if decision == 'success_crop':
            derivation = _validate_derivation(derivation, episode_id)
        elif derivation is not None:
            raise ConversionError(
                'only success_crop may define derivation: %s' % episode_id)
        normalized = {
            'episode_id': episode_id,
            'decision': decision,
            'reason': reason,
            'derivation': derivation,
        }
        by_id[episode_id] = normalized
        normalized_entries.append(normalized)
        counts[decision] += 1

    return {
        'path': path,
        'sha256': hashlib.sha256(raw).hexdigest(),
        'raw': raw,
        'source_root': source_root,
        'reviewed_at_utc': reviewed_at_utc,
        'review_method': review_method,
        'entries': tuple(normalized_entries),
        'by_id': by_id,
        'counts': counts,
    }


def _is_error_state(record):
    return (record.error_code != 0 or record.state in (STATE_ERROR, STATE_ESTOP)
            or record.command_phase == 4)


def apply_derivation(streams, derivation):
    """Apply one audited crop/repair rule and return an explicit audit."""
    minimum = derivation.get('min_command_sequence_id')
    drop_commands = frozenset(derivation['drop_command_sequence_ids'])
    drop_lifecycle = frozenset(derivation['drop_lifecycle_sequence_ids'])
    drop_error_states = frozenset(
        derivation['drop_error_state_sequence_ids'])
    drop_image_sha256 = frozenset(derivation.get('drop_image_sha256', ()))
    acknowledged_only = derivation['keep_acknowledged_only']

    missing_commands = sorted(
        drop_commands
        - {record.sequence_id for record in streams.commands})
    if missing_commands:
        raise ConversionError(
            'derivation command sequence IDs were not found: %s'
            % missing_commands)
    missing_lifecycle = sorted(
        drop_lifecycle
        - {record.sequence_id for record in streams.lifecycle_states})
    if missing_lifecycle:
        raise ConversionError(
            'derivation lifecycle sequence IDs were not found: %s'
            % missing_lifecycle)
    missing_error_states = sorted(
        drop_error_states
        - {record.sequence_id for record in streams.states
           if _is_error_state(record)})
    if missing_error_states:
        raise ConversionError(
            'derivation error-state sequence IDs were not found: %s'
            % missing_error_states)

    acknowledged = frozenset(
        sequence_id for sequence_id, lifecycle in _lifecycle_by_sequence(
            streams.lifecycle_states).items()
        if _has_firmware_ack(lifecycle))
    commands = tuple(
        command for command in streams.commands
        if (minimum is None or command.sequence_id >= minimum)
        and command.sequence_id not in drop_commands
        and (not acknowledged_only or command.sequence_id in acknowledged))
    if not commands:
        raise ConversionError('derivation removed every command')
    removed_commands = sorted(
        command.sequence_id for command in streams.commands
        if command not in commands)
    lifecycle_states = tuple(
        record for record in streams.lifecycle_states
        if record.sequence_id not in drop_lifecycle)
    states = tuple(
        record for record in streams.states
        if not (record.sequence_id in drop_error_states
                and _is_error_state(record)))
    image_hashes = tuple(
        (record, hashlib.sha256(record.data).hexdigest())
        for record in streams.images)
    images = tuple(
        record for record, digest in image_hashes
        if digest not in drop_image_sha256)
    removed_image_sha256 = sorted(
        {digest for _, digest in image_hashes
         if digest in drop_image_sha256})
    missing_image_sha256 = sorted(
        drop_image_sha256 - set(removed_image_sha256))
    if missing_image_sha256:
        raise ConversionError(
            'derivation image SHA-256 was not found: %s'
            % ', '.join(missing_image_sha256))
    removed_lifecycle_messages = (
        len(streams.lifecycle_states) - len(lifecycle_states))
    removed_filtered_state_messages = len(streams.states) - len(states)
    if (not removed_commands and not removed_lifecycle_messages
            and not removed_filtered_state_messages
            and not removed_image_sha256):
        raise ConversionError('success_crop derivation removed no records')
    derived = EpisodeStreams(
        images=images,
        states=states,
        lifecycle_states=lifecycle_states,
        commands=commands,
    )
    return derived, {
        'original_commands': len(streams.commands),
        'derived_commands': len(commands),
        'removed_command_sequence_ids': removed_commands,
        'removed_lifecycle_messages': removed_lifecycle_messages,
        'removed_filtered_state_messages': removed_filtered_state_messages,
        'removed_image_sha256': removed_image_sha256,
        'rule': derivation,
    }


def validate_manifest_for_export(manifest, allow_unreviewed=False):
    status = manifest.get('status')
    if status == 'success':
        return
    if status == 'unreviewed' and allow_unreviewed:
        return
    if status == 'unreviewed':
        raise ConversionError(
            'episode is unreviewed; mark it success first, or use '
            '--allow-unreviewed only for pipeline testing')
    raise ConversionError('episode status is not exportable: %r' % status)


def _check_finite(values, label):
    if not all(math.isfinite(value) for value in values):
        raise ConversionError('%s contains non-finite values' % label)


def _action_bounds(commands, config):
    active = [command for command in commands
              if command.mode in ACTIVE_TARGET_MODES]
    if not active:
        raise ConversionError('episode contains no supported target commands')
    start = active[0].timestamp - config.pre_roll_sec
    end = commands[-1].timestamp + config.post_roll_sec
    return start, end


def _has_firmware_ack(lifecycle):
    return any(state.command_phase in (PHASE_EXECUTING, PHASE_COMPLETED)
               for state in lifecycle)


def _lifecycle_by_sequence(states):
    result = {}
    for state in states:
        result.setdefault(state.sequence_id, []).append(state)
    return result


def _is_stream_successor(family, mode):
    if STREAM_TARGET_FAMILIES.get(mode) == family:
        return True
    if STREAM_END_FAMILIES.get(mode) == family:
        return True
    return family == 'gripper' and mode == MODE_GRIPPER_STOP


def _is_superseded_stream_command(commands, index, acknowledged):
    """Return true when a missing ACK is explained by controller coalescing."""
    command = commands[index]
    family = STREAM_TARGET_FAMILIES.get(command.mode)
    if family is None:
        return False

    chain_start = command.timestamp
    previous = command
    for successor_index in range(index + 1, len(commands)):
        successor = commands[successor_index]
        if successor.timestamp - previous.timestamp > (
                MAX_STREAM_SUPERSESSION_GAP_SEC + 1e-9):
            return False
        if successor.timestamp - chain_start > (
                MAX_STREAM_SUPERSESSION_CHAIN_SEC + 1e-9):
            return False
        if not _is_stream_successor(family, successor.mode):
            return False
        if acknowledged[successor_index]:
            return True
        # Only another target can continue an unacknowledged replacement
        # chain.  An unacknowledged END/STOP remains a hard failure.
        if STREAM_TARGET_FAMILIES.get(successor.mode) != family:
            return False
        previous = successor
    return False


def _validate_command_lifecycle(streams, start, end):
    commands = [command for command in streams.commands
                if start <= command.timestamp <= end]
    sequence_ids = [command.sequence_id for command in commands]
    if len(sequence_ids) != len(set(sequence_ids)):
        raise ConversionError(
            'duplicate command sequence_id occurs inside the crop')
    if any(command.mode == MODE_STOP for command in commands):
        raise ConversionError('emergency stop command occurs inside the crop')
    unsupported = [command.mode for command in commands
                   if command.mode not in SUPPORTED_MODES
                   or command.mode == MODE_JOINT]
    if unsupported:
        raise ConversionError(
            'unsupported command mode occurs inside the crop: %s'
            % sorted(set(unsupported)))

    lifecycle_by_sequence = _lifecycle_by_sequence(streams.lifecycle_states)

    failed = []
    acknowledged = []
    for command in commands:
        lifecycle = lifecycle_by_sequence.get(command.sequence_id, ())
        if any(state.error_code != 0 or state.command_phase == 4
               for state in lifecycle):
            failed.append(command.sequence_id)
        acknowledged.append(_has_firmware_ack(lifecycle))
    if failed:
        raise ConversionError(
            'commands have failed/error lifecycle: %s' % failed[:10])

    superseded = []
    missing_ack = []
    for index, command in enumerate(commands):
        if acknowledged[index]:
            continue
        if _is_superseded_stream_command(commands, index, acknowledged):
            superseded.append(command.sequence_id)
        else:
            missing_ack.append(command.sequence_id)
    if missing_ack:
        raise ConversionError(
            'commands have no EXECUTING/COMPLETED acknowledgement: %s'
            % missing_ack[:10])

    for state in streams.lifecycle_states:
        if (start <= state.timestamp <= end
                and (state.state in (STATE_ERROR, STATE_ESTOP)
                     or state.error_code != 0)):
            raise ConversionError(
                'arm error/estop state occurs inside the crop at %.6f'
                % state.timestamp)
    return {
        'commands_in_crop': len(commands),
        'acknowledged_commands': sum(acknowledged),
        'superseded_stream_commands': len(superseded),
        'superseded_stream_sequence_ids': superseded,
    }


def _apply_command(target, command):
    updated = list(target)
    if command.mode in ARM_TARGET_MODES:
        updated[:4] = command.target[:4]
    elif command.mode in WRIST_TARGET_MODES:
        updated[4] = command.target[4]
    elif command.mode in GRIPPER_TARGET_MODES:
        updated[5] = command.target[5]
    return tuple(updated)


def _held_targets(commands, timestamps):
    first = next(
        (command for command in commands
         if command.mode in ACTIVE_TARGET_MODES), None)
    if first is None:
        raise ConversionError('cannot initialize the held target')
    _check_finite(first.target, 'initial command target')
    target = tuple(first.target)
    result = []
    command_index = 0
    for timestamp in timestamps:
        while (command_index < len(commands)
               and commands[command_index].timestamp <= timestamp + 1e-9):
            command = commands[command_index]
            _check_finite(command.target, 'command target')
            target = _apply_command(target, command)
            command_index += 1
        result.append(target)
    return result


def _latest_record(records, timestamps, timestamp, label, max_age):
    index = bisect_right(timestamps, timestamp + 1e-9) - 1
    if index < 0:
        raise ConversionError('no causal %s is available at %.6f'
                              % (label, timestamp))
    record = records[index]
    age = timestamp - record.timestamp
    if age > max_age + 1e-9:
        raise ConversionError(
            '%s is stale by %.6fs at %.6f (limit %.6fs)'
            % (label, age, timestamp, max_age))
    return record, max(0.0, age)


def align_episode(streams, config=None):
    """Create causal 10 Hz observation/action samples from one episode."""
    config = config or AlignmentConfig()
    config.validate()
    start, end = _action_bounds(streams.commands, config)
    start = max(start, streams.images[0].timestamp, streams.states[0].timestamp)
    end = min(end, streams.images[-1].timestamp, streams.states[-1].timestamp)
    if end <= start:
        raise ConversionError('episode has no overlapping image/state/action window')

    lifecycle_report = _validate_command_lifecycle(
        streams, start, end)
    step = 1.0 / config.fps
    frame_count = math.floor((end - start) * config.fps + 1e-9) + 1
    if frame_count < 2:
        raise ConversionError('aligned episode contains fewer than two frames')
    timestamps = [start + index * step for index in range(frame_count)]
    targets = _held_targets(streams.commands, [
        start + index * step for index in range(frame_count + 1)])
    image_timestamps = [record.timestamp for record in streams.images]
    valid_states = tuple(
        record for record in streams.states if record.position_valid)
    if not valid_states:
        raise ConversionError('episode contains no position-valid arm states')
    invalid_state_messages_in_crop = 0
    consecutive_invalid_states = 0
    for record in streams.states:
        if not start <= record.timestamp <= end:
            continue
        if record.position_valid:
            consecutive_invalid_states = 0
            continue
        invalid_state_messages_in_crop += 1
        consecutive_invalid_states += 1
        if consecutive_invalid_states > 1:
            raise ConversionError(
                'consecutive position_valid=false states inside crop at %.6f'
                % record.timestamp)
    state_timestamps = [record.timestamp for record in valid_states]
    all_state_timestamps = [record.timestamp for record in streams.states]

    frames = []
    max_image_age = 0.0
    max_state_age = 0.0
    movement = 0.0
    state_fallback_frames = 0
    for index, timestamp in enumerate(timestamps):
        image, image_age = _latest_record(
            streams.images, image_timestamps, timestamp, 'image',
            config.max_image_age_sec)
        raw_state_index = bisect_right(
            all_state_timestamps, timestamp + 1e-9) - 1
        state_age_limit = config.max_state_age_sec
        if (raw_state_index >= 0
                and not streams.states[raw_state_index].position_valid):
            state_fallback_frames += 1
            state_age_limit += step
        state, state_age = _latest_record(
            valid_states, state_timestamps, timestamp, 'state',
            state_age_limit)
        if state.state in (STATE_ERROR, STATE_ESTOP) or state.error_code != 0:
            raise ConversionError(
                'invalid arm state inside crop at %.6f' % state.timestamp)
        if len(state.joint_position) < 5:
            raise ConversionError(
                'ArmState joint_position has fewer than 5 values')
        state_values = (*state.joint_position[:5], state.gripper_position)
        _check_finite(state_values, 'observation.state')
        if not 0.0 <= state.gripper_position <= 1.0:
            raise ConversionError(
                'gripper state is outside [0, 1] at %.6f'
                % state.timestamp)

        current_target = targets[index]
        next_target = targets[index + 1]
        action = tuple(
            next_target[axis] - current_target[axis] for axis in range(5)
        ) + (next_target[5],)
        _check_finite(action, 'action')
        if not 0.0 <= action[5] <= 1.0:
            raise ConversionError(
                'gripper action is outside [0, 1] at %.6f' % timestamp)
        movement += sum(abs(value) for value in action[:5])
        if index:
            movement += abs(action[5] - frames[-1].action[5])

        frames.append(AlignedFrame(
            source_timestamp=timestamp,
            image=image,
            state=tuple(state_values),
            action=action,
            state_age_sec=state_age,
            image_age_sec=image_age,
        ))
        max_image_age = max(max_image_age, image_age)
        max_state_age = max(max_state_age, state_age)

    if movement <= 1e-9:
        raise ConversionError('aligned episode contains no target movement')

    report = {
        'source_start_timestamp': start,
        'source_end_timestamp': timestamps[-1],
        'duration_sec': timestamps[-1] - start,
        'frames': len(frames),
        **lifecycle_report,
        'invalid_state_messages_in_crop': invalid_state_messages_in_crop,
        'state_fallback_frames': state_fallback_frames,
        'max_image_age_sec': max_image_age,
        'max_state_age_sec': max_state_age,
        'max_abs_action': [
            max(abs(frame.action[index]) for frame in frames)
            for index in range(5)
        ] + [max(frame.action[5] for frame in frames)],
    }
    return tuple(frames), report


def _resize_with_pad(image_bytes, size):
    try:
        import numpy as np
        from PIL import Image
    except ImportError as exc:
        raise ConversionError('numpy and Pillow are required for export') from exc

    try:
        with Image.open(BytesIO(image_bytes)) as source:
            image = source.convert('RGB')
            ratio = max(image.width / size, image.height / size)
            width = max(1, int(image.width / ratio))
            height = max(1, int(image.height / ratio))
            image = image.resize((width, height), resample=Image.Resampling.BILINEAR)
            padded = Image.new('RGB', (size, size), 0)
            padded.paste(image, ((size - width) // 2, (size - height) // 2))
            return np.asarray(padded, dtype=np.uint8)
    except Exception as exc:
        raise ConversionError('cannot decode compressed image: %s' % exc) from exc


def _header_to_bag_latency(records):
    values = sorted(
        record.bag_timestamp - record.header_timestamp
        for record in records if record.header_timestamp > 0)
    if not values:
        return None

    def quantile(fraction):
        return values[round((len(values) - 1) * fraction)]

    return {
        'min_sec': values[0],
        'p50_sec': quantile(0.5),
        'p99_sec': quantile(0.99),
        'max_sec': values[-1],
    }


def _dataset_features(image_size, storage):
    return {
        IMAGE_KEY: {
            'dtype': storage,
            'shape': (3, image_size, image_size),
            'names': ['channels', 'height', 'width'],
        },
        STATE_KEY: {
            'dtype': 'float32',
            'shape': (len(STATE_NAMES),),
            'names': list(STATE_NAMES),
        },
        ACTION_KEY: {
            'dtype': 'float32',
            'shape': (len(ACTION_NAMES),),
            'names': list(ACTION_NAMES),
        },
    }


def export_dataset(
        episode_dirs, output, repo_id, *, allow_unreviewed=False,
        image_topic='/image', state_topic='/arm/state_filtered', clock='bag',
        message_dir=None, config=None, image_size=224,
        storage='image', processing_plan=None):
    """Atomically export one or more rosbag2 episodes as LeRobot v2.x."""
    config = config or AlignmentConfig()
    if image_size <= 0:
        raise ConversionError('image_size must be positive')
    if storage not in ('image', 'video'):
        raise ConversionError('storage must be image or video')
    try:
        import numpy as np
        from lerobot.common.datasets.lerobot_dataset import (
            CODEBASE_VERSION,
            LeRobotDataset,
        )
    except ImportError as exc:
        raise ConversionError(
            'run this exporter inside the OpenPI environment so its pinned '
            'LeRobot package is available') from exc

    if processing_plan is not None and not isinstance(processing_plan, dict):
        processing_plan = load_processing_plan(processing_plan)
    episode_dirs = [Path(value).expanduser().resolve() for value in episode_dirs]
    if not episode_dirs:
        raise ConversionError('no episode directories were provided')
    if len(episode_dirs) != len(set(episode_dirs)):
        raise ConversionError('duplicate episode directory was provided')
    manifests = [load_manifest(episode_dir) for episode_dir in episode_dirs]
    episode_ids = [
        manifest.get('episode_id', episode_dir.name)
        for episode_dir, manifest in zip(episode_dirs, manifests)]
    if len(episode_ids) != len(set(episode_ids)):
        raise ConversionError('duplicate episode_id was provided')
    if processing_plan is not None:
        expected_ids = {
            entry['episode_id'] for entry in processing_plan['entries']
            if entry['decision'] in ('success_usable', 'success_crop')}
        provided_ids = set(episode_ids)
        if provided_ids != expected_ids:
            missing = sorted(expected_ids - provided_ids)
            unexpected = sorted(provided_ids - expected_ids)
            raise ConversionError(
                'episode selection does not match processing plan; '
                'missing=%s unexpected=%s' % (missing, unexpected))
        for episode_dir, manifest, episode_id in zip(
                episode_dirs, manifests, episode_ids):
            expected_path = (processing_plan['source_root'] / episode_id).resolve()
            if episode_dir != expected_path:
                raise ConversionError(
                    'episode path does not match processing plan source_root: %s'
                    % episode_dir)
            if manifest.get('status') not in ('unreviewed', 'success'):
                raise ConversionError(
                    'processing plan cannot override source status %r: %s'
                    % (manifest.get('status'), episode_id))
    else:
        for manifest in manifests:
            validate_manifest_for_export(manifest, allow_unreviewed)

    output = Path(output).expanduser().resolve()
    if output.exists():
        raise ConversionError('output already exists: %s' % output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(
        '.%s.tmp-%s' % (output.name, uuid.uuid4().hex[:8]))
    reports = []
    dataset = None
    try:
        dataset = LeRobotDataset.create(
            repo_id=repo_id,
            root=temporary,
            robot_type='armbot',
            fps=config.fps,
            features=_dataset_features(image_size, storage),
            use_videos=storage == 'video',
            image_writer_threads=4,
            image_writer_processes=0,
        )
        for episode_dir, manifest, episode_id in zip(
                episode_dirs, manifests, episode_ids):
            streams = read_episode_streams(
                episode_dir,
                image_topic=image_topic,
                state_topic=state_topic,
                clock=clock,
                message_dir=message_dir,
            )
            decision = None
            derivation_audit = None
            if processing_plan is not None:
                plan_entry = processing_plan['by_id'][episode_id]
                decision = plan_entry['decision']
                if decision == 'success_crop':
                    streams, derivation_audit = apply_derivation(
                        streams, plan_entry['derivation'])
            frames, alignment_report = align_episode(streams, config)
            task = str(manifest.get('task', '')).strip()
            if not task:
                raise ConversionError('manifest task is empty: %s' % episode_dir)
            for frame in frames:
                dataset.add_frame({
                    IMAGE_KEY: _resize_with_pad(frame.image.data, image_size),
                    STATE_KEY: np.asarray(frame.state, dtype=np.float32),
                    ACTION_KEY: np.asarray(frame.action, dtype=np.float32),
                    'task': task,
                })
            dataset.save_episode()
            reports.append({
                'episode_id': episode_id,
                'source': str(episode_dir),
                'source_status': manifest.get('status'),
                'qc_decision': decision,
                'derivation': derivation_audit,
                'task': task,
                'clock': clock,
                'image_header_to_bag_latency': _header_to_bag_latency(
                    streams.images),
                'state_header_to_bag_latency': _header_to_bag_latency(
                    streams.states),
                **alignment_report,
            })
        dataset.stop_image_writer()
        conversion = {
            'schema_version': 1,
            'repo_id': repo_id,
            'lerobot_codebase_version': CODEBASE_VERSION,
            'fps': config.fps,
            'image_size': [image_size, image_size],
            'image_storage': storage,
            'features': {
                IMAGE_KEY: 'uint8 RGB, aspect-preserving zero padding',
                STATE_KEY: list(STATE_NAMES),
                ACTION_KEY: list(ACTION_NAMES),
            },
            'action_semantics': (
                'next 1/fps-second held Cartesian target delta; gripper '
                'remains an absolute [0, 1] target'),
            'episodes': reports,
        }
        if processing_plan is not None:
            conversion['processing_plan'] = {
                'file': 'meta/armbot_processing_plan.json',
                'sha256': processing_plan['sha256'],
                'reviewed_at_utc': processing_plan['reviewed_at_utc'],
                'review_method': processing_plan['review_method'],
                'counts': processing_plan['counts'],
            }
        meta_dir = temporary / 'meta'
        meta_dir.mkdir(parents=True, exist_ok=True)
        if processing_plan is not None:
            (meta_dir / 'armbot_processing_plan.json').write_bytes(
                processing_plan['raw'])
        (meta_dir / 'armbot_conversion.json').write_text(
            json.dumps(conversion, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8')
        os.replace(temporary, output)
    except Exception:
        if dataset is not None:
            try:
                dataset.stop_image_writer()
            except Exception:
                pass
        if temporary.exists():
            shutil.rmtree(temporary)
        raise
    return conversion


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Convert Armbot rosbag2 episodes to OpenPI LeRobot data.')
    parser.add_argument(
        'episode', nargs='*', help=(
            'Raw episode directories; omitted with --processing-plan to use '
            'all success entries from its source_root'))
    parser.add_argument('--output', required=True, help='New dataset directory')
    parser.add_argument(
        '--repo-id', default='local/armbot_pi05',
        help='LeRobot repository id stored in metadata')
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--pre-roll-sec', type=float, default=0.2)
    parser.add_argument('--post-roll-sec', type=float, default=0.5)
    parser.add_argument('--max-image-age-sec', type=float, default=0.12)
    parser.add_argument('--max-state-age-sec', type=float, default=0.15)
    parser.add_argument('--image-size', type=int, default=224)
    parser.add_argument('--storage', choices=('image', 'video'), default='image')
    parser.add_argument('--image-topic', default='/image')
    parser.add_argument('--state-topic', default='/arm/state_filtered')
    parser.add_argument(
        '--clock', choices=('bag', 'header'), default='bag',
        help='Causal alignment clock; bag matches policy-time message arrival')
    parser.add_argument('--message-dir')
    parser.add_argument(
        '--processing-plan', help=(
            'Audited external QC plan used to select and derive immutable '
            'unreviewed episodes'))
    parser.add_argument(
        '--allow-unreviewed', action='store_true',
        help='Pipeline testing only; production export requires success status')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    config = AlignmentConfig(
        fps=args.fps,
        pre_roll_sec=args.pre_roll_sec,
        post_roll_sec=args.post_roll_sec,
        max_image_age_sec=args.max_image_age_sec,
        max_state_age_sec=args.max_state_age_sec,
    )
    try:
        processing_plan = (
            load_processing_plan(args.processing_plan)
            if args.processing_plan else None)
        episode_dirs = list(args.episode)
        if processing_plan is not None and not episode_dirs:
            episode_dirs = [
                processing_plan['source_root'] / entry['episode_id']
                for entry in processing_plan['entries']
                if entry['decision'] in ('success_usable', 'success_crop')]
        result = export_dataset(
            episode_dirs,
            args.output,
            args.repo_id,
            allow_unreviewed=args.allow_unreviewed,
            image_topic=args.image_topic,
            state_topic=args.state_topic,
            clock=args.clock,
            message_dir=args.message_dir,
            config=config,
            image_size=args.image_size,
            storage=args.storage,
            processing_plan=processing_plan,
        )
    except ConversionError as exc:
        print('export_lerobot: %s' % exc, file=sys.stderr)
        return 2
    except Exception as exc:
        print('export_lerobot: unexpected failure: %s' % exc, file=sys.stderr)
        return 1
    print(json.dumps({
        'output': str(Path(args.output).expanduser().resolve()),
        'episodes': len(result['episodes']),
        'frames': sum(item['frames'] for item in result['episodes']),
    }, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
