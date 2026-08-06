"""CLI that records one passive VLA episode with rosbag2."""

import argparse
import os
from pathlib import Path
import platform
import signal
import subprocess

from vla_dataset.episode import create_episode_dir
from vla_dataset.episode import find_git_root
from vla_dataset.episode import git_snapshot
from vla_dataset.episode import normalize_topics
from vla_dataset.episode import utc_now
from vla_dataset.episode import validate_firmware_sha256
from vla_dataset.episode import validate_task
from vla_dataset.episode import write_manifest


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Record one passive Armbot VLA episode with rosbag2.')
    parser.add_argument('--task', required=True, help='Natural-language task')
    parser.add_argument(
        '--output-root', default='~/vla_episodes',
        help='Parent directory for episode folders')
    parser.add_argument('--image-topic', default='/image')
    parser.add_argument(
        '--topic', action='append', default=[], dest='extra_topics',
        help='Additional absolute ROS topic; may be repeated')
    parser.add_argument('--firmware-sha256')
    return parser.parse_args(argv)


def build_record_command(bag_dir, topics):
    return ['ros2', 'bag', 'record', '-o', str(bag_dir), *topics]


def stop_recorder(process):
    if process.poll() is not None:
        return process.returncode, False
    os.killpg(process.pid, signal.SIGINT)
    try:
        return process.wait(timeout=15), False
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
    try:
        return process.wait(timeout=5), True
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        return process.wait(), True


def run(args):
    task = validate_task(args.task)
    firmware_sha256 = validate_firmware_sha256(args.firmware_sha256)
    topics = normalize_topics(args.image_topic, args.extra_topics)
    started_at = utc_now()
    episode_id, episode_dir = create_episode_dir(
        args.output_root, started_at)
    manifest_path = episode_dir / 'manifest.json'
    bag_dir = episode_dir / 'bag'
    repo = find_git_root(Path.cwd())
    manifest = {
        'schema_version': 1,
        'episode_id': episode_id,
        'task': task,
        'status': 'recording',
        'started_at_utc': started_at.isoformat(),
        'ended_at_utc': None,
        'duration_sec': None,
        'topics': topics,
        'bag_path': 'bag',
        'recorder_exit_code': None,
        'forced_shutdown': False,
        'software': {
            'armbot': git_snapshot(repo),
            'firmware_sha256': firmware_sha256,
            'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        },
        'host': platform.node(),
    }
    write_manifest(manifest_path, manifest)

    process = None
    exit_code = 1
    forced_shutdown = False
    requested_stop = False
    error = None
    try:
        process = subprocess.Popen(
            build_record_command(bag_dir, topics), start_new_session=True)
        print('Recording %s; press Ctrl+C to stop.' % episode_id, flush=True)
        exit_code = process.wait()
    except KeyboardInterrupt:
        requested_stop = True
        if process is not None:
            exit_code, forced_shutdown = stop_recorder(process)
    except OSError as exc:
        error = str(exc)
    finally:
        ended_at = utc_now()
        manifest['ended_at_utc'] = ended_at.isoformat()
        manifest['duration_sec'] = max(
            0.0, (ended_at - started_at).total_seconds())
        manifest['recorder_exit_code'] = exit_code
        manifest['forced_shutdown'] = forced_shutdown
        stopped_cleanly = requested_stop and exit_code in (0, -signal.SIGINT)
        if (exit_code == 0 or stopped_cleanly) and not forced_shutdown:
            manifest['status'] = 'unreviewed'
        else:
            manifest['status'] = 'failed'
            manifest['error'] = error or 'rosbag2 recorder exited abnormally'
        write_manifest(manifest_path, manifest)

    print('Episode: %s' % episode_dir, flush=True)
    return 0 if manifest['status'] == 'unreviewed' else 1


def main(argv=None):
    def request_stop(_signum, _frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        return run(parse_args(argv))
    except ValueError as exc:
        print('record_episode: %s' % exc, flush=True)
        return 2
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == '__main__':
    raise SystemExit(main())
