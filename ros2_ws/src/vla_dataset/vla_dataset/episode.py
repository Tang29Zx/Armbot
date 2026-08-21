"""Pure helpers for episode identity, provenance, and manifests."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess
import uuid


CONTROL_TOPICS = (
    '/joy',
    '/arm/command',
    '/arm/state',
    '/arm/state_filtered',
)
SHA256_PATTERN = re.compile(r'^[0-9a-fA-F]{64}$')
TOPIC_PATTERN = re.compile(r'^/[A-Za-z0-9_/]+$')


def validate_task(task):
    value = task.strip()
    if not value:
        raise ValueError('task must not be empty')
    if len(value) > 500:
        raise ValueError('task must be at most 500 characters')
    return value


def validate_firmware_sha256(value):
    if value is None:
        return 'unknown'
    if not SHA256_PATTERN.fullmatch(value):
        raise ValueError('firmware SHA-256 must contain exactly 64 hex digits')
    return value.lower()


def normalize_topics(image_topic, extra_topics=()):
    topics = []
    for topic in (image_topic, *CONTROL_TOPICS, *extra_topics):
        if not TOPIC_PATTERN.fullmatch(topic) or '//' in topic:
            raise ValueError('invalid absolute ROS topic: %s' % topic)
        if topic not in topics:
            topics.append(topic)
    return topics


def utc_now():
    return datetime.now(timezone.utc)


def create_episode_dir(output_root, started_at):
    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    episode_id = '%s_%s' % (
        started_at.strftime('episode_%Y%m%dT%H%M%SZ'), uuid.uuid4().hex[:8])
    episode_dir = root / episode_id
    episode_dir.mkdir()
    return episode_id, episode_dir


def find_git_root(start):
    current = Path(start).expanduser().resolve()
    for candidate in (current, *current.parents):
        if (candidate / '.git').exists():
            return candidate
    return None


def git_snapshot(repo):
    if repo is None:
        return {'commit': 'unknown', 'dirty': None}
    try:
        commit = subprocess.run(
            ['git', '-C', str(repo), 'rev-parse', 'HEAD'],
            check=True, capture_output=True, text=True).stdout.strip()
        dirty = bool(subprocess.run(
            ['git', '-C', str(repo), 'status', '--porcelain'],
            check=True, capture_output=True, text=True).stdout.strip())
        return {'commit': commit, 'dirty': dirty}
    except (OSError, subprocess.CalledProcessError):
        return {'commit': 'unknown', 'dirty': None}


def write_manifest(path, manifest):
    target = Path(path)
    temporary = target.with_suffix(target.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(
            manifest, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, target)
