import argparse
import json
from pathlib import Path

import pytest

from vla_dataset import episode
from vla_dataset import record_episode


def test_validates_task_and_firmware_hash():
    assert episode.validate_task('  pick cube  ') == 'pick cube'
    assert episode.validate_firmware_sha256(None) == 'unknown'
    assert episode.validate_firmware_sha256('A' * 64) == 'a' * 64
    with pytest.raises(ValueError):
        episode.validate_task('   ')
    with pytest.raises(ValueError):
        episode.validate_firmware_sha256('abc')


def test_normalizes_and_validates_topics():
    assert episode.normalize_topics('/image', ['/joy', '/camera/info']) == [
        '/image', '/joy', '/arm/command', '/arm/state',
        '/arm/state_filtered', '/camera/info']
    with pytest.raises(ValueError):
        episode.normalize_topics('image')
    with pytest.raises(ValueError):
        episode.normalize_topics('/camera//image')


def test_build_record_command_uses_humble_positional_topics(tmp_path):
    command = record_episode.build_record_command(
        tmp_path / 'bag', ['/image', '/arm/state'])
    assert command == [
        'ros2', 'bag', 'record', '-o', str(tmp_path / 'bag'),
        '/image', '/arm/state']


def test_write_manifest_replaces_existing_file(tmp_path):
    path = tmp_path / 'manifest.json'
    episode.write_manifest(path, {'status': 'recording'})
    episode.write_manifest(path, {'status': 'unreviewed'})
    assert json.loads(path.read_text(encoding='utf-8')) == {
        'status': 'unreviewed'}
    assert not Path(str(path) + '.tmp').exists()


def test_run_finalizes_successful_episode(tmp_path, monkeypatch):
    class FakeProcess:
        def wait(self, timeout=None):
            return 0

    monkeypatch.setattr(record_episode, 'git_snapshot', lambda _repo: {
        'commit': 'test', 'dirty': False})
    monkeypatch.setattr(
        record_episode.subprocess, 'Popen',
        lambda *args, **kwargs: FakeProcess())
    args = argparse.Namespace(
        task='pick cube', output_root=str(tmp_path), image_topic='/image',
        extra_topics=[], firmware_sha256='1' * 64)

    assert record_episode.run(args) == 0
    manifests = list(tmp_path.glob('episode_*/manifest.json'))
    assert len(manifests) == 1
    manifest = json.loads(manifests[0].read_text(encoding='utf-8'))
    assert manifest['status'] == 'unreviewed'
    assert manifest['software']['firmware_sha256'] == '1' * 64
    assert manifest['topics'][0] == '/image'


def test_run_preserves_failed_episode(tmp_path, monkeypatch):
    def fail_to_start(*_args, **_kwargs):
        raise FileNotFoundError('ros2 not found')

    monkeypatch.setattr(record_episode, 'git_snapshot', lambda _repo: {
        'commit': 'test', 'dirty': False})
    monkeypatch.setattr(record_episode.subprocess, 'Popen', fail_to_start)
    args = argparse.Namespace(
        task='pick cube', output_root=str(tmp_path), image_topic='/image',
        extra_topics=[], firmware_sha256=None)

    assert record_episode.run(args) == 1
    manifest_path = next(tmp_path.glob('episode_*/manifest.json'))
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    assert manifest['status'] == 'failed'
    assert manifest['recorder_exit_code'] == 1
