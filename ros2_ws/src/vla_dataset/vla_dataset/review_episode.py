"""Record an explicit human review result in an episode manifest."""

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

from vla_dataset.episode import write_manifest
from vla_dataset.lerobot_export import ConversionError, load_manifest


def review_episode(episode_dir, result, notes=''):
    episode_dir = Path(episode_dir).expanduser().resolve()
    manifest = load_manifest(episode_dir)
    if manifest.get('status') != 'unreviewed':
        raise ConversionError(
            'only an unreviewed episode can be reviewed; current status is %r'
            % manifest.get('status'))
    if result not in ('success', 'failure'):
        raise ConversionError('review result must be success or failure')
    notes = notes.strip()
    if len(notes) > 1000:
        raise ConversionError('review notes must be at most 1000 characters')

    reviewed_at = datetime.now(timezone.utc).isoformat()
    manifest['status'] = result
    manifest['review'] = {
        'result': result,
        'reviewed_at_utc': reviewed_at,
        'notes': notes,
    }
    write_manifest(episode_dir / 'manifest.json', manifest)
    return manifest


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Mark one unreviewed VLA episode as success or failure.')
    parser.add_argument('episode', help='Raw episode directory')
    parser.add_argument('--result', required=True, choices=('success', 'failure'))
    parser.add_argument('--notes', default='')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        manifest = review_episode(args.episode, args.result, args.notes)
    except ConversionError as exc:
        print('review_episode: %s' % exc, file=sys.stderr)
        return 2
    print('%s: %s' % (manifest['episode_id'], manifest['status']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
