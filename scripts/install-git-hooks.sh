#!/bin/sh

set -eu

repo_root=$(git rev-parse --show-toplevel 2>/dev/null) || {
    printf '%s\n' 'ERROR: Run this script from inside a Git repository.' >&2
    exit 1
}
cd "$repo_root"

hook_path='.githooks/pre-push'
if [ ! -f "$hook_path" ]; then
    printf 'ERROR: Versioned hook not found: %s\n' "$hook_path" >&2
    exit 1
fi

existing_hooks_path=$(git config --local --get core.hooksPath 2>/dev/null || true)
if [ -n "$existing_hooks_path" ] && [ "$existing_hooks_path" != '.githooks' ]; then
    printf '%s\n' \
        "ERROR: core.hooksPath is already set to '$existing_hooks_path'." \
        'Merge the existing hooks into .githooks before changing this setting.' >&2
    exit 1
fi

git_common_dir=$(git rev-parse --git-common-dir)
case "$git_common_dir" in
    /*) ;;
    *) git_common_dir="$repo_root/$git_common_dir" ;;
esac
legacy_hook="$git_common_dir/hooks/pre-push"

if [ -e "$legacy_hook" ] || [ -L "$legacy_hook" ]; then
    printf '%s\n' \
        "ERROR: Existing hook found at '$legacy_hook'." \
        'Merge its logic into .githooks/pre-push before installing versioned hooks.' >&2
    exit 1
fi

chmod +x "$hook_path"
git config --local core.hooksPath .githooks

configured_hooks_path=$(git config --local --get core.hooksPath)
if [ "$configured_hooks_path" != '.githooks' ] || [ ! -x "$hook_path" ]; then
    printf '%s\n' 'ERROR: Git hook installation verification failed.' >&2
    exit 1
fi

printf 'core.hooksPath=%s\n' "$configured_hooks_path"
printf 'pre-push hook=%s (executable)\n' "$hook_path"
printf '%s\n' 'Git hooks installed successfully for this repository.'

