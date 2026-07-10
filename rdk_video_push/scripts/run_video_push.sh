#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_PATH="${1:-$PROJECT_DIR/config.yaml}"

cd "$PROJECT_DIR"

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is not installed." >&2
  exit 1
fi

exec python3 "$PROJECT_DIR/main.py" --config "$CONFIG_PATH"
