#!/bin/bash -p
case "$-" in
  *p*) ;;
  *)
    printf '%s\n' \
      'DEPENDENCY_BOUNDARY_FAIL: execute the probe directly or with /bin/bash -p' >&2
    exit 78
    ;;
esac
set -euo pipefail
DEPENDENCY_PATH=/usr/bin:/bin:/usr/sbin:/sbin
PATH="$DEPENDENCY_PATH"
export PATH
DEPENDENCY_ENV=/usr/bin/env
DEPENDENCY_HERMETIC_HOME=/var/empty
DEPENDENCY_PYTHON=/usr/bin/python3

if [ -L "$DEPENDENCY_HERMETIC_HOME" ] || \
  [ ! -d "$DEPENDENCY_HERMETIC_HOME" ]; then
  printf '%s\n' \
    'DEPENDENCY_BOUNDARY_FAIL: credential-blind HOME is unavailable' >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
INSPECTOR="$REPO_ROOT/tools/release/dependency_boundary.py"

if [ ! -f "$INSPECTOR" ] || [ -L "$INSPECTOR" ]; then
  printf 'DEPENDENCY_BOUNDARY_FAIL: inspector is missing or a symlink: %s\n' \
    "$INSPECTOR" >&2
  exit 1
fi

case "$#:${1:-}" in
  0:)
    exec "$DEPENDENCY_ENV" -i \
      PATH="$DEPENDENCY_PATH" HOME="$DEPENDENCY_HERMETIC_HOME" \
      LANG=C LC_ALL=C \
      "$DEPENDENCY_PYTHON" -I "$INSPECTOR" "$REPO_ROOT"
    ;;
  1:--snapshot)
    exec "$DEPENDENCY_ENV" -i \
      PATH="$DEPENDENCY_PATH" HOME="$DEPENDENCY_HERMETIC_HOME" \
      LANG=C LC_ALL=C \
      "$DEPENDENCY_PYTHON" -I "$INSPECTOR" snapshot "$REPO_ROOT"
    ;;
  *)
    printf 'usage: %s [--snapshot]\n' "$0" >&2
    exit 2
    ;;
esac
