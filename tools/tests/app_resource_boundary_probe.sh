#!/bin/bash -p
case "$-" in
  *p*) ;;
  *)
    printf '%s\n' \
      'APP_RESOURCE_BOUNDARY_FAIL: execute the probe directly or with /bin/bash -p' >&2
    exit 78
    ;;
esac
set -euo pipefail
RESOURCE_PATH=/usr/bin:/bin:/usr/sbin:/sbin
PATH="$RESOURCE_PATH"
export PATH
RESOURCE_ENV=/usr/bin/env
RESOURCE_HERMETIC_HOME=/var/empty
RESOURCE_PYTHON=/usr/bin/python3

if [ -L "$RESOURCE_HERMETIC_HOME" ] || \
  [ ! -d "$RESOURCE_HERMETIC_HOME" ]; then
  printf '%s\n' \
    'APP_RESOURCE_BOUNDARY_FAIL: credential-blind HOME is unavailable' >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
INSPECTOR="$REPO_ROOT/tools/release/resource_boundary.py"

if [ ! -f "$INSPECTOR" ] || [ -L "$INSPECTOR" ]; then
  printf 'resource-boundary inspector is missing or a symlink: %s\n' \
    "$INSPECTOR" >&2
  exit 2
fi

exec "$RESOURCE_ENV" -i \
  PATH="$RESOURCE_PATH" \
  HOME="$RESOURCE_HERMETIC_HOME" \
  LANG=C \
  LC_ALL=C \
  "$RESOURCE_PYTHON" -I "$INSPECTOR" "$REPO_ROOT" "$@"
