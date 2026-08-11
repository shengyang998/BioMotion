#!/bin/bash
set -euo pipefail
PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
INSPECTOR="$REPO_ROOT/tools/release/resource_boundary.py"

if [ ! -f "$INSPECTOR" ] || [ -L "$INSPECTOR" ]; then
  printf 'resource-boundary inspector is missing or a symlink: %s\n' \
    "$INSPECTOR" >&2
  exit 2
fi

/usr/bin/python3 -I "$INSPECTOR" "$REPO_ROOT" "$@"
