#!/bin/bash
set -euo pipefail
PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TEST_ROOT="$(/usr/bin/mktemp -d "${TMPDIR:-/tmp}/biomotion-profile-cms-tests.XXXXXX")"
trap '/bin/rm -r -- "$TEST_ROOT" 2>/dev/null || true' EXIT

/usr/bin/openssl req -x509 -newkey rsa:2048 -nodes -days 1 \
  -subj '/C=US/O=Fixture/CN=Test Profile Signer' \
  -keyout "$TEST_ROOT/key.pem" -out "$TEST_ROOT/certificate.pem" \
  >/dev/null 2>&1

/usr/bin/python3 - "$TEST_ROOT/payload.plist" <<'PY'
from pathlib import Path
import plistlib
import sys

with Path(sys.argv[1]).open("wb") as output:
    plistlib.dump({"Name": "BioMotionMutableValue"}, output)
PY

/usr/bin/openssl cms -sign -binary -nodetach \
  -in "$TEST_ROOT/payload.plist" \
  -signer "$TEST_ROOT/certificate.pem" \
  -inkey "$TEST_ROOT/key.pem" \
  -outform DER -out "$TEST_ROOT/original.mobileprovision" \
  >/dev/null 2>&1

/usr/bin/python3 - \
  "$TEST_ROOT/original.mobileprovision" \
  "$TEST_ROOT/mutated.mobileprovision" <<'PY'
from pathlib import Path
import sys

source = Path(sys.argv[1]).read_bytes()
needle = b"BioMotionMutableValue"
if source.count(needle) != 1:
    raise SystemExit("fixture CMS did not embed the expected payload bytes")
Path(sys.argv[2]).write_bytes(source.replace(needle, b"XioMotionMutableValue", 1))
PY

run_verifier() {
  /usr/bin/python3 -I - "$REPO_ROOT" "$1" <<'PY'
import importlib.util
from pathlib import Path
import sys

repo = Path(sys.argv[1])
spec = importlib.util.spec_from_file_location(
    "resource_boundary", repo / "tools/release/resource_boundary.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.decode_signed_profile(Path(sys.argv[2]), "fixture profile")
PY
}

expect_failure() {
  local label="$1"
  local expected="$2"
  local profile="$3"
  local output
  local status
  set +e
  output="$(run_verifier "$profile" 2>&1)"
  status=$?
  set -e
  if [[ "$status" -eq 0 || "$output" != *"$expected"* ]]; then
    printf '%s failed for the wrong reason (status %s): %s\n' \
      "$label" "$status" "$output" >&2
    exit 1
  fi
}

# A correctly signed but non-Apple CMS reaches the pinned-root check. This
# distinguishes trust rejection from a parser-only rejection.
expect_failure untrusted_signer \
  'CMS signer chain is not trusted' \
  "$TEST_ROOT/original.mobileprovision"

# An equal-length payload mutation leaves the plist parseable but must fail the
# CMS messageDigest/signature check before any trust or entitlement inspection.
expect_failure mutated_payload \
  'CMS content signature is invalid' \
  "$TEST_ROOT/mutated.mobileprovision"

printf '%s\n' 'PROFILE_CMS_VERIFIER_TESTS_PASS 2/2'
