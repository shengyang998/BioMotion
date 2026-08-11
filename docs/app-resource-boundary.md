# App and test resource boundary

BioMotion treats Xcode resource membership and the final built bundle as release
inputs, not incidental build output. The gate is:

```bash
/bin/bash -p tools/tests/app_resource_boundary_probe.sh
```

With no artifact argument it validates the checked-in sources, `project.yml`, and
generated `project.pbxproj`. It pins the two OpenSim models, privacy manifest, root
`NOTICE`, consolidated `THIRD-PARTY-NOTICES.txt`, complete app-icon source catalog,
test-fixture filename inventory, and the exact tracked TestFlight export plist. It
also resolves PBX group ancestry, requires the exact target-scoped resource
memberships rather than trusting display names, and rejects a generated project whose
app/extension version no longer matches `project.yml`.

The separate native-dependency gate is also mandatory before a test or archive:

```bash
/bin/bash -p tools/tests/dependency_boundary_probe.sh
```

It pins Xcode 26.4 build `17E192` and iPhoneOS SDK 26.4 build `23E237`, the
root PBXProject and exactly the three reviewed native targets. Their complete
Debug/Release setting surfaces, configuration-list ownership, dependency edges,
ordered phases and referenced build files must match. Extra targets,
framework/package linkage, per-file flags, base xcconfigs, tool overrides and
unattached graph objects fail closed. The project container itself may contain
only the generated pbxproj and canonical self workspace; schemes, `xcuserdata`
and sidecars are not accepted. The two developer-model phases embed the exact
guard script pinned at SHA-256
`a83bd4b5fbafb6442358ce6dd06627c574514a97c2fa7c28bf8750c8a29223d6`.

## Artifact modes

Use exactly one mode when validating a built artifact:

```bash
# Ordinary Debug or Release Simulator app smoke test.
/bin/bash -p tools/tests/app_resource_boundary_probe.sh \
  --simulator-smoke /path/to/BioMotion.app

# Test resource smoke test; pass the .xctest itself, not its containing app.
/bin/bash -p tools/tests/app_resource_boundary_probe.sh \
  --tests-bundle-smoke /path/to/BioMotionTests.xctest

# Distribution gate; the argument must be the signed device archive itself.
/bin/bash -p tools/tests/app_resource_boundary_probe.sh \
  --release-archive /path/to/BioMotion.xcarchive

# Final distribution gate after Xcode locally exports/re-signs the archive.
/bin/bash -p tools/tests/app_resource_boundary_probe.sh \
  --release-ipa /path/to/BioMotion.ipa /path/to/BioMotion.xcarchive
```

`--simulator-smoke` proves only a Simulator product. It checks the exact app and
extension file/directory allowlists, byte-identical models and legal resources,
known app icons and compiled asset-catalog records, bundle metadata, Mach-O images,
and size budgets. It is never accepted as release provenance.

`--tests-bundle-smoke` checks the exact two model copies and all seven reviewed
fixtures in a Simulator `.xctest`. This separate mode avoids confusing a test host's
embedded test bundle with shipping app resources.

`--release-archive` accepts only a `.xcarchive` whose archive receipt identifies the
BioMotion scheme, app path, bundle identifier, team, non-empty signing identity, and
arm64 architecture. The app and extension must identify as iPhoneOS 26.0 products,
carry the exact reviewed inventory, contain no symlink or special entry, and expose
arm64 Mach-O `IOS` build receipts. Each bundle must be signed by an Apple Distribution
certificate for the reviewed team, carry the exact reviewed signed entitlements, and
embed an unexpired App Store provisioning profile whose bundle id, team, authorized
certificate, and entitlements agree with the code signature. Provisioning profiles are
not merely decoded: their CMS content signature and signer chain must validate to the
pinned classic Apple Root CA, and the signer must carry Apple's provisioning-profile
identity and private extension. A final strict deep signature verification closes the
stable-artifact check. Passing an app directory by itself, an ad-hoc signed archive, a
tampered-but-parseable profile, or self-reported platform plist fields are deliberately
insufficient release evidence.

Both the embedded profile and the leaf distribution certificate must retain at least
30 days of validity. This is a release-renewal window, not merely an expiration check.

`--release-ipa` safely extracts exactly one `Payload/BioMotion.app` into a private
directory. It rejects traversal, NUL/truncated names, duplicate/case-colliding paths,
symlinks, special entries, unreviewed compression, and size/entry-count expansion. It
independently binds every raw central-directory record to its local header, decodes the
actual STORED or DEFLATED byte span under the total expansion budget, and requires the
real EOF, CRC, compressed/uncompressed sizes, and optional data descriptor to agree.
Only reviewed timestamp/UID-GID extra fields are accepted; alternate path/type metadata
and entry/archive comments are rejected rather than left to extractor interpretation.
This prevents a ZIP parser from accepting a declared one-byte entry that Apple's
extractor would expand into megabytes. Unix file types and execute bits are also part
of the contract: the app and extension code images must be executable, ordinary
resources must not be, and the exported execute-bit inventory must match the archive.
The exported app then repeats the iPhoneOS resource, Mach-O, profile, entitlement, and
signature checks and must match its reviewed archive byte-for-byte outside
`_CodeSignature/CodeResources`, `embedded.mobileprovision`, and normalized Mach-O
signatures. The mode also reruns the privacy gate and prints the final IPA SHA-256.

Create the archive through the controlled wrapper, then use the fail-fast export
driver instead of assembling those steps manually. Both protected shell entry
points must be executed directly through their `#!/bin/bash -p` shebang or with
`/bin/bash -p`:

```bash
/bin/bash -p tools/release/archive_release.sh \
  --archive build/BioMotion.xcarchive
/bin/bash -p tools/release/testflight_release.sh \
  --archive build/BioMotion.xcarchive \
  --export-dir build/testflight-30
```

An unprotected `bash script.sh` invocation is unsupported and is not release
evidence. In that mode an inherited `BASH_ENV` can execute before the script
starts, so the script may never reach its own guard or environment cleanup.
The archive wrapper also supplies fresh private DerivedData and derives the
active SDK's exact static-library paths from the dependency lock. Tracked
Nimble, Eigen/tinyxml2 and OSQP headers precede ignored generated roots; those
roots may expose only the locked configuration headers.

In default mode the wrapper makes no explicit App Store Connect validate/upload
request. The tracked export plist forces `destination=export`; `--validate`
explicitly adds remote validation, and only `--upload` authorizes validation followed
by upload of the same private, byte-pinned snapshot. The real
`xcodebuild -exportArchive` process is not network-sandboxed, so this is not a claim
that every default-mode subprocess is physically unable to use the network. Controlled
archive/export does not use `-allowProvisioningUpdates`.

This gate assumes a trusted, quiescent same-user build machine; it is not a sandbox
against a malicious process already running as the same macOS UID. Such a process
could race pathname-based inspection or change the archive after the command returns.
Keep the archive quiescent, run the gate immediately before export, and treat arbitrary
same-account code execution as a compromised release host. The adjacent dependency
receipt proves that every tracked blob in the Nimble, OSQP and two QDLDL checkouts
matched the recorded `HEAD` at both archive observations; physical ignored/untracked
files outside the reviewed build roots are rejected. It is not a signature or a
link-map/dependency-closure proof that every inspected source, header or archive
member contributed to the executable.

## Developer model boundary

`build/DevBundledModel` is a local escape hatch for Background Assets, which does not
serve packs in the Simulator. It is allowed only for a Debug iOS Simulator build.
`tools/release/reject_dev_model.sh` runs as the app target's first and last build
phases: the first invocation rejects an enabled source model for every other
configuration/platform, and the last rescans the copied product for `.mlmodel`,
`.mlpackage`, or `.mlmodelc`. A device, Release, archive, or distribution build must
first run:

```bash
/bin/bash -p tools/assetpack/dev_bundle_model.sh off
xcodegen generate
```

The build guards reduce the chance of an accidental large-model archive; the final
archive gate remains authoritative.

## Maintainer checks

Run the causal test suites whenever the allowlist, assets, fixtures, notices, build
phases, or archive layout changes:

```bash
/bin/bash -p tools/tests/app_resource_boundary_probe_tests.sh
/bin/bash -p tools/tests/release_dev_model_guard_tests.sh
/bin/bash -p tools/tests/profile_cms_verifier_tests.sh
/bin/bash -p tools/tests/testflight_release_gate_tests.sh
```

The resource suite is green **42/42**; the TestFlight wrapper suite is green
**15/15**. The resource suite constructs accepted source, arm64 iOS Simulator app, and test-bundle
fixtures plus an ad-hoc arm64 device archive that must be rejected, then mutates one
boundary at a time. Its negative cases cover
wrong target membership, stale guards, changed source identities, nested/renamed
models, extra OSIM or large data, hidden asset-catalog entries, altered legal files,
symlinks, unsafe IPA paths, forged ZIP expansion or semantic metadata, ZIP comments,
missing executable bits, macOS images relabelled as iOS, and wrong archive provenance.
The CMS suite signs a parseable fixture then mutates its embedded plist
without changing its length, proving payload parsing cannot substitute for signature
verification. The TestFlight suite causally proves gate-before-export ordering,
credential deferral, unique IPA selection, byte-change rejection, and
validate-before-upload ordering; default mode makes no explicit App Store Connect
validate/upload call. The successful Apple Distribution path
requires a real certificate/private key and App Store profiles, so it is exercised by
the release-host archive and IPA verification rather than forged inside the
self-contained suites.

The legal files prove notice delivery only. They do not resolve the separate
commercial-use restriction recorded for the MoBL-ARMS-derived content in
`FullBody.osim`; close that owner/legal decision before commercial distribution.
