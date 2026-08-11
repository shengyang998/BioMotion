# Shipping the SAM 3D Body model as a Background Assets pack

The Core ML pose model is **not** in the app binary. It ships as an
**Apple-Hosted Managed Background Assets** pack (iOS 26). ODR is not used — it is
deprecated as of iOS 27.

Measured on the Release device archive:

| | app download |
|---|---|
| before (model bundled) | **1.3151 GiB** — of which `SAM3DBodyPose.mlmodelc` was 1.3084 GiB |
| after (model in a pack) | **0.0069 GiB** (7.0 MiB) |
| the pack itself | 1.0210 GiB `.aar` (1.31 GiB uncompressed) |

The model never changes between app versions, so the OS also stops re-downloading
it on every app update.

---

## Locked supply-chain contract

The 1.3 GiB model is intentionally not committed. Instead, the repository pins
the only accepted source package, compiled model, license, provenance, and Core
ML interface in:

- `BioMotion/Resources/SAM3DBodyPose.lock.json`
- `BioMotion/Resources/SAM-LICENSE.txt`
- `tools/assetpack/verify_model_lock.py`

The stable asset-pack identifier is exactly `sam3d-body-pose`. Verify the
checked-in contract and license without the large artifact:

```bash
/usr/bin/python3 tools/assetpack/verify_model_lock.py repository
/usr/bin/python3 tools/assetpack/verify_model_lock.py toolchain
/usr/bin/python3 tools/tests/assetpack_supply_chain_tests.py
/bin/bash tools/tests/assetpack_package_receipt_tests.sh
```

When the artifacts are present, verify every regular file, size, SHA-256, and
the complete Core ML interface:

```bash
/usr/bin/python3 tools/assetpack/verify_model_lock.py source \
  ../sam-3d-body/export/coreml/SAM3DBodyPose.mlpackage
/usr/bin/python3 tools/assetpack/verify_model_lock.py compiled \
  path/to/SAM3DBodyPose.mlmodelc
```

The verifier rejects unknown or duplicate JSON keys, unsafe paths, symlinks,
missing/extra/non-regular files, byte or license drift, unapproved archive
metadata, archive-size drift, and interface type, dtype, shape, optionality, or
flexibility drift. It uses absolute `/usr/bin/xcrun`, `/usr/bin/xcodebuild`, and
`/usr/bin/what` paths and requires the lock's exact Xcode, Core ML compiler, and
`ba-package` versions. The package driver itself uses `/bin/bash`, resets PATH
to `/usr/bin:/bin:/usr/sbin:/sbin`, and invokes `/usr/bin/python3`, so an
untrusted caller PATH cannot substitute its verifier or basic system tools.

**Integration status:** repository, source, compile, package, extraction,
receipt, and local upload-preflight enforcement are complete. Developer
bundling and runtime loading remain separate release gates. The existing App
Store Connect version 1 still lacks the lock and license and is not a compliant
shipping artifact; this slice did not upload a replacement. `--upload` remains
an explicit release operation and must not be used without release approval and
closure of the remaining delivery gates.

---

## Package

Run the only approved local package entry point:

```bash
/bin/bash tools/assetpack/package.sh \
  ../sam-3d-body/export/coreml/SAM3DBodyPose.mlpackage
```

The script first freezes private copies of Manifest/lock/license, then verifies
the frozen repository authority and locked Apple toolchain back-to-back. It
verifies the source package, compiles in a mode-0700 transaction directory,
verifies the compiled tree/interface, and stages exactly these selectors:

- `SAM3DBodyPose.mlmodelc`
- `SAM3DBodyPose.lock.json`
- `SAM-LICENSE.txt`

It writes a temporary AAR, uses `aa list` to reject paths/types/sizes before
extraction, extracts it privately, byte-compares all three authority files, and
re-verifies the compiled tree/interface. A strict JSON receipt binds schema,
pack/revision/model identity, AAR name/size/SHA-256, and all three authority
SHA-256 values. `seal` first copies the AAR, Manifest, lock, and license through
opened regular-file descriptors into one random mode-0700 sibling snapshot;
archive verification, receipt construction, and receipt bindings read only that
snapshot. It then proves the live inputs still equal the verified snapshot and
installs the complete receipt with an atomic no-clobber hard link. An ABA change
cannot switch generations under the receipt, and a partial receipt is never
visible at the final pathname. The publish gate repeats the full archive
verification against the current repository before making the pair visible.
This release boundary assumes host/account integrity: the random mode-0700
snapshot isolates other users and ordinary concurrent changes, but it is not a
sandbox against a malicious process already executing as the same macOS UID.

`coremlcompiler` emits the 503-byte root `coremldata.bin` with its four protobuf
metadata-map entries in random order. The private compiler output is normalized
only by permuting those entries, and only when one permutation reconstructs the
complete raw SHA-256 already pinned by the lock. Any value or other-byte drift
still fails closed; the packaged compiled tree remains byte-exact.

The final pair is:

```text
build/assetpack/release/sam3d-body-pose.aar
build/assetpack/release/sam3d-body-pose.aar.receipt.json
```

Any pre-migration files directly under `build/assetpack/` are historical local
artifacts; only the pair inside `release/` is a publication candidate.

The `release/` directory is the atomic publication unit: the first publication
uses one directory rename; replacements use Darwin
`renameatx_np(RENAME_SWAP)`. Before either namespace operation, the verified
candidate AAR, receipt, and candidate directory are fsynced; the affected parent
directories are fsynced after it. If a post-namespace fsync fails, the verifier
immediately performs the inverse rename/swap and fsyncs both parents again. A
durably completed rollback returns ordinary failure and restores the previous
namespace. If the rollback namespace or its fsync cannot be proven, the verifier
returns status 2 (`MODEL_LOCK_RECOVERY_REQUIRED`). Unexpected Python exceptions
inside the final publisher are converted to that same recovery status.
`package.sh` enters preservation mode *before* invoking the publisher and clears
it only after success or status 1, which is reserved for a caught, safely
classified `VerificationError` (including a durably completed rollback). A
signal or any other unclassified exit therefore retains the full transaction
and package lock and prints the candidate/destination recovery paths. Do not
remove that lock or rerun packaging until those paths are inspected and
reconciled. A crash cannot expose an AAR with a receipt from a different
generation while silently deleting the displaced release. Verify the published
pair (including another list and extraction) with:

```bash
/usr/bin/python3 tools/assetpack/verify_model_lock.py receipt \
  build/assetpack/release/sam3d-body-pose.aar \
  build/assetpack/release/sam3d-body-pose.aar.receipt.json
```

Direct `coremlcompiler` / `ba-package` commands are not an approved bypass.

`ba-package` resolves the manifest's relative `fileSelectors` against the **current
working directory**, which is why the stage directory is the CWD.

## Upload

The pack is uploaded on a **separate channel** from the app binary. With no
arguments, or with `--verify-only`, the script performs only the complete local
receipt/archive gate over the atomic published pair:

```bash
/bin/bash tools/assetpack/upload.sh
/bin/bash tools/assetpack/upload.sh --verify-only
```

These modes do not inspect App Store Connect credential variables or the API
key path, do not invoke `altool`, and do not make a network request. The
verifier's local `xcrun aa` list/extract operations are part of the archive
gate, not App Store Connect access.

Only the literal `--upload` flag authorizes the external upload and the
subsequent version-list request:

```bash
ASC_API_KEY_ID=YOUR_KEY_ID \
ASC_API_ISSUER=YOUR_ISSUER_UUID \
  /bin/bash tools/assetpack/upload.sh --upload
```

The target Apple ID is fixed to BioMotion's `6761994383`. Only after the full
receipt gate succeeds does the script require those two variables and a
non-symlink regular key at
`$HOME/.appstoreconnect/private_keys/AuthKey_${ASC_API_KEY_ID}.p8`. An
app-specific password is not accepted by `altool` for asset-pack upload. If the
upload succeeds but the following list request fails, the command returns
failure even though the remote upload may already have happened; inspect App
Store Connect before retrying.

Upload mode copies the canonical AAR and receipt into a random mode-0700 private
directory with mode-0600 files before verification. `/usr/bin/python3 -I`
performs the complete receipt/archive gate on that snapshot, and `altool` opens
the exact same snapshot AAR. A concurrent atomic replacement of `release/`
therefore cannot switch the uploaded generation after validation. The snapshot
survives through both authorized `altool` calls and is removed on ordinary and
error exits. Source or snapshot symlinks and non-regular files fail closed.

For a private candidate, `--aar` and `--receipt` must be provided together;
both must use the canonical filenames below in the same physical directory.
There is no AAR-only or positional bypass:

```bash
/bin/bash tools/assetpack/upload.sh --verify-only \
  --aar /private/candidate/sam3d-body-pose.aar \
  --receipt /private/candidate/sam3d-body-pose.aar.receipt.json
```

The driver fixes `/bin/bash`, a trusted system PATH, `/usr/bin/python3 -I`, and
`/usr/bin/xcrun`; it removes ambient `DEVELOPER_DIR`, `TOOLCHAINS`, and
`SDKROOT` selection before verification. The remaining boundary is the same
host/account-integrity boundary as packaging: a malicious process already
running as the same macOS UID is not contained, and the active system
`xcode-select` installation and credential store remain trusted. Do not bypass
this gate with a raw `altool` command.

## Ship the app

Nothing about archiving changes, except that the archive is now 7 MiB.

```bash
xcodegen generate
xcodebuild archive -project BioMotion.xcodeproj -scheme BioMotion -configuration Release \
  -destination 'generic/platform=iOS' -archivePath build/BioMotion.xcarchive \
  -allowProvisioningUpdates
```

Bump `CURRENT_PROJECT_VERSION` in `project.yml` first — it feeds **both** the app
and the extension, which must carry the same version or the upload warns
ITMS-90473.

TestFlight distribution on this Mac still needs the manual-signing path recorded
in memory (`project_ios_appstore_signing`): a self-created DISTRIBUTION
certificate plus an `IOS_APP_STORE` profile. Cloud signing is refused here. That
is unchanged by this work — and note the profile now has to cover the extension
bundle id `com.soleil.BioMotion.AssetPackDownloader` as well.

---

## How the app finds the model

`BioMotion/AssetPack/AssetPackModelStore.swift`, first hit wins:

1. `SAM3DBodyPose.mlmodelc` **in the app bundle** — developer/Simulator builds.
2. `SAM3DBodyPose.mlpackage` in the app bundle → `MLModel.compileModel(at:)`, cached
   in `Application Support/CompiledModels`.
3. `SAM3DBodyPose.mlmodelc` **in the asset pack** — the shipping path.
4. `SAM3DBodyPose.mlpackage` in the asset pack → compiled + cached.
5. Nothing: start the download in the background and throw immediately with a
   message carrying the live percentage.

The pack carries a **pre-compiled `.mlmodelc`**, not the `.mlpackage`. Xcode
compiles a bundled `.mlpackage` at build time; an asset-delivered one gets no
build step. Compiling on the Mac instead of the phone means the shipped
directory is the same artifact Xcode was already embedding (verified: `model.mil`,
`metadata.json`, `weights/weight.bin` and `analytics/coremldata.bin` are
byte-identical to the 2026-08-07 archive's copy; only the 503-byte root
`coremldata.bin` differs, and only in the key order of the coremltools metadata
dictionary), it avoids ~2.6 GiB of transient device disk, and it keeps the only
copy inside the OS-managed asset container where iOS can evict/update it.

### Local iteration / Simulator

Background Assets serves **no packs in the Simulator**, and only serves them to
App Store / TestFlight installs on a real device. So bundle a developer copy:

```bash
bash tools/assetpack/dev_bundle_model.sh on    # copies the .mlpackage to build/DevBundledModel/
xcodegen generate
# ... build/run ...
bash tools/assetpack/dev_bundle_model.sh off
xcodegen generate
```

`build/DevBundledModel` is an `optional:` source path in `project.yml`, so it
simply does not exist in a normal checkout — and it sits under `build/`, which
`.gitignore` already covers, so a 1.3 GiB copy can never be committed by
accident. **Never archive with it on** — that re-adds the full 1.31 GiB.

### When the pack is missing

`loadModelIfNeeded()` never blocks on the transfer. It starts the download and
throws immediately; `OfflineImportView` renders the message in its red "Error"
section, e.g.

> Couldn't load the pose model: Downloading the pose model — 37% (487 MB of 1310 MB).
> It keeps going in the background; try again when it finishes.

Tapping **Run** again re-reads the current percentage. `AssetPackModelStore` is an
`ObservableObject` publishing `.downloading(fraction:receivedBytes:totalBytes:)`,
so a live progress bar is one `@ObservedObject var store = SAM3DPoseEstimator.modelStore`
away in `OfflineImportView` — that view is not part of this change.

---

## Project wiring

| piece | where |
|---|---|
| `BAUsesAppleHosting` / `BAHasManagedAssetPacks` / `BAAppGroupID` | `project.yml` → generated `BioMotion/Info.plist` (never edit the plist) |
| app group `group.com.soleilyu.biomotion` | `BioMotion/AssetPack/Support/BioMotion.entitlements` + the extension's |
| Background Download extension | target `AssetPackDownloader`, `BioMotion/AssetPack/Downloader/` |
| runtime loader | `BioMotion/AssetPack/AssetPackModelStore.swift` |
| pack id `sam3d-body-pose` | `Manifest.json`, `AssetPackModelStore.assetPackID`, `AssetPackIdentity.modelPackID` — all three must agree |

Only those three `BA*` Info.plist keys are legal in Apple-hosted mode; adding any
self-hosting key (`BAManifestURL`, …) invalidates the configuration.

## Verified / not verified

Verified on this machine (2026-08-07 through 2026-08-11):

* Release device archive builds and signs, extension embedded at
  `BioMotion.app/Extensions/AssetPackDownloader.appex`, app group present in both
  binaries' entitlements. App bundle 1.3151 → 0.0069 GiB.
* `sam3d-body-pose.aar` builds (1.0210 GiB) and lists
  `Contents/SAM3DBodyPose.mlmodelc/{analytics/coremldata.bin,coremldata.bin,metadata.json,model.mil,weights/weight.bin}`,
  `Contents/SAM3DBodyPose.lock.json`, `Contents/SAM-LICENSE.txt`, and the root
  `Manifest.json` — no other archive entry is accepted.
* On 2026-08-11, the final real package/receipt chain passed four times. The
  latest run exercised the private seal snapshot, atomic no-clobber receipt
  install, frozen-authority validation, trusted system tools, pre-publication
  fsync, and replacement of an existing release via the real atomic swap. Its
  published AAR is 1,096,258,817 bytes at SHA-256
  `910ba2f3c1578810d0202de782412ac8f52e5f3f13529f70acd7747a7f29d7db`;
  its 722-byte receipt passes a fresh full list/extract verification. AAR bytes
  can change between builds because Apple Archive records filesystem metadata;
  each receipt binds its exact generation.
* The self-contained verifier suite passes **60/60** under `/usr/bin/python3`
  and the package transaction suite passes **16/16**. Failure injection covers
  compiler/package errors, extra/missing/unsafe/symlink/hash entries, authority
  and AAR mutation during sealing, ABA generation swaps, partial receipt writes,
  public receipt rejection of a text AAR, receipt drift, root symlink writes,
  end-to-end atomic-swap failure, fstat/fsync/close errors, first-publication and
  replacement post-fsync rollback, rollback namespace/fsync failure, restoration
  of the previous release when provable, and preservation of the complete
  recovery transaction and lock when it is not, including unexpected publisher
  exceptions and signal termination after the namespace swap.
* The upload gate passes **11/11** hermetic causal groups (27 scenarios) with a
  copied fixture repository and fake verifier/`xcrun`; no real `altool` is
  reachable from the suite. Coverage includes default and explicit read-only
  modes, exact verifier and `altool` argv/order, receipt failure before any
  credential-path access, malformed/custom-pair arguments, missing/malformed
  credentials, missing/directory/symlink keys, upload/list failures, source
  symlinks, hostile PATH/PYTHONPATH/Xcode-selection environments, private
  snapshot modes and cleanup, and source replacement after verification while
  the exact verified snapshot generation is uploaded. The real 1,096,258,817-
  byte published AAR also passed default verification and an upload-mode local
  preflight; that preflight stopped on an intentionally empty API key ID after
  the receipt gate, cleaned the snapshot, and never entered `altool`. No real
  credential was inspected and no App Store Connect request or mutation was
  performed by this slice.
* Dev-bundled toggle produces `BioMotion.app/SAM3DBodyPose.mlmodelc` exactly where
  `Bundle.main.url(forResource:withExtension:)` looks.
* Simulator build + launch, and the existing test suite still builds and passes.
* The failure path, driven through the real UI on the Simulator (where the pack
  genuinely does not exist). `AssetPackManager` initialises without trapping —
  it takes **~2.5 s**, which is why the probe runs off the main actor — and the
  system log confirms the Info.plist is understood:
  `The app uses Apple hosting ("BAUsesAppleHosting" is set to "YES")`.
  The app then reports, in order and without hanging or crashing:
  1. `The 1.3 GB pose model isn't on this device yet — the download was just requested. Try again in a moment for progress, or the reason it can't start.`
  2. `Couldn't get the pose model from the App Store: No team ID was specified for the app with the bundle ID "com.soleil.BioMotion".`
  The second is the Simulator's real reason, surfaced on the next attempt once
  the asynchronous lookup has failed.
* Historical asset pack uploaded to App Store Connect: `sam3d-body-pose` version 1,
  `0 errors, 0 warnings`, version id `137b671a-f5dc-4c6c-9abd-1b1fad37eb16`,
  state **READY_FOR_TESTING**, platform IOS (it went PROCESSING →
  READY_FOR_TESTING in about 10 minutes). That AAR omitted
  `SAM-LICENSE.txt` and `SAM3DBodyPose.lock.json`; it is now obsolete and must
  not be used as the shipping compliance artifact.

**Not** verified, and only verifiable once a TestFlight build carrying these
Info.plist keys is installed on a real device:

* That `AssetPackManager.url(for: FilePath("SAM3DBodyPose.mlmodelc"))` resolves a
  *directory* entry. `assetPackURL(for:)` has a second attempt that asks for
  `SAM3DBodyPose.mlmodelc/coremldata.bin` and takes its parent, in case the
  manager only indexes leaf files. One of the two must work; which one is
  unknown from here.
* Real download progress numbers, and that `MLModel(contentsOf:)` loads the
  pack's copy. Every earlier link in that chain is verified; the model itself is
  the artifact that already loaded on device from the app bundle.
