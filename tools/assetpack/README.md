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
python3 tools/assetpack/verify_model_lock.py repository
python3 tools/tests/assetpack_supply_chain_tests.py
```

When the artifacts are present, verify every regular file, size, SHA-256, and
the complete Core ML interface:

```bash
python3 tools/assetpack/verify_model_lock.py source \
  BioMotion/CoreML/SAM3DBodyPose.mlpackage
python3 tools/assetpack/verify_model_lock.py compiled \
  build/assetpack/stage/SAM3DBodyPose.mlmodelc
```

The verifier rejects unknown or duplicate JSON keys, unsafe paths, symlinks,
missing/extra/non-regular files, byte drift, license drift, and interface type,
dtype, shape, optionality, or flexibility drift.

**Integration status:** the lock/verifier is the completed foundation. The
current `package.sh`, `upload.sh`, developer bundle, and runtime loader predate
it and are not yet release-approved. Do not package or upload until the next
integration commits make the lock mandatory at every entry point and emit a
verifiable receipt. The existing App Store Connect version 1 lacks the lock and
license and must not be treated as a compliant shipping artifact.

---

## Package

`package.sh` currently compiles `SAM3DBodyPose.mlpackage` with
`xcrun coremlcompiler`, then archives it with `xcrun ba-package`. That older
path does not yet enforce the checked-in lock or re-extract its AAR, so it is
documented for migration context only and must not be used for a release.
Direct `coremlcompiler` / `ba-package` commands are also not an approved bypass.

`ba-package` resolves the manifest's relative `fileSelectors` against the **current
working directory**, which is why the stage directory is the CWD.

## Upload

The pack is uploaded on a **separate channel** from the app binary. The current
script reads credentials before it proves the AAR and has no receipt or
verification-only mode, so neither it nor a raw `altool` call is an approved
release path. The next integration slice must verify the repository contract,
receipt, re-extracted AAR, and compiled model before it is even allowed to read
App Store Connect credentials; upload must then require an explicit flag.

Check state (both read-only):

```bash
xcrun altool --list-asset-packs --apple-id 6761994383 \
  --apiKey 4KH2G3HUYG --apiIssuer 25194e91-5f40-43b4-b598-98a189994f54
xcrun altool --list-asset-pack-versions --apple-id 6761994383 \
  --asset-pack-identifier sam3d-body-pose \
  --apiKey 4KH2G3HUYG --apiIssuer 25194e91-5f40-43b4-b598-98a189994f54
```

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

Verified on this machine (2026-08-07):

* Release device archive builds and signs, extension embedded at
  `BioMotion.app/Extensions/AssetPackDownloader.appex`, app group present in both
  binaries' entitlements. App bundle 1.3151 → 0.0069 GiB.
* `sam3d-body-pose.aar` builds (1.0210 GiB) and lists
  `Contents/SAM3DBodyPose.mlmodelc/{analytics/coremldata.bin,coremldata.bin,metadata.json,model.mil,weights/weight.bin}`
  — the directory selector preserves the `.mlmodelc` directory rather than
  flattening it.
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
