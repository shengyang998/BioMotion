# BioMotion

Pose-analysis iOS app. ARKit or imported-video tracking feeds Nimble
IK, kinematics-only findings, gait contact timing, and a 3-D anatomy view.

## Current status

The **local MVP code/test boundary is complete** as of 2026-08-12. The final
protected runner passed all **698** fast tests plus the one slow E1 experiment,
with zero failures, skips, expected failures, or test-host restarts, and a fresh
unsigned generic-device Release build passed the reviewed source, privacy,
resource, build-setting, and binary-literal gates. The final integration also
closed imported-video ownership, the app-side Asset Pack recovery state/UI
contract, live-calibration integrity, capture-atomic recording/frozen export,
and Release UI isolation.

The replacement Apple-hosted `sam3d-body-pose` Asset Pack version 2 is
**READY_FOR_TESTING**, and BioMotion 1.0.0 build 32 reached TestFlight state
**VALID** on 2026-08-12. Build 32 fixes the build-31 device failure that asked
Background Assets for one compiled-model leaf and then tried to open its parent
package without package-wide access. The uploaded IPA is only 1,542,789 bytes;
the 1,096,258,817-byte model remains a separately hosted asset and is not in
the main app bundle. This does **not** mean App Store distribution is ready.
Commercial rights for the MoBL-ARMS-derived upper-limb material, a full
TestFlight/device product smoke using the hosted model and real photo/video
inputs, and the required product/review metadata remain separate release
gates. See [`STATUS.md`](./STATUS.md) for the exact receipts and owner
decisions.

The two bundled OpenSim models do **not** currently publish dynamics. Their
`ContactGeometrySet`s are empty, and the active near-CoP routine does not add a
validated support polygon, unilateral-contact, or friction constraint in their
place. Production therefore fails closed before inverse dynamics: joint torque,
ground-reaction force, centre of pressure, muscle effort, and gait-load values
stay absent. A cleaner clip, a longer clip, or an explicit floor cannot supply
the missing foot-support model. The native raw-ID entry points are absent from
the public header and every Release binary. A Debug-host diagnostics category,
declared only to XCTest, retains the older numbers solely for unvalidated
engineering characterization; Release-configuration tests do not support it.

## Release UI boundary

Release builds expose the scientific product outputs: joint-angle trajectories,
gait timing, posture findings, calibration quality, and actionable model/import
state. Engineering panels, backend names, self-tests, frame checksums, raw solver
metrics, unsupported torque controls, and internal file/tool paths are compiled
only when the BioMotion target has the exact `BIOMOTION_INTERNAL_UI` condition.
That condition exists only in Debug; a compile-time guard rejects it outside
Debug. It is not a preference, `UserDefaults`, or remotely switchable feature.

Failures cross view boundaries as typed product failures with fixed public copy.
Underlying framework errors remain optional internal diagnostics and are never
written into a shared export warning. The resource gate checks the Swift guard
shape and scans the Release Mach-O for the reviewed internal literals, while the
dependency gate pins the Debug-only build-setting surface in both `project.yml`
and the generated project.

## Live recording and export

A live take is one atomic capture, not three independent buffers. Pressing the
record button creates a unique capture ID and one uptime-clock origin shared by
the AR marker frames and Nimble IK/ID history. Recording is refused until body
tracking and the musculoskeletal model are ready. A take stops both sides
together on a user stop, the 3,600-frame or 60-second limit, tracking loss,
offline-import entry, app deactivation, or an internal solver reset.
An AR frame already queued before the button press is rejected before it can
enter the new take's native IK warm-start or temporal filters.

Stopping freezes an immutable export snapshot before any tracking/session reset
can clear the engine. A completed take cannot be replaced silently: starting a
new one requires exporting it or explicitly confirming discard. A successful
share with no activity error marks only that exact capture as exported;
cancelling or failing the share, sharing only a warning with no motion-data
file, or receiving a late completion from an older share sheet leaves the
current take protected.

Export runs from the frozen snapshot on a detached worker. The `.trc`, `.mot`,
and capability-valid `.sto` use the same time origin and UUID basename in a
fresh temporary directory; the TRC `PathFileType` header carries that same
UUID filename. If the bundled model cannot provide validated joint
torques, the shared bundle carries `BioMotion_export_warnings.txt` instead of
inventing an STO. Temporary files are removed when sharing finishes. The live
import, record/stop, and export controls expose explicit VoiceOver labels,
values, and hints.

## Repo layout

```
labs/BioMotion/
├── BioMotion/              # Swift + ObjC++ source (~1.2MB)
├── BioMotion.xcodeproj/    # generated by XcodeGen — do not edit by hand
├── BioMotionTests/
├── project.yml             # XcodeGen project definition
├── nimblephysics/          # third-party — clone the maintained fork at the pinned receipt
├── osqp/                   # third-party — clone at the locked receipt
├── tools/dependencies.lock.json # native source/build receipt
├── CLAUDE.md               # architecture + gotchas (LLM context)
└── README.md               # this file — setup + build
```

The `nimblephysics/` and `osqp/` folders are third-party C++ libraries. They are
**not** committed to the project repo. Clone the maintained Nimble fork at the
exact receipt below, then build the static archives that the iOS app links
against.

The large SAM3DBodyPose Core ML artifact is also not committed. Its accepted
source/compiled file trees, hashes, toolchain provenance, exact interface, and
license are pinned by `BioMotion/Resources/SAM3DBodyPose.lock.json` and checked
by `tools/assetpack/verify_model_lock.py`. The lock/verifier is the supply-chain
foundation; packaging, upload, and runtime enforcement are tracked as separate
fail-closed integration steps in `tools/assetpack/README.md` and `STATUS.md`.

## What ships in the binary

A historical development `.ipa` measured about 3.5 MB; no current controlled
distribution IPA is claimed. The repo on disk can balloon to 1.2 GB if you keep
the upstream nimblephysics tree intact, but almost all of that is never linked
into the app:

| Path | Size | Required for build? |
|------|------|---------------------|
| `BioMotion/`, `BioMotion.xcodeproj/`, `BioMotionTests/`, `project.yml` | ~1.3MB | yes |
| `nimblephysics/` clean pinned checkout | upstream checkout size | yes (to rebuild the static archives) |
| `nimblephysics/build_ios/libnimble_ios.a`, `build_sim/libnimble_ios.a` | ~47MB | yes (or rebuild from source) |
| `nimblephysics/build_xcframework/NimbleIOS.xcframework` | 47MB | **no** — legacy output, not linked |
| `osqp/` source | ~14MB | yes |
| `osqp/build_ios/out/libosqpstatic.a`, `build_sim/out/libosqpstatic.a` | generated | yes (or rebuild from source) |
| `nimblephysics/data/` (osim/grf/c3d datasets) | 674MB | **no** — research fixtures |
| `nimblephysics/javascript/`, `python/` | ~80MB | **no** — language bindings |
| `nimblephysics/unittests/`, `www/`, `wiki_resources/` | ~14MB | **no** |
| `build/` (Xcode DerivedData spillover) | ~15MB | **no** |

Do **not** delete tracked directories from `nimblephysics/` to make a transfer
smaller. Those datasets and language bindings are not linked into the app, but
they are tracked by the pinned fork; deleting them creates thousands of local
changes and invalidates the clean-checkout receipt. For a new machine, transfer
the outer project without `nimblephysics/` and clone the pinned fork there.
Generated outer `build/` output is also unnecessary for transfer.

## Fresh setup on a new Mac

### 1. Toolchain

- Xcode 26.4 (build `17E192`) with iPhoneOS SDK 26.4 (build `23E237`) and an arm64 iOS 26
  Simulator installed. The app target is iOS 26.0. The native static archives
  intentionally retain an independent iOS 17.0 minimum deployment target.
- Command line tools: `xcode-select --install`
- CMake 3.24 or newer, Ninja, and XcodeGen:
  ```bash
  brew install cmake ninja xcodegen
  ```

Homebrew supplies build tools only. Nimble's Eigen and tinyxml2 sources are
vendored at pinned revisions in its fork; the iOS target does not discover
Boost or dependency headers from Homebrew.

The frozen SAM artifact was also compiled with Xcode 26.4,
`coremlcompiler` 3520.5.1 and `ba-package` 1.2. Verify that separate toolchain
receipt before rebuilding or packaging the model:

```bash
/usr/bin/python3 tools/assetpack/verify_model_lock.py toolchain
```

### 2. Clone the project

```bash
git clone <your-project-repo> labs/BioMotion
cd labs/BioMotion
```

### 3. Clone the pinned BioMotion nimblephysics fork

```bash
git clone --branch biomotion/ios-static-c405b05 --single-branch \
  https://github.com/shengyang998/nimblephysics.git
git -C nimblephysics checkout --detach \
  0ecf26a1557ee738146511cd81fbe99f2bc94d38
```

On a default case-insensitive macOS volume, Git may warn that upstream's
`data/osim/NoArms_v3/Delp1990.osim` and `delp1990.osim` collide. Both names pin
the same blob, neither is an iOS build input, and the checkout remains clean;
the dual-SDK archive receipts below are unchanged.

The maintained fork is the complete source of truth for the BioMotion port: it
contains the reviewed behavior changes, standalone iOS CMake target, public
header boundary, generated-config template, and pinned Eigen/tinyxml2 sources
and licenses. Do not clone upstream and reproduce this state with hand edits,
and do not apply `nimble-patches/*.patch` on top of this checkout.

The historical patch files remain in this repository for audit, baseline replay,
and `git apply --reverse --check` provenance checks. See
`nimble-patches/README.md`; they are not the fresh-setup mechanism.

The reviewed commits are published on the maintained
[`biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05)
branch. At the 2026-08-11 integration receipt, `git ls-remote` resolved that
branch to the same exact SHA checked out above:
`0ecf26a1557ee738146511cd81fbe99f2bc94d38`.

The OpenSim parser patch rejects unsupported or incomplete joint topology
instead of substituting a plausible-but-wrong joint. For valid CustomJoints it
also preserves non-identity functions and explicit coordinate mappings rather
than silently simplifying them to a different motion.

The iOS build does not provide collision simulation. Direct detector creation,
factory creation through the `"dart"` key, both `ConstraintSolver` and
`BoxedLcpConstraintSolver` constructor forms, `World()` and `World::create()`
all reject with the same descriptive `std::runtime_error`; they no longer
return a null detector and crash at a later dereference. A factory-only probe
verifies that ordinary dead-stripped static-archive linking extracts the
fail-closed implementation without `-all_load` or `-force_load`. This is a
refusal boundary, not collision/contact support.

The iOS build also does not provide ezc3d file loading or its force-platform
adapter. `C3D` remains a usable value type, including its `ForcePlate` data,
and `OpenSimParser::loadMotAtLowestMarkerRMSERotation(..., C3D&, ...)` remains
linked. `C3DLoader`, the loader-only weighted-convention heuristic, and the
`ForcePlatform` / `ForcePlatforms` ezc3d adapters are absent from the iOS public
surface. Consumers must compile with `DART_IOS_BUILD=1`, as the current app,
tests, CMake target, and probes do.

The XML scalar/vector helpers no longer depend on Boost or a machine-specific
Homebrew include path. Their standard-library replacement pins the classic
locale for XML decimal dots, preserves the reviewed `std::bad_cast`, `errno`,
formatting, tokenization, and unsigned-conversion behavior, and is checked in
both simulator and device archives for zero Boost symbols.

OpenSim mesh geometry is also an explicit unavailable capability on iOS. Both
`parseOsim` overloads reject `ignoreGeometry=false` with one reviewed
`std::runtime_error` before reading a URI, inspecting XML, or consulting a
retriever. BioMotion passes `ignoreGeometry=true` explicitly at every supported
model-loading call site. The archive probe ordinary-links both SDK variants and
runs both refusal paths in an iOS simulator; the XCTest contract separately
pins FullBody/Rajagopal parsing and bridge-marker counts with geometry ignored.

Three desktop OpenSim conversion utilities are absent from the iOS archive and
are no longer advertised there: `translateOsimMarkers`, `convertOsimToSDF`, and
`convertOsimToMJCF`. Their declarations, implementations, and MarkerFitter/SDF/
MJCF dependencies are paired behind `DART_IOS_BUILD`; desktop keeps the complete
surface. Both `parseOsim` overloads plus `loadTRC`, `loadMot`, `loadGRF`, and the
C3D rotation consumer remain public and linked. A comment-aware source contract,
dual-SDK symbol/count checks, ordinary dead-strip links, and a simulator run pin
both sides of that boundary.

Mesh-backed anthropometric scoring is also unavailable on iOS. The public
`Anthropometrics` surface now retains its data-only load, metric, distribution,
conditioning, and metric-pose methods while hiding the GUI, mesh measurement,
PDF, and gradient APIs. `IKErrorReport` continues to support its prior-free
path; a non-null anthropometric prior throws one descriptive runtime error
before reading or mutating skeleton state. Desktop keeps the complete scoring
surface. Dual-SDK current-source object, archive, link-map, and simulator probes
pin that split without relying on stale archive members.

`MeshShape` and `SoftMeshShape` themselves also fail closed when
`DART_IOS_BUILD=1`. Their headers expose only incomplete Assimp types, both
constructors and every backend-dependent method throw one pinned
`std::runtime_error`, and only type metadata plus destructors remain safe. A
failed `SoftBodyNode` construction now rolls back its notifier, point masses,
parent BodyNode entry, and hidden Jacobian-child entry. The archive probe
ordinary-links both SDK variants and runs fresh plus archived implementations
on an iOS simulator; a separate host fault-injection probe proves 32 root and
32 child rejection transactions, address-balanced notifier allocation, and an
ASan-clean normal path. This is a refusal boundary, not mesh rendering support.

Here “linear extrapolation” means continuation along an endpoint tangent only:
SimmSpline remains cubic inside its knot domain, and `MomentArmComputer` uses
that same evaluator for exact MovingPathPoint locations.

### 4. Clone the pinned OSQP checkout

```bash
git clone https://github.com/osqp/osqp.git osqp
git -C osqp checkout --detach \
  1572ae068e9ce9ca723cf8223548ade1ff7acc29
```

At the 2026-08-12 receipt, `git ls-remote` resolved OSQP `master` to that
exact SHA. The checkout is pinned by commit, not by the moving branch name.

### 5. Build Nimble for iOS

```bash
# iOS device (arm64 iphoneos)
cmake --fresh -S nimblephysics/ios -B nimblephysics/build_ios -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_SYSTEM_NAME=iOS \
  -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=17.0 \
  -DCMAKE_OSX_SYSROOT=iphoneos
cmake --build nimblephysics/build_ios --target nimble_ios --parallel

# iOS simulator (arm64 iphonesimulator)
cmake --fresh -S nimblephysics/ios -B nimblephysics/build_sim -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_SYSTEM_NAME=iOS \
  -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=17.0 \
  -DCMAKE_OSX_SYSROOT=iphonesimulator
cmake --build nimblephysics/build_sim --target nimble_ios --parallel
```

The standalone target is intentionally arm64-only for both device and
Simulator. Each configure generates its own `dart/config.hpp` beneath the
matching build directory. A source-tree `nimblephysics/dart/config.hpp` is
forbidden. Xcode searches the tracked Nimble, vendored Eigen/tinyxml2 and OSQP
source roots before either ignored build root; the build roots may contribute
only the locked `dart/config.hpp` and `osqp_configure.h` for the active SDK.
The target publicly exports `DART_IOS_BUILD=1`,
`DART_USE_IDENTITY_JACOBIAN=1`, `EIGEN_DONT_PARALLELIZE=1`, and
`EIGEN_MPL2_ONLY=1`.

Host-only fault and leak probes must opt in explicitly with
`-DNIMBLE_IOS_HOST_PROBE=ON`; that mode is not an iOS product build. The exact
commands and header-boundary probe live in `nimblephysics/ios/README.md`.

The generated Xcode project links `build_ios/libnimble_ios.a` and
`build_sim/libnimble_ios.a` directly according to the active SDK. The older
`build_xcframework/NimbleIOS.xcframework` is not part of the current link path.

### 6. Build OSQP

```bash
# Keep local checkout paths out of the object bytes. Run from the BioMotion
# repository root; the dependency gate requires these exact prefix maps.
BIOMOTION_REPO_ROOT="$(pwd -P)"

# iOS device (arm64 iphoneos)
cmake --fresh -S osqp -B osqp/build_ios -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_SYSTEM_NAME=iOS \
  -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=17.0 \
  -DCMAKE_OSX_SYSROOT=iphoneos \
  "-DCMAKE_C_FLAGS=-ffile-prefix-map=${BIOMOTION_REPO_ROOT}/osqp=osqp -ffile-prefix-map=${BIOMOTION_REPO_ROOT}/osqp/build_ios/_deps/qdldl-src=qdldl" \
  "-DCMAKE_C_FLAGS_RELEASE=-O3 -DNDEBUG" \
  -DOSQP_BUILD_SHARED_LIB=OFF -DOSQP_BUILD_STATIC_LIB=ON \
  -DOSQP_BUILD_DEMO_EXE=OFF -DOSQP_BUILD_UNITTESTS=OFF \
  -DOSQP_USE_FLOAT=OFF -DOSQP_USE_LONG=OFF \
  -DOSQP_ALGEBRA_BACKEND=builtin -DOSQP_ASAN=OFF \
  -DOSQP_CODEGEN=ON -DOSQP_ENABLE_DERIVATIVES=ON \
  -DOSQP_ENABLE_INTERRUPT=ON -DOSQP_ENABLE_PRINTING=ON \
  -DOSQP_ENABLE_PROFILING=ON -DOSQP_PACK_SETTINGS=OFF \
  -DOSQP_PROFILER_ANNOTATIONS=OFF \
  -DQDLDL_BUILD_SHARED_LIB=OFF -DQDLDL_BUILD_STATIC_LIB=OFF \
  -DQDLDL_DEV_ANALYSIS=OFF -DQDLDL_DEV_ASAN=OFF \
  -DQDLDL_DEV_COVERAGE=OFF -DQDLDL_FLOAT=OFF -DQDLDL_LONG=OFF
cmake --build osqp/build_ios --target osqpstatic --parallel

# iOS simulator (arm64 iphonesimulator)
cmake --fresh -S osqp -B osqp/build_sim -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_SYSTEM_NAME=iOS \
  -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=17.0 \
  -DCMAKE_OSX_SYSROOT=iphonesimulator \
  "-DCMAKE_C_FLAGS=-ffile-prefix-map=${BIOMOTION_REPO_ROOT}/osqp=osqp -ffile-prefix-map=${BIOMOTION_REPO_ROOT}/osqp/build_sim/_deps/qdldl-src=qdldl" \
  "-DCMAKE_C_FLAGS_RELEASE=-O3 -DNDEBUG" \
  -DOSQP_BUILD_SHARED_LIB=OFF -DOSQP_BUILD_STATIC_LIB=ON \
  -DOSQP_BUILD_DEMO_EXE=OFF -DOSQP_BUILD_UNITTESTS=OFF \
  -DOSQP_USE_FLOAT=OFF -DOSQP_USE_LONG=OFF \
  -DOSQP_ALGEBRA_BACKEND=builtin -DOSQP_ASAN=OFF \
  -DOSQP_CODEGEN=ON -DOSQP_ENABLE_DERIVATIVES=ON \
  -DOSQP_ENABLE_INTERRUPT=ON -DOSQP_ENABLE_PRINTING=ON \
  -DOSQP_ENABLE_PROFILING=ON -DOSQP_PACK_SETTINGS=OFF \
  -DOSQP_PROFILER_ANNOTATIONS=OFF \
  -DQDLDL_BUILD_SHARED_LIB=OFF -DQDLDL_BUILD_STATIC_LIB=OFF \
  -DQDLDL_DEV_ANALYSIS=OFF -DQDLDL_DEV_ASAN=OFF \
  -DQDLDL_DEV_COVERAGE=OFF -DQDLDL_FLOAT=OFF -DQDLDL_LONG=OFF
cmake --build osqp/build_sim --target osqpstatic --parallel
```

These options pin OSQP's C ABI to double precision plus 32-bit integers and
exclude demo, shared-library and unit-test targets from the product build.
OSQP fetches QDLDL v0.1.8 at
`138fdac58b9cd1c4137ff1b99152c8108a6cff5b`; the dependency gate verifies both
fetched checkouts as clean exact repositories. The prefix maps make the 29
named object members reproducible across checkout directories. Darwin's archive
symbol table itself is not deterministic, so the lock binds those normalized
member bytes rather than claiming a stable raw OSQP `.a` hash.

### 7. Generate the Xcode project and build

```bash
# Regeneration must happen first: the probe checks both project.yml and the
# generated pbxproj, so a stale or drifted generated project cannot reach Xcode.
xcodegen generate
/bin/bash -p tools/tests/dependency_boundary_probe.sh

# Compile check (no signing)
xcodebuild -project BioMotion.xcodeproj -scheme BioMotion \
  -destination 'generic/platform=iOS' \
  CODE_SIGN_IDENTITY=- CODE_SIGNING_REQUIRED=NO CODE_SIGNING_ALLOWED=NO build

# Tests on simulator — always via the script, never a hand-typed xcodebuild line.
# It provisions a private simulator (naming one shared with another xcodebuild
# process is what made this suite untrustworthy) and emits an xcresult receipt.
tools/run_tests.sh fast    # exactly 698; 0 failed/skipped/restarted
tools/run_tests.sh slow    # only testE1RunAll; exactly 1; 0 failed/skipped
tools/run_tests.sh all     # commit gate: fast, then slow

# A diagnostic selection. It must execute >= 1 test and is never a commit gate.
tools/run_tests.sh subset \
  -only-testing:BioMotionTests/SomeTests
```

Protected shell gates must be executed directly (using their `#!/bin/bash -p`
shebang) or with `/bin/bash -p` as shown. An unprotected `bash script.sh`
invocation is unsupported and is not evidence: an inherited `BASH_ENV` can run
before the script starts, so the script may never reach its own rejection or
sanitization. The dependency boundary suite is green **83/83**, and the runner's
independent gate-policy harness is green **53/53**. On the historical reviewed
Asset Pack slice—before the later calibration, capture-atomic recording,
Release-UI, and privacy-inventory work—the exact protected gate passed fast
**652/652** and slow **1/1**, with zero failures, skips, expected failures, or
test-host restarts; both structured receipts are under
`/tmp/biomotion-tests.SbMAmG`. Those are slice-specific local commit-gate
receipts, not the current 698+1 inventory and not a TestFlight hosted-delivery
receipt.

On the final integrated tree, the direct dependency, app-resource, and privacy
probes pass, the privacy adversarial harness is green **41/41**, and a fresh
unsigned generic-device Release build succeeds for the app plus asset-pack
extension with `BIOMOTION_INTERNAL_UI` absent. Both Release executables pass the
14-literal internal-diagnostics byte scan. This is local compile/binary evidence,
not a signed archive or TestFlight hosted-delivery receipt.

The final protected `tools/run_tests.sh all` receipt is under
`/tmp/biomotion-tests.DLG7RX`: fast passed **698/698** (runner wall 1,381 s;
xcresult interval 1,377.573 s) and slow E1 passed **1/1** (runner wall 6,084 s;
xcresult interval 6,080.845 s). Both structured results are `Passed`, both logs
contain `** TEST SUCCEEDED **`, all failure/skip/expected-failure/restart counts
are zero, and the runner ended with `ALL GATE PASS`. Only the five documentation
files named in the final documentation commit changed after the tested
product/test/tool sources, so that doc-only commit does not invalidate this
receipt.

`fast`, `slow`, and `all` accept no caller arguments; their fixed invocation is
part of the reviewed receipt. `subset` is the diagnostic escape hatch, but it
still rejects skips, alternate test configurations, and retry/repetition
controls. A lane passes only when
`xcodebuild` exits zero, the log ends in `TEST SUCCEEDED`, no test host
restarts, and the structured xcresult receipt has the exact reviewed count with
zero failures, skips, or expected failures. Missing evidence is a failure. See
STATUS.md, “The commit gate”.

## Regenerating the OpenSim moment-arm reference

`BioMotionTests/Fixtures/opensim_moment_arms.txt` is generated, not hand-written. It needs the
OpenSim Python API, which installs from PyPI on Apple Silicon (no conda required):

```bash
cd tools/opensim_ref
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python opensim numpy

./.venv/bin/python dump_reference.py        # ~5 min, writes ~98 MB of CSV to out/ (gitignored)
./.venv/bin/python analyse.py --write-fixture   # report + the committed 2.5 MB fixture
./.venv/bin/python pose_coverage.py         # what the pose grid covers, and what the clips say

# The DEFINITION-MATCHED column: OpenSim's own central difference of its own path
# length. `computeMomentArm` does NOT differentiate the length (it asks
# MomentArmSolver for a generalized force), and the two disagree by centimetres
# where a wrap solution is marginal, so this is the column a `-dL/dq`
# implementation is comparable with. 7 s, 649 KB.
./.venv/bin/python dump_finite_difference.py \
    --out ../../BioMotionTests/Fixtures/opensim_moment_arms_fd.txt

# Debugging aids, not part of the fixtures:
./.venv/bin/python fd_check.py --pose spine_flexed --muscle TR2_l --coordinates L2_L3_FE
./.venv/bin/python inspect_wrap.py --pose neutral --muscle gasmed_r
```

Everything here is read-only against `BioMotion/Resources/FullBody.osim`.

## Third-party notices

`BioMotion/Muscle/MusclePathWrap.{h,cpp}` is a port of OpenSim's path-wrapping code —
`WrapCylinder`, `WrapEllipsoid`, `WrapObject`, `WrapMath` and
`GeometryPath::applyWrapObjects` (Apache License 2.0, Stanford University). The licence
header is in both files and the attribution is in [`NOTICE`](./NOTICE). Complete selected
binary-redistribution terms for BioMotion, Nimble/DART, ODE, OSQP/QDLDL/AMD, Eigen,
tinyxml2, OpenSim, and the Rajagopal model are consolidated in
[`BioMotion/Resources/THIRD-PARTY-NOTICES.txt`](./BioMotion/Resources/THIRD-PARTY-NOTICES.txt).
Both files are explicit app resources and must ship byte-for-byte with every binary
distribution.

Those notices do not grant commercial rights to the 42 MoBL-ARMS-derived upper-limb
muscles in `FullBody.osim`. A commercial/App Store release remains blocked until the
owner obtains written commercial permission or replaces/removes that material. See
[`STATUS.md`](./STATUS.md) for the evidence and open owner decision.

A passing current **698+1** protected gate can establish integrated local-MVP
code/test completion; it cannot establish App Store readiness. Distribution
remains separately blocked by the arm-material rights decision; a final
TestFlight/device product
smoke covering hosted delivery/load/inference, real photo/video workflows,
Photos-provider lifecycle, Vision behavior, performance/memory, and
cancellation; and the required App Store Connect product/review metadata. The
hosted Asset Pack v2 and current build-32 archive/IPA/upload receipts now exist.

## TestFlight upload

### Build 32 recovery receipt (2026-08-12)

Build 31 installed, but a cellular-device Run failed with `The pose model could
not be opened. Update BioMotion, then try again.` The network was not the root
cause. `AssetPackModelStore` requested only
`SAM3DBodyPose.mlmodelc/coremldata.bin`, discarded the last path component, and
then gave Core ML that parent URL. The Background Assets URL authorized the
leaf, not necessarily sibling files such as `weights/weight.bin` that Core ML
needs to open the package. Build 32 requests the complete
`SAM3DBodyPose.mlmodelc` package directly and verifies its required metadata
leaf before loading it. The runtime source gate rejects the former leaf/parent
pattern.

BioMotion 1.0.0 build 32 was validated and uploaded without errors. App Store
Connect then reported `VALID`, `APP_STORE_ELIGIBLE`, `expired=false`, minimum
OS 26.0, and no non-exempt encryption. Delivery UUID:
`5b42b98a-d315-41de-ad29-05aafc7f38af`. The exact IPA is 1,542,789 bytes at
SHA-256
`60bbe6fa2633c0226a611fa07e4f1e80ec1987f25ca90ef4a6de8121a0f6b8a6`.
The signed archive, dependency receipt, IPA, and private SHA receipt are under
`build/releases/32/` and remain untracked. The already-hosted 1.096 GB Asset
Pack v2 was not re-uploaded and remains outside the IPA.

The workstation's stable ASC key references now live in macOS Keychain; the
private `.p8` remains in its owner-only standard directory. `--upload` keeps
its explicit transaction boundary, but no longer requires transient shell
variables: after all local gates pass, the wrapper uses explicit environment
values when supplied and otherwise reads the Key ID and Issuer from Keychain.
See [`docs/unattended-testflight.md`](./docs/unattended-testflight.md). The
guarded wrapper regression suite is **16/16**.

### Build 31 upload receipt (historical, superseded on devices)

BioMotion 1.0.0 build 31 was validated by Apple with no errors, then the exact
same byte-pinned IPA was uploaded with no errors. The upload delivery UUID is
`933fa8fb-7f34-4b80-994e-7fae069b490c`. The IPA is 1,541,663 bytes at SHA-256
`3b86fedafefcca46e946d10237a8d7f30bab1dfb81832047be94d24dbf56b850`.
The signed archive and its dependency receipt are retained under
`build/releases/31/`; generated release artifacts and signing material remain
untracked.

The 1.096 GB model is not an On-Demand Resources tag and is not in that IPA.
It is the separately uploaded Apple-hosted Managed Background Asset Pack
`sam3d-body-pose` version 2, whose App Store Connect state reached
`READY_FOR_TESTING` before the app upload. A physical-device TestFlight smoke
must still prove download progress, relaunch recovery, complete-package lookup,
`MLModel(contentsOf:)`, and one real inference.

The Release configuration uses manual App Store signing. At the 2026-08-12
workstation review, a valid Apple Distribution identity for team `N7VVB6PWZS`
and the two App Store profiles named `BioMotion AppStore AG` and
`BioMotion Ext AppStore AG` are installed; both profiles authorize
`group.com.soleilyu.biomotion` and expire on 2027-08-06. That inventory is not a
signed-archive receipt. The archive gate still rejects the wrong team, bundle
id, certificate, entitlement set, development/ad-hoc signing, or a profile/
certificate with less than 30 days of remaining validity.

```bash
# Build 32 is already used by the 2026-08-12 TestFlight upload. For the next
# upload, choose a number greater than every ASC build, then set BOTH target-level
# CURRENT_PROJECT_VERSION entries in project.yml to that same number.
# Regenerate only after the bump; the source gate rejects a stale pbxproj.
xcodegen generate

# The controlled entry point compares the complete observed dependency
# snapshot before and after xcodebuild, checks the signed archive, and writes
# the required adjacent receipt. It also gives this archive a fresh private
# DerivedData. Neither output path may already exist.
/usr/bin/install -d -m 0700 build
/bin/bash -p tools/release/archive_release.sh \
  --archive build/BioMotion.xcarchive

# Default mode makes no explicit App Store Connect validate/upload request:
# reverify the dependency receipt plus current dependency,
# archive/resource/privacy gates, tracked export options, local re-sign/export,
# and a final IPA-vs-archive gate. The export directory must be new or empty.
/bin/bash -p tools/release/testflight_release.sh \
  --archive build/BioMotion.xcarchive \
  --export-dir build/testflight-CONFIRMED_BUILD_NUMBER
```

The tracked `tools/release/ExportOptions-TestFlight.plist` uses manual signing,
the two named profiles, the generic `Apple Distribution` certificate selector,
and `destination=export`; export never requests an upload implicitly. The real
`xcodebuild -exportArchive` process is not network-sandboxed, so default mode is
not a claim of universal network isolation. To authorize an App Store Connect
validation without upload, add `--validate`. To authorize the complete
validate-then-upload transaction for the same private byte-pinned IPA, add
`--upload`. `ASC_API_KEY_ID` and `ASC_API_ISSUER` remain optional per-run
overrides; when absent, the wrapper reads the workstation's stable references
from macOS Keychain after every local gate has passed. The wrapper passes
the current user's exact
`~/.appstoreconnect/private_keys/AuthKey_<KEY_ID>.p8` path to `altool`; both key
directories must be physical, current-user-owned and owner-only (0700 is the
normal mode), and the key must be a current-user mode-0600 regular file. Do not invoke
raw `xcodebuild archive`, `xcodebuild destination=upload`, or `altool` around
the wrappers. `BioMotion.xcarchive.dependency-receipt.json` is part of the
archive handoff; copying or renaming an archive without its adjacent receipt is
fail-closed.

At the 2026-08-12 build-32 release, both ASC key directories were owner-only,
the `.p8` was mode 0600, the Key ID and Issuer were verified against the
BioMotion App Store Connect record, and their stable references were saved in
Keychain. A missing/revoked key, invalid reference, unsafe permission, or
reused build number still fails closed.

Before review submission, the owner must also complete and verify Apple's
current [required App Store Connect properties](https://developer.apple.com/help/app-store-connect/reference/app-information/required-localizable-and-editable-properties):
the privacy-policy/support URLs and App Privacy answers, age rating and content
rights, screenshots and localized product copy, App Review contact/notes, and
any prompted export-compliance or regulated-medical-device declarations. No
completed App Store listing or review-metadata receipt is claimed here.

The observed snapshot contains the actual checkout identities, both SDKs'
artifacts, generated headers, CMake settings and exact project linkage; it is
not merely a copy of the lock file. For Nimble, OSQP and both SDK-local QDLDL
repositories, inspection resolves the physical Git root, requires a local real
`.git` directory, rejects replace refs and index assume-unchanged/skip-worktree
flags, and hashes every tracked working-tree blob against `HEAD`. A physical
walk also rejects ignored or untracked material outside the reviewed build
roots. Its standard output is one canonical JSON payload followed by a newline,
so consumers must parse that record instead of depending on a historical byte
count.

Project validation pins Xcode 26.4 build `17E192` and iPhoneOS SDK 26.4 build
`23E237`, then checks the PBXProject and exactly three native targets
(`BioMotion`, `BioMotionTests`, `AssetPackDownloader`). It owns every
configuration list, Debug/Release setting surface, target dependency, ordered
phase and referenced build file. Extra targets, frameworks/packages, per-file
flags, base xcconfigs, tool overrides and unattached graph objects fail closed.
The `.xcodeproj` container may contain only `project.pbxproj` and the canonical
self workspace—shared/user schemes, `xcuserdata` and sidecars are rejected—and
the two developer-model phases must contain the SHA-256-pinned guard script
(`a83bd4b5fbafb6442358ce6dd06627c574514a97c2fa7c28bf8750c8a29223d6`).
The active SDK's direct `.a` paths are derived from the lock; tracked source,
Eigen/tinyxml2 and OSQP headers precede the generated roots, whose inventory is
limited to the locked configuration headers. Name-based Nimble/OSQP `-l`
lookup is forbidden, so a same-name dylib cannot redirect the link, and an
unattached decoy build configuration cannot satisfy the gate.

The test runner and archive wrapper each pass a fresh private DerivedData path
to Xcode, so a reviewed run does not reuse an ambient shared build cache. These
archive-local dependency, source, resource, privacy and receipt gates run with
`HOME=/var/empty`; only the real `xcodebuild` receives the current account's
passwd-derived HOME. After the post-build snapshot matches, the wrapper passes
the exact initial snapshot to the receipt sealer explicitly on standard input,
so sealing cannot silently substitute a fresh ambient observation. The
dependency-receipt suite is green **38/38**, the archive-wrapper suite
**14/14**, and the TestFlight-wrapper suite **16/16**. These
controls assume a trusted, quiescent same-user build machine. They make ordinary
dependency drift and inherited shell/Xcode/Python environment pollution fail
closed, but they are not a defence against malicious code already executing as
the same macOS user. The receipt proves that every tracked blob in all four
nested checkouts matched its recorded `HEAD` at both observations and binds the
separately inspected build artifacts to the resulting archive bytes. It is not
a signature or a link-map/dependency-closure proof that every inspected source,
header or static-archive member contributed to the executable.

Do not export, validate, or upload unless the resource gate prints
`APP_RESOURCE_BOUNDARY_PROBE_PASS source-project` and
`APP_RESOURCE_BOUNDARY_PROBE_PASS release-archive`, then the exported-package gate
prints `APP_RESOURCE_BOUNDARY_PROBE_PASS release-ipa` with its SHA-256, the privacy
gate prints `PRIVACY_MANIFEST_PROBE_PASS`, and every documented legal/product release
blocker is closed. The resource gate requires a signed arm64 device `.xcarchive`; it
pins the complete app/extension resource inventory, model and notice identities, asset
catalog, bundle metadata, platform, team, architecture, CMS-authenticated provisioning
profiles, and signatures. The final IPA gate independently verifies the raw ZIP headers,
compressed streams, CRCs, real expansion sizes, and executable permissions before it
extracts the locally re-signed package; it then repeats those checks and permits only
signature/profile bytes to differ from the reviewed archive. Simulator apps and test
bundles have separate smoke modes and are not accepted as release evidence; see
[`docs/app-resource-boundary.md`](./docs/app-resource-boundary.md).
The resource boundary's source/binary adversarial harness is green **45/45**;
its negatives include weaker Debug guards and an injected internal literal in
an otherwise valid Release executable.

## Architecture & gotchas

See [`CLAUDE.md`](./CLAUDE.md) for the runtime pipeline, Swift ↔ C++ bridge layout, key files, and build gotchas (Eigen versions, marker name mapping, C++ vs ObjC exceptions, etc.).
