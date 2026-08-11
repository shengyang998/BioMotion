# Nimble patches

This folder holds reviewed local patches to `nimblephysics/` that BioMotion
needs but has not upstreamed. `nimblephysics/` itself is a gitignored vendored
clone, so every behaviour change made for BioMotion needs a patch here.

The folder is authoritative for the patches listed below, not yet for the
entire vendored tree. The current iOS port also contains older build and header
changes that predate this record; a fresh clone therefore still needs the
repository's iOS-port setup in addition to these patches. Do not describe a
fresh Nimble checkout as reproducible until those remaining changes have also
been exported and checked.

## `opensimparser-fail-closed.patch`

**Applies to**: `nimblephysics/dart/biomechanics/OpenSimParser.cpp`

**Baseline**: Nimble
`c405b056fc35068027e03e0c384e84e12870b475`

**Reviewed branch commits** on
[`biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05):

- `7ecf61c` — fail closed on unsupported or incomplete joint topology
- `6b082fd` — preserve CustomJoint functions and coordinate mappings

**Patch receipt**: 562 lines; SHA-256
`50701bb5ae848f9192c1c0e5ffcfdef4a94314f98a95c00e6f7390b751482b3b`.

### Problem

`dart::biomechanics::createJoint()` used release-disabled `assert()` calls for
unsupported `CustomJoint` functions, non-orthogonal rotation axes and invalid
coordinate counts. Missing `SpatialTransform`, parent and axis elements could
also reach unchecked dereferences or invalid topology.

In **Release**, these checks did not have one uniform failure mode. A path that
never constructed a joint could reach `joint->setName(jointName)` with
`joint == nullptr`; the first local workaround converted only that null-joint
case into a silent `WeldJoint`. The unsupported-function RED followed a
different path: the Rajagopal fixture still selected a specialized joint and
returned success. Its total remained 37 DOFs, but the bridge accepted the load
as a replacement, cleared the installed one-DOF mask and changed the free count
from 36 to 37.

The successful paths also changed valid model semantics. Specialized DART
joints were selected without checking non-identity slope/intercept, translation
axis sign, or the `TransformAxis` coordinate mapping. That could turn `2*q`
into `q`, bind a transform to a different coordinate, or drop all three root
translations when a six-axis joint fell through to the looser Euler branch.

### Fix

The patch replaces the release-disabled failure paths with descriptive runtime
errors; validates parent/root topology, exactly six transform axes, coordinate
references, function structure, and Cartesian translation-axis allocation;
and requires both `joint` and `childBody` before use. Root Weld joints are
constructed through the skeleton root API. `NimbleBridge.loadModel` catches a
rejection and keeps the previously loaded skeleton transactionally. No
substitute joint and no partial skeleton may cross the bridge.

Specialized EulerFree/Euler/Revolute/Prismatic/Universal joints are now used
only for the exact canonical function and coordinate layout they can represent.
Everything else remains a `CustomJoint`, including non-identity linear
functions and permuted valid coordinate mappings. Baking a `-1` rotation slope
into the axis also negates its intercept, preserving
`a*(-q+b) == (-a)*(q-b)`.

Regression coverage links the rebuilt simulator archive rather than merely
inspecting source:

- `DOFMaskTests.testFailedReloadPreservesActiveDOFMask` rejects unsupported
  functions, non-Cartesian/negative translation axes, incomplete transforms,
  unknown parents and unknown coordinates while preserving the old DOF count
  and active mask.
- `IKSolverInternalsTests.testP8ModelCoordinateRepresentation` proves `2*q`
  produces `0.2 m` at `q=0.1`, pins the `-q+0.5` intercept sign, and verifies a
  valid remapping makes `pelvis_ty` drive both X and Y. The focused receipt also
  recorded the reflected fixture's root as `CustomJoint<6>` and the unmodified
  Rajagopal fixture at 37 total DOFs; those are fixture observations, not
  general parser guarantees.
- Both bundled models still load through their reviewed paths. The final
  focused receipt was **3/3**, with no failures, skips, expected failures or
  test-host restarts. Simulator Release, device Release and simulator
  non-`NDEBUG` syntax builds all passed with zero warnings/errors; an
  independent staged-diff review found no blocker/high issue.
- The enclosing product commit gate passed fast **519/519 in 1690 s** and slow
  E1 **1/1 in 6170 s**, with zero failures, skips, expected failures or
  test-host restarts; the runner ended with `ALL GATE PASS`.

### How to apply after a fresh `nimblephysics/` clone

```sh
cd labs/BioMotion/nimblephysics
git checkout --detach c405b056fc35068027e03e0c384e84e12870b475
git apply ../nimble-patches/opensimparser-fail-closed.patch
cmake --build build_ios --target nimble_ios --parallel
cmake --build build_sim --target nimble_ios --parallel
```

Then run the product regressions:

```sh
cd ..
tools/run_tests.sh subset \
  -only-testing:BioMotionTests/DOFMaskTests/testFailedReloadPreservesActiveDOFMask \
  -only-testing:BioMotionTests/IKSolverInternalsTests/testP8ModelCoordinateRepresentation \
  -only-testing:BioMotionTests/NimbleBridgeTests/testBundledModelsFailClosedWithoutValidatedFootContactSupport
```

On an already patched tree, verify provenance with:

```sh
git -C nimblephysics apply --reverse --check \
  ../nimble-patches/opensimparser-fail-closed.patch
```

## `ios-collision-fail-closed.patch`

**Applies to**:

- `nimblephysics/dart/collision/CollisionDetector.cpp`
- `nimblephysics/dart/collision/dart/DARTCollisionDetector_ios.cpp`
- `nimblephysics/dart/simulation/World.cpp`

**Baseline**: Nimble
`c405b056fc35068027e03e0c384e84e12870b475`

**Reviewed branch commit** on
[`biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05):
`23e359d516e3d6da38cda0207ab057c37c9c7779` — fail closed when the iOS
collision backend is absent.

**Patch receipt**: 210 lines; SHA-256
`d8c50a8f58e4e4a79c43d4e1be173e9d4f5d539d3be44b717c561ac97b304399`.

### Problem

The old local iOS stub declared a shortened, incompatible version of
`DARTCollisionDetector` and returned `nullptr` from `create()`. Its object file
also had no ordinary static-archive reference from `CollisionDetector.cpp`, so
the linker did not extract it and the factory did not expose the `"dart"` key.
Direct consumers then either received null or dereferenced it while constructing
`ConstraintSolver`, `BoxedLcpConstraintSolver`, or `World`.

`World` allocated its raw `Recording` before constructing that solver. Once the
collision rejection became an exception, the old order would leak the recording
because a partially constructed `World` does not run `World::~World()`.

### Fix and verification

The iOS implementation now uses the real public class declaration and defines
its complete virtual surface. `getType()` and `getStaticType()` retain the
stable `"dart"` identifier; construction and every operation that would require
Assimp/libccd throw one exact descriptive `std::runtime_error`.
`CollisionDetector::getFactory()` performs thread-safe, one-time registration
and directly references the unavailable implementation, which makes an
ordinary static link extract it. `World` allocates its `Recording` only after
the solver has been constructed successfully.

Regression evidence is intentionally layered:

- The causal RED was **0/5**: direct creation returned null, the factory lacked
  `"dart"`, and all three consumer tests stopped at their safe preflight. The
  pre-fix archive probe also proved that the linker did not extract
  `DARTCollisionDetector_ios.cpp.o`.
- Both arm64 simulator and device archives rebuilt. The focused linked-archive
  receipt then passed **5/5 in 7 s**, with no failures, skips, expected failures,
  or test-host restarts.
- `collision_static_link_probe.sh` links a factory-only consumer with
  dead-stripping, without `-all_load` or `-force_load`; its link map and
  `-why_load` receipt require both archive members and its simulator run ends in
  `ARCHIVE_FACTORY_PROBE_PASS`.
- `collision_world_leak_probe.sh` performs a fresh current-port host-native
  build. Its separate positive-control process reported one deliberate
  `leakForPositiveControl()` allocation (160 bytes after allocator rounding),
  while 32 `World()` and 32 `World::create()` rejection attempts reported
  **0 leaks / 0 bytes** and ended in
  `WORLD_COLLISION_REJECTION_LEAK_PROBE_PASS`. This proves the exception-order
  fix with the macOS allocator; it is not an iOS allocator measurement and it
  currently depends on the still-unexported iOS CMake source manifest.
- The shell gate policy self-tests pass **49/49**. The enclosing full gate
  passed the fast lane **524/524 in 1685s** and the slow lane **1/1 in
  6172s**. Both `xcodebuild` and `xcresulttool` returned 0, both result bundles
  reported `Passed`, and the receipts contained zero failures, skips, expected
  failures, or runner restarts before `ALL GATE PASS`.

This is an explicit refusal boundary. It does not add collision/contact
simulation and does not reopen any dynamics product claim.

### Apply and verify

The patch can be applied independently at the pinned source baseline, but its
new iOS source file still has to be selected by the current iOS-port CMake
manifest. That older manifest is not yet represented by this patch, so the
commands below describe the maintained current port, not a complete fresh-clone
bootstrap:

```sh
cd labs/BioMotion/nimblephysics
git checkout --detach c405b056fc35068027e03e0c384e84e12870b475
git apply ../nimble-patches/ios-collision-fail-closed.patch
# Install/apply the current iOS source manifest before these two builds.
cmake --build build_ios --target nimble_ios --parallel
cmake --build build_sim --target nimble_ios --parallel

cd ..
tools/run_tests.sh subset \
  -only-testing:BioMotionTests/CollisionFailClosedTests
bash tools/tests/collision_static_link_probe.sh
bash tools/tests/collision_world_leak_probe.sh
```

On the reviewed branch head, provenance must reverse-check:

```sh
git -C nimblephysics apply --reverse --check \
  ../nimble-patches/ios-collision-fail-closed.patch
```

## `ios-c3d-boundary.patch`

**Applies to**:

- `nimblephysics/dart/biomechanics/C3DLoader.hpp`
- `nimblephysics/dart/biomechanics/C3DLoader.cpp`
- `nimblephysics/dart/biomechanics/C3DForcePlatforms.hpp`
- `nimblephysics/dart/biomechanics/C3DForcePlatforms.cpp`

**Baseline**: Nimble
`c405b056fc35068027e03e0c384e84e12870b475`

**Reviewed branch commit** on
[`biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05):
`03fa30ca524376747f7e0e884c8c8c14c4d5526f` — hide C3D loading APIs whose
ezc3d implementation is not linked into the iOS archive.

**Patch receipt**: 116 lines; SHA-256
`63b5bc8ad9206738eedabf89100a1fee84ce856f3cba6895dc63bb5fc50ea6a7`.

### Problem

The iOS source manifest deliberately omits `C3DLoader.cpp` and
`C3DForcePlatforms.cpp`, because ezc3d is not present. The public headers did
not match that binary boundary: `C3DLoader`, its weighted force-convention
heuristic, and all ezc3d force-platform adapters remained declared. Some iOS
callers therefore compiled and failed only at link time, while directly
including `C3DForcePlatforms.hpp` failed immediately on a missing ezc3d header.

### Fix and verification

The pure `C3D` value struct and its `ForcePlate` vector remain public. The
OpenSim consumer
`OpenSimParser::loadMotAtLowestMarkerRMSERotation(..., C3D&, ...)` also remains
defined in both archives; it uses only `dataRotation` and `markerTimesteps` and
does not call ezc3d. iOS now hides the loader-only weighted method, the entire
`C3DLoader` class, and the `ForcePlatform` / `ForcePlatforms` adapter surface.
Both implementation files always include their own header first, then compile
their ezc3d dependencies and bodies only outside `DART_IOS_BUILD`.

Regression evidence is layered:

- The causal surface probe was RED with **8 contract failures**: both production
  translation units and the adapter header leaked missing ezc3d dependencies;
  `C3DLoader` and the weighted method unexpectedly compiled; and the three
  adapter negatives failed for the missing include instead of the reviewed
  absent-API diagnostic.
- Both arm64 simulator and device archives rebuilt. Two focused XCTest receipts
  each passed **1/1** in 12 s and 7 s, with zero failures, skips, expected
  failures, or runner restarts. They pin the value surface and the retained
  OpenSim consumer independently.
- `c3d_ios_boundary_probe.sh` positive-compiles the supported headers and both
  production sources, negative-compiles all five unavailable surfaces with
  same-line diagnostic classes. It checks every archive object's
  `LC_BUILD_VERSION` (simulator platform 7, device platform 2), requires exactly
  one OpenSimParser/ForcePlate member and definition in each archive, and scans
  both defined and undefined symbols for the complete forbidden surface. Its
  ordinary simulator and device links use dead-stripping without `-all_load` or
  `-force_load`, require `OpenSimParser.cpp.o` and `ForcePlate.cpp.o`, and reject
  ezc3d/loader extraction or unresolved symbols. The simulator binary runs to
  `C3D_IOS_ARCHIVE_PROBE_PASS` / `C3D_IOS_BOUNDARY_PROBE_PASS`.
- Independent review found no blocker, high, medium, or low issue in the nested
  four-file diff.
- The enclosing `tools/run_tests.sh all` gate passed: the fast lane completed
  **526/526** tests in 1696 s and the slow lane completed **1/1** in 6274 s.
  Both `xcodebuild` and `xcresulttool` returned 0 for both lanes; each xcresult
  was `Passed`, with zero failures, skips, expected failures, or test-host
  restarts. The runner emitted `ALL GATE PASS`.

Every iOS consumer must define `DART_IOS_BUILD=1`; the current CMake target,
app, tests, and probes do. This patch aligns the header contract with an absent
backend. It does not add C3D file loading.

### Apply and verify

```sh
cd labs/BioMotion/nimblephysics
git checkout --detach c405b056fc35068027e03e0c384e84e12870b475
git apply ../nimble-patches/ios-c3d-boundary.patch
# Install/apply the current iOS source manifest before these two builds.
cmake --build build_ios --target nimble_ios --parallel
cmake --build build_sim --target nimble_ios --parallel

cd ..
tools/run_tests.sh subset \
  -only-testing:BioMotionTests/C3DIOSBoundaryTests/testC3DAndForcePlateRemainUsableValueTypes
tools/run_tests.sh subset \
  -only-testing:BioMotionTests/C3DIOSBoundaryTests/testOpenSimC3DConsumerRemainsLinked
bash tools/tests/c3d_ios_boundary_probe.sh
```

On the reviewed branch head, provenance must reverse-check:

```sh
git -C nimblephysics apply --reverse --check \
  ../nimble-patches/ios-c3d-boundary.patch
```

## `xml-helpers-no-boost.patch`

**Applies to**: `nimblephysics/dart/utils/XmlHelpers.cpp`

**Baseline**: Nimble
`c405b056fc35068027e03e0c384e84e12870b475`

**Reviewed branch commit** on
[`biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05):
`78b292e19af13ad77501c9b22f49c1fa06146501` — remove the XML helper's Boost
string/lexical dependency while preserving the reviewed conversion contract.

**Patch receipt**: 492 lines; SHA-256
`86bf0987efa9961a06679115e926408fc451a5ca4ee165fee29c5b808ec58aa9`.

### Problem

`XmlHelpers.cpp` used header-only Boost string algorithms and
`boost::lexical_cast`. That forced both app and test targets to carry an
absolute `/opt/homebrew/Cellar/boost/1.90.0_1/include` search path even though
the final archive needed no Boost library. The path is host-specific and made
a clean machine depend on one exact Homebrew installation.

A direct replacement is behavior-sensitive. Existing callers observe exact
float/double formatting, a `std::bad_cast` base exception, unsigned `"-1"`
wrapping, literal-space vector tokenization, and Apple libc++ `ERANGE` behavior
for overflow, underflow, and subnormal parsing. Boost also follows the global
C++ locale, which is unsuitable for XML's locale-independent decimal dot.

### Fix and verification

The replacement uses standard streams imbued with `std::locale::classic()` and
a local exception derived from `std::bad_cast`. It preserves the characterized
scalar, vector, transform, tinyxml2, `errno`, NaN/Infinity, hexadecimal-float,
and formatting behavior. The intentional improvement is locale stability:
hostile global comma/grouping punctuation can no longer change XML output or
input.

Verification is split so the refactor cannot define its own expected behavior:

- `xml_helpers_characterization_probe.sh` first passed against the old Boost
  archive and pins the legacy contract independently.
- Both arm64 simulator and device archives were rebuilt after the source
  change. Fresh source compiles use `-Wall -Wextra -Werror` for both SDKs and
  emit no Boost symbol.
- `xml_helpers_refactor_probe.sh` requires exactly one `XmlHelpers.cpp.o` in
  each archive and rejects `boost::` in those exact members. Ordinary
  dead-stripped links extract the member without force-loading.
- The simulator hostile-locale run emitted
  `XML_HELPERS_CLASSIC_LOCALE_PASS`; the rebuilt archive then reran the complete
  characterization to `XML_HELPERS_CHARACTERIZATION_PASS` and
  `XML_HELPERS_ARCHIVE_PROBE_PASS`, before the final
  `XML_HELPERS_NO_BOOST_ARCHIVES_PASS` sentinel.
- App and test build settings no longer contain the absolute Homebrew Boost
  include path. Whole-app, no-signing builds passed for the dedicated arm64
  simulator and the generic arm64 device destination. This removes the XML
  consumer dependency; the still-dirty older iOS CMake port is tracked
  separately and is not claimed reproducible by this patch.

### Apply and verify

```sh
cd labs/BioMotion/nimblephysics
git checkout --detach c405b056fc35068027e03e0c384e84e12870b475
git apply ../nimble-patches/xml-helpers-no-boost.patch
# Install/apply the current iOS source manifest before these two builds.
cmake --build build_ios --target nimble_ios --parallel
cmake --build build_sim --target nimble_ios --parallel

cd ..
bash tools/tests/xml_helpers_refactor_probe.sh
```

On the reviewed branch head, provenance must reverse-check:

```sh
git -C nimblephysics apply --reverse --check \
  ../nimble-patches/xml-helpers-no-boost.patch
```

## `simmspline-linear-extrapolation.patch`

**Applies to**: `nimblephysics/dart/math/SimmSpline.cpp` and its upstream unit
test

**Baseline**: Nimble
`c405b056fc35068027e03e0c384e84e12870b475`

**Reference semantics**: OpenSim
[`3b2cb19f29e34fc33f85705223ccdb6dad348cd0`](https://github.com/opensim-org/opensim-core/blob/3b2cb19f29e34fc33f85705223ccdb6dad348cd0/OpenSim/Common/SimmSpline.cpp),
whose `SimmSpline` continues an out-of-domain value along the endpoint tangent,
keeps the first derivative at the endpoint slope, and returns zero for higher
derivatives.

### Problem

Nimble carried those two endpoint-extrapolation branches in comments, then
continued the first or last cubic outside the knot domain. FullBody exposes the
bug: `walker_knee_r` permits 0–140 degrees, while its five nonlinear transform
axes have knots only through 120 degrees. At the 130-degree
`run_4_mid_swing` pose, the wrap-free, fixed-point `bflh140_r` knee moment arm
was 16.059165 mm in BioMotion versus 13.713465 mm in OpenSim.

Restoring endpoint-linear extrapolation changes that result to 13.713465 mm.
The direct regression test also pins both sides of a three-knot spline: the old
implementation returned `(value, d1, d2) = (-3, +4, -2)` below the domain and
`(-3, -4, -2)` above it; OpenSim-compatible output is `(-2, +2, 0)` and
`(-2, -2, 0)`.

### Fix and regression coverage

The patch restores the existing out-of-range branches in `calcValue()` and
`calcDerivative()` and adds the same two-sided regression to Nimble's GTest.
BioMotion separately carries `SimmSplineExtrapolationTests.mm`, which links the
actual iOS/simulator archive and therefore also catches a patched source file
that was not rebuilt. `MomentArmTests` pins both in-domain cubic evaluation and
out-of-domain endpoint-tangent evaluation through the production
`MomentArmComputer`; FullBody's fidelity report is `Moving 4 parsed
(0 approximated, 0 skipped)`.

### Apply and rebuild

```sh
cd labs/BioMotion/nimblephysics
git apply ../nimble-patches/simmspline-linear-extrapolation.patch
cmake --build build_ios --target nimble_ios --parallel
cmake --build build_sim --target nimble_ios --parallel

cd ..
xcodegen generate
tools/run_tests.sh subset \
  -only-testing:BioMotionTests/SimmSplineExtrapolationTests
```

That selection is a patch-specific diagnostic, not the commit gate. Run
`tools/run_tests.sh all` before committing.

On an already patched tree, this must succeed instead of applying it again:

```sh
cd labs/BioMotion/nimblephysics
git apply --reverse --check \
  ../nimble-patches/simmspline-linear-extrapolation.patch
```
