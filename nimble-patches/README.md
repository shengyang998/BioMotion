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
