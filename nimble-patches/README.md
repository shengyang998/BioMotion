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

## `opensimparser-null-joint-fallback.patch`

**Applies to**: `nimblephysics/dart/biomechanics/OpenSimParser.cpp`
**Introduced in**: build 13 (BioMotion accuracy-overhaul branch)

### Problem

`dart::biomechanics::createJoint()` has several paths where it uses
`assert(false && "...")` to signal that it couldn't construct a joint —
for example when a `CustomJoint`'s `dofNames.size()` doesn't match any
of the 1-6 `createCustomJoint<N>` specializations, or when a
`TransformAxis` uses a function type the parser doesn't recognize.

In **release** builds `assert()` is a no-op, so those paths fall
through with the local `joint` variable left as `nullptr`. Later, at
line 5779, `joint->setName(jointName)` dereferences a null `this` and
the app segfaults at `this + 0x3f` inside `Joint::setName`.

This was discovered when swapping the bundled osim to
`cyclistFullBodyMuscle.osim`, which has 53 CustomJoints — at least one
triggers the uncovered branch and takes the whole parse down.

### Fix

Before `joint->setName(jointName)`, check for null and substitute a
`WeldJoint` in place. Log the failing joint name + type to stderr so
we can later decide whether to (a) extend nimble's parser to handle
that joint type or (b) pre-process the XML.

```cpp
if (joint == nullptr) {
    std::cerr << "OpenSimParser: failed to construct joint \""
              << jointName << "\" of type \"" << jointType << "\" "
              << "— substituting a WeldJoint so parse can continue."
              << std::endl;
    dynamics::WeldJoint::Properties props;
    props.mName = jointName;
    // create through skel/parentBody as usual...
}
```

The affected joint becomes immobile in the kinematic chain, but the
skeleton structure stays valid and the rest of the model keeps working.

### How to apply after a fresh `nimblephysics/` clone

```sh
cd labs/BioMotion/nimblephysics
git apply ../nimble-patches/opensimparser-null-joint-fallback.patch
cd build_ios && cmake --build .   # rebuild static archive for device
cd ../build_sim && cmake --build .   # rebuild for simulator
```

After that the BioMotion Xcode build will link against the patched
`libnimble_ios.a` automatically because the project.yml points at
`nimblephysics/build_ios/libnimble_ios.a`.

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
tools/run_tests.sh \
  -only-testing:BioMotionTests/SimmSplineExtrapolationTests
```

On an already patched tree, this must succeed instead of applying it again:

```sh
cd labs/BioMotion/nimblephysics
git apply --reverse --check \
  ../nimble-patches/simmspline-linear-extrapolation.patch
```
