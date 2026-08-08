/* -------------------------------------------------------------------------- *
 *  MusclePathWrap.h — muscle path wrapping over cylinder and ellipsoid        *
 *                     geometry.                                              *
 * -------------------------------------------------------------------------- *
 *  Ported from OpenSim (opensim-core), which carries:                         *
 *                                                                            *
 *      OpenSim:  WrapCylinder.cpp / WrapEllipsoid.cpp / WrapObject.cpp /      *
 *                WrapMath.cpp / GeometryPath.cpp                             *
 *      Copyright (c) 2005-2017 Stanford University and the Authors            *
 *      Author(s): Peter Loan, Frank C. Anderson                              *
 *                                                                            *
 *      Licensed under the Apache License, Version 2.0 (the "License"); you    *
 *      may not use this file except in compliance with the License. You may   *
 *      obtain a copy of the License at                                        *
 *      http://www.apache.org/licenses/LICENSE-2.0.                            *
 *                                                                            *
 *      Unless required by applicable law or agreed to in writing, software    *
 *      distributed under the License is distributed on an "AS IS" BASIS,      *
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        *
 *      implied. See the License for the specific language governing           *
 *      permissions and limitations under the License.                         *
 *                                                                            *
 *  Modifications (c) 2026 Soleil, released under this project's licence, are  *
 *  listed in DEVIATIONS at the top of MusclePathWrap.cpp. See ./NOTICE.       *
 * -------------------------------------------------------------------------- */

#ifndef BIOMOTION_MUSCLE_PATH_WRAP_H
#define BIOMOTION_MUSCLE_PATH_WRAP_H

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace biomotion {

/// Which surface a `<WrapObject>` describes. `Cylinder` and `Ellipsoid` are
/// solved; every other kind is parsed, counted and left to the straight line, so
/// the fidelity report can keep saying which paths are still incomplete.
enum class WrapKind { Cylinder, Ellipsoid, Unsupported };

/// One `<WrapObject>` off a `<Body>`'s `<WrapObjectSet>`.
///
/// `pose` is OpenSim's `_pose`: the transform from the owning BODY's frame to
/// the WRAP OBJECT's frame, built from `<translation>` and `<xyz_body_rotation>`
/// exactly as `WrapObject::extendFinalizeFromProperties` builds it. The
/// cylinder's axis is the wrap frame's +z, centred on z = 0; the ellipsoid is
/// centred on the wrap frame's origin with `dimensions` as its semi-axes.
struct WrapObjectSpec {
    std::string name;
    std::string bodyName;
    WrapKind kind = WrapKind::Unsupported;
    bool active = true;

    double radius = 0.0;   // WrapCylinder
    double length = 0.0;   // WrapCylinder

    /// WrapEllipsoid `<dimensions>` — the three semi-axes, in metres. All three
    /// must be > 0 or OpenSim throws at load, and so this refuses to solve.
    Eigen::Vector3d dimensions = Eigen::Vector3d::Zero();

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    /// `<quadrant>` decoded. `wrapSign == 0` means "all" — the wrap is
    /// unconstrained and `wrapAxis` is unused. Otherwise the path may only wrap
    /// on the side of the cylinder where `point[wrapAxis]` has sign `wrapSign`.
    /// Getting this backwards produces a perfectly plausible-looking path on the
    /// wrong side of the bone, so it is decoded once, here, and never guessed.
    int wrapAxis = 0;
    int wrapSign = 1;
};

/// `<PathWrap><method>`. `WrapCylinder::wrapLine` never reads it;
/// `WrapEllipsoid::wrapLine` branches on it three ways and returns a materially
/// different path for each, so a `<method>` this port does not implement is
/// `Unsupported` and the wrap stays unmodelled rather than being solved as
/// though it said `hybrid`.
enum class PathWrapMethod { Hybrid, Unsupported };

/// One `<PathWrap>` off a muscle's `<PathWrapSet>`.
struct PathWrapSpec {
    /// Index into the model-wide wrap-object table. -1 = the reference could
    /// not be resolved, in which case this wrap does nothing.
    int wrapObject = -1;
    /// `<range>`, 1-based indices into the muscle's ORIGINAL `<PathPointSet>`
    /// (including points this parser dropped and points a `ConditionalPathPoint`
    /// has switched off). Anything < 1 means "the first"/"the last".
    int startPoint = -1;
    int endPoint = -1;
    /// `<method>`. Only consulted for an ellipsoid.
    PathWrapMethod method = PathWrapMethod::Hybrid;
};

/// `WrapObject::WrapAction`, verbatim.
enum class WrapAction { NoWrap = 0, InsideRadius, Wrapped, MandatoryWrap };

/// What `wrapCylinderLine` decided for one path segment. Points are in the wrap
/// object's own frame, like OpenSim's `WrapResult` before `wrapPathSegment`
/// converts them back.
struct WrapSegmentResult {
    WrapAction action = WrapAction::NoWrap;
    Eigen::Vector3d r1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d r2 = Eigen::Vector3d::Zero();
    double wrapPathLength = 0.0;
    /// The discrete branch choices inside the solve. They are not outputs
    /// anybody consumes — they are carried so the caller can tell one branch of
    /// the length function from another, which is what makes a finite
    /// difference across a branch switch detectable instead of silent.
    bool longWrap = false;     // cylinder: the long way round
    bool farSideWrap = false;  // both: the long way round
    /// Ellipsoid: `<quadrant>` put the contact point on the wrong side and it
    /// was mirrored. A different branch of L(q).
    bool quadrantFlipped = false;
    /// Ellipsoid: how many chords the surface distance was summed over.
    /// `(int)(|r1-r2| / 1 mm)`, so it steps by one as the path grows — a
    /// genuine, if small, discontinuity in L, and therefore part of the branch.
    int pathSegments = 0;
    /// The solve hit a case OpenSim answers with a NaN or a division by zero
    /// (tangent segment, negative discriminant on the surface ray). `action` is
    /// `NoWrap`: a refusal the caller can COUNT rather than a NaN in a length.
    bool numericalRefusal = false;
};

/// Calculate the wrapping of one line segment over a cylinder.
///
/// Ported from `WrapCylinder::wrapLine` + `_make_spiral_path` +
/// `_adjust_tangent_point`. `point1` and `point2` are in the CYLINDER's frame.
///
/// `singleWrap` reproduces `WrapResult::singleWrap`: OpenSim only runs the
/// axial tangency iteration (`MAX_ITERATIONS` 100, 0.1 deg) when a muscle
/// carries more than one `PathWrap`. With exactly one, the spiral length is
/// closed-form and no iteration happens at all — see DEVIATIONS note 1.
WrapSegmentResult wrapCylinderLine(const WrapObjectSpec& cylinder,
                                   const Eigen::Vector3d& point1,
                                   const Eigen::Vector3d& point2,
                                   bool singleWrap);

/// Calculate the wrapping of one line segment over an ellipsoid, `hybrid`
/// method only.
///
/// Ported from `WrapEllipsoid::wrapLine` + `calcTangentPoint` +
/// `CalcDistanceOnEllipsoid` + `findClosestPoint` + `closestPointToEllipse`.
/// `point1` and `point2` are in the ELLIPSOID's frame.
///
/// There is no `singleWrap` parameter and no previous-wrap parameter. OpenSim
/// seeds `r1`/`r2`/`c1`/`sv` from the PREVIOUS call's result, but on the
/// `hybrid` branch every one of those four is overwritten before it is read
/// (`r1`,`r2` by the line/ellipsoid intersection and then by `c1`; `c1`,`sv` by
/// Frans, by the fan, or by the blend of the two) — so hybrid wrapping is a
/// pure function of the pose, which is what makes differentiating it legitimate.
/// `axial` and `midpoint` are NOT like that and are not implemented.
WrapSegmentResult wrapEllipsoidLine(const WrapObjectSpec& ellipsoid,
                                    const Eigen::Vector3d& point1,
                                    const Eigen::Vector3d& point2);

/// The point on the ellipsoid `(x/a)^2+(y/b)^2+(z/c)^2 = 1` closest to
/// `(u,v,w)`, and whether it converged. `WrapEllipsoid::findClosestPoint`
/// (Graphics Gems IV). `specialCaseAxis >= 0` forces the 2-D reduction on that
/// axis; -1 lets the routine detect it. Returns false when the Newton iteration
/// did not converge, in which case `closest` holds the last iterate rather than
/// the uninitialised memory OpenSim returns (DEVIATION 10).
///
/// ⚠ `radii` MUST BE NORMALISED — of order 1, i.e. scaled by OpenSim's
/// `factor = 3 / (a + b + c)`, which is why that factor exists. The Newton
/// iteration stops on `|f| < 1e-9` where `f` is a degree-6 polynomial in the
/// radii, so at metre scale (`a·b·c ≈ 2e-5`) `f ≈ 1e-19` at the very first
/// iterate and the routine returns the QUERY POINT unchanged — a plausible
/// answer, off by centimetres. `wrapEllipsoidLine` always calls it normalised;
/// anything else calling it has to do the same.
bool closestPointOnEllipsoid(const Eigen::Vector3d& radii,
                             const Eigen::Vector3d& point,
                             Eigen::Vector3d& closest,
                             int specialCaseAxis);

/// The result of running a muscle's whole `<PathWrapSet>` over its polyline.
struct WrappedPathResult {
    /// Total musculotendon path length, in metres, with the wraps applied.
    double length = 0.0;

    /// How many wrap points were inserted (2 per engaged wrap object). Directly
    /// comparable with the reference fixture's `wrapPoints` column.
    int wrapPointCount = 0;

    /// A hash of every DISCRETE choice the solve made: which wrap objects
    /// engaged, on which segment, and which branch. Two poses with the same
    /// signature lie on the same smooth piece of L(q); two with different
    /// signatures have a switch between them, and a centred finite difference
    /// across it is a fabricated moment arm rather than a derivative.
    std::uint64_t signature = 0;

    /// Set when the path exceeded `kMaxWrappedPathNodes` or
    /// `kMaxPathWrapsPerMuscle`. `length` is then the STRAIGHT polyline: a
    /// refusal the caller can count, not a truncated wrap presented as a wrap.
    bool refused = false;

    /// How many `wrapEllipsoidLine` calls returned `numericalRefusal`. Those
    /// segments took the straight line. Non-zero means a pose reached a case
    /// OpenSim answers with a NaN, and it should be reported, not averaged.
    int numericalRefusals = 0;
};

/// Caps on the fixed stack storage the solver uses. FullBody.osim's largest
/// muscle path is 11 points with 3 `PathWrap`s; these are ~3x that.
constexpr int kMaxWrappedPathNodes = 32;
constexpr int kMaxPathWrapsPerMuscle = 8;

/// Sum a muscle's path length, applying its wrap objects.
///
/// Ported from `GeometryPath::applyWrapObjects` + `calcLengthAfterPathComputation`.
///
/// Raw pointers rather than vectors because this runs ~176,000 times per
/// moment-arm solve (520 muscles x 2 x 169 coordinates) and must not allocate.
///
/// @param activeWorldPoints   world positions of the muscle's ACTIVE path
///                            points, in path order.
/// @param activeOriginalIndex parallel to `activeWorldPoints`: each point's
///                            index in the muscle's original `<PathPointSet>`,
///                            which is what `<range>` counts.
/// @param activePointCount    length of both of those arrays.
/// @param originalPointCount  size of the original `<PathPointSet>`.
/// @param pathWraps           the muscle's `<PathWrapSet>`, in file order.
/// @param pathWrapCount       length of that array.
/// @param wrapObjects         the model-wide wrap-object table.
/// @param wrapBodyTransforms  parallel to `wrapObjects`: the world transform of
///                            each wrap object's owning body at this pose.
/// @param wrapObjectCount     length of both of those arrays.
///
/// With no path wraps this is exactly the straight polyline sum, so it is safe
/// to call for every muscle in the model.
WrappedPathResult solveWrappedPathLength(
    const Eigen::Vector3d* activeWorldPoints,
    const int* activeOriginalIndex,
    int activePointCount,
    int originalPointCount,
    const PathWrapSpec* pathWraps,
    int pathWrapCount,
    const WrapObjectSpec* wrapObjects,
    const Eigen::Isometry3d* wrapBodyTransforms,
    int wrapObjectCount);

/// `<quadrant>` -> (`wrapAxis`, `wrapSign`), following
/// `WrapObject::extendFinalizeFromProperties`. Returns false for a spelling
/// OpenSim would have thrown on, so the caller can refuse the wrap object
/// instead of silently wrapping on whichever side `all` happens to pick.
bool decodeWrapQuadrant(const std::string& quadrant, int& wrapAxis, int& wrapSign);

/// The rotation `SimTK::Rotation::setRotationToBodyFixedXYZ` builds from
/// `<xyz_body_rotation>`: three body-fixed rotations, X then Y then Z.
Eigen::Matrix3d bodyFixedXYZRotation(const Eigen::Vector3d& angles);

}  // namespace biomotion

#endif  // BIOMOTION_MUSCLE_PATH_WRAP_H
