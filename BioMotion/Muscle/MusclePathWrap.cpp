/* -------------------------------------------------------------------------- *
 *  MusclePathWrap.cpp — muscle path wrapping over cylinder and ellipsoid      *
 *                       geometry.                                            *
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
 *  Modifications (c) 2026 Soleil, released under this project's licence.      *
 *  See ./NOTICE.                                                             *
 * -------------------------------------------------------------------------- *
 *
 *  # What this is
 *
 *  A line-for-line port of the length half of OpenSim's cylinder and ellipsoid
 *  wrapping.
 *  SimTK::Vec3 becomes Eigen::Vector3d, `~a*b` becomes `a.dot(b)`, `a % b`
 *  becomes `a.cross(b)`; the control flow, the branch order, the magic
 *  constants and the tolerances are OpenSim's. It was ported rather than
 *  derived because the branch selection (which of four tangent-point
 *  candidates, near or far side, short or long way round) is where a
 *  re-derivation goes quietly wrong, and "quietly wrong" is the failure mode
 *  this project keeps paying for.
 *
 *  # DEVIATIONS from opensim-core — every one of them, with its reason
 *
 *  1. NO WRAP POINTS ARE GENERATED. `_make_spiral_path` samples the spiral
 *     every 2 mm to build `wrap_pts` for rendering and for line-of-action
 *     direction. Moment arms here are r = -dL/dq, so only the scalar
 *     `wrap_path_length` is consumed, and that is computed BEFORE the sampling
 *     loop in OpenSim and is unchanged by it. The loop is kept only where it
 *     feeds back into the answer: the `i == 1` tangency adjustment, which
 *     OpenSim runs only when `!singleWrap`. `numWrapSegments` is still computed
 *     exactly as OpenSim computes it, because the adjustment is evaluated at
 *     `t = 1/numWrapSegments` and skipped entirely when `numWrapSegments < 2`.
 *
 *  2. `acos` ARGUMENTS ARE CLAMPED to [-1, 1]. OpenSim calls
 *     `acos(dot(a,b)/(|a||b|))` on vectors it has just normalised; rounding can
 *     put that a few ulp outside the domain and return NaN, which then
 *     propagates into a length. The clamp changes no in-domain result.
 *
 *  3. `_adjust_tangent_point` REFUSES TO ADJUST when its two lines are
 *     parallel. OpenSim calls `WrapMath::IntersectLines` and uses `p1aw1a_int`
 *     unconditionally; on the parallel branch that function assigns neither
 *     intersection point, so OpenSim reads an uninitialised stack variable.
 *     Here the adjustment is skipped, which is what "no intersection to slide
 *     to" means.
 *
 *  4. NO ELLIPSOID `wrap_pts` LEAVE THE SOLVER EITHER, but unlike the cylinder
 *     the ellipsoid cannot avoid GENERATING them: `CalcDistanceOnEllipsoid`
 *     computes the surface distance BY summing the chords between them, and the
 *     wrong-way-wrap test reads `wrap_pts[1]` and `wrap_pts[size-2]`. They are
 *     streamed instead of stored — a running sum plus the two the test reads —
 *     so the 12 KB `s[500][3]` stack array is gone and the arithmetic is
 *     unchanged.
 *
 *  5. THE SIGNATURE. `WrappedPathResult::signature` has no counterpart in
 *     OpenSim. It exists because OpenSim never differentiates its own path
 *     length by finite differences, and this code does.
 *
 *  6. THE PARALLEL-LINES BRANCH IS DEFINED. When the muscle segment is parallel
 *     to the cylinder axis, `WrapMath::IntersectLines` writes neither output
 *     and OpenSim then reads `near12` — a `SimTK::Vec3` that is NaN-filled in a
 *     debug build and uninitialised in a release one. Here `near12`/`near00`
 *     are zeroed and `t12` is NaN, which makes the "line crosses the axis" test
 *     false and `DSIGN(near12[axis])` a deterministic +1.
 *
 *  7. NO HEAP. Every path this runs on has at most 11 path points and 3
 *     `PathWrap`s, so the working path lives in a fixed stack array. A muscle
 *     that exceeded the cap would be REFUSED (straight line, `refused` set),
 *     never silently truncated.
 *
 *  8. ONLY THE `hybrid` ELLIPSOID METHOD IS IMPLEMENTED. `WrapEllipsoid`
 *     branches three ways on `<PathWrap><method>` and the three produce
 *     materially different paths. All 12 ellipsoid `PathWrap`s in FullBody.osim
 *     say `hybrid`. `axial` and `midpoint` are refused at parse time and counted
 *     as unmodelled, so the tested surface and the shipped surface are the same
 *     set. Also: `hybrid` is the only one of the three that is a PURE FUNCTION
 *     of the pose. OpenSim seeds `r1`/`r2`/`c1`/`sv` from the PREVIOUS call's
 *     `WrapResult`, and on the `axial` branch `use_c1_to_find_tangent_pts` can
 *     be false, which leaves the previous call's `r1`/`r2` as the seed for
 *     `calcTangentPoint`. Differentiating a function of call history is not
 *     differentiating a function of q. Hybrid overwrites all four.
 *
 *  9. THE ELLIPSOID REFUSES INSTEAD OF RETURNING A NaN. Two places have no
 *     guard upstream: `t = (m - r1)/(r2 - r1)` divides by zero when the segment
 *     is exactly tangent (`disc == 0`, so `r1 == r2`), and the surface ray in
 *     `CalcDistanceOnEllipsoid` takes `sqrt(bb*bb - 4*aa*cc)` with no check that
 *     it is non-negative. Both return `NoWrap` with `numericalRefusal` set, and
 *     `WrappedPathResult::numericalRefusals` counts them, because a NaN length
 *     becomes a NaN moment arm becomes a QP that fails in a different file.
 *
 * 10. `findClosestPoint` RETURNS ITS LAST ITERATE. OpenSim's general (non
 *     axis-aligned) branch writes `*x`,`*y`,`*z` only on the iteration where
 *     `|f| < 1e-9`; if 64 Newton steps do not get there it returns -1.0 having
 *     written nothing, and every caller uses the outputs regardless — reading
 *     uninitialised stack. Here the same expressions are evaluated at the final
 *     `t` (which is what OpenSim's own 2-D `closestPointToEllipse` does after
 *     its loop) and the caller is told convergence failed.
 *
 * 11. FRANS IS EVALUATED ONCE, NOT THREE TIMES. `wrapLine` computes `t[i]`,
 *     `t_sv[i]` and `t_c1[i]` for all three axes and then reads only
 *     `[bestMu]`; `bestMu` depends on `mu` alone, so choosing first and solving
 *     once is bit-identical and drops two point-to-ellipsoid solves per call.
 *     It also stops the routine evaluating a NaN: `t[i]` divides by `r1r2[i]`,
 *     which is zero on exactly the axes `bestMu` can never be.
 *
 * 12. `EQUAL_WITHIN_ERROR(fanWeight, -Infinity)` IS REPRODUCED, NOT REPAIRED.
 *     It expands to `fabs(-inf - -inf) <= 2e-13`, i.e. `NaN <= 2e-13`, i.e.
 *     FALSE — so the sentinel it is testing for never matches and the
 *     quadrant-flip bisection is skipped whenever the fan did not run. That is
 *     the reference's behaviour, and this port has to reproduce it to be
 *     comparable with the reference. Fixing it here would be a silent fork.
 */

#include "MusclePathWrap.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace biomotion {
namespace {

// OpenSim/Common/SimmMacros.h
constexpr double kRoundoffError = 0.0000000000002;
inline int dsign(double a) { return a >= 0.0 ? 1 : -1; }
inline bool equalWithinError(double a, double b) {
    return std::abs(a - b) <= kRoundoffError;
}

// WrapMath.cpp
constexpr double kLineEpsilon = 0.00001;
// WrapCylinder.cpp
constexpr int kMaxIterations = 100;
const double kTangencyThreshold = 0.1 * M_PI / 180.0;  // 0.1 degrees
// The cylinder axis in the wrap object's own frame: OpenSim's file-scope
// `p0` and `dn`. p0 is any point on the axis; the axis is +z through the
// origin and the cylinder is centred on z = 0.
const Eigen::Vector3d kAxisPoint(0.0, 0.0, -1.0);
const Eigen::Vector3d kAxisDirection(0.0, 0.0, 1.0);

/// `WrapMath::NormalizeOrZero`.
inline double normalizeOrZero(const Eigen::Vector3d& in, Eigen::Vector3d& out) {
    const double mag = in.norm();
    if (mag >= std::numeric_limits<double>::epsilon()) {
        out = in * (1.0 / mag);
    } else {
        out.setZero();
    }
    return mag;
}

inline double clampedAcos(double x) {
    return std::acos(std::max(-1.0, std::min(1.0, x)));
}

/// `WrapMath::CalcDistanceSquaredPointToLine`.
inline double distanceSquaredPointToLine(const Eigen::Vector3d& point,
                                         const Eigen::Vector3d& linePoint,
                                         const Eigen::Vector3d& line) {
    const Eigen::Vector3d pToLinePt = linePoint - point;
    const Eigen::Vector3d n = line.normalized();
    return (pToLinePt - pToLinePt.dot(n) * n).squaredNorm();
}

/// `WrapMath::GetClosestPointOnLineToPoint`.
inline void closestPointOnLineToPoint(const Eigen::Vector3d& point,
                                      const Eigen::Vector3d& linePoint,
                                      const Eigen::Vector3d& line,
                                      Eigen::Vector3d& closest,
                                      double& t) {
    Eigen::Vector3d v1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d v2 = Eigen::Vector3d::Zero();
    const double mag = normalizeOrZero(point - linePoint, v1);
    const double mag2 = normalizeOrZero(line, v2);
    t = v1.dot(v2) * mag;
    closest = linePoint + t * v2;
    t = (mag2 != 0.0) ? t / mag2 : 0.0;
}

/// `WrapMath::IntersectLines`. Returns false when the lines are parallel, in
/// which case NOTHING is written to the outputs (OpenSim leaves them alone too;
/// unlike OpenSim, no caller here reads them on that branch).
bool intersectLines(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                    const Eigen::Vector3d& p3, const Eigen::Vector3d& p4,
                    Eigen::Vector3d& intersection1, double& s,
                    Eigen::Vector3d& intersection2, double& t) {
    Eigen::Vector3d vec1 = Eigen::Vector3d::Zero();
    const double mag1 = normalizeOrZero(p2 - p1, vec1);
    Eigen::Vector3d vec2 = Eigen::Vector3d::Zero();
    const double mag2 = normalizeOrZero(p4 - p3, vec2);

    const Eigen::Vector3d cross = vec1.cross(vec2);
    const double denominator = cross.squaredNorm();
    if (equalWithinError(denominator, 0.0)) {
        s = t = std::numeric_limits<double>::quiet_NaN();
        return false;
    }

    Eigen::Matrix3d m;
    m.row(0) = (p3 - p1).transpose();
    m.row(1) = vec1.transpose();
    m.row(2) = cross.transpose();
    t = m.determinant() / denominator;
    intersection2 = p3 + t * vec2;

    m.row(1) = vec2.transpose();
    s = m.determinant() / denominator;
    intersection1 = p1 + s * vec1;

    s = (mag1 != 0.0) ? s / mag1 : 0.0;
    t = (mag2 != 0.0) ? t / mag2 : 0.0;
    return true;
}

/// `WrapMath::IntersectLineSegPlane`.
bool intersectLineSegPlane(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2,
                           const Eigen::Vector3d& planeNormal, double d,
                           Eigen::Vector3d& intersection) {
    const Eigen::Vector3d vec = pt2 - pt1;
    const double dotProduct = vec.dot(planeNormal);
    if (std::abs(dotProduct) < kLineEpsilon) return false;
    const double t = (-d - planeNormal.dot(pt1)) / dotProduct;
    if (t < -kLineEpsilon || t > 1.0 + kLineEpsilon) return false;
    intersection = pt1 + t * vec;
    return true;
}

/// `WrapCylinder::_adjust_tangent_point`. Mutates `r1`; returns whether it did.
bool adjustTangentPoint(const Eigen::Vector3d& pt1,
                        const Eigen::Vector3d& axis,
                        Eigen::Vector3d& r1,
                        const Eigen::Vector3d& w1) {
    Eigen::Vector3d prVec = Eigen::Vector3d::Zero();
    Eigen::Vector3d rwVec = Eigen::Vector3d::Zero();
    normalizeOrZero(r1 - pt1, prVec);
    normalizeOrZero(w1 - r1, rwVec);

    const double alpha = clampedAcos(prVec.dot(axis));
    const double omega = clampedAcos(rwVec.dot(axis));
    if (std::abs(alpha - omega) <= kTangencyThreshold) return false;

    Eigen::Vector3d p1a = Eigen::Vector3d::Zero();
    Eigen::Vector3d w1a = Eigen::Vector3d::Zero();
    double t = 0.0;
    closestPointOnLineToPoint(pt1, r1, axis, p1a, t);
    closestPointOnLineToPoint(w1, r1, axis, w1a, t);

    Eigen::Vector3d intersectionOnMuscle = Eigen::Vector3d::Zero();
    Eigen::Vector3d intersectionOnAxisLine = Eigen::Vector3d::Zero();
    double tMuscle = 0.0;
    double tAxisLine = 0.0;
    // DEVIATION 3: OpenSim uses `intersectionOnAxisLine` even when this
    // returns false, where it is uninitialised.
    if (!intersectLines(pt1, w1, p1a, w1a, intersectionOnMuscle, tMuscle,
                        intersectionOnAxisLine, tAxisLine)) {
        return false;
    }

    r1 += 1.5 * (intersectionOnAxisLine - r1);
    return true;
}

/// `WrapCylinder::_make_spiral_path`, length only (DEVIATION 1).
void makeSpiralPath(const Eigen::Vector3d& point1,
                    const Eigen::Vector3d& point2,
                    bool farSideWrap,
                    bool singleWrap,
                    double radius,
                    Eigen::Vector3d& r1,
                    Eigen::Vector3d& r2,
                    double& wrapPathLength) {
    const double sense = farSideWrap ? -1.0 : 1.0;
    int iterations = 0;

    for (;;) {
        Eigen::Vector3d r1a = Eigen::Vector3d::Zero();
        Eigen::Vector3d r2a = Eigen::Vector3d::Zero();
        double t = 0.0;
        closestPointOnLineToPoint(r1, kAxisPoint, kAxisDirection, r1a, t);
        closestPointOnLineToPoint(r2, kAxisPoint, kAxisDirection, r2a, t);

        const double axialDistance = (r2a - r1a).dot(kAxisDirection);

        const Eigen::Vector3d uu = r1 - r1a;
        const Eigen::Vector3d vv = r2 - r2a;
        double theta = std::atan2(uu.cross(vv).dot(kAxisDirection), uu.dot(vv));
        if (farSideWrap) theta = 2.0 * M_PI - theta;

        const double x = radius * theta;
        const double y = axialDistance;
        wrapPathLength = std::sqrt(x * x + y * y);

        // Rotate r1 about the cylinder axis and slide it along the axis.
        const auto spiralPoint = [&](double fraction) {
            const Eigen::AngleAxisd rotation(sense * fraction * theta, kAxisDirection);
            return Eigen::Vector3d(rotation * r1 + kAxisDirection * (fraction * axialDistance));
        };

        if (singleWrap) return;  // OpenSim never adjusts tangency for one wrap

        int numWrapSegments = static_cast<int>(wrapPathLength / 0.002);
        if (numWrapSegments < 1) numWrapSegments = 1;
        // The adjustment happens at i == 1 of a loop over [0, numWrapSegments).
        if (numWrapSegments < 2 || iterations >= kMaxIterations) return;

        const double fraction = 1.0 / static_cast<double>(numWrapSegments);
        const bool adjusted1 = adjustTangentPoint(point1, kAxisDirection, r1,
                                                  spiralPoint(fraction));
        const bool adjusted2 = adjustTangentPoint(point2, kAxisDirection, r2,
                                                  spiralPoint(1.0 - fraction));
        if (!adjusted1 && !adjusted2) return;
        iterations++;
    }
}

}  // namespace

// MARK: - Quadrant and pose

bool decodeWrapQuadrant(const std::string& quadrant, int& wrapAxis, int& wrapSign) {
    std::string q;
    q.reserve(quadrant.size());
    for (char c : quadrant) {
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
        q.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    // OpenSim's defaults, kept so `all` never leaves wrapAxis undefined.
    wrapAxis = 0;
    wrapSign = 1;
    if (q == "-x") { wrapAxis = 0; wrapSign = -1; return true; }
    if (q == "x" || q == "+x") { wrapAxis = 0; wrapSign = 1; return true; }
    if (q == "-y") { wrapAxis = 1; wrapSign = -1; return true; }
    if (q == "y" || q == "+y") { wrapAxis = 1; wrapSign = 1; return true; }
    if (q == "-z") { wrapAxis = 2; wrapSign = -1; return true; }
    if (q == "z" || q == "+z") { wrapAxis = 2; wrapSign = 1; return true; }
    if (q == "all" || q == "unassigned" || q.empty()) { wrapSign = 0; return true; }
    return false;
}

Eigen::Matrix3d bodyFixedXYZRotation(const Eigen::Vector3d& angles) {
    return (Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

// MARK: - WrapCylinder::wrapLine

WrapSegmentResult wrapCylinderLine(const WrapObjectSpec& cylinder,
                                   const Eigen::Vector3d& point1,
                                   const Eigen::Vector3d& point2,
                                   bool singleWrap) {
    WrapSegmentResult out;
    const double radius = cylinder.radius;
    const double rSquared = radius * radius;
    const bool constrained = (cylinder.wrapSign != 0);
    const int wrapAxis = cylinder.wrapAxis;
    const int wrapSign = cylinder.wrapSign;
    bool farSideWrap = false;
    bool longWrap = false;
    WrapAction returnCode = WrapAction::Wrapped;

    // Abort if either point is inside the cylinder.
    if (distanceSquaredPointToLine(point1, kAxisPoint, kAxisDirection) < rSquared ||
        distanceSquaredPointToLine(point2, kAxisPoint, kAxisDirection) < rSquared) {
        out.action = WrapAction::InsideRadius;
        return out;
    }

    // Closest approach between the muscle segment and the cylinder's axis
    // segment. Used by several of the wrap conditions below.
    const Eigen::Vector3d cylinderStart(0.0, 0.0, -0.5 * cylinder.length);
    const Eigen::Vector3d cylinderEnd(0.0, 0.0, 0.5 * cylinder.length);
    Eigen::Vector3d near12 = Eigen::Vector3d::Zero();
    Eigen::Vector3d near00 = Eigen::Vector3d::Zero();
    double t12 = 0.0;
    double t00 = 0.0;
    const bool linesIntersect = intersectLines(point1, point2, cylinderStart,
                                               cylinderEnd, near12, t12, near00, t00);
    // Parallel lines leave near12/near00 unset; OpenSim's `Vec3` default-inits
    // to zero and its checks then compare zeros. Match that explicitly rather
    // than leaving the vectors at whatever they held.
    if (!linesIntersect) {
        near12.setZero();
        near00.setZero();
        t12 = std::numeric_limits<double>::quiet_NaN();
    }
    const bool axisCrossed =
        (near12 - near00).squaredNorm() < rSquared && t12 > 0.0 && t12 < 1.0;

    if (!constrained) {
        if (axisCrossed) {
            returnCode = WrapAction::MandatoryWrap;
        } else {
            out.action = WrapAction::NoWrap;
            return out;
        }
    }

    // Points on the axis closest to point1 and point2.
    Eigen::Vector3d p11 = Eigen::Vector3d::Zero();
    Eigen::Vector3d p22 = Eigen::Vector3d::Zero();
    double t = 0.0;
    closestPointOnLineToPoint(point1, kAxisPoint, kAxisDirection, p11, t);
    closestPointOnLineToPoint(point2, kAxisPoint, kAxisDirection, p22, t);

    // Preliminary tangent point candidates from point1.
    Eigen::Vector3d vv = Eigen::Vector3d::Zero();
    const double p11Distance = normalizeOrZero(point1 - p11, vv);
    double sinTheta = radius / p11Distance;
    double dist = radius * sinTheta;
    Eigen::Vector3d pp = p11 + dist * vv;
    dist = std::sqrt(std::max(0.0, rSquared - dist * dist));
    Eigen::Vector3d uu = kAxisDirection.cross(vv);
    Eigen::Vector3d r1a = pp + dist * uu;
    Eigen::Vector3d r1b = pp - dist * uu;

    // ...and from point2.
    const double p22Distance = normalizeOrZero(point2 - p22, vv);
    sinTheta = radius / p22Distance;
    dist = radius * sinTheta;
    pp = p22 + dist * vv;
    dist = std::sqrt(std::max(0.0, rSquared - dist * dist));
    uu = kAxisDirection.cross(vv);
    Eigen::Vector3d r2a = pp + dist * uu;
    Eigen::Vector3d r2b = pp - dist * uu;

    Eigen::Vector3d r1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d r2 = Eigen::Vector3d::Zero();

    if (constrained) {
        if (dsign(point1[wrapAxis]) == wrapSign || dsign(point2[wrapAxis]) == wrapSign) {
            if (axisCrossed) {
                returnCode = WrapAction::MandatoryWrap;
            } else if (dsign(point1[wrapAxis]) != dsign(point2[wrapAxis]) &&
                       dsign(near12[wrapAxis]) != wrapSign) {
                returnCode = WrapAction::Wrapped;
            } else {
                out.action = WrapAction::NoWrap;
                return out;
            }
        }

        const Eigen::Vector3d r1am = r1a - p11;
        const Eigen::Vector3d r1bm = r1b - p11;
        const Eigen::Vector3d r2am = r2a - p22;
        const Eigen::Vector3d r2bm = r2b - p22;

        const double alpha = clampedAcos(r1am.dot(r2bm) / (r1am.norm() * r2bm.norm()));
        const double beta = clampedAcos(r1bm.dot(r2am) / (r1bm.norm() * r2am.norm()));

        const bool r1aActive = dsign(r1a[wrapAxis]) == wrapSign;
        const bool r1bActive = dsign(r1b[wrapAxis]) == wrapSign;
        if (r1aActive && r1bActive) {
            if (dsign(r2a[wrapAxis]) == wrapSign) {
                r1 = r1b; r2 = r2a; farSideWrap = !(alpha > beta);
            } else {
                r1 = r1a; r2 = r2b; farSideWrap = (alpha > beta);
            }
        } else if (r1aActive && !r1bActive) {
            r1 = r1a; r2 = r2b; farSideWrap = (alpha > beta);
        } else if (!r1aActive && r1bActive) {
            r1 = r1b; r2 = r2a; farSideWrap = !(alpha > beta);
        } else {
            if (dsign(r2a[wrapAxis]) == wrapSign) {
                r1 = r1b; r2 = r2a; farSideWrap = !(alpha > beta);
            } else if (dsign(r2b[wrapAxis]) == wrapSign) {
                r1 = r1a; r2 = r2b; farSideWrap = (alpha > beta);
            } else if (alpha > beta) {
                r1 = r1a; r2 = r2b; farSideWrap = true;
            } else {
                r1 = r1b; r2 = r2a; farSideWrap = true;
            }
        }

        // Short wrap (less than half the cylinder) or long wrap?
        const Eigen::Vector3d sumMuscle = (r1 - point1) + (r2 - point2);
        const Eigen::Vector3d sumR = (r1 - p11) + (r2 - p22);
        if (sumR.dot(sumMuscle) < 0.0) longWrap = true;
    } else {
        Eigen::Vector3d r1am = Eigen::Vector3d::Zero();
        Eigen::Vector3d r1bm = Eigen::Vector3d::Zero();
        Eigen::Vector3d r2am = Eigen::Vector3d::Zero();
        Eigen::Vector3d r2bm = Eigen::Vector3d::Zero();
        normalizeOrZero(r1a - p11, r1am);
        normalizeOrZero(r1b - p11, r1bm);
        normalizeOrZero(r2a - p22, r2am);
        normalizeOrZero(r2b - p22, r2bm);

        const double dot1 = r1am.dot(r2am);
        const double dot2 = r1am.dot(r2bm);
        const double dot3 = r1bm.dot(r2am);
        const double dot4 = r1bm.dot(r2bm);

        if (dot1 > dot2 && dot1 > dot3 && dot1 > dot4) {
            r1 = r1a; r2 = r2a;
        } else if (dot2 > dot3 && dot2 > dot4) {
            r1 = r1a; r2 = r2b;
        } else if (dot3 > dot4) {
            r1 = r1b; r2 = r2a;
        } else {
            r1 = r1b; r2 = r2b;
        }
    }

    // Bisect the angle between r1 and r2 to find the apex edge of the cylinder.
    uu = r1 - p11;
    vv = r2 - p22;
    Eigen::Vector3d bisector = Eigen::Vector3d::Zero();
    normalizeOrZero(uu + vv, bisector);

    // A point along the straight muscle line, weighted by how far each end sits
    // from the axis, and the axis point nearest to it.
    const double split = (p11Distance + p22Distance) != 0.0
        ? p11Distance / (p11Distance + p22Distance)
        : 0.5;
    const Eigen::Vector3d midPoint = point1 + split * (point2 - point1);
    Eigen::Vector3d axisPoint = Eigen::Vector3d::Zero();
    closestPointOnLineToPoint(midPoint, kAxisPoint, kAxisDirection, axisPoint, t);

    Eigen::Vector3d l1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d l2 = Eigen::Vector3d::Zero();
    normalizeOrZero(point1 - axisPoint, l1);
    normalizeOrZero(point2 - axisPoint, l2);

    Eigen::Vector3d planeNormal = Eigen::Vector3d::Zero();
    normalizeOrZero(l1.cross(l2), planeNormal);

    Eigen::Vector3d vert1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d vert2 = Eigen::Vector3d::Zero();
    normalizeOrZero(kAxisDirection.cross(planeNormal), vert1);
    normalizeOrZero(planeNormal.cross(kAxisDirection), vert2);

    const Eigen::Vector3d apex1 = axisPoint + radius * vert1;
    const Eigen::Vector3d apex2 = axisPoint + radius * vert2;
    const double dist1 = (midPoint - apex1).squaredNorm();
    const double dist2 = (midPoint - apex2).squaredNorm();
    Eigen::Vector3d apex;
    if (farSideWrap) {
        apex = (dist1 < dist2) ? apex2 : apex1;
    } else {
        apex = (dist1 < dist2) ? apex1 : apex2;
    }

    // Slide r1/r2 along their edge of tangency by intersecting each with the
    // plane through point1, point2 and the apex.
    normalizeOrZero(point1 - apex, uu);
    normalizeOrZero(point2 - apex, vv);
    normalizeOrZero(uu.cross(vv), planeNormal);
    const double d = -planeNormal.dot(point1);

    const Eigen::Vector3d r1Low = r1 - 10.0 * kAxisDirection;
    const Eigen::Vector3d r1High = r1 + 10.0 * kAxisDirection;
    const Eigen::Vector3d r2Low = r2 - 10.0 * kAxisDirection;
    const Eigen::Vector3d r2High = r2 + 10.0 * kAxisDirection;

    Eigen::Vector3d r1p = Eigen::Vector3d::Zero();
    Eigen::Vector3d r2p = Eigen::Vector3d::Zero();
    Eigen::Vector3d scratch = Eigen::Vector3d::Zero();
    if (intersectLineSegPlane(r1Low, r1High, planeNormal, d, r1p)) {
        closestPointOnLineToPoint(r1p, p11, p22, scratch, t);
        if ((scratch - p22).squaredNorm() < (p11 - p22).squaredNorm()) r1 = r1p;
    }
    if (intersectLineSegPlane(r2Low, r2High, planeNormal, d, r2p)) {
        // Same line as above — OpenSim passes (p11, p22) for BOTH slides, and
        // p11/p22 both sit on the cylinder axis, so `p22` acts as the axis
        // direction. Swapping them here would silently change the line.
        closestPointOnLineToPoint(r2p, p11, p22, scratch, t);
        if ((scratch - p11).squaredNorm() < (p22 - p11).squaredNorm()) r2 = r2p;
    }

    // If BOTH tangent points fall off the ends of the cylinder there is no wrap.
    const double halfLength = cylinder.length / 2.0;
    if ((r1.z() < -halfLength || r1.z() > halfLength) &&
        (r2.z() < -halfLength || r2.z() > halfLength)) {
        out.action = WrapAction::NoWrap;
        return out;
    }

    double wrapPathLength = 0.0;
    makeSpiralPath(point1, point2, longWrap, singleWrap, radius, r1, r2, wrapPathLength);

    out.action = returnCode;
    out.r1 = r1;
    out.r2 = r2;
    out.wrapPathLength = wrapPathLength;
    out.longWrap = longWrap;
    out.farSideWrap = farSideWrap;
    return out;
}

// MARK: - WrapEllipsoid::wrapLine

namespace {

// WrapEllipsoid.cpp's own constants, verbatim.
constexpr double kEllipsoidTiny = 0.00000001;
constexpr double kMuBlendMin = 0.7073;   // 100% fan (must exceed cos(45 deg))
constexpr double kMuBlendMax = 0.9;      // 100% Frans
constexpr int kNumFanSamples = 300;      // larger produces less jitter
constexpr double kSvBoundaryBlend = 0.3;
constexpr double kDesiredSegLength = 0.001;  // metres
constexpr int kMaxSurfaceSegments = 499;
constexpr int kTangentMaxIterations = 50;
constexpr int kTangentInnerMaxIterations = 1000;

inline double sqr(double x) { return x * x; }

/// `WrapEllipsoid::closestPointToEllipse`, Graphics Gems IV. Returns the
/// distance; writes the closest point to (x, y).
double closestPointToEllipse(double a, double b, double u, double v,
                             double& x, double& y) {
    const double a2 = a * a, b2 = b * b;
    const double a2u2 = a2 * u * u, b2v2 = b2 * v * v;

    const bool nearXOrigin = equalWithinError(0.0, u);
    const bool nearYOrigin = equalWithinError(0.0, v);

    if (nearXOrigin && nearYOrigin) {
        if (a < b) {
            x = (u < 0.0 ? -a : a);
            y = v;
            return a;
        }
        x = u;
        y = (v < 0.0 ? -b : b);
        return b;
    }

    if (nearXOrigin) {
        if (a >= b || std::abs(v) >= b - a2 / b) {
            x = u;
            y = (v >= 0 ? b : -b);
            return std::abs(y - v);
        }
        y = b2 * v / (b2 - a2);
        const double dy = y - v;
        const double ydb = y / b;
        x = a * std::sqrt(std::abs(1 - ydb * ydb));
        return std::sqrt(x * x + dy * dy);
    }

    if (nearYOrigin) {
        if (b >= a || std::abs(u) >= a - b2 / a) {
            x = (u >= 0 ? a : -a);
            y = v;
            return std::abs(x - u);
        }
        x = a2 * u / (a2 - b2);
        const double dx = x - u;
        const double xda = x / a;
        y = b * std::sqrt(std::abs(1 - xda * xda));
        return std::sqrt(dx * dx + y * y);
    }

    double t = 0.0;
    if ((u / a) * (u / a) + (v / b) * (v / b) >= 1.0) {
        t = std::max(a, b) * std::sqrt(u * u + v * v);
    }

    double P = t + a2;
    double Q = t + b2;
    for (int i = 0; i < 64; i++) {
        P = t + a2;
        Q = t + b2;
        const double P2 = P * P, Q2 = Q * Q;
        const double f = P2 * Q2 - a2u2 * Q2 - b2v2 * P2;
        if (std::abs(f) < 1e-09) break;
        const double fp = 2.0 * (P * Q * (P + Q) - a2u2 * Q - b2v2 * P);
        t -= f / fp;
    }

    x = a2 * u / P;
    y = b2 * v / Q;
    const double dx = x - u, dy = y - v;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

bool closestPointOnEllipsoid(const Eigen::Vector3d& radii,
                             const Eigen::Vector3d& point,
                             Eigen::Vector3d& closest,
                             int specialCaseAxis) {
    const double a = radii[0], b = radii[1], c = radii[2];
    const double u = point[0], v = point[1], w = point[2];

    // Points near a coordinate plane reduce to a 2-D point-to-ellipse, which
    // the general branch below is not stable for. When more than one plane is
    // close, OpenSim picks the narrowest cross-section.
    if (specialCaseAxis < 0) {
        double minEllipseRadiiSum = std::numeric_limits<double>::infinity();
        for (int i = 0; i < 3; i++) {
            if (!equalWithinError(0.0, point[i])) continue;
            double ellipseRadiiSum = 0.0;
            for (int j = 0; j < 3; j++) {
                if (j != i) ellipseRadiiSum += radii[j];
            }
            if (minEllipseRadiiSum > ellipseRadiiSum) {
                specialCaseAxis = i;
                minEllipseRadiiSum = ellipseRadiiSum;
            }
        }
    }
    if (specialCaseAxis == 0) {
        closest[0] = u;
        closestPointToEllipse(b, c, v, w, closest[1], closest[2]);
        return true;
    }
    if (specialCaseAxis == 1) {
        closest[1] = v;
        closestPointToEllipse(c, a, w, u, closest[2], closest[0]);
        return true;
    }
    if (specialCaseAxis == 2) {
        closest[2] = w;
        closestPointToEllipse(a, b, u, v, closest[0], closest[1]);
        return true;
    }

    const double a2 = a * a, b2 = b * b, c2 = c * c;
    const double a2u2 = a2 * u * u, b2v2 = b2 * v * v, c2w2 = c2 * w * w;

    double t = 0.0;
    if ((u / a) * (u / a) + (v / b) * (v / b) + (w / c) * (w / c) >= 1.0) {
        t = std::max(a, std::max(b, c)) * std::sqrt(u * u + v * v + w * w);
    }

    double P = t + a2, Q = t + b2, R = t + c2;
    bool converged = false;
    for (int i = 0; i < 64; i++) {
        P = t + a2;
        Q = t + b2;
        R = t + c2;
        const double P2 = P * P, Q2 = Q * Q, R2 = R * R;
        const double f = P2 * Q2 * R2 - a2u2 * Q2 * R2 - b2v2 * P2 * R2
                       - c2w2 * P2 * Q2;
        if (std::abs(f) < 1e-09) { converged = true; break; }
        const double PQ = P * Q, PR = P * R, QR = Q * R, PQR = P * Q * R;
        const double fp = 2.0 * (PQR * (QR + PR + PQ) - a2u2 * QR * (Q + R)
                                 - b2v2 * PR * (P + R) - c2w2 * PQ * (P + Q));
        t -= f / fp;
    }

    // DEVIATION 10: OpenSim leaves the outputs unwritten when this fails.
    closest[0] = a2 * u / P;
    closest[1] = b2 * v / Q;
    closest[2] = c2 * w / R;
    return converged;
}

namespace {

/// `WrapEllipsoid::calcTangentPoint`. Slides `r1` along the wrapping plane
/// (`vs`, `vs4`) until the segment `p1 -> r1` is tangent to the ellipsoid.
/// Levenberg-Marquardt on four residuals; every constant is OpenSim's.
void calcTangentPoint(double p1e, Eigen::Vector3d& r1,
                      const Eigen::Vector3d& p1, const Eigen::Vector3d& m,
                      const Eigen::Vector3d& a, const Eigen::Vector3d& vs,
                      double vs4) {
    if (std::abs(p1e) < 0.0001) {
        r1 = p1;
        return;
    }

    Eigen::Vector3d nr1;
    for (int i = 0; i < 3; i++) nr1[i] = 2.0 * (r1[i] - m[i]) / sqr(a[i]);

    double d1 = -nr1.dot(r1);
    Eigen::Vector4d ee;
    auto residuals = [&](const Eigen::Vector3d& rr, double dd1) {
        Eigen::Vector4d e;
        e[0] = vs.dot(rr) + vs4;
        e[1] = -1.0;
        for (int i = 0; i < 3; i++) e[1] += sqr((rr[i] - m[i]) / a[i]);
        Eigen::Vector3d n;
        for (int i = 0; i < 3; i++) n[i] = 2.0 * (rr[i] - m[i]) / sqr(a[i]);
        e[2] = n.dot(rr) + dd1;
        e[3] = n.dot(p1) + dd1;
        return e;
    };
    ee = residuals(r1, d1);

    double ssqo = ee.squaredNorm();
    double ssq = ssqo;
    double alpha = 0.01;
    Eigen::Vector4d vt = Eigen::Vector4d::Zero();

    int nit = 0;
    while (ssq > kEllipsoidTiny && nit < kTangentMaxIterations) {
        nit++;

        // dedth(i, j) = d e_j / d theta_i, theta = (r1.x, r1.y, r1.z, d1).
        Eigen::Matrix4d dedth = Eigen::Matrix4d::Zero();
        for (int i = 0; i < 3; i++) {
            dedth(i, 0) = vs[i];
            dedth(i, 1) = 2.0 * (r1[i] - m[i]) / sqr(a[i]);
            dedth(i, 2) = 2.0 * (2.0 * r1[i] - m[i]) / sqr(a[i]);
            dedth(i, 3) = 2.0 * p1[i] / sqr(a[i]);
        }
        dedth(3, 0) = 0.0;
        dedth(3, 1) = 0.0;
        dedth(3, 2) = 1.0;
        dedth(3, 3) = 1.0;

        Eigen::Vector3d p1r1, p1m;
        normalizeOrZero(p1 - r1, p1r1);
        normalizeOrZero(p1 - m, p1m);
        const double pcos = p1r1.dot(p1m);
        const double dd = (pcos > 0.1) ? 1.0 - std::pow(pcos, 100) : 1.0;

        const Eigen::Vector4d v = -(dedth * ee);
        Eigen::Matrix4d dedth2 = dedth * dedth.transpose();
        Eigen::Vector4d diag = dedth2.diagonal();

        int nit2 = 0;
        while (ssq >= ssqo && nit2 < kTangentInnerMaxIterations) {
            for (int i = 0; i < 4; i++) dedth2(i, i) = diag[i] * (1.0 + alpha);
            const Eigen::Matrix4d ddinv2 = dedth2.inverse();
            if (!ddinv2.allFinite()) return;  // DEVIATION 9: refuse, not NaN

            vt = (dd / 16.0) * (ddinv2 * v);
            r1 += vt.head<3>();
            d1 += vt[3];

            ee = residuals(r1, d1);
            ssqo = ssq;
            ssq = ee.squaredNorm();

            alpha *= 4.0;
            nit2++;
        }

        alpha /= 8.0;

        double fakt = 0.5;
        nit2 = 0;
        while (ssq <= ssqo && nit2 < kTangentInnerMaxIterations) {
            fakt *= 2.0;
            r1 += vt.head<3>() * fakt;
            d1 += vt[3] * fakt;

            ee = residuals(r1, d1);
            ssqo = ssq;
            ssq = ee.squaredNorm();
            nit2++;
        }

        r1 -= vt.head<3>() * fakt;
        d1 -= vt[3] * fakt;

        ee = residuals(r1, d1);
        ssq = ee.squaredNorm();
        ssqo = ssq;

        if (!r1.allFinite()) return;  // DEVIATION 9
    }
}

/// `WrapEllipsoid::CalcDistanceOnEllipsoid` — the surface distance between two
/// points on the ellipsoid, in the plane (`vs`, `vs4`).
///
/// DEVIATION 4: the interior points are streamed. `firstInterior` /
/// `lastInterior` are `wrap_pts[1]` and `wrap_pts[size-2]`, the only two the
/// wrong-way test reads, and `haveInterior` is `wrap_pts.getSize() > 2`.
/// Returns false on the negative-discriminant case OpenSim answers with a NaN.
bool calcDistanceOnEllipsoid(const Eigen::Vector3d& r1, const Eigen::Vector3d& r2,
                             const Eigen::Vector3d& m, const Eigen::Vector3d& a,
                             const Eigen::Vector3d& vs, double vs4,
                             bool farSideWrap, double factor,
                             double& wrapPathLength, int& pathSegments,
                             Eigen::Vector3d& firstInterior,
                             Eigen::Vector3d& lastInterior, bool& haveInterior) {
    haveInterior = false;
    const double len = (r1 - r2).norm() / factor;

    if (len < kDesiredSegLength) {
        // Too short to be worth sampling: r1 and r2 are the whole path.
        wrapPathLength = len * factor;
        pathSegments = 1;
        return true;
    }

    int numPathSegments = static_cast<int>(len / kDesiredSegLength);
    if (numPathSegments <= 0) {
        // Unreachable — len >= kDesiredSegLength above — and OpenSim's version
        // returns an unnormalised length here. Kept as a refusal rather than
        // reproducing a unit bug nothing can reach.
        wrapPathLength = len;
        pathSegments = 0;
        return false;
    }
    if (numPathSegments > kMaxSurfaceSegments) numPathSegments = kMaxSurfaceSegments;

    const int numInteriorPts = numPathSegments - 1;

    int imax = 0;
    for (int i = 1; i < 3; i++) {
        if (std::abs(vs[i]) > std::abs(vs[imax])) imax = i;
    }
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    u[imax] = 1.0;

    const double denom = vs.dot(u);
    if (denom == 0.0) return false;  // DEVIATION 9
    const double mu = (-vs.dot(m) - vs4) / denom;
    const Eigen::Vector3d a0 = m + mu * u;

    Eigen::Vector3d ar1, ar2;
    normalizeOrZero(r1 - a0, ar1);
    normalizeOrZero(r2 - a0, ar2);

    const double phi0 = clampedAcos(ar1.dot(ar2));
    const double dphi = farSideWrap ? -(2.0 * M_PI - phi0) / numPathSegments
                                    : phi0 / numPathSegments;

    Eigen::Vector3d vsz;
    normalizeOrZero(ar1.cross(ar2), vsz);
    const Eigen::Vector3d vsy = vsz.cross(ar1);

    // r0 * (cos phi, sin phi, 0) with r0's columns ar1, vsy, vsz.
    wrapPathLength = 0.0;
    Eigen::Vector3d previous = r1;
    for (int i = 0; i < numInteriorPts; i++) {
        const double phi = (i + 1) * dphi;
        const Eigen::Vector3d r = std::cos(phi) * ar1 + std::sin(phi) * vsy;

        Eigen::Vector3d f1, f2;
        for (int j = 0; j < 3; j++) {
            f1[j] = r[j] / a[j];
            f2[j] = (a0[j] - m[j]) / a[j];
        }
        const double aa = f1.dot(f1);
        const double bb = 2.0 * f1.dot(f2);
        const double cc = f2.dot(f2) - 1.0;
        const double disc = bb * bb - 4.0 * aa * cc;
        if (disc < 0.0 || aa == 0.0) return false;  // DEVIATION 9
        const double mu3 = (-bb + std::sqrt(disc)) / (2.0 * aa);
        const Eigen::Vector3d s = a0 + mu3 * r;

        if (i == 0) firstInterior = s;
        lastInterior = s;
        haveInterior = true;

        wrapPathLength += (s - previous).norm();
        previous = s;
    }
    wrapPathLength += (r2 - previous).norm();
    pathSegments = numPathSegments;
    return true;
}

}  // namespace

WrapSegmentResult wrapEllipsoidLine(const WrapObjectSpec& ellipsoid,
                                    const Eigen::Vector3d& point1,
                                    const Eigen::Vector3d& point2) {
    WrapSegmentResult out;

    const Eigen::Vector3d& dims = ellipsoid.dimensions;
    if (!(dims[0] > 0.0 && dims[1] > 0.0 && dims[2] > 0.0)) return out;

    // Work in units where the ellipsoid is about 1 across: OpenSim's `factor`,
    // which depends only on the geometry and so is the same on every call.
    const double factor = 3.0 / (dims[0] + dims[1] + dims[2]);
    const Eigen::Vector3d p1 = point1 * factor;
    const Eigen::Vector3d p2 = point2 * factor;
    const Eigen::Vector3d m = Eigen::Vector3d::Zero();  // OpenSim's `origin * factor`
    const Eigen::Vector3d a = dims * factor;

    double p1e = -1.0, p2e = -1.0;
    for (int i = 0; i < 3; i++) {
        p1e += sqr((p1[i] - m[i]) / a[i]);
        p2e += sqr((p2[i] - m[i]) / a[i]);
    }
    if (p1e < -0.0001 || p2e < -0.0001) {
        out.action = WrapAction::InsideRadius;
        return out;
    }

    const Eigen::Vector3d p1p2 = p1 - p2;
    Eigen::Vector3d p1m, p2m;
    normalizeOrZero(p1 - m, p1m);
    normalizeOrZero(p2 - m, p2m);
    if (std::abs(p1m.dot(p2m) - 1.0) < 0.0001) {
        out.action = WrapAction::NoWrap;  // p1 -> m and p2 -> m are collinear
        return out;
    }

    // Does the line through p1 and p2 cut the ellipsoid, between the two points?
    Eigen::Vector3d f1, f2;
    for (int i = 0; i < 3; i++) {
        f1[i] = p1p2[i] / a[i];
        f2[i] = (p2[i] - m[i]) / a[i];
    }
    const double aa = f1.dot(f1);
    const double bb = 2.0 * f1.dot(f2);
    const double cc = f2.dot(f2) - 1.0;
    const double disc = bb * bb - 4.0 * aa * cc;
    if (disc < 0.0 || aa == 0.0) {
        out.action = WrapAction::NoWrap;
        return out;
    }
    const double l1 = (-bb + std::sqrt(disc)) / (2.0 * aa);
    const double l2 = (-bb - std::sqrt(disc)) / (2.0 * aa);
    if (!(0.0 < l1 && l1 < 1.0) || !(0.0 < l2 && l2 < 1.0)) {
        out.action = WrapAction::NoWrap;
        return out;
    }

    Eigen::Vector3d r1 = p2 + l1 * p1p2;
    Eigen::Vector3d r2 = p2 + l2 * p1p2;
    const Eigen::Vector3d r1r2 = r2 - r1;

    // ==== the wrapping plane ====

    Eigen::Vector3d mu;
    normalizeOrZero(p1p2, mu);
    for (int i = 0; i < 3; i++) mu[i] = std::abs(mu[i]);
    int bestMu = 0;
    for (int i = 1; i < 3; i++) {
        if (mu[i] > mu[bestMu]) bestMu = i;
    }

    // (1) Frans: cross the plane where the most-parallel major axis is zero,
    // which reduces the point-to-ellipsoid problem to a point-to-ellipse.
    // DEVIATION 11: only bestMu's is computed, because only bestMu's is read.
    if (r1r2[bestMu] == 0.0) {  // DEVIATION 9: exactly tangent, r1 == r2
        out.action = WrapAction::NoWrap;
        out.numericalRefusal = true;
        return out;
    }
    const double tBest = (m[bestMu] - r1[bestMu]) / r1r2[bestMu];
    const Eigen::Vector3d svFrans = r1 + tBest * r1r2;
    Eigen::Vector3d c1Frans = Eigen::Vector3d::Zero();
    closestPointOnEllipsoid(a, svFrans, c1Frans, bestMu);

    double muBest = mu[bestMu];
    Eigen::Vector3d c1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d sv = Eigen::Vector3d::Zero();
    double fanWeight = -std::numeric_limits<double>::infinity();

    // A Frans `sv` outside the r1 -> r2 segment sits outside the ellipsoid and
    // can point sv->c1 nearly 180 deg away from the fan's answer, so it is faded
    // out near the ends. OpenSim mutates mu[bestMu] here and every later test
    // reads the mutated value.
    if (muBest > kMuBlendMin) {
        double s = 1.0;
        if (tBest < 0.0 || tBest > 1.0) {
            s = 0.0;
        } else if (tBest < kSvBoundaryBlend) {
            s = tBest / kSvBoundaryBlend;
        } else if (tBest > (1.0 - kSvBoundaryBlend)) {
            s = (1.0 - tBest) / kSvBoundaryBlend;
        }
        if (s < 1.0) muBest = kMuBlendMin + s * (muBest - kMuBlendMin);
    }
    if (muBest > kMuBlendMin) {
        c1 = c1Frans;
        sv = svFrans;
    }

    // (2) Fan: average the normalised sv->c1 "blades" sampled along r1 -> r2.
    if (muBest < kMuBlendMax) {
        const Eigen::Vector3d svMid = r1 + 0.5 * r1r2;
        Eigen::Vector3d vSum = Eigen::Vector3d::Zero();
        for (int i = 1; i < kNumFanSamples - 1; i++) {
            const double tt = static_cast<double>(i) / kNumFanSamples;
            const Eigen::Vector3d svSample = r1 + tt * r1r2;
            Eigen::Vector3d c1Sample = Eigen::Vector3d::Zero();
            closestPointOnEllipsoid(a, svSample, c1Sample, -1);
            Eigen::Vector3d blade;
            normalizeOrZero(c1Sample - svSample, blade);
            vSum += blade;
        }
        normalizeOrZero(vSum, vSum);
        const Eigen::Vector3d c1Raw = svMid + vSum;

        if (muBest <= kMuBlendMin) {
            closestPointOnEllipsoid(a, c1Raw, c1, -1);
            sv = svMid;
            fanWeight = 1.0;
        } else {
            const double tt = (muBest - kMuBlendMin) / (kMuBlendMax - kMuBlendMin);
            const double oneMinusT = 1.0 - tt;
            Eigen::Vector3d c1Fan = Eigen::Vector3d::Zero();
            closestPointOnEllipsoid(a, c1Raw, c1Fan, -1);
            const Eigen::Vector3d blended = tt * c1 + oneMinusT * c1Fan;
            sv = tt * sv + oneMinusT * svMid;
            closestPointOnEllipsoid(a, blended, c1, -1);
            fanWeight = oneMinusT;
        }
    }
    (void)sv;  // OpenSim stores it for the NEXT call; hybrid never reads it back

    // Seeding r1/r2 from c1 rather than from the line/ellipsoid intersection is
    // what stops the path jumping to the other side while c1 stays put.
    r1 = c1;
    r2 = c1;

    // Wrapping restricted to one half of the ellipsoid: mirror c1 if it landed
    // on the inactive side. Getting this backwards produces a plausible path on
    // the wrong side of the bone.
    bool quadrantFlipped = false;
    if (ellipsoid.wrapSign != 0) {
        const int axis = ellipsoid.wrapAxis;
        const double dist = c1[axis] - m[axis];
        if (dsign(dist) != ellipsoid.wrapSign) {
            quadrantFlipped = true;
            const Eigen::Vector3d origC1 = c1;
            c1[axis] = -c1[axis];
            r1 = c1;
            r2 = c1;

            // DEVIATION 12: this test is false when fanWeight IS the sentinel.
            if (equalWithinError(fanWeight,
                                 -std::numeric_limits<double>::infinity())) {
                fanWeight = 1.0 - (muBest - kMuBlendMin)
                                      / (kMuBlendMax - kMuBlendMin);
            }
            if (fanWeight > 1.0) fanWeight = 1.0;
            if (fanWeight > 0.0) {
                const double bisection = (origC1[axis] + c1[axis]) / 2.0;
                c1[axis] = c1[axis] + fanWeight * (bisection - c1[axis]);
                const Eigen::Vector3d tc1 = c1;
                closestPointOnEllipsoid(a, tc1, c1, -1);
            }
        }
    }

    const Eigen::Vector3d p1c1 = p1 - c1;
    Eigen::Vector3d vs;
    normalizeOrZero(p1p2.cross(p1c1), vs);
    const double vs4 = -vs.dot(c1);

    calcTangentPoint(p1e, r1, p1, m, a, vs, vs4);
    calcTangentPoint(p2e, r2, p2, m, a, vs, vs4);
    if (!r1.allFinite() || !r2.allFinite()) {  // DEVIATION 9
        out.action = WrapAction::NoWrap;
        out.numericalRefusal = true;
        return out;
    }

    bool farSideWrap = false;
    double wrapPathLength = 0.0;
    int pathSegments = 0;
    Eigen::Vector3d w1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d w2 = Eigen::Vector3d::Zero();
    bool haveInterior = false;

    // OpenSim's `goto calc_wrap_path`: at most one retry, and only from the
    // near side to the far side.
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!calcDistanceOnEllipsoid(r1, r2, m, a, vs, vs4, farSideWrap, factor,
                                     wrapPathLength, pathSegments,
                                     w1, w2, haveInterior)) {
            out.action = WrapAction::NoWrap;
            out.numericalRefusal = true;
            return out;
        }
        if (ellipsoid.wrapSign == 0 || !haveInterior || farSideWrap) break;

        // Wrong-way check: the first and last surface segments must leave the
        // tangent points AWAY from p1 and p2.
        Eigen::Vector3d r1p1, r1w1, r2p2, r2w2;
        normalizeOrZero(p1 - r1, r1p1);
        normalizeOrZero(w1 - r1, r1w1);
        normalizeOrZero(p2 - r2, r2p2);
        normalizeOrZero(w2 - r2, r2w2);
        if (r1p1.dot(r1w1) > 0.0 || r2p2.dot(r2w2) > 0.0) {
            farSideWrap = true;
            continue;
        }
        break;
    }

    if (!std::isfinite(wrapPathLength)) {  // DEVIATION 9
        out.action = WrapAction::NoWrap;
        out.numericalRefusal = true;
        return out;
    }

    out.action = WrapAction::MandatoryWrap;
    out.r1 = r1 / factor;
    out.r2 = r2 / factor;
    out.wrapPathLength = wrapPathLength / factor;
    out.farSideWrap = farSideWrap;
    out.quadrantFlipped = quadrantFlipped;
    out.pathSegments = pathSegments;
    return out;
}

// MARK: - GeometryPath::applyWrapObjects

namespace {

/// One vertex of the path being built. `wrapOwner >= 0` marks one of the two
/// points a `PathWrap` inserts; the spiral length is carried on the SECOND of
/// the pair, exactly like `PathWrapPoint::getWrapLength`.
struct PathNode {
    Eigen::Vector3d world = Eigen::Vector3d::Zero();
    int wrapOwner = -1;
    int originalIndex = -1;
    double wrapLength = 0.0;
};

/// `GeometryPath::calcLengthAfterPathComputation`.
double pathLength(const PathNode* path, int count) {
    double length = 0.0;
    for (int i = 1; i < count; i++) {
        const PathNode& a = path[i - 1];
        const PathNode& b = path[i];
        if (a.wrapOwner >= 0 && b.wrapOwner >= 0 && a.wrapOwner == b.wrapOwner) {
            length += b.wrapLength;
        } else {
            length += (b.world - a.world).norm();
        }
    }
    return length;
}

/// Erase the two consecutive nodes at `at`.
void erasePair(PathNode* path, int& count, int at) {
    for (int i = at; i + 2 < count; i++) path[i] = path[i + 2];
    count -= 2;
}

/// Insert `first`,`second` so that they land at [at] and [at+1].
void insertPair(PathNode* path, int& count, int at,
                const PathNode& first, const PathNode& second) {
    for (int i = count - 1; i >= at; i--) path[i + 2] = path[i];
    path[at] = first;
    path[at + 1] = second;
    count += 2;
}

inline void mixSignature(std::uint64_t& signature, std::uint64_t value) {
    signature ^= value + 0x9E3779B97F4A7C15ull + (signature << 6) + (signature >> 2);
}

/// The discrete choices one `PathWrap` made, for the signature.
struct Branch {
    bool engaged = false;
    int segment = -1;
    bool longWrap = false;
    bool farSideWrap = false;
    bool quadrantFlipped = false;
    /// Ellipsoid only. The chord count the surface distance is summed over
    /// steps by one as the arc grows past each millimetre, and L steps with it.
    int pathSegments = 0;
};

}  // namespace

WrappedPathResult solveWrappedPathLength(
    const Eigen::Vector3d* activeWorldPoints,
    const int* activeOriginalIndex,
    int activePointCount,
    int originalPointCount,
    const PathWrapSpec* pathWraps,
    int pathWrapCount,
    const WrapObjectSpec* wrapObjects,
    const Eigen::Isometry3d* wrapBodyTransforms,
    int wrapObjectCount) {

    WrappedPathResult out;
    if (activePointCount < 2) return out;

    // DEVIATION 7: refuse rather than truncate.
    const bool overCap =
        activePointCount + 2 * pathWrapCount > kMaxWrappedPathNodes ||
        pathWrapCount > kMaxPathWrapsPerMuscle;

    PathNode path[kMaxWrappedPathNodes];
    int count = 0;
    const int seeded = overCap ? std::min(activePointCount, kMaxWrappedPathNodes)
                               : activePointCount;
    for (int i = 0; i < seeded; i++) {
        path[i].world = activeWorldPoints[i];
        path[i].originalIndex = activeOriginalIndex ? activeOriginalIndex[i] : i;
        path[i].wrapOwner = -1;
        path[i].wrapLength = 0.0;
    }
    count = seeded;

    if (overCap) {
        out.refused = true;
        out.length = pathLength(path, count);
        return out;
    }
    if (pathWrapCount < 1) {
        out.length = pathLength(path, count);
        return out;
    }

    int order[kMaxPathWrapsPerMuscle];
    WrapAction result[kMaxPathWrapsPerMuscle];
    Branch branch[kMaxPathWrapsPerMuscle];
    for (int i = 0; i < pathWrapCount; i++) {
        order[i] = i;
        result[i] = WrapAction::NoWrap;
    }

    const int maxIterations = pathWrapCount < 2 ? 1 : 8;
    double lastLength = std::numeric_limits<double>::infinity();

    for (int kk = 0; kk < maxIterations; kk++) {
        for (int i = 0; i < pathWrapCount; i++) {
            const int wrapIndex = order[i];
            const PathWrapSpec& spec = pathWraps[wrapIndex];
            result[i] = WrapAction::NoWrap;
            branch[wrapIndex] = Branch();

            // Remove this object's own wrap points before re-solving it.
            for (int j = 0; j + 1 < count; j++) {
                if (path[j].wrapOwner == wrapIndex) { erasePair(path, count, j); break; }
            }

            if (spec.wrapObject < 0 || spec.wrapObject >= wrapObjectCount) continue;
            const WrapObjectSpec& wrapObject = wrapObjects[spec.wrapObject];
            if (!wrapObject.active) continue;
            const bool isCylinder = wrapObject.kind == WrapKind::Cylinder;
            const bool isEllipsoid = wrapObject.kind == WrapKind::Ellipsoid;
            if (!isCylinder && !isEllipsoid) continue;
            if (isCylinder && wrapObject.radius <= 0.0) continue;
            if (isEllipsoid) {
                // DEVIATION 8: only `hybrid` is a pure function of the pose.
                if (spec.method != PathWrapMethod::Hybrid) continue;
                if (!(wrapObject.dimensions[0] > 0.0 &&
                      wrapObject.dimensions[1] > 0.0 &&
                      wrapObject.dimensions[2] > 0.0)) continue;
            }

            const Eigen::Isometry3d toWorld =
                wrapBodyTransforms[spec.wrapObject] * wrapObject.pose;
            const Eigen::Isometry3d toWrapFrame = toWorld.inverse();

            // `<range>` counts the ORIGINAL path point set, so scan it for the
            // first and last of those points that survived into `path`.
            const int wrapStart = (spec.startPoint < 1) ? 0 : spec.startPoint - 1;
            const int wrapEnd = (spec.endPoint < 1) ? originalPointCount - 1
                                                    : spec.endPoint - 1;
            int start = -1;
            int end = -1;
            for (int j = wrapStart; j <= wrapEnd && start < 0; j++) {
                for (int k = 0; k < count; k++) {
                    if (path[k].originalIndex == j) { start = k; break; }
                }
            }
            for (int j = wrapEnd; j >= wrapStart && end < 0; j--) {
                for (int k = 0; k < count; k++) {
                    if (path[k].originalIndex == j) { end = k; break; }
                }
            }
            // OpenSim returns from the WHOLE routine here, leaving the path as
            // it stands. Same.
            if (start < 0 || end < 0) {
                out.length = pathLength(path, count);
                for (int n = 0; n < count; n++) if (path[n].wrapOwner >= 0) out.wrapPointCount++;
                return out;
            }

            bool haveBest = false;
            int bestEndPoint = 0;
            WrapSegmentResult best;
            double minLengthChange = std::numeric_limits<double>::infinity();

            for (int pt1 = start; pt1 < end; pt1++) {
                const int pt2 = pt1 + 1;
                const PathNode& a = path[pt1];
                const PathNode& b = path[pt2];
                // Two auto-wrap points of the SAME object bracket a spiral, not
                // a straight segment; never try to wrap that.
                if (a.wrapOwner >= 0 && b.wrapOwner >= 0 && a.wrapOwner == b.wrapOwner) {
                    continue;
                }

                const Eigen::Vector3d local1 = toWrapFrame * a.world;
                const Eigen::Vector3d local2 = toWrapFrame * b.world;
                const WrapSegmentResult wr =
                    isCylinder
                        ? wrapCylinderLine(wrapObject, local1, local2, pathWrapCount == 1)
                        : wrapEllipsoidLine(wrapObject, local1, local2);
                result[i] = wr.action;
                if (wr.numericalRefusal) out.numericalRefusals++;

                if (wr.action == WrapAction::MandatoryWrap) {
                    best = wr;
                    bestEndPoint = pt2;
                    haveBest = true;
                    break;
                }
                if (wr.action != WrapAction::Wrapped) continue;

                const Eigen::Vector3d r1World = toWorld * wr.r1;
                const Eigen::Vector3d r2World = toWorld * wr.r2;
                const double straight = (b.world - a.world).norm();
                const double wrapped = (r1World - a.world).norm() + wr.wrapPathLength
                                     + (b.world - r2World).norm();
                const double change = wrapped - straight;
                if (change < minLengthChange) {
                    best = wr;
                    bestEndPoint = pt2;
                    haveBest = true;
                    minLengthChange = change;
                }
            }

            if (!haveBest) continue;

            PathNode first;
            first.world = toWorld * best.r1;
            first.wrapOwner = wrapIndex;
            PathNode second;
            second.world = toWorld * best.r2;
            second.wrapOwner = wrapIndex;
            second.wrapLength = best.wrapPathLength;
            insertPair(path, count, bestEndPoint, first, second);

            Branch& record = branch[wrapIndex];
            record.engaged = true;
            record.segment = bestEndPoint;
            record.longWrap = best.longWrap;
            record.farSideWrap = best.farSideWrap;
            record.quadrantFlipped = best.quadrantFlipped;
            record.pathSegments = best.pathSegments;
        }

        const double length = pathLength(path, count);
        if (std::abs(length - lastLength) < 0.0005) break;
        lastLength = length;

        if (kk == 0 && pathWrapCount > 1 &&
            result[0] == WrapAction::NoWrap && result[1] == WrapAction::InsideRadius) {
            order[0] = 1;
            order[1] = 0;
            for (int j = 0; j + 1 < count; j++) {
                if (path[j].wrapOwner == 0) { erasePair(path, count, j); break; }
            }
            branch[0] = Branch();
        }
    }

    out.length = pathLength(path, count);
    for (int i = 0; i < count; i++) {
        if (path[i].wrapOwner >= 0) out.wrapPointCount++;
    }
    for (int i = 0; i < pathWrapCount; i++) {
        const Branch& record = branch[i];
        std::uint64_t packed = record.engaged ? 1ull : 0ull;
        packed |= static_cast<std::uint64_t>(record.longWrap ? 1 : 0) << 1;
        packed |= static_cast<std::uint64_t>(record.farSideWrap ? 1 : 0) << 2;
        packed |= static_cast<std::uint64_t>(record.quadrantFlipped ? 1 : 0) << 3;
        packed |= static_cast<std::uint64_t>(record.segment + 1) << 4;
        packed |= static_cast<std::uint64_t>(order[i] + 1) << 20;
        packed |= static_cast<std::uint64_t>(record.pathSegments & 0x3FF) << 36;
        mixSignature(out.signature, packed);
    }
    // A path whose ACTIVE point set changed is a different function of q too,
    // and the conditional via points are latched per pose, not per stencil.
    mixSignature(out.signature, static_cast<std::uint64_t>(activePointCount));
    return out;
}

}  // namespace biomotion
