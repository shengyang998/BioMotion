/* -------------------------------------------------------------------------- *
 *  MusclePathWrap.cpp — muscle path wrapping over WrapCylinder geometry.      *
 * -------------------------------------------------------------------------- *
 *  Ported from OpenSim (opensim-core), which carries:                         *
 *                                                                            *
 *      OpenSim:  WrapCylinder.cpp / WrapObject.cpp / WrapMath.cpp /           *
 *                GeometryPath.cpp                                            *
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
 *  A line-for-line port of the length half of OpenSim's cylinder wrapping.
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
 *  4. ELLIPSOIDS ARE NOT SOLVED. `WrapEllipsoid` is a numerical geodesic and is
 *     a separate piece of work; those `PathWrap`s stay counted as unmodelled.
 *     A muscle carrying both (TRIlong_r/_l) gets its two cylinders solved and
 *     its ellipsoid skipped, which is a partial path — reported as such rather
 *     than presented as complete.
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
            if (wrapObject.kind != WrapKind::Cylinder) continue;  // DEVIATION 4
            if (wrapObject.radius <= 0.0) continue;

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
                    wrapCylinderLine(wrapObject, local1, local2, pathWrapCount == 1);
                result[i] = wr.action;

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
        packed |= static_cast<std::uint64_t>(record.segment + 1) << 3;
        packed |= static_cast<std::uint64_t>(order[i] + 1) << 20;
        mixSignature(out.signature, packed);
    }
    // A path whose ACTIVE point set changed is a different function of q too,
    // and the conditional via points are latched per pose, not per stencil.
    mixSignature(out.signature, static_cast<std::uint64_t>(activePointCount));
    return out;
}

}  // namespace biomotion
