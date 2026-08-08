// The ported cylinder wrap solver, tested as GEOMETRY — no skeleton, no model,
// no fixture.
//
// ObjC++ (like `IKSolverInternalsTests.mm`) so it can call
// `biomotion::wrapCylinderLine` directly. Everything here is a closed-form
// answer somebody can check on paper, which is the point: the reference fixture
// says whether the whole chain agrees with OpenSim, and these say WHICH PIECE
// is wrong when it does not.
//
// The case that matters most is `quadrant`. It selects which side of the bone
// the path runs around, a backwards one produces a perfectly plausible path,
// and no aggregate error statistic separates "wrapped on the wrong side" from
// "wrapped on the right side with noise" when the geometry is symmetric. So it
// is tested on an ASYMMETRIC configuration where the two sides have different
// lengths, and on a symmetric one where the tangent points must mirror.

#import <XCTest/XCTest.h>

#import "MusclePathWrap.h"

#include <cmath>

using biomotion::WrapAction;
using biomotion::WrapKind;
using biomotion::WrapObjectSpec;
using biomotion::WrapSegmentResult;

@interface MusclePathWrapTests : XCTestCase
@end

@implementation MusclePathWrapTests

/// A cylinder of radius `r` and length `l` at the origin, axis along +z.
static WrapObjectSpec MakeCylinder(double r, double l, const char* quadrant) {
    WrapObjectSpec spec;
    spec.name = "test";
    spec.bodyName = "ground";
    spec.kind = WrapKind::Cylinder;
    spec.radius = r;
    spec.length = l;
    spec.pose = Eigen::Isometry3d::Identity();
    (void)biomotion::decodeWrapQuadrant(quadrant, spec.wrapAxis, spec.wrapSign);
    return spec;
}

static double TotalLength(const WrapSegmentResult& wrap,
                          const Eigen::Vector3d& p1,
                          const Eigen::Vector3d& p2) {
    return (wrap.r1 - p1).norm() + wrap.wrapPathLength + (p2 - wrap.r2).norm();
}

/// Both `wrapped` and `mandatoryWrap` mean the path ran around the surface;
/// they differ only in whether the caller may still prefer another segment.
static bool WrapOccurred(const WrapSegmentResult& wrap) {
    return wrap.action == WrapAction::Wrapped || wrap.action == WrapAction::MandatoryWrap;
}

// MARK: - The closed-form answer

/// Two points on opposite sides of the axis, both in the z = 0 plane. The
/// wrapped path is two tangent lines plus a circular arc, and every term is
/// elementary: `2*sqrt(d^2 - r^2) + r*(pi - 2*acos(r/d))`.
- (void)testSymmetricWrapMatchesTheClosedFormArcLength {
    const double r = 0.05;
    const double d = 0.20;
    const WrapObjectSpec cylinder = MakeCylinder(r, 1.0, "all");
    const Eigen::Vector3d p1(0.0, d, 0.0);
    const Eigen::Vector3d p2(0.0, -d, 0.0);

    const WrapSegmentResult wrap = biomotion::wrapCylinderLine(cylinder, p1, p2, true);

    XCTAssertTrue(wrap.action == WrapAction::MandatoryWrap,
                  @"the straight line runs through the cylinder, so the wrap is mandatory");

    const double tangent = std::sqrt(d * d - r * r);
    const double arc = r * (M_PI - 2.0 * std::acos(r / d));
    const double expected = 2.0 * tangent + arc;
    XCTAssertEqualWithAccuracy(TotalLength(wrap, p1, p2), expected, 1e-9);
    XCTAssertEqualWithAccuracy(wrap.wrapPathLength, arc, 1e-9);
    XCTAssertGreaterThan(TotalLength(wrap, p1, p2), (p2 - p1).norm(),
                         @"wrapping around an obstacle cannot shorten the path");

    // Both tangent points sit ON the surface.
    XCTAssertEqualWithAccuracy(std::hypot(wrap.r1.x(), wrap.r1.y()), r, 1e-9);
    XCTAssertEqualWithAccuracy(std::hypot(wrap.r2.x(), wrap.r2.y()), r, 1e-9);
}

/// The same two points slid along the axis. Pythagoras on the unrolled
/// cylinder: the spiral is `sqrt((r*theta)^2 + dz^2)`.
- (void)testHelicalWrapAddsTheAxialTermUnderPythagoras {
    const double r = 0.05;
    const double d = 0.20;
    const double dz = 0.10;
    const WrapObjectSpec cylinder = MakeCylinder(r, 1.0, "all");
    const Eigen::Vector3d p1(0.0, d, -dz);
    const Eigen::Vector3d p2(0.0, -d, dz);

    const WrapSegmentResult wrap = biomotion::wrapCylinderLine(cylinder, p1, p2, true);
    XCTAssertTrue(wrap.action == WrapAction::MandatoryWrap);

    const double axial = wrap.r2.z() - wrap.r1.z();
    const double planar = std::sqrt(std::max(0.0, wrap.wrapPathLength * wrap.wrapPathLength
                                                  - axial * axial));
    // planar / r is the arc angle; recover it and rebuild the length.
    const double theta = planar / r;
    XCTAssertEqualWithAccuracy(wrap.wrapPathLength,
                               std::sqrt(r * r * theta * theta + axial * axial), 1e-9);
    XCTAssertGreaterThan(std::abs(axial), 0.0,
                         @"the tangent points must separate along the axis");
    XCTAssertGreaterThan(TotalLength(wrap, p1, p2), (p2 - p1).norm());
}

// MARK: - quadrant picks the SIDE

/// Symmetric geometry: the two quadrants must produce MIRRORED tangent points.
/// A length comparison cannot see this — both sides are the same length — which
/// is exactly why a backwards quadrant survives an error statistic.
- (void)testQuadrantMirrorsTheTangentPointsOnSymmetricGeometry {
    const Eigen::Vector3d p1(0.0, 0.20, 0.0);
    const Eigen::Vector3d p2(0.0, -0.20, 0.0);

    const WrapSegmentResult positive =
        biomotion::wrapCylinderLine(MakeCylinder(0.05, 1.0, "x"), p1, p2, true);
    const WrapSegmentResult negative =
        biomotion::wrapCylinderLine(MakeCylinder(0.05, 1.0, "-x"), p1, p2, true);

    XCTAssertTrue(WrapOccurred(positive));
    XCTAssertTrue(WrapOccurred(negative));

    XCTAssertGreaterThan(positive.r1.x(), 0.0, @"quadrant +x must wrap on the +x side");
    XCTAssertGreaterThan(positive.r2.x(), 0.0);
    XCTAssertLessThan(negative.r1.x(), 0.0, @"quadrant -x must wrap on the -x side");
    XCTAssertLessThan(negative.r2.x(), 0.0);

    XCTAssertEqualWithAccuracy(positive.r1.x(), -negative.r1.x(), 1e-9);
    XCTAssertEqualWithAccuracy(TotalLength(positive, p1, p2),
                               TotalLength(negative, p1, p2), 1e-9);
}

/// Asymmetric geometry, where the two sides have DIFFERENT lengths. This is the
/// test that would fail if the quadrant decode were inverted.
- (void)testQuadrantChangesTheLengthWhenTheTwoSidesDiffer {
    const Eigen::Vector3d p1(0.0, 0.20, 0.0);
    const Eigen::Vector3d p2(0.09, -0.20, 0.0);

    const WrapSegmentResult positive =
        biomotion::wrapCylinderLine(MakeCylinder(0.05, 1.0, "x"), p1, p2, true);
    const WrapSegmentResult negative =
        biomotion::wrapCylinderLine(MakeCylinder(0.05, 1.0, "-x"), p1, p2, true);

    XCTAssertTrue(WrapOccurred(positive));
    XCTAssertTrue(WrapOccurred(negative));
    XCTAssertGreaterThan(positive.r1.x(), 0.0);
    XCTAssertLessThan(negative.r1.x(), 0.0);

    const double positiveLength = TotalLength(positive, p1, p2);
    const double negativeLength = TotalLength(negative, p1, p2);
    // The +x side is the short way round here: p2 is displaced towards +x.
    XCTAssertLessThan(positiveLength, negativeLength,
                      @"the side nearer the far point must be the shorter wrap");
    XCTAssertGreaterThan(negativeLength - positiveLength, 0.001,
                         @"the two sides must be separable by more than a millimetre, "
                         @"or this test could not detect an inverted quadrant");
}

/// `DSIGN(0) == +1`, so a path point sitting EXACTLY on the wrap-axis plane
/// counts as being on the +side. The consequence is asymmetric and real: with
/// `quadrant -x` and both points at x == 0, OpenSim never runs its
/// "does the line cut the cylinder" test and returns `wrapped` rather than
/// `mandatoryWrap` — the path still wraps, but a caller comparing several
/// segments may prefer a different one. Recorded here because it is the kind of
/// edge that looks like a bug in a port and is not.
- (void)testAPointExactlyOnTheWrapPlaneCountsAsThePositiveSide {
    const Eigen::Vector3d p1(0.0, 0.20, 0.0);
    const Eigen::Vector3d p2(0.0, -0.20, 0.0);
    const WrapSegmentResult positive =
        biomotion::wrapCylinderLine(MakeCylinder(0.05, 1.0, "x"), p1, p2, true);
    const WrapSegmentResult negative =
        biomotion::wrapCylinderLine(MakeCylinder(0.05, 1.0, "-x"), p1, p2, true);
    XCTAssertTrue(positive.action == WrapAction::MandatoryWrap);
    XCTAssertTrue(negative.action == WrapAction::Wrapped);

    // Nudge both points to x < 0 and the roles swap.
    const Eigen::Vector3d q1(-1e-6, 0.20, 0.0);
    const Eigen::Vector3d q2(-1e-6, -0.20, 0.0);
    XCTAssertTrue(biomotion::wrapCylinderLine(MakeCylinder(0.05, 1.0, "-x"), q1, q2, true)
                      .action == WrapAction::MandatoryWrap);
    XCTAssertTrue(biomotion::wrapCylinderLine(MakeCylinder(0.05, 1.0, "x"), q1, q2, true)
                      .action == WrapAction::Wrapped);
}

- (void)testQuadrantDecodeCoversOpenSimsSpellingsAndRefusesOthers {
    int axis = -7;
    int sign = -7;
    XCTAssertTrue(biomotion::decodeWrapQuadrant("x", axis, sign));
    XCTAssertEqual(axis, 0); XCTAssertEqual(sign, 1);
    XCTAssertTrue(biomotion::decodeWrapQuadrant("+X", axis, sign));
    XCTAssertEqual(axis, 0); XCTAssertEqual(sign, 1);
    XCTAssertTrue(biomotion::decodeWrapQuadrant("-y", axis, sign));
    XCTAssertEqual(axis, 1); XCTAssertEqual(sign, -1);
    XCTAssertTrue(biomotion::decodeWrapQuadrant("Z", axis, sign));
    XCTAssertEqual(axis, 2); XCTAssertEqual(sign, 1);
    XCTAssertTrue(biomotion::decodeWrapQuadrant(" all\n", axis, sign));
    XCTAssertEqual(sign, 0, @"`all` means unconstrained, and leaves no axis to consult");
    XCTAssertTrue(biomotion::decodeWrapQuadrant("Unassigned", axis, sign));
    XCTAssertEqual(sign, 0);
    // OpenSim throws here. Refusing lets the caller drop the wrap object rather
    // than wrap on whichever side the default happens to name.
    XCTAssertFalse(biomotion::decodeWrapQuadrant("w", axis, sign));
    XCTAssertFalse(biomotion::decodeWrapQuadrant("xy", axis, sign));
}

// MARK: - When there is no wrap

- (void)testALineThatMissesTheCylinderDoesNotWrap {
    const WrapObjectSpec cylinder = MakeCylinder(0.05, 1.0, "all");
    const Eigen::Vector3d p1(0.20, 0.20, 0.0);
    const Eigen::Vector3d p2(0.20, -0.20, 0.0);
    const WrapSegmentResult wrap = biomotion::wrapCylinderLine(cylinder, p1, p2, true);
    XCTAssertTrue(wrap.action == WrapAction::NoWrap);
    XCTAssertEqual(wrap.wrapPathLength, 0.0);
}

/// The cylinder is a finite segment of surface. A path crossing the infinite
/// cylinder well beyond both ends must NOT wrap, and this is the check that
/// keeps a short bony landmark from acting like an infinite wall.
- (void)testALineBeyondBothEndsOfTheCylinderDoesNotWrap {
    const WrapObjectSpec cylinder = MakeCylinder(0.05, 0.02, "all");
    const Eigen::Vector3d p1(0.0, 0.20, 0.50);
    const Eigen::Vector3d p2(0.0, -0.20, 0.50);
    const WrapSegmentResult wrap = biomotion::wrapCylinderLine(cylinder, p1, p2, true);
    XCTAssertTrue(wrap.action == WrapAction::NoWrap);
}

- (void)testAPointInsideTheCylinderIsReportedRatherThanWrapped {
    const WrapObjectSpec cylinder = MakeCylinder(0.05, 1.0, "all");
    const Eigen::Vector3d inside(0.01, 0.0, 0.0);
    const Eigen::Vector3d outside(0.0, -0.20, 0.0);
    const WrapSegmentResult wrap = biomotion::wrapCylinderLine(cylinder, inside, outside, true);
    XCTAssertTrue(wrap.action == WrapAction::InsideRadius,
                  @"a path point inside the surface is a modelling problem, not a wrap");
    XCTAssertEqual(wrap.wrapPathLength, 0.0);
}

/// Sweeping a point past the tangency boundary: the wrap must switch on at the
/// point where the wrapped length still equals the straight length, so L(q) is
/// continuous THERE even though its derivative need not be.
- (void)testTheWrapEngagesContinuouslyInLengthAtTheTangencyBoundary {
    const double r = 0.05;
    const WrapObjectSpec cylinder = MakeCylinder(r, 1.0, "all");
    double lastStraight = 0.0;
    double lastWrapped = 0.0;
    bool sawSwitch = false;

    // Slide the segment sideways so it goes from missing the cylinder to
    // cutting through it.
    for (int i = 0; i <= 400; i++) {
        const double x = 0.10 - 0.001 * i;  // 0.10 down to -0.30
        const Eigen::Vector3d p1(x, 0.20, 0.0);
        const Eigen::Vector3d p2(x, -0.20, 0.0);
        const WrapSegmentResult wrap = biomotion::wrapCylinderLine(cylinder, p1, p2, true);
        const double straight = (p2 - p1).norm();
        if (wrap.action == WrapAction::NoWrap) {
            lastStraight = straight;
            continue;
        }
        const double wrapped = TotalLength(wrap, p1, p2);
        if (!sawSwitch) {
            sawSwitch = true;
            // First engaged sample: |x| has just dropped below r, so the wrap
            // is a sliver. The jump in L across the switch must be tiny.
            XCTAssertLessThan(std::abs(wrapped - lastStraight), 0.002,
                              @"L(q) must not step at the moment the wrap engages");
        }
        XCTAssertGreaterThanOrEqual(wrapped, straight - 1e-9);
        lastWrapped = wrapped;
    }
    XCTAssertTrue(sawSwitch, @"the sweep never engaged the wrap, so it tested nothing");
    XCTAssertGreaterThan(lastWrapped, 0.0);
}

// MARK: - The pose transform

/// `<xyz_body_rotation>` is three BODY-FIXED rotations, X then Y then Z. Get the
/// order or the frame wrong and every wrap object sits somewhere plausible and
/// wrong.
- (void)testBodyFixedXYZRotationIsRxRyRz {
    const Eigen::Vector3d angles(-0.6, 0.45, 0.2);
    const Eigen::Matrix3d expected =
        (Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ())).toRotationMatrix();
    const Eigen::Matrix3d actual = biomotion::bodyFixedXYZRotation(angles);
    XCTAssertLessThan((expected - actual).cwiseAbs().maxCoeff(), 1e-12);

    // And it is a rotation, not merely a matrix.
    XCTAssertEqualWithAccuracy(actual.determinant(), 1.0, 1e-12);
    XCTAssertLessThan((actual * actual.transpose() - Eigen::Matrix3d::Identity())
                          .cwiseAbs().maxCoeff(), 1e-12);
}

/// A cylinder that is translated and rotated on its body must wrap identically
/// to one at the origin with the two path points carried into its frame — this
/// is the whole content of `WrapObject::wrapPathSegment`.
- (void)testWrappingIsInvariantUnderTheWrapObjectsOwnPose {
    const Eigen::Vector3d p1(0.0, 0.20, 0.0);
    const Eigen::Vector3d p2(0.06, -0.20, 0.02);
    WrapObjectSpec atOrigin = MakeCylinder(0.05, 1.0, "x");

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = biomotion::bodyFixedXYZRotation(Eigen::Vector3d(-0.6, 0.45, 0.1));
    pose.translation() = Eigen::Vector3d(-0.077, -0.099, 0.061);

    const WrapSegmentResult direct =
        biomotion::wrapCylinderLine(atOrigin, p1, p2, true);
    const WrapSegmentResult moved =
        biomotion::wrapCylinderLine(atOrigin, pose.inverse() * (pose * p1),
                                    pose.inverse() * (pose * p2), true);
    XCTAssertEqual((int)direct.action, (int)moved.action);
    XCTAssertEqualWithAccuracy(direct.wrapPathLength, moved.wrapPathLength, 1e-12);
}

// MARK: - The driver

/// A two-point path with no wrap object is exactly its straight length, so
/// `solveWrappedPathLength` is safe to call for every muscle in the model.
- (void)testAPathWithNoWrapsIsTheStraightPolyline {
    const Eigen::Vector3d points[3] = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.1, 0.0, 0.0),
        Eigen::Vector3d(0.1, 0.2, 0.0),
    };
    const int indices[3] = {0, 1, 2};
    const biomotion::WrappedPathResult result = biomotion::solveWrappedPathLength(
        points, indices, 3, 3, nullptr, 0, nullptr, nullptr, 0);
    XCTAssertEqualWithAccuracy(result.length, 0.3, 1e-12);
    XCTAssertEqual(result.wrapPointCount, 0);
    XCTAssertEqual(result.signature, 0ull);
    XCTAssertFalse(result.refused);
}

/// `<range>` restricts a wrap to part of the path. A wrap whose range excludes
/// the segment that would wrap must leave the length alone.
- (void)testTheRangeRestrictsWhichSegmentsAreConsidered {
    const WrapObjectSpec cylinder = MakeCylinder(0.05, 1.0, "all");
    const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

    // Three points: the segment 0->1 passes through the cylinder, 1->2 does not.
    const Eigen::Vector3d points[3] = {
        Eigen::Vector3d(0.0, 0.20, 0.0),
        Eigen::Vector3d(0.0, -0.20, 0.0),
        Eigen::Vector3d(0.30, -0.20, 0.0),
    };
    const int indices[3] = {0, 1, 2};

    biomotion::PathWrapSpec whole;
    whole.wrapObject = 0;
    const biomotion::WrappedPathResult wrapped = biomotion::solveWrappedPathLength(
        points, indices, 3, 3, &whole, 1, &cylinder, &identity, 1);
    XCTAssertEqual(wrapped.wrapPointCount, 2);
    XCTAssertGreaterThan(wrapped.length, 0.70);

    // Restricted to points 2..3 (1-based), i.e. the segment that misses.
    biomotion::PathWrapSpec tail;
    tail.wrapObject = 0;
    tail.startPoint = 2;
    tail.endPoint = 3;
    const biomotion::WrappedPathResult restricted = biomotion::solveWrappedPathLength(
        points, indices, 3, 3, &tail, 1, &cylinder, &identity, 1);
    XCTAssertEqual(restricted.wrapPointCount, 0);
    XCTAssertEqualWithAccuracy(restricted.length, 0.40 + 0.30, 1e-12);
    XCTAssertNotEqual(restricted.signature, wrapped.signature,
                      @"engaged and disengaged are different branches of L(q)");
}

/// An unsupported surface is skipped, not approximated — and it still counts
/// towards the set size, because OpenSim's own numerics key off that.
- (void)testAnEllipsoidIsSkippedRatherThanTreatedAsACylinder {
    WrapObjectSpec ellipsoid = MakeCylinder(0.05, 1.0, "all");
    ellipsoid.kind = WrapKind::Ellipsoid;
    const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
    const Eigen::Vector3d points[2] = {
        Eigen::Vector3d(0.0, 0.20, 0.0),
        Eigen::Vector3d(0.0, -0.20, 0.0),
    };
    const int indices[2] = {0, 1};
    biomotion::PathWrapSpec spec;
    spec.wrapObject = 0;
    const biomotion::WrappedPathResult result = biomotion::solveWrappedPathLength(
        points, indices, 2, 2, &spec, 1, &ellipsoid, &identity, 1);
    XCTAssertEqualWithAccuracy(result.length, 0.40, 1e-12);
    XCTAssertEqual(result.wrapPointCount, 0);
}

/// The signature is what makes a wrap switch visible to a finite difference. It
/// has to change when engagement changes, and NOT change when it does not.
- (void)testTheSignatureTracksEngagementAndNothingElse {
    const WrapObjectSpec cylinder = MakeCylinder(0.05, 1.0, "all");
    const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
    const int indices[2] = {0, 1};
    biomotion::PathWrapSpec spec;
    spec.wrapObject = 0;

    auto solve = [&](double x) {
        const Eigen::Vector3d points[2] = {
            Eigen::Vector3d(x, 0.20, 0.0),
            Eigen::Vector3d(x, -0.20, 0.0),
        };
        return biomotion::solveWrappedPathLength(points, indices, 2, 2, &spec, 1,
                                                 &cylinder, &identity, 1);
    };

    const auto missA = solve(0.20);
    const auto missB = solve(0.10);
    const auto hitA = solve(0.01);
    const auto hitB = solve(-0.01);

    XCTAssertEqual(missA.wrapPointCount, 0);
    XCTAssertEqual(missB.wrapPointCount, 0);
    XCTAssertEqual(hitA.wrapPointCount, 2);
    XCTAssertEqual(hitB.wrapPointCount, 2);

    XCTAssertEqual(missA.signature, missB.signature,
                   @"two disengaged poses are the same branch");
    XCTAssertEqual(hitA.signature, hitB.signature,
                   @"two engaged poses on the same side are the same branch");
    XCTAssertNotEqual(missA.signature, hitA.signature,
                      @"engaging the wrap MUST change the signature, or the finite "
                      @"difference cannot tell it happened");
}

// MARK: - THE DISCONTINUITY, constructed

/// **A centred finite difference straddling a wrap on/off switch is a
/// fabrication, and here is the number.**
///
/// The switch with a genuine STEP in L is the cylinder-END rule: the surface is
/// a finite segment, so when BOTH tangent points slide past `length/2` the wrap
/// stops being applied and the path snaps back to the straight line — from a
/// wrapped length that is NOT close to it. (The tangency boundary, tested
/// above, is the benign switch: L is continuous there.)
///
/// This finds that switch by bisection, measures the step, and shows what a
/// centred difference across it returns. `MomentArmComputer` is what has to
/// survive this; `MomentArmWrapDiscontinuityTests` drives the same situation
/// through the shipped stencil.
- (void)testACentredDifferenceAcrossTheCylinderEndSwitchFabricatesAMomentArm {
    // Short cylinder, so the end rule fires while the wrap is still large.
    const WrapObjectSpec cylinder = MakeCylinder(0.06, 0.02, "y");
    // `t` slides both points along the cylinder axis. At t = 0 the wrap sits on
    // the surface; as t grows both tangent points run off the end.
    auto lengthAt = [&](double t) {
        const Eigen::Vector3d p1(-0.05, 0.09, t);
        const Eigen::Vector3d p2(0.05, -0.09, t + 0.004);
        const WrapSegmentResult wrap = biomotion::wrapCylinderLine(cylinder, p1, p2, true);
        const double straight = (p2 - p1).norm();
        if (wrap.action == WrapAction::NoWrap || wrap.action == WrapAction::InsideRadius) {
            return std::make_pair(straight, false);
        }
        return std::make_pair(TotalLength(wrap, p1, p2), true);
    };

    // Bracket the switch.
    double engaged = 0.0;
    double free = 0.0;
    bool bracketed = false;
    XCTAssertTrue(lengthAt(0.0).second, @"the sweep must start engaged");
    for (double t = 0.0; t <= 0.40; t += 0.002) {
        if (!lengthAt(t).second) { engaged = t - 0.002; free = t; bracketed = true; break; }
    }
    XCTAssertTrue(bracketed, @"the sweep never disengaged, so it tested nothing");

    for (int i = 0; i < 80; i++) {
        const double middle = 0.5 * (engaged + free);
        if (lengthAt(middle).second) engaged = middle; else free = middle;
    }

    const double step = 1e-4;
    const double switchPoint = 0.5 * (engaged + free);
    const double before = lengthAt(switchPoint - step).first;
    const double after = lengthAt(switchPoint + step).first;
    const double jump = std::abs(after - before);
    const double centred = (after - before) / (2.0 * step);
    const double oneSided = (lengthAt(switchPoint - step).first
                             - lengthAt(switchPoint - 2.0 * step).first) / step;
    NSLog(@"WRAP-SWITCH t=%.9f  L- %.6f  L+ %.6f  jump %.6f m  "
          @"centred %.3f m/unit  one-sided %.3f m/unit",
          switchPoint, before, after, jump, centred, oneSided);

    XCTAssertGreaterThan(jump, 0.005,
                         @"the cylinder-end switch must step L by more than 5 mm, or this "
                         @"construction is not the hazard it claims to be");
    XCTAssertGreaterThan(std::abs(centred), 20.0,
                         @"a centred difference across the step returns a derivative of "
                         @"order jump/2eps — metres per radian, not centimetres");
    XCTAssertLessThan(std::abs(oneSided), 5.0,
                      @"while the one-sided difference on the engaged branch stays bounded");
    XCTAssertGreaterThan(std::abs(centred) / std::max(1e-9, std::abs(oneSided)), 10.0,
                         @"and the fabrication is at least an order of magnitude larger");
}

@end
