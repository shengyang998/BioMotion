// The ported cylinder and ellipsoid wrap solvers, tested as GEOMETRY — no
// skeleton, no model, no fixture.
//
// ObjC++ so it can call `biomotion::wrapCylinderLine` and
// `biomotion::wrapEllipsoidLine` directly.
// Everything here is an answer somebody can check on paper — a closed form, an
// invariant, or an exhaustive search — which is the point: the reference fixture
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
#include <limits>
#include <utility>

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

/// An ellipsoid with semi-axes `(a, b, c)` at the origin.
static WrapObjectSpec MakeEllipsoid(double a, double b, double c,
                                    const char* quadrant) {
    WrapObjectSpec spec;
    spec.name = "test";
    spec.bodyName = "ground";
    spec.kind = WrapKind::Ellipsoid;
    spec.dimensions = Eigen::Vector3d(a, b, c);
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

/// An ellipsoid whose `<method>` is not `hybrid` is skipped, not approximated
/// as hybrid. The three methods choose the wrapping plane differently and only
/// hybrid is a pure function of the pose (DEVIATION 8), so treating one as the
/// other would silently differentiate a function of call history.
- (void)testANonHybridEllipsoidIsSkippedRatherThanSolvedAsHybrid {
    const WrapObjectSpec ellipsoid = MakeEllipsoid(0.05, 0.05, 0.05, "all");
    const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
    const Eigen::Vector3d points[2] = {
        Eigen::Vector3d(0.0, 0.20, 0.0),
        Eigen::Vector3d(0.0, -0.20, 0.0),
    };
    const int indices[2] = {0, 1};

    biomotion::PathWrapSpec hybrid;
    hybrid.wrapObject = 0;
    hybrid.method = biomotion::PathWrapMethod::Hybrid;
    const biomotion::WrappedPathResult solved = biomotion::solveWrappedPathLength(
        points, indices, 2, 2, &hybrid, 1, &ellipsoid, &identity, 1);
    XCTAssertEqual(solved.wrapPointCount, 2, @"hybrid must actually wrap here");
    XCTAssertGreaterThan(solved.length, 0.40);

    biomotion::PathWrapSpec other;
    other.wrapObject = 0;
    other.method = biomotion::PathWrapMethod::Unsupported;
    const biomotion::WrappedPathResult skipped = biomotion::solveWrappedPathLength(
        points, indices, 2, 2, &other, 1, &ellipsoid, &identity, 1);
    XCTAssertEqualWithAccuracy(skipped.length, 0.40, 1e-12);
    XCTAssertEqual(skipped.wrapPointCount, 0);
}

/// A surface the port has no solver for at all is still skipped.
- (void)testAnUnsupportedSurfaceIsSkippedRatherThanTreatedAsACylinder {
    WrapObjectSpec other = MakeCylinder(0.05, 1.0, "all");
    other.kind = WrapKind::Unsupported;
    const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
    const Eigen::Vector3d points[2] = {
        Eigen::Vector3d(0.0, 0.20, 0.0),
        Eigen::Vector3d(0.0, -0.20, 0.0),
    };
    const int indices[2] = {0, 1};
    biomotion::PathWrapSpec spec;
    spec.wrapObject = 0;
    const biomotion::WrappedPathResult result = biomotion::solveWrappedPathLength(
        points, indices, 2, 2, &spec, 1, &other, &identity, 1);
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

// MARK: - The ellipsoid

/// The one closed form an ellipsoid has: make it a SPHERE. Then the geodesic
/// between the two tangent points is a great-circle arc and every term is
/// elementary — `r*(angle between the two rays − acos(r/d1) − acos(r/d2))`,
/// plus two tangent lines of length `sqrt(d^2 − r^2)`.
///
/// This is the test that would catch a wrapping plane chosen wrongly, a
/// tangent-point solve that did not converge, or a geodesic summed over the
/// wrong arc — none of which an aggregate error statistic separates from noise.
/// `FullBody.osim`'s `ANC_l`/`ANC_r` are literally this: `0.02 0.02 0.02`.
///
/// The residual is not zero and must not be: `CalcDistanceOnEllipsoid` sums
/// CHORDS, one per millimetre of arc, so it under-reads the arc by
/// `s·(Δφ)²/24`. That is a property of OpenSim's algorithm, and reproducing it
/// is the point — the tolerance below is that quantity, not a fudge.
- (void)testSphericalWrapMatchesTheGreatCircleArc {
    const double r = 0.02;
    const WrapObjectSpec sphere = MakeEllipsoid(r, r, r, "all");
    const Eigen::Vector3d p1(-0.06, 0.005, 0.0);
    const Eigen::Vector3d p2(0.06, 0.005, 0.0);
    const WrapSegmentResult wrap = biomotion::wrapEllipsoidLine(sphere, p1, p2);
    XCTAssertTrue(WrapOccurred(wrap), @"the segment passes 5 mm from a 20 mm sphere");

    const double d1 = p1.norm();
    const double d2 = p2.norm();
    const double apart = std::acos(p1.normalized().dot(p2.normalized()));
    const double arc = r * (apart - std::acos(r / d1) - std::acos(r / d2));
    const double tangent1 = std::sqrt(d1 * d1 - r * r);
    const double tangent2 = std::sqrt(d2 * d2 - r * r);

    // The chord sum under-reads by s(dphi)^2/24 with dphi = arc/(r*segments).
    const double dphi = (arc / r) / std::max(1, wrap.pathSegments);
    const double chordDeficit = arc * dphi * dphi / 24.0;
    NSLog(@"SPHERE-WRAP arc ours %.9f closed %.9f delta %.3e  predicted chord deficit %.3e  "
          @"segments %d  |r1| %.9f", wrap.wrapPathLength, arc,
          wrap.wrapPathLength - arc, chordDeficit, wrap.pathSegments, wrap.r1.norm());

    XCTAssertEqualWithAccuracy(wrap.r1.norm(), r, 1e-5,
                               @"the first tangent point must land ON the sphere");
    XCTAssertEqualWithAccuracy(wrap.r2.norm(), r, 1e-5);
    XCTAssertEqualWithAccuracy((p1 - wrap.r1).norm(), tangent1, 1e-5);
    XCTAssertEqualWithAccuracy((p2 - wrap.r2).norm(), tangent2, 1e-5);
    XCTAssertLessThan(wrap.wrapPathLength, arc,
                      @"a chord sum can only under-read an arc");
    XCTAssertEqualWithAccuracy(wrap.wrapPathLength, arc, 4.0 * chordDeficit,
                               @"and by no more than the chord deficit its own segment "
                               @"count implies");
}

/// The same construction on a genuinely triaxial ellipsoid has no closed form,
/// so the check is a different invariant: every point the solver reports lies
/// ON the surface, and the wrapped path is longer than the straight line but
/// shorter than going round the other way.
- (void)testTriaxialWrapPutsBothTangentPointsOnTheSurface {
    const WrapObjectSpec ellipsoid = MakeEllipsoid(0.035, 0.02, 0.02, "all");
    const Eigen::Vector3d p1(-0.05, 0.004, -0.05);
    const Eigen::Vector3d p2(0.05, 0.004, 0.05);
    const WrapSegmentResult wrap = biomotion::wrapEllipsoidLine(ellipsoid, p1, p2);
    XCTAssertTrue(WrapOccurred(wrap));

    auto onSurface = [&](const Eigen::Vector3d& p) {
        const Eigen::Vector3d a(0.035, 0.02, 0.02);
        return std::pow(p.x() / a.x(), 2) + std::pow(p.y() / a.y(), 2)
             + std::pow(p.z() / a.z(), 2);
    };
    NSLog(@"TRIAXIAL-WRAP f(r1)=%.9f f(r2)=%.9f straight %.6f wrapped %.6f segments %d",
          onSurface(wrap.r1), onSurface(wrap.r2), (p2 - p1).norm(),
          TotalLength(wrap, p1, p2), wrap.pathSegments);
    XCTAssertEqualWithAccuracy(onSurface(wrap.r1), 1.0, 1e-4);
    XCTAssertEqualWithAccuracy(onSurface(wrap.r2), 1.0, 1e-4);
    XCTAssertGreaterThan(TotalLength(wrap, p1, p2), (p2 - p1).norm(),
                         @"going around cannot be shorter than going through");
}

/// `<quadrant>` picks WHICH side of the bone the path runs around, and on a
/// sphere with the chord through the centre the two sides mirror exactly.
/// Getting it backwards is the failure mode that renders beautifully and is
/// wrong, so it is tested where the answer is unambiguous: `+z` and `-z` must
/// put the contact on opposite sides of z = 0 and cost the same.
- (void)testEllipsoidQuadrantSelectsTheSideAndMirrorsOnASphere {
    const double r = 0.02;
    // Straight through the centre along x: the two ways round are congruent, so
    // a length comparison cannot see the quadrant and the tangent points must.
    const Eigen::Vector3d p1(-0.06, 0.0, 0.0);
    const Eigen::Vector3d p2(0.06, 0.0, 0.0);
    const WrapSegmentResult plus =
        biomotion::wrapEllipsoidLine(MakeEllipsoid(r, r, r, "z"), p1, p2);
    const WrapSegmentResult minus =
        biomotion::wrapEllipsoidLine(MakeEllipsoid(r, r, r, "-z"), p1, p2);
    XCTAssertTrue(WrapOccurred(plus));
    XCTAssertTrue(WrapOccurred(minus));
    NSLog(@"ELLIPSOID-QUADRANT +z r1 (%+.6f %+.6f %+.6f) len %.9f | "
          @"-z r1 (%+.6f %+.6f %+.6f) len %.9f",
          plus.r1.x(), plus.r1.y(), plus.r1.z(), TotalLength(plus, p1, p2),
          minus.r1.x(), minus.r1.y(), minus.r1.z(), TotalLength(minus, p1, p2));
    XCTAssertGreaterThan(plus.r1.z(), 0.001, @"+z must contact the +z side");
    XCTAssertLessThan(minus.r1.z(), -0.001, @"-z must contact the -z side");
    XCTAssertEqualWithAccuracy(plus.r1.z(), -minus.r1.z(), 1e-9);
    XCTAssertEqualWithAccuracy(TotalLength(plus, p1, p2), TotalLength(minus, p1, p2), 1e-9,
                               @"a sphere is symmetric, so the two sides cost the same");
}

/// …and on an ellipsoid whose two sides are NOT symmetric, the quadrant has to
/// change the LENGTH. A test that only ever checks mirrored geometry cannot
/// tell a correct quadrant from one that is ignored.
- (void)testEllipsoidQuadrantChangesTheLengthWhenTheTwoSidesDiffer {
    // Offset the segment towards +x so the two ways round are different lengths.
    const Eigen::Vector3d p1(0.006, -0.06, 0.0);
    const Eigen::Vector3d p2(0.006, 0.06, 0.0);
    const WrapSegmentResult plus =
        biomotion::wrapEllipsoidLine(MakeEllipsoid(0.03, 0.02, 0.02, "x"), p1, p2);
    const WrapSegmentResult minus =
        biomotion::wrapEllipsoidLine(MakeEllipsoid(0.03, 0.02, 0.02, "-x"), p1, p2);
    XCTAssertTrue(WrapOccurred(plus));
    XCTAssertTrue(WrapOccurred(minus));
    const double lengthPlus = TotalLength(plus, p1, p2);
    const double lengthMinus = TotalLength(minus, p1, p2);
    NSLog(@"ELLIPSOID-QUADRANT-ASYM +x %.9f  -x %.9f  delta %.6f mm",
          lengthPlus, lengthMinus, 1000.0 * (lengthMinus - lengthPlus));
    XCTAssertGreaterThan(plus.r1.x(), 0.0);
    XCTAssertLessThan(minus.r1.x(), 0.0);
    XCTAssertGreaterThan(std::abs(lengthMinus - lengthPlus), 5e-5,
                         @"the two sides of this ellipsoid are different lengths; a solver "
                         @"that ignored the quadrant would return the same number twice");
}

/// The ellipsoid engages only when the straight segment actually PIERCES it —
/// a line that misses, or whose intersections lie outside the segment, is not a
/// wrap. And a point inside is reported rather than wrapped.
- (void)testALineThatMissesOrStopsShortOfTheEllipsoidDoesNotWrap {
    const WrapObjectSpec ellipsoid = MakeEllipsoid(0.02, 0.02, 0.02, "all");
    const WrapSegmentResult misses = biomotion::wrapEllipsoidLine(
        ellipsoid, Eigen::Vector3d(-0.06, 0.05, 0.0), Eigen::Vector3d(0.06, 0.05, 0.0));
    XCTAssertEqual((int)misses.action, (int)WrapAction::NoWrap);
    XCTAssertEqualWithAccuracy(misses.wrapPathLength, 0.0, 0.0);

    // The infinite line pierces it, but both intersections are behind p2.
    const WrapSegmentResult shortOf = biomotion::wrapEllipsoidLine(
        ellipsoid, Eigen::Vector3d(0.10, 0.005, 0.0), Eigen::Vector3d(0.04, 0.005, 0.0));
    XCTAssertEqual((int)shortOf.action, (int)WrapAction::NoWrap);

    const WrapSegmentResult inside = biomotion::wrapEllipsoidLine(
        ellipsoid, Eigen::Vector3d(0.001, 0.0, 0.0), Eigen::Vector3d(0.06, 0.03, 0.0));
    XCTAssertEqual((int)inside.action, (int)WrapAction::InsideRadius,
                   @"a point inside is a modelling error to report, not a wrap to solve");
}

/// L must be CONTINUOUS where the ellipsoid starts wrapping: at first contact
/// the two intersection points coincide, so the arc is zero and the wrapped
/// path equals the straight one. If it were not, every moment arm near the
/// engagement boundary would be fabricated.
- (void)testTheEllipsoidEngagesContinuouslyInLength {
    const WrapObjectSpec ellipsoid = MakeEllipsoid(0.02, 0.02, 0.02, "all");
    // `h` is the segment's distance from the centre: h > r misses, h < r wraps.
    auto lengthAt = [&](double h) {
        const Eigen::Vector3d p1(-0.06, h, 0.0);
        const Eigen::Vector3d p2(0.06, h, 0.0);
        const WrapSegmentResult wrap = biomotion::wrapEllipsoidLine(ellipsoid, p1, p2);
        const double straight = (p2 - p1).norm();
        if (!WrapOccurred(wrap)) return std::make_pair(straight, false);
        return std::make_pair(TotalLength(wrap, p1, p2), true);
    };

    double wrapping = 0.0195;
    double free = 0.0205;
    XCTAssertTrue(lengthAt(wrapping).second, @"the sweep must start engaged");
    XCTAssertFalse(lengthAt(free).second, @"and end disengaged");
    for (int i = 0; i < 60; i++) {
        const double middle = 0.5 * (wrapping + free);
        if (lengthAt(middle).second) wrapping = middle; else free = middle;
    }
    const double jump = std::abs(lengthAt(free).first - lengthAt(wrapping).first);
    NSLog(@"ELLIPSOID-ENGAGE h*=%.9f  L_wrapped %.9f  L_free %.9f  jump %.3e m",
          0.5 * (wrapping + free), lengthAt(wrapping).first, lengthAt(free).first, jump);
    XCTAssertLessThan(jump, 1e-6,
                      @"the engagement boundary is the BENIGN switch: the arc goes to zero "
                      @"there, so L is continuous and a centred difference is legitimate");
}

/// `closestPointOnEllipsoid` is Graphics Gems IV and is used ~300 times per
/// engaged solve, including on the two axis-aligned special cases. Checked
/// against a brute-force search over the surface — slow, obvious, independent.
///
/// The radii are NORMALISED (`3/(a+b+c)` times the metre dimensions of
/// `TRIlonghh_*`), because that is the only domain the routine is valid on: its
/// convergence test is `|f| < 1e-9` on a degree-6 polynomial in the radii, so at
/// metre scale `f` starts below the tolerance and it returns the query point
/// untouched. `wrapEllipsoidLine` normalises before every call.
- (void)testClosestPointOnEllipsoidAgreesWithABruteForceSearch {
    const Eigen::Vector3d metres(0.035, 0.02, 0.02);
    const double factor = 3.0 / (metres.x() + metres.y() + metres.z());
    const Eigen::Vector3d radii = metres * factor;
    const Eigen::Vector3d probes[4] = {
        Eigen::Vector3d(0.05, 0.03, 0.02) * factor,    // general
        Eigen::Vector3d(0.0, 0.04, 0.03) * factor,     // on the x = 0 plane
        Eigen::Vector3d(0.06, 0.0, 0.0) * factor,      // on two planes at once
        Eigen::Vector3d(0.004, 0.003, 0.002) * factor  // inside
    };
    for (const Eigen::Vector3d& probe : probes) {
        Eigen::Vector3d closest = Eigen::Vector3d::Zero();
        const bool converged =
            biomotion::closestPointOnEllipsoid(radii, probe, closest, -1);
        XCTAssertTrue(converged, @"the Newton iteration must converge on ordinary input");

        double best = std::numeric_limits<double>::infinity();
        Eigen::Vector3d bestPoint = Eigen::Vector3d::Zero();
        const int steps = 900;
        for (int i = 0; i <= steps; i++) {
            const double theta = M_PI * i / steps;
            for (int j = 0; j < 2 * steps; j++) {
                const double phi = M_PI * j / steps;
                const Eigen::Vector3d s(radii.x() * std::sin(theta) * std::cos(phi),
                                        radii.y() * std::sin(theta) * std::sin(phi),
                                        radii.z() * std::cos(theta));
                const double d = (s - probe).squaredNorm();
                if (d < best) { best = d; bestPoint = s; }
            }
        }
        NSLog(@"CLOSEST-POINT probe (%.3f %.3f %.3f) ours (%.6f %.6f %.6f) "
              @"brute (%.6f %.6f %.6f) gap %.3e m",
              probe.x(), probe.y(), probe.z(), closest.x(), closest.y(), closest.z(),
              bestPoint.x(), bestPoint.y(), bestPoint.z(),
              (closest - bestPoint).norm());
        XCTAssertLessThan((closest - bestPoint).norm(), 2e-3,
                          @"the analytic closest point must land where an exhaustive "
                          @"search puts it, to the search's own grid resolution");
    }

    // …and the trap itself, so nobody "fixes" the normalisation away: the same
    // query at metre scale returns the query point untouched.
    const Eigen::Vector3d probe(0.004, 0.003, 0.002);
    Eigen::Vector3d unnormalised = Eigen::Vector3d::Zero();
    (void)biomotion::closestPointOnEllipsoid(metres, probe, unnormalised, -1);
    NSLog(@"CLOSEST-POINT-UNNORMALISED radii in metres -> (%.6f %.6f %.6f), query was "
          @"(%.6f %.6f %.6f)", unnormalised.x(), unnormalised.y(), unnormalised.z(),
          probe.x(), probe.y(), probe.z());
    XCTAssertEqualWithAccuracy((unnormalised - probe).norm(), 0.0, 1e-12,
                               @"at metre scale the degree-6 residual is already below "
                               @"the 1e-9 tolerance, so the routine stops at t = 0 and "
                               @"hands back the query point. This is not a bug to fix — "
                               @"it is why OpenSim's `factor` exists");
}

/// The ellipsoid's geodesic is summed over `(int)(|r1−r2| / 1 mm)` chords, so L
/// STEPS by a small amount every time that integer changes. Small is not zero:
/// divided by `2·eps = 2e-4` it becomes a moment arm. The signature carries the
/// chord count for exactly this reason, so `MomentArmComputer` sees the switch.
- (void)testTheChordCountIsPartOfTheSignatureBecauseLStepsWithIt {
    const WrapObjectSpec ellipsoid = MakeEllipsoid(0.02, 0.02, 0.02, "all");
    const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
    const int indices[2] = {0, 1};
    biomotion::PathWrapSpec spec;
    spec.wrapObject = 0;

    auto solve = [&](double h) {
        const Eigen::Vector3d points[2] = {Eigen::Vector3d(-0.06, h, 0.0),
                                           Eigen::Vector3d(0.06, h, 0.0)};
        return biomotion::solveWrappedPathLength(points, indices, 2, 2, &spec, 1,
                                                 &ellipsoid, &identity, 1);
    };

    // Sweep the segment towards the centre; the arc grows and the chord count
    // ticks up. Find one tick.
    biomotion::WrappedPathResult previous = solve(0.019);
    XCTAssertGreaterThan(previous.wrapPointCount, 0, @"the sweep must start engaged");
    bool foundStep = false;
    double stepAt = 0.0;
    for (double h = 0.019; h > 0.004; h -= 0.00002) {
        const biomotion::WrappedPathResult now = solve(h);
        if (now.wrapPointCount == 0) break;
        if (now.signature != previous.signature) {
            foundStep = true;
            stepAt = h;
            NSLog(@"ELLIPSOID-CHORD-STEP h=%.6f  L %.9f -> %.9f  delta %.3e m",
                  h, previous.length, now.length, now.length - previous.length);
            break;
        }
        previous = now;
    }
    XCTAssertTrue(foundStep,
                  @"the chord count never changed over a 15 mm sweep, so either the "
                  @"signature does not carry it or the sweep is degenerate");
    const double before = solve(stepAt + 0.00002).length;
    const double after = solve(stepAt).length;
    XCTAssertNotEqual(before, after,
                      @"the branch changed, so L changed with it");
    XCTAssertLessThan(std::abs(after - before), 1e-4,
                      @"a chord-count tick is a SMALL step — this is not the "
                      @"cylinder-end hazard, and calling it one would be alarmism");
}

/// Hybrid ellipsoid wrapping must be a pure function of its inputs. OpenSim
/// seeds the solve from the PREVIOUS call's result; if any of that leaked
/// through, the same pose would answer differently depending on what ran before
/// it, and differentiating it would not be differentiating a function of q.
- (void)testEllipsoidWrappingIsAPureFunctionOfItsInputs {
    const WrapObjectSpec target = MakeEllipsoid(0.025, 0.02, 0.02, "-y");
    const Eigen::Vector3d p1(-0.05, -0.05, 0.003);
    const Eigen::Vector3d p2(0.05, 0.05, 0.003);
    const WrapSegmentResult first = biomotion::wrapEllipsoidLine(target, p1, p2);
    XCTAssertTrue(WrapOccurred(first));

    // Anything else, on different geometry, on the other side, at other angles.
    (void)biomotion::wrapEllipsoidLine(MakeEllipsoid(0.015, 0.015, 0.10, "x"),
                                       Eigen::Vector3d(-0.04, -0.04, 0.0),
                                       Eigen::Vector3d(0.04, 0.04, 0.0));
    (void)biomotion::wrapEllipsoidLine(MakeEllipsoid(0.02, 0.02, 0.02, "x"),
                                       Eigen::Vector3d(0.0, -0.06, 0.004),
                                       Eigen::Vector3d(0.0, 0.06, 0.004));
    (void)biomotion::wrapEllipsoidLine(target, p2, p1);

    const WrapSegmentResult again = biomotion::wrapEllipsoidLine(target, p1, p2);
    XCTAssertEqual(again.wrapPathLength, first.wrapPathLength,
                   @"bit-identical, not close: hybrid overwrites every field OpenSim "
                   @"seeds from the previous wrap, so there is no state to carry");
    XCTAssertEqual((again.r1 - first.r1).norm(), 0.0);
    XCTAssertEqual((again.r2 - first.r2).norm(), 0.0);
    XCTAssertEqual(again.pathSegments, first.pathSegments);
}

/// Moving the ellipsoid's owning BODY and the path points together must not
/// change the length. This is the driver's frame arithmetic, not the solver's:
/// an ellipsoid is triaxial, so it is only invariant when the rotation is
/// applied to both, and a driver that dropped or transposed one of the two
/// transforms would still look plausible on a sphere.
- (void)testEllipsoidWrappingIsInvariantUnderTheOwningBodysPose {
    WrapObjectSpec ellipsoid = MakeEllipsoid(0.035, 0.02, 0.02, "z");
    ellipsoid.pose.linear() =
        biomotion::bodyFixedXYZRotation(Eigen::Vector3d(0.21, -0.34, 0.12));
    ellipsoid.pose.translation() = Eigen::Vector3d(0.004, -0.002, 0.001);

    const Eigen::Vector3d points[2] = {Eigen::Vector3d(-0.05, 0.004, -0.05),
                                       Eigen::Vector3d(0.05, 0.004, 0.05)};
    const int indices[2] = {0, 1};
    biomotion::PathWrapSpec spec;
    spec.wrapObject = 0;

    const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
    const biomotion::WrappedPathResult reference = biomotion::solveWrappedPathLength(
        points, indices, 2, 2, &spec, 1, &ellipsoid, &identity, 1);
    XCTAssertEqual(reference.wrapPointCount, 2, @"the reference pose must wrap");

    Eigen::Isometry3d body = Eigen::Isometry3d::Identity();
    body.linear() = biomotion::bodyFixedXYZRotation(Eigen::Vector3d(-0.6, 0.45, 0.1));
    body.translation() = Eigen::Vector3d(-0.077, -0.099, 0.061);
    const Eigen::Vector3d moved[2] = {body * points[0], body * points[1]};
    const biomotion::WrappedPathResult transformed = biomotion::solveWrappedPathLength(
        moved, indices, 2, 2, &spec, 1, &ellipsoid, &body, 1);

    XCTAssertEqual(transformed.wrapPointCount, reference.wrapPointCount);
    XCTAssertEqualWithAccuracy(transformed.length, reference.length, 1e-9);
    XCTAssertEqual(transformed.signature, reference.signature,
                   @"the same problem must take the same branch");
}

@end
