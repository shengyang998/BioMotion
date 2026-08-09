// The four gastrocnemius muscles wrap around TWO cylinders, and the reference
// column every other wrap test is measured against is NOT A PATH LENGTH for
// them. This file is the gate for that class.
//
// # What was measured, and why this file exists
//
// `CylinderWrapValidationTests` gates the moment arm at |ours - OpenSim| < 20 mm
// (W1) and the path length at < 20 mm (C5), over the 60 poses the committed grid
// samples. The multi-wrap class passes that. It passes because of where the grid
// lands: sweep `knee_angle_r` finely instead and `gaslat140_r` reads **41.26 mm**
// at 26.01866 deg — twice the bar — while the port's own answer at that pose is
// within 0.5 mm of a length that a path actually has.
//
// The cause is the reference. `GeometryPath::calcLengthAfterPathComputation` sums
//
//     straight segments measured BETWEEN the wrap points OpenSim reports
//   + the spiral length OpenSim STORED beside them
//
// and for a two-cylinder path those two halves describe different paths.
// Measured on `gasmed_r` at knee 0 deg: the stored spiral is 0.038054 m while the
// straight-line CHORD between the two tangent points it is supposed to connect is
// 0.045350 m. No curve joining two points is shorter than the line between them,
// so the reported total is at least 7.30 mm below the length of any path through
// its own points. The mechanism is `WrapCylinder::_adjust_tangent_point`, which
// runs only when a muscle carries more than one `PathWrap`: OpenSim moves the
// tangent points and does not recompute the arc between them. That is also why
// the arm error is SYSTEMATIC — the adjustment shrinks smoothly as the knee
// flexes, so its derivative is a roughly constant ~10 mm offset across the whole
// running range rather than an isolated worst case.
//
// `opensim_multiwrap.txt` therefore carries a second length column,
// `reconciled`: the same total with each cylinder spiral's stored arc replaced by
// the shortest helix on that cylinder between the two tangent points OpenSim
// ITSELF reports. Every term then belongs to one path. Against that column the
// port agrees to 0.005 mm of length and 0.03 mm of moment arm at the median.
//
// # What this file is NOT
//
// It is not a claim that the port's tangent points are the ones OpenSim would
// produce if OpenSim recomputed. It is the narrower, checkable statement that the
// number the port returns is the length of a path, that it is the same path
// OpenSim's own tangent points describe, and that the residual 41 mm is the
// reference's bookkeeping rather than ours.
//
// It also does NOT exercise nimble's forward kinematics: the fixture carries
// OpenSim's own path points in ground and its own wrap-object frames, and the
// port is run on those. That is deliberate — it separates "the wrap solve
// disagrees" from "the forward kinematics disagree", and the FK half is already
// covered end-to-end by `CylinderWrapValidationTests`' control class.

#import <XCTest/XCTest.h>

#import "MusclePathWrap.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct WrapRecord {
    biomotion::WrapObjectSpec object;
    biomotion::PathWrapSpec wrap;
};

struct MuscleRecord {
    std::string name;
    std::string driver;
    int originalPointCount = 0;
    std::vector<WrapRecord> wraps;
};

struct SampleRecord {
    int group = 0;
    int muscle = 0;
    int slot = 0;               // 0 = q-eps, 1 = q, 2 = q+eps
    std::string kind;           // "blind" or "adversarial"
    double qDegrees = 0;
    double reported = 0;
    double reconciled = 0;
    double slack = 0;           // min(stored arc - chord) over the reported spirals
    /// `GeometryPath::computeMomentArm` for the driving coordinate — OpenSim's
    /// OTHER moment-arm column. It asks `MomentArmSolver` for the generalized
    /// force a unit tension along the current path produces with the wrap points
    /// held fixed, so it reads the reported wrap POINTS and never touches
    /// `calcLengthAfterPathComputation`. Independent of the length defect AND of
    /// this generator's reconciliation arithmetic.
    double analytic = 0;
    int referenceWrapPoints = 0;
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Isometry3d> frames;
};

struct Fixture {
    double eps = 0;
    std::vector<MuscleRecord> muscles;
    std::vector<SampleRecord> samples;
    std::string error;
};

/// Nothing here throws or force-unwraps: a malformed file comes back as
/// `error`, which the tests assert on.
Fixture LoadFixture(NSURL* url) {
    Fixture fixture;
    if (url == nil) {
        fixture.error = "opensim_multiwrap.txt is not reachable from the test bundle";
        return fixture;
    }
    std::ifstream in(url.fileSystemRepresentation);
    if (!in) {
        fixture.error = "opensim_multiwrap.txt could not be opened";
        return fixture;
    }
    std::string line;
    bool sawFormat = false;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream stream(line);
        std::string tag;
        stream >> tag;
        if (tag == "format") {
            std::string id;
            stream >> id;
            sawFormat = (id == "biomotion-osim-multiwrap-v1");
            if (!sawFormat) { fixture.error = "unexpected format id " + id; return fixture; }
        } else if (tag == "eps") {
            stream >> fixture.eps;
        } else if (tag == "muscles") {
            int count = 0;
            stream >> count;
            if (count <= 0) { fixture.error = "no muscles"; return fixture; }
            fixture.muscles.resize((size_t)count);
        } else if (tag == "muscle") {
            int index = 0, points = 0, wraps = 0;
            std::string name, driver;
            stream >> index >> name >> driver >> points >> wraps;
            if (index < 0 || index >= (int)fixture.muscles.size()) {
                fixture.error = "muscle index out of range"; return fixture;
            }
            MuscleRecord& record = fixture.muscles[(size_t)index];
            record.name = name;
            record.driver = driver;
            record.originalPointCount = points;
            record.wraps.resize((size_t)wraps);
        } else if (tag == "wrap") {
            int muscle = 0, slot = 0, kind = 0, method = 0, start = 0, end = 0;
            std::string name, quadrant;
            double radius = 0, length = 0, d0 = 0, d1 = 0, d2 = 0;
            stream >> muscle >> slot >> name >> kind >> radius >> length
                   >> d0 >> d1 >> d2 >> quadrant >> method >> start >> end;
            if (muscle < 0 || muscle >= (int)fixture.muscles.size()) {
                fixture.error = "wrap names an unknown muscle"; return fixture;
            }
            MuscleRecord& record = fixture.muscles[(size_t)muscle];
            if (slot < 0 || slot >= (int)record.wraps.size()) {
                fixture.error = "wrap slot out of range"; return fixture;
            }
            WrapRecord& wrap = record.wraps[(size_t)slot];
            wrap.object.name = name;
            wrap.object.kind = kind == 0 ? biomotion::WrapKind::Cylinder
                             : kind == 1 ? biomotion::WrapKind::Ellipsoid
                                         : biomotion::WrapKind::Unsupported;
            wrap.object.radius = radius;
            wrap.object.length = length;
            wrap.object.dimensions = Eigen::Vector3d(d0, d1, d2);
            wrap.object.pose = Eigen::Isometry3d::Identity();
            if (!biomotion::decodeWrapQuadrant(quadrant, wrap.object.wrapAxis,
                                               wrap.object.wrapSign)) {
                fixture.error = "unparseable quadrant " + quadrant; return fixture;
            }
            wrap.wrap.wrapObject = slot;
            wrap.wrap.startPoint = start;
            wrap.wrap.endPoint = end;
            wrap.wrap.method = method == 0 ? biomotion::PathWrapMethod::Hybrid
                                           : biomotion::PathWrapMethod::Unsupported;
        } else if (tag == "sample") {
            SampleRecord sample;
            int pointCount = 0;
            stream >> sample.group >> sample.muscle >> sample.slot >> sample.kind
                   >> sample.qDegrees >> sample.reported >> sample.reconciled
                   >> sample.slack >> sample.analytic >> sample.referenceWrapPoints
                   >> pointCount;
            if (sample.muscle < 0 || sample.muscle >= (int)fixture.muscles.size()) {
                fixture.error = "sample names an unknown muscle"; return fixture;
            }
            const size_t wrapCount = fixture.muscles[(size_t)sample.muscle].wraps.size();
            for (int i = 0; i < pointCount; i++) {
                if (!std::getline(in, line)) { fixture.error = "truncated points"; return fixture; }
                std::istringstream point(line);
                std::string pointTag;
                double x = 0, y = 0, z = 0;
                point >> pointTag >> x >> y >> z;
                if (pointTag != "point") { fixture.error = "expected point, got " + pointTag; return fixture; }
                sample.points.push_back(Eigen::Vector3d(x, y, z));
            }
            sample.frames.resize(wrapCount, Eigen::Isometry3d::Identity());
            for (size_t i = 0; i < wrapCount; i++) {
                if (!std::getline(in, line)) { fixture.error = "truncated frames"; return fixture; }
                std::istringstream frame(line);
                std::string frameTag;
                int slot = 0;
                frame >> frameTag >> slot;
                if (frameTag != "frame" || slot < 0 || slot >= (int)wrapCount) {
                    fixture.error = "malformed frame line"; return fixture;
                }
                Eigen::Matrix3d rotation;
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++) frame >> rotation(r, c);
                double tx = 0, ty = 0, tz = 0;
                frame >> tx >> ty >> tz;
                Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
                transform.linear() = rotation;
                transform.translation() = Eigen::Vector3d(tx, ty, tz);
                sample.frames[(size_t)slot] = transform;
            }
            fixture.samples.push_back(std::move(sample));
        }
    }
    if (!sawFormat) fixture.error = "no format line";
    if (fixture.samples.empty() && fixture.error.empty()) fixture.error = "no samples";
    return fixture;
}

/// The shipping solver, run on OpenSim's own geometry for one sample.
biomotion::WrappedPathResult Solve(const Fixture& fixture, const SampleRecord& sample) {
    const MuscleRecord& muscle = fixture.muscles[(size_t)sample.muscle];
    std::vector<biomotion::WrapObjectSpec> objects;
    std::vector<biomotion::PathWrapSpec> wraps;
    objects.reserve(muscle.wraps.size());
    wraps.reserve(muscle.wraps.size());
    for (const WrapRecord& record : muscle.wraps) {
        objects.push_back(record.object);
        wraps.push_back(record.wrap);
    }
    std::vector<int> originalIndex((size_t)sample.points.size());
    for (size_t i = 0; i < originalIndex.size(); i++) originalIndex[i] = (int)i;
    return biomotion::solveWrappedPathLength(
        sample.points.data(), originalIndex.data(), (int)sample.points.size(),
        muscle.originalPointCount, wraps.data(), (int)wraps.size(),
        objects.data(), sample.frames.data(), (int)objects.size());
}

double Percentile(std::vector<double> values, double fraction) {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    size_t index = (size_t)(fraction * (double)(values.size() - 1));
    return values[std::min(index, values.size() - 1)];
}

NSString* Describe(std::vector<double> values, const char* label) {
    if (values.empty()) return [NSString stringWithFormat:@"%s: empty", label];
    std::sort(values.begin(), values.end());
    return [NSString stringWithFormat:@"%s n=%zu median %.6f mm p90 %.6f mm max %.6f mm",
            label, values.size(), Percentile(values, 0.5) * 1000,
            Percentile(values, 0.9) * 1000, values.back() * 1000];
}

}  // namespace

@interface MultiWrapReferenceTests : XCTestCase
@end

@implementation MultiWrapReferenceTests {
    Fixture _fixture;
}

- (void)setUp {
    [super setUp];
    if (!_fixture.samples.empty() || !_fixture.error.empty()) return;
    NSURL* url = [[NSBundle bundleForClass:[self class]] URLForResource:@"opensim_multiwrap"
                                                          withExtension:@"txt"
                                                           subdirectory:@"Fixtures"];
    _fixture = LoadFixture(url);
}

/// The fixture is what it says it is. A gate on a file that silently loaded
/// three rows is not a gate.
- (void)testTheFixtureCoversEveryMultiWrapMuscleAndBothGrids {
    XCTAssertEqual(_fixture.error.size(), 0u,
                   @"fixture failed to load: %s", _fixture.error.c_str());
    XCTAssertEqual(_fixture.muscles.size(), 8u,
                   @"FullBody.osim has 8 muscles with more than one PathWrap");
    XCTAssertEqualWithAccuracy(_fixture.eps, 1e-4, 1e-12,
                               @"a different eps is not comparable with the fd fixture");
    int blind = 0, adversarial = 0, engaged = 0;
    for (const SampleRecord& sample : _fixture.samples) {
        if (sample.slot != 1) continue;
        if (sample.kind == "blind") blind++;
        if (sample.kind == "adversarial") adversarial++;
        if (sample.referenceWrapPoints > 0) engaged++;
    }
    NSLog(@"MULTIWRAP groups blind=%d adversarial=%d engaged=%d samples=%zu",
          blind, adversarial, engaged, _fixture.samples.size());
    XCTAssertGreaterThanOrEqual(blind, 8 * 51,
                                @"the blind grid is the one chosen before any result");
    XCTAssertGreaterThan(adversarial, 0, @"no adversarial rows were kept");
    XCTAssertGreaterThan(engaged, blind / 2,
                         @"the reference barely engages a wrap here, so this suite "
                         @"would pass with no solver at all");
}

/// **M4 — the reference's multi-wrap length is not a path length.**
///
/// `slack` is `min(stored arc - chord)` over the spiral segments OpenSim
/// reports. A curve joining two points cannot be shorter than the straight line
/// between them, so a negative value is impossible geometry in the REFERENCE,
/// measured from the reference's own numbers.
///
/// If a future OpenSim fixes `_adjust_tangent_point`'s bookkeeping this test
/// fails, and that is the signal to delete the `reconciled` column and gate the
/// class on `reported` again.
- (void)testTheReferencesOwnSpiralIsShorterThanTheChordItSpans {
    XCTAssertEqual(_fixture.error.size(), 0u, @"fixture failed to load");
    double worst = 0;
    std::string worstMuscle;
    double worstQ = 0;
    int impossible = 0;
    for (const SampleRecord& sample : _fixture.samples) {
        if (sample.slot != 1 || sample.referenceWrapPoints < 4) continue;
        if (sample.slack < 0) impossible++;
        if (sample.slack < worst) {
            worst = sample.slack;
            worstMuscle = _fixture.muscles[(size_t)sample.muscle].name;
            worstQ = sample.qDegrees;
        }
    }
    NSLog(@"MULTIWRAP reference slack: %d rows with stored arc < chord, worst "
          @"%.4f mm on %s at %.5f deg", impossible, worst * 1000,
          worstMuscle.c_str(), worstQ);
    XCTAssertGreaterThan(impossible, 0,
                         @"the reference is self-consistent here, so the reconciled "
                         @"column is unnecessary and this whole file should go");
    XCTAssertLessThan(worst, -0.005,
                      @"the defect this file exists for is worth millimetres, not "
                      @"rounding; measured -7.30 mm on gasmed at knee 0 deg");
}

/// **M1 — path length, against a column that is a path length.**
///
/// Bar carried over from C5's pre-registered single-wrap bar (0.005 m). Nothing
/// new was chosen: the multi-wrap class is being held to the bar the class the
/// reference CAN gate is already held to.
- (void)testMultiWrapPathLengthsMatchTheReconciledReference {
    XCTAssertEqual(_fixture.error.size(), 0u, @"fixture failed to load");
    std::vector<double> reconciled, reported;
    int engagementDisagreements = 0;
    for (const SampleRecord& sample : _fixture.samples) {
        if (sample.slot != 1) continue;
        const biomotion::WrappedPathResult result = Solve(_fixture, sample);
        XCTAssertFalse(result.refused, @"the solver refused a real muscle path");
        XCTAssertTrue(std::isfinite(result.length), @"non-finite length");
        reconciled.push_back(std::fabs(result.length - sample.reconciled));
        reported.push_back(std::fabs(result.length - sample.reported));
        if (result.wrapPointCount != sample.referenceWrapPoints) engagementDisagreements++;
    }
    NSLog(@"%@", Describe(reconciled, "MULTIWRAP length ours vs RECONCILED"));
    NSLog(@"%@", Describe(reported, "MULTIWRAP length ours vs reported"));
    NSLog(@"MULTIWRAP engagement disagreements: %d of %zu",
          engagementDisagreements, reconciled.size());
    XCTAssertGreaterThan(reconciled.size(), 400u, @"too few rows to mean anything");
    XCTAssertLessThan(*std::max_element(reconciled.begin(), reconciled.end()), 0.005,
                      @"M1: multi-wrap path length against the reconciled column, at "
                      @"C5's single-wrap bar");
    XCTAssertEqual(engagementDisagreements, 0,
                   @"a solver that agrees on length while disagreeing about which "
                   @"wraps engaged agrees by accident");
}

/// **M2 / M3 — moment arm, against the same column, at C1 and C2's bars.**
///
/// `-dL/dq` by the same centred difference at the same eps the fd fixture uses,
/// on the reconciled length. The `reported` column's number is printed beside it
/// because that is the one `CylinderWrapValidationTests` gates, and the gap
/// between the two is the finding.
- (void)testMultiWrapMomentArmsMatchTheReconciledReference {
    XCTAssertEqual(_fixture.error.size(), 0u, @"fixture failed to load");
    std::vector<double> reconciledError, reportedError;
    double worstReported = 0, worstReportedQ = 0;
    std::string worstReportedMuscle;
    for (size_t i = 0; i + 2 < _fixture.samples.size(); i++) {
        const SampleRecord& minus = _fixture.samples[i];
        const SampleRecord& base = _fixture.samples[i + 1];
        const SampleRecord& plus = _fixture.samples[i + 2];
        if (minus.slot != 0 || base.slot != 1 || plus.slot != 2) continue;
        if (minus.group != base.group || base.group != plus.group) continue;
        const double ours = -(Solve(_fixture, plus).length - Solve(_fixture, minus).length)
                          / (2 * _fixture.eps);
        const double reconciled = -(plus.reconciled - minus.reconciled) / (2 * _fixture.eps);
        const double reported = -(plus.reported - minus.reported) / (2 * _fixture.eps);
        reconciledError.push_back(std::fabs(ours - reconciled));
        reportedError.push_back(std::fabs(ours - reported));
        if (std::fabs(ours - reported) > worstReported) {
            worstReported = std::fabs(ours - reported);
            worstReportedQ = base.qDegrees;
            worstReportedMuscle = _fixture.muscles[(size_t)base.muscle].name;
        }
    }
    NSLog(@"%@", Describe(reconciledError, "MULTIWRAP arm ours vs RECONCILED"));
    NSLog(@"%@", Describe(reportedError, "MULTIWRAP arm ours vs reported"));
    NSLog(@"MULTIWRAP worst arm gap against the REPORTED column: %.4f mm on %s at "
          @"%.5f deg — this is the quantity CylinderWrapValidationTests gates at 20 mm",
          worstReported * 1000, worstReportedMuscle.c_str(), worstReportedQ);
    XCTAssertGreaterThan(reconciledError.size(), 400u, @"too few groups");
    XCTAssertLessThan(*std::max_element(reconciledError.begin(), reconciledError.end()),
                      0.005, @"M2: multi-wrap moment arm at C1's single-wrap bar");
    XCTAssertLessThan(Percentile(reconciledError, 0.99), 0.004,
                      @"M3: p99, at C2's single-wrap bar");
}

/// **M6 — the third witness, and the one that owes this file nothing.**
///
/// `reconciled` is this repo's arithmetic on OpenSim's numbers. OpenSim's
/// ANALYTIC moment arm is not: `GeometryPath::computeMomentArm` reads the
/// reported wrap POINTS and never calls `calcLengthAfterPathComputation`, so it
/// is blind to the length defect by construction. If the defect is the
/// reference's bookkeeping, this column must side with the port — and it does,
/// to **1.05 mm worst case over every multi-wrap muscle's full clamped range**,
/// while a central difference of the reported length is out by up to 41.26 mm.
///
/// A max-based assertion is legitimate HERE and not against the `reported`
/// column, because this one does not jump: it is 0.0005 mm at the median and its
/// worst row is a smooth pose, not a step.
///
/// The bar is C1's pre-registered single-wrap bar. Note this is a comparison of
/// two DIFFERENT definitions (envelope theorem vs `-dL/dq`), which CLAUDE.md
/// records as worth up to millimetres where a wrap solution is marginal — so a
/// failure here is a place to look, not automatically a port defect.
- (void)testMultiWrapMomentArmsMatchOpenSimsOwnAnalyticColumn {
    XCTAssertEqual(_fixture.error.size(), 0u, @"fixture failed to load");
    std::vector<double> errors;
    double worst = 0, worstQ = 0;
    std::string worstMuscle;
    for (size_t i = 0; i + 2 < _fixture.samples.size(); i++) {
        const SampleRecord& minus = _fixture.samples[i];
        const SampleRecord& base = _fixture.samples[i + 1];
        const SampleRecord& plus = _fixture.samples[i + 2];
        if (minus.slot != 0 || base.slot != 1 || plus.slot != 2) continue;
        if (minus.group != base.group || base.group != plus.group) continue;
        const double ours = -(Solve(_fixture, plus).length - Solve(_fixture, minus).length)
                          / (2 * _fixture.eps);
        const double error = std::fabs(ours - base.analytic);
        errors.push_back(error);
        if (error > worst) {
            worst = error;
            worstQ = base.qDegrees;
            worstMuscle = _fixture.muscles[(size_t)base.muscle].name;
        }
    }
    NSLog(@"%@", Describe(errors, "MULTIWRAP arm ours vs OpenSim ANALYTIC"));
    NSLog(@"MULTIWRAP worst analytic gap %.4f mm on %s at %.5f deg",
          worst * 1000, worstMuscle.c_str(), worstQ);
    XCTAssertGreaterThan(errors.size(), 400u, @"too few groups");
    XCTAssertLessThan(worst, 0.005,
                      @"M6: OpenSim's own analytic column sides with this port, at C1's bar");
    XCTAssertLessThan(Percentile(errors, 0.99), 0.004, @"M6: p99, at C2's bar");
}

/// **M5 — the adversarial rows, which is the answer to \"it passes by luck\".**
///
/// These q are not a grid. They are where the REFERENCE's own length function
/// jumps, found by scanning its second difference at a step finer than the
/// centred-difference stencil (`dump_multiwrap.py`, stage 1). At those poses the
/// quantity W1 gates blows through its 20 mm bar — and the port is still within
/// C1's bar of a length that a path has. Both halves are asserted, so this fails
/// if the excursion stops being the reference's OR if the port stops tracking the
/// reconciled column there.
- (void)testTheReferencesWorstExcursionsAreTheReferencesAndNotOurs {
    XCTAssertEqual(_fixture.error.size(), 0u, @"fixture failed to load");
    int overW1 = 0, groups = 0;
    double worstReported = 0, worstReconciled = 0, worstAnalytic = 0;
    double worstQ = 0;
    std::string worstMuscle;
    for (size_t i = 0; i + 2 < _fixture.samples.size(); i++) {
        const SampleRecord& minus = _fixture.samples[i];
        const SampleRecord& base = _fixture.samples[i + 1];
        const SampleRecord& plus = _fixture.samples[i + 2];
        if (minus.slot != 0 || base.slot != 1 || plus.slot != 2) continue;
        if (base.kind != "adversarial") continue;
        if (minus.group != base.group || base.group != plus.group) continue;
        groups++;
        const double ours = -(Solve(_fixture, plus).length - Solve(_fixture, minus).length)
                          / (2 * _fixture.eps);
        const double reported = -(plus.reported - minus.reported) / (2 * _fixture.eps);
        const double reconciled = -(plus.reconciled - minus.reconciled) / (2 * _fixture.eps);
        if (std::fabs(ours - reported) >= 0.020) overW1++;
        if (std::fabs(ours - reported) > worstReported) {
            worstReported = std::fabs(ours - reported);
            worstQ = base.qDegrees;
            worstMuscle = _fixture.muscles[(size_t)base.muscle].name;
        }
        worstReconciled = std::max(worstReconciled, std::fabs(ours - reconciled));
        worstAnalytic = std::max(worstAnalytic, std::fabs(ours - base.analytic));
    }
    NSLog(@"MULTIWRAP adversarial groups=%d overW1=%d worst_reported=%.4f mm on %s at "
          @"%.5f deg worst_reconciled=%.4f mm worst_analytic=%.4f mm",
          groups, overW1, worstReported * 1000, worstMuscle.c_str(), worstQ,
          worstReconciled * 1000, worstAnalytic * 1000);
    XCTAssertGreaterThan(groups, 0, @"no adversarial groups in the fixture");
    XCTAssertGreaterThan(overW1, 0,
                         @"the reference's length function no longer jumps here, so W1's "
                         @"multi-wrap component is safe and this test should be retired");
    XCTAssertLessThan(worstReconciled, 0.005,
                      @"M5: at the reference's own worst rows the port is still within "
                      @"C1's bar of the reconciled column, so the excursion is the "
                      @"reference's bookkeeping and not this port's geometry");
    XCTAssertLessThan(worstAnalytic, 0.005,
                      @"M5: and OpenSim's OWN analytic column agrees with the port at those "
                      @"same rows, which owes this repo's reconciliation nothing");
}

@end
