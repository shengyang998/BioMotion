// Measures, rather than asserts, the claim that `shoulder_rot_{r,l}` (axial
// humeral rotation) is unobservable from the shipped 20-marker set.
//
// This is an ObjC++ XCTestCase so it can reach the marker Jacobian directly:
// `Skeleton::getMarkerWorldPositionsJacobianWrtJointPositions` is the exact
// matrix IK differentiates, so column j of it IS the sensitivity of every
// marker to coordinate j. Its norm is in metres of total marker displacement
// per radian, which is the only number that can settle "unobservable" without
// arguing from anatomy.
//
// The poses are the two the rest of the suite uses: the real Core ML dancer
// fixture and the hand-built upright standing pose. They differ in exactly the
// way that matters — the dancer's elbows are bent, the standing pose's are not.

#import <XCTest/XCTest.h>

#import "NimbleBridge.h"
#import "NimbleBridge+Internal.h"

#include <string>
#include <vector>
#include <cmath>

#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/math/MathTypes.hpp"

#include <Eigen/Dense>

using namespace dart;

namespace {

// The bridge's virtual-marker table for FullBody.osim, verbatim from
// NimbleBridge.mm's `virtualMarkers` (cyclist rows only — the Rajagopal2016
// fallbacks never resolve on this model). Kept here because `_markers` is a
// private ivar with no accessor.
struct VM { const char* marker; const char* body; double ox, oy, oz; };
static const VM kVirtualMarkers[] = {
    {"PELVIS","pelvis",0,0,0},
    {"LHJC","femur_l",0,0,0},     {"RHJC","femur_r",0,0,0},
    {"LKJC","tibia_l",0,0,0},     {"RKJC","tibia_r",0,0,0},
    {"LAJC","talus_l",0,0,0},     {"RAJC","talus_r",0,0,0},
    {"LTOE","toes_l",0,0,0},      {"RTOE","toes_r",0,0,0},
    {"SPINE_L","lumbar3",0,0,0},  {"SPINE_M","thoracic7",0,0,0},
    {"C7","thoracic1",0,0,0},     {"NECK","head_neck",0,0,0},
    {"HEAD","head_neck",0,0.15,0},
    {"LSJC","humerus_l",0,0,0},   {"RSJC","humerus_r",0,0,0},
    {"LEJC","ulna_l",0,0,0},      {"REJC","ulna_r",0,0,0},
    {"LWJC","hand_l",0,0,0},      {"RWJC","hand_r",0,0,0},
};

struct Target { const char* name; double x, y, z; };

// Upright standing, arms hanging with the elbows EXTENDED. Same literals as
// StaticEquilibriumBenchmarkTests / ShoulderRotMaskTests.
static const Target kStanding[] = {
    {"PELVIS", 0.0000, 0.9595,  0.0000},
    {"LHJC",  -0.0563, 0.8810, -0.0773}, {"RHJC",  -0.0563, 0.8810,  0.0773},
    {"LKJC",  -0.0527, 0.4750, -0.0770}, {"RKJC",  -0.0527, 0.4750,  0.0770},
    {"LAJC",  -0.0627, 0.0750, -0.0770}, {"RAJC",  -0.0627, 0.0750,  0.0770},
    {"LTOE",   0.0673, 0.0310, -0.0860}, {"RTOE",   0.0673, 0.0310,  0.0860},
    {"SPINE_L",-0.1019, 1.1042, 0.0000}, {"SPINE_M",-0.1595, 1.3444, 0.0000},
    {"C7",     -0.1173, 1.4812, 0.0000}, {"NECK",   -0.1130, 1.5021, 0.0000},
    {"HEAD",   -0.1130, 1.6521, 0.0000},
    {"LSJC",   -0.0997, 1.4310, -0.1706}, {"RSJC",  -0.0997, 1.4310,  0.1706},
    {"LEJC",   -0.0865, 1.1448, -0.1610}, {"REJC",  -0.0865, 1.1448,  0.1610},
    {"LWJC",   -0.1020, 0.8959, -0.2007}, {"RWJC",  -0.1020, 0.8959,  0.2007},
};

} // namespace

@interface ShoulderRotObservabilityTests : XCTestCase
@end

@implementation ShoulderRotObservabilityTests {
    NimbleBridge* _bridge;
    std::shared_ptr<dynamics::Skeleton> _skel;
    std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>> _mk;
    std::vector<std::string> _mkNames;
}

- (BOOL)loadFullBody {
    _bridge = [[NimbleBridge alloc] init];
    NSString* p = [[NSBundle bundleForClass:[self class]] pathForResource:@"FullBody" ofType:@"osim"];
    if (p == nil) p = [[NSBundle mainBundle] pathForResource:@"FullBody" ofType:@"osim"];
    if (p == nil) { XCTFail(@"Cannot find FullBody.osim"); return NO; }
    if (![_bridge loadModelFromPath:p]) { XCTFail(@"loadModel failed"); return NO; }
    _skel = [_bridge sharedSkeleton];
    if (_skel == nullptr) { XCTFail(@"sharedSkeleton null"); return NO; }

    _mk.clear();
    _mkNames.clear();
    for (const auto& vm : kVirtualMarkers) {
        dynamics::BodyNode* b = _skel->getBodyNode(std::string(vm.body));
        if (b == nullptr) continue;
        _mk.push_back({b, Eigen::Vector3s(vm.ox, vm.oy, vm.oz)});
        _mkNames.push_back(std::string(vm.marker));
    }
    return YES;
}

/// Solve `targets` through the production bridge path. The bridge leaves the
/// shared skeleton at the solved pose, which is where the Jacobian must be
/// evaluated.
- (void)solveTargets:(const Target*)t count:(int)n tag:(const char*)tag {
    NSMutableArray<NSNumber*>* pos = [NSMutableArray array];
    NSMutableArray<NSString*>* names = [NSMutableArray array];
    for (int i = 0; i < n; i++) {
        [names addObject:[NSString stringWithUTF8String:t[i].name]];
        [pos addObject:@(t[i].x)]; [pos addObject:@(t[i].y)]; [pos addObject:@(t[i].z)];
    }
    [_bridge resetSessionState];
    NimbleIKResult* r = [_bridge solveIKWithMarkerPositions:pos markerNames:names];
    XCTAssertNotNil(r, @"solveIK returned nil");
    // Warm solve so the reported pose is the fixed point, not the cold pass.
    r = [_bridge solveIKWithMarkerPositions:pos markerNames:names];
    XCTAssertNotNil(r);
    printf("SHROT-JAC [%s] ik_rms_cm=%.6f iters=%ld converged=%d\n",
           tag, r.markerRMSMeters * 100.0, (long)r.iterations, (int)r.converged);
}

/// Prints ‖J[:, j]‖ for the six shoulder coordinates plus reference columns, at
/// whatever pose the skeleton currently holds.
- (void)reportColumnNormsForTag:(const char*)tag {
    Eigen::MatrixXs J = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(_mk);
    const int nCols = (int)J.cols();

    // Reference scale: the largest column norm in the whole matrix, and a
    // coordinate nobody disputes is observable.
    double maxNorm = 0.0;
    std::string maxName;
    for (int j = 0; j < nCols; j++) {
        double c = J.col(j).norm();
        if (c > maxNorm) { maxNorm = c; maxName = _skel->getDof(j)->getName(); }
    }

    static const char* kInterest[] = {
        "elv_angle_r", "shoulder_elv_r", "shoulder_rot_r",
        "elv_angle_l", "shoulder_elv_l", "shoulder_rot_l",
        "elbow_flex_r", "elbow_flex_l", "knee_angle_r", "hip_flexion_r",
    };
    for (const char* want : kInterest) {
        for (int j = 0; j < nCols; j++) {
            if (_skel->getDof(j)->getName() != std::string(want)) continue;
            // Split the column into the arm markers on that side vs everything
            // else, because a shoulder coordinate can only move its own arm.
            double whole = J.col(j).norm();
            double armOnly = 0.0;
            for (size_t m = 0; m < _mkNames.size(); m++) {
                const std::string& mn = _mkNames[m];
                bool isArm = (mn == "LSJC" || mn == "RSJC" || mn == "LEJC" ||
                              mn == "REJC" || mn == "LWJC" || mn == "RWJC");
                if (!isArm) continue;
                armOnly += J.block(3 * (int)m, j, 3, 1).squaredNorm();
            }
            armOnly = std::sqrt(armOnly);
            printf("SHROT-JAC [%s] col_%s_norm_m_per_rad=%.9f arm_markers_only=%.9f "
                   "frac_of_max=%.9f\n",
                   tag, want, whole, armOnly, whole / (maxNorm > 0 ? maxNorm : 1.0));
            break;
        }
    }
    printf("SHROT-JAC [%s] largest_column=%s norm=%.9f n_cols=%d n_marker_rows=%d\n",
           tag, maxName.c_str(), maxNorm, nCols, (int)J.rows());
    fflush(stdout);
}

/// Per-marker displacement caused by 1 rad of `shoulder_rot_r`, which is the
/// same information as the column norm but says WHERE it lands.
- (void)reportPerMarkerSensitivityFor:(const char*)dofName tag:(const char*)tag {
    Eigen::MatrixXs J = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(_mk);
    int col = -1;
    for (int j = 0; j < (int)J.cols(); j++) {
        if (_skel->getDof(j)->getName() == std::string(dofName)) { col = j; break; }
    }
    if (col < 0) { printf("SHROT-JAC [%s] %s not found\n", tag, dofName); return; }
    for (size_t m = 0; m < _mkNames.size(); m++) {
        double v = J.block(3 * (int)m, col, 3, 1).norm();
        if (v < 1e-9) continue;   // only print the markers it can actually move
        printf("SHROT-JAC [%s] d%s/d%s_mm_per_rad=%.6f\n",
               tag, _mkNames[m].c_str(), dofName, v * 1000.0);
    }
    fflush(stdout);
}

// MARK: - Tests

/// Standing, arms down, elbows extended. The prediction under test: the
/// shoulder_rot columns are ~0 because the whole arm chain lies on the humeral
/// axis.
- (void)testShoulderRotObservabilityAtUprightStanding {
    if (![self loadFullBody]) return;
    [self solveTargets:kStanding count:(int)(sizeof(kStanding) / sizeof(kStanding[0]))
                   tag:"standing"];
    [self reportColumnNormsForTag:"standing"];
    [self reportPerMarkerSensitivityFor:"shoulder_rot_r" tag:"standing"];
    [self reportPerMarkerSensitivityFor:"shoulder_elv_r" tag:"standing"];
}

/// The same measurement at the model's NEUTRAL pose (all coordinates zero),
/// which is the configuration the mask pins to. Reported separately so the
/// standing number cannot be mistaken for a property of the coordinate itself.
- (void)testShoulderRotObservabilityAtNeutralPose {
    if (![self loadFullBody]) return;
    _skel->setPositions(Eigen::VectorXs::Zero(_skel->getNumDofs()));
    _skel->clampPositionsToLimits();
    [self reportColumnNormsForTag:"neutral"];
    [self reportPerMarkerSensitivityFor:"shoulder_rot_r" tag:"neutral"];
}

/// A deliberately BENT elbow. If the reasoning is right, this is where
/// shoulder_rot becomes visible, and it is the counter-example to calling the
/// coordinate unobservable without qualification.
- (void)testShoulderRotBecomesObservableWithABentElbow {
    if (![self loadFullBody]) return;
    _skel->setPositions(Eigen::VectorXs::Zero(_skel->getNumDofs()));
    _skel->clampPositionsToLimits();

    int elbowR = -1;
    for (int j = 0; j < (int)_skel->getNumDofs(); j++) {
        if (_skel->getDof(j)->getName() == std::string("elbow_flex_r")) { elbowR = j; break; }
    }
    XCTAssertGreaterThanOrEqual(elbowR, 0, "elbow_flex_r must exist");

    for (double flex : {0.0, 0.25, 0.5, 1.0, 1.5708}) {
        Eigen::VectorXs q = Eigen::VectorXs::Zero(_skel->getNumDofs());
        q(elbowR) = flex;
        _skel->setPositions(q);
        _skel->clampPositionsToLimits();
        Eigen::MatrixXs J = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(_mk);
        int col = -1;
        for (int j = 0; j < (int)J.cols(); j++) {
            if (_skel->getDof(j)->getName() == std::string("shoulder_rot_r")) { col = j; break; }
        }
        XCTAssertGreaterThanOrEqual(col, 0);
        printf("SHROT-JAC [bentelbow] elbow_flex_r=%.4f rad "
               "col_shoulder_rot_r_norm_m_per_rad=%.9f actual_elbow_q=%.6f\n",
               flex, J.col(col).norm(), (double)_skel->getPositions()(elbowR));
    }
    fflush(stdout);
}

@end
