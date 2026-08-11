// TEMPORARY DIAGNOSTIC — root-causing `testRepeatedIKOnIdenticalMarkersIsStable`.
//
// This is an ObjC++ XCTestCase (not Swift) so it can call straight into
// nimblephysics and see what `NimbleBridge -solveIKWithMarkerPositions:` hides:
// the marker Jacobian's singular spectrum, the gradient at the pose IK returns,
// and — by re-calling `math::solveIK` with INSTRUMENTED copies of the same
// lambdas `Skeleton::fitMarkersToWorldPositions` uses — the full internal step
// trace, including every position mutation and every clamp.
//
// It does not modify any shipped code path. Delete once the drift is fixed.

#import <XCTest/XCTest.h>

#import "NimbleBridge.h"
#import "NimbleBridge+Internal.h"

#include <vector>
#include <string>
#include <cmath>

#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/CustomJoint.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/math/MathTypes.hpp"
#include "dart/math/IKSolver.hpp"
#include "dart/biomechanics/OpenSimParser.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace dart;

namespace {

struct MarkerRow { const char* name; double x, y, z; };

// The ORIGINAL fixture from NimbleBridgeTests (planar, z = 0).
static const MarkerRow kPlanar[] = {
    {"PELVIS",  0.00, 0.95, 0.00},
    {"LHJC",   -0.09, 0.92, 0.00},
    {"RHJC",    0.09, 0.92, 0.00},
    {"LKJC",   -0.09, 0.52, 0.00},
    {"RKJC",    0.09, 0.52, 0.00},
    {"LAJC",   -0.09, 0.10, 0.00},
    {"RAJC",    0.09, 0.10, 0.00},
    {"C7",      0.00, 1.40, 0.00},
    {"LSJC",   -0.18, 1.35, 0.00},
    {"RSJC",    0.18, 1.35, 0.00},
    {"LEJC",   -0.20, 1.07, 0.00},
    {"REJC",    0.20, 1.07, 0.00},
};
static const int kPlanarCount = sizeof(kPlanar) / sizeof(kPlanar[0]);

// Mirror of NimbleBridge.mm's markerReliabilityWeight (kept in sync by hand;
// only the entries this fixture touches matter).
static double weightFor(const std::string& n) {
    if (n == "PELVIS" || n == "SPINE_L" || n == "SPINE_M" || n == "C7" ||
        n == "NECK" || n == "HEAD") return 1.00;
    if (n == "LHJC" || n == "RHJC" || n == "LSJC" || n == "RSJC") return 0.85;
    if (n == "LKJC" || n == "RKJC" || n == "LEJC" || n == "REJC") return 0.70;
    if (n == "LAJC" || n == "RAJC" || n == "LWJC" || n == "RWJC") return 0.55;
    if (n == "LTOE" || n == "RTOE") return 0.40;
    return 1.00;
}

} // namespace

@interface IKSolverInternalsTests : XCTestCase
@end

@implementation IKSolverInternalsTests {
    NimbleBridge* _bridge;
    std::shared_ptr<dynamics::Skeleton> _skel;
    std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>> _markers;
    Eigen::VectorXs _targets;
    Eigen::VectorXs _weights;
    NSArray<NSNumber*>* _posArray;
    NSArray<NSString*>* _nameArray;
}

- (void)setUp {
    [super setUp];
    _bridge = [[NimbleBridge alloc] init];
    NSString* path = [[NSBundle bundleForClass:[self class]] pathForResource:@"Rajagopal2016"
                                                                     ofType:@"osim"];
    XCTAssertNotNil(path);
    XCTAssertTrue([_bridge loadModelFromPath:path]);
    _skel = [_bridge sharedSkeleton];
    XCTAssertTrue(_skel != nullptr);

    // Rebuild exactly what solveIKWithMarkerPositions builds internally.
    NSMutableArray<NSNumber*>* pos = [NSMutableArray array];
    NSMutableArray<NSString*>* names = [NSMutableArray array];
    _markers.clear();
    std::vector<double> w;
    std::vector<Eigen::Vector3s> tgt;
    for (int i = 0; i < kPlanarCount; i++) {
        std::string n(kPlanar[i].name);
        [names addObject:[NSString stringWithUTF8String:kPlanar[i].name]];
        [pos addObject:@(kPlanar[i].x)];
        [pos addObject:@(kPlanar[i].y)];
        [pos addObject:@(kPlanar[i].z)];
        dynamics::BodyNode* body = nullptr;
        // Resolve through the same virtual-marker table the bridge installed.
        // markerNames on the bridge is name-only, so re-derive via the skeleton.
        // (All 12 of these resolve on Rajagopal2016 — verified by the Swift
        //  diagnostic printing fixtureMarkersPlaced=12/12.)
        (void)body;
        w.push_back(weightFor(n));
        tgt.push_back(Eigen::Vector3s(kPlanar[i].x, kPlanar[i].y, kPlanar[i].z));
    }
    _posArray = pos;
    _nameArray = names;

    // Resolve each fixture marker EXACTLY the way NimbleBridge.mm does:
    // `_markers` is seeded from `osimFile.markersMap` (NimbleBridge.mm:304-307)
    // and the virtual joint-center table is only consulted for names the model
    // does not already define (NimbleBridge.mm:388-390). 11 of these 12 names
    // ARE model-native in Rajagopal2016 (C7's real offset is
    // (-0.085, 0.435, 0.0017) on torso, not the virtual (0, 0.38, 0)), so a
    // hand-built virtual-only table does NOT reproduce the bridge's objective.
    biomechanics::OpenSimFile osim = biomechanics::OpenSimParser::parseOsim(
        std::string([path UTF8String]), "", true);
    XCTAssertTrue(osim.skeleton != nullptr);

    struct VM { const char* marker; const char* body; double ox, oy, oz; };
    static const VM virtualFallback[] = {
        {"PELVIS", "pelvis",    0, 0,    0},
        {"LHJC",   "femur_l",   0, 0,    0},
        {"RHJC",   "femur_r",   0, 0,    0},
        {"LKJC",   "tibia_l",   0, 0,    0},
        {"RKJC",   "tibia_r",   0, 0,    0},
        {"LAJC",   "talus_l",   0, 0,    0},
        {"RAJC",   "talus_r",   0, 0,    0},
        {"C7",     "torso",     0, 0.38, 0},
        {"LSJC",   "humerus_l", 0, 0,    0},
        {"RSJC",   "humerus_r", 0, 0,    0},
        {"LEJC",   "ulna_l",    0, 0,    0},
        {"REJC",   "ulna_r",    0, 0,    0},
    };
    _targets.resize(kPlanarCount * 3);
    _weights.resize(kPlanarCount);
    for (int i = 0; i < kPlanarCount; i++) {
        std::string name(kPlanar[i].name);
        dynamics::BodyNode* b = nullptr;
        Eigen::Vector3s off = Eigen::Vector3s::Zero();
        auto it = osim.markersMap.find(name);
        if (it != osim.markersMap.end() && it->second.first != nullptr) {
            b = _skel->getBodyNode(it->second.first->getName());  // model-native
            off = it->second.second;
        } else {
            b = _skel->getBodyNode(std::string(virtualFallback[i].body));
            off = Eigen::Vector3s(virtualFallback[i].ox,
                                  virtualFallback[i].oy,
                                  virtualFallback[i].oz);
        }
        XCTAssertTrue(b != nullptr, @"unresolved marker %s", kPlanar[i].name);
        printf("IKINT|MARKER|%s|body=%s|off=%.6f,%.6f,%.6f|source=%s\n",
               kPlanar[i].name, b->getName().c_str(), off.x(), off.y(), off.z(),
               (it != osim.markersMap.end()) ? "model" : "virtual");
        _markers.push_back({b, off});
        _targets.segment<3>(i * 3) = tgt[i];
        _weights(i) = w[i];
    }
    // RMS-1 renormalisation, same as the bridge.
    double ss = _weights.squaredNorm();
    if (ss > 0) _weights *= std::sqrt((double)kPlanarCount / ss);
}

// Weighted residual + Jacobian, identical to the lambdas in
// Skeleton::fitMarkersToWorldPositions (scaleBodies == false).
- (void)evalDiff:(Eigen::VectorXs&)diff jac:(Eigen::MatrixXs&)J {
    diff = _skel->getMarkerWorldPositions(_markers) - _targets;
    for (int j = 0; j < _weights.size(); j++) diff.segment<3>(j * 3) *= _weights(j);
    J = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(_markers);
}

/// P1. Is the pose `solveIK` returns a stationary point of the least-squares
/// objective? If not, the next call's very first (unconditional) step at
/// IKSolver.cpp:321 i==0 will move q by exactly the leftover DLS step.
- (void)testP1StationarityAtReturnedPose {
    for (int solve = 0; solve < 6; solve++) {
        Eigen::VectorXs qBefore = _skel->getPositions();
        NimbleIKResult* r = [_bridge solveIKWithMarkerPositions:_posArray
                                                    markerNames:_nameArray];
        XCTAssertNotNil(r);
        Eigen::VectorXs qAfter = _skel->getPositions();

        Eigen::VectorXs diff(kPlanarCount * 3);
        Eigen::MatrixXs J(kPlanarCount * 3, _skel->getNumDofs());
        [self evalDiff:diff jac:J];

        double loss = diff.squaredNorm();
        Eigen::VectorXs grad = J.transpose() * diff;      // 0.5 * dLoss/dq
        // The exact DLS step refineIK would take next, with lr = 1.0.
        Eigen::MatrixXs toInvert =
            J.transpose() * J + 0.01 * Eigen::MatrixXs::Identity(J.cols(), J.cols());
        Eigen::VectorXs delta = toInvert.llt().solve(J.transpose() * diff);

        Eigen::JacobiSVD<Eigen::MatrixXs> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXs sv = svd.singularValues();

        double actualMove = (qAfter - qBefore).cwiseAbs().maxCoeff();

        printf("IKINT|P1|solve=%d|loss=%.12e|gradNorm=%.6e|dlsStepInf=%.6e|"
               "actualMoveInf=%.6e|sigmaMax=%.4e|sigmaMin=%.4e|cond=%.4e\n",
               solve, loss, grad.norm(), delta.cwiseAbs().maxCoeff(),
               actualMove, sv(0), sv(sv.size() - 1), sv(0) / sv(sv.size() - 1));
        fflush(stdout);
    }

    // Singular spectrum of J at the final pose, plus which DOF each small
    // singular direction lives on.
    Eigen::VectorXs diff(kPlanarCount * 3);
    Eigen::MatrixXs J(kPlanarCount * 3, _skel->getNumDofs());
    [self evalDiff:diff jac:J];
    Eigen::JacobiSVD<Eigen::MatrixXs> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXs sv = svd.singularValues();
    printf("IKINT|SPECTRUM|rows=%d|cols=%d|rank=%d\n",
           (int)J.rows(), (int)J.cols(), (int)svd.rank());
    for (int i = 0; i < sv.size(); i++) {
        // Dominant DOF of this right-singular vector.
        Eigen::VectorXs v = svd.matrixV().col(i);
        int arg = 0; v.cwiseAbs().maxCoeff(&arg);
        printf("IKINT|SV|i=%d|sigma=%.6e|dominantDof=%s|weight=%.3f\n",
               i, sv(i), _skel->getDof(arg)->getName().c_str(), std::abs(v(arg)));
    }
    fflush(stdout);
}

/// P2. Full internal trace of ONE solveIK, using instrumented copies of the
/// exact lambdas `Skeleton::fitMarkersToWorldPositions` passes to
/// `math::solveIK`. Records every position mutation, every clamp that actually
/// changed the vector, and the loss at every eval.
- (void)testP2InternalStepTrace {
    // Warm the solver the same way the red test does.
    for (int i = 0; i < 3; i++) {
        XCTAssertNotNil([_bridge solveIKWithMarkerPositions:_posArray markerNames:_nameArray]);
    }

    Eigen::VectorXs q0 = _skel->getPositions();

    int setCalls = 0, evalCalls = 0, clampMutations = 0;
    double maxClampJump = 0;

    auto setPosAndClamp = [&](Eigen::VectorXs pos, bool clamp) -> Eigen::VectorXs {
        setCalls++;
        _skel->setPositions(pos);
        if (clamp) {
            _skel->clampPositionsToLimits();
            Eigen::VectorXs clamped = _skel->getPositions();
            double jump = (clamped - pos).cwiseAbs().maxCoeff();
            if (jump > 0) {
                clampMutations++;
                if (jump > maxClampJump) maxClampJump = jump;
                printf("IKINT|CLAMP|setCall=%d|jumpInf=%.6e\n", setCalls, jump);
            }
            return clamped;
        }
        return pos;
    };

    auto eval = [&](Eigen::Ref<Eigen::VectorXs> diff, Eigen::Ref<Eigen::MatrixXs> jac) {
        evalCalls++;
        Eigen::VectorXs d = _skel->getMarkerWorldPositions(_markers) - _targets;
        for (int j = 0; j < _weights.size(); j++) d.segment<3>(j * 3) *= _weights(j);
        diff = d;
        jac = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(_markers);
        Eigen::VectorXs g = jac.transpose() * diff;
        printf("IKINT|EVAL|call=%d|loss=%.12e|gradNorm=%.6e|qInfFromStart=%.6e\n",
               evalCalls, (double)d.squaredNorm(), (double)g.norm(),
               (double)(_skel->getPositions() - q0).cwiseAbs().maxCoeff());
        fflush(stdout);
    };

    math::IKConfig config;
    // Same as the warm path in NimbleBridge.mm:691,701.
    config.setLossLowerBound((double)kPlanarCount * 0.02 * 0.02);
    config.setMaxRestarts(1);

    double err = math::solveIK(
        _skel->getPositions(),
        _skel->getPositionUpperLimits(),
        _skel->getPositionLowerLimits(),
        kPlanarCount * 3,
        setPosAndClamp,
        eval,
        [&](Eigen::Ref<Eigen::VectorXs> val) { val = _skel->getRandomPose(); },
        config);

    Eigen::VectorXs q1 = _skel->getPositions();
    printf("IKINT|P2|returnedErr=%.12e|setCalls=%d|evalCalls=%d|clampMutations=%d|"
           "maxClampJumpInf=%.6e|totalQMoveInf=%.6e\n",
           err, setCalls, evalCalls, clampMutations, maxClampJump,
           (double)(q1 - q0).cwiseAbs().maxCoeff());
    fflush(stdout);
}

/// P3. Where does the per-solve Δq live relative to J's singular spectrum?
/// Splits the move into the exact-null part, the small-σ part, and the
/// well-conditioned part.
- (void)testP3DriftProjectionOntoSingularSubspaces {
    for (int i = 0; i < 3; i++) {
        XCTAssertNotNil([_bridge solveIKWithMarkerPositions:_posArray markerNames:_nameArray]);
    }
    Eigen::VectorXs qA = _skel->getPositions();

    Eigen::VectorXs diff(kPlanarCount * 3);
    Eigen::MatrixXs J(kPlanarCount * 3, _skel->getNumDofs());
    [self evalDiff:diff jac:J];
    Eigen::JacobiSVD<Eigen::MatrixXs> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXs sv = svd.singularValues();

    XCTAssertNotNil([_bridge solveIKWithMarkerPositions:_posArray markerNames:_nameArray]);
    Eigen::VectorXs qB = _skel->getPositions();
    Eigen::VectorXs dq = qB - qA;

    // V is n x n with FullV; columns beyond sv.size() span the exact null space.
    Eigen::MatrixXs V = svd.matrixV();
    double total = dq.norm();
    double nullPart = 0, tinyPart = 0, midPart = 0, wellPart = 0;
    for (int i = 0; i < V.cols(); i++) {
        double c = V.col(i).dot(dq);
        double s = (i < sv.size()) ? sv(i) : 0.0;
        if (i >= sv.size() || s < 1e-10)      nullPart += c * c;
        else if (s < 0.01)                    tinyPart += c * c;   // sigma^2 << damping 0.01
        else if (s < 0.1)                     midPart  += c * c;   // sigma^2 ~ damping
        else                                  wellPart += c * c;
    }
    printf("IKINT|P3|dqNorm=%.6e|nullFrac=%.6f|sigmaLt0.01Frac=%.6f|"
           "sigma0.01to0.1Frac=%.6f|sigmaGt0.1Frac=%.6f\n",
           total,
           total > 0 ? nullPart / (total * total) : 0.0,
           total > 0 ? tinyPart / (total * total) : 0.0,
           total > 0 ? midPart / (total * total) : 0.0,
           total > 0 ? wellPart / (total * total) : 0.0);

    int arg = 0; dq.cwiseAbs().maxCoeff(&arg);
    printf("IKINT|P3|dominantDof=%s|dqInf=%.6e|lossBefore=%.12e\n",
           _skel->getDof(arg)->getName().c_str(), dq.cwiseAbs().maxCoeff(),
           (double)diff.squaredNorm());
    fflush(stdout);
}

// ---------------------------------------------------------------------------
// Generic driver: repeated warm solves against an ARBITRARY marker set and
// target vector, bypassing NimbleBridge so the marker set can be changed.
// Mirrors the bridge's warm path: maxRestarts = 1, lossLowerBound = n*tol^2.
// ---------------------------------------------------------------------------
- (void)runArm:(const char*)label
       markers:(const std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>>&)mk
       targets:(const Eigen::VectorXs&)tgt
       weights:(const Eigen::VectorXs&)w
         seedQ:(const Eigen::VectorXs&)seed
         count:(int)count {
    _skel->setPositions(seed);

    math::IKConfig config;
    config.setLossLowerBound((double)mk.size() * 0.02 * 0.02);
    config.setMaxRestarts(1);

    // Rank of J at the seed pose.
    Eigen::MatrixXs J0 = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(mk);
    Eigen::JacobiSVD<Eigen::MatrixXs> svd0(J0);
    printf("IKINT|ARM|%s|markers=%d|obs=%d|dofs=%d|rank=%d\n",
           label, (int)mk.size(), (int)J0.rows(), (int)J0.cols(), (int)svd0.rank());

    Eigen::VectorXs prev = _skel->getPositions();
    for (int n = 0; n < count; n++) {
        _skel->fitMarkersToWorldPositions(mk, tgt, w, false, config);
        Eigen::VectorXs q = _skel->getPositions();

        Eigen::VectorXs d = _skel->getMarkerWorldPositions(mk) - tgt;
        for (int j = 0; j < w.size(); j++) d.segment<3>(j * 3) *= w(j);
        double loss = d.squaredNorm();

        double move = (q - prev).cwiseAbs().maxCoeff();
        int arg = 0; (q - prev).cwiseAbs().maxCoeff(&arg);
        if (n < 6 || n == 10 || n == 20 || n == count - 1) {
            printf("IKINT|ARM|%s|n=%d|moveInf=%.6e|loss=%.12e|dominantDof=%s\n",
                   label, n, move, loss, _skel->getDof(arg)->getName().c_str());
            fflush(stdout);
        }
        prev = q;
    }
}

/// P4. THE CAUSAL TEST. Four arms that separate the three candidate causes of
/// the drift: rank deficiency (observability), marker residual (data
/// mismatch), and the residual/Jacobian weighting mismatch introduced by
/// NimbleBridge.mm:673-680.
///
///   A  full-rank  + zero residual   (3 offset markers per body, targets = FK(q*))
///   B  rank-21    + zero residual   (12 joint-center markers, targets = FK(q*))
///   C  rank-21    + real residual   (the red test's fixture)  [reference]
///   D  rank-21    + real residual + UNIFORM weights
///
/// Zero residual means the target is exactly achievable, so the only thing
/// left that can move q is the solver itself.
- (void)testP4CausalArms {
    const int nd = (int)_skel->getNumDofs();

    // A reference pose q*: something non-degenerate but plausible.
    Eigen::VectorXs qStar = Eigen::VectorXs::Zero(nd);
    for (int i = 0; i < nd; i++) qStar(i) = 0.05 * std::sin(0.7 * i + 0.3);
    _skel->setPositions(qStar);
    qStar = _skel->getPositions();

    // Seed = q* nudged, so the solver has something to do on the first call.
    Eigen::VectorXs seed = qStar;
    for (int i = 0; i < nd; i++) seed(i) += 0.02 * std::cos(1.1 * i);

    // ---- Arm A: dense, well-conditioned marker set (3 offsets per body).
    {
        std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>> mk;
        for (int b = 0; b < (int)_skel->getNumBodyNodes(); b++) {
            dynamics::BodyNode* bn = _skel->getBodyNode(b);
            mk.push_back({bn, Eigen::Vector3s(0.07, 0.02, 0.01)});
            mk.push_back({bn, Eigen::Vector3s(0.01, 0.06, 0.03)});
            mk.push_back({bn, Eigen::Vector3s(0.02, 0.01, 0.08)});
        }
        _skel->setPositions(qStar);
        Eigen::VectorXs tgt = _skel->getMarkerWorldPositions(mk);
        Eigen::VectorXs w = Eigen::VectorXs::Ones((int)mk.size());
        [self runArm:"A_fullrank_zeroresid" markers:mk targets:tgt weights:w
               seedQ:seed count:40];
    }

    // ---- Arm B: the 12 joint-center markers, but with an EXACTLY reachable
    // target (targets generated by FK from q*).
    {
        _skel->setPositions(qStar);
        Eigen::VectorXs tgt = _skel->getMarkerWorldPositions(_markers);
        [self runArm:"B_rank21_zeroresid" markers:_markers targets:tgt
             weights:_weights seedQ:seed count:40];
    }

    // ---- Arm C: the actual red-test fixture (rank-21 + real residual).
    [self runArm:"C_rank21_realresid" markers:_markers targets:_targets
         weights:_weights seedQ:seed count:40];

    // ---- Arm D: same as C but with uniform marker weights, isolating the
    // "diff is weighted, J is not" mismatch the bridge documents at
    // NimbleBridge.mm:668-672.
    {
        Eigen::VectorXs ones = Eigen::VectorXs::Ones((int)_markers.size());
        [self runArm:"D_rank21_realresid_uniformW" markers:_markers
             targets:_targets weights:ones seedQ:seed count:40];
    }
}

/// P5. Steady state (after 60 warm solves through the real bridge path):
/// where does the residual per-solve motion live, and is anything pinned at a
/// joint limit?
- (void)testP5SteadyStateCharacter {
    for (int i = 0; i < 60; i++) {
        XCTAssertNotNil([_bridge solveIKWithMarkerPositions:_posArray markerNames:_nameArray]);
    }
    Eigen::VectorXs qA = _skel->getPositions();
    Eigen::VectorXs diff(kPlanarCount * 3);
    Eigen::MatrixXs J(kPlanarCount * 3, _skel->getNumDofs());
    [self evalDiff:diff jac:J];
    double lossA = diff.squaredNorm();
    Eigen::JacobiSVD<Eigen::MatrixXs> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXs sv = svd.singularValues();
    Eigen::MatrixXs V = svd.matrixV();

    XCTAssertNotNil([_bridge solveIKWithMarkerPositions:_posArray markerNames:_nameArray]);
    Eigen::VectorXs qB = _skel->getPositions();
    [self evalDiff:diff jac:J];
    double lossB = diff.squaredNorm();
    Eigen::VectorXs dq = qB - qA;

    double total = dq.norm(), nullPart = 0, restPart = 0;
    for (int i = 0; i < V.cols(); i++) {
        double c = V.col(i).dot(dq);
        double s = (i < sv.size()) ? sv(i) : 0.0;
        if (s < 1e-10) nullPart += c * c; else restPart += c * c;
    }
    int arg = 0; dq.cwiseAbs().maxCoeff(&arg);
    printf("IKINT|P5|dqInf=%.6e|dqNorm=%.6e|nullFrac=%.6f|rowFrac=%.6f|"
           "dominantDof=%s|lossA=%.12e|lossB=%.12e|dLoss=%.6e\n",
           dq.cwiseAbs().maxCoeff(), total,
           total > 0 ? nullPart / (total * total) : 0.0,
           total > 0 ? restPart / (total * total) : 0.0,
           _skel->getDof(arg)->getName().c_str(), lossA, lossB, lossB - lossA);

    // Any DOF sitting on a limit (which is what makes clampPositionsToLimits
    // able to move q outside row(J))?
    for (int i = 0; i < (int)_skel->getNumDofs(); i++) {
        auto* dof = _skel->getDof(i);
        double q = dof->getPosition();
        double lo = dof->getPositionLowerLimit(), hi = dof->getPositionUpperLimit();
        if (q <= lo + 1e-6 || q >= hi - 1e-6) {
            printf("IKINT|P5|atLimit|dof=%s|q=%.6f|lo=%.6f|hi=%.6f\n",
                   dof->getName().c_str(), q, lo, hi);
        }
    }
    fflush(stdout);
}

/// P7. The same rank / drift measurement on the SHIPPED model
/// (FullBody.osim, the 163-DOF one) with the full ARKit virtual marker set —
/// this is the number the DOF-masking and pose-source experiments actually
/// have to move.
- (void)testP7FullBodyRankAndDrift {
    NimbleBridge* fb = [[NimbleBridge alloc] init];
    NSString* p = [[NSBundle bundleForClass:[self class]] pathForResource:@"FullBody"
                                                                   ofType:@"osim"];
    if (p == nil) { printf("IKINT|P7|SKIP|FullBody.osim not in test bundle\n"); return; }
    XCTAssertTrue([fb loadModelFromPath:p]);
    std::shared_ptr<dynamics::Skeleton> sk = [fb sharedSkeleton];

    // Full ARKit virtual-marker table from NimbleBridge.mm:335-378, in the same
    // order, with model-native markers taking precedence (NimbleBridge.mm:304).
    struct VM { const char* marker; const char* body; double ox, oy, oz; };
    static const VM table[] = {
        {"PELVIS","pelvis",0,0,0},   {"LHJC","femur_l",0,0,0}, {"RHJC","femur_r",0,0,0},
        {"LKJC","tibia_l",0,0,0},    {"RKJC","tibia_r",0,0,0}, {"LAJC","talus_l",0,0,0},
        {"RAJC","talus_r",0,0,0},    {"LTOE","toes_l",0,0,0},  {"RTOE","toes_r",0,0,0},
        {"SPINE_L","lumbar3",0,0,0}, {"SPINE_M","thoracic7",0,0,0}, {"C7","thoracic1",0,0,0},
        {"NECK","head_neck",0,0,0},  {"HEAD","head_neck",0,0.15,0},
        {"LSJC","humerus_l",0,0,0},  {"RSJC","humerus_r",0,0,0},
        {"LEJC","ulna_l",0,0,0},     {"REJC","ulna_r",0,0,0},
        {"LWJC","hand_l",0,0,0},     {"RWJC","hand_r",0,0,0},
    };
    biomechanics::OpenSimFile osim = biomechanics::OpenSimParser::parseOsim(
        std::string([p UTF8String]), "", true);

    // A plausible ARKit standing pose for every marker the model can host.
    struct T { const char* n; double x, y, z; };
    static const T stand[] = {
        {"PELVIS",0,0.95,0},   {"LHJC",-0.09,0.92,0},  {"RHJC",0.09,0.92,0},
        {"LKJC",-0.09,0.52,0}, {"RKJC",0.09,0.52,0},   {"LAJC",-0.09,0.10,0},
        {"RAJC",0.09,0.10,0},  {"LTOE",-0.09,0.03,0.16},{"RTOE",0.09,0.03,0.16},
        {"SPINE_L",0,1.05,0},  {"SPINE_M",0,1.22,0},   {"C7",0,1.40,0},
        {"NECK",0,1.46,0},     {"HEAD",0,1.60,0},
        {"LSJC",-0.18,1.35,0}, {"RSJC",0.18,1.35,0},
        {"LEJC",-0.20,1.07,0}, {"REJC",0.20,1.07,0},
        {"LWJC",-0.21,0.80,0}, {"RWJC",0.21,0.80,0},
    };

    std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>> mk;
    NSMutableArray<NSNumber*>* posA = [NSMutableArray array];
    NSMutableArray<NSString*>* nameA = [NSMutableArray array];
    for (const auto& vm : table) {
        std::string name(vm.marker);
        dynamics::BodyNode* b = nullptr; Eigen::Vector3s off = Eigen::Vector3s::Zero();
        auto it = osim.markersMap.find(name);
        if (it != osim.markersMap.end() && it->second.first != nullptr) {
            b = sk->getBodyNode(it->second.first->getName());
            off = it->second.second;
        } else {
            b = sk->getBodyNode(std::string(vm.body));
            off = Eigen::Vector3s(vm.ox, vm.oy, vm.oz);
        }
        if (b == nullptr) continue;              // body absent → bridge skips too
        bool dup = false;
        for (NSString* s in nameA) if ([s isEqualToString:[NSString stringWithUTF8String:vm.marker]]) dup = true;
        if (dup) continue;
        mk.push_back({b, off});
        for (const auto& t : stand) if (name == t.n) {
            [posA addObject:@(t.x)]; [posA addObject:@(t.y)]; [posA addObject:@(t.z)];
        }
        [nameA addObject:[NSString stringWithUTF8String:vm.marker]];
    }

    Eigen::MatrixXs J = sk->getMarkerWorldPositionsJacobianWrtJointPositions(mk);
    Eigen::JacobiSVD<Eigen::MatrixXs> svd(J);
    Eigen::VectorXs sv = svd.singularValues();
    int nWell = 0, nMarginal = 0, nNull = 0;
    for (int i = 0; i < sv.size(); i++) {
        if (sv(i) > 0.1) nWell++; else if (sv(i) > 1e-10) nMarginal++; else nNull++;
    }
    nNull += (int)J.cols() - (int)sv.size();   // cols beyond thin-SVD are null too
    printf("IKINT|P7|model=FullBody|dofs=%d|markers=%d|obs=%d|rank=%d|"
           "sigmaGt0.1=%d|marginal=%d|exactNull=%d\n",
           (int)J.cols(), (int)mk.size(), (int)J.rows(), (int)svd.rank(),
           nWell, nMarginal, nNull);

    // Per-solve drift through the real bridge path.
    Eigen::VectorXs prev;
    for (int n = 0; n < 8; n++) {
        NimbleIKResult* r = [fb solveIKWithMarkerPositions:posA markerNames:nameA];
        if (r == nil) { printf("IKINT|P7|solve %d returned nil\n", n); return; }
        Eigen::VectorXs q = sk->getPositions();
        if (n > 0) {
            Eigen::VectorXs dq = q - prev;
            int arg = 0; dq.cwiseAbs().maxCoeff(&arg);
            printf("IKINT|P7|n=%d|moveInf=%.6e|reportedErr=%.6e|dominantDof=%s\n",
                   n, dq.cwiseAbs().maxCoeff(), r.error, sk->getDof(arg)->getName().c_str());
        }
        prev = q;
        fflush(stdout);
    }
}

/// P8. Does nimble honour `<locked>true</locked>`? Rajagopal2016's 8 locked
/// coordinates show up at runtime with lo == hi == 0 even though their XML
/// <range> is non-degenerate — i.e. the lock may be honoured as a
/// zero-width position limit, not as a removed DOF. Check that on FullBody.
- (void)testP8ModelCoordinateRepresentation {
    for (NSString* model in @[@"Rajagopal2016", @"FullBody"]) {
        NimbleBridge* b = [[NimbleBridge alloc] init];
        NSString* p = [[NSBundle bundleForClass:[self class]] pathForResource:model
                                                                       ofType:@"osim"];
        if (p == nil) { printf("IKINT|P8|SKIP|%s\n", model.UTF8String); continue; }
        XCTAssertTrue([b loadModelFromPath:p]);
        std::shared_ptr<dynamics::Skeleton> sk = [b sharedSkeleton];
        int nd = (int)sk->getNumDofs(), pinned = 0;
        std::string names;
        for (int i = 0; i < nd; i++) {
            auto* d = sk->getDof(i);
            if (std::abs(d->getPositionUpperLimit() - d->getPositionLowerLimit()) < 1e-12) {
                pinned++;
                if (pinned <= 60) { names += d->getName(); names += ","; }
            }
        }
        printf("IKINT|P8|model=%s|dofs=%d|zeroWidthLimitDofs=%d\n",
               model.UTF8String, nd, pinned);
        printf("IKINT|P8|model=%s|pinnedNames=%s\n", model.UTF8String, names.c_str());
        fflush(stdout);
    }

    // The all-linear six-DOF parser fast path used to discard LinearFunction
    // slope/intercept and always instantiate an identity EulerFreeJoint. Force
    // pelvis_tx to 2*q: the exact CustomJoint fallback must move the pelvis by
    // 0.2 m when q is 0.1 m, rather than silently moving it by 0.1 m.
    NSString* sourcePath = [[NSBundle bundleForClass:[self class]]
        pathForResource:@"Rajagopal2016"
                 ofType:@"osim"];
    XCTAssertNotNil(sourcePath);
    if (sourcePath == nil) return;

    NSError* readError = nil;
    NSString* source = [NSString stringWithContentsOfFile:sourcePath
                                                  encoding:NSUTF8StringEncoding
                                                     error:&readError];
    XCTAssertNotNil(source, @"%@", readError);
    if (source == nil) return;

    NSRange anchor = [source rangeOfString:@"<coordinates>pelvis_tx</coordinates>"];
    XCTAssertNotEqual(anchor.location, NSNotFound);
    if (anchor.location == NSNotFound) return;
    NSRange search = NSMakeRange(NSMaxRange(anchor), source.length - NSMaxRange(anchor));
    NSRange coefficients = [source rangeOfString:@"<coefficients> 1 0</coefficients>"
                                          options:0
                                            range:search];
    XCTAssertNotEqual(coefficients.location, NSNotFound);
    if (coefficients.location == NSNotFound) return;

    NSMutableString* doubled = [source mutableCopy];
    [doubled replaceCharactersInRange:coefficients
                           withString:@"<coefficients> 2 0</coefficients>"];
    NSString* tempPath = [NSTemporaryDirectory() stringByAppendingPathComponent:
        [NSString stringWithFormat:@"nonidentity-linear-%@.osim",
                                   NSUUID.UUID.UUIDString]];
    NSError* writeError = nil;
    BOOL didWrite = [doubled writeToFile:tempPath
                              atomically:YES
                                encoding:NSUTF8StringEncoding
                                   error:&writeError];
    XCTAssertTrue(didWrite, @"%@", writeError);
    if (!didWrite) return;

    biomechanics::OpenSimFile parsed = biomechanics::OpenSimParser::parseOsim(
        std::string(tempPath.UTF8String), "", true);
    [[NSFileManager defaultManager] removeItemAtPath:tempPath error:nil];
    XCTAssertTrue(parsed.skeleton != nullptr);
    if (parsed.skeleton == nullptr) return;

    dynamics::DegreeOfFreedom* pelvisTX = parsed.skeleton->getDof("pelvis_tx");
    dynamics::BodyNode* pelvis = parsed.skeleton->getBodyNode("pelvis");
    XCTAssertTrue(pelvisTX != nullptr);
    XCTAssertTrue(pelvis != nullptr);
    if (pelvisTX == nullptr || pelvis == nullptr) return;

    Eigen::VectorXs q = Eigen::VectorXs::Zero(parsed.skeleton->getNumDofs());
    parsed.skeleton->setPositions(q);
    Eigen::Vector3s before = pelvis->getWorldTransform().translation();
    q(pelvisTX->getIndexInSkeleton()) = 0.1;
    parsed.skeleton->setPositions(q);
    Eigen::Vector3s delta = pelvis->getWorldTransform().translation() - before;
    XCTAssertEqualWithAccuracy(delta.x(), 0.2, 1e-12);
    XCTAssertEqualWithAccuracy(delta.y(), 0.0, 1e-12);
    XCTAssertEqualWithAccuracy(delta.z(), 0.0, 1e-12);

    // Baking a -1 rotational slope into the axis must negate the intercept as
    // well: a*(-q + b) == (-a)*(q - b). Preserve that exact pair in the
    // CustomJoint fallback instead of silently changing the constant term.
    NSRange tiltAnchor
        = [source rangeOfString:@"<coordinates>pelvis_tilt</coordinates>"];
    XCTAssertNotEqual(tiltAnchor.location, NSNotFound);
    if (tiltAnchor.location == NSNotFound) return;
    search = NSMakeRange(
        NSMaxRange(tiltAnchor), source.length - NSMaxRange(tiltAnchor));
    NSRange tiltCoefficients
        = [source rangeOfString:@"<coefficients> 1 0</coefficients>"
                        options:0
                          range:search];
    XCTAssertNotEqual(tiltCoefficients.location, NSNotFound);
    if (tiltCoefficients.location == NSNotFound) return;

    NSMutableString* reflected = [source mutableCopy];
    [reflected replaceCharactersInRange:tiltCoefficients
                             withString:@"<coefficients> -1 0.5</coefficients>"];
    NSString* reflectedPath = [NSTemporaryDirectory() stringByAppendingPathComponent:
        [NSString stringWithFormat:@"reflected-linear-%@.osim",
                                   NSUUID.UUID.UUIDString]];
    writeError = nil;
    didWrite = [reflected writeToFile:reflectedPath
                           atomically:YES
                             encoding:NSUTF8StringEncoding
                                error:&writeError];
    XCTAssertTrue(didWrite, @"%@", writeError);
    if (!didWrite) return;

    biomechanics::OpenSimFile reflectedFile
        = biomechanics::OpenSimParser::parseOsim(
            std::string(reflectedPath.UTF8String), "", true);
    [[NSFileManager defaultManager] removeItemAtPath:reflectedPath error:nil];
    XCTAssertTrue(reflectedFile.skeleton != nullptr);
    if (reflectedFile.skeleton == nullptr) return;

    auto* reflectedRoot = dynamic_cast<dynamics::CustomJoint<6>*>(
        reflectedFile.skeleton->getJoint("ground_pelvis"));
    XCTAssertTrue(reflectedRoot != nullptr);
    if (reflectedRoot == nullptr) return;
    XCTAssertEqualWithAccuracy(reflectedRoot->getFlipAxisMap()(0), -1.0, 1e-12);
    XCTAssertEqualWithAccuracy(
        reflectedRoot->getCustomFunction(0)->calcValue(0.0), -0.5, 1e-12);
    XCTAssertEqualWithAccuracy(
        reflectedRoot->getCustomFunction(0)->calcValue(0.1), -0.4, 1e-12);

    // A specialized EulerFreeJoint can only preserve the XML when each
    // TransformAxis is driven by the corresponding joint DOF. Point the X
    // translation at another valid coordinate: an exact CustomJoint must now
    // make pelvis_ty drive both X and Y, while pelvis_tx drives neither.
    NSMutableString* remapped = [source mutableCopy];
    [remapped replaceCharactersInRange:anchor
                             withString:@"<coordinates>pelvis_ty</coordinates>"];
    NSString* remappedPath = [NSTemporaryDirectory() stringByAppendingPathComponent:
        [NSString stringWithFormat:@"remapped-coordinate-%@.osim",
                                   NSUUID.UUID.UUIDString]];
    writeError = nil;
    didWrite = [remapped writeToFile:remappedPath
                          atomically:YES
                            encoding:NSUTF8StringEncoding
                               error:&writeError];
    XCTAssertTrue(didWrite, @"%@", writeError);
    if (!didWrite) return;

    biomechanics::OpenSimFile remappedFile
        = biomechanics::OpenSimParser::parseOsim(
            std::string(remappedPath.UTF8String), "", true);
    [[NSFileManager defaultManager] removeItemAtPath:remappedPath error:nil];
    XCTAssertTrue(remappedFile.skeleton != nullptr);
    if (remappedFile.skeleton == nullptr) return;

    dynamics::DegreeOfFreedom* remappedTX
        = remappedFile.skeleton->getDof("pelvis_tx");
    dynamics::DegreeOfFreedom* remappedTY
        = remappedFile.skeleton->getDof("pelvis_ty");
    dynamics::BodyNode* remappedPelvis
        = remappedFile.skeleton->getBodyNode("pelvis");
    XCTAssertTrue(remappedTX != nullptr);
    XCTAssertTrue(remappedTY != nullptr);
    XCTAssertTrue(remappedPelvis != nullptr);
    if (remappedTX == nullptr || remappedTY == nullptr
        || remappedPelvis == nullptr)
      return;

    q = Eigen::VectorXs::Zero(remappedFile.skeleton->getNumDofs());
    remappedFile.skeleton->setPositions(q);
    before = remappedPelvis->getWorldTransform().translation();
    q(remappedTX->getIndexInSkeleton()) = 0.1;
    remappedFile.skeleton->setPositions(q);
    delta = remappedPelvis->getWorldTransform().translation() - before;
    XCTAssertEqualWithAccuracy(delta.x(), 0.0, 1e-12);
    XCTAssertEqualWithAccuracy(delta.y(), 0.0, 1e-12);
    XCTAssertEqualWithAccuracy(delta.z(), 0.0, 1e-12);

    q.setZero();
    q(remappedTY->getIndexInSkeleton()) = 0.1;
    remappedFile.skeleton->setPositions(q);
    delta = remappedPelvis->getWorldTransform().translation() - before;
    XCTAssertEqualWithAccuracy(delta.x(), 0.1, 1e-12);
    XCTAssertEqualWithAccuracy(delta.y(), 0.1, 1e-12);
    XCTAssertEqualWithAccuracy(delta.z(), 0.0, 1e-12);
}

/// P6. At steady state, WHICH mutation moves q into the null space?
/// Traces one instrumented solve and attributes every position change to
/// either a solver step (`pos - lr*delta`) or a `clampPositionsToLimits` call.
- (void)testP6SteadyStateMutationAttribution {
    for (int i = 0; i < 60; i++) {
        XCTAssertNotNil([_bridge solveIKWithMarkerPositions:_posArray markerNames:_nameArray]);
    }
    Eigen::VectorXs q0 = _skel->getPositions();

    // Fixed null-space basis at the steady-state pose.
    Eigen::VectorXs d0(kPlanarCount * 3);
    Eigen::MatrixXs J0(kPlanarCount * 3, _skel->getNumDofs());
    [self evalDiff:d0 jac:J0];
    Eigen::JacobiSVD<Eigen::MatrixXs> svd(J0, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXs sv = svd.singularValues();
    Eigen::MatrixXs V = svd.matrixV();
    auto nullFrac = [&](const Eigen::VectorXs& v) -> double {
        double tot = v.squaredNorm(); if (tot <= 0) return 0;
        double np = 0;
        for (int i = 0; i < V.cols(); i++) {
            double s = (i < sv.size()) ? sv(i) : 0.0;
            if (s < 1e-10) { double c = V.col(i).dot(v); np += c * c; }
        }
        return np / tot;
    };

    int setCalls = 0;
    double stepNullSum = 0, clampNullSum = 0, clampInfMax = 0;
    int clampMutations = 0;
    Eigen::VectorXs prevQ = q0;

    auto setPosAndClamp = [&](Eigen::VectorXs pos, bool clamp) -> Eigen::VectorXs {
        setCalls++;
        // Attribution: the solver's own step is (pos - prevQ).
        Eigen::VectorXs stepPart = pos - prevQ;
        stepNullSum += nullFrac(stepPart) * stepPart.squaredNorm();
        _skel->setPositions(pos);
        Eigen::VectorXs out = pos;
        if (clamp) {
            _skel->clampPositionsToLimits();
            Eigen::VectorXs clamped = _skel->getPositions();
            Eigen::VectorXs clampPart = clamped - pos;
            double inf = clampPart.cwiseAbs().maxCoeff();
            if (inf > 0) {
                clampMutations++;
                clampInfMax = std::max(clampInfMax, inf);
                clampNullSum += nullFrac(clampPart) * clampPart.squaredNorm();
                int arg = 0; clampPart.cwiseAbs().maxCoeff(&arg);
                printf("IKINT|P6|clamp|setCall=%d|inf=%.6e|nullFrac=%.6f|dof=%s\n",
                       setCalls, inf, nullFrac(clampPart),
                       _skel->getDof(arg)->getName().c_str());
            }
            out = clamped;
        }
        prevQ = out;
        return out;
    };
    auto eval = [&](Eigen::Ref<Eigen::VectorXs> diff, Eigen::Ref<Eigen::MatrixXs> jac) {
        Eigen::VectorXs d = _skel->getMarkerWorldPositions(_markers) - _targets;
        for (int j = 0; j < _weights.size(); j++) d.segment<3>(j * 3) *= _weights(j);
        diff = d;
        jac = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(_markers);
    };

    math::IKConfig config;
    config.setLossLowerBound((double)kPlanarCount * 0.02 * 0.02);
    config.setMaxRestarts(1);
    math::solveIK(_skel->getPositions(),
                  _skel->getPositionUpperLimits(), _skel->getPositionLowerLimits(),
                  kPlanarCount * 3, setPosAndClamp, eval,
                  [&](Eigen::Ref<Eigen::VectorXs> val) { val = _skel->getRandomPose(); },
                  config);

    Eigen::VectorXs dq = _skel->getPositions() - q0;
    printf("IKINT|P6|SUMMARY|setCalls=%d|clampMutations=%d|clampInfMax=%.6e|"
           "netDqInf=%.6e|netNullFrac=%.6f|stepNullEnergy=%.6e|clampNullEnergy=%.6e\n",
           setCalls, clampMutations, clampInfMax,
           dq.cwiseAbs().maxCoeff(), nullFrac(dq), stepNullSum, clampNullSum);
    fflush(stdout);
}

@end
