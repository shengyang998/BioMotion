#import "NimbleBridge.h"
#import "NimbleBridge+Internal.h"

#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <set>

#include "dart/biomechanics/OpenSimParser.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/math/MathTypes.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/IKSolver.hpp"

using namespace dart;

// MARK: - NimbleIKResult

@interface NimbleIKResult ()
/// Populated by the solver after the fit, so the accuracy fields describe the
/// pose that was actually returned rather than an intermediate iterate.
- (void)setMarkerRMSMeters:(double)rms
      markerMaxErrorMeters:(double)maxErr
        markerErrorsMeters:(NSDictionary<NSString *, NSNumber *> *)errors
               markerCount:(NSInteger)count
                iterations:(NSInteger)iterations
                 converged:(BOOL)converged;
@end

@implementation NimbleIKResult {
    NSArray<NSNumber *> *_jointAngles;
    double _error;
    double _markerRMSMeters;
    double _markerMaxErrorMeters;
    NSDictionary<NSString *, NSNumber *> *_markerErrorsMeters;
    NSInteger _markerCount;
    NSInteger _iterations;
    BOOL _converged;
    NSInteger _numDOFs;
    NSArray<NSString *> *_dofNames;
}

- (instancetype)initWithAngles:(NSArray<NSNumber *> *)angles
                         error:(double)error
                       numDOFs:(NSInteger)numDOFs
                      dofNames:(NSArray<NSString *> *)dofNames {
    self = [super init];
    if (self) {
        _jointAngles = [angles copy];
        _error = error;
        _markerRMSMeters = NAN;
        _markerMaxErrorMeters = NAN;
        _markerErrorsMeters = @{};
        _markerCount = 0;
        _iterations = 0;
        _converged = NO;
        _numDOFs = numDOFs;
        _dofNames = [dofNames copy];
    }
    return self;
}

- (void)setMarkerRMSMeters:(double)rms
      markerMaxErrorMeters:(double)maxErr
        markerErrorsMeters:(NSDictionary<NSString *, NSNumber *> *)errors
               markerCount:(NSInteger)count
                iterations:(NSInteger)iterations
                 converged:(BOOL)converged {
    _markerRMSMeters = rms;
    _markerMaxErrorMeters = maxErr;
    _markerErrorsMeters = [errors copy];
    _markerCount = count;
    _iterations = iterations;
    _converged = converged;
}

- (NSArray<NSNumber *> *)jointAngles { return _jointAngles; }
- (double)error { return _error; }
- (double)markerRMSMeters { return _markerRMSMeters; }
- (double)markerMaxErrorMeters { return _markerMaxErrorMeters; }
- (NSDictionary<NSString *, NSNumber *> *)markerErrorsMeters { return _markerErrorsMeters; }
- (NSInteger)markerCount { return _markerCount; }
- (NSInteger)iterations { return _iterations; }
- (BOOL)converged { return _converged; }
- (NSInteger)numDOFs { return _numDOFs; }
- (NSArray<NSString *> *)dofNames { return _dofNames; }

@end

// MARK: - NimbleIDResult

@implementation NimbleIDResult {
    NSArray<NSNumber *> *_jointTorques;
    NSArray<NSNumber *> *_leftFootForce;
    NSArray<NSNumber *> *_rightFootForce;
    NSArray<NSNumber *> *_leftFootCoP;
    NSArray<NSNumber *> *_rightFootCoP;
    BOOL _leftFootInContact;
    BOOL _rightFootInContact;
    double _rootResidualNorm;
}

- (instancetype)initWithTorques:(NSArray<NSNumber *> *)torques {
    self = [super init];
    if (self) {
        _jointTorques = [torques copy];
        NSArray<NSNumber *> *zero3 = @[@0.0, @0.0, @0.0];
        _leftFootForce = zero3; _rightFootForce = zero3;
        _leftFootCoP = zero3; _rightFootCoP = zero3;
        _leftFootInContact = NO; _rightFootInContact = NO;
        _rootResidualNorm = 0;
    }
    return self;
}

- (instancetype)initWithTorques:(NSArray<NSNumber *> *)torques
                   leftForce:(NSArray<NSNumber *> *)leftForce
                   rightForce:(NSArray<NSNumber *> *)rightForce
                     leftCoP:(NSArray<NSNumber *> *)leftCoP
                    rightCoP:(NSArray<NSNumber *> *)rightCoP
               leftInContact:(BOOL)leftInContact
              rightInContact:(BOOL)rightInContact
            rootResidualNorm:(double)residual {
    self = [super init];
    if (self) {
        _jointTorques = [torques copy];
        _leftFootForce = [leftForce copy];
        _rightFootForce = [rightForce copy];
        _leftFootCoP = [leftCoP copy];
        _rightFootCoP = [rightCoP copy];
        _leftFootInContact = leftInContact;
        _rightFootInContact = rightInContact;
        _rootResidualNorm = residual;
    }
    return self;
}

- (NSArray<NSNumber *> *)jointTorques { return _jointTorques; }
- (NSArray<NSNumber *> *)leftFootForce { return _leftFootForce; }
- (NSArray<NSNumber *> *)rightFootForce { return _rightFootForce; }
- (NSArray<NSNumber *> *)leftFootCoP { return _leftFootCoP; }
- (NSArray<NSNumber *> *)rightFootCoP { return _rightFootCoP; }
- (BOOL)leftFootInContact { return _leftFootInContact; }
- (BOOL)rightFootInContact { return _rightFootInContact; }
- (double)rootResidualNorm { return _rootResidualNorm; }

@end

// MARK: - NimbleBridge

// --- Ground-height estimator tuning ---
//
// The estimator answers "where is the floor in the current ARKit world frame"
// from nothing but the subject's own feet. A running minimum cannot do this:
// it only ever descends, so one crouch, one landing spike or one bout of
// vertical ARKit drift permanently sinks the floor, both feet then read as
// airborne forever, and ID silently switches to the zero-external-force
// (flight) branch for the rest of the session — joint torques then absorb
// bodyweight as internal torque and every muscle activation is wrong by order
// of bodyweight. A low percentile over a bounded window of recent samples has
// the same "the floor is where the feet get lowest" intuition but forgets, so
// it can rise as well as fall.
//
// Window length trades outlier tolerance against how fast the estimate can
// follow a genuine floor change. 180 samples is ~3 s at ARKit's 60 Hz body
// tracking rate: long enough that a jump (< 1 s airborne) never dominates the
// window, short enough that walking onto a different floor level re-converges
// within a few seconds.
static const size_t kGroundWindowSamples = 180;
// Below this fill level the percentile has too few samples to reject outliers,
// so the estimate is published but flagged untrustworthy. ~0.5 s at 60 Hz.
static const size_t kGroundMinTrustedSamples = 30;
// 10th percentile: ignores up to 10% of the window sitting below the floor
// (i.e. ~0.3 s of glitched frames at full fill) without chasing the median,
// which sits above the floor whenever the subject spends time airborne.
static const double kGroundPercentile = 0.10;
// The calcn body origin sits slightly above the true contact point, so the
// ground plane is placed 1 cm below the observed heel percentile.
static const double kGroundContactOffsetMeters = 0.01;

// --- IK solver tuning ---
//
// Nimble's IKConfig defaults (5 random restarts, lossLowerBound 1e-10) are
// tuned for offline marker-set fitting where the data is clean enough to drive
// the loss to numerical zero. ARKit joint positions carry 1-3 cm of error, so
// that bound is unreachable, every restart always runs, and each restart calls
// getRandomPose() — discarding the previous frame's solution at 169 DOF and
// injecting joint-angle jitter that the Savitzky-Golay stage differentiates
// twice (gain ~1/dt^2) straight into the accelerations that drive ID.
//
// A warm-started solve landing above this per-marker residual is not a
// refinement of the previous pose at all (subject left and re-entered frame,
// recovery from a long occlusion), so it is redone from the neutral seed —
// once.
static const double kIKWarmStartRejectMeters = 0.15;

// --- The app-side solve that replaced math::solveIK / refineIK -------------
//
// WHY THE SOLVE IS REIMPLEMENTED HERE RATHER THAN CONFIGURED
//
// `math::refineIK` (IKSolver.cpp:291-493) cannot be made to converge from the
// outside, and the failure is structural, not a matter of tuning:
//
//  * It terminates on error-CHANGE (`errorChange > -convergenceThreshold`,
//    IKSolver.cpp:392) or on step count. It never tests stationarity, and it
//    never consults `lossLowerBound` — that bound is only read by the enclosing
//    restart loop. So it stops while `q` is still moving along the flat
//    directions of the objective, and the next call resets `lr` to 1.0 and
//    resumes from there. Measured on the dancer fixture: 0.11-0.22 rad of pose
//    movement per solve on IDENTICAL markers, not decaying over 8 solves.
//  * Its damping is a FIXED `leastSquaresDamping` (0.01). A fixed damping is
//    wrong in both directions at once: too large near the solution (it caps the
//    convergence rate at `1 - sigma^2/(sigma^2+lambda)` per step, which for the
//    weakly-observed coordinates of a 163-DOF model is glacial) and too small
//    far from it (steps overshoot, the learning rate halves, and the solver
//    falls into its gradient-transpose branch at `lr = 5e-5` and gives up).
//  * `getRandomPose()` draws from `Eigen::VectorXs::Random`, i.e. the
//    process-global `std::rand()`. Any solve that uses a random restart
//    therefore depends on how much unrelated work ran earlier in the process.
//    Measured: the same dancer markers after `resetSessionState` produced
//    ‖q‖ 4.54 and 6.86 — 1.69 rad apart, 5.5 cm vs 42 cm marker RMS.
//  * `fitMarkersToWorldPositions` seeds `math::solveIK` with the skeleton's
//    CURRENT positions (Skeleton.cpp:8001), and the skeleton is shared with
//    `MomentArmComputer` and the ID path. `resetSessionState` cleared the
//    bridge's warm-start pose but not the skeleton, so a "cold" solve still
//    inherited whatever pose the process had last written.
//
// The replacement is a Levenberg-Marquardt solve in two phases, run on the free
// coordinates only (all of them when no DOF mask is active):
//
//   phase A   min_q  1/2‖W(f(q) − x*)‖²  +  1/2·mu·‖q − q_seed‖²
//   phase B   min_q  1/2‖W(f(q) − x*)‖²          (started from phase A's answer)
//
// Phase A is the null-space damping E1 measured. It is what decides WHICH of
// the many poses that fit the markers equally well gets returned: coordinates
// the markers barely move stay at the seed instead of wandering.
//
// Phase B exists because phase A alone has no fixed point. Phase A's stationary
// point satisfies `Jᵀr = −mu·(q − q_seed)`, so re-entering with `q_seed` set to
// the previous answer leaves a residual gradient of order `mu·‖Δq‖` and the
// pose keeps creeping — a proximal-point iteration that only converges
// geometrically. Phase B drives `Jᵀr` to zero. Its steps lie in the row space
// of `J`, so it cannot undo phase A's choice in the unobservable directions; it
// only finishes the fit in the observable ones. The pose that comes back is
// therefore a stationary point of the marker fit, which means the next call on
// the same markers exits on its first convergence test having moved nothing.
// That is the fixed point, and it is a property of the termination test rather
// than of any tolerance.
//
// mu = 1e-3 m²/rad² is E1's value. Read it as an observability threshold: a
// coordinate is fit-driven when the markers move more than sqrt(mu) = 3.2 cm
// per radian of it (0.55 mm/degree) and seed-driven below that. Every real limb
// rotation is far above that line; the spine and rib coordinates ARKit and MHR
// cannot see are far below it.
static const double kIKSeedDamping = 1e-3;
// Convergence tests. Both are absolute and neither is a fit-quality bound.
//   * step: no free coordinate would move by 1e-9 rad (6e-8 degrees).
//   * gradient: the BOUND-PROJECTED gradient, ‖Jᵀr + mu(q−q_seed)‖_inf over the
//     coordinates that are free to move in the descent direction, in m²/rad.
//     Projecting matters: this model's extreme poses put several coordinates on
//     their joint limits, and an unprojected gradient there never goes to zero,
//     so the solver would keep proposing steps the clamp immediately undoes.
// The fixed-point property holds for ANY value of these, because it comes from
// the entry test passing on re-entry, not from the tolerance being tight.
//
// ⚠️ MEASURED, AND IT REFUTES THE OBVIOUS HYPOTHESIS. A moving subject costs
// ~78 solver iterations per frame, and the natural guess was that most of them
// are spent grinding the step from "physically settled" down to machine
// precision. That guess is wrong: relaxing this tolerance 100x, from 1e-9 to
// 1e-7 rad, changed the iteration count by 6% (77.8 -> 73.2) and changed the
// dancer's marker RMS by nothing at all (2.1224 cm either way, all printed
// digits identical). The iterations are real convergence work, not last-digit
// polishing, so the tolerance was left at the conservative value. Do not
// relax it expecting speed.
//
// For the record, the headroom that made 1e-7 look safe: the pose is
// differentiated TWICE by `SavitzkyGolayFilter` (gain ~1/dt² ≈ 3600 at 60 fps),
// so a residual step of `eps` rad manufactures at most `eps · 3600` rad/s²;
// `StaticHoldDetector`'s discarded-acceleration budget of 0.08 m/s² is ~0.16
// rad/s² over a 0.5 m segment, so anything under ~4e-6 rad is invisible. 1e-9
// is 4400x inside that. The bound is not what costs the iterations.
static const double kIKStepTolerance = 1e-9;
static const double kIKGradientTolerance = 1e-12;
// Levenberg-Marquardt trust-region schedule. lambda multiplies the identity
// added to JᵀJ: small lambda = Gauss-Newton (fast, may overshoot), large lambda
// = short gradient steps (always descends). It is adapted from the observed
// decrease, so it is not a tuned constant in the way nimble's fixed 0.01 is.
// Both ends are expressed as multiples of `max(diag(JᵀJ))` so they mean the
// same thing whatever the marker set and model scale are.
static const double kIKLambdaInitRel = 1e-4;
static const double kIKLambdaMaxRel  = 1e12;
static const double kIKLambdaDown = 0.25;
static const double kIKLambdaUp   = 8.0;
// Numerical floor on the damping, also relative to `max(diag(JᵀJ))`.
//
// This is a conditioning constant, not a regularisation choice: `JᵀJ` is
// 163x163 with rank at most 3 x 20 markers = 60, so 103 of its eigenvalues are
// exactly zero. Solving `(JᵀJ + lambda I) d = g` with lambda below the level at
// which double-precision round-off in `g` (relative ~1e-16) is amplified past
// the step tolerance produces pure noise in the null space: at lambda = 1e-9
// that noise is ~1e-7 rad per step, a hundred times the step tolerance, so the
// solver could never terminate and burned its whole budget ramping lambda up
// and back down. 1e-6 caps the condition number at 1e6 and the amplified
// round-off at ~1e-10 rad, below the step tolerance.
//
// It is added to the MATRIX only, never to the gradient, so it changes how far
// each step goes but not where the solver is allowed to stop: stationarity is
// still exactly `g = 0`.
static const double kIKConditionFloorRel = 1e-6;
// Iteration ceiling per phase. Only a cold solve ever approaches it; a warm
// solve on unchanged markers exits on the first test.
static const int kIKMaxIterations = 120;
// Flip to YES to get a per-solve trace on stderr while diagnosing. Compile-time
// constant so the shipping build carries no logging in the per-frame path.
static const BOOL kIKTraceSolve = NO;

// Static reliability prior over the ARKit virtual markers, used as IK marker
// weights. ARKit's positional error is not uniform across the body: the pelvis
// and spine are inferred from the largest, most-visible mass and are stable,
// while distal joints are the ones that get occluded, swapped L/R, or
// hallucinated. Weighting by this ordering stops a degraded wrist or toe from
// dragging the whole pose. Values are relative only — they are renormalised
// below so the reported IK loss keeps its previous scale.
static double markerReliabilityWeight(const std::string& name) {
    if (name == "PELVIS" || name == "SPINE_L" || name == "SPINE_M" ||
        name == "C7" || name == "NECK" || name == "HEAD") {
        return 1.00;  // trunk: most stable ARKit estimates
    }
    if (name == "LHJC" || name == "RHJC" || name == "LSJC" || name == "RSJC") {
        return 0.85;  // proximal limb
    }
    if (name == "LKJC" || name == "RKJC" || name == "LEJC" || name == "REJC") {
        return 0.70;  // mid limb
    }
    if (name == "LAJC" || name == "RAJC" || name == "LWJC" || name == "RWJC") {
        return 0.55;  // distal limb
    }
    if (name == "LTOE" || name == "RTOE") {
        return 0.40;  // terminal segments: noisiest, and least constraining
    }
    // Model-native surface markers (RASI, LASI, ...) aren't ARKit-derived and
    // have no reliability prior — treat them as the neutral case.
    return 1.00;
}

// Model-native scale references. `scaleModelWithHeight:` receives joint-centre
// distances, so its denominator must come from the same joint-centre body
// origins on the model that was actually loaded. A Rajagopal-era constant is
// not a reference for FullBody (and was not exact for Rajagopal either).
static double modelBodyOriginDistance(
    const std::shared_ptr<dynamics::Skeleton>& skeleton,
    const char *firstBody,
    const char *secondBody) {
    if (!skeleton) return NAN;
    dynamics::BodyNode *first = skeleton->getBodyNode(std::string(firstBody));
    dynamics::BodyNode *second = skeleton->getBodyNode(std::string(secondBody));
    if (first == nullptr || second == nullptr) return NAN;
    const double length = (
        first->getWorldTransform().translation()
        - second->getWorldTransform().translation()
    ).norm();
    return std::isfinite(length) && length > 0.0 ? length : NAN;
}

static double averageAvailableLengths(double left, double right) {
    const bool hasLeft = std::isfinite(left) && left > 0.0;
    const bool hasRight = std::isfinite(right) && right > 0.0;
    if (hasLeft && hasRight) return 0.5 * (left + right);
    if (hasLeft) return left;
    if (hasRight) return right;
    return NAN;
}

static double modelTrunkReferenceLength(
    const std::shared_ptr<dynamics::Skeleton>& skeleton) {
    if (!skeleton) return NAN;
    dynamics::BodyNode *pelvis = skeleton->getBodyNode("pelvis");
    dynamics::BodyNode *leftShoulder = skeleton->getBodyNode("humerus_l");
    dynamics::BodyNode *rightShoulder = skeleton->getBodyNode("humerus_r");
    if (pelvis == nullptr || leftShoulder == nullptr || rightShoulder == nullptr) {
        return NAN;
    }
    const Eigen::Vector3s shoulderMidpoint = 0.5 * (
        leftShoulder->getWorldTransform().translation()
        + rightShoulder->getWorldTransform().translation()
    );
    const double length = (
        shoulderMidpoint - pelvis->getWorldTransform().translation()
    ).norm();
    return std::isfinite(length) && length > 0.0 ? length : NAN;
}

@implementation NimbleBridge {
    std::shared_ptr<dynamics::Skeleton> _skeleton;
    std::map<std::string, std::pair<dynamics::BodyNode*, Eigen::Vector3s>> _markers;
    BOOL _modelLoaded;

    // Immutable-for-one-load scaling baseline. Every subject scale is derived
    // from these values, never from the skeleton's current (possibly already
    // scaled) state. A successful model reload replaces all four together.
    Eigen::VectorXs _loadedDefaultBodyScales;
    double _loadedLowerReferenceLength;
    double _loadedTrunkReferenceLength;
    double _loadedUpperReferenceLength;

    // Ground-plane estimate for GRF detection, in the ARKit world frame (y-up).
    double _groundHeightY;
    NimbleGroundHeightSource _groundHeightSource;
    // Ring buffer of the most recent per-frame lowest-foot heights. Capacity is
    // fixed at kGroundWindowSamples and never grows, so the estimator's memory
    // is bounded regardless of session length. `_footHeightScratch` is a
    // pre-sized copy buffer so the per-frame percentile does not allocate.
    std::vector<double> _footHeightSamples;
    std::vector<double> _footHeightScratch;
    size_t _footHeightWriteIndex;

    // Previous frame's IK solution, used to warm-start the next solve. Held
    // here rather than read back from the skeleton because the skeleton is
    // shared (MomentArmComputer, ID) and its positions may have been moved to
    // an unrelated pose since the last IK.
    Eigen::VectorXs _lastIKPose;
    BOOL _hasLastIKPose;

    // Runtime DOF mask. `_dofMasked[i]` is 1 when DOF i is held fixed at
    // `_dofPinnedValues(i)` for the duration of an IK solve. Masking is a
    // solver-side restriction only: the skeleton keeps all its DOFs, the .osim
    // is untouched, no joint is welded, and `clearDOFMask` restores the full
    // solve exactly. See `applyDOFMaskWithNames:` for why this cannot be done
    // through `math::IKConfig`.
    std::vector<char> _dofMasked;
    std::vector<int> _freeDofIndices;
    Eigen::VectorXs _dofPinnedValues;
    BOOL _dofMaskActive;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _modelLoaded = NO;
        _loadedLowerReferenceLength = NAN;
        _loadedTrunkReferenceLength = NAN;
        _loadedUpperReferenceLength = NAN;
        _footHeightSamples.reserve(kGroundWindowSamples);
        _footHeightScratch.reserve(kGroundWindowSamples);
        [self resetSessionState];
    }
    return self;
}

- (void)resetSessionState {
    _groundHeightY = 0.0;
    _groundHeightSource = NimbleGroundHeightSourceUncalibrated;
    _footHeightSamples.clear();
    _footHeightScratch.clear();
    _footHeightWriteIndex = 0;
    _lastIKPose.resize(0);
    _hasLastIKPose = NO;
}

- (void)setGroundHeightY:(double)y {
    _groundHeightY = y;
    _groundHeightSource = NimbleGroundHeightSourceExplicit;
    NSLog(@"NimbleBridge: Ground plane set to y=%.4f m", y);
}

- (void)observeLowestFootHeightY:(double)y {
    if (!std::isfinite(y)) return;

    if (_footHeightSamples.size() < kGroundWindowSamples) {
        _footHeightSamples.push_back(y);
    } else {
        _footHeightSamples[_footHeightWriteIndex] = y;
    }
    _footHeightWriteIndex = (_footHeightWriteIndex + 1) % kGroundWindowSamples;

    // An explicitly-calibrated plane wins, but we keep filling the window so
    // that a later resetSessionState leaves the estimator warm rather than
    // blind.
    if (_groundHeightSource == NimbleGroundHeightSourceExplicit) return;

    const size_t n = _footHeightSamples.size();

    // `assign` over a vector whose capacity is already kGroundWindowSamples
    // reuses the existing storage; nth_element is O(n) on 180 doubles.
    _footHeightScratch.assign(_footHeightSamples.begin(), _footHeightSamples.end());
    const size_t index = (size_t)std::floor(kGroundPercentile * (double)(n - 1));
    std::nth_element(_footHeightScratch.begin(),
                     _footHeightScratch.begin() + (ptrdiff_t)index,
                     _footHeightScratch.end());

    _groundHeightY = _footHeightScratch[index] - kGroundContactOffsetMeters;
    _groundHeightSource = (n >= kGroundMinTrustedSamples)
        ? NimbleGroundHeightSourceEstimated
        : NimbleGroundHeightSourceProvisional;
}

// 6 cm. Unchanged from the value that has always shipped, and deliberately so:
// see `GaitContactAgreementTests` for the measurement that says the owner's
// three clips produce ZERO double contacts under it, which is the only evidence
// available about whether it is right and it does not support moving it.
+ (double)contactDetectionThresholdMeters { return 0.06; }

- (double)groundHeightY { return _groundHeightY; }

- (NimbleGroundHeightSource)groundHeightSource { return _groundHeightSource; }

- (BOOL)groundHeightCalibrated {
    return _groundHeightSource != NimbleGroundHeightSourceUncalibrated;
}

- (BOOL)groundHeightTrusted {
    return _groundHeightSource == NimbleGroundHeightSourceEstimated ||
           _groundHeightSource == NimbleGroundHeightSourceExplicit;
}

- (BOOL)ikWarmStartAvailable { return _hasLastIKPose; }

- (BOOL)loadModelFromPath:(NSString *)path {
    try {
        std::string pathStr = std::string([path UTF8String]);
        NSLog(@"NimbleBridge: Loading model from %@", path);

        // Parse the .osim file
        biomechanics::OpenSimFile osimFile = biomechanics::OpenSimParser::parseOsim(pathStr);

        if (!osimFile.skeleton) {
            NSLog(@"NimbleBridge: Failed to parse skeleton from %@", path);
            return NO;
        }

        _skeleton = osimFile.skeleton;

        // The .osim declares `<gravity>0 -9.8066 0</gravity>` (OpenSim models
        // are Y-up), but `OpenSimParser` never reads that element — it builds
        // the skeleton with `Skeleton::create()`, and DART's default gravity is
        // `Eigen::Vector3s(0, 0, -9.81)` (Z-up; see
        // nimblephysics/dart/dynamics/detail/SkeletonAspect.hpp:82). Left
        // unset, the whole ID stack pulls along the subject's medio-lateral
        // axis, which turns body HEIGHT into the moment arm instead of the
        // few-centimetre horizontal offsets that actually load a standing leg,
        // and inflates every joint torque by one to two orders of magnitude.
        //
        // Every nimble biomechanics entry point sets this explicitly right
        // after parsing for the same reason — SubjectOnDisk.cpp:807,
        // DynamicsFitter.cpp:13706.
        _skeleton->setGravity(Eigen::Vector3s(0.0, -9.81, 0.0));

        // Store the model's own markers
        _markers.clear();
        for (const auto& [name, pair] : osimFile.markersMap) {
            _markers[name] = pair;
        }

        // Register virtual markers at joint centers for ARKit compatibility.
        // ARKit gives us joint-center positions, not surface-marker positions,
        // so we attach one marker per body at a well-defined local point.
        //
        // Strategy is adaptive across the two supported models:
        //
        // - Rajagopal2016 (old default): single `torso` body from pelvis to
        //   shoulders, no separate spine/head/clavicle/scapula. ARKit spine
        //   / C7 / neck / head markers attach to the torso body with
        //   heuristic +Y offsets.
        //
        // - cyclistFullBodyMuscle.osim (new full-body): detailed spine with
        //   lumbar1-5, thoracic1-12, sacrum, head_neck, plus clavicle and
        //   scapula segments. We map ARKit markers to the closest real
        //   segment and let IK fit the multi-segment spine properly.
        //
        // A marker whose target body doesn't exist in the loaded skeleton is
        // silently skipped (and reported in the "missing bodies" log below),
        // so the same table works for both models.
        //
        // Each entry: ARKit marker name -> (body name, local offset in meters).
        struct VirtualMarker {
            const char* name;
            const char* bodyName;
            double offsetX, offsetY, offsetZ;
        };
        VirtualMarker virtualMarkers[] = {
            // Pelvis / lower extremities — all at body origin (= joint center)
            {"PELVIS",  "pelvis",    0.0,  0.0,  0.0},
            {"LHJC",    "femur_l",   0.0,  0.0,  0.0},
            {"RHJC",    "femur_r",   0.0,  0.0,  0.0},
            {"LKJC",    "tibia_l",   0.0,  0.0,  0.0},
            {"RKJC",    "tibia_r",   0.0,  0.0,  0.0},
            {"LAJC",    "talus_l",   0.0,  0.0,  0.0},
            {"RAJC",    "talus_r",   0.0,  0.0,  0.0},
            {"LTOE",    "toes_l",    0.0,  0.0,  0.0},
            {"RTOE",    "toes_r",    0.0,  0.0,  0.0},

            // --- Spine markers, new-model mapping (cyclist full body) ---
            // lumbar3 ≈ mid-lumbar, thoracic7 ≈ mid-thoracic, thoracic1 ≈ C7,
            // head_neck is a single body that spans neck + skull.
            {"SPINE_L", "lumbar3",   0.0,  0.0,  0.0},
            {"SPINE_M", "thoracic7", 0.0,  0.0,  0.0},
            {"C7",      "thoracic1", 0.0,  0.0,  0.0},
            {"NECK",    "head_neck", 0.0,  0.0,  0.0},
            {"HEAD",    "head_neck", 0.0,  0.15, 0.0},
            // --- Spine markers, old-model fallback (Rajagopal2016 torso) ---
            // Name-collision: for entries with the same marker name, only the
            // first-resolved body wins because `_markers[name] = ...` is a map
            // assignment. Since Rajagopal2016 doesn't have lumbar3 / thoracic*,
            // the cyclist-model rows above are silently skipped and these
            // torso-based fallbacks are used instead.
            {"SPINE_L", "torso",     0.0,  0.10, 0.0},
            {"SPINE_M", "torso",     0.0,  0.22, 0.0},
            {"C7",      "torso",     0.0,  0.38, 0.0},
            {"NECK",    "torso",     0.0,  0.44, 0.0},
            {"HEAD",    "torso",     0.0,  0.58, 0.0},

            // --- Upper extremities ---
            // New full-body model has clavicle + scapula bodies ahead of
            // humerus. ARKit's shoulder marker is the gleno-humeral joint
            // center, which corresponds to the humerus body origin on both
            // models, so the mapping is the same.
            {"LSJC",    "humerus_l", 0.0, 0.0,  0.0},
            {"RSJC",    "humerus_r", 0.0, 0.0,  0.0},
            {"LEJC",    "ulna_l",    0.0, 0.0,  0.0},
            {"REJC",    "ulna_r",    0.0, 0.0,  0.0},
            {"LWJC",    "hand_l",    0.0, 0.0,  0.0},
            {"RWJC",    "hand_r",    0.0, 0.0,  0.0},
        };

        // First-write-wins: cyclist-model rows come before Rajagopal2016
        // fallbacks in `virtualMarkers`, so on cyclist the detailed spine
        // mapping is installed first and any later fallback row for the
        // same marker is ignored. On Rajagopal2016 the cyclist rows don't
        // resolve (no lumbar3 / thoracic7 / head_neck bodies), so the
        // torso-based fallbacks fill in.
        std::set<std::string> attemptedMarkers;
        for (const auto& vm : virtualMarkers) {
            std::string markerName(vm.name);
            attemptedMarkers.insert(markerName);
            if (_markers.find(markerName) != _markers.end()) {
                continue;  // Already placed — earlier row won.
            }
            dynamics::BodyNode* body = _skeleton->getBodyNode(std::string(vm.bodyName));
            if (body) {
                _markers[markerName] = {
                    body,
                    Eigen::Vector3s(vm.offsetX, vm.offsetY, vm.offsetZ)
                };
            }
        }

        // Report: how many unique ARKit markers got placed vs. attempted.
        NSMutableArray<NSString *> *unplacedNames = [NSMutableArray array];
        for (const auto& name : attemptedMarkers) {
            if (_markers.find(name) == _markers.end()) {
                [unplacedNames addObject:[NSString stringWithUTF8String:name.c_str()]];
            }
        }
        NSLog(@"NimbleBridge: Placed %lu/%lu unique virtual markers",
              (unsigned long)(attemptedMarkers.size() - unplacedNames.count),
              (unsigned long)attemptedMarkers.size());
        if (unplacedNames.count > 0) {
            NSLog(@"NimbleBridge: ⚠ Unresolvable markers (no fallback body worked): %@",
                  [unplacedNames componentsJoinedByString:@", "]);
        }

        // Cache the baseline only after the replacement model is fully
        // installed. The body origins are the same joint centres consumed by
        // scaleModelWithHeight:, and the default vector preserves any native
        // anisotropic or non-unit scale declared by this exact model.
        _loadedDefaultBodyScales = _skeleton->getBodyScales();
        _loadedLowerReferenceLength = averageAvailableLengths(
            modelBodyOriginDistance(_skeleton, "femur_l", "talus_l"),
            modelBodyOriginDistance(_skeleton, "femur_r", "talus_r")
        );
        _loadedTrunkReferenceLength = modelTrunkReferenceLength(_skeleton);
        _loadedUpperReferenceLength = averageAvailableLengths(
            modelBodyOriginDistance(_skeleton, "humerus_l", "hand_l"),
            modelBodyOriginDistance(_skeleton, "humerus_r", "hand_r")
        );
        NSLog(@"NimbleBridge: Loaded scale references — lower %.4f m, trunk %.4f m, upper %.4f m",
              _loadedLowerReferenceLength,
              _loadedTrunkReferenceLength,
              _loadedUpperReferenceLength);

        _modelLoaded = YES;
        // A different skeleton invalidates both the IK warm-start pose (wrong
        // DOF layout) and the ground samples (measured through the old model's
        // foot geometry).
        [self resetSessionState];
        NSLog(@"NimbleBridge: Loaded model with %ld DOFs, %lu markers",
              (long)_skeleton->getNumDofs(), (unsigned long)_markers.size());
        return YES;
    } catch (const std::exception& e) {
        NSLog(@"NimbleBridge: C++ exception loading model: %s", e.what());
        return NO;
    } catch (...) {
        NSLog(@"NimbleBridge: Unknown exception loading model");
        return NO;
    }
}

- (BOOL)isModelLoaded {
    return _modelLoaded;
}

- (double)totalMass {
    if (!_modelLoaded || !_skeleton) return 0.0;
    return _skeleton->getMass();
}

#pragma mark - NimbleBridge (Internal)

- (std::shared_ptr<dart::dynamics::Skeleton>)sharedSkeleton {
    // Returns the live shared_ptr. If no model is loaded this is a null
    // shared_ptr, which is an explicit "no skeleton yet" signal to callers
    // like MomentArmComputer. Any mutation through this pointer (positions,
    // body scales, external forces) is shared with every other holder —
    // intentional, so scaling and IK state stay consistent across objects.
    return _skeleton;
}

- (NSInteger)numDOFs {
    if (!_modelLoaded) return 0;
    return (NSInteger)_skeleton->getNumDofs();
}

- (NSArray<NSString *> *)dofNames {
    if (!_modelLoaded) return @[];
    NSMutableArray *names = [NSMutableArray array];
    for (size_t i = 0; i < _skeleton->getNumDofs(); i++) {
        std::string name = _skeleton->getDof(i)->getName();
        [names addObject:[NSString stringWithUTF8String:name.c_str()]];
    }
    return names;
}

- (NSArray<NSString *> *)markerNames {
    if (!_modelLoaded) return @[];
    NSMutableArray *names = [NSMutableArray array];
    for (const auto& [name, _] : _markers) {
        [names addObject:[NSString stringWithUTF8String:name.c_str()]];
    }
    return names;
}

- (BOOL)scaleModelWithHeight:(double)height
             markerPositions:(NSArray<NSNumber *> *)markerPositions
                 markerNames:(NSArray<NSString *> *)markerNames {
    if (!_modelLoaded || !_skeleton) return NO;

    const size_t numBodies = _skeleton->getNumBodyNodes();
    if (_loadedDefaultBodyScales.size() != (int)(numBodies * 3)) {
        NSLog(@"NimbleBridge: Refusing scale without the loaded model baseline");
        return NO;
    }

    // Per-segment anthropometric scaling.
    //
    // Uniform height-ratio scaling (the previous implementation) is wrong:
    // real anthropometry varies between segments even for subjects of the
    // same height (e.g. limb length relative to height varies ±5%).
    //
    // If the IK pass has already been run and `markerPositions` contain
    // joint-center positions, we derive per-group scale factors directly
    // from inter-joint distances measured in the ARKit world:
    //
    //   lower-limb ratio = (hip→ankle distance, averaged L/R) / loaded-model reference
    //   trunk ratio      = (pelvis→shoulder-midpoint distance) / loaded-model reference
    //   upper-limb ratio = (shoulder→wrist distance, averaged L/R) / loaded-model reference
    //
    // If markers are missing, we fall back to height/1.8 for that group.
    //
    // The denominators and body-scale vector were cached together when this
    // exact model loaded. Repeated calls therefore cannot compound, and a
    // model reload cannot inherit the previous model's proportions.

    auto markerWorld = [&](const std::string& name) -> Eigen::Vector3s {
        for (NSUInteger i = 0; i < markerNames.count; i++) {
            if (std::string([markerNames[i] UTF8String]) == name &&
                i * 3 + 2 < markerPositions.count) {
                return Eigen::Vector3s(
                    [markerPositions[i * 3 + 0] doubleValue],
                    [markerPositions[i * 3 + 1] doubleValue],
                    [markerPositions[i * 3 + 2] doubleValue]
                );
            }
        }
        return Eigen::Vector3s::Zero();
    };
    auto hasMarker = [&](const std::string& name) -> bool {
        for (NSUInteger i = 0; i < markerNames.count; i++) {
            if (std::string([markerNames[i] UTF8String]) == name) return true;
        }
        return false;
    };

    const double fallbackScale = height / 1.8;

    auto segLength = [&](const std::string& a, const std::string& b) -> double {
        if (!hasMarker(a) || !hasMarker(b)) return -1.0;
        return (markerWorld(a) - markerWorld(b)).norm();
    };

    auto ratioFromMeasurement = [](
        double measured,
        double reference,
        double fallback) -> double {
        if (std::isfinite(measured) && measured > 0.0
            && std::isfinite(reference) && reference > 0.0) {
            return measured / reference;
        }
        return fallback;
    };

    // Lower extremity: average of L and R
    double lhl = segLength("LHJC", "LAJC");
    double rhl = segLength("RHJC", "RAJC");
    const double measuredLowerLength = averageAvailableLengths(lhl, rhl);
    double lowerScale = ratioFromMeasurement(
        measuredLowerLength,
        _loadedLowerReferenceLength,
        fallbackScale
    );

    // Upper extremity: average of L and R
    double lal = segLength("LSJC", "LWJC");
    double ral = segLength("RSJC", "RWJC");
    const double measuredUpperLength = averageAvailableLengths(lal, ral);
    double upperScale = ratioFromMeasurement(
        measuredUpperLength,
        _loadedUpperReferenceLength,
        fallbackScale
    );

    // Trunk: pelvis to midpoint of shoulders
    double measuredTrunkLength = NAN;
    if (hasMarker("PELVIS") && hasMarker("LSJC") && hasMarker("RSJC")) {
        Eigen::Vector3s p = markerWorld("PELVIS");
        Eigen::Vector3s shoulderMid = 0.5 * (markerWorld("LSJC") + markerWorld("RSJC"));
        double len = (shoulderMid - p).norm();
        if (len > 0.1) {
            measuredTrunkLength = len;
        }
    }
    double trunkScale = ratioFromMeasurement(
        measuredTrunkLength,
        _loadedTrunkReferenceLength,
        fallbackScale
    );

    // Clamp to sensible anthropometric bounds so a bad single frame can't
    // blow up the skeleton (e.g. partially-tracked frame with wrist near ground).
    auto clampScale = [](double s) { return std::max(0.7, std::min(1.4, s)); };
    lowerScale = clampScale(lowerScale);
    upperScale = clampScale(upperScale);
    trunkScale = clampScale(trunkScale);

    // Assign per-body scale: nimble expects a flat VectorXs of size 3*numBodies
    // arranged as [x,y,z, x,y,z, ...]. Multiply the subject ratio into the
    // loaded default component, preserving native anisotropy and making the
    // operation idempotent.
    Eigen::VectorXs bodyScales = _loadedDefaultBodyScales;

    auto groupScale = [&](const std::string& bodyName) -> double {
        // Upper extremity bodies (incl. clavicle + scapula on the new
        // full-body model, which exist in the kinematic chain between
        // torso/thoracic and humerus).
        if (bodyName.find("humerus") != std::string::npos ||
            bodyName.find("radius") != std::string::npos ||
            bodyName.find("ulna") != std::string::npos ||
            bodyName.find("hand") != std::string::npos ||
            bodyName.find("clavicle") != std::string::npos ||
            bodyName.find("scapula") != std::string::npos) {
            return upperScale;
        }
        // Lower extremity bodies.
        //
        // "kneecap" is the patella. It is spelled that way in FullBody.osim
        // because nimble skips any body literally named `patella_r` /
        // `patella_l` (OpenSimParser.cpp:6562 exact-string match, plus the
        // matching joint skip at :6737-6739). Under the old name the body was
        // never built, so it never reached this lambda at all; under the new
        // name it does, and without this branch it would silently fall through
        // to `trunkScale` and scale the kneecap — and therefore the four
        // quadriceps attachment points it carries — with the torso instead of
        // the leg. `patella` is kept as an alias so the lambda stays correct if
        // the model is ever reverted or a different .osim is loaded.
        if (bodyName.find("femur") != std::string::npos ||
            bodyName.find("tibia") != std::string::npos ||
            bodyName.find("kneecap") != std::string::npos ||
            bodyName.find("patella") != std::string::npos ||
            bodyName.find("talus") != std::string::npos ||
            bodyName.find("calcn") != std::string::npos ||
            bodyName.find("toes") != std::string::npos) {
            return lowerScale;
        }
        // Trunk: pelvis, torso, plus the detailed spine/ribcage on the new
        // full-body model — lumbar*, thoracic*, sacrum, rib*, sternum,
        // head_neck, Abdomen, Abd_*. All get the trunk scale.
        return trunkScale;
    };

    for (size_t i = 0; i < numBodies; i++) {
        auto* body = _skeleton->getBodyNode(i);
        double s = groupScale(body->getName());
        bodyScales(i * 3 + 0) = _loadedDefaultBodyScales(i * 3 + 0) * s;
        bodyScales(i * 3 + 1) = _loadedDefaultBodyScales(i * 3 + 1) * s;
        bodyScales(i * 3 + 2) = _loadedDefaultBodyScales(i * 3 + 2) * s;
    }
    _skeleton->setBodyScales(bodyScales);

    NSLog(@"NimbleBridge: Per-segment ratios — lower %.3f, trunk %.3f, upper %.3f (height fallback %.3f)",
          lowerScale, trunkScale, upperScale, fallbackScale);

    return YES;
}

// MARK: - Runtime DOF mask

- (NSInteger)applyDOFMaskWithNames:(NSArray<NSString *> *)dofNamesToMask {
    // WHY THIS IS NOT DONE THROUGH math::IKConfig
    //
    // `math::IKConfig` (nimblephysics/dart/math/IKSolver.hpp:16-42) carries
    // exactly eleven fields — convergenceThreshold, maxStepCount,
    // leastSquaresDamping, maxRestarts, lossLowerBound, startClamped,
    // dontExitTranspose, lineSearch, logOutput, inputNames, outputNames. There
    // is no DOF selection vector of any kind, and `refineIK` explicitly
    // discards the bounds it is handed (`(void)upperBound; (void)lowerBound;`
    // at IKSolver.cpp:303-304). So a mask cannot be expressed as a config
    // option; it has to be expressed as a smaller *parameterisation* handed to
    // `math::solveIK`. That is what `solveMaskedIKWithMarkers:` does.
    //
    // Reversibility: this touches only `_dofMasked` / `_freeDofIndices` /
    // `_dofPinnedValues`. The skeleton keeps all 163 DOFs, no joint is welded,
    // and the .osim on disk is never written. `clearDOFMask` restores the
    // original solve bit-for-bit.
    if (!_modelLoaded) return 0;

    const int n = (int)_skeleton->getNumDofs();
    _dofMasked.assign((size_t)n, 0);

    // WHERE THE PIN COMES FROM, AND WHY IT IS NOT `getPositions()`
    //
    // This used to read `_skeleton->getPositions()`, i.e. "pin each masked
    // coordinate wherever it currently sits". The skeleton is SHARED across
    // NimbleBridge instances and with `MomentArmComputer` and the ID path
    // (`sharedSkeleton`, NimbleBridge.mm:296), so "currently" meant "whatever
    // pose the last stage in the process happened to write". Measured
    // 2026-08-07: masking `shoulder_rot_{r,l}` right after solving the dancer
    // fixture pinned them at 0.6235 / 0.2877 rad — the dancer's own answer —
    // whereas the same call on a freshly loaded model pins them at 0. That is
    // the same class of order dependence the cold-start seed was fixed for, and
    // it makes a masked coordinate's value a function of process history rather
    // than of the model.
    //
    // The pin is now the model's NEUTRAL pose: the all-zero coordinate vector
    // clamped into the coordinate limits. For the 54 coordinates carrying
    // <locked>true</locked> this is unchanged behaviour — nimble represents the
    // lock as a degenerate [lo, lo] range (OpenSimParser.cpp:5923-5943) and the
    // clamp lands exactly on `lo`, which is what the old special case did by
    // hand. For an UNLOCKED masked coordinate it is a change: the pin is now a
    // declared prior (anatomical neutral) instead of an inherited accident.
    _dofPinnedValues = [self neutralSeedPose];

    std::set<std::string> wanted;
    for (NSString *name in dofNamesToMask) {
        wanted.insert(std::string([name UTF8String]));
    }

    NSInteger matched = 0;
    for (int i = 0; i < n; i++) {
        const std::string dofName = _skeleton->getDof(i)->getName();
        if (wanted.find(dofName) == wanted.end()) continue;
        _dofMasked[(size_t)i] = 1;
        matched++;
    }

    _freeDofIndices.clear();
    _freeDofIndices.reserve((size_t)n);
    for (int i = 0; i < n; i++) {
        if (!_dofMasked[(size_t)i]) _freeDofIndices.push_back(i);
    }
    _dofMaskActive = (matched > 0);

    // The warm-start pose is still dimensionally valid (masking does not
    // change getNumDofs()), but its masked entries may disagree with the pin,
    // so bring it onto the constraint manifold.
    if (_hasLastIKPose && _lastIKPose.size() == n) {
        for (int i = 0; i < n; i++) {
            if (_dofMasked[(size_t)i]) _lastIKPose(i) = _dofPinnedValues(i);
        }
    }

    NSLog(@"NimbleBridge: DOF mask applied — %ld of %ld requested names matched; "
          @"%lu free DOFs of %d",
          (long)matched, (long)dofNamesToMask.count,
          (unsigned long)_freeDofIndices.size(), n);
    return matched;
}

- (void)clearDOFMask {
    _dofMasked.clear();
    _freeDofIndices.clear();
    _dofPinnedValues = Eigen::VectorXs();
    _dofMaskActive = NO;
}

- (BOOL)isDOFMaskActive {
    return _dofMaskActive;
}

- (NSInteger)numFreeDOFs {
    if (!_modelLoaded) return 0;
    if (!_dofMaskActive) return (NSInteger)_skeleton->getNumDofs();
    return (NSInteger)_freeDofIndices.size();
}

- (NSArray<NSString *> *)maskedDOFNames {
    NSMutableArray<NSString *> *out = [NSMutableArray array];
    if (!_modelLoaded || !_dofMaskActive) return out;
    for (int i = 0; i < (int)_dofMasked.size(); i++) {
        if (!_dofMasked[(size_t)i]) continue;
        [out addObject:[NSString stringWithUTF8String:
                        _skeleton->getDof(i)->getName().c_str()]];
    }
    return out;
}

// MARK: - The IK solve

/// The deterministic pose a cold solve starts from: the model's all-zero
/// coordinate vector, clamped into the coordinate limits.
///
/// It has to be an explicit pose rather than "whatever the skeleton currently
/// holds", because the skeleton is shared with `MomentArmComputer` and the ID
/// path (`sharedSkeleton`), so the previous cold-solve seed was in practice the
/// last pose any stage in the process had written. That is the second half of
/// the order-dependence — the first half being `getRandomPose()`'s use of the
/// process-global `rand()`.
- (Eigen::VectorXs)neutralSeedPose {
    const int n = (int)_skeleton->getNumDofs();
    Eigen::VectorXs saved = _skeleton->getPositions();
    _skeleton->setPositions(Eigen::VectorXs::Zero(n));
    _skeleton->clampPositionsToLimits();
    Eigen::VectorXs neutral = _skeleton->getPositions();
    _skeleton->setPositions(saved);
    return neutral;
}

/// Levenberg-Marquardt marker fit over the free coordinates, optionally with a
/// quadratic pull toward `seedPose`.
///
/// Leaves the skeleton at the solved pose. Returns the weighted loss
/// `‖W(f(q) − x*)‖²` (same quantity `math::solveIK` returned, so loss-domain
/// bounds at the call site stay valid), and reports through `outIterations` /
/// `outConverged` whether it stopped because it reached a stationary point or
/// because it ran out of budget.
///
/// Masked coordinates are written from `_dofPinnedValues` on every
/// `setPositions`, so they are held exactly rather than merely clamped at the
/// end of a run, which is all nimble's `<locked>` handling achieves.
- (double)runLMWithMarkers:(const std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>>&)markerList
           targetPositions:(const Eigen::VectorXs&)targetPositions
             markerWeights:(const Eigen::VectorXs&)markerWeights
                  seedPose:(const Eigen::VectorXs&)seedPose
               seedDamping:(double)mu
             outIterations:(int*)outIterations
              outConverged:(BOOL*)outConverged {
    const int n = (int)_skeleton->getNumDofs();
    const int m = (int)markerList.size();
    const int rows = m * 3;

    // Column selection. Without a DOF mask every coordinate is free, so the
    // masked and unmasked solves are literally the same code path — a mask can
    // therefore not change the answer for the coordinates it leaves free.
    std::vector<int> allIdx;
    const std::vector<int>* idxPtr = &_freeDofIndices;
    if (!_dofMaskActive) {
        allIdx.resize((size_t)n);
        for (int i = 0; i < n; i++) allIdx[(size_t)i] = i;
        idxPtr = &allIdx;
    }
    const std::vector<int>& idx = *idxPtr;
    const int f = (int)idx.size();

    // Full-length pose that masked coordinates are written back into.
    Eigen::VectorXs base = seedPose;
    if (_dofMaskActive) {
        for (int i = 0; i < n; i++) {
            if (_dofMasked[(size_t)i]) base(i) = _dofPinnedValues(i);
        }
    }

    if (f == 0) {
        // Everything is masked. Still put the skeleton at the pose the caller
        // asked for, so the pose it reads back afterwards is the seed and not
        // whatever the previous stage left behind.
        _skeleton->setPositions(base);
        if (outIterations) *outIterations = 0;
        if (outConverged) *outConverged = YES;
        Eigen::VectorXs r0 = _skeleton->getMarkerWorldPositions(markerList) - targetPositions;
        for (int j = 0; j < m; j++) r0.segment<3>(j * 3) *= markerWeights(j);
        return (double)r0.squaredNorm();
    }

    // Writes `x` (free coordinates) into the skeleton, applies the model's own
    // joint limits, re-imposes the pins, and reads the clamped values back into
    // `x` so the optimiser's state and the skeleton never disagree.
    auto applyAndClamp = [&](Eigen::VectorXs& x) {
        Eigen::VectorXs full = base;
        for (int k = 0; k < f; k++) full(idx[(size_t)k]) = x(k);
        _skeleton->setPositions(full);
        _skeleton->clampPositionsToLimits();
        Eigen::VectorXs clamped = _skeleton->getPositions();
        bool moved = false;
        for (int k = 0; k < f; k++) {
            if (clamped(idx[(size_t)k]) != x(k)) { x(k) = clamped(idx[(size_t)k]); moved = true; }
        }
        if (moved && _dofMaskActive) {
            // clampPositionsToLimits() enforces [lo, hi], which for an unlocked
            // masked coordinate is wide open, so the pin has to be reapplied.
            for (int i = 0; i < n; i++) {
                if (_dofMasked[(size_t)i]) clamped(i) = _dofPinnedValues(i);
            }
            _skeleton->setPositions(clamped);
        }
    };

    // Coordinate limits, gathered onto the free set once.
    Eigen::VectorXs loAll = _skeleton->getPositionLowerLimits();
    Eigen::VectorXs hiAll = _skeleton->getPositionUpperLimits();
    Eigen::VectorXs lo(f), hi(f);
    for (int k = 0; k < f; k++) {
        lo(k) = loAll(idx[(size_t)k]);
        hi(k) = hiAll(idx[(size_t)k]);
    }

    Eigen::VectorXs xSeed(f);
    for (int k = 0; k < f; k++) xSeed(k) = seedPose(idx[(size_t)k]);
    Eigen::VectorXs x = xSeed;
    applyAndClamp(x);

    Eigen::VectorXs r(rows), rTry(rows);
    auto residualAtCurrentPose = [&](Eigen::VectorXs& out) {
        out = _skeleton->getMarkerWorldPositions(markerList) - targetPositions;
        for (int j = 0; j < m; j++) out.segment<3>(j * 3) *= markerWeights(j);
    };
    residualAtCurrentPose(r);

    auto objective = [&](const Eigen::VectorXs& res, const Eigen::VectorXs& xx) -> double {
        double v = 0.5 * (double)res.squaredNorm();
        if (mu > 0) v += 0.5 * mu * (double)(xx - xSeed).squaredNorm();
        return v;
    };
    double F = objective(r, x);

    Eigen::MatrixXs J(rows, f);
    Eigen::MatrixXs H(f, f);
    Eigen::VectorXs g(f), delta(f), xTry(f);
    std::vector<int> inactive;
    inactive.reserve((size_t)f);

    double lambda = 0.0;   // set from the first Jacobian's scale
    double lambdaMax = 0.0;
    double conditionFloor = 0.0;
    BOOL lambdaInitialised = NO;
    int iterations = 0;
    BOOL converged = NO;
    const char* stopReason = "iteration-cap";

    for (int it = 0; it < kIKMaxIterations; it++) {
        // The Jacobian is scaled by the SAME marker weights as the residual.
        // Nimble scaled only the residual (Skeleton.cpp:7979-7986), which makes
        // its step the minimiser of ‖J·d − W·r‖ rather than the weighted
        // Gauss-Newton direction — a descent direction for no objective the
        // solver is measuring.
        {
            Eigen::MatrixXs fullJac =
                _skeleton->getMarkerWorldPositionsJacobianWrtJointPositions(markerList);
            for (int k = 0; k < f; k++) J.col(k) = fullJac.col(idx[(size_t)k]);
            for (int j = 0; j < m; j++) J.middleRows(j * 3, 3) *= markerWeights(j);
        }
        g.noalias() = J.transpose() * r;
        if (mu > 0) g += (s_t)mu * (x - xSeed);

        if (!lambdaInitialised) {
            // `J.colwise().squaredNorm()` IS `diag(JᵀJ)`, computed without
            // forming the full matrix so the convergence test below can run
            // before any O(f³) work. A warm solve on unchanged markers exits
            // there, so it costs one Jacobian per phase and no factorisation.
            const double scale =
                std::max(1e-12, (double)J.colwise().squaredNorm().maxCoeff() + mu);
            lambda = kIKLambdaInitRel * scale;
            lambdaMax = kIKLambdaMaxRel * scale;
            conditionFloor = kIKConditionFloorRel * scale;
            lambdaInitialised = YES;
        }

        // Active set: a coordinate sitting on a joint limit whose gradient
        // wants to push it further out cannot move, so it is excluded from both
        // the step and the convergence test. Without this the solver spends its
        // whole budget proposing steps that `clampPositionsToLimits` undoes,
        // which is exactly what an extreme pose like the dancer's produces.
        // The descent step is `x − A⁻¹g` with `A` positive definite, so a
        // positive `g` component pushes `x` down.
        inactive.clear();
        double projGradInf = 0.0;
        for (int k = 0; k < f; k++) {
            const double gk = (double)g(k);
            const bool atLower = std::isfinite((double)lo(k)) && (double)x(k) <= (double)lo(k);
            const bool atUpper = std::isfinite((double)hi(k)) && (double)x(k) >= (double)hi(k);
            if ((atLower && gk > 0) || (atUpper && gk < 0)) continue;   // blocked
            inactive.push_back(k);
            projGradInf = std::max(projGradInf, std::abs(gk));
        }

        if (inactive.empty() || projGradInf <= kIKGradientTolerance) {
            converged = YES;
            stopReason = "gradient";
            break;
        }

        H.noalias() = J.transpose() * J;
        if (mu > 0) H.diagonal().array() += (s_t)mu;

        const int nf = (int)inactive.size();
        Eigen::MatrixXs Ared(nf, nf), Adamped(nf, nf);
        Eigen::VectorXs gred(nf), dred(nf);
        for (int a = 0; a < nf; a++) {
            gred(a) = g(inactive[(size_t)a]);
            for (int b = 0; b < nf; b++) {
                Ared(a, b) = H(inactive[(size_t)a], inactive[(size_t)b]);
            }
        }

        // Trust-region loop: shrink the step (raise lambda) until it decreases
        // the objective. This is what nimble's fixed `leastSquaresDamping`
        // cannot do, and why its step count is spent halving a learning rate
        // and then falling back to the gradient transpose.
        BOOL accepted = NO;
        BOOL tinyStep = NO;
        while (true) {
            Adamped = Ared;
            Adamped.diagonal().array() += (s_t)(lambda + conditionFloor);
            dred = Adamped.ldlt().solve(gred);
            if (!dred.allFinite()) {
                lambda = std::max(lambda, conditionFloor) * kIKLambdaUp;
                if (lambda > lambdaMax) break;
                continue;
            }
            delta.setZero();
            for (int a = 0; a < nf; a++) delta(inactive[(size_t)a]) = dred(a);

            if ((double)delta.cwiseAbs().maxCoeff() <= kIKStepTolerance) {
                tinyStep = YES;
                break;
            }
            xTry = x - delta;
            applyAndClamp(xTry);
            residualAtCurrentPose(rTry);
            const double FTry = objective(rTry, xTry);
            if (FTry < F) {
                x = xTry;
                r = rTry;
                F = FTry;
                lambda *= kIKLambdaDown;
                accepted = YES;
                break;
            }
            lambda = std::max(lambda, conditionFloor) * kIKLambdaUp;
            if (lambda > lambdaMax) break;
        }

        iterations++;

        if (tinyStep) { converged = YES; stopReason = "step"; break; }
        if (!accepted) {
            // No damping up to lambdaMax produced a decrease. That is a
            // stationary point to the precision this arithmetic supports.
            converged = YES;
            stopReason = "no-decrease";
            break;
        }
    }

    applyAndClamp(x);
    residualAtCurrentPose(r);

    if (kIKTraceSolve) {
        NSLog(@"IKTRACE mu=%g iters=%d stop=%s loss=%.9g", mu, iterations, stopReason,
              (double)r.squaredNorm());
    }
    if (outIterations) *outIterations = iterations;
    if (outConverged) *outConverged = converged;
    return (double)r.squaredNorm();
}

/// Phase A (seed-damped) followed by phase B (undamped polish). See the note by
/// `kIKSeedDamping` for why both are needed.
- (double)solveIKFromSeed:(const Eigen::VectorXs&)seedPose
                  markers:(const std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>>&)markerList
          targetPositions:(const Eigen::VectorXs&)targetPositions
            markerWeights:(const Eigen::VectorXs&)markerWeights
            outIterations:(int*)outIterations
             outConverged:(BOOL*)outConverged {
    int itA = 0, itB = 0;
    BOOL convA = NO, convB = NO;
    [self runLMWithMarkers:markerList
           targetPositions:targetPositions
             markerWeights:markerWeights
                  seedPose:seedPose
               seedDamping:kIKSeedDamping
             outIterations:&itA
              outConverged:&convA];
    Eigen::VectorXs afterA = _skeleton->getPositions();
    double loss = [self runLMWithMarkers:markerList
                         targetPositions:targetPositions
                           markerWeights:markerWeights
                                seedPose:afterA
                             seedDamping:0.0
                           outIterations:&itB
                            outConverged:&convB];
    if (outIterations) *outIterations = itA + itB;
    if (outConverged) *outConverged = (convA && convB);
    return loss;
}

- (nullable NimbleIKResult *)solveIKWithMarkerPositions:(NSArray<NSNumber *> *)markerPositions
                                            markerNames:(NSArray<NSString *> *)markerNames {
    if (!_modelLoaded) return nil;
    if (markerPositions.count != markerNames.count * 3) return nil;

    // Build marker list — ONLY include markers that exist in the model (no nullptrs)
    std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>> markerList;
    std::vector<Eigen::Vector3s> targetList;
    std::vector<double> weightList;
    NSMutableArray<NSString *> *resolvedNames = [NSMutableArray array];

    for (NSUInteger i = 0; i < markerNames.count; i++) {
        std::string name = std::string([markerNames[i] UTF8String]);

        auto it = _markers.find(name);
        if (it == _markers.end() || it->second.first == nullptr) {
            continue;  // Skip unknown markers entirely — don't pass nullptr to Nimble
        }

        [resolvedNames addObject:markerNames[i]];
        markerList.push_back(it->second);
        targetList.push_back(Eigen::Vector3s(
            [markerPositions[i * 3 + 0] doubleValue],
            [markerPositions[i * 3 + 1] doubleValue],
            [markerPositions[i * 3 + 2] doubleValue]
        ));
        weightList.push_back(markerReliabilityWeight(name));
    }

    if (markerList.empty()) return nil;

    // Flatten target positions
    Eigen::VectorXs targetPositions(markerList.size() * 3);
    for (size_t i = 0; i < targetList.size(); i++) {
        targetPositions(i * 3 + 0) = targetList[i].x();
        targetPositions(i * 3 + 1) = targetList[i].y();
        targetPositions(i * 3 + 2) = targetList[i].z();
    }

    // Reliability weights, renormalised to RMS 1. Only the ratios between
    // weights affect the solution; the common factor is fixed so that the loss
    // keeps the same magnitude as an all-ones weighting, and so the loss bounds
    // computed below stay valid.
    //
    // The solver scales the Jacobian by these same weights, so the step really
    // is the weighted Gauss-Newton direction. (Nimble scaled only the residual,
    // Skeleton.cpp:7979-7986, which is why the previous implementation had to
    // keep the spread mild — a wide spread produced steps that were not descent
    // directions for anything it was measuring, and its line search reverted
    // them.)
    Eigen::VectorXs weights(markerList.size());
    for (size_t i = 0; i < weightList.size(); i++) {
        weights((int)i) = (s_t)weightList[i];
    }
    s_t weightSumSq = weights.squaredNorm();
    if (weightSumSq > 0) {
        weights *= sqrt((s_t)markerList.size() / weightSumSq);
    }

    // The loss is the squared norm of the weighted residual stack, so a
    // per-marker tolerance `t` corresponds to a loss of sum_i(w_i^2 * t^2),
    // which after the RMS-1 renormalisation is exactly numMarkers * t^2.
    auto lossBoundForResidual = [&](double residualMeters) -> double {
        return (double)markerList.size() * residualMeters * residualMeters;
    };

    try {
        // Seed: the previous frame's pose when there is one, so the solve
        // refines it and joint angles stay temporally continuous; otherwise the
        // model's neutral pose. Both are written explicitly — never inherited
        // from the shared skeleton, which ID and the moment-arm computer also
        // write to.
        const BOOL warmStarted =
            _hasLastIKPose && _lastIKPose.size() == (int)_skeleton->getNumDofs();
        Eigen::VectorXs seed = warmStarted ? _lastIKPose : [self neutralSeedPose];

        int iterations = 0;
        BOOL converged = NO;
        double error = [self solveIKFromSeed:seed
                                     markers:markerList
                             targetPositions:targetPositions
                               markerWeights:weights
                               outIterations:&iterations
                                outConverged:&converged];

        if (warmStarted && error > lossBoundForResidual(kIKWarmStartRejectMeters)) {
            // The previous pose was not a usable seed at all — the subject left
            // and re-entered frame, or came back from a long occlusion. Redo it
            // from the neutral pose, once. This is a genuine re-search, not a
            // lottery: the seed is deterministic, so the recovery pose is too.
            int coldIterations = 0;
            BOOL coldConverged = NO;
            double coldError = [self solveIKFromSeed:[self neutralSeedPose]
                                             markers:markerList
                                     targetPositions:targetPositions
                                       markerWeights:weights
                                       outIterations:&coldIterations
                                        outConverged:&coldConverged];
            error = coldError;
            iterations += coldIterations;
            converged = coldConverged;
        }

        // Extract joint angles
        Eigen::VectorXs positions = _skeleton->getPositions();
        _lastIKPose = positions;
        _hasLastIKPose = YES;
        NSMutableArray<NSNumber *> *angles = [NSMutableArray arrayWithCapacity:positions.size()];
        for (int i = 0; i < positions.size(); i++) {
            [angles addObject:@(positions(i))];
        }

        NimbleIKResult *result =
            [[NimbleIKResult alloc] initWithAngles:angles
                                             error:error
                                           numDOFs:positions.size()
                                          dofNames:self.dofNames];
        // Accuracy readback, unweighted, from the pose actually returned.
        Eigen::VectorXs residual =
            _skeleton->getMarkerWorldPositions(markerList) - targetPositions;
        double sumSq = 0.0, maxSq = 0.0;
        NSMutableDictionary<NSString *, NSNumber *> *perMarker =
            [NSMutableDictionary dictionaryWithCapacity:markerList.size()];
        for (size_t i = 0; i < markerList.size(); i++) {
            const double d2 = (double)residual.segment<3>((int)i * 3).squaredNorm();
            sumSq += d2;
            maxSq = std::max(maxSq, d2);
            perMarker[resolvedNames[i]] = @(std::sqrt(d2));
        }
        [result setMarkerRMSMeters:std::sqrt(sumSq / (double)markerList.size())
              markerMaxErrorMeters:std::sqrt(maxSq)
                markerErrorsMeters:perMarker
                       markerCount:(NSInteger)markerList.size()
                        iterations:(NSInteger)iterations
                         converged:converged];
        return result;
    } catch (const std::exception& e) {
        NSLog(@"NimbleBridge: IK exception: %s", e.what());
        return nil;
    } catch (...) {
        NSLog(@"NimbleBridge: IK unknown exception");
        return nil;
    }
}

- (nullable NimbleIDResult *)solveIDWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                   jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                               jointAccelerations:(NSArray<NSNumber *> *)jointAccelerations {
    if (!_modelLoaded) return nil;

    NSInteger numDOFs = (NSInteger)_skeleton->getNumDofs();
    if (jointAngles.count != numDOFs ||
        jointVelocities.count != numDOFs ||
        jointAccelerations.count != numDOFs) {
        return nil;
    }

    try {
        // Set skeleton state
        Eigen::VectorXs q(numDOFs), dq(numDOFs), ddq(numDOFs);
        for (NSInteger i = 0; i < numDOFs; i++) {
            q(i) = [jointAngles[i] doubleValue];
            dq(i) = [jointVelocities[i] doubleValue];
            ddq(i) = [jointAccelerations[i] doubleValue];
        }

        _skeleton->setPositions(q);
        _skeleton->setVelocities(dq);

        // Compute inverse dynamics: tau = M*ddq + C(q,dq) - J^T * F_ext
        Eigen::VectorXs torques = _skeleton->getInverseDynamics(ddq);

        // Convert to NSArray
        NSMutableArray<NSNumber *> *torqueArray = [NSMutableArray arrayWithCapacity:numDOFs];
        for (NSInteger i = 0; i < numDOFs; i++) {
            [torqueArray addObject:@(torques(i))];
        }

        return [[NimbleIDResult alloc] initWithTorques:torqueArray];
    } catch (const std::exception& e) {
        NSLog(@"NimbleBridge: ID exception: %s", e.what());
        return nil;
    } catch (...) {
        NSLog(@"NimbleBridge: ID unknown exception");
        return nil;
    }
}

// MARK: - ID with ground reaction force estimation
//
// This is the production ID path — it uses Nimble's built-in multi-contact,
// near-CoP inverse-dynamics solver to decompose the system wrench into
// per-foot GRFs plus joint torques. Foot contact is detected from the y
// coordinate of `calcn_l` / `calcn_r` versus a running ground-height
// estimate. When neither foot is in contact (flight), we fall back to
// plain getInverseDynamics which implicitly assumes zero external force.

static inline NSArray<NSNumber *> *vec3ToNSArray(const Eigen::Vector3s& v) {
    return @[@(v.x()), @(v.y()), @(v.z())];
}

- (nullable NimbleIDResult *)solveIDGRFWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                       jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                                    jointAccelerations:(NSArray<NSNumber *> *)jointAccelerations {
    if (!_modelLoaded) return nil;

    NSInteger numDOFs = (NSInteger)_skeleton->getNumDofs();
    if (jointAngles.count != numDOFs ||
        jointVelocities.count != numDOFs ||
        jointAccelerations.count != numDOFs) {
        return nil;
    }

    try {
        // Set skeleton state
        Eigen::VectorXs q(numDOFs), dq(numDOFs), ddq(numDOFs);
        for (NSInteger i = 0; i < numDOFs; i++) {
            q(i)   = [jointAngles[i] doubleValue];
            dq(i)  = [jointVelocities[i] doubleValue];
            ddq(i) = [jointAccelerations[i] doubleValue];
        }

        _skeleton->setPositions(q);
        _skeleton->setVelocities(dq);
        // ddq is passed to the solver explicitly, but the skeleton also has to
        // carry it for `getCOMLinearAcceleration()` in the residual check below
        // to describe THIS frame rather than the previous one.
        _skeleton->setAccelerations(ddq);

        // --- 1. Ground height auto-calibration ---
        // Feed this frame's lowest heel height to the rolling estimator (see
        // observeLowestFootHeightY). It needs no explicit calibration, and
        // unlike a running minimum it recovers from transient dips instead of
        // permanently sinking the floor and flipping ID into its flight-phase
        // branch for the rest of the session.
        dynamics::BodyNode* calcnL = _skeleton->getBodyNode("calcn_l");
        dynamics::BodyNode* calcnR = _skeleton->getBodyNode("calcn_r");
        if (!calcnL || !calcnR) {
            // No foot bodies → can't do GRF; fall back to regular ID.
            Eigen::VectorXs torques = _skeleton->getInverseDynamics(ddq);
            NSMutableArray<NSNumber *> *ta = [NSMutableArray arrayWithCapacity:numDOFs];
            for (NSInteger i = 0; i < numDOFs; i++) [ta addObject:@(torques(i))];
            return [[NimbleIDResult alloc] initWithTorques:ta];
        }

        double calcnLY = calcnL->getWorldTransform().translation().y();
        double calcnRY = calcnR->getWorldTransform().translation().y();
        double lowest = std::min(calcnLY, calcnRY);
        [self observeLowestFootHeightY:lowest];

        // --- 2. Contact detection ---
        // A foot is in contact if its heel y sits within CONTACT_THRESHOLD
        // of the ground and it's moving slowly (we gate on velocity too via
        // getCOMLinearVelocity later if needed — for baseline we go by
        // position alone and let the near-CoP solver absorb the rest).
        const double CONTACT_THRESHOLD = [NimbleBridge contactDetectionThresholdMeters];
        BOOL leftContact  = (calcnLY - _groundHeightY) < CONTACT_THRESHOLD;
        BOOL rightContact = (calcnRY - _groundHeightY) < CONTACT_THRESHOLD;

        // --- 3. Build initial wrench guesses (Newton-Euler baseline) ---
        // Whole-body weight: GRF_total ≈ mass * g (static stance). We split
        // 50/50 if both in contact, 100% on the single foot if only one,
        // and zero if airborne (caller falls through to plain ID).
        std::vector<const dynamics::BodyNode*> contactBodies;
        std::vector<Eigen::Vector6s> wrenchGuesses;

        double mass = _skeleton->getMass();
        Eigen::Vector3s weightUp(0.0, mass * 9.81, 0.0);

        int contactCount = (leftContact ? 1 : 0) + (rightContact ? 1 : 0);
        if (contactCount == 0) {
            // Flight phase — zero GRF, plain ID.
            Eigen::VectorXs torques = _skeleton->getInverseDynamics(ddq);
            NSMutableArray<NSNumber *> *ta = [NSMutableArray arrayWithCapacity:numDOFs];
            for (NSInteger i = 0; i < numDOFs; i++) [ta addObject:@(torques(i))];
            NSArray<NSNumber *> *zero3 = @[@0.0, @0.0, @0.0];
            return [[NimbleIDResult alloc] initWithTorques:ta
                                                 leftForce:zero3
                                                rightForce:zero3
                                                   leftCoP:zero3
                                                  rightCoP:zero3
                                             leftInContact:NO
                                            rightInContact:NO
                                          rootResidualNorm:0.0];
        }

        Eigen::Vector3s perFootForce = weightUp / contactCount;
        // getMultipleContactInverseDynamicsNearCoP takes and returns wrenches
        // in each contact body's OWN frame, at the body origin, ordered
        // [angular; linear]: it builds its Jacobians with
        // `getJacobian(bodies[i])` (body-frame, body-origin — MetaSkeleton.hpp:
        // 559) and maps the guesses to world with `dAdInvT` before projecting
        // them to a CoP (Skeleton.cpp:10205). Handing it a world-frame wrench
        // silently rotates the guess into the foot's frame, so on a two-foot
        // stance the least-squares solve lands on a physically wrong split of
        // bodyweight between the feet. The reference caller does the same
        // conversion: DynamicsFitter.cpp:16541.
        //
        // The guess itself is the Newton-Euler baseline: this foot's share of
        // bodyweight, pushing straight up, applied on the ground directly
        // below the foot's origin. `projectWrenchToCoP` then recovers exactly
        // that point as the CoP guess.
        auto makeWrenchGuess = [&](dynamics::BodyNode* foot) -> Eigen::Vector6s {
            Eigen::Vector3s origin = foot->getWorldTransform().translation();
            Eigen::Vector3s applyAt(origin.x(), (s_t)_groundHeightY, origin.z());
            Eigen::Vector6s world;
            world.head<3>() = applyAt.cross(perFootForce);  // moment about the world origin
            world.tail<3>() = perFootForce;                 // force, world frame
            return math::dAdT(foot->getWorldTransform(), world);
        };

        if (leftContact)  {
            contactBodies.push_back(calcnL);
            wrenchGuesses.push_back(makeWrenchGuess(calcnL));
        }
        if (rightContact) {
            contactBodies.push_back(calcnR);
            wrenchGuesses.push_back(makeWrenchGuess(calcnR));
        }

        // --- 4. Solve ---
        // getMultipleContactInverseDynamicsNearCoP finds per-foot wrenches
        // that (a) satisfy the Newton-Euler acceleration constraint, (b) sit
        // as close as possible to the wrench guesses, and (c) have their
        // center-of-pressure inside each foot's support polygon / on the
        // ground plane. Vertical axis = 1 (y).
        auto result = _skeleton->getMultipleContactInverseDynamicsNearCoP(
            ddq,
            contactBodies,
            wrenchGuesses,
            (s_t)_groundHeightY,
            1,          // vertical axis = y
            0.001,      // default weightForceToMeters
            false);     // not verbose

        // --- 5. Package results ---
        NSMutableArray<NSNumber *> *torqueArray = [NSMutableArray arrayWithCapacity:numDOFs];
        for (NSInteger i = 0; i < result.jointTorques.size(); i++) {
            [torqueArray addObject:@(result.jointTorques(i))];
        }

        // Pull left/right wrenches back out of result.contactWrenches in the
        // same order we pushed them. Extract force and CoP where applicable.
        //
        // `result.contactWrenches[i]` comes back in body-local coordinates —
        // nimble's own world conversion is commented out at Skeleton.cpp:10354
        // — so it has to be mapped back before any of it means anything in the
        // world frame. `dAdInvT` is that dual transform: f_world = R*f_local,
        // m_world = R*m_local + p x f_world.
        //
        // Once the wrench IS in world coordinates, the centre of pressure is
        // nimble's own `projectWrenchToCoP`, which solves for the point on the
        // ground plane where the wrench reduces to a force plus a normal free
        // moment. The hand-rolled projection this replaces used `force.y()` as
        // the vertical load, which is only the vertical load in the world
        // frame, and divided by it — on a body-local wrench that is an
        // arbitrary component of the force.
        Eigen::Vector3s zero3 = Eigen::Vector3s::Zero();
        Eigen::Vector3s leftForce = zero3, rightForce = zero3;
        Eigen::Vector3s leftCoP = zero3, rightCoP = zero3;
        size_t idx = 0;
        auto worldWrench = [&](size_t i, const dynamics::BodyNode* foot) -> Eigen::Vector6s {
            return math::dAdInvT(foot->getWorldTransform(), result.contactWrenches[i]);
        };
        Eigen::Vector3s totalContactForce = Eigen::Vector3s::Zero();
        if (leftContact) {
            Eigen::Vector6s w = worldWrench(idx, calcnL);
            leftForce = w.tail<3>();
            leftCoP = math::projectWrenchToCoP(w, (s_t)_groundHeightY, 1).head<3>();
            totalContactForce += leftForce;
            idx++;
        }
        if (rightContact) {
            Eigen::Vector6s w = worldWrench(idx, calcnR);
            rightForce = w.tail<3>();
            rightCoP = math::projectWrenchToCoP(w, (s_t)_groundHeightY, 1).head<3>();
            totalContactForce += rightForce;
            idx++;
        }

        // Root residual: how far this frame is from global linear equilibrium,
        // in Newtons.
        //
        // This deliberately does NOT read `jointTorques.head<6>()`. Nimble ends
        // `getMultipleContactInverseDynamicsNearCoP` with
        // `result.jointTorques.head<6>().setZero()` (Skeleton.cpp:10365) and the
        // assert above that line is compiled out of the Release static libs, so
        // that quantity is a hard-coded zero and carries no information.
        //
        // The falsifiable version: the solved contact forces plus gravity must
        // sum to the whole-body linear momentum rate. Quasi-statically that is
        // ~0; under acceleration it is m * a_com, so the residual is reported
        // against that, not against zero.
        Eigen::Vector3s momentumRate = _skeleton->getMass() * _skeleton->getCOMLinearAcceleration();
        double rootResidual =
            (totalContactForce + _skeleton->getMass() * _skeleton->getGravity() - momentumRate)
                .norm();

        return [[NimbleIDResult alloc] initWithTorques:torqueArray
                                             leftForce:vec3ToNSArray(leftForce)
                                            rightForce:vec3ToNSArray(rightForce)
                                               leftCoP:vec3ToNSArray(leftCoP)
                                              rightCoP:vec3ToNSArray(rightCoP)
                                         leftInContact:leftContact
                                        rightInContact:rightContact
                                      rootResidualNorm:rootResidual];
    } catch (const std::exception& e) {
        NSLog(@"NimbleBridge: ID+GRF exception: %s", e.what());
        return nil;
    } catch (...) {
        NSLog(@"NimbleBridge: ID+GRF unknown exception");
        return nil;
    }
}

@end
