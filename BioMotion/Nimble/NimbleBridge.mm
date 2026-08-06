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

@implementation NimbleIKResult {
    NSArray<NSNumber *> *_jointAngles;
    double _error;
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
        _numDOFs = numDOFs;
        _dofNames = [dofNames copy];
    }
    return self;
}

- (NSArray<NSNumber *> *)jointAngles { return _jointAngles; }
- (double)error { return _error; }
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
// getRandomPose() — discarding the previous frame's solution at 171 DOF and
// injecting joint-angle jitter that the Savitzky-Golay stage differentiates
// twice (gain ~1/dt^2) straight into the accelerations that drive ID.
//
// Per-marker residual we consider "converged" given the ARKit noise floor.
static const double kIKMarkerToleranceMeters = 0.02;
// A warm-started solve landing above this per-marker residual is not a
// refinement of the previous pose at all (subject left and re-entered frame,
// recovery from a long occlusion), so it is redone cold — once.
static const double kIKWarmStartRejectMeters = 0.15;
// Restart budget for a cold solve, i.e. the first frame of a session or a
// rejected warm solve. Matches Nimble's default; the point of the fix is that
// warm frames don't pay it, not that global search is never useful.
static const int kIKColdRestarts = 5;
static const int kIKWarmRestarts = 1;

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

@implementation NimbleBridge {
    std::shared_ptr<dynamics::Skeleton> _skeleton;
    std::map<std::string, std::pair<dynamics::BodyNode*, Eigen::Vector3s>> _markers;
    BOOL _modelLoaded;

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
    if (!_modelLoaded) return NO;

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
    //   lower-limb scale = (hip→ankle distance, averaged L/R) / model default
    //   trunk scale      = (pelvis→shoulder-midpoint distance) / model default
    //   upper-limb scale = (shoulder→wrist distance, averaged L/R) / model default
    //
    // If markers are missing, we fall back to height/1.8 for that group.
    //
    // For Rajagopal2016 with default male proportions the approximate
    // reference segment lengths at 1.8m standing height are:
    //   lower extremity (hip→ankle)    ≈ 0.88 m
    //   trunk (pelvis→shoulder midpt)  ≈ 0.52 m
    //   upper extremity (shoulder→wrist) ≈ 0.54 m

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

    // Lower extremity: average of L and R
    double lowerScale = fallbackScale;
    double lhl = segLength("LHJC", "LAJC");
    double rhl = segLength("RHJC", "RAJC");
    std::vector<double> lowerLengths;
    if (lhl > 0) lowerLengths.push_back(lhl);
    if (rhl > 0) lowerLengths.push_back(rhl);
    if (!lowerLengths.empty()) {
        double avg = 0;
        for (double l : lowerLengths) avg += l;
        avg /= lowerLengths.size();
        lowerScale = avg / 0.88;  // reference lower-limb length at 1.8m height
    }

    // Upper extremity: average of L and R
    double upperScale = fallbackScale;
    double lal = segLength("LSJC", "LWJC");
    double ral = segLength("RSJC", "RWJC");
    std::vector<double> upperLengths;
    if (lal > 0) upperLengths.push_back(lal);
    if (ral > 0) upperLengths.push_back(ral);
    if (!upperLengths.empty()) {
        double avg = 0;
        for (double l : upperLengths) avg += l;
        avg /= upperLengths.size();
        upperScale = avg / 0.54;  // reference upper-limb length at 1.8m height
    }

    // Trunk: pelvis to midpoint of shoulders
    double trunkScale = fallbackScale;
    if (hasMarker("PELVIS") && hasMarker("LSJC") && hasMarker("RSJC")) {
        Eigen::Vector3s p = markerWorld("PELVIS");
        Eigen::Vector3s shoulderMid = 0.5 * (markerWorld("LSJC") + markerWorld("RSJC"));
        double len = (shoulderMid - p).norm();
        if (len > 0.1) {
            trunkScale = len / 0.52;  // reference pelvis→shoulder length at 1.8m
        }
    }

    // Clamp to sensible anthropometric bounds so a bad single frame can't
    // blow up the skeleton (e.g. partially-tracked frame with wrist near ground).
    auto clampScale = [](double s) { return std::max(0.7, std::min(1.4, s)); };
    lowerScale = clampScale(lowerScale);
    upperScale = clampScale(upperScale);
    trunkScale = clampScale(trunkScale);

    // Assign per-body scale: nimble expects a flat VectorXs of size 3*numBodies
    // arranged as [x,y,z, x,y,z, ...]. We group each body into lower/trunk/upper.
    size_t numBodies = _skeleton->getNumBodyNodes();
    Eigen::VectorXs bodyScales(numBodies * 3);

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
        bodyScales(i * 3 + 0) = s;
        bodyScales(i * 3 + 1) = s;
        bodyScales(i * 3 + 2) = s;
    }
    _skeleton->setBodyScales(bodyScales);

    NSLog(@"NimbleBridge: Per-segment scale — lower %.3f, trunk %.3f, upper %.3f (height fallback %.3f)",
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
    _dofPinnedValues = _skeleton->getPositions();

    std::set<std::string> wanted;
    for (NSString *name in dofNamesToMask) {
        wanted.insert(std::string([name UTF8String]));
    }

    NSInteger matched = 0;
    for (int i = 0; i < n; i++) {
        const std::string dofName = _skeleton->getDof(i)->getName();
        if (wanted.find(dofName) == wanted.end()) continue;
        _dofMasked[(size_t)i] = 1;
        // Pin at the coordinate's own rest position when the parser gave it a
        // degenerate [lo, lo] range (that is what nimble does for
        // <locked>true</locked>: OpenSimParser.cpp:5923-5943), otherwise pin at
        // wherever the DOF currently sits.
        const double lo = (double)_skeleton->getDof(i)->getPositionLowerLimit();
        const double hi = (double)_skeleton->getDof(i)->getPositionUpperLimit();
        if (std::isfinite(lo) && std::isfinite(hi) && std::abs(hi - lo) < 1e-12) {
            _dofPinnedValues(i) = (s_t)lo;
        }
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

/// Solve IK over only the unmasked DOFs.
///
/// Structurally identical to `Skeleton::fitMarkersToWorldPositions`
/// (Skeleton.cpp:7959-7987) except that the optimisation variable is the
/// F-vector of free coordinates rather than the full n-vector, and the
/// Jacobian handed to the solver is the corresponding F-column slice. Masked
/// coordinates are written from `_dofPinnedValues` on every `setPositions`, so
/// they are held exactly — not merely clamped at the end of a run, which is
/// all that nimble's `<locked>` handling achieves.
- (double)solveMaskedIKWithMarkers:(const std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>>&)markerList
                   targetPositions:(const Eigen::VectorXs&)targetPositions
                     markerWeights:(const Eigen::VectorXs&)markerWeights
                            config:(math::IKConfig)config {
    const int f = (int)_freeDofIndices.size();
    const std::vector<int>& freeIdx = _freeDofIndices;
    Eigen::VectorXs pinned = _dofPinnedValues;

    auto gather = [&](const Eigen::VectorXs& full) {
        Eigen::VectorXs sub(f);
        for (int k = 0; k < f; k++) sub(k) = full(freeIdx[(size_t)k]);
        return sub;
    };
    auto scatter = [&](const Eigen::VectorXs& sub) {
        Eigen::VectorXs full = pinned;
        for (int k = 0; k < f; k++) full(freeIdx[(size_t)k]) = sub(k);
        return full;
    };

    Eigen::VectorXs initial = gather(_skeleton->getPositions());
    Eigen::VectorXs upper = gather(_skeleton->getPositionUpperLimits());
    Eigen::VectorXs lower = gather(_skeleton->getPositionLowerLimits());

    auto* skel = _skeleton.get();

    return (double)math::solveIK(
        initial,
        upper,
        lower,
        (int)markerList.size() * 3,
        [skel, scatter, gather](const Eigen::VectorXs pos, bool clamp) {
            skel->setPositions(scatter(pos));
            if (clamp) {
                skel->clampPositionsToLimits();
                // Re-impose the pin: clampPositionsToLimits() only enforces
                // [lo, hi], which for an unlocked masked DOF is wide open.
                skel->setPositions(scatter(gather(skel->getPositions())));
                return gather(skel->getPositions());
            }
            return pos;
        },
        [skel, targetPositions, markerList, markerWeights, freeIdx, f](
            Eigen::Ref<Eigen::VectorXs> diff, Eigen::Ref<Eigen::MatrixXs> jac) {
            diff = skel->getMarkerWorldPositions(markerList) - targetPositions;
            for (int j = 0; j < markerWeights.size(); j++) {
                diff.segment<3>(j * 3) *= markerWeights(j);
            }
            Eigen::MatrixXs fullJac =
                skel->getMarkerWorldPositionsJacobianWrtJointPositions(markerList);
            for (int k = 0; k < f; k++) jac.col(k) = fullJac.col(freeIdx[(size_t)k]);
        },
        [skel, gather](Eigen::Ref<Eigen::VectorXs> val) {
            val = gather(skel->getRandomPose());
        },
        config);
}

- (nullable NimbleIKResult *)solveIKWithMarkerPositions:(NSArray<NSNumber *> *)markerPositions
                                            markerNames:(NSArray<NSString *> *)markerNames {
    if (!_modelLoaded) return nil;
    if (markerPositions.count != markerNames.count * 3) return nil;

    // Build marker list — ONLY include markers that exist in the model (no nullptrs)
    std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>> markerList;
    std::vector<Eigen::Vector3s> targetList;
    std::vector<double> weightList;

    for (NSUInteger i = 0; i < markerNames.count; i++) {
        std::string name = std::string([markerNames[i] UTF8String]);

        auto it = _markers.find(name);
        if (it == _markers.end() || it->second.first == nullptr) {
            continue;  // Skip unknown markers entirely — don't pass nullptr to Nimble
        }

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
    // Nimble returns (a weighted sum of squared marker residuals) keeps the
    // same magnitude as the previous all-ones weighting, and so the loss bound
    // computed below stays valid.
    //
    // Nimble scales the residual by these weights but not the Jacobian, so the
    // damped-least-squares step is only approximately the weighted Gauss-Newton
    // direction. That is why the spread is kept mild (0.4-1.0): the solver's
    // line search reverts any step that increases the loss, but a wide spread
    // would make it revert often and converge slowly.
    Eigen::VectorXs weights(markerList.size());
    for (size_t i = 0; i < weightList.size(); i++) {
        weights((int)i) = (s_t)weightList[i];
    }
    s_t weightSumSq = weights.squaredNorm();
    if (weightSumSq > 0) {
        weights *= sqrt((s_t)markerList.size() / weightSumSq);
    }

    // Nimble's loss is the squared norm of the weighted residual stack, so a
    // per-marker tolerance `t` corresponds to a loss of sum_i(w_i^2 * t^2),
    // which after the RMS-1 renormalisation is exactly numMarkers * t^2.
    auto lossBoundForResidual = [&](double residualMeters) -> s_t {
        return (s_t)((double)markerList.size() * residualMeters * residualMeters);
    };

    try {
        math::IKConfig config;
        config.setLossLowerBound(lossBoundForResidual(kIKMarkerToleranceMeters));

        // Warm start: seed the solve with the previous frame's pose so the
        // solver refines it instead of re-searching from a random pose. This is
        // what keeps joint angles temporally continuous frame-to-frame; the
        // pose is written explicitly because the shared skeleton may have been
        // moved by ID or the moment-arm computer since the last IK.
        BOOL warmStarted = NO;
        if (_hasLastIKPose && _lastIKPose.size() == (int)_skeleton->getNumDofs()) {
            _skeleton->setPositions(_lastIKPose);
            config.setMaxRestarts(kIKWarmRestarts);
            warmStarted = YES;
        } else {
            config.setMaxRestarts(kIKColdRestarts);
        }

        double error = _dofMaskActive
            ? [self solveMaskedIKWithMarkers:markerList
                             targetPositions:targetPositions
                               markerWeights:weights
                                      config:config]
            : _skeleton->fitMarkersToWorldPositions(
                  markerList, targetPositions, weights, false, config);

        if (warmStarted && error > lossBoundForResidual(kIKWarmStartRejectMeters)) {
            // The previous pose was not a usable seed — fall back to a cold
            // search for this one frame.
            math::IKConfig coldConfig;
            coldConfig.setLossLowerBound(lossBoundForResidual(kIKMarkerToleranceMeters));
            coldConfig.setMaxRestarts(kIKColdRestarts);
            error = _dofMaskActive
                ? [self solveMaskedIKWithMarkers:markerList
                                 targetPositions:targetPositions
                                   markerWeights:weights
                                          config:coldConfig]
                : _skeleton->fitMarkersToWorldPositions(
                      markerList, targetPositions, weights, false, coldConfig);
        }

        // Extract joint angles
        Eigen::VectorXs positions = _skeleton->getPositions();
        _lastIKPose = positions;
        _hasLastIKPose = YES;
        NSMutableArray<NSNumber *> *angles = [NSMutableArray arrayWithCapacity:positions.size()];
        for (int i = 0; i < positions.size(); i++) {
            [angles addObject:@(positions(i))];
        }

        return [[NimbleIKResult alloc] initWithAngles:angles
                                                error:error
                                              numDOFs:positions.size()
                                             dofNames:self.dofNames];
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
        const double CONTACT_THRESHOLD = 0.06;  // 6 cm
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
