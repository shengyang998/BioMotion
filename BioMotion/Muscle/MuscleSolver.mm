#import "MuscleSolver.h"
#import <QuartzCore/QuartzCore.h>

#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>

#include <Eigen/Core>
#include <tinyxml2.h>
#include "osqp.h"


// MARK: - Millard2012 Force-Length-Velocity Model

namespace muscle {

struct MuscleParams {
    std::string name;
    double maxIsometricForce;      // F0, Newtons
    double optimalFiberLength;     // l0, meters
    double tendonSlackLength;      // lTs, meters
    double pennationAngleAtOptimal; // alpha0, radians
    double maxContractionVelocity; // vmax, l0/s (default 10)
};

// Millard2012 active force-length curve (normalized)
// All muscles use the same default parameters
static double activeForceLengthCurve(double normFiberLength) {
    // Gaussian-like curve centered at 1.0
    // Simplified Millard2012: bell curve from 0.5 to 1.5
    if (normFiberLength < 0.25 || normFiberLength > 1.9) return 0.0;

    // Piecewise approximation of the Millard curve
    double x = normFiberLength;
    if (x < 0.5) {
        return 0.0;
    } else if (x < 0.77) {
        // Shallow ascending
        double t = (x - 0.5) / (0.77 - 0.5);
        return 0.75 * t * t; // quadratic ramp
    } else if (x < 1.0) {
        // Steep ascending
        double t = (x - 0.77) / (1.0 - 0.77);
        return 0.75 * (1.0 - t * t) + t * t; // blend to 1.0
    } else if (x < 1.2) {
        // Peak plateau
        double t = (x - 1.0) / (1.2 - 1.0);
        return 1.0 - 0.1 * t; // slight decline
    } else {
        // Descending
        double t = (x - 1.2) / (1.9 - 1.2);
        return std::max(0.0, 0.9 * (1.0 - t));
    }
}

// Millard2012 force-velocity curve (normalized)
static double forceVelocityCurve(double normFiberVelocity) {
    // normFiberVelocity: -1 (max shortening) to +1 (max lengthening)
    // Returns force multiplier: 0 at max shortening, 1 at isometric,
    // ~1.17 at max lengthening.
    //
    // The multiplier MUST stay ≥ 0: it feeds forceScale = F_max·f_AL·f_FV·cosα
    // and hence A_eff = R·forceScale, so a negative value would tell the QP
    // that activating a muscle produces torque in the direction OPPOSITE to
    // its moment arm — the solver then recruits fast-shortening muscles
    // backwards to cancel torque.
    double v = std::clamp(normFiberVelocity, -1.0, 1.0);

    if (v <= 0) {
        // Concentric (shortening): Hill's hyperbola in normalized form,
        //   f_FV(ṽ) = (1 + ṽ) / (1 − ṽ/A_f),  A_f = 0.25,
        // which is Hill's (F + a)(v + b) = const rewritten with ṽ = v/v_max
        // and a/F₀ = A_f. It is 0 at ṽ = −1, 1 at ṽ = 0, non-negative and
        // strictly increasing throughout (df/dṽ = (1 + 1/A_f)/(1 − ṽ/A_f)² > 0).
        const double Af = 0.25;  // Hill shape factor, Millard2012 default
        return (1.0 + v) / (1.0 - v / Af);
    } else {
        // Eccentric (lengthening): rises above isometric and stays bounded
        // in [1, ~1.17] over ṽ ∈ (0, 1]; never negative.
        return 1.0 + v * 0.4 * (1.0 - 0.6 * v);
    }
}

} // namespace muscle

// MARK: - .osim Muscle Parser

static std::vector<muscle::MuscleParams> parseMusclesFromOsim(const std::string& path) {
    std::vector<muscle::MuscleParams> muscles;

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        NSLog(@"MuscleSolver: Failed to load .osim file");
        return muscles;
    }

    // Navigate to ForceSet
    auto* model = doc.FirstChildElement("OpenSimDocument");
    if (!model) model = doc.FirstChildElement("Model");
    if (model) {
        auto* modelEl = model->FirstChildElement("Model");
        if (modelEl) model = modelEl;
    }
    if (!model) return muscles;

    auto* forceSet = model->FirstChildElement("ForceSet");
    if (!forceSet) return muscles;
    auto* objects = forceSet->FirstChildElement("objects");
    if (!objects) return muscles;

    // Iterate both Millard2012 and Thelen2003 muscle classes — full-body
    // models routinely mix the two.
    static const char* const muscleClasses[] = {
        "Millard2012EquilibriumMuscle",
        "Thelen2003Muscle",
        nullptr,
    };
    for (int ci = 0; muscleClasses[ci]; ci++) {
        for (auto* muscleEl = objects->FirstChildElement(muscleClasses[ci]);
             muscleEl;
             muscleEl = muscleEl->NextSiblingElement(muscleClasses[ci])) {

            muscle::MuscleParams params;
            params.maxContractionVelocity = 10.0; // default

            const char* name = muscleEl->Attribute("name");
            if (name) params.name = name;

            auto readDouble = [&](const char* tag) -> double {
                auto* el = muscleEl->FirstChildElement(tag);
                if (el && el->GetText()) return std::stod(el->GetText());
                return 0.0;
            };

            params.maxIsometricForce = readDouble("max_isometric_force");
            params.optimalFiberLength = readDouble("optimal_fiber_length");
            params.tendonSlackLength = readDouble("tendon_slack_length");
            params.pennationAngleAtOptimal = readDouble("pennation_angle_at_optimal");

            auto* vel = muscleEl->FirstChildElement("max_contraction_velocity");
            if (vel && vel->GetText()) params.maxContractionVelocity = std::stod(vel->GetText());

            if (params.maxIsometricForce > 0 && params.optimalFiberLength > 0) {
                muscles.push_back(params);
            }
        }
    }

    return muscles;
}

// MARK: - MuscleActivationResult

@implementation MuscleActivationResult {
    NSArray<NSString *> *_muscleNames;
    NSArray<NSNumber *> *_activations;
    NSArray<NSNumber *> *_forces;
    double _solveTimeMs;
    BOOL _converged;
    double _torqueResidualNm;
    double _relativeTorqueResidual;
}

- (instancetype)initWithNames:(NSArray<NSString *> *)names
                   activations:(NSArray<NSNumber *> *)activations
                        forces:(NSArray<NSNumber *> *)forces
                   solveTimeMs:(double)solveTimeMs
                     converged:(BOOL)converged
              torqueResidualNm:(double)torqueResidualNm
        relativeTorqueResidual:(double)relativeTorqueResidual {
    self = [super init];
    if (self) {
        _muscleNames = [names copy];
        _activations = [activations copy];
        _forces = [forces copy];
        _solveTimeMs = solveTimeMs;
        _converged = converged;
        _torqueResidualNm = torqueResidualNm;
        _relativeTorqueResidual = relativeTorqueResidual;
    }
    return self;
}

- (NSArray<NSString *> *)muscleNames { return _muscleNames; }
- (NSArray<NSNumber *> *)activations { return _activations; }
- (NSArray<NSNumber *> *)forces { return _forces; }
- (double)solveTimeMs { return _solveTimeMs; }
- (BOOL)converged { return _converged; }
- (double)torqueResidualNm { return _torqueResidualNm; }
- (double)relativeTorqueResidual { return _relativeTorqueResidual; }

@end

// MARK: - MuscleSolver

// Lower bound on every activation. Skeletal muscle is never electrically
// silent under postural load — stance muscles hold roughly 2-5% activation
// at rest — so a floor of 0.02 keeps the optimizer inside the physiological
// range instead of driving redundant muscles to exactly zero.
// This is an optimizer bound, not a display choice; see `minActivation`.
static const double kMuscleMinActivation = 0.02;

@implementation MuscleSolver {
    std::vector<muscle::MuscleParams> _muscles;

    // Previous activations for warm starting
    std::vector<double> _prevActivations;

    // Previous-frame musculotendon lengths. Only the wall-clock fallback
    // path (caller supplied no joint velocities) finite-differences these;
    // they are kept up to date on every frame so the fallback stays usable
    // if velocities disappear mid-stream.
    // Keyed by muscle name (insertion order from the caller) so the caller's
    // muscle ordering can change between frames without corrupting history.
    std::map<std::string, double> _prevMuscleLengths;

    // Set once the first time the wall-clock fallback is taken, so the
    // warning is not emitted every frame.
    BOOL _loggedVelocityFallback;

    // Persistent OSQP workspace for the production solver. Kept alive
    // across frames so `osqp_update_data_vec` / `osqp_update_data_mat` can
    // replace the expensive `osqp_setup` + KKT factorization with a much
    // cheaper in-place update. Reset whenever nMuscles changes.
    OSQPSolver* _realSolver;
    NSInteger _realSolverNMuscles;
    // Retained storage backing the OSQP matrix pointers — OSQP does not
    // copy the input arrays, so these buffers must outlive the solver.
    std::vector<OSQPInt> _realP_col_ptr;
    std::vector<OSQPInt> _realP_row_idx;
    std::vector<OSQPFloat> _realP_val;
    std::vector<OSQPInt> _realA_col_ptr;
    std::vector<OSQPInt> _realA_row_idx;
    std::vector<OSQPFloat> _realA_val;
    std::vector<OSQPFloat> _realQ;
    std::vector<OSQPFloat> _realL;
    std::vector<OSQPFloat> _realU;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _realSolver = nullptr;
        _realSolverNMuscles = 0;
        _loggedVelocityFallback = NO;
    }
    return self;
}

- (void)dealloc {
    if (_realSolver) {
        osqp_cleanup(_realSolver);
    }
}

- (BOOL)loadMusclesFromOsimPath:(NSString *)path {
    _muscles = parseMusclesFromOsim(std::string([path UTF8String]));
    _prevActivations.assign(_muscles.size(), 0.01);
    NSLog(@"MuscleSolver: Loaded %lu muscles", (unsigned long)_muscles.size());
    return !_muscles.empty();
}

- (NSInteger)numMuscles {
    return (NSInteger)_muscles.size();
}

- (NSArray<NSString *> *)muscleNames {
    NSMutableArray *names = [NSMutableArray array];
    for (const auto& m : _muscles) {
        [names addObject:[NSString stringWithUTF8String:m.name.c_str()]];
    }
    return names;
}

- (double)minActivation {
    return kMuscleMinActivation;
}

// MARK: - Production solver (real moment arms, soft equality, real Hill)

- (nullable MuscleActivationResult *)
  solveRealWithJointTorques:(NSArray<NSNumber *> *)jointTorques
                 momentArms:(NSArray<NSNumber *> *)momentArms
                muscleNames:(NSArray<NSString *> *)muscleNames
              muscleLengths:(NSArray<NSNumber *> *)muscleLengths
                  maxForces:(NSArray<NSNumber *> *)maxForces
        optimalFiberLengths:(NSArray<NSNumber *> *)optimalFiberLengths
         tendonSlackLengths:(NSArray<NSNumber *> *)tendonSlackLengths
            pennationAngles:(NSArray<NSNumber *> *)pennationAngles
            jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                   dofNames:(NSArray<NSString *> *)dofNames
                         dt:(double)dt
                softPenalty:(double)softPenalty {

    const NSInteger nMuscles = muscleNames.count;
    const NSInteger nDOFs = dofNames.count;

    if (nMuscles == 0 || nDOFs == 0) return nil;
    if (momentArms.count != (NSUInteger)(nMuscles * nDOFs)) {
        NSLog(@"MuscleSolver: moment arm matrix has wrong size (%lu vs %ld)",
              (unsigned long)momentArms.count, (long)(nMuscles * nDOFs));
        return nil;
    }
    if (muscleLengths.count != (NSUInteger)nMuscles ||
        maxForces.count != (NSUInteger)nMuscles ||
        optimalFiberLengths.count != (NSUInteger)nMuscles ||
        tendonSlackLengths.count != (NSUInteger)nMuscles ||
        pennationAngles.count != (NSUInteger)nMuscles ||
        jointTorques.count != (NSUInteger)nDOFs) {
        NSLog(@"MuscleSolver: input array size mismatch");
        return nil;
    }
    // An EMPTY jointVelocities array is legal and selects the wall-clock
    // fallback below; any other wrong length is a caller bug.
    if (jointVelocities.count != 0 && jointVelocities.count != (NSUInteger)nDOFs) {
        NSLog(@"MuscleSolver: jointVelocities has wrong size (%lu vs %ld)",
              (unsigned long)jointVelocities.count, (long)nDOFs);
        return nil;
    }

    double startTime = CACurrentMediaTime();

    // R (nMuscles × nDOFs, row-major): R[m, j] at flat index (m * nDOFs + j).
    // Needed both for the fiber-velocity identity below and for the QP.
    auto R = [&](NSInteger m, NSInteger j) -> double {
        return [momentArms[m * nDOFs + j] doubleValue];
    };

    // --- 1. Per-muscle Hill state (force-length + force-velocity) ---
    // L_fiber = (L_MT - L_Ts) / cos(α₀)        (rigid-tendon approximation)
    // L̃      = L_fiber / l_opt                (normalized)
    // dL_MT/dt = −Rᵀ·dq                        (analytic, see below)
    // V_max   = 10 · l_opt                     (default from Millard2012)
    // Ṽ       = (dL_MT/dt) / V_max              (approximation: ignores tendon velocity)
    //
    // Max force at current state:
    //   F_max_state = F_max · f_AL(L̃) · f_FV(Ṽ) · cos(α₀)
    //
    // We enforce this as the only per-frame state-dependent scaling. The
    // full equilibrium-tendon model requires solving an implicit equation
    // per muscle per frame, which is too slow for realtime — rigid-tendon
    // is the standard approximation for online muscle optimization.
    //
    // SIGN CONVENTION for the velocity identity. R here is the OpenSim
    // moment arm, R[m, j] = −∂L_MT_m/∂q_j (that minus sign is applied by
    // MomentArmComputer, and it is what makes the torque relation used in
    // step 2, τ_j = Σ_m R[m, j]·F_m, correct by virtual work). Differentiating
    // L_MT along the trajectory therefore gives
    //     dL_MT_m/dt = Σ_j (∂L_MT_m/∂q_j)·dq_j = −Σ_j R[m, j]·dq_j,
    // i.e. dL_MT/dt = −Rᵀ·dq. Positive R with positive dq means the joint is
    // rotating the way the muscle pulls it, so the muscle SHORTENS.
    //
    // This replaces finite-differencing L_MT against the previous solved
    // frame: that estimate divided by a wall-clock dt that jitters whenever
    // a frame is dropped, and disagreed with the Savitzky-Golay filter that
    // produced dq in the first place. Both derivative pipelines now describe
    // the same kinematic event.

    const BOOL haveJointVelocities = (jointVelocities.count == (NSUInteger)nDOFs);
    std::vector<double> dq;
    if (haveJointVelocities) {
        dq.resize(nDOFs);
        for (NSInteger j = 0; j < nDOFs; j++) {
            dq[j] = [jointVelocities[j] doubleValue];
        }
    } else if (!_loggedVelocityFallback) {
        _loggedVelocityFallback = YES;
        NSLog(@"MuscleSolver: no joint velocities supplied — falling back to "
              @"wall-clock finite differencing of L_MT for fiber velocity "
              @"(sensitive to dropped frames). Logged once.");
    }

    std::vector<double> forceScale(nMuscles);  // F_max * f_AL * f_FV * cos(α)
    std::vector<double> normFiberLength(nMuscles);
    std::vector<double> normFiberVelocity(nMuscles);

    for (NSInteger m = 0; m < nMuscles; m++) {
        double LMT = [muscleLengths[m] doubleValue];
        double lopt = [optimalFiberLengths[m] doubleValue];
        double lTs  = [tendonSlackLengths[m] doubleValue];
        double alpha = [pennationAngles[m] doubleValue];
        double cosAlpha = std::cos(alpha);
        double Fmax = [maxForces[m] doubleValue];

        // Guard against zero / negative optimal fiber length.
        if (lopt <= 1e-6) {
            forceScale[m] = 0;
            normFiberLength[m] = 1.0;
            normFiberVelocity[m] = 0.0;
            continue;
        }

        // Rigid-tendon fiber length.
        double fiberLen = (LMT - lTs) / std::max(cosAlpha, 0.1);
        double Ltilde = fiberLen / lopt;

        // Force-length: physiological plateau is ~0.5-1.5 normalized length.
        // Clamp to [0.3, 1.8] to avoid zero force in extreme configurations
        // (early ARKit frames can produce nonsense poses).
        Ltilde = std::max(0.3, std::min(1.8, Ltilde));

        // dL_MT/dt, m/s along the muscle path.
        double dLdt = 0.0;
        std::string name = std::string([muscleNames[m] UTF8String]);
        if (haveJointVelocities) {
            double RtDq = 0.0;
            for (NSInteger j = 0; j < nDOFs; j++) {
                RtDq += R(m, j) * dq[j];
            }
            dLdt = -RtDq;
        } else {
            // Fallback: finite-difference against the previous solved frame.
            // First frame → assume 0.
            auto it = _prevMuscleLengths.find(name);
            if (it != _prevMuscleLengths.end() && dt > 1e-5) {
                dLdt = (LMT - it->second) / dt;
            }
        }
        _prevMuscleLengths[name] = LMT;

        double Vmax = 10.0 * lopt;              // Millard default
        double Vtilde = dLdt / std::max(Vmax, 1e-6);
        // Clamp to avoid edge singularities in Hill curves.
        Vtilde = std::max(-1.0, std::min(1.0, Vtilde));

        double fAL = muscle::activeForceLengthCurve(Ltilde);
        double fFV = muscle::forceVelocityCurve(Vtilde);

        forceScale[m] = Fmax * fAL * fFV * cosAlpha;
        normFiberLength[m] = Ltilde;
        normFiberVelocity[m] = Vtilde;
    }

    // --- 2. Build the QP ---
    // Variables: a ∈ ℝ^nMuscles.
    // Objective: ½ aᵀ (λ·RᵀR + ε·I) a − λ (τᵀ R) a  + const
    // where R_row[j] is the row of R·diag(forceScale) corresponding to DOF j.
    //
    // We split the full R matrix into ACTIVE DOFs — DOFs that have at
    // least one non-zero moment-arm entry — and only soft-penalize those.
    // DOFs with no muscles (e.g. upper body in Rajagopal2016) are ignored.
    // Unmuscled DOFs are not the muscle solver's responsibility; ID
    // already published the torques there.

    // Compute effective R*forceScale as a nDOFs × nMuscles matrix in
    // column-major for OSQP ingestion.
    // A[j, m] = R[m, j] * forceScale[m]  (if DOF j has non-zero moment)
    // We scan once to find active DOFs so we can drop empty rows.

    std::vector<int> activeDOFs;
    activeDOFs.reserve(nDOFs);
    for (NSInteger j = 0; j < nDOFs; j++) {
        bool any = false;
        for (NSInteger m = 0; m < nMuscles; m++) {
            if (std::abs(R(m, j)) > 1e-10) { any = true; break; }
        }
        if (any) activeDOFs.push_back((int)j);
    }

    if (activeDOFs.empty()) {
        NSLog(@"MuscleSolver: no active DOFs — moment arm matrix is all zero");
        return nil;
    }

    const NSInteger nActive = (NSInteger)activeDOFs.size();

    // Build A_eff (nActive × nMuscles): A_eff[k, m] = R[m, dof_k] * fs[m]
    // Target τ_eff[k] = jointTorques[dof_k]
    Eigen::MatrixXd Aeff(nActive, nMuscles);
    Eigen::VectorXd tauEff(nActive);
    for (NSInteger k = 0; k < nActive; k++) {
        int j = activeDOFs[k];
        tauEff(k) = [jointTorques[j] doubleValue];
        for (NSInteger m = 0; m < nMuscles; m++) {
            Aeff(k, m) = R(m, j) * forceScale[m];
        }
    }

    // Hessian for the full QP objective:
    //   ½ aᵀ (ε I + λ Aᵀ A) a − λ (τᵀ A) a
    // Gradient contribution:  g = − λ Aᵀ τ
    // Note OSQP requires P to be upper triangular.

    // Weight split: the soft-equality QP solves
    //   min ½ aᵀ (εI + λ AᵀA) a − λ τᵀ A a
    // We want τ matching to DOMINATE the activation L2 regularizer — otherwise
    // ~520 muscles of redundancy cause the solver to flatten every a toward
    // aMin, making visualization permanently blue. εA=0.01 keeps a² as a
    // tie-breaker among redundant muscle sets; λ is tuned at the call site
    // (NimbleEngine passes 100) so τ-match is the primary objective.
    const double epsA = 0.01;
    const double lambda = softPenalty;

    Eigen::MatrixXd P = epsA * Eigen::MatrixXd::Identity(nMuscles, nMuscles)
                      + lambda * Aeff.transpose() * Aeff;
    Eigen::VectorXd g = -lambda * Aeff.transpose() * tauEff;

    // Bounds: aMin ≤ a ≤ 1. aMin reflects physiological postural tone —
    // stance muscles maintain ~2-5% activation at rest, so zero activation
    // is not a state a loaded muscle occupies.
    const double aMin = kMuscleMinActivation;
    const double aMax = 1.0;

    // --- 3. OSQP setup or in-place update (workspace reuse) ---
    //
    // If nMuscles hasn't changed since the last call and we still have a
    // live solver, we update the existing workspace in-place via the
    // `osqp_update_data_*` APIs. This skips the expensive KKT
    // factorization that `osqp_setup` performs, which is by far the
    // dominant cost in a per-frame solve. If the dimensions changed (e.g.
    // a new osim model was loaded) or no solver exists yet, we do a full
    // setup and allocate fresh backing buffers.
    //
    // NOTE: OSQP does not copy input arrays into its own storage — it
    // holds pointers into the user-provided buffers. The `_realP_*` etc.
    // ivars keep those buffers alive for the lifetime of the solver so
    // repeated solves don't read freed memory.

    const bool dimsChanged = (_realSolver == nullptr) ||
                             (_realSolverNMuscles != nMuscles);

    if (dimsChanged) {
        // Tear down any stale solver and rebuild the CSC buffers from scratch.
        if (_realSolver) { osqp_cleanup(_realSolver); _realSolver = nullptr; }

        _realP_col_ptr.assign(nMuscles + 1, 0);
        _realP_row_idx.clear();
        _realP_val.clear();
        _realP_row_idx.reserve(nMuscles * (nMuscles + 1) / 2);
        _realP_val.reserve(nMuscles * (nMuscles + 1) / 2);
        for (NSInteger col = 0; col < nMuscles; col++) {
            _realP_col_ptr[col] = (OSQPInt)_realP_row_idx.size();
            for (NSInteger row = 0; row <= col; row++) {
                double v = P(row, col);
                _realP_row_idx.push_back((OSQPInt)row);
                _realP_val.push_back((OSQPFloat)v);
            }
        }
        _realP_col_ptr[nMuscles] = (OSQPInt)_realP_row_idx.size();

        _realA_col_ptr.assign(nMuscles + 1, 0);
        _realA_row_idx.assign(nMuscles, 0);
        _realA_val.assign(nMuscles, 1.0);
        for (NSInteger i = 0; i < nMuscles; i++) {
            _realA_col_ptr[i] = (OSQPInt)i;
            _realA_row_idx[i] = (OSQPInt)i;
        }
        _realA_col_ptr[nMuscles] = (OSQPInt)nMuscles;

        _realQ.assign(nMuscles, 0);
        _realL.assign(nMuscles, (OSQPFloat)aMin);
        _realU.assign(nMuscles, (OSQPFloat)aMax);
        for (NSInteger i = 0; i < nMuscles; i++) _realQ[i] = (OSQPFloat)g(i);

        OSQPCscMatrix P_csc, A_csc;
        OSQPCscMatrix_set_data(&P_csc, (OSQPInt)nMuscles, (OSQPInt)nMuscles,
                               (OSQPInt)_realP_val.size(),
                               _realP_val.data(),
                               _realP_row_idx.data(),
                               _realP_col_ptr.data());
        OSQPCscMatrix_set_data(&A_csc, (OSQPInt)nMuscles, (OSQPInt)nMuscles,
                               (OSQPInt)_realA_val.size(),
                               _realA_val.data(),
                               _realA_row_idx.data(),
                               _realA_col_ptr.data());

        OSQPSettings settings;
        osqp_set_default_settings(&settings);
        settings.max_iter = 200;
        settings.eps_abs = 1e-3;
        settings.eps_rel = 1e-3;
        settings.verbose = false;
        settings.warm_starting = true;
        settings.polishing = false;

        OSQPInt exitflag = osqp_setup(&_realSolver, &P_csc, _realQ.data(), &A_csc,
                                      _realL.data(), _realU.data(),
                                      (OSQPInt)nMuscles, (OSQPInt)nMuscles,
                                      &settings);
        if (exitflag != 0 || !_realSolver) {
            NSLog(@"MuscleSolver: OSQP setup failed with code %d", (int)exitflag);
            _realSolverNMuscles = 0;
            return nil;
        }
        _realSolverNMuscles = nMuscles;
    } else {
        // Same problem dimensions — update P values and q in place.
        // P has fixed sparsity (upper-triangular dense), so we just write
        // the new upper-triangular values column-by-column in the same
        // order we originally built them.
        size_t idx = 0;
        for (NSInteger col = 0; col < nMuscles; col++) {
            for (NSInteger row = 0; row <= col; row++) {
                _realP_val[idx++] = (OSQPFloat)P(row, col);
            }
        }
        for (NSInteger i = 0; i < nMuscles; i++) {
            _realQ[i] = (OSQPFloat)g(i);
        }

        // Update P values (NULL idx array = all values). l/u are
        // constant so we pass NULL for those to skip updates.
        OSQPInt pUpdate = osqp_update_data_mat(_realSolver,
                                               _realP_val.data(), OSQP_NULL,
                                               (OSQPInt)_realP_val.size(),
                                               OSQP_NULL, OSQP_NULL, 0);
        OSQPInt qUpdate = osqp_update_data_vec(_realSolver,
                                               _realQ.data(),
                                               OSQP_NULL, OSQP_NULL);
        if (pUpdate != 0 || qUpdate != 0) {
            // Fall back to full setup on update failure (should be rare).
            NSLog(@"MuscleSolver: OSQP in-place update failed (P=%d q=%d) — resetting",
                  (int)pUpdate, (int)qUpdate);
            osqp_cleanup(_realSolver);
            _realSolver = nullptr;
            _realSolverNMuscles = 0;
            return nil;
        }
    }

    // Warm start from previous frame's activations.
    if (_prevActivations.size() != (size_t)nMuscles) {
        _prevActivations.assign(nMuscles, 0.05);  // > aMin; OSQP prefers feasible warm start
    }
    osqp_warm_start(_realSolver, _prevActivations.data(), OSQP_NULL);

    osqp_solve(_realSolver);

    BOOL converged = (_realSolver->info->status_val == OSQP_SOLVED ||
                      _realSolver->info->status_val == OSQP_SOLVED_INACCURATE);

    NSMutableArray<NSString *> *outNames =
        [NSMutableArray arrayWithCapacity:nMuscles];
    NSMutableArray<NSNumber *> *outActivations =
        [NSMutableArray arrayWithCapacity:nMuscles];
    NSMutableArray<NSNumber *> *outForces =
        [NSMutableArray arrayWithCapacity:nMuscles];

    Eigen::VectorXd aOut(nMuscles);
    for (NSInteger m = 0; m < nMuscles; m++) {
        double a = converged
            ? std::clamp((double)_realSolver->solution->x[m], aMin, aMax)
            : aMin;
        double f = a * forceScale[m];
        [outNames addObject:muscleNames[m]];
        [outActivations addObject:@(a)];
        [outForces addObject:@(f)];
        _prevActivations[m] = a;
        aOut(m) = a;
    }

    // --- 4. Torque residual of the solution we are actually returning ---
    // The τ match is a penalty term, not a constraint, so `converged` alone
    // cannot distinguish "muscles reproduce the ID torques" from "the ‖a‖²
    // regularizer won and the torques were abandoned". Evaluate the residual
    // on the post-clamp activations, over the active (muscled) DOFs only —
    // those are the rows the objective actually contains.
    const Eigen::VectorXd residual = Aeff * aOut - tauEff;
    const double residualNm = residual.norm();
    // The 1e-6 Nm floor only prevents a division by zero; it does not make
    // the ratio meaningful when ‖τ‖ is itself near zero (the aMin floor
    // guarantees some residual torque there). On such frames read
    // torqueResidualNm instead.
    const double relativeResidual = residualNm / std::max(tauEff.norm(), 1e-6);

    double solveTime = (CACurrentMediaTime() - startTime) * 1000.0;

    // Keep _realSolver alive for the next frame.
    return [[MuscleActivationResult alloc] initWithNames:outNames
                                             activations:outActivations
                                                  forces:outForces
                                             solveTimeMs:solveTime
                                               converged:converged
                                        torqueResidualNm:residualNm
                                  relativeTorqueResidual:relativeResidual];
}

@end
