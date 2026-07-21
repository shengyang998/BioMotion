#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/// Result from muscle static optimization.
@interface MuscleActivationResult : NSObject
@property (nonatomic, readonly) NSArray<NSString *> *muscleNames;
@property (nonatomic, readonly) NSArray<NSNumber *> *activations;  // 0-1 per muscle
@property (nonatomic, readonly) NSArray<NSNumber *> *forces;       // Newtons per muscle
@property (nonatomic, readonly) double solveTimeMs;
@property (nonatomic, readonly) BOOL converged;

/// ‖A_eff·a − τ‖₂ in Nm, evaluated on the activations actually returned
/// (post-clamp) and over the muscled DOFs only — unmuscled DOFs are not
/// part of the objective and would otherwise dominate this number.
///
/// The torque match is a SOFT penalty, not a constraint, so `converged`
/// only says OSQP reached its tolerance on the objective it was given; it
/// says nothing about whether the physics was satisfied. A solve can trade
/// the whole torque residual away to shrink the ‖a‖² regularizer, and this
/// is the only signal that distinguishes those two outcomes.
@property (nonatomic, readonly) double torqueResidualNm;

/// `torqueResidualNm` normalized by ‖τ‖₂, with the denominator floored at
/// 1e-6 Nm so the ratio is never a division by zero. Reading:
///   ≤ 0.05    trustworthy — the muscle set reproduces the ID torques;
///   0.05–0.3  partial — usually activation saturation (a hit 1) or a DOF
///             the loaded model has no muscle capacity for;
///   > 0.3     do not read the activations as biomechanics; the
///             regularizer, not the torque balance, chose them.
/// On a near-zero-torque frame this ratio is not meaningful (the activation
/// floor guarantees some residual torque) — read `torqueResidualNm` there.
@property (nonatomic, readonly) double relativeTorqueResidual;
@end

/// Solves the muscle static optimization problem:
/// Given joint torques, find muscle activations that minimize sum(a^2)
/// subject to moment arm constraints and activation bounds.
@interface MuscleSolver : NSObject

/// Parse muscle definitions from a .osim model file.
/// @param path Path to the .osim file.
/// @return YES if muscles were parsed successfully.
- (BOOL)loadMusclesFromOsimPath:(NSString *)path;

/// Number of muscles loaded.
@property (nonatomic, readonly) NSInteger numMuscles;

/// Names of all loaded muscles.
@property (nonatomic, readonly) NSArray<NSString *> *muscleNames;

/// Lower bound imposed on every activation by the optimizer, representing
/// resting postural tone. Exposed so a display layer can apply its own
/// (different) visual floor instead of inheriting the optimizer's bound.
@property (nonatomic, readonly) double minActivation;

/// Production muscle static optimization.
///
/// Solves the soft-equality QP:
///   min  ½ aᵀPa + ½ λ ‖R·diag(F_max·f_AL·f_FV·cos(α))·a − τ‖²
///   s.t. a_min ≤ a ≤ 1
///
/// with per-muscle F_max and Hill-model force-length / force-velocity
/// multipliers computed from the REAL current musculotendon length and
/// its first time derivative (both supplied by the caller from a
/// FK-driven moment arm computer). Properties of this formulation:
///   1. no hardcoded moment arms — R comes from outside;
///   2. never goes infeasible — the torque constraint is a soft
///      quadratic penalty rather than an equality;
///   3. computes real normalized fiber length / velocity per frame.
///
/// @param jointTorques    Joint torques from ID (Nm), indexed by dofNames.
/// @param momentArms      Flat row-major [nMuscles × nDOFs] matrix, metres.
/// @param muscleNames     Names of muscles, length nMuscles.
/// @param muscleLengths   Current L_MT(q) for each muscle (m), length nMuscles.
/// @param maxForces       F_max for each muscle (N), length nMuscles.
/// @param optimalFiberLengths l_opt for each muscle (m), length nMuscles.
/// @param tendonSlackLengths  l_Ts for each muscle (m), length nMuscles.
/// @param pennationAngles α₀ for each muscle (rad), length nMuscles.
/// @param jointVelocities dq for each DOF (rad/s), length dofNames.count.
///                        Fiber velocity is obtained analytically from these
///                        via dL_MT/dt = −Rᵀ·dq. Pass an EMPTY array only if
///                        no velocity estimate exists; the solver then falls
///                        back to finite-differencing L_MT across frames,
///                        which is sensitive to dropped frames.
/// @param dofNames        DOF names, ordering matches jointTorques columns.
/// @param dt              Time since the previous solve (s). Used ONLY by the
///                        empty-jointVelocities fallback to finite-difference
///                        L_MT; ignored when jointVelocities is supplied.
/// @param softPenalty     Soft-equality weight λ. Larger → forces tau match
///                        at the expense of activations; smaller → looser.
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
                softPenalty:(double)softPenalty;

@end

NS_ASSUME_NONNULL_END
