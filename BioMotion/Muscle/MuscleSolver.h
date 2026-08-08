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
/// (post-clamp), over the ROTATIONAL rows of the QP only.
///
/// TWO ROW GROUPS ARE EXCLUDED AND THE REASONS ARE DIFFERENT.
///
/// 1. Coordinates no muscle crosses are not in the QP at all. Their
///    generalised force is carried by the skeleton, and no activation vector
///    can change it, so including them would add a constant no solver can
///    reduce. In `FullBody.osim` that is exactly 10 of 169 coordinates: the
///    six floating-base coordinates (nothing can pull on the root from
///    outside the body) and `wrist_flex_{r,l}` / `wrist_dev_{r,l}`, which
///    have no muscles in the model. Measured separation is 9 decades —
///    every other coordinate has some muscle with |r| > 1e-3 m, and these
///    ten peak at 3e-12 — so `kMomentArmFloor` is reading a structural gap,
///    not a tuned threshold.
///
/// 2. TRANSLATIONAL coordinates are in the QP but NOT in this number,
///    because their generalised force is a FORCE IN NEWTONS, not a moment.
///    `FullBody.osim` has six: `pelvis_t{x,y,z}` (excluded anyway by rule 1)
///    and `Sternum{X,Y,Z}`. The last three are real and load-bearing —
///    `SternumY` reads 72.697 N in every standing pose, which is exactly
///    9.81 × 7.4104 kg, the weight of the shoulder girdle and both arms,
///    since `sterR_clavR_jnt` and `clavR_scapR_jnt` are WeldJoints and the
///    sternocostal joint is the whole upper limb's only load path. Summing
///    that square with newton-metre squares is a unit error; see
///    `forceResidualN`.
///
/// The torque match is a SOFT penalty, not a constraint, so `converged`
/// only says OSQP reached its tolerance on the objective it was given; it
/// says nothing about whether the physics was satisfied.
@property (nonatomic, readonly) double torqueResidualNm;

/// `torqueResidualNm` normalized by ‖τ‖₂ over the same rotational rows, with
/// the denominator floored at 1e-6 Nm so the ratio is never a division by
/// zero. Reading:
///   ≤ 0.05    trustworthy — the muscle set reproduces the ID torques;
///   0.05–0.3  partial;
///   > 0.3     do not read the activations as biomechanics.
/// On a near-zero-torque frame this ratio is not meaningful (the activation
/// floor guarantees some residual torque) — read `torqueResidualNm` there.
///
/// WHAT THIS NUMBER IS **NOT**. Measured 2026-08-07 on `FullBody.osim`:
/// sweeping the soft-penalty weight λ from 1 to 1e8 moves it by at most 11%,
/// and UPWARD, so it is not reporting how the objective was weighted. It is
/// the distance from τ to the set reachable by ANY admissible activation
/// vector, A_eff·[a_min, 1]^520.
///
/// WHERE IT COMES FROM, in neutral standing. `a_min` is a LOWER bound, so
/// A_eff·(a_min·1) is a moment field the solve can add to but never remove.
/// That field measures 10.89 Nm on the 72 costovertebral coordinates, whose
/// own demand is 1.68 Nm — and the achieved residual there is 9.89 Nm. In
/// other words essentially the whole standing residual is the activation
/// floor, projected onto coordinates whose musculature is too one-sided to
/// cancel it. On the coordinates the product actually reports (hip, knee,
/// ankle, shoulder, intervertebral levels) the same floor field is 23.3 Nm
/// against a 38.5 Nm demand and the optimizer cancels it down to 2.66 Nm, a
/// relative residual of 0.069.
///
/// This matters because STATUS.md records `a_min` as having been chosen so the
/// visualisation would not go "permanently blue" — a rendering parameter.
/// Lowering it to make this number look better would be tuning a constant, so
/// it has NOT been done. The falsifiable claim is the one above: if the
/// costovertebral rows' residual ever stops tracking the floor field, the
/// explanation is wrong.
@property (nonatomic, readonly) double relativeTorqueResidual;

/// ‖A_eff·a − τ‖₂ in NEWTONS over the translational rows of the QP (see
/// `torqueResidualNm` note 2). Zero rows ⇒ 0. Kept separate rather than
/// dropped: those rows carry the entire upper limb.
@property (nonatomic, readonly) double forceResidualN;

/// `forceResidualN` normalized by ‖τ‖₂ over the same translational rows,
/// denominator floored at 1e-6 N. Same reading scale as
/// `relativeTorqueResidual`. 0 when there are no translational rows.
@property (nonatomic, readonly) double relativeForceResidual;
@end

/// Solves the muscle static optimization problem:
/// Given joint torques, find muscle activations that minimize sum(a^2)
/// subject to moment arm constraints and activation bounds.
@interface MuscleSolver : NSObject

/// Parse muscle definitions from a .osim model file.
/// @param path Path to the .osim file.
/// @return YES if muscles were parsed successfully.
- (BOOL)loadMusclesFromOsimPath:(NSString *)path;

/// Drops every piece of state that carries from one solve to the next, so the
/// next solve is identical to the first solve after `loadMusclesFromOsimPath:`.
///
/// # Why a muscle solver has session state at all, and why it must be droppable
///
/// The QP is warm-started from the previous frame's activations, and 520
/// muscles over ~169 coordinates leave a wide null space of equally optimal
/// answers — so *where OSQP stops* depends on where it started. Within one clip
/// that is exactly what a warm start is for. ACROSS clips it means the same
/// video, imported twice in one app session, publishes different per-muscle
/// numbers and therefore a different "N% harder on the left". The product's
/// whole deliverable is a comparison, so an answer that depends on what was
/// analysed before it is not an answer.
///
/// Three things carry, and all three are dropped here:
///   1. `_prevActivations`, the primal warm start;
///   2. the OSQP workspace itself, which keeps its own primal AND DUAL iterate
///      across `osqp_solve` calls (`warm_starting = true`, and the caller only
///      ever overrides the primal) — so resetting the activation vector alone
///      would leave `y` from the previous clip. The workspace is torn down and
///      rebuilt on the next solve, which costs one KKT factorization per clip.
///   3. `_prevMuscleLengths`, the finite-difference history behind the
///      wall-clock fiber-velocity fallback, which would otherwise difference
///      the new clip's first frame against the old clip's last pose.
///
/// Called by `NimbleEngine.resetSessionState()` at a clip boundary. Safe to
/// call before any model is loaded.
- (void)resetSessionState;

/// Number of muscles loaded.
@property (nonatomic, readonly) NSInteger numMuscles;

/// Names of all loaded muscles.
@property (nonatomic, readonly) NSArray<NSString *> *muscleNames;

/// Lower bound imposed on every activation by the optimizer, representing
/// resting postural tone. Exposed so a display layer can apply its own
/// (different) visual floor instead of inheriting the optimizer's bound.
@property (nonatomic, readonly) double minActivation;

/// Upper bound imposed on every activation. A muscle at this value is CLIPPED:
/// the QP has stopped being linear in the external load for it, which is the
/// one place a common ground-force scale stops cancelling out of a ratio.
@property (class, nonatomic, readonly) double maxActivation;

/// **How far below `maxActivation` a genuinely clipped activation can come
/// back.** Test for saturation with `a >= maxActivation - this`, never with a
/// finer band.
///
/// OSQP terminates on a tolerance, not at the exact vertex: the primal check is
/// `eps_abs + eps_rel·max(‖Ax‖∞, ‖z‖∞)`, and with `A = I` and `z ∈ [aMin, 1]`
/// that is `eps_abs + eps_rel`. This solver also accepts
/// `OSQP_SOLVED_INACCURATE`, which is the same check with both tolerances
/// multiplied by ten, and it runs with `polishing = false` so nothing snaps the
/// solution back onto the active set. Published rather than restated at the
/// call site because a display layer that hard-codes its own band drifts from
/// the solver the moment either tolerance moves.
@property (class, nonatomic, readonly) double saturationActivationTolerance;

/// Coordinates the loaded model declares `<locked>true</locked>`. Empty until
/// `loadMusclesFromOsimPath:` succeeds. 54 for `FullBody.osim`.
@property (nonatomic, readonly) NSArray<NSString *> *lockedCoordinateNames;

/// Coordinates of the loaded model whose value is a DISPLACEMENT, i.e. every
/// `<TransformAxis>` naming them is a `translationN` axis. Empty until
/// `loadMusclesFromOsimPath:` succeeds. Six for `FullBody.osim`.
/// `knee_angle_{r,l}` is deliberately absent: it drives coupled translation
/// splines but is itself a `rotation1` axis, so its generalised force is a
/// moment.
@property (nonatomic, readonly) NSArray<NSString *> *translationalCoordinateNames;

/// Whether coordinates in `lockedCoordinateNames` are left out of the
/// torque-matching penalty. Default YES.
///
/// A locked coordinate is one the model author declared immobile, so its
/// generalised force is carried by that constraint, not by muscle. nimble
/// does not implement `<locked>`, so IK moves those coordinates anyway and
/// inverse dynamics reports a generalised force at them; without this the QP
/// is asked to produce it with muscle. In `FullBody.osim` the set is 48
/// costovertebral coordinates (whose rib bodies weigh 0.0001 kg each, so the
/// demand there is ~0 and the penalty is really "do not net-rotate a massless
/// locked rib"), plus `mtp_angle_{r,l}` and the four wrist coordinates.
///
/// Exposed as a switch so its effect stays separable from the unit split in
/// `MuscleActivationResult`. Measured on `FullBody.osim`, mixed-unit basis:
/// neutral standing 0.1245 → 0.0938; dancer 0.6233 → 0.6150.
@property (nonatomic) BOOL excludesLockedCoordinates;

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
