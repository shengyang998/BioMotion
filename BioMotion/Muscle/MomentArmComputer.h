#import <Foundation/Foundation.h>

@class NimbleBridge;

NS_ASSUME_NONNULL_BEGIN

/// Parsed muscle path point (body name + local offset).
/// For a MovingPathPoint the offset is a snapshot evaluated at the skeleton's
/// pose when the snapshot was taken, not a fixed attachment.
@interface MusclePathPoint : NSObject
@property (nonatomic, readonly) NSString *bodyName;
@property (nonatomic, readonly) double x, y, z;  // Local offset in body frame (meters)
@end

/// Parsed muscle with path geometry for moment arm computation.
@interface MusclePathData : NSObject
@property (nonatomic, readonly) NSString *name;
@property (nonatomic, readonly) NSArray<MusclePathPoint *> *pathPoints;
@property (nonatomic, readonly) double maxIsometricForce;
@property (nonatomic, readonly) double optimalFiberLength;
@property (nonatomic, readonly) double pennationAngle;
@end

/// What the parser actually managed to reproduce of a model's muscle path
/// geometry. Every field here is a place where the shipped geometry can be
/// LESS than what the .osim describes; without the report those shortfalls
/// are invisible and show up much later as wrong moment arms.
///
/// Counts cover only the muscles that survive parsing (≥2 usable path points
/// and F_max > 0), i.e. they describe the geometry actually in use, not the
/// raw element count in the file.
@interface MusclePathFidelityReport : NSObject

/// Muscles retained after parsing.
@property (nonatomic, readonly) NSInteger musclesParsed;

/// Plain `<PathPoint>` vertices included in the retained paths.
@property (nonatomic, readonly) NSInteger pathPointsParsed;

/// `<ConditionalPathPoint>` vertices included in the retained paths.
@property (nonatomic, readonly) NSInteger conditionalPathPointsParsed;

/// `<ConditionalPathPoint>` vertices dropped (missing parent frame).
@property (nonatomic, readonly) NSInteger conditionalPathPointsSkipped;

/// Subset of `conditionalPathPointsParsed` whose gating coordinate does not
/// resolve to a skeleton DOF. Those points are treated as UNCONDITIONALLY
/// active — the range is never evaluated. Non-zero means the path is a
/// superset of the real one at extreme poses.
@property (nonatomic, readonly) NSInteger conditionalPathPointsUnresolvedCoordinate;

/// `<MovingPathPoint>` vertices included in the retained paths.
@property (nonatomic, readonly) NSInteger movingPathPointsParsed;

/// Subset of `movingPathPointsParsed` where at least one location component is
/// a cubic spline (`SimmSpline` / `NaturalCubicSpline` / `GCVSpline`) with ≥3
/// control points. Those are evaluated by LINEAR interpolation between the
/// spline's control points (clamped outside the knot span), not by evaluating
/// the cubic. The error is bounded by the spline's curvature between knots.
@property (nonatomic, readonly) NSInteger movingPathPointsApproximated;

/// `<MovingPathPoint>` vertices dropped — unsupported location function, or a
/// driving coordinate that does not resolve to a skeleton DOF (we refuse to
/// substitute a fabricated coordinate value).
@property (nonatomic, readonly) NSInteger movingPathPointsSkipped;

/// Child elements of `<PathPointSet>/<objects>` whose tag we do not recognise.
@property (nonatomic, readonly) NSInteger unknownPathPointElementsSkipped;

/// `<PathWrap>` references on the retained muscles that are SOLVED — the path
/// wraps around the surface instead of cutting through it. `WrapCylinder` and
/// `WrapEllipsoid` (`hybrid` method) are solved
/// (`MusclePathWrap.cpp`, ported from opensim-core); that is all 76 of
/// FullBody.osim's references and all 46 of Rajagopal2016's.
@property (nonatomic, readonly) NSInteger solvedPathWraps;

/// `<PathWrap>` references that are NOT solved, so those muscles still take a
/// straight-line shortcut where the real path wraps around bone. Non-zero means
/// the model is geometrically incomplete.
///
/// Since 2026-08-08 this is the count of the wraps that REMAIN unmodelled, not
/// of all wraps, and on both shipped models it is **0**. What can still land
/// here: a `WrapObject` subclass with no solver (sphere, torus), an ellipsoid
/// `<PathWrap>` whose `<method>` is not `hybrid` (see DEVIATION 8 in
/// `MusclePathWrap.cpp`), a `<wrap_object>` naming something the model does not
/// define, and a path too long for the solver's fixed storage. A muscle carrying
/// one solved and one unsolved wrap counts here and appears in
/// `musclesWithUnmodelledPathWraps`, because a partly-wrapped path is not a
/// wrapped path.
@property (nonatomic, readonly) NSInteger unmodelledPathWraps;

/// **Which muscles those wraps belong to**, in parse order. A count alone
/// cannot answer the only question the UI has to answer — "is THIS muscle's
/// number on a different scale from that one" — and the answer decides what a
/// screen showing named muscles is allowed to claim. See
/// `GaitLoadSummary.musclesWithUnmodelledPaths`, which is checked against this
/// list by `MomentArmTests`.
@property (nonatomic, readonly) NSArray<NSString *> *musclesWithUnmodelledPathWraps;

/// `<WrapObject>`s parsed off the model's bodies and available to solve.
@property (nonatomic, readonly) NSInteger wrapObjectsParsed;

/// `<WrapObject>`s the parser refused: an unsupported subclass, a body the
/// skeleton does not carry, a `<quadrant>` spelling OpenSim would have thrown
/// on, a non-positive cylinder radius, or a non-positive ellipsoid semi-axis
/// (OpenSim throws at load on that one). A `PathWrap` pointing at one of these counts
/// as unmodelled — silently wrapping on a guessed side is the exact failure
/// this project keeps paying for.
@property (nonatomic, readonly) NSInteger wrapObjectsRejected;

/// Muscles whose `tendon_slack_length` was missing or unparseable and fell
/// back to 0. Downstream that makes fiber length = L_MT / cos(α), which
/// permanently mis-scales that muscle's force-length and force-velocity curves.
@property (nonatomic, readonly) NSArray<NSString *> *musclesWithDefaultedTendonSlackLength;

/// One-line human-readable digest, also written to the log at parse time.
@property (nonatomic, readonly) NSString *summary;

@end

/// Computes moment arms from muscle path geometry using numerical differentiation
/// of musculotendon length with respect to joint angles: r = -dL/dq.
///
/// Uses the Nimble skeleton's forward kinematics to transform muscle attachment
/// points to world coordinates and compute total muscle-tendon path length.
///
/// Path WRAPPING is applied where the model defines it: a `<PathWrap>` naming a
/// `WrapCylinder` or a `WrapEllipsoid` makes the path run around the surface
/// instead of through it (`MusclePathWrap.h`, ported from opensim-core). Every
/// `PathWrap` in both shipped models is solved; anything that is not — an
/// unsupported surface, or an ellipsoid whose `<method>` is not `hybrid` — stays
/// counted in `MusclePathFidelityReport.unmodelledPathWraps` rather than being
/// approximated. Because only the path's scalar LENGTH is differentiated, only
/// the length of the wrapped path has to be right; the tangent-point geometry
/// OpenSim also computes for rendering is not reproduced.
///
/// ConditionalPathPoint gating: the coordinate value fed to a conditional
/// point's `<range>` test is the CURRENT POSITION OF THE MATCHING SKELETON DOF
/// — the same pose used for FK, never an assumed zero. It is latched once per
/// `computeMomentArmsWithJointAngles:dofNames:` / `currentMuscleLengths` call
/// and held fixed across the finite-difference perturbations, so a via point
/// switching on inside the ±eps stencil cannot inject a step discontinuity
/// into dL/dq. If the gating coordinate is not a DOF of this skeleton the
/// point is kept unconditionally and counted in the fidelity report.
@interface MomentArmComputer : NSObject

/// Parse muscle path geometry from a .osim file and adopt the skeleton
/// from a pre-loaded NimbleBridge. This avoids re-parsing the .osim and,
/// crucially, uses the SAME skeleton instance that NimbleBridge has
/// already scaled via `scaleModelWithHeight:` — otherwise per-segment
/// scaling never propagates to moment-arm / muscle-length computation.
///
/// @param path  Path to the .osim file (read for muscle GeometryPath
///              definitions only; the skeleton is NOT re-parsed).
/// @param bridge A NimbleBridge whose model has already been loaded.
///              Its `sharedSkeleton` must be non-null.
/// @return YES if muscles were parsed successfully.
- (BOOL)parseMusclePathsFromOsimPath:(NSString *)path
                          fromBridge:(NimbleBridge *)bridge;

/// Number of muscles with parsed paths.
@property (nonatomic, readonly) NSInteger numMuscles;

/// Geometry fidelity of the most recent parse. Never nil — before any parse it
/// is an all-zero report.
@property (nonatomic, readonly) MusclePathFidelityReport *fidelityReport;

/// Compute the moment arm matrix R(q) at the current skeleton configuration.
/// Returns a flat row-major matrix [nMuscles x nDOFs] where R[i][j] = -dL_i/dq_j.
///
/// @param jointAngles Current joint angles (from IK).
/// @param dofNames DOF names corresponding to the angle array.
/// @return Flat array of moment arms (nMuscles * nDOFs), or nil on failure.
- (nullable NSArray<NSNumber *> *)computeMomentArmsWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                                          dofNames:(NSArray<NSString *> *)dofNames;

/// Current musculotendon path lengths L_MT(q) for all parsed muscles, at the
/// skeleton's CURRENT pose (as left by the most recent call to
/// computeMomentArmsWithJointAngles:dofNames:). Lengths are in meters.
/// Returns an array of size numMuscles.
@property (nonatomic, readonly) NSArray<NSNumber *> *currentMuscleLengths;

/// Maximum isometric force F_max for each parsed muscle, in newtons.
/// Array of size numMuscles, in the same order as currentMuscleLengths.
@property (nonatomic, readonly) NSArray<NSNumber *> *maxIsometricForces;

/// Optimal fiber length l_opt for each muscle, in meters.
@property (nonatomic, readonly) NSArray<NSNumber *> *optimalFiberLengths;

/// Tendon slack length l_Ts for each muscle, in meters. Falls back to
/// 0.0 if the underlying path did not parse this field.
@property (nonatomic, readonly) NSArray<NSNumber *> *tendonSlackLengths;

/// Pennation angle α₀ at optimal fiber length, in radians.
@property (nonatomic, readonly) NSArray<NSNumber *> *pennationAngles;

/// Get the muscle path data for a specific muscle.
- (nullable MusclePathData *)musclePathDataForName:(NSString *)name;

/// How many `<PathWrap>` references this muscle carries, solvable or not.
/// -1 when the muscle is not in the parsed set.
///
/// This is a structural fact about the model with a numerical consequence:
/// OpenSim solves a ONE-wrap path in closed form and an N-wrap path by
/// re-solving the whole set up to 8 times, and the two code paths do not agree
/// to the same tolerance. Anything comparing this implementation with OpenSim
/// has to stratify on it.
- (NSInteger)pathWrapCountForMuscleNamed:(NSString *)name;

/// How many of that muscle's `<PathWrap>` references name a `WrapEllipsoid`.
/// -1 when the muscle is not in the parsed set.
///
/// Also a structural fact with a numerical consequence, and a much larger one:
/// the cylinder spiral is closed form while the ellipsoid path is a fan of ~300
/// point-to-ellipsoid Newton solves followed by two Levenberg-Marquardt tangent
/// solves and a chorded geodesic. Any comparison with OpenSim has to stratify on
/// which surface it is looking at, and this is the parser's own answer rather
/// than a hand-written muscle list that can drift away from the code.
- (NSInteger)ellipsoidPathWrapCountForMuscleNamed:(NSString *)name;

/// All muscle names.
@property (nonatomic, readonly) NSArray<NSString *> *muscleNames;

/// How many wrap points the solver inserted for each muscle at the skeleton's
/// CURRENT pose — 2 per engaged wrap object, 0 when nothing wraps. Same
/// quantity as the reference fixture's `wrapPoints` column, so the two can be
/// compared directly. Ordered like `muscleNames`.
@property (nonatomic, readonly) NSArray<NSNumber *> *currentWrapPointCounts;

/// # dL/dq is DISCONTINUOUS where a muscle starts or stops wrapping
///
/// A centred difference straddling that switch divides a finite jump in L by
/// `2·eps` and returns it as a moment arm — metres, not centimetres, and it
/// looks entirely plausible. `computeMomentArmsWithJointAngles:dofNames:`
/// therefore compares the wrap solver's discrete state (`WrappedPathResult`'s
/// signature: which objects engaged, on which segment, which branch) at
/// `q`, `q+eps` and `q−eps`, and drops to a ONE-SIDED difference on the side
/// that stays on the base pose's branch. These three counters are that
/// decision, over the most recent call, and they sum to nMuscles × nDOFs.
@property (nonatomic, readonly) NSInteger lastCentredDifferenceSamples;

/// Samples where the wrap state changed on one side of the stencil, so a
/// one-sided difference on the other side was used instead.
@property (nonatomic, readonly) NSInteger lastOneSidedDifferenceSamples;

/// Samples where the base pose sits alone — the wrap state differs on BOTH
/// sides, i.e. the switch is at `q` itself. The step is halved up to 8 times to
/// find a side that agrees; this counts the ones where none ever did and the
/// forward difference at the smallest step was used. Expected to be 0.
@property (nonatomic, readonly) NSInteger lastUnresolvedDiscontinuitySamples;

/// How many ellipsoid wrap solves refused during the most recent
/// `computeMomentArmsWithJointAngles:dofNames:` — cases OpenSim answers with a
/// NaN (an exactly tangent segment, a negative discriminant on the surface
/// ray). Those segments took the straight line. Expected to be 0; a non-zero
/// value is a pose to report, not to average away.
@property (nonatomic, readonly) NSInteger lastEllipsoidNumericalRefusals;

/// Switch every parsed `WrapEllipsoid` on or off, exactly as
/// `<active>false</active>` in the model would, and return how many changed.
///
/// It exists for ONE reason: the per-frame cost of a wrap solver is an A/B that
/// has to happen inside one process, on one machine, at one pose set, or it is
/// two numbers from two runs being subtracted. The ellipsoid is 130–450× the
/// cost of a cylinder solve, so this is the difference that decides whether it
/// can ship.
- (NSInteger)setEllipsoidWrapObjectsActive:(BOOL)active;

/// World-space start/end positions of each muscle path at the current skeleton
/// pose. Returned as a flat `[x0, y0, z0, x1, y1, z1, ...]` array of 6 floats
/// per muscle, ordered to match `muscleNames`. This skips through-path
/// wrapping and treats the first and last PathPoints as the visual endpoints —
/// good enough for an overlay capsule; moment-arm computation still uses the
/// full path.
@property (nonatomic, readonly) NSArray<NSNumber *> *muscleEndpointsWorld;

@end

NS_ASSUME_NONNULL_END
