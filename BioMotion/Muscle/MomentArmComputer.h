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

/// `<PathWrap>` references found on the retained muscles. Wrap-object geometry
/// (WrapCylinder / WrapEllipsoid) is NOT implemented: those muscles take a
/// straight-line shortcut where the real path wraps around bone. Non-zero
/// means the model is geometrically incomplete.
@property (nonatomic, readonly) NSInteger unmodelledPathWraps;

/// **Which muscles those wraps belong to**, in parse order. A count alone
/// cannot answer the only question the UI has to answer — "is THIS muscle's
/// number on a different scale from that one" — and the answer decides what a
/// screen showing named muscles is allowed to claim. See
/// `GaitLoadSummary.musclesWithUnmodelledPaths`, which is checked against this
/// list by `MomentArmTests`.
@property (nonatomic, readonly) NSArray<NSString *> *musclesWithUnmodelledPathWraps;

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

/// All muscle names.
@property (nonatomic, readonly) NSArray<NSString *> *muscleNames;

/// World-space start/end positions of each muscle path at the current skeleton
/// pose. Returned as a flat `[x0, y0, z0, x1, y1, z1, ...]` array of 6 floats
/// per muscle, ordered to match `muscleNames`. This skips through-path
/// wrapping and treats the first and last PathPoints as the visual endpoints —
/// good enough for an overlay capsule; moment-arm computation still uses the
/// full path.
@property (nonatomic, readonly) NSArray<NSNumber *> *muscleEndpointsWorld;

@end

NS_ASSUME_NONNULL_END
