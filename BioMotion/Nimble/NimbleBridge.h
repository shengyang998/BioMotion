#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/// Provenance of the ground-plane height currently used for contact detection.
/// Callers that care about GRF validity should gate on `groundHeightTrusted`
/// rather than on this enum directly; the enum exists for diagnostics.
typedef NS_ENUM(NSInteger, NimbleGroundHeightSource) {
    /// No foot-height sample has been observed and no calibration has been
    /// applied. `groundHeightY` is a placeholder (0) and means nothing.
    NimbleGroundHeightSourceUncalibrated = 0,
    /// Rolling estimate computed over fewer samples than the estimator needs
    /// to reject outliers. Usable, but a handful of bad frames can still move
    /// it, so contact detection may be wrong.
    NimbleGroundHeightSourceProvisional,
    /// Rolling low-percentile estimate over a full-enough sample window.
    NimbleGroundHeightSourceEstimated,
    /// Pinned by `setGroundHeightY:`. Observed foot heights never move an
    /// explicitly-calibrated ground plane; only `resetSessionState` releases it.
    NimbleGroundHeightSourceExplicit,
};

/// Result from inverse kinematics solve.
@interface NimbleIKResult : NSObject
@property (nonatomic, readonly) NSArray<NSNumber *> *jointAngles;  // DOF values in radians
@property (nonatomic, readonly) double error;  // RMS marker error in meters
@property (nonatomic, readonly) NSInteger numDOFs;
@property (nonatomic, readonly) NSArray<NSString *> *dofNames;
@end

/// Result from inverse dynamics solve.
@interface NimbleIDResult : NSObject
@property (nonatomic, readonly) NSArray<NSNumber *> *jointTorques;  // Nm per DOF

// Ground reaction force diagnostics (valid only for ID-with-GRF variants).
// World-frame 3-vectors in newtons and meters respectively. Zero vectors when
// the foot was not in contact on this frame.
@property (nonatomic, readonly) NSArray<NSNumber *> *leftFootForce;   // [fx, fy, fz] N
@property (nonatomic, readonly) NSArray<NSNumber *> *rightFootForce;  // [fx, fy, fz] N
@property (nonatomic, readonly) NSArray<NSNumber *> *leftFootCoP;     // [x, y, z] m
@property (nonatomic, readonly) NSArray<NSNumber *> *rightFootCoP;    // [x, y, z] m
@property (nonatomic, readonly) BOOL leftFootInContact;
@property (nonatomic, readonly) BOOL rightFootInContact;
/// Linear-momentum residual in NEWTONS: ‖ΣF_contact + m·g − m·a_com‖, with the
/// contact forces taken from the solved wrenches after they are mapped out of
/// body-local coordinates.
///
/// This is a self-consistency check on the readback, NOT evidence that the pose
/// is balanced. Nimble solves the six floating-base equations exactly, so a
/// correct pipeline reports ~0 here for every frame, balanced or not. It goes
/// nonzero when the contact wrenches are being interpreted in the wrong frame,
/// when the root joint is not a free joint (nimble then returns all zeros), or
/// when gravity is not what the model expects.
///
/// It replaced a read of `jointTorques.head<6>()`, which nimble unconditionally
/// `setZero()`s at the end of the solve (Skeleton.cpp:10365) with its assert
/// compiled out of the Release static libs — that field was a hard-coded zero
/// and had been mistaken for proof of static equilibrium.
@property (nonatomic, readonly) double rootResidualNorm;
@end

/// C++ bridge to nimblephysics IK and ID solvers.
@interface NimbleBridge : NSObject

/// Load an OpenSim .osim model from a file path.
/// @param path Path to the .osim file.
/// @return YES if the model was loaded successfully.
- (BOOL)loadModelFromPath:(NSString *)path;

/// Get the number of degrees of freedom in the loaded model.
@property (nonatomic, readonly) NSInteger numDOFs;

/// Get the names of all DOFs in the loaded model.
@property (nonatomic, readonly) NSArray<NSString *> *dofNames;

/// Get marker names defined in the loaded model.
@property (nonatomic, readonly) NSArray<NSString *> *markerNames;

/// Scale the model to match a person's body proportions.
/// @param height Body height in meters.
/// @param markerPositions Flat array of marker 3D positions [x0,y0,z0, x1,y1,z1, ...] in meters.
/// @param markerNames Names of the markers corresponding to positions.
- (BOOL)scaleModelWithHeight:(double)height
             markerPositions:(NSArray<NSNumber *> *)markerPositions
                 markerNames:(NSArray<NSString *> *)markerNames;

/// Run inverse kinematics: given 3D marker positions, solve for joint angles.
/// @param markerPositions Flat array of marker 3D positions [x0,y0,z0, x1,y1,z1, ...] in meters.
/// @param markerNames Names of the markers corresponding to positions.
/// @return IK result with joint angles and error, or nil on failure.
- (nullable NimbleIKResult *)solveIKWithMarkerPositions:(NSArray<NSNumber *> *)markerPositions
                                            markerNames:(NSArray<NSString *> *)markerNames;

/// Run inverse dynamics: given joint angles and accelerations, solve for joint torques.
/// @param jointAngles Current joint angles (from IK).
/// @param jointVelocities Current joint velocities (finite difference from IK).
/// @param jointAccelerations Current joint accelerations (finite difference from velocities).
/// @return ID result with joint torques, or nil on failure.
- (nullable NimbleIDResult *)solveIDWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                   jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                               jointAccelerations:(NSArray<NSNumber *> *)jointAccelerations;

/// Run inverse dynamics with automatic ground reaction force estimation.
///
/// Detects which feet are in contact with the ground (based on `calcn_l` and
/// `calcn_r` body position versus the current ground height) and uses
/// Nimble's multi-contact near-CoP ID solver to decompose the system wrench
/// into per-foot GRFs + joint torques. This is the physically correct way
/// to run ID for any scenario where the subject has ground contact (standing,
/// walking, squatting, sit-to-stand). Use `solveIDWithJointAngles:...` only
/// for pure flight-phase motions.
///
/// @param jointAngles       Smoothed joint angles from IK (q).
/// @param jointVelocities   Smoothed joint velocities (dq), temporally aligned with q.
/// @param jointAccelerations Smoothed joint accelerations (ddq), aligned with q.
/// @return An NimbleIDResult with jointTorques populated plus per-foot
///         force / CoP / contact-state fields. Returns nil on failure.
- (nullable NimbleIDResult *)solveIDGRFWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                       jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                                    jointAccelerations:(NSArray<NSNumber *> *)jointAccelerations;

/// Sets the ground plane y-coordinate in the ARKit world frame. Call once
/// after calibration (or let it auto-calibrate from observed foot heights).
/// Ground is assumed flat; M4+ may support tilted floors.
///
/// An explicitly-set ground plane is pinned: the auto-estimator keeps
/// collecting samples but will not overwrite the calibrated value until
/// `resetSessionState` is called.
- (void)setGroundHeightY:(double)y;

/// Feeds one observation of the lowest foot height (ARKit world-frame y, in
/// meters) into the rolling ground-height estimator.
///
/// `solveIDGRFWithJointAngles:...` calls this itself with min(calcn_l.y,
/// calcn_r.y) every frame, so normal callers never need to. It is public so
/// that a caller with a better contact cue (e.g. a depth-derived floor plane)
/// can drive the same estimator, and so the estimator is testable without a
/// full IK/ID frame.
///
/// The estimate is a low percentile of a bounded window of recent samples, so
/// it tolerates transient dips (one bad frame, a landing spike, a momentary
/// tracking glitch) without permanently ratcheting the floor downwards, and it
/// can rise again when ARKit's world origin drifts upwards. Non-finite samples
/// are ignored.
- (void)observeLowestFootHeightY:(double)y;

/// Current ground height used for contact detection.
@property (nonatomic, readonly) double groundHeightY;

/// Whether a ground height has been explicitly set or auto-calibrated.
/// YES as soon as any estimate exists, including an untrustworthy one.
@property (nonatomic, readonly) BOOL groundHeightCalibrated;

/// Where `groundHeightY` came from.
@property (nonatomic, readonly) NimbleGroundHeightSource groundHeightSource;

/// Whether `groundHeightY` is good enough to gate physics on. YES only for an
/// explicitly-calibrated plane or a rolling estimate backed by enough samples
/// to survive outliers. When NO, foot-contact decisions (and therefore the GRF
/// decomposition) should be treated as unverified.
@property (nonatomic, readonly) BOOL groundHeightTrusted;

/// Whether the next IK solve will be warm-started from the previously solved
/// pose instead of doing a cold search with random restarts. NO before the
/// first successful solve of a session and immediately after
/// `resetSessionState`.
@property (nonatomic, readonly) BOOL ikWarmStartAvailable;

/// Drops all per-session state that is only valid while one continuous body
/// track is in view: the ground-height sample window (including any explicit
/// calibration) and the IK warm-start pose.
///
/// Must be called whenever body tracking is lost or the AR session's world
/// origin is re-established, otherwise the ground estimate describes a floor
/// that no longer exists in the current world frame and the IK warm start is a
/// pose belonging to a different subject/placement.
- (void)resetSessionState;

/// Restrict IK to a subset of the model's coordinates, holding the named ones
/// fixed for the duration of every subsequent solve.
///
/// This is a *runtime* restriction, not a model edit: the skeleton keeps all
/// of its DOFs, no joint is welded, the .osim file is never rewritten, and
/// `clearDOFMask` restores the unmasked solve exactly. Inverse dynamics, the
/// moment-arm computer and the muscle QP are unaffected — they continue to see
/// the full coordinate vector — so masking cannot silently change a reported
/// muscle moment arm.
///
/// Note that this could NOT be expressed through `math::IKConfig`: that struct
/// has no DOF-selection field (see `IKSolver.hpp:16-42`). The mask is applied
/// by reparameterising the solve, which is why it lives here rather than in a
/// config object.
///
/// @param dofNamesToMask Coordinate names as reported by `dofNames`. Names that
///        do not resolve to a DOF in the loaded model are ignored.
/// @return How many names actually matched a DOF. 0 leaves the mask inactive.
- (NSInteger)applyDOFMaskWithNames:(NSArray<NSString *> *)dofNamesToMask;

/// Drop any active DOF mask and return to solving all `numDOFs` coordinates.
- (void)clearDOFMask;

/// Whether a DOF mask is currently restricting IK.
@property (nonatomic, readonly) BOOL isDOFMaskActive;

/// Number of coordinates IK actually optimises. Equals `numDOFs` when no mask
/// is active.
@property (nonatomic, readonly) NSInteger numFreeDOFs;

/// Names of the currently masked coordinates. Empty when no mask is active.
@property (nonatomic, readonly) NSArray<NSString *> *maskedDOFNames;

/// Whether a model is currently loaded.
@property (nonatomic, readonly) BOOL isModelLoaded;

/// Total mass of the current skeleton (after scaling), in kilograms.
/// Returns 0 if no model is loaded.
@property (nonatomic, readonly) double totalMass;

@end

NS_ASSUME_NONNULL_END
