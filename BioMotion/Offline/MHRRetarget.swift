import Foundation
import simd

/// Retarget from SAM 3D Body / MHR output to a BioMotion `BodyFrame`.
///
/// The whole integration surface is this: `NimbleEngine.processFrame` takes a plain
/// `BodyFrame` (Models/BodyJoint.swift) and has zero ARKit dependency, so producing a
/// `BodyFrame` whose `TrackedJoint.id`s are the twenty ARKit joint id strings in
/// `JointMapping.primary` makes the entire downstream pipeline (IK, Savitzky-Golay, ID,
/// moment arms, muscle QP, MuscleOverlay) work unchanged.
///
/// # Input contract
/// `jointCoords` is the exported `joint_coords` output: 127 MHR joint positions in METERS
/// in the MHR-NATIVE frame, i.e. the values BEFORE the `[..., [1, 2]] *= -1` flip that the
/// released python applies at `sam_3d_body/models/heads/mhr_head.py:353` and
/// `models/meta_arch/sam3d_body.py:1618`.
///
/// # Coordinate frame — VERIFIED, NOT ASSUMED
/// The MHR-native frame is **X = image-right, Y = up, Z = toward the camera**, right-handed,
/// which is the same handedness and up-axis ARKit hands BioMotion. So `axisTransform` below
/// is the identity. Evidence (see labs/sam-3d-body/export/retarget/):
///
///  1. Source: the model projects with `u = f·X/Z + w/2`, `v = f·Y/Z + h/2` on the FLIPPED
///     coords (sam3d_body.py:1627-1645). That makes the flipped frame OpenCV
///     (X-right / Y-down / Z-away), hence the un-flipped frame X-right / Y-up / Z-toward-camera.
///     Un-flipping negates two axes, which is a 180° rotation about X — it changes the up
///     axis, it does NOT change handedness.
///  2. Measured on real predictions (6 runs, 5 repo images): head Y > pelvis Y > ankle Y on
///     every upright subject (dancer: head +0.512 m above pelvis, pelvis +0.498 m above ankles).
///  3. Mirror detector: the four pelvis-rigid joints (root, l_upleg, r_upleg, c_spine0) satisfy
///     `worldOffsets = M · restOffsets` with `M = R·S`. `det(M) > 0` on all 6 runs
///     (+0.42 … +1.48); deliberately negating X drives it to −0.727. No hidden reflection.
///  4. `det[subject_right, pelvis_up, pelvis_anterior] = −1.0000` on all 6 runs — the sign a
///     real (chiral) human must have in any right-handed frame.
///  5. Left/right anchored externally, not by trusting MHR's own names:
///     (a) the MHR `l_*`/`r_*` joints project to exactly the same pixels (0 px for
///         shoulders/hips/knees) as the model's own COCO-named `left-*`/`right-*` 2D
///         keypoints (sam_3d_body/metadata/mhr70.py), and COCO defines left/right as the
///         SUBJECT's anatomical side;
///     (b) on `notebook/images/dancing.jpg` the subject demonstrably faces image-right
///         (throat/jacket front on the image-right of the head, hair behind on the left,
///         knee raised forward-right) and the model places her RIGHT hip/shoulder nearer the
///         camera (Z_r = +0.065 / +0.123 vs Z_l = −0.058 / −0.083) — which is what
///         `right = forward × up` requires;
///     (c) mirroring the input image swaps the labels as physics requires (mean residual
///         0.017 m swapped vs 0.628 m unswapped).
///
/// # The root translation: `cam_t` is the quantity `joint_coords` is missing
/// `joint_coords` has `global_trans` zeroed (sam3d_body.py:1600), so the pelvis is pinned at
/// exactly (0, 0.924, 0) in EVERY prediction — that 0.924 is a model constant, not the
/// subject's pelvis height, and y = 0 is NOT the floor. The model does not throw the
/// translation away though: it emits it separately as `cam_t`, which this app already
/// exports, already stores on `FrameResult`, and already consumes in `projectToImage`.
///
/// `makeBodyFrame(jointCoords:camT:…)` composes it back in. **VERIFIED, 2026-08-07**, by
/// running the shipped Core ML model over 309 frames of a real clip
/// (`labs/sam-3d-body/export/camt_probe.py`):
///
///  * `cam_t` is the CLIFF/CameraHMR full-frame root translation
///    (`sam_3d_body/models/heads/camera_head.py:84-96`):
///    `tz = 2·f/(bbox_side·s)`, `cx = 2(bbox_cx − w/2)/bs`, `cam_t = [tx+cx, ty+cy, tz]`.
///  * The numbers are physically right: subject at 4.34 m depth, 1.10 m below the optical
///    axis, ±0.37 m lateral, and `corr(1/bbox_side, depth) = +0.74` exactly as that formula
///    requires.
///
/// ⚠️ **It is a POSITION, not an acceleration.** Its depth channel is derived from apparent
/// size and carries 12.7 cm (std) / 68 cm (max) of high-frequency residual about its own
/// 0.5 s mean. Pushed through the app's 9-tap Savitzky-Golay filter that is **3.1 g of pure
/// noise at 30 fps**, 0.56 g at 15, 0.23 g at 10, 0.02 g at 2. The in-plane channels are
/// 8-25× cleaner (1% person-box jitter moves `cam_t` by x 2.2 mm / y 2.6 mm / **z 18.9 mm**).
/// Temporally smoothing the person box cut the box's own wobble 12× and the depth residual
/// only 28%, so the noise is the model's monocular depth, not preprocessing. See STATUS.md,
/// "cam_t recovers the root translation; its depth cannot be differentiated twice".
///
/// # What this file deliberately does NOT do
///  * It does not floor-align. Even with `camT` composed in, y = 0 is the camera's optical
///    axis, not the ground; the ground-contact / GRF stage must keep estimating the floor.
///  * It does not smooth. Filtering is `NimbleEngine`'s job.
///  * It does not compensate camera motion. `cam_t` is camera-relative, so a camera that
///    ROTATES makes the reconstructed frame non-inertial (measured on the user's own clips:
///    13.5 °/s on `video_012`, which displaces a point 1 m from the subject by 6.4 cm over a
///    single 0.27 s filter window). Constant-velocity camera translation is Galilean and
///    harmless; camera acceleration is not, and is not measured here.
enum MHRRetarget {

    // MARK: - MHR joint indices
    //
    // Index -> name from findings/mhr_skeleton_summary.json ("joints_ordered", 127 entries).
    private enum MHR {
        static let root            = 1    // pelvis origin
        static let lUpleg          = 2    // L hip joint centre
        static let lLowleg         = 3    // L knee joint centre
        static let lFoot           = 4    // L ankle joint centre
        static let lBall           = 8    // L MTP (toe) joint centre
        static let rUpleg          = 18
        static let rLowleg         = 19
        static let rFoot           = 20
        static let rBall           = 24
        static let cSpine1         = 35
        static let cSpine2         = 36
        static let cSpine3         = 37
        static let rUparm          = 39   // R glenohumeral joint centre
        static let rLowarm         = 40   // R elbow joint centre
        static let rWrist          = 42   // R wrist (zero-length child of r_wrist_twist)
        static let lUparm          = 75
        static let lLowarm         = 76
        static let lWrist          = 78
        static let cNeck           = 110
        static let cHead           = 113  // skull base
        static let cHeadNull       = 126  // top of skull, 0.19076 m along the head axis
        static let jointCount      = 127
    }

    // MARK: - Axis transform
    //
    // THE ONE PLACE an axis correction would live. It is the identity because the frame check
    // above passed: MHR-native is already X-right / Y-up / Z-toward-camera, right-handed, the
    // same convention ARKit hands BioMotion. If that ever has to change, change it HERE and
    // nowhere else — every marker in this file goes through `mhrToARKit`.
    static let axisTransform = simd_float3x3(diagonal: SIMD3<Float>(repeating: 1))

    @inline(__always)
    static func mhrToARKit(_ p: SIMD3<Float>) -> SIMD3<Float> {
        axisTransform * p
    }

    // MARK: - The 20-joint table

    /// One BioMotion marker's recipe.
    ///
    /// The marker position is `lerp(jointCoords[mhrJointIndex], jointCoords[blendJointIndex],
    /// blendT) + localOffset`. `blendT == 0` means "use `mhrJointIndex` verbatim".
    ///
    /// Why blending instead of a constant `localOffset` where MHR and OpenSim disagree:
    /// the two skeletons have different trunk proportions (findings/retarget_correspondence.json
    /// measures MHR pelvis→T1 at 0.797× the OpenSim one), so a metric offset copied off the
    /// unscaled OpenSim model is wrong for any subject who is not that model's size, and it
    /// also does not rotate with the segment. A fraction along an existing MHR bone tracks
    /// both the subject's size and the segment's orientation. `localOffset` is kept in the
    /// table because it is part of the frozen deliverable shape and is the right escape hatch
    /// if a purely translational correction is ever needed; all twenty rows are currently zero.
    struct JointSource {
        let arkitJointId: String
        let displayName: String
        let opensimMarker: String
        let mhrJointIndex: Int
        let blendJointIndex: Int
        let blendT: Float
        let localOffset: SIMD3<Float>

        init(_ arkitJointId: String,
             _ displayName: String,
             _ opensimMarker: String,
             _ mhrJointIndex: Int,
             blendTo blendJointIndex: Int? = nil,
             t blendT: Float = 0,
             offset localOffset: SIMD3<Float> = .zero) {
            self.arkitJointId = arkitJointId
            self.displayName = displayName
            self.opensimMarker = opensimMarker
            self.mhrJointIndex = mhrJointIndex
            self.blendJointIndex = blendJointIndex ?? mhrJointIndex
            self.blendT = blendT
            self.localOffset = localOffset
        }
    }

    // Fractions derived by normalising both skeletons' rest heights by their own
    // pelvis→T1 length, so they are proportion-matched rather than metre-matched.
    // Recomputed in export/retarget/analyse_runs.py / README.md:
    //   MHR rest (root-relative, /0.5179):  c_spine1 0.2522  c_spine2 0.4634
    //                                       c_spine3 0.8193  c_neck 1.0000
    //                                       c_head 1.1836    c_head_null 1.5519
    //   FullBody.osim rest (/0.5217):       lumbar3 0.2774   thoracic7 0.7378
    //                                       thoracic1 1.0000 head_neck 1.0401
    //                                       head_neck+0.15Y 1.3276
    private static let tSpineL: Float = 0.119   // c_spine1 -> c_spine2, hits lumbar3
    private static let tSpineM: Float = 0.771   // c_spine2 -> c_spine3, hits thoracic7
    private static let tNeck:   Float = 0.218   // c_neck   -> c_head,   hits head_neck origin
    private static let tHead:   Float = 0.391   // c_head   -> c_head_null, hits head_neck + 0.15 Y

    /// The twenty markers `NimbleBridge` registers (NimbleBridge.mm:346-390), in the exact
    /// order of `JointMapping.primary` (Models/BodyJoint.swift:33-58).
    ///
    /// Sources: MHR joint -> OpenSim body from findings/mhr_osim_correspondence.json;
    /// OpenSim body -> marker name from the `virtualMarkers` table in NimbleBridge.mm;
    /// marker -> ARKit id from `JointMapping.primary`.
    static let table: [JointSource] = [
        // --- Pelvis ---
        // correspondence: "MHR root == pelvis origin". Direct, no judgement.
        .init("hips_joint", "Pelvis", "PELVIS", MHR.root),

        // --- Lower body: all exact joint-centre correspondences, no judgement calls ---
        // correspondence: l_upleg/r_upleg -> femur_l/femur_r, "hip joint centre"; NimbleBridge
        // registers LHJC/RHJC at the femur body origin.
        .init("left_upLeg_joint",  "L Hip", "LHJC", MHR.lUpleg),
        .init("right_upLeg_joint", "R Hip", "RHJC", MHR.rUpleg),
        // correspondence: l_lowleg/r_lowleg -> tibia_l/tibia_r, "knee joint centre".
        .init("left_leg_joint",  "L Knee", "LKJC", MHR.lLowleg),
        .init("right_leg_joint", "R Knee", "RKJC", MHR.rLowleg),
        // correspondence: l_foot/r_foot -> talus_l/talus_r, "ankle joint centre".
        // NOT l_talocrural/r_talocrural: correspondence flags those as zero-length children
        // with identical world position and zero new information.
        .init("left_foot_joint",  "L Ankle", "LAJC", MHR.lFoot),
        .init("right_foot_joint", "R Ankle", "RAJC", MHR.rFoot),
        // correspondence: l_ball/r_ball -> toes_l/toes_r, "MTP joint centre". JUDGEMENT: the
        // correspondence note warns mtp_angle_l/r are <locked>true</locked> in FullBody.osim,
        // so this marker probably buys no DOF — but the live ARKit path also supplies LTOE/RTOE
        // and dropping it would change IK conditioning relative to the live path, so keep it.
        .init("left_toes_joint",  "L Toe", "LTOE", MHR.lBall),
        .init("right_toes_joint", "R Toe", "RTOE", MHR.rBall),

        // --- Spine: the two genuinely interpolated rows ---
        // JUDGEMENT: NimbleBridge registers SPINE_L on lumbar3 (norm height 0.2774). The
        // nearest MHR joint is c_spine1 (0.2522) — 13 mm short on the MHR rest scale. The
        // correspondence file instead pairs c_spine1 with lumbar2 (0.181 m / norm 0.347); that
        // was answering "which OpenSim body does this MHR joint sit on", the opposite
        // direction, and lumbar2 is not what NimbleBridge registers. Blend 11.9% toward
        // c_spine2 to land on lumbar3.
        .init("spine_1_joint", "Lower Spine", "SPINE_L", MHR.cSpine1, blendTo: MHR.cSpine2, t: tSpineL),
        // JUDGEMENT: NimbleBridge registers SPINE_M on thoracic7 (norm 0.7378) and MHR has NO
        // joint near it — c_spine2 is 0.4634 and c_spine3 is 0.8193. 77.1% along that bone.
        // This is the weakest row in the table: it is an interpolation between two levels
        // 18 cm apart, so treat mid-thoracic angles as a prior, not a measurement.
        .init("spine_4_joint", "Mid Spine", "SPINE_M", MHR.cSpine2, blendTo: MHR.cSpine3, t: tSpineM),
        // correspondence: c_neck -> thoracic1, 4 mm gap, and thoracic1 is exactly where
        // NimbleBridge puts C7. Normalised heights agree to 4 decimal places (1.0000 vs 1.0000).
        .init("spine_7_joint", "C7", "C7", MHR.cNeck),
        // JUDGEMENT: NECK is registered at the head_neck body ORIGIN (norm 1.0401), which sits
        // between MHR c_neck (1.0000) and c_head (1.1836) — 21.8% along the neck bone.
        .init("neck_1_joint", "Neck", "NECK", MHR.cNeck, blendTo: MHR.cHead, t: tNeck),
        // JUDGEMENT: HEAD is registered at head_neck + 0.15 m local Y (norm 1.3276).
        // correspondence explicitly warns MHR c_head is the SKULL BASE, 70 mm above the
        // head_neck origin, and must not be treated as coincident. c_head is 1.1836 and
        // c_head_null (top of skull) is 1.5519, so 39.1% along the skull axis. Using
        // c_head_null makes the offset follow head orientation instead of world +Y.
        .init("head_joint", "Head", "HEAD", MHR.cHead, blendTo: MHR.cHeadNull, t: tHead),

        // --- Upper body ---
        // correspondence: l_uparm/r_uparm -> humerus_l/humerus_r, "glenohumeral joint centre";
        // NimbleBridge registers LSJC/RSJC at the humerus origin. Note l_clavicle/r_clavicle are
        // deliberately NOT used: correspondence marks them near-useless (sterL_clavL_jnt is a
        // 0-dof WeldJoint) and no BioMotion marker maps to them.
        .init("left_shoulder_1_joint",  "L Shoulder", "LSJC", MHR.lUparm),
        .init("right_shoulder_1_joint", "R Shoulder", "RSJC", MHR.rUparm),
        // correspondence: l_lowarm/r_lowarm -> ulna_l/ulna_r, "elbow joint centre".
        .init("left_forearm_joint",  "L Elbow", "LEJC", MHR.lLowarm),
        .init("right_forearm_joint", "R Elbow", "REJC", MHR.rLowarm),
        // JUDGEMENT: NimbleBridge registers LWJC/RWJC on hand_l/hand_r. correspondence maps
        // MHR l_wrist -> hand_l and notes it is a zero-length child of l_wrist_twist, so
        // l_wrist and l_wrist_twist have the SAME world position — either index gives the same
        // number. l_wrist (78) is used because it is the one the correspondence pairs with
        // hand_l. (Only the ORIENTATIONS differ, and orientations are not consumed here.)
        .init("left_hand_joint",  "L Wrist", "LWJC", MHR.lWrist),
        .init("right_hand_joint", "R Wrist", "RWJC", MHR.rWrist),
    ]

    // MARK: - Frame construction

    /// The root translation `joint_coords` is missing, in the Y-up frame this file emits.
    ///
    /// `cam_t` is expressed in the OpenCV-style camera frame (X right, Y DOWN, Z AWAY from
    /// the camera) because that is the frame the model's own projection uses:
    /// `cam = [p.x, −p.y, −p.z] + cam_t` (see `projectToImage`, and `overlay_check.py` /
    /// `projection_selfcheck.py`, which measure the projected 3-D joints against the model's
    /// own `keypoints_2d` at 1.0 px mean). Undoing that axis flip gives the translation in
    /// the Y-up frame `joint_coords` and every marker in this file live in.
    ///
    /// The resulting frame has the CAMERA at the origin looking along −Z. It is metric and
    /// (for a non-rotating camera) inertial. It is NOT floor-aligned and its Y axis is the
    /// camera's up, not gravity — a tilted phone tilts the whole reconstruction.
    static func rootTranslation(camT: SIMD3<Float>) -> SIMD3<Float> {
        mhrToARKit(SIMD3<Float>(camT.x, -camT.y, -camT.z))
    }

    /// Build a `BodyFrame` that `NimbleEngine.processFrame` can consume directly.
    ///
    /// - Parameters:
    ///   - jointCoords: 127 MHR joint positions, meters, MHR-native (un-flipped) frame.
    ///   - camT: the model's `cam_t` output for the same frame. When supplied, the root
    ///     translation is composed back in and the emitted markers are camera-frame world
    ///     positions — so the pelvis descends in a squat instead of the feet appearing to
    ///     rise, and `q̈` describes motion that actually happened. When nil (the default),
    ///     the markers stay pelvis-pinned, which is bit-for-bit the behaviour every caller
    ///     had before this parameter existed.
    /// - Returns: a frame with the twenty joints of `JointMapping.primary`, or a frame with an
    ///   empty joint list if `jointCoords` is the wrong length (`processFrame` already guards
    ///   `names.isEmpty` and returns without touching the solver).
    ///
    /// ⚠️ Supplying `camT` is necessary for dynamics but NOT sufficient — read
    /// `rootTranslation` and this type's header for the measured depth noise, and
    /// `NimbleEngine.StaticHoldDetector` for the conditions the engine then checks.
    static func makeBodyFrame(jointCoords: [SIMD3<Float>],
                              camT: SIMD3<Float>? = nil,
                              timestamp: TimeInterval,
                              frameNumber: Int) -> BodyFrame {
        guard jointCoords.count >= MHR.jointCount else {
            return BodyFrame(timestamp: timestamp, frameNumber: frameNumber, joints: [])
        }
        let offset = camT.map(rootTranslation(camT:)) ?? .zero
        let joints = table.map { src -> TrackedJoint in
            TrackedJoint(id: src.arkitJointId,
                         name: src.displayName,
                         worldPosition: markerPosition(src, in: jointCoords) + offset,
                         isTracked: true)
        }
        return BodyFrame(timestamp: timestamp, frameNumber: frameNumber, joints: joints)
    }

    /// Resolve one table row into a position, in the ARKit-facing frame, RELATIVE to the
    /// pelvis-pinned origin. Add `rootTranslation(camT:)` for a camera-frame world position.
    static func markerPosition(_ src: JointSource, in jointCoords: [SIMD3<Float>]) -> SIMD3<Float> {
        let a = jointCoords[src.mhrJointIndex]
        let b = jointCoords[src.blendJointIndex]
        let p = src.blendT == 0 ? a : a + (b - a) * src.blendT
        return mhrToARKit(p + src.localOffset)
    }

    // MARK: - Body scaling without T-pose calibration

    /// Marker set for `NimbleBridge.scaleModelWithHeight(_:markerPositions:markerNames:)`.
    ///
    /// # Why this is a synthetic straight-limb pose and not the measured markers
    /// `scaleModelWithHeight` derives its scales from STRAIGHT-LINE marker distances
    /// (`|LHJC-LAJC|`, `|LSJC-LWJC|`, `|PELVIS - mid(LSJC,RSJC)|`, NimbleBridge.mm:540-590).
    /// Those only equal the anatomical limb length when the limb is straight — which is exactly
    /// why the live path demands a T-pose. Feeding a single arbitrary MHR frame straight in
    /// FAILS, measured:
    ///
    ///   dancing.jpg  raised knee -> |LHJC-LAJC| collapses to 0.291 m (vs 0.816 m on the
    ///                               straight side); L/R mean 0.554 -> lowerScale 0.629 -> clamped
    ///   yoga         folded      -> lowerScale 0.351, upperScale 0.312 -> BOTH clamped
    ///   football     bent elbow  -> upperScale 0.633 -> clamped
    ///   All six runs measured (incl. the mirrored control) fail the clamp this way.
    ///
    /// MHR gives us the joint centres, so the pose-invariant CHAIN sum
    /// (hip→knee + knee→ankle, shoulder→elbow + elbow→wrist) is free. Rebuilding a canonical
    /// straight-limb marker set from those chain sums makes the straight-line distances the
    /// bridge measures equal the true segment lengths. Measured on the same three frames the
    /// raw markers failed on, every scale then lands inside [0.7, 1.4]:
    ///
    ///   dancing  lower 0.954  upper 0.984  trunk 1.017
    ///   yoga     lower 0.897  upper 0.921  trunk 0.911
    ///   football lower 1.011  upper 0.999  trunk 0.785
    ///   sample4  lower 0.918  upper 0.896  trunk 0.885
    ///
    /// 5 of 6 runs pass. The exception is `sample2`, a small heavily-occluded rider on a horse
    /// where the model itself produced a degenerate fit — 0.070 m hip width, 0.116 m shoulder
    /// width, 0.178 m humerus — giving upperScale 0.591, outside the clamp. This method cannot
    /// rescue a bad prediction; the clamp truncates the damage, but a caller that wants to fail
    /// loudly should gate the frame first (e.g. reject hip width outside 0.10-0.28 m or
    /// estimated stature outside 1.3-2.1 m). Deliberately NOT implemented here.
    ///
    /// So one good frame replaces the 60-frame T-pose capture. Only distances are read, so the
    /// synthetic layout's orientation is arbitrary; it is written pelvis-at-origin, Y-up.
    ///
    /// NOTE (not fixed here — NimbleBridge.mm is not this file's to edit): the bridge's
    /// reference constants 0.88 / 0.54 / 0.52 are Rajagopal2016-era numbers, but the shipped
    /// model is FullBody.osim whose rest lengths are 0.8061 / 0.5360 / 0.4820 (FK'd from
    /// findings/osim_skeleton_dump.json). Scale 1.0 therefore leaves FullBody's legs ~9% and
    /// trunk ~8% shorter than the constants imply. The live ARKit path has the identical bias,
    /// so this is not a regression introduced by the offline path.
    ///
    /// - Returns: flat xyz triples and the matching OpenSim marker names, ready to be boxed
    ///   into `[NSNumber]` for the bridge.
    static func segmentScaleMarkers(jointCoords: [SIMD3<Float>])
        -> (positions: [Float], names: [String]) {
        guard jointCoords.count >= MHR.jointCount else { return ([], []) }

        // Bilateral note: MHR's bone-scale parameters are symmetric — measured
        // |l_femur| - |r_femur| = 1.1e-8 m on real predictions — so the L/R averaging inside
        // scaleModelWithHeight is a no-op here. Both sides are still emitted so the bridge's
        // own averaging path is exercised exactly as in the live case.
        let lLower = norm(jointCoords[MHR.lUpleg] - jointCoords[MHR.lLowleg])
                   + norm(jointCoords[MHR.lLowleg] - jointCoords[MHR.lFoot])
        let rLower = norm(jointCoords[MHR.rUpleg] - jointCoords[MHR.rLowleg])
                   + norm(jointCoords[MHR.rLowleg] - jointCoords[MHR.rFoot])
        let lUpper = norm(jointCoords[MHR.lUparm] - jointCoords[MHR.lLowarm])
                   + norm(jointCoords[MHR.lLowarm] - jointCoords[MHR.lWrist])
        let rUpper = norm(jointCoords[MHR.rUparm] - jointCoords[MHR.rLowarm])
                   + norm(jointCoords[MHR.rLowarm] - jointCoords[MHR.rWrist])

        let halfHip = 0.5 * norm(jointCoords[MHR.lUpleg] - jointCoords[MHR.rUpleg])
        let halfShoulder = 0.5 * norm(jointCoords[MHR.lUparm] - jointCoords[MHR.rUparm])
        let shoulderMid = 0.5 * (jointCoords[MHR.lUparm] + jointCoords[MHR.rUparm])
        let trunk = norm(shoulderMid - jointCoords[MHR.root])

        let pelvis = SIMD3<Float>(0, 0, 0)
        let lhjc = SIMD3<Float>(halfHip, 0, 0)
        let rhjc = SIMD3<Float>(-halfHip, 0, 0)
        let lajc = lhjc - SIMD3<Float>(0, lLower, 0)
        let rajc = rhjc - SIMD3<Float>(0, rLower, 0)
        let lsjc = SIMD3<Float>(halfShoulder, trunk, 0)
        let rsjc = SIMD3<Float>(-halfShoulder, trunk, 0)
        let lwjc = lsjc + SIMD3<Float>(lUpper, 0, 0)
        let rwjc = rsjc - SIMD3<Float>(rUpper, 0, 0)

        // Exactly the nine markers scaleModelWithHeight reads.
        let entries: [(String, SIMD3<Float>)] = [
            ("PELVIS", pelvis),
            ("LHJC", lhjc), ("RHJC", rhjc),
            ("LAJC", lajc), ("RAJC", rajc),
            ("LSJC", lsjc), ("RSJC", rsjc),
            ("LWJC", lwjc), ("RWJC", rwjc),
        ]
        var positions: [Float] = []
        positions.reserveCapacity(entries.count * 3)
        var names: [String] = []
        names.reserveCapacity(entries.count)
        for (name, p) in entries {
            names.append(name)
            positions.append(p.x)
            positions.append(p.y)
            positions.append(p.z)
        }
        return (positions, names)
    }

    // MARK: - Gross-implausibility gate

    /// Verdict on whether one prediction is worth scaling the skeleton to.
    enum Plausibility: Equatable {
        case plausible(hipWidthMeters: Double, statureMeters: Double)
        /// `reason` is written for the user and always carries the measured
        /// number that failed, because "we threw your photo away" without the
        /// number is indistinguishable from a crash.
        case implausible(reason: String, hipWidthMeters: Double, statureMeters: Double)

        var isPlausible: Bool { if case .plausible = self { return true }; return false }
        var reason: String? { if case .implausible(let r, _, _) = self { return r }; return nil }
        var hipWidthMeters: Double {
            switch self {
            case .plausible(let w, _), .implausible(_, let w, _): return w
            }
        }
        var statureMeters: Double {
            switch self {
            case .plausible(_, let s), .implausible(_, _, let s): return s
            }
        }
    }

    /// Bounds on a prediction that is about to drive `scaleModelWithHeight`.
    ///
    /// # These are a GROSS-IMPLAUSIBILITY gate, not an anthropometric norm
    /// They exist to catch a prediction that has collapsed — the recorded case
    /// is `sample2`, a small heavily-occluded rider on a horse, which produced a
    /// **0.070 m** inter-hip distance, 0.116 m shoulder width and a 0.178 m
    /// humerus, i.e. a person roughly half scale (STATUS.md, "The limitation
    /// that shapes the product claim"; `segmentScaleMarkers`' doc comment).
    /// Nothing downstream noticed: `scaleModelWithHeight` clamps its per-segment
    /// factors into `[0.7, 1.4]`, which TRUNCATES the damage into a silently
    /// wrong model instead of reporting it.
    ///
    /// A real person outside these bounds would be rejected, and that is the
    /// deliberate trade: the alternative is a muscle number computed on a
    /// skeleton scaled to somebody who does not exist. Nothing here is a
    /// clinical or normative statement about a body.
    ///
    /// Margins on the five predictions this pipeline is known to fit
    /// (`segmentScaleMarkers`' recorded runs — dancing / yoga / football /
    /// sample4 pass, sample2 fails): the passing statures are 1.602–1.715 m,
    /// so the 1.3 m floor sits ~19% below the smallest and the 2.1 m ceiling
    /// ~22% above the largest. `sample2`'s 0.070 m hip width is 30% below the
    /// 0.10 m floor.
    static let minHipWidthMeters = 0.10
    static let maxHipWidthMeters = 0.28
    static let minStatureMeters = 1.30
    static let maxStatureMeters = 2.10

    /// Distance between the two hip joint centres, in metres. This is an
    /// inter-JOINT-CENTRE distance, not a bi-trochanteric or a surface width.
    static func hipWidthMeters(jointCoords: [SIMD3<Float>]) -> Double {
        guard jointCoords.count >= MHR.jointCount else { return .nan }
        return Double(norm(jointCoords[MHR.lUpleg] - jointCoords[MHR.rUpleg]))
    }

    /// Gate one prediction before it reaches `scaleModelWithHeight`.
    ///
    /// Deliberately reads only pose-INVARIANT quantities — an inter-joint-centre
    /// distance across a rigid pelvis, and the chain-sum stature. A bent knee or
    /// a raised arm cannot trip it; only a prediction whose skeleton is the
    /// wrong size can. That is why it is safe to run on every frame rather than
    /// only on the calibration frame.
    static func plausibility(jointCoords: [SIMD3<Float>]) -> Plausibility {
        guard jointCoords.count >= MHR.jointCount else {
            return .implausible(reason: "the pose model returned \(jointCoords.count) joints, not \(MHR.jointCount)",
                                hipWidthMeters: .nan, statureMeters: .nan)
        }
        let hip = hipWidthMeters(jointCoords: jointCoords)
        let stature = Double(estimatedStatureMeters(jointCoords: jointCoords))

        guard hip.isFinite, stature.isFinite else {
            return .implausible(reason: "the pose model returned a non-finite body size",
                                hipWidthMeters: hip, statureMeters: stature)
        }
        if hip < minHipWidthMeters || hip > maxHipWidthMeters {
            return .implausible(
                reason: String(format: "hip width came out %.0f cm, outside the %.0f–%.0f cm this app can measure — the person is probably too small in frame, or partly hidden",
                               hip * 100, minHipWidthMeters * 100, maxHipWidthMeters * 100),
                hipWidthMeters: hip, statureMeters: stature)
        }
        if stature < minStatureMeters || stature > maxStatureMeters {
            return .implausible(
                reason: String(format: "estimated height came out %.2f m, outside the %.1f–%.1f m this app can measure — the person is probably too small in frame, or partly hidden",
                               stature, minStatureMeters, maxStatureMeters),
                hipWidthMeters: hip, statureMeters: stature)
        }
        return .plausible(hipWidthMeters: hip, statureMeters: stature)
    }

    /// Standing height in meters, estimated from segment lengths rather than vertical extent.
    ///
    /// Vertical extent is useless off a single arbitrary frame: measured top-of-head-to-lowest-
    /// foot was 1.498 m for the dancer (bent knee), 0.945 m for the seated yoga subject and
    /// −0.050 m for the football player (lying down). The earlier
    /// findings/height_scale_bound.json figure of 1.074 m is that kind of measurement and is
    /// superseded by this one.
    ///
    /// Summing the chain instead — leg + trunk + neck + skull, plus the ankle-to-sole offset —
    /// gave 1.715 m / 1.602 m / 1.646 m on the same three frames, all plausible adults.
    ///
    /// `ankleToSole` is the one cross-model constant: 0.050 m, bracketed by FullBody.osim's
    /// talus-above-ground 0.0454 m and MHR's own rest ankle→MTP drop 0.0531 m, scaled with the
    /// subject's leg. Worth about ±0.6% of stature.
    ///
    /// This value is only consumed as `scaleModelWithHeight`'s `height:` argument, which that
    /// method uses solely as the per-segment fallback `height / 1.8` when a marker pair is
    /// missing — and `segmentScaleMarkers` always supplies all nine. So it is effectively a
    /// display / sanity number, not the scaling driver.
    static func estimatedStatureMeters(jointCoords: [SIMD3<Float>]) -> Float {
        guard jointCoords.count >= MHR.jointCount else { return 1.8 }
        let lLeg = norm(jointCoords[MHR.lUpleg] - jointCoords[MHR.lLowleg])
                 + norm(jointCoords[MHR.lLowleg] - jointCoords[MHR.lFoot])
        let rLeg = norm(jointCoords[MHR.rUpleg] - jointCoords[MHR.rLowleg])
                 + norm(jointCoords[MHR.rLowleg] - jointCoords[MHR.rFoot])
        let leg = 0.5 * (lLeg + rLeg)
        let trunk = norm(jointCoords[MHR.cNeck] - jointCoords[MHR.root])
        let neck = norm(jointCoords[MHR.cHead] - jointCoords[MHR.cNeck])
        let skull = norm(jointCoords[MHR.cHeadNull] - jointCoords[MHR.cHead])
        // MHR rest leg (hip→knee→ankle) is 0.8405 m; scale the sole offset with the subject.
        let ankleToSole: Float = 0.050 * (leg / 0.8405)
        return leg + trunk + neck + skull + ankleToSole
    }

    // MARK: - Projection back onto the source image

    /// Projects an MHR-native (Y-up) position into ORIGINAL source-image pixels,
    /// through the same pinhole camera the model itself assumed.
    ///
    /// This is what makes a skeleton overlay comparable to the photo: it is not
    /// a fitted alignment, it is the model's own projection, so any visible gap
    /// between a drawn joint and the corresponding body part in the photo IS the
    /// model's error rather than a display artefact.
    ///
    /// The camera is the default `prepare_batch.py` fallback that
    /// `SAM3DPoseEstimator` also feeds the network: focal length
    /// `sqrt(w^2 + h^2)` with the principal point at the image centre. MHR-native
    /// is Y-up / Z-toward-camera, so Y and Z flip to reach the OpenCV-style
    /// camera frame before `cam_t` is added.
    ///
    /// Returns nil for points at or behind the camera plane, which would
    /// otherwise project to a mirrored point in front of it.
    static func projectToImage(_ p: SIMD3<Float>,
                               camT: SIMD3<Float>,
                               imageSize: CGSize) -> CGPoint? {
        let w = Float(imageSize.width), h = Float(imageSize.height)
        guard w > 0, h > 0 else { return nil }
        let focal = (w * w + h * h).squareRoot()
        let cam = SIMD3<Float>(p.x, -p.y, -p.z) + camT
        guard cam.z > 1e-4 else { return nil }
        return CGPoint(x: CGFloat(focal * cam.x / cam.z + w / 2),
                       y: CGFloat(focal * cam.y / cam.z + h / 2))
    }

    // MARK: - Self-check

    /// Returns the rows whose ARKit id / OpenSim marker name disagree with
    /// `JointMapping.primary`, and reports any count mismatch. Empty means the table is in
    /// sync with the mapping `NimbleEngine.processFrame` actually looks up. Cheap; call it
    /// once from a debug path or a unit test rather than per frame.
    static func inconsistenciesWithJointMapping() -> [String] {
        var problems: [String] = []
        if table.count != JointMapping.primary.count {
            problems.append("table has \(table.count) rows, JointMapping.primary has \(JointMapping.primary.count)")
        }
        for src in table {
            guard let m = JointMapping.primary.first(where: { $0.arkitName == src.arkitJointId }) else {
                problems.append("\(src.arkitJointId): not present in JointMapping.primary")
                continue
            }
            if m.opensimName != src.opensimMarker {
                problems.append("\(src.arkitJointId): maps to \(m.opensimName), table says \(src.opensimMarker)")
            }
        }
        return problems
    }

    // MARK: - Helpers

    @inline(__always)
    private static func norm(_ v: SIMD3<Float>) -> Float {
        (v.x * v.x + v.y * v.y + v.z * v.z).squareRoot()
    }
}
