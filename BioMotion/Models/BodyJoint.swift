import simd
import ARKit

/// A single tracked joint with its 3D position in the owning frame's space.
struct TrackedJoint: Identifiable, Sendable {
    let id: String          // ARSkeleton.JointName rawValue
    let name: String        // Human-readable name
    /// Metres, Y-up. `BodyFrame.dynamicsReference` states whether this is an
    /// inertial world, camera-relative, root-relative, or unmeasured space.
    let worldPosition: SIMD3<Float>
    let isTracked: Bool

    /// Source-specific OpenSim marker semantics. Live ARKit joints leave this
    /// nil and use `JointMapping.primary`; offline pose sources may name a
    /// different point for the same stable body-joint id.
    let opensimMarkerNameOverride: String?

    init(id: String,
         name: String,
         worldPosition: SIMD3<Float>,
         isTracked: Bool,
         opensimMarkerNameOverride: String? = nil) {
        self.id = id
        self.name = name
        self.worldPosition = worldPosition
        self.isTracked = isTracked
        self.opensimMarkerNameOverride = opensimMarkerNameOverride
    }
}

/// A complete body frame — all joints at one instant.
struct BodyFrame: Sendable {
    /// The two independent spatial facts required before marker kinematics can
    /// support an inverse-dynamics claim. A camera-relative metric position is
    /// useful for IK and overlays, but it is not an inertial trajectory: image
    /// up need not equal gravity up, and monocular depth is not qualified for a
    /// second derivative. Keeping those facts typed prevents a valid-looking
    /// `SIMD3` from silently becoming a world-space dynamics measurement.
    struct DynamicsReference: Equatable, Sendable {
        enum Gravity: Equatable, Sendable {
            /// No synchronized camera-to-gravity transform accompanies this
            /// frame. Upright pixels and a stationary tripod do not establish
            /// this transform.
            case unmeasured
            /// The marker axes are expressed in a frame whose -Y direction is
            /// the physical gravity direction used by Nimble/OpenSim.
            case gravityAligned
        }

        enum RootTrajectory: Equatable, Sendable {
            /// The source did not state how global root translation was handled.
            case unmeasured
            /// Marker geometry is metric but remains pinned to the pose model's
            /// root constant. Relative pose is usable; global translation is not.
            case rootRelative
            /// `cam_t` has been composed, so position is available in the camera
            /// frame. Its depth/noise/camera-motion evidence is insufficient for
            /// temporal inverse dynamics.
            case cameraRelativePositionOnly
            /// Global position is present in a gravity-aligned world frame, but
            /// tracking continuity and root derivative noise have not been
            /// measured against the temporal-dynamics budget.
            case worldPositionOnly
            /// A synchronized inertial frame and a calibrated root-noise/depth
            /// policy established that this trajectory may be differentiated.
            case dynamicsQualified
        }

        let gravity: Gravity
        let rootTrajectory: RootTrajectory

        /// Fail-closed default for legacy/adversarial constructors.
        static let unmeasured = Self(
            gravity: .unmeasured,
            rootTrajectory: .unmeasured
        )

        /// ARKit world tracking uses gravity alignment and supplies global body
        /// translation. Source class alone does not establish that tracking
        /// continuity and second-derivative noise meet the dynamics budget.
        static let liveARKit = Self(
            gravity: .gravityAligned,
            rootTrajectory: .worldPositionOnly
        )

        /// Explicit authorization seam for a future source/run whose inertial
        /// alignment, continuity and root derivative noise have all passed
        /// their measured policy. Tests use it to exercise gates beyond root.
        static let dynamicsQualifiedWorld = Self(
            gravity: .gravityAligned,
            rootTrajectory: .dynamicsQualified
        )

        /// Current offline MHR solver input: metric relative anatomy with the
        /// model's root pinned and no synchronized gravity reference.
        static let mhrRootRelative = Self(
            gravity: .unmeasured,
            rootTrajectory: .rootRelative
        )

        /// A body with raw MHR `cam_t` composed. This is a camera-relative
        /// position product, never a dynamics authorization by itself.
        static let mhrCameraRelativePosition = Self(
            gravity: .unmeasured,
            rootTrajectory: .cameraRelativePositionOnly
        )

        /// Static equilibrium needs a physical gravity direction but does not
        /// differentiate root translation. Temporal dynamics needs both.
        func permits(_ solveClass: NimbleEngine.DynamicsSolveClass) -> Bool {
            guard gravity == .gravityAligned else { return false }
            switch solveClass {
            case .staticEquilibrium:
                return true
            case .temporal:
                return rootTrajectory == .dynamicsQualified
            }
        }
    }

    let timestamp: TimeInterval
    let frameNumber: Int
    let joints: [TrackedJoint]
    let dynamicsReference: DynamicsReference

    init(timestamp: TimeInterval,
         frameNumber: Int,
         joints: [TrackedJoint],
         dynamicsReference: DynamicsReference = .unmeasured) {
        self.timestamp = timestamp
        self.frameNumber = frameNumber
        self.joints = joints
        self.dynamicsReference = dynamicsReference
    }

    /// The pose source's root position in this frame's declared coordinate
    /// space. Its anatomical meaning is carried separately by the joint's
    /// marker override: live uses PELVIS, while MHR keeps raw joint 1 and
    /// labels it MHR_ROOT.
    var rootPosition: SIMD3<Float>? {
        joints.first(where: { $0.id == "hips_joint" })?.worldPosition
    }
}

/// Stable body-joint ids and their live/default OpenSim marker names.
enum JointMapping {
    struct Mapping: Sendable {
        let arkitName: String
        let opensimName: String
        let displayName: String
    }

    /// Live/default mappings. Offline sources retain these stable ids but may
    /// override a marker name when the source point has different anatomy.
    static let primary: [Mapping] = [
        // Pelvis / Root
        Mapping(arkitName: "hips_joint", opensimName: "PELVIS", displayName: "Pelvis"),
        // Lower body
        Mapping(arkitName: "left_upLeg_joint", opensimName: "LHJC", displayName: "L Hip"),
        Mapping(arkitName: "right_upLeg_joint", opensimName: "RHJC", displayName: "R Hip"),
        Mapping(arkitName: "left_leg_joint", opensimName: "LKJC", displayName: "L Knee"),
        Mapping(arkitName: "right_leg_joint", opensimName: "RKJC", displayName: "R Knee"),
        Mapping(arkitName: "left_foot_joint", opensimName: "LAJC", displayName: "L Ankle"),
        Mapping(arkitName: "right_foot_joint", opensimName: "RAJC", displayName: "R Ankle"),
        Mapping(arkitName: "left_toes_joint", opensimName: "LTOE", displayName: "L Toe"),
        Mapping(arkitName: "right_toes_joint", opensimName: "RTOE", displayName: "R Toe"),
        // Spine
        Mapping(arkitName: "spine_1_joint", opensimName: "SPINE_L", displayName: "Lower Spine"),
        Mapping(arkitName: "spine_4_joint", opensimName: "SPINE_M", displayName: "Mid Spine"),
        Mapping(arkitName: "spine_7_joint", opensimName: "C7", displayName: "C7"),
        Mapping(arkitName: "neck_1_joint", opensimName: "NECK", displayName: "Neck"),
        Mapping(arkitName: "head_joint", opensimName: "HEAD", displayName: "Head"),
        // Upper body
        Mapping(arkitName: "left_shoulder_1_joint", opensimName: "LSJC", displayName: "L Shoulder"),
        Mapping(arkitName: "right_shoulder_1_joint", opensimName: "RSJC", displayName: "R Shoulder"),
        Mapping(arkitName: "left_forearm_joint", opensimName: "LEJC", displayName: "L Elbow"),
        Mapping(arkitName: "right_forearm_joint", opensimName: "REJC", displayName: "R Elbow"),
        Mapping(arkitName: "left_hand_joint", opensimName: "LWJC", displayName: "L Wrist"),
        Mapping(arkitName: "right_hand_joint", opensimName: "RWJC", displayName: "R Wrist"),
    ]

    /// Resolve a tracked joint through the stable ARKit-id whitelist, then
    /// apply any source-specific marker semantics. An override never grants an
    /// unknown joint access to the native solver, and an empty override fails
    /// closed rather than silently falling back to a different anatomical
    /// point.
    static func opensimMarkerName(for joint: TrackedJoint) -> String? {
        guard let mapping = primary.first(where: { $0.arkitName == joint.id }) else {
            return nil
        }
        if let override = joint.opensimMarkerNameOverride {
            return override.isEmpty ? nil : override
        }
        return mapping.opensimName
    }

    /// Bones: pairs of joint indices (into `primary`) to draw as skeleton lines.
    static let bones: [(Int, Int)] = [
        // Spine chain
        (0, 9), (9, 10), (10, 11), (11, 12), (12, 13),
        // Left leg
        (0, 1), (1, 3), (3, 5), (5, 7),
        // Right leg
        (0, 2), (2, 4), (4, 6), (6, 8),
        // Left arm
        (11, 14), (14, 16), (16, 18),
        // Right arm
        (11, 15), (15, 17), (17, 19),
    ]
}
