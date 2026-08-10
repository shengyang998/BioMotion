import simd
import ARKit

/// A single tracked joint with its 3D position in world space.
struct TrackedJoint: Identifiable {
    let id: String          // ARSkeleton.JointName rawValue
    let name: String        // Human-readable name
    let worldPosition: SIMD3<Float>  // Meters, world space (Y-up)
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
struct BodyFrame {
    let timestamp: TimeInterval
    let frameNumber: Int
    let joints: [TrackedJoint]

    /// The pose source's root position in world space. Its anatomical meaning
    /// is carried separately by the joint's marker override: live uses
    /// PELVIS, while MHR keeps raw joint 1 and labels it MHR_ROOT.
    var rootPosition: SIMD3<Float>? {
        joints.first(where: { $0.id == "hips_joint" })?.worldPosition
    }
}

/// Stable body-joint ids and their live/default OpenSim marker names.
enum JointMapping {
    struct Mapping {
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
