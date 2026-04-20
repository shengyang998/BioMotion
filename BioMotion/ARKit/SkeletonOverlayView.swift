import SwiftUI
import ARKit
import RealityKit

/// ARView wrapper that displays the camera feed with skeleton + muscle overlay.
struct SkeletonARView: UIViewRepresentable {
    let session: ARSession
    @Binding var currentFrame: BodyFrame?
    var isTracking: Bool = true
    var muscleOutput: NimbleEngine.MuscleOutput?
    var showMuscles: Bool = true

    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
        arView.session = session
        arView.environment.background = .cameraFeed()

        // Add a root anchor for skeleton entities
        let anchor = AnchorEntity()
        arView.scene.addAnchor(anchor)
        context.coordinator.rootAnchor = anchor
        context.coordinator.arView = arView
        context.coordinator.muscleOverlay.setup(anchor: anchor)

        return arView
    }

    func updateUIView(_ arView: ARView, context: Context) {
        guard isTracking, let frame = currentFrame else {
            context.coordinator.hideAll()
            return
        }
        context.coordinator.updateSkeleton(frame: frame)
        context.coordinator.muscleOverlay.setVisible(showMuscles)
        if showMuscles, let output = muscleOutput {
            context.coordinator.muscleOverlay.update(joints: frame.joints, muscle: output)
        }
    }

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    class Coordinator {
        var rootAnchor: AnchorEntity?
        var arView: ARView?
        let muscleOverlay = MuscleOverlay()
        private var jointEntities: [String: ModelEntity] = [:]
        private var boneEntities: [String: ModelEntity] = [:]

        func updateSkeleton(frame: BodyFrame) {
            guard let anchor = rootAnchor else { return }

            // Update or create joint spheres
            for joint in frame.joints where joint.isTracked {
                if let entity = jointEntities[joint.id] {
                    entity.position = joint.worldPosition
                    entity.isEnabled = true
                } else {
                    let mesh = MeshResource.generateSphere(radius: 0.02)
                    let material = SimpleMaterial(
                        color: jointColor(for: joint.id),
                        isMetallic: false
                    )
                    let entity = ModelEntity(mesh: mesh, materials: [material])
                    entity.position = joint.worldPosition
                    anchor.addChild(entity)
                    jointEntities[joint.id] = entity
                }
            }

            // Hide untracked joints
            for joint in frame.joints where !joint.isTracked {
                jointEntities[joint.id]?.isEnabled = false
            }

            // Update bones (lines between joints)
            for (index, bone) in JointMapping.bones.enumerated() {
                let startJoint = frame.joints[bone.0]
                let endJoint = frame.joints[bone.1]

                guard startJoint.isTracked && endJoint.isTracked else {
                    boneEntities["bone_\(index)"]?.isEnabled = false
                    continue
                }

                let start = startJoint.worldPosition
                let end = endJoint.worldPosition
                let midpoint = (start + end) / 2.0
                let length = simd_length(end - start)

                guard length > 0.001 else {
                    boneEntities["bone_\(index)"]?.isEnabled = false
                    continue
                }

                let key = "bone_\(index)"
                let entity: ModelEntity
                if let existing = boneEntities[key] {
                    entity = existing
                    entity.model?.mesh = MeshResource.generateBox(
                        size: SIMD3<Float>(0.008, 0.008, length),
                        cornerRadius: 0.004
                    )
                    entity.isEnabled = true
                } else {
                    let mesh = MeshResource.generateBox(
                        size: SIMD3<Float>(0.008, 0.008, length),
                        cornerRadius: 0.004
                    )
                    let material = SimpleMaterial(
                        color: .white.withAlphaComponent(0.7),
                        isMetallic: false
                    )
                    entity = ModelEntity(mesh: mesh, materials: [material])
                    anchor.addChild(entity)
                    boneEntities[key] = entity
                }

                entity.position = midpoint
                entity.look(at: end, from: midpoint, relativeTo: nil)
            }
        }

        func hideAll() {
            for (_, entity) in jointEntities {
                entity.isEnabled = false
            }
            for (_, entity) in boneEntities {
                entity.isEnabled = false
            }
            muscleOverlay.setVisible(false)
        }

        private func jointColor(for jointId: String) -> UIColor {
            if jointId.contains("left") {
                return .systemBlue
            } else if jointId.contains("right") {
                return .systemRed
            } else {
                return .systemGreen
            }
        }
    }
}
