import SwiftUI
import ARKit
import RealityKit

/// One fail-closed decision for every surface that can present the live
/// anatomical capsules. The renderer and its disclosure must consume the same
/// `anatomyIsPresented` value; neither depends on the Nimble model because the
/// fixed capsule plan only consumes tracked joints.
struct LiveAnatomyPresentation: Equatable {
    enum Surface: CaseIterable, Hashable {
        case calibration
        case tracking
    }

    let showsControl: Bool
    let anatomyIsPresented: Bool

    init(
        surface: Surface,
        isTracking: Bool,
        hasCurrentFrame: Bool,
        isEnabled: Bool
    ) {
        let hasUsableLivePose = surface == .tracking && isTracking && hasCurrentFrame
        showsControl = hasUsableLivePose
        anatomyIsPresented = hasUsableLivePose && isEnabled
    }
}

/// ARView wrapper that displays the camera feed with the skeleton and the
/// muscle ANATOMY layer.
///
/// The muscle solve is not an input here any more. `MuscleOverlay` used to be
/// handed `nimble.displayMuscleResult` and coloured its capsules from it, which
/// stated a cross-muscle ordering this model cannot support — see that type's
/// doc. The live screen carries `MuscleOverlay.anatomyOnlyNote` instead.
struct SkeletonARView: UIViewRepresentable {
    let session: ARSession
    @Binding var currentFrame: BodyFrame?
    let isTracking: Bool
    let anatomyPresentation: LiveAnatomyPresentation

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
        context.coordinator.muscleOverlay.setVisible(anatomyPresentation.anatomyIsPresented)
        if anatomyPresentation.anatomyIsPresented {
            context.coordinator.muscleOverlay.update(joints: frame.joints)
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
