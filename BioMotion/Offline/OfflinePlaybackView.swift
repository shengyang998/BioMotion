import SwiftUI
import RealityKit
import UIKit
import simd

/// Non-AR 3D playback surface for an offline import run, plus a frame scrubber.
///
/// `SkeletonARView` (BioMotion/ARKit/SkeletonOverlayView.swift) is hard-wired to
/// a live `ARSession` (`arView.session = session`,
/// `.environment.background = .cameraFeed()`) and only shows anything once
/// `isTracking` flips true inside the ARSessionDelegate callback — none of that
/// applies to an imported clip, so this view builds its own `ARView` in
/// `.nonAR` camera mode with a manual `PerspectiveCamera` entity instead, per
/// this file's task brief. It reuses `MuscleOverlay` verbatim (both its render
/// passes work unchanged: pass 1 is keyed by the ARKit joint id strings our
/// `BodyFrame` already supplies, pass 2 by world-space muscle paths).
struct OfflinePlaybackView: View {
    @ObservedObject var resultStore: OfflineResultStore
    let onDone: () -> Void

    @State private var showMuscles = true
    @State private var showSourceImage = true

    var body: some View {
        VStack(spacing: 0) {
            ZStack {
                OfflineSceneView(frame: resultStore.selectedFrame, showMuscles: showMuscles)
                    .ignoresSafeArea(edges: .top)

                if showSourceImage, let frame = resultStore.selectedFrame {
                    VStack {
                        HStack {
                            Image(uiImage: frame.sourceImage)
                                .resizable()
                                .scaledToFit()
                                .frame(width: 120)
                                .clipShape(RoundedRectangle(cornerRadius: 8))
                                .overlay(RoundedRectangle(cornerRadius: 8).stroke(.white.opacity(0.5)))
                                .padding(8)
                                .shadow(radius: 4)
                            Spacer()
                        }
                        Spacer()
                    }
                }

                VStack {
                    HStack {
                        Spacer()
                        statusBadge
                    }
                    .padding(8)
                    Spacer()
                }
            }

            controls
        }
        .navigationTitle("Playback")
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .confirmationAction) {
                Button("Done", action: onDone)
            }
        }
    }

    @ViewBuilder
    private var statusBadge: some View {
        if let frame = resultStore.selectedFrame {
            VStack(alignment: .trailing, spacing: 2) {
                Text(statusText(frame))
                    .font(.caption2)
                if frame.usedFallbackBBox {
                    Text("no person detected — used full frame")
                        .font(.caption2)
                        .foregroundStyle(.orange)
                }
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .background(.black.opacity(0.6), in: RoundedRectangle(cornerRadius: 8))
            .foregroundStyle(.white)
        }
    }

    private func statusText(_ frame: OfflineResultStore.FrameResult) -> String {
        switch frame.status {
        case .success:
            if frame.isStaticHoldEstimate { return "Pose + static-hold muscle estimate" }
            return frame.hasFullBiomechanics ? "Pose + muscles" : "Pose only (warming up)"
        case .poseEstimationFailed(let reason):
            return "Pose failed: \(reason)"
        case .nimbleTimeout:
            return "Solver timed out on this frame"
        }
    }

    private var controls: some View {
        VStack(spacing: 8) {
            if resultStore.frames.count > 1 {
                Slider(
                    value: Binding(
                        get: { Double(resultStore.selectedIndex) },
                        set: { resultStore.selectedIndex = Int($0.rounded()) }
                    ),
                    in: 0...Double(max(resultStore.frames.count - 1, 0)),
                    step: 1
                )
                .padding(.horizontal)
            }
            HStack {
                Text(frameLabel).font(.caption).foregroundStyle(.secondary)
                Spacer()
                Button { showMuscles.toggle() } label: {
                    Image(systemName: "figure.run")
                        .foregroundStyle(showMuscles ? Color.green : Color.gray)
                }
                Button { showSourceImage.toggle() } label: {
                    Image(systemName: "photo")
                        .foregroundStyle(showSourceImage ? Color.blue : Color.gray)
                }
            }
            .padding(.horizontal)
        }
        .padding(.vertical, 8)
        .background(.thinMaterial)
    }

    private var frameLabel: String {
        guard !resultStore.frames.isEmpty else { return "No frames" }
        return "Frame \(resultStore.selectedIndex + 1)/\(resultStore.frames.count) — \(resultStore.biomechanicsCount) with muscle data"
    }
}

// MARK: - RealityKit surface

private struct OfflineSceneView: UIViewRepresentable {
    let frame: OfflineResultStore.FrameResult?
    var showMuscles: Bool

    func makeUIView(context: Context) -> ARView {
        // Non-AR: no camera session, no world tracking (see this file's header
        // comment). RealityKit renders through a manual PerspectiveCamera entity
        // instead of device motion.
        let arView = ARView(frame: .zero, cameraMode: .nonAR, automaticallyConfigureSession: false)
        arView.environment.background = .color(.black)

        // Plain `AnchorEntity()` (no tracking target) — same pattern already
        // proven working in this codebase's live view (SkeletonOverlayView.swift
        // `makeUIView`), anchored at the identity transform.
        let cameraEntity = PerspectiveCamera()
        cameraEntity.camera.fieldOfViewInDegrees = 60
        let cameraAnchor = AnchorEntity()
        cameraAnchor.addChild(cameraEntity)
        arView.scene.addAnchor(cameraAnchor)
        context.coordinator.cameraEntity = cameraEntity

        let rootAnchor = AnchorEntity()
        arView.scene.addAnchor(rootAnchor)
        context.coordinator.rootAnchor = rootAnchor
        context.coordinator.muscleOverlay.setup(anchor: rootAnchor)

        return arView
    }

    func updateUIView(_ arView: ARView, context: Context) {
        guard let frame, let bodyFrame = frame.bodyFrame, bodyFrame.joints.contains(where: \.isTracked) else {
            context.coordinator.hideAll()
            return
        }

        context.coordinator.frameCameraIfNeeded(joints: bodyFrame.joints)
        context.coordinator.updateSkeleton(joints: bodyFrame.joints)
        context.coordinator.muscleOverlay.setVisible(showMuscles)

        if showMuscles, let muscle = frame.muscleResult {
            context.coordinator.muscleOverlay.update(joints: bodyFrame.joints, muscle: muscle)
        } else {
            // No muscle data for this exact frame (SG warm-up / failed solve) —
            // hide rather than let stale muscle geometry from a previously
            // scrubbed-to frame linger and be misread as belonging to this one.
            context.coordinator.muscleOverlay.setVisible(false)
        }
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    final class Coordinator {
        var rootAnchor: AnchorEntity?
        var cameraEntity: PerspectiveCamera?
        let muscleOverlay = MuscleOverlay()

        private var jointEntities: [String: ModelEntity] = [:]
        private var boneEntities: [String: ModelEntity] = [:]
        private var hasFramedCamera = false

        /// Positions the camera once, from the first frame's tracked-joint
        /// bounding box, and leaves it fixed for the rest of the scrub session
        /// (like a real camera that stayed put while filming). Recomputing every
        /// frame would jar the view as the subject moves; a hardcoded world
        /// position would risk a blank screen since MHRRetarget documents that
        /// `joint_coords` pins the pelvis at a MODEL-CONSTANT (0, 0.924, 0) in
        /// every prediction rather than a real-world camera distance — this
        /// auto-framing is robust to that regardless of the exact constant.
        func frameCameraIfNeeded(joints: [TrackedJoint]) {
            guard !hasFramedCamera, let cameraEntity else { return }
            let tracked = joints.filter(\.isTracked).map(\.worldPosition)
            guard !tracked.isEmpty else { return }
            var minP = tracked[0], maxP = tracked[0]
            for p in tracked {
                minP = simd_min(minP, p)
                maxP = simd_max(maxP, p)
            }
            let center = (minP + maxP) / 2
            let extent = simd_length(maxP - minP)
            // Full-body extent is usually ~1.7-2.0m head-to-toe; back off enough
            // to fit it plus margin, with a floor so a near-degenerate (e.g.
            // single-joint) bounding box doesn't put the camera inside the body.
            let distance = max(extent * 1.6, 1.5)
            // Arbitrary viewing angle (not derived from any body-facing
            // convention — world-space facing direction isn't something this
            // view has a signal for): back off along +Z, look at center.
            let position = center + SIMD3<Float>(0, 0, Float(distance))
            cameraEntity.position = position
            cameraEntity.look(at: center, from: position, relativeTo: nil)
            hasFramedCamera = true
        }

        /// Minimal joint/bone rendering for visual context around the muscle
        /// overlay. This mirrors SkeletonOverlayView.Coordinator's approach (same
        /// `JointMapping.bones` list, same sphere+box primitives) but is an
        /// independent implementation — that Coordinator is a private nested
        /// type inside a `UIViewRepresentable` hard-wired to a live ARSession and
        /// isn't reusable here. Uses `UnlitMaterial` (like `MuscleOverlay`
        /// itself) rather than a lit `SimpleMaterial`, so this doesn't also need
        /// to set up scene lighting, which `.nonAR` mode does not provide by
        /// default the way `.ar` mode's camera-feed environment does.
        func updateSkeleton(joints: [TrackedJoint]) {
            guard let anchor = rootAnchor else { return }

            for joint in joints where joint.isTracked {
                if let entity = jointEntities[joint.id] {
                    entity.position = joint.worldPosition
                    entity.isEnabled = true
                } else {
                    let mesh = MeshResource.generateSphere(radius: 0.02)
                    let material = UnlitMaterial(color: jointColor(for: joint.id))
                    let entity = ModelEntity(mesh: mesh, materials: [material])
                    entity.position = joint.worldPosition
                    anchor.addChild(entity)
                    jointEntities[joint.id] = entity
                }
            }
            for joint in joints where !joint.isTracked {
                jointEntities[joint.id]?.isEnabled = false
            }

            for (index, bone) in JointMapping.bones.enumerated() {
                let key = "bone_\(index)"
                guard bone.0 < joints.count, bone.1 < joints.count else {
                    boneEntities[key]?.isEnabled = false
                    continue
                }
                let startJoint = joints[bone.0]
                let endJoint = joints[bone.1]
                guard startJoint.isTracked, endJoint.isTracked else {
                    boneEntities[key]?.isEnabled = false
                    continue
                }
                let start = startJoint.worldPosition
                let end = endJoint.worldPosition
                let length = simd_length(end - start)
                guard length > 0.001 else {
                    boneEntities[key]?.isEnabled = false
                    continue
                }

                let entity: ModelEntity
                if let existing = boneEntities[key] {
                    entity = existing
                    entity.model?.mesh = MeshResource.generateBox(size: SIMD3<Float>(0.008, 0.008, length), cornerRadius: 0.004)
                    entity.isEnabled = true
                } else {
                    let mesh = MeshResource.generateBox(size: SIMD3<Float>(0.008, 0.008, length), cornerRadius: 0.004)
                    let material = UnlitMaterial(color: .white.withAlphaComponent(0.7))
                    entity = ModelEntity(mesh: mesh, materials: [material])
                    anchor.addChild(entity)
                    boneEntities[key] = entity
                }
                let midpoint = (start + end) / 2
                entity.position = midpoint
                entity.look(at: end, from: midpoint, relativeTo: nil)
            }
        }

        func hideAll() {
            for (_, entity) in jointEntities { entity.isEnabled = false }
            for (_, entity) in boneEntities { entity.isEnabled = false }
            muscleOverlay.setVisible(false)
        }

        private func jointColor(for jointId: String) -> UIColor {
            if jointId.contains("left") { return .systemBlue }
            if jointId.contains("right") { return .systemRed }
            return .systemGreen
        }
    }
}
