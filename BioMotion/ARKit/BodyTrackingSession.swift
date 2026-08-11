import ARKit
import Combine
import QuartzCore

/// Manages the ARSession with body tracking configuration.
/// Publishes BodyFrame updates at camera frame rate (~60 fps).
final class BodyTrackingSession: NSObject, ObservableObject {
    let arSession = ARSession()

    @Published var currentFrame: BodyFrame?
    @Published var isTracking = false
    @Published var trackingMessage = "Point camera at a person"

    private var frameCount = 0
    private let skeletonFilter = SkeletonFilter(minCutoff: 1.0, beta: 0.007)
    private var lastBodyAnchorUpdateTime: CFTimeInterval?
    private let trackingLossTimeout: CFTimeInterval = 0.35

    override init() {
        super.init()
        arSession.delegate = self
    }

    var isBodyTrackingSupported: Bool {
        ARBodyTrackingConfiguration.isSupported
    }

    func start() {
        guard isBodyTrackingSupported else {
            trackingMessage = "Body tracking not supported on this device"
            return
        }

        let config = ARBodyTrackingConfiguration()
        config.worldAlignment = .gravity
        config.automaticSkeletonScaleEstimationEnabled = true
        config.frameSemantics.insert(.bodyDetection)
        lastBodyAnchorUpdateTime = nil
        skeletonFilter.reset()
        arSession.run(config, options: [.resetTracking, .removeExistingAnchors])
        trackingMessage = "Looking for body..."
    }

    func pause() {
        arSession.pause()
        isTracking = false
    }
}

extension BodyTrackingSession: ARSessionDelegate {
    func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        guard let bodyAnchor = anchors.compactMap({ $0 as? ARBodyAnchor }).first else { return }
        lastBodyAnchorUpdateTime = CACurrentMediaTime()

        let skeleton = bodyAnchor.skeleton
        let bodyWorldTransform = bodyAnchor.transform

        var joints: [TrackedJoint] = []

        for mapping in JointMapping.primary {
            let jointName = ARSkeleton.JointName(rawValue: mapping.arkitName)

            guard let modelTransform = skeleton.modelTransform(for: jointName) else {
                joints.append(TrackedJoint(
                    id: mapping.arkitName,
                    name: mapping.displayName,
                    worldPosition: .zero,
                    isTracked: false
                ))
                continue
            }

            // Model transform is relative to hip (root). Multiply by body world transform.
            let worldTransform = bodyWorldTransform * modelTransform
            let position = SIMD3<Float>(
                worldTransform.columns.3.x,
                worldTransform.columns.3.y,
                worldTransform.columns.3.z
            )

            // If modelTransform returned non-nil, the joint is tracked
            let jointIndex = skeleton.definition.index(for: jointName)
            let tracked = jointIndex != NSNotFound && skeleton.isJointTracked(jointIndex)

            joints.append(TrackedJoint(
                id: mapping.arkitName,
                name: mapping.displayName,
                worldPosition: position,
                isTracked: tracked
            ))
        }

        frameCount += 1
        let rawFrame = BodyFrame(
            timestamp: CACurrentMediaTime(),
            frameNumber: frameCount,
            joints: joints,
            dynamicsReference: .liveARKit
        )

        // Apply 1-euro filter to smooth joint positions
        let frame = skeletonFilter.filter(rawFrame)

        DispatchQueue.main.async {
            self.currentFrame = frame
            self.isTracking = true
            self.trackingMessage = "Tracking"
        }
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard let lastBodyAnchorUpdateTime else { return }
        guard (CACurrentMediaTime() - lastBodyAnchorUpdateTime) > trackingLossTimeout else { return }
        guard isTracking || currentFrame != nil else { return }

        skeletonFilter.reset()
        self.lastBodyAnchorUpdateTime = nil

        DispatchQueue.main.async {
            self.currentFrame = nil
            self.isTracking = false
            self.trackingMessage = "Looking for body..."
        }
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        trackingMessage = "AR error: \(error.localizedDescription)"
        isTracking = false
    }

    func sessionWasInterrupted(_ session: ARSession) {
        trackingMessage = "Session interrupted"
        isTracking = false
    }

    func sessionInterruptionEnded(_ session: ARSession) {
        start()
    }
}
