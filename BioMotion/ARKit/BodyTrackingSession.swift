import ARKit
import AVFoundation
import Combine
import QuartzCore

enum LiveCameraState: Equatable {
    case idle
    case requestingPermission
    case searching
    case tracking
    case interrupted
    case paused
    case permissionDenied
    case restricted
    case unsupported
    case failed(String)

    var isTracking: Bool { self == .tracking }

    var message: String {
        switch self {
        case .idle: return "Camera is not started"
        case .requestingPermission: return "Waiting for camera permission..."
        case .searching: return "Looking for body..."
        case .tracking: return "Tracking"
        case .interrupted: return "Camera session interrupted"
        case .paused: return "Camera paused"
        case .permissionDenied: return "Camera access is off"
        case .restricted: return "Camera access is restricted"
        case .unsupported: return "Body tracking is not supported on this device"
        case .failed: return "Camera session failed"
        }
    }
}

protocol LiveCameraAuthorizationProviding {
    func authorizationStatus() -> AVAuthorizationStatus
    func requestAccess(_ completion: @escaping (Bool) -> Void)
}

private struct SystemLiveCameraAuthorizationProvider: LiveCameraAuthorizationProviding {
    func authorizationStatus() -> AVAuthorizationStatus {
        AVCaptureDevice.authorizationStatus(for: .video)
    }

    func requestAccess(_ completion: @escaping (Bool) -> Void) {
        AVCaptureDevice.requestAccess(for: .video, completionHandler: completion)
    }
}

/// Manages the ARSession with body tracking configuration.
/// Publishes BodyFrame updates at camera frame rate (~60 fps).
final class BodyTrackingSession: NSObject, ObservableObject {
    let arSession: ARSession

    @Published var currentFrame: BodyFrame?
    @Published private(set) var cameraState: LiveCameraState = .idle

    var isTracking: Bool { cameraState.isTracking }
    var trackingMessage: String { cameraState.message }

    private var frameCount = 0
    private let skeletonFilter = SkeletonFilter(minCutoff: 1.0, beta: 0.007)
    private var lastBodyAnchorUpdateTime: CFTimeInterval?
    private let trackingLossTimeout: CFTimeInterval = 0.35
    private let cameraAuthorization: LiveCameraAuthorizationProviding
    private let bodyTrackingSupport: () -> Bool
    private let runSession: (ARSession, ARBodyTrackingConfiguration) -> Void
    private let pauseSession: (ARSession) -> Void
    /// Main-thread lifecycle fence. A late permission callback must not restart
    /// an AR session after the view paused or superseded that request.
    private var lifecycleGeneration = 0

    override init() {
        arSession = ARSession()
        cameraAuthorization = SystemLiveCameraAuthorizationProvider()
        bodyTrackingSupport = { ARBodyTrackingConfiguration.isSupported }
        runSession = { session, configuration in
            session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        }
        pauseSession = { $0.pause() }
        super.init()
        arSession.delegate = self
    }

    init(
        arSession: ARSession,
        cameraAuthorization: LiveCameraAuthorizationProviding,
        bodyTrackingSupport: @escaping () -> Bool,
        runSession: @escaping (ARSession, ARBodyTrackingConfiguration) -> Void = {
            session, configuration in
            session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        },
        pauseSession: @escaping (ARSession) -> Void = { $0.pause() }
    ) {
        self.arSession = arSession
        self.cameraAuthorization = cameraAuthorization
        self.bodyTrackingSupport = bodyTrackingSupport
        self.runSession = runSession
        self.pauseSession = pauseSession
        super.init()
        arSession.delegate = self
    }

    var isBodyTrackingSupported: Bool {
        bodyTrackingSupport()
    }

    func start() {
        guard Thread.isMainThread else {
            DispatchQueue.main.async { [weak self] in self?.start() }
            return
        }

        lifecycleGeneration &+= 1
        let generation = lifecycleGeneration
        currentFrame = nil

        guard isBodyTrackingSupported else {
            cameraState = .unsupported
            return
        }

        switch cameraAuthorization.authorizationStatus() {
        case .authorized:
            beginTracking(generation: generation)
        case .notDetermined:
            cameraState = .requestingPermission
            cameraAuthorization.requestAccess { [weak self] granted in
                DispatchQueue.main.async {
                    guard let self, self.lifecycleGeneration == generation else { return }
                    if granted {
                        self.beginTracking(generation: generation)
                    } else {
                        self.publishInactive(.permissionDenied)
                    }
                }
            }
        case .denied:
            publishInactive(.permissionDenied)
        case .restricted:
            publishInactive(.restricted)
        @unknown default:
            publishInactive(.failed("Unknown camera authorization state"))
        }
    }

    private func beginTracking(generation: Int) {
        guard lifecycleGeneration == generation else { return }
        let config = ARBodyTrackingConfiguration()
        config.worldAlignment = .gravity
        config.automaticSkeletonScaleEstimationEnabled = true
        config.frameSemantics.insert(.bodyDetection)
        lastBodyAnchorUpdateTime = nil
        skeletonFilter.reset()
        runSession(arSession, config)
        cameraState = .searching
    }

    func pause() {
        guard Thread.isMainThread else {
            DispatchQueue.main.async { [weak self] in self?.pause() }
            return
        }
        lifecycleGeneration &+= 1
        pauseSession(arSession)
        lastBodyAnchorUpdateTime = nil
        skeletonFilter.reset()
        publishInactive(.paused)
    }

    func retry() {
        start()
    }

    private func publishInactive(_ state: LiveCameraState) {
        currentFrame = nil
        cameraState = state
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
            guard self.cameraState == .searching || self.cameraState == .tracking else {
                return
            }
            self.currentFrame = frame
            self.cameraState = .tracking
        }
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard let lastBodyAnchorUpdateTime else { return }
        guard (CACurrentMediaTime() - lastBodyAnchorUpdateTime) > trackingLossTimeout else { return }
        skeletonFilter.reset()
        self.lastBodyAnchorUpdateTime = nil

        DispatchQueue.main.async {
            guard self.cameraState == .tracking else { return }
            self.currentFrame = nil
            self.cameraState = .searching
        }
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        DispatchQueue.main.async {
            guard self.cameraState == .searching ||
                    self.cameraState == .tracking ||
                    self.cameraState == .interrupted else {
                return
            }
            self.lifecycleGeneration &+= 1
            self.lastBodyAnchorUpdateTime = nil
            self.skeletonFilter.reset()
            self.publishInactive(.failed(error.localizedDescription))
        }
    }

    func sessionWasInterrupted(_ session: ARSession) {
        DispatchQueue.main.async {
            guard self.cameraState == .searching || self.cameraState == .tracking else {
                return
            }
            self.lifecycleGeneration &+= 1
            self.lastBodyAnchorUpdateTime = nil
            self.skeletonFilter.reset()
            self.publishInactive(.interrupted)
        }
    }

    func sessionInterruptionEnded(_ session: ARSession) {
        DispatchQueue.main.async {
            guard self.cameraState == .interrupted else { return }
            self.start()
        }
    }
}
