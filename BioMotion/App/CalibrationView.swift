import SwiftUI
import Foundation
import ARKit
import RealityKit
import simd
import QuartzCore
import UIKit

/// T-pose calibration flow with live camera preview.
/// User sees themselves, positions into T-pose, then taps Capture.
struct CalibrationView: View {
    @ObservedObject var bodyTracking: BodyTrackingSession
    @ObservedObject var nimble: NimbleEngine
    let onComplete: () -> Void
    let onUseOffline: () -> Void

    @Environment(\.openURL) private var openURL

    @State private var phase: CalibrationPhase = .livePreview
    @State private var capturedHeight: Double?
    @State private var calibrationFrames: [BodyFrame] = []
    @State private var captureAccumulator: CalibrationCaptureAccumulator?
    @State private var timer: Timer?
    @State private var qualityScore: CalibrationQualityScore?
    @State private var manualHeightText: String = ""
    @State private var scaleTask: Task<Void, Never>?
    @State private var scaleAttemptID = 0

    enum CalibrationPhase: Equatable {
        case livePreview          // Live camera + skeleton, user positions themselves
        case capturing            // Brief capture (~2s)
        case captureInterrupted   // Tracking disappeared before 60 unique frames
        case captureTimedOut      // Frozen/sparse stream never produced 60 unique frames
        case heightEntryNeeded    // Height couldn't be estimated (or was implausible) — retry or enter manually
        case insufficientQuality  // Per-joint quality gate failed — must redo, no silent degraded scaling
        case applyingScale        // Native scaling is FIFO-async; wait for its real result
        case scaleFailed          // Native mutation/admission failed — never claim success
        case done                 // Show results
    }

    private var cameraControls: CalibrationCameraControls {
        .resolve(cameraState: bodyTracking.cameraState, isModelLoaded: nimble.isModelLoaded)
    }

    private var cameraStatusColor: Color {
        switch bodyTracking.cameraState {
        case .tracking: return .green
        case .permissionDenied, .restricted, .unsupported, .failed: return .red
        default: return .orange
        }
    }

    /// Whether the text currently in `manualHeightText` is a plausible height, i.e. safe
    /// to use in place of the failed automatic estimate.
    private var isManualHeightValid: Bool {
        guard let value = CalibrationHeightParser.parse(manualHeightText) else { return false }
        return CalibrationCalculator.isPlausibleHeight(value)
    }

    var body: some View {
        ZStack {
            // Live camera feed with skeleton — always visible
            SkeletonARView(
                session: bodyTracking.arSession,
                currentFrame: $bodyTracking.currentFrame,
                isTracking: bodyTracking.isTracking,
                anatomyPresentation: LiveAnatomyPresentation(
                    surface: .calibration,
                    isTracking: bodyTracking.isTracking,
                    hasCurrentFrame: bodyTracking.currentFrame != nil,
                    isEnabled: true
                )
            )
            .ignoresSafeArea()

            // Overlay UI on top of camera
            VStack(spacing: 0) {
                // Top: status + instructions
                VStack(spacing: 8) {
                    Text("Calibration")
                        .font(.title2.bold())
                        .foregroundStyle(.white)

                    HStack(spacing: 6) {
                        Circle().fill(cameraStatusColor).frame(width: 8, height: 8)
                        Text(bodyTracking.trackingMessage)
                            .font(.caption)
                            .foregroundStyle(cameraStatusColor)
                    }

                    Text("Stand in a T-pose facing the camera")
                        .font(.callout)
                        .foregroundStyle(.white.opacity(0.8))
                }
                .padding(.top, 16)
                .padding(.horizontal)
                .background(
                    LinearGradient(colors: [.black.opacity(0.7), .clear],
                                   startPoint: .top, endPoint: .bottom)
                    .ignoresSafeArea()
                )

                Spacer()

                // Center: T-pose guide overlay (when not yet tracking)
                if !bodyTracking.isTracking {
                    Image(systemName: "figure.arms.open")
                        .font(.system(size: 120))
                        .foregroundStyle(.white.opacity(0.2))
                }

                Spacer()

                // Bottom: action buttons
                VStack(spacing: 12) {
                    switch phase {
                    case .livePreview:
                        Button {
                            startCapturing()
                        } label: {
                            HStack(spacing: 8) {
                                Image(systemName: "camera.fill")
                                Text("Capture T-Pose")
                            }
                            .font(.headline)
                            .foregroundStyle(.white)
                            .padding(.horizontal, 32)
                            .padding(.vertical, 14)
                            .background(
                                cameraControls.canCapture ? Color.blue : Color.gray,
                                in: RoundedRectangle(cornerRadius: 12)
                            )
                        }
                        .disabled(!cameraControls.canCapture)

                        if bodyTracking.cameraState == .tracking && !nimble.isModelLoaded {
                            HStack(spacing: 8) {
                                ProgressView().tint(.white)
                                Text("Loading biomechanical model...")
                            }
                            .font(.caption)
                            .foregroundStyle(.white.opacity(0.8))
                        }

                        if cameraControls.showsSettings {
                            Button("Open Camera Settings") {
                                guard let url = URL(string: UIApplication.openSettingsURLString) else {
                                    return
                                }
                                openURL(url)
                            }
                            .buttonStyle(.borderedProminent)
                        }

                        if cameraControls.showsRetry {
                            Button("Retry Camera") { bodyTracking.retry() }
                                .buttonStyle(.bordered)
                                .tint(.white)
                        }

                        if cameraControls.showsOffline {
                            Button("Use Offline Video") { useOffline() }
                                .buttonStyle(.bordered)
                                .tint(.white)
                        }

                        Button("Skip Calibration") {
                            onComplete()
                        }
                        .foregroundStyle(.white.opacity(0.5))
                        .font(.callout)

                    case .capturing:
                        HStack(spacing: 12) {
                            ProgressView()
                                .tint(.white)
                            Text("Capturing... \(calibrationFrames.count)/60 frames")
                                .font(.callout)
                                .foregroundStyle(.white)
                        }
                        .padding(.horizontal, 24)
                        .padding(.vertical, 12)
                        .background(.blue.opacity(0.8), in: Capsule())

                        Text("Hold your pose")
                            .font(.caption)
                            .foregroundStyle(.white.opacity(0.6))

                    case .captureInterrupted:
                        VStack(spacing: 10) {
                            Image(systemName: "person.crop.circle.badge.exclamationmark")
                                .font(.system(size: 36))
                                .foregroundStyle(.orange)
                            Text("Tracking was lost")
                                .font(.headline)
                                .foregroundStyle(.white)
                            Text("No partial or repeated frames were used. Re-enter the full T-pose and try again.")
                                .font(.caption)
                                .foregroundStyle(.white.opacity(0.7))
                                .multilineTextAlignment(.center)
                            Button("Retry Capture") { resetAttempt() }
                                .buttonStyle(.borderedProminent)
                        }

                    case .captureTimedOut:
                        VStack(spacing: 10) {
                            Image(systemName: "clock.badge.exclamationmark")
                                .font(.system(size: 36))
                                .foregroundStyle(.orange)
                            Text("Not enough new camera frames")
                                .font(.headline)
                                .foregroundStyle(.white)
                            Text("The camera stream stalled or tracking was too sparse. A frozen frame is never counted twice.")
                                .font(.caption)
                                .foregroundStyle(.white.opacity(0.7))
                                .multilineTextAlignment(.center)
                            Button("Retry Capture") { resetAttempt() }
                                .buttonStyle(.borderedProminent)
                            Button("Use Offline Video") { useOffline() }
                                .foregroundStyle(.white.opacity(0.8))
                        }

                    case .heightEntryNeeded:
                        VStack(spacing: 12) {
                            Image(systemName: "exclamationmark.triangle.fill")
                                .font(.system(size: 36))
                                .foregroundStyle(.orange)

                            Text("Couldn't measure height reliably")
                                .font(.headline)
                                .foregroundStyle(.white)

                            Text("Head or ankle tracking was lost during the hold. Redo the T-pose, or enter your height manually.")
                                .font(.caption)
                                .foregroundStyle(.white.opacity(0.7))
                                .multilineTextAlignment(.center)
                                .padding(.horizontal, 24)

                            HStack(spacing: 8) {
                                TextField("Height in meters, e.g. 1.75", text: $manualHeightText)
                                    .keyboardType(.decimalPad)
                                    .textFieldStyle(.roundedBorder)
                                    .frame(width: 190)

                                Button("Use") {
                                    submitManualHeight()
                                }
                                .disabled(!isManualHeightValid)
                            }

                            Button("Redo Calibration") {
                                resetAttempt()
                            }
                            .foregroundStyle(.white.opacity(0.7))
                            .font(.callout)
                        }

                    case .insufficientQuality:
                        VStack(spacing: 12) {
                            Image(systemName: "exclamationmark.triangle.fill")
                                .font(.system(size: 36))
                                .foregroundStyle(.orange)

                            Text("Calibration quality too low")
                                .font(.headline)
                                .foregroundStyle(.white)

                            if let score = qualityScore {
                                if !score.rejectedJoints.isEmpty {
                                    Text("\(score.rejectedJointCount) joint(s) lost tracking during the hold: \(score.rejectedJointNames.joined(separator: ", ")).")
                                        .font(.caption)
                                        .foregroundStyle(.white.opacity(0.7))
                                        .multilineTextAlignment(.center)
                                        .padding(.horizontal, 24)
                                } else {
                                    Text("Tracking was unstable — position drifted during the hold. Hold very still in T-pose.")
                                        .font(.caption)
                                        .foregroundStyle(.white.opacity(0.7))
                                        .multilineTextAlignment(.center)
                                        .padding(.horizontal, 24)
                                }
                            }

                            Button {
                                resetAttempt()
                            } label: {
                                Text("Redo Calibration")
                                    .font(.callout)
                                    .foregroundStyle(.white)
                                    .padding(.horizontal, 24)
                                    .padding(.vertical, 10)
                                    .background(.gray.opacity(0.6), in: RoundedRectangle(cornerRadius: 10))
                            }
                        }

                    case .applyingScale:
                        HStack(spacing: 12) {
                            ProgressView().tint(.white)
                            Text("Scaling biomechanical model...")
                                .font(.callout)
                                .foregroundStyle(.white)
                        }
                        .padding(.horizontal, 24)
                        .padding(.vertical, 12)
                        .background(.blue.opacity(0.8), in: Capsule())

                    case .scaleFailed:
                        VStack(spacing: 10) {
                            Image(systemName: "xmark.circle.fill")
                                .font(.system(size: 38))
                                .foregroundStyle(.red)
                            Text("Model scaling failed")
                                .font(.headline)
                                .foregroundStyle(.white)
                            Text("Your calibration was not applied, so tracking has not started with stale body geometry.")
                                .font(.caption)
                                .foregroundStyle(.white.opacity(0.7))
                                .multilineTextAlignment(.center)

                            if let height = capturedHeight {
                                Button("Retry Scaling") { applyScale(height: height) }
                                    .buttonStyle(.borderedProminent)
                            }
                            Button("Redo Calibration") { resetAttempt() }
                                .foregroundStyle(.white.opacity(0.8))
                            Button("Use Offline Video") { useOffline() }
                                .foregroundStyle(.white.opacity(0.8))
                        }

                    case .done:
                        VStack(spacing: 8) {
                            Image(systemName: "checkmark.circle.fill")
                                .font(.system(size: 40))
                                .foregroundStyle(.green)

                            if let height = capturedHeight {
                                Text(String(format: "Height: %.2f m", height))
                                    .font(.headline)
                                    .foregroundStyle(.white)
                            }

                            if let score = qualityScore {
                                Text(String(format: "Tracking coverage: %.0f%% avg · all %d joints passed quality gate",
                                             score.averageTrackedFraction * 100, score.acceptedJoints.count))
                                    .font(.caption2)
                                    .foregroundStyle(.white.opacity(0.6))
                            }

                            Text("Model scaled to your body")
                                .font(.caption)
                                .foregroundStyle(.white.opacity(0.7))
                        }

                        HStack(spacing: 16) {
                            Button {
                                resetAttempt()
                            } label: {
                                Text("Redo")
                                    .font(.callout)
                                    .foregroundStyle(.white)
                                    .padding(.horizontal, 24)
                                    .padding(.vertical, 10)
                                    .background(.gray.opacity(0.6), in: RoundedRectangle(cornerRadius: 10))
                            }

                            Button {
                                onComplete()
                            } label: {
                                HStack(spacing: 6) {
                                    Text("Start Tracking")
                                    Image(systemName: "arrow.right")
                                }
                                .font(.headline)
                                .foregroundStyle(.white)
                                .padding(.horizontal, 24)
                                .padding(.vertical, 10)
                                .background(.green, in: RoundedRectangle(cornerRadius: 10))
                            }
                        }
                    }
                }
                .padding(.bottom, 40)
                .padding(.horizontal)
                .background(
                    LinearGradient(colors: [.clear, .black.opacity(0.7)],
                                   startPoint: .top, endPoint: .bottom)
                    .ignoresSafeArea()
                )
            }
        }
        .onChange(of: bodyTracking.cameraState) { _, state in
            if phase == .capturing, !state.isTracking {
                stopCapture()
                phase = .captureInterrupted
            }
        }
        .onDisappear { cleanUpAttempt() }
    }

    // MARK: - Calibration Logic (thin view-side glue over CalibrationCalculator)

    private func startCapturing() {
        guard cameraControls.canCapture else { return }
        stopCapture()
        phase = .capturing
        calibrationFrames.removeAll()
        captureAccumulator = CalibrationCaptureAccumulator(
            targetFrameCount: 60,
            timeout: 6.0,
            startedAt: CACurrentMediaTime()
        )

        timer = Timer.scheduledTimer(withTimeInterval: 1.0 / 30.0, repeats: true) { _ in
            guard var capture = captureAccumulator else { return }
            let outcome = capture.observe(
                frame: bodyTracking.currentFrame,
                isTracking: bodyTracking.cameraState.isTracking,
                now: CACurrentMediaTime()
            )
            captureAccumulator = capture
            calibrationFrames = capture.frames

            switch outcome {
            case .waiting:
                break
            case .completed:
                stopCapture()
                processCalibration()
            case .trackingLost:
                stopCapture()
                phase = .captureInterrupted
            case .timedOut:
                stopCapture()
                phase = .captureTimedOut
            }
        }
    }

    private func stopCapture() {
        timer?.invalidate()
        timer = nil
    }

    private func resetAttempt() {
        stopCapture()
        scaleAttemptID &+= 1
        scaleTask?.cancel()
        scaleTask = nil
        captureAccumulator = nil
        calibrationFrames.removeAll()
        qualityScore = nil
        capturedHeight = nil
        manualHeightText = ""
        phase = .livePreview
    }

    private func cleanUpAttempt() {
        stopCapture()
        scaleAttemptID &+= 1
        scaleTask?.cancel()
        scaleTask = nil
        captureAccumulator = nil
    }

    private func processCalibration() {
        guard !calibrationFrames.isEmpty else {
            phase = .captureTimedOut
            return
        }

        // The per-joint quality gate (below) operates on this hips-tracked subset, not the
        // raw capture window — a frame with no root position isn't usable as a pose sample
        // regardless of what any individual limb joint reports.
        let trackedFrames = calibrationFrames.filter { frame in
            frame.joints.contains(where: { $0.id == "hips_joint" && $0.isTracked })
        }

        guard !trackedFrames.isEmpty else {
            phase = .captureInterrupted
            return
        }

        qualityScore = CalibrationCalculator.evaluateCalibration(frames: trackedFrames)

        guard let height = CalibrationCalculator.estimateFloorToCrownHeight(from: trackedFrames),
              CalibrationCalculator.isPlausibleHeight(height) else {
            // No silent 1.75m fallback: an unmeasurable or implausible height would bias
            // every downstream segment scale factor. Ask the user to retry or enter it.
            phase = .heightEntryNeeded
            return
        }

        applyScale(height: height)
    }

    private func submitManualHeight() {
        guard let height = CalibrationHeightParser.parse(manualHeightText),
              CalibrationCalculator.isPlausibleHeight(height) else {
            return
        }
        applyScale(height: height)
    }

    private func useOffline() {
        // Stop polling and fence any late native-scale completion before the
        // parent switches coordinate sessions and presents offline import.
        cleanUpAttempt()
        onUseOffline()
    }

    /// Applies a validated height (auto-estimated or manually entered) against the
    /// already-computed quality score. Joints that failed the per-joint gate never reach
    /// `nimble.scaleLiveModel` — see `CalibrationQualityScore.acceptedJoints`.
    private func applyScale(height: Double) {
        guard let score = qualityScore else { return }
        capturedHeight = height

        guard score.isAcceptable else {
            // Insufficient coverage or unstable tracking must not silently degrade every
            // downstream scale factor — surface it and require a redo.
            phase = .insufficientQuality
            return
        }

        let markerNames = score.acceptedJoints.map(\.mapping.opensimName)
        var markerPositions: [Float] = []
        markerPositions.reserveCapacity(score.acceptedJoints.count * 3)
        for sample in score.acceptedJoints {
            markerPositions.append(sample.averagePosition.x)
            markerPositions.append(sample.averagePosition.y)
            markerPositions.append(sample.averagePosition.z)
        }

        scaleAttemptID &+= 1
        let attemptID = scaleAttemptID
        scaleTask?.cancel()
        phase = .applyingScale
        scaleTask = Task { @MainActor in
            let result = await nimble.scaleLiveModel(
                height: height,
                markerPositions: markerPositions,
                markerNames: markerNames
            )
            guard scaleAttemptID == attemptID else { return }
            scaleTask = nil
            switch result {
            case .applied:
                phase = .done
            case .nativeFailure, .rejected:
                phase = .scaleFailed
            }
        }
    }
}

/// A capture accepts a frame only when both identities move forward. Polling a
/// frozen `currentFrame` therefore cannot manufacture temporal observations.
struct CalibrationFrameIdentityGate {
    private var lastFrameNumber: Int?
    private var lastTimestamp: TimeInterval?

    mutating func accepts(_ frame: BodyFrame) -> Bool {
        guard frame.timestamp.isFinite else { return false }
        if let lastFrameNumber, frame.frameNumber <= lastFrameNumber { return false }
        if let lastTimestamp, frame.timestamp <= lastTimestamp { return false }
        lastFrameNumber = frame.frameNumber
        lastTimestamp = frame.timestamp
        return true
    }
}

struct CalibrationCaptureAccumulator {
    enum Outcome: Equatable {
        case waiting
        case completed
        case trackingLost
        case timedOut
    }

    let targetFrameCount: Int
    let timeout: TimeInterval
    let startedAt: TimeInterval
    private(set) var frames: [BodyFrame] = []
    private var identityGate = CalibrationFrameIdentityGate()

    init(targetFrameCount: Int, timeout: TimeInterval, startedAt: TimeInterval) {
        self.targetFrameCount = targetFrameCount
        self.timeout = timeout
        self.startedAt = startedAt
    }

    mutating func observe(
        frame: BodyFrame?,
        isTracking: Bool,
        now: TimeInterval
    ) -> Outcome {
        guard isTracking else { return .trackingLost }
        guard now.isFinite, now - startedAt < timeout else { return .timedOut }
        guard let frame, identityGate.accepts(frame) else { return .waiting }
        frames.append(frame)
        return frames.count >= targetFrameCount ? .completed : .waiting
    }
}

struct CalibrationCameraControls: Equatable {
    let canCapture: Bool
    let showsSettings: Bool
    let showsRetry: Bool
    let showsOffline: Bool

    static func resolve(
        cameraState: LiveCameraState,
        isModelLoaded: Bool
    ) -> Self {
        let recoverableFailure: Bool
        switch cameraState {
        case .interrupted, .paused, .permissionDenied, .failed:
            recoverableFailure = true
        default:
            recoverableFailure = false
        }

        let needsOfflineAlternative: Bool
        switch cameraState {
        case .permissionDenied, .restricted, .unsupported, .failed:
            needsOfflineAlternative = true
        default:
            needsOfflineAlternative = false
        }

        return Self(
            canCapture: cameraState.isTracking && isModelLoaded,
            showsSettings: cameraState == .permissionDenied,
            showsRetry: recoverableFailure,
            showsOffline: needsOfflineAlternative
        )
    }
}

enum CalibrationHeightParser {
    static func parse(_ text: String, locale: Locale = .current) -> Double? {
        let formatter = NumberFormatter()
        formatter.locale = locale
        formatter.numberStyle = .decimal
        formatter.isLenient = false

        let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty,
              let value = formatter.number(from: trimmed)?.doubleValue,
              value.isFinite else {
            return nil
        }
        return value
    }
}

// MARK: - Pure calibration math (extracted for unit testing; no SwiftUI/ARKit dependency
// beyond the plain `BodyFrame`/`TrackedJoint` value types).

/// Tunable thresholds for calibration quality gating, kept as named constants so the
/// reasoning behind each number is documented in one place.
enum CalibrationQualityConstants {
    /// A joint must be tracked in at least this fraction of the capture window to be
    /// trusted as an anthropometric scale reference. 70% tolerates brief occlusion (e.g.
    /// a wrist swinging behind the torso mid-hold) while still rejecting joints that were
    /// mostly invisible — their few surviving samples are disproportionately likely to be
    /// tracking-noise outliers rather than genuine T-pose positions.
    static let minTrackedFraction: Double = 0.70

    /// Absolute floor on top of the fraction gate. Protects against a short capture window
    /// (e.g. the app was backgrounded mid-hold and only a handful of frames were recorded)
    /// where a tiny total frame count could let a handful of noisy samples clear the 70%
    /// bar despite being far too few to average reliably.
    static let minTrackedSampleCount: Int = 20

    /// Per-joint position variance (m²) above which the T-pose hold is considered unstable
    /// even though the joint cleared the tracked-fraction gate — i.e. the subject moved, or
    /// tracking was jittery. ~2cm std-dev (variance ≈ 4e-4) is generous slack above ARKit's
    /// typical sub-millimeter jitter for a genuinely still subject.
    static let maxAcceptablePositionVariance: Float = 4e-4

    /// Plausible human floor-to-crown height range, in meters. Used as a VALIDATOR only —
    /// anything outside this range is rejected outright, never silently replaced.
    static let minPlausibleHeight: Double = 1.0
    static let maxPlausibleHeight: Double = 2.5
}

/// Result of averaging one mapped joint's tracked positions across the capture window.
struct JointCalibrationSample {
    let mapping: JointMapping.Mapping
    let averagePosition: SIMD3<Float>
    let trackedCount: Int
    let totalFrames: Int
    /// Mean squared distance of each tracked sample from `averagePosition` — a variance
    /// proxy. High variance during what should be a static T-pose hold means the subject
    /// moved or tracking was unstable, independent of how many frames were tracked.
    let positionVariance: Float

    var trackedFraction: Double {
        totalFrames > 0 ? Double(trackedCount) / Double(totalFrames) : 0
    }

    /// Whether this joint has enough tracked, temporally-spread samples to serve as a
    /// scale reference at all. Deliberately does NOT gate on `positionVariance` — that is
    /// reported separately in `CalibrationQualityScore` so the caller can distinguish
    /// "not enough data" from "data present but unstable".
    var passesTrackingGate: Bool {
        trackedFraction >= CalibrationQualityConstants.minTrackedFraction &&
        trackedCount >= CalibrationQualityConstants.minTrackedSampleCount
    }
}

/// Aggregate, user-facing summary of one calibration attempt.
struct CalibrationQualityScore {
    let acceptedJoints: [JointCalibrationSample]
    let rejectedJoints: [JointCalibrationSample]
    let averageTrackedFraction: Double
    let maxPositionVariance: Float

    var rejectedJointCount: Int { rejectedJoints.count }
    var rejectedJointNames: [String] { rejectedJoints.map(\.mapping.displayName) }

    /// A calibration is acceptable only if every mapped joint cleared the per-joint gate
    /// AND the least-stable accepted joint stayed within the variance budget. Either
    /// failure means at least one downstream segment scale factor would be biased.
    var isAcceptable: Bool {
        rejectedJoints.isEmpty &&
        maxPositionVariance <= CalibrationQualityConstants.maxAcceptablePositionVariance
    }
}

enum CalibrationCalculator {
    /// Averages one mapped joint's tracked positions across `frames` and computes its
    /// tracked fraction + position variance. Returns a sample even when `trackedCount == 0`
    /// so callers can report *why* a joint was rejected; `passesTrackingGate` is what
    /// actually decides acceptance (see (A): joints failing the gate must not silently
    /// contribute to `markerPositions`/`markerNames`).
    static func averageJointPosition(mapping: JointMapping.Mapping, frames: [BodyFrame]) -> JointCalibrationSample {
        var positions: [SIMD3<Float>] = []
        for frame in frames {
            if let joint = frame.joints.first(where: { $0.id == mapping.arkitName }), joint.isTracked {
                positions.append(joint.worldPosition)
            }
        }

        guard !positions.isEmpty else {
            return JointCalibrationSample(mapping: mapping, averagePosition: .zero,
                                           trackedCount: 0, totalFrames: frames.count, positionVariance: 0)
        }

        let sum = positions.reduce(SIMD3<Float>.zero, +)
        let average = sum / Float(positions.count)
        let variance = positions.reduce(Float(0)) { acc, p in
            let d = p - average
            return acc + (d.x * d.x + d.y * d.y + d.z * d.z)
        } / Float(positions.count)

        return JointCalibrationSample(mapping: mapping, averagePosition: average,
                                       trackedCount: positions.count, totalFrames: frames.count,
                                       positionVariance: variance)
    }

    /// Runs the per-joint gate across every primary mapping and produces the aggregate
    /// quality score. The view already admits only monotonic frames; this second gate is
    /// defense in depth for direct callers and future capture implementations.
    static func evaluateCalibration(frames: [BodyFrame]) -> CalibrationQualityScore {
        var identityGate = CalibrationFrameIdentityGate()
        let uniqueFrames = frames.filter { identityGate.accepts($0) }
        let samples = JointMapping.primary.map {
            averageJointPosition(mapping: $0, frames: uniqueFrames)
        }
        let accepted = samples.filter(\.passesTrackingGate)
        let rejected = samples.filter { !$0.passesTrackingGate }

        let averageFraction = samples.isEmpty ? 0 :
            samples.reduce(0.0) { $0 + $1.trackedFraction } / Double(samples.count)
        let maxVariance = accepted.map(\.positionVariance).max() ?? 0

        return CalibrationQualityScore(acceptedJoints: accepted, rejectedJoints: rejected,
                                        averageTrackedFraction: averageFraction, maxPositionVariance: maxVariance)
    }

    /// Estimates floor-to-crown height from head and ankle joints. This is a floor-to-crown
    /// estimate, NOT a stature measurement: it ignores shoe sole thickness and any ARKit
    /// camera pitch bias, both of which can shift the result by a few centimeters. Per-frame
    /// samples outside the plausible range are treated as tracking glitches and excluded
    /// from the average (data cleaning, not substitution). Returns nil if no frame yields a
    /// plausible sample — callers must treat nil as an explicit failure (retry or manual
    /// entry), never substitute a default.
    static func estimateFloorToCrownHeight(from frames: [BodyFrame]) -> Double? {
        var heights: [Float] = []

        for frame in frames {
            let head = frame.joints.first(where: { $0.id == "head_joint" })
            let leftAnkle = frame.joints.first(where: { $0.id == "left_foot_joint" })
            let rightAnkle = frame.joints.first(where: { $0.id == "right_foot_joint" })

            guard let h = head, h.isTracked else { continue }

            let ankleY: Float
            if let la = leftAnkle, la.isTracked, let ra = rightAnkle, ra.isTracked {
                ankleY = min(la.worldPosition.y, ra.worldPosition.y)
            } else if let la = leftAnkle, la.isTracked {
                ankleY = la.worldPosition.y
            } else if let ra = rightAnkle, ra.isTracked {
                ankleY = ra.worldPosition.y
            } else {
                continue
            }

            let height = h.worldPosition.y - ankleY + 0.08
            if isPlausibleHeight(Double(height)) {
                heights.append(height)
            }
        }

        guard !heights.isEmpty else { return nil }
        return Double(heights.reduce(0, +) / Float(heights.count))
    }

    /// Sanity check ONLY — a VALIDATOR, never a value substitute. A height outside this
    /// range signals something wrong upstream (bad tracking, subject not fully framed);
    /// the caller must prompt for retry or manual entry rather than silently using it.
    static func isPlausibleHeight(_ height: Double) -> Bool {
        height >= CalibrationQualityConstants.minPlausibleHeight &&
        height <= CalibrationQualityConstants.maxPlausibleHeight
    }
}
