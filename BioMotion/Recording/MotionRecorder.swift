import Foundation
import QuartzCore

/// Immutable identity and shared clock origin for one atomic live capture.
/// ARKit frame timestamps and `CACurrentMediaTime()` use the same uptime clock.
struct CaptureEpoch: Hashable, Sendable {
    let id: UUID
    let timeOrigin: TimeInterval

    init(id: UUID = UUID(), timeOrigin: TimeInterval) {
        self.id = id
        self.timeOrigin = timeOrigin
    }

    /// Compatibility for engine-only tests that predate coordinated capture.
    /// Product recording always supplies a unique, finite real origin.
    static let legacy = CaptureEpoch(
        id: UUID(uuidString: "00000000-0000-0000-0000-000000000000")!,
        timeOrigin: -.greatestFiniteMagnitude
    )
}

/// Pure capture-ownership policy shared by solver admission and publication.
/// Keeping the decision value-only makes the A→B late-result race testable
/// without adding a test-only blocking hook to the native physics bridge.
enum RecordingCapturePolicy {
    /// Reject a live frame timestamped before the record-button epoch before
    /// it can enter native IK warm-start, SG filters, or motion history.
    /// Offline analysis has an independent timestamp domain, while ordinary
    /// unrecorded live preview must remain available.
    static func mayProcessFrame(activeEpoch: CaptureEpoch?,
                                frameTimestamp: TimeInterval,
                                isOfflineSubmission: Bool) -> Bool {
        guard !isOfflineSubmission, let activeEpoch else { return true }
        return activeEpoch == .legacy || frameTimestamp >= activeEpoch.timeOrigin
    }

    static func epochForSubmission(activeEpoch: CaptureEpoch?,
                                   frameTimestamp: TimeInterval,
                                   isOfflineSubmission: Bool) -> CaptureEpoch? {
        guard !isOfflineSubmission,
              let activeEpoch,
              activeEpoch == .legacy || frameTimestamp >= activeEpoch.timeOrigin else {
            return nil
        }
        return activeEpoch
    }

    static func mayPublish(submissionEpoch: CaptureEpoch?,
                           activeEpoch: CaptureEpoch?,
                           isArmed: Bool,
                           hasMotion: Bool) -> Bool {
        isArmed
            && hasMotion
            && submissionEpoch != nil
            && submissionEpoch == activeEpoch
    }
}

/// Records body frames over a session for later export.
final class MotionRecorder: ObservableObject {
    static let maximumFrameCount = 3_600
    static let maximumDuration: TimeInterval = 60

    enum FrameDisposition: Equatable {
        case ignored
        case recorded
        case recordedAndStoppedAtLimit
    }

    @Published var isRecording = false
    @Published var recordedFrameCount = 0
    @Published var duration: TimeInterval = 0

    private(set) var frames: [BodyFrame] = []
    private(set) var captureEpoch: CaptureEpoch?
    private var startTime: TimeInterval?

    /// Starts a new take only when doing so cannot silently erase a completed
    /// one. The UI may call `discardRecording()` after explicit confirmation.
    @discardableResult
    func startRecording(for captureEpoch: CaptureEpoch = .legacy) -> Bool {
        guard !hasRecording else { return false }
        frames.removeAll()
        recordedFrameCount = 0
        duration = 0
        startTime = nil
        self.captureEpoch = captureEpoch
        isRecording = true
        return true
    }

    func stopRecording() {
        isRecording = false
    }

    @discardableResult
    func recordFrame(_ frame: BodyFrame,
                     for suppliedEpoch: CaptureEpoch? = nil) -> FrameDisposition {
        guard isRecording,
              let recordingEpoch = captureEpoch else { return .ignored }

        if recordingEpoch == .legacy {
            guard suppliedEpoch == nil || suppliedEpoch == recordingEpoch else {
                return .ignored
            }
        } else {
            guard suppliedEpoch == recordingEpoch,
                  frame.timestamp >= recordingEpoch.timeOrigin else {
                return .ignored
            }
        }

        if recordingEpoch == .legacy, startTime == nil {
            startTime = frame.timestamp
        }

        let durationOrigin = recordingEpoch == .legacy
            ? (startTime ?? frame.timestamp)
            : recordingEpoch.timeOrigin
        let nextDuration = frame.timestamp - durationOrigin
        guard frames.count < Self.maximumFrameCount,
              nextDuration <= Self.maximumDuration else {
            isRecording = false
            return .ignored
        }

        frames.append(frame)
        recordedFrameCount = frames.count
        duration = nextDuration

        if frames.count == Self.maximumFrameCount
            || duration >= Self.maximumDuration {
            isRecording = false
            return .recordedAndStoppedAtLimit
        }
        return .recorded
    }

    /// Destructive replacement is deliberately separate from `startRecording`
    /// so a button tap cannot erase an unexported take by accident.
    func discardRecording() {
        guard !isRecording else { return }
        frames.removeAll(keepingCapacity: false)
        recordedFrameCount = 0
        duration = 0
        startTime = nil
        captureEpoch = nil
    }

    var hasRecording: Bool {
        !frames.isEmpty
    }

    /// Average frame rate of the recording.
    var averageFPS: Double {
        guard frames.count > 1,
              let first = frames.first,
              let last = frames.last else { return 0 }
        let elapsed = last.timestamp - first.timestamp
        guard elapsed > 0 else { return 0 }
        return Double(frames.count - 1) / elapsed
    }
}

/// One main-actor owner for the marker recorder and asynchronous solver
/// recorder. Every start/stop boundary passes through here so neither side can
/// remain armed after tracking loss, offline import, backgrounding or a limit.
@MainActor
final class LiveRecordingCoordinator: ObservableObject {
    enum StartResult: Equatable {
        case started(CaptureEpoch)
        case blockedByUnexportedCapture
        case unavailable
    }

    enum StopReason: Equatable {
        case user
        case frameLimit
        case trackingLost
        case offlineImport
        case appInactive
        case solverReset
    }

    @Published private(set) var activeEpoch: CaptureEpoch?
    @Published private(set) var hasUnexportedCapture = false
    @Published private(set) var lastStopReason: StopReason?
    private(set) var completedSnapshot: CaptureExportSnapshot?
    private(set) var completedSnapshotError: CaptureExportSnapshot.SnapshotError?
    private var unexportedEpoch: CaptureEpoch?

    func start(recorder: MotionRecorder,
               nimble: NimbleEngine,
               timeOrigin: TimeInterval = CACurrentMediaTime()) -> StartResult {
        if recorder.hasRecording {
            guard !hasUnexportedCapture else {
                return .blockedByUnexportedCapture
            }
            recorder.discardRecording()
            completedSnapshot = nil
            completedSnapshotError = nil
        }

        guard nimble.isModelLoaded else { return .unavailable }

        // A capture owns its own temporal filter window. Without this reset a
        // new epoch's first centred IK row could still depend on marker samples
        // from the previous take even though its history arrays were cleared.
        guard nimble.resetRealtimeState() else { return .unavailable }

        let epoch = CaptureEpoch(timeOrigin: timeOrigin)
        guard recorder.startRecording(for: epoch) else { return .unavailable }
        nimble.startRecordingResults(for: epoch)
        activeEpoch = epoch
        completedSnapshot = nil
        completedSnapshotError = nil
        hasUnexportedCapture = false
        lastStopReason = nil
        return .started(epoch)
    }

    func discardAndStart(recorder: MotionRecorder,
                         nimble: NimbleEngine,
                         timeOrigin: TimeInterval = CACurrentMediaTime()) -> StartResult {
        stop(recorder: recorder, nimble: nimble, reason: .user)
        recorder.discardRecording()
        completedSnapshot = nil
        completedSnapshotError = nil
        hasUnexportedCapture = false
        unexportedEpoch = nil
        return start(recorder: recorder, nimble: nimble, timeOrigin: timeOrigin)
    }

    func process(_ frame: BodyFrame,
                 recorder: MotionRecorder,
                 nimble: NimbleEngine) {
        if activeEpoch != nil, !nimble.recordingResultsAreArmed {
            stop(recorder: recorder, nimble: nimble, reason: .solverReset)
            _ = nimble.processFrame(frame)
            return
        }
        let hadActiveCapture = activeEpoch != nil
        _ = recorder.recordFrame(frame, for: activeEpoch)
        _ = nimble.processFrame(frame)
        if hadActiveCapture && !recorder.isRecording {
            stop(recorder: recorder, nimble: nimble, reason: .frameLimit)
        }
    }

    func stop(recorder: MotionRecorder,
              nimble: NimbleEngine,
              reason: StopReason) {
        let stoppedEpoch = activeEpoch
        recorder.stopRecording()
        nimble.stopRecordingResults()
        activeEpoch = nil
        lastStopReason = reason
        if let stoppedEpoch, recorder.hasRecording {
            do {
                completedSnapshot = try CaptureExportSnapshot(
                    markerEpoch: recorder.captureEpoch,
                    resultEpoch: nimble.resultHistoryEpoch,
                    frames: recorder.frames,
                    ikHistory: nimble.ikHistory,
                    idHistory: nimble.idHistory,
                    isModelLoaded: nimble.isModelLoaded,
                    hasValidatedFootContactSupport: nimble.hasValidatedFootContactSupport
                )
                completedSnapshotError = nil
            } catch let error as CaptureExportSnapshot.SnapshotError {
                completedSnapshot = nil
                completedSnapshotError = error
            } catch {
                assertionFailure("unexpected capture snapshot error: \(error)")
                completedSnapshot = nil
                completedSnapshotError = .missingEpoch
            }
            hasUnexportedCapture = true
            unexportedEpoch = stoppedEpoch
        }
    }

    func markExported(epoch: CaptureEpoch) {
        guard epoch == unexportedEpoch else { return }
        hasUnexportedCapture = false
        unexportedEpoch = nil
    }
}
