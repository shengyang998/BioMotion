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
/// this file's task brief. It reuses `MuscleOverlay` verbatim — which since
/// 2026-08-08 is an ANATOMY layer keyed by the ARKit joint id strings our
/// `BodyFrame` already supplies, with no activation input and one constant
/// capsule colour. Its old second pass, which drew the strongest 24 muscles by
/// world-space path and coloured them against each other, is gone from both
/// this surface and the live one.
struct OfflinePlaybackView: View {
    @ObservedObject var resultStore: OfflineResultStore
    let onDone: () -> Void

    @State private var showMuscles = true
    @State private var showSourceImage = true

    /// Posture findings for the frame currently on screen.
    ///
    /// Computed from `bodyFrame.joints` — the retargeted marker positions —
    /// and NOT from `ikResult` / `idResult` / `muscleResult`. That is
    /// deliberate and load-bearing: it makes this panel independent of every
    /// open defect downstream of the pose model (STATUS.md records IK landing
    /// on two different solutions from identical markers, and the muscle
    /// solve's redundancy caveats), and it means the numbers describe exactly
    /// the points `PhotoOverlayView` draws on the photo beside them.
    private var findings: PostureReport? {
        guard let frame = resultStore.selectedFrame, let body = frame.bodyFrame else { return nil }
        // A frame rejected by the body-size gate still carries its retargeted
        // skeleton, so the user can see WHAT was wrong on the photo. It must
        // NOT produce findings: every one of them is a distance or an angle on
        // that skeleton, so a half-scale prediction would report half-scale
        // centimetres with no indication they are meaningless.
        if case .implausibleBody = frame.status { return nil }
        // The offline path's joints are in MHRRetarget's camera-aligned frame,
        // so the camera's optical axis is known exactly. The live ARKit path is
        // NOT this frame and must not inherit this constant.
        return PostureFindings.report(joints: body.joints,
                                      cameraDepthAxis: PostureFindings.offlineCameraDepthAxis)
    }

    /// Pure presentation policy, separated so tests can pin that the fixed
    /// anatomy layer does not acquire a dependency on ID, muscle output, gait
    /// comparability, or any other load claim.
    static func anatomyIsVisible(
        showingSourceImage: Bool,
        anatomyEnabled: Bool,
        frame: OfflineResultStore.FrameResult?
    ) -> Bool {
        !showingSourceImage && anatomyEnabled && (frame?.hasDrawableAnatomy ?? false)
    }

    /// **Exactly when muscle capsules are on screen.** The scene and the legend
    /// both read this one value. The capsules are a fixed anatomical layer, so
    /// a successful tracked pose remains drawable even when dynamics are
    /// unavailable.
    private var anatomyCapsulesAreOnScreen: Bool {
        Self.anatomyIsVisible(showingSourceImage: showSourceImage,
                              anatomyEnabled: showMuscles,
                              frame: resultStore.selectedFrame)
    }

    var body: some View {
        VStack(spacing: 0) {
            ZStack {
                // Two views of the same solve, because they answer different
                // questions. The photo overlay answers "is this pose right",
                // which the 3-D scene cannot — a skeleton in an empty scene has
                // nothing to be wrong against. The 3-D scene answers "what does
                // this posture look like from another angle", which a single
                // photo cannot.
                if showSourceImage, let frame = resultStore.selectedFrame {
                    PhotoOverlayView(frame: frame)
                        .ignoresSafeArea(edges: .top)
                } else {
                    OfflineSceneView(frame: resultStore.selectedFrame,
                                     showMuscles: anatomyCapsulesAreOnScreen)
                        .ignoresSafeArea(edges: .top)
                }

                VStack {
                    HStack {
                        Spacer()
                        statusBadge
                    }
                    .padding(8)
                    Spacer()
                    // The legend belongs against the picture, not three blocks
                    // down a scroll view. It is the same sentence the live
                    // screen carries — one constant, so the two cannot drift.
                    if anatomyCapsulesAreOnScreen {
                        Text(MuscleOverlay.anatomyOnlyNote)
                            .font(.caption2)
                            .foregroundStyle(.white.opacity(0.85))
                            .fixedSize(horizontal: false, vertical: true)
                            .padding(8)
                            .background(.black.opacity(0.6), in: RoundedRectangle(cornerRadius: 8))
                            .padding(.horizontal, 8)
                            .padding(.bottom, 8)
                    }
                }
            }

            // Keep the permanent model limit above the scrollable gait or
            // posture panel. A refused panel is capped at 150 pt and begins
            // with clip-specific re-filming advice; placing this notice below
            // it made that advice look capable of unlocking load mechanics.
            if resultStore.hasValidatedFootContactSupport == false {
                contactSupportCapabilityBanner
            }

            // On an ANALYSED running clip the gait panel replaces the posture
            // findings: the findings are single-pose measurements and a runner
            // has no single pose.
            //
            // A REFUSAL does not replace them. `isAboutRunning` is true for
            // every refusal including `.notRunning`, so a side-on squat or a
            // walking clip was routed to a panel headed "Running, but withheld"
            // and lost the measurements it could actually support. The refusal
            // is now a banner above the findings, not a replacement for them.
            //
            // `.notAttempted` shows nothing at all — the gait pass runs on every
            // clip and declines most of them, so treating "declined" as "this is
            // a gait screen" would have put a sentence about strides in front of
            // every photo in the app.
            if let gait = resultStore.gait, gait.replacesPostureFindings {
                GaitReportPanel(outcome: gait,
                                hasValidatedFootContactSupport:
                                    resultStore.hasValidatedFootContactSupport)
                    .frame(maxHeight: 320)
            } else {
                // The refusal banner is NOT conditional on there being findings
                // to sit above: `findings` is nil for a frame the body-size gate
                // rejected, and a user scrubbed onto one of those would otherwise
                // be told nothing at all about why the run was not measured.
                if let gait = resultStore.gait, case .refused = gait {
                    GaitReportPanel(
                        outcome: gait,
                        hasValidatedFootContactSupport:
                            resultStore.hasValidatedFootContactSupport)
                        .frame(maxHeight: 150)
                }
                // Findings sit between the image and the transport controls, with
                // no tap needed: for a single imported photo — the common case —
                // this is the whole reason the user opened the app, so it must be
                // on screen the moment the frame resolves. Capped so the photo
                // above it stays visible; the panel scrolls inside its own bounds.
                if let findings {
                    PostureFindingsPanel(report: findings)
                        .frame(maxHeight: 300)
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

    private var contactSupportCapabilityBanner: some View {
        VStack(alignment: .leading, spacing: 3) {
            Text("Musculoskeletal loads unavailable")
                .font(.caption.weight(.semibold))
            Text(GaitReportPanel.contactSupportUnavailableMessage)
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(.horizontal)
        .padding(.vertical, 8)
    }

    @ViewBuilder
    private var statusBadge: some View {
        if let frame = resultStore.selectedFrame {
            VStack(alignment: .trailing, spacing: 2) {
                Text(Self.frameStatusText(frame))
                    .font(.caption2)
                    .foregroundStyle(statusTint(frame))
                // The reason a frame has no muscle numbers is the point of this
                // badge, not a footnote: "warming up" is a startup artifact,
                // "moving" is a statement about what this input can support.
                if let detail = motionDetail(frame) {
                    Text(detail)
                        .font(.caption2)
                        .foregroundStyle(.white.opacity(0.75))
                }
                if frame.usedFallbackBBox && frame.temporalAnalysisExclusion == nil {
                    Text("no person detected — used full frame")
                        .font(.caption2)
                        .foregroundStyle(.orange)
                }
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .background(.black.opacity(0.6), in: RoundedRectangle(cornerRadius: 8))
        }
    }

    static func frameStatusText(_ frame: OfflineResultStore.FrameResult) -> String {
        if let exclusion = frame.temporalAnalysisExclusion {
            return exclusion.badgeTitle
        }
        switch frame.status {
        case .success:
            switch frame.dynamicsAvailability {
            case .waitingForMotionWindow, .groundPlaneUntrusted,
                 .inverseDynamicsFailed, .missingRootVerticalDOF,
                 .contactSupportUnavailable,
                 .analysisPassIncomplete:
                return frame.dynamicsAvailability.title
            case .withheld, .available:
                break
            }
            if frame.dynamicsAvailability.hasInverseDynamics,
               !frame.hasValidatedDynamicsPayload {
                return NimbleEngine.DynamicsAvailability.inverseDynamicsFailed.title
            }
            // The gait cases come first: on a running clip they are what
            // decided the frame, and the stillness wording ("hold the
            // position") is advice a runner cannot act on.
            if case .gait(let verdict, _) = frame.motionState {
                switch verdict {
                case .gaitStance:
                    // A frame excluded from the load comparison must not be
                    // captioned "relative loads" — the summary has already
                    // discarded it. The optional fixed-colour anatomy layer
                    // makes no load claim and is governed separately.
                    guard frame.gaitLoadsAreComparable else {
                        return "Pose only — foot down, outside the load comparison"
                    }
                    // And a frame INSIDE the comparison must not claim
                    // "relative loads" either, now that no relative load is
                    // published: the muscle solve ran, but the panel says why
                    // its output is not shown. The badge used to say the frame
                    // carried a comparison the panel refused on the same screen.
                    return frame.hasFullBiomechanics
                        ? "Pose — foot down, muscle solve not shown"
                        : "Pose only — foot down, no solve"
                case .gaitFlight: return "Pose only — both feet off the ground"
                case .gaitOutsideAnalysis: return "Pose only — outside the analysed strides"
                case .gaitRefused: return "Pose only — strides too uneven to model"
                default: break
                }
            }
            if frame.hasFullBiomechanics {
                // `isStaticHoldEstimate` now means the ID and muscle solve ran
                // with q̇ = q̈ = 0 on a DETECTED hold, so say so — these are
                // posture loads, not measured dynamics.
                return frame.isStaticHoldEstimate ? "Pose + muscle (static hold)" : "Pose + muscle"
            }
            if frame.isPoseOnlyBecauseNotStill {
                // Naming the reason matters: one of these is something the user
                // can change by holding still, the other is the app failing to
                // resolve movement at all and telling them to hold still would
                // be advice that cannot work.
                guard case .measured(let verdict, _, _, _) = frame.motionState else {
                    return "Pose only"
                }
                switch verdict {
                case .indistinguishableFromNoise:
                    return "Pose only — movement below what this clip can resolve"
                case .poseDidNotConverge:
                    return "Pose only — the skeleton did not settle here"
                default:
                    return "Pose only — subject moving"
                }
            }
            return "Pose only (warming up)"
        case .poseEstimationFailed(let reason):
            return "Pose failed: \(reason)"
        case .implausibleBody:
            // The measured numbers go on screen, not just the verdict: this is
            // the whole difference between "we rejected your photo" and a
            // silent drop the user cannot act on. Built on the model side so it
            // is covered by `BodyPlausibilityTests`.
            return frame.status.implausibleBodyDescription ?? "Body size not measurable"
        case .nimbleTimeout:
            return "Solver timed out on this frame"
        }
    }

    private func statusTint(_ frame: OfflineResultStore.FrameResult) -> Color {
        if frame.temporalAnalysisExclusion != nil { return .orange }
        switch frame.status {
        case .success:
            if frame.hasFullBiomechanics { return .green }
            if frame.dynamicsAvailability.hasInverseDynamics,
               !frame.hasValidatedDynamicsPayload { return .red }
            switch frame.dynamicsAvailability {
            case .inverseDynamicsFailed, .missingRootVerticalDOF,
                 .contactSupportUnavailable, .analysisPassIncomplete: return .red
            case .groundPlaneUntrusted: return .orange
            case .waitingForMotionWindow: return .white
            case .withheld, .available: break
            }
            if frame.isPoseOnlyBecauseNotStill { return .orange }
            return .white
        case .poseEstimationFailed, .nimbleTimeout, .implausibleBody:
            return .red
        }
    }

    /// One line of the actual measurement behind the verdict, so the number is
    /// inspectable rather than a badge the user has to trust.
    private func motionDetail(_ frame: OfflineResultStore.FrameResult) -> String? {
        if let exclusion = frame.temporalAnalysisExclusion {
            return exclusion.badgeDetail
        }
        switch frame.dynamicsAvailability {
        case .waitingForMotionWindow, .groundPlaneUntrusted,
             .inverseDynamicsFailed, .missingRootVerticalDOF,
             .contactSupportUnavailable,
             .analysisPassIncomplete:
            return frame.dynamicsAvailability.detail
        case .withheld, .available:
            break
        }
        if frame.dynamicsAvailability.hasInverseDynamics,
           !frame.hasValidatedDynamicsPayload {
            return NimbleEngine.DynamicsAvailability.inverseDynamicsFailed.detail
        }
        switch frame.motionState {
        case .undetermined:
            return nil

        case .gait(let verdict, _):
            // The product store deliberately strips the native gait outcome.
            // Keep the kinematic verdict and its actionable advice; force and
            // residual values are research diagnostics with no UI route.
            return verdict.advice.isEmpty ? nil : verdict.advice

        case .measured(let verdict, let speed, let window, let floor):
            switch verdict {
            case .hold:
                return String(format: "still: peak %.1f cm/s over %.1f s", speed * 100, window)
            case .noMeasurement, .poseDidNotConverge:
                return verdict.advice
            case .indistinguishableFromNoise:
                // The measured speed alone would read as the subject's fault here.
                // Showing the floor beside it is the whole point: the instrument
                // cannot resolve movement this small on this clip.
                return String(format: "peak %.1f cm/s over %.1f s, but the pose estimate itself "
                              + "jitters %.1f cm/s — %@",
                              speed * 100, window, floor * 100, verdict.advice)
            case .movingBeyondStaticBudget:
                return String(format: "moving: peak %.1f cm/s over %.1f s — %@",
                              speed * 100, window, verdict.advice)
            case .gaitStance, .gaitFlight, .gaitOutsideAnalysis, .gaitRefused:
                // A gait verdict can only arrive through `.gait`; this arm
                // exists so a new verdict cannot be added without deciding what
                // this screen says about it.
                return verdict.advice.isEmpty ? nil : verdict.advice
            }
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
                VStack(alignment: .leading, spacing: 1) {
                    Text(frameLabel).font(.caption).foregroundStyle(.secondary)
                    // Model input/output fingerprints for the on-device vs Mac
                    // comparison. Screenshot-readable because the phone is the
                    // only place the divergence appears and console logs are not
                    // reachable from a TestFlight install.
                    if let c = resultStore.selectedFrame?.modelChecksums {
                        Text(String(format: "src %016llx\nbox %016llx  warp %016llx\nin  %016llx  out %016llx",
                                    c.source, c.bbox, c.warp, c.input, c.output))
                            .font(.system(size: 9, design: .monospaced))
                            .foregroundStyle(.secondary)
                            .textSelection(.enabled)
                    }
                }
                Spacer()
                // Muscle anatomy only exists in the 3-D scene. It is a fixed
                // pose layer, not a dynamics claim, so running/load availability
                // does not enter this control's gate.
                if !showSourceImage
                    && (resultStore.selectedFrame?.hasDrawableAnatomy ?? false) {
                    Button { showMuscles.toggle() } label: {
                        Label("Anatomy", systemImage: "figure.stand")
                            .font(.caption)
                            .foregroundStyle(showMuscles ? Color.green : Color.gray)
                    }
                }
                Button { showSourceImage.toggle() } label: {
                    Label(showSourceImage ? "Photo" : "3D",
                          systemImage: showSourceImage ? "photo" : "rotate.3d")
                        .font(.caption)
                        .foregroundStyle(Color.blue)
                }
            }
            .padding(.horizontal)
        }
        .padding(.vertical, 8)
        .background(.thinMaterial)
    }

    @MainActor
    static func frameLabelText(for resultStore: OfflineResultStore) -> String {
        guard !resultStore.frames.isEmpty else { return "No frames" }
        let poseCount = resultStore.successCount
        let poseNoun = poseCount == 1 ? "pose result" : "pose results"
        var label = "Frame \(resultStore.selectedIndex + 1)/\(resultStore.frames.count)"
            + " — \(poseCount) \(poseNoun)"
        // Zero is deliberately omitted. Leading with "0 with load data" makes
        // a permanent model capability boundary look like a solver that failed
        // on every frame; the named reason below is the relevant result.
        if resultStore.biomechanicsCount > 0 {
            label += ", \(resultStore.biomechanicsCount) with load data"
        }
        // Without this, a clip of a moving subject can look like the solver
        // failed repeatedly rather than correctly withholding a static solve.
        let notStill = resultStore.poseOnlyNotStillCount
        if notStill > 0 { label += ", \(notStill) pose-only (not a still pose)" }
        // Same reasoning one step earlier in the chain: a clip where the person
        // is too small in frame otherwise reads as "0 with muscle data" and
        // looks like a solver failure rather than a framing problem.
        let rejected = resultStore.implausibleBodyCount
        if rejected > 0 { label += ", \(rejected) rejected (body size)" }
        let excluded = resultStore.frames.filter { !$0.isEligibleForTemporalAnalysis }.count
        if excluded > 0 { label += ", \(excluded) pose-only (no person box)" }
        let groundUntrusted = resultStore.groundUntrustedCount
        if groundUntrusted > 0 {
            label += ", \(groundUntrusted) pose-only (ground not established)"
        }
        let contactUnsupported = resultStore.contactSupportUnavailableCount
        if contactUnsupported > 0 {
            label += ", \(contactUnsupported) pose-only (foot contact permanently unsupported)"
        }
        return label
    }

    private var frameLabel: String {
        Self.frameLabelText(for: resultStore)
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

        // Orbit / zoom / reset. A fixed viewpoint is not enough for this data:
        // pelvic tilt, rounded shoulders and knee valgus are all invisible from
        // a single frontal angle, which is exactly what the analysis is for.
        let pan = UIPanGestureRecognizer(target: context.coordinator,
                                         action: #selector(Coordinator.handlePan(_:)))
        pan.maximumNumberOfTouches = 1
        arView.addGestureRecognizer(pan)

        let pinch = UIPinchGestureRecognizer(target: context.coordinator,
                                             action: #selector(Coordinator.handlePinch(_:)))
        arView.addGestureRecognizer(pinch)

        let doubleTap = UITapGestureRecognizer(target: context.coordinator,
                                               action: #selector(Coordinator.handleDoubleTap(_:)))
        doubleTap.numberOfTapsRequired = 2
        arView.addGestureRecognizer(doubleTap)

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

        // One condition decides this, and it lives in `OfflinePlaybackView`
        // beside the legend that has to appear with it — see
        // `anatomyCapsulesAreOnScreen`. A second copy here is how the picture
        // and its caption drift apart.
        if showMuscles {
            context.coordinator.muscleOverlay.update(joints: bodyFrame.joints)
        }
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    final class Coordinator: NSObject {
        var rootAnchor: AnchorEntity?
        var cameraEntity: PerspectiveCamera?
        let muscleOverlay = MuscleOverlay()

        private var jointEntities: [String: ModelEntity] = [:]
        private var boneEntities: [String: ModelEntity] = [:]
        private var hasFramedCamera = false

        // MARK: Orbit camera state
        //
        // The camera is parameterised in spherical coordinates around the
        // subject rather than stored as a position, so drag and pinch stay
        // independent: azimuth/elevation from the pan, radius from the pinch.
        // A stored position would need re-deriving angles from it every gesture
        // and would drift.
        private var orbitCenter: SIMD3<Float> = .zero
        private var orbitRadius: Float = 2.5
        private var orbitAzimuth: Float = 0      // radians, 0 = facing -Z toward subject
        private var orbitElevation: Float = 0    // radians, + = looking down
        private var framedRadius: Float = 2.5    // auto-framed default, for reset
        private var pinchStartRadius: Float = 2.5

        /// Clamped just short of the poles. At exactly +-90 degrees the look-at
        /// up-vector becomes parallel to the view direction and the camera
        /// orientation is undefined, which shows up as a spin at the top of a
        /// drag.
        private static let maxElevation: Float = .pi / 2 - 0.05
        private static let minRadius: Float = 0.4
        private static let maxRadius: Float = 12.0

        /// Positions the camera once, from the first frame's tracked-joint
        /// bounding box, and leaves it fixed for the rest of the scrub session
        /// (like a real camera that stayed put while filming). Recomputing every
        /// frame would jar the view as the subject moves; a hardcoded world
        /// position would risk a blank screen since MHRRetarget documents that
        /// `joint_coords` pins the raw MHR source root at a MODEL-CONSTANT (0, 0.924, 0) in
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
            // Starting angle only — the user can orbit from here. Not derived
            // from any body-facing convention; world-space facing direction is
            // not something this view has a signal for.
            orbitCenter = center
            orbitRadius = distance
            framedRadius = distance
            orbitAzimuth = 0
            orbitElevation = 0
            applyCamera()
            hasFramedCamera = true
        }

        /// Places the camera on the orbit sphere and aims it at the subject.
        func applyCamera() {
            guard let cameraEntity else { return }
            let ce = cos(orbitElevation), se = sin(orbitElevation)
            let ca = cos(orbitAzimuth), sa = sin(orbitAzimuth)
            // Azimuth 0 / elevation 0 puts the camera on +Z looking toward -Z,
            // which reproduces the previous fixed framing exactly.
            let offset = SIMD3<Float>(ce * sa, se, ce * ca) * orbitRadius
            let position = orbitCenter + offset
            cameraEntity.position = position
            cameraEntity.look(at: orbitCenter, from: position, relativeTo: nil)
        }

        // MARK: Gestures

        @objc func handlePan(_ gr: UIPanGestureRecognizer) {
            guard hasFramedCamera else { return }
            let t = gr.translation(in: gr.view)
            gr.setTranslation(.zero, in: gr.view)
            // Full drag across the view is about half a turn horizontally, which
            // makes inspecting the far side a one-gesture action.
            let width = Float(gr.view?.bounds.width ?? 400)
            let height = Float(gr.view?.bounds.height ?? 700)
            orbitAzimuth -= Float(t.x) / width * .pi
            orbitElevation += Float(t.y) / height * .pi
            orbitElevation = min(max(orbitElevation, -Self.maxElevation), Self.maxElevation)
            applyCamera()
        }

        @objc func handlePinch(_ gr: UIPinchGestureRecognizer) {
            guard hasFramedCamera else { return }
            if gr.state == .began { pinchStartRadius = orbitRadius }
            // Divide: pinching OUT (scale > 1) should move the camera closer.
            orbitRadius = min(max(pinchStartRadius / Float(gr.scale), Self.minRadius), Self.maxRadius)
            applyCamera()
        }

        @objc func handleDoubleTap(_ gr: UITapGestureRecognizer) {
            guard hasFramedCamera else { return }
            orbitAzimuth = 0
            orbitElevation = 0
            orbitRadius = framedRadius
            applyCamera()
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
