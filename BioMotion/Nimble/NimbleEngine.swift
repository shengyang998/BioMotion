import Foundation
import Combine
import QuartzCore
import simd
import os

/// Manages the Nimble physics engine lifecycle and provides real-time IK/ID results.
/// Runs Nimble on a background queue to avoid blocking the main thread.
final class NimbleEngine: ObservableObject {
    @Published var isModelLoaded = false
    @Published var lastIKResult: IKOutput?
    @Published var lastIDResult: IDOutput?
    @Published var lastMuscleResult: MuscleOutput?
    @Published var displayMuscleResult: MuscleOutput?
    @Published var ikSolveTimeMs: Double = 0
    @Published var idSolveTimeMs: Double = 0
    @Published var muscleSolveTimeMs: Double = 0

    // --- Accuracy metrics (for UI diagnostics) ---
    /// RMS marker residual from the most recent IK solve, in meters.
    @Published var ikMarkerResidualMeters: Double = 0
    /// Max |joint torque| / total body mass from the most recent ID solve, in Nm/kg.
    /// Physiological range for walking/squat: ~1–3 Nm/kg. Values above 10 indicate
    /// broken pipeline (usually missing GRF or bad ddq).
    @Published var maxTorquePerKg: Double = 0
    /// Total mass of the loaded (possibly scaled) skeleton, in kg.
    @Published var totalMassKg: Double = 0
    /// Left/right foot vertical GRF as fractions of body weight (0-1.x typically).
    /// Sum should be ~1.0 in steady stance, 0 in flight, 1.0-3.0 during impact.
    @Published var leftFootLoadFraction: Double = 0
    @Published var rightFootLoadFraction: Double = 0
    /// Linear-momentum residual after the GRF solve, in NEWTONS per kg:
    /// ‖ΣF_contact + m·g − m·a_com‖ / m. A correct pipeline reports ~0 every
    /// frame — it is a consistency check on the contact-wrench readback, not a
    /// measure of how balanced the pose is. See `NimbleIDResult.rootResidualNorm`.
    @Published var rootResidualPerKg: Double = 0
    /// Current ground-plane height (ARKit world y), for display only.
    @Published var groundHeightY: Double = 0
    /// The most recent complete solve, or nil while the Savitzky-Golay window
    /// is still filling. Prefer this over the individual fields above when you
    /// need IK, ID, muscle and the motion verdict to describe the SAME instant.
    @Published private(set) var lastSolve: SolveRecord?

    /// When true, inverse dynamics and the muscle solve run ONLY on frames the
    /// hold detector marks as static, and they run as a static-equilibrium
    /// problem (q̇ = q̈ = 0). Frames that fail the hold test publish pose and a
    /// `MotionClassification` with no ID/muscle at all.
    ///
    /// Default OFF, and the offline import path turns it on. The reason it is
    /// a switch rather than unconditional is that the two input paths differ in
    /// what they can observe:
    ///
    /// * The offline (photo/video) path goes through `MHRRetarget`, whose
    ///   `joint_coords` source pins the pelvis at the model constant
    ///   (0, 0.924, 0) in EVERY frame because `global_trans` is zeroed
    ///   (`sam3d_body.py:1600`). Joint ANGLES survive that, but the body has no
    ///   global translation, so `M·q̈` and the centre-of-mass acceleration are
    ///   computed from motion that did not happen — in a squat the pelvis never
    ///   descends and the feet appear to rise instead.
    /// * The live ARKit path supplies real world-space joint positions
    ///   including global translation, so its q̈ is observable and gating it
    ///   would remove working behaviour. Untouched by default.
    ///
    /// ⚠️ **The pinning is not a limitation of the pose model — it is a
    /// quantity the app has and does not consume.** The model emits the root
    /// translation separately as `cam_t`; it is exported, stored on
    /// `FrameResult`, and already used to project the overlay.
    /// `MHRRetarget.makeBodyFrame(jointCoords:camT:…)` composes it back in, and
    /// `OfflineSessionRunner` has `estimate.camT` in hand at the call site.
    /// Verified on 309 frames of real video: depth 4.34 m, 1.10 m below the
    /// optical axis, `corr(1/bbox_side, depth) = +0.74`.
    ///
    /// Composing it in is NECESSARY for dynamics and NOT SUFFICIENT. Measured,
    /// same clip: the depth channel carries 12.7 cm (std) of high-frequency
    /// residual about its own 0.5 s mean, which through this engine's own
    /// 9-tap Savitzky-Golay filter is 3.1 g of pure acceleration noise at
    /// 30 fps, 0.23 g at 10 fps. The in-plane channels are 8-25× cleaner. And
    /// `cam_t` is CAMERA-relative, so a rotating camera (13.5 °/s on the user's
    /// `video_012`) makes the frame non-inertial regardless. See STATUS.md,
    /// "cam_t recovers the root translation; its depth cannot be differentiated
    /// twice", and `MotionClassification.rootTranslationObservable`, which
    /// reports whether the stream this engine is being fed actually carries it.
    ///
    /// Read and captured on the MAIN thread inside `processFrame`, which is
    /// already a main-thread API (`isFrameInFlight`/`droppedFrameCount` are
    /// documented as main-only), so there is no cross-queue race.
    var staticHoldGating: Bool = false

    /// Processed IK output with named DOFs.
    struct IKOutput {
        let jointAngles: [String: Double]  // DOF name → angle in radians

        /// TRUE per-marker RMS position error, in METRES. This is the number to
        /// show a user or compare against a distance.
        let markerRMSMeters: Double

        /// The solver's LOSS: `Σ wᵢ²‖p_model,i − p_target,i‖²`, in **m²**.
        /// Kept because loss-domain bounds are compared against it, and named
        /// so it cannot be printed as a length again.
        ///
        /// ⚠️ This field was called `error` and documented as "RMS marker error
        /// in meters" until 2026-08-07. It is neither. On the dancer fixture the
        /// loss is 0.0138, which read as metres says "1.4 cm" while the true
        /// per-marker RMS is 5.5 cm — and the weights are below 1 on exactly the
        /// markers that fit worst, so the misreading always flatters the fit.
        /// `ContentView` printed it as `"%.3f m"` with a green cut at 0.05, i.e.
        /// it showed a *squared* quantity as a length and called anything under
        /// 0.05 good.
        let ikLossSquaredMeters: Double

        let timestamp: TimeInterval
    }

    /// Processed ID output with named DOFs.
    struct IDOutput {
        let jointTorques: [String: Double]  // DOF name → torque in Nm
        let timestamp: TimeInterval

        // Ground-reaction-force diagnostics. Forces in newtons (world frame),
        // CoPs in meters (world frame). Zero when the foot is not in contact.
        var leftFootForce: SIMD3<Double> = .zero
        var rightFootForce: SIMD3<Double> = .zero
        var leftFootCoP: SIMD3<Double> = .zero
        var rightFootCoP: SIMD3<Double> = .zero
        var leftFootInContact: Bool = false
        var rightFootInContact: Bool = false
        /// Norm of the 6D residual at the floating root joint. Should be
        /// small (< ~10 Nm) when GRF and kinematics are consistent.
        var rootResidualNorm: Double = 0
    }

    /// Processed muscle optimization output.
    struct MuscleOutput {
        let activations: [String: Double]  // display name (alias-merged) → activation 0-1
        let forces: [String: Double]       // display name → force in N
        let converged: Bool
        let timestamp: TimeInterval
        /// Activations keyed by RAW solver names (pre alias-merge). Path
        /// rendering needs these because `paths` is keyed by raw names.
        var rawActivations: [String: Double] = [:]
        /// Anatomy-exact world-space endpoints for every muscle, captured
        /// from the skeleton FK at the same pose the solver consumed. Used
        /// by the overlay to draw path-based capsules for muscles not in
        /// the hardcoded-def set (i.e. upper body, trunk, etc.).
        var paths: [String: MusclePath] = [:]
        /// Peak isometric force (N) per muscle (raw names), for radius scaling.
        var maxForces: [String: Double] = [:]
    }

    struct MusclePath {
        let start: SIMD3<Float>
        let end: SIMD3<Float>
    }

    /// Why a frame does or does not carry muscle magnitudes.
    ///
    /// The point of splitting this out of a boolean: "no muscle numbers" had
    /// exactly one user-facing explanation — *"muscle loads need a still pose"*
    /// — and the user can do nothing with it except freeze. These cases have
    /// different causes and different remedies, and two of them are not the
    /// user's fault at all.
    enum MotionVerdict: Equatable {
        /// Measured still to within the budget. ID and the muscle QP run as a
        /// static-equilibrium problem (q̇ = q̈ = 0).
        case hold
        /// Measurably moving, by more than the budget static equilibrium can
        /// absorb AND by more than the instrument's own noise. Withhold: the
        /// static reading would be a different answer, not a cleaner one.
        ///
        /// This build has no dynamic branch to fall through to, so a moving
        /// frame is withheld whether or not `rootTranslationObservable` is
        /// true. Read that field alongside this case — it says which of the two
        /// things is missing.
        case movingBeyondStaticBudget
        /// The measured marker speed exceeds the stillness bound, but so does
        /// this clip's own pose-estimation noise floor, so the two cannot be
        /// told apart. NOT the same as "the subject moved": at a sparse
        /// sampling rate a perfectly still subject lands here.
        case indistinguishableFromNoise
        /// The subject is moving TOWARD OR AWAY from the camera faster than the
        /// budget allows. This path holds the root's depth at one per-clip value
        /// because monocular depth cannot be differentiated (see
        /// `RootDepthHold`), which is exact for motion in the image plane and
        /// wrong for motion along the optical axis. This is that assumption
        /// failing its check, and it is the one failure the user can fix by
        /// standing side-on to the camera.
        case depthMotionNotResolvable
        /// Nothing measurable in the window (first sample of a clip, no marker
        /// in common with the predecessor, non-increasing timestamps).
        case noMeasurement

        /// One sentence the user can act on. Lives here rather than in the view
        /// so it is covered by unit tests and cannot drift from the numbers.
        var advice: String {
            switch self {
            case .hold:
                return ""
            case .movingBeyondStaticBudget:
                return "The subject moved too fast for a still-pose reading. Hold the position, or sample the clip at a higher rate so a shorter stretch of stillness is enough."
            case .indistinguishableFromNoise:
                return "The pose estimate jitters as much as the movement being measured, so stillness cannot be confirmed. Fill more of the frame, improve the lighting, or sample at a higher rate."
            case .depthMotionNotResolvable:
                return "The subject is moving toward or away from the camera. A single camera cannot measure that direction accurately — stand side-on to the camera, or keep your distance from it constant."
            case .noMeasurement:
                return "Not enough frames around this instant to measure motion."
            }
        }
    }

    /// What the hold detector concluded about one instant.
    ///
    /// `timestamp` is the Savitzky-Golay window CENTRE — the same instant
    /// `IDOutput`/`MuscleOutput` are dated at — not the newest pushed frame.
    struct MotionClassification: Equatable {
        let timestamp: TimeInterval
        /// True iff every measured marker speed in the examined window stayed
        /// under `StaticHoldDetector.holdSpeedThresholdMetersPerSecond` AND the
        /// window was long enough to bound the discarded acceleration.
        /// Exactly `verdict == .hold`.
        let isHold: Bool
        /// Largest per-marker speed seen anywhere in the window, m/s.
        let peakMarkerSpeedMetersPerSecond: Double
        /// Median over the window of the per-sample median-over-markers speed,
        /// m/s. Diagnostic only: if `peak` is high but this is low, one marker
        /// moved (or one marker is noisy), not the body.
        let medianMarkerSpeedMetersPerSecond: Double
        /// Span of the examined window, seconds.
        let windowSeconds: Double
        /// Number of samples examined (including the ones with no predecessor).
        let sampleCount: Int
        /// `2 · peak / windowSeconds` — the bound this window puts on the mean
        /// acceleration that static-equilibrium ID throws away. m/s².
        let impliedMeanAccelMetersPerSecondSquared: Double
        /// The reason behind `isHold`, and the sentence to show for it.
        let verdict: MotionVerdict
        /// A LOWER BOUND on how much of `peakMarkerSpeedMetersPerSecond` is
        /// pose-estimation noise rather than the subject moving, in m/s,
        /// measured on THIS clip. `max` of the two floors below, because a
        /// marker inherits both.
        let poseNoiseFloorMetersPerSecond: Double
        /// The ARTICULATION half of that floor: measured from distances that
        /// physically cannot change (`StaticHoldDetector.rigidPairs`). It is
        /// invariant to root translation by construction — every rigid pair is
        /// unchanged when the same offset is added to both endpoints — so it is
        /// structurally blind to `cam_t` jitter, which is why the root has its
        /// own floor below.
        let articulationNoiseFloorMetersPerSecond: Double
        /// The ROOT half: measured from the root marker's own 4th difference,
        /// which annihilates anything the cubic Savitzky-Golay filter can
        /// represent, so what it leaves is what the filter will turn into
        /// acceleration. Zero on a pelvis-pinned stream, where the root does not
        /// move at all.
        let rootNoiseFloorMetersPerSecond: Double
        /// Least-squares slope of the root's RAW depth over the examined window,
        /// m/s — the quantity `RootDepthHold` is assuming to be zero. Positive
        /// means moving away from the camera. NaN when the root is not
        /// observable (a pinned stream carries no depth to trend).
        let depthDriftMetersPerSecond: Double
        /// False when the pose source pins the pelvis, i.e. the body carries no
        /// global translation and the root's contribution to `M·q̈` is missing.
        /// See `MHRRetarget.rootTranslation(camT:)`.
        let rootTranslationObservable: Bool
    }

    /// Everything one warm solve produced, all dated at the SAME Savitzky-Golay
    /// centre timestamp.
    ///
    /// The individual `@Published` fields below are updated independently, so a
    /// consumer that reads `lastMuscleResult` and `lastIKResult` in the same
    /// turn can pick up a muscle result from an OLDER solve alongside a fresh
    /// IK result — which matters now that a moving frame publishes IK with no
    /// muscle at all. This bundle is the consistent view: everything in it
    /// belongs to `centerTimestamp` or is nil.
    struct SolveRecord {
        let centerTimestamp: TimeInterval
        let motion: MotionClassification
        let ik: IKOutput
        let id: IDOutput?
        let muscle: MuscleOutput?
        /// True iff `id` and `muscle` were solved with q̇ = q̈ = 0 because the
        /// subject was measured to be holding still. False means they came from
        /// the Savitzky-Golay derivatives (live-camera path).
        let isStaticHoldEstimate: Bool
    }

    // Normalize model-specific muscle ids to the stable ids used by the
    // overlay and diagnostic bar.
    private static let displayMuscleAliases: [String: String] = [
        "bflh140_r": "bflh_r",
        "bflh140_l": "bflh_l",
        "gaslat140_r": "gaslat_r",
        "gaslat140_l": "gaslat_l",
        "vaslat140_r": "vaslat_r",
        "vaslat140_l": "vaslat_l",
        "multifidus_T9_T7": "ercspn_r",
        "multifidus_T9_T7_L": "ercspn_l",
    ]

    private let bridge = NimbleBridge()
    private let muscleSolver = MuscleSolver()
    private let momentArmComputer = MomentArmComputer()
    private let solverQueue = DispatchQueue(label: "com.biomotion.nimble", qos: .userInteractive)

    // Per-DOF Savitzky–Golay filters for smoothed q / dq / ddq.
    // Warms up after 9 frames (~150 ms @ 60 fps); once warm, outputs lag
    // the raw input by 4 frames (~66 ms @ 60 fps) in exchange for much
    // cleaner numerical derivatives than naive finite differences.
    private var dofFilters: [SavitzkyGolayFilter] = []

    // Marker-motion history behind `staticHoldGating`. Pushed in lockstep with
    // `dofFilters` (both only on a frame whose IK succeeded), so "the last 9
    // samples" means the same nine frames in both. solverQueue-only state.
    private var holdDetector = StaticHoldDetector()

    // Holds the root's depth at one per-clip value on the gated (offline) path.
    // A bit-exact no-op until `cam_t` is composed in — see `RootDepthHold`.
    // solverQueue-only state, cleared with the filters.
    private var depthHold = RootDepthHold()

    // Timestamp of the last successful muscle solve, used to derive dt for
    // musculotendon length finite differencing inside the muscle Hill model.
    private var lastMuscleSolveTimestamp: TimeInterval?
    private var lastDisplayMuscleTimestamp: TimeInterval?
    private let displayMuscleHoldDuration: TimeInterval = 0.35

    // Per-display-muscle 1€ filter. The rebalanced solver (ε_a=0.01,
    // λ=100) now tracks τ tightly, which also tracks ID torque noise —
    // without smoothing, activations jitter visibly frame-to-frame.
    // Aggressive minCutoff=2.0 Hz (vs 1.0 Hz for kinematics) because
    // activation transitions during actual movement are ~100-300 ms and
    // we don't want to soften real force onset.
    private var activationFilters: [String: OneEuroFilter] = [:]

    // IK history for recording
    /// `markerRMSMeters` is the TRUE per-marker RMS in metres, not the solver
    /// loss. See `IKOutput.ikLossSquaredMeters` for why the distinction matters.
    private(set) var ikHistory: [(timestamp: TimeInterval, angles: [String: Double], markerRMSMeters: Double)] = []
    private(set) var idHistory: [(timestamp: TimeInterval, jointTorques: [String: Double])] = []
    private var isRecordingResults = false

    // Generation token bumped on reset. Frames captured before the bump are
    // still in flight on solverQueue; their late publishes are discarded by
    // comparing against the current generation on main.
    private var currentGeneration: UInt64 = 0
    private var genLock = os_unfair_lock_s()

    private func readGeneration() -> UInt64 {
        os_unfair_lock_lock(&genLock)
        defer { os_unfair_lock_unlock(&genLock) }
        return currentGeneration
    }

    private func bumpGeneration() -> UInt64 {
        os_unfair_lock_lock(&genLock)
        defer { os_unfair_lock_unlock(&genLock) }
        currentGeneration &+= 1
        return currentGeneration
    }

    // Backpressure: at most one frame in flight on solverQueue at a time.
    // Accessed only from main.
    private var isFrameInFlight = false
    private(set) var droppedFrameCount: Int = 0

    // NO RUNTIME DOF MASK IS INSTALLED HERE, AND THAT IS A MEASURED DECISION.
    //
    // STATUS.md next-step 8 asked for `shoulder_rot_{r,l}` (axial humeral
    // rotation) to be masked, on the premise that they are "structurally
    // unobservable from one marker per shoulder plus one at the elbow" and that
    // unobservable coordinates get excited by the solver. Measured 2026-08-07,
    // the premise is false and the change is a regression:
    //
    //   * The marker-Jacobian column for `shoulder_rot_r` has norm
    //     0.0343 m/rad at the model's neutral pose — small next to
    //     `shoulder_elv_r` (0.6077) but not null. It moves REJC 16.3 mm and
    //     RWJC 30.2 mm per radian with the elbow STRAIGHT, because the ulna and
    //     hand body origins are offset from the humeral long axis, and 0.266
    //     m/rad at 90° of elbow flexion. It is not one of the 72 identically-
    //     zero columns in `FullBodyDOFFixture.structurallyUnreachableCoordinates`.
    //   * Masking it costs the dancer fixture 0.565 cm of marker RMS
    //     (2.122 -> 2.687) and 0.045 of relative torque residual
    //     (0.3545 -> 0.3991).
    //   * At upright standing it buys nothing (the unmasked solver puts 0.04°
    //     into the coordinate) and it breaks convergence: 0 -> 123 iterations,
    //     converged YES -> NO, per-solve drift 0 -> 9.3e-5 rad.
    //
    // Harness and every number: `ShoulderRotMaskTests` /
    // `ShoulderRotObservabilityTests.mm`. If a future marker set drops the
    // wrist markers, or a future model puts the ulna origin on the humeral
    // axis, re-measure before reviving this — the column norm is the number
    // that decides it.

    /// Load the bundled .osim model.
    func loadBundledModel() {
        // Production full-body model: cyclistFullBodyMuscle.osim shipped
        // as FullBody.osim in the app bundle. 80 bodies, 520 muscles
        // (Millard2012 lower + Thelen2003 upper/trunk/spine).
        //
        // The libnimble_ios.a in this build includes a patched
        // OpenSimParser that no longer crashes on CustomJoints it can't
        // construct — it logs them and substitutes a WeldJoint. So any
        // joint in cyclist that nimble doesn't fully support becomes
        // locked rather than segfaulting the whole skeleton.
        //
        // Fallback: if FullBody.osim is missing from the bundle, load
        // the old lower-extremity-only Rajagopal2016 — useful for
        // developers who want to diff behavior without re-ripping assets.
        let path: String
        if let fb = Bundle.main.path(forResource: "FullBody", ofType: "osim") {
            path = fb
            print("NimbleEngine: loading FullBody.osim (cyclistFullBodyMuscle — 520 muscles, nimble OpenSimParser patched)")
        } else if let raj = Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim") {
            path = raj
            print("NimbleEngine: ⚠ FullBody.osim not found — falling back to Rajagopal2016 (81 lower-extremity muscles only)")
        } else {
            print("NimbleEngine: no .osim model found in bundle")
            return
        }
        solverQueue.async { [weak self] in
            guard let self else { return }
            let success = self.bridge.loadModel(fromPath: path)
            if success {
                self.muscleSolver.loadMuscles(fromOsimPath: path)
                // MomentArmComputer adopts the bridge's skeleton instead of
                // parsing a second copy — so per-segment scaling propagates
                // from bridge.scaleModelWithHeight through to R(q) and L_MT.
                self.momentArmComputer.parseMusclePaths(fromOsimPath: path,
                                                         from: self.bridge)
                // Drop any stale SG state from a previous model — the new
                // model may have a different DOF count / ordering, and even
                // if not, the sample history is no longer valid.
                self.dofFilters.removeAll(keepingCapacity: false)
            }
            DispatchQueue.main.async {
                self.isModelLoaded = success
                if success {
                    let modelMarkers = self.bridge.markerNames as [String]
                    self.totalMassKg = self.bridge.totalMass
                    print("NimbleEngine: Model loaded — \(self.bridge.numDOFs) DOFs, \(modelMarkers.count) markers, \(self.muscleSolver.numMuscles) muscles, mass \(String(format: "%.1f", self.totalMassKg)) kg")
                    print("NimbleEngine: Model marker names (first 10): \(modelMarkers.prefix(10))")
                    print("NimbleEngine: Our mapping names (first 10): \(JointMapping.primary.map(\.opensimName).prefix(10))")
                    // Check how many of our mappings match model markers
                    let matchCount = JointMapping.primary.filter { modelMarkers.contains($0.opensimName) }.count
                    print("NimbleEngine: Marker matches: \(matchCount)/\(JointMapping.primary.count)")
                }
            }
        }
    }

    /// Scale the model for a specific user.
    func scaleModel(height: Double, markerPositions: [Float], markerNames: [String]) {
        guard isModelLoaded else { return }
        let positions = markerPositions.map { NSNumber(value: Double($0)) }
        solverQueue.async { [weak self] in
            self?.bridge.scaleModel(withHeight: height,
                                    markerPositions: positions,
                                    markerNames: markerNames)
        }
    }

    /// Process a body frame: run IK (and optionally ID) on a background thread.
    func processFrame(_ frame: BodyFrame) {
        guard isModelLoaded else { return }

        // Backpressure: drop this frame if the solver is still busy with the
        // previous one. Keeps visualization on the newest pose rather than
        // draining a stale FIFO when OSQP transiently stalls.
        if isFrameInFlight {
            droppedFrameCount &+= 1
            return
        }

        // Build marker arrays from the frame
        var positions: [NSNumber] = []
        var names: [String] = []
        var rawRootDepth: Double?

        for joint in frame.joints where joint.isTracked {
            // Map ARKit joint to OpenSim marker name
            if let mapping = JointMapping.primary.first(where: { $0.arkitName == joint.id }) {
                names.append(mapping.opensimName)
                positions.append(NSNumber(value: Double(joint.worldPosition.x)))
                positions.append(NSNumber(value: Double(joint.worldPosition.y)))
                positions.append(NSNumber(value: Double(joint.worldPosition.z)))
                if mapping.opensimName == StaticHoldDetector.rootMarkerName {
                    rawRootDepth = Double(joint.worldPosition.z)
                }
            }
        }

        guard !names.isEmpty else { return }

        isFrameInFlight = true
        let frameGeneration = readGeneration()
        // Captured on main (see `staticHoldGating`) so the whole solve uses one
        // consistent policy even if the flag flips mid-clip.
        let gateOnHolds = staticHoldGating

        solverQueue.async { [weak self] in
            guard let self else { return }
            defer {
                DispatchQueue.main.async { [weak self] in
                    self?.isFrameInFlight = false
                }
            }

            // --- Hold the root's depth (gated path only) ---
            // Monocular depth is the one channel of the recovered root
            // translation that cannot be differentiated: its error saturates at
            // 12-15 cm by τ ≈ 0.15 s and no rolling window separates it from
            // motion (measurements in `RootDepthHold`). So it is held at one
            // per-clip value and the assumption behind that — "the subject's
            // distance from the camera does not change" — is CHECKED below via
            // `depthDriftMetersPerSecond`, not merely declared.
            //
            // The shift is common-mode, so relative geometry is untouched and,
            // under uniform gravity, no joint torque changes. On a pelvis-pinned
            // stream the root's z is the model constant every frame, so this is
            // a bit-exact no-op until `cam_t` is composed in.
            var depthDrift = Double.nan
            if gateOnHolds, let rootZ = rawRootDepth {
                self.depthHold.ingest(rootDepth: rootZ, timestamp: frame.timestamp)
                let offset = self.depthHold.offset(forRootDepth: rootZ)
                if offset != 0 {
                    for i in stride(from: 2, to: positions.count, by: 3) {
                        positions[i] = NSNumber(value: positions[i].doubleValue - offset)
                    }
                }
                depthDrift = self.depthHold.driftMetersPerSecond(
                    overLast: SavitzkyGolayFilter.windowSize)
            }

            // --- IK (runs on every frame, on 1€-filtered markers) ---
            let ikStart = CACurrentMediaTime()
            guard let ikResult = self.bridge.solveIK(
                withMarkerPositions: positions,
                markerNames: names
            ) else { return }
            let ikTime = (CACurrentMediaTime() - ikStart) * 1000.0

            // Record marker motion for the hold detector. Deliberately fed the
            // SAME arrays IK consumed — post-`isTracked` filter, post-
            // `JointMapping.primary` lookup — so the stillness test is measured
            // on exactly the observations that constrained the solve, and a
            // marker dropping out cannot be mistaken for a marker moving.
            //
            // This is raw marker motion ON PURPOSE. The obvious alternative,
            // thresholding the Savitzky-Golay `ddq` below, is circular: `ddq` is
            // a twice-differentiated function of this very data (gain ~1/dt²,
            // ≈3600 at 60 fps), and on the offline path the data it is
            // differentiating is missing its global translation component
            // entirely. A small `ddq` there would mean "the unobservable part
            // of the motion stayed unobservable", not "the subject was still".
            self.holdDetector.ingest(flatMarkerPositions: positions,
                                     markerNames: names,
                                     timestamp: frame.timestamp)

            let numDOFs = ikResult.jointAngles.count
            let dofNames = ikResult.dofNames

            // Lazy-init per-DOF SG filters whenever the DOF count changes.
            if self.dofFilters.count != numDOFs {
                self.dofFilters = (0..<numDOFs).map { _ in SavitzkyGolayFilter() }
            }

            // --- Push raw IK angles into SG filters, pull smoothed q/dq/ddq ---
            // Each filter only emits output once its 9-sample window is full,
            // so the first 8 frames after start (or after a DOF-count change)
            // yield "raw" IK only, without ID or muscle results.
            var smoothedQ = [Double](); smoothedQ.reserveCapacity(numDOFs)
            var smoothedDQ = [Double](); smoothedDQ.reserveCapacity(numDOFs)
            var smoothedDDQ = [Double](); smoothedDDQ.reserveCapacity(numDOFs)
            var centerTimestamp = frame.timestamp
            var sgWarmedUp = true

            // EVERY filter must be pushed on EVERY frame. Bailing out of this
            // loop on the first not-yet-warm filter starves all the later ones:
            // filter[k] would only start receiving samples once filters 0..<k
            // were already full, so warm-up would need 9 + 8*(numDOFs-1) frames
            // — about 1350 for this model — instead of 9.
            //
            // The live ARKit path hid that: at 60 fps it grinds through in ~22 s
            // of tracking. The offline path pushes exactly 9 frames per clip and
            // so never warmed up at all, which is why an imported photo reported
            // "Pose only (warming up)" and 0 frames with muscle data forever.
            for i in 0..<numDOFs {
                let q = ikResult.jointAngles[i].doubleValue
                if let out = self.dofFilters[i].push(q, timestamp: frame.timestamp) {
                    smoothedQ.append(out.pos)
                    smoothedDQ.append(out.vel)
                    smoothedDDQ.append(out.acc)
                    centerTimestamp = out.center
                } else {
                    sgWarmedUp = false
                }
            }
            // A partially-filled window would leave the smoothed arrays shorter
            // than numDOFs; every consumer below is behind the `sgWarmedUp`
            // guard, so they only ever see complete ones.
            if smoothedQ.count != numDOFs { sgWarmedUp = false }

            // Build the "raw" IK output (used for live UI skeleton overlay).
            var liveAngles: [String: Double] = [:]
            for i in 0..<min(numDOFs, dofNames.count) {
                liveAngles[dofNames[i]] = ikResult.jointAngles[i].doubleValue
            }
            let liveIkOutput = IKOutput(
                jointAngles: liveAngles,
                markerRMSMeters: ikResult.markerRMSMeters,
                ikLossSquaredMeters: ikResult.error,
                timestamp: frame.timestamp
            )

            guard sgWarmedUp else {
                // Still warming up: publish live IK only, no ID/muscle yet.
                // No motion verdict either — there is no window centre to date
                // one at, so `lastSolve` stays nil rather than carrying a guess.
                self.publishResults(ik: liveIkOutput, id: nil, muscle: nil,
                                    motion: nil, isStaticHoldEstimate: false,
                                    ikTime: ikTime, idTime: 0, muscleTime: 0,
                                    ikResidual: ikResult.markerRMSMeters, maxTorqueNm: 0,
                                    groundY: self.bridge.groundHeightY,
                                    generation: frameGeneration)
                return
            }

            // Smoothed IK output, dated at the center of the SG window.
            // This is what ID and muscle solves operate on — it matches dq/ddq
            // temporally, which is essential for correct inverse dynamics.
            var smoothedAngles: [String: Double] = [:]
            for i in 0..<min(numDOFs, dofNames.count) {
                smoothedAngles[dofNames[i]] = smoothedQ[i]
            }
            let smoothedIkOutput = IKOutput(
                jointAngles: smoothedAngles,
                markerRMSMeters: ikResult.markerRMSMeters,
                ikLossSquaredMeters: ikResult.error,
                timestamp: centerTimestamp
            )

            // --- Was the subject still at the window centre? ---
            // Classified at `centerTimestamp`, not at the newest push: that is
            // the instant ID and the muscle solve are dated at, and the centred
            // window means we already hold 4 samples of "future" around it.
            let motion = self.holdDetector.classify(centeredAt: centerTimestamp,
                                                    depthDriftMetersPerSecond: depthDrift)

            // An IK solve that exited on the iteration cap has NOT reached a
            // fixed point, and the next solve on identical markers will still be
            // moving. Adversarial testing found a legitimate in-limits pose where
            // that residual motion is 2.0e-2 rad — twenty times the drift bound
            // the old known-red test asserted. The Savitzky-Golay stage
            // differentiates twice (gain ≈ 1/dt² ≈ 3600 at 60 fps), so it becomes
            // ~73 rad/s² of acceleration the subject never had, and every torque
            // downstream inherits it.
            //
            // `converged` was already reported and nothing read it. Withhold the
            // dynamics exactly as a moving frame does: the pose is still good
            // enough to draw and to export, the derivatives are not.
            guard ikResult.converged else {
                if self.isRecordingResults {
                    self.ikHistory.append((centerTimestamp, smoothedAngles, ikResult.markerRMSMeters))
                }
                self.publishResults(ik: smoothedIkOutput, id: nil, muscle: nil,
                                    motion: motion, isStaticHoldEstimate: false,
                                    ikTime: ikTime, idTime: 0, muscleTime: 0,
                                    ikResidual: ikResult.markerRMSMeters, maxTorqueNm: 0,
                                    groundY: self.bridge.groundHeightY,
                                    generation: frameGeneration)
                return
            }

            guard !gateOnHolds || motion.isHold else {
                // Measurably moving. Publish the pose and the verdict; publish
                // NO muscle magnitudes. The alternative — reporting them anyway
                // — would be reporting `M·q̈` computed from a translation this
                // input cannot see.
                //
                // The POSE is still recorded. Only the dynamics are withheld,
                // so a .mot export of a moving clip stays complete; it is the
                // matching .sto that legitimately has a gap.
                if self.isRecordingResults {
                    self.ikHistory.append((centerTimestamp, smoothedAngles, ikResult.markerRMSMeters))
                }
                self.publishResults(ik: smoothedIkOutput, id: nil, muscle: nil,
                                    motion: motion, isStaticHoldEstimate: false,
                                    ikTime: ikTime, idTime: 0, muscleTime: 0,
                                    ikResidual: ikResult.markerRMSMeters, maxTorqueNm: 0,
                                    groundY: self.bridge.groundHeightY,
                                    generation: frameGeneration)
                return
            }

            // On a detected hold, solve statics: q̇ = q̈ = 0 exactly. That is
            // the whole point of the detector — it deletes the 1/dt²
            // Savitzky-Golay amplification chain from τ = M·q̈ + C − JᵀF_ext
            // instead of feeding it derivatives of a motion we half-observed.
            // It also zeroes the Hill force-velocity term downstream
            // (`dL_MT/dt = −Rᵀ·q̇`), which is what "isometric" means.
            let solveAsStatics = gateOnHolds && motion.isHold
            let idDQ = solveAsStatics ? [Double](repeating: 0, count: numDOFs) : smoothedDQ
            let idDDQ = solveAsStatics ? [Double](repeating: 0, count: numDOFs) : smoothedDDQ

            // --- ID on SG-smoothed q, and dq/ddq per the policy above ---
            let smoothedQNS: [NSNumber] = smoothedQ.map { NSNumber(value: $0) }
            let smoothedDQNS: [NSNumber] = idDQ.map { NSNumber(value: $0) }
            let smoothedDDQNS: [NSNumber] = idDDQ.map { NSNumber(value: $0) }

            var idOutput: IDOutput?
            var idTime = 0.0
            var maxTorqueNm = 0.0

            let idStart = CACurrentMediaTime()
            // Use the GRF-aware ID solver. It runs Nimble's near-CoP
            // multi-contact inverse dynamics which auto-detects foot contact
            // and decomposes the system wrench into GRFs + joint torques.
            if let idResult = self.bridge.solveIDGRF(
                withJointAngles: smoothedQNS,
                jointVelocities: smoothedDQNS,
                jointAccelerations: smoothedDDQNS
            ) {
                idTime = (CACurrentMediaTime() - idStart) * 1000.0

                var torques: [String: Double] = [:]
                for i in 0..<min(idResult.jointTorques.count, dofNames.count) {
                    let t = idResult.jointTorques[i].doubleValue
                    torques[dofNames[i]] = t
                    if abs(t) > maxTorqueNm { maxTorqueNm = abs(t) }
                }

                var out = IDOutput(jointTorques: torques, timestamp: centerTimestamp)
                func toSimd(_ arr: [NSNumber]) -> SIMD3<Double> {
                    guard arr.count >= 3 else { return .zero }
                    return SIMD3<Double>(arr[0].doubleValue, arr[1].doubleValue, arr[2].doubleValue)
                }
                out.leftFootForce  = toSimd(idResult.leftFootForce)
                out.rightFootForce = toSimd(idResult.rightFootForce)
                out.leftFootCoP    = toSimd(idResult.leftFootCoP)
                out.rightFootCoP   = toSimd(idResult.rightFootCoP)
                out.leftFootInContact  = idResult.leftFootInContact
                out.rightFootInContact = idResult.rightFootInContact
                out.rootResidualNorm   = idResult.rootResidualNorm
                idOutput = out
            }

            // --- Muscle static optimization (on same SG-centered state) ---
            // Production path: real moment arms from FK + soft-equality QP +
            // real Hill-model force-length/velocity. Requires that the
            // skeleton is at the smoothed pose, which solveIDGRF() already
            // sets when it runs — MomentArmComputer picks up from there.
            var muscleOutput: MuscleOutput?
            var muscleTime = 0.0

            if let id = idOutput {
                // IMPORTANT: jointTorques arrives as a dictionary, which has
                // no defined order. We must force a consistent ordering here
                // so that `dofNames[i]` refers to the same DOF as `torques[i]`
                // AND as the j-th column of the moment-arm matrix. Use the
                // DOF names from IK (they are in skeleton order).
                let torqueKeys: [String] = ikResult.dofNames as [String]
                let torqueVals = torqueKeys.map { NSNumber(value: id.jointTorques[$0] ?? 0) }

                // Compute the moment-arm matrix R(q) at the smoothed pose.
                // MomentArmComputer runs its own FK so it expects the pose
                // to be driven through setPositions. Pass the SG-smoothed q.
                let momentArms = self.momentArmComputer.computeMomentArms(
                    withJointAngles: smoothedQNS,
                    dofNames: torqueKeys
                ) ?? []
                let muscleLengthsNS = self.momentArmComputer.currentMuscleLengths
                let muscleNamesNS = self.momentArmComputer.muscleNames
                let maxForcesNS = self.momentArmComputer.maxIsometricForces
                let optimalFibersNS = self.momentArmComputer.optimalFiberLengths
                let tendonSlacksNS = self.momentArmComputer.tendonSlackLengths
                let pennationsNS = self.momentArmComputer.pennationAngles

                if !momentArms.isEmpty && muscleNamesNS.count > 0 {
                    // dt for fiber-velocity finite differencing. We use the
                    // SG-window centered timestamp vs. the previous frame's.
                    let nowSec = centerTimestamp
                    let dt = max(nowSec - (self.lastMuscleSolveTimestamp ?? nowSec - 0.0167),
                                 1e-3)
                    self.lastMuscleSolveTimestamp = nowSec

                    if let result = self.muscleSolver.solveReal(
                        withJointTorques: torqueVals,
                        momentArms: momentArms,
                        muscleNames: muscleNamesNS,
                        muscleLengths: muscleLengthsNS,
                        maxForces: maxForcesNS,
                        optimalFiberLengths: optimalFibersNS,
                        tendonSlackLengths: tendonSlacksNS,
                        pennationAngles: pennationsNS,
                        jointVelocities: smoothedDQNS,
                        dofNames: torqueKeys,
                        dt: dt,
                        // 100× the activation L2 regularizer (ε=0.01 inside
                        // solver). τ-match dominates; ‖a‖² only breaks ties
                        // among the redundant ~520-muscle set.
                        softPenalty: 100.0
                    ) {
                        muscleTime = result.solveTimeMs

                        // Harvest path endpoints from the skeleton FK at the
                        // SAME pose the solver consumed (computeMomentArms
                        // left the skeleton at smoothedQ before restoring).
                        // Doing this in the same solverQueue block avoids
                        // races against next-frame setPositions.
                        let endpoints = self.momentArmComputer.muscleEndpointsWorld
                        let pathCount = muscleNamesNS.count
                        var paths: [String: MusclePath] = [:]
                        paths.reserveCapacity(pathCount)
                        if endpoints.count == pathCount * 6 {
                            for i in 0..<pathCount {
                                let base = i * 6
                                let s = SIMD3<Float>(
                                    Float(truncating: endpoints[base]),
                                    Float(truncating: endpoints[base + 1]),
                                    Float(truncating: endpoints[base + 2])
                                )
                                let e = SIMD3<Float>(
                                    Float(truncating: endpoints[base + 3]),
                                    Float(truncating: endpoints[base + 4]),
                                    Float(truncating: endpoints[base + 5])
                                )
                                paths[muscleNamesNS[i]] = MusclePath(start: s, end: e)
                            }
                        }

                        var activations: [String: Double] = [:]
                        var forces: [String: Double] = [:]
                        var maxForceMap: [String: Double] = [:]
                        activations.reserveCapacity(result.muscleNames.count)
                        forces.reserveCapacity(result.muscleNames.count)
                        maxForceMap.reserveCapacity(result.muscleNames.count)
                        for i in 0..<result.muscleNames.count {
                            let name = result.muscleNames[i]
                            activations[name] = result.activations[i].doubleValue
                            forces[name] = result.forces[i].doubleValue
                            if i < maxForcesNS.count {
                                maxForceMap[name] = maxForcesNS[i].doubleValue
                            }
                        }
                        var out = MuscleOutput(
                            activations: activations,
                            forces: forces,
                            converged: result.converged,
                            timestamp: centerTimestamp
                        )
                        out.rawActivations = activations
                        out.paths = paths
                        out.maxForces = maxForceMap
                        muscleOutput = out
                    }
                }
            }

            // Record if enabled (history always uses the SG-centered timestamp
            // so downstream .mot/.sto exports are temporally consistent).
            if self.isRecordingResults {
                self.ikHistory.append((centerTimestamp, smoothedAngles, ikResult.markerRMSMeters))
                if let id = idOutput {
                    self.idHistory.append((centerTimestamp, id.jointTorques))
                }
            }

            self.publishResults(ik: smoothedIkOutput, id: idOutput, muscle: muscleOutput,
                                motion: motion, isStaticHoldEstimate: solveAsStatics,
                                ikTime: ikTime, idTime: idTime, muscleTime: muscleTime,
                                ikResidual: ikResult.markerRMSMeters, maxTorqueNm: maxTorqueNm,
                                groundY: self.bridge.groundHeightY,
                                generation: frameGeneration)
        }
    }

    private func publishResults(ik: IKOutput, id: IDOutput?, muscle: MuscleOutput?,
                                motion: MotionClassification?,
                                isStaticHoldEstimate: Bool,
                                ikTime: Double, idTime: Double, muscleTime: Double,
                                ikResidual: Double, maxTorqueNm: Double,
                                groundY: Double,
                                generation: UInt64) {
        let mass = max(totalMassKg, 1e-6)
        let torquePerKg = maxTorqueNm / mass
        // Vertical load on each foot as a fraction of body weight. Useful for
        // validating stance vs. swing phase and impact peaks during gait.
        let weightN = mass * 9.81
        let leftLoad  = (id?.leftFootForce.y  ?? 0) / weightN
        let rightLoad = (id?.rightFootForce.y ?? 0) / weightN
        let rootResPerKg = (id?.rootResidualNorm ?? 0) / mass
        let displayMuscle = muscle.map(normalizeForDisplay)
        let solve = motion.map {
            SolveRecord(centerTimestamp: $0.timestamp, motion: $0, ik: ik, id: id,
                        muscle: muscle, isStaticHoldEstimate: isStaticHoldEstimate)
        }
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            // Drop late publishes from a pre-reset generation so they don't
            // overwrite the cleared @Published state.
            guard self.readGeneration() == generation else { return }
            self.lastIKResult = ik
            self.lastIDResult = id
            self.lastMuscleResult = muscle
            // Only overwritten by a warm solve. A warm-up publish leaves the
            // previous record in place rather than blanking it, matching how
            // `lastMuscleResult` behaves.
            if let solve { self.lastSolve = solve }
            if let displayMuscle {
                self.displayMuscleResult = displayMuscle
                self.lastDisplayMuscleTimestamp = displayMuscle.timestamp
            } else if let lastTimestamp = self.lastDisplayMuscleTimestamp,
                      (ik.timestamp - lastTimestamp) > self.displayMuscleHoldDuration {
                self.displayMuscleResult = nil
                self.lastDisplayMuscleTimestamp = nil
            }
            self.ikSolveTimeMs = ikTime
            self.idSolveTimeMs = idTime
            self.muscleSolveTimeMs = muscleTime
            self.ikMarkerResidualMeters = ikResidual
            self.maxTorquePerKg = torquePerKg
            self.leftFootLoadFraction = leftLoad
            self.rightFootLoadFraction = rightLoad
            self.rootResidualPerKg = rootResPerKg
            self.groundHeightY = groundY
        }
    }

    func resetRealtimeState() {
        // Bump first so any in-flight frame's publish will be discarded.
        _ = bumpGeneration()

        solverQueue.async { [weak self] in
            guard let self else { return }
            self.dofFilters.removeAll(keepingCapacity: false)
            // Must be cleared with the SG filters, not separately: the two are
            // pushed in lockstep and the classifier reads "the last 9 samples"
            // as "the samples that produced this ddq".
            self.holdDetector.reset()
            // Pushed in lockstep with the filters and the hold detector: the
            // depth reference is a per-CLIP constant, so carrying it across a
            // clip boundary would hold the new subject at the old one's depth.
            self.depthHold.reset()
            self.lastMuscleSolveTimestamp = nil
        }

        activationFilters.removeAll(keepingCapacity: false)
        lastDisplayMuscleTimestamp = nil
        lastIKResult = nil
        lastIDResult = nil
        lastMuscleResult = nil
        lastSolve = nil
        displayMuscleResult = nil
        ikSolveTimeMs = 0
        idSolveTimeMs = 0
        muscleSolveTimeMs = 0
        ikMarkerResidualMeters = 0
        maxTorquePerKg = 0
        leftFootLoadFraction = 0
        rightFootLoadFraction = 0
        rootResidualPerKg = 0
    }

    /// Full session reset for a clip boundary. In addition to everything
    /// `resetRealtimeState()` clears, this drops NimbleBridge's session-only
    /// state: the IK warm-start pose and the rolling ground-height window
    /// (`NimbleBridge.h` `resetSessionState`). Without it a new clip warm-starts
    /// from the previous clip's unrelated pose and GRF contact detection reads a
    /// stale floor.
    func resetSessionState() {
        resetRealtimeState()
        solverQueue.async { [weak self] in
            self?.bridge.resetSessionState()
        }
    }

    private func normalizeForDisplay(_ muscle: MuscleOutput) -> MuscleOutput {
        let mergedActivations = normalizeActivations(muscle.activations)
        let smoothed = smoothActivations(mergedActivations, timestamp: muscle.timestamp)
        var out = MuscleOutput(
            // Activations are 0–1 effort ratios — merging heads takes max,
            // then 1€-filter removes per-frame QP jitter.
            activations: smoothed,
            // Forces in N from separate physical heads are additive.
            forces: normalizeForces(muscle.forces),
            converged: muscle.converged,
            timestamp: muscle.timestamp
        )
        // Pass raw paths/maxForces/rawActivations through (keyed by the
        // SOLVER's muscle names, not the display aliases) — the overlay
        // uses these to render muscles that the hardcoded def set
        // doesn't cover. `rawActivations` is intentionally NOT smoothed
        // because we don't keep a per-raw-name filter — path rendering
        // uses activation only for a visibility threshold and color tier,
        // which is not sensitive to one-frame jitter.
        out.paths = muscle.paths
        out.maxForces = muscle.maxForces
        out.rawActivations = muscle.rawActivations
        return out
    }

    private func smoothActivations(_ source: [String: Double],
                                   timestamp: TimeInterval) -> [String: Double] {
        var out: [String: Double] = [:]
        out.reserveCapacity(source.count)
        for (name, a) in source {
            let filter = activationFilters[name] ?? {
                let f = OneEuroFilter(minCutoff: 2.0, beta: 0.01, dCutoff: 1.0)
                activationFilters[name] = f
                return f
            }()
            out[name] = filter.filter(a, timestamp: timestamp)
        }
        return out
    }

    private func normalizeActivations(_ source: [String: Double]) -> [String: Double] {
        var normalized: [String: Double] = [:]
        normalized.reserveCapacity(source.count)
        for (name, value) in source {
            let displayName = Self.displayMuscleAliases[name] ?? name
            if let existing = normalized[displayName] {
                normalized[displayName] = max(existing, value)
            } else {
                normalized[displayName] = value
            }
        }
        return normalized
    }

    private func normalizeForces(_ source: [String: Double]) -> [String: Double] {
        var normalized: [String: Double] = [:]
        normalized.reserveCapacity(source.count)
        for (name, value) in source {
            let displayName = Self.displayMuscleAliases[name] ?? name
            normalized[displayName, default: 0] += value
        }
        return normalized
    }

    // MARK: - Recording

    func startRecordingResults() {
        ikHistory.removeAll()
        idHistory.removeAll()
        isRecordingResults = true
    }

    func stopRecordingResults() {
        isRecordingResults = false
    }

    /// Export IK results as .mot file.
    func exportMOT(filename: String = "BioMotion_ik") throws -> URL {
        guard !ikHistory.isEmpty else { throw ExportError.noData }

        let startTime = ikHistory.first!.timestamp
        let allDOFs = Array(ikHistory.first!.angles.keys).sorted()

        var lines: [String] = []
        lines.append(filename)
        lines.append("version=1")
        lines.append("nRows=\(ikHistory.count)")
        lines.append("nColumns=\(allDOFs.count + 1)")
        lines.append("inDegrees=yes")
        lines.append("endheader")

        // Column headers
        lines.append("time\t" + allDOFs.joined(separator: "\t"))

        // Data rows
        for entry in ikHistory {
            let time = entry.timestamp - startTime
            var row = String(format: "%.6f", time)
            for dof in allDOFs {
                let angleRad = entry.angles[dof] ?? 0.0
                let angleDeg = angleRad * 180.0 / .pi
                row += String(format: "\t%.4f", angleDeg)
            }
            lines.append(row)
        }

        let content = lines.joined(separator: "\n")
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("\(filename).mot")
        try content.write(to: url, atomically: true, encoding: .utf8)
        return url
    }

    /// Export ID results as .sto file.
    func exportSTO(filename: String = "BioMotion_id") throws -> URL {
        guard !idHistory.isEmpty else { throw ExportError.noData }

        let startTime = idHistory.first!.timestamp
        let allDOFs = Array(idHistory.first!.jointTorques.keys).sorted()

        var lines: [String] = []
        lines.append(filename)
        lines.append("version=1")
        lines.append("nRows=\(idHistory.count)")
        lines.append("nColumns=\(allDOFs.count + 1)")
        lines.append("inDegrees=no")
        lines.append("endheader")

        lines.append("time\t" + allDOFs.joined(separator: "\t"))

        for entry in idHistory {
            let time = entry.timestamp - startTime
            var row = String(format: "%.6f", time)
            for dof in allDOFs {
                row += String(format: "\t%.4f", entry.jointTorques[dof] ?? 0.0)
            }
            lines.append(row)
        }

        let content = lines.joined(separator: "\n")
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("\(filename).sto")
        try content.write(to: url, atomically: true, encoding: .utf8)
        return url
    }

    enum ExportError: Error {
        case noData
    }
}

// MARK: - Static-hold detection

/// Decides, from RAW MARKER MOTION alone, whether the subject was effectively
/// still around a given instant — so inverse dynamics there can be solved as a
/// static-equilibrium problem (q̇ = q̈ = 0) instead of from derivatives of a
/// motion we cannot fully observe.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHY THE CRITERION IS ON MARKERS, NOT ON `ddq`
/// ─────────────────────────────────────────────────────────────────────────
/// The tempting test is "is the Savitzky-Golay `ddq` small". It is circular
/// twice over. `ddq` is a twice-differentiated function of these same markers
/// (noise gain ≈ 1/dt², ≈3600 at 60 fps), and on the offline path the signal
/// being differentiated is missing its global translation entirely — the pose
/// model zeroes `global_trans`, pinning the pelvis at (0, 0.924, 0) every
/// frame. A small `ddq` on that path says the unobservable part of the motion
/// stayed unobservable. Marker displacement is the closest thing to a direct
/// observation this pipeline has.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHERE THE TWO CONSTANTS COME FROM (neither is tuned to a result)
/// ─────────────────────────────────────────────────────────────────────────
/// Static ID keeps gravity and discards `M·q̈ + C(q,q̇)q̇`. Both discarded
/// terms have to be small next to the gravitational term that survives.
///
/// 1. THE VELOCITY TERM. Centrifugal/Coriolis torque on a segment scales as
///    `m·v²/r` against a gravitational `m·g·r_g`; for comparable lever arms the
///    ratio is `v²/(g·r)`. On a 0.4 m thigh that is 1.0e-4 at v = 0.02 m/s and
///    **1.0% at v = 0.20 m/s**. So 0.20 m/s is where this term reaches one
///    percent, and that is what sets the speed cap.
///
/// 2. THE ACCELERATION TERM, bounded through duration. If no marker exceeds
///    `v` anywhere in a window of span `T`, then no marker's velocity changed
///    by more than `2v` across it, so the MEAN acceleration over the window is
///    at most `2v/T`. That mean must stay under `maxDiscardedMeanAccel`.
///
/// ⚠️ **Both constants were loosened 10× and 6.1× on 2026-08-07, and the
/// reason is that the old ones were not an error budget.** They were 0.02 m/s
/// and 0.08 m/s² — a discarded inertial term of **0.82% of g** — while the
/// stage immediately downstream, the muscle QP, carries a documented
/// **relative torque residual of 0.20 (neutral standing) to 0.35 (dancer)**,
/// and the pose source carries ~4.7° of joint-angle error. Demanding 0.8% from
/// one term while accepting 20-35% from the next one is a knob, not a budget:
/// it cannot improve the answer, because the term it is protecting is already
/// two orders of magnitude below the dominant error.
///
/// The replacement is stated as a fraction of g and chosen so the discarded
/// term stays an order of magnitude BELOW the muscle stage's own residual, so
/// it can never become the dominant error:
///
///     maxDiscardedMeanAccel = 0.05 · g = 0.49 m/s²      (5% of gravity)
///     holdSpeedThreshold    = 0.20 m/s                  (1% velocity term)
///
/// The implied minimum window span is `2v/a` = **0.8155 s** (it was 0.5 s).
/// Measured effect: at the offline path's 2 fps default the admissible peak
/// marker speed goes 2 cm/s → 20 cm/s, because the 4 s window the 9-tap filter
/// spans there makes the acceleration bound `2·0.2/4 = 0.1 m/s²`, far inside
/// the budget, so the speed cap is what binds.
///
/// ⚠️ This does NOT make a squat or a run publishable, and it is not meant to.
/// A runner's markers move at metres per second; no budget expressed as a
/// fraction of g reaches that. Motions that fast need the ROOT TRANSLATION,
/// which this input only carries when `cam_t` is composed in
/// (`MHRRetarget.makeBodyFrame(jointCoords:camT:…)`) — see
/// `rootTranslationObservable` below and STATUS.md.
///
/// ─────────────────────────────────────────────────────────────────────────
/// THE NOISE FLOOR — why "moving" and "cannot tell" are different answers
/// ─────────────────────────────────────────────────────────────────────────
/// `peakSpeed` is measured on marker positions that come from a pose model,
/// so it contains that model's per-frame jitter as well as the subject's
/// motion. At a sparse sampling rate the jitter alone can exceed the stillness
/// bound: `labs/sam-3d-body/findings/stability.json` measured body-joint
/// displacement of median 6.0 mm / max 24.7 mm under a 1% person-box
/// perturbation, which at 2 fps is 1.2 / 4.9 cm/s.
///
/// So this type measures the floor instead of assuming it, from distances that
/// physically CANNOT change: the rigid inter-joint-centre distances in
/// `rigidPairs` (hip width, femur, shank, humerus, …). Both skeletons hold
/// those fixed, so any frame-to-frame change in one is pure measurement noise.
/// If two markers carry independent position noise, a distance change of `d`
/// needs at least `d/2` on one of them, so `median|Δd| / (2·dt)` is a rigorous
/// LOWER bound on the per-marker speed attributable to noise.
///
/// When the floor is itself above the stillness bound, the honest verdict is
/// `.indistinguishableFromNoise` — the instrument cannot resolve the question —
/// and the advice is about the footage, not about holding still.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHAT THIS CANNOT DO — the bound is on the MEAN, over observed samples
/// ─────────────────────────────────────────────────────────────────────────
/// * Sub-sampling-rate oscillation (tremor, a bounce faster than the frame
///   interval) has small displacement and arbitrarily large instantaneous
///   acceleration. Nothing here sees it. At the offline path's 2 fps default
///   that is a wide blind spot.
/// * Rigid whole-body acceleration with no articulation — a subject
///   accelerating in a vehicle — moves no marker RELATIVE to the pinned
///   pelvis and reads as a hold. Articulated motion (a squat: the feet appear
///   to rise) does move markers and is caught.
/// * `peakSpeed` is a MAX over markers, deliberately. That is the strict
///   reading of "still", and it makes the detector fail toward "moving" —
///   refusing to report muscle numbers — which is the correct direction for
///   this product claim. It also means the detector inherits the pose
///   estimator's per-frame noise floor, which is why that floor is now
///   MEASURED (see above) instead of being an unstated assumption:
///   `labs/sam-3d-body/findings/stability.json` measured the model itself as
///   bit-identical across repeats (0.0 mm) but body-joint displacement of
///   median 2.4 mm / max 17.4 mm under a 0.1% bounding-box perturbation, and
///   median 6.0 mm / max 24.7 mm at 1%. Against the OLD 2 cm/s cap those
///   maxima (3.5 and 4.9 cm/s at 2 fps) were already above the threshold, so a
///   perfectly still subject was reported as "moving" — with advice they could
///   not act on. `.indistinguishableFromNoise` is that case, named.
/// * It does NOT check that the CAMERA was still, because it never sees an
///   image. `cam_t` and `joint_coords` are both camera-relative, so a rotating
///   camera makes the reconstructed frame non-inertial. Measured on the user's
///   own clips at native frame rate (background phase correlation outside a
///   dilated person box, chained): `video_012` drifts 1.79 image diagonals in
///   10.2 s — 13.5 °/s, which displaces a point 1 m from the subject by 6.4 cm
///   within ONE 0.27 s filter window — against 0.2-0.6 °/s (0.1-0.3 cm) on
///   `video_013`/`video_015`. That is a 20-60× separation, so the check is
///   buildable; it belongs upstream, where the frames are. ⚠️ It must run at
///   the video's NATIVE rate: at a 10 fps analysis sampling the same estimator
///   aliased `video_012`'s pan down to ~0.
/// Holds the root's DEPTH at one per-clip value, because monocular depth is the
/// one channel of the recovered root translation that cannot be differentiated.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHY HELD AND NOT FILTERED — this is a measurement, and it ruled out the
/// obvious design
/// ─────────────────────────────────────────────────────────────────────────
/// `cam_t`'s depth comes from apparent size (`tz = 2f/(bbox_side·s)`), and its
/// error is NOT high-frequency. Structure function on 284 consecutive frames of
/// a real clip at 30 fps — median `|T(t+τ) − T(t)|`, cm:
///
///     τ        0.033  0.100  0.167  0.400  0.500
///     x         0.66   1.73   1.99   2.26   2.53
///     y         1.36   3.43   4.14   3.55   3.81
///     z         5.15  12.56  14.33  15.44  15.27     <- saturates by ~0.15 s
///
/// The depth error reaches 12-15 cm by τ ≈ 0.15 s and then plateaus, i.e. it
/// lives at exactly the timescale a 9-tap window is trying to measure motion
/// at. A rolling low-pass therefore does nothing: measured, a rolling mean over
/// 0.25 / 0.5 / 1.0 / 2.0 s leaves the smoothed depth still moving at
/// 0.298 / 0.305 / 0.267 / 0.217 m/s, against a 0.20 m/s hold cap. There is no
/// window that separates this error from motion.
///
/// The only operation that removes an error which is smooth at the window scale
/// is to hold the channel at a single per-clip value — which is also the literal
/// content of the assumption this path is making: **the subject's distance from
/// the camera does not change**. That assumption is not declared and hoped for;
/// `StaticHoldDetector` gates on `depthDriftMetersPerSecond` and refuses the
/// frame with `.depthMotionNotResolvable` when it fails.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHY THE RESIDUAL POSITION ERROR IS BENIGN
/// ─────────────────────────────────────────────────────────────────────────
/// The correction is the SAME shift applied to every marker, so it is a rigid
/// translation of the whole body. Under uniform gravity a rigid translation
/// changes no joint torque, and it moves the contact polygon with the body so
/// the centre of pressure relative to the foot is unchanged. Only the body's
/// absolute z moves, and nothing downstream reads that as an absolute.
/// `StaticHoldTests` measures this rather than trusting it.
///
/// ─────────────────────────────────────────────────────────────────────────
/// IT IS A BIT-EXACT NO-OP UNTIL `cam_t` IS COMPOSED IN
/// ─────────────────────────────────────────────────────────────────────────
/// On a pelvis-pinned stream the root marker's z is the model constant 0 in
/// every frame, so the reference equals the current value and the offset is
/// exactly 0.0. This ships live and provably does nothing until
/// `OfflineSessionRunner` passes `camT:` — see `MHRRetarget.makeBodyFrame`.
struct RootDepthHold {

    /// Samples kept for the reference and the trend. 600 covers 10 s at 60 fps.
    static let maxHistorySamples = 600

    private var history: [(timestamp: TimeInterval, depth: Double)] = []
    /// The per-clip reference. Deliberately the MEDIAN of everything seen so
    /// far rather than the first value: the first frame is as noisy as any
    /// other, and a 15 cm wander on frame 1 would offset the whole clip.
    private var reference: Double?

    mutating func ingest(rootDepth: Double, timestamp: TimeInterval) {
        history.append((timestamp, rootDepth))
        if history.count > Self.maxHistorySamples {
            history.removeFirst(history.count - Self.maxHistorySamples)
        }
        let sorted = history.map(\.depth).sorted()
        reference = sorted[sorted.count / 2]
    }

    /// How much to SUBTRACT from every marker's z so the root sits at the
    /// reference depth. Exactly 0 when nothing has been seen, and exactly 0 on
    /// a pinned stream.
    func offset(forRootDepth z: Double) -> Double {
        guard let reference else { return 0 }
        return z - reference
    }

    /// Least-squares slope of the raw depth over the last `sampleCount`
    /// samples, m/s. Positive = moving away from the camera.
    ///
    /// A slope is used rather than an endpoint difference because the endpoints
    /// carry the full 12-15 cm wander while the slope averages it down: measured
    /// on a real clip, split-half slopes agreed to 0.0020 m/s against a 0.20 m/s
    /// budget, so the estimator is 100× sharper than the thing it gates.
    func driftMetersPerSecond(overLast sampleCount: Int) -> Double {
        let w = history.suffix(max(2, sampleCount))
        guard w.count >= 2 else { return .nan }
        let t0 = w.first!.timestamp
        var sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0
        let n = Double(w.count)
        for s in w {
            let x = s.timestamp - t0
            sx += x; sy += s.depth; sxx += x * x; sxy += x * s.depth
        }
        let denom = n * sxx - sx * sx
        guard abs(denom) > 1e-12 else { return .nan }
        return (n * sxy - sx * sy) / denom
    }

    mutating func reset() {
        history.removeAll(keepingCapacity: true)
        reference = nil
    }
}

struct StaticHoldDetector {

    /// Standard gravity, the term static ID keeps. Both budgets below are
    /// stated as fractions of it so they cannot drift into being knobs.
    static let gravityMetersPerSecondSquared: Double = 9.81

    /// Fraction of g the discarded inertial term is allowed to reach.
    /// 5% — an order of magnitude below the muscle QP's own 20-35% relative
    /// torque residual, so it cannot become the dominant error.
    static let discardedAccelFractionOfG: Double = 0.05

    /// Per-marker speed at or below which a sample counts as still, m/s.
    /// 0.20 m/s is where the centrifugal/Coriolis term `v²/(g·r)` reaches 1%
    /// on a 0.4 m segment — see the type comment, item 1.
    static let holdSpeedThresholdMetersPerSecond: Double = 0.20

    /// Ceiling on `2·peakSpeed/windowSpan`, the bound this window puts on the
    /// mean acceleration static ID throws away. 0.49 m/s² = 5% of g.
    static let maxDiscardedMeanAccelMetersPerSecondSquared: Double =
        discardedAccelFractionOfG * gravityMetersPerSecondSquared

    /// Ring capacity. Only has to cover the Savitzky-Golay window plus enough
    /// history to reach `2v/a` = 0.8155 s; 600 samples is 10 s at 60 fps and
    /// costs a few tens of kB.
    static let maxHistorySamples = 600

    /// Inter-joint-centre distances that are RIGID in both the MHR skeleton and
    /// `FullBody.osim`, so any frame-to-frame change in one is measurement
    /// noise rather than motion. Only direct joint reads are used — the blended
    /// spine markers (`SPINE_L`, `SPINE_M`, `NECK`, `HEAD`) interpolate between
    /// two MHR joints and are not rigid to each other.
    static let rigidPairs: [(String, String)] = [
        ("LHJC", "RHJC"), ("LSJC", "RSJC"),
        ("LHJC", "LKJC"), ("RHJC", "RKJC"),
        ("LKJC", "LAJC"), ("RKJC", "RAJC"),
        ("LSJC", "LEJC"), ("RSJC", "REJC"),
        ("LEJC", "LWJC"), ("REJC", "RWJC"),
    ]

    /// Marker whose constancy reveals that the pose source pinned the root.
    /// `MHRRetarget` emits `PELVIS` straight from MHR joint 1, which
    /// `joint_coords` fixes at the model constant (0, 0.924, 0) to the last
    /// bit whenever `cam_t` has not been composed in.
    static let rootMarkerName = "PELVIS"

    /// Below this, two `PELVIS` samples are treated as the same value. The
    /// pinned constant repeats bit-exactly and any real translation is metres,
    /// so this only has to be smaller than float noise.
    static let rootPinnedToleranceMeters: Double = 1e-9

    /// One pushed frame's motion relative to its predecessor.
    struct Sample {
        let timestamp: TimeInterval
        /// nil when there was no comparable predecessor (first sample of a
        /// clip, or no marker in common with the previous frame). Carries no
        /// motion information and is excluded from the max rather than being
        /// counted as zero.
        let peakSpeed: Double?
        let medianSpeed: Double?
        /// Largest change, in metres, of any distance in `rigidPairs` since the
        /// previous sample. Those distances cannot change, so this is pure
        /// pose-estimation noise. nil when no pair was measurable.
        let rigidDistanceDriftMeters: Double?
        /// Distance the root marker moved since the previous sample, metres.
        /// nil when it was absent from either frame.
        let rootDisplacementMeters: Double?
        /// The root marker's position, for the root noise floor. nil when the
        /// marker was absent.
        let rootPosition: SIMD3<Double>?
    }

    /// Scaling for the 4th-difference noise estimator. For a signal that is
    /// locally cubic — exactly what the 9-tap Savitzky-Golay filter fits — the
    /// 4th difference is identically zero, so what survives is what the filter
    /// will turn into acceleration. For gaussian noise of std σ the 4th
    /// difference has std `σ·√C(8,4) = σ·√70`, and `median|·| = 0.6745·std`.
    static let fourthDifferenceToSigma: Double = 0.6745 * 70.0.squareRoot()

    private var history: [Sample] = []
    private var previous: (timestamp: TimeInterval, markers: [String: SIMD3<Double>])?

    /// Feed one frame's markers, in the same flat `[x,y,z,…]` layout
    /// `NimbleBridge.solveIK` takes.
    mutating func ingest(flatMarkerPositions positions: [NSNumber],
                         markerNames names: [String],
                         timestamp: TimeInterval) {
        var markers: [String: SIMD3<Double>] = [:]
        markers.reserveCapacity(names.count)
        for (i, name) in names.enumerated() where positions.count >= (i + 1) * 3 {
            markers[name] = SIMD3<Double>(positions[i * 3].doubleValue,
                                          positions[i * 3 + 1].doubleValue,
                                          positions[i * 3 + 2].doubleValue)
        }

        var peak: Double?
        var median: Double?
        var rigidDrift: Double?
        var rootStep: Double?
        if let prev = previous {
            let dt = timestamp - prev.timestamp
            // Non-increasing timestamps mean the caller reset or replayed;
            // there is no speed to compute, so record the sample as
            // unmeasured rather than dividing by zero.
            if dt > 0 {
                var speeds: [Double] = []
                speeds.reserveCapacity(markers.count)
                for (name, p) in markers {
                    guard let q = prev.markers[name] else { continue }
                    speeds.append(simd_length(p - q) / dt)
                }
                if !speeds.isEmpty {
                    speeds.sort()
                    peak = speeds[speeds.count - 1]
                    median = speeds[speeds.count / 2]
                }

                // Noise probe: distances that physically cannot change.
                var drifts: [Double] = []
                for (a, b) in Self.rigidPairs {
                    guard let pa = markers[a], let pb = markers[b],
                          let qa = prev.markers[a], let qb = prev.markers[b] else { continue }
                    drifts.append(abs(simd_length(pa - pb) - simd_length(qa - qb)))
                }
                if !drifts.isEmpty { rigidDrift = drifts.max() }

                if let p = markers[Self.rootMarkerName], let q = prev.markers[Self.rootMarkerName] {
                    rootStep = simd_length(p - q)
                }
            }
        }

        history.append(Sample(timestamp: timestamp, peakSpeed: peak, medianSpeed: median,
                              rigidDistanceDriftMeters: rigidDrift,
                              rootDisplacementMeters: rootStep,
                              rootPosition: markers[Self.rootMarkerName]))
        if history.count > Self.maxHistorySamples {
            history.removeFirst(history.count - Self.maxHistorySamples)
        }
        previous = (timestamp, markers)
    }

    /// Verdict for the Savitzky-Golay window centred at `center`.
    ///
    /// The examined window is the last `SavitzkyGolayFilter.windowSize`
    /// samples, extended further back while its span is shorter than the
    /// duration the acceleration budget needs (`2·threshold/maxAccel`).
    ///
    /// Two reasons it starts from the whole filter window rather than from a
    /// short interval around `center`:
    ///
    /// 1. Those are the samples that produced the `ddq` being replaced.
    /// 2. They are also the samples that produced the smoothed `q` that IS
    ///    still used. If part of the window belongs to a different pose, the
    ///    cubic fit at the centre is pulled toward it, so requiring the whole
    ///    window to be still protects the POSE, not just the acceleration.
    ///
    /// ⚠️ Consequence worth knowing before reading a clip: the effective
    /// stillness requirement is therefore `max(8 × sampleInterval, 2v/a)`, not
    /// `2v/a` alone. At the offline path's 2 fps default the filter width alone
    /// makes it FOUR SECONDS, so a two-second hold mid-clip yields no muscle
    /// frames at all. That comes from the filter width times the sampling rate,
    /// not from either constant here — the actionable remedy is a higher
    /// sampling rate, not a longer hold. At 10 fps the same window is 0.8 s.
    ///
    /// The extension is backward-only, because samples past `center + 4` have
    /// not been pushed yet. That asymmetry is harmless: the symmetric filter
    /// window is always fully inside the examined window, and the extension
    /// only adds further evidence on the past side.
    func classify(centeredAt center: TimeInterval,
                  depthDriftMetersPerSecond depthDrift: Double = .nan)
        -> NimbleEngine.MotionClassification {
        guard let newest = history.last else {
            return Self.empty(at: center, windowSeconds: 0, sampleCount: 0)
        }

        let requiredSpan = 2 * Self.holdSpeedThresholdMetersPerSecond
            / Self.maxDiscardedMeanAccelMetersPerSecondSquared
        var start = max(0, history.count - SavitzkyGolayFilter.windowSize)
        while start > 0 && (newest.timestamp - history[start].timestamp) < requiredSpan {
            start -= 1
        }

        let window = history[start...]
        let span = newest.timestamp - window.first!.timestamp
        let measured = window.compactMap(\.peakSpeed)
        let medians = window.compactMap(\.medianSpeed).sorted()

        // The root translation is observable iff the root marker moved AT ALL
        // anywhere in the window. A pose source that pins the pelvis repeats
        // one model constant bit-for-bit, so this separates a pinned stream
        // from a `cam_t`-composed one with no flag to plumb and no way for the
        // two to disagree. A held pose reads as pinned, which is harmless: the
        // hold branch does not use the root acceleration.
        let rootSteps = window.compactMap(\.rootDisplacementMeters)
        let rootObservable = (rootSteps.max() ?? 0) > Self.rootPinnedToleranceMeters

        // Lower bound on the part of `peak` that is instrument noise. Median
        // over the window of the largest rigid-distance drift, halved (two
        // markers share the drift) and divided by the sampling interval.
        let drifts = window.compactMap(\.rigidDistanceDriftMeters).sorted()
        let sampleInterval = window.count > 1 ? span / Double(window.count - 1) : 0
        let articulationFloor: Double
        if drifts.isEmpty || sampleInterval <= 0 {
            articulationFloor = 0
        } else {
            articulationFloor = drifts[drifts.count / 2] / (2 * sampleInterval)
        }

        // The root's own floor. `rigidPairs` is invariant to root translation —
        // adding the same offset to both endpoints of a pair changes nothing —
        // so the articulation floor above is STRUCTURALLY BLIND to `cam_t`
        // jitter and would report a still subject as moving the moment the root
        // translation is composed in.
        let rootFloor = Self.fourthDifferenceFloor(
            window.compactMap(\.rootPosition), sampleInterval: sampleInterval)
        let noiseFloor = max(articulationFloor, rootFloor)

        // No measured sample at all means no motion information — not stillness.
        guard let peak = measured.max() else {
            return Self.empty(at: center, windowSeconds: span, sampleCount: window.count,
                              noiseFloor: noiseFloor, rootObservable: rootObservable,
                              articulationFloor: articulationFloor, rootFloor: rootFloor,
                              depthDrift: depthDrift)
        }

        // 2·v/T. A zero-span window (every sample at one timestamp, i.e. a
        // single photo before edge padding spreads it) can only pass if
        // nothing moved at all.
        let impliedAccel: Double
        if span > 0 {
            impliedAccel = 2 * peak / span
        } else {
            impliedAccel = peak > 0 ? .infinity : 0
        }

        let withinBudget = peak <= Self.holdSpeedThresholdMetersPerSecond
            && impliedAccel <= Self.maxDiscardedMeanAccelMetersPerSecondSquared

        // The depth assumption, checked rather than declared. `RootDepthHold`
        // pins the root's depth at one per-clip value, which is exact for
        // motion in the image plane and wrong for motion along the optical
        // axis. The discarded term is the depth inertia, so it takes the SAME
        // two constants as everything else — no new knobs: the depth speed must
        // be inside the speed cap, and `2·v_z/T` inside the acceleration budget.
        //
        // Tested on the RAW depth trend, not on the held value, because the
        // held value is zero by construction and would always pass.
        let depthWithinBudget: Bool
        if depthDrift.isNaN {
            depthWithinBudget = true      // no depth information: nothing to fail
        } else {
            let dz = abs(depthDrift)
            depthWithinBudget = dz <= Self.holdSpeedThresholdMetersPerSecond
                && (span <= 0 || 2 * dz / span <= Self.maxDiscardedMeanAccelMetersPerSecondSquared)
        }

        // Order matters. A hold is a hold regardless of the noise floor — the
        // floor can only ever make `peak` look BIGGER, so a peak that is
        // already inside the budget is inside it a fortiori. The floor is only
        // consulted to explain a FAILURE, where it decides between "the subject
        // moved" and "this footage cannot tell us".
        //
        // The depth check comes FIRST among the failures because it is the only
        // one that names something the user can fix by turning ninety degrees,
        // and because a subject walking at the camera also trips the speed cap —
        // reporting that as "you moved" would hide the actionable reason behind
        // a generic one.
        let verdict: NimbleEngine.MotionVerdict
        if !depthWithinBudget {
            verdict = .depthMotionNotResolvable
        } else if withinBudget {
            verdict = .hold
        } else if noiseFloor >= Self.holdSpeedThresholdMetersPerSecond {
            verdict = .indistinguishableFromNoise
        } else {
            verdict = .movingBeyondStaticBudget
        }

        return NimbleEngine.MotionClassification(
            timestamp: center,
            isHold: verdict == .hold,
            peakMarkerSpeedMetersPerSecond: peak,
            medianMarkerSpeedMetersPerSecond: medians.isEmpty ? 0 : medians[medians.count / 2],
            windowSeconds: span,
            sampleCount: window.count,
            impliedMeanAccelMetersPerSecondSquared: impliedAccel,
            verdict: verdict,
            poseNoiseFloorMetersPerSecond: noiseFloor,
            articulationNoiseFloorMetersPerSecond: articulationFloor,
            rootNoiseFloorMetersPerSecond: rootFloor,
            depthDriftMetersPerSecond: depthDrift,
            rootTranslationObservable: rootObservable)
    }

    /// σ of the per-sample noise on `points`, as a SPEED, from the 4th
    /// difference — which is identically zero for anything locally cubic, i.e.
    /// for anything the 9-tap Savitzky-Golay filter can represent. Whatever it
    /// leaves is what that filter will differentiate into acceleration.
    ///
    /// ⚠️ Blind, by construction, to error that is SMOOTH at the window scale.
    /// The `cam_t` depth error is exactly that — it saturates at 12-15 cm by
    /// τ ≈ 0.15 s and then plateaus — which is why depth is HELD by
    /// `RootDepthHold` rather than merely floored here. This estimator is for
    /// the in-plane channels, where the error is small and fast.
    static func fourthDifferenceFloor(_ points: [SIMD3<Double>],
                                      sampleInterval: Double) -> Double {
        guard points.count >= 5, sampleInterval > 0 else { return 0 }
        var mags: [Double] = []
        mags.reserveCapacity(points.count - 4)
        for i in 4..<points.count {
            // Δ⁴ = p[i] − 4p[i−1] + 6p[i−2] − 4p[i−3] + p[i−4]
            let d4 = points[i] - 4 * points[i - 1] + 6 * points[i - 2]
                - 4 * points[i - 3] + points[i - 4]
            mags.append(simd_length(d4))
        }
        guard !mags.isEmpty else { return 0 }
        mags.sort()
        let med = mags[mags.count / 2]
        return med / fourthDifferenceToSigma / sampleInterval
    }

    private static func empty(at center: TimeInterval,
                              windowSeconds: Double,
                              sampleCount: Int,
                              noiseFloor: Double = 0,
                              rootObservable: Bool = false,
                              articulationFloor: Double = 0,
                              rootFloor: Double = 0,
                              depthDrift: Double = .nan) -> NimbleEngine.MotionClassification {
        NimbleEngine.MotionClassification(
            timestamp: center, isHold: false,
            peakMarkerSpeedMetersPerSecond: 0, medianMarkerSpeedMetersPerSecond: 0,
            windowSeconds: windowSeconds, sampleCount: sampleCount,
            impliedMeanAccelMetersPerSecondSquared: .infinity,
            verdict: .noMeasurement,
            poseNoiseFloorMetersPerSecond: noiseFloor,
            articulationNoiseFloorMetersPerSecond: articulationFloor,
            rootNoiseFloorMetersPerSecond: rootFloor,
            depthDriftMetersPerSecond: depthDrift,
            rootTranslationObservable: rootObservable)
    }

    mutating func reset() {
        history.removeAll(keepingCapacity: true)
        previous = nil
    }
}
