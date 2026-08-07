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
    ///   descends and the feet appear to rise instead. Dynamic ID is therefore
    ///   not sound on that path at all, and static-equilibrium ID over a
    ///   detected hold is the only honest reading (STATUS.md, "The limitation
    ///   that shapes the product claim").
    /// * The live ARKit path supplies real world-space joint positions
    ///   including global translation, so its q̈ is observable and gating it
    ///   would remove working behaviour. Untouched by default.
    ///
    /// Read and captured on the MAIN thread inside `processFrame`, which is
    /// already a main-thread API (`isFrameInFlight`/`droppedFrameCount` are
    /// documented as main-only), so there is no cross-queue race.
    var staticHoldGating: Bool = false

    /// Processed IK output with named DOFs.
    struct IKOutput {
        let jointAngles: [String: Double]  // DOF name → angle in radians
        let error: Double                   // RMS marker error in meters
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

    /// What the hold detector concluded about one instant.
    ///
    /// `timestamp` is the Savitzky-Golay window CENTRE — the same instant
    /// `IDOutput`/`MuscleOutput` are dated at — not the newest pushed frame.
    struct MotionClassification: Equatable {
        let timestamp: TimeInterval
        /// True iff every measured marker speed in the examined window stayed
        /// under `StaticHoldDetector.holdSpeedThresholdMetersPerSecond` AND the
        /// window was long enough to bound the discarded acceleration.
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
    private(set) var ikHistory: [(timestamp: TimeInterval, angles: [String: Double], error: Double)] = []
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

        for joint in frame.joints where joint.isTracked {
            // Map ARKit joint to OpenSim marker name
            if let mapping = JointMapping.primary.first(where: { $0.arkitName == joint.id }) {
                names.append(mapping.opensimName)
                positions.append(NSNumber(value: Double(joint.worldPosition.x)))
                positions.append(NSNumber(value: Double(joint.worldPosition.y)))
                positions.append(NSNumber(value: Double(joint.worldPosition.z)))
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
                error: ikResult.error,
                timestamp: frame.timestamp
            )

            guard sgWarmedUp else {
                // Still warming up: publish live IK only, no ID/muscle yet.
                // No motion verdict either — there is no window centre to date
                // one at, so `lastSolve` stays nil rather than carrying a guess.
                self.publishResults(ik: liveIkOutput, id: nil, muscle: nil,
                                    motion: nil, isStaticHoldEstimate: false,
                                    ikTime: ikTime, idTime: 0, muscleTime: 0,
                                    ikResidual: ikResult.error, maxTorqueNm: 0,
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
                error: ikResult.error,
                timestamp: centerTimestamp
            )

            // --- Was the subject still at the window centre? ---
            // Classified at `centerTimestamp`, not at the newest push: that is
            // the instant ID and the muscle solve are dated at, and the centred
            // window means we already hold 4 samples of "future" around it.
            let motion = self.holdDetector.classify(centeredAt: centerTimestamp)

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
                    self.ikHistory.append((centerTimestamp, smoothedAngles, ikResult.error))
                }
                self.publishResults(ik: smoothedIkOutput, id: nil, muscle: nil,
                                    motion: motion, isStaticHoldEstimate: false,
                                    ikTime: ikTime, idTime: 0, muscleTime: 0,
                                    ikResidual: ikResult.error, maxTorqueNm: 0,
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
                self.ikHistory.append((centerTimestamp, smoothedAngles, ikResult.error))
                if let id = idOutput {
                    self.idHistory.append((centerTimestamp, id.jointTorques))
                }
            }

            self.publishResults(ik: smoothedIkOutput, id: idOutput, muscle: muscleOutput,
                                motion: motion, isStaticHoldEstimate: solveAsStatics,
                                ikTime: ikTime, idTime: idTime, muscleTime: muscleTime,
                                ikResidual: ikResult.error, maxTorqueNm: maxTorqueNm,
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
/// 1. THE VELOCITY TERM is not what binds. Centrifugal/Coriolis torque on a
///    segment scales as `m·v²/r` against a gravitational `m·g·r_g`; for
///    comparable lever arms the ratio is `v²/(g·r)`. A 0.4 m thigh at
///    v = 0.02 m/s gives 1.0e-4. Even at 0.2 m/s it is only ~1%. So the speed
///    cap is not set by this term — it is set by what it buys us in (2).
///
/// 2. THE ACCELERATION TERM is what binds, and speed bounds it through
///    duration. If no marker exceeds `v` anywhere in a window of span `T`,
///    then no marker's velocity changed by more than `2v` across it, so the
///    MEAN acceleration over the window is at most `2v/T`. Requiring that mean
///    to stay under `maxDiscardedMeanAccel` = 0.08 m/s² — 0.8% of g, i.e. the
///    discarded term is under 1% of the term it is being compared against —
///    is the actual criterion. At v = 0.02 m/s it implies T ≥ 0.5 s.
///
/// `holdSpeedThresholdMetersPerSecond` = 0.02 and the 0.5 s duration in
/// STATUS.md next-step 5 are therefore ONE statement, not two knobs: 0.5 s is
/// `2 × 0.02 / 0.08`. The window-span term is enforced explicitly rather than
/// as a fixed minimum duration so that a short clip degrades gracefully — a
/// 0.27 s window simply has to be proportionally slower to pass.
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
///   estimator's per-frame noise floor: `labs/sam-3d-body/findings/stability.json`
///   measured the model itself as bit-identical across repeats (0.0 mm) but
///   body-joint displacement of median 2.4 mm / max 17.4 mm under a 0.1%
///   bounding-box perturbation, and median 6.0 mm / max 24.7 mm at 1%. At the
///   2 fps offline default those maxima are 3.5 and 4.9 cm/s — ABOVE the
///   2 cm/s cap. So on real video a still subject can still be classified
///   moving, purely from person-detector box wobble. Real per-frame jitter in
///   the shipped pipeline has not been measured (the Core ML model lives in an
///   asset pack and is not in the test bundle); see this file set's report.
struct StaticHoldDetector {

    /// Per-marker speed at or below which a sample counts as still, m/s.
    /// See the type comment: 2 cm/s, with the acceleration budget below, is
    /// STATUS.md next-step 5's "< ~2 cm/s sustained for ≥ 0.5 s".
    static let holdSpeedThresholdMetersPerSecond: Double = 0.02

    /// Ceiling on `2·peakSpeed/windowSpan`, the bound this window puts on the
    /// mean acceleration static ID throws away. 0.08 m/s² = 0.82% of g.
    static let maxDiscardedMeanAccelMetersPerSecondSquared: Double = 0.08

    /// Ring capacity. Only has to cover the Savitzky-Golay window plus enough
    /// history to reach `2v/a` = 0.5 s; 600 samples is 10 s at 60 fps and
    /// costs a few tens of kB.
    static let maxHistorySamples = 600

    /// One pushed frame's motion relative to its predecessor.
    struct Sample {
        let timestamp: TimeInterval
        /// nil when there was no comparable predecessor (first sample of a
        /// clip, or no marker in common with the previous frame). Carries no
        /// motion information and is excluded from the max rather than being
        /// counted as zero.
        let peakSpeed: Double?
        let medianSpeed: Double?
    }

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
            }
        }

        history.append(Sample(timestamp: timestamp, peakSpeed: peak, medianSpeed: median))
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
    /// stillness requirement is therefore `8 × sampleInterval`, not 0.5 s. At
    /// the offline path's 2 fps default that is FOUR SECONDS, so a two-second
    /// hold mid-clip yields no muscle frames at all. That comes from the filter
    /// width times the sampling rate, not from either constant here.
    ///
    /// The extension is backward-only, because samples past `center + 4` have
    /// not been pushed yet. That asymmetry is harmless: the symmetric filter
    /// window is always fully inside the examined window, and the extension
    /// only adds further evidence on the past side.
    func classify(centeredAt center: TimeInterval) -> NimbleEngine.MotionClassification {
        guard let newest = history.last else {
            return NimbleEngine.MotionClassification(
                timestamp: center, isHold: false,
                peakMarkerSpeedMetersPerSecond: 0, medianMarkerSpeedMetersPerSecond: 0,
                windowSeconds: 0, sampleCount: 0,
                impliedMeanAccelMetersPerSecondSquared: .infinity)
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

        // No measured sample at all means no motion information — not stillness.
        guard let peak = measured.max() else {
            return NimbleEngine.MotionClassification(
                timestamp: center, isHold: false,
                peakMarkerSpeedMetersPerSecond: 0, medianMarkerSpeedMetersPerSecond: 0,
                windowSeconds: span, sampleCount: window.count,
                impliedMeanAccelMetersPerSecondSquared: .infinity)
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

        let isHold = peak <= Self.holdSpeedThresholdMetersPerSecond
            && impliedAccel <= Self.maxDiscardedMeanAccelMetersPerSecondSquared

        return NimbleEngine.MotionClassification(
            timestamp: center,
            isHold: isHold,
            peakMarkerSpeedMetersPerSecond: peak,
            medianMarkerSpeedMetersPerSecond: medians.isEmpty ? 0 : medians[medians.count / 2],
            windowSeconds: span,
            sampleCount: window.count,
            impliedMeanAccelMetersPerSecondSquared: impliedAccel)
    }

    mutating func reset() {
        history.removeAll(keepingCapacity: true)
        previous = nil
    }
}
