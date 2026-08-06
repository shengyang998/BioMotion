import Foundation
import Combine
import QuartzCore
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
    /// Root 6D residual after GRF solve. < ~0.5 Nm/kg means GRF is consistent.
    @Published var rootResidualPerKg: Double = 0
    /// Current ground-plane height (ARKit world y), for display only.
    @Published var groundHeightY: Double = 0

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

            for i in 0..<numDOFs {
                let q = ikResult.jointAngles[i].doubleValue
                if let out = self.dofFilters[i].push(q, timestamp: frame.timestamp) {
                    smoothedQ.append(out.pos)
                    smoothedDQ.append(out.vel)
                    smoothedDDQ.append(out.acc)
                    centerTimestamp = out.center
                } else {
                    sgWarmedUp = false
                    break
                }
            }

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
                self.publishResults(ik: liveIkOutput, id: nil, muscle: nil,
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

            // --- ID on SG-smoothed q, dq, ddq ---
            let smoothedQNS: [NSNumber] = smoothedQ.map { NSNumber(value: $0) }
            let smoothedDQNS: [NSNumber] = smoothedDQ.map { NSNumber(value: $0) }
            let smoothedDDQNS: [NSNumber] = smoothedDDQ.map { NSNumber(value: $0) }

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
                                ikTime: ikTime, idTime: idTime, muscleTime: muscleTime,
                                ikResidual: ikResult.error, maxTorqueNm: maxTorqueNm,
                                groundY: self.bridge.groundHeightY,
                                generation: frameGeneration)
        }
    }

    private func publishResults(ik: IKOutput, id: IDOutput?, muscle: MuscleOutput?,
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
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            // Drop late publishes from a pre-reset generation so they don't
            // overwrite the cleared @Published state.
            guard self.readGeneration() == generation else { return }
            self.lastIKResult = ik
            self.lastIDResult = id
            self.lastMuscleResult = muscle
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
            self.lastMuscleSolveTimestamp = nil
        }

        activationFilters.removeAll(keepingCapacity: false)
        lastDisplayMuscleTimestamp = nil
        lastIKResult = nil
        lastIDResult = nil
        lastMuscleResult = nil
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
