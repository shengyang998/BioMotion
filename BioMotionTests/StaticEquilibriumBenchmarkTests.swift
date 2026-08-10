import XCTest
@testable import BioMotion

/// A KNOWN-ANSWER benchmark for the inverse-dynamics stage.
///
/// `OfflineMuscleChainTests` runs a real dancer pose. That pose is
/// representative but useless as a reference, because nobody can say by hand
/// what its ankle moment ought to be. Here the pose is built by hand — a
/// symmetric, upright, feet-flat standing pose — so every number the solver
/// reports can be checked against arithmetic written out in the comments.
///
/// Nothing in this file runs an ML model, reads a fixture, or scales the
/// skeleton. The marker positions are literals derived from adult segment
/// lengths, and the expected torques come from free-body statics on the
/// FullBody.osim mass table.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHAT THIS FILE GUARDS (regression, 2026-08-07)
/// ─────────────────────────────────────────────────────────────────────────
/// Every assertion here failed before the two-part fix in `NimbleBridge.mm`
/// and passes after it. Do not relax a bound to make a number fit — each one
/// is derived from arithmetic in the comment above it, not chosen.
///
/// 1. GRAVITY DIRECTION. DART's Skeleton default gravity is
///    `Eigen::Vector3s(0, 0, -9.81)` (Z-up;
///    nimblephysics/dart/dynamics/detail/SkeletonAspect.hpp:82). OpenSim models
///    are Y-up and declare `<gravity>0 -9.8066 0</gravity>`, but
///    `OpenSimParser` never reads that element. Unset, the whole ID stack
///    pulled along the subject's medio-lateral axis, which turns body HEIGHT
///    into the moment arm instead of the few-centimetre horizontal offsets that
///    actually load a standing leg. `testSkeletonGravityPointsDownTheModelsYAxis`
///    measured `tau(pelvis_ty) = 3.6e-15 N`, `tau(pelvis_tz) = 780.71 N`.
///
/// 2. CONTACT-WRENCH FRAME. `getMultipleContactInverseDynamicsNearCoP` takes
///    and returns wrenches in each contact body's OWN frame (it builds its
///    Jacobians with `getJacobian(body)`, which MetaSkeleton.hpp:559 documents
///    as body-frame/body-origin, and maps guesses to world with `dAdInvT` at
///    Skeleton.cpp:10205; the matching world conversion on the way out is
///    commented out at Skeleton.cpp:10354). BioMotion passed world-frame
///    guesses and read the results back as world, so the reported GRF vector,
///    the CoP, and — on a two-foot stance, where the six root equations leave a
///    six-dimensional null space — the actual load split were all wrong.
///
/// `rootResidualNorm` was NOT evidence of equilibrium either: nimble ends the
/// solve with `result.jointTorques.head<6>().setZero()` and the assert above
/// that line is compiled out (Release, -DNDEBUG), so the old readback was a
/// hard-coded zero. It now measures ‖ΣF_contact + m·g − m·a_com‖ in newtons.
///
/// ─────────────────────────────────────────────────────────────────────────
/// FRAME
/// ─────────────────────────────────────────────────────────────────────────
/// Marker world frame matches the OpenSim model frame:
///   +x = anterior (the direction the subject faces)
///   +y = up
///   +z = the subject's right
/// Floor is y = 0. The pose is mirror-symmetric about the z = 0 plane.
///
/// ─────────────────────────────────────────────────────────────────────────
/// MODEL CONSTANTS (read out of BioMotion/Resources/FullBody.osim; the mass
/// table is fixed because no test here calls `scaleModelWithHeight:`)
/// ─────────────────────────────────────────────────────────────────────────
///   total mass                       79.5835 kg   -> bodyweight 780.71 N
///   femur                             9.3014 kg   (each side)
///   tibia+kneecap+talus+calcn+toes    5.3603 kg   (each side)
///   whole leg                        14.6617 kg   (each side)
/// Neutral (all-coordinates-zero) forward kinematics, relative to the pelvis
/// body origin — these are the rigid geometry the markers must respect:
///   femur origin  (-0.0563, -0.0785, ±0.0773)
///   tibia origin  (-0.0527, -0.4846, ±0.0770)
///   talus origin  (-0.0627, -0.8846, ±0.0770)   <- ankle joint centre
///   calcn origin  (-0.1115, -0.9265, ±0.0849)   <- heel
///   toes  origin  ( 0.0673, -0.9285, ±0.0860)   <- MTP joint
///   whole-body centre of mass (-0.0746, -0.0006, 0.0000)
final class StaticEquilibriumBenchmarkTests: XCTestCase {

    private var bridge: NimbleBridge!

    override func setUp() {
        super.setUp()
        continueAfterFailure = true   // every metric must print, even after a failed assert
        bridge = NimbleBridge()
    }

    private func loadFullBody() throws {
        let path = try XCTUnwrap(
            Bundle(for: type(of: self)).path(forResource: "FullBody", ofType: "osim")
                ?? Bundle.main.path(forResource: "FullBody", ofType: "osim"),
            "Cannot find FullBody.osim")
        XCTAssertTrue(bridge.loadModel(fromPath: path), "bridge failed to load FullBody.osim")
    }

    /// Same, plus the two muscle-side parsers, for the end-to-end dancer test.
    private func loadFullBodyWithMuscles() throws -> (MomentArmComputer, MuscleSolver) {
        let path = try XCTUnwrap(
            Bundle(for: type(of: self)).path(forResource: "FullBody", ofType: "osim")
                ?? Bundle.main.path(forResource: "FullBody", ofType: "osim"),
            "Cannot find FullBody.osim")
        XCTAssertTrue(bridge.loadModel(fromPath: path), "bridge failed to load FullBody.osim")
        let computer = MomentArmComputer()
        let solver = MuscleSolver()
        XCTAssertTrue(computer.parseMusclePaths(fromOsimPath: path, from: bridge))
        XCTAssertTrue(solver.loadMuscles(fromOsimPath: path))
        return (computer, solver)
    }

    // MARK: - Pose construction

    /// Adult standing anthropometry, in metres. Each value is a realistic adult
    /// number AND agrees with FullBody.osim's own neutral geometry to <= 0.5 mm.
    ///
    /// That agreement is deliberate, not circular. Segment *lengths* and the
    /// pelvis/foot widths are rigid geometry: no joint angle can change them,
    /// so inventing a different number would not produce a different pose, it
    /// would only produce IK residual and contaminate the benchmark. The free
    /// choices in this pose — legs straight, feet flat, trunk vertical, arms
    /// down, 50/50 stance, the lean angle below — are all made here.
    private enum Anthro {
        static let ankleHeight   = 0.0750   // ankle JC above floor (~4.3% of a 1.75 m stature)
        static let shankLength   = 0.4000   // ankle JC -> knee JC     (osim: 0.4000)
        static let thighLength   = 0.4060   // knee JC  -> hip JC      (osim: 0.4061)
        static let pelvisRise    = 0.0785   // hip JC   -> pelvis origin (osim: 0.0785)

        // Anterior offsets from the pelvis origin (x = 0 at the pelvis).
        static let hipX          = -0.0563
        static let kneeX         = -0.0527
        static let ankleX        = -0.0627
        static let ankleToToeX   =  0.1300  // MTP is 13.0 cm anterior of the ankle JC
        static let ankleToToeY   = -0.0440  // ...and 4.4 cm below it

        // Half-separations about the sagittal plane (the stance is hip-width).
        static let hipHalfWidth   = 0.0773
        static let kneeHalfWidth  = 0.0770
        static let ankleHalfWidth = 0.0770
        static let toeHalfWidth   = 0.0860

        static var pelvisY: Double { ankleHeight + shankLength + thighLength + pelvisRise }
        //                         = 0.0750 + 0.4000 + 0.4060 + 0.0785 = 0.9595 m
        static var hipY: Double   { ankleHeight + shankLength + thighLength }   // 0.8810
        static var kneeY: Double  { ankleHeight + shankLength }                 // 0.4750
        static var toeY: Double   { ankleHeight + ankleToToeY }                 // 0.0310
        static var toeX: Double   { ankleX + ankleToToeX }                      // 0.0673
    }

    /// Trunk, head and arm markers as offsets from the pelvis origin. These are
    /// rigid multi-segment geometry (a chain of vertebrae, a clavicle/scapula
    /// complex), so they are taken from the model's neutral FK rather than
    /// invented, for the same reason as the segment lengths above. The pose
    /// choice they encode is "trunk vertical, arms hanging at the sides".
    private static let trunkOffsets: [(String, Double, Double, Double)] = [
        ("SPINE_L", -0.1019,  0.1447,  0.0000),   // lumbar3
        ("SPINE_M", -0.1595,  0.3849,  0.0000),   // thoracic7
        ("C7",      -0.1173,  0.5217,  0.0000),   // thoracic1
        ("NECK",    -0.1130,  0.5426,  0.0000),   // head_neck origin
        ("HEAD",    -0.1130,  0.6926,  0.0000),   // head_neck + 0.15 m (the bridge's HEAD offset)
        ("RSJC",    -0.0997,  0.4715,  0.1706),   // humerus_r
        ("LSJC",    -0.0997,  0.4715, -0.1706),
        ("REJC",    -0.0865,  0.1853,  0.1610),   // ulna_r
        ("LEJC",    -0.0865,  0.1853, -0.1610),
        ("RWJC",    -0.1020, -0.0636,  0.2007),   // hand_r
        ("LWJC",    -0.1020, -0.0636, -0.2007),
    ]

    /// Builds the 20 `JointMapping.primary` markers for a symmetric upright
    /// stance, optionally leaning the whole body forward about the ankle axis.
    ///
    /// `leanRad` rotates every marker at or above the ankle joint centre about
    /// the medio-lateral line through both ankle joint centres, and leaves the
    /// foot markers (ankle JC is on the axis; MTP) where they are — i.e. the
    /// feet stay flat on the floor and the ankles dorsiflex. A NEGATIVE angle
    /// is a rotation about +z, which carries points above the axis towards +x,
    /// i.e. anteriorly. This is exactly the inverted-pendulum model of quiet
    /// standing, and it is the only degree of freedom this benchmark varies.
    private static func standingMarkers(leanRad: Double) -> [(String, SIMD3<Double>)] {
        let ax = Anthro.ankleX
        let ay = Anthro.ankleHeight
        func lean(_ p: SIMD3<Double>) -> SIMD3<Double> {
            let dx = p.x - ax, dy = p.y - ay
            let c = cos(leanRad), s = sin(leanRad)
            return SIMD3<Double>(ax + dx * c - dy * s, ay + dx * s + dy * c, p.z)
        }

        var out: [(String, SIMD3<Double>)] = []

        // Feet — NOT rotated (flat on the floor).
        for (name, sign) in [("RTOE", 1.0), ("LTOE", -1.0)] {
            out.append((name, SIMD3<Double>(Anthro.toeX, Anthro.toeY, sign * Anthro.toeHalfWidth)))
        }
        // Ankle joint centres sit ON the rotation axis, so leaning does not move them.
        for (name, sign) in [("RAJC", 1.0), ("LAJC", -1.0)] {
            out.append((name, SIMD3<Double>(ax, ay, sign * Anthro.ankleHalfWidth)))
        }
        // Everything above the ankle leans as one rigid body.
        for (name, sign) in [("RKJC", 1.0), ("LKJC", -1.0)] {
            out.append((name, lean(SIMD3<Double>(Anthro.kneeX, Anthro.kneeY, sign * Anthro.kneeHalfWidth))))
        }
        for (name, sign) in [("RHJC", 1.0), ("LHJC", -1.0)] {
            out.append((name, lean(SIMD3<Double>(Anthro.hipX, Anthro.hipY, sign * Anthro.hipHalfWidth))))
        }
        let pelvis = SIMD3<Double>(0.0, Anthro.pelvisY, 0.0)
        out.append(("PELVIS", lean(pelvis)))
        for (name, dx, dy, dz) in trunkOffsets {
            out.append((name, lean(SIMD3<Double>(pelvis.x + dx, pelvis.y + dy, pelvis.z + dz))))
        }
        return out
    }

    // MARK: - Shared plumbing

    private struct Solved {
        let ik: NimbleIKResult
        let id: NimbleIDResult
        let dofNames: [String]
        let torques: [Double]
        func torque(_ name: String) -> Double {
            guard let i = dofNames.firstIndex(of: name) else { return .nan }
            return torques[i]
        }
    }

    /// Runs the production photo path on a marker set: IK -> Savitzky-Golay
    /// warm-up on a repeated frame (so dq, ddq collapse to ~0) -> ID with GRF.
    /// Same call sequence as `OfflineMuscleChainTests`, so nothing here is a
    /// special-cased solver path.
    private func solve(markers: [(String, SIMD3<Double>)], tag: String) throws -> Solved {
        var positions: [NSNumber] = []
        var names: [String] = []
        let supportedSourceMarkers = Set(JointMapping.primary.map(\.opensimName)).union(["MHR_ROOT"])
        for (opensimName, p) in markers {
            XCTAssertTrue(supportedSourceMarkers.contains(opensimName),
                          "\(opensimName) is not a supported source marker")
            names.append(opensimName)
            positions.append(NSNumber(value: p.x))
            positions.append(NSNumber(value: p.y))
            positions.append(NSNumber(value: p.z))
        }
        XCTAssertEqual(names.count, JointMapping.primary.count, "benchmark must use all 20 markers")

        let ik = try XCTUnwrap(bridge.solveIK(withMarkerPositions: positions, markerNames: names),
                               "solveIK returned nil")
        let n = ik.jointAngles.count
        // `NimbleIKResult.error` is Nimble's LOSS (sum of squared weighted
        // residuals over the marker stack), not a distance — see
        // `lossBoundForResidual` in NimbleBridge.mm. Per-marker RMS is
        // sqrt(loss / nMarkers).
        let perMarkerRMS = (ik.error / Double(names.count)).squareRoot()
        print("BENCH-METRIC [\(tag)] ik_dofs=\(n) ik_loss=\(ik.error) ik_per_marker_rms_m=\(perMarkerRMS)")

        var filters = (0..<n).map { _ in SavitzkyGolayFilter() }
        var q: [Double] = [], dq: [Double] = [], ddq: [Double] = []
        let dt = 0.5
        for push in 0..<SavitzkyGolayFilter.windowSize {
            q.removeAll(); dq.removeAll(); ddq.removeAll()
            for i in 0..<n {
                if let out = filters[i].push(ik.jointAngles[i].doubleValue, timestamp: Double(push) * dt) {
                    q.append(out.pos); dq.append(out.vel); ddq.append(out.acc)
                }
            }
        }
        XCTAssertEqual(q.count, n, "Savitzky-Golay did not warm up")
        print("BENCH-METRIC [\(tag)] max_dq=\(dq.map { abs($0) }.max() ?? 0) max_ddq=\(ddq.map { abs($0) }.max() ?? 0)")

        let id = try XCTUnwrap(
            bridge.solveIDGRF(withJointAngles: q.map { NSNumber(value: $0) },
                              jointVelocities: dq.map { NSNumber(value: $0) },
                              jointAccelerations: ddq.map { NSNumber(value: $0) }),
            "solveIDGRF returned nil")
        return Solved(ik: ik, id: id, dofNames: ik.dofNames,
                      torques: id.jointTorques.map { $0.doubleValue })
    }

    private func report(_ s: Solved, tag: String) {
        let f = s.id.rightFootForce.map { $0.doubleValue }
        let fl = s.id.leftFootForce.map { $0.doubleValue }
        let cop = s.id.rightFootCoP.map { $0.doubleValue }
        let copl = s.id.leftFootCoP.map { $0.doubleValue }
        let fMag = (f[0]*f[0] + f[1]*f[1] + f[2]*f[2]).squareRoot()
        let flMag = (fl[0]*fl[0] + fl[1]*fl[1] + fl[2]*fl[2]).squareRoot()
        print("BENCH-METRIC [\(tag)] mass_kg=\(bridge.totalMass) bodyweight_N=\(bridge.totalMass * 9.81)")
        print("BENCH-METRIC [\(tag)] contact L=\(s.id.leftFootInContact) R=\(s.id.rightFootInContact) ground_y=\(bridge.groundHeightY)")
        print("BENCH-METRIC [\(tag)] rightForce=(\(f[0]),\(f[1]),\(f[2])) |F|=\(fMag)")
        print("BENCH-METRIC [\(tag)] leftForce=(\(fl[0]),\(fl[1]),\(fl[2])) |F|=\(flMag)")
        print("BENCH-METRIC [\(tag)] grf_sum_of_magnitudes_N=\(fMag + flMag)")
        // The VECTOR sum is the diagnostic that matters: whatever the solver
        // splits between the feet, the total contact force must be bodyweight
        // straight up, i.e. (0, +780.71, 0). A total that points along z is the
        // solver balancing a gravity vector that points along z.
        let sum = (fR: f[0] + fl[0], up: f[1] + fl[1], lat: f[2] + fl[2])
        print("BENCH-METRIC [\(tag)] grf_vector_sum=(\(sum.fR),\(sum.up),\(sum.lat)) expected=(0,\(bridge.totalMass * 9.81),0)")
        print("BENCH-METRIC [\(tag)] rightCoP=(\(cop[0]),\(cop[1]),\(cop[2])) leftCoP=(\(copl[0]),\(copl[1]),\(copl[2]))")
        print("BENCH-METRIC [\(tag)] rootResidualNorm=\(s.id.rootResidualNorm)")

        let ranked = zip(s.dofNames, s.torques.map { abs($0) }).sorted { $0.1 > $1.1 }.prefix(12)
        for (name, t) in ranked { print("BENCH-METRIC [\(tag)] top_torque \(name)=\(t)") }
        for name in Self.legDOFs {
            print("BENCH-METRIC [\(tag)] leg_torque \(name)=\(s.torque(name))")
        }
    }

    /// For a quasi-static pose the contact forces must sum to bodyweight
    /// pointing straight UP in the world frame — not just have bodyweight's
    /// magnitude. This is the assertion the body-local readback made
    /// impossible: before the frame fix the two feet reported
    /// (464.5, -333.7, +539.3) N and (-465.1, +331.7, +238.6) N, which sum to
    /// bodyweight along the subject's lateral axis.
    private func assertContactForceIsUprightBodyweight(_ s: Solved, tag: String) {
        let fR = s.id.rightFootForce.map { $0.doubleValue }
        let fL = s.id.leftFootForce.map { $0.doubleValue }
        let bw = bridge.totalMass * 9.81
        let sum = (fR[0] + fL[0], fR[1] + fL[1], fR[2] + fL[2])
        print("BENCH-METRIC [\(tag)] grf_vector_sum_assert=(\(sum.0),\(sum.1),\(sum.2)) expected=(0,\(bw),0)")
        XCTAssertEqual(sum.1, bw, accuracy: 0.02 * bw,
                       "vertical GRF must be bodyweight; got \(sum.1) N")
        XCTAssertLessThan(abs(sum.0), 0.05 * bw,
                          "anterior GRF must vanish in a static pose; got \(sum.0) N")
        XCTAssertLessThan(abs(sum.2), 0.05 * bw,
                          "lateral GRF must vanish in a static pose; got \(sum.2) N")
    }

    /// In a static pose with zero net horizontal GRF, the net centre of
    /// pressure must sit directly under the whole-body centre of mass. That is
    /// a one-line consequence of ΣM = 0 and it is the sharpest available check
    /// on the contact-wrench readback, because it compares a solved quantity
    /// against a number derived by hand from the mass table alone.
    ///
    /// It is also the check that separates the two halves of the fix. With
    /// gravity corrected but the wrenches still read back in body-local
    /// coordinates, the lean4deg CoP came out at x = +0.0987 m — 11 cm from
    /// where the CoM is — while every torque bound in this file still passed.
    private func assertNetCoPSitsUnderCoM(_ s: Solved, expectedX: Double, tag: String) {
        let fR = s.id.rightFootForce.map { $0.doubleValue }
        let fL = s.id.leftFootForce.map { $0.doubleValue }
        let cR = s.id.rightFootCoP.map { $0.doubleValue }
        let cL = s.id.leftFootCoP.map { $0.doubleValue }
        let wR = fR[1], wL = fL[1]                    // vertical load per foot
        let total = wR + wL
        XCTAssertGreaterThan(total, 1.0, "no vertical load to weight the CoP with")
        let netX = (cR[0] * wR + cL[0] * wL) / total
        let netZ = (cR[2] * wR + cL[2] * wL) / total
        print("BENCH-METRIC [\(tag)] net_CoP_x=\(netX) expected=\(expectedX) net_CoP_z=\(netZ)")
        // 1 cm, versus a 0.13 mm per-marker IK residual on these hand-built
        // poses: two orders of magnitude of slack, and still an order of
        // magnitude tighter than the 11 cm error the frame bug produced.
        XCTAssertEqual(netX, expectedX, accuracy: 0.01,
                       "net CoP must lie under the centre of mass")
        XCTAssertEqual(netZ, 0.0, accuracy: 0.01,
                       "a mirror-symmetric stance puts the net CoP on the sagittal plane")
    }

    private static let legDOFs = [
        "hip_flexion_r", "hip_adduction_r", "hip_rotation_r",
        "knee_angle_r", "ankle_angle_r", "subtalar_angle_r", "mtp_angle_r",
        "hip_flexion_l", "hip_adduction_l", "hip_rotation_l",
        "knee_angle_l", "ankle_angle_l", "subtalar_angle_l", "mtp_angle_l",
    ]

    // ══════════════════════════════════════════════════════════════════════
    // PART 1 — the known-answer benchmark
    // ══════════════════════════════════════════════════════════════════════

    /// Quiet standing with the whole body leaned 4.012° forward about the
    /// ankles, which is the classic inverted-pendulum picture of quiet stance.
    ///
    /// ── HAND DERIVATION ────────────────────────────────────────────────────
    /// Lean angle. At zero lean, the model's centre of mass sits at x =
    /// -0.0746 (relative to the pelvis) and the ankle joint centres at x =
    /// -0.0627, i.e. the CoM is 0.0119 m BEHIND the ankles. CoM height above
    /// the ankle axis is 0.9589 - 0.0750 = 0.8839 m. To place the CoM 0.0500 m
    /// AHEAD of the ankles the body must rotate forward by
    ///     sin(theta) = (0.0500 + 0.0119) / 0.8839 = 0.06999  ->  theta = 4.012°
    /// so `leanRad = -0.070027` (negative = about +z = anterior; see
    /// `standingMarkers`).
    ///
    /// Vertical GRF. Static, so sum(F) = 0:
    ///     total  = m g = 79.5835 * 9.81 = 780.71 N
    ///     each foot = 390.36 N   (mirror-symmetric pose, 50/50 split)
    ///
    /// Centre of pressure. Static, so the net moment about any point is zero;
    /// with a horizontal GRF of zero that forces the net CoP to lie directly
    /// under the CoM:  x_CoP = x_CoM = -0.01264.
    ///
    /// Ankle moment, per foot. Free body = one foot. The GRF acts at that
    /// foot's CoP; the foot's own weight (1.567 kg) acts 0.0009 m ahead of the
    /// ankle, worth 0.02 Nm, and is dropped:
    ///     lever   = x_CoP - x_ankle = -0.01264 - (-0.06269) = +0.05005 m
    ///     M_ankle = 390.36 N * 0.05005 m = 19.54 Nm   (plantarflexor)
    /// `ankle_r` is a PinJoint whose axis is (-0.1050, -0.1740, 0.9791) — the
    /// standard oblique talocrural axis — so the GENERALISED force reported at
    /// `ankle_angle_r` is the medio-lateral moment projected onto it:
    ///     tau(ankle_angle_r) = 0.9791 * 19.54 = 19.1 Nm
    /// The subtalar PinJoint axis is (0.7872, 0.6047, -0.1209), so the same
    /// sagittal moment shows up there as only
    ///     tau(subtalar_angle_r) = 0.1209 * ~19 = ~2.3 Nm
    ///
    /// Knee moment. Free body = shank + patella + foot (5.360 kg, weight
    /// 52.58 N, CoM at x = -0.04715). Knee JC after the lean is at x =
    /// -0.02472:
    ///     M_knee = 390.36 * (-0.01264 + 0.02472) - 52.58 * (-0.04715 + 0.02472)
    ///            = 390.36 * 0.01209  -  52.58 * (-0.00083)
    ///            = 4.72 + 0.04 = +4.76 Nm
    ///
    /// Hip moment. Free body = whole leg (14.662 kg, weight 143.83 N, CoM at
    /// x = -0.08124). Hip JC after the lean is at x = +0.00010:
    ///     M_hip = 390.36 * (-0.01264 - 0.00010) - 143.83 * (-0.08124 - 0.00010)
    ///           = 390.36 * (-0.01274) - 143.83 * (-0.08134)
    ///           = -4.97 + 11.70 = ... using the CoM measured in the leaned
    ///     configuration (x_legCoM - x_hip = -0.01693):
    ///           = 390.36 * (-0.01274) - 143.83 * (-0.01693) = -2.54 Nm
    ///
    /// ── WHAT THIS PREDICTS ─────────────────────────────────────────────────
    ///   |tau(ankle_angle)|    ~ 19 Nm     (NOT hundreds)
    ///   |tau(knee_angle)|     ~  5 Nm
    ///   |tau(hip_flexion)|    ~  3 Nm
    ///   |tau(subtalar_angle)| ~  2 Nm
    ///   left == right to within the IK residual
    ///
    /// Note that in quiet stance the ankle is legitimately the LARGEST leg
    /// moment — "torque decreases distally" is a statement about how much mass
    /// hangs below a joint, and it does not survive as an ordering rule for a
    /// balanced upright pose, where every lever arm is a few centimetres. The
    /// pose-independent statement that DOES hold is the support-polygon bound
    /// asserted below.
    func testQuietStandingAnkleMomentIsTensOfNewtonMetres() throws {
        try loadFullBody()
        let markers = Self.standingMarkers(leanRad: -0.070027)   // -4.012°
        let s = try solve(markers: markers, tag: "lean4deg")
        report(s, tag: "lean4deg")

        // --- GRF ---------------------------------------------------------
        let bw = bridge.totalMass * 9.81
        let fR = s.id.rightFootForce.map { $0.doubleValue }
        let fL = s.id.leftFootForce.map { $0.doubleValue }
        let total = (fR[0]*fR[0] + fR[1]*fR[1] + fR[2]*fR[2]).squareRoot()
                  + (fL[0]*fL[0] + fL[1]*fL[1] + fL[2]*fL[2]).squareRoot()
        XCTAssertTrue(s.id.leftFootInContact && s.id.rightFootInContact,
                      "both feet must be detected in contact for a two-foot stance")
        XCTAssertEqual(total, bw, accuracy: 0.10 * bw,
                       "total GRF must equal bodyweight (780.7 N)")

        // --- the headline number -----------------------------------------
        // Hand-derived 19.1 Nm; 25 Nm of slack absorbs IK residual (1 cm of
        // pose error is worth 390 N * 0.01 m = 3.9 Nm) and the arms/trunk
        // settling slightly differently than the rigid construction.
        for name in ["ankle_angle_r", "ankle_angle_l"] {
            let t = abs(s.torque(name))
            XCTAssertLessThan(t, 45.0,
                "\(name) = \(t) Nm. Hand-derived value for this pose is 19.1 Nm.")
        }

        // --- the pose-independent bound ----------------------------------
        // The centre of pressure cannot leave the foot. The foot spans from
        // the heel (0.049 m behind the ankle JC) to the toe tip (~0.19 m
        // ahead of it), so for a foot carrying at most the whole bodyweight
        //     |M_ankle| <= 780.71 N * 0.19 m = 148 Nm
        // and for a symmetric two-foot stance (390 N per foot) half that.
        // This holds for ANY human pose, not just this one.
        for name in ["ankle_angle_r", "ankle_angle_l"] {
            XCTAssertLessThan(abs(s.torque(name)), 148.0,
                "\(name) = \(s.torque(name)) Nm exceeds the support-polygon bound " +
                "|F| * max foot lever = 780.71 * 0.19 = 148 Nm. No CoP inside a " +
                "human foot can produce this.")
        }
        // Frontal-plane lever at the subtalar joint is at most the foot's
        // half-width, ~0.045 m: |M| <= 390.36 * 0.045 = 17.6 Nm, and the
        // projection onto the oblique subtalar axis makes it smaller still.
        for name in ["subtalar_angle_r", "subtalar_angle_l"] {
            XCTAssertLessThan(abs(s.torque(name)), 30.0,
                "\(name) = \(s.torque(name)) Nm. Hand-derived value ~2.3 Nm; " +
                "the frontal-plane bound is 390 N * 0.045 m = 17.6 Nm.")
        }

        // --- knee / hip ---------------------------------------------------
        for (name, expected) in [("knee_angle_r", 4.76), ("knee_angle_l", 4.76),
                                 ("hip_flexion_r", -2.54), ("hip_flexion_l", -2.54)] {
            let t = s.torque(name)
            XCTAssertLessThan(abs(t), 40.0,
                "\(name) = \(t) Nm, hand-derived \(expected) Nm")
        }

        // --- no leg DOF may be large -------------------------------------
        for name in Self.legDOFs {
            XCTAssertLessThan(abs(s.torque(name)), 150.0,
                "\(name) = \(s.torque(name)) Nm. In a balanced two-foot stance " +
                "no leg moment can exceed the support-polygon bound.")
        }

        // --- symmetry -----------------------------------------------------
        // A mirror-symmetric pose on a mirror-symmetric mass table must give
        // equal torques left and right. OpenSim gait conventions make BOTH
        // sides' flexion/adduction positive in the same anatomical sense, so
        // the comparison is against equality, not negation.
        assertLeftRightSymmetric(s, tag: "lean4deg")
        assertContactForceIsUprightBodyweight(s, tag: "lean4deg")
        // Hand-derived in this test's docstring: x_CoP = x_CoM = -0.01264 m.
        assertNetCoPSitsUnderCoM(s, expectedX: -0.01264, tag: "lean4deg")

        // --- the readback consistency check -------------------------------
        // ‖ΣF_contact + m·g − m·a_com‖, in newtons, with the contact forces
        // taken out of body-local coordinates first. Nimble solves the six
        // floating-base equations exactly, so this is ~0 whenever the frame
        // conversion on the way out is right — and grows to bodyweight scale
        // when it is not.
        XCTAssertLessThan(s.id.rootResidualNorm, 1.0,
                          "linear-momentum residual \(s.id.rootResidualNorm) N: the solved " +
                          "contact forces do not sum with gravity to m·a_com")
    }

    /// The same construction with zero lean: the model's neutral posture, feet
    /// flat, standing upright.
    ///
    /// ── HAND DERIVATION ────────────────────────────────────────────────────
    ///   x_CoM   = -0.07456,  x_ankle = -0.06269  ->  lever = -0.01187 m
    ///   M_ankle = 390.36 * (-0.01187) = -4.63 Nm  (a small DORSIflexor
    ///             moment, because this posture's CoM sits just behind the
    ///             ankles)
    ///   M_knee  = 390.36 * (-0.02187) - 52.58 * (+0.01636) = -9.40 Nm
    ///   M_hip   = 390.36 * (-0.01828) - 143.83 * (+0.00729) = -8.19 Nm
    /// Everything is single-digit Nm. This is the cleanest possible statement
    /// of the expected magnitude scale for standing.
    func testNeutralUprightStandingIsSingleDigitNewtonMetres() throws {
        try loadFullBody()
        let s = try solve(markers: Self.standingMarkers(leanRad: 0.0), tag: "upright")
        report(s, tag: "upright")

        let bw = bridge.totalMass * 9.81
        let fR = s.id.rightFootForce.map { $0.doubleValue }
        let fL = s.id.leftFootForce.map { $0.doubleValue }
        let total = (fR[0]*fR[0] + fR[1]*fR[1] + fR[2]*fR[2]).squareRoot()
                  + (fL[0]*fL[0] + fL[1]*fL[1] + fL[2]*fL[2]).squareRoot()
        XCTAssertEqual(total, bw, accuracy: 0.10 * bw, "total GRF must equal bodyweight")

        for (name, expected) in [("ankle_angle_r", -4.63), ("ankle_angle_l", -4.63),
                                 ("knee_angle_r", -9.40), ("knee_angle_l", -9.40),
                                 ("hip_flexion_r", -8.19), ("hip_flexion_l", -8.19)] {
            XCTAssertLessThan(abs(s.torque(name)), 40.0,
                "\(name) = \(s.torque(name)) Nm, hand-derived \(expected) Nm")
        }
        for name in Self.legDOFs {
            XCTAssertLessThan(abs(s.torque(name)), 150.0,
                "\(name) = \(s.torque(name)) Nm exceeds the support-polygon bound")
        }
        assertLeftRightSymmetric(s, tag: "upright")
        assertContactForceIsUprightBodyweight(s, tag: "upright")
        // Hand-derived in this test's docstring: x_CoM = -0.07456 m.
        assertNetCoPSitsUnderCoM(s, expectedX: -0.07456, tag: "upright")
    }

    /// Left/right torque equality on a mirror-symmetric pose. Also PART 2's
    /// cheapest indexing trap: if the torque vector were permuted relative to
    /// `dofNames`, `hip_flexion_r` and `hip_flexion_l` would almost certainly
    /// stop matching.
    private func assertLeftRightSymmetric(_ s: Solved, tag: String) {
        var worst = 0.0
        var worstName = ""
        for base in ["hip_flexion", "hip_adduction", "hip_rotation",
                     "knee_angle", "ankle_angle", "subtalar_angle", "mtp_angle"] {
            let r = s.torque(base + "_r"), l = s.torque(base + "_l")
            let diff = abs(r - l)
            print("BENCH-METRIC [\(tag)] symmetry \(base): r=\(r) l=\(l) diff=\(diff)")
            if diff > worst { worst = diff; worstName = base }
            // Tolerance: 2 Nm floor for IK residual, or 5% of the larger side.
            XCTAssertLessThan(diff, max(2.0, 0.05 * max(abs(r), abs(l))),
                "\(base): r=\(r) l=\(l) — a mirror-symmetric pose must be symmetric")
        }
        print("BENCH-METRIC [\(tag)] worst_symmetry_break=\(worstName) \(worst) Nm")
    }


    // ══════════════════════════════════════════════════════════════════════
    // PART 1b — the real dancer pose, checked against its OWN solved wrench
    // ══════════════════════════════════════════════════════════════════════

    /// The photo-derived dancer pose from `OfflineMuscleChainFixture`, carried
    /// all the way through the muscle QP.
    ///
    /// Nobody can hand-derive this pose's ankle moment the way Part 1 does —
    /// it is a real Core ML output, single-foot stance, and not necessarily
    /// balanced. So the reference here is not arithmetic on anthropometry, it
    /// is the solver's OWN contact wrench, which is a much harder standard to
    /// game:
    ///
    ///   Free body = everything distal to the right ankle (talus + calcn +
    ///   toes, 1.567 kg, weight 15.4 N). Only two things act on it — the
    ///   contact wrench and its own weight — so
    ///
    ///     tau(ankle) = axis · [ (p_CoP − c_ankle) × F_GRF
    ///                        + (com_foot − c_ankle) × W_foot ]
    ///
    ///   and therefore, for a unit joint axis,
    ///
    ///     |tau(ankle)| ≤ |F_GRF| · |p_CoP − c_ankle| + 15.4 N · 0.1 m
    ///
    /// That bound is an identity of rigid-body statics. It uses no
    /// anthropometric assumption and no tuned constant: both |F_GRF| and
    /// p_CoP are read back out of `result.contactWrenches`, i.e. the wrench
    /// the solver itself produced. A torque that violates it cannot be
    /// generated by the contact force the solver reports, whatever the pose.
    ///
    /// The bug this pins: ID reported 472 Nm at `ankle_angle_r` while its own
    /// contact wrench (780 N, 0.0897 m from the ankle) permits at most ~72.
    func testDancerAnkleTorqueMatchesItsOwnContactWrench() throws {
        let (computer, solver) = try loadFullBodyWithMuscles()
        let markers = OfflineMuscleChainFixture.markers.map {
            ($0.1, SIMD3<Double>(Double($0.2.x), Double($0.2.y), Double($0.2.z)))
        }
        let s = try solve(markers: markers, tag: "dancer")
        report(s, tag: "dancer")

        // --- 4. ankle torque vs the solver's own wrench -------------------
        // Single-foot stance: the right foot is the one on the floor.
        XCTAssertTrue(s.id.rightFootInContact, "the dancer stands on the right foot")
        let f = s.id.rightFootForce.map { $0.doubleValue }
        let cop = s.id.rightFootCoP.map { $0.doubleValue }
        let ankleMarker = OfflineMuscleChainFixture.markers.first { $0.1 == "RAJC" }!.2
        let ankle = SIMD3<Double>(Double(ankleMarker.x), Double(ankleMarker.y), Double(ankleMarker.z))
        let fMag = (f[0]*f[0] + f[1]*f[1] + f[2]*f[2]).squareRoot()
        let r = SIMD3<Double>(cop[0] - ankle.x, cop[1] - ankle.y, cop[2] - ankle.z)
        let lever = (r.x*r.x + r.y*r.y + r.z*r.z).squareRoot()
        // 1.567 kg distal to the ankle, at most 0.1 m from the joint centre.
        let footWeightTerm = 1.567 * 9.81 * 0.1
        let bound = fMag * lever + footWeightTerm
        print("BENCH-METRIC [dancer] grf_N=\(fMag) bodyweight_N=\(bridge.totalMass * 9.81)")
        print("BENCH-METRIC [dancer] cop_to_ankle_m=\(lever) implied_ankle_bound_Nm=\(bound)")
        print("BENCH-METRIC [dancer] ankle_angle_r=\(s.torque("ankle_angle_r"))")

        XCTAssertEqual(fMag, bridge.totalMass * 9.81, accuracy: 0.05 * bridge.totalMass * 9.81,
                       "single-foot stance: the one contact must carry bodyweight")
        XCTAssertLessThan(abs(s.torque("ankle_angle_r")), bound,
            "ankle_angle_r = \(s.torque("ankle_angle_r")) Nm, but the contact wrench the " +
            "solver itself produced (|F| = \(fMag) N at \(lever) m from the ankle joint " +
            "centre) can generate at most \(bound) Nm there. The torque handed to the " +
            "muscle QP is not producible by the force ID says is acting.")

        // Same identity one joint further down: everything distal to the
        // subtalar joint is calcn + toes (1.467 kg) and the contact wrench
        // acts ON calcn, so the same bound applies with a shorter lever.
        XCTAssertLessThan(abs(s.torque("subtalar_angle_r")), bound,
            "subtalar_angle_r = \(s.torque("subtalar_angle_r")) Nm exceeds the same " +
            "contact-wrench bound of \(bound) Nm")

        // --- 2. the distal gradient ---------------------------------------
        // WHAT DECREASES DISTALLY IS THE MOMENT VECTOR, NOT THE GENERALISED
        // TORQUE. For a single distal contact the net moment at joint j is
        //
        //     M_j = (p_CoP − c_j) × F  +  Σ_{i distal to j} (com_i − c_j) × W_i
        //
        // and BOTH terms shrink as j moves towards the foot: the contact lever
        // |p_CoP − c_j| shrinks, and fewer segments remain distal. So |M_j| is
        // monotone. `jointTorques[j]` is that vector PROJECTED onto joint j's
        // axis, and projections onto non-parallel axes inherit no ordering:
        // the talocrural axis is (−0.105, −0.174, 0.979) and the subtalar axis
        // is (0.787, 0.605, −0.121), nearly orthogonal to it, so a single
        // moment vector legitimately reads larger at the subtalar than at the
        // ankle. Measured after the fix: ankle 22.50 Nm, subtalar 41.34 Nm,
        // both under the 58.9 Nm the CoP lever permits — same vector, two
        // projections. Asserting |tau_subtalar| < |tau_ankle| would be
        // asserting a coincidence of axis geometry.
        //
        // The falsifiable statement is therefore the per-joint lever bound.
        // Joint centres come from the input markers (the IK targets), and the
        // ±2.6 cm per-marker residual is carried explicitly rather than hidden
        // in a fudge factor.
        let ikSlackM = (s.ik.error / Double(JointMapping.primary.count)).squareRoot()
        for (dof, markerName, distalKg, distalLeverM) in [
            ("hip_flexion_r",    "RHJC", 14.662, 0.45),   // whole leg
            ("knee_angle_r",     "RKJC",  5.360, 0.30),   // shank + patella + foot
            ("ankle_angle_r",    "RAJC",  1.567, 0.10),   // talus + calcn + toes
            ("subtalar_angle_r", "RAJC",  1.467, 0.10),   // calcn + toes (JC within IK slack of RAJC)
        ] {
            let mk = OfflineMuscleChainFixture.markers.first { $0.1 == markerName }!.2
            let d = SIMD3<Double>(cop[0] - Double(mk.x), cop[1] - Double(mk.y), cop[2] - Double(mk.z))
            let contactLever = (d.x*d.x + d.y*d.y + d.z*d.z).squareRoot() + ikSlackM
            let jointBound = fMag * contactLever + distalKg * 9.81 * distalLeverM
            print("BENCH-METRIC [dancer] lever_bound \(dof) tau=\(s.torque(dof)) " +
                  "contact_lever_m=\(contactLever) bound_Nm=\(jointBound)")
            XCTAssertLessThan(abs(s.torque(dof)), jointBound,
                "\(dof) = \(s.torque(dof)) Nm exceeds \(jointBound) Nm, the largest moment " +
                "the solver's own contact wrench (\(fMag) N at \(contactLever) m) plus the " +
                "\(distalKg) kg distal to it can produce about that joint centre.")
        }

        // --- 5. the muscle QP is what all of this is for -------------------
        let dofNames = s.dofNames
        let momentArms = try XCTUnwrap(
            computer.computeMomentArms(withJointAngles: s.ik.jointAngles, dofNames: dofNames))
        let muscle = try XCTUnwrap(
            solver.solveReal(withJointTorques: s.id.jointTorques,
                             momentArms: momentArms,
                             muscleNames: computer.muscleNames,
                             muscleLengths: computer.currentMuscleLengths,
                             maxForces: computer.maxIsometricForces,
                             optimalFiberLengths: computer.optimalFiberLengths,
                             tendonSlackLengths: computer.tendonSlackLengths,
                             pennationAngles: computer.pennationAngles,
                             jointVelocities: [NSNumber](repeating: 0.0, count: dofNames.count),
                             dofNames: dofNames,
                             dt: 0.5,
                             softPenalty: 100.0))
        let a = muscle.activations.map { $0.doubleValue }.sorted()
        let floor = solver.minActivation
        let atFloor = a.filter { $0 <= floor + 1e-6 }.count
        let saturated = a.filter { $0 >= 1.0 - 1e-6 }.count
        let above008 = a.filter { $0 > 0.08 }.count
        print("BENCH-METRIC [dancer] muscles=\(a.count) floor=\(floor) at_floor=\(atFloor) " +
              "saturated_at_1=\(saturated) above_0.08=\(above008)")
        print("BENCH-METRIC [dancer] activation median=\(a[a.count / 2]) " +
              "p90=\(a[Int(Double(a.count) * 0.9)]) max=\(a.last ?? 0)")
        print("BENCH-METRIC [dancer] torqueResidualNm=\(muscle.torqueResidualNm) " +
              "relativeTorqueResidual=\(muscle.relativeTorqueResidual)")

        // WHICH coordinates are impossible to satisfy? For each DOF, the
        // largest torque the whole musculature could produce there is
        // Σ_m |r_m,dof| · F_max,m (every muscle fully on, all pulling the same
        // way). Anything the QP is asked to match beyond that is unreachable no
        // matter how the activations are distributed, and the soft penalty
        // responds by pinning muscles to 1.0. This separates "ID torque is too
        // big" from "this coordinate has no muscles on it".
        let nDOF = dofNames.count
        let nMus = computer.muscleNames.count
        let fMaxes = computer.maxIsometricForces.map { $0.doubleValue }
        let arms = momentArms.map { $0.doubleValue }
        var capacity = [Double](repeating: 0, count: nDOF)
        for m in 0..<nMus {
            for d in 0..<nDOF { capacity[d] += abs(arms[m * nDOF + d]) * fMaxes[m] }
        }
        var over: [(String, Double, Double)] = []
        for d in 0..<nDOF where abs(s.torques[d]) > capacity[d] {
            over.append((dofNames[d], s.torques[d], capacity[d]))
        }
        over.sort { abs($0.1) - $0.2 > abs($1.1) - $1.2 }
        print("BENCH-METRIC [dancer] dofs_over_muscle_capacity=\(over.count) of \(nDOF)")
        for (n, t, c) in over.prefix(12) {
            print("BENCH-METRIC [dancer] over_capacity \(n) demand=\(t) capacity=\(c)")
        }
        let byDemand = (0..<nDOF).sorted { abs(s.torques[$0]) > abs(s.torques[$1]) }
        for d in byDemand.prefix(10) {
            print("BENCH-METRIC [dancer] top_demand \(dofNames[d]) tau=\(s.torques[d]) " +
                  "capacity=\(capacity[d])")
        }
        let acts = muscle.activations.map { $0.doubleValue }
        let satNames = (0..<acts.count).filter { acts[$0] >= 1.0 - 1e-6 }
            .map { computer.muscleNames[$0] }
        print("BENCH-METRIC [dancer] saturated_names=\(satNames.joined(separator: ","))")

        // THE GATE. Muscles pin to 1.0 when the QP is asked for a torque its
        // musculature cannot produce, so the falsifiable statement is a
        // comparison against that capacity, not against a count of saturated
        // muscles (which also moves with how the soft penalty trades off 169
        // coupled coordinates, and is not something inverse dynamics owns).
        //
        // The moment arms are pure kinematics — the gravity vector does not
        // enter them — so `capacity` here is identical before and after the
        // fix and the comparison is clean. Before: `subtalar_angle_r` demanded
        // 672.7 Nm against a capacity of 166.9 Nm, and `ankle_angle_r` 472.5 Nm.
        // After: 41.3 Nm and 22.5 Nm. The only coordinates still over capacity
        // are the four wrist DOFs, which have NO muscles in FullBody.osim at
        // all (capacity exactly 0) and are asked for less than 0.3 Nm.
        let overWithMuscles = over.filter { $0.2 > 0 }
        XCTAssertTrue(overWithMuscles.isEmpty,
            "these actuated coordinates demand more torque than every muscle crossing them " +
            "could produce at full activation: " +
            overWithMuscles.map { "\($0.0) demand=\($0.1) capacity=\($0.2)" }
                .joined(separator: "; "))
        for (n, _, c) in over {
            XCTAssertEqual(c, 0.0, accuracy: 1e-12,
                "\(n) is over capacity and DOES have muscles crossing it")
        }

        // CONTROL — is the leftover QP residual a leg problem or a trunk one?
        // Re-solve with every non-leg torque zeroed. If the residual collapses,
        // the leg chain (the part inverse dynamics was inflating) is fully
        // satisfiable and what remains lives in the spine/neck, which is a
        // separate defect in the muscle stage, not in ID.
        var legOnly = [NSNumber](repeating: NSNumber(value: 0.0), count: nDOF)
        for d in 0..<nDOF where Self.legDOFs.contains(dofNames[d]) {
            legOnly[d] = s.id.jointTorques[d]
        }
        if let legSolve = solver.solveReal(withJointTorques: legOnly,
                                           momentArms: momentArms,
                                           muscleNames: computer.muscleNames,
                                           muscleLengths: computer.currentMuscleLengths,
                                           maxForces: computer.maxIsometricForces,
                                           optimalFiberLengths: computer.optimalFiberLengths,
                                           tendonSlackLengths: computer.tendonSlackLengths,
                                           pennationAngles: computer.pennationAngles,
                                           jointVelocities: [NSNumber](repeating: 0.0, count: nDOF),
                                           dofNames: dofNames,
                                           dt: 0.5,
                                           softPenalty: 100.0) {
            let la = legSolve.activations.map { $0.doubleValue }
            print("BENCH-METRIC [dancer] legonly residualNm=\(legSolve.torqueResidualNm) " +
                  "relative=\(legSolve.relativeTorqueResidual) " +
                  "saturated=\(la.filter { $0 >= 1.0 - 1e-6 }.count) " +
                  "at_floor=\(la.filter { $0 <= floor + 1e-6 }.count)")
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    // PART 2 — is `jointTorques[i]` really the DOF called `dofNames[i]`?
    // ══════════════════════════════════════════════════════════════════════

    /// PRIMARY ORDERING PROOF — a mass-matrix column.
    ///
    /// With q = 0 and dq = 0, inverse dynamics gives tau = M(q) ddq + G(q).
    /// Feeding a unit acceleration on ONE coordinate and subtracting the
    /// ddq = 0 baseline therefore returns exactly one COLUMN of the joint-space
    /// mass matrix. That column has a structural sparsity pattern that follows
    /// from the tree alone and is completely independent of gravity, of the
    /// contact solver, and of the pose:
    ///
    ///     M[j][k] != 0  <=>  some body is distal to BOTH joint j and joint k.
    ///
    /// So the column for `hip_flexion_r` must be nonzero on exactly
    ///   {the six pelvis root coordinates} U {the right-leg coordinates}
    /// and IDENTICALLY zero on the left leg, both arms and every spine
    /// coordinate — no body hangs below both a right hip and a left knee.
    ///
    /// This is the test to trust. It uses the DOF name to choose the index it
    /// perturbs and the DOF name to interpret the indices that respond, so a
    /// permutation between the two orderings shows up immediately.
    ///
    /// Magnitude anchor: the diagonal entry is the moment of inertia of the
    /// whole right leg about the hip flexion axis. `hip_r` is a CustomJoint
    /// whose parent offset frame has zero orientation and whose rotation1 axis
    /// is (0,0,1), so that axis is the world z line through the femur origin.
    /// Summing I_zz + m*(dx^2 + dy^2) over the leg segments from FullBody.osim:
    ///     femur    0.14120 + 9.3014*(0.0000^2 + 0.1700^2) = 0.41001
    ///     tibia    0.05110 + 3.7075*(0.0036^2 + 0.5928^2) = 1.35385
    ///     kneecap  0.00001 + 0.0862*(0.0461^2 + 0.3924^2) = 0.01347
    ///     talus    0.00100 + 0.1000*(0.0064^2 + 0.8061^2) = 0.06598
    ///     calcn    0.00410 + 1.2500*(0.0448^2 + 0.8180^2) = 0.84305
    ///     toes     0.00100 + 0.2166*(0.1582^2 + 0.8440^2) = 0.16072
    ///                                                    = 2.847 kg m^2
    func testTorqueIndexOrderingMatchesDOFNameOrdering() throws {
        try loadFullBody()
        let n = bridge.numDOFs
        let names = bridge.dofNames
        XCTAssertEqual(names.count, n, "dofNames must be one per DOF")
        print("BENCH-METRIC dof_count=\(n)")
        print("BENCH-METRIC dof_names_first_24=\(names.prefix(24).joined(separator: ","))")

        let zeros = [NSNumber](repeating: NSNumber(value: 0.0), count: n)
        let qz = [NSNumber](repeating: NSNumber(value: 0.0), count: n)
        func torques(ddq: [Double]) -> [Double] {
            bridge.solveID(withJointAngles: qz,
                           jointVelocities: zeros,
                           jointAccelerations: ddq.map { NSNumber(value: $0) })!
                .jointTorques.map { $0.doubleValue }
        }
        func index(_ name: String) throws -> Int {
            try XCTUnwrap(names.firstIndex(of: name), "no DOF named \(name)")
        }

        let root = Set(names.filter { $0.hasPrefix("pelvis_") })
        func legDOFNames(side: String) -> Set<String> {
            Set(names.filter {
                $0.hasSuffix("_" + side) && ($0.hasPrefix("hip_") || $0.hasPrefix("knee_")
                    || $0.hasPrefix("ankle_") || $0.hasPrefix("subtalar_") || $0.hasPrefix("mtp_"))
            })
        }
        let rightLeg = legDOFNames(side: "r"), leftLeg = legDOFNames(side: "l")
        print("BENCH-METRIC right_leg_dofs=\(rightLeg.sorted().joined(separator: ","))")

        let base = torques(ddq: [Double](repeating: 0.0, count: n))

        for probe in ["hip_flexion_r", "hip_flexion_l", "knee_angle_r"] {
            var ddq = [Double](repeating: 0.0, count: n)
            ddq[try index(probe)] = 1.0
            let col = zip(torques(ddq: ddq), base).map { $0 - $1 }

            var responded: [String] = []
            for i in 0..<n where abs(col[i]) > 1e-9 { responded.append(names[i]) }
            print("BENCH-METRIC masscol=\(probe) responded_count=\(responded.count)")
            print("BENCH-METRIC masscol=\(probe) responded=\(responded.sorted().joined(separator: ","))")
            print("BENCH-METRIC masscol=\(probe) diagonal=\(col[try index(probe)])")

            let ownSide = probe.hasSuffix("_r") ? rightLeg : leftLeg
            let otherSide = probe.hasSuffix("_r") ? leftLeg : rightLeg
            let unexpected = Set(responded).subtracting(root.union(ownSide)).sorted()
            XCTAssertTrue(unexpected.isEmpty,
                "accelerating \(probe) produced torque at \(unexpected), which share no " +
                "body with it — jointTorques is NOT aligned with dofNames")
            for name in otherSide.sorted() {
                XCTAssertLessThan(abs(col[try index(name)]), 1e-9,
                    "\(name) is on the opposite leg from \(probe); M must be exactly 0 there")
            }
            for name in ["elbow_flex_r", "elbow_flex_l"] where names.contains(name) {
                XCTAssertLessThan(abs(col[try index(name)]), 1e-9,
                    "\(name) shares no body with \(probe); M must be exactly 0 there")
            }
            XCTAssertGreaterThan(col[try index(probe)], 0.0,
                "the mass-matrix diagonal is positive definite")
        }

        // Magnitude anchor on the diagonal (hand-derived above: 2.847 kg m^2).
        var ddqHip = [Double](repeating: 0.0, count: n)
        ddqHip[try index("hip_flexion_r")] = 1.0
        let diag = torques(ddq: ddqHip)[try index("hip_flexion_r")] - base[try index("hip_flexion_r")]
        print("BENCH-METRIC M[hip_flexion_r][hip_flexion_r]=\(diag) expected=2.847")
        XCTAssertEqual(diag, 2.847, accuracy: 0.15,
            "M[hip_flexion_r][hip_flexion_r] must be the right leg's inertia about the hip axis")

        // Mirror anchor: identical mass tables on the two sides.
        var ddqHipL = [Double](repeating: 0.0, count: n)
        ddqHipL[try index("hip_flexion_l")] = 1.0
        let diagL = torques(ddq: ddqHipL)[try index("hip_flexion_l")] - base[try index("hip_flexion_l")]
        print("BENCH-METRIC M[hip_flexion_l][hip_flexion_l]=\(diagL)")
        XCTAssertEqual(diag, diagL, accuracy: 1e-6, "left and right legs have identical inertia")
    }

    /// SECONDARY ORDERING PROOF — move one coordinate, see which gravity
    /// torques move.
    ///
    /// G_j is the moment of everything DISTAL to joint j about j's axis, so
    /// rotating joint p changes G_j for every j that is PROXIMAL to p (its
    /// distal set contains the segments that moved) plus p's own chain. It
    /// cannot change G_j for a joint on a different limb.
    ///
    /// The probe is `hip_adduction_r`, whose axis is (1,0,0). That choice is
    /// itself a measurement: under the OLD Z-up gravity, `hip_flexion_r` (axis
    /// (0,0,1)) was parallel to gravity, so rotating it changed no
    /// gravitational potential and every delta was exactly zero. The adduction
    /// axis is perpendicular to BOTH the old and the corrected gravity vector,
    /// so this test is valid either way and stays an independent ordering check
    /// rather than a second gravity-direction test.
    func testGravityTorqueResponseStaysOnTheLimbThatMoved() throws {
        try loadFullBody()
        let n = bridge.numDOFs
        let names = bridge.dofNames
        let zeros = [NSNumber](repeating: NSNumber(value: 0.0), count: n)
        func gravity(_ q: [Double]) -> [Double] {
            bridge.solveID(withJointAngles: q.map { NSNumber(value: $0) },
                           jointVelocities: zeros, jointAccelerations: zeros)!
                .jointTorques.map { $0.doubleValue }
        }
        func index(_ name: String) throws -> Int {
            try XCTUnwrap(names.firstIndex(of: name), "no DOF named \(name)")
        }
        let base = gravity([Double](repeating: 0.0, count: n))

        for probe in ["hip_adduction_r", "hip_adduction_l"] {
            var q = [Double](repeating: 0.0, count: n)
            q[try index(probe)] = 0.5
            let after = gravity(q)
            var changed: [String] = []
            for i in 0..<n where abs(after[i] - base[i]) > 1e-6 { changed.append(names[i]) }
            print("BENCH-METRIC perturb=\(probe) changed_count=\(changed.count)")
            print("BENCH-METRIC perturb=\(probe) changed=\(changed.sorted().joined(separator: ","))")

            let side = probe.hasSuffix("_r") ? "_r" : "_l"
            let other = probe.hasSuffix("_r") ? "_l" : "_r"
            // Nothing on the OTHER leg, and no arm coordinate, may respond.
            for name in names where
                (name.hasSuffix(other) &&
                 (name.hasPrefix("hip_") || name.hasPrefix("knee_") || name.hasPrefix("ankle_")
                  || name.hasPrefix("subtalar_") || name.hasPrefix("mtp_")))
                || name.hasPrefix("elbow_flex") || name.hasPrefix("shoulder_") {
                XCTAssertLessThan(abs(after[try index(name)] - base[try index(name)]), 1e-9,
                    "\(name) responded to \(probe); it is on a different limb")
            }
            // The probe's own coordinate must respond — the leg is a real mass.
            let own = after[try index(probe)] - base[try index(probe)]
            print("BENCH-METRIC perturb=\(probe) delta[\(probe)]=\(own)")
            XCTAssertGreaterThan(abs(own), 1.0,
                "rotating \(probe) by 0.5 rad must change its own gravity moment")
            _ = side
        }
    }

    /// Absolute, hand-checkable anchors on the root coordinates, and the
    /// sharpest single statement of the gravity-direction bug: it is one
    /// scalar, it needs no pose, no IK and no contact solver, and it is
    /// bodyweight-or-nothing rather than a magnitude that could be argued.
    ///
    /// `pelvis_tx/ty/tz` translate the whole skeleton along the ground frame's
    /// axes, so for EVERY body d(position)/d(pelvis_ty) = (0,1,0). Therefore
    ///   * M[pelvis_tx][pelvis_tx] is exactly sum(m_i) = 79.5835 kg, and
    ///   * the gravity generalised force at the translation DOF that is
    ///     ANTIPARALLEL TO GRAVITY is exactly sum(m_i)*g = 780.71 N, while the
    ///     other two are zero.
    /// The second one is a direct read-out of the gravity direction: for an
    /// OpenSim (Y-up) model it must land on `pelvis_ty`.
    func testSkeletonGravityPointsDownTheModelsYAxis() throws {
        try loadFullBody()
        let n = bridge.numDOFs
        let names = bridge.dofNames
        let zeros = [NSNumber](repeating: NSNumber(value: 0.0), count: n)
        let qz = [NSNumber](repeating: NSNumber(value: 0.0), count: n)

        let g = try XCTUnwrap(bridge.solveID(withJointAngles: qz,
                                             jointVelocities: zeros,
                                             jointAccelerations: zeros))
        let gt = g.jointTorques.map { $0.doubleValue }
        let iTx = try XCTUnwrap(names.firstIndex(of: "pelvis_tx"))
        let iTy = try XCTUnwrap(names.firstIndex(of: "pelvis_ty"))
        let iTz = try XCTUnwrap(names.firstIndex(of: "pelvis_tz"))
        let mass = bridge.totalMass
        print("BENCH-METRIC root_gravity tx=\(gt[iTx]) ty=\(gt[iTy]) tz=\(gt[iTz]) bodyweight=\(mass * 9.81)")

        // Mass anchor — pose- and gravity-independent, and it pins the name to
        // the index.
        var ddq = [Double](repeating: 0.0, count: n)
        ddq[iTx] = 1.0
        let a = try XCTUnwrap(bridge.solveID(withJointAngles: qz,
                                             jointVelocities: zeros,
                                             jointAccelerations: ddq.map { NSNumber(value: $0) }))
        let mtx = a.jointTorques[iTx].doubleValue - gt[iTx]
        print("BENCH-METRIC mass_matrix M[pelvis_tx][pelvis_tx]=\(mtx) expected=\(mass)")
        XCTAssertEqual(mtx, mass, accuracy: 0.05,
            "M[pelvis_tx][pelvis_tx] must equal the model's total mass")

        // Gravity direction.
        XCTAssertEqual(abs(gt[iTy]), mass * 9.81, accuracy: 0.5,
            "gravity must load pelvis_ty (the model is Y-up). Measured: " +
            "ty=\(gt[iTy]), tz=\(gt[iTz]). DART's Skeleton default gravity is " +
            "(0,0,-9.81) and nothing in OpenSimParser or NimbleBridge calls " +
            "setGravity, so the whole ID stack runs with gravity along the " +
            "subject's medio-lateral axis.")
        XCTAssertEqual(abs(gt[iTz]), 0.0, accuracy: 0.5,
            "no gravity component may load pelvis_tz for a Y-up model")
    }

    /// CONTROLLED EXPERIMENT — the moment arm that gravity orientation buys.
    ///
    /// A straight, vertical leg standing under gravity has almost no gravity
    /// moment at any of its joints: the segment centres of mass sit within a
    /// couple of centimetres of the joint axes, horizontally. From the
    /// FullBody.osim table, the right leg (14.6617 kg, weight 143.83 N) has its
    /// centre of mass at
    ///     d = (com - hip) = (+0.00729, -0.34775, +0.00045) m
    /// so with gravity ANTIPARALLEL to the body's superior axis the moments are
    ///     about the flexion axis (body z):   143.83 * 0.00729 = 1.05 Nm
    ///     about the adduction axis (body x): 143.83 * 0.00045 = 0.07 Nm
    /// — of order 1 Nm. That is a pose fact, and it is true whatever the
    /// skeleton thinks "down" is.
    ///
    /// Turn the same body broadside to gravity and the SAME 34.8 cm vertical
    /// offset becomes the moment arm:
    ///     143.83 * 0.34775 = 50.02 Nm
    /// i.e. a 48x inflation at one joint from orientation alone, with nothing
    /// else changed. Scaled up to the segments that carry the trunk, this is
    /// where the hundreds of Nm in the Part-1 benchmark come from.
    ///
    /// The test measures the skeleton's ACTUAL gravity direction first (from
    /// which root translation coordinate the weight lands on) and then rotates
    /// the root so the body is aligned with it, so it stays valid after the
    /// gravity vector is corrected. No IK, no contact solver, no filtering —
    /// just `solveID` on hand-written coordinate vectors.
    func testAligningTheBodyWithGravityCollapsesTheGravityMoments() throws {
        try loadFullBody()
        let n = bridge.numDOFs
        let names = bridge.dofNames
        let zeros = [NSNumber](repeating: NSNumber(value: 0.0), count: n)
        func gravityTorques(_ q: [Double]) -> [Double] {
            bridge.solveID(withJointAngles: q.map { NSNumber(value: $0) },
                           jointVelocities: zeros, jointAccelerations: zeros)!
                .jointTorques.map { $0.doubleValue }
        }
        func index(_ name: String) throws -> Int {
            try XCTUnwrap(names.firstIndex(of: name), "no DOF named \(name)")
        }

        // Which way is down, as far as this skeleton is concerned?
        let neutral = gravityTorques([Double](repeating: 0.0, count: n))
        let wTx = abs(neutral[try index("pelvis_tx")])
        let wTy = abs(neutral[try index("pelvis_ty")])
        let wTz = abs(neutral[try index("pelvis_tz")])
        print("BENCH-METRIC gravity_probe |tau_tx|=\(wTx) |tau_ty|=\(wTy) |tau_tz|=\(wTz)")

        // `ground_pelvis` rotation2 is `pelvis_list` about (1,0,0), so a +pi/2
        // list maps the body's +y to world +z. Rotation1 (`pelvis_tilt`) is
        // about (0,0,1) and would map +y to -x.
        var alignedQ = [Double](repeating: 0.0, count: n)
        if wTy > wTx && wTy > wTz {
            // Gravity already opposes the model's +y: the neutral pose is
            // already the aligned one.
        } else if wTz > wTx && wTz > wTy {
            alignedQ[try index("pelvis_list")] = Double.pi / 2
        } else {
            XCTFail("gravity loads pelvis_tx; this harness only handles a y- or z-aligned gravity")
            return
        }
        let aligned = gravityTorques(alignedQ)

        var worstAligned = 0.0, worstAlignedName = ""
        for name in Self.legDOFs {
            let t = abs(aligned[try index(name)])
            print("BENCH-METRIC aligned_gravity_torque \(name)=\(aligned[try index(name)])")
            if t > worstAligned { worstAligned = t; worstAlignedName = name }
        }
        for name in Self.legDOFs {
            print("BENCH-METRIC broadside_gravity_torque \(name)=\(neutral[try index(name)])")
        }
        print("BENCH-METRIC worst_aligned_leg_gravity_torque=\(worstAlignedName) \(worstAligned)")

        // Hand-derived maximum for a straight vertical leg is 1.05 Nm.
        XCTAssertLessThan(worstAligned, 5.0,
            "with the body's superior axis antiparallel to the skeleton's own " +
            "gravity vector, the largest leg gravity moment is \(worstAlignedName) " +
            "= \(worstAligned) Nm; a straight vertical leg can only produce ~1 Nm")
        XCTAssertEqual(abs(aligned[try index("hip_flexion_r")]), 1.05, accuracy: 0.6,
            "gravity moment at the hip for a straight vertical leg = 143.83 N * 0.00729 m")
    }
}
