import Foundation
import simd

/// Kinematic posture findings, computed from the RETARGETED MARKER POSITIONS
/// (`BodyFrame.joints`) and from nothing else.
///
/// # Why this layer reads markers and not the solver
/// It is deliberately independent of every open defect downstream. IK is not
/// reliable on hard poses (STATUS.md: the dancer fixture misses its own markers
/// by 3.5 cm RMS and lands on two different solutions from identical input), and
/// the muscle solve carries the redundancy caveats STATUS.md documents (~520
/// unknowns against ~110 torque equations, activation floor chosen so the
/// visualisation would not go "permanently blue"). The marker positions, by
/// contrast, come straight from the pose model and are exactly what
/// `PhotoOverlayView` already draws on the user's own photo — so a number
/// computed here is checkable by eye against the image it came from.
///
/// Nothing in this file may reach into `NimbleEngine.IKOutput`,
/// `IDOutput` or `MuscleOutput`. That is the property that makes it trustworthy.
///
/// # What is deliberately NOT here: normal ranges
/// No finding carries a verdict, a colour, or a "normal / abnormal" line,
/// because no source for one was established for this measurement chain. The
/// numbers are reported with the landmarks they were measured between, and the
/// UI says plainly that no normal range is applied. A number without a verdict
/// is honest; a verdict without a basis is not.
///
/// The one numeric cut in this file is `displayFloor*`, and it is a DISPLAY
/// convention, not an accuracy bound: values under it are grouped as "no
/// measurable deviation" instead of headlining the panel. It is not derived
/// from any published landmark-accuracy figure, because none is established
/// here — STATUS.md's 4.66°/3.46° MHR row is a joint-ANGLE figure from an
/// unreviewed n=9 preprint, and that file explicitly forbids deriving ratios
/// from those rows.
///
/// # View dependence is the biggest correctness risk, so it is explicit
/// Every measurement in this file is a component of a 3-D displacement along
/// one body axis. A single photo observes the image plane well and the depth
/// axis badly. So each finding records `depthFraction` — literally
/// `|measurement axis · camera optical axis|`, which is the derivative of the
/// reported number with respect to a depth error on one contributing landmark —
/// and a finding whose axis is more depth than image plane
/// (`depthFraction > 0.5`, i.e. the subject is turned more than 45° away from
/// the view that measurement needs) is SUPPRESSED with its reason, not
/// reported. 0.5 is the equal-contribution crossover, not a tuned constant. A
/// three-tier "supported / degraded / suppressed" scheme was considered and
/// rejected: the second boundary would have no basis at all.
enum PostureFindings {

    // MARK: - Camera conventions

    /// The camera's optical axis expressed in the coordinate frame of an
    /// OFFLINE (`MHRRetarget`) `BodyFrame`.
    ///
    /// `MHRRetarget` documents and verifies that frame as X = image-right,
    /// Y = up, Z = toward the camera, so the optical axis — the depth
    /// direction, the one a monocular model observes worst — is Z. Only the
    /// axis matters here, not its sign; every use takes an absolute value.
    ///
    /// The live ARKit path is NOT this frame: it is ARKit world space, whose
    /// X/Z depend on session origin, so it must supply its own axis from the
    /// camera transform. Passing `nil` disables view classification and
    /// suppresses every view-dependent finding — which is all of them.
    static let offlineCameraDepthAxis = SIMD3<Double>(0, 0, 1)

    // MARK: - Display conventions (NOT accuracy claims — see the type doc)

    /// Below this a centimetre finding is grouped as "no measurable deviation".
    static let displayFloorCentimetres: Double = 0.5
    /// Below this a degree finding is grouped as "no measurable deviation".
    static let displayFloorDegrees: Double = 1.0
    /// The crossover at which a measurement axis stops being mostly image
    /// plane and starts being mostly depth. 45° of subject yaw.
    static let depthSuppressionFraction: Double = 0.5

    /// The one line the panel must show WITHOUT any tap. It is the honesty
    /// contract of this whole layer, so it is not allowed behind a disclosure.
    static let alwaysVisibleNote =
        "No normal range is applied. These are measurements, not diagnoses — nothing here is a clinical threshold."

    /// The rest of the method disclosure. Verbatim, kept here so a test can pin
    /// them: they are part of the contract too, just not the headline of it.
    static let methodNotes: [String] = [
        "Measured from the pose model's marker positions — the same points drawn on your photo — not from the muscle solver.",
        "Ordered by magnitude. Centimetre and degree findings share one numeric scale for ordering only — that is a display convention, not a claim that 1 cm and 1° are equivalent.",
        "Directions are resolved in a body frame built from the pelvis and the pelvis→C7 trunk axis, so a leaning subject or a tilted camera does not read as an asymmetry.",
    ]

    // MARK: - Public entry point

    /// Compute the report for one frame.
    ///
    /// - Parameters:
    ///   - joints: the frame's tracked joints, ids as in `JointMapping.primary`.
    ///   - cameraDepthAxis: camera optical axis IN THE SAME FRAME as `joints`.
    ///     `nil` means unknown, which suppresses every view-dependent finding.
    static func report(joints: [TrackedJoint],
                       cameraDepthAxis: SIMD3<Double>?) -> PostureReport {
        var p: [String: SIMD3<Double>] = [:]
        for j in joints where j.isTracked {
            p[j.id] = SIMD3<Double>(Double(j.worldPosition.x),
                                    Double(j.worldPosition.y),
                                    Double(j.worldPosition.z))
        }

        var builder = Builder(cameraDepthAxis: cameraDepthAxis.map(normalized))

        // --- Trunk-stable basis ------------------------------------------
        //
        // Same idea as `MuscleOverlay.computeBodyFrame` (trunk axis + pelvis
        // transverse axis, re-orthogonalised), so the findings and the rendered
        // muscles are expressed in one convention. Two deliberate differences:
        //
        //  * the vertical is PELVIS -> C7, not PELVIS -> mid-spine. The longer
        //    baseline (~0.52 m on the model versus ~0.13 m to the lumbar
        //    marker) matters: a 1 cm landmark error tilts a 0.13 m axis by 4.4°
        //    and a 0.52 m axis by 1.1°, and every offset measured against that
        //    axis inherits the tilt times its own lever arm.
        //  * `anterior` here really is anterior. `MuscleOverlay` computes
        //    `forward = pelvisRight x up`, which by the right-hand rule points
        //    POSTERIORLY (`right = anterior x up`, so `right x up = -anterior`).
        //    That is harmless for symmetric capsule offsets but would silently
        //    flip the sign of every sagittal finding here. The sign used below
        //    is pinned by a test against the dancer fixture, whose facing
        //    direction MHRRetarget establishes from evidence outside the model.
        guard let pelvis = p["hips_joint"],
              let c7 = p["spine_7_joint"],
              let lHip = p["left_upLeg_joint"],
              let rHip = p["right_upLeg_joint"] else {
            return PostureReport(findings: [], negligible: [],
                                 suppressed: [SuppressedFinding(
                                    id: "basis",
                                    title: "All posture findings",
                                    reason: "pelvis, C7 or a hip joint is missing from this frame, so no trunk reference frame can be built")],
                                 view: ViewAssessment(orientation: .undetermined,
                                                      anteriorDepthFraction: nil,
                                                      lateralDepthFraction: nil,
                                                      verticalDepthFraction: nil))
        }

        guard let basis = trunkBasis(pelvis: pelvis, c7: c7, leftHip: lHip, rightHip: rHip) else {
            return PostureReport(findings: [], negligible: [],
                                 suppressed: [SuppressedFinding(
                                    id: "basis",
                                    title: "All posture findings",
                                    reason: "the trunk axis and the hip line are degenerate or collinear in this frame, so no trunk reference frame can be built")],
                                 view: ViewAssessment(orientation: .undetermined,
                                                      anteriorDepthFraction: nil,
                                                      lateralDepthFraction: nil,
                                                      verticalDepthFraction: nil))
        }
        let up = basis.up
        let right = basis.right
        let anterior = basis.anterior
        let hipLine = rHip - lHip

        let lSho = p["left_shoulder_1_joint"]
        let rSho = p["right_shoulder_1_joint"]

        // --- View support -------------------------------------------------
        //
        // The depth fraction of an axis is taken as the WORST of the pelvis
        // basis and the shoulder girdle's own axes, because a trunk with real
        // transverse rotation faces two different directions and a view can
        // support one half and not the other. Taking the worse of the two is
        // the conservative reading.
        var anteriorDepth: Double?
        var lateralDepth: Double?
        var verticalDepth: Double?
        if let d = builder.depthAxis {
            var aDepth = abs(simd_dot(anterior, d))
            var lDepth = abs(simd_dot(right, d))
            if let lSho, let rSho,
               let shoulderLine = unit(rSho - lSho),
               let shoulderRight = unit(shoulderLine - simd_dot(shoulderLine, up) * up) {
                let shoulderAnterior = simd_cross(up, shoulderRight)
                aDepth = max(aDepth, abs(simd_dot(shoulderAnterior, d)))
                lDepth = max(lDepth, abs(simd_dot(shoulderRight, d)))
            }
            anteriorDepth = aDepth
            lateralDepth = lDepth
            verticalDepth = abs(simd_dot(up, d))
        }
        builder.anteriorDepth = anteriorDepth
        builder.lateralDepth = lateralDepth
        builder.verticalDepth = verticalDepth

        let midShoulder: SIMD3<Double>? = {
            guard let lSho, let rSho else { return nil }
            return (lSho + rSho) * 0.5
        }()

        // MARK: Sagittal findings (need the A-P axis in the image plane)

        // Forward head: how far the head marker sits AHEAD of the midpoint of
        // the two shoulder joint centres, along the body's anterior axis.
        //
        // NOT the clinical craniovertebral measure. That one uses the tragus /
        // external auditory meatus against C7; MHRRetarget places this marker at
        // the model's `head_neck` origin + 0.15 m along the skull axis, i.e. a
        // head-centre point, not an ear landmark. The number is real; the name
        // it is given in clinic is not claimed.
        builder.add(id: "forward_head",
                    title: "Forward head",
                    value: midShoulder.flatMap { ms in p["head_joint"].map { simd_dot($0 - ms, anterior) * 100 } },
                    unit: .centimetres,
                    axis: .anterior,
                    measuredBetween: "head marker vs midpoint of the two shoulder joint centres, along the body's forward axis",
                    positiveMeans: "ahead of the shoulders",
                    negativeMeans: "behind the shoulders",
                    missingReason: "head or a shoulder joint is missing from this frame")

        // Rounded shoulders: shoulder-girdle protraction — how far the midpoint
        // of the glenohumeral centres sits in front of C7. Referenced to C7 and
        // not to the pelvis on purpose: the pelvis-referenced version is
        // ALGEBRAICALLY IDENTICAL to this one, because `anterior` is built
        // perpendicular to the PELVIS->C7 axis, so (C7 - pelvis) . anterior = 0
        // by construction. The distinct question "does the whole trunk lean
        // forward over the legs" needs an external vertical, and is the
        // trunk-lean finding below.
        builder.add(id: "rounded_shoulders",
                    title: "Shoulder protraction (rounded shoulders)",
                    value: midShoulder.map { ms in simd_dot(ms - c7, anterior) * 100 },
                    unit: .centimetres,
                    axis: .anterior,
                    measuredBetween: "midpoint of the shoulder joint centres vs C7, along the body's forward axis",
                    positiveMeans: "shoulders ahead of C7",
                    negativeMeans: "shoulders behind C7",
                    missingReason: "a shoulder joint is missing from this frame")

        // Thoracic kyphosis PROXY: the whole-chain deflection from the lowest
        // trunk segment (pelvis->lumbar) to the highest (mid-thoracic->C7),
        // measured in the sagittal plane. Positive = the upper segment is
        // tipped further forward than the lower one, the kyphotic direction.
        //
        // The reference axis cancels: this is the angle BETWEEN two segments,
        // so it does not inherit the trunk axis's own tilt.
        let kyphosis: Double? = {
            guard let sL = p["spine_1_joint"], let sM = p["spine_4_joint"] else { return nil }
            let lower = sL - pelvis
            let upper = c7 - sM
            guard simd_length(lower) > 1e-6, simd_length(upper) > 1e-6 else { return nil }
            let tiltLower = atan2(simd_dot(lower, anterior), simd_dot(lower, up))
            let tiltUpper = atan2(simd_dot(upper, anterior), simd_dot(upper, up))
            return wrapDegrees((tiltUpper - tiltLower) * 180 / .pi)
        }()
        builder.add(id: "kyphosis_proxy",
                    title: "Upper-back curve (whole-chain proxy)",
                    value: kyphosis,
                    unit: .degrees,
                    axis: .anterior,
                    measuredBetween: "pelvis→lower-spine segment vs mid-spine→C7 segment, in the side-on plane",
                    positiveMeans: "upper back tipped further forward than the lower back",
                    negativeMeans: "upper back tipped further back than the lower back",
                    missingReason: "a spine marker is missing from this frame",
                    caveat: "Whole-chain proxy over four markers — NOT a per-vertebra measurement. STATUS.md records that the solved per-intervertebral spine angles are priors, not measurements: at 8 mm marker noise no marker set beat the null model. The mid-spine marker is itself an interpolation between two levels 18 cm apart.")

        // MARK: Coronal findings (need the L/R axis in the image plane)

        // Shoulder height asymmetry, measured ALONG THE TRUNK AXIS rather than
        // along world vertical, so a leaning subject or a tilted camera does not
        // read as an asymmetry. Sign: positive = left higher.
        let shoulderHeight: Double? = {
            guard let lSho, let rSho else { return nil }
            return simd_dot(lSho - rSho, up) * 100
        }()
        builder.add(id: "shoulder_height",
                    title: "Shoulder height asymmetry",
                    value: shoulderHeight,
                    unit: .centimetres,
                    axis: .verticalAndLateral,
                    measuredBetween: "left vs right shoulder joint centre, along the trunk axis",
                    positiveMeans: "left shoulder higher",
                    negativeMeans: "right shoulder higher",
                    missingReason: "a shoulder joint is missing from this frame",
                    sideForPositive: .left)

        // Lateral head tilt: the head-on-neck segment's angle away from the
        // trunk axis, in the coronal (front-facing) plane. Positive = tipped
        // toward the subject's right.
        let headTilt: Double? = {
            guard let neck = p["neck_1_joint"], let head = p["head_joint"] else { return nil }
            let v = head - neck
            guard simd_length(v) > 1e-6 else { return nil }
            return atan2(simd_dot(v, right), simd_dot(v, up)) * 180 / .pi
        }()
        builder.add(id: "head_tilt",
                    title: "Lateral head tilt",
                    value: headTilt,
                    unit: .degrees,
                    axis: .verticalAndLateral,
                    measuredBetween: "neck→head segment vs the trunk axis, in the front-facing plane",
                    positiveMeans: "head tipped toward the subject's right",
                    negativeMeans: "head tipped toward the subject's left",
                    missingReason: "the neck or head marker is missing from this frame",
                    sideForPositive: .right)

        // MARK: Findings that additionally need a standing pose

        // Trunk lean / plumb line. This is the one measurement that needs a
        // reference OUTSIDE the trunk chain: referenced to the trunk's own axis
        // it is identically zero by construction. The leg axis
        // (ankle midpoint -> pelvis) is used as that external vertical, which
        // keeps the whole layer independent of gravity and of camera roll — but
        // it only means anything when the subject is actually standing over
        // their feet, so it is gated on the leg axis sitting within 45° of the
        // trunk axis.
        let ankleMid: SIMD3<Double>? = {
            guard let la = p["left_foot_joint"], let ra = p["right_foot_joint"] else { return nil }
            return (la + ra) * 0.5
        }()
        let stance: SIMD3<Double>? = ankleMid.flatMap { unit(pelvis - $0) }
        let standingAligned: Bool = stance.map { simd_dot($0, up) > 0.7071 } ?? false
        let stanceReason: String = {
            if ankleMid == nil { return "an ankle marker is missing from this frame" }
            if stance == nil { return "the pelvis and the ankle midpoint coincide in this frame" }
            return "the leg axis is more than 45° away from the trunk axis — the subject is not standing over their feet, so the legs are not a plumb reference"
        }()

        if let stance, let ankleMid, standingAligned, let ms = midShoulder {
            // Components of the shoulder-to-pelvis offset that are NOT along
            // the leg axis, resolved onto the body's forward and sideways axes
            // (each first made perpendicular to the leg axis, so the two
            // components are an orthogonal decomposition of the offset).
            let offset = ms - pelvis
            let perp = offset - simd_dot(offset, stance) * stance
            let aStance = unit(anterior - simd_dot(anterior, stance) * stance)
            let rStance = unit(right - simd_dot(right, stance) * stance)

            builder.add(id: "trunk_lean_sagittal",
                        title: "Trunk lean (forward/back)",
                        value: aStance.map { simd_dot(perp, $0) * 100 },
                        unit: .centimetres,
                        axis: .anterior,
                        measuredBetween: "shoulder midpoint vs pelvis, across the leg axis, along the body's forward axis",
                        positiveMeans: "shoulders ahead of the pelvis",
                        negativeMeans: "shoulders behind the pelvis",
                        missingReason: "the forward axis is degenerate against the leg axis in this frame")

            builder.add(id: "trunk_lean_lateral",
                        title: "Trunk lean (sideways)",
                        value: rStance.map { simd_dot(perp, $0) * 100 },
                        unit: .centimetres,
                        axis: .lateral,
                        measuredBetween: "shoulder midpoint vs pelvis, across the leg axis, along the body's sideways axis",
                        positiveMeans: "shoulders shifted to the subject's right",
                        negativeMeans: "shoulders shifted to the subject's left",
                        missingReason: "the sideways axis is degenerate against the leg axis in this frame",
                        sideForPositive: .right)

            builder.add(id: "weight_shift",
                        title: "Lateral weight shift",
                        value: simd_dot(pelvis - ankleMid, right) * 100,
                        unit: .centimetres,
                        axis: .lateral,
                        measuredBetween: "pelvis vs the midpoint of the two ankles, along the body's sideways axis",
                        positiveMeans: "pelvis over the subject's right foot",
                        negativeMeans: "pelvis over the subject's left foot",
                        missingReason: "an ankle marker is missing from this frame",
                        sideForPositive: .right)
        } else {
            for (id, title) in [("trunk_lean_sagittal", "Trunk lean (forward/back)"),
                                ("trunk_lean_lateral", "Trunk lean (sideways)"),
                                ("weight_shift", "Lateral weight shift")] {
                builder.suppress(id: id, title: title, reason: stanceReason)
            }
        }

        // MARK: Transverse rotation
        //
        // The sensitive direction here is NOT the shoulder line — it is the
        // in-transverse-plane normal to it. Rotating a line about the vertical
        // is most affected by moving one end PERPENDICULAR to the line, and
        // `d(angle)/d(displacement along n̂) = 1/|line|` exactly. So the axis to
        // gate on is n̂, and `|n̂ · depth axis|` is the same projection
        // derivative every other finding here uses.
        //
        // This corrects an earlier version of this file that hardcoded the
        // depth fraction at 1.0 on the reasoning "rotation about vertical is
        // always a depth measurement". That is false, and the geometry says
        // why: in a FRONT-on view the shoulder line lies in the image plane and
        // its normal lies along depth (fraction 1 — unmeasurable); in a SIDE-on
        // view the shoulder line lies along depth and its normal lies in the
        // image plane (fraction 0 — a twist shows up as the shoulders
        // separating horizontally in the photo, which is directly visible).
        // Transverse rotation therefore needs a side view, like the other
        // sagittal findings, rather than being unmeasurable everywhere.
        let shoulderLine: SIMD3<Double>? = {
            guard let lSho, let rSho else { return nil }
            return rSho - lSho
        }()
        if let shoulderLine, let d = builder.depthAxis {
            let sp = shoulderLine - simd_dot(shoulderLine, up) * up
            let hipPerp = hipLine - simd_dot(hipLine, up) * up
            let spLen = simd_length(sp)
            if spLen > 1e-6, simd_length(hipPerp) > 1e-6,
               let sNormal = unit(simd_cross(up, sp)),
               let hNormal = unit(simd_cross(up, hipPerp)) {
                let shoulderYaw = atan2(simd_dot(sp, anterior), simd_dot(sp, right))
                let hipYaw = atan2(simd_dot(hipPerp, anterior), simd_dot(hipPerp, right))
                let rotation = wrapDegrees((shoulderYaw - hipYaw) * 180 / .pi)
                // Worse of the two lines, as elsewhere: both ends of the
                // comparison have to be observable for the difference to be.
                let fraction = max(abs(simd_dot(sNormal, d)), abs(simd_dot(hNormal, d)))
                // 1/|line|, in degrees per centimetre — the irreducible
                // noise gain of the measurement, view-independent.
                let gain = (180 / .pi) * 0.01 / spLen

                builder.add(id: "transverse_rotation",
                            title: "Trunk rotation (shoulders vs hips)",
                            value: rotation,
                            unit: .degrees,
                            axis: .custom(fraction: fraction, needs: "a side-on (sagittal) view"),
                            measuredBetween: "shoulder line vs hip line, viewed down the trunk axis",
                            positiveMeans: "subject's right shoulder rotated forward",
                            negativeMeans: "subject's left shoulder rotated forward",
                            missingReason: "a shoulder joint is missing from this frame",
                            caveat: String(format: "Measured across a %.0f cm shoulder baseline: 1 cm of landmark error perpendicular to the shoulder line moves this by %.1f°.", spLen * 100, gain),
                            sideForPositive: .right)
            } else {
                builder.suppress(id: "transverse_rotation",
                                 title: "Trunk rotation (shoulders vs hips)",
                                 reason: "the shoulder or hip line collapses onto the trunk axis in this frame")
            }
        } else {
            let reason = shoulderLine == nil
                ? "a shoulder joint is missing from this frame"
                : "the camera direction for this frame is unknown, so no view-dependent measurement can be qualified"
            builder.suppress(id: "transverse_rotation",
                             title: "Trunk rotation (shoulders vs hips)",
                             reason: reason)
        }

        return builder.finish(orientation: orientation(anteriorDepth: anteriorDepth,
                                                       lateralDepth: lateralDepth))
    }

    // MARK: - Trunk-stable basis

    /// The orthonormal body frame every measurement in this file is resolved in.
    struct TrunkBasis: Equatable {
        /// Trunk axis, PELVIS → C7.
        let up: SIMD3<Double>
        /// The subject's own right, from the hip line, orthogonalised against `up`.
        let right: SIMD3<Double>
        /// The direction the subject faces. `up × right`.
        let anterior: SIMD3<Double>
    }

    /// Internal so a test can pin the ANTERIOR SIGN against a pose whose facing
    /// direction is established by evidence OUTSIDE the pose model (see
    /// `PostureFindingsTests`). A silent sign flip here would invert every
    /// sagittal finding while every magnitude still looked plausible, which is
    /// exactly the failure a magnitude-only test would miss.
    static func trunkBasis(pelvis: SIMD3<Double>,
                           c7: SIMD3<Double>,
                           leftHip: SIMD3<Double>,
                           rightHip: SIMD3<Double>) -> TrunkBasis? {
        guard let up = unit(c7 - pelvis),
              let hipLine = unit(rightHip - leftHip),
              let right = unit(hipLine - simd_dot(hipLine, up) * up) else { return nil }
        // right = anterior × up  ⇒  anterior = up × right.
        return TrunkBasis(up: up, right: right, anterior: simd_cross(up, right))
    }

    /// Convenience for callers holding a joint list.
    static func trunkBasis(joints: [TrackedJoint]) -> TrunkBasis? {
        var p: [String: SIMD3<Double>] = [:]
        for j in joints where j.isTracked {
            p[j.id] = SIMD3<Double>(Double(j.worldPosition.x),
                                    Double(j.worldPosition.y),
                                    Double(j.worldPosition.z))
        }
        guard let pelvis = p["hips_joint"], let c7 = p["spine_7_joint"],
              let l = p["left_upLeg_joint"], let r = p["right_upLeg_joint"] else { return nil }
        return trunkBasis(pelvis: pelvis, c7: c7, leftHip: l, rightHip: r)
    }

    // MARK: - View classification

    private static func orientation(anteriorDepth: Double?, lateralDepth: Double?) -> ViewAssessment.Orientation {
        guard let a = anteriorDepth, let l = lateralDepth else { return .undetermined }
        if a <= depthSuppressionFraction && a < l { return .sagittal }
        if l <= depthSuppressionFraction && l < a { return .frontal }
        return .oblique
    }

    // MARK: - Assembly

    /// Which body axis a finding is measured along. The gate is on that axis's
    /// depth fraction, because that fraction IS the derivative of the reported
    /// number with respect to a depth error on a contributing landmark.
    enum MeasurementAxis {
        /// Fore-aft. Needs a side-on view.
        case anterior
        /// Left-right. Needs a front-on or back-on view.
        case lateral
        /// Along the trunk axis, but with a left/right attribution that needs
        /// the two sides resolved in the image. Two different conditions —
        /// the vertical term is the projection derivative, the lateral term is
        /// the identifiability of which shoulder is which — so both must hold
        /// and the gate takes the worse.
        case verticalAndLateral
        /// A measurement whose sensitive direction is geometry-dependent and
        /// computed at the call site.
        case custom(fraction: Double, needs: String)
    }

    private struct Builder {
        let depthAxis: SIMD3<Double>?
        var anteriorDepth: Double?
        var lateralDepth: Double?
        var verticalDepth: Double?
        var findings: [PostureFinding] = []
        var negligible: [PostureFinding] = []
        var suppressed: [SuppressedFinding] = []

        init(cameraDepthAxis: SIMD3<Double>?) { self.depthAxis = cameraDepthAxis }

        func depthFraction(for axis: MeasurementAxis) -> Double? {
            switch axis {
            case .anterior: return anteriorDepth
            case .lateral: return lateralDepth
            case .verticalAndLateral:
                guard let v = verticalDepth, let l = lateralDepth else { return nil }
                return max(v, l)
            case .custom(let fraction, _): return fraction
            }
        }

        mutating func suppress(id: String, title: String, reason: String) {
            suppressed.append(SuppressedFinding(id: id, title: title, reason: reason))
        }

        mutating func addUngated(_ finding: PostureFinding) {
            let floor = finding.unit == .centimetres
                ? PostureFindings.displayFloorCentimetres
                : PostureFindings.displayFloorDegrees
            if finding.magnitude < floor { negligible.append(finding) } else { findings.append(finding) }
        }

        mutating func add(id: String,
                          title: String,
                          value: Double?,
                          unit: PostureUnit,
                          axis: MeasurementAxis,
                          measuredBetween: String,
                          positiveMeans: String,
                          negativeMeans: String,
                          missingReason: String,
                          caveat: String? = nil,
                          sideForPositive: PostureSide? = nil) {
            guard let value, value.isFinite else {
                suppress(id: id, title: title, reason: missingReason)
                return
            }
            guard let fraction = depthFraction(for: axis) else {
                suppress(id: id, title: title,
                         reason: "the camera direction for this frame is unknown, so no view-dependent measurement can be qualified")
                return
            }
            guard fraction <= PostureFindings.depthSuppressionFraction else {
                suppress(id: id, title: title, reason: Self.viewReason(axis: axis, fraction: fraction))
                return
            }
            let side: PostureSide? = sideForPositive.map { value >= 0 ? $0 : $0.opposite }
            addUngated(PostureFinding(id: id,
                                      title: title,
                                      value: value,
                                      unit: unit,
                                      side: side,
                                      sideMeaning: value >= 0 ? positiveMeans : negativeMeans,
                                      measuredBetween: measuredBetween,
                                      depthFraction: fraction,
                                      caveat: caveat))
        }

        private static func viewReason(axis: MeasurementAxis, fraction: Double) -> String {
            let needed: String
            switch axis {
            case .anterior: needed = "a side-on (sagittal) view"
            case .lateral, .verticalAndLateral: needed = "a front-on or back-on view"
            case .custom(_, let needs): needed = needs
            }
            return String(format: "needs %@ — in this frame %.0f%% of this measurement's axis lies along the camera's depth direction, which a single photo observes worst",
                          needed, fraction * 100)
        }

        func finish(orientation: ViewAssessment.Orientation) -> PostureReport {
            PostureReport(findings: findings.sorted { $0.magnitude > $1.magnitude },
                          negligible: negligible.sorted { $0.magnitude > $1.magnitude },
                          suppressed: suppressed,
                          view: ViewAssessment(orientation: orientation,
                                               anteriorDepthFraction: anteriorDepth,
                                               lateralDepthFraction: lateralDepth,
                                               verticalDepthFraction: verticalDepth))
        }
    }

    // MARK: - Small helpers

    private static func unit(_ v: SIMD3<Double>) -> SIMD3<Double>? {
        let n = simd_length(v)
        guard n > 1e-6, n.isFinite else { return nil }
        return v / n
    }

    private static func normalized(_ v: SIMD3<Double>) -> SIMD3<Double> {
        unit(v) ?? SIMD3<Double>(0, 0, 1)
    }

    /// Wrap to (-180, 180]. Both angular findings are differences of two
    /// `atan2` results and can otherwise come back as e.g. 350° for a -10° turn.
    static func wrapDegrees(_ d: Double) -> Double {
        var x = d.truncatingRemainder(dividingBy: 360)
        if x > 180 { x -= 360 }
        if x <= -180 { x += 360 }
        return x
    }
}

// MARK: - Types

enum PostureUnit: Equatable {
    case centimetres
    case degrees
}

enum PostureSide: String, Equatable {
    case left
    case right

    var opposite: PostureSide { self == .left ? .right : .left }
}

/// One measurement. Always carries the number and the two landmarks it was
/// measured between — never a bare verdict.
struct PostureFinding: Identifiable, Equatable {
    let id: String
    let title: String
    /// Signed, in `unit`. `sideMeaning` is the human-readable resolution.
    let value: Double
    let unit: PostureUnit
    let side: PostureSide?
    let sideMeaning: String
    let measuredBetween: String
    /// `|measurement axis · camera optical axis|`: the derivative of `value`
    /// with respect to a depth error on one contributing landmark. 0 = the
    /// measurement lies wholly in the image plane.
    let depthFraction: Double
    let caveat: String?

    var magnitude: Double { abs(value) }

    var formattedValue: String {
        switch unit {
        case .centimetres: return String(format: "%.1f cm", magnitude)
        case .degrees: return String(format: "%.1f°", magnitude)
        }
    }

    /// "4.2 cm ahead of the shoulders" — the number and its direction in one
    /// readable phrase.
    var headline: String { "\(formattedValue) \(sideMeaning)" }

    /// How exposed this number is to the axis a single photo sees worst.
    ///
    /// Worded as a property of the AXIS, not of the value. "N% of this number
    /// comes from depth" would be false: how much of the value came from depth
    /// depends on the displacement being measured, not only on its axis. What
    /// `depthFraction` actually is, exactly, is `∂value/∂(depth error on a
    /// contributing landmark)`.
    var projectionNote: String {
        String(format: "%.0f%% of this measurement's axis lies along the camera's depth direction — the one a single photo sees worst",
               depthFraction * 100)
    }
}

struct SuppressedFinding: Identifiable, Equatable {
    let id: String
    let title: String
    let reason: String
}

struct ViewAssessment: Equatable {
    enum Orientation: Equatable {
        case sagittal
        case frontal
        case oblique
        case undetermined
    }

    let orientation: Orientation
    /// `nil` when the camera direction was not supplied.
    let anteriorDepthFraction: Double?
    let lateralDepthFraction: Double?
    let verticalDepthFraction: Double?

    var label: String {
        switch orientation {
        case .sagittal: return "Side-on view"
        case .frontal: return "Front/back view"
        case .oblique: return "Angled view"
        case .undetermined: return "View unknown"
        }
    }

    /// States what this view can and cannot support, with the numbers behind it.
    var summary: String {
        guard let a = anteriorDepthFraction, let l = lateralDepthFraction else {
            return "The camera direction for this frame is unknown, so no measurement can be qualified against the view."
        }
        let numbers = String(format: "forward/back axis %.0f%% depth, left/right axis %.0f%% depth", a * 100, l * 100)
        switch orientation {
        case .sagittal:
            return "Fore-aft findings (forward head, rounded shoulders, trunk lean) are supported. Left/right findings are not — the two sides overlap in depth; retake facing the camera for those. (\(numbers))"
        case .frontal:
            return "Left/right findings (shoulder height, head tilt, weight shift) are supported. Fore-aft findings are not — they would be measured along the camera's depth axis; retake side-on for those. (\(numbers))"
        case .oblique:
            return "The subject is turned roughly halfway between side-on and front-on, so neither the fore-aft nor the left/right axis lies in the image plane and findings needing either are withheld. Retake either straight-on or side-on. (\(numbers))"
        case .undetermined:
            return "No trunk reference frame could be built for this frame. (\(numbers))"
        }
    }
}

struct PostureReport: Equatable {
    /// Above the display floor, ranked by magnitude, biggest first.
    let findings: [PostureFinding]
    /// Computed and supported by the view, but too small to headline.
    let negligible: [PostureFinding]
    /// Not reported, each with the reason.
    let suppressed: [SuppressedFinding]
    let view: ViewAssessment

    var hasAnything: Bool {
        !findings.isEmpty || !negligible.isEmpty || !suppressed.isEmpty
    }
}
