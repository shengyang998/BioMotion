import UIKit
import RealityKit
import simd

/// **The anatomy layer of the 3-D view: WHERE these muscles sit on the pose.**
/// It carries no effort reading, and it cannot be handed one — `update(joints:)`
/// takes no muscle solve.
///
/// That separation is also why this layer remains available while both bundled
/// models report `.contactSupportUnavailable`. Empty contact geometry prevents
/// torque/GRF/CoP/muscle-effort output; it does not prevent a fixed-colour
/// anatomical drawing derived from tracked joint positions alone.
///
/// # What this used to draw, and why it is gone (2026-08-08)
///
/// It filtered `rawActivations`, sorted them descending, kept the strongest 24
/// and coloured every capsule — both render passes — from one shared
/// blue→red colormap. That is a CROSS-MUSCLE ORDERING: it says this muscle is
/// working harder than that one. The model cannot support that statement. When
/// this was written, 66 of `FullBody.osim`'s muscles were given a straight-line
/// path where the real tendon wraps around bone (42 of the `Rajagopal2016`
/// fallback's), so the moment arm that divides each joint moment was wrong by a
/// factor of its own and the activation it returned was inflated by 1/k for an
/// unknown, POSE-DEPENDENT k.
///
/// **That count is 0 now** — cylinder wrapping shipped on 2026-08-08 and
/// ellipsoid wrapping the same day, so all 76 `PathWrap` references in
/// `FullBody.osim` and all 46 in `Rajagopal2016.osim` are solved. (This
/// paragraph said "down to 10 elbow muscles" for a day after the ellipsoid
/// commit took it to none, which is the same staleness the note below carried.)
/// The ordering is still retired, because retiring it took arguments that
/// wrapping does not touch: nothing establishes that two DIFFERENT muscles'
/// activations share a scale, and the QP redistributes load between synergists.
/// Even one muscle's number against ITSELF on the other side is not resolved to
/// the size the panel would print: the moment-arm leak is median 0.977 pp and
/// worst 123.10 pp (`WrappedMomentArmLeakTests`). The QP's own termination slack
/// was a third reason for one commit, 14.88 pp at fixed geometry, and reads
/// 4.4994e-05 pp since `scaling = 0` and `polishing = 1` on 2026-08-09.
///
/// The panel retired exactly this ordering on 2026-08-08
/// (`GaitLoadSummary.perMuscleLeftRightClaimIsSupported`,
/// `MomentArmErrorCancellationTests`), and this renderer went on making it in
/// colour on the more authoritative surface — a picture, with no number, no
/// caption and no floor, above the paragraph that refuses it.
///
/// So the colour is now a CONSTANT and the drawn set is FIXED. Both properties
/// are load-bearing:
///
/// * one colour for every capsule ⇒ no muscle is drawn against another;
/// * a fixed anatomical set ⇒ nothing is SELECTED by a magnitude either. The
///   old top-24 was a selection by the same corrupted number, so the capsules
///   that appeared at all were an ordering even before they were coloured.
///
/// `Self.anatomyOnlyNote` is the sentence the screens carry beside it. Both
/// surfaces (live `SkeletonARView`, offline `OfflineSceneView`) read it from
/// here so they cannot drift apart.
final class MuscleOverlay {

    /// Definition of a visual muscle representation.
    struct MuscleDef {
        let name: String           // Matches MuscleSolver muscle name
        let startJoint: String     // ARKit joint name for origin
        let endJoint: String       // ARKit joint name for insertion
        let offsetStart: SIMD3<Float>  // Offset from start joint (meters)
        let offsetEnd: SIMD3<Float>    // Offset from end joint (meters)
        let radius: Float          // Visual thickness (meters)
        let side: Side

        enum Side { case left, right, center }
    }

    /// One capsule to draw, in world space.
    ///
    /// **It carries no activation and no colour.** The drawn set is a function
    /// of the pose alone — see `capsulePlan(joints:)`, whose signature is the
    /// guarantee. A colour field here would be the seam through which a
    /// per-muscle magnitude could come back.
    struct Capsule: Equatable {
        let name: String
        let start: SIMD3<Float>
        let end: SIMD3<Float>
        let radius: Float
    }

    private var muscleEntities: [String: ModelEntity] = [:]
    private var anchor: AnchorEntity?

    /// The one colour every capsule is drawn in. A muted clay tone, deliberately
    /// OFF the retired blue→cyan→green→yellow→red ramp so a user who saw the old
    /// display cannot read a heat value into a uniform field.
    static let capsuleColor = UIColor(red: 0.72, green: 0.51, blue: 0.47, alpha: 0.55)

    /// The colour for a capsule. Constant by construction: this is the seam a
    /// re-introduced cross-muscle colormap would have to come through, and
    /// `MuscleOverlayClaimTests` asserts the whole plan maps to ONE colour.
    static func color(for capsule: Capsule) -> UIColor { capsuleColor }

    /// What the screens say beside the capsules, so the picture is not left to
    /// speak for itself. Shared by the live and offline surfaces.
    ///
    /// It states the absence first ("effort is not shown"), then what the
    /// capsules ARE, then the mechanism — in that order, because the first
    /// clause is the one a user scanning the screen needs.
    ///
    /// ⚠️ **Twice now it has named a defect that was fixed under it.** Until
    /// 2026-08-09 the mechanism clause read "it gives many of its muscles a
    /// straight line where the real tendon wraps around bone" — false since the
    /// ellipsoid commit (`MomentArmComputer`: 76 solved / 0 unmodelled). It was
    /// rewritten to "that sharing stops as soon as it is close enough rather than
    /// at the exact answer", and the solver fix in the next commit took that gap
    /// from a median of 14.88 pp to 4.4994e-05 pp.
    ///
    /// The reason that survived BOTH rounds is the one that retired the ordering
    /// in the first place, it is structural rather than numerical, and it is now
    /// the only thing this string says: **nothing in this model puts two different
    /// muscles' efforts on a common scale.** The sharing step splits a joint
    /// moment between synergists using each muscle's own leverage and its own
    /// maximum force, so the resulting fraction is per-muscle by construction —
    /// no amount of solver accuracy makes two of them comparable.
    ///
    /// No digit appears in it, and `MuscleOverlayClaimTests` asserts that: a
    /// number here would be the magnitude claim the sentence exists to deny. That
    /// is also why the surviving reason is the right one to carry — a mechanism
    /// that cannot be repaired cannot go stale.
    static let anatomyOnlyNote =
        "Muscle effort is not shown. These capsules mark WHERE the muscles are on your pose — "
        + "their colour is fixed and means nothing. The model reaches a muscle's effort by "
        + "dividing a joint moment by a moment arm and then sharing that moment among the "
        + "muscles that cross the joint, using each muscle's own leverage and its own maximum "
        + "force. Nothing in it puts two different muscles' efforts on one scale, so each "
        + "number is on a scale of its own and cannot be read against another muscle's."

    // Pre-defined muscle visual positions (approximate anatomical placement)
    static let muscleDefs: [MuscleDef] = {
        var defs: [MuscleDef] = []

        // Helper to create bilateral pair
        func bilateral(_ name: String, startJoint: String, endJoint: String,
                       offsetStart: SIMD3<Float> = .zero, offsetEnd: SIMD3<Float> = .zero,
                       radius: Float = 0.018) {
            let rStart = startJoint.replacingOccurrences(of: "left", with: "right")
            let rEnd = endJoint.replacingOccurrences(of: "left", with: "right")
            defs.append(MuscleDef(name: name + "_r", startJoint: rStart, endJoint: rEnd,
                                  offsetStart: SIMD3(offsetStart.x, offsetStart.y, offsetStart.z),
                                  offsetEnd: SIMD3(offsetEnd.x, offsetEnd.y, offsetEnd.z),
                                  radius: radius, side: .right))
            defs.append(MuscleDef(name: name + "_l", startJoint: startJoint, endJoint: endJoint,
                                  offsetStart: SIMD3(-offsetStart.x, offsetStart.y, offsetStart.z),
                                  offsetEnd: SIMD3(-offsetEnd.x, offsetEnd.y, offsetEnd.z),
                                  radius: radius, side: .left))
        }

        // === Thigh — Anterior ===

        // Rectus femoris (hip → knee, front of thigh)
        bilateral("recfem",
                  startJoint: "left_upLeg_joint", endJoint: "left_leg_joint",
                  offsetStart: SIMD3(0, 0, 0.04), offsetEnd: SIMD3(0, 0.03, 0.03),
                  radius: 0.022)

        // Vastus medialis (mid-thigh → knee, inner front)
        bilateral("vasmed",
                  startJoint: "left_upLeg_joint", endJoint: "left_leg_joint",
                  offsetStart: SIMD3(-0.02, -0.08, 0.02), offsetEnd: SIMD3(-0.01, 0.02, 0.02),
                  radius: 0.02)

        // Vastus lateralis (mid-thigh → knee, outer front)
        bilateral("vaslat",
                  startJoint: "left_upLeg_joint", endJoint: "left_leg_joint",
                  offsetStart: SIMD3(0.03, -0.05, 0.02), offsetEnd: SIMD3(0.02, 0.02, 0.02),
                  radius: 0.02)

        // === Thigh — Posterior ===

        // Semimembranosus (hip → knee, back of thigh)
        bilateral("semimem",
                  startJoint: "left_upLeg_joint", endJoint: "left_leg_joint",
                  offsetStart: SIMD3(-0.01, 0, -0.04), offsetEnd: SIMD3(-0.02, 0.02, -0.02),
                  radius: 0.018)

        // Biceps femoris long head
        bilateral("bflh",
                  startJoint: "left_upLeg_joint", endJoint: "left_leg_joint",
                  offsetStart: SIMD3(0.02, 0, -0.04), offsetEnd: SIMD3(0.03, 0.02, -0.01),
                  radius: 0.016)

        // === Lower Leg ===

        // Gastrocnemius medial (knee → ankle, back of calf)
        bilateral("gasmed",
                  startJoint: "left_leg_joint", endJoint: "left_foot_joint",
                  offsetStart: SIMD3(-0.01, -0.02, -0.03), offsetEnd: SIMD3(0, 0.02, -0.01),
                  radius: 0.02)

        // Gastrocnemius lateral
        bilateral("gaslat",
                  startJoint: "left_leg_joint", endJoint: "left_foot_joint",
                  offsetStart: SIMD3(0.02, -0.02, -0.03), offsetEnd: SIMD3(0, 0.02, -0.01),
                  radius: 0.018)

        // Soleus (below knee → ankle, deep calf)
        bilateral("soleus",
                  startJoint: "left_leg_joint", endJoint: "left_foot_joint",
                  offsetStart: SIMD3(0, -0.06, -0.02), offsetEnd: SIMD3(0, 0.02, -0.01),
                  radius: 0.022)

        // Tibialis anterior (knee → foot, front of shin)
        bilateral("tibant",
                  startJoint: "left_leg_joint", endJoint: "left_foot_joint",
                  offsetStart: SIMD3(0.01, -0.03, 0.03), offsetEnd: SIMD3(0.01, 0, 0.02),
                  radius: 0.014)

        // === Hip / Gluteal ===

        // Gluteus maximus (3 parts in the model — use glmax1 as representative)
        bilateral("glmax1",
                  startJoint: "hips_joint", endJoint: "left_upLeg_joint",
                  offsetStart: SIMD3(0.06, -0.02, -0.06), offsetEnd: SIMD3(0.02, -0.05, -0.03),
                  radius: 0.03)

        // Gluteus medius
        bilateral("glmed1",
                  startJoint: "hips_joint", endJoint: "left_upLeg_joint",
                  offsetStart: SIMD3(0.08, 0.02, -0.02), offsetEnd: SIMD3(0.04, -0.02, 0),
                  radius: 0.022)

        // Psoas (lumbar → hip, deep hip flexor)
        bilateral("psoas",
                  startJoint: "spine_1_joint", endJoint: "left_upLeg_joint",
                  offsetStart: SIMD3(0.02, -0.03, 0.03), offsetEnd: SIMD3(0, 0.02, 0.01),
                  radius: 0.014)

        // === Trunk ===

        // Erector spinae (approximate as center muscles)
        defs.append(MuscleDef(name: "ercspn_r",
                              startJoint: "hips_joint", endJoint: "spine_4_joint",
                              offsetStart: SIMD3(0.03, 0, -0.06),
                              offsetEnd: SIMD3(0.03, 0, -0.04),
                              radius: 0.02, side: .right))
        defs.append(MuscleDef(name: "ercspn_l",
                              startJoint: "hips_joint", endJoint: "spine_4_joint",
                              offsetStart: SIMD3(-0.03, 0, -0.06),
                              offsetEnd: SIMD3(-0.03, 0, -0.04),
                              radius: 0.02, side: .left))

        return defs
    }()

    func setup(anchor: AnchorEntity) {
        self.anchor = anchor
    }

    /// **The whole drawn set, as pure data, from the pose alone.**
    ///
    /// There is no muscle-solver argument, and that is the point rather than an
    /// omission: with no activation in scope, neither the SELECTION nor the
    /// colour of a capsule can depend on one. The returned set is the fixed
    /// anatomical list `muscleDefs`, minus any capsule whose two joints are not
    /// both tracked in this frame and any that comes out degenerate.
    ///
    /// Split out of `update` so it can be tested without RealityKit.
    static func capsulePlan(joints: [String: SIMD3<Float>]) -> [Capsule] {
        // Trunk-stable body frame: (right, up, forward) in meters.
        // Muscle offsets are interpreted in this frame rather than world
        // axes, so capsules stay anchored anatomically as the body rotates.
        let body = computeBodyFrame(joints)

        var plan: [Capsule] = []
        plan.reserveCapacity(muscleDefs.count)
        for def in muscleDefs {
            guard let startPos = joints[def.startJoint],
                  let endPos = joints[def.endJoint] else { continue }
            let start = startPos + body.transform(def.offsetStart)
            let end = endPos + body.transform(def.offsetEnd)
            guard simd_length(end - start) > 0.01 else { continue }
            plan.append(Capsule(name: def.name, start: start, end: end, radius: def.radius))
        }
        return plan
    }

    /// Draw the anatomy layer for this pose. Nothing else is an input.
    func update(joints: [TrackedJoint]) {
        guard let anchor else { return }

        var jointPositions: [String: SIMD3<Float>] = [:]
        for joint in joints where joint.isTracked {
            jointPositions[joint.id] = joint.worldPosition
        }

        let plan = Self.capsulePlan(joints: jointPositions)
        var drawn: Set<String> = []
        for capsule in plan {
            drawn.insert(capsule.name)
            updateCapsule(capsule, anchor: anchor)
        }
        // A capsule whose joints dropped out this frame is hidden rather than
        // left at its last position, where it would read as anatomy that is
        // still being tracked.
        for (name, entity) in muscleEntities where !drawn.contains(name) {
            entity.isEnabled = false
        }
    }

    private func updateCapsule(_ capsule: Capsule, anchor: AnchorEntity) {
        let delta = capsule.end - capsule.start
        let length = simd_length(delta)

        let entity: ModelEntity
        if let existing = muscleEntities[capsule.name] {
            entity = existing
            entity.isEnabled = true
        } else {
            let mesh = MeshResource.generateBox(
                size: SIMD3(capsule.radius * 2, capsule.radius * 2, 1.0),
                cornerRadius: capsule.radius
            )
            // The material is built once and never rebuilt: with a constant
            // colour there is no per-frame value to re-bake, which is also why
            // the old 64-bucket quantiser is gone.
            entity = ModelEntity(mesh: mesh,
                                 materials: [UnlitMaterial(color: Self.color(for: capsule))])
            anchor.addChild(entity)
            muscleEntities[capsule.name] = entity
        }

        let midpoint = (capsule.start + capsule.end) / 2
        entity.position = midpoint
        entity.scale = SIMD3(1, 1, length)
        let dir = delta / length
        let up: SIMD3<Float> = abs(dir.y) > 0.9 ? SIMD3(1, 0, 0) : SIMD3(0, 1, 0)
        entity.look(at: capsule.end, from: midpoint, upVector: up, relativeTo: nil)
    }

    func setVisible(_ visible: Bool) {
        for (_, entity) in muscleEntities {
            entity.isEnabled = visible
        }
    }

    /// Remove all muscle entities.
    func clear() {
        for (_, entity) in muscleEntities {
            entity.removeFromParent()
        }
        muscleEntities.removeAll()
    }

    // MARK: - Body frame

    /// Orthonormal basis centered on the pelvis.
    /// `right` = body-lateral (user's right side), `up` = spine axis,
    /// `forward` = anterior. Falls back to world axes when joints are missing.
    struct BodyFrame {
        let right: SIMD3<Float>
        let up: SIMD3<Float>
        let forward: SIMD3<Float>

        func transform(_ offset: SIMD3<Float>) -> SIMD3<Float> {
            offset.x * right + offset.y * up + offset.z * forward
        }

        static let identity = BodyFrame(
            right:   SIMD3(1, 0, 0),
            up:      SIMD3(0, 1, 0),
            forward: SIMD3(0, 0, 1)
        )
    }

    /// Test hook. The orientation of this basis decides which side of the body
    /// every muscle capsule lands on, and a cross-product sign error there is
    /// invisible in review — see `BodyFrameOrientationTests`.
    static func bodyFrameForTesting(_ joints: [TrackedJoint]) -> BodyFrame? {
        var dict: [String: SIMD3<Float>] = [:]
        for j in joints where j.isTracked { dict[j.id] = j.worldPosition }
        return computeBodyFrame(dict)
    }

    private static func computeBodyFrame(_ joints: [String: SIMD3<Float>]) -> BodyFrame {
        let hips  = joints["hips_joint"]
        let spine = joints["spine_4_joint"] ?? joints["spine_7_joint"] ?? joints["spine_1_joint"]
        let leftHip  = joints["left_upLeg_joint"]
        let rightHip = joints["right_upLeg_joint"]

        // Up: spine direction from pelvis upward. Fallback to world up.
        let up: SIMD3<Float>
        if let hips, let spine {
            let delta = spine - hips
            let n = simd_length(delta)
            up = n > 1e-4 ? delta / n : SIMD3(0, 1, 0)
        } else {
            up = SIMD3(0, 1, 0)
        }

        // Pelvis-right: from left hip to right hip. Fallback to world +X.
        let pelvisRight: SIMD3<Float>
        if let leftHip, let rightHip {
            let delta = rightHip - leftHip
            let n = simd_length(delta)
            pelvisRight = n > 1e-4 ? delta / n : SIMD3(1, 0, 0)
        } else {
            pelvisRight = SIMD3(1, 0, 0)
        }

        // forward = up × right (ANTERIOR), then re-orthogonalise
        // right = forward × up so the basis stays orthonormal even when pelvis
        // and spine are not perpendicular in the captured pose.
        //
        // The operand order matters and was previously inverted: `right × up`
        // is POSTERIOR, not anterior. Concretely, for a subject facing the
        // camera in ARKit world axes (X image-right, Y up, Z toward camera),
        // their anterior is +Z and their own right hand is at −X, so
        // right × up = (−1,0,0) × (0,1,0) = (0,0,−1) — behind them.
        //
        // That was not cosmetic. `muscleDefs` places capsules with +Z meaning
        // anterior — the quadriceps (recfem, vasmed, vaslat) all carry positive
        // Z offsets and the hamstrings (semimem, bflh) all carry negative ones —
        // so with a posterior third axis the quadriceps were drawn on the BACK
        // of the thigh and the hamstrings on the front.
        var forward = simd_cross(up, pelvisRight)
        let fwdNorm = simd_length(forward)
        if fwdNorm < 1e-3 {
            // Body axis and pelvis axis collinear — degenerate. Use world Z.
            return .identity
        }
        forward /= fwdNorm
        let right = simd_normalize(simd_cross(forward, up))

        return BodyFrame(right: right, up: up, forward: forward)
    }

    // MARK: - Colour
    //
    // There is no colour MAPPING any more, and this is where it used to be: a
    // sqrt-warped blue→cyan→green→yellow→red ramp over activation, with the
    // alpha rising from 0.45 to 0.95 so that "max effort" also read as the most
    // opaque thing on screen. Every one of those channels carried the same
    // cross-muscle claim. `capsuleColor` replaced it — see the type doc.
    //
    // `UnlitMaterial` is kept because it bypasses scene lighting, so the
    // capsules look the same against any camera feed and their appearance
    // stays a constant rather than a function of the room.
}
