import UIKit
import RealityKit
import simd

/// Renders 3D muscle visualization as colored capsules positioned anatomically,
/// overlaid on the camera feed via RealityKit.
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

    private struct MuscleState {
        let entity: ModelEntity
        var lastBucket: Int
        /// Marks which pass last refreshed this entity: hardcoded-def or
        /// path-based. Allows garbage-collecting stale path entities when
        /// a muscle drops below the activation threshold.
        var kind: Kind
        enum Kind { case def, path }
    }

    private var muscleEntities: [String: MuscleState] = [:]
    private var anchor: AnchorEntity?

    /// Path-based rendering: muscles with raw activation above this
    /// threshold AND not covered by the hardcoded set get a capsule drawn
    /// directly between their .osim path endpoints. 0.08 is empirically
    /// above the postural-tone floor (≈0.02–0.05 after the rebalance).
    private static let pathRenderActivationThreshold: Float = 0.08
    /// How far above the solver's activation floor a muscle must sit before it
    /// counts as firing. The floor itself is a solver bound, not an observation.
    private static let floorMargin: Float = 0.05
    /// Hard cap on simultaneously drawn path capsules. The static-optimisation
    /// QP has ~520 unknowns against ~110 torque equations, so most of what it
    /// returns is the cost function choosing among a ~410-dimensional null
    /// space. Rendering all of it would imply far more measurement than exists.
    private static let maxRenderedPathMuscles = 24

    /// Names (after alias merging) already handled by the hardcoded defs.
    /// Computed once so per-frame lookup is O(1).
    private static let hardcodedDisplayNames: Set<String> = {
        Set(muscleDefs.map(\.name))
    }()

    /// Aliases from solver-native names to display names — a subset copy
    /// of NimbleEngine.displayMuscleAliases. Kept local to avoid a
    /// cross-module reach; the entries are load-bearing only when the
    /// overlay decides whether a path-named muscle is already drawn by a
    /// hardcoded def.
    private static let solverToDisplayAlias: [String: String] = [
        "bflh140_r": "bflh_r",
        "bflh140_l": "bflh_l",
        "gaslat140_r": "gaslat_r",
        "gaslat140_l": "gaslat_l",
        "vaslat140_r": "vaslat_r",
        "vaslat140_l": "vaslat_l",
        "multifidus_T9_T7": "ercspn_r",
        "multifidus_T9_T7_L": "ercspn_l",
    ]

    // Quantize activation into this many buckets; material only rebuilt on bucket change.
    // 64 (vs the original 32) because the display mapping now spreads 0.02-0.30
    // across the full color band — we need finer resolution in the visible range.
    private static let activationBuckets = 64

    // Visual calibration: activation values are remapped via sqrt onto
    // [displayFloor, displaySaturation] so the normal physiological range
    // (postural tone ~0.02 → active effort ~0.30) spans the full colormap.
    // Higher values saturate at red.
    private static let displayFloor: Float = 0.02
    private static let displaySaturation: Float = 0.30

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

    /// Update muscle visualization with current joint positions and the
    /// latest muscle-solver output. `muscle` carries both alias-merged
    /// display activations (used by the hardcoded defs) and raw
    /// solver-native paths + activations (used for path-based rendering
    /// of muscles the hardcoded set doesn't cover).
    func update(joints: [TrackedJoint], muscle: NimbleEngine.MuscleOutput) {
        guard let anchor else { return }

        var jointPositions: [String: SIMD3<Float>] = [:]
        for joint in joints where joint.isTracked {
            jointPositions[joint.id] = joint.worldPosition
        }

        // Trunk-stable body frame: (right, up, forward) in meters.
        // Muscle offsets are interpreted in this frame rather than world
        // axes, so capsules stay anchored anatomically as the body rotates.
        let body = Self.computeBodyFrame(jointPositions)

        // --- Pass 1: hardcoded anatomical defs (visible even at rest) ---
        for def in Self.muscleDefs {
            guard let startPos = jointPositions[def.startJoint],
                  let endPos = jointPositions[def.endJoint] else {
                muscleEntities[def.name]?.entity.isEnabled = false
                continue
            }

            let worldStart = startPos + body.transform(def.offsetStart)
            let worldEnd = endPos + body.transform(def.offsetEnd)
            let activation = muscle.activations[def.name] ?? Double(Self.displayFloor)
            updateCapsule(key: def.name,
                          start: worldStart,
                          end: worldEnd,
                          radius: def.radius,
                          activation: activation,
                          kind: .def,
                          anchor: anchor)
        }

        // --- Pass 2: path-based rendering for active uncovered muscles ---
        // Only draw what the solver says is firing AND that isn't already
        // represented by a hardcoded def (avoids double-rendering).
        // A fixed activation threshold does not work on a 520-muscle model.
        // Measured on a real solve (OfflineOrchestrationTests): 139 of 520
        // muscles cleared 0.08, so the figure disappeared under a thicket of
        // capsules. The distribution is not a spread — the median sits at
        // 0.0200027 against an `aMin` floor of 0.02, with a handful saturated at
        // 1.0. Almost every muscle is either pinned at the floor or railed at
        // the ceiling.
        //
        // The floor is not a measurement. STATUS.md records that `aMin`/`epsA`
        // were tuned so the visualisation would not go "permanently blue" — a
        // rendering parameter. Drawing floor-valued muscles presents that
        // artefact as though it were measured effort.
        //
        // So: drop anything not meaningfully above the floor, then keep only the
        // strongest few. Ranking rather than thresholding also makes the display
        // stable — with 139 near-identical floor values, which ones cleared a
        // fixed cut flickered frame to frame, which is what read as twitching.
        let floor = Float(muscle.rawActivations.values.min() ?? 0)
        let ranked = muscle.rawActivations
            .filter { Float($0.value) >= max(Self.pathRenderActivationThreshold, floor + Self.floorMargin) }
            .sorted { $0.value > $1.value }
            .prefix(Self.maxRenderedPathMuscles)

        var activePathKeys: Set<String> = []
        for (rawName, rawActivation) in ranked {
            let displayName = Self.solverToDisplayAlias[rawName] ?? rawName
            if Self.hardcodedDisplayNames.contains(displayName) { continue }
            guard let path = muscle.paths[rawName] else { continue }
            let length = simd_length(path.end - path.start)
            guard length > 0.02 else { continue }  // skip degenerate/tiny muscles

            // Radius scales with √F_max so big muscles (quad, glute) read
            // thicker than small ones (intrinsic hand muscles). Clamped.
            let fmax = Float(muscle.maxForces[rawName] ?? 500)
            let radius = max(0.008, min(0.022, 0.0005 * sqrt(fmax)))

            let key = "path_\(rawName)"
            activePathKeys.insert(key)
            updateCapsule(key: key,
                          start: path.start,
                          end: path.end,
                          radius: radius,
                          activation: rawActivation,
                          kind: .path,
                          anchor: anchor)
        }

        // Hide path entities that are no longer active.
        for (key, state) in muscleEntities where state.kind == .path
            && !activePathKeys.contains(key) {
            state.entity.isEnabled = false
        }
    }

    private func updateCapsule(key: String,
                               start: SIMD3<Float>,
                               end: SIMD3<Float>,
                               radius: Float,
                               activation: Double,
                               kind: MuscleState.Kind,
                               anchor: AnchorEntity) {
        let delta = end - start
        let length = simd_length(delta)
        guard length > 0.01 else {
            muscleEntities[key]?.entity.isEnabled = false
            return
        }

        let bucket = min(Self.activationBuckets - 1,
                         max(0, Int(Float(activation) * Float(Self.activationBuckets))))

        let entity: ModelEntity
        if let existing = muscleEntities[key] {
            entity = existing.entity
            if existing.lastBucket != bucket {
                entity.model?.materials = [Self.makeMaterial(activation)]
                muscleEntities[key]?.lastBucket = bucket
            }
            entity.isEnabled = true
        } else {
            let mesh = MeshResource.generateBox(
                size: SIMD3(radius * 2, radius * 2, 1.0),
                cornerRadius: radius
            )
            entity = ModelEntity(mesh: mesh, materials: [Self.makeMaterial(activation)])
            anchor.addChild(entity)
            muscleEntities[key] = MuscleState(entity: entity, lastBucket: bucket, kind: kind)
        }

        let midpoint = (start + end) / 2
        entity.position = midpoint
        entity.scale = SIMD3(1, 1, length)
        let dir = delta / length
        let up: SIMD3<Float> = abs(dir.y) > 0.9 ? SIMD3(1, 0, 0) : SIMD3(0, 1, 0)
        entity.look(at: end, from: midpoint, upVector: up, relativeTo: nil)
    }

    func setVisible(_ visible: Bool) {
        for (_, state) in muscleEntities {
            state.entity.isEnabled = visible
        }
    }

    /// Remove all muscle entities.
    func clear() {
        for (_, state) in muscleEntities {
            state.entity.removeFromParent()
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

    // MARK: - Color Mapping

    /// Remap raw activation via sqrt onto [0, 1] over the observable
    /// physiological band [displayFloor, displaySaturation]. sqrt expands
    /// the low end where normal postural tone lives.
    private static func displayValue(_ activation: Double) -> Float {
        let a = max(Float(activation), 0)
        let lo = sqrt(displayFloor)
        let hi = sqrt(displaySaturation)
        let t = (sqrt(a) - lo) / max(hi - lo, 1e-6)
        return min(max(t, 0), 1)
    }

    /// Blue → cyan → green → yellow → red over the already-normalized display value.
    private static func activationColor(_ activation: Double) -> UIColor {
        let t = displayValue(activation)
        let r: Float, g: Float, b: Float
        if t < 0.25 {
            let u = t / 0.25;              r = 0;           g = u;          b = 1.0
        } else if t < 0.5 {
            let u = (t - 0.25) / 0.25;     r = 0;           g = 1.0;        b = 1.0 - u
        } else if t < 0.75 {
            let u = (t - 0.5) / 0.25;      r = u;           g = 1.0;        b = 0
        } else {
            let u = (t - 0.75) / 0.25;     r = 1.0;         g = 1.0 - u;    b = 0
        }
        // Alpha floor 0.45 keeps rest tone visible; scales to 0.95 at max
        // effort so the colored capsule reads clearly against any camera feed.
        let alpha = 0.45 + 0.50 * t
        return UIColor(red: CGFloat(r), green: CGFloat(g), blue: CGFloat(b), alpha: CGFloat(alpha))
    }

    /// UnlitMaterial bypasses scene lighting so muscle color reads correctly
    /// in any environment; its `color: UIColor` initializer honors the
    /// UIColor alpha for transparent blending.
    private static func makeMaterial(_ activation: Double) -> Material {
        UnlitMaterial(color: activationColor(activation))
    }
}
