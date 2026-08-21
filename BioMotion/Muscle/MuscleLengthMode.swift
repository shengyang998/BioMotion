import Foundation

/// Per-muscle MUSCLE-TENDON LENGTH-CHANGE MODE for an analysed OFFLINE clip.
///
/// ─────────────────────────────────────────────────────────────────────────
/// THE FROZEN PRE-REGISTRATION (2026-08-13, round-4 revision; adjudicated
/// 2026-08-14). The registration lives VERBATIM in THIS doc comment: it was
/// frozen here before the first gate ran, and it is NOT restated in `STATUS.md`
/// — STATUS.md's 2026-08-14 entry points at this file instead.
/// After that moment the claim, the anti-claims, the signal definition, `k`,
/// the deadband formula and all three of its faces, the suppression rules and
/// their predicates, `sigmaVisible`, the admitted set, the gate definitions and
/// every bar are IMMUTABLE. A defect may be fixed only by pinning a RED test
/// first, then fixing, then re-running the SAME gates. A FAIL verdict — no UI
/// change, measured numbers written down — is a complete and valid deliverable.
/// ─────────────────────────────────────────────────────────────────────────
///
/// **What happened to the battery afterwards (2026-08-14, OUTSIDE the frozen
/// text above).** The verdict was FAIL and nothing in the registration moved:
/// no bar, no constant, no suppression rule, no UI. Seven
/// `MuscleLengthModeTests` methods asserted bars the measurement did not meet
/// and were therefore RED, which blocked the commit gate; under an explicit
/// owner authorisation (STATUS next-step 41 option (a)) those seven were
/// converted into MEASURED-OUTCOME PINS — registered bar still asserted,
/// measured number pinned to its receipt, FAIL/NOT-SHIPPED verdict asserted
/// beside it. Only the way the battery RECORDS its refusal changed.
///
/// **Successor lineage, registered 2026-08-21 (also OUTSIDE the frozen text
/// above, and RED-FIRST: every clause below was written before the measurement
/// that adjudicates it).** No bar moved, no constant moved, no code path
/// changed and no test method was added by the commit that recorded these; the
/// clause TEXT lives above the three methods it governs in
/// `BioMotionTests/MuscleLengthModeTests.swift`. Three superseded clauses stay
/// SUPERSEDED-NOT-ERASED at FAILED and none of them may ever be reported as
/// passing:
///
/// 1. **G4(a) → G4(f) + G4(g).** G4(a) conflated "the port reproduces the
///    shipped model" with "the shipped model reproduces the textbook", and only
///    the second was falsified. G4(f) scores port-vs-model fidelity over all 26
///    anchors on the oracle's own sweep poses; G4(g) pins the MODEL-ANCHOR
///    CONFLICT REGISTER — exactly 4 anchors / 7 cells (`TRIlong_r`, `TRImed_r`,
///    `TRIlat_r` at elbow midpoints 135.0 and 145.0 deg; `bfsh140_r` at knee
///    midpoint 137.5 deg), with all 22 other anchors EMPTY over the full oracle
///    range — so the conflict is a falsifiable fact rather than an excuse. The
///    product-surface population restriction was REFUSED as laundering, and
///    G4(g)'s abstention branch is declared VACUOUS-BY-CONSTRUCTION today.
/// 2. **G9(b) → G9(b2).** G9(b)'s control was inert BY ALGEBRA, not by weakness:
///    `jitterMetres` is homogeneous of degree 1 in the moment-arm row and
///    `lengthRate` is linear in it, so a uniform positive row scale leaves both
///    `sign(v)` and `|v| > D` exactly invariant at ANY size. G9(b2) replaces the
///    CLASS, not the constant, with a one-sided SIGN flip — the class this very
///    doc comment names two paragraphs above in "WHAT D DOES NOT CONTAIN" —
///    and pre-registers its own escape hatches, including the aliasing under
///    which it would read 0 despite a real defect.
/// 3. **G3(iv-b) → G3(iv-b2).** "All 12 hip-spanning capsules suppressed" was
///    true only because a 5-marker drive left the pelvis with 0.000000000 range,
///    so Rule 2 suppressed the block by construction. Under a hip-identifying
///    drive that clause CAN NEVER PASS AGAIN — the only route to a pass is
///    deleting the hip markers and returning to the vacuous 5-marker
///    populations — so it stands FAILED PERMANENTLY. G3(iv-b2) asserts EARNED
///    ADMISSION instead: an admitted hip capsule must clear the closed-form
///    marker-sensitivity floor `‖J·eⱼ‖₂ ≥ √0.75·sigmaVisible` implied by Rule 1
///    but computed through an independent path, every suppression must name its
///    witness coordinate, and the clause is bidirectional with an explicit
///    anti-vacuity arm.
///
/// # The claim
///
/// For an analysed **offline** clip, for each admitted muscle and each admitted
/// frame, BioMotion reports **how that muscle's MUSCLE-TENDON PATH LENGTH
/// (muscle plus tendon, origin to insertion around the wrap surfaces) is
/// changing along the pose trajectory**: LENGTHENING, SHORTENING, or NO CHANGE
/// THIS VIEW CAN RESOLVE (within a derived, per-muscle deadband).
///
/// The quantity is `dL_MT/dt = -R_m(q)ᵀ dq` on the Savitzky-Golay-smoothed IK
/// pose, using the validated muscle-path model. The identity, and the fact that
/// positive `R` with positive `dq` means the muscle SHORTENS, are already
/// shipped and documented at `MuscleSolver.mm:598-611` (computation at `:664`);
/// `R = -dL_MT/dq` is set by `MomentArmComputer.mm:1192`.
///
/// # Anti-claims, binding, stated in the UI
///
/// 1. NOT effort, activation, force, tension or work.
/// 2. NOT eccentric-vs-concentric — that needs the force chain, gated behind
///    foot-contact capability both bundled models structurally lack
///    (`NimbleBridge.mm:1770`; `STATUS.md:4577-4584`).
/// 3. NOT fibre or fascicle length — the tendon can absorb or supply the whole
///    change, so an MT path that lengthens does NOT mean the contractile fibres
///    lengthened. On running clips exactly like the two this layer is gated on,
///    the calf MT unit lengthens through stance while its fascicles are near
///    isometric; the copy must therefore say "muscle and tendon", never
///    "muscle".
/// 4. NOT a magnitude, percentage or rate; no per-muscle numbers.
/// 5. NOT a cross-muscle comparison or ordering.
/// 6. NOT a per-muscle LEFT/RIGHT claim, and this is enforced by BOTH a gate and
///    a pinned caption line, because the gate cannot reach the whole failure
///    class. `MomentArmErrorCancellationTests.swift:44-45` measured that "A
///    ONE-SIDED error (different paths on the two legs, or the same wrap error
///    evaluated at two different poses) costs more than 10 pp". G9 (bilateral
///    mirror coherence) reaches ONLY the first sub-class — different paths on
///    the two legs at ONE shared pose. It structurally CANNOT reach the second,
///    because a mirrored pose puts both legs at the same configuration by
///    construction — and that second sub-class IS the running regime, where the
///    legs are antiphase on every frame.
///
/// # Third-state vocabulary
///
/// Fixed by the repo's own three-way precedent (`PostureFindingsPanel.swift:11-13`:
/// findings / "no measurable deviation" / "Not measurable from this view"): the
/// deadband state is **"no change this view can resolve"**, never "steady" as a
/// positive finding and never "holding", which in ordinary English names an
/// isometric CONTRACTION and leaks effort onto the one state where a viewer
/// most wants it. A muscle-tendon path getting longer is not a muscle being
/// loaded; getting shorter is not a muscle working.
///
/// # Why the identity, not frame-to-frame length differencing
///
/// 1. The registration names the identity as the deadband's propagation path.
/// 2. `MuscleSolver.mm:607-611` records that differencing `L_MT` against the
///    previous solved frame "disagreed with the Savitzky-Golay filter that
///    produced `dq` in the first place".
/// 3. Wrap-switch safety runs OPPOSITE to intuition. `R` comes from the
///    signature-aware stencil with one-sided fallback and up to 8× step-halving
///    (`MomentArmComputer.mm:1188-1202`, `:1239`); a raw inter-frame difference
///    has none. `STATUS.md:3056-3061`: raw centred difference **−19.62 m** vs
///    shipped **−0.033693 m** (`grac_r`/`knee_angle_r` at `q* = −1.465481812`).
///
/// Direct differencing becomes **witness B in G7**, so the fork is measured
/// rather than argued.
///
/// # Deadband: ONE formula, THREE frozen faces
///
///     s_m(q)      = sqrt( Σⱼ R[m,j](q)² · σ̂_q[j]² )                [metres]
///     D_step_m(q) = max( k · g_vel(taps) · s_m(q), L_quant_floor )  [metres]
///     D_rate_m(t) = D_step_m(q_t) / dt                              [m/s]
///     D_diff_m(q) = max( k · sqrt(2·c₀) · s_m(q), L_quant_floor )   [metres]
///
/// `D_rate` is the shipped classifier. `D_step` is `D_rate` over one sample
/// interval. `D_diff` is frozen for G7's witness B (a raw two-point difference
/// of two SG-smoothed lengths, not an SG-filtered derivative): the smoothed
/// output has noise std `σ·√c₀`, and a difference of two such samples has std at
/// most `√(2c₀)·σ`. `√(2c₀) = √0.510822… = 0.714718`. Adjacent SG outputs are
/// positively correlated, so this is an UPPER bound — deliberately conservative,
/// which shrinks B's scoring population and is disclosed rather than discovered.
///
/// - `R[m,j](q)`: the shipped `MomentArmComputer` moment arm, from the
///   signature-aware stencil.
/// - `g_vel(9,3) = 0.338139` — `WindowedDerivativeFilter.velocityNoiseGain`.
/// - `c₀ = posCoefficients[halfWindow] = 59/231 = 0.255411`, read at runtime.
///   `Σ cᵢ² = c₀` exactly, so a projection smoother's residual variance is
///   `σ²(1 − c₀)` for INDEPENDENT samples: residual std `0.862896σ`, a 13.710 %
///   deflation, corrected by `1/√(1−c₀) = 1.158889`.
/// - `σ̂_q[j]`, **clip face** (G2/G7/G8/production): `r_j(t_c) = q_j_raw(t_c) −
///   q_j_SG(t_c)` on the SG POSITION channel at the window centre;
///   `σ̂ = 1.4826 · median|r − median r| / √(1 − c₀)`. A true MAD, so nothing
///   presumes zero-median residuals. 1.4826 is a standard EXTERNAL statistical
///   constant, not a repo artefact.
/// - `σ̂_q[j]`, **fixture face** (G1/G4/G9): FROZEN at `1.0e-6` rad for every `j`
///   — on that population no pose estimator and no IK runs, the pose is imposed
///   exactly, and `1e-6` rad is the pinned identical-marker per-solve bound
///   (`NimbleBridgeTests.swift:601`). Consequence, stated before any run: for a
///   typical `|R| ≈ 5e-2 m/rad` this gives `D_step ≈ 5e-8 m` against a 5° sweep
///   step that moves length by `≈4e-3 m`, so essentially every spanning cell
///   clears the deadband and **the deadband cannot buy G1/G4/G9 a pass**.
/// - `L_quant_floor = 1.0e-8 m` — ten quanta of the reference's own 9-decimal
///   storage precision.
/// - `k = 3`. `P(|noise| > 3σ) = 0.27 %` two-tailed, so at most 0.135 % of
///   noise-only frames receive a wrong direction.
///
/// **THE ESTIMATOR IS EXPLICITLY SCOPED.** `Var(r) = σ²(1−c₀)` holds only for
/// white noise. `σ̂` is the MAD of exactly the component the smoother REJECTS, so
/// it bounds the HIGH-FREQUENCY band and nothing else. Pose error from
/// SAM3D/MHR is temporally correlated; its low-frequency component passes the
/// 9-tap cubic essentially unattenuated, lands in `q_SG` rather than in `r`, and
/// contributes ZERO to `σ̂`. The genuinely uncovered case is error near the
/// STRIDE band (≈1.667 Hz at the frozen 0.600 s stride), in particular pose
/// error CORRELATED WITH THE MOTION — that is a bias, not noise, and no deadband
/// of any construction bounds it. G8 is a PARTIAL cover for gross secular drift
/// and says so.
///
/// **WHAT D DOES NOT CONTAIN.** `D` is built from pose jitter only. Moment-arm
/// MODEL error — the one error term this repo actually has receipts for
/// (cylinder wrapping median 0.048 mm / max 8.07 mm with 4 SIGN FLIPS over the
/// 173-pose fixture; multi-wrap max 1.05 mm against the analytic column; R8's
/// p99 relative residual 1.114 %) — is NOT in it, and a sign flip in `R` is a
/// full-magnitude sign flip in `dL/dt` that no 3σ pose band absorbs. That term
/// is bounded elsewhere, by G1 (external oracle), G9 (bilateral coherence,
/// narrowed) and G7 (independent derivative witness).
///
/// **BIAS DIRECTION: UNKNOWN, registered as unknown.** Sub-window motion inflates
/// `σ̂`; smoother shrinkage deflates it; quadrature summation assumes cross-DOF
/// independence that IK coupling violates with unmodelled sign. The squeeze is
/// ONE GATE WIDE: G2(a) flicker punishes a too-narrow clip-face deadband,
/// G2(b)/(c) non-degeneracy punish a too-wide one.
///
/// # Temporal smoothing and dating
///
/// `taps = WindowedDerivativeFilter.maximumTaps = 9`, order 3 — the production
/// default (`NimbleEngine.swift:1522`), reused unchanged, no new filter bank.
/// Centred, so the answer is dated `halfWindow = 4` samples behind the newest
/// push: 267 ms span, 133 ms lag at 30 fps. Every measured stance on the two
/// scored clips is 161.9–206.7 ms, i.e. 4.86–6.20 frames, all SHORTER than
/// 267 ms — so mode is smeared across stance transitions. Registered
/// limitation. Taps are FROZEN at 9. Warmed frames are pinned once: a 9-tap
/// centred filter dates its first output at index 4 and its last at `n−5`, so
/// warmed frames `= n − 8 = 114` per scored clip, and any `t`-vs-`t−1`
/// comparison has at most 113 usable frames per muscle.
enum MuscleLengthMode: String, Equatable, CaseIterable {
    /// `dL_MT/dt > +D_rate`. The muscle-tendon path is getting LONGER.
    case lengthening
    /// `dL_MT/dt < −D_rate`. The muscle-tendon path is getting SHORTER.
    case shortening
    /// `|dL_MT/dt| ≤ D_rate`. A RESOLUTION statement, not a positive finding:
    /// `D` is per-muscle and pose-dependent (`s_m` scales with ‖R_m‖), so a
    /// high-leverage capsule needs a larger true `dL/dt` to escape it than a
    /// low-leverage one.
    case noChangeThisViewCanResolve
    /// Suppressed / not yet warm / unresolved wrap switch / multi-head
    /// disagreement. Renders in the neutral anatomy colour and is NEVER
    /// silently folded into the third state.
    case indeterminate

    /// The three states that carry a length statement. `indeterminate` is an
    /// abstention, not an answer.
    var isDefined: Bool { self != .indeterminate }

    /// `lengthening` or `shortening` — what G2's non-degeneracy clauses count.
    var isDirectional: Bool { self == .lengthening || self == .shortening }
}

/// The frozen deadband formula and classifier. Pure arithmetic: no model, no
/// skeleton, no I/O, so every gate can exercise it directly.
enum MuscleLengthModeClassifier {

    // MARK: Frozen constants

    /// `k` in `D = k·g·s_m`. Frozen at 3: `P(|noise| > 3σ) = 0.27 %` two-tailed.
    static let k: Double = 3.0

    /// `L_quant_floor`, metres. Derived from the reference's storage precision:
    /// `opensim_moment_arms.txt` stores lengths to 9 decimals, so a difference
    /// of two stored lengths has a `1e-9 m` quantum and its SIGN is
    /// rounding-dominated below `≈1e-8 m`. Ten quanta.
    static let lengthQuantisationFloorMetres: Double = 1.0e-8

    /// `σ̂_q[j]` on the FIXTURE face, radians. The pose is imposed exactly, so
    /// the only noise source is solver numerics given exact input — the
    /// identical-marker regime, whose pinned per-solve bound this is.
    static let fixtureFaceJointNoiseRadians: Double = 1.0e-6

    /// The standard MAD-to-σ consistency constant for a normal distribution. An
    /// EXTERNAL statistical constant, not a repo artefact.
    static let madToSigma: Double = 1.4826

    /// Frozen production window. Not a tunable: the layer reuses the existing
    /// per-DOF filter bank rather than introducing a second one.
    static let taps: Int = 9

    // MARK: The formula

    /// `s_m(q) = sqrt( Σⱼ R[m,j]² · σ̂ⱼ² )`, metres.
    ///
    /// Quadrature summation assumes cross-DOF independence that IK coupling
    /// violates with unmodelled sign — registered as an unknown bias direction,
    /// not as a claim.
    static func jitterMetres(momentArmRow: [Double], jointNoiseRadians: [Double]) -> Double {
        let n = min(momentArmRow.count, jointNoiseRadians.count)
        var sum = 0.0
        for j in 0..<n {
            let term = momentArmRow[j] * jointNoiseRadians[j]
            sum += term * term
        }
        return sum.squareRoot()
    }

    /// `D_step_m(q) = max( k · g_vel(taps) · s_m(q), L_quant_floor )`, metres.
    static func stepDeadbandMetres(momentArmRow: [Double],
                                   jointNoiseRadians: [Double],
                                   velocityNoiseGain: Double) -> Double {
        let s = jitterMetres(momentArmRow: momentArmRow, jointNoiseRadians: jointNoiseRadians)
        return max(k * velocityNoiseGain * s, lengthQuantisationFloorMetres)
    }

    /// `D_rate_m(t) = D_step_m(q_t) / dt`, metres per second. The SHIPPED
    /// classifier's threshold.
    static func rateDeadbandMetresPerSecond(momentArmRow: [Double],
                                            jointNoiseRadians: [Double],
                                            velocityNoiseGain: Double,
                                            sampleInterval: Double) -> Double {
        guard sampleInterval > 0 else { return .infinity }
        return stepDeadbandMetres(momentArmRow: momentArmRow,
                                  jointNoiseRadians: jointNoiseRadians,
                                  velocityNoiseGain: velocityNoiseGain) / sampleInterval
    }

    /// `D_diff_m(q) = max( k · sqrt(2·c₀) · s_m(q), L_quant_floor )`, metres.
    /// **Witness B only** — a raw two-point difference of two SG-smoothed
    /// lengths, whose noise this bounds from ABOVE (adjacent SG outputs are
    /// positively correlated, so the independence assumption is conservative).
    static func differenceDeadbandMetres(momentArmRow: [Double],
                                         jointNoiseRadians: [Double],
                                         centreCoefficient c0: Double) -> Double {
        let s = jitterMetres(momentArmRow: momentArmRow, jointNoiseRadians: jointNoiseRadians)
        return max(k * (2.0 * c0).squareRoot() * s, lengthQuantisationFloorMetres)
    }

    /// `dL_MT/dt = -R_m(q)ᵀ dq`. Positive `R` with positive `dq` means the
    /// muscle SHORTENS — the shipped sign convention.
    static func lengthRate(momentArmRow: [Double], jointVelocity: [Double]) -> Double {
        let n = min(momentArmRow.count, jointVelocity.count)
        var sum = 0.0
        for j in 0..<n { sum += momentArmRow[j] * jointVelocity[j] }
        return -sum
    }

    /// The classification table. Identical in form on every face; only the
    /// units of `value`/`deadband` differ (m/s for `D_rate`, m for `D_step` and
    /// `D_diff`).
    static func classify(value: Double, deadband: Double) -> MuscleLengthMode {
        guard value.isFinite, deadband.isFinite else { return .indeterminate }
        if value > deadband { return .lengthening }
        if value < -deadband { return .shortening }
        return .noChangeThisViewCanResolve
    }

    // MARK: The clip-face noise estimator

    /// `σ̂_q[j] = 1.4826 · median|r − median r| / √(1 − c₀)` on the SG POSITION
    /// residual at the window centre.
    ///
    /// A TRUE MAD, so nothing presumes zero-median residuals. The shrinkage
    /// correction is exact for a projection smoother (`Σcᵢ² = c₀`), and holds
    /// only for white noise — see the scope note in `MuscleLengthMode`'s doc.
    static func robustJointNoiseRadians(residuals: [Double], centreCoefficient c0: Double) -> Double {
        guard !residuals.isEmpty, c0 < 1.0 else { return 0 }
        let centre = median(residuals)
        let deviations = residuals.map { abs($0 - centre) }
        let mad = median(deviations)
        return madToSigma * mad / (1.0 - c0).squareRoot()
    }

    /// Plain median with the usual even-count average. Deterministic.
    static func median(_ values: [Double]) -> Double {
        guard !values.isEmpty else { return 0 }
        let sorted = values.sorted()
        let n = sorted.count
        if n % 2 == 1 { return sorted[n / 2] }
        return 0.5 * (sorted[n / 2 - 1] + sorted[n / 2])
    }

    /// Warmed-frame count for a centred window: the filter dates its first
    /// output at `halfWindow` and its last at `n − 1 − halfWindow`.
    static func warmedFrameCount(samples: Int, taps: Int = taps) -> Int {
        max(0, samples - (taps - 1))
    }

    /// Index of the first warmed (SG-centre-dated) sample.
    static func firstWarmedIndex(taps: Int = taps) -> Int { taps / 2 }
}
