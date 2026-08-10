import Foundation

/// **Research-only, unvalidated load hypothesis retained for regression work.**
///
/// This turns contact/flight TIMING into a hypothetical peak vertical ground
/// reaction force in body weights. It is not a product measurement and no
/// current bundled-model frame may publish it. Both bundled `ContactGeometrySet`s
/// are empty, and the active near-CoP routine supplies no validated support
/// polygon, unilateral-contact, or friction constraint. The production engine
/// therefore stops at `.contactSupportUnavailable` before ID, GRF, CoP, muscle,
/// or gait-load output. The derivation below documents the historical timing
/// model; it does not fill that missing mechanics layer.
///
/// # The derivation, written out because the formula alone cannot be checked
///
/// Take one complete stride of duration `T`. Steady running means the body's
/// vertical velocity at the end of the stride equals the velocity at its start,
/// so the net vertical impulse over the stride is zero:
///
///     ∫₀ᵀ ( F(t) − m·g ) dt = 0        ⟹        ∫₀ᵀ F(t) dt = m·g·T
///
/// During flight `F = 0`, so the whole of `m·g·T` has to be delivered by the
/// contacts. A stride contains two contacts (one per foot) of duration `tc`:
///
///     2 · ∫_contact F(t) dt = m·g·T
///
/// Assume the stance force is a half sine, `F(t) = Fmax · sin(π·t/tc)` — the
/// standard running model, and the same one the earlier scouting used:
///
///     ∫_contact F dt = Fmax · 2·tc/π
///     ⟹ 2 · Fmax · 2·tc/π = m·g·T
///     ⟹ Fmax = π·m·g·T / (4·tc)
///
/// A stride is two contacts plus two flights, `T = 2·tc + 2·tf`, hence
///
///     T / (4·tc) = (2·tc + 2·tf) / (4·tc) = ½·(1 + tf/tc)
///     ⟹ **Fmax = (π/2)·m·g·(1 + tf/tc)**
///
/// Nothing in that chain needs the runner's mass, the camera, the depth channel,
/// or the root's acceleration: `Fmax/(m·g)` is a pure function of the RATIO of
/// flight time to contact time. That is why the historical model could derive
/// a hypothetical force-shaped number from a hand-held tracking shot and
/// expressed it in body weights rather than newtons. Derivability is not
/// validation, and production publishes neither form.
///
/// # What the half-sine assumption is worth
///
/// The shape enters only through the mean-to-peak ratio, `2/π = 0.637` for a
/// half sine. Measured running vGRF traces sit around 0.6-0.7, so a shape error
/// is worth roughly ±10 % on `Fmax`. **That error is a COMMON SCALE over every
/// muscle in the contact**: the muscle QP is linear in the external load while
/// no muscle is saturated, so muscle-to-muscle and left-to-right ratios survive
/// it exactly. It degrades only where activations reach `a ≤ 1`, which running
/// peaks do reach — so it is a caveat on the ratios, not a free pass, and it is
/// one more reason the historical diagnostic used a ratio and not a newton
/// figure. It is not permission to surface either one.
///
/// # What would refute this model
///
/// `tf ≤ 0` (no flight phase — the subject is walking, and the impulse is then
/// shared by two feet in double support, so `Fmax` above is simply the wrong
/// equation), or a duty factor ≥ 0.5, or the two independent flight-time
/// estimates in `GaitReport` disagreeing by more than one frame. All three are
/// checked; the first two refuse, the third refuses.
struct GaitForceModel: Equatable {

    /// Flight time divided by contact time — the only input.
    let flightToContactRatio: Double
    /// `Fmax / (m·g)`. Body weights, dimensionless.
    let peakVerticalForceInBodyWeights: Double
    /// `tc / T`, ONE foot's contact time as a fraction of the whole stride —
    /// the standard definition, and the one that reads 0.5 exactly when flight
    /// vanishes (`T = 2·tc + 2·tf`, so `tc/T < 0.5 ⟺ tf > 0`). Measured 0.274
    /// and 0.320 on the two usable clips, which is the running range.
    let dutyFactor: Double

    /// - Parameters:
    ///   - contactSeconds: mean stance duration over both feet.
    ///   - flightSeconds: mean flight duration between consecutive contacts.
    init(contactSeconds: Double, flightSeconds: Double) {
        let ratio = contactSeconds > 0 ? flightSeconds / contactSeconds : .nan
        flightToContactRatio = ratio
        peakVerticalForceInBodyWeights = (Double.pi / 2) * (1 + ratio)
        dutyFactor = contactSeconds / (2 * (contactSeconds + flightSeconds))
    }

    /// A running stride must contain flight. Without it the derivation's
    /// "`F = 0` during flight" step is false and the number is meaningless.
    var describesRunning: Bool {
        flightToContactRatio.isFinite && flightToContactRatio > 0 && dutyFactor < 0.5
    }

    /// `Fmax/(m·g)` for ONE contact of `contactSeconds`, closed against the same
    /// stride flight time.
    ///
    /// The derivation above never required the two feet to have equal contact
    /// times — it required their two impulses to sum to `m·g·T`. Writing
    /// `T = tcL + tcR + 2·tf` and giving each contact `Fᵢ·2·tcᵢ/π` of impulse,
    ///
    ///     Σ Fᵢ·2·tcᵢ/π = (tcL + tf) + (tcR + tf) = T
    ///
    /// when `Fᵢ = (π/2)(1 + tf/tcᵢ)`. So per-leg peaks close the stride exactly,
    /// just as the symmetric form does, and they do not throw away the peak-force
    /// asymmetry the shared form sets to zero.
    static func peakInBodyWeights(contactSeconds: Double, flightSeconds: Double) -> Double {
        guard contactSeconds > 0 else { return .nan }
        return (Double.pi / 2) * (1 + flightSeconds / contactSeconds)
    }

    /// The two legs' hypothetical peaks — **an internal research value, and
    /// per leg only where the clip can resolve the contact-time difference they
    /// are built from.**
    ///
    /// # The circularity this closes
    ///
    /// `Fmax_side = (π/2)(1 + tf/tc_side)` turns the measured left/right contact
    /// difference into a per-leg FORCE SCALE, and the muscle QP is linear in the
    /// external load while unsaturated — so that scale lands inside every one of
    /// the 520 muscles' left/right comparisons. On `video_012` the contact
    /// difference is 2.899 % against a resolution floor of 10.145 %: the panel
    /// says, correctly, "left and right contact times are even to within what
    /// this clip resolves", and then the same screen showed muscle bars carrying
    /// that same unresolvable 2.9 % re-expressed as a −1.31 % force asymmetry.
    /// A difference that is not trustworthy enough to display cannot be
    /// trustworthy enough to silently scale the comparison either.
    ///
    /// Worst case for keeping it: a clip with 9 % of contact asymmetry (just
    /// under a 10 % floor, correctly refused as a timing finding) injects ≈4 %
    /// of force scale, so a muscle whose true difference is 6.5 % — below the
    /// gate, and so due to be refused — reads 10.5 % and gets published as
    /// "11 % harder on the left".
    ///
    /// So ONE timing gate governs the displayed contact-time finding and this
    /// internal hypothesis. Above the clip's resolution the timing difference
    /// may be displayed and the research plan carries separate peaks. Below it
    /// the timing difference is refused and the plan uses the peak closed on
    /// the MEAN contact. Neither regime makes the peak a product output.
    ///
    /// The stride impulse closes exactly in BOTH regimes. Shared:
    /// `Σ Fᵢ·2·tcᵢ/π = (π/2)(1 + tf/t̄c)·(2/π)(tcL + tcR) = (1 + tf/t̄c)·2·t̄c
    /// = 2(t̄c + tf) = T`.
    ///
    /// - Returns: the peaks, and whether they were shared. The flag is published
    ///   rather than inferred, because "the two legs read the same" and "this
    ///   clip refused to distinguish them" are different statements.
    static func perLegPeaksInBodyWeights(contactSeconds: Bilateral<Double>,
                                         flightSeconds: Double,
                                         resolvableAsymmetryPercent: Double)
        -> (peaks: Bilateral<Double>, sharedBetweenLegs: Bool) {
        let l = contactSeconds.left, r = contactSeconds.right
        let perLeg = contactSeconds.map { peakInBodyWeights(contactSeconds: $0,
                                                            flightSeconds: flightSeconds) }
        // A side with no contacts has no contact time; there is nothing to share
        // and the degenerate answer is the per-leg one (NaN where unknown).
        guard l.isFinite, r.isFinite, l > 0, r > 0 else { return (perLeg, false) }
        let mean = 0.5 * (l + r)
        let asymmetryPercent = 100 * (l - r) / mean
        if resolvableAsymmetryPercent.isFinite,
           abs(asymmetryPercent) >= resolvableAsymmetryPercent {
            return (perLeg, false)
        }
        let shared = peakInBodyWeights(contactSeconds: mean, flightSeconds: flightSeconds)
        return (Bilateral(left: shared, right: shared), true)
    }
}
