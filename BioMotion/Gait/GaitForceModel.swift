import Foundation

/// Peak vertical ground reaction force during stance, from contact and flight
/// TIMING alone, expressed in body weights.
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
/// flight time to contact time. That is why this module can produce a force
/// number from a hand-held tracking shot at all, and why it publishes body
/// weights rather than newtons.
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
/// one more reason this module's headline is a ratio and not a newton figure.
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
}
