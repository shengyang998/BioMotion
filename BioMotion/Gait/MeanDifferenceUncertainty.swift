import Foundation

/// **The sampling half-width of a DIFFERENCE OF TWO MEANS, in percent of their
/// mean.** One implementation, because this repo has now shipped two claims
/// gated on the wrong variance and the second one was gated on the wrong
/// variance *after* the first had been fixed.
///
/// # Why this file exists
///
/// A gate has to contain the scatter of the quantity it gates. Twice it did
/// not:
///
/// 1. The per-muscle left/right claim was gated on `resolvableAsymmetryPercent`
///    — quantisation plus stride-period scatter, i.e. TIMING — while the
///    statistic was a difference of two means of per-contact activations whose
///    own scatter measured 9.47 % against a 10.145 % floor. That gap was judged
///    claim-ending and `MuscleLoad.samplingUncertaintyPercent` was added.
/// 2. The CONTACT-TIME claim, which is what survived that round, was left on
///    exactly the same wrong floor: `max(50/framesPerContact, stride-period
///    scatter)`, while the statistic is the difference of two means of ~5
///    CONTACT DURATIONS. `video_015`'s contact durations scatter by 11.144 %
///    against its 8.086 % floor. Monte-Carlo on a symmetric runner at that
///    configuration published a false finding on **25.3 %** of clips. See
///    `GaitContactClaimTests`.
///
/// The two consumers now call the same function, so a third claim cannot get
/// this wrong by being written in a different file.
///
/// # What it is
///
///     halfWidth = t(α/N, df) · √(s²_L/n_L + s²_R/n_R) · 100 / mean
///
/// `s²` is the sample variance (`n − 1`), so the interval covers the fact that
/// the scatter itself was estimated from 5 numbers. Infinite below two samples
/// on a side — one number has no scatter, and reading its absence as certainty
/// is the failure this whole file is about.
///
/// **Degrees of freedom are `min(n_L, n_R) − 1`, not Welch–Satterthwaite.**
/// That is deliberately conservative: measured on the symmetric-runner
/// Monte-Carlo, the shipped rule publishes on 2.4 % of clips against a 5 %
/// nominal, where Welch's df lands at 4.0 %. Both are pinned in
/// `GaitContactClaimTests.testTheDegreesOfFreedomChoiceIsConservativeNotNominal`
/// so the choice is visible rather than assumed, and the error is on the side
/// of refusing a claim.
///
/// **Bonferroni over `comparisons`, not Šidák**: the comparisons of one clip
/// share a pose stream and a force model, so they are positively dependent and
/// Šidák's independence assumption is not one this pipeline can support.
enum MeanDifferenceUncertainty {

    /// The half-width, as a percentage of the two samples' common mean.
    ///
    /// - Parameter comparisons: how many differences of this kind the clip
    ///   screened. `1` means "this is the only comparison being made" — true of
    ///   the contact-time claim, which is the single left/right statement the
    ///   running screen makes, and false of the ~175 muscle pairs.
    /// - Parameter alpha: the FAMILY-wise error rate, split across
    ///   `comparisons`.
    static func halfWidthPercent(left: [Double], right: [Double],
                                 comparisons: Int = 1,
                                 alpha: Double) -> Double {
        let nL = left.count, nR = right.count
        guard nL >= 2, nR >= 2 else { return .infinity }
        let m = 0.5 * (mean(left) + mean(right))
        guard m > 0 else { return .infinity }
        let standardError = (variance(left) / Double(nL) + variance(right) / Double(nR))
            .squareRoot()
        return tMultiplier(degreesOfFreedom: Swift.min(nL, nR) - 1,
                           comparisons: comparisons, alpha: alpha)
            * 100 * standardError / m
    }

    /// Two-sided Student-t multiplier at `alpha / comparisons`.
    ///
    /// **Degrees of freedom are clamped at 15**, which is what the hand-copied
    /// table this replaced did by ending there. It only binds above 16 contacts
    /// on a side — four times what a 4 s clip carries — and clamping keeps this
    /// function from ever returning something NARROWER than that table.
    static func tMultiplier(degreesOfFreedom df: Int, comparisons: Int = 1,
                            alpha: Double) -> Double {
        guard df >= 1 else { return .infinity }
        let n = Swift.max(1, comparisons)
        return StudentT.twoSidedQuantile(alpha: alpha / Double(n),
                                         degreesOfFreedom: Swift.min(df, 15))
    }

    static func mean(_ values: [Double]) -> Double {
        values.isEmpty ? 0 : values.reduce(0, +) / Double(values.count)
    }

    /// Sample variance, `n − 1` denominator. Zero for fewer than two samples,
    /// which callers must not read as "no scatter" — they check the count.
    static func variance(_ values: [Double]) -> Double {
        guard values.count > 1 else { return 0 }
        let m = mean(values)
        return values.map { ($0 - m) * ($0 - m) }.reduce(0, +) / Double(values.count - 1)
    }
}

/// Student-t quantiles, **computed rather than tabulated**.
///
/// A table works while the tail is a fixed 2.5 %. It stops working the moment
/// the level has to be corrected for how many comparisons a clip screened,
/// because that count is a property of the clip — `GaitLoadSummary` asks for
/// `α / N` with `N` around 175, i.e. the 99.986th percentile, which no
/// hand-copied table in this repo would have carried.
enum StudentT {

    /// The `t` with `P(|T| ≤ t) = 1 − alpha`, i.e. `alpha` split between the two
    /// tails.
    ///
    /// Found by bisection on `cdf`, which is monotone, so the bracket cannot be
    /// wrong — 200 halvings of `[0, 1e6]` puts the answer inside 1e-294 of its
    /// true value long before the loop ends, and the loop is bounded so a
    /// pathological input cannot hang the run.
    static func twoSidedQuantile(alpha: Double, degreesOfFreedom df: Int) -> Double {
        guard df >= 1 else { return .infinity }
        guard alpha > 0, alpha < 1 else { return alpha <= 0 ? .infinity : 0 }
        let target = 1 - alpha / 2
        var lo = 0.0, hi = 1e6
        for _ in 0..<200 {
            let mid = 0.5 * (lo + hi)
            if cdf(mid, degreesOfFreedom: df) < target { lo = mid } else { hi = mid }
        }
        return 0.5 * (lo + hi)
    }

    /// `P(T ≤ t)` for `t` real, via the regularized incomplete beta.
    static func cdf(_ t: Double, degreesOfFreedom df: Int) -> Double {
        guard df >= 1, t.isFinite else { return t > 0 ? 1 : 0 }
        let v = Double(df)
        let x = v / (v + t * t)
        let tail = 0.5 * regularizedIncompleteBeta(x, 0.5 * v, 0.5)
        return t >= 0 ? 1 - tail : tail
    }

    /// `I_x(a, b)`. Numerical Recipes' `betai`: the continued fraction converges
    /// fast on one side of `(a+1)/(a+b+2)` and the symmetry
    /// `I_x(a,b) = 1 − I_{1−x}(b,a)` covers the other.
    static func regularizedIncompleteBeta(_ x: Double, _ a: Double, _ b: Double) -> Double {
        guard x > 0 else { return 0 }
        guard x < 1 else { return 1 }
        let logBeta: Double = lgamma(a + b) - lgamma(a) - lgamma(b)
        let logPower: Double = a * log(x) + b * log(1 - x)
        let front: Double = exp(logBeta + logPower)
        return x < (a + 1) / (a + b + 2)
            ? front * continuedFraction(x, a, b) / a
            : 1 - front * continuedFraction(1 - x, b, a) / b
    }

    /// The modified Lentz evaluation of the beta continued fraction.
    private static func continuedFraction(_ x: Double, _ a: Double, _ b: Double) -> Double {
        let tiny = 1e-300, epsilon = 1e-15
        let qab = a + b, qap = a + 1, qam = a - 1
        var c = 1.0
        var d = 1 - qab * x / qap
        if abs(d) < tiny { d = tiny }
        d = 1 / d
        var h = d
        for m in 1...300 {
            let mD = Double(m), m2 = Double(2 * m)
            // Even step.
            var numerator = mD * (b - mD) * x / ((qam + m2) * (a + m2))
            d = 1 + numerator * d
            if abs(d) < tiny { d = tiny }
            c = 1 + numerator / c
            if abs(c) < tiny { c = tiny }
            d = 1 / d
            h *= d * c
            // Odd step.
            numerator = -(a + mD) * (qab + mD) * x / ((a + m2) * (qap + m2))
            d = 1 + numerator * d
            if abs(d) < tiny { d = tiny }
            c = 1 + numerator / c
            if abs(c) < tiny { c = tiny }
            d = 1 / d
            let delta = d * c
            h *= delta
            if abs(delta - 1) < epsilon { break }
        }
        return h
    }
}
