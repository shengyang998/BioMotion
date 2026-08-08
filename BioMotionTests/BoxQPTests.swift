import XCTest
@testable import BioMotion

/// `BoxQP` is an INSTRUMENT — `WrappedMomentArmLeakTests` uses it to decide
/// whether a retired product claim comes back — so it is checked against things
/// that do not share a line of code with it: a closed form, a dense direct
/// solve, and cyclic coordinate descent, which is slow, obvious and converges to
/// the unique minimiser of a strictly convex box-constrained QP from anywhere.
final class BoxQPTests: XCTestCase {

    /// `min ½(ε + λβ²)a² − λβτ·a` over `[lo, hi]`, whose unconstrained minimiser
    /// is `λβτ/(ε + λβ²)`. One variable, one coordinate, no active set.
    func testTheOneVariableCaseMatchesTheClosedForm() {
        for beta in [0.03, 0.05, -0.04] {
            for tau in [1.0, 40.0, -25.0] {
                let solution = BoxQP.solve(arms: [beta], nMuscles: 1, nDOFs: 1, torques: [tau],
                                           lower: -1e6, upper: 1e6)
                let expected = 100 * beta * tau / (0.01 + 100 * beta * beta)
                XCTAssertEqual(solution.activations[0], expected, accuracy: 1e-9 * (1 + abs(expected)),
                               "beta \(beta) tau \(tau)")
                XCTAssertLessThan(solution.kktResidual, 1e-9)
            }
        }
    }

    /// Wide bounds, so no constraint is active and the answer is the direct
    /// solve of `(εI + λAᵀA)a = λAᵀτ` — computed here by Gaussian elimination on
    /// the dense matrix, which shares nothing with `BoxQP`'s Woodbury path.
    func testAnUnconstrainedProblemMatchesADenseDirectSolve() {
        let (arms, torques, n, d) = Self.problem(seed: 7, nMuscles: 9, nDOFs: 3, scale: 60)
        let solution = BoxQP.solve(arms: arms, nMuscles: n, nDOFs: d, torques: torques,
                                   lower: -1e6, upper: 1e6)
        let direct = Self.denseSolve(arms: arms, nMuscles: n, nDOFs: d, torques: torques)
        for i in 0..<n {
            XCTAssertEqual(solution.activations[i], direct[i],
                           accuracy: 1e-7 * (1 + abs(direct[i])), "muscle \(i)")
        }
        XCTAssertEqual(solution.activeAtLower + solution.activeAtUpper, 0)
    }

    /// **Bounds that bite, on a problem coordinate descent can actually finish.**
    /// Both halves matter. The target activation alternates in sign, so muscles
    /// that "want" a negative activation are pushed onto `aMin` and the active
    /// set is non-empty by construction rather than by luck. And the moment arms
    /// are at unit scale, not muscle scale, so `εI + λAᵀA` is well conditioned
    /// and cyclic coordinate descent converges — at muscle scale it does not, and
    /// a reference that has not converged is not a reference. (Measured: at
    /// `scale = 40` after 60 000 sweeps it returned an objective of 0.00938
    /// against the active-set answer's 0.00499, and the "disagreement" was
    /// entirely its own.)
    ///
    /// Coordinate descent is a COARSE cross-check even here — it converges
    /// linearly at a rate set by the conditioning, and after 60 000 sweeps it is
    /// still ~7e-4 from the answer. So the loose vector agreement is the sanity
    /// check and the two PRECISE statements are that no feasible perturbation
    /// improves the returned point and that its objective is not beaten.
    func testABoundedProblemMatchesCoordinateDescent() {
        let (arms, torques, n, d) = Self.problem(seed: 11, nMuscles: 14, nDOFs: 4, scale: 1,
                                                 alternatingTarget: true)
        let solution = BoxQP.solve(arms: arms, nMuscles: n, nDOFs: d, torques: torques,
                                   lower: 0.02, upper: 1.0)
        let reference = Self.coordinateDescent(arms: arms, nMuscles: n, nDOFs: d, torques: torques,
                                               lower: 0.02, upper: 1.0, sweeps: 60_000)
        var worst = 0.0
        for i in 0..<n { worst = max(worst, abs(solution.activations[i] - reference[i])) }
        let ours = Self.objective(arms: arms, nMuscles: n, nDOFs: d, torques: torques,
                                  a: solution.activations)
        let theirs = Self.objective(arms: arms, nMuscles: n, nDOFs: d, torques: torques,
                                    a: reference)
        let perturbed = Self.perturbedObjectiveIsNeverLower(
            arms: arms, nMuscles: n, nDOFs: d, torques: torques, a: solution.activations,
            lower: 0.02, upper: 1.0)
        print("BOXQP-METRIC bounded worst_gap=\(worst) perturbation=\(perturbed) "
              + "kkt=\(solution.kktResidual) "
              + "at_lower=\(solution.activeAtLower) at_upper=\(solution.activeAtUpper) "
              + "iterations=\(solution.iterations) objective_ours=\(ours) objective_cd=\(theirs) "
              + "solution=\(solution.activations.map { ($0 * 1e6).rounded() / 1e6 })")
        XCTAssertGreaterThan(solution.activeAtLower + solution.activeAtUpper, 0,
                             "this case has to have an active set, or it tests nothing new")
        XCTAssertLessThan(worst, 1e-2, "coordinate descent's own convergence limit here")
        XCTAssertLessThan(solution.kktResidual, 1e-9)
        XCTAssertLessThanOrEqual(ours, theirs + 1e-9 * abs(theirs))
        XCTAssertLessThanOrEqual(perturbed, 1e-9 * abs(ours))
    }

    /// **The case the real rig is.** Eighty muscles, twelve coordinates, moment
    /// arms and forces at the magnitudes `FullBody.osim` produces, and a
    /// rank-deficient `BBᵀ` — which is what made the first version of `BoxQP`
    /// return a KKT residual of 1.0. The objective is compared as well as the
    /// vector, because on a flat, ill-conditioned problem two nearly-equal
    /// objectives can sit at visibly different points and only the objective
    /// says which one is the minimiser.
    func testItSolvesAnIllConditionedMuscleSizedProblem() {
        let (arms, torques, n, d) = Self.problem(seed: 3, nMuscles: 80, nDOFs: 12, scale: 90,
                                                 deadCoordinates: [5, 11])
        let solution = BoxQP.solve(arms: arms, nMuscles: n, nDOFs: d, torques: torques,
                                   lower: 0.02, upper: 1.0)
        let reference = Self.coordinateDescent(arms: arms, nMuscles: n, nDOFs: d, torques: torques,
                                               lower: 0.02, upper: 1.0, sweeps: 40_000)
        let ours = Self.objective(arms: arms, nMuscles: n, nDOFs: d, torques: torques,
                                  a: solution.activations)
        let theirs = Self.objective(arms: arms, nMuscles: n, nDOFs: d, torques: torques,
                                    a: reference)
        let perturbed = Self.perturbedObjectiveIsNeverLower(arms: arms, nMuscles: n, nDOFs: d,
                                                            torques: torques, a: solution.activations,
                                                            lower: 0.02, upper: 1.0)
        var worst = 0.0
        for i in 0..<n { worst = max(worst, abs(solution.activations[i] - reference[i])) }
        print("BOXQP-METRIC ill_conditioned kkt=\(solution.kktResidual) "
              + "iterations=\(solution.iterations) at_lower=\(solution.activeAtLower) "
              + "at_upper=\(solution.activeAtUpper) worst_gap=\(worst) "
              + "objective_ours=\(ours) objective_cd=\(theirs) "
              + "worst_perturbation_improvement=\(perturbed)")
        XCTAssertLessThan(solution.kktResidual, 1e-9)
        XCTAssertLessThanOrEqual(ours, theirs + 1e-6 * abs(theirs),
                                 "an active-set solve must not be beaten by coordinate descent")
        // On a flat ill-conditioned problem two nearly-equal objectives can sit
        // at visibly different points, so the vector gap is reported and the
        // OPTIMALITY is asserted directly: no feasible perturbation improves it.
        XCTAssertLessThanOrEqual(perturbed, 1e-9 * abs(ours))
    }

    /// Direct optimality check: over many feasible perturbations of the returned
    /// point, the largest objective IMPROVEMENT anyone finds. Zero at the
    /// minimiser. Independent of every solver in this file.
    static func perturbedObjectiveIsNeverLower(arms: [Double], nMuscles: Int, nDOFs: Int,
                                               torques: [Double], a: [Double],
                                               lower: Double, upper: Double) -> Double {
        let base = objective(arms: arms, nMuscles: nMuscles, nDOFs: nDOFs, torques: torques, a: a)
        var state: UInt64 = 20260809
        func next() -> Double {
            state = state &* 6364136223846793005 &+ 1442695040888963407
            return Double((state >> 11) % 1_000_000) / 1_000_000 - 0.5
        }
        var best = 0.0
        for magnitude in [1e-1, 1e-2, 1e-3, 1e-4, 1e-6] {
            for _ in 0..<60 {
                var candidate = a
                for m in 0..<nMuscles {
                    candidate[m] = Swift.min(Swift.max(a[m] + next() * magnitude, lower), upper)
                }
                let value = objective(arms: arms, nMuscles: nMuscles, nDOFs: nDOFs,
                                      torques: torques, a: candidate)
                best = Swift.max(best, base - value)
            }
        }
        return best
    }

    // MARK: - Helpers

    /// A deterministic pseudo-random problem at muscle magnitudes. `scale` is the
    /// force-unit size of a moment-arm entry (`R · F_max`, i.e. metres × newtons).
    /// `deadCoordinates` get a near-zero row so `BBᵀ` is rank-deficient, which is
    /// the condition the real model produces at coordinates almost no muscle
    /// spans.
    static func problem(seed: UInt64, nMuscles: Int, nDOFs: Int, scale: Double,
                        deadCoordinates: [Int] = [], alternatingTarget: Bool = false)
        -> (arms: [Double], torques: [Double], nMuscles: Int, nDOFs: Int) {
        var state = seed &* 6364136223846793005 &+ 1442695040888963407
        func next() -> Double {
            state = state &* 6364136223846793005 &+ 1442695040888963407
            return Double((state >> 11) % 1_000_000) / 1_000_000 - 0.5
        }
        var arms = [Double](repeating: 0, count: nMuscles * nDOFs)
        for m in 0..<nMuscles {
            // Each muscle spans one half of the coordinates, as a leg muscle does.
            let half = (m % 2) * (nDOFs / 2)
            for j in half..<(half + nDOFs / 2) {
                var value = next() * 2 * scale
                if deadCoordinates.contains(j) { value *= 1e-7 }
                arms[m * nDOFs + j] = value
            }
        }
        var torques = [Double](repeating: 0, count: nDOFs)
        for m in 0..<nMuscles {
            let target = alternatingTarget ? (m % 3 == 0 ? -0.4 : 0.5) : 0.3
            for j in 0..<nDOFs { torques[j] += arms[m * nDOFs + j] * target }
        }
        return (arms, torques, nMuscles, nDOFs)
    }

    static func objective(arms: [Double], nMuscles: Int, nDOFs: Int, torques: [Double],
                          a: [Double], epsA: Double = 0.01, lambda: Double = 100) -> Double {
        var residual = [Double](repeating: 0, count: nDOFs)
        for m in 0..<nMuscles {
            for j in 0..<nDOFs { residual[j] += arms[m * nDOFs + j] * a[m] }
        }
        var value = 0.0
        for j in 0..<nDOFs {
            let e = residual[j] - torques[j]
            value += 0.5 * lambda * e * e
        }
        for m in 0..<nMuscles { value += 0.5 * epsA * a[m] * a[m] }
        return value
    }

    /// `(εI + λAᵀA)x = λAᵀτ` by Gaussian elimination with partial pivoting.
    static func denseSolve(arms: [Double], nMuscles: Int, nDOFs: Int, torques: [Double],
                           epsA: Double = 0.01, lambda: Double = 100) -> [Double] {
        var M = [Double](repeating: 0, count: nMuscles * nMuscles)
        var rhs = [Double](repeating: 0, count: nMuscles)
        for i in 0..<nMuscles {
            for j in 0..<nMuscles {
                var dot = 0.0
                for k in 0..<nDOFs { dot += arms[i * nDOFs + k] * arms[j * nDOFs + k] }
                M[i * nMuscles + j] = lambda * dot + (i == j ? epsA : 0)
            }
            var dot = 0.0
            for k in 0..<nDOFs { dot += arms[i * nDOFs + k] * torques[k] }
            rhs[i] = lambda * dot
        }
        for col in 0..<nMuscles {
            var pivot = col
            for row in (col + 1)..<nMuscles
            where abs(M[row * nMuscles + col]) > abs(M[pivot * nMuscles + col]) { pivot = row }
            if pivot != col {
                for k in 0..<nMuscles {
                    M.swapAt(col * nMuscles + k, pivot * nMuscles + k)
                }
                rhs.swapAt(col, pivot)
            }
            let diagonal = M[col * nMuscles + col]
            for row in (col + 1)..<nMuscles {
                let factor = M[row * nMuscles + col] / diagonal
                guard factor != 0 else { continue }
                for k in col..<nMuscles { M[row * nMuscles + k] -= factor * M[col * nMuscles + k] }
                rhs[row] -= factor * rhs[col]
            }
        }
        var x = [Double](repeating: 0, count: nMuscles)
        for row in stride(from: nMuscles - 1, through: 0, by: -1) {
            var sum = rhs[row]
            for k in (row + 1)..<nMuscles { sum -= M[row * nMuscles + k] * x[k] }
            x[row] = sum / M[row * nMuscles + row]
        }
        return x
    }

    /// Cyclic coordinate descent with exact per-coordinate minimisation, which is
    /// monotone and converges to the unique minimiser of a strictly convex
    /// box-constrained QP. Slow on purpose: it is the reference, not the solver.
    static func coordinateDescent(arms: [Double], nMuscles: Int, nDOFs: Int, torques: [Double],
                                  lower: Double, upper: Double, sweeps: Int,
                                  epsA: Double = 0.01, lambda: Double = 100) -> [Double] {
        var a = [Double](repeating: lower, count: nMuscles)
        // Aa in coordinate space, updated incrementally.
        var Aa = [Double](repeating: 0, count: nDOFs)
        for m in 0..<nMuscles {
            for j in 0..<nDOFs { Aa[j] += arms[m * nDOFs + j] * a[m] }
        }
        var diagonal = [Double](repeating: 0, count: nMuscles)
        for m in 0..<nMuscles {
            var norm = 0.0
            for j in 0..<nDOFs { norm += arms[m * nDOFs + j] * arms[m * nDOFs + j] }
            diagonal[m] = epsA + lambda * norm
        }
        for _ in 0..<sweeps {
            var moved = 0.0
            for m in 0..<nMuscles {
                guard diagonal[m] > 0 else { continue }
                // ∂f/∂a_m = ε a_m + λ Σ_j A_mj (Aa_j − τ_j)
                var gradient = epsA * a[m]
                for j in 0..<nDOFs { gradient += lambda * arms[m * nDOFs + j] * (Aa[j] - torques[j]) }
                let candidate = min(max(a[m] - gradient / diagonal[m], lower), upper)
                let delta = candidate - a[m]
                guard delta != 0 else { continue }
                for j in 0..<nDOFs { Aa[j] += arms[m * nDOFs + j] * delta }
                a[m] = candidate
                moved = max(moved, abs(delta))
            }
            if moved < 1e-15 { break }
        }
        return a
    }
}
