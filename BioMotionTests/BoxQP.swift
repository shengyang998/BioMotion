import Foundation

/// **The muscle QP, solved to machine precision instead of to OSQP's stopping
/// tolerance.** Test-only. It exists because a measurement needs an instrument
/// finer than the effect it is measuring, and the shipping solver is not one:
/// `MuscleSolver` runs OSQP at `eps_abs = eps_rel = 1e-3` with polishing off and
/// ACCEPTS `OSQP_SOLVED_INACCURATE`, so a returned activation carries up to
/// `10·(1e-3 + 1e-3) = 0.02` of absolute slack — and a left/right percentage
/// built from two of them carries `≈ 100·2·0.02/ā`, which at a typical `ā` is
/// tens of percentage points.
///
/// # The problem
///
///     min ½ aᵀ(εI + λAᵀA)a − λ τᵀA a      s.t.  aMin ≤ a ≤ aMax
///
/// exactly the objective `MuscleSolver.mm` builds, with `ε = 0.01` and `λ` the
/// caller's soft penalty. `A` is `R · diag(F_max · f_AL · f_FV · cos α)`; a
/// caller that wants `A = R · diag(F_max)` must hold every muscle at its own
/// `l_opt + l_Ts` with zero pennation and zero velocity, and CHECK that it got
/// what it asked for rather than assume — `MuscleSolver` owns that force model,
/// not this file.
///
/// # Why an active-set method and not a projected gradient
///
/// `εI + λAᵀA` has a condition number around `λ‖A‖²/ε ≈ 1e8` on a realistic
/// muscle set, where first-order methods crawl. But `A` has only as many rows as
/// the problem has coordinates — twelve for one pair of legs — so `AᵀA` is
/// rank-deficient by construction and the free-set solve collapses to a
/// `nDOF × nDOF` system through Woodbury:
///
///     (εI + λBᵀB)⁻¹ b = (1/ε)·(b − λBᵀ(εI + λBBᵀ)⁻¹Bb)
///
/// so each iteration costs a 12×12 Cholesky rather than an 80×80 one.
enum BoxQP {

    struct Solution {
        let activations: [Double]
        /// `max |∂f/∂a_i|` over the FREE variables plus the worst wrong-signed
        /// gradient on a bound — zero at an exact KKT point. Callers must check
        /// it; a solver that did not converge must not be read.
        let kktResidual: Double
        let iterations: Int
        let activeAtLower: Int
        let activeAtUpper: Int
    }

    /// - Parameter armsByMuscle: row-major `[nMuscles × nDOFs]`, ALREADY scaled
    ///   into force units (i.e. `R · F_max`).
    static func solve(arms: [Double], nMuscles: Int, nDOFs: Int, torques: [Double],
                      epsA: Double = 0.01, lambda: Double = 100.0,
                      lower: Double = 0.02, upper: Double = 1.0,
                      maxIterations: Int = 800) -> Solution {
        precondition(arms.count == nMuscles * nDOFs)
        precondition(torques.count == nDOFs)

        // A is nDOFs × nMuscles; `arms` is the transpose, so column m of A is
        // row m of `arms`.
        func column(_ m: Int) -> ArraySlice<Double> {
            arms[(m * nDOFs)..<((m + 1) * nDOFs)]
        }
        // g = −λ Aᵀτ
        var g = [Double](repeating: 0, count: nMuscles)
        for m in 0..<nMuscles {
            var dot = 0.0
            let col = column(m)
            for (offset, value) in col.enumerated() { dot += value * torques[offset] }
            g[m] = -lambda * dot
        }

        var a = [Double](repeating: lower, count: nMuscles)
        var atLower = [Bool](repeating: true, count: nMuscles)
        var atUpper = [Bool](repeating: false, count: nMuscles)
        var iterations = 0

        /// Solve `(εI + λB_FᵀB_F) x_F = b_F` for the current free set.
        ///
        /// # Why this needs iterative refinement, measured
        ///
        /// Woodbury returns `x = (b − Bᵀy)/ε` with `ε = 0.01`. On a real muscle
        /// set `b = λAᵀτ` is of order `1e7` while `εx` is of order `1e-2`, so the
        /// subtraction cancels about NINE decimal digits — and `(ε/λ)I + BBᵀ` is
        /// itself near-singular wherever a coordinate has almost no muscle
        /// action, so a small relative error in `y` becomes a large absolute one
        /// in `x`. The first version of this file skipped refinement and reported
        /// a KKT residual of 1.0, i.e. no solution at all.
        ///
        /// The residual `r = b − (εx + λBᵀBx)` is formed in the same precision
        /// but is SMALL, so re-solving for a correction recovers the digits the
        /// cancellation destroyed. Three passes; the caller checks KKT anyway.
        func solveFree(_ free: [Int], _ b: [Double]) -> [Double] {
            // M = B Bᵀ, nDOFs × nDOFs, over the free columns only.
            var M = [Double](repeating: 0, count: nDOFs * nDOFs)
            for m in free {
                let col = Array(column(m))
                for i in 0..<nDOFs where col[i] != 0 {
                    let ci = col[i]
                    for j in 0..<nDOFs { M[i * nDOFs + j] += ci * col[j] }
                }
            }
            for i in 0..<nDOFs { M[i * nDOFs + i] += epsA / lambda }
            let factor = choleskyFactor(M, nDOFs)

            // Woodbury with `A = εI`, `U = Bᵀ`, `C = λI`, `V = B`:
            //   (εI + λBᵀB)⁻¹ b = (1/ε)·(b − Bᵀ((ε/λ)I + BBᵀ)⁻¹ B b)
            // Checked on the 1×1 case, where it collapses to `b/(ε + λβ²)`.
            func apply(_ rhs: [Double]) -> [Double] {
                var Bb = [Double](repeating: 0, count: nDOFs)
                for (slot, m) in free.enumerated() {
                    let col = column(m)
                    let scale = rhs[slot]
                    guard scale != 0 else { continue }
                    for (offset, value) in col.enumerated() { Bb[offset] += value * scale }
                }
                let y = choleskyApply(factor, Bb, nDOFs)
                var out = [Double](repeating: 0, count: free.count)
                for (slot, m) in free.enumerated() {
                    let col = column(m)
                    var dot = 0.0
                    for (offset, value) in col.enumerated() { dot += value * y[offset] }
                    out[slot] = (rhs[slot] - dot) / epsA
                }
                return out
            }

            var x = apply(b)
            for _ in 0..<3 {
                // r = b − (εx + λBᵀ(Bx))
                var Bx = [Double](repeating: 0, count: nDOFs)
                for (slot, m) in free.enumerated() {
                    let col = column(m)
                    let scale = x[slot]
                    guard scale != 0 else { continue }
                    for (offset, value) in col.enumerated() { Bx[offset] += value * scale }
                }
                var r = [Double](repeating: 0, count: free.count)
                for (slot, m) in free.enumerated() {
                    let col = column(m)
                    var dot = 0.0
                    for (offset, value) in col.enumerated() { dot += value * Bx[offset] }
                    r[slot] = b[slot] - (epsA * x[slot] + lambda * dot)
                }
                let correction = apply(r)
                for slot in 0..<x.count { x[slot] += correction[slot] }
            }
            return x
        }

        // Warm start: the unconstrained minimiser, clamped into the box. It is
        // feasible, which is all the loop below requires, and it usually lands
        // within a few constraints of the answer.
        do {
            let all = Array(0..<nMuscles)
            let x = solveFree(all, g.map { -$0 })
            for m in 0..<nMuscles {
                a[m] = Swift.min(Swift.max(x[m], lower), upper)
                atLower[m] = x[m] <= lower
                atUpper[m] = x[m] >= upper
            }
        }

        // **The textbook primal active-set loop, and why it is not the obvious
        // one.** The first version solved on the free set, CLAMPED whatever came
        // back out of bounds, and released every wrong-signed multiplier at once.
        // Both shortcuts cycle: a clamp is not a descent step, and releasing a
        // whole batch can undo the previous iteration's work. Measured on a
        // 14-muscle problem it returned a KKT residual of 1.0.
        //
        // What follows takes a STEP toward the free-set solution, stopping at the
        // first bound it meets, and releases exactly ONE constraint per
        // iteration — the most violated. Every iteration then strictly decreases
        // a strictly convex objective over a polyhedron, so no active set can
        // repeat and the loop terminates.
        while iterations < maxIterations {
            iterations += 1
            var free: [Int] = []
            for m in 0..<nMuscles where !atLower[m] && !atUpper[m] { free.append(m) }

            var step = 0.0
            if !free.isEmpty {
                // b_F = −(g_F + λ A_Fᵀ(A_A a_A)); the εI term never couples i ≠ j.
                var bound = [Double](repeating: 0, count: nDOFs)
                for m in 0..<nMuscles where atLower[m] || atUpper[m] {
                    let col = column(m)
                    for (offset, entry) in col.enumerated() { bound[offset] += entry * a[m] }
                }
                var b = [Double](repeating: 0, count: free.count)
                for (slot, m) in free.enumerated() {
                    let col = column(m)
                    var dot = 0.0
                    for (offset, value) in col.enumerated() { dot += value * bound[offset] }
                    b[slot] = -(g[m] + lambda * dot)
                }
                let x = solveFree(free, b)

                // Longest feasible step toward x, and which constraint blocks it.
                var alpha = 1.0
                var blocking = -1
                var blockingAtLower = false
                for (slot, m) in free.enumerated() {
                    let direction = x[slot] - a[m]
                    if direction < 0, a[m] + direction < lower {
                        let candidate = (lower - a[m]) / direction
                        if candidate < alpha { alpha = candidate; blocking = m; blockingAtLower = true }
                    } else if direction > 0, a[m] + direction > upper {
                        let candidate = (upper - a[m]) / direction
                        if candidate < alpha { alpha = candidate; blocking = m; blockingAtLower = false }
                    }
                }
                alpha = Swift.max(0, Swift.min(1, alpha))
                for (slot, m) in free.enumerated() {
                    let delta = alpha * (x[slot] - a[m])
                    a[m] += delta
                    step = Swift.max(step, abs(delta))
                }
                if blocking >= 0 {
                    a[blocking] = blockingAtLower ? lower : upper
                    if blockingAtLower { atLower[blocking] = true } else { atUpper[blocking] = true }
                    continue
                }
                _ = step
            }

            // On the free-set minimiser. Release the single most violated bound.
            let gradient = objectiveGradient(a: a, arms: arms, nMuscles: nMuscles, nDOFs: nDOFs,
                                             g: g, epsA: epsA, lambda: lambda)
            var worst = 0.0
            var release = -1
            for m in 0..<nMuscles {
                if atLower[m], -gradient[m] > worst { worst = -gradient[m]; release = m }
                if atUpper[m], gradient[m] > worst { worst = gradient[m]; release = m }
            }
            let scale = Swift.max(gradientTermScale(a: a, arms: arms, nMuscles: nMuscles,
                                                    nDOFs: nDOFs, g: g, epsA: epsA,
                                                    lambda: lambda), 1e-300)
            if release < 0 || worst <= 1e-12 * scale { break }
            atLower[release] = false
            atUpper[release] = false
        }

        let gradient = objectiveGradient(a: a, arms: arms, nMuscles: nMuscles, nDOFs: nDOFs,
                                         g: g, epsA: epsA, lambda: lambda)
        // **The KKT residual has to be normalised by the terms that CANCEL, not
        // by the result.** Dividing by `max|gradient|` reads 1.0 at a perfect
        // interior solution, where every gradient component is legitimately at
        // rounding level — which is exactly what the first version of this file
        // reported and what sent a correct answer back as a failure. The scale
        // below is the largest term entering any component of `∇f`, so a residual
        // of 1e-16 means "the cancellation was complete".
        let scale = Swift.max(gradientTermScale(a: a, arms: arms, nMuscles: nMuscles,
                                                nDOFs: nDOFs, g: g, epsA: epsA, lambda: lambda),
                              1e-300)
        var residual = 0.0
        for m in 0..<nMuscles {
            if atLower[m] { residual = Swift.max(residual, Swift.max(0, -gradient[m]) / scale) }
            else if atUpper[m] { residual = Swift.max(residual, Swift.max(0, gradient[m]) / scale) }
            else { residual = Swift.max(residual, abs(gradient[m]) / scale) }
        }
        return Solution(activations: a, kktResidual: residual, iterations: iterations,
                        activeAtLower: atLower.filter { $0 }.count,
                        activeAtUpper: atUpper.filter { $0 }.count)
    }

    /// `∇f = (εI + λAᵀA)a + g`, formed through the coordinate space so it is
    /// `O(nMuscles · nDOFs)` rather than `O(nMuscles²)`.
    private static func objectiveGradient(a: [Double], arms: [Double], nMuscles: Int, nDOFs: Int,
                                          g: [Double], epsA: Double, lambda: Double) -> [Double] {
        var Aa = [Double](repeating: 0, count: nDOFs)
        for m in 0..<nMuscles {
            let value = a[m]
            guard value != 0 else { continue }
            for j in 0..<nDOFs { Aa[j] += arms[m * nDOFs + j] * value }
        }
        var out = [Double](repeating: 0, count: nMuscles)
        for m in 0..<nMuscles {
            var dot = 0.0
            for j in 0..<nDOFs { dot += arms[m * nDOFs + j] * Aa[j] }
            out[m] = epsA * a[m] + lambda * dot + g[m]
        }
        return out
    }

    /// The largest single TERM entering any component of `∇f = εa + λAᵀ(Aa) + g`.
    /// This is the denominator a stationarity check needs: the components
    /// themselves cancel to rounding at the answer, so normalising by them turns
    /// success into a residual of 1.
    private static func gradientTermScale(a: [Double], arms: [Double], nMuscles: Int, nDOFs: Int,
                                          g: [Double], epsA: Double, lambda: Double) -> Double {
        var Aa = [Double](repeating: 0, count: nDOFs)
        for m in 0..<nMuscles {
            let value = a[m]
            guard value != 0 else { continue }
            for j in 0..<nDOFs { Aa[j] += arms[m * nDOFs + j] * value }
        }
        var scale = 0.0
        for m in 0..<nMuscles {
            var dot = 0.0
            for j in 0..<nDOFs { dot += arms[m * nDOFs + j] * Aa[j] }
            scale = Swift.max(scale, epsA * abs(a[m]) + lambda * abs(dot) + abs(g[m]))
        }
        return scale
    }

    /// Dense Cholesky factorisation of a small symmetric positive-definite
    /// matrix. Falls back to a diagonally-loaded retry rather than returning a
    /// silent NaN; the caller's KKT residual catches anything that survives.
    static func choleskyFactor(_ matrix: [Double], _ n: Int) -> [Double] {
        var L = [Double](repeating: 0, count: n * n)
        var loading = 0.0
        // A scale-relative floor, so the retry means the same thing on a
        // torque-scale matrix as on a unit one.
        let scale = Swift.max((0..<n).map { abs(matrix[$0 * n + $0]) }.max() ?? 1, 1e-300)
        for attempt in 0..<10 {
            var ok = true
            for i in 0..<n {
                for j in 0...i {
                    var sum = matrix[i * n + j] + (i == j ? loading : 0)
                    for k in 0..<j { sum -= L[i * n + k] * L[j * n + k] }
                    if i == j {
                        if sum <= 0 { ok = false; break }
                        L[i * n + i] = sum.squareRoot()
                    } else {
                        L[i * n + j] = sum / L[j * n + j]
                    }
                }
                if !ok { break }
            }
            if ok { return L }
            loading = attempt == 0 ? scale * 1e-14 : loading * 100
            L = [Double](repeating: 0, count: n * n)
        }
        return L
    }

    /// Forward/back substitution against a factor from `choleskyFactor`.
    static func choleskyApply(_ L: [Double], _ rhs: [Double], _ n: Int) -> [Double] {
        var y = [Double](repeating: 0, count: n)
        for i in 0..<n {
            var sum = rhs[i]
            for k in 0..<i { sum -= L[i * n + k] * y[k] }
            y[i] = L[i * n + i] != 0 ? sum / L[i * n + i] : 0
        }
        var x = [Double](repeating: 0, count: n)
        for i in stride(from: n - 1, through: 0, by: -1) {
            var sum = y[i]
            for k in (i + 1)..<n { sum -= L[k * n + i] * x[k] }
            x[i] = L[i * n + i] != 0 ? sum / L[i * n + i] : 0
        }
        return x
    }

    static func choleskySolve(_ matrix: [Double], _ rhs: [Double], _ n: Int) -> [Double] {
        choleskyApply(choleskyFactor(matrix, n), rhs, n)
    }
}
