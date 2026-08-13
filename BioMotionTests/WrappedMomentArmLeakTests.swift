import XCTest
@testable import BioMotion

/// **Does the moment-arm error that survives path wrapping still leak into a
/// LEFT/RIGHT comparison?** This is the re-measurement the retirement of the
/// per-muscle claim registered as its own falsifier, run on REAL geometry
/// instead of a three-muscle rig.
///
/// # What the retirement rested on, and what has changed under it
///
/// `MomentArmErrorCancellationTests` retired the claim on two halves. The first
/// — the QP is linear in `τ`, so a proportional right leg makes any moment-arm
/// perturbation cancel by identity — is algebra and has not changed. The second
/// is a MEASUREMENT: give the right leg a different torque SHAPE and a
/// bilateral `×0.6` moment-arm error moves a published figure by **9.92 pp**,
/// against publication floors of 8.086 % / 10.145 %.
///
/// `×0.6` was a stand-in for "this muscle's path is a straight line where the
/// real one wraps around bone", and on 2026-08-08 that stopped being what the
/// code does: cylinder and then ellipsoid wrapping shipped, all 76 `PathWrap`
/// references in `FullBody.osim` are solved, and the residual against OpenSim
/// 4.6 is millimetres rather than the 146.6 mm the straight line was out by.
/// The 9.92 pp is therefore a measurement of a defect that no longer exists. It
/// is NOT evidence about the defect that does.
///
/// # The rig, and why it is not the three-muscle one
///
/// `MomentArmErrorCancellationTests` perturbs one synthetic muscle by a number
/// somebody chose. This file perturbs nothing: it takes the moment arms this
/// build actually computes, and the moment arms OpenSim computes for the same
/// model at the same pose, and solves the SAME QP twice.
///
/// * **Geometry** — the 40 right-leg muscles of `FullBody.osim` that span at
///   least one of the six unlocked right-leg coordinates, at every non-arm pose
///   the shared `WrapValidationHarness` sweep visits.
/// * **The bilateral structure** — the left leg is the right leg's exact
///   MIRROR: same muscles, same moment arms, acting on the six left
///   coordinates. That is what makes the modelling error bilateral BY
///   CONSTRUCTION, which is the case the cancellation argument was about. A
///   real stride samples the two legs at two poses; that is a different and
///   larger effect (the one-sided bound, 23.8 pp with the old arms) and this
///   file does not measure it.
/// * **The asymmetry** — the left leg's joint torques are the right's with a
///   per-joint scale, so the two legs differ in the SHAPE of their torque and
///   not in its size. Joint torques come from inverse dynamics, which never
///   touches a moment arm, so `τ` is IDENTICAL in the two solves. The only
///   thing that differs is `A`.
/// * **The statistic** — `100·(a_l − a_r)/mean`, the panel's own.
/// * **The force scale** — every muscle is held at its own `l_opt + l_Ts` with
///   zero pennation and zero velocity, so `f_AL = f_FV = cos α = 1` and the QP's
///   `A` is exactly `R · diag(F_max)`. That is the same convention the
///   three-muscle rig uses, and it is what makes the objective reproducible
///   outside `MuscleSolver` — see the next section.
///   `testTheForceScaleIsExactlyFmaxAtTheseLengths` checks it against the
///   solver's own returned forces rather than assuming it.
///
/// # THE INSTRUMENT: the shipping solver was not fine enough to measure this,
/// and this file is what showed it and then what fixed it
///
/// The first run of this file took its maximum over every cell and reported a
/// 54 pp leak on a cell with 9 readable muscles. That number was not a
/// moment-arm effect. `MuscleSolver` ran OSQP with Ruiz scaling on, polishing
/// off and a 200-iteration cap, and its answer sat a median of 14.88 pp from the
/// exact minimiser of the SAME objective. The three-muscle rig did not see it —
/// its QP is small enough that OSQP solves it accurately — and neither did
/// anything else in this project. Since 2026-08-09 (`scaling = 0`,
/// `polishing = 1`) that quantity is **4.4994e-05 pp**, and the three-way split
/// below is what proved it: `solverSlack` collapsed while `leakExact` stayed
/// bit-identical.
///
/// So this file solves the SAME objective a second time, to machine precision,
/// with `BoxQP` (an active-set solver using Woodbury on the twelve coordinate
/// rows), and reports THREE quantities rather than one:
///
/// * `leakExact` = `|d(ours, exact) − d(truth, exact)|` — the moment arms alone.
/// * `solverSlack` = `|d(ours, OSQP) − d(ours, exact)|` — the shipping solver
///   alone, at fixed geometry.
/// * `leakShipped` = `|d(ours, OSQP) − d(truth, exact)|` — what the product's
///   published number would actually be out by, both causes together.
///
/// # Pre-registered gates — v1 (2026-08-08). R1/R2/R5/R6/R7 were RE-REGISTERED
/// as v2 on 2026-08-13; that registration is the "# R1 v2" section below and it
/// is the one in force. This section is kept because a registration that is
/// silently repointed is not a registration.
///
/// `floor` is read from the pinned clips, not copied: the smallest
/// `resolvableAsymmetryPercent` any usable pinned clip achieves (8.086 % at the
/// time of writing). Each quantity is maximised over the screened bases, the
/// pose set, the torque-shape sweep and the effort sweep.
///
/// **v1 additionally maximised every quantity over BOTH definitions of the
/// OpenSim reference, and THAT is the clause v2 replaces** — quoted here rather
/// than deleted: "Each quantity is maximised over … BOTH definitions of the
/// OpenSim reference." Measured (sixth-stage section below, and
/// `testTheReferenceDisagreesWithItselfByMoreThanTheGateAllows`), the two
/// columns disagree with EACH OTHER by 126.44 pp — more than R1's own worst of
/// 123.08 pp and 78× the bar — so R1's maximum was dominated by a term
/// containing no BioMotion geometry. Replaced rather than annotated: a gate
/// whose binding term is outside the code under test is not conservative, it is
/// unfalsifiable, and the flag doc itself recorded that it could not be passed
/// as registered.
///
/// **REOPEN** — flip `GaitLoadSummary.perMuscleLeftRightClaimIsSupported` to
/// `true` — requires ALL of:
///
/// * **R1** `leakExact < floor / 5`. Why a fifth and not "below the floor": the
///   floor is a 95 % half-width on the statistic's RANDOM error, and the leak is
///   a BIAS that adds to it. In a normal approximation a bias of `h/5` on a
///   half-width `h = 1.96σ` moves a nominal 5 % false-positive rate to 6.8 %;
///   `h/3` moves it to 10.0 %, i.e. doubles it. A fifth is the largest bias that
///   leaves the quoted confidence roughly what it says. **The bar is unchanged
///   in v2; only the population and the `truth` it is measured against move.**
/// * **R2** `leakShipped < floor / 5` — the same bar applied to the number the
///   product would actually print, so a clean moment arm cannot be cancelled out
///   by a sloppy solve. (Amended: see L-A4.)
/// * **R3** the three-muscle rig re-run with the perturbation resized from the
///   guessed `×0.6` to the MEASURED residual also lands under `floor / 5`
///   (`MomentArmErrorCancellationTests.testTheShapeAsymmetryLeakWithTheResidualThisBuildLeaves`).
/// * **R4** `unmodelledPathWraps == 0` on both shipped models, and no muscle in
///   `GaitLoadSummary.displayNames` has an unmodelled path.
/// * **R5 — THE CONTROL.** The identical rig, driven with the STRAIGHT-LINE
///   moment arms this project shipped until 2026-08-08, leaks MORE than the
///   floor. Without it a small leak is indistinguishable from a rig too blunt to
///   detect anything.
/// * **R6** at least 20 screened bases at the worst-case cell, so the maximum is
///   taken over a population rather than over a handful, and at least 30 cells.
/// * **R7 — the claim must be INFORMATIVE as well as safe.** The retirement's
///   second half was that the regime where the error cancels is the regime where
///   every muscle reads the same number, so there is no regime that is both.
///   That is an argument about a RATIO: require the MEDIAN over the cells of
///   (spread of the true left/right figures across muscles) / (leakShipped) to
///   exceed 4. The minimum is reported too — a cell whose ratio is low is a
///   near-proportional configuration, where the rows carry little per-muscle
///   information for a reason that predates this work and is unchanged by it.
///
/// **STAYS RETIRED** if any of: a gated quantity `≥ floor` (W1); a gated
/// quantity in `[floor/5, floor)` — "comparable to the floor" is not "below" it
/// (W2); R4 fails (W3); R5 fails, because then the rig proves nothing either way
/// (W4); R7 fails (W5).
///
/// No threshold here may be adjusted after a number is read. If the measurement
/// lands between the bands, the claim stays retired and the number is reported.
///
/// # Amendments, with the mechanism that forced each
///
/// * **L-A1/L-A2, WITHDRAWN.** The first two amendments tried to rescue the
///   original instrument by discarding cells whose "solver noise floor" was
///   large and by widening the effort ladder. Both were built on a
///   mis-specified instrument: that noise floor solved the same arms under a
///   PROPORTIONAL torque and compared against the analytic `100(c−1)/(0.5(1+c))`,
///   which is the right answer only while NO activation sits on a bound. In an
///   80-muscle rig most muscles sit on `aMin`, and a clamped muscle's
///   contribution does not scale with `τ`, so the interior muscles compensate
///   differently at different torque scales. The 18–45 pp it reported was the
///   ACTIVE SET, not the solver — a genuine nonlinearity of the QP, and not
///   evidence about anything this file measures. Both amendments are withdrawn
///   and replaced by L-A4. The effort ladder stays widened, which costs nothing.
/// * **L-A3 — a sign error in that instrument, found on the way.** It compared
///   against `+100(1−c)/(0.5(1+c))` while this rig scales the LEFT leg, so the
///   analytic answer is negative. It reported ~44 pp of pure sign.
/// * **L-A4 — the solver's contribution is now measured, not estimated.** R2 was
///   registered as `leak + 2·noise < floor` with `noise` from the instrument
///   above. With that instrument withdrawn, R2 becomes a DIRECT measurement
///   against a machine-precision solve of the same objective, at the same
///   `floor/5` bar as R1. That is strictly stronger than what was registered:
///   the original admitted a leak up to `floor` and this one does not.
///
/// # WHAT IT MEASURED — 2026-08-09, 582 readable cells, RE-RUN after the solver fix
///
/// **The claim stays retired, and only one of its two causes is left.**
///
/// * **The wrap solver did what it was for.** Median moment-arm leak
///   **0.977 pp** against the straight line's **7.939 pp** on the identical rig
///   — 8.1× — and the three-muscle rig re-run with the perturbation resized from
///   the guessed `×0.6` to the measured p99 residual (1.114 %) reads **1.4022 pp**
///   where it read **9.92 pp**. R3 passes.
/// * **The solver is no longer a cause.** `MuscleSolver` used to run OSQP with
///   Ruiz scaling on, polishing off and 200 iterations, and its answer differed
///   from the exact minimiser of the SAME objective by a median of 14.88 pp
///   (p90 37.83, max 100.98). With `scaling = 0` and `polishing = 1` the same
///   measurement reads **median 4.4994e-05 pp, p90 0.0471, max 21.98**, and the
///   median relative torque residual falls from 2.7995e-03 to **8.316e-09**.
///   Every number below that involves OSQP moved with it; every number that does
///   not — `leakExact`, the control — is bit-identical to the earlier run, which
///   is the check that the change touched only the solve.
/// * **R1 still fails, unchanged and untouched.** Worst moment-arm leak
///   **123.10 pp** against the central-difference reference (p99 104.54, median
///   7.09); against the analytic reference alone, max 42.46 / p99 9.94 / median
///   0.41. This is now the ONLY reason the rows are withheld.
/// * **R2 still fails, and now it is R1 wearing a different hat**: worst printed
///   number **108.58 pp**, at the same cell and the same muscle as R1's worst,
///   with only 14.52 pp of solver slack there. The MEDIAN printed-number leak
///   fell from 14.42 pp to **1.045 pp**, which is inside the 1.617 pp bar — so
///   the typical cell would now pass and the tail is what fails.
/// * **R7 now PASSES**: median spread-over-error **47.52** against the required
///   4 (it was 2.90), and 48.52 against the moment-arm error alone. The
///   retirement's second half — "the regime where the error cancels is the regime
///   where every muscle reads the same number, so there is no regime that is both
///   safe and informative" — is fully defeated: the rows carry ~48× more
///   per-muscle signal than the error in them.
/// * **R4 and R5 and R6 pass**: 0 unmodelled `PathWrap`s on both models, the
///   straight-line control leaks 66.88 pp (271 of 549 cells over the floor), 582
///   readable cells.
///
/// # RE-RUN 2026-08-09 (fourth stage): every gated number is bit-identical, and
/// the tail is now ATTRIBUTED
///
/// Same 582 cells, same protocol, same thresholds — R1 123.09713008193307 pp,
/// R2 108.57519173214942 pp, R7 47.52166243963286, control 66.8824128835621,
/// medians 0.977 / 1.045, to the last stored digit. Two DIAGNOSTICS were added
/// (nothing gated changed, and that identity is the proof):
///
/// 1. **`Cell.worstExactBase`.** `worstBase` is the base of the worst SHIPPED
///    leak, and it was being printed beside `leakExact` as if it were R1's
///    muscle. It is not the same maximum, and the two parted company the moment
///    the solver stopped contributing — which is how STATUS.md came to record
///    R1's worst as `piri` and `glmed3`, muscles that carry **no `PathWrap` at
///    all**. R1's worst is on **`bflh140`**, at `grid_h060_k000_a+00` against
///    the central difference and at `run_4_mid_swing` against the analytic
///    column.
/// 2. **The other reference is swept as a SUBJECT** — same τ, same truth solve,
///    same statistic — so "how far apart are OpenSim's own two answers" is a
///    number on R1's scale. It is **126.44 pp** worst, a paired median of
///    **5.28 pp** against our **0.977 pp**, and **our leak is the SMALLER of the
///    two in 466 of 582 cells**.
///
/// **So R1, as registered, is not a measurement of this codebase.** It is
/// maximised over the worse of two `truth` definitions that differ from each
/// other by more than R1's own worst value and by 78× the reopening bar. No work
/// on `MomentArmComputer` can pass it while the gate is taken that way.
///
/// **And `bflh140` carries no `PathWrap`, no `MovingPathPoint` and no row in the
/// finite-difference fixture** — so its moment arm is the SAME NUMBER in the
/// `analytic` and `centralDifference` matrices by construction, and it still
/// moves 126.44 pp between them. The tail is not a path error on the muscle that
/// shows it: it is the SHARING STEP redistributing a neighbour's error onto a
/// muscle whose own path is exact. `bflh140` is a hamstring — hip extensor and
/// knee flexor — sharing the knee with `gasmed`/`gaslat140`, the two muscles the
/// central-difference column is measurably wrong about (`MultiWrapReferenceTests`:
/// 10.5 and 11.0 mm median against that column, 0.033 and 0.035 mm against it
/// reconciled).
///
/// Before the endpoint-extrapolation fix, the ordering of causes on the same rig
/// and statistic was:
/// **the reference against itself 5.28 pp median / 126.44 worst**, our arms
/// against OpenSim's better-founded column 0.41 / 42.46, the sharing step
/// 4.5e-05 / 21.98. The largest term is no longer inside this repository — but
/// 42.46 pp against the analytic column is still 26× the bar, so the claim does
/// not come back on that argument either.
///
/// # RE-RUN 2026-08-09 (fifth stage): the analytic tail is on `bflh140`'s own row
///
/// The previous stage proved that `bflh140`'s arms are identical across all four
/// sources at the CENTRAL-DIFFERENCE worst cell, then inferred that the separate
/// 42.46 pp ANALYTIC tail must arrive from a synergist. Printing every screened
/// row at that analytic cell rejects the inference. At `run_4_mid_swing`, the
/// largest arm error among all 24 screened muscles is `bflh140_r`'s own
/// `knee_angle_r`: ours **16.059 mm**, analytic **13.713 mm**, a **+2.346 mm**
/// gap. Its figure moves −42.462 pp. The next row is `gaslat140_r` at 1.597 mm;
/// no unnamed neighbour carries a larger discrepancy.
///
/// This does not explain WHY three fixed path points disagree at that pose. It
/// localises the next question away from wrapping and onto the kinematic/path
/// derivative seam. The central-difference cell still proves the separate
/// sharing-step result: an exact row can move when its neighbours change.
///
/// # RE-RUN 2026-08-09 (sixth stage): the 42.46 pp tail was endpoint-cubic extrapolation
///
/// `walker_knee_r` permits 140° of flexion, while its five nonlinear
/// `SimmSpline` transform axes have knots only through 120°. Nimble had OpenSim's
/// endpoint-tangent branches commented out and continued the final cubic at the
/// 130° `run_4_mid_swing` pose. A two-sided low-level regression first failed on
/// value, first derivative and second derivative; restoring both branches makes
/// `bflh140_r` read **13.713464915 mm** versus OpenSim's **13.713465000 mm**.
///
/// The analytic-only maximum falls **42.4623 → 3.6932 pp** (p99 9.94 → 3.332,
/// median 0.412 → 0.312). It is now `glmax2` at `grid_h090_k000_a+00`; the
/// largest arm discrepancy in that cell is a different muscle, `gasmed`, at
/// 1.047 mm. That is a 91.3% reduction, but 3.6932 pp is still 2.28× the
/// unchanged 1.617 pp reopening bar. The claim therefore remains retired, and
/// the separate 123.08 pp central-difference/reference tail still dominates R1.
///
/// # R1 v2 — FROZEN 2026-08-13, BEFORE ANY NEW NUMBER WAS READ
///
/// One truth, a 2×2 population that deletes nothing, and no reopening
/// authority. Nothing in this section may be edited after the run starts (§7).
/// **Predicted verdict: FAIL — the claim stays retired (§8).**
///
/// ## 0. Why R1 was re-registered
///
/// v1's `sweep()` used ONE `reference` variable for three roles: the LOADING
/// (the τ source), the TRUTH (what the leak is measured against) and the
/// EXCLUSION (`subject != reference`). `readable(_:in:)` filters on `subject`
/// only, so both reference families landed in one array and the unfiltered
/// `.max()` inside `supported` collapsed them. Record at freeze time: registered
/// R1 max **123.0833 pp** (median 0.6571), registered R2 max **108.5576 pp**,
/// analytic-only max/p99/median **3.6932 / 3.3322 / 0.3121 pp**. Sweeping the
/// other column as a SUBJECT on the identical rig puts the two columns' mutual
/// disagreement at **126.44 pp** — larger than R1's own worst and 78× the bar.
/// R1's worst cell, `bflh140`, carries three fixed path points, no `PathWrap`
/// and no FD row, so `value(_:_:)` returns the SAME stored number for both
/// references there; its printed figure moves anyway, because the QP sharing
/// step redistributes a neighbour's reference disagreement onto an exact row.
///
/// **Two numbers, not one.** 126.44 pp is the REFERENCE SELF-DISAGREEMENT;
/// 123.0833 pp is R1's OWN worst. Same muscle, same pose family, different
/// measurements. This registration never merges them.
///
/// **Correction carried into this text.** The widely-quoted "our leak is the
/// smaller of the two in 466 of 582 cells" is (a) a PRE-SimmSpline count — the
/// fix moved `pairedOurs`' median 0.9770 → 0.6571, so the count moved too,
/// unmeasured — and (b) the COMPLEMENT of the field the instrument prints,
/// `cells_where_ours_exceeds_the_reference`. This registration cites no such
/// count; the verdict reader reads that field as printed, under its own name.
///
/// ## 1. TRUTH DEFINITION
///
/// **The reference moment arm for every gated statistic (R1, R2, R1-M, R5, R6,
/// R7) is OpenSim 4.6's ANALYTIC column and nothing else:**
/// `GeometryPath::computeMomentArm` with all 76 `PathWrap`s solved — the `on`
/// column of `Fixtures/opensim_moment_arms.txt` (`poses 173`, `muscles 104`),
/// surfaced as `WrapValidationHarness.Sample.wrapOn`, selected by
/// `ArmSource.analytic`, registered as `registeredTruth`.
///
/// ### 1.2 The population: the FULL 2×2 grid, so nothing is deleted
///
/// A post-hoc filter on `reference == .analytic` would silently delete an entire
/// LOADING family and re-key every instrument. So the sweep is re-parameterised
/// into independent axes: `loading ∈ {.analytic, .centralDifference}` ×
/// `truth ∈ {.analytic, .centralDifference}`, with the subject loop preserved
/// verbatim as `for subject in ArmSource.allCases where subject != truth`. Four
/// registered consequences:
///
/// 1. **The v1 population survives byte-for-byte as the diagonal `loading ==
///    truth`.** τ depends only on `(pose, loading, effort, shape)`, the truth
///    solve on `(pose, truth, τ)`, and the exclusion is v1's.
/// 2. **No structurally-zero leg can enter any statistic** — `subject != truth`
///    is preserved, so analytic-vs-analytic (leak ≡ 0) is never built.
/// 3. **The gated population is `truth == .analytic`, BOTH loading families** —
///    the v1 analytic half plus a never-before-computed CD-loading half.
///    Expected ≈582 gated `.ours` cells, preserving R6's magnitude.
/// 4. **Runtime roughly doubles.** Accepted as the price of breaking no v1 leg.
///    (Implementation note, not a registration change: `solve` is a pure
///    function of `(pose, source, τ)`, so one pose's four (loading, truth)
///    pairs share their solves — the 2×2 costs the same QP count as v1 and the
///    diagonal cells are computed in v1's own order. `V1-CONT` is what proves
///    the cells did not move.)
///
/// Every instrument keeps the population it was validated on: **R1-P on the v1
/// diagonal**, **I2 on the v1 diagonal**, **R7's pooled leg on the v1
/// diagonal**, and the v1 gate quantity itself (max over both truths) stays
/// computed and printed as `V1-CONT`. It is **not** a gate.
///
/// ### 1.3 Why the analytic column
///
/// It never calls `calcLengthAfterPathComputation`, the routine whose
/// bookkeeping is broken for two-cylinder paths, so it is blind BY CONSTRUCTION
/// to the one PROVEN reference defect.
///
/// * **REJECTED — `max(analytic, centralDifference)`, the v1 registration.** §0.
/// * **REJECTED — `centralDifference` alone.** Its `L` is not a path length for
///   muscles engaging two cylinders: `gasmed_r` at knee 0° stores a spiral of
///   0.038054 m against a CHORD between its own tangent points of 0.045350 m;
///   88 of 440 sampled multi-wrap rows carry negative slack, worst −7.2957 mm;
///   the consequence is a systematic ~10–11 mm arm error across the running knee
///   range on `gasmed`/`gaslat140`. Its own `L(q)` steps 6.2 µm over 0.0005°
///   with no wrap-point-count change, so a centred stencil can manufacture a
///   41.26 mm excursion. And for the 38 named muscles the FD fixture does not
///   cover — including `bflh140`, carrier of R1's v1 worst — it silently returns
///   the analytic value. **This rejection is derived from the defect mechanism
///   and is logically independent of R1's value.**
/// * **NOT REJECTED — the `reconciled` column.** Two earlier rejection reasons
///   are WITHDRAWN as factually wrong: it does NOT miss R1's tail (the analytic
///   worst cell's largest arm discrepancy is `gasmed` at 1.047 mm, and `gasmed`
///   is one of the 8 muscles `opensim_multiwrap.txt` covers — dismissing it via
///   the figure-mover `glmax2` would repeat the `piri`/`glmed3` attribution
///   error), and it is NOT missing a derivative (`MultiWrapReferenceTests.mm`
///   already differentiates it and gates that at M2/M3). The true, narrower
///   reason it is not today's anchor is GRID COVERAGE: it exists only on
///   `opensim_multiwrap.txt`'s own ±eps triplet grid over 8 of 104 muscles, not
///   on the shared 173-pose grid. Registered as **R9, FUTURE WORK** — scheduled,
///   not dismissed, and not built this session.
/// * **NOT AVAILABLE — a reference wholly outside OpenSim's lineage.** This
///   port is an Apache-2.0 port of the same `opensim-core` routines, so analytic
///   agreement is PARTIAL corroboration. Owner-level; deferred.
///
/// ### 1.4 The strongest objection, answered
///
/// *"`centralDifference` is the column definition-matched to a `−dL/dq`
/// implementation; you picked the mismatched one."* A number previously used
/// here is **withdrawn**: "median 0.000 mm / max 3.569 mm over 11,760 pairs" is
/// C1's `|ours − reference|`, not a column-vs-column agreement. **The
/// column-vs-column single-wrap agreement is measured nowhere in this
/// repository**, so no figure for it is cited; it is registered as the
/// diagnostic **M1** with a numeric falsifier trigger instead. What survives:
/// (a) definition-matching is a tiebreak between two INTERNALLY VALID columns
/// and one of these two is not internally valid on the multi-wrap class — a
/// proven defect beats a definitional preference; (b) where they disagree away
/// from that class, the FD column's own `L(q)` is measurably non-smooth, and a
/// finite difference of a non-smooth `L` is an artefact amplifier.
///
/// **Symmetry, registered so it cannot become an escape hatch.** BioMotion's own
/// subject is a finite difference too, so if `L(q)` is non-smooth at a pose OUR
/// number is a stencil artefact as well — the argument cuts both ways and is not
/// evidence for our correctness. Therefore: any claim that a failing cell sits
/// at a "marginal wrap pose" is admissible **only if** the wrap-point count
/// changes across the ε = 1e-4 rad stencil **or** the printed second difference
/// of the reference's own length exceeds 6.2 µm (`worst_cell_smoothness`).
/// Otherwise it is inadmissible. **No cell, muscle, pose, torque shape, effort
/// level, loading family or truth family may be excluded from R1/R2 on this or
/// any other basis.**
///
/// ### 1.5 The registered limit on what the analytic column can attribute
///
/// M6 (`MultiWrapReferenceTests.mm`) reads worst 1.05 mm / median 0.0005 mm
/// against bars of 0.005 m and 0.004 m p99 — **4.8× and 3.8×** the 1.047 mm arm
/// discrepancy at R1's binding cell. **Not an order of magnitude**; the earlier
/// draft inflated this in the direction that flattered the argument. So R1 v2
/// MEASURES the gap between this build and the analytic column and does **not**
/// ATTRIBUTE it. **No reference-uncertainty budget is subtracted** — that would
/// be gate-weakening. The instruments that can discriminate are **R8** (is it
/// the sharing step? — runs this session, diagnostic) and **R9** (is it the
/// reference? — future work).
///
/// ### 1.6 STATUS next-step 33, quoted and answered
///
/// > "a `WrapValidationHarness` sample carrying a `reconciled` field … would let
/// > R1 be re-registered against a single defensible truth. **Do not re-register
/// > it quietly** — R1's current value would change, and a gate whose reference
/// > is chosen after a number is read is not pre-registered."
///
/// **The charge is ACCEPTED, not rebutted.** `.analytic` is chosen with both
/// candidate values already known (123.0833 vs 3.6932). A prior draft's fix —
/// folding a constant `ownerReopenAuthorityGranted = false` into `supported` —
/// is **WITHDRAWN as itself a defect**: it would make `supported` a
/// compile-time constant, degrading `XCTAssertEqual(flag, supported)` to
/// `XCTAssertEqual(false, false)`. R1's and R2's MAX thresholds are asserted
/// NOWHERE ELSE in the suite, so a later `reopenFractionOfFloor = 20.0` would
/// land silently — that neuters the one test repo law names as un-neuterable and
/// kills the `LEAK-METRIC decision supported=` observable. The registered
/// mechanism instead:
///
/// 1. `supported` stays a function of the MEASURED gates only —
///    `R1 ∧ R2 ∧ R4 ∧ R5 ∧ R6 ∧ R7` — same six terms, same operators, same
///    constants, repointed onto the gated population with R7 in its `min(...)`
///    form. The pin stays BIDIRECTIONAL.
/// 2. The no-silent-reopen rule is a SEPARATE LOUD TEST,
///    `testReopeningThisClaimNeedsAnInstrumentChosenBeforeItsNumbersWereRead`.
/// 3. If the measured gates ever all pass, BOTH tests go RED in the flag-behind
///    direction. That is the intended surfacing mechanism.
///
/// ## 2. BAR — UNCHANGED, and registered as the FORMULA, not the numeral
///
/// `bar = floor × reopenFractionOfFloor = floor × 0.2`, `floor` read LIVE by
/// `smallestPublicationFloorOnThePinnedClips()`. Currently 8.086 % → 1.6172 pp.
/// The record is inconsistent in the fourth digit (STATUS.md writes both 1.6173
/// and 1.617), so every "< 1.6172 pp" below means `< floor × 0.2` with `floor`
/// read live and printed beside the verdict; the printed values govern. Two
/// moves are FORBIDDEN: raising it (in particular re-anchoring to
/// `GaitReport.contactClaimFloorPercent`, which is `max(...)` of this and
/// another term and therefore ≥ it by construction), and softening the
/// statistic from a MAX over cells to any percentile, trimmed max, per-cell
/// noise allowance or reference-uncertainty subtraction.
///
/// ## 3. GATES
///
/// `gated(subject) := readable(subject).filter { $0.truth == registeredTruth }`;
/// `v1(subject) := readable(subject).filter { $0.loading == $0.truth }`.
///
/// * **R1** — max `leakExact` over `gated(.ours)` `< floor × 0.2`.
/// * **R2** — max `leakShipped` over `gated(.ours)` `< floor × 0.2`.
/// * **R1-M** (newly NAMED; the two assertions were already live and are
///   re-populated) — `median(gated(.ours)) < median(gated(.straightLine)) / 3`
///   AND `median(gated(.ours)) < floor × 0.2`.
/// * **R3** — UNCHANGED, byte-for-byte, in `MomentArmErrorCancellationTests`.
///   Its perturbation is the measured p99 relative residual at **reference arms
///   ≥ 20 mm** (`minimumReferenceMetres: 0.020`), POOLED over both definitions.
///   A prior draft twice wrote "≥ 1 mm", misreading the doc's counterexample as
///   the threshold; the hazard is live because
///   `relativeMomentArmResiduals` DEFAULTS to 0.001. Pooling stays in R3 while
///   R1 drops it, and that is not inconsistent: R1 asks "how far is our number
///   from the truth?" (needs one truth; maximising over two inconsistent
///   candidates measures their disagreement), R3 asks "if an arm is off by a
///   plausible-worst residual, how far does a published figure move?" (the two
///   columns are two upper-bound estimates of OUR OWN residual, so the larger
///   INFLATES the perturbation). Pooling is conservative for R3 and
///   non-conservative for R1; restricting R3 would be gate-weakening.
/// * **R4** — unchanged.
/// * **R5** — max `leakExact` over `gated(.straightLine)` `>` the FULL floor.
///   **The subset-monotonicity argument is WITHDRAWN**: `gated(.straightLine)`
///   is neither a subset nor a superset of v1's control, so NO relation to
///   v1's 66.88 pp is asserted in either direction. R5 is justified on its own
///   terms — unchanged bar, unchanged assertion, and half its population has
///   never been computed.
/// * **R6** — min `screened` over `gated(.ours)` ≥ 20 AND `gated(.ours).count`
///   ≥ 30.
/// * **R7** — re-populated and STRENGTHENED: `min(median_gated, median_v1) >
///   4.0`, where the median is of `trueSpread / leakShipped`. **Correction:
///   R7's denominator is `leakShipped`, not `leakExact`** — the 0.312 vs 0.977
///   figures previously used to argue the filtering direction are `leakExact`
///   medians and 0.977 is itself pre-fix. The v1 `leakShipped` median on record
///   is 1.045 pp with ratio median 47.52; the GATED leakShipped median is
///   recorded NOWHERE, so the direction of the filtering effect is UNMEASURED —
///   which is exactly why `min(...)` is registered: it makes R7 no easier than
///   v1 under either outcome. The pooled leg is reproducible **only because the
///   2×2 keeps the v1 diagonal**; under a naive truth filter both legs would
///   collapse to one number and the strengthening would be vacuous. The
///   `min(...)` is wired into `supported` itself, not merely printed.
///
/// ### Preconditions, instruments and diagnostics (these never flip the flag)
///
/// * **R1-P** — `testTheReferenceDisagreesWithItselfByMoreThanTheGateAllows` on
///   the **v1 diagonal only**, pairing key `pose|loading|truth|shape|effort`
///   restricted to `loading == truth`: `worstDisagreement > floor × 0.2` AND
///   paired cells > 200 AND `MultiWrapReferenceTests` green. If it FAILS, no
///   verdict flips — the registration re-opens (falsifier 2).
/// * **I1** — worst KKT over ALL `.ours` cells of the full 2×2 (a strict
///   superset of v1) `< 1e-6`, and `gated(.ours).count ≥ 30`.
/// * **I2** — `solverSlack` median/p90 over **`v1(.ours)`** (byte-for-byte v1,
///   retaining the 21.981 pp outlier); the full-2×2 version printed beside it.
///   Registering on v1 avoids relying on an unproven claim about percentiles of
///   a doubled multiset.
/// * **I3** — rig preconditions, unchanged.
/// * **V1-CONT** — the v1 registered quantity, reported not gated. It must
///   reproduce max 123.0833 / median 0.6571 / R2 max 108.5576, and its analytic
///   half 3.6932 / 3.3322 / 0.3121 at `glmax2 @ grid_h090_k000_a+00`; R1-P must
///   reproduce 126.44 worst and 5.28 paired reference median. **REVIEW STEP,
///   NOT AN ASSERTION** — hardcoding an expected number into a test is
///   forbidden. If any moves, the run is VOID (falsifier 4).
/// * **M1** — `column_vs_column_singlewrap`: `|analytic − centralDifference|` in
///   mm over the rig's own rows, SINGLE-WRAP muscles only. No pass condition.
///   **Falsifier 6 FIRES if p99 > 1.047 mm or max > 10.43 mm.** Plus
///   `worst_cell_smoothness` (§1.4).
/// * **R8** — NEW, runs this session AFTER the R1 v2 measurement, DIAGNOSTIC and
///   **not** a conjunct in `supported`: perturb muscle `j`'s arm bilaterally by
///   the same measured p99 relative residual R3 uses, resolve the exact QP, and
///   record `max over i≠j` of the induced change in muscle `i`'s left/right
///   figure — one Jacobian of the QP solution map (STATUS next-step 35).
///   Reported against `floor × 0.2`; above it, per-muscle figures are not
///   attributable to per-muscle paths (falsifier 7).
/// * **D2** — conditional diagnostic, `min(stored arc − chord) ≥ 0` across the
///   wrapped muscles (STATUS next-step 30). Run only if cheap; else reported
///   NOT RUN rather than treated as absence of the defect.
/// * **R9** — FUTURE WORK, not built this session, not a conjunct.
///
/// **REOPEN** requires all of R1–R7 **plus** an instrument chosen before its
/// numbers were read (the R9 lineage) **plus** an explicit owner commit.
/// **STAYS RETIRED** if: any gated quantity ≥ floor (W1); any gated quantity in
/// `[floor/5, floor)` (W2); R4 fails (W3); R5 fails, because the rig then proves
/// nothing either way (W4); R7 fails (W5); no pre-registered confirmatory
/// instrument exists (W6); owner authority absent (W7).
///
/// ## 5. FALSIFIERS of THIS REGISTRATION
///
/// 1. The analytic column carries its own systematic defect on the muscle class
///    driving R1's tail (M6 degrading; R9 disagreeing by more than the bar on
///    R1's/R2's worst muscles; D2 implicating the reported POINTS).
/// 2. OpenSim's two columns converge — R1-P fails. Then `truth` is well-defined
///    and R1 re-registers as the max over both. No verdict flips on this alone.
/// 3. Common-mode error: this port and the analytic column share a specific
///    inherited wrap-geometry error. Then agreement is not evidence.
/// 4. `V1-CONT` or R1-P does not reproduce the recorded v1 numbers — the run is
///    VOID, not informative.
/// 5. R5 fails on `gated(.straightLine)`. Report the FAIL; do NOT restore
///    pooling.
/// 6. M1's trigger fires (p99 > 1.047 mm or max > 10.43 mm) — the FD column's
///    disagreement is not confined to the multi-wrap class.
/// 7. R8 comes back above `floor × 0.2` — per-muscle figures are not
///    attributable to per-muscle paths at this rig's coupling.
///
/// ## 6. What a PASS would, and would not, license
///
/// Exactly: *"on this bilateral rig, at these poses, torque shapes, effort
/// levels and both loading families, the left/right figures produced from this
/// build's moment arms sit within `floor/5` of the figures produced from OpenSim
/// 4.6's analytic column."* NOT "the moment arms are correct" (shared
/// Apache-2.0 lineage). NOT an attribution of the residual. NOT a bound on the
/// QP's per-muscle coupling — that is R8's job, and until R8 reports, "validate
/// a muscle's path, then trust its row" remains an invalid inference. And it
/// does not move the flag: it turns the pin RED onto a human.
///
/// ## 7. FREEZE RULE
///
/// After the first number of this run is read, the registration text, the truth
/// definition, the 2×2 population, the bar, every gate definition, statistic,
/// population and numeric falsifier trigger are FROZEN. A DEFECT found
/// afterwards may be fixed — pin it RED first, then fix, then re-run THE SAME
/// gates, and report both readings. **A FAIL verdict is a complete
/// deliverable.** `testTheShippedFlagMatchesWhatTheMeasurementSupports`,
/// `MuscleOverlayClaimTests` and `ClaimSurfaceTests` are not to be deleted,
/// weakened or made conditional. No expected number is ever hardcoded.
///
/// ## 8. PREDICTED VERDICT, stated before the run
///
/// **R1 FAILS.** `gated(.ours)` ⊇ the v1 analytic half, which contains the
/// 3.6932 pp maximum, so `R1 ≥ 3.6932 pp` against a ≈1.6172 pp bar (≥ 2.28×) —
/// a genuine superset argument, unlike the withdrawn R5 one. The
/// `loading == .centralDifference` half is unmeasured and can only raise the
/// max. **R2 FAILS**, expected in `[≈3.7, ≈25.7] pp`; the upper end is the
/// triangle bound with the v1 slack max 21.981 pp, the lower end is an
/// EXPECTATION and not a bound. R4 passes. R6 passes. **R3's only recorded
/// value, 1.4022 pp, is a PRE-SimmSpline reading with 13 % headroom and no
/// post-fix reading exists** — expected to pass, but not a safe prediction. R7
/// expected to pass. R1-M and R5 are re-populated and their gated values are
/// unrecorded. `supported` expected `false`, matching the shipped flag, so the
/// pin stays green. The binding term is the still-unattributed analytic residual
/// at `glmax2 / grid_h090_k000_a+00`, whose largest arm discrepancy belongs to
/// `gasmed` (1.047 mm) while `glmax2`'s own is 0.234 mm — i.e. it may be the
/// sharing step, which is **R8's** question, not R1's.
final class WrappedMomentArmLeakTests: XCTestCase {

    // MARK: - Pre-registered constants

    /// R1, R2. A fifth of the publication floor.
    static let reopenFractionOfFloor = 0.2
    /// **R1 v2's single `truth`.** Every GATED statistic (R1, R2, R1-M, R5, R6,
    /// R7) is measured against this column and no other. v1 maximised over BOTH
    /// of OpenSim's columns, which made its maximum a measurement of their
    /// mutual disagreement (126.44 pp) rather than of this codebase; see the
    /// "# R1 v2" section of the type doc for the full frozen registration,
    /// including the rejected alternatives and the accepted charge that this
    /// column was chosen with both candidate values already known.
    static let registeredTruth: ArmSource = .analytic
    /// R6.
    static let minimumScreenedBases = 20
    static let minimumCells = 30
    /// R7.
    static let minimumInformationToLeakRatio = 4.0
    /// How far inside the box a muscle has to be, in the EXACT solution, to be
    /// read. A muscle within a thousandth of `aMin` is one OSQP cannot tell from
    /// the bound, and the panel excludes bound muscles for the same reason.
    static let interiorMargin = 1e-3

    // MARK: - The rig

    /// The six unlocked right-leg coordinates. `mtp_angle_r` is excluded because
    /// `FullBody.osim` locks it and the reference therefore carries no value for
    /// it — a locked coordinate's `computeMomentArm` is a refusal (exactly 0.0),
    /// not a measurement.
    static let rightLegCoordinates = ["hip_flexion_r", "hip_adduction_r", "hip_rotation_r",
                                      "knee_angle_r", "ankle_angle_r", "subtalar_angle_r"]
    static let leftLegCoordinates = ["hip_flexion_l", "hip_adduction_l", "hip_rotation_l",
                                     "knee_angle_l", "ankle_angle_l", "subtalar_angle_l"]
    static let dofNames = rightLegCoordinates + leftLegCoordinates

    /// Where a moment arm comes from. The two `isReference` cases are OpenSim's
    /// two mutually-inconsistent answers for the same quantity — see the
    /// "readings that lie" entry on `computeMomentArm`.
    ///
    /// **Until 2026-08-13 this doc said "the gates are taken over the WORSE of
    /// them rather than over the flattering one", and that is what R1 v2
    /// replaced.** Taking the worse of two mutually inconsistent columns is not
    /// conservatism when they disagree with each other by more than the whole
    /// gate budget: it makes the gate's binding term a quantity containing no
    /// BioMotion geometry. Since R1 v2 the two cases are two independent AXES —
    /// `Cell.loading` (which column supplied τ) and `Cell.truth` (which column
    /// the leak is measured against) — and every GATED statistic fixes
    /// `truth == registeredTruth` while keeping BOTH loading families. The v1
    /// pooled quantity is still computed and printed as `V1-CONT`.
    enum ArmSource: String, CaseIterable {
        /// This build: `−dL/dq` with every `PathWrap` solved.
        case ours
        /// What shipped until 2026-08-08: OpenSim with every `WrapObject`
        /// deactivated, which the 2026-08-08 measurement showed this code
        /// reproduced to 4.39 mm. The CONTROL.
        case straightLine
        /// OpenSim's `GeometryPath::computeMomentArm`, wraps solved.
        case analytic
        /// OpenSim's own central difference of its own length — the column that
        /// is definition-matched to a `−dL/dq` implementation. Falls back to
        /// `analytic` for rows the finite-difference fixture does not carry,
        /// which is exactly the muscles with no `PathWrap`, where the two
        /// columns are the same number.
        case centralDifference

        var isReference: Bool { self == .analytic || self == .centralDifference }
    }

    static func value(_ sample: WrapValidationHarness.Sample, _ source: ArmSource) -> Double {
        switch source {
        case .ours: return sample.ours
        case .straightLine: return sample.wrapOff
        case .analytic: return sample.wrapOn
        case .centralDifference: return sample.centralDifference ?? sample.wrapOn
        }
    }

    // MARK: - Index over the shared sweep, built once

    /// `muscle name → coordinate → sample`, for one pose.
    typealias PoseTable = [String: [String: WrapValidationHarness.Sample]]

    private(set) static var byPose: [String: PoseTable] = [:]
    private(set) static var poseOrder: [String] = []
    private(set) static var bases: [String] = []
    private static var built = false

    /// Arm-sweep poses hold the LEGS fixed, so including them would multiply the
    /// cell count by near-copies of one leg configuration and make a maximum over
    /// cells look better sampled than it is.
    static let excludedPosePrefixes = ["elbow_sweep_", "shoulder_sweep_"]

    /// Membership is decided by the FIXTURE's structural span, never by a
    /// moment-arm magnitude, so which muscles are in the rig cannot depend on
    /// which arm source is under test.
    static func buildIndex() {
        guard !built else { return }
        built = true
        var tables: [String: PoseTable] = [:]
        let legs = Set(rightLegCoordinates)
        for sample in WrapValidationHarness.samples {
            guard legs.contains(sample.coordinate), sample.muscle.hasSuffix("_r") else { continue }
            guard !excludedPosePrefixes.contains(where: { sample.pose.hasPrefix($0) }) else {
                continue
            }
            tables[sample.pose, default: [:]][sample.muscle, default: [:]][sample.coordinate] = sample
        }
        byPose = tables
        poseOrder = tables.keys.sorted()
        var found = Set<String>()
        for table in tables.values {
            for muscle in table.keys where WrapValidationHarness.muscleParameters[muscle] != nil {
                if let split = GaitLoadSummary.split(muscle) { found.insert(split.base) }
            }
        }
        bases = found.sorted()
    }

    /// The matrices the two solvers are handed. Separate from the solve so a test
    /// can inspect the mirror rather than trust a comment about it.
    struct Built {
        let names: [String]
        /// Row-major `[names.count × 12]`, moment arms in metres.
        let arms: [Double]
        /// The same, scaled by `F_max` — the QP's own `A`, transposed.
        let armsInForceUnits: [Double]
        let lengths: [Double]
        let maxForces: [Double]
        let optimalFiberLengths: [Double]
        let tendonSlackLengths: [Double]
    }

    static func build(pose: String, source: ArmSource) -> Built? {
        guard let table = byPose[pose] else { return nil }
        var names: [String] = []
        var arms: [Double] = []
        var scaled: [Double] = []
        var lengths: [Double] = []
        var maxForces: [Double] = []
        var optimal: [Double] = []
        var slack: [Double] = []
        let columns = dofNames.count
        for base in bases {
            let rightName = "\(base)_r"
            guard let row = table[rightName],
                  let parameters = WrapValidationHarness.muscleParameters[rightName],
                  parameters.maxForce > 0, parameters.optimalFiberLength > 0 else { continue }
            var right = [Double](repeating: 0, count: columns)
            for (slot, coordinate) in rightLegCoordinates.enumerated() {
                if let sample = row[coordinate] { right[slot] = value(sample, source) }
            }
            // The left leg IS the right leg, mirrored: identical arms, moved to
            // the left coordinates. That makes the modelling error bilateral by
            // construction rather than by assumption.
            var left = [Double](repeating: 0, count: columns)
            for slot in 0..<rightLegCoordinates.count { left[slot + 6] = right[slot] }
            // Isometric: normalised fibre length 1, so f_AL = f_FV = cos α = 1.
            let isometric = parameters.optimalFiberLength + parameters.tendonSlackLength
            for (name, arm) in [(rightName, right), ("\(base)_l", left)] {
                names.append(name)
                arms.append(contentsOf: arm)
                scaled.append(contentsOf: arm.map { $0 * parameters.maxForce })
                lengths.append(isometric)
                maxForces.append(parameters.maxForce)
                optimal.append(parameters.optimalFiberLength)
                slack.append(parameters.tendonSlackLength)
            }
        }
        guard !names.isEmpty else { return nil }
        return Built(names: names, arms: arms, armsInForceUnits: scaled, lengths: lengths,
                     maxForces: maxForces, optimalFiberLengths: optimal,
                     tendonSlackLengths: slack)
    }

    struct SolveOutcome {
        /// What `MuscleSolver` returns — OSQP at its shipping tolerance.
        let shipped: [String: Double]
        /// The same objective at machine precision.
        let exact: [String: Double]
        let kktResidual: Double
        let converged: Bool
        let relativeTorqueResidual: Double
        let atLower: Int
        let atUpper: Int
    }

    /// One QP on a FRESH `MuscleSolver` — no warm start, no `L_MT` history — plus
    /// the same objective solved exactly.
    static func solve(pose: String, source: ArmSource, torques: [Double]) -> SolveOutcome? {
        guard let model = build(pose: pose, source: source) else { return nil }
        guard let result = MuscleSolver().solveReal(
            withJointTorques: torques.map(NSNumber.init(value:)),
            momentArms: model.arms.map(NSNumber.init(value:)),
            muscleNames: model.names,
            muscleLengths: model.lengths.map(NSNumber.init(value:)),
            maxForces: model.maxForces.map(NSNumber.init(value:)),
            optimalFiberLengths: model.optimalFiberLengths.map(NSNumber.init(value:)),
            tendonSlackLengths: model.tendonSlackLengths.map(NSNumber.init(value:)),
            pennationAngles: model.names.map { _ in NSNumber(value: 0.0) },
            jointVelocities: dofNames.map { _ in NSNumber(value: 0.0) },
            dofNames: dofNames,
            dt: 1.0 / 30.0,
            softPenalty: 100.0) else { return nil }
        var shipped: [String: Double] = [:]
        for (index, name) in result.muscleNames.enumerated() {
            shipped[name] = result.activations[index].doubleValue
        }
        let precise = BoxQP.solve(arms: model.armsInForceUnits, nMuscles: model.names.count,
                                  nDOFs: dofNames.count, torques: torques,
                                  lower: MuscleSolver().minActivation,
                                  upper: MuscleSolver.maxActivation)
        var exact: [String: Double] = [:]
        for (index, name) in model.names.enumerated() { exact[name] = precise.activations[index] }
        return SolveOutcome(shipped: shipped, exact: exact, kktResidual: precise.kktResidual,
                            converged: result.converged,
                            relativeTorqueResidual: result.relativeTorqueResidual,
                            atLower: precise.activeAtLower, atUpper: precise.activeAtUpper)
    }

    /// The joint torques a leg at this pose would need to hold every one of its
    /// muscles at `activation`, computed through the LOADING column's moment
    /// arms. Inverse dynamics never touches a moment arm, so the same `τ` is
    /// handed to both solves; this construction only has to be a plausible,
    /// feasible one.
    ///
    /// The label is `loading` and not `reference` since R1 v2: this is the τ
    /// source, which is a different role from the `truth` the leak is measured
    /// against and from the `subject` exclusion. v1 used one variable for all
    /// three, which is why filtering R1 onto one column could not be done as a
    /// filter.
    static func torques(pose: String, loading: ArmSource, activation: Double,
                        shape: [Double]) -> [Double]? {
        guard let table = byPose[pose] else { return nil }
        var right = [Double](repeating: 0, count: rightLegCoordinates.count)
        for base in bases {
            let name = "\(base)_r"
            guard let row = table[name],
                  let parameters = WrapValidationHarness.muscleParameters[name] else { continue }
            for (slot, coordinate) in rightLegCoordinates.enumerated() {
                guard let sample = row[coordinate] else { continue }
                right[slot] += value(sample, loading) * activation * parameters.maxForce
            }
        }
        return right + zip(right, shape).map { $0 * $1 }
    }

    /// `100·(L − R)/mean` — the panel's own statistic.
    static func differencePercent(_ activations: [String: Double], _ base: String) -> Double? {
        guard let l = activations["\(base)_l"], let r = activations["\(base)_r"] else { return nil }
        let mean = 0.5 * (l + r)
        guard mean > 0 else { return nil }
        return 100 * (l - r) / mean
    }

    /// A base is READABLE when all four of its EXACT activations sit strictly
    /// inside the box. Screening on the exact solution rather than on OSQP's is
    /// the point: the true active set is a property of the problem, and letting
    /// the solver decide which muscles are readable would make the screen depend
    /// on the thing being measured.
    static func isScreened(_ truth: SolveOutcome, _ test: SolveOutcome, _ base: String) -> Bool {
        let lower = MuscleSolver().minActivation + interiorMargin
        let upper = MuscleSolver.maxActivation - interiorMargin
        for outcome in [truth, test] {
            for side in ["l", "r"] {
                guard let value = outcome.exact["\(base)_\(side)"] else { return false }
                if value <= lower || value >= upper { return false }
            }
        }
        return true
    }

    // MARK: - The sweep

    struct Cell {
        let pose: String
        /// **Which OpenSim column supplied the JOINT TORQUES.** v1 called this
        /// `reference` and used the same variable for the truth and for the
        /// subject exclusion; splitting the roles is what let R1 be registered
        /// against one truth without deleting a loading family. Both values
        /// appear under `truth == registeredTruth`, so R1 v2's population is
        /// strictly larger than the analytic half of v1's.
        let loading: ArmSource
        /// **Which OpenSim column the leak is measured AGAINST.** The gated
        /// statistics keep only `truth == registeredTruth`; the v1 quantity is
        /// the diagonal `loading == truth`, pooled over both.
        let truth: ArmSource
        let subject: ArmSource
        let shape: String
        let effort: Double
        let screened: Int
        /// `|d(subject, exact) − d(truth, exact)|` — the moment arms alone.
        let leakExact: Double
        /// `|d(subject, OSQP) − d(subject, exact)|` — the shipping solver alone.
        let solverSlack: Double
        /// `|d(subject, OSQP) − d(truth, exact)|` — both causes together.
        let leakShipped: Double
        /// The base at which `leakShipped` is maximised. **Not** the base at which
        /// `leakExact` is: those are different maxima and they landed on different
        /// muscles the moment the solver stopped contributing. Printing this one
        /// beside `leakExact` is how STATUS.md came to record R1's worst as
        /// `piri`/`glmed3` — muscles that carry NO `PathWrap` at all, so the
        /// attribution it invited ("the tail is a wrapping residual") was wrong
        /// twice over. Read `worstExactBase` for the exact leak.
        let worstBase: String
        /// The base at which `leakExact` is maximised — R1's own muscle.
        let worstExactBase: String
        /// `leakShipped` restricted to muscles `displayNames` prints a row for.
        let leakAmongNamed: Double
        let namedScreened: Int
        /// Spread of the TRUE left/right figures across the screened bases.
        let trueSpread: Double
        let converged: Bool
        let kktResidual: Double
        let medianActivation: Double
        let torqueResidual: Double
    }

    /// The torque shapes. `hip 0.80 / knee 1.00` is the case that retired the
    /// claim; the sweep around it exists because the leak is zero at the
    /// proportional point and grows away from it, so one point is not a
    /// measurement of the worst case.
    static let shapes: [(name: String, scales: [Double])] = [
        ("hip0.80_knee0.60", [0.8, 0.8, 0.8, 0.6, 1.0, 1.0]),
        ("hip0.80_knee0.80", [0.8, 0.8, 0.8, 0.8, 1.0, 1.0]),
        ("hip0.80_knee1.00", [0.8, 0.8, 0.8, 1.0, 1.0, 1.0]),
        ("hip1.00_knee0.80", [1.0, 1.0, 1.0, 0.8, 1.0, 1.0]),
        ("hip0.90_ankle1.20", [0.9, 0.9, 0.9, 1.0, 1.2, 1.2]),
    ]

    /// How hard the leg is working. Swept because it decides WHICH muscles sit on
    /// a bound and how much of the solver's ABSOLUTE slack reaches a RELATIVE
    /// statistic — not what the exact leak is. Where no bound is active the QP is
    /// linear in `τ`, so a common scale divides out of `100·(a_l − a_r)/mean`.
    static let effortLevels = [0.3, 0.9, 2.7]

    private static var cells: [Cell] = []
    private static var cellsBuilt = false

    /// **The FULL 2×2 GRID since R1 v2**: `loading` (which column supplied τ) ×
    /// `truth` (which column the leak is measured against), with the subject
    /// loop preserved verbatim as `subject != truth`.
    ///
    /// v1 iterated ONE `reference` and used it for all three roles, so R1 could
    /// not be pointed at one truth by a filter: filtering would have deleted an
    /// entire LOADING family and re-keyed every instrument built on the v1
    /// population. Under the 2×2 the v1 population survives byte-for-byte as the
    /// DIAGONAL `loading == truth` — τ depends only on
    /// `(pose, loading, effort, shape)`, the truth solve on `(pose, truth, τ)`,
    /// and `subject != truth` is v1's `subject != reference` — and the diagonal
    /// cells are emitted in v1's own order, so `V1-CONT` can check that this
    /// change ADDED an axis rather than perturbing the cells that were there.
    ///
    /// `solve` is a pure function of `(pose, source, τ)` on a fresh
    /// `MuscleSolver`, so one pose's four `(loading, truth)` pairs share their
    /// solves: the truth solve of `(A, CD)` is the subject solve of `(A, A)`.
    /// The grid therefore costs the same QP count as v1 — 2 loadings × 4 sources
    /// per (pose, effort, shape) — which is an implementation property and not a
    /// registration change; the registration budgeted for double.
    static func sweep() -> [Cell] {
        guard !cellsBuilt else { return cells }
        buildIndex()
        let named = Set(bases.filter { GaitLoadSummary.displayNames[$0] != nil })
        let references = ArmSource.allCases.filter { $0.isReference }
        var out: [Cell] = []
        for pose in poseOrder {
            var tauCache: [String: [Double]?] = [:]
            var solveCache: [String: SolveOutcome?] = [:]
            func tau(_ loading: ArmSource, _ effort: Double,
                     _ shape: (name: String, scales: [Double])) -> [Double]? {
                let key = "\(loading.rawValue)|\(effort)|\(shape.name)"
                if let cached = tauCache[key] { return cached }
                var value = torques(pose: pose, loading: loading, activation: effort,
                                    shape: shape.scales)
                if let candidate = value, !candidate.contains(where: { abs($0) > 1e-9 }) {
                    value = nil
                }
                tauCache[key] = value
                return value
            }
            func outcome(_ loading: ArmSource, _ effort: Double,
                         _ shape: (name: String, scales: [Double]),
                         _ source: ArmSource) -> SolveOutcome? {
                let key = "\(loading.rawValue)|\(effort)|\(shape.name)|\(source.rawValue)"
                if let cached = solveCache[key] { return cached }
                guard let torques = tau(loading, effort, shape) else {
                    let miss: SolveOutcome? = nil
                    solveCache[key] = miss
                    return nil
                }
                let value = solve(pose: pose, source: source, torques: torques)
                solveCache[key] = value
                return value
            }
            // Flattened so the loop DEPTH is v1's and the diagonal cells are
            // emitted in v1's own order: (A,A), (A,CD), (CD,A), (CD,CD).
            let grid = references.flatMap { loading in references.map { (loading, $0) } }
            for (loading, truthSource) in grid {
                for effort in effortLevels {
                    for shape in shapes {
                        guard let truth = outcome(loading, effort, shape, truthSource)
                        else { continue }
                        // The OTHER reference is swept as a SUBJECT too. Same τ,
                        // same truth solve, same statistic — so "how far apart are
                        // OpenSim's own two answers" lands on the identical scale
                        // as R1 instead of being argued about. Every gate here
                        // filters by `subject` (`gated(.ours)` /
                        // `gated(.straightLine)`), so this adds cells and
                        // changes no gated number; that is asserted in
                        // `testTheReferenceDisagreesWithItselfByMoreThanTheGateAllows`
                        // and was verified bit-for-bit against the run before it.
                        // Keeping the exclusion as `subject != truth` is what
                        // stops a structurally-zero analytic-vs-analytic leg
                        // (leak ≡ 0 by construction) entering any statistic.
                        for subject in ArmSource.allCases where subject != truthSource {
                            guard let test = outcome(loading, effort, shape, subject)
                            else { continue }
                            var exactLeak = 0.0, slack = 0.0, shippedLeak = 0.0, namedLeak = 0.0
                            var worstBase = "", worstExactBase = "", screened = 0, namedScreened = 0
                            var trueFigures: [Double] = []
                            var levels: [Double] = []
                            for base in bases {
                                guard isScreened(truth, test, base),
                                      let dTruth = differencePercent(truth.exact, base),
                                      let dExact = differencePercent(test.exact, base),
                                      let dShipped = differencePercent(test.shipped, base)
                                else { continue }
                                screened += 1
                                trueFigures.append(dTruth)
                                if let l = truth.exact["\(base)_l"],
                                   let r = truth.exact["\(base)_r"] {
                                    levels.append(0.5 * (l + r))
                                }
                                if abs(dExact - dTruth) > exactLeak {
                                    exactLeak = abs(dExact - dTruth)
                                    worstExactBase = base
                                }
                                slack = Swift.max(slack, abs(dShipped - dExact))
                                let total = abs(dShipped - dTruth)
                                if total > shippedLeak { shippedLeak = total; worstBase = base }
                                if named.contains(base) {
                                    namedScreened += 1
                                    namedLeak = Swift.max(namedLeak, total)
                                }
                            }
                            out.append(Cell(pose: pose, loading: loading, truth: truthSource,
                                            subject: subject,
                                            shape: shape.name, effort: effort, screened: screened,
                                            leakExact: exactLeak, solverSlack: slack,
                                            leakShipped: shippedLeak, worstBase: worstBase,
                                            worstExactBase: worstExactBase,
                                            leakAmongNamed: namedLeak, namedScreened: namedScreened,
                                            trueSpread: (trueFigures.max() ?? 0)
                                                      - (trueFigures.min() ?? 0),
                                            converged: truth.converged && test.converged,
                                            kktResidual: Swift.max(truth.kktResidual,
                                                                   test.kktResidual),
                                            medianActivation: WrapValidationHarness.percentile(
                                                levels, 0.5),
                                            torqueResidual: Swift.max(truth.relativeTorqueResidual,
                                                                      test.relativeTorqueResidual)))
                        }
                    }
                }
            }
        }
        cells = out
        cellsBuilt = true
        return out
    }

    /// R6: a cell is read only where the exact solver reached a KKT point and at
    /// least `minimumScreenedBases` muscles are interior. **Unchanged by R1 v2**
    /// — it filters on `subject` alone, which is exactly why the truth had to
    /// become an axis rather than a filter.
    static func readable(_ subject: ArmSource, in cells: [Cell]) -> [Cell] {
        cells.filter {
            $0.subject == subject && $0.screened >= minimumScreenedBases
                && $0.kktResidual < 1e-6 && $0.converged
        }
    }

    /// **The GATED population of R1 v2**: the registered truth, across BOTH
    /// loading families. Strictly larger than v1's analytic half — the
    /// `loading == .centralDifference` half had never been computed when this
    /// was registered, so R1's maximum over it was unknown at freeze time and
    /// can only be ≥ the analytic half's 3.6932 pp.
    static func gated(_ subject: ArmSource, in cells: [Cell]) -> [Cell] {
        readable(subject, in: cells).filter { $0.truth == registeredTruth }
    }

    /// **The v1 population, byte-for-byte**: the diagonal of the 2×2 grid. Kept
    /// because R1-P, I2 and R7's pooled leg were each validated on it, and
    /// because `V1-CONT` needs it to prove the re-parameterisation added an axis
    /// instead of perturbing the cells that were already there.
    static func v1(_ subject: ArmSource, in cells: [Cell]) -> [Cell] {
        readable(subject, in: cells).filter { $0.loading == $0.truth }
    }

    /// The median of `trueSpread / leakShipped` — R7's statistic. Extracted so
    /// the gated leg, the v1 leg and the printed value cannot drift apart.
    static func informationRatio(_ population: [Cell]) -> Double {
        WrapValidationHarness.percentile(population.map {
            $0.leakShipped > 0 ? $0.trueSpread / $0.leakShipped : Double.infinity
        }.filter { $0.isFinite }, 0.5)
    }

    /// **The verdict, computed in ONE place.** Both the flag pin
    /// (`testTheShippedFlagMatchesWhatTheMeasurementSupports`) and the loud
    /// no-silent-reopen test read this, so the two cannot state different
    /// decisions.
    ///
    /// `supported` keeps v1's SIX terms, operators and constants — R1 ∧ R2 ∧ R4
    /// ∧ R5 ∧ R6 ∧ R7 — repointed onto `gated` with R7 in its registered
    /// `min(gated, v1)` form. **No constant conjunct is folded in**: a
    /// compile-time-constant `supported` would degrade
    /// `XCTAssertEqual(flag, supported)` to `XCTAssertEqual(false, false)`, and
    /// R1's and R2's MAX thresholds are asserted nowhere else in the suite, so a
    /// later `reopenFractionOfFloor = 20.0` would land silently. The pin stays
    /// BIDIRECTIONAL; the "chosen after its numbers were read" caveat is a
    /// separate loud test.
    struct Decision {
        let supported: Bool
        let worstExact: Double
        let worstShipped: Double
        let controlWorst: Double
        let gatedCells: Int
        let ratio: Double
        let ratioGated: Double
        let ratioV1: Double
        let floor: Double
        let threshold: Double
    }

    static func decide(cells: [Cell], floor: Double) -> Decision {
        let threshold = floor * reopenFractionOfFloor
        let gatedOurs = gated(.ours, in: cells)
        let gatedControl = gated(.straightLine, in: cells)
        let worstExact = gatedOurs.map(\.leakExact).max() ?? .infinity
        let worstShipped = gatedOurs.map(\.leakShipped).max() ?? .infinity
        let controlWorst = gatedControl.map(\.leakExact).max() ?? 0
        let ratioGated = informationRatio(gatedOurs)
        let ratioV1 = informationRatio(v1(.ours, in: cells))
        let ratio = Swift.min(ratioGated, ratioV1)

        let supported = worstExact < threshold                                   // R1
            && worstShipped < threshold                                          // R2
            && GaitLoadSummary.musclesWithUnmodelledPaths.isEmpty                // R4
            && controlWorst > floor                                              // R5
            && gatedOurs.count >= minimumCells                                   // R6
            && ratio > minimumInformationToLeakRatio                             // R7

        return Decision(supported: supported, worstExact: worstExact,
                        worstShipped: worstShipped, controlWorst: controlWorst,
                        gatedCells: gatedOurs.count, ratio: ratio,
                        ratioGated: ratioGated, ratioV1: ratioV1,
                        floor: floor, threshold: threshold)
    }

    // MARK: - The floor

    /// The finest left/right difference any pinned clip can assert, read from the
    /// clips rather than copied — same helper as
    /// `MomentArmErrorCancellationTests`.
    func smallestPublicationFloorOnThePinnedClips() throws -> Double {
        let bundle = Bundle(for: type(of: self))
        var floors: [Double] = []
        for id in GaitClipFixture.allIds {
            let report = try GaitAnalysis.analyse(
                frames: try GaitClipFixture.load(id, bundle: bundle).frames)
            guard report.isUsable else { continue }
            floors.append(report.resolution.resolvableAsymmetryPercent)
        }
        return try XCTUnwrap(floors.min(), "no pinned clip is usable")
    }

    // MARK: - Structure, checked before anything is concluded from it

    override func setUpWithError() throws {
        try WrapValidationHarness.requireBuild(bundle: Bundle(for: type(of: self)))
        Self.buildIndex()
    }

    /// The rig is what it says: real muscles, real coordinates, and a muscle
    /// population that does not depend on which arm source is under test.
    func testTheRigIsBuiltFromTheModelAndNotFromAList() throws {
        let named = Set(Self.bases).intersection(GaitLoadSummary.displayNames.keys)
        print("LEAK-METRIC rig bases=\(Self.bases.count) poses=\(Self.poseOrder.count) "
              + "dofs=\(Self.dofNames.count) named=\(named.count) "
              + "named_list=\(named.sorted()) poses_list=\(Self.poseOrder)")
        XCTAssertGreaterThanOrEqual(Self.bases.count, 40)
        XCTAssertGreaterThanOrEqual(Self.poseOrder.count, 20)
        for pose in Self.poseOrder {
            let table = try XCTUnwrap(Self.byPose[pose])
            for muscle in table.keys { XCTAssertTrue(muscle.hasSuffix("_r")) }
        }
        for base in Self.bases {
            XCTAssertNotNil(WrapValidationHarness.muscleParameters["\(base)_r"])
        }
        XCTAssertGreaterThanOrEqual(named.count, 25,
                                    "the rig has to contain the muscles the product would name")
    }

    /// FullBody permits 140 degrees of knee flexion, but the five nonlinear
    /// `walker_knee_r` transform splines end at 120 degrees. This fixed-point,
    /// wrap-free path at 130 degrees is the product-level regression for the
    /// endpoint-linear SimmSpline patch: the old endpoint-cubic continuation
    /// produced 16.059 mm instead of OpenSim's 13.713 mm.
    func testEndpointLinearKinematicsMatchOpenSimBeyondTheLastKneeKnot() throws {
        let sample = try XCTUnwrap(WrapValidationHarness.samples.first {
            $0.pose == "run_4_mid_swing"
                && $0.muscle == "bflh140_r"
                && $0.coordinate == "knee_angle_r"
        })
        print(String(format: "SIMMSPLINE-METRIC pose=run_4_mid_swing "
                     + "muscle=bflh140_r coordinate=knee_angle_r "
                     + "ours_mm=%.9f opensim_mm=%.9f delta_mm=%+.9f",
                     1000 * sample.ours, 1000 * sample.wrapOn,
                     1000 * (sample.ours - sample.wrapOn)))
        XCTAssertEqual(sample.ours, 0.013713464958, accuracy: 1e-6)
        XCTAssertEqual(sample.ours, sample.wrapOn, accuracy: 1e-6,
                       "Nimble must use the same endpoint-tangent extrapolation as OpenSim")
    }

    /// **The mirror is exact in the matrix the solver is handed**, so the
    /// modelling error really is bilateral. Read off `Built.arms`, not asserted
    /// in a comment: a left row must be zero on every right coordinate, equal to
    /// its right twin on every left one, and the pair must not be all-zero.
    func testTheLeftLegIsTheRightLegsExactMirrorInTheMatrixTheSolverSees() throws {
        var checked = 0
        for pose in [Self.poseOrder.first, Self.poseOrder.last].compactMap({ $0 }) {
            for source in Self.ArmSource.allCases {
                let model = try XCTUnwrap(Self.build(pose: pose, source: source))
                let columns = Self.dofNames.count
                XCTAssertEqual(model.arms.count, model.names.count * columns)
                XCTAssertEqual(model.names.count % 2, 0)
                for pair in stride(from: 0, to: model.names.count, by: 2) {
                    XCTAssertTrue(model.names[pair].hasSuffix("_r"))
                    XCTAssertTrue(model.names[pair + 1].hasSuffix("_l"))
                    var anyNonZero = false
                    for slot in 0..<6 {
                        let right = model.arms[pair * columns + slot]
                        XCTAssertEqual(model.arms[(pair + 1) * columns + slot + 6], right,
                                       accuracy: 0,
                                       "\(model.names[pair]) is not mirrored at slot \(slot)")
                        XCTAssertEqual(model.arms[pair * columns + slot + 6], 0)
                        XCTAssertEqual(model.arms[(pair + 1) * columns + slot], 0)
                        if right != 0 { anyNonZero = true }
                        checked += 1
                    }
                    XCTAssertTrue(anyNonZero, "\(model.names[pair]) has an all-zero row")
                    XCTAssertEqual(model.lengths[pair], model.lengths[pair + 1])
                    XCTAssertEqual(model.maxForces[pair], model.maxForces[pair + 1])
                }
            }
        }
        XCTAssertGreaterThan(checked, 500)
    }

    /// **`A = R · diag(F_max)` is asserted against the solver's own output, not
    /// assumed.** `BoxQP` reproduces `MuscleSolver`'s objective only if the Hill
    /// multipliers are all 1 at these lengths; if a future change to the force
    /// model breaks that, every exact number in this file becomes a different
    /// objective's answer and this test is the only thing that would say so.
    func testTheForceScaleIsExactlyFmaxAtTheseLengths() throws {
        let pose = try XCTUnwrap(Self.poseOrder.first)
        let model = try XCTUnwrap(Self.build(pose: pose, source: .ours))
        let tau = try XCTUnwrap(Self.torques(pose: pose, loading: .analytic, activation: 0.9,
                                             shape: [Double](repeating: 0.8, count: 6)))
        let result = try XCTUnwrap(MuscleSolver().solveReal(
            withJointTorques: tau.map(NSNumber.init(value:)),
            momentArms: model.arms.map(NSNumber.init(value:)),
            muscleNames: model.names,
            muscleLengths: model.lengths.map(NSNumber.init(value:)),
            maxForces: model.maxForces.map(NSNumber.init(value:)),
            optimalFiberLengths: model.optimalFiberLengths.map(NSNumber.init(value:)),
            tendonSlackLengths: model.tendonSlackLengths.map(NSNumber.init(value:)),
            pennationAngles: model.names.map { _ in NSNumber(value: 0.0) },
            jointVelocities: Self.dofNames.map { _ in NSNumber(value: 0.0) },
            dofNames: Self.dofNames, dt: 1.0 / 30.0, softPenalty: 100.0))
        var worst = 0.0
        for (index, name) in result.muscleNames.enumerated() {
            let a = result.activations[index].doubleValue
            let f = result.forces[index].doubleValue
            guard a > 0, let row = model.names.firstIndex(of: name) else { continue }
            worst = Swift.max(worst, abs(f / a - model.maxForces[row]) / model.maxForces[row])
        }
        print("LEAK-METRIC force_scale worst_relative_departure_from_Fmax=\(worst)")
        XCTAssertLessThan(worst, 1e-9,
                          "at l_opt + l_Ts with zero pennation and zero velocity the force scale "
                          + "must be exactly F_max, or BoxQP is solving a different objective")
    }

    /// **The exact solver is checked against the shipping one where they must
    /// agree**, and its own KKT conditions are checked everywhere. Two solvers
    /// that never agree are two bugs; two that agree to OSQP's tolerance are one
    /// instrument and one measurement.
    ///
    /// **I1 stays on the FULL 2×2**, deliberately unfiltered by the registered
    /// truth: that population is a strict SUPERSET of v1's, so the check is
    /// strictly stronger and still contains every v1 cell. The v1-diagonal worst
    /// is printed beside it for continuity. The R6 count it re-checks is the
    /// GATED one, because that is the population R1/R2 maximise over.
    func testTheExactSolverSatisfiesKKTAndAgreesWithOSQPToItsOwnTolerance() throws {
        let cells = Self.sweep()
        let readable = Self.readable(.ours, in: cells)
        let gated = Self.gated(.ours, in: cells)
        let diagonal = Self.v1(.ours, in: cells)
        let all = cells.filter { $0.subject == .ours }
        let diagonalAll = all.filter { $0.loading == $0.truth }
        let worstKKT = all.map(\.kktResidual).max() ?? .nan
        let slacks = diagonal.map(\.solverSlack)
        print("LEAK-METRIC exact_solver worst_kkt_residual=\(worstKKT) "
              + "v1_diagonal_worst_kkt_residual=\(diagonalAll.map(\.kktResidual).max() ?? .nan) "
              + "cells=\(all.count) readable=\(readable.count) "
              + "gated_cells=\(gated.count) v1_cells=\(diagonal.count) "
              + "median_solver_slack_pp=\(WrapValidationHarness.percentile(slacks, 0.5)) "
              + "p90=\(WrapValidationHarness.percentile(slacks, 0.9)) "
              + "max_solver_slack_pp=\(slacks.max() ?? 0) "
              + "median_torque_residual=\(WrapValidationHarness.percentile(all.map(\.torqueResidual), 0.5)) "
              + "median_activation=\(WrapValidationHarness.percentile(readable.map(\.medianActivation), 0.5))")
        XCTAssertLessThan(worstKKT, 1e-6,
                          "every exact solve must reach a KKT point, or the instrument is not one")
        XCTAssertGreaterThanOrEqual(gated.count, Self.minimumCells,
                                    "R6: at least \(Self.minimumCells) gated cells")
    }

    // MARK: - R4: every path is modelled, and no displayed muscle is not

    /// **How many muscles still have unmodelled paths, which ones, and whether
    /// any of them is one the product names.** Read from the parser's own
    /// fidelity report on BOTH shipped models, never from the hand-written table
    /// — the table is checked against this, not the other way round.
    func testNoMuscleTheProductNamesHasAnUnmodelledPath() throws {
        var unmodelled: [String] = []
        let fullBody = try XCTUnwrap(WrapValidationHarness.fullBodyReport)
        print("LEAK-METRIC unmodelled model=FullBody solved=\(fullBody.solvedPathWraps) "
              + "unmodelled=\(fullBody.unmodelledPathWraps) "
              + "muscles=\(fullBody.musclesWithUnmodelledPathWraps)")
        XCTAssertEqual(fullBody.unmodelledPathWraps, 0,
                       "FullBody.osim still has unsolved PathWrap references: "
                       + "\(fullBody.musclesWithUnmodelledPathWraps)")
        unmodelled += fullBody.musclesWithUnmodelledPathWraps

        let bundle = Bundle(for: type(of: self))
        let path = try XCTUnwrap(bundle.path(forResource: "Rajagopal2016", ofType: "osim"))
        let bridge = NimbleBridge()
        XCTAssertTrue(bridge.loadModel(fromPath: path))
        let computer = MomentArmComputer()
        XCTAssertTrue(computer.parseMusclePaths(fromOsimPath: path, from: bridge))
        let report = computer.fidelityReport
        print("LEAK-METRIC unmodelled model=Rajagopal2016 solved=\(report.solvedPathWraps) "
              + "unmodelled=\(report.unmodelledPathWraps) "
              + "muscles=\(report.musclesWithUnmodelledPathWraps)")
        XCTAssertEqual(report.unmodelledPathWraps, 0)
        unmodelled += report.musclesWithUnmodelledPathWraps

        let named = Set(unmodelled).compactMap { GaitLoadSummary.split($0)?.base }
                                   .filter { GaitLoadSummary.displayNames[$0] != nil }
        XCTAssertTrue(named.isEmpty,
                      "muscles the product names still have unmodelled paths: \(named)")
        XCTAssertEqual(GaitLoadSummary.musclesWithUnmodelledPaths, [],
                       "the shipped table must agree with the parser")
    }

    // MARK: - R5: the control, which is what makes a small leak mean something

    /// **The BEFORE.** The same rig, the same torques, the same truth solve —
    /// driven with the straight-line moment arms this project shipped until
    /// 2026-08-08. If this does not leak, the rig cannot see a moment-arm error
    /// at all and nothing else in this file is evidence.
    ///
    /// **R1 v2 re-populates this onto `gated(.straightLine)`, and the
    /// subset-monotonicity argument for it is WITHDRAWN.**
    /// `{(loading=A, truth=A), (loading=CD, truth=A)}` is neither a subset nor a
    /// superset of v1's `{(A,A), (CD,CD)}`, so no relation to v1's pooled
    /// 66.88 pp is asserted in either direction. What justifies this leg on its
    /// own terms: the bar is the UNCHANGED full floor (not floor/5), the
    /// assertion is unchanged, and half its population has never been computed.
    /// If it fails the rig proves nothing under the registered truth (W4) — the
    /// response is to report the FAIL, never to restore pooling.
    func testTheStraightLineArmsLeakMoreThanThePinnedClipsCanResolve() throws {
        let cells = Self.sweep()
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let readable = Self.gated(.straightLine, in: cells)
        let diagonal = Self.v1(.straightLine, in: cells)
        let worst = try XCTUnwrap(readable.max { $0.leakExact < $1.leakExact },
                                  "the control produced no readable cell")
        print("LEAK-METRIC control straight_line worst_exact_leak_pp=\(worst.leakExact) "
              + "at pose=\(worst.pose) loading=\(worst.loading.rawValue) "
              + "truth=\(worst.truth.rawValue) shape=\(worst.shape) "
              + "effort=\(worst.effort) on=\(worst.worstBase) screened=\(worst.screened) "
              + "worst_shipped_leak_pp=\(readable.map(\.leakShipped).max() ?? 0) "
              + "named_leak_pp=\(readable.map(\.leakAmongNamed).max() ?? 0) "
              + "median_exact_leak_pp="
              + "\(WrapValidationHarness.percentile(readable.map(\.leakExact), 0.5)) "
              + "cells_over_floor=\(readable.filter { $0.leakExact > floor }.count)"
              + "/\(readable.count) floor_percent=\(floor) "
              + "v1_diagonal_worst_exact_leak_pp=\(diagonal.map(\.leakExact).max() ?? 0) "
              + "v1_diagonal_median_exact_leak_pp="
              + "\(WrapValidationHarness.percentile(diagonal.map(\.leakExact), 0.5)) "
              + "v1_diagonal_cells=\(diagonal.count)")
        XCTAssertGreaterThan(worst.leakExact, floor,
                             "the straight-line arms must leak more than the finest pinned clip "
                             + "can resolve, or this rig is not sensitive enough to certify "
                             + "anything about the wrapped ones")
    }

    // MARK: - THE MEASUREMENT

    /// **R1, R2, R1-M and R6.** The leak this build's own moment arms produce,
    /// over every leg pose in the sweep, BOTH loading families under the ONE
    /// registered truth, five torque shapes and three effort levels — separated
    /// into the moment-arm cause and the solver cause, and reported against the
    /// floor read from the pinned clips.
    /// **The verdict is asserted in `testTheShippedFlagMatchesWhatTheMeasurementSupports`
    /// and nowhere else.** This test measures and reports; what it ASSERTS is
    /// R1-M — the two medians that were already live here before R1 v2 named
    /// them — as a regression tripwire with a number, because a test that
    /// asserted `leak < threshold` would be asserting a hypothesis, and this
    /// measurement's answer is no.
    ///
    /// It also prints, gating nothing: **`V1-CONT`** (the v1 registered quantity
    /// on the diagonal, whose job is to detect the single most dangerous
    /// implementation error — perturbing the existing cells instead of adding an
    /// axis) and **`M1`** (`column_vs_column_singlewrap` plus
    /// `worst_cell_smoothness`).
    func testTheWrappedMomentArmsLeakLessThanAFifthOfThePublicationFloor() throws {
        let cells = Self.sweep()
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let threshold = floor * Self.reopenFractionOfFloor
        let readable = Self.gated(.ours, in: cells)
        let control = Self.gated(.straightLine, in: cells)
        let worstExact = try XCTUnwrap(readable.max { $0.leakExact < $1.leakExact },
                                       "no readable cell — the sweep measured nothing")
        let worstShipped = try XCTUnwrap(readable.max { $0.leakShipped < $1.leakShipped })
        let worstNamed = try XCTUnwrap(readable.max { $0.leakAmongNamed < $1.leakAmongNamed })
        let median = WrapValidationHarness.percentile(readable.map(\.leakExact), 0.5)
        let controlMedian = WrapValidationHarness.percentile(control.map(\.leakExact), 0.5)

        // The gated population's two LOADING families, and — separately — the
        // whole 2x2 broken out by (loading, truth). The second is what makes the
        // never-before-computed CD-loading half visible beside the v1 halves.
        var perReference: [String] = []
        for loading in Self.ArmSource.allCases where loading.isReference {
            for truth in Self.ArmSource.allCases where truth.isReference {
                let subset = Self.readable(.ours, in: cells)
                    .filter { $0.loading == loading && $0.truth == truth }
                let worst = subset.max { $0.leakExact < $1.leakExact }
                perReference.append(String(format:
                    "loading=%@/truth=%@%@ exact[max %.4f on %@ at %@ | p99 %.4f median %.4f] "
                    + "shipped[max %.4f median %.4f] n=%d",
                    loading.rawValue, truth.rawValue,
                    truth == Self.registeredTruth ? "(GATED)" : "",
                    subset.map(\.leakExact).max() ?? 0,
                    worst?.worstExactBase ?? "-", worst?.pose ?? "-",
                    WrapValidationHarness.percentile(subset.map(\.leakExact), 0.99),
                    WrapValidationHarness.percentile(subset.map(\.leakExact), 0.5),
                    subset.map(\.leakShipped).max() ?? 0,
                    WrapValidationHarness.percentile(subset.map(\.leakShipped), 0.5),
                    subset.count))
            }
        }
        print("LEAK-METRIC wrapped worst_exact_leak_pp=\(worstExact.leakExact) "
              + "at pose=\(worstExact.pose) loading=\(worstExact.loading.rawValue) "
              + "truth=\(worstExact.truth.rawValue) "
              + "shape=\(worstExact.shape) effort=\(worstExact.effort) "
              + "on=\(worstExact.worstExactBase) screened=\(worstExact.screened) | "
              + "worst_shipped_leak_pp=\(worstShipped.leakShipped) "
              + "at pose=\(worstShipped.pose) loading=\(worstShipped.loading.rawValue) "
              + "truth=\(worstShipped.truth.rawValue) "
              + "shape=\(worstShipped.shape) effort=\(worstShipped.effort) "
              + "on=\(worstShipped.worstBase) solver_slack_there_pp=\(worstShipped.solverSlack) "
              + "exact_leak_there_pp=\(worstShipped.leakExact) | "
              + "worst_named_shipped_pp=\(worstNamed.leakAmongNamed) | "
              + "median_exact_pp=\(median) control_median_exact_pp=\(controlMedian) "
              + "p99_exact_pp="
              + "\(WrapValidationHarness.percentile(readable.map(\.leakExact), 0.99)) "
              + "median_shipped_pp="
              + "\(WrapValidationHarness.percentile(readable.map(\.leakShipped), 0.5)) "
              + "cells=\(readable.count) per_reference=\(perReference) "
              + "floor_percent=\(floor) threshold_pp=\(threshold) "
              + "R1_pass=\(worstExact.leakExact < threshold) "
              + "R2_pass=\(worstShipped.leakShipped < threshold)")

        // **V1-CONT — the v1 registered quantity, reported and gating nothing.**
        // Its whole job is to catch the one implementation error that would make
        // this run VOID rather than informative: re-parameterising the sweep as a
        // PERTURBATION of the existing cells instead of as an added axis. It must
        // reproduce max 123.0833 / median 0.6571 / R2 max 108.5576, and its
        // `truth == .analytic` half must reproduce 3.6932 / 3.3322 / 0.3121 at
        // `glmax2 @ grid_h090_k000_a+00`. That comparison is a REVIEW STEP and
        // deliberately NOT an assertion — hardcoding an expected number into a
        // test is forbidden here.
        let diagonal = Self.v1(.ours, in: cells)
        let diagonalAnalytic = diagonal.filter { $0.truth == Self.registeredTruth }
        let diagonalWorst = diagonal.max { $0.leakExact < $1.leakExact }
        let diagonalAnalyticWorst = diagonalAnalytic.max { $0.leakExact < $1.leakExact }
        print("LEAK-METRIC V1-CONT pooled_worst_exact_leak_pp="
              + "\(diagonal.map(\.leakExact).max() ?? 0) "
              + "pooled_median_exact_pp="
              + "\(WrapValidationHarness.percentile(diagonal.map(\.leakExact), 0.5)) "
              + "pooled_worst_shipped_leak_pp=\(diagonal.map(\.leakShipped).max() ?? 0) "
              + "pooled_worst_on=\(diagonalWorst?.worstExactBase ?? "-") "
              + "at pose=\(diagonalWorst?.pose ?? "-") "
              + "truth=\(diagonalWorst?.truth.rawValue ?? "-") "
              + "pooled_cells=\(diagonal.count) | "
              + "analytic_half_max_pp=\(diagonalAnalytic.map(\.leakExact).max() ?? 0) "
              + "analytic_half_p99_pp="
              + "\(WrapValidationHarness.percentile(diagonalAnalytic.map(\.leakExact), 0.99)) "
              + "analytic_half_median_pp="
              + "\(WrapValidationHarness.percentile(diagonalAnalytic.map(\.leakExact), 0.5)) "
              + "analytic_half_worst_on=\(diagonalAnalyticWorst?.worstExactBase ?? "-") "
              + "at pose=\(diagonalAnalyticWorst?.pose ?? "-") "
              + "shape=\(diagonalAnalyticWorst?.shape ?? "-") "
              + "effort=\(diagonalAnalyticWorst?.effort ?? 0) "
              + "analytic_half_cells=\(diagonalAnalytic.count)")

        try printColumnAgreementAndSmoothness(worstExact: worstExact,
                                              worstShipped: worstShipped)

        // **The worst cell's own muscle, in METRES beside the pp.** A moment arm
        // out by 2 mm on a 5 mm arm is a different finding from one out by 20 mm
        // on a 50 mm arm, and until this printed, the pp number was quoted alone
        // and attributed to "the wrapping residual" — on `bflh140`, which carries
        // no `PathWrap`, no `MovingPathPoint` and no entry in the
        // finite-difference fixture. Its row is therefore the SAME NUMBER in the
        // `analytic` and `centralDifference` matrices by construction, and what
        // moves its figure is the other muscles it shares the joint with.
        for cell in [worstExact, worstShipped] where !cell.worstExactBase.isEmpty {
            let name = "\(cell.worstExactBase)_r"
            guard let row = Self.byPose[cell.pose]?[name] else { continue }
            var arms: [String] = []
            for coordinate in Self.rightLegCoordinates {
                guard let sample = row[coordinate] else { continue }
                arms.append(String(format: "%@[ours %.3f analytic %.3f centralDiff %.3f "
                                   + "straightLine %.3f mm]",
                                   coordinate, 1000 * Self.value(sample, .ours),
                                   1000 * Self.value(sample, .analytic),
                                   1000 * Self.value(sample, .centralDifference),
                                   1000 * Self.value(sample, .straightLine)))
            }
            // A row in the finite-difference fixture exists only for muscles
            // that carry a `PathWrap`, so this doubles as "does this muscle wrap
            // at all" — read off the fixture rather than from a list.
            let carriesAPathWrap = WrapValidationHarness.samples.contains {
                $0.muscle == name && $0.centralDifference != nil
            }
            print("LEAK-METRIC worst_cell_arms muscle=\(name) pose=\(cell.pose) "
                  + "loading=\(cell.loading.rawValue) truth=\(cell.truth.rawValue) "
                  + "leak_exact_pp=\(cell.leakExact) "
                  + "leak_shipped_pp=\(cell.leakShipped) "
                  + "has_path_wrap=\(carriesAPathWrap) "
                  + "\(arms.joined(separator: " "))")
        }

        // The muscle whose FIGURE moves most need not be the muscle whose ARM is
        // wrong: the QP couples every muscle crossing a coordinate. Print every
        // readable row at the CURRENT worst ANALYTIC cell. This instrument first
        // localised the 42.46 pp bflh140 tail, then moved automatically to the
        // 3.69 pp glmax2 cell after endpoint-linear SimmSpline extrapolation fixed
        // bflh140. Keep it on the analytic column: unlike OpenSim's central
        // difference, that reference does not inherit the known multi-wrap
        // path-length bookkeeping defect. Since R1 v2 the gated population IS the
        // analytic-truth one, so this is R1's own binding cell — but the τ has to
        // be rebuilt from that cell's own LOADING column, which is no longer the
        // same thing as its truth.
        let analyticCells = readable
        let worstAnalytic = try XCTUnwrap(
            analyticCells.max { $0.leakExact < $1.leakExact },
            "the analytic reference produced no readable cell")
        let analyticShape = try XCTUnwrap(
            Self.shapes.first { $0.name == worstAnalytic.shape },
            "the worst cell's torque shape is not in the registered sweep")
        let analyticTau = try XCTUnwrap(Self.torques(
            pose: worstAnalytic.pose, loading: worstAnalytic.loading,
            activation: worstAnalytic.effort, shape: analyticShape.scales))
        let analyticTruth = try XCTUnwrap(Self.solve(
            pose: worstAnalytic.pose, source: Self.registeredTruth, torques: analyticTau))
        let analyticTest = try XCTUnwrap(Self.solve(
            pose: worstAnalytic.pose, source: .ours, torques: analyticTau))
        var screenedRows: [(maxArmError: Double, line: String)] = []
        for base in Self.bases where Self.isScreened(analyticTruth, analyticTest, base) {
            let name = "\(base)_r"
            guard let row = Self.byPose[worstAnalytic.pose]?[name],
                  let truthFigure = Self.differencePercent(analyticTruth.exact, base),
                  let oursFigure = Self.differencePercent(analyticTest.exact, base)
            else { continue }
            var arms: [String] = []
            var maxArmError = 0.0
            for coordinate in Self.rightLegCoordinates {
                guard let sample = row[coordinate] else { continue }
                let error = Self.value(sample, .ours) - Self.value(sample, .analytic)
                maxArmError = Swift.max(maxArmError, abs(error))
                arms.append(String(format: "%@[ours %.3f analytic %.3f delta %+.3f "
                                   + "centralDiff %.3f straightLine %.3f mm]",
                                   coordinate, 1000 * Self.value(sample, .ours),
                                   1000 * Self.value(sample, .analytic), 1000 * error,
                                   1000 * Self.value(sample, .centralDifference),
                                   1000 * Self.value(sample, .straightLine)))
            }
            let carriesAPathWrap = WrapValidationHarness.samples.contains {
                $0.muscle == name && $0.centralDifference != nil
            }
            let line = String(format:
                "muscle=%@ figure_truth_pp=%.6f figure_ours_pp=%.6f "
                + "figure_leak_pp=%+.6f max_arm_error_mm=%.6f has_path_wrap=%@ %@",
                name, truthFigure, oursFigure, oursFigure - truthFigure,
                1000 * maxArmError, carriesAPathWrap.description,
                arms.joined(separator: " "))
            screenedRows.append((maxArmError, line))
        }
        screenedRows.sort {
            if $0.maxArmError != $1.maxArmError { return $0.maxArmError > $1.maxArmError }
            return $0.line < $1.line
        }
        print("LEAK-METRIC worst_analytic_cell pose=\(worstAnalytic.pose) "
              + "loading=\(worstAnalytic.loading.rawValue) "
              + "truth=\(worstAnalytic.truth.rawValue) "
              + "shape=\(worstAnalytic.shape) effort=\(worstAnalytic.effort) "
              + "figure_worst=\(worstAnalytic.worstExactBase)_r "
              + "worst_exact_leak_pp=\(worstAnalytic.leakExact) "
              + "screened_rows=\(screenedRows.count)")
        for row in screenedRows {
            print("LEAK-METRIC worst_analytic_cell_row \(row.line)")
        }
        XCTAssertEqual(screenedRows.count, worstAnalytic.screened,
                       "the diagnostic must print every muscle admitted by R1's screen")

        // R6, both halves, on the GATED population R1/R2 are maximised over.
        XCTAssertGreaterThanOrEqual(worstExact.screened, Self.minimumScreenedBases,
                                    "R6: the maximum must be over a population")
        XCTAssertGreaterThanOrEqual(readable.map(\.screened).min() ?? 0,
                                    Self.minimumScreenedBases,
                                    "R6: every gated cell must carry at least "
                                    + "\(Self.minimumScreenedBases) screened bases")
        XCTAssertGreaterThanOrEqual(readable.count, Self.minimumCells,
                                    "R6: at least \(Self.minimumCells) gated cells")
        // **R1-M**, newly NAMED by the v2 registration. These two assertions were
        // already live here and were re-populated onto `gated(...)` by it, so
        // both changed meaning and both are now registered rather than incidental.
        XCTAssertLessThan(median, controlMedian / 3,
                          "R1-M: the wrap solver has to have bought something measurable: median "
                          + "moment-arm leak \(median) pp against the straight line's "
                          + "\(controlMedian) pp")
        XCTAssertLessThan(median, threshold,
                          "R1-M: and the TYPICAL cell has to be inside the reopening threshold, "
                          + "or the wrap work did not move the distribution at all")
    }

    /// **M1 — the diagnostic that replaces a misattributed number.**
    ///
    /// The strongest objection to the registered truth is that
    /// `centralDifference` is the column definition-matched to a `−dL/dq`
    /// implementation. A figure previously used to answer it — "median 0.000 mm /
    /// max 3.569 mm over 11,760 pairs" — is WITHDRAWN: that is
    /// `CylinderWrapValidationTests`' `|ours − reference|`, not a
    /// column-vs-column agreement. **The column-vs-column single-wrap agreement
    /// was measured nowhere in this repository**, so the registration cites no
    /// figure for it and prints one instead.
    ///
    /// Population: the rig's own rows — every (pose, muscle, coordinate) that
    /// `build(pose:source:)` reads — restricted to SINGLE-WRAP muscles
    /// (`wrapCount == 1`, and a row in the finite-difference fixture, so the two
    /// columns are genuinely two measurements rather than the same stored number)
    /// and excluding the 8 multi-wrap instances `opensim_multiwrap.txt` covers.
    /// Muscles with NO `PathWrap` are excluded for the same reason: there
    /// `value(_:.centralDifference)` returns the analytic value by construction,
    /// so including them would pack the distribution with structural zeros and
    /// deflate the p99 the falsifier is keyed to.
    ///
    /// **Falsifier 6 fires if p99 > 1.047 mm (the arm-discrepancy scale at R1's
    /// binding cell) or max > 10.43 mm (the smaller of the two recorded
    /// multi-wrap systematic errors).** Either firing means the FD column's
    /// disagreement is NOT confined to the multi-wrap class and the rejection of
    /// `centralDifference` has to be re-derived on the wider evidence.
    ///
    /// The second half — `worst_cell_smoothness` — is the anti-narration rule's
    /// discriminator: the wrap-point count across the ε = 1e-4 rad stencil and
    /// the second difference of THE REFERENCE'S OWN length. **Neither is
    /// derivable from the stored fixtures**: `opensim_moment_arms_fd.txt` stores
    /// the derivative and not the two lengths, and no fixture carries OpenSim's
    /// wrap-point count at `q ± ε`. Supplying them is an OpenSim regeneration,
    /// which this run is registered NOT to do. They are therefore printed as
    /// `NOT_AVAILABLE_FROM_STORED_FIXTURES` — which leaves the rule biting in the
    /// strict direction, since it makes a "marginal wrap pose" explanation
    /// INADMISSIBLE rather than assumed. What IS available at `q` is printed:
    /// both wrap-point counts and both lengths.
    func printColumnAgreementAndSmoothness(worstExact: Cell, worstShipped: Cell) throws {
        // The 8 instances `opensim_multiwrap.txt` covers, named rather than
        // inferred, because the registration names them.
        let multiWrap: Set<String> = ["gasmed_r", "gasmed_l", "gaslat140_r", "gaslat140_l",
                                      "TRIlong_r", "TRIlong_l", "BIClong_r", "BIClong_l"]
        var deltas: [Double] = []
        var worstRow = ""
        var worstDelta = -1.0
        for pose in Self.poseOrder {
            guard let table = Self.byPose[pose] else { continue }
            for (muscle, row) in table {
                guard !multiWrap.contains(muscle) else { continue }
                for coordinate in Self.rightLegCoordinates {
                    guard let sample = row[coordinate],
                          let central = sample.centralDifference,
                          sample.wrapCount == 1 else { continue }
                    let delta = abs(sample.wrapOn - central)
                    deltas.append(delta)
                    if delta > worstDelta {
                        worstDelta = delta
                        worstRow = String(format: "%@/%@ at %@ analytic %.6f mm central %.6f mm",
                                          muscle, coordinate, pose,
                                          1000 * sample.wrapOn, 1000 * central)
                    }
                }
            }
        }
        let p99 = 1000 * WrapValidationHarness.percentile(deltas, 0.99)
        let maximum = 1000 * (deltas.max() ?? 0)
        print("LEAK-METRIC column_vs_column_singlewrap n=\(deltas.count) "
              + "median_mm=\(1000 * WrapValidationHarness.percentile(deltas, 0.5)) "
              + "p99_mm=\(p99) max_mm=\(maximum) worst=\(worstRow) "
              + "falsifier6_p99_trigger_mm=1.047 falsifier6_max_trigger_mm=10.43 "
              + "falsifier6_fires=\(p99 > 1.047 || maximum > 10.43)")

        for (label, cell) in [("R1", worstExact), ("R2", worstShipped)] {
            let base = label == "R1" ? cell.worstExactBase : cell.worstBase
            guard !base.isEmpty else { continue }
            let name = "\(base)_r"
            let length = WrapValidationHarness.lengthSamples.first {
                $0.pose == cell.pose && $0.muscle == name
            }
            print("LEAK-METRIC worst_cell_smoothness gate=\(label) muscle=\(name) "
                  + "pose=\(cell.pose) loading=\(cell.loading.rawValue) "
                  + "truth=\(cell.truth.rawValue) shape=\(cell.shape) effort=\(cell.effort) "
                  + "eps_rad=1e-4 "
                  + "reference_wrap_points_at_q=\(length?.referenceWrapPoints.description ?? "-") "
                  + "our_wrap_points_at_q=\(length?.ourWrapPoints.description ?? "-") "
                  + "reference_length_at_q_m=\(length?.wrapOn.description ?? "-") "
                  + "our_length_at_q_m=\(length?.ours.description ?? "-") "
                  + "wrap_point_count_across_stencil=NOT_AVAILABLE_FROM_STORED_FIXTURES "
                  + "second_difference_of_reference_length_mm="
                  + "NOT_AVAILABLE_FROM_STORED_FIXTURES "
                  + "marginal_wrap_explanation_admissible=false")
        }
    }

    /// **THE REFERENCE DISAGREES WITH ITSELF BY MORE THAN THE WHOLE GATE BUDGET,
    /// and it is measured here on the identical scale as R1 rather than argued
    /// about.**
    ///
    /// R1 is `|d(ours, exact) − d(truth, exact)|` and it is maximised over BOTH
    /// definitions of `truth`, so it is only a statement about this codebase to
    /// the extent that the two definitions agree. They do not. This test puts
    /// OpenSim's OTHER column in the SUBJECT slot — same pose, same τ, same truth
    /// solve, same statistic — so "how far apart are OpenSim's own two answers"
    /// is a number in the same units as R1's 123.10 pp, and neither arm of the
    /// comparison contains a line of BioMotion geometry code.
    ///
    /// Why this is not a way out of R1: it is not. R1 stays failed and the flag
    /// stays `false`. What it decides is what the NEXT stage should work on. If
    /// this quantity is the size of R1's tail, then no amount of work on
    /// `MomentArmComputer` can pass R1 as registered, because the gate is taken
    /// over a `truth` that is not one — and the honest next step is a reference
    /// this repo can defend (`MultiWrapReferenceTests`' reconciled column is the
    /// first instalment) rather than another wrap solver.
    ///
    /// Delete this test if OpenSim ever ships a `calcLengthAfterPathComputation`
    /// that reports the length of the path it reports the points of; the failure
    /// of the assertion below is the signal that it did.
    ///
    /// **R1-P since R1 v2: computed on the v1 DIAGONAL ONLY**, with the pairing
    /// key extended to `pose|loading|truth|shape|effort` and restricted to
    /// `loading == truth`. Without that restriction the 2×2 would fill
    /// `pairedThem` with cells that never existed in v1 and the recorded
    /// 126.44 pp worst / 5.28 pp paired reference median would stop being
    /// comparable. This is an ADMISSIBILITY PRECONDITION for the truth
    /// definition, never a reopening condition: if it FAILS — OpenSim's two
    /// columns have converged to within the bar — no verdict flips; the
    /// registration re-opens and R1 must go back to the conservative max over
    /// both columns, because the reason for dropping one has evaporated.
    ///
    /// ⚠️ **The printed `cells_where_ours_exceeds_the_reference` is the count
    /// where OUR leak is LARGER** — the COMPLEMENT of the "466 of 582" narrated
    /// in `GaitLoadSummary`, which is additionally a PRE-SimmSpline count (the
    /// fix moved `pairedOurs`' median 0.9770 → 0.6571, so the count moved too,
    /// unmeasured). Read the field as printed, under its own name.
    func testTheReferenceDisagreesWithItselfByMoreThanTheGateAllows() throws {
        let cells = Self.sweep()
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let threshold = floor * Self.reopenFractionOfFloor
        let ours = Self.v1(.ours, in: cells)
        var rows: [String] = []
        var worstDisagreement = 0.0
        var pairedOurs: [Double] = [], pairedThem: [Double] = []
        for subject in Self.ArmSource.allCases where subject.isReference {
            let subset = Self.v1(subject, in: cells)
            guard !subset.isEmpty else { continue }
            let worst = subset.max { $0.leakExact < $1.leakExact }
            worstDisagreement = Swift.max(worstDisagreement, subset.map(\.leakExact).max() ?? 0)
            rows.append(String(format: "subject=%@ vs the other column: max %.4f on %@ at %@ "
                               + "| p99 %.4f median %.4f n=%d",
                               subject.rawValue, subset.map(\.leakExact).max() ?? 0,
                               worst?.worstExactBase ?? "-", worst?.pose ?? "-",
                               WrapValidationHarness.percentile(subset.map(\.leakExact), 0.99),
                               WrapValidationHarness.percentile(subset.map(\.leakExact), 0.5),
                               subset.count))
            // Paired on the identical cell key, so the comparison is not between
            // two differently-sampled populations. The key carries BOTH axes so
            // it cannot silently pair a cell with one that never existed in v1.
            func key(_ cell: Cell) -> String {
                "\(cell.pose)|\(cell.loading.rawValue)|\(cell.truth.rawValue)"
                    + "|\(cell.shape)|\(cell.effort)"
            }
            let index = Dictionary(subset.map { (key($0), $0) },
                                   uniquingKeysWith: { first, _ in first })
            for cell in ours {
                guard let theirs = index[key(cell)] else { continue }
                pairedOurs.append(cell.leakExact)
                pairedThem.append(theirs.leakExact)
            }
        }
        let ourMedian = WrapValidationHarness.percentile(pairedOurs, 0.5)
        let theirMedian = WrapValidationHarness.percentile(pairedThem, 0.5)
        let oursExceeds = zip(pairedOurs, pairedThem).filter { $0 > $1 }.count
        print("LEAK-METRIC reference_self_disagreement worst_pp=\(worstDisagreement) "
              + "threshold_pp=\(threshold) floor_percent=\(floor) "
              + "R1_worst_pp=\(ours.map(\.leakExact).max() ?? 0) "
              + "paired_cells=\(pairedOurs.count) paired_median_ours_pp=\(ourMedian) "
              + "paired_median_reference_pp=\(theirMedian) "
              + "paired_p90_ours_pp=\(WrapValidationHarness.percentile(pairedOurs, 0.9)) "
              + "paired_p90_reference_pp=\(WrapValidationHarness.percentile(pairedThem, 0.9)) "
              + "cells_where_ours_exceeds_the_reference=\(oursExceeds)/\(pairedOurs.count) "
              + "per_subject=\(rows)")

        XCTAssertGreaterThan(pairedOurs.count, 200,
                             "the comparison must be over a population of paired cells")
        XCTAssertGreaterThan(worstDisagreement, threshold,
                             "OpenSim's two columns agree to within the reopening bar "
                             + "(\(threshold) pp), so `truth` is well-defined after all and R1's "
                             + "tail is this codebase's to own — re-read the decision")
    }

    /// **THE SOLVER WAS THE BINDING CONSTRAINT AND IS NOT ANY MORE — this test
    /// changed direction on 2026-08-09, and that is what it was written to do.**
    ///
    /// It used to assert `median > floor`, in the direction of the defect, with
    /// the instruction "if it ever fails, the solver has been tightened and the
    /// whole per-muscle decision has to be re-read — do not delete it, re-run the
    /// decision". It failed at `4.4994e-05` against `8.086`, the decision was
    /// re-run (`testTheShippedFlagMatchesWhatTheMeasurementSupports`, still
    /// `false`, now on R1/R2 alone), and the assertion is now the tripwire in the
    /// other direction with the SAME quantity and a tighter number.
    ///
    /// Measured with the geometry held FIXED: the same arms, the same torques,
    /// OSQP's answer against the exact minimiser of the same objective. No
    /// reference model is involved, so no disagreement between OpenSim's two
    /// columns can explain either the old number or the new one.
    ///
    /// | | before (`scaling = 10`, no polish, 200 iterations) | after |
    /// |---|---|---|
    /// | median | 14.883 pp | **4.4994e-05 pp** |
    /// | p90 | 37.826 pp | **0.04714 pp** |
    /// | max | 100.977 pp | **21.981 pp** |
    /// | median relative torque residual | 2.7995e-03 | **8.316e-09** |
    ///
    /// **The max is 466× the p90 and it is NOT a solver failure in the sense the
    /// median measures.** A cell is screened on the EXACT solution being at least
    /// `interiorMargin = 1e-3` inside the box; a muscle 1.1e-3 inside is one an
    /// exact solver calls interior and any finite solver may put on the bound, and
    /// `100·(a_l − a_r)/mean` at `ā ≈ aMin` divides by a small number. The worst
    /// cell is printed with its base so the next stage can attribute it rather
    /// than inherit a number. The gate below is on the MEDIAN, which is what was
    /// pre-registered, and the p90 is asserted too so the tail cannot grow
    /// unnoticed.
    ///
    /// **R1 v2 registers this on `v1(.ours)` — the BYTE-FOR-BYTE v1 population,
    /// which retains the 21.981 pp outlier** — and prints the full-2×2 version
    /// beside it. Registering on the diagonal rather than the whole grid avoids
    /// resting a tripwire on an unproven claim that percentiles of an exactly
    /// doubled multiset equal the original's under this repo's percentile
    /// implementation. No reference model is involved in `solverSlack` at all, so
    /// neither population can be defended on truth grounds; continuity with the
    /// measurement this tripwire was set from is the reason.
    func testTheShippingSolversOwnSlackIsBelowWhatAnyClipCouldResolve() throws {
        let cells = Self.sweep()
        let readable = Self.v1(.ours, in: cells)
        let everything = Self.readable(.ours, in: cells)
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let threshold = floor * Self.reopenFractionOfFloor
        let slacks = readable.map(\.solverSlack)
        let median = WrapValidationHarness.percentile(slacks, 0.5)
        let p90 = WrapValidationHarness.percentile(slacks, 0.9)
        let activation = WrapValidationHarness.percentile(readable.map(\.medianActivation), 0.5)
        let worst = readable.max { $0.solverSlack < $1.solverSlack }
        let allSlacks = everything.map(\.solverSlack)
        print("LEAK-METRIC solver_slack median_pp=\(median) p90_pp=\(p90) "
              + "max_pp=\(slacks.max() ?? 0) at pose=\(worst?.pose ?? "-") "
              + "shape=\(worst?.shape ?? "-") effort=\(worst?.effort ?? 0) "
              // `worstBase` is the cell's worst SHIPPED leak, not its worst solver
              // slack — the cell does not record the latter per base. Named so,
              // rather than printed as if it were the muscle to look at.
              + "loading=\(worst?.loading.rawValue ?? "-") "
              + "truth=\(worst?.truth.rawValue ?? "-") "
              + "worst_shipped_base_in_that_cell=\(worst?.worstBase ?? "-") "
              + "screened_there=\(worst?.screened ?? 0) "
              + "median_activation=\(activation) "
              + "median_torque_residual="
              + "\(WrapValidationHarness.percentile(readable.map(\.torqueResidual), 0.5)) "
              + "floor_percent=\(floor) threshold_pp=\(threshold) cells=\(readable.count) | "
              + "full_2x2_median_pp=\(WrapValidationHarness.percentile(allSlacks, 0.5)) "
              + "full_2x2_p90_pp=\(WrapValidationHarness.percentile(allSlacks, 0.9)) "
              + "full_2x2_max_pp=\(allSlacks.max() ?? 0) full_2x2_cells=\(everything.count)")
        XCTAssertLessThan(median, threshold / 100,
                          "the solver's own termination slack moves a published left/right "
                          + "figure by \(median) pp; it was 14.883 pp before `scaling = 0` and "
                          + "`polishing = 1`, and the bar this file reopens a claim at is "
                          + "\(threshold) pp")
        XCTAssertLessThan(p90, threshold,
                          "and nine cells in ten must be inside the reopening bar itself, not "
                          + "just the median: \(p90) pp")
    }

    /// **R7 — the claim has to be informative as well as safe.** The retirement's
    /// second half was that where the error cancels, every muscle reads the same
    /// number. Measure both quantities on the same solves: the SPREAD of the true
    /// left/right figures across muscles, and the error in them.
    ///
    /// **R1 v2 re-populates AND STRENGTHENS R7 to `min(median_gated,
    /// median_v1) > 4.0`.** The v1 `leakShipped` median on record is 1.045 pp
    /// with ratio median 47.52; the GATED leakShipped median is recorded
    /// NOWHERE, so the direction of the filtering effect is genuinely unmeasured
    /// — which is exactly why `min(...)` is registered: it makes R7 no easier
    /// than v1 under either outcome. (Correction: R7's denominator is
    /// `leakShipped`, not `leakExact`; the 0.312-vs-0.977 figures once used to
    /// argue the direction are `leakExact` medians and 0.977 is itself pre-fix.)
    /// The pooled leg is reproducible only because the 2×2 keeps the v1 diagonal
    /// — under a naive truth filter both legs would collapse to one number and
    /// the strengthening would be vacuous.
    func testThePerMuscleDifferencesAreLargerThanTheErrorInThem() throws {
        let cells = Self.sweep()
        let readable = Self.gated(.ours, in: cells)
        let diagonal = Self.v1(.ours, in: cells)
        func ratio(_ population: [Cell], _ error: (Cell) -> Double) -> Double {
            WrapValidationHarness.percentile(
                population.map { error($0) > 0 ? $0.trueSpread / error($0) : Double.infinity }
                          .filter { $0.isFinite }, 0.5)
        }
        let shippedGated = ratio(readable, \.leakShipped)
        let shippedV1 = ratio(diagonal, \.leakShipped)
        let shipped = Swift.min(shippedGated, shippedV1)
        let exactGated = ratio(readable, \.leakExact)
        let exactV1 = ratio(diagonal, \.leakExact)
        let exact = Swift.min(exactGated, exactV1)
        let worstCell = readable.min {
            ($0.leakShipped > 0 ? $0.trueSpread / $0.leakShipped : .infinity)
                < ($1.leakShipped > 0 ? $1.trueSpread / $1.leakShipped : .infinity)
        }
        print("LEAK-METRIC information median_spread_over_shipped_error=\(shipped) "
              + "gated=\(shippedGated) v1_diagonal=\(shippedV1) "
              + "median_spread_over_moment_arm_error=\(exact) "
              + "exact_gated=\(exactGated) exact_v1_diagonal=\(exactV1) "
              + "min_at pose=\(worstCell?.pose ?? "-") shape=\(worstCell?.shape ?? "-") "
              + "spread_pp=\(worstCell?.trueSpread ?? 0) "
              + "shipped_error_pp=\(worstCell?.leakShipped ?? 0) median_spread_pp="
              + "\(WrapValidationHarness.percentile(readable.map(\.trueSpread), 0.5)) "
              + "cells=\(readable.count) v1_cells=\(diagonal.count) "
              + "R7_pass=\(shipped > Self.minimumInformationToLeakRatio)")
        XCTAssertGreaterThanOrEqual(readable.count, Self.minimumCells)
        // The MOMENT-ARM half of R7 is the half this stage's work could move, and
        // it is asserted. The shipped half is reported and consumed by the
        // decision test, which is where the verdict lives. Both take the same
        // `min(gated, v1)` form, so neither can be made easier by the
        // re-population: `min` is no larger than the v1 leg, which is what this
        // assertion measured before R1 v2.
        XCTAssertGreaterThan(exact, Self.minimumInformationToLeakRatio,
                             "with the arms this build computes, the per-muscle differences the "
                             + "panel would print must be larger than the moment-arm error in "
                             + "them")
    }

    // MARK: - The decision, pinned to the flag

    /// **The tripwire that keeps the shipped flag and this measurement in the
    /// same state.** It states the decision in one place: if the gates above
    /// pass, `perMuscleLeftRightClaimIsSupported` must be true; if any fails, it
    /// must be false. Weakening a gate to move the flag therefore breaks this
    /// test as well as that one.
    func testTheShippedFlagMatchesWhatTheMeasurementSupports() throws {
        let cells = Self.sweep()
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let decision = Self.decide(cells: cells, floor: floor)
        print("LEAK-METRIC decision supported=\(decision.supported) "
              + "exact_pp=\(decision.worstExact) "
              + "shipped_pp=\(decision.worstShipped) threshold_pp=\(decision.threshold) "
              + "floor_percent=\(decision.floor) "
              + "control_pp=\(decision.controlWorst) cells=\(decision.gatedCells) "
              + "median_spread_over_error=\(decision.ratio) "
              + "median_spread_over_error_gated=\(decision.ratioGated) "
              + "median_spread_over_error_v1=\(decision.ratioV1) "
              + "truth=\(Self.registeredTruth.rawValue) "
              + "shipped_flag=\(GaitLoadSummary.perMuscleLeftRightClaimIsSupported)")
        XCTAssertEqual(GaitLoadSummary.perMuscleLeftRightClaimIsSupported, decision.supported,
                       "the shipped flag and the measurement have diverged: moment arms "
                       + "\(decision.worstExact) pp, printed number \(decision.worstShipped) pp, "
                       + "threshold \(decision.threshold) pp")
    }

    /// **REOPENING THIS CLAIM NEEDS AN INSTRUMENT CHOSEN BEFORE ITS NUMBERS WERE
    /// READ — and this instrument was not.**
    ///
    /// `registeredTruth` was set to `.analytic` on 2026-08-13 with BOTH candidate
    /// values already known: 123.0833 pp pooled against 3.6932 pp analytic-only.
    /// STATUS next-step 33 warned about exactly this move — "Do not re-register
    /// it quietly … a gate whose reference is chosen after a number is read is
    /// not pre-registered" — and the R1 v2 registration ACCEPTS that charge
    /// rather than rebutting it. The consequence is asymmetric and is what this
    /// test states: R1 v2 may TIGHTEN a verdict (it drops the wall for every
    /// future run by roughly 33×) but it may never REOPEN a claim.
    ///
    /// **Why this is a separate test and not a conjunct in `supported`.** Folding
    /// a constant `false` into `supported` would make it a compile-time constant,
    /// degrading `XCTAssertEqual(flag, supported)` in the test above to
    /// `XCTAssertEqual(false, false)`. R1's and R2's MAX thresholds are asserted
    /// NOWHERE ELSE in this suite, so a later `reopenFractionOfFloor = 20.0`
    /// would then land silently — that neuters the one test the repo's own laws
    /// name as un-neuterable, and it kills the `LEAK-METRIC decision supported=`
    /// observable. So `supported` stays a function of the MEASURED gates and the
    /// pin stays bidirectional; this test carries the authority question.
    ///
    /// If the measured gates ever all pass while the flag is `false`, BOTH this
    /// test and the pin go RED. **That is the intended surfacing mechanism** — it
    /// forces the owner decision at that moment instead of flipping a product
    /// flag automatically. Reopening additionally needs a fully pre-registered
    /// instrument (the R9 lineage: a `reconciled` truth on the shared 173-pose
    /// grid, whose value is unknown) plus an explicit owner commit.
    func testReopeningThisClaimNeedsAnInstrumentChosenBeforeItsNumbersWereRead() throws {
        let cells = Self.sweep()
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let decision = Self.decide(cells: cells, floor: floor)
        print("LEAK-METRIC reopen_authority measured_gates_supported=\(decision.supported) "
              + "shipped_flag=\(GaitLoadSummary.perMuscleLeftRightClaimIsSupported) "
              + "registered_truth=\(Self.registeredTruth.rawValue) "
              + "truth_chosen_after_both_candidates_were_read=true "
              + "reopening_requires=[R9-lineage instrument, explicit owner commit]")
        XCTAssertFalse(decision.supported
                       && !GaitLoadSummary.perMuscleLeftRightClaimIsSupported,
                       "R1 v2's truth column was selected AFTER both candidate values were known "
                       + "(123.0833 pp pooled vs 3.6932 pp analytic-only). STATUS next-step 33: "
                       + "'a gate whose reference is chosen after a number is read is not "
                       + "pre-registered'. This instrument may TIGHTEN a verdict but may not "
                       + "REOPEN a claim. The measured gates now all pass while the shipped flag "
                       + "is false — that is an OWNER decision, not an automatic flip: it needs a "
                       + "fully pre-registered instrument (the R9 lineage, whose number is "
                       + "unknown) and an explicit owner commit. Do NOT satisfy this test by "
                       + "flipping the flag.")
    }

    // MARK: - R8: the attribution instrument (DIAGNOSTIC — it cannot move the flag)

    /// **R8 — is R1's residual even a path question?** Registered as a
    /// DIAGNOSTIC by R1 v2 and deliberately NOT a conjunct in `supported`.
    ///
    /// Restricting R1's gated population to one truth removes the only place the
    /// QP sharing step's amplification was ever measured: at the v1
    /// central-difference cell, `bflh140` — no `PathWrap`, no `MovingPathPoint`,
    /// no FD row, so its own arms are the SAME NUMBER in both reference matrices
    /// — took 126 pp from `gasmed`/`gaslat140`'s reference disagreement. STATUS
    /// next-step 35 states that this "applies to every per-muscle statement this
    /// product could ever make … validate a muscle's path, then trust its row is
    /// not a valid inference", and that nothing measures it. This is the
    /// replacement measurement rather than the removal of one.
    ///
    /// The method is one Jacobian of the QP solution map, at R1's own binding
    /// cell: perturb muscle `j`'s moment arms BILATERALLY (both `_l` and `_r`, so
    /// the perturbation is bilateral by construction exactly as the leak rig's
    /// is) by the SAME measured p99 relative residual R3 uses — `displayNames`
    /// muscles, reference arms ≥ 20 mm, pooled over both definitions — re-solve
    /// the EXACT QP, and record `max over i ≠ j` of the induced change in muscle
    /// `i`'s printed left/right figure.
    ///
    /// Reported against `floor × 0.2`. A value BELOW the bar would bound the
    /// coupling and make the ~3.7 pp analytic residual a genuine per-muscle
    /// geometry question (which is what R9 would then answer). A value ABOVE it
    /// does NOT change the flag — it constrains what the next stage may conclude
    /// (falsifier 7): per-muscle figures would not be attributable to per-muscle
    /// paths at this rig's coupling, and no amount of moment-arm accuracy could
    /// support the claim.
    func testTheSharingStepsPerMuscleCouplingIsMeasuredAndNotAssumed() throws {
        let cells = Self.sweep()
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let threshold = floor * Self.reopenFractionOfFloor
        let gated = Self.gated(.ours, in: cells)
        let cell = try XCTUnwrap(gated.max { $0.leakExact < $1.leakExact },
                                 "no gated cell — R8 has nothing to attribute")

        // The SAME perturbation size R3 uses, re-measured here rather than copied,
        // so the two cannot drift apart.
        var pooled: [Double] = []
        for definitionMatched in [true, false] {
            pooled += WrapValidationHarness.relativeMomentArmResiduals(
                bases: Set(GaitLoadSummary.displayNames.keys),
                minimumReferenceMetres: 0.020,
                definitionMatched: definitionMatched).ratios
        }
        XCTAssertGreaterThan(pooled.count, 1000,
                             "the perturbation must be sized from a population, not a handful")
        let perturbation = WrapValidationHarness.percentile(pooled, 0.99)

        let shape = try XCTUnwrap(Self.shapes.first { $0.name == cell.shape })
        let tau = try XCTUnwrap(Self.torques(pose: cell.pose, loading: cell.loading,
                                             activation: cell.effort, shape: shape.scales))
        let model = try XCTUnwrap(Self.build(pose: cell.pose, source: .ours))
        let lower = MuscleSolver().minActivation
        let upper = MuscleSolver.maxActivation
        func exactFigures(_ arms: [Double]) -> [String: Double] {
            let solution = BoxQP.solve(arms: arms, nMuscles: model.names.count,
                                       nDOFs: Self.dofNames.count, torques: tau,
                                       lower: lower, upper: upper)
            var activations: [String: Double] = [:]
            for (index, name) in model.names.enumerated() {
                activations[name] = solution.activations[index]
            }
            var figures: [String: Double] = [:]
            for base in Self.bases {
                if let d = Self.differencePercent(activations, base) { figures[base] = d }
            }
            return figures
        }
        // The rows are interleaved `_r`, `_l` per base by `build(pose:source:)`.
        var rowOf: [String: Int] = [:]
        for (index, name) in model.names.enumerated() { rowOf[name] = index }
        let baseline = exactFigures(model.armsInForceUnits)
        // The SAME screen the cell itself used: a base is read only where all
        // four of its exact activations sit strictly inside the box, in both the
        // truth solve and the subject solve. Using a looser set here would let
        // R8 report coupling onto a muscle the panel would never print.
        let truthSolve = try XCTUnwrap(Self.solve(pose: cell.pose, source: Self.registeredTruth,
                                                  torques: tau))
        let subjectSolve = try XCTUnwrap(Self.solve(pose: cell.pose, source: .ours, torques: tau))
        let readableBases = Self.bases.filter {
            Self.isScreened(truthSolve, subjectSolve, $0) && baseline[$0] != nil
        }

        var worstInduced = 0.0
        var worstPerturbed = "", worstMoved = ""
        let columns = Self.dofNames.count
        for perturbed in readableBases {
            var arms = model.armsInForceUnits
            var touched = false
            for side in ["r", "l"] {
                guard let row = rowOf["\(perturbed)_\(side)"] else { continue }
                touched = true
                for column in 0..<columns {
                    arms[row * columns + column] *= (1 + perturbation)
                }
            }
            guard touched else { continue }
            let moved = exactFigures(arms)
            for other in readableBases where other != perturbed {
                guard let before = baseline[other], let after = moved[other] else { continue }
                let induced = abs(after - before)
                if induced > worstInduced {
                    worstInduced = induced
                    worstPerturbed = perturbed
                    worstMoved = other
                }
            }
        }
        print("LEAK-METRIC R8_sharing_step_jacobian at pose=\(cell.pose) "
              + "loading=\(cell.loading.rawValue) truth=\(cell.truth.rawValue) "
              + "shape=\(cell.shape) effort=\(cell.effort) "
              + "perturbation_relative=\(perturbation) "
              + "bases_perturbed=\(readableBases.count) "
              + "worst_induced_change_pp=\(worstInduced) "
              + "perturbed_muscle=\(worstPerturbed) moved_muscle=\(worstMoved) "
              + "floor_percent=\(floor) threshold_pp=\(threshold) "
              + "coupling_bounded_by_the_bar=\(worstInduced < threshold) "
              + "gates_nothing=true")
        XCTAssertGreaterThan(readableBases.count, 20,
                             "R8 must perturb a population of muscles, not a handful")
    }

    // MARK: - TAIL ATTRIBUTION (DIAGNOSTIC — gates nothing, asserts no threshold)

    /// **Where R1 v2's 25.70 pp actually comes from, and how much moment-arm
    /// accuracy would remove it.** Added AFTER the R1 v2 verdict was read, under
    /// the freeze rule's "a defect found afterwards may be fixed" clause: this
    /// runs the localisation that decides whether there IS a defect to fix. It
    /// touches no gate, no population, no constant and no threshold, and it
    /// asserts nothing but its own preconditions.
    ///
    /// Three measurements, at BOTH binding cells (R1's `leakExact` argmax and
    /// R2's `leakShipped` argmax):
    ///
    /// 1. **The denominator.** `100·(a_l − a_r)/mean` diverges as `mean → aMin`,
    ///    and the screen only requires `mean > aMin + 1e-3`. Printing the carrier's
    ///    four exact activations says whether a 25 pp "leak" is a large change in a
    ///    force or a small change divided by a small number.
    /// 2. **Per-muscle substitution — the ACTUAL attribution, not a synthetic
    ///    one.** R8 perturbs every muscle by the same p99 residual and reports the
    ///    largest coupling. This instead starts from the TRUTH matrix and swaps in
    ///    OUR rows for exactly one base at a time, so each number is the share of
    ///    the observed leak carried by that muscle's own measured arm error. Also
    ///    run in the opposite direction (start from ours, restore one base to
    ///    truth) because the map is not linear and the two need not agree.
    /// 3. **The α ladder.** `A(α) = A_truth + α·(A_ours − A_truth)` for the WHOLE
    ///    matrix. `leak(α)` answers the only question that decides whether a fix
    ///    is worth looking for: by what factor would every moment arm in this build
    ///    have to improve for this cell to clear `floor × 0.2`? A ratio `leak(α)/α`
    ///    that is flat means the amplification is linear and the required accuracy
    ///    can be read straight off it; one that collapses at small `α` means an
    ///    active-set boundary, not a path error.
    ///
    /// Plus the population shape per quadrant, because "a tail" and "half the
    /// cells" call for different verdicts and R1 prints only a maximum.
    func testWhatCarriesTheTailAndHowMuchArmAccuracyWouldRemoveIt() throws {
        let cells = Self.sweep()
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let threshold = floor * Self.reopenFractionOfFloor
        let gated = Self.gated(.ours, in: cells)
        XCTAssertGreaterThanOrEqual(gated.count, Self.minimumCells,
                                    "nothing to attribute without a gated population")

        // --- Population shape, per (loading, truth) quadrant of the 2x2 ---
        for loading in Self.ArmSource.allCases where loading.isReference {
            for truth in Self.ArmSource.allCases where truth.isReference {
                let quadrant = Self.readable(.ours, in: cells)
                    .filter { $0.loading == loading && $0.truth == truth }
                guard !quadrant.isEmpty else { continue }
                let leaks = quadrant.map(\.leakExact)
                let over = leaks.filter { $0 >= threshold }.count
                var line = "LEAK-METRIC tail_shape loading=\(loading.rawValue) "
                line += "truth=\(truth.rawValue) n=\(quadrant.count) "
                line += "p50=\(WrapValidationHarness.percentile(leaks, 0.5)) "
                line += "p75=\(WrapValidationHarness.percentile(leaks, 0.75)) "
                line += "p90=\(WrapValidationHarness.percentile(leaks, 0.90)) "
                line += "p95=\(WrapValidationHarness.percentile(leaks, 0.95)) "
                line += "p99=\(WrapValidationHarness.percentile(leaks, 0.99)) "
                line += "max=\(leaks.max() ?? 0) cells_at_or_over_bar=\(over)/\(quadrant.count) "
                line += "bar_pp=\(threshold)"
                print(line)
            }
        }

        let r1 = try XCTUnwrap(gated.max { $0.leakExact < $1.leakExact })
        let r2 = try XCTUnwrap(gated.max { $0.leakShipped < $1.leakShipped })
        try attributeTail(label: "R1", cell: r1, carrier: r1.worstExactBase, threshold: threshold)
        if r2.pose != r1.pose || r2.worstBase != r1.worstExactBase {
            try attributeTail(label: "R2", cell: r2, carrier: r2.worstBase, threshold: threshold)
        }
    }

    private func attributeTail(label: String, cell: WrappedMomentArmLeakTests.Cell,
                               carrier: String, threshold: Double) throws {
        let shape = try XCTUnwrap(Self.shapes.first { $0.name == cell.shape })
        let tau = try XCTUnwrap(Self.torques(pose: cell.pose, loading: cell.loading,
                                             activation: cell.effort, shape: shape.scales))
        let truthModel = try XCTUnwrap(Self.build(pose: cell.pose, source: Self.registeredTruth))
        let ourModel = try XCTUnwrap(Self.build(pose: cell.pose, source: .ours))
        XCTAssertEqual(truthModel.names, ourModel.names,
                       "the two matrices must be row-aligned for a substitution to mean anything")
        let lower = MuscleSolver().minActivation
        let upper = MuscleSolver.maxActivation
        let columns = Self.dofNames.count
        var rowOf: [String: Int] = [:]
        for (index, name) in truthModel.names.enumerated() { rowOf[name] = index }

        func activations(_ arms: [Double]) -> [String: Double] {
            let solution = BoxQP.solve(arms: arms, nMuscles: truthModel.names.count,
                                       nDOFs: columns, torques: tau, lower: lower, upper: upper)
            var out: [String: Double] = [:]
            for (index, name) in truthModel.names.enumerated() {
                out[name] = solution.activations[index]
            }
            return out
        }
        func figure(_ arms: [Double]) -> Double {
            Self.differencePercent(activations(arms), carrier) ?? .nan
        }

        let truthActivations = activations(truthModel.armsInForceUnits)
        let ourActivations = activations(ourModel.armsInForceUnits)
        let dTruth = try XCTUnwrap(Self.differencePercent(truthActivations, carrier))
        let dOurs = try XCTUnwrap(Self.differencePercent(ourActivations, carrier))
        let observed = abs(dOurs - dTruth)

        // 1. The denominator.
        let tl = truthActivations["\(carrier)_l"] ?? .nan
        let tr = truthActivations["\(carrier)_r"] ?? .nan
        let ol = ourActivations["\(carrier)_l"] ?? .nan
        let orr = ourActivations["\(carrier)_r"] ?? .nan
        var denominator = "LEAK-METRIC tail_denominator gate=\(label) carrier=\(carrier) "
        denominator += "pose=\(cell.pose) loading=\(cell.loading.rawValue) "
        denominator += "truth=\(cell.truth.rawValue) shape=\(cell.shape) effort=\(cell.effort) "
        denominator += "a_min=\(lower) interior_margin=\(Self.interiorMargin) "
        denominator += "truth[a_l=\(tl) a_r=\(tr) mean=\(0.5 * (tl + tr)) figure_pp=\(dTruth)] "
        denominator += "ours[a_l=\(ol) a_r=\(orr) mean=\(0.5 * (ol + orr)) figure_pp=\(dOurs)] "
        denominator += "observed_leak_pp=\(observed) "
        denominator += "absolute_activation_change_l=\(abs(ol - tl)) "
        denominator += "absolute_activation_change_r=\(abs(orr - tr))"
        print(denominator)

        // 2. Per-muscle substitution, both directions, on the cell's own screen.
        let truthSolve = try XCTUnwrap(Self.solve(pose: cell.pose, source: Self.registeredTruth,
                                                  torques: tau))
        let ourSolve = try XCTUnwrap(Self.solve(pose: cell.pose, source: .ours, torques: tau))
        let screened = Self.bases.filter { Self.isScreened(truthSolve, ourSolve, $0) }
        func swapRows(_ into: [Double], from: [Double], base: String) -> [Double] {
            var arms = into
            for side in ["r", "l"] {
                guard let row = rowOf["\(base)_\(side)"] else { continue }
                for column in 0..<columns {
                    arms[row * columns + column] = from[row * columns + column]
                }
            }
            return arms
        }
        func armErrorMillimetres(_ base: String) -> Double {
            var worst = 0.0
            guard let row = rowOf["\(base)_r"] else { return 0 }
            for column in 0..<columns {
                worst = Swift.max(worst, abs(ourModel.arms[row * columns + column]
                                             - truthModel.arms[row * columns + column]) * 1000)
            }
            return worst
        }
        var forward: [(String, Double, Double)] = []   // base, induced pp, own arm error mm
        var backward: [(String, Double, Double)] = []
        // What the cell would read if ONE muscle's arms were exactly the truth's
        // and every other arm stayed as this build computes it. This is the
        // "best case if that muscle's path were perfect" number, and it is the one
        // that decides whether a fix on our side could clear the bar at all.
        var residualAfterRestoring: [(String, Double)] = []
        for base in screened {
            let onlyThisOneOurs = swapRows(truthModel.armsInForceUnits,
                                           from: ourModel.armsInForceUnits, base: base)
            let onlyThisOneTruth = swapRows(ourModel.armsInForceUnits,
                                            from: truthModel.armsInForceUnits, base: base)
            let restored = figure(onlyThisOneTruth)
            forward.append((base, abs(figure(onlyThisOneOurs) - dTruth), armErrorMillimetres(base)))
            backward.append((base, abs(restored - dOurs), armErrorMillimetres(base)))
            residualAfterRestoring.append((base, abs(restored - dTruth)))
        }
        forward.sort { $0.1 > $1.1 }
        backward.sort { $0.1 > $1.1 }
        residualAfterRestoring.sort { $0.1 < $1.1 }
        let bestSingleFix = residualAfterRestoring.prefix(4)
            .map { "\($0.0)->\($0.1)pp" }.joined(separator: " ")
        print("LEAK-METRIC tail_best_single_fix gate=\(label) carrier=\(carrier) "
              + "observed_leak_pp=\(observed) bar_pp=\(threshold) "
              + "leak_if_that_one_muscles_arms_were_exactly_the_truths: \(bestSingleFix)")
        func render(_ rows: [(String, Double, Double)]) -> String {
            rows.prefix(6).map { "\($0.0):\($0.1)pp(own_arm_err_mm=\($0.2))" }.joined(separator: " ")
        }
        let worstArmError: Double = screened.map(armErrorMillimetres).max() ?? 0
        let worstArmMuscle: String =
            screened.max { armErrorMillimetres($0) < armErrorMillimetres($1) } ?? "-"
        var substitution = "LEAK-METRIC tail_substitution gate=\(label) carrier=\(carrier) "
        substitution += "carrier_own_arm_error_mm=\(armErrorMillimetres(carrier)) "
        substitution += "screened=\(screened.count) observed_leak_pp=\(observed) "
        substitution += "worst_arm_error_in_cell_mm=\(worstArmError) at=\(worstArmMuscle) "
        substitution += "| truth_plus_one_ours: \(render(forward)) "
        substitution += "| ours_minus_one: \(render(backward))"
        print(substitution)

        // 3. The alpha ladder: how much better would EVERY arm have to be?
        var ladder: [String] = []
        var clearingAlpha: Double? = nil
        for alpha in [1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625, 0.0078125] {
            var arms = truthModel.armsInForceUnits
            for index in 0..<arms.count {
                arms[index] += alpha * (ourModel.armsInForceUnits[index] - arms[index])
            }
            let leak = abs(figure(arms) - dTruth)
            ladder.append("a=\(alpha):leak=\(leak)pp:ratio=\(leak / alpha)")
            if leak < threshold, clearingAlpha == nil { clearingAlpha = alpha }
        }
        // The same ladder scoped to the single dominant contributor, so the verdict
        // reader can price a partial improvement on ONE muscle's path against the
        // bar without extrapolating off the whole-matrix ladder.
        if let dominant = forward.first?.0 {
            var scoped: [String] = []
            for alpha in [1.0, 0.9, 0.8, 0.75, 0.7, 0.6, 0.5, 0.25, 0.1] {
                var arms = truthModel.armsInForceUnits
                for side in ["r", "l"] {
                    guard let row = rowOf["\(dominant)_\(side)"] else { continue }
                    for column in 0..<columns {
                        let index = row * columns + column
                        arms[index] += alpha * (ourModel.armsInForceUnits[index] - arms[index])
                    }
                }
                scoped.append("a=\(alpha):leak=\(abs(figure(arms) - dTruth))pp")
            }
            var scopedLine = "LEAK-METRIC tail_dominant_ladder gate=\(label) "
            scopedLine += "carrier=\(carrier) dominant=\(dominant) bar_pp=\(threshold) "
            scopedLine += "dominant_arm_error_mm=\(armErrorMillimetres(dominant)) "
            scopedLine += scoped.joined(separator: " ")
            print(scopedLine)
        }

        let clearedAt: String = clearingAlpha.map { "\($0)" } ?? "none_in_ladder"
        let factor: String = clearingAlpha.map { "\(1 / $0)" } ?? ">128"
        var report = "LEAK-METRIC tail_alpha_ladder gate=\(label) carrier=\(carrier) "
        report += "bar_pp=\(threshold) "
        report += "first_alpha_under_the_bar=\(clearedAt) "
        report += "required_arm_accuracy_factor=\(factor) "
        report += ladder.joined(separator: " ")
        print(report)
        XCTAssertGreaterThan(screened.count, Self.minimumScreenedBases - 1,
                             "attribution needs the cell's own screen, not a subset")
    }
}
