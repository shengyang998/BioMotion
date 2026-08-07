# Adversarial verification — IK stability / findings layer / polish (2026-08-07)

Everything below was measured by me on this machine. Where I could not measure something I say so.
I edited no source. I did create one temporary ObjC++ test (`BioMotionTests/ZZAuditTempTests.mm`),
ran it, deleted it, and re-ran `xcodegen generate`; `BioMotion.xcodeproj/project.pbxproj` is now
**byte-identical** to its pre-audit state (verified by `diff` against a copy taken before I started —
xcodegen is idempotent here, which I checked first).

---

## Headline

| Claim | Verdict |
|---|---|
| IK stability | **PARTIAL** — the improvement is real and large, but I broke the *unconditional* fixed-point claim on a pose the agent never tried |
| Findings layer | **CONFIRMED** — no clinical authority anywhere; my own hand calculations match to 5 dp |
| Polish (mask + gate + docs) | **CONFIRMED** with one caveat about an unsourced, user-visible "expected" range |
| Full suite except E1 | **197 executed, 0 failures, 0 crash-restarts** — reproduces the reported number exactly |

---

## 1. Was the red test made green by weakening it? NO.

* `git status BioMotionTests/NimbleBridgeTests.swift` → empty. The file is **unmodified**.
  `testRepeatedIKOnIdenticalMarkersIsStable` still asserts `XCTAssertLessThan(maxDelta, 1e-3)`
  against its original fixture, and passes.
* `git status nimblephysics/ osqp/` → **0 lines**. The vendored trees are untouched.
* The only test-file diffs are:
  * `StaticHoldTests.swift` — `XCTAssertGreaterThan(maxConsecutive, 0)` → `XCTAssertEqual(maxConsecutive, 0, accuracy: 0)`.
    That is **strictly tighter**, not looser: it now fails on *any* nonzero drift instead of requiring
    some. A second equally-tight assertion on `maxDriftFromFirst` was added.
  * `DOFMaskTests.swift` and `MuscleQPUnitsTests.swift` — **comment-only** (verified by reading the diffs).

No bound moved anywhere. This part of the claim is clean.

## 2. Is the dancer RMS genuinely lower, or measured differently? GENUINELY LOWER — on the same ruler.

I ran **both solvers in one process, on the same fixture, and measured both the same way** (unweighted
per-marker RMS over all 20 markers at the pose each returned). The old path is still callable:
`Skeleton::fitMarkersToWorldPositions` with the old config (`lossLowerBound = N·0.02²`, 5 restarts).

```
AUDIT|OLD|cold |unweightedRMS_cm=6.2372  weightedRMS_cm=2.6984  loss=1.456247e-02   (x3 trials, identical)
AUDIT|NEW|cold |reportedRMS_cm=2.1224    weightedRMS_cm=2.3951  loss=1.147258e-02   iters=177 converged=1
                my own recomputation of the NEW pose's RMS: 2.1224 cm  (matches the reported field exactly)
```

So on the *unweighted* ruler 6.24 → 2.12 cm, and on the *weighted* ruler (the one STATUS used to quote
as "2.6 cm") 2.6984 → 2.3951 cm. **Both conventions improved**; the headline is not a change of ruler.
My old-solver number (6.2372) is worse than the agent's reported 5.4913 — same direction, different
magnitude, most likely a different cold seed / process history. I did not chase it.

Repeated warm solves on **identical** dancer markers:

```
AUDIT|OLD|warm-drift k=0..4  maxAbsDelta_rad = 2.33e-1, 1.31e-1, 1.19e-1, 1.52e-1, 1.88e-1
                             and the fit gets WORSE each time: 4.44 → 5.33 → 5.73 → 5.85 → 5.39 cm
AUDIT|NEW|warm-drift k=0..3  maxAbsDelta_rad = 0.000000e+00 (all four), RMS 2.1224 cm unchanged
```

The old defect is real and the new behaviour on this fixture is real.

**Not returning the seed** (the obvious cheat): RMS at the neutral seed is 115.02 cm, after the solve
2.1224 cm, with `maxAbs(q − neutral) = 2.443 rad` and `‖q‖ = 5.746453` — the same ‖q‖ the agent
reported. The solver is doing work.

## 3. A pose the agent never used — AND HERE THE CLAIM BREAKS

I generated a pose with a deterministic LCG (seed 20260807; no `std::rand()`), sampling each *bounded*
coordinate at 35 % of its own range, pinning `pelvis_t{x,y,z}` to a realistic (0.06, 0.93, −0.04), then
forward-kinematics'd the 20 virtual markers. The target is therefore **exactly reachable** — the true
optimum is a zero residual.

```
AUDIT|D|cold  RMS_cm=0.000711  loss=5.771734e-10   iters=240  converged=0   <-- hit the 120/phase cap
AUDIT|D|k=0   drift=2.036813e-02 rad  dof=pro_sup_l  RMS_cm=0.000012  loss=1.608844e-13  iters=125 conv=0
AUDIT|D|k=1   drift=3.598383e-04 rad  dof=pro_sup_l  RMS_cm=0.000000  loss=4.479192e-17  iters=124 conv=0
AUDIT|D|k=2   drift=5.045141e-06 rad  dof=pro_sup_l  RMS_cm=0.000000  loss=1.998245e-18  iters=51  conv=1
AUDIT|D|k=3.. drift=0.000000e+00                                                          iters=0   conv=1
```

STATUS.md line 23 says *"Repeated solves on identical markers move exactly 0 rad"*, and line 876 says
*"The pose returned IS a stationary point, so the next call on the same markers passes its FIRST test
having moved nothing — the fixed point is a property of the termination rule, not of a tolerance."*

**That is over-general.** The fixed point is a property of `converged == YES`, not of the termination
rule. When a solve exits on the **iteration cap** (`kIKMaxIterations = 120` per phase) the returned
pose is *not* a stationary point, and the next call on identical markers keeps moving — here by
**2.04e-2 rad**, which is 20× the old red test's own 1e-3 rad bound. Three further solves are needed
before it settles.

The loss falls monotonically (5.77e-10 → 1.61e-13 → 4.48e-17 → 2.00e-18), so this is genuine
convergence work, not round-off. Fed through the Savitzky–Golay double differentiation
(gain ≈ 1/dt² ≈ 3600 at 60 fps), 2.04e-2 rad manufactures ≈ 73 rad/s² of acceleration the subject
never had — the exact link-2 mechanism STATUS claims is now broken.

Two aggravating details:

* **Nothing consumes `converged`.** `NimbleIKResult.converged` → `IKOutput.converged` exists, but
  `grep -rn converged BioMotion/Offline/ BioMotion/App/ BioMotion/ARKit/` returns **nothing**. A
  non-converged pose flows into SG → ID → QP with no flag and no badge.
* At 92 % of range (many coordinates pinned at joint limits) the cold solve *does* converge
  (166 iters) but the first two warm re-solves still drift 8.58e-9 and 5.51e-9 rad before hitting 0.

**What does NOT break:** determinism. Every order-independence probe I ran returned
`maxAbsDelta = 0.000000e+00` exactly — including a variant where a second freshly-loaded bridge ran
three dancer solves in between, and including the degenerate case below. Order independence is solid.

**When it doesn't happen:** add ±1.5 cm of marker noise (i.e. realistic input) and the cold solve
converges in 131 iters and warm drift is exactly 0 from the first repeat. The failure needs a
near-exactly-reachable target.

**Bonus degenerate case** (my first attempt, before I pinned the root translation): targets ~390 m from
the origin gave `RMS = 38726 cm, iters = 240, converged = 0`, and every warm solve after that burned
480 iterations (warm phase A+B, then the warm-start-reject cold retry A+B) without moving. Drift and
order-independence were still exactly 0. The solver never recovers from a far seed and never says so
except through `converged`, which nobody reads.

---

## 4. Findings layer — invented clinical authority? NONE FOUND.

**Every numeric literal in `PostureFindings.swift`, checked one by one:**

| Literal | What it is |
|---|---|
| `displayFloorCentimetres = 0.5`, `displayFloorDegrees = 1.0` | display grouping only; the file states in its own doc comment that it is NOT derived from any accuracy figure |
| `depthSuppressionFraction = 0.5` | equal-contribution crossover, `\|axis · optical axis\|` — derived, not tuned |
| `0.7071` | cos 45°, the stance-alignment gate |
| `0.15` | the HEAD marker's own offset from `head_neck`, i.e. MHRRetarget's marker definition |
| `1e-6` (×4) | degeneracy guards |
| `0.5` (×2) | midpoints of two joints |
| `180 / .pi`, `0.01`, `180`, `360` | unit conversion and angle wrapping |

**No normal range, no red/green line, no verdict, no severity colour.** The
`"No normal range is applied. These are measurements, not diagnoses — nothing here is a clinical
threshold."` line is rendered unconditionally in `PostureFindingsPanel.notes` (not behind the
disclosure toggle — I read the view body). The only non-`.secondary` colour in the panel is
`.orange` on a caveat glyph.
`PostureFindingsTests.testNoFindingCarriesAVerdictAndTheNoRangeNoteIsAlwaysVisible` checks each
finding's text against `["normal", "abnormal", "healthy", "poor posture", "should be", "ideal"]` —
a real, non-vacuous assertion.

**Markers only?** Yes. `grep -n "ikResult\|muscleResult\|idResult\|IKOutput\|MuscleOutput\|IDOutput\|NimbleEngine" BioMotion/Findings/*.swift`
returns only the doc comment forbidding it. `OfflinePlaybackView` passes `body.joints` and nothing else.

### My own hand calculations vs the code

I compiled `PostureFindings.swift` standalone with `swiftc` in `/tmp` against a 5-line stub
`TrackedJoint` — nothing in the repo was touched — and fed it subjects I built and computed by hand.

**Subject A — side view** (up = +Y, subject's right = +Z, anterior = +X, camera depth = +Z):

| Finding | My hand calculation | Code |
|---|---|---|
| forward_head | +7.000 cm | **+7.00000** |
| rounded_shoulders | +3.000 cm | **+3.00000** |
| kyphosis_proxy | atan(0.03/0.19) = 8.972627° | **+8.97262** |
| trunk_lean_sagittal | +3.000 cm | **+3.00000** |
| transverse_rotation | +15.000° (right shoulder forward) | **+15.00000, side = right** |
| shoulder_height / head_tilt / trunk_lean_lateral / weight_shift | must be suppressed, lateral axis is 100 % depth | **all 4 suppressed** |

**Subject B — frontal view**, which is the brief's explicit adversarial case:

| Finding | My hand calculation | Code |
|---|---|---|
| shoulder_height | +2.000 cm, left higher | **+2.00000, side = left** |
| head_tilt | +10.000°, toward the right | **+10.00000, side = right** |
| weight_shift | +4.000 cm, over the right foot | **+4.00000, side = right** |
| trunk_lean_lateral | −1.9980276 cm (independent numpy calc) | **−1.99803, side = left** |
| **forward_head** | **must be suppressed** | **SUPPRESSED**: *"needs a side-on (sagittal) view — in this frame 100 % of this measurement's axis lies along the camera's depth direction"* |
| rounded_shoulders, kyphosis_proxy, trunk_lean_sagittal, transverse_rotation | suppressed | **all suppressed** |

So a frontal photo does **not** produce a forward-head number. It produces a named reason. Confirmed.

Other gates I exercised myself: 45° oblique → 0 reported / 9 suppressed. `cameraDepthAxis = nil`
→ 9 suppressed with "camera direction unknown". Seated (leg axis > 45° off the trunk) → the three
stance findings suppressed with the stance reason, the rest still reported.

**Dancer fixture reproduced exactly**: orientation `oblique`, aDepth `0.6216724639187372`,
lDepth `0.7989297739873671`, vDepth `0.0833245480532883`, **0 findings, 9 suppressed**.

**Frame convention checked, not assumed**: `MHRRetarget.swift:19` documents and verifies
*"X = image-right, Y = up, Z = toward the camera"*, so `offlineCameraDepthAxis = (0,0,1)` is right.

**Property worth recording (not a defect):** the 0.5 gate is a cliff. Yawing the dancer fixture in 15°
steps flips the report between 0 and 5 findings across a single step (yaw 0° → 0 reported,
yaw +15° → 5 reported). Near the boundary, pose-model error of a few degrees changes whether a number
appears at all. That is the honest consequence of a hard gate; it is not hidden, but it is worth
knowing before the three-photo validation in task #25.

---

## 5. Polish

**DOF mask — correctly NOT adopted, and the tests assert the harm rather than a pass.**
`grep -rn "applyDOFMask\|clearDOFMask" BioMotion/` hits only `NimbleBridge`'s own definition and
comments — **no app code installs a mask**. `ShoulderRotMaskTests` asserts in both directions:
`delta["dancer"] > 0.05` (the mask must *hurt* by more than the pre-registered gate),
`abs(delta["standing"]) < 0.001` (it buys nothing), `maskedDrift["standing"] > 1e-9` (it *introduces*
drift). Those are live tripwires, not a vacuous green. The mask "changed numbers", it did not "help",
and the code reflects that. The side fix (mask pin `getPositions()` → `neutralSeedPose`) is a genuine
order-dependence bug removal and is in the diff.

**Plausibility gate — rejects the degenerate case, and I could not make it reject a normal subject,
but its bounds are unsourced and the user-facing wording is normative.**
`MHRRetarget.plausibility` reads only pose-invariant quantities (inter-hip-joint-centre distance and
the chain-sum stature) and runs in `OfflineSessionRunner.processOneFrame` *before* `nimble.scaleModel`.
Rejected frames keep their image and skeleton, get a red badge, are counted in the frame label, and
have posture findings suppressed. `BodyPlausibilityTests` (11) pass.

Important provenance check: the bounds were **not invented in this change**.
`git show HEAD:STATUS.md` line 857 already read *"hip width falls outside ~0.10–0.28 m or whose
chain-sum stature falls outside ~1.3–2.1 m"*. The polish agent implemented what was already written.
They are nonetheless **unsourced** — no citation exists anywhere in the repo — and the user-visible
string renders them as *"hip width came out 7 cm (expected 10–28 cm)"*. To an ordinary consumer
"expected" reads as a statement about bodies, which is exactly the register this workstream is
otherwise careful to avoid. The code comment says the right thing; the UI string does not inherit it.

Also confirmed: the gate has never run on the real `sample2` prediction — the tests build a synthetic
MHR skeleton to reproduce the recorded 0.070 m hip width. The agent stated this; it is true.

**`error` → RMS fix is complete, not partial.** `ikMarkerResidualMeters` is fed
`ikResult.markerRMSMeters` at all three `publishResults` sites (NimbleEngine.swift:484, 525, 707);
`ContentView` prints mm of the true RMS; `SessionChartsView` reads `markerRMSMeters`; the loss is
carried separately as `ikLossSquaredMeters`. No caller reads the loss as a length any more.

---

## 6. Full suite, run by me

```
xcodebuild -project BioMotion.xcodeproj -scheme BioMotion \
  -destination "platform=iOS Simulator,name=iPhone 17" \
  -skip-testing:BioMotionTests/E1MarkerSetComparisonTests test
→ Executed 197 tests, with 0 failures (0 unexpected) in 280.645 s
→ ** TEST SUCCEEDED **      "Restarting after unexpected exit": 0 occurrences
```

E1 excluded as instructed (>1 h, and it fails pre-existing at `E1MarkerSetComparisonTests.mm:475`,
163 vs 169 coordinates — I did not run it and cannot independently confirm that diagnosis).

**No regressions found.** Nothing went green by weakening.

---

## 7. What I could not check

* **On-device / Release timing.** Everything here is iPhone 17 Simulator, Debug. The reported
  ~1567 ms/frame moving-subject cost is unverified by me.
* **The old solver's 5.4913 cm.** I measured 6.2372 cm for the same configuration; I did not
  reconstruct the agent's exact process state.
* **E1MarkerSetComparisonTests.** Not run.
* **The panel on a real photo.** Same gap the two agents already declared: no Core ML model in this
  checkout, and the only real pose in the repo (the dancer) is correctly suppressed to zero findings.

## 8. Unsourced numeric thresholds, ranked by how much authority they imply

1. `MHRRetarget.min/maxHipWidthMeters = 0.10 / 0.28 m` and `min/maxStatureMeters = 1.30 / 2.10 m` —
   no citation; pre-dates this change; **surfaced to the user as "(expected 10–28 cm)"**.
2. `PostureFindings.displayFloorCentimetres = 0.5` / `displayFloorDegrees = 1.0` — arbitrary by the
   file's own admission; decides what headlines the panel vs. what is filed under "no measurable
   deviation".
3. `ContentView` marker-RMS badge turns green under **20 mm** — a quality cut with no stated basis.
4. `depthSuppressionFraction = 0.5` and `0.7071` — derived (cos 45°, equal-contribution crossover),
   so these are sourced by argument rather than unsourced.
