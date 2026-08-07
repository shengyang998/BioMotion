import Foundation
import Combine
import QuartzCore
import simd
import os

/// Manages the Nimble physics engine lifecycle and provides real-time IK/ID results.
/// Runs Nimble on a background queue to avoid blocking the main thread.
final class NimbleEngine: ObservableObject {
    @Published var isModelLoaded = false
    @Published var lastIKResult: IKOutput?
    @Published var lastIDResult: IDOutput?
    @Published var lastMuscleResult: MuscleOutput?
    @Published var displayMuscleResult: MuscleOutput?
    @Published var ikSolveTimeMs: Double = 0
    @Published var idSolveTimeMs: Double = 0
    @Published var muscleSolveTimeMs: Double = 0

    // --- Accuracy metrics (for UI diagnostics) ---
    /// RMS marker residual from the most recent IK solve, in meters.
    @Published var ikMarkerResidualMeters: Double = 0
    /// Max |joint torque| / total body mass from the most recent ID solve, in Nm/kg.
    /// Physiological range for walking/squat: ~1–3 Nm/kg. Values above 10 indicate
    /// broken pipeline (usually missing GRF or bad ddq).
    @Published var maxTorquePerKg: Double = 0
    /// Total mass of the loaded (possibly scaled) skeleton, in kg.
    @Published var totalMassKg: Double = 0
    /// Left/right foot vertical GRF as fractions of body weight (0-1.x typically).
    /// Sum should be ~1.0 in steady stance, 0 in flight, 1.0-3.0 during impact.
    @Published var leftFootLoadFraction: Double = 0
    @Published var rightFootLoadFraction: Double = 0
    /// Linear-momentum residual after the GRF solve, in NEWTONS per kg:
    /// ‖ΣF_contact + m·g − m·a_com‖ / m. A correct pipeline reports ~0 every
    /// frame — it is a consistency check on the contact-wrench readback, not a
    /// measure of how balanced the pose is. See `NimbleIDResult.rootResidualNorm`.
    @Published var rootResidualPerKg: Double = 0
    /// Current ground-plane height (ARKit world y), for display only.
    @Published var groundHeightY: Double = 0
    /// The most recent complete solve, or nil while the Savitzky-Golay window
    /// is still filling. Prefer this over the individual fields above when you
    /// need IK, ID, muscle and the motion verdict to describe the SAME instant.
    @Published private(set) var lastSolve: SolveRecord?

    /// When true, inverse dynamics and the muscle solve run ONLY on frames the
    /// hold detector marks as static, and they run as a static-equilibrium
    /// problem (q̇ = q̈ = 0). Frames that fail the hold test publish pose and a
    /// `MotionClassification` with no ID/muscle at all.
    ///
    /// Default OFF, and the offline import path turns it on. The reason it is
    /// a switch rather than unconditional is that the two input paths differ in
    /// what they can observe:
    ///
    /// * The offline (photo/video) path goes through `MHRRetarget`, whose
    ///   `joint_coords` source pins the pelvis at the model constant
    ///   (0, 0.924, 0) in EVERY frame because `global_trans` is zeroed
    ///   (`sam3d_body.py:1600`). Joint ANGLES survive that, but the body has no
    ///   global translation, so `M·q̈` and the centre-of-mass acceleration are
    ///   computed from motion that did not happen — in a squat the pelvis never
    ///   descends and the feet appear to rise instead.
    /// * The live ARKit path supplies real world-space joint positions
    ///   including global translation, so its q̈ is observable and gating it
    ///   would remove working behaviour. Untouched by default.
    ///
    /// ⚠️ **The pinning is not a limitation of the pose model — it is a
    /// quantity the app has and does not consume.** The model emits the root
    /// translation separately as `cam_t`; it is exported, stored on
    /// `FrameResult`, and already used to project the overlay.
    /// `MHRRetarget.makeBodyFrame(jointCoords:camT:…)` composes it back in, and
    /// `OfflineSessionRunner` has `estimate.camT` in hand at the call site.
    /// Verified on 309 frames of real video: depth 4.34 m, 1.10 m below the
    /// optical axis, `corr(1/bbox_side, depth) = +0.74`.
    ///
    /// Composing it in is NECESSARY for dynamics and NOT SUFFICIENT. Measured,
    /// same clip: the depth channel carries 12.7 cm (std) of high-frequency
    /// residual about its own 0.5 s mean, which through this engine's own
    /// 9-tap Savitzky-Golay filter is 3.1 g of pure acceleration noise at
    /// 30 fps, 0.23 g at 10 fps. The in-plane channels are 8-25× cleaner. And
    /// `cam_t` is CAMERA-relative, so a rotating camera (13.5 °/s on the user's
    /// `video_012`) makes the frame non-inertial regardless. See STATUS.md,
    /// "cam_t recovers the root translation; its depth cannot be differentiated
    /// twice", and `MotionClassification.rootTranslationObservable`, which
    /// reports whether the stream this engine is being fed actually carries it.
    ///
    /// Read and captured on the MAIN thread inside `processFrame`, which is
    /// already a main-thread API (`isFrameInFlight`/`droppedFrameCount` are
    /// documented as main-only), so there is no cross-queue race.
    var staticHoldGating: Bool = false

    /// Processed IK output with named DOFs.
    struct IKOutput {
        let jointAngles: [String: Double]  // DOF name → angle in radians

        /// TRUE per-marker RMS position error, in METRES. This is the number to
        /// show a user or compare against a distance.
        let markerRMSMeters: Double

        /// The solver's LOSS: `Σ wᵢ²‖p_model,i − p_target,i‖²`, in **m²**.
        /// Kept because loss-domain bounds are compared against it, and named
        /// so it cannot be printed as a length again.
        ///
        /// ⚠️ This field was called `error` and documented as "RMS marker error
        /// in meters" until 2026-08-07. It is neither. On the dancer fixture the
        /// loss is 0.0138, which read as metres says "1.4 cm" while the true
        /// per-marker RMS is 5.5 cm — and the weights are below 1 on exactly the
        /// markers that fit worst, so the misreading always flatters the fit.
        /// `ContentView` printed it as `"%.3f m"` with a green cut at 0.05, i.e.
        /// it showed a *squared* quantity as a length and called anything under
        /// 0.05 good.
        let ikLossSquaredMeters: Double

        let timestamp: TimeInterval
    }

    /// Processed ID output with named DOFs.
    struct IDOutput {
        let jointTorques: [String: Double]  // DOF name → torque in Nm
        let timestamp: TimeInterval

        // Ground-reaction-force diagnostics. Forces in newtons (world frame),
        // CoPs in meters (world frame). Zero when the foot is not in contact.
        var leftFootForce: SIMD3<Double> = .zero
        var rightFootForce: SIMD3<Double> = .zero
        var leftFootCoP: SIMD3<Double> = .zero
        var rightFootCoP: SIMD3<Double> = .zero
        var leftFootInContact: Bool = false
        var rightFootInContact: Bool = false
        /// Norm of the 6D residual at the floating root joint. Should be
        /// small (< ~10 Nm) when GRF and kinematics are consistent.
        var rootResidualNorm: Double = 0
    }

    /// Processed muscle optimization output.
    struct MuscleOutput {
        let activations: [String: Double]  // display name (alias-merged) → activation 0-1
        let forces: [String: Double]       // display name → force in N
        let converged: Bool
        let timestamp: TimeInterval
        /// Activations keyed by RAW solver names (pre alias-merge). Path
        /// rendering needs these because `paths` is keyed by raw names.
        var rawActivations: [String: Double] = [:]
        /// Anatomy-exact world-space endpoints for every muscle, captured
        /// from the skeleton FK at the same pose the solver consumed. Used
        /// by the overlay to draw path-based capsules for muscles not in
        /// the hardcoded-def set (i.e. upper body, trunk, etc.).
        var paths: [String: MusclePath] = [:]
        /// Peak isometric force (N) per muscle (raw names), for radius scaling.
        var maxForces: [String: Double] = [:]
    }

    struct MusclePath {
        let start: SIMD3<Float>
        let end: SIMD3<Float>
    }

    /// Why a frame does or does not carry muscle magnitudes.
    ///
    /// The point of splitting this out of a boolean: "no muscle numbers" had
    /// exactly one user-facing explanation — *"muscle loads need a still pose"*
    /// — and the user can do nothing with it except freeze. These cases have
    /// different causes and different remedies, and two of them are not the
    /// user's fault at all.
    enum MotionVerdict: Equatable {
        /// Measured still to within the budget. ID and the muscle QP run as a
        /// static-equilibrium problem (q̇ = q̈ = 0).
        case hold
        /// Measurably moving, by more than the budget static equilibrium can
        /// absorb AND by more than the instrument's own noise. Withhold: the
        /// static reading would be a different answer, not a cleaner one.
        ///
        /// This build has no dynamic branch to fall through to, so a moving
        /// frame is withheld whether or not `rootTranslationObservable` is
        /// true. Read that field alongside this case — it says which of the two
        /// things is missing.
        case movingBeyondStaticBudget
        /// The measured marker speed exceeds the stillness bound, but so does
        /// this clip's own pose-estimation noise floor, so the two cannot be
        /// told apart. NOT the same as "the subject moved": at a sparse
        /// sampling rate a perfectly still subject lands here.
        case indistinguishableFromNoise
        /// Nothing measurable in the window (first sample of a clip, no marker
        /// in common with the predecessor, non-increasing timestamps).
        case noMeasurement
        /// The IK solve stopped on its iteration cap rather than at a
        /// stationary point, so the pose is drawable but its derivatives are
        /// not trustworthy and no dynamics ran.
        ///
        /// This case existed as behaviour before it existed as a word: the
        /// engine already withheld dynamics on a non-converged solve, but it
        /// published whatever the STILLNESS test had concluded, so a frame the
        /// solver could not settle was reported to the user as "the subject
        /// moved" — advice they cannot act on, about a thing that did not
        /// happen. On the running path it was worse: the frame fell out of the
        /// gait vocabulary entirely.
        case poseDidNotConverge
        /// RUNNING. This instant is inside a foot contact, and inverse dynamics
        /// ran with the root acceleration the gait cycle implies rather than
        /// with one differentiated out of the data. Muscle magnitudes exist and
        /// are to be read as RELATIVE loads within this contact.
        case gaitStance
        /// RUNNING. This instant is in flight. Neither foot is on the ground,
        /// so there is no stance load to report and none is claimed.
        case gaitFlight
        /// A running clip was analysed, but this frame fell outside the window
        /// the gait model covers (before the first complete contact, after the
        /// last, or in a gap where the pose was lost).
        case gaitOutsideAnalysis
        /// The clip's own gait model refused — the strides are not alike enough,
        /// or there are too few complete contacts — so no dynamics ran at all.
        case gaitRefused

        /// One sentence the user can act on. Lives here rather than in the view
        /// so it is covered by unit tests and cannot drift from the numbers.
        ///
        /// Every case has to answer "what do I do differently?". Two of them
        /// answer "film at a higher frame rate", which is a real, actionable
        /// lever and the ONLY one that moves the left/right resolution.
        var advice: String {
            switch self {
            case .hold:
                return ""
            case .movingBeyondStaticBudget:
                return "The subject moved too fast for a still-pose reading. Hold the position, or sample the clip at a higher rate so a shorter stretch of stillness is enough."
            case .indistinguishableFromNoise:
                return "The pose estimate jitters as much as the movement being measured, so stillness cannot be confirmed. Fill more of the frame, improve the lighting, or sample at a higher rate."
            case .noMeasurement:
                return "Not enough frames around this instant to measure motion."
            case .poseDidNotConverge:
                return "The skeleton did not settle on this frame, so no load is claimed for it. Keep the whole body in frame and unobstructed."
            case .gaitStance:
                return ""
            case .gaitFlight:
                return "Both feet are off the ground here, so there is no ground load to measure. Scrub to a frame where a foot is down."
            case .gaitOutsideAnalysis:
                return "This frame sits outside the strides the analysis covers. Scrub into the middle of the clip, or film a longer stretch of steady running."
            case .gaitRefused:
                return "The strides in this clip are not alike enough to model as a repeating cycle. Film a longer run at a steady pace, side-on, and keep the whole body in frame."
            }
        }

        /// True for the two cases that mean "running, and the gait cycle — not
        /// the static-hold detector — decided what happens on this frame".
        var isGait: Bool {
            switch self {
            case .gaitStance, .gaitFlight, .gaitOutsideAnalysis, .gaitRefused: return true
            case .hold, .movingBeyondStaticBudget, .indistinguishableFromNoise,
                 .noMeasurement, .poseDidNotConverge: return false
            }
        }
    }

    /// What the gait dynamics path did on ONE frame, and the number that can
    /// contradict it.
    ///
    /// # The falsifier, and why it is not closed by construction
    ///
    /// The root's vertical acceleration is SET from the gait model:
    /// `a_root,y = g·(F/mg − 1)`, where `F/mg` is the half-sine stance force
    /// implied by contact and flight timing alone. That is the whole point of
    /// the route — the root acceleration need not be measured.
    ///
    /// Inverse dynamics then returns its OWN answer for the total contact
    /// force, `ΣF_contact = m·a_com − m·g`. Those two are not the same
    /// quantity: `a_com = a_root + a_artic`, and `a_artic` — the centre of mass
    /// accelerating relative to the root as the limbs swing — comes from the
    /// skeleton's mass distribution and the measured joint accelerations, which
    /// the gait model has no access to. So
    ///
    ///     residual = ‖ΣF_contact − F_gait‖ / (m·g) = ‖a_artic‖ / g
    ///
    /// is a real, computed disagreement between the timing model and the
    /// articulated body, in body weights. It is exactly the term the
    /// whole-body half-sine model throws away, and when it is large the
    /// model's force is not a common rescaling of the contact any more — which
    /// is the assumption every ratio this product reports rests on.
    ///
    /// ⚠️ **What it does NOT test, stated without a substitute that does.** The
    /// half-sine SHAPE and the peak magnitude enter both sides (`a_root` was
    /// derived from them), so they cancel exactly and this residual is blind to
    /// them. **Nothing in this pipeline tests the peak magnitude**, and the
    /// clip-level checks do not fill the gap:
    /// `GaitReport.contactSequencePeriodicityErrorFrames` is an algebraic
    /// identity on any periodic alternating schedule (proved in
    /// `GaitReportTests`) and `GaitReport.steadiness` tests periodicity, not
    /// force. That gap is admissible only because of what the product claims —
    /// see `GaitLoadSummary`: a peak-force error is a COMMON SCALE over every
    /// muscle in a contact and cancels out of every ratio, provided (i) it
    /// really is common, which is what this residual tests, and (ii) the muscle
    /// QP stays linear, which fails at `a ≤ 1` and is gated per muscle.
    ///
    /// # Why the root acceleration is overridden rather than left to residual
    ///
    /// The alternative design applies the gait GRF as an external wrench and
    /// runs plain inverse dynamics, so the ROOT residual becomes a genuine
    /// force/moment residual. It was rejected on a measured fact, not a
    /// preference: this pose source PINS THE PELVIS (`MHRRetarget`, root
    /// translation held at the model constant 0.92398697), so the measured
    /// `a_root` is identically 0 and the root residual would be
    /// `‖m·a_artic − m·g − F_gait‖ ≈ 3.9·m·g` on every stance frame of every
    /// clip. A quantity that reports the same failure on good and bad footage
    /// alike is a constant, not a falsifier. It becomes available if and when
    /// `cam_t` is composed into the stream AND its depth channel is usable —
    /// STATUS measures 3.11 g of pure noise there at 30 fps, so not yet.
    struct GaitFrameOutcome: Equatable {
        /// `F/(m·g)` from the gait model — timing only.
        let modelledVerticalForceInBodyWeights: Double
        /// `ΣF_contact,y/(m·g)` as inverse dynamics solved it.
        let solvedVerticalForceInBodyWeights: Double
        /// `‖ΣF_contact − F_gait‖/(m·g)`. The falsifier.
        let residualInBodyWeights: Double
        /// −1 left foot down, +1 right, 0 flight — from the KINEMATIC stance
        /// detector (pelvis-relative horizontal foot velocity).
        let contactSide: Int
        /// What the ID solver's own GEOMETRIC contact detector saw (foot height
        /// versus the estimated ground plane). Two independent detectors on two
        /// different signals; when they disagree, one of them is wrong about
        /// which foot is carrying the load.
        let solverSawLeftContact: Bool
        let solverSawRightContact: Bool
        /// The acceleration actually written into the root's vertical DOF.
        let rootVerticalAccelerationMetersPerSecondSquared: Double
        /// **Always false, and published rather than hidden.** The gait model
        /// supplies a vertical force only; it says nothing about the braking
        /// and propulsive force along the running direction. That term is left
        /// UNMODELLED rather than silently set to zero, and this flag is what
        /// carries that fact to the user.
        let horizontalRootAccelerationModelled: Bool
        /// True when this frame's centred derivative window lies entirely inside
        /// its own contact, so `q̈` was not fitted across a touchdown or a
        /// toe-off. False frames still publish a pose; their muscle numbers do
        /// not enter the load summary. See `GaitPlan.Frame`.
        let derivativeWindowInsideContact: Bool

        /// True when the two contact detectors agree about this frame — which
        /// means agreeing about BOTH feet, not just the claimed one.
        ///
        /// ⚠️ Asking only "did the solver also see the claimed foot down?" lets
        /// a DOUBLE contact through, and a double contact is the one error mode
        /// this product cannot absorb. `NimbleBridge.solveIDGRF` splits the
        /// Newton-Euler wrench guess `weightUp / contactCount` between the feet
        /// it thinks are down, and
        /// `Skeleton::getMultipleContactInverseDynamicsNearCoP` is a
        /// least-squares solve *around that guess* whose steps live in the
        /// constraint null space — so a spurious second contact leaves the
        /// stance foot carrying roughly HALF its real ground force and hands the
        /// swing leg a ground reaction that does not exist. Every activation the
        /// QP derives from those torques is halved on one side only, which
        /// fabricates precisely the left/right asymmetry the product exists to
        /// report.
        ///
        /// The residual cannot catch it. `residualInBodyWeights` is built from
        /// `leftFootForce.y + rightFootForce.y` — the SUM over both feet — and
        /// the near-CoP constraint fixes that sum exactly, so a 50/50 split and
        /// a 100/0 split produce the identical number. This flag is the only
        /// place the split is visible, so it has to be the place that checks it.
        ///
        /// ⚠️ When the detectors DISAGREE the residual above is not `‖a_artic‖/g`
        /// at all: `solveIDGRF` returns zero ground force when its own geometric
        /// detector sees no foot down, so the residual becomes the ENTIRE
        /// modelled force (~2-3 BW) and says nothing about limb inertia. The two
        /// regimes are therefore separated by `GaitLoadSummary`, which computes
        /// its residual statistic over agreeing frames only and gates the
        /// disagreements as their own, separately-named failure.
        var contactDetectorsAgree: Bool {
            switch contactSide {
            case -1: return solverSawLeftContact && !solverSawRightContact
            case 1: return solverSawRightContact && !solverSawLeftContact
            default: return !solverSawLeftContact && !solverSawRightContact
            }
        }

        /// The solver put ground force under BOTH feet on a frame the kinematic
        /// detector called single stance. Published separately from
        /// `contactDetectorsAgree` because it is a different failure with a
        /// different lever: "no foot down" points at the ground-height estimate,
        /// "both feet down" points at the 6 cm contact threshold against the
        /// swing foot's clearance. Diagnostic only — the gate is
        /// `contactDetectorsAgree`, which already covers this case.
        var solverSawDoubleContact: Bool {
            solverSawLeftContact && solverSawRightContact
        }

        /// Every condition this frame's muscle numbers need in order to enter a
        /// left/right or muscle-to-muscle comparison.
        var isUsableForLoadComparison: Bool {
            contactDetectorsAgree && derivativeWindowInsideContact
        }
    }

    /// What the hold detector concluded about one instant.
    ///
    /// `timestamp` is the Savitzky-Golay window CENTRE — the same instant
    /// `IDOutput`/`MuscleOutput` are dated at — not the newest pushed frame.
    struct MotionClassification: Equatable {
        let timestamp: TimeInterval
        /// True iff every measured marker speed in the examined window stayed
        /// under `StaticHoldDetector.holdSpeedThresholdMetersPerSecond` AND the
        /// window was long enough to bound the discarded acceleration.
        /// Exactly `verdict == .hold`.
        let isHold: Bool
        /// Largest per-marker speed seen anywhere in the window, m/s.
        let peakMarkerSpeedMetersPerSecond: Double
        /// Median over the window of the per-sample median-over-markers speed,
        /// m/s. Diagnostic only: if `peak` is high but this is low, one marker
        /// moved (or one marker is noisy), not the body.
        let medianMarkerSpeedMetersPerSecond: Double
        /// Span of the examined window, seconds.
        let windowSeconds: Double
        /// Number of samples examined (including the ones with no predecessor).
        let sampleCount: Int
        /// `2 · peak / windowSeconds` — the bound this window puts on the mean
        /// acceleration that static-equilibrium ID throws away. m/s².
        let impliedMeanAccelMetersPerSecondSquared: Double
        /// The reason behind `isHold`, and the sentence to show for it.
        let verdict: MotionVerdict
        /// A LOWER BOUND on how much of `peakMarkerSpeedMetersPerSecond` is
        /// pose-estimation noise rather than the subject moving, in m/s,
        /// measured on THIS clip from distances that physically cannot change.
        /// See `StaticHoldDetector.rigidPairs`.
        let poseNoiseFloorMetersPerSecond: Double
        /// False when the pose source pins the pelvis, i.e. the body carries no
        /// global translation and the root's contribution to `M·q̈` is missing.
        /// See `MHRRetarget.rootTranslation(camT:)`.
        let rootTranslationObservable: Bool

        /// The same measurement under a different verdict. Used by the gait
        /// path, where the reason a frame does or does not carry muscle numbers
        /// is decided by the gait cycle rather than by the stillness test — but
        /// the stillness numbers are still worth publishing, because they are
        /// what a reader checks when the two disagree.
        func replacingVerdict(_ newVerdict: MotionVerdict) -> MotionClassification {
            MotionClassification(timestamp: timestamp,
                                 isHold: newVerdict == .hold,
                                 peakMarkerSpeedMetersPerSecond: peakMarkerSpeedMetersPerSecond,
                                 medianMarkerSpeedMetersPerSecond: medianMarkerSpeedMetersPerSecond,
                                 windowSeconds: windowSeconds,
                                 sampleCount: sampleCount,
                                 impliedMeanAccelMetersPerSecondSquared: impliedMeanAccelMetersPerSecondSquared,
                                 verdict: newVerdict,
                                 poseNoiseFloorMetersPerSecond: poseNoiseFloorMetersPerSecond,
                                 rootTranslationObservable: rootTranslationObservable)
        }
    }

    /// Everything one warm solve produced, all dated at the SAME Savitzky-Golay
    /// centre timestamp.
    ///
    /// The individual `@Published` fields below are updated independently, so a
    /// consumer that reads `lastMuscleResult` and `lastIKResult` in the same
    /// turn can pick up a muscle result from an OLDER solve alongside a fresh
    /// IK result — which matters now that a moving frame publishes IK with no
    /// muscle at all. This bundle is the consistent view: everything in it
    /// belongs to `centerTimestamp` or is nil.
    struct SolveRecord {
        let centerTimestamp: TimeInterval
        let motion: MotionClassification
        let ik: IKOutput
        let id: IDOutput?
        let muscle: MuscleOutput?
        /// True iff `id` and `muscle` were solved with q̇ = q̈ = 0 because the
        /// subject was measured to be holding still. False means they came from
        /// the Savitzky-Golay derivatives (live-camera path).
        let isStaticHoldEstimate: Bool
        /// Non-nil only on the running path. See `GaitFrameOutcome`.
        var gait: GaitFrameOutcome? = nil
    }

    // MARK: - Gait dynamics

    /// Per-frame root acceleration and ground force for a RUNNING clip, derived
    /// from contact/flight timing by `GaitAnalysis` and handed to the engine.
    ///
    /// Deliberately made of primitives: the engine does not import the gait
    /// module's types, so the two can be tested apart and the seam is one
    /// value type wide.
    struct GaitPlan: Equatable {
        struct Frame: Equatable {
            let timestamp: TimeInterval
            /// Vertical GRF at this instant in BODY WEIGHTS. Zero in flight.
            let verticalForceInBodyWeights: Double
            /// −1 left foot down, +1 right foot down, 0 flight.
            let contactSide: Int
            /// True when a centred `filterTaps`-wide window around this sample
            /// lies entirely inside this contact.
            ///
            /// The window has to fit or the acceleration is a polynomial fitted
            /// across a discontinuity in the very quantity being differentiated.
            /// Sizing the window from the shortest contact made every window fit
            /// at the price of a 21.5× noise amplification (see
            /// `WindowedDerivativeFilter.minimumSmoothingTaps`); sizing it from
            /// the MEDIAN contact costs 4.69× and leaves a minority of frames —
            /// the first and last `taps/2` of each contact — without a clean
            /// window. Those are marked here and excluded from the load summary
            /// rather than being silently averaged in.
            let derivativeWindowInsideContact: Bool
        }

        let frames: [Frame]
        /// Taps for this clip's derivative filter — chosen so the window fits
        /// INSIDE one contact. See `WindowedDerivativeFilter`.
        let filterTaps: Int
        let sampleInterval: TimeInterval

        /// The plan's entry for a window-centre timestamp, matched to within
        /// half a sampling interval. `nil` means this instant is outside the
        /// strides the model covers, which the caller reports rather than
        /// guessing through.
        func entry(at t: TimeInterval) -> Frame? {
            guard sampleInterval > 0 else { return nil }
            let tolerance = sampleInterval / 2
            var best: Frame?
            var bestGap = Double.infinity
            for f in frames {
                let gap = abs(f.timestamp - t)
                if gap < bestGap { bestGap = gap; best = f }
            }
            return bestGap <= tolerance ? best : nil
        }
    }

    /// The gate every gait frame's force output has to pass, in body weights.
    ///
    /// **Pre-registered before the residual was measured on the real skeleton.**
    /// 0.5 BW is half the subject's own weight: above that, the segment
    /// acceleration the timing model omits is comparable to the gravitational
    /// term the model is built out of, so the modelled force stops being a
    /// common rescaling of the whole contact — and "a common scale cancels in
    /// every ratio" is the entire reason relative output is defensible here.
    static let maxGaitForceResidualInBodyWeights: Double = 0.5

    /// Set to run the offline path as RUNNING rather than as a static hold.
    /// Nil restores the previous behaviour exactly. Read and captured on the
    /// main thread inside `processFrame`, like `staticHoldGating`.
    var gaitPlan: GaitPlan?

    // Normalize model-specific muscle ids to the stable ids used by the
    // overlay and diagnostic bar.
    private static let displayMuscleAliases: [String: String] = [
        "bflh140_r": "bflh_r",
        "bflh140_l": "bflh_l",
        "gaslat140_r": "gaslat_r",
        "gaslat140_l": "gaslat_l",
        "vaslat140_r": "vaslat_r",
        "vaslat140_l": "vaslat_l",
        "multifidus_T9_T7": "ercspn_r",
        "multifidus_T9_T7_L": "ercspn_l",
    ]

    private let bridge = NimbleBridge()
    private let muscleSolver = MuscleSolver()
    private let momentArmComputer = MomentArmComputer()
    private let solverQueue = DispatchQueue(label: "com.biomotion.nimble", qos: .userInteractive)

    // Per-DOF Savitzky–Golay filters for smoothed q / dq / ddq.
    //
    // The window is now CHOSEN PER CLIP rather than fixed at 9 taps, because a
    // 9-tap centred window spans 8·dt = 267 ms at 30 fps — longer than every
    // foot contact this app measures (167-247 ms on the owner's clips). Under
    // that window no stance frame has a neighbourhood free of a touchdown or a
    // toe-off (0 of 114 interior frames on `video_012`), so the second
    // derivative at every stance frame is fitted across a discontinuity. See
    // `WindowedDerivativeFilter` for the measured frequency response.
    //
    // Default stays 9 taps / cubic, which reproduces the previous filter
    // exactly (asserted in `DerivativeWindowTests`), so the live camera and
    // static-hold paths are unchanged.
    private var dofFilters: [WindowedDerivativeFilter] = []
    private var dofFilterTaps = SavitzkyGolayFilter.windowSize

    // Marker-motion history behind `staticHoldGating`. Pushed in lockstep with
    // `dofFilters` (both only on a frame whose IK succeeded), so "the last 9
    // samples" means the same nine frames in both. solverQueue-only state.
    private var holdDetector = StaticHoldDetector()

    // Timestamp of the last successful muscle solve, used to derive dt for
    // musculotendon length finite differencing inside the muscle Hill model.
    private var lastMuscleSolveTimestamp: TimeInterval?
    private var lastDisplayMuscleTimestamp: TimeInterval?
    private let displayMuscleHoldDuration: TimeInterval = 0.35

    // Per-display-muscle 1€ filter. The rebalanced solver (ε_a=0.01,
    // λ=100) now tracks τ tightly, which also tracks ID torque noise —
    // without smoothing, activations jitter visibly frame-to-frame.
    // Aggressive minCutoff=2.0 Hz (vs 1.0 Hz for kinematics) because
    // activation transitions during actual movement are ~100-300 ms and
    // we don't want to soften real force onset.
    private var activationFilters: [String: OneEuroFilter] = [:]

    // IK history for recording
    /// `markerRMSMeters` is the TRUE per-marker RMS in metres, not the solver
    /// loss. See `IKOutput.ikLossSquaredMeters` for why the distinction matters.
    private(set) var ikHistory: [(timestamp: TimeInterval, angles: [String: Double], markerRMSMeters: Double)] = []
    private(set) var idHistory: [(timestamp: TimeInterval, jointTorques: [String: Double])] = []
    private var isRecordingResults = false

    // Generation token bumped on reset. Frames captured before the bump are
    // still in flight on solverQueue; their late publishes are discarded by
    // comparing against the current generation on main.
    private var currentGeneration: UInt64 = 0
    private var genLock = os_unfair_lock_s()

    private func readGeneration() -> UInt64 {
        os_unfair_lock_lock(&genLock)
        defer { os_unfair_lock_unlock(&genLock) }
        return currentGeneration
    }

    private func bumpGeneration() -> UInt64 {
        os_unfair_lock_lock(&genLock)
        defer { os_unfair_lock_unlock(&genLock) }
        currentGeneration &+= 1
        return currentGeneration
    }

    // Backpressure: at most one frame in flight on solverQueue at a time.
    // Accessed only from main.
    private var isFrameInFlight = false
    private(set) var droppedFrameCount: Int = 0

    // NO RUNTIME DOF MASK IS INSTALLED HERE, AND THAT IS A MEASURED DECISION.
    //
    // STATUS.md next-step 8 asked for `shoulder_rot_{r,l}` (axial humeral
    // rotation) to be masked, on the premise that they are "structurally
    // unobservable from one marker per shoulder plus one at the elbow" and that
    // unobservable coordinates get excited by the solver. Measured 2026-08-07,
    // the premise is false and the change is a regression:
    //
    //   * The marker-Jacobian column for `shoulder_rot_r` has norm
    //     0.0343 m/rad at the model's neutral pose — small next to
    //     `shoulder_elv_r` (0.6077) but not null. It moves REJC 16.3 mm and
    //     RWJC 30.2 mm per radian with the elbow STRAIGHT, because the ulna and
    //     hand body origins are offset from the humeral long axis, and 0.266
    //     m/rad at 90° of elbow flexion. It is not one of the 72 identically-
    //     zero columns in `FullBodyDOFFixture.structurallyUnreachableCoordinates`.
    //   * Masking it costs the dancer fixture 0.565 cm of marker RMS
    //     (2.122 -> 2.687) and 0.045 of relative torque residual
    //     (0.3545 -> 0.3991).
    //   * At upright standing it buys nothing (the unmasked solver puts 0.04°
    //     into the coordinate) and it breaks convergence: 0 -> 123 iterations,
    //     converged YES -> NO, per-solve drift 0 -> 9.3e-5 rad.
    //
    // Harness and every number: `ShoulderRotMaskTests` /
    // `ShoulderRotObservabilityTests.mm`. If a future marker set drops the
    // wrist markers, or a future model puts the ulna origin on the humeral
    // axis, re-measure before reviving this — the column norm is the number
    // that decides it.

    /// Load the bundled .osim model.
    func loadBundledModel() {
        // Production full-body model: cyclistFullBodyMuscle.osim shipped
        // as FullBody.osim in the app bundle. 80 bodies, 520 muscles
        // (Millard2012 lower + Thelen2003 upper/trunk/spine).
        //
        // The libnimble_ios.a in this build includes a patched
        // OpenSimParser that no longer crashes on CustomJoints it can't
        // construct — it logs them and substitutes a WeldJoint. So any
        // joint in cyclist that nimble doesn't fully support becomes
        // locked rather than segfaulting the whole skeleton.
        //
        // Fallback: if FullBody.osim is missing from the bundle, load
        // the old lower-extremity-only Rajagopal2016 — useful for
        // developers who want to diff behavior without re-ripping assets.
        let path: String
        if let fb = Bundle.main.path(forResource: "FullBody", ofType: "osim") {
            path = fb
            print("NimbleEngine: loading FullBody.osim (cyclistFullBodyMuscle — 520 muscles, nimble OpenSimParser patched)")
        } else if let raj = Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim") {
            path = raj
            print("NimbleEngine: ⚠ FullBody.osim not found — falling back to Rajagopal2016 (81 lower-extremity muscles only)")
        } else {
            print("NimbleEngine: no .osim model found in bundle")
            return
        }
        solverQueue.async { [weak self] in
            guard let self else { return }
            let success = self.bridge.loadModel(fromPath: path)
            if success {
                self.muscleSolver.loadMuscles(fromOsimPath: path)
                // MomentArmComputer adopts the bridge's skeleton instead of
                // parsing a second copy — so per-segment scaling propagates
                // from bridge.scaleModelWithHeight through to R(q) and L_MT.
                self.momentArmComputer.parseMusclePaths(fromOsimPath: path,
                                                         from: self.bridge)
                // Drop any stale SG state from a previous model — the new
                // model may have a different DOF count / ordering, and even
                // if not, the sample history is no longer valid.
                self.dofFilters.removeAll(keepingCapacity: false)
            }
            DispatchQueue.main.async {
                self.isModelLoaded = success
                if success {
                    let modelMarkers = self.bridge.markerNames as [String]
                    self.totalMassKg = self.bridge.totalMass
                    print("NimbleEngine: Model loaded — \(self.bridge.numDOFs) DOFs, \(modelMarkers.count) markers, \(self.muscleSolver.numMuscles) muscles, mass \(String(format: "%.1f", self.totalMassKg)) kg")
                    print("NimbleEngine: Model marker names (first 10): \(modelMarkers.prefix(10))")
                    print("NimbleEngine: Our mapping names (first 10): \(JointMapping.primary.map(\.opensimName).prefix(10))")
                    // Check how many of our mappings match model markers
                    let matchCount = JointMapping.primary.filter { modelMarkers.contains($0.opensimName) }.count
                    print("NimbleEngine: Marker matches: \(matchCount)/\(JointMapping.primary.count)")
                }
            }
        }
    }

    /// Scale the model for a specific user.
    func scaleModel(height: Double, markerPositions: [Float], markerNames: [String]) {
        guard isModelLoaded else { return }
        let positions = markerPositions.map { NSNumber(value: Double($0)) }
        solverQueue.async { [weak self] in
            self?.bridge.scaleModel(withHeight: height,
                                    markerPositions: positions,
                                    markerNames: markerNames)
        }
    }

    /// Process a body frame: run IK (and optionally ID) on a background thread.
    func processFrame(_ frame: BodyFrame) {
        guard isModelLoaded else { return }

        // Backpressure: drop this frame if the solver is still busy with the
        // previous one. Keeps visualization on the newest pose rather than
        // draining a stale FIFO when OSQP transiently stalls.
        if isFrameInFlight {
            droppedFrameCount &+= 1
            return
        }

        // Build marker arrays from the frame
        var positions: [NSNumber] = []
        var names: [String] = []

        for joint in frame.joints where joint.isTracked {
            // Map ARKit joint to OpenSim marker name
            if let mapping = JointMapping.primary.first(where: { $0.arkitName == joint.id }) {
                names.append(mapping.opensimName)
                positions.append(NSNumber(value: Double(joint.worldPosition.x)))
                positions.append(NSNumber(value: Double(joint.worldPosition.y)))
                positions.append(NSNumber(value: Double(joint.worldPosition.z)))
            }
        }

        guard !names.isEmpty else { return }

        isFrameInFlight = true
        let frameGeneration = readGeneration()
        // Captured on main (see `staticHoldGating`) so the whole solve uses one
        // consistent policy even if the flag flips mid-clip.
        let gateOnHolds = staticHoldGating
        let plan = gaitPlan

        solverQueue.async { [weak self] in
            guard let self else { return }
            defer {
                DispatchQueue.main.async { [weak self] in
                    self?.isFrameInFlight = false
                }
            }

            // --- IK (runs on every frame, on 1€-filtered markers) ---
            let ikStart = CACurrentMediaTime()
            guard let ikResult = self.bridge.solveIK(
                withMarkerPositions: positions,
                markerNames: names
            ) else { return }
            let ikTime = (CACurrentMediaTime() - ikStart) * 1000.0

            // Record marker motion for the hold detector. Deliberately fed the
            // SAME arrays IK consumed — post-`isTracked` filter, post-
            // `JointMapping.primary` lookup — so the stillness test is measured
            // on exactly the observations that constrained the solve, and a
            // marker dropping out cannot be mistaken for a marker moving.
            //
            // This is raw marker motion ON PURPOSE. The obvious alternative,
            // thresholding the Savitzky-Golay `ddq` below, is circular: `ddq` is
            // a twice-differentiated function of this very data (gain ~1/dt²,
            // ≈3600 at 60 fps), and on the offline path the data it is
            // differentiating is missing its global translation component
            // entirely. A small `ddq` there would mean "the unobservable part
            // of the motion stayed unobservable", not "the subject was still".
            self.holdDetector.ingest(flatMarkerPositions: positions,
                                     markerNames: names,
                                     timestamp: frame.timestamp)

            let numDOFs = ikResult.jointAngles.count
            let dofNames = ikResult.dofNames

            // Lazy-init per-DOF SG filters whenever the DOF count OR the window
            // length changes. The window is a per-clip choice on the gait path
            // (see `WindowedDerivativeFilter`), so a plan arriving mid-session
            // has to rebuild them — carrying 9-tap history into a 5-tap filter
            // would date the first outputs at the wrong instant.
            let taps = WindowedDerivativeFilter.admissibleTaps(plan?.filterTaps ?? WindowedDerivativeFilter.maximumTaps)
            if self.dofFilters.count != numDOFs || self.dofFilterTaps != taps {
                self.dofFilters = (0..<numDOFs).map { _ in WindowedDerivativeFilter(taps: taps) }
                self.dofFilterTaps = taps
            }

            // --- Push raw IK angles into SG filters, pull smoothed q/dq/ddq ---
            // Each filter only emits output once its 9-sample window is full,
            // so the first 8 frames after start (or after a DOF-count change)
            // yield "raw" IK only, without ID or muscle results.
            var smoothedQ = [Double](); smoothedQ.reserveCapacity(numDOFs)
            var smoothedDQ = [Double](); smoothedDQ.reserveCapacity(numDOFs)
            var smoothedDDQ = [Double](); smoothedDDQ.reserveCapacity(numDOFs)
            var centerTimestamp = frame.timestamp
            var sgWarmedUp = true

            // EVERY filter must be pushed on EVERY frame. Bailing out of this
            // loop on the first not-yet-warm filter starves all the later ones:
            // filter[k] would only start receiving samples once filters 0..<k
            // were already full, so warm-up would need 9 + 8*(numDOFs-1) frames
            // — about 1350 for this model — instead of 9.
            //
            // The live ARKit path hid that: at 60 fps it grinds through in ~22 s
            // of tracking. The offline path pushes exactly 9 frames per clip and
            // so never warmed up at all, which is why an imported photo reported
            // "Pose only (warming up)" and 0 frames with muscle data forever.
            for i in 0..<numDOFs {
                let q = ikResult.jointAngles[i].doubleValue
                if let out = self.dofFilters[i].push(q, timestamp: frame.timestamp) {
                    smoothedQ.append(out.pos)
                    smoothedDQ.append(out.vel)
                    smoothedDDQ.append(out.acc)
                    centerTimestamp = out.center
                } else {
                    sgWarmedUp = false
                }
            }
            // A partially-filled window would leave the smoothed arrays shorter
            // than numDOFs; every consumer below is behind the `sgWarmedUp`
            // guard, so they only ever see complete ones.
            if smoothedQ.count != numDOFs { sgWarmedUp = false }

            // Build the "raw" IK output (used for live UI skeleton overlay).
            var liveAngles: [String: Double] = [:]
            for i in 0..<min(numDOFs, dofNames.count) {
                liveAngles[dofNames[i]] = ikResult.jointAngles[i].doubleValue
            }
            let liveIkOutput = IKOutput(
                jointAngles: liveAngles,
                markerRMSMeters: ikResult.markerRMSMeters,
                ikLossSquaredMeters: ikResult.error,
                timestamp: frame.timestamp
            )

            guard sgWarmedUp else {
                // Still warming up: publish live IK only, no ID/muscle yet.
                // No motion verdict either — there is no window centre to date
                // one at, so `lastSolve` stays nil rather than carrying a guess.
                self.publishResults(ik: liveIkOutput, id: nil, muscle: nil,
                                    motion: nil, isStaticHoldEstimate: false,
                                    ikTime: ikTime, idTime: 0, muscleTime: 0,
                                    ikResidual: ikResult.markerRMSMeters, maxTorqueNm: 0,
                                    groundY: self.bridge.groundHeightY,
                                    generation: frameGeneration)
                return
            }

            // Smoothed IK output, dated at the center of the SG window.
            // This is what ID and muscle solves operate on — it matches dq/ddq
            // temporally, which is essential for correct inverse dynamics.
            var smoothedAngles: [String: Double] = [:]
            for i in 0..<min(numDOFs, dofNames.count) {
                smoothedAngles[dofNames[i]] = smoothedQ[i]
            }
            let smoothedIkOutput = IKOutput(
                jointAngles: smoothedAngles,
                markerRMSMeters: ikResult.markerRMSMeters,
                ikLossSquaredMeters: ikResult.error,
                timestamp: centerTimestamp
            )

            // --- Was the subject still at the window centre? ---
            // Classified at `centerTimestamp`, not at the newest push: that is
            // the instant ID and the muscle solve are dated at, and the centred
            // window means we already hold 4 samples of "future" around it.
            let motion = self.holdDetector.classify(centeredAt: centerTimestamp)

            // An IK solve that exited on the iteration cap has NOT reached a
            // fixed point, and the next solve on identical markers will still be
            // moving. Adversarial testing found a legitimate in-limits pose where
            // that residual motion is 2.0e-2 rad — twenty times the drift bound
            // the old known-red test asserted. The Savitzky-Golay stage
            // differentiates twice (gain ≈ 1/dt² ≈ 3600 at 60 fps), so it becomes
            // ~73 rad/s² of acceleration the subject never had, and every torque
            // downstream inherits it.
            //
            // `converged` was already reported and nothing read it. Withhold the
            // dynamics exactly as a moving frame does: the pose is still good
            // enough to draw and to export, the derivatives are not.
            guard ikResult.converged else {
                // Reported as what it IS. Publishing the stillness verdict here
                // labelled a solver failure as subject motion, and on the gait
                // path it put a frame outside the gait vocabulary altogether.
                self.publishPoseOnly(ik: smoothedIkOutput,
                                     motion: motion.replacingVerdict(.poseDidNotConverge),
                                     smoothedAngles: smoothedAngles,
                                     markerRMS: ikResult.markerRMSMeters,
                                     ikTime: ikTime, generation: frameGeneration)
                return
            }

            // --- Which policy decides this frame? ------------------------------
            //
            // Two mutually exclusive regimes, and the gait plan wins when it is
            // present. On a running clip the static-hold question is not just
            // answered "no", it is the WRONG QUESTION: nothing about a runner is
            // still, and the reason the dynamics are computable anyway is that
            // the gait cycle supplies the root acceleration the stillness test
            // was standing in for.
            var solveAsStatics = false
            var idDQ: [Double]
            var idDDQ: [Double]
            var publishedMotion = motion
            var plannedForce: Double?          // body weights, from timing alone
            var plannedSide = 0
            var rootVerticalAccel = 0.0
            var plannedWindowIsClean = false

            if let plan {
                guard let entry = plan.entry(at: centerTimestamp) else {
                    self.publishPoseOnly(ik: smoothedIkOutput,
                                         motion: motion.replacingVerdict(.gaitOutsideAnalysis),
                                         smoothedAngles: smoothedAngles,
                                         markerRMS: ikResult.markerRMSMeters,
                                         ikTime: ikTime, generation: frameGeneration)
                    return
                }
                guard entry.contactSide != 0 else {
                    // Flight. No foot is on the ground, so there is no stance
                    // load to report — and reporting one would be inventing a
                    // contact. The pose is still published.
                    self.publishPoseOnly(ik: smoothedIkOutput,
                                         motion: motion.replacingVerdict(.gaitFlight),
                                         smoothedAngles: smoothedAngles,
                                         markerRMS: ikResult.markerRMSMeters,
                                         ikTime: ikTime, generation: frameGeneration)
                    return
                }
                guard let rootTY = dofNames.firstIndex(of: Self.rootVerticalDOFName) else {
                    // Defensive: both shipped models carry `pelvis_ty`
                    // (asserted in `GaitDynamicsTests`). Without it there is no
                    // channel to put the root acceleration into, so nothing is
                    // claimed.
                    self.publishPoseOnly(ik: smoothedIkOutput,
                                         motion: motion.replacingVerdict(.gaitRefused),
                                         smoothedAngles: smoothedAngles,
                                         markerRMS: ikResult.markerRMSMeters,
                                         ikTime: ikTime, generation: frameGeneration)
                    return
                }

                // THE SUBSTITUTION. Whole-body Newton in the vertical:
                // `F − m·g = m·a`, so `a = g·(F/(m·g) − 1)`. In flight
                // `F/(m·g) = 0` and this is exactly `−g`, free fall; at the
                // half-sine peak of 2.87 BW it is +1.87 g upward. No mass, no
                // camera, no depth channel enters it — which is the whole
                // reason a hand-held tracking shot is analysable at all.
                rootVerticalAccel = StaticHoldDetector.gravityMetersPerSecondSquared
                    * (entry.verticalForceInBodyWeights - 1.0)

                idDQ = smoothedDQ
                idDDQ = smoothedDDQ
                idDDQ[rootTY] = rootVerticalAccel
                // ⚠️ `pelvis_tx` / `pelvis_tz` are LEFT ALONE, at whatever the
                // filter produced from the data — which on a pelvis-pinned
                // stream is ~0. The gait model supplies a vertical force only;
                // it says nothing about braking and propulsion along the
                // running direction, and writing a zero there deliberately
                // would be asserting there is none. That term is reported as
                // UNMODELLED (`GaitFrameOutcome.horizontalRootAccelerationModelled`)
                // and surfaced to the user, rather than being absorbed
                // silently — an earlier implementation forced it to zero and
                // injected an undisclosed 0.2-0.35 BW error into every joint
                // moment.
                plannedForce = entry.verticalForceInBodyWeights
                plannedSide = entry.contactSide
                plannedWindowIsClean = entry.derivativeWindowInsideContact
                publishedMotion = motion.replacingVerdict(.gaitStance)
            } else {
                guard !gateOnHolds || motion.isHold else {
                    // Measurably moving. Publish the pose and the verdict; publish
                    // NO muscle magnitudes. The alternative — reporting them anyway
                    // — would be reporting `M·q̈` computed from a translation this
                    // input cannot see.
                    //
                    // The POSE is still recorded. Only the dynamics are withheld,
                    // so a .mot export of a moving clip stays complete; it is the
                    // matching .sto that legitimately has a gap.
                    self.publishPoseOnly(ik: smoothedIkOutput, motion: motion,
                                         smoothedAngles: smoothedAngles,
                                         markerRMS: ikResult.markerRMSMeters,
                                         ikTime: ikTime, generation: frameGeneration)
                    return
                }

                // On a detected hold, solve statics: q̇ = q̈ = 0 exactly. That is
                // the whole point of the detector — it deletes the 1/dt²
                // Savitzky-Golay amplification chain from τ = M·q̈ + C − JᵀF_ext
                // instead of feeding it derivatives of a motion we half-observed.
                // It also zeroes the Hill force-velocity term downstream
                // (`dL_MT/dt = −Rᵀ·q̇`), which is what "isometric" means.
                solveAsStatics = gateOnHolds && motion.isHold
                idDQ = solveAsStatics ? [Double](repeating: 0, count: numDOFs) : smoothedDQ
                idDDQ = solveAsStatics ? [Double](repeating: 0, count: numDOFs) : smoothedDDQ
            }

            // --- ID on SG-smoothed q, and dq/ddq per the policy above ---
            let smoothedQNS: [NSNumber] = smoothedQ.map { NSNumber(value: $0) }
            let smoothedDQNS: [NSNumber] = idDQ.map { NSNumber(value: $0) }
            let smoothedDDQNS: [NSNumber] = idDDQ.map { NSNumber(value: $0) }

            var idOutput: IDOutput?
            var idTime = 0.0
            var maxTorqueNm = 0.0

            let idStart = CACurrentMediaTime()
            // Use the GRF-aware ID solver. It runs Nimble's near-CoP
            // multi-contact inverse dynamics which auto-detects foot contact
            // and decomposes the system wrench into GRFs + joint torques.
            if let idResult = self.bridge.solveIDGRF(
                withJointAngles: smoothedQNS,
                jointVelocities: smoothedDQNS,
                jointAccelerations: smoothedDDQNS
            ) {
                idTime = (CACurrentMediaTime() - idStart) * 1000.0

                var torques: [String: Double] = [:]
                for i in 0..<min(idResult.jointTorques.count, dofNames.count) {
                    let t = idResult.jointTorques[i].doubleValue
                    torques[dofNames[i]] = t
                    if abs(t) > maxTorqueNm { maxTorqueNm = abs(t) }
                }

                var out = IDOutput(jointTorques: torques, timestamp: centerTimestamp)
                func toSimd(_ arr: [NSNumber]) -> SIMD3<Double> {
                    guard arr.count >= 3 else { return .zero }
                    return SIMD3<Double>(arr[0].doubleValue, arr[1].doubleValue, arr[2].doubleValue)
                }
                out.leftFootForce  = toSimd(idResult.leftFootForce)
                out.rightFootForce = toSimd(idResult.rightFootForce)
                out.leftFootCoP    = toSimd(idResult.leftFootCoP)
                out.rightFootCoP   = toSimd(idResult.rightFootCoP)
                out.leftFootInContact  = idResult.leftFootInContact
                out.rightFootInContact = idResult.rightFootInContact
                out.rootResidualNorm   = idResult.rootResidualNorm
                idOutput = out
            }

            // --- Muscle static optimization (on same SG-centered state) ---
            // Production path: real moment arms from FK + soft-equality QP +
            // real Hill-model force-length/velocity. Requires that the
            // skeleton is at the smoothed pose, which solveIDGRF() already
            // sets when it runs — MomentArmComputer picks up from there.
            var muscleOutput: MuscleOutput?
            var muscleTime = 0.0

            if let id = idOutput {
                // IMPORTANT: jointTorques arrives as a dictionary, which has
                // no defined order. We must force a consistent ordering here
                // so that `dofNames[i]` refers to the same DOF as `torques[i]`
                // AND as the j-th column of the moment-arm matrix. Use the
                // DOF names from IK (they are in skeleton order).
                let torqueKeys: [String] = ikResult.dofNames as [String]
                let torqueVals = torqueKeys.map { NSNumber(value: id.jointTorques[$0] ?? 0) }

                // Compute the moment-arm matrix R(q) at the smoothed pose.
                // MomentArmComputer runs its own FK so it expects the pose
                // to be driven through setPositions. Pass the SG-smoothed q.
                let momentArms = self.momentArmComputer.computeMomentArms(
                    withJointAngles: smoothedQNS,
                    dofNames: torqueKeys
                ) ?? []
                let muscleLengthsNS = self.momentArmComputer.currentMuscleLengths
                let muscleNamesNS = self.momentArmComputer.muscleNames
                let maxForcesNS = self.momentArmComputer.maxIsometricForces
                let optimalFibersNS = self.momentArmComputer.optimalFiberLengths
                let tendonSlacksNS = self.momentArmComputer.tendonSlackLengths
                let pennationsNS = self.momentArmComputer.pennationAngles

                if !momentArms.isEmpty && muscleNamesNS.count > 0 {
                    // dt for fiber-velocity finite differencing. We use the
                    // SG-window centered timestamp vs. the previous frame's.
                    let nowSec = centerTimestamp
                    let dt = max(nowSec - (self.lastMuscleSolveTimestamp ?? nowSec - 0.0167),
                                 1e-3)
                    self.lastMuscleSolveTimestamp = nowSec

                    if let result = self.muscleSolver.solveReal(
                        withJointTorques: torqueVals,
                        momentArms: momentArms,
                        muscleNames: muscleNamesNS,
                        muscleLengths: muscleLengthsNS,
                        maxForces: maxForcesNS,
                        optimalFiberLengths: optimalFibersNS,
                        tendonSlackLengths: tendonSlacksNS,
                        pennationAngles: pennationsNS,
                        jointVelocities: smoothedDQNS,
                        dofNames: torqueKeys,
                        dt: dt,
                        // 100× the activation L2 regularizer (ε=0.01 inside
                        // solver). τ-match dominates; ‖a‖² only breaks ties
                        // among the redundant ~520-muscle set.
                        softPenalty: 100.0
                    ) {
                        muscleTime = result.solveTimeMs

                        // Harvest path endpoints from the skeleton FK at the
                        // SAME pose the solver consumed (computeMomentArms
                        // left the skeleton at smoothedQ before restoring).
                        // Doing this in the same solverQueue block avoids
                        // races against next-frame setPositions.
                        let endpoints = self.momentArmComputer.muscleEndpointsWorld
                        let pathCount = muscleNamesNS.count
                        var paths: [String: MusclePath] = [:]
                        paths.reserveCapacity(pathCount)
                        if endpoints.count == pathCount * 6 {
                            for i in 0..<pathCount {
                                let base = i * 6
                                let s = SIMD3<Float>(
                                    Float(truncating: endpoints[base]),
                                    Float(truncating: endpoints[base + 1]),
                                    Float(truncating: endpoints[base + 2])
                                )
                                let e = SIMD3<Float>(
                                    Float(truncating: endpoints[base + 3]),
                                    Float(truncating: endpoints[base + 4]),
                                    Float(truncating: endpoints[base + 5])
                                )
                                paths[muscleNamesNS[i]] = MusclePath(start: s, end: e)
                            }
                        }

                        var activations: [String: Double] = [:]
                        var forces: [String: Double] = [:]
                        var maxForceMap: [String: Double] = [:]
                        activations.reserveCapacity(result.muscleNames.count)
                        forces.reserveCapacity(result.muscleNames.count)
                        maxForceMap.reserveCapacity(result.muscleNames.count)
                        for i in 0..<result.muscleNames.count {
                            let name = result.muscleNames[i]
                            activations[name] = result.activations[i].doubleValue
                            forces[name] = result.forces[i].doubleValue
                            if i < maxForcesNS.count {
                                maxForceMap[name] = maxForcesNS[i].doubleValue
                            }
                        }
                        var out = MuscleOutput(
                            activations: activations,
                            forces: forces,
                            converged: result.converged,
                            timestamp: centerTimestamp
                        )
                        out.rawActivations = activations
                        out.paths = paths
                        out.maxForces = maxForceMap
                        muscleOutput = out
                    }
                }
            }

            // --- The falsifier ------------------------------------------------
            //
            // Only on the gait path, and only where a contact was claimed. The
            // gait model said the ground pushed with `plannedForce` body
            // weights; inverse dynamics, from the skeleton's own mass
            // distribution and the measured joint accelerations, says it pushed
            // with `solved`. Their difference is `‖a_artic‖/g` — see
            // `GaitFrameOutcome`.
            var gaitOutcome: GaitFrameOutcome?
            if let planned = plannedForce {
                let weightN = max(self.bridge.totalMass, 1e-6) * StaticHoldDetector.gravityMetersPerSecondSquared
                let solvedN = (idOutput?.leftFootForce.y ?? 0) + (idOutput?.rightFootForce.y ?? 0)
                let solved = solvedN / weightN
                gaitOutcome = GaitFrameOutcome(
                    modelledVerticalForceInBodyWeights: planned,
                    solvedVerticalForceInBodyWeights: solved,
                    residualInBodyWeights: abs(solved - planned),
                    contactSide: plannedSide,
                    solverSawLeftContact: idOutput?.leftFootInContact ?? false,
                    solverSawRightContact: idOutput?.rightFootInContact ?? false,
                    rootVerticalAccelerationMetersPerSecondSquared: rootVerticalAccel,
                    horizontalRootAccelerationModelled: false,
                    derivativeWindowInsideContact: plannedWindowIsClean)
            }

            // Record if enabled (history always uses the SG-centered timestamp
            // so downstream .mot/.sto exports are temporally consistent).
            if self.isRecordingResults {
                self.ikHistory.append((centerTimestamp, smoothedAngles, ikResult.markerRMSMeters))
                if let id = idOutput {
                    self.idHistory.append((centerTimestamp, id.jointTorques))
                }
            }

            self.publishResults(ik: smoothedIkOutput, id: idOutput, muscle: muscleOutput,
                                motion: publishedMotion, isStaticHoldEstimate: solveAsStatics,
                                ikTime: ikTime, idTime: idTime, muscleTime: muscleTime,
                                ikResidual: ikResult.markerRMSMeters, maxTorqueNm: maxTorqueNm,
                                groundY: self.bridge.groundHeightY,
                                generation: frameGeneration,
                                gait: gaitOutcome)
        }
    }

    /// The root's world-vertical translation coordinate. `FullBody.osim` and
    /// `Rajagopal2016.osim` both name it this, and both define it as a
    /// translation along the ground frame's +Y (`<axis>0 1 0</axis>` on the
    /// pelvis `CustomJoint`), which is why writing an acceleration into it is a
    /// world-frame vertical acceleration and not a body-local one.
    static let rootVerticalDOFName = "pelvis_ty"

    /// Publishes pose and a verdict with NO ID and NO muscle magnitudes.
    /// solverQueue-only (it touches `ikHistory`).
    private func publishPoseOnly(ik: IKOutput,
                                 motion: MotionClassification,
                                 smoothedAngles: [String: Double],
                                 markerRMS: Double,
                                 ikTime: Double,
                                 generation: UInt64) {
        if isRecordingResults {
            ikHistory.append((ik.timestamp, smoothedAngles, markerRMS))
        }
        publishResults(ik: ik, id: nil, muscle: nil,
                       motion: motion, isStaticHoldEstimate: false,
                       ikTime: ikTime, idTime: 0, muscleTime: 0,
                       ikResidual: markerRMS, maxTorqueNm: 0,
                       groundY: bridge.groundHeightY,
                       generation: generation)
    }

    private func publishResults(ik: IKOutput, id: IDOutput?, muscle: MuscleOutput?,
                                motion: MotionClassification?,
                                isStaticHoldEstimate: Bool,
                                ikTime: Double, idTime: Double, muscleTime: Double,
                                ikResidual: Double, maxTorqueNm: Double,
                                groundY: Double,
                                generation: UInt64,
                                gait: GaitFrameOutcome? = nil) {
        let mass = max(totalMassKg, 1e-6)
        let torquePerKg = maxTorqueNm / mass
        // Vertical load on each foot as a fraction of body weight. Useful for
        // validating stance vs. swing phase and impact peaks during gait.
        let weightN = mass * 9.81
        let leftLoad  = (id?.leftFootForce.y  ?? 0) / weightN
        let rightLoad = (id?.rightFootForce.y ?? 0) / weightN
        let rootResPerKg = (id?.rootResidualNorm ?? 0) / mass
        let displayMuscle = muscle.map(normalizeForDisplay)
        let solve = motion.map {
            SolveRecord(centerTimestamp: $0.timestamp, motion: $0, ik: ik, id: id,
                        muscle: muscle, isStaticHoldEstimate: isStaticHoldEstimate,
                        gait: gait)
        }
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            // Drop late publishes from a pre-reset generation so they don't
            // overwrite the cleared @Published state.
            guard self.readGeneration() == generation else { return }
            self.lastIKResult = ik
            self.lastIDResult = id
            self.lastMuscleResult = muscle
            // Only overwritten by a warm solve. A warm-up publish leaves the
            // previous record in place rather than blanking it, matching how
            // `lastMuscleResult` behaves.
            if let solve { self.lastSolve = solve }
            if let displayMuscle {
                self.displayMuscleResult = displayMuscle
                self.lastDisplayMuscleTimestamp = displayMuscle.timestamp
            } else if let lastTimestamp = self.lastDisplayMuscleTimestamp,
                      (ik.timestamp - lastTimestamp) > self.displayMuscleHoldDuration {
                self.displayMuscleResult = nil
                self.lastDisplayMuscleTimestamp = nil
            }
            self.ikSolveTimeMs = ikTime
            self.idSolveTimeMs = idTime
            self.muscleSolveTimeMs = muscleTime
            self.ikMarkerResidualMeters = ikResidual
            self.maxTorquePerKg = torquePerKg
            self.leftFootLoadFraction = leftLoad
            self.rightFootLoadFraction = rightLoad
            self.rootResidualPerKg = rootResPerKg
            self.groundHeightY = groundY
        }
    }

    func resetRealtimeState() {
        // Bump first so any in-flight frame's publish will be discarded.
        _ = bumpGeneration()

        solverQueue.async { [weak self] in
            guard let self else { return }
            self.dofFilters.removeAll(keepingCapacity: false)
            self.dofFilterTaps = WindowedDerivativeFilter.maximumTaps
            // Must be cleared with the SG filters, not separately: the two are
            // pushed in lockstep and the classifier reads "the last 9 samples"
            // as "the samples that produced this ddq".
            self.holdDetector.reset()
            self.lastMuscleSolveTimestamp = nil
        }

        activationFilters.removeAll(keepingCapacity: false)
        lastDisplayMuscleTimestamp = nil
        lastIKResult = nil
        lastIDResult = nil
        lastMuscleResult = nil
        lastSolve = nil
        displayMuscleResult = nil
        ikSolveTimeMs = 0
        idSolveTimeMs = 0
        muscleSolveTimeMs = 0
        ikMarkerResidualMeters = 0
        maxTorquePerKg = 0
        leftFootLoadFraction = 0
        rightFootLoadFraction = 0
        rootResidualPerKg = 0
    }

    /// Full session reset for a clip boundary. In addition to everything
    /// `resetRealtimeState()` clears, this drops NimbleBridge's session-only
    /// state: the IK warm-start pose and the rolling ground-height window
    /// (`NimbleBridge.h` `resetSessionState`). Without it a new clip warm-starts
    /// from the previous clip's unrelated pose and GRF contact detection reads a
    /// stale floor.
    ///
    /// It also drops `MuscleSolver`'s warm start. That is the same class of bug
    /// one stage further down and it was live until 2026-08-08: the QP's landing
    /// point in its own null space depends on where it started, so without this
    /// the same clip imported twice in one session published different
    /// activations — measured at 0.836 on the worst muscle and 1432 N of total
    /// muscle force between two byte-identical runs. Everything this engine
    /// ships is a COMPARISON, so an answer that depends on what was analysed
    /// before it is not an answer. After this call two identical inputs produce
    /// identical output (`testTwoIdenticalRunsProduceIdenticalActivations`).
    func resetSessionState() {
        resetRealtimeState()
        solverQueue.async { [weak self] in
            self?.bridge.resetSessionState()
            self?.muscleSolver.resetSessionState()
        }
    }

    private func normalizeForDisplay(_ muscle: MuscleOutput) -> MuscleOutput {
        let mergedActivations = normalizeActivations(muscle.activations)
        let smoothed = smoothActivations(mergedActivations, timestamp: muscle.timestamp)
        var out = MuscleOutput(
            // Activations are 0–1 effort ratios — merging heads takes max,
            // then 1€-filter removes per-frame QP jitter.
            activations: smoothed,
            // Forces in N from separate physical heads are additive.
            forces: normalizeForces(muscle.forces),
            converged: muscle.converged,
            timestamp: muscle.timestamp
        )
        // Pass raw paths/maxForces/rawActivations through (keyed by the
        // SOLVER's muscle names, not the display aliases) — the overlay
        // uses these to render muscles that the hardcoded def set
        // doesn't cover. `rawActivations` is intentionally NOT smoothed
        // because we don't keep a per-raw-name filter — path rendering
        // uses activation only for a visibility threshold and color tier,
        // which is not sensitive to one-frame jitter.
        out.paths = muscle.paths
        out.maxForces = muscle.maxForces
        out.rawActivations = muscle.rawActivations
        return out
    }

    private func smoothActivations(_ source: [String: Double],
                                   timestamp: TimeInterval) -> [String: Double] {
        var out: [String: Double] = [:]
        out.reserveCapacity(source.count)
        for (name, a) in source {
            let filter = activationFilters[name] ?? {
                let f = OneEuroFilter(minCutoff: 2.0, beta: 0.01, dCutoff: 1.0)
                activationFilters[name] = f
                return f
            }()
            out[name] = filter.filter(a, timestamp: timestamp)
        }
        return out
    }

    private func normalizeActivations(_ source: [String: Double]) -> [String: Double] {
        var normalized: [String: Double] = [:]
        normalized.reserveCapacity(source.count)
        for (name, value) in source {
            let displayName = Self.displayMuscleAliases[name] ?? name
            if let existing = normalized[displayName] {
                normalized[displayName] = max(existing, value)
            } else {
                normalized[displayName] = value
            }
        }
        return normalized
    }

    private func normalizeForces(_ source: [String: Double]) -> [String: Double] {
        var normalized: [String: Double] = [:]
        normalized.reserveCapacity(source.count)
        for (name, value) in source {
            let displayName = Self.displayMuscleAliases[name] ?? name
            normalized[displayName, default: 0] += value
        }
        return normalized
    }

    // MARK: - Recording

    func startRecordingResults() {
        ikHistory.removeAll()
        idHistory.removeAll()
        isRecordingResults = true
    }

    func stopRecordingResults() {
        isRecordingResults = false
    }

    /// Export IK results as .mot file.
    func exportMOT(filename: String = "BioMotion_ik") throws -> URL {
        guard !ikHistory.isEmpty else { throw ExportError.noData }

        let startTime = ikHistory.first!.timestamp
        let allDOFs = Array(ikHistory.first!.angles.keys).sorted()

        var lines: [String] = []
        lines.append(filename)
        lines.append("version=1")
        lines.append("nRows=\(ikHistory.count)")
        lines.append("nColumns=\(allDOFs.count + 1)")
        lines.append("inDegrees=yes")
        lines.append("endheader")

        // Column headers
        lines.append("time\t" + allDOFs.joined(separator: "\t"))

        // Data rows
        for entry in ikHistory {
            let time = entry.timestamp - startTime
            var row = String(format: "%.6f", time)
            for dof in allDOFs {
                let angleRad = entry.angles[dof] ?? 0.0
                let angleDeg = angleRad * 180.0 / .pi
                row += String(format: "\t%.4f", angleDeg)
            }
            lines.append(row)
        }

        let content = lines.joined(separator: "\n")
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("\(filename).mot")
        try content.write(to: url, atomically: true, encoding: .utf8)
        return url
    }

    /// Export ID results as .sto file.
    func exportSTO(filename: String = "BioMotion_id") throws -> URL {
        guard !idHistory.isEmpty else { throw ExportError.noData }

        let startTime = idHistory.first!.timestamp
        let allDOFs = Array(idHistory.first!.jointTorques.keys).sorted()

        var lines: [String] = []
        lines.append(filename)
        lines.append("version=1")
        lines.append("nRows=\(idHistory.count)")
        lines.append("nColumns=\(allDOFs.count + 1)")
        lines.append("inDegrees=no")
        lines.append("endheader")

        lines.append("time\t" + allDOFs.joined(separator: "\t"))

        for entry in idHistory {
            let time = entry.timestamp - startTime
            var row = String(format: "%.6f", time)
            for dof in allDOFs {
                row += String(format: "\t%.4f", entry.jointTorques[dof] ?? 0.0)
            }
            lines.append(row)
        }

        let content = lines.joined(separator: "\n")
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("\(filename).sto")
        try content.write(to: url, atomically: true, encoding: .utf8)
        return url
    }

    enum ExportError: Error {
        case noData
    }
}

// MARK: - Derivative window

/// Savitzky–Golay smoothing and differentiation over a window whose LENGTH is a
/// parameter, because the right length is a property of the motion being
/// measured and not of the code.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHY THIS EXISTS: 9 TAPS IS LONGER THAN A FOOT CONTACT
/// ─────────────────────────────────────────────────────────────────────────
/// A centred `T`-tap window spans `(T−1)·dt`. At 30 fps the shipped 9-tap
/// window spans **266.7 ms**, against foot contacts measured at **167-247 ms**
/// on the owner's own clips. So no stance frame has a window free of a
/// touchdown or a toe-off — measured, 0 of 114 interior frames on `video_012`
/// — and every stance acceleration is a cubic fitted across a discontinuity in
/// the very quantity being differentiated.
///
/// That is a correctness problem for exactly the thing a correction product
/// shows: the SHAPE of the activation curve through stance. A window that
/// straddles the edges flattens the onset and the release, so the curve looks
/// smoother and later than it is.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHAT THE SHORTER WINDOWS COST AND BUY — measured, not asserted
/// ─────────────────────────────────────────────────────────────────────────
/// Second-derivative gain `H(ω)/(−ω²)` at `dt = 1/30 s`. 3.30 Hz is the step
/// fundamental of a 606 ms stride; 1.0 would be an ideal differentiator.
///
///     taps  order  span     @1 Hz   @3.30 Hz  @5 Hz   @7 Hz    @10 Hz
///       9     3    266.7ms  0.9413   0.4894   0.1401  −0.0392   0.0044
///       7     2    200.0ms  0.9655   0.6699   0.3691   0.0855  −0.0489
///       5     2    133.3ms  0.9839   0.8348   0.6514   0.4069   0.0977
///       3     2     66.7ms  0.9963   0.9608   0.9119   0.8332   0.6839
///
/// **The 9-tap window halves the very term the gait route depends on** (0.4894
/// at the step fundamental) and INVERTS ITS SIGN above 7 Hz. The 5-tap window
/// passes 83 % at the fundamental and does not invert anywhere in the band. It
/// pays for that with noise: a shorter window averages fewer samples, so the
/// variance of `ddq` rises. That trade is the right way round here, because on
/// this path the alternative to noise is BIAS — a systematically halved and
/// smeared stance transient, which no amount of averaging elsewhere recovers.
///
/// ─────────────────────────────────────────────────────────────────────────
/// EQUIVALENCE WITH WHAT SHIPPED
/// ─────────────────────────────────────────────────────────────────────────
/// At `taps = 9, order = 3` the coefficients this type derives are the ones
/// `SavitzkyGolayFilter` hard-codes, to within 1e-15. `DerivativeWindowTests`
/// asserts that on both the coefficients and on filtered output, so the live
/// camera path and the static-hold path are provably unchanged.
final class WindowedDerivativeFilter {

    /// Shortest usable centred window. Two taps cannot carry a second
    /// derivative at all; three is the plain second difference.
    static let minimumTaps = 3
    /// Shortest window that actually SMOOTHS the position channel, and the floor
    /// the gait path is allowed to size down to.
    ///
    /// At 3 taps the position coefficients are `[0, 1, 0]` — the "smoothed" pose
    /// the moment-arm and muscle-length stage consumes is then the raw IK
    /// output, unfiltered — and the second-derivative coefficients are
    /// `[1, −2, 1]`, whose white-noise gain is **21.49× the 9-tap window's**.
    /// That noise is independent per DOF, so it does NOT cancel out of a
    /// muscle-to-muscle ratio the way a peak-force error does, and the ratio is
    /// the product. 5 taps costs 4.69× instead, and smooths.
    static let minimumSmoothingTaps = 5
    /// Longest window this app uses. Beyond 9 the span grows past a stride.
    static let maximumTaps = SavitzkyGolayFilter.windowSize

    /// White-noise gain of the second-derivative coefficients, `‖c_acc‖`. For
    /// input samples with standard deviation σ the returned acceleration has
    /// standard deviation `σ·gain/dt²`.
    ///
    /// Exact values (asserted in `DerivativeWindowTests`): 9 taps 0.113961,
    /// 7 taps 0.218218, 5 taps 0.534522, 3 taps 2.449490.
    static func accelerationNoiseGain(taps: Int) -> Double {
        let t = admissibleTaps(taps)
        let c = coefficients(taps: t, order: order(forTaps: t), derivative: 2)
        return c.reduce(0) { $0 + $1 * $1 }.squareRoot()
    }

    /// `accelerationNoiseGain(taps:)` relative to the 9-tap window the live
    /// camera path uses — the number the gait screen shows, because it is the
    /// price paid for a window that fits inside a contact.
    static func accelerationNoiseAmplification(taps: Int) -> Double {
        accelerationNoiseGain(taps: taps) / accelerationNoiseGain(taps: maximumTaps)
    }

    /// Rounds any requested length into an odd, in-range tap count. A CENTRED
    /// window must be odd — an even one has no middle sample to date the
    /// answer at.
    static func admissibleTaps(_ requested: Int) -> Int {
        let clamped = min(max(requested, minimumTaps), maximumTaps)
        return clamped % 2 == 1 ? clamped : clamped - 1
    }

    /// Polynomial order for a given window. 9 taps keep the cubic that shipped;
    /// shorter windows drop to quadratic, because a cubic through 5 points is
    /// nearly an interpolant and its second derivative is then dominated by
    /// noise.
    static func order(forTaps taps: Int) -> Int { taps >= 9 ? 3 : 2 }

    let taps: Int
    let order: Int
    let posCoefficients: [Double]
    let velCoefficients: [Double]
    let accCoefficients: [Double]

    private var samples: [Double] = []
    private var timestamps: [Double] = []

    init(taps: Int = maximumTaps) {
        let t = Self.admissibleTaps(taps)
        self.taps = t
        self.order = Self.order(forTaps: t)
        posCoefficients = Self.coefficients(taps: t, order: order, derivative: 0)
        velCoefficients = Self.coefficients(taps: t, order: order, derivative: 1)
        accCoefficients = Self.coefficients(taps: t, order: order, derivative: 2)
    }

    var halfWindow: Int { taps / 2 }
    /// `(taps − 1)·dt` for a given sampling interval — the quantity that has to
    /// fit inside one contact.
    func spanSeconds(sampleInterval: Double) -> Double { Double(taps - 1) * sampleInterval }

    /// Push a sample; returns nil until the window is full.
    func push(_ x: Double, timestamp: Double) -> (pos: Double, vel: Double, acc: Double, center: Double)? {
        samples.append(x)
        timestamps.append(timestamp)
        if samples.count > taps {
            samples.removeFirst()
            timestamps.removeFirst()
        }
        guard samples.count == taps else { return nil }
        let dt = (timestamps[taps - 1] - timestamps[0]) / Double(taps - 1)
        guard dt > 1e-6 else { return nil }

        var pos = 0.0, vel = 0.0, acc = 0.0
        for i in 0..<taps {
            let s = samples[i]
            pos += posCoefficients[i] * s
            vel += velCoefficients[i] * s
            acc += accCoefficients[i] * s
        }
        return (pos, vel / dt, acc / (dt * dt), timestamps[halfWindow])
    }

    func reset() {
        samples.removeAll(keepingCapacity: true)
        timestamps.removeAll(keepingCapacity: true)
    }

    var isWarmedUp: Bool { samples.count == taps }

    // MARK: Coefficients

    /// Least-squares polynomial fit of `order` to `taps` uniformly spaced
    /// samples, evaluated for the `derivative`-th derivative at the centre.
    ///
    /// `c = d! · [ (AᵀA)⁻¹Aᵀ ]_d` with `A[i][p] = k_i^p`, `k = −h…h`. Derived
    /// rather than tabulated so a new window length cannot ship with a
    /// transcription error; the derivation is checked against the hard-coded
    /// 9-tap table in `DerivativeWindowTests`.
    static func coefficients(taps: Int, order: Int, derivative: Int) -> [Double] {
        precondition(taps > order, "a polynomial of order \(order) needs more than \(order) samples")
        let h = taps / 2
        let ks = (0..<taps).map { Double($0 - h) }
        let cols = order + 1

        // A (taps × cols)
        var a = [[Double]](repeating: [Double](repeating: 0, count: cols), count: taps)
        for i in 0..<taps {
            var power = 1.0
            for p in 0..<cols {
                a[i][p] = power
                power *= ks[i]
            }
        }
        // N = AᵀA (cols × cols)
        var n = [[Double]](repeating: [Double](repeating: 0, count: cols), count: cols)
        for p in 0..<cols {
            for q in 0..<cols {
                var s = 0.0
                for i in 0..<taps { s += a[i][p] * a[i][q] }
                n[p][q] = s
            }
        }
        let nInv = invert(n)
        var out = [Double](repeating: 0, count: taps)
        var scale = 1.0
        if derivative > 1 { for k in 2...derivative { scale *= Double(k) } }
        for i in 0..<taps {
            var s = 0.0
            for p in 0..<cols { s += nInv[derivative][p] * a[i][p] }
            out[i] = scale * s
        }
        return out
    }

    /// Gauss-Jordan with partial pivoting. The matrix is at most 4×4 and always
    /// non-singular here (a Vandermonde Gram matrix on distinct nodes).
    private static func invert(_ m: [[Double]]) -> [[Double]] {
        let n = m.count
        var a = m
        var inv = (0..<n).map { i in (0..<n).map { $0 == i ? 1.0 : 0.0 } }
        for col in 0..<n {
            var pivot = col
            for r in col..<n where abs(a[r][col]) > abs(a[pivot][col]) { pivot = r }
            if pivot != col { a.swapAt(col, pivot); inv.swapAt(col, pivot) }
            let d = a[col][col]
            precondition(d != 0, "singular normal-equation matrix")
            for c in 0..<n { a[col][c] /= d; inv[col][c] /= d }
            for r in 0..<n where r != col {
                let f = a[r][col]
                guard f != 0 else { continue }
                for c in 0..<n {
                    a[r][c] -= f * a[col][c]
                    inv[r][c] -= f * inv[col][c]
                }
            }
        }
        return inv
    }

    /// `H(ω)/(−ω²)` for the acceleration coefficients — 1.0 is an ideal
    /// differentiator. Used by tests to re-measure the table in this type's
    /// documentation instead of trusting it.
    func secondDerivativeGain(atHz f: Double, sampleInterval dt: Double) -> Double {
        guard f > 0, dt > 0 else { return .nan }
        let w = 2 * Double.pi * f
        var real = 0.0, imag = 0.0
        for i in 0..<taps {
            let k = Double(i - halfWindow)
            real += accCoefficients[i] * cos(w * k * dt)
            imag += accCoefficients[i] * sin(w * k * dt)
        }
        _ = imag  // symmetric window: the imaginary part is zero by construction
        return (real / (dt * dt)) / (-(w * w))
    }
}

// MARK: - Static-hold detection

/// Decides, from RAW MARKER MOTION alone, whether the subject was effectively
/// still around a given instant — so inverse dynamics there can be solved as a
/// static-equilibrium problem (q̇ = q̈ = 0) instead of from derivatives of a
/// motion we cannot fully observe.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHY THE CRITERION IS ON MARKERS, NOT ON `ddq`
/// ─────────────────────────────────────────────────────────────────────────
/// The tempting test is "is the Savitzky-Golay `ddq` small". It is circular
/// twice over. `ddq` is a twice-differentiated function of these same markers
/// (noise gain ≈ 1/dt², ≈3600 at 60 fps), and on the offline path the signal
/// being differentiated is missing its global translation entirely — the pose
/// model zeroes `global_trans`, pinning the pelvis at (0, 0.924, 0) every
/// frame. A small `ddq` on that path says the unobservable part of the motion
/// stayed unobservable. Marker displacement is the closest thing to a direct
/// observation this pipeline has.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHERE THE TWO CONSTANTS COME FROM (neither is tuned to a result)
/// ─────────────────────────────────────────────────────────────────────────
/// Static ID keeps gravity and discards `M·q̈ + C(q,q̇)q̇`. Both discarded
/// terms have to be small next to the gravitational term that survives.
///
/// 1. THE VELOCITY TERM. Centrifugal/Coriolis torque on a segment scales as
///    `m·v²/r` against a gravitational `m·g·r_g`; for comparable lever arms the
///    ratio is `v²/(g·r)`. On a 0.4 m thigh that is 1.0e-4 at v = 0.02 m/s and
///    **1.0% at v = 0.20 m/s**. So 0.20 m/s is where this term reaches one
///    percent, and that is what sets the speed cap.
///
/// 2. THE ACCELERATION TERM, bounded through duration. If no marker exceeds
///    `v` anywhere in a window of span `T`, then no marker's velocity changed
///    by more than `2v` across it, so the MEAN acceleration over the window is
///    at most `2v/T`. That mean must stay under `maxDiscardedMeanAccel`.
///
/// ⚠️ **Both constants were loosened 10× and 6.1× on 2026-08-07, and the
/// reason is that the old ones were not an error budget.** They were 0.02 m/s
/// and 0.08 m/s² — a discarded inertial term of **0.82% of g** — while the
/// stage immediately downstream, the muscle QP, carries a documented
/// **relative torque residual of 0.20 (neutral standing) to 0.35 (dancer)**,
/// and the pose source carries ~4.7° of joint-angle error. Demanding 0.8% from
/// one term while accepting 20-35% from the next one is a knob, not a budget:
/// it cannot improve the answer, because the term it is protecting is already
/// two orders of magnitude below the dominant error.
///
/// The replacement is stated as a fraction of g and chosen so the discarded
/// term stays an order of magnitude BELOW the muscle stage's own residual, so
/// it can never become the dominant error:
///
///     maxDiscardedMeanAccel = 0.05 · g = 0.49 m/s²      (5% of gravity)
///     holdSpeedThreshold    = 0.20 m/s                  (1% velocity term)
///
/// The implied minimum window span is `2v/a` = **0.8155 s** (it was 0.5 s).
/// Measured effect: at the offline path's 2 fps default the admissible peak
/// marker speed goes 2 cm/s → 20 cm/s, because the 4 s window the 9-tap filter
/// spans there makes the acceleration bound `2·0.2/4 = 0.1 m/s²`, far inside
/// the budget, so the speed cap is what binds.
///
/// ⚠️ This does NOT make a squat or a run publishable, and it is not meant to.
/// A runner's markers move at metres per second; no budget expressed as a
/// fraction of g reaches that. Motions that fast need the ROOT TRANSLATION,
/// which this input only carries when `cam_t` is composed in
/// (`MHRRetarget.makeBodyFrame(jointCoords:camT:…)`) — see
/// `rootTranslationObservable` below and STATUS.md.
///
/// ─────────────────────────────────────────────────────────────────────────
/// THE NOISE FLOOR — why "moving" and "cannot tell" are different answers
/// ─────────────────────────────────────────────────────────────────────────
/// `peakSpeed` is measured on marker positions that come from a pose model,
/// so it contains that model's per-frame jitter as well as the subject's
/// motion. At a sparse sampling rate the jitter alone can exceed the stillness
/// bound: `labs/sam-3d-body/findings/stability.json` measured body-joint
/// displacement of median 6.0 mm / max 24.7 mm under a 1% person-box
/// perturbation, which at 2 fps is 1.2 / 4.9 cm/s.
///
/// So this type measures the floor instead of assuming it, from distances that
/// physically CANNOT change: the rigid inter-joint-centre distances in
/// `rigidPairs` (hip width, femur, shank, humerus, …). Both skeletons hold
/// those fixed, so any frame-to-frame change in one is pure measurement noise.
/// If two markers carry independent position noise, a distance change of `d`
/// needs at least `d/2` on one of them, so `median|Δd| / (2·dt)` is a rigorous
/// LOWER bound on the per-marker speed attributable to noise.
///
/// When the floor is itself above the stillness bound, the honest verdict is
/// `.indistinguishableFromNoise` — the instrument cannot resolve the question —
/// and the advice is about the footage, not about holding still.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHAT THIS CANNOT DO — the bound is on the MEAN, over observed samples
/// ─────────────────────────────────────────────────────────────────────────
/// * Sub-sampling-rate oscillation (tremor, a bounce faster than the frame
///   interval) has small displacement and arbitrarily large instantaneous
///   acceleration. Nothing here sees it. At the offline path's 2 fps default
///   that is a wide blind spot.
/// * Rigid whole-body acceleration with no articulation — a subject
///   accelerating in a vehicle — moves no marker RELATIVE to the pinned
///   pelvis and reads as a hold. Articulated motion (a squat: the feet appear
///   to rise) does move markers and is caught.
/// * `peakSpeed` is a MAX over markers, deliberately. That is the strict
///   reading of "still", and it makes the detector fail toward "moving" —
///   refusing to report muscle numbers — which is the correct direction for
///   this product claim. It also means the detector inherits the pose
///   estimator's per-frame noise floor, which is why that floor is now
///   MEASURED (see above) instead of being an unstated assumption:
///   `labs/sam-3d-body/findings/stability.json` measured the model itself as
///   bit-identical across repeats (0.0 mm) but body-joint displacement of
///   median 2.4 mm / max 17.4 mm under a 0.1% bounding-box perturbation, and
///   median 6.0 mm / max 24.7 mm at 1%. Against the OLD 2 cm/s cap those
///   maxima (3.5 and 4.9 cm/s at 2 fps) were already above the threshold, so a
///   perfectly still subject was reported as "moving" — with advice they could
///   not act on. `.indistinguishableFromNoise` is that case, named.
/// * It does NOT check that the CAMERA was still, because it never sees an
///   image. `cam_t` and `joint_coords` are both camera-relative, so a rotating
///   camera makes the reconstructed frame non-inertial. Measured on the user's
///   own clips at native frame rate (background phase correlation outside a
///   dilated person box, chained): `video_012` drifts 1.79 image diagonals in
///   10.2 s — 13.5 °/s, which displaces a point 1 m from the subject by 6.4 cm
///   within ONE 0.27 s filter window — against 0.2-0.6 °/s (0.1-0.3 cm) on
///   `video_013`/`video_015`. That is a 20-60× separation, so the check is
///   buildable; it belongs upstream, where the frames are. ⚠️ It must run at
///   the video's NATIVE rate: at a 10 fps analysis sampling the same estimator
///   aliased `video_012`'s pan down to ~0.
struct StaticHoldDetector {

    /// Standard gravity, the term static ID keeps. Both budgets below are
    /// stated as fractions of it so they cannot drift into being knobs.
    static let gravityMetersPerSecondSquared: Double = 9.81

    /// Fraction of g the discarded inertial term is allowed to reach.
    /// 5% — an order of magnitude below the muscle QP's own 20-35% relative
    /// torque residual, so it cannot become the dominant error.
    static let discardedAccelFractionOfG: Double = 0.05

    /// Per-marker speed at or below which a sample counts as still, m/s.
    /// 0.20 m/s is where the centrifugal/Coriolis term `v²/(g·r)` reaches 1%
    /// on a 0.4 m segment — see the type comment, item 1.
    static let holdSpeedThresholdMetersPerSecond: Double = 0.20

    /// Ceiling on `2·peakSpeed/windowSpan`, the bound this window puts on the
    /// mean acceleration static ID throws away. 0.49 m/s² = 5% of g.
    static let maxDiscardedMeanAccelMetersPerSecondSquared: Double =
        discardedAccelFractionOfG * gravityMetersPerSecondSquared

    /// Ring capacity. Only has to cover the Savitzky-Golay window plus enough
    /// history to reach `2v/a` = 0.8155 s; 600 samples is 10 s at 60 fps and
    /// costs a few tens of kB.
    static let maxHistorySamples = 600

    /// Inter-joint-centre distances that are RIGID in both the MHR skeleton and
    /// `FullBody.osim`, so any frame-to-frame change in one is measurement
    /// noise rather than motion. Only direct joint reads are used — the blended
    /// spine markers (`SPINE_L`, `SPINE_M`, `NECK`, `HEAD`) interpolate between
    /// two MHR joints and are not rigid to each other.
    static let rigidPairs: [(String, String)] = [
        ("LHJC", "RHJC"), ("LSJC", "RSJC"),
        ("LHJC", "LKJC"), ("RHJC", "RKJC"),
        ("LKJC", "LAJC"), ("RKJC", "RAJC"),
        ("LSJC", "LEJC"), ("RSJC", "REJC"),
        ("LEJC", "LWJC"), ("REJC", "RWJC"),
    ]

    /// Marker whose constancy reveals that the pose source pinned the root.
    /// `MHRRetarget` emits `PELVIS` straight from MHR joint 1, which
    /// `joint_coords` fixes at the model constant (0, 0.924, 0) to the last
    /// bit whenever `cam_t` has not been composed in.
    static let rootMarkerName = "PELVIS"

    /// Below this, two `PELVIS` samples are treated as the same value. The
    /// pinned constant repeats bit-exactly and any real translation is metres,
    /// so this only has to be smaller than float noise.
    static let rootPinnedToleranceMeters: Double = 1e-9

    /// One pushed frame's motion relative to its predecessor.
    struct Sample {
        let timestamp: TimeInterval
        /// nil when there was no comparable predecessor (first sample of a
        /// clip, or no marker in common with the previous frame). Carries no
        /// motion information and is excluded from the max rather than being
        /// counted as zero.
        let peakSpeed: Double?
        let medianSpeed: Double?
        /// Largest change, in metres, of any distance in `rigidPairs` since the
        /// previous sample. Those distances cannot change, so this is pure
        /// pose-estimation noise. nil when no pair was measurable.
        let rigidDistanceDriftMeters: Double?
        /// Distance the root marker moved since the previous sample, metres.
        /// nil when it was absent from either frame.
        let rootDisplacementMeters: Double?
    }

    private var history: [Sample] = []
    private var previous: (timestamp: TimeInterval, markers: [String: SIMD3<Double>])?

    /// Feed one frame's markers, in the same flat `[x,y,z,…]` layout
    /// `NimbleBridge.solveIK` takes.
    mutating func ingest(flatMarkerPositions positions: [NSNumber],
                         markerNames names: [String],
                         timestamp: TimeInterval) {
        var markers: [String: SIMD3<Double>] = [:]
        markers.reserveCapacity(names.count)
        for (i, name) in names.enumerated() where positions.count >= (i + 1) * 3 {
            markers[name] = SIMD3<Double>(positions[i * 3].doubleValue,
                                          positions[i * 3 + 1].doubleValue,
                                          positions[i * 3 + 2].doubleValue)
        }

        var peak: Double?
        var median: Double?
        var rigidDrift: Double?
        var rootStep: Double?
        if let prev = previous {
            let dt = timestamp - prev.timestamp
            // Non-increasing timestamps mean the caller reset or replayed;
            // there is no speed to compute, so record the sample as
            // unmeasured rather than dividing by zero.
            if dt > 0 {
                var speeds: [Double] = []
                speeds.reserveCapacity(markers.count)
                for (name, p) in markers {
                    guard let q = prev.markers[name] else { continue }
                    speeds.append(simd_length(p - q) / dt)
                }
                if !speeds.isEmpty {
                    speeds.sort()
                    peak = speeds[speeds.count - 1]
                    median = speeds[speeds.count / 2]
                }

                // Noise probe: distances that physically cannot change.
                var drifts: [Double] = []
                for (a, b) in Self.rigidPairs {
                    guard let pa = markers[a], let pb = markers[b],
                          let qa = prev.markers[a], let qb = prev.markers[b] else { continue }
                    drifts.append(abs(simd_length(pa - pb) - simd_length(qa - qb)))
                }
                if !drifts.isEmpty { rigidDrift = drifts.max() }

                if let p = markers[Self.rootMarkerName], let q = prev.markers[Self.rootMarkerName] {
                    rootStep = simd_length(p - q)
                }
            }
        }

        history.append(Sample(timestamp: timestamp, peakSpeed: peak, medianSpeed: median,
                              rigidDistanceDriftMeters: rigidDrift,
                              rootDisplacementMeters: rootStep))
        if history.count > Self.maxHistorySamples {
            history.removeFirst(history.count - Self.maxHistorySamples)
        }
        previous = (timestamp, markers)
    }

    /// Verdict for the Savitzky-Golay window centred at `center`.
    ///
    /// The examined window is the last `SavitzkyGolayFilter.windowSize`
    /// samples, extended further back while its span is shorter than the
    /// duration the acceleration budget needs (`2·threshold/maxAccel`).
    ///
    /// Two reasons it starts from the whole filter window rather than from a
    /// short interval around `center`:
    ///
    /// 1. Those are the samples that produced the `ddq` being replaced.
    /// 2. They are also the samples that produced the smoothed `q` that IS
    ///    still used. If part of the window belongs to a different pose, the
    ///    cubic fit at the centre is pulled toward it, so requiring the whole
    ///    window to be still protects the POSE, not just the acceleration.
    ///
    /// ⚠️ Consequence worth knowing before reading a clip: the effective
    /// stillness requirement is therefore `max(8 × sampleInterval, 2v/a)`, not
    /// `2v/a` alone. At the offline path's 2 fps default the filter width alone
    /// makes it FOUR SECONDS, so a two-second hold mid-clip yields no muscle
    /// frames at all. That comes from the filter width times the sampling rate,
    /// not from either constant here — the actionable remedy is a higher
    /// sampling rate, not a longer hold. At 10 fps the same window is 0.8 s.
    ///
    /// The extension is backward-only, because samples past `center + 4` have
    /// not been pushed yet. That asymmetry is harmless: the symmetric filter
    /// window is always fully inside the examined window, and the extension
    /// only adds further evidence on the past side.
    func classify(centeredAt center: TimeInterval) -> NimbleEngine.MotionClassification {
        guard let newest = history.last else {
            return Self.empty(at: center, windowSeconds: 0, sampleCount: 0)
        }

        let requiredSpan = 2 * Self.holdSpeedThresholdMetersPerSecond
            / Self.maxDiscardedMeanAccelMetersPerSecondSquared
        var start = max(0, history.count - SavitzkyGolayFilter.windowSize)
        while start > 0 && (newest.timestamp - history[start].timestamp) < requiredSpan {
            start -= 1
        }

        let window = history[start...]
        let span = newest.timestamp - window.first!.timestamp
        let measured = window.compactMap(\.peakSpeed)
        let medians = window.compactMap(\.medianSpeed).sorted()

        // The root translation is observable iff the root marker moved AT ALL
        // anywhere in the window. A pose source that pins the pelvis repeats
        // one model constant bit-for-bit, so this separates a pinned stream
        // from a `cam_t`-composed one with no flag to plumb and no way for the
        // two to disagree. A held pose reads as pinned, which is harmless: the
        // hold branch does not use the root acceleration.
        let rootSteps = window.compactMap(\.rootDisplacementMeters)
        let rootObservable = (rootSteps.max() ?? 0) > Self.rootPinnedToleranceMeters

        // Lower bound on the part of `peak` that is instrument noise. Median
        // over the window of the largest rigid-distance drift, halved (two
        // markers share the drift) and divided by the sampling interval.
        let drifts = window.compactMap(\.rigidDistanceDriftMeters).sorted()
        let sampleInterval = window.count > 1 ? span / Double(window.count - 1) : 0
        let noiseFloor: Double
        if drifts.isEmpty || sampleInterval <= 0 {
            noiseFloor = 0
        } else {
            noiseFloor = drifts[drifts.count / 2] / (2 * sampleInterval)
        }

        // No measured sample at all means no motion information — not stillness.
        guard let peak = measured.max() else {
            return Self.empty(at: center, windowSeconds: span, sampleCount: window.count,
                              noiseFloor: noiseFloor, rootObservable: rootObservable)
        }

        // 2·v/T. A zero-span window (every sample at one timestamp, i.e. a
        // single photo before edge padding spreads it) can only pass if
        // nothing moved at all.
        let impliedAccel: Double
        if span > 0 {
            impliedAccel = 2 * peak / span
        } else {
            impliedAccel = peak > 0 ? .infinity : 0
        }

        let withinBudget = peak <= Self.holdSpeedThresholdMetersPerSecond
            && impliedAccel <= Self.maxDiscardedMeanAccelMetersPerSecondSquared

        // Order matters. A hold is a hold regardless of the noise floor — the
        // floor can only ever make `peak` look BIGGER, so a peak that is
        // already inside the budget is inside it a fortiori. The floor is only
        // consulted to explain a FAILURE, where it decides between "the subject
        // moved" and "this footage cannot tell us".
        let verdict: NimbleEngine.MotionVerdict
        if withinBudget {
            verdict = .hold
        } else if noiseFloor >= Self.holdSpeedThresholdMetersPerSecond {
            verdict = .indistinguishableFromNoise
        } else {
            verdict = .movingBeyondStaticBudget
        }

        return NimbleEngine.MotionClassification(
            timestamp: center,
            isHold: verdict == .hold,
            peakMarkerSpeedMetersPerSecond: peak,
            medianMarkerSpeedMetersPerSecond: medians.isEmpty ? 0 : medians[medians.count / 2],
            windowSeconds: span,
            sampleCount: window.count,
            impliedMeanAccelMetersPerSecondSquared: impliedAccel,
            verdict: verdict,
            poseNoiseFloorMetersPerSecond: noiseFloor,
            rootTranslationObservable: rootObservable)
    }

    private static func empty(at center: TimeInterval,
                              windowSeconds: Double,
                              sampleCount: Int,
                              noiseFloor: Double = 0,
                              rootObservable: Bool = false) -> NimbleEngine.MotionClassification {
        NimbleEngine.MotionClassification(
            timestamp: center, isHold: false,
            peakMarkerSpeedMetersPerSecond: 0, medianMarkerSpeedMetersPerSecond: 0,
            windowSeconds: windowSeconds, sampleCount: sampleCount,
            impliedMeanAccelMetersPerSecondSquared: .infinity,
            verdict: .noMeasurement,
            poseNoiseFloorMetersPerSecond: noiseFloor,
            rootTranslationObservable: rootObservable)
    }

    mutating func reset() {
        history.removeAll(keepingCapacity: true)
        previous = nil
    }
}
