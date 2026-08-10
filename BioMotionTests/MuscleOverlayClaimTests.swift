import XCTest
import UIKit
import simd
@testable import BioMotion

/// **What the 3-D picture is allowed to say.**
///
/// The panel retired the per-muscle claim on 2026-08-08; this renderer went on
/// making the cross-muscle half of it in colour, on both surfaces, until
/// 2026-08-08 (second round). Two mechanisms carried it and both are asserted
/// here:
///
/// 1. **Selection.** The old second pass took `rawActivations`, dropped
///    everything near the solver's floor, sorted the rest descending and drew
///    the strongest 24. Which capsules EXISTED was therefore already a ranking
///    by a number whose per-muscle scale is unknown. The reason given at the
///    time was that 66 of `FullBody.osim`'s muscles were given a straight-line
///    path where the tendon wraps around bone, inflating each activation by 1/k
///    for its own pose-dependent k; **those paths are wrapped since 2026-08-08**
///    (76 solved / 0 unmodelled) and the ranking is still refused, because
///    nothing puts two muscles' activations on one scale. The QP's own
///    termination slack was a second reason for one commit — a median 14.88 pp
///    of a left/right activation figure at fixed geometry — and `scaling = 0`
///    plus `polishing = 1` took it to 4.4994e-05 pp on 2026-08-09. The absent
///    common scale is the one that cannot be fixed by solving harder.
/// 2. **Colour.** Every capsule was coloured from one shared blue→red ramp, with
///    alpha rising alongside it, so the drawn set was ordered a second time.
///
/// The guarantee against (1) and (2) coming back is mostly structural —
/// `MuscleOverlay.update(joints:)` takes no muscle solve, so a magnitude cannot
/// reach the renderer at all — and the compiler enforces that at every call
/// site. What a test can still add is a guard on the seams a future change would
/// have to widen: the plan's contents, the `Capsule` type's fields, and the
/// colour function.
final class MuscleOverlayClaimTests: XCTestCase {

    /// Renderer, control, and disclosure share one fail-closed presentation
    /// decision. The final expectation is deliberately singular: it is both
    /// the capsule visibility and the disclosure visibility.
    func testLiveAnatomyPresentationUsesOneFailClosedTruthTable() {
        struct Row {
            let surface: LiveAnatomyPresentation.Surface
            let isTracking: Bool
            let hasCurrentFrame: Bool
            let isEnabled: Bool
            let showsControl: Bool
            let anatomyIsPresented: Bool
        }

        let rows: [Row] = [
            Row(surface: .calibration, isTracking: false, hasCurrentFrame: false,
                isEnabled: false, showsControl: false, anatomyIsPresented: false),
            Row(surface: .calibration, isTracking: false, hasCurrentFrame: false,
                isEnabled: true, showsControl: false, anatomyIsPresented: false),
            Row(surface: .calibration, isTracking: false, hasCurrentFrame: true,
                isEnabled: false, showsControl: false, anatomyIsPresented: false),
            Row(surface: .calibration, isTracking: false, hasCurrentFrame: true,
                isEnabled: true, showsControl: false, anatomyIsPresented: false),
            Row(surface: .calibration, isTracking: true, hasCurrentFrame: false,
                isEnabled: false, showsControl: false, anatomyIsPresented: false),
            Row(surface: .calibration, isTracking: true, hasCurrentFrame: false,
                isEnabled: true, showsControl: false, anatomyIsPresented: false),
            Row(surface: .calibration, isTracking: true, hasCurrentFrame: true,
                isEnabled: false, showsControl: false, anatomyIsPresented: false),
            Row(surface: .calibration, isTracking: true, hasCurrentFrame: true,
                isEnabled: true, showsControl: false, anatomyIsPresented: false),
            Row(surface: .tracking, isTracking: false, hasCurrentFrame: false,
                isEnabled: false, showsControl: false, anatomyIsPresented: false),
            Row(surface: .tracking, isTracking: false, hasCurrentFrame: false,
                isEnabled: true, showsControl: false, anatomyIsPresented: false),
            Row(surface: .tracking, isTracking: false, hasCurrentFrame: true,
                isEnabled: false, showsControl: false, anatomyIsPresented: false),
            Row(surface: .tracking, isTracking: false, hasCurrentFrame: true,
                isEnabled: true, showsControl: false, anatomyIsPresented: false),
            Row(surface: .tracking, isTracking: true, hasCurrentFrame: false,
                isEnabled: false, showsControl: false, anatomyIsPresented: false),
            Row(surface: .tracking, isTracking: true, hasCurrentFrame: false,
                isEnabled: true, showsControl: false, anatomyIsPresented: false),
            Row(surface: .tracking, isTracking: true, hasCurrentFrame: true,
                isEnabled: false, showsControl: true, anatomyIsPresented: false),
            Row(surface: .tracking, isTracking: true, hasCurrentFrame: true,
                isEnabled: true, showsControl: true, anatomyIsPresented: true),
        ]

        XCTAssertEqual(
            rows.count,
            LiveAnatomyPresentation.Surface.allCases.count * 8,
            "each surface must cover every tracking/frame/enabled combination"
        )
        let keys = Set(rows.map {
            "\($0.surface)|\($0.isTracking)|\($0.hasCurrentFrame)|\($0.isEnabled)"
        })
        XCTAssertEqual(keys.count, 16, "the truth table must cover each input combination once")

        for row in rows {
            let presentation = LiveAnatomyPresentation(
                surface: row.surface,
                isTracking: row.isTracking,
                hasCurrentFrame: row.hasCurrentFrame,
                isEnabled: row.isEnabled
            )
            let context = "\(row.surface), tracking=\(row.isTracking), "
                + "frame=\(row.hasCurrentFrame), enabled=\(row.isEnabled)"
            XCTAssertEqual(presentation.showsControl, row.showsControl, context)
            XCTAssertEqual(
                presentation.anatomyIsPresented,
                row.anatomyIsPresented,
                "renderer and disclosure diverged: \(context)"
            )
            XCTAssertFalse(
                presentation.anatomyIsPresented && !presentation.showsControl,
                "anatomy cannot be visible without its control: \(context)"
            )
        }
    }

    /// The pure truth table is necessary but not sufficient: leaving it unused
    /// would let the old raw-toggle/model-gated wiring return while this suite
    /// stayed green. Pin the three production call sites to the unified value.
    func testLiveAnatomyCallSitesConsumeTheUnifiedPresentation() throws {
        let overlaySource = try normalizedSource(
            at: "BioMotion/ARKit/SkeletonOverlayView.swift"
        )
        let contentSource = try normalizedSource(at: "BioMotion/App/ContentView.swift")
        let calibrationSource = try normalizedSource(
            at: "BioMotion/App/CalibrationView.swift"
        )

        XCTAssertFalse(overlaySource.contains("var isTracking: Bool = true"),
                       "tracking must remain a required renderer input")
        XCTAssertFalse(overlaySource.contains("var showMuscles: Bool = true"),
                       "the old fail-open capsule toggle returned")
        XCTAssertEqual(
            occurrences(of: "anatomyPresentation.anatomyIsPresented", in: overlaySource),
            2,
            "renderer visibility and joint update must consume the same final gate"
        )

        XCTAssertEqual(
            occurrences(of: "anatomyPresentation: liveAnatomyPresentation", in: contentSource),
            1,
            "the tracking renderer is no longer wired to the shared presentation"
        )
        XCTAssertEqual(
            occurrences(of: "if liveAnatomyPresentation.anatomyIsPresented {", in: contentSource),
            1,
            "the disclosure is no longer wired to the renderer's final gate"
        )
        XCTAssertEqual(
            occurrences(of: "if liveAnatomyPresentation.showsControl {", in: contentSource),
            1,
            "the Anatomy control is no longer wired to the shared presentation"
        )
        XCTAssertFalse(
            contentSource.contains(
                "nimble.isModelLoaded && bodyTracking.isTracking && showAnatomyOverlay"
            ),
            "the joint-only anatomy layer must not regain a Nimble model gate"
        )

        XCTAssertEqual(
            occurrences(of: "anatomyPresentation: LiveAnatomyPresentation(",
                           in: calibrationSource),
            1
        )
        XCTAssertEqual(occurrences(of: "surface: .calibration", in: calibrationSource), 1)
        XCTAssertEqual(occurrences(of: "isEnabled: true", in: calibrationSource), 1,
                       "calibration must be refused by surface policy, not a lucky false toggle")
    }

    private func source(at relativePath: String) throws -> String {
        let repositoryRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        return try String(
            contentsOf: repositoryRoot.appendingPathComponent(relativePath),
            encoding: .utf8
        )
    }

    private func normalizedSource(at relativePath: String) throws -> String {
        try source(at: relativePath)
            .split(whereSeparator: \.isWhitespace)
            .joined(separator: " ")
    }

    private func occurrences(of needle: String, in haystack: String) -> Int {
        haystack.components(separatedBy: needle).count - 1
    }

    /// A subject standing upright and facing the camera, in ARKit world axes:
    /// X image-right, Y up, Z toward the camera. Their OWN right is at −X, the
    /// same convention `BodyFrameOrientationTests` pins.
    private func standingJoints() -> [String: SIMD3<Float>] {
        [
            "hips_joint":          SIMD3(0, 0.95, 0),
            "spine_1_joint":       SIMD3(0, 1.10, 0),
            "spine_4_joint":       SIMD3(0, 1.35, 0),
            "right_upLeg_joint":   SIMD3(-0.09, 0.90, 0),
            "left_upLeg_joint":    SIMD3(0.09, 0.90, 0),
            "right_leg_joint":     SIMD3(-0.09, 0.50, 0),
            "left_leg_joint":      SIMD3(0.09, 0.50, 0),
            "right_foot_joint":    SIMD3(-0.09, 0.08, 0),
            "left_foot_joint":     SIMD3(0.09, 0.08, 0),
        ]
    }

    /// **The drawn set is the fixed anatomical list, not a selection.**
    ///
    /// Fails if a magnitude-ranked pass comes back: those capsules were keyed
    /// `path_<raw solver name>` and are not in `muscleDefs`.
    func testTheOverlayDrawsAFixedAnatomicalSetAndNotARanking() {
        let plan = MuscleOverlay.capsulePlan(joints: standingJoints())
        let drawn = Set(plan.map(\.name))
        let defined = Set(MuscleOverlay.muscleDefs.map(\.name))

        XCTAssertEqual(drawn, defined,
                       "the drawn set is not the anatomical list — extra: "
                       + "\(drawn.subtracting(defined).sorted()), missing: "
                       + "\(defined.subtracting(drawn).sorted())")
        XCTAssertEqual(plan.count, MuscleOverlay.muscleDefs.count)
        XCTAssertEqual(plan.count, 26, "the anatomical set changed size; if that is deliberate, "
                       + "say so here — it is the number of capsules a user sees")
        XCTAssertTrue(plan.allSatisfy { !$0.name.hasPrefix("path_") },
                      "a path-keyed capsule is one the old ranking pass selected by activation")
        // Every capsule the plan emits is drawable: a zero-length one would be
        // scaled to nothing and read as a missing muscle rather than a dropped
        // one.
        for capsule in plan {
            XCTAssertGreaterThan(simd_length(capsule.end - capsule.start), 0.01, capsule.name)
            XCTAssertGreaterThan(capsule.radius, 0, capsule.name)
        }
    }

    /// **One colour for the whole body.** A capsule drawn differently from its
    /// neighbour is a cross-muscle statement whatever the legend says, and the
    /// old ramp made it in three channels at once (hue, and alpha 0.45 → 0.95).
    func testEveryCapsuleIsDrawnInTheSameColour() {
        let plan = MuscleOverlay.capsulePlan(joints: standingJoints())
        XCTAssertFalse(plan.isEmpty)

        var seen = Set<String>()
        for capsule in plan {
            var r: CGFloat = 0, g: CGFloat = 0, b: CGFloat = 0, a: CGFloat = 0
            MuscleOverlay.color(for: capsule).getRed(&r, green: &g, blue: &b, alpha: &a)
            seen.insert(String(format: "%.4f,%.4f,%.4f,%.4f", r, g, b, a))
        }
        XCTAssertEqual(seen.count, 1,
                       "capsules are coloured against each other again: \(seen.sorted())")

        // And the constant is not fully transparent or fully opaque — the layer
        // has to be visible over the camera feed without hiding the skeleton.
        var alpha: CGFloat = 0
        MuscleOverlay.capsuleColor.getRed(nil, green: nil, blue: nil, alpha: &alpha)
        XCTAssertGreaterThan(alpha, 0.2)
        XCTAssertLessThan(alpha, 0.9)
    }

    /// **The capsule type has no magnitude channel**, so there is nowhere for an
    /// activation to ride into the renderer alongside the geometry. Reflection
    /// rather than a compile check, because adding a field is exactly the change
    /// that would otherwise pass review unremarked.
    func testTheCapsuleTypeCarriesNoActivationOrColourField() throws {
        let capsule = try XCTUnwrap(MuscleOverlay.capsulePlan(joints: standingJoints()).first)
        let fields = Mirror(reflecting: capsule).children.compactMap(\.label).sorted()
        XCTAssertEqual(fields, ["end", "name", "radius", "start"],
                       "a field was added to MuscleOverlay.Capsule: \(fields). If it carries a "
                       + "per-muscle magnitude, the retired cross-muscle claim is back.")
    }

    /// A capsule whose joints are not both tracked is dropped, not drawn from a
    /// stale or defaulted position — and dropping one does not disturb the rest.
    func testACapsuleWhoseJointsAreMissingIsNotDrawn() {
        var joints = standingJoints()
        joints.removeValue(forKey: "left_foot_joint")
        let names = Set(MuscleOverlay.capsulePlan(joints: joints).map(\.name))

        for shank in ["gasmed_l", "gaslat_l", "soleus_l", "tibant_l"] {
            XCTAssertFalse(names.contains(shank), "\(shank) drawn without its foot joint")
        }
        for shank in ["gasmed_r", "gaslat_r", "soleus_r", "tibant_r"] {
            XCTAssertTrue(names.contains(shank), "\(shank) lost to the other side's missing joint")
        }
        XCTAssertEqual(names.count, MuscleOverlay.muscleDefs.count - 4)
    }

    /// The sentence the screens carry beside the capsules. It has one job: stop
    /// the picture speaking for itself, in the order a reader scans.
    func testTheNoteStatesTheAbsenceFirstAndThenTheReason() {
        let note = MuscleOverlay.anatomyOnlyNote
        print("UI-METRIC anatomy_only_note=\(note)")

        XCTAssertTrue(note.hasPrefix("Muscle effort is not shown."),
                      "the absence has to be the first thing read: \(note)")
        XCTAssertTrue(note.contains("WHERE"), note)
        XCTAssertTrue(note.contains("colour is fixed"), note)
        XCTAssertTrue(note.contains("moment arm"), "it names the mechanism: \(note)")
        // ⚠️ This assertion has pinned a repaired defect TWICE. It read
        // `contains("wraps around bone")` until 2026-08-09, after the ellipsoid
        // commit had already removed it (76 solved / 0 unmodelled); its
        // replacement read `contains("close enough")`, and the solver fix in the
        // next commit took that gap from 14.88 pp to 4.4994e-05 pp.
        //
        // The reason that survived BOTH is structural and cannot be repaired:
        // the sharing step divides by each muscle's own leverage and its own
        // maximum force, so no two of the resulting fractions share a scale
        // however exactly they are computed. Both refuted versions are now
        // negative assertions.
        XCTAssertTrue(note.contains("sharing that moment"), "it names the step: \(note)")
        XCTAssertTrue(note.contains("own leverage and its own maximum force"),
                      "and why the fractions cannot share a scale: \(note)")
        XCTAssertTrue(note.contains("Nothing in it puts two different muscles' efforts on one "
                                    + "scale"),
                      "and the reason that never depended on the paths: \(note)")
        XCTAssertFalse(note.contains("straight line"),
                       "the path defect was fixed on 2026-08-08 and this sentence outlived it "
                       + "on both screens: \(note)")
        XCTAssertFalse(note.contains("close enough"),
                       "the sharing step returns its own exact answer since `scaling = 0` and "
                       + "`polishing = 1`; this is the second reason to outlive its own "
                       + "repair: \(note)")
        XCTAssertTrue(note.contains("cannot be read against another muscle's"),
                      "it names the comparison being refused: \(note)")
        // No number in it. A figure here would be the claim it exists to deny.
        XCTAssertFalse(note.contains("%"), note)
        XCTAssertNil(note.rangeOfCharacter(from: CharacterSet.decimalDigits), note)
    }
}
