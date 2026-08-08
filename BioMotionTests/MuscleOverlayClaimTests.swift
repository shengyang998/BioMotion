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
///    by a number whose per-muscle scale is unknown — 66 of `FullBody.osim`'s
///    muscles are given a straight-line path where the tendon wraps around bone,
///    so each activation is inflated by 1/k for its own pose-dependent k.
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
        XCTAssertTrue(note.contains("wraps around bone"), note)
        XCTAssertTrue(note.contains("cannot be read against another muscle's"),
                      "it names the comparison being refused: \(note)")
        // No number in it. A figure here would be the claim it exists to deny.
        XCTAssertFalse(note.contains("%"), note)
        XCTAssertNil(note.rangeOfCharacter(from: CharacterSet.decimalDigits), note)
    }
}
