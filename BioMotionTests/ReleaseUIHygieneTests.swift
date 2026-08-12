import XCTest
@testable import BioMotion

/// Regression coverage for the boundary between actionable product messages
/// and internal diagnostics. Each sentinel resembles data an NSError or an
/// operator-only path can carry; none may be rendered to a shipping user.
final class ReleaseUIHygieneTests: XCTestCase {
    private static let rawDiagnostic =
        "NSCocoaErrorDomain 513 /private/var/mobile/Containers/tools/assetpack/package.sh"

    func testAssetPackFailuresExposeOnlyStablePublicMessages() {
        let temporary = AssetPackModelStore.Failure(
            kind: .temporary,
            internalDiagnostic: Self.rawDiagnostic
        )
        let update = AssetPackModelStore.Failure(
            kind: .updateRequired,
            internalDiagnostic: "missing SAM3DBodyPose.mlmodelc/coremldata.bin"
        )
        let download = AssetPackModelStore.Failure(
            kind: .download,
            internalDiagnostic: "BAErrorDomain 17"
        )

        for failure in [temporary, update, download] {
            XCTAssertFalse(failure.publicMessage.contains(Self.rawDiagnostic))
            XCTAssertFalse(failure.publicMessage.contains("ErrorDomain"))
            XCTAssertFalse(failure.publicMessage.contains(".mlmodelc"))
            XCTAssertFalse(failure.publicMessage.contains("tools/"))
        }
        XCTAssertTrue(temporary.publicMessage.contains("connection"))
        XCTAssertTrue(update.publicMessage.contains("update BioMotion"))
        XCTAssertTrue(download.publicMessage.contains("Retry"))
    }

    func testOfflineRunAndPickerFailuresAreTypedPublicMessages() {
        let runFailures: [OfflineSessionRunner.RunFailure] = [
            .poseModelLoad,
            .musculoskeletalModelLoad,
            .selectionRead,
            .noFrames,
        ]
        for failure in runFailures {
            XCTAssertFalse(failure.publicMessage.contains(Self.rawDiagnostic))
            XCTAssertFalse(failure.publicMessage.contains("FullBody.osim"))
        }

        let pickerFailures: [OfflineImportSelectionState.Failure] = [
            .photoUnavailable,
            .videoUnavailable,
            .selectionUnavailable,
        ]
        for failure in pickerFailures {
            XCTAssertFalse(failure.publicMessage.contains(Self.rawDiagnostic))
            XCTAssertTrue(failure.publicMessage.contains("selected"))
        }
    }

    func testPerFramePoseFailuresNeverRenderEstimatorDiagnostics() {
        let failures: [OfflineResultStore.FrameStatus.PoseFailure] = [
            .modelProcessing,
            .noUsableJoints,
        ]
        for failure in failures {
            XCTAssertFalse(failure.publicDescription.contains(Self.rawDiagnostic))
            XCTAssertFalse(failure.publicDescription.contains("joint_coords"))
            XCTAssertFalse(failure.publicDescription.contains("CoreML"))
        }
    }

    func testBodyPlausibilityUsesTypedFailureAndKeepsFinitePoseEstimates() {
        let incomplete = MHRRetarget.plausibility(jointCoords: [])
        guard case .implausible(let failure, _, _) = incomplete else {
            return XCTFail("an incomplete pose must fail closed")
        }
        XCTAssertEqual(failure, .incompletePrediction)
        XCTAssertTrue(failure.publicDescription.contains("full body"))
        XCTAssertFalse(failure.publicDescription.contains("0 joints"))

        let outOfRange = MHRRetarget.PlausibilityFailure.hipWidthOutOfRange
            .publicDescription(hipWidthMeters: 0.07, statureMeters: 1.62)
        XCTAssertTrue(outOfRange.contains("7 cm"))
        XCTAssertTrue(outOfRange.contains("pose estimate"))
    }

    func testGaitNotAttemptedUsesTypedActionableCopy() {
        let short = OfflineResultStore.GaitAttemptFailure.insufficientFrames(
            usable: 1,
            required: 9
        )
        let failed = OfflineResultStore.GaitAttemptFailure.analysisFailed

        XCTAssertTrue(short.publicMessage.contains("1"))
        XCTAssertTrue(short.publicMessage.contains("9"))
        XCTAssertFalse(failed.publicMessage.contains(Self.rawDiagnostic))
        XCTAssertTrue(failed.publicMessage.contains("clear running clip"))
    }

    func testCaptureExportOutcomeSeparatesTypedFailureFromDiagnostics() {
        let outcome = CaptureExportOutcome(
            urls: [],
            failure: .writeFailed,
            internalDiagnostic: Self.rawDiagnostic,
            hasMotionArtifact: false
        )

        XCTAssertEqual(outcome.failure?.publicMessage,
                       "BioMotion could not write the export files. Check that the device has free storage, then try again.")
        XCTAssertFalse(outcome.failure?.publicMessage.contains(Self.rawDiagnostic) == true)
        XCTAssertEqual(outcome.internalDiagnostic, Self.rawDiagnostic)
    }

    func testSharedExportWarningsCannotCarryRawErrors() throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent("biomotion-release-warning-\(UUID().uuidString)")
        defer { try? FileManager.default.removeItem(at: directory) }
        try FileManager.default.createDirectory(
            at: directory,
            withIntermediateDirectories: true
        )

        let urls = try ExportDisclosure.prepareShareURLs(
            successfulURLs: [],
            warnings: [.markerFileUnavailable],
            directory: directory
        )
        let warningURL = try XCTUnwrap(urls.last)
        let warning = try String(contentsOf: warningURL)

        XCTAssertFalse(warning.contains(Self.rawDiagnostic))
        XCTAssertFalse(warning.contains("ErrorDomain"))
        XCTAssertFalse(warning.contains("/private/var"))
        XCTAssertTrue(warning.contains("marker-position file could not be produced"))
    }

    func testPostureCopyUsesProductLanguageInsteadOfRepositoryDiagnostics() throws {
        let sourceURL = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .appendingPathComponent("BioMotion/Findings/PostureFindings.swift")
        let source = try String(contentsOf: sourceURL)
        let userCopyFields = [
            "title:",
            "reason:",
            "missingReason:",
            "caveat:",
            "measuredBetween:",
            "positiveMeans:",
            "negativeMeans:",
        ]
        let userVisible = source
            .split(separator: "\n")
            .filter { line in userCopyFields.contains { line.contains($0) } }
            .joined(separator: "\n")

        XCTAssertFalse(userVisible.contains("STATUS.md"))
        XCTAssertFalse(userVisible.contains("null model"))
        XCTAssertFalse(userVisible.contains("degenerate"))
        XCTAssertFalse(userVisible.contains("collinear"))
    }
}
