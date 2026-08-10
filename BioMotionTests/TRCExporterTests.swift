import XCTest
@testable import BioMotion

final class TRCExporterTests: XCTestCase {

    private func makeJoints(tracked: Bool = true,
                            rootMarkerOverride: String? = nil) -> [TrackedJoint] {
        JointMapping.primary.map { mapping in
            TrackedJoint(
                id: mapping.arkitName,
                name: mapping.displayName,
                worldPosition: SIMD3<Float>(0.1, 0.9, 0.0),
                isTracked: tracked,
                opensimMarkerNameOverride: mapping.arkitName == "hips_joint"
                    ? rootMarkerOverride
                    : nil
            )
        }
    }

    private func makeFrames(count: Int, fps: Double = 60.0) -> [BodyFrame] {
        (0..<count).map { i in
            BodyFrame(
                timestamp: Double(i) / fps,
                frameNumber: i + 1,
                joints: makeJoints()
            )
        }
    }

    // MARK: - Header

    func testTRCHeaderLine1() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 3))
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")

        XCTAssertTrue(lines[0].hasPrefix("PathFileType\t4\t(X/Y/Z)"))
    }

    func testTRCHeaderLine2MetadataKeys() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 3))
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")

        let expectedKeys = "DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames"
        XCTAssertEqual(lines[1], expectedKeys)
    }

    func testTRCHeaderLine3Values() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 5))
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")
        let values = lines[2].components(separatedBy: "\t")

        // NumFrames
        XCTAssertEqual(values[2], "5")
        // NumMarkers
        XCTAssertEqual(values[3], "\(JointMapping.primary.count)")
        // Units
        XCTAssertEqual(values[4], "m")
    }

    func testTRCHeaderLine4MarkerNames() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 1))
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")

        XCTAssertTrue(lines[3].hasPrefix("Frame#\tTime"))
        // Should contain all OpenSim marker names
        for mapping in JointMapping.primary {
            XCTAssertTrue(lines[3].contains(mapping.opensimName),
                          "Header should contain marker \(mapping.opensimName)")
        }
    }

    func testSourceMarkerNamesAreConsistentOrExportFailsClosed() throws {
        let mhrFrame = BodyFrame(timestamp: 0, frameNumber: 1,
                                 joints: makeJoints(rootMarkerOverride: "MHR_ROOT"))
        let mhrOutput = try TRCExporter(frames: [mhrFrame]).generate()
        let header = mhrOutput.components(separatedBy: "\n")[3]
        XCTAssertTrue(header.contains("\tMHR_ROOT\t\t"))
        XCTAssertFalse(header.contains("\tPELVIS\t\t"),
                       "MHR coordinates must not be relabelled as the live pelvis origin")

        let liveFrame = BodyFrame(timestamp: 1, frameNumber: 2, joints: makeJoints())
        XCTAssertThrowsError(try TRCExporter(frames: [mhrFrame, liveFrame]).generate()) { error in
            XCTAssertEqual(error as? TRCExporterError,
                           .inconsistentMarkerNames(jointID: "hips_joint",
                                                    names: ["MHR_ROOT", "PELVIS"]))
        }

        var duplicated = makeJoints(rootMarkerOverride: "MHR_ROOT")
        let leftHipIndex = try XCTUnwrap(
            duplicated.firstIndex(where: { $0.id == "left_upLeg_joint" })
        )
        let leftHip = duplicated[leftHipIndex]
        duplicated[leftHipIndex] = TrackedJoint(
            id: leftHip.id,
            name: leftHip.name,
            worldPosition: leftHip.worldPosition,
            isTracked: leftHip.isTracked,
            opensimMarkerNameOverride: "MHR_ROOT"
        )
        let duplicateFrame = BodyFrame(timestamp: 0, frameNumber: 1, joints: duplicated)
        XCTAssertThrowsError(try TRCExporter(frames: [duplicateFrame]).generate()) { error in
            XCTAssertEqual(error as? TRCExporterError,
                           .duplicateMarkerName(name: "MHR_ROOT",
                                                jointIDs: ["hips_joint", "left_upLeg_joint"]))
        }
    }

    func testTRCHeaderLine5ComponentLabels() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 1))
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")

        // Line 5 should have X/Y/Z component labels
        XCTAssertTrue(lines[4].contains("X1"))
        XCTAssertTrue(lines[4].contains("Y1"))
        XCTAssertTrue(lines[4].contains("Z1"))
    }

    func testTRCHeaderLine6Blank() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 1))
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")

        XCTAssertEqual(lines[5], "", "Line 6 should be blank")
    }

    // MARK: - Data rows

    func testTRCDataRowCount() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 10))
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")

        // 6 header lines + 10 data rows
        XCTAssertEqual(lines.count, 16)
    }

    func testTRCDataRowFormat() throws {
        let joints = JointMapping.primary.enumerated().map { (i, mapping) in
            TrackedJoint(
                id: mapping.arkitName,
                name: mapping.displayName,
                worldPosition: SIMD3<Float>(Float(i) * 0.1, 1.0, 0.0),
                isTracked: true
            )
        }
        let frames = [BodyFrame(timestamp: 0.0, frameNumber: 1, joints: joints)]
        let exporter = TRCExporter(frames: frames)
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")
        let dataRow = lines[6] // First data row
        let columns = dataRow.components(separatedBy: "\t")

        // Frame number
        XCTAssertEqual(columns[0], "1")
        // Time
        XCTAssertEqual(columns[1], "0.000000")
        // First marker X (index 0 * 0.1 = 0.0)
        XCTAssertEqual(columns[2], "0.000000")
        // First marker Y
        XCTAssertEqual(columns[3], "1.000000")
    }

    func testTRCTimeStartsAtZero() throws {
        let frames = [
            BodyFrame(timestamp: 100.5, frameNumber: 1, joints: makeJoints()),
            BodyFrame(timestamp: 100.55, frameNumber: 2, joints: makeJoints()),
        ]
        let exporter = TRCExporter(frames: frames)
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")

        let row1Cols = lines[6].components(separatedBy: "\t")
        let row2Cols = lines[7].components(separatedBy: "\t")

        // Time should be relative to first frame
        XCTAssertEqual(row1Cols[1], "0.000000")
        XCTAssertTrue(row2Cols[1].hasPrefix("0.05"))
    }

    func testTRCUntrackedMarkersEmpty() throws {
        let joints = JointMapping.primary.map { mapping in
            TrackedJoint(
                id: mapping.arkitName,
                name: mapping.displayName,
                worldPosition: .zero,
                isTracked: false
            )
        }
        let frames = [BodyFrame(timestamp: 0.0, frameNumber: 1, joints: joints)]
        let exporter = TRCExporter(frames: frames)
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")
        let dataRow = lines[6]

        // After Frame# and Time, untracked markers should produce empty tab-separated fields
        let columns = dataRow.components(separatedBy: "\t")
        // Columns 2,3,4 should be empty (first untracked marker)
        XCTAssertEqual(columns[2], "")
        XCTAssertEqual(columns[3], "")
        XCTAssertEqual(columns[4], "")
    }

    // MARK: - Export

    func testTRCExportCreatesFile() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 3))
        let url = try exporter.export(filename: "test_export")

        XCTAssertTrue(FileManager.default.fileExists(atPath: url.path))
        XCTAssertTrue(url.lastPathComponent.hasSuffix(".trc"))

        // Clean up
        try? FileManager.default.removeItem(at: url)
    }

    func testTRCExportedFileIsReadable() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 2))
        let url = try exporter.export(filename: "test_readable")
        let content = try String(contentsOf: url, encoding: .utf8)

        XCTAssertTrue(content.hasPrefix("PathFileType"))
        XCTAssertTrue(content.contains("DataRate"))

        try? FileManager.default.removeItem(at: url)
    }

    func testPartialExportIncludesAReadableWarningFile() throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent("biomotion-export-disclosure-\(UUID().uuidString)",
                                   isDirectory: true)
        try FileManager.default.createDirectory(at: directory,
                                                withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let motURL = directory.appendingPathComponent("motion.mot")
        let prepared = try ExportDisclosure.prepareShareURLs(
            successfulURLs: [motURL],
            errors: ["TRC: marker source changed from MHR_ROOT to PELVIS"],
            directory: directory
        )

        XCTAssertEqual(prepared.first, motURL)
        XCTAssertEqual(prepared.count, 2,
                       "a partial export must carry its warning alongside successful files")
        let warningURL = try XCTUnwrap(prepared.last)
        XCTAssertEqual(warningURL.lastPathComponent, "BioMotion_export_warnings.txt")
        let warning = try String(contentsOf: warningURL, encoding: .utf8)
        XCTAssertTrue(warning.contains("Partial export"))
        XCTAssertTrue(warning.contains("TRC: marker source changed from MHR_ROOT to PELVIS"))

        let blocked = try ExportDisclosure.prepareShareURLs(
            successfulURLs: [motURL],
            errors: ["STO: unavailable — no validated foot-support mechanics"],
            hasValidatedFootContactSupport: false,
            directory: directory)
        let blockedWarning = try String(
            contentsOf: try XCTUnwrap(blocked.last), encoding: .utf8)
        XCTAssertTrue(blockedWarning.contains("permanently unavailable"), blockedWarning)
        XCTAssertTrue(blockedWarning.lowercased().contains("refilming"), blockedWarning)
        XCTAssertFalse(blockedWarning.contains("record for at least a few seconds"),
                       blockedWarning)
        XCTAssertFalse(blockedWarning.contains("model must load"), blockedWarning)
    }

    // MARK: - Edge cases

    func testTRCEmptyFrames() throws {
        let exporter = TRCExporter(frames: [])
        let output = try exporter.generate()
        XCTAssertEqual(output, "")
    }

    func testTRCSingleFrame() throws {
        let exporter = TRCExporter(frames: makeFrames(count: 1))
        let output = try exporter.generate()
        let lines = output.components(separatedBy: "\n")

        // 6 header lines + 1 data row
        XCTAssertEqual(lines.count, 7)
    }
}
