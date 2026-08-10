import Foundation

enum TRCExporterError: LocalizedError, Equatable {
    case inconsistentMarkerNames(jointID: String, names: [String])
    case duplicateMarkerName(name: String, jointIDs: [String])
    case emptyMarkerName(jointID: String)

    var errorDescription: String? {
        switch self {
        case let .inconsistentMarkerNames(jointID, names):
            return "Joint \(jointID) changes OpenSim marker semantics across frames: \(names.joined(separator: ", "))."
        case let .duplicateMarkerName(name, jointIDs):
            return "OpenSim marker \(name) is assigned to multiple joints: \(jointIDs.joined(separator: ", "))."
        case let .emptyMarkerName(jointID):
            return "Joint \(jointID) has an empty OpenSim marker name."
        }
    }
}

/// Exports recorded BodyFrames to OpenSim .trc format.
/// TRC (Track Row Column) is a tab-delimited format storing 3D marker positions over time.
struct TRCExporter {
    let frames: [BodyFrame]
    let markerMappings: [JointMapping.Mapping]

    init(frames: [BodyFrame], markerMappings: [JointMapping.Mapping] = JointMapping.primary) {
        self.frames = frames
        self.markerMappings = markerMappings
    }

    /// Generates a .trc file string compatible with OpenSim.
    func generate() throws -> String {
        guard let firstFrame = frames.first else { return "" }

        let numFrames = frames.count
        let numMarkers = markerMappings.count
        let dataRate = calculateDataRate()
        let markerNames = try resolvedMarkerNames()

        var lines: [String] = []

        // Line 1: PathFileType header
        lines.append("PathFileType\t4\t(X/Y/Z)\tBioMotion_capture.trc")

        // Line 2: Metadata keys
        lines.append("DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames")

        // Line 3: Metadata values
        lines.append(String(format: "%.2f\t%.2f\t%d\t%d\tm\t%.2f\t1\t%d",
                            dataRate, dataRate, numFrames, numMarkers, dataRate, numFrames))

        // Line 4: Marker names (each followed by two empty tabs for Y/Z columns)
        var markerLine = "Frame#\tTime"
        for markerName in markerNames {
            markerLine += "\t\(markerName)\t\t"
        }
        lines.append(markerLine)

        // Line 5: Component labels (X/Y/Z per marker)
        var componentLine = "\t"
        for i in 1...numMarkers {
            componentLine += "\tX\(i)\tY\(i)\tZ\(i)"
        }
        lines.append(componentLine)

        // Line 6: Blank line
        lines.append("")

        // Data rows
        let startTime = firstFrame.timestamp
        for frame in frames {
            let time = frame.timestamp - startTime
            var row = "\(frame.frameNumber)\t\(String(format: "%.6f", time))"

            for mapping in markerMappings {
                if let joint = frame.joints.first(where: { $0.id == mapping.arkitName }),
                   joint.isTracked {
                    row += String(format: "\t%.6f\t%.6f\t%.6f",
                                  joint.worldPosition.x,
                                  joint.worldPosition.y,
                                  joint.worldPosition.z)
                } else {
                    // Missing marker: use NaN (OpenSim convention)
                    row += "\t\t\t"
                }
            }

            lines.append(row)
        }

        return lines.joined(separator: "\n")
    }

    /// Writes the .trc file to a temporary location and returns the URL.
    func export(filename: String = "BioMotion_capture") throws -> URL {
        let content = try generate()
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("\(filename).trc")
        try content.write(to: url, atomically: true, encoding: .utf8)
        return url
    }

    private func calculateDataRate() -> Double {
        guard frames.count > 1,
              let first = frames.first,
              let last = frames.last else { return 60.0 }
        let elapsed = last.timestamp - first.timestamp
        guard elapsed > 0 else { return 60.0 }
        return Double(frames.count - 1) / elapsed
    }

    /// Resolve one stable marker name per joint id before writing any bytes.
    /// A nil override is the live/default marker, so mixing nil/PELVIS with
    /// MHR_ROOT is a real provenance conflict rather than permission to relabel
    /// the earlier frames. Different ids may not collapse into one TRC column.
    private func resolvedMarkerNames() throws -> [String] {
        var namesByJointID: [String: String] = [:]

        for mapping in markerMappings {
            var observed = Set<String>()
            for frame in frames {
                guard let joint = frame.joints.first(where: { $0.id == mapping.arkitName }) else {
                    continue
                }
                let markerName = joint.opensimMarkerNameOverride ?? mapping.opensimName
                guard !markerName.isEmpty else {
                    throw TRCExporterError.emptyMarkerName(jointID: mapping.arkitName)
                }
                observed.insert(markerName)
            }

            let sorted = observed.sorted()
            guard sorted.count <= 1 else {
                throw TRCExporterError.inconsistentMarkerNames(
                    jointID: mapping.arkitName,
                    names: sorted
                )
            }
            namesByJointID[mapping.arkitName] = sorted.first ?? mapping.opensimName
        }

        var jointIDsByMarkerName: [String: [String]] = [:]
        for mapping in markerMappings {
            let markerName = namesByJointID[mapping.arkitName] ?? mapping.opensimName
            jointIDsByMarkerName[markerName, default: []].append(mapping.arkitName)
        }
        if let duplicate = jointIDsByMarkerName
            .filter({ $0.value.count > 1 })
            .sorted(by: { $0.key < $1.key })
            .first {
            throw TRCExporterError.duplicateMarkerName(
                name: duplicate.key,
                jointIDs: duplicate.value.sorted()
            )
        }

        return markerMappings.map { namesByJointID[$0.arkitName] ?? $0.opensimName }
    }
}
