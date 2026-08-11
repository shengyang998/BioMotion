import Foundation

@testable import BioMotion

/// Loads the pinned gait clip fixtures written by
/// `tools/gait_fixture/export_gait_fixture.py`.
///
/// Three 4-second windows of real running footage, as the five joints a gait
/// analysis needs (raw MHR root, both ankles, both toes), in the frame
/// `MHRRetarget.makeBodyFrame(camT: nil)` produces: metres, Y-up, source root
/// pinned at the model constant 0.92398697, unsmoothed.
///
/// # Nothing here force-unwraps a parsed value
///
/// The previous fixture trapped on its FIRST data line. Its generator wrote
/// numpy reprs (`np.float64(3.0)`) and its loader parsed them with
/// `Double(f[0])!`. A force-unwrap of `nil` is not a test failure — it is a
/// `SIGILL` inside the xctest process, so it takes every other test in the
/// target down with it and nothing downstream of the fixture is ever runnable.
///
/// So every field goes through a strict scanner and then a `guard let`, and a
/// malformed file produces a `LoadError` a test can assert on. The scanner is
/// deliberately stricter than `Double(_: String)`, which happily accepts
/// `"nan"`, `"inf"` and `"0x1p3"` — none of which is a metre coordinate, and
/// all of which would sail through a naive `guard let` into the gait maths.
enum GaitClipFixture {

    static let allIds = ["video_012", "video_013", "video_015"]

    private static let sourceMarkerByJointID = Dictionary(uniqueKeysWithValues:
        MHRRetarget.table.map { ($0.arkitJointId, $0.opensimMarker) })

    /// The format tag on line 1 of every fixture. Bumped by the generator when
    /// the column layout changes, so a stale fixture is refused instead of
    /// being read with the wrong column meanings.
    static let formatId = "biomotion-gait-clip-v1"

    struct Clip {
        /// `video_012` etc, as declared IN the file (not inferred from its name).
        let id: String
        /// ARKit joint ids, in the file's column order.
        let jointIds: [String]
        /// One entry per record. `frameNumber` is the decoder slot, so a clip
        /// that lost frames to a failed person detection skips numbers —
        /// `video_013` skips slots 25 and 30-31.
        let frames: [BodyFrame]
    }

    // MARK: - Errors

    enum LoadError: Error, Equatable, CustomStringConvertible {
        case fileNotFound(String)
        case fileUnreadable(name: String, reason: String)
        case notText(name: String)
        case notASCII(line: Int, byte: UInt8)
        case missingHeaderLine(expectedKey: String)
        case badHeaderKey(line: Int, expected: String, got: String)
        case unsupportedFormat(line: Int, got: String)
        case emptyHeaderValue(line: Int, key: String)
        case badFieldCount(line: Int, expected: Int, got: Int)
        case badInteger(line: Int, field: Int, text: String)
        case badDecimal(line: Int, field: Int, text: String)
        case notFinite(line: Int, field: Int, text: String)
        case frameCountMismatch(declared: Int, found: Int)
        case clipIdMismatch(requested: String, declared: String)

        var description: String {
            switch self {
            case .fileNotFound(let name):
                return "no fixture resource named \(name) in the test bundle"
            case .fileUnreadable(let name, let reason):
                return "fixture \(name) could not be read: \(reason)"
            case .notText(let name):
                return "fixture \(name) is not decodable text"
            case .notASCII(let line, let byte):
                return "line \(line): byte 0x\(String(byte, radix: 16)) is not printable ASCII"
            case .missingHeaderLine(let key):
                return "file ended before the `\(key)` header line"
            case .badHeaderKey(let line, let expected, let got):
                return "line \(line): expected header key `\(expected)`, got `\(got)`"
            case .unsupportedFormat(let line, let got):
                return "line \(line): format `\(got)` is not `\(GaitClipFixture.formatId)`"
            case .emptyHeaderValue(let line, let key):
                return "line \(line): header `\(key)` has no value"
            case .badFieldCount(let line, let expected, let got):
                return "line \(line): expected \(expected) fields, got \(got)"
            case .badInteger(let line, let field, let text):
                return "line \(line) field \(field): `\(text)` is not a plain decimal integer"
            case .badDecimal(let line, let field, let text):
                return "line \(line) field \(field): `\(text)` is not a plain decimal number"
            case .notFinite(let line, let field, let text):
                return "line \(line) field \(field): `\(text)` is not finite"
            case .frameCountMismatch(let declared, let found):
                return "header declares \(declared) frames, file holds \(found)"
            case .clipIdMismatch(let requested, let declared):
                return "asked for \(requested), file declares \(declared)"
            }
        }
    }

    // MARK: - Loading

    /// The fixtures are a FOLDER reference in the bundle, not loose files. A
    /// `.txt` copied flat to the root of the shallow `.xctest` bundle makes
    /// `codesign` treat it as an unsigned nested code object, which fails the
    /// build before a single test runs (measured, not guessed). See
    /// `project.yml`.
    static let resourceSubdirectory = "Fixtures"

    /// - Parameter bundle: the bundle holding the fixture resources. Tests pass
    ///   their own (`Bundle(for: type(of: self))`); there is no implicit
    ///   fallback, so a missing resource is a named error rather than a search
    ///   through whatever bundle happened to be `.main`.
    static func load(_ clipId: String, bundle: Bundle) throws -> Clip {
        let resource = "gait_\(clipId)"
        guard let url = bundle.url(forResource: resource, withExtension: "txt",
                                   subdirectory: resourceSubdirectory) else {
            throw LoadError.fileNotFound("\(resourceSubdirectory)/\(resource).txt")
        }
        let data: Data
        do {
            data = try Data(contentsOf: url)
        } catch {
            throw LoadError.fileUnreadable(name: resource, reason: "\(error)")
        }
        guard let text = String(data: data, encoding: .utf8) else {
            throw LoadError.notText(name: resource)
        }
        let clip = try parse(text)
        guard clip.id == clipId else {
            throw LoadError.clipIdMismatch(requested: clipId, declared: clip.id)
        }
        return clip
    }

    static func loadAll(bundle: Bundle) throws -> [Clip] {
        try allIds.map { try load($0, bundle: bundle) }
    }

    // MARK: - Parsing

    private static let headerKeys = ["format", "clip", "frames", "joints"]

    /// Grammar, checked line by line:
    ///
    ///     '#' comment | blank        anywhere, ignored
    ///     format biomotion-gait-clip-v1
    ///     clip   <id>
    ///     frames <count>
    ///     joints <id> <id> ...       these four, in this order, first
    ///     <slot> <t> (<x> <y> <z>)*  exactly `count` data lines
    ///
    /// Integers are `[0-9]+`, decimals are `-?[0-9]+.[0-9]+`, fields are
    /// separated by single spaces.
    static func parse(_ text: String) throws -> Clip {
        var clipId = ""
        var declaredFrames = 0
        var jointIds: [String] = []
        var headersSeen = 0
        var frames: [BodyFrame] = []
        var expectedFieldCount = 0

        for (index, rawLine) in text.split(separator: "\n", omittingEmptySubsequences: false).enumerated() {
            let lineNumber = index + 1
            var line = rawLine
            if line.hasSuffix("\r") { line = line.dropLast() }

            if let offending = firstNonASCIIByte(line) {
                throw LoadError.notASCII(line: lineNumber, byte: offending)
            }
            if line.isEmpty || line.hasPrefix("#") { continue }

            let fields = line.split(separator: " ", omittingEmptySubsequences: false)

            if headersSeen < headerKeys.count {
                let key = headerKeys[headersSeen]
                guard let got = fields.first, got == key else {
                    throw LoadError.badHeaderKey(line: lineNumber, expected: key,
                                                 got: String(fields.first ?? ""))
                }
                let values = fields.dropFirst().map(String.init)
                guard !values.isEmpty, !values.contains(where: { $0.isEmpty }) else {
                    throw LoadError.emptyHeaderValue(line: lineNumber, key: key)
                }
                switch key {
                case "format":
                    guard values.count == 1, values[0] == formatId else {
                        throw LoadError.unsupportedFormat(line: lineNumber,
                                                          got: values.joined(separator: " "))
                    }
                case "clip":
                    guard values.count == 1 else {
                        throw LoadError.badFieldCount(line: lineNumber, expected: 2,
                                                      got: fields.count)
                    }
                    clipId = values[0]
                case "frames":
                    guard values.count == 1 else {
                        throw LoadError.badFieldCount(line: lineNumber, expected: 2,
                                                      got: fields.count)
                    }
                    guard let count = strictInt(Substring(values[0])) else {
                        throw LoadError.badInteger(line: lineNumber, field: 1, text: values[0])
                    }
                    declaredFrames = count
                default:
                    jointIds = values
                    expectedFieldCount = 2 + 3 * jointIds.count
                    frames.reserveCapacity(declaredFrames)
                }
                headersSeen += 1
                continue
            }

            guard fields.count == expectedFieldCount else {
                throw LoadError.badFieldCount(line: lineNumber, expected: expectedFieldCount,
                                              got: fields.count)
            }
            guard let slot = strictInt(fields[0]) else {
                throw LoadError.badInteger(line: lineNumber, field: 0, text: String(fields[0]))
            }
            guard let timestamp = strictDecimal(fields[1]) else {
                throw LoadError.badDecimal(line: lineNumber, field: 1, text: String(fields[1]))
            }
            guard timestamp.isFinite else {
                throw LoadError.notFinite(line: lineNumber, field: 1, text: String(fields[1]))
            }

            var joints: [TrackedJoint] = []
            joints.reserveCapacity(jointIds.count)
            for (k, jointId) in jointIds.enumerated() {
                var xyz = SIMD3<Float>(repeating: 0)
                for axis in 0..<3 {
                    let field = 2 + 3 * k + axis
                    guard let value = strictDecimal(fields[field]) else {
                        throw LoadError.badDecimal(line: lineNumber, field: field,
                                                   text: String(fields[field]))
                    }
                    let narrowed = Float(value)
                    guard narrowed.isFinite else {
                        throw LoadError.notFinite(line: lineNumber, field: field,
                                                  text: String(fields[field]))
                    }
                    xyz[axis] = narrowed
                }
                joints.append(TrackedJoint(id: jointId, name: jointId,
                                           worldPosition: xyz, isTracked: true,
                                           opensimMarkerNameOverride:
                                               sourceMarkerByJointID[jointId]))
            }
            frames.append(BodyFrame(
                timestamp: timestamp,
                frameNumber: slot,
                joints: joints,
                dynamicsReference: .mhrRootRelative
            ))
        }

        guard headersSeen == headerKeys.count else {
            throw LoadError.missingHeaderLine(expectedKey: headerKeys[headersSeen])
        }
        guard frames.count == declaredFrames else {
            throw LoadError.frameCountMismatch(declared: declaredFrames, found: frames.count)
        }
        return Clip(id: clipId, jointIds: jointIds, frames: frames)
    }

    // MARK: - Strict scanners

    /// The grammar moved to `FixtureScanner` when a second generated fixture
    /// arrived, so the two cannot drift apart. These stay as the names the rest
    /// of this file already reads.
    private static func firstNonASCIIByte(_ s: Substring) -> UInt8? {
        FixtureScanner.firstNonASCIIByte(s)
    }

    private static func strictInt(_ s: Substring) -> Int? {
        FixtureScanner.strictInt(s)
    }

    private static func strictDecimal(_ s: Substring) -> Double? {
        FixtureScanner.strictDecimal(s)
    }
}
