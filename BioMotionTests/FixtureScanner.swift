import Foundation

/// The one number grammar every generated fixture in this target is parsed
/// with.
///
/// # Why this is a shared file and not two private copies
///
/// A previous fixture trapped on its FIRST data line: its generator wrote numpy
/// reprs (`np.float64(3.0)`) and its loader used `Double(f[0])!`. A force-unwrap
/// of `nil` is a `SIGILL` inside the xctest process, so it takes the whole test
/// target down and nothing downstream of the fixture is runnable.
///
/// `GaitClipFixture` fixed that with a deliberately strict scanner. The moment a
/// SECOND fixture arrived, the choice was to copy that scanner or to share it —
/// and a copy is how one of the two ends up accepting `nan` again. Both fixtures
/// call these.
enum FixtureScanner {

    /// The first byte outside printable ASCII, or nil.
    static func firstNonASCIIByte(_ s: Substring) -> UInt8? {
        s.utf8.first { $0 < 0x20 || $0 > 0x7E }
    }

    /// `[0-9]+`, no sign, no leading zeros beyond `0` itself.
    static func strictInt(_ s: Substring) -> Int? {
        guard !s.isEmpty, s.utf8.allSatisfy({ $0 >= 0x30 && $0 <= 0x39 }) else { return nil }
        if s.count > 1 && s.hasPrefix("0") { return nil }
        return Int(s)
    }

    /// `-?[0-9]+.[0-9]+`, then `Double(_:)`, then finite.
    ///
    /// The explicit grammar is the point: `Double("nan")`, `Double("inf")` and
    /// `Double("0x1p3")` all succeed, and `Double("np.float64(3.0)")` is the
    /// exact string that killed the previous fixture's whole test target.
    static func strictDecimal(_ s: Substring) -> Double? {
        var body = s
        if body.hasPrefix("-") { body = body.dropFirst() }
        guard let dot = body.firstIndex(of: ".") else { return nil }
        let whole = body[body.startIndex..<dot]
        let fraction = body[body.index(after: dot)...]
        guard !whole.isEmpty, !fraction.isEmpty else { return nil }
        let digits: (Substring) -> Bool = { $0.utf8.allSatisfy { $0 >= 0x30 && $0 <= 0x39 } }
        guard digits(whole), digits(fraction) else { return nil }
        if whole.count > 1 && whole.hasPrefix("0") { return nil }
        guard let value = Double(s), value.isFinite else { return nil }
        return value
    }
}
