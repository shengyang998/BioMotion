import Foundation

/// The one horizontal signal every gait measurement in this module is read from,
/// plus the sampling facts a consumer needs in order to know what it may claim.
///
/// # Why this signal and not foot height
///
/// The criterion this module replaced was "the foot is down when its height sits
/// within `frac` of the ankle's vertical range". That has no physical meaning:
/// `frac` is a fraction of a range that is itself a property of the runner's
/// swing, so the level moves with the subject. Measured consequence on the
/// pinned clips: at `frac = 0.08` `video_012` reads the LEFT contact 24 % SHORTER
/// than the right, and at `frac = 0.25` it reads it 7 % LONGER. **The sign of the
/// asymmetry flips with the constant**, so the product's headline claim
/// ("your left side loads longer than your right") was a function of an
/// arbitrary number. That criterion must not ship, and nothing here reads foot
/// height as a detector.
///
/// # What replaces it
///
/// A foot in contact with the ground is at rest **in the ground frame**. The
/// reconstruction is pelvis-pinned, so the ground is not directly observable —
/// but a foot at rest on the ground moves backwards *relative to the pelvis* at
/// exactly the runner's speed, and it is the only part of the cycle that does so
/// at a constant rate. So stance is the interval where
///
///     w(t) = forwardSign · d/dt [ (foot − pelvis) · x̂ ]
///
/// sits on its plateau, and the plateau level **is** the running speed, measured
/// from the signal rather than assumed. Measured on the three pinned clips:
/// 5.80, 5.01 and 4.11 m/s — 15-21 km/h, which is the right order for the
/// footage.
///
/// # Why the horizontal axis only
///
/// `x̂` is image-right in `MHRRetarget`'s frame. `z` is depth, and STATUS records
/// 3.1 g of pure acceleration noise on the root's depth channel at 30 fps; this
/// file never reads `z`. `y` is vertical, and the vertical channel is dominated
/// by the pelvis's own bounce, which is exactly what the pelvis-pinning removed.
///
/// # Everything is pelvis-RELATIVE, which is what makes a tracking shot legal
///
/// Subtracting the pelvis cancels any rigid translation of the whole
/// reconstruction, so a camera that pans with the runner cannot forge or hide a
/// contact. It also means this module never needs `cam_t`, and never needs the
/// root translation to have been composed in.
struct GaitSignal {

    /// ARKit joint ids, as `MHRRetarget.table` emits them.
    enum JointID {
        static let pelvis = "hips_joint"
        static func ankle(_ side: GaitSide) -> String { "\(side.rawValue)_foot_joint" }
        static func toe(_ side: GaitSide) -> String { "\(side.rawValue)_toes_joint" }
    }

    enum Failure: Error, Equatable, CustomStringConvertible {
        case tooFewFrames(count: Int, needed: Int)
        case missingJoint(String, frameIndex: Int)
        case timestampsNotIncreasing(frameIndex: Int)
        case sampleIntervalNotPositive
        /// The toe and the ankle project onto the same point along `x̂`, so the
        /// foot's forward axis — and with it the sign of the whole signal — is
        /// unresolved. Happens on a subject walking straight at the camera.
        case forwardDirectionUnresolved(separationMeters: Double)

        var description: String {
            switch self {
            case .tooFewFrames(let count, let needed):
                return "gait needs at least \(needed) frames, got \(count)"
            case .missingJoint(let id, let index):
                return "frame \(index) has no joint `\(id)`"
            case .timestampsNotIncreasing(let index):
                return "frame \(index) does not advance the clock"
            case .sampleIntervalNotPositive:
                return "the median sampling interval is not positive"
            case .forwardDirectionUnresolved(let sep):
                return "toe and ankle are only \(sep) m apart along the horizontal axis; "
                     + "the running direction cannot be read from foot geometry"
            }
        }
    }

    /// A foot is 0.15-0.25 m long, so a clip filmed anywhere near side-on
    /// separates toe from ankle by more than this along `x̂`. Measured on the
    /// pinned clips: 0.0415, 0.0280 and 0.0276 m (summed over both feet, i.e.
    /// ~0.014-0.021 m per foot — the toe-ankle vector is mostly VERTICAL in
    /// these frames because the model returns a toe joint close under the
    /// ankle). The floor is set an order of magnitude below the smallest
    /// measured value, so it rejects only a genuinely ambiguous view.
    static let minimumForwardSeparationMeters = 0.002

    /// Enough frames to hold two full strides at the slowest plausible cadence.
    static let minimumFrames = 24

    let timestamps: [TimeInterval]
    /// Decoder slots, so a clip that lost frames shows gaps rather than a
    /// silently compressed clock.
    let frameNumbers: [Int]
    /// Median adjacent timestamp difference, seconds.
    let sampleInterval: Double
    /// `+1` if the runner travels along `+x̂`, `−1` otherwise. Read from foot
    /// GEOMETRY (the toe is anterior of the ankle), never from the velocity
    /// signal it is used to interpret — so the sign is an independent input and
    /// the two can be cross-checked. Measured: `video_012` and `video_013` run
    /// along `−x̂`, `video_015` along `+x̂`.
    let forwardSign: Double
    /// `|Σ (toe − ankle) · x̂|`, the evidence behind `forwardSign`.
    let forwardSeparationMeters: Double
    /// Pelvis-relative horizontal foot velocity, sign-corrected so stance is the
    /// POSITIVE plateau. `nil` at the two ends, where a centred difference has
    /// no neighbour.
    let plateauVelocity: Bilateral<[Double?]>
    /// Gaps in `frameNumbers`. `video_013` has two (3 missing slots) because
    /// Vision found no person there.
    let droppedFrameCount: Int
    /// Every adjacent gap, in whole sampling intervals.
    let maximumGapInFrames: Int

    var frameCount: Int { timestamps.count }
    var framesPerSecond: Double { 1.0 / sampleInterval }

    // MARK: - Construction

    static func build(frames: [BodyFrame]) throws -> GaitSignal {
        guard frames.count >= minimumFrames else {
            throw Failure.tooFewFrames(count: frames.count, needed: minimumFrames)
        }
        let timestamps = frames.map(\.timestamp)
        for i in 1..<timestamps.count where timestamps[i] <= timestamps[i - 1] {
            throw Failure.timestampsNotIncreasing(frameIndex: i)
        }
        let deltas = (1..<timestamps.count).map { timestamps[$0] - timestamps[$0 - 1] }
        let dt = median(deltas)
        guard dt > 0 else { throw Failure.sampleIntervalNotPositive }

        // Positions, by joint id, in the order the caller handed them over.
        func position(_ id: String, _ index: Int) throws -> SIMD3<Float> {
            guard let joint = frames[index].joints.first(where: { $0.id == id }) else {
                throw Failure.missingJoint(id, frameIndex: index)
            }
            return joint.worldPosition
        }

        var relativeAnkleX = Bilateral<[Double]>(left: [], right: [])
        var toeMinusAnkle = 0.0
        for side in GaitSide.allCases {
            var xs: [Double] = []
            xs.reserveCapacity(frames.count)
            for i in frames.indices {
                let pelvis = try position(JointID.pelvis, i)
                let ankle = try position(JointID.ankle(side), i)
                let toe = try position(JointID.toe(side), i)
                xs.append(Double(ankle.x) - Double(pelvis.x))
                toeMinusAnkle += Double(toe.x) - Double(ankle.x)
            }
            relativeAnkleX[side] = xs
        }
        toeMinusAnkle /= Double(frames.count)   // per-frame sum over both feet

        guard abs(toeMinusAnkle) >= minimumForwardSeparationMeters else {
            throw Failure.forwardDirectionUnresolved(separationMeters: abs(toeMinusAnkle))
        }
        // The toe leads, so `toeMinusAnkle` points the way the runner is going.
        // A planted foot then travels the OTHER way relative to the pelvis,
        // which is why the sign is negated here: it makes stance the positive
        // plateau for every clip regardless of which way the subject ran.
        let forwardSign = toeMinusAnkle > 0 ? -1.0 : 1.0

        var velocity = Bilateral<[Double?]>(left: [], right: [])
        for side in GaitSide.allCases {
            let x = relativeAnkleX[side]
            var v = [Double?](repeating: nil, count: x.count)
            // Centred difference over ±1 sample. It spans 2·dt = 67 ms at
            // 30 fps, which fits inside the SHORTEST contact measured here
            // (133 ms) — unlike the engine's 9-tap Savitzky-Golay window, which
            // spans 267 ms and therefore straddles a touchdown or toe-off on
            // every stance frame. See `GaitReport.filterTapsThatFitOneContact`.
            for i in 1..<(x.count - 1) {
                let span = timestamps[i + 1] - timestamps[i - 1]
                guard span > 0 else { continue }
                v[i] = forwardSign * (x[i + 1] - x[i - 1]) / span
            }
            velocity[side] = v
        }

        var dropped = 0
        var maxGap = 1
        let numbers = frames.map(\.frameNumber)
        for i in 1..<numbers.count {
            let gap = numbers[i] - numbers[i - 1]
            if gap > 1 { dropped += gap - 1 }
            maxGap = max(maxGap, gap)
        }

        return GaitSignal(timestamps: timestamps,
                          frameNumbers: numbers,
                          sampleInterval: dt,
                          forwardSign: forwardSign,
                          forwardSeparationMeters: abs(toeMinusAnkle),
                          plateauVelocity: velocity,
                          droppedFrameCount: dropped,
                          maximumGapInFrames: maxGap)
    }
}

// MARK: - Small shared vocabulary

enum GaitSide: String, CaseIterable, Hashable {
    case left, right
}

/// One value per foot. Cheaper to read than a dictionary and it cannot be
/// missing a side.
struct Bilateral<T> {
    var left: T
    var right: T

    subscript(side: GaitSide) -> T {
        get { side == .left ? left : right }
        set { if side == .left { left = newValue } else { right = newValue } }
    }

    func map<U>(_ transform: (T) throws -> U) rethrows -> Bilateral<U> {
        Bilateral<U>(left: try transform(left), right: try transform(right))
    }
}

extension Bilateral: Equatable where T: Equatable {}

// MARK: - Order statistics

/// Median with the usual even-count convention (mean of the two middle values).
func median(_ values: [Double]) -> Double {
    guard !values.isEmpty else { return .nan }
    let s = values.sorted()
    let mid = s.count / 2
    return s.count % 2 == 1 ? s[mid] : 0.5 * (s[mid - 1] + s[mid])
}

/// Median absolute deviation, scaled to a Gaussian standard deviation.
func medianAbsoluteDeviation(_ values: [Double]) -> Double {
    guard !values.isEmpty else { return .nan }
    let m = median(values)
    return 1.4826 * median(values.map { abs($0 - m) })
}

func mean(_ values: [Double]) -> Double {
    values.isEmpty ? .nan : values.reduce(0, +) / Double(values.count)
}

/// Population standard deviation (divides by N, matching the sample the
/// coefficient of variation is reported over — these are all the strides there
/// are, not a draw from a larger set).
func standardDeviation(_ values: [Double]) -> Double {
    guard values.count > 1 else { return 0 }
    let m = mean(values)
    return (values.reduce(0) { $0 + ($1 - m) * ($1 - m) } / Double(values.count)).squareRoot()
}

/// sd / mean, the dimensionless scatter this module reports everywhere.
func coefficientOfVariation(_ values: [Double]) -> Double {
    let m = mean(values)
    guard m != 0, m.isFinite else { return .nan }
    return standardDeviation(values) / abs(m)
}
