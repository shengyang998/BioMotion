import Foundation

/// One detected foot-ground contact.
///
/// # Duration comes from the CLOCK, never from counting array slots
///
/// `lastIndex − firstIndex + 1` counts SURVIVING SAMPLES. Vision loses the
/// person on about 7 % of frames (STATUS Finding 6: 22 of 309 at 30 fps), and
/// every lost frame inside a contact removes one slot from that count — so a
/// duration computed as `samples · dt` silently subtracts 33 ms from that leg's
/// contact and manufactures a left/right asymmetry out of a decoder artefact.
///
/// Measured on the pinned fixtures: `video_013`'s second left contact counts 4
/// samples (133.3 ms) and spans 200.0 ms on the clock — a whole frame is missing
/// from inside it. Monte Carlo over the two clean clips at the measured 7.1 %
/// drop rate, 400 trials: counting slots fabricates up to **23.4 %**
/// (`video_012`, truth 2.9 %) and **29.1 %** (`video_015`, truth 0.5 %) of
/// left/right asymmetry.
///
/// `seconds` is therefore `lastStanceSample − touchdown + dt`. That fixes every
/// INTERIOR hole exactly. It does NOT fix a hole at an EDGE — losing the first
/// or last sample of a contact moves the retained edge inward by a real
/// sampling interval, and the same Monte Carlo still reaches 19.4 % / 17.0 %.
/// Only refusing the contact does, which is why `droppedSamplesInside` and
/// `droppedSamplesAtEdges` are recorded here and refused in `GaitAnalysis`.
struct StanceInterval: Equatable {
    let side: GaitSide
    /// Indices into `GaitSignal.timestamps`, inclusive.
    let firstIndex: Int
    let lastIndex: Int
    let touchdown: TimeInterval
    /// The timestamp of the LAST stance frame. The foot leaves the ground
    /// somewhere in the following sampling interval, which is why
    /// `GaitReport.measuredFlightTime` adds one `dt` before measuring the gap.
    let lastStanceSample: TimeInterval
    /// The clock time of every sample in this contact, ascending. The plan is
    /// laid on THESE instants rather than on `touchdown + k·dt`, so a hole in
    /// the decode cannot push the frames after it out of the plan's ±dt/2 match
    /// window and have them solved as flight while the foot is still planted.
    let sampleTimestamps: [TimeInterval]
    /// Contact duration, seconds, read off the clock. See the type's note.
    let seconds: TimeInterval
    /// Decoder slots the video lost strictly INSIDE this contact.
    let droppedSamplesInside: Int
    /// Decoder slots lost across either edge — between the sample before
    /// touchdown and touchdown, or between the last stance sample and the one
    /// after it. A contact with one of these has an edge located to ±(gap + ½)
    /// sampling intervals rather than ±½, so its duration is not resolved to
    /// what `GaitResolution` publishes.
    let droppedSamplesAtEdges: Int

    /// Surviving samples, inclusive of both ends. This is what a filter window
    /// has to fit inside; it is NOT a duration. See `seconds`.
    var samples: Int { lastIndex - firstIndex + 1 }
    /// True when the clip's decoder handed over every slot this contact spans.
    var samplingIsComplete: Bool { droppedSamplesInside == 0 && droppedSamplesAtEdges == 0 }
}

/// Stance detection by the plateau criterion described in `GaitSignal`.
///
/// # The shape of the algorithm, and why each piece is where it is
///
/// 1. **Stride period** by normalised autocorrelation, taking the FIRST peak
///    within 80 % of the best one. Taking the global maximum instead is an
///    octave trap and it was measured doing real damage: on `video_012` three
///    of twelve window offsets locked onto 36 frames instead of 18, which
///    doubled the plateau's frame budget, dropped the level from 3.8 to 1.5 m/s
///    and turned every 5-frame contact into an 8-frame one. That single defect
///    was the whole of the window-shift failure.
/// 2. **One peak per cycle per foot**, by greedy non-maximum suppression with a
///    guard of 0.6 stride. Anchoring each contact to its own peak is what makes
///    the answer independent of what else is in the analysis window.
/// 3. **Per-cycle plateau statistics with a FIXED-SIZE support** — the top
///    `stanceFrameBudget` velocities of that cycle. Fixed size is the point: a
///    statistic computed over "the top 30 % of the window" changes when the
///    window's composition changes, and that is a second mechanism by which a
///    window shift moves a contact. Measured on the pooled-quantile variant:
///    11 of 11 contacts on `video_012` moved by up to 3 frames. With per-cycle
///    support, 0 of 11 move.
/// 4. **One level for both feet**, `V − k·σ`, where `V` and `σ` are medians over
///    all cycles of BOTH feet. A shared level is a deliberate protection for the
///    product's actual claim: any error in the level moves both feet's contacts
///    in the same direction, so it largely cancels in the left/right RATIO,
///    which is what a correction app reports.
///
/// # The constants, and what they are worth
///
/// `bandInStandardDeviations = 2.5`. The scatter of the velocity WITHIN a
/// plateau is the measurement noise (the true value there is constant), so a
/// 2.5σ band keeps 98.8 % of genuinely planted frames under a Gaussian while
/// the ramp frames sit 4-8σ out. Measured sensitivity, contact time in ms,
/// left/right:
///
///     k       1.5       2.0       2.5       3.0       3.5
///     012   122/133   167/152   167/162   167/171   189/190
///     013   117/105   139/110   150/148   172/162   194/190
///     015   156/160   189/187   206/207   222/227   244/247
///
/// so a unit of `k` is worth about one frame. That dependence is real and is not
/// argued away: it is why the resolution this module publishes is a floor on
/// what may be claimed, and why the LEFT/RIGHT RATIO (spread 0.18, 0.25, 0.04
/// over that whole sweep) is the quantity the product is allowed to use.
///
/// `assumedDutyFactor = 0.30` sets how many of a cycle's frames are treated as
/// plateau when estimating its level. It is a running-gait prior, and it is the
/// STRONGER of the two constants: `GaitReport.stanceFrameBudget` and
/// `GaitReport.measuredStanceFrames` agree on all three pinned clips (5/5, 5/5,
/// 6/6), and a clip where they disagree is refused as
/// `.stanceBudgetInconsistent` — but that check was measured and it is WEAK. On
/// `video_012`, 0.20, 0.25, 0.30 and 0.35 are ALL self-consistent, at 100, 167,
/// 167 and 200 ms of contact; only 0.40 inflates its own support and fails. So
/// the refusal catches the runaway direction and nothing else, and the prior is
/// worth about ±1 frame of absolute contact time that this data cannot pin.
/// Measured in `GaitConstantSensitivityTests`.
enum StanceDetector {

    static let bandInStandardDeviations = 2.5
    static let assumedDutyFactor = 0.30
    /// Plausible running stride periods. 0.30 s is faster than any human
    /// cadence (400 steps/min); 1.40 s is slower than a walk.
    static let minimumStridePeriodSeconds = 0.30
    static let maximumStridePeriodSeconds = 1.40
    /// A candidate period must correlate at least this well relative to the best
    /// lag before it is accepted as the fundamental.
    static let octaveAcceptanceFraction = 0.8

    struct Result {
        let stridePeriodFrames: Int
        let stridePeriodCorrelation: Double
        let stanceFrameBudget: Int
        /// The plateau level: the runner's speed, m/s, measured not assumed.
        let plateauSpeed: Double
        /// Scatter within the plateau — the velocity measurement noise, m/s.
        let plateauScatter: Double
        /// `plateauSpeed − k·plateauScatter`.
        let level: Double
        let stance: Bilateral<[StanceInterval]>
        /// Runs that touch an end of the analysis window. Their duration is
        /// unknowable — the contact started before the window opened or ends
        /// after it closes — so they are DROPPED, not trimmed to the window.
        /// An earlier implementation counted one as a touchdown and that single
        /// invalid onset is what failed a whole clip.
        let edgeClipped: Bilateral<[StanceInterval]>
    }

    /// - Parameters:
    ///   - band: width of the plateau band in measured standard deviations.
    ///   - duty: fraction of a cycle treated as plateau when estimating the
    ///     level.
    ///
    /// Both constants are arguments rather than literals for one reason: their
    /// sensitivity is asserted in `GaitConstantSensitivityTests`, so the tables
    /// in this file's documentation are re-measured by the test suite instead of
    /// being a claim about a script that no longer exists.
    static func detect(_ signal: GaitSignal,
                       band: Double = bandInStandardDeviations,
                       duty: Double = assumedDutyFactor) -> Result {
        let period = stridePeriod(signal)
        let budget = max(3, Int((duty * Double(period.frames)).rounded()))
        let half = period.frames / 2

        var peaks = Bilateral<[Int]>(left: [], right: [])
        for side in GaitSide.allCases {
            peaks[side] = peakIndices(signal.plateauVelocity[side], guardFrames: Int((0.6 * Double(period.frames)).rounded()))
        }

        var speeds: [Double] = []
        var scatters: [Double] = []
        for side in GaitSide.allCases {
            let w = signal.plateauVelocity[side]
            for i in peaks[side] {
                let lo = max(0, i - half), hi = min(w.count - 1, i + half)
                let top = Array(w[lo...hi].compactMap { $0 }.sorted().suffix(budget))
                guard top.count >= 3 else { continue }
                speeds.append(median(top))
                scatters.append(medianAbsoluteDeviation(top))
            }
        }
        let speed = median(speeds)
        let scatter = median(scatters)
        let level = speed - band * scatter

        var stance = Bilateral<[StanceInterval]>(left: [], right: [])
        var clipped = Bilateral<[StanceInterval]>(left: [], right: [])
        let dt = signal.sampleInterval
        for side in GaitSide.allCases {
            let w = signal.plateauVelocity[side]
            for i in peaks[side] {
                guard let here = w[i], here >= level else { continue }
                var a = i, b = i
                while a - 1 >= 0, let v = w[a - 1], v >= level { a -= 1 }
                while b + 1 < w.count, let v = w[b + 1], v >= level { b += 1 }
                // Sampling integrity, from the DECODER SLOTS rather than from
                // array positions: consecutive array entries whose slot numbers
                // differ by more than one have a hole between them.
                let slots = signal.frameNumbers
                let inside = (slots[b] - slots[a]) - (b - a)
                var atEdges = 0
                if a > 0 { atEdges += (slots[a] - slots[a - 1]) - 1 }
                if b + 1 < slots.count { atEdges += (slots[b + 1] - slots[b]) - 1 }
                let interval = StanceInterval(side: side, firstIndex: a, lastIndex: b,
                                              touchdown: signal.timestamps[a],
                                              lastStanceSample: signal.timestamps[b],
                                              sampleTimestamps: Array(signal.timestamps[a...b]),
                                              seconds: signal.timestamps[b] - signal.timestamps[a] + dt,
                                              droppedSamplesInside: max(0, inside),
                                              droppedSamplesAtEdges: max(0, atEdges))
                // `<= 1` and `>= count − 2`: index 0 and index count−1 carry no
                // velocity at all (a centred difference has no neighbour there),
                // so a run reaching index 1 or count−2 is already against the
                // wall of what the window can see.
                if a <= 1 || b >= w.count - 2 {
                    clipped[side].append(interval)
                } else {
                    stance[side].append(interval)
                }
            }
            stance[side].sort { $0.firstIndex < $1.firstIndex }
            clipped[side].sort { $0.firstIndex < $1.firstIndex }
        }

        return Result(stridePeriodFrames: period.frames,
                      stridePeriodCorrelation: period.correlation,
                      stanceFrameBudget: budget,
                      plateauSpeed: speed,
                      plateauScatter: scatter,
                      level: level,
                      stance: stance,
                      edgeClipped: clipped)
    }

    // MARK: - Period

    struct Period { let frames: Int; let correlation: Double; let best: Double }

    /// Normalised autocorrelation of the plateau-velocity signal, averaged over
    /// both feet (they share one cadence, so averaging halves the noise without
    /// changing the period), scanned for its FIRST acceptable peak.
    static func stridePeriod(_ signal: GaitSignal) -> Period {
        let dt = signal.sampleInterval
        let minLag = max(2, Int((minimumStridePeriodSeconds / dt).rounded()))
        let maxLag = Int((maximumStridePeriodSeconds / dt).rounded())
        guard maxLag > minLag else { return Period(frames: minLag, correlation: 0, best: 0) }

        var accumulated = [Double](repeating: 0, count: maxLag - minLag + 1)
        for side in GaitSide.allCases {
            let raw = signal.plateauVelocity[side].compactMap { $0 }
            guard raw.count > 4 else { continue }
            let m = mean(raw)
            let x = raw.map { $0 - m }
            for (k, lag) in (minLag...maxLag).enumerated() {
                guard lag < x.count - 3 else { continue }
                var num = 0.0, da = 0.0, db = 0.0
                for i in 0..<(x.count - lag) {
                    num += x[i] * x[i + lag]
                    da += x[i] * x[i]
                    db += x[i + lag] * x[i + lag]
                }
                let den = (da * db).squareRoot()
                accumulated[k] += den > 0 ? num / den : 0
            }
        }
        let r = accumulated.map { $0 / Double(GaitSide.allCases.count) }
        let best = r.max() ?? 0
        for k in 1..<(r.count - 1)
        where r[k] >= r[k - 1] && r[k] >= r[k + 1] && r[k] >= octaveAcceptanceFraction * best {
            return Period(frames: minLag + k, correlation: r[k], best: best)
        }
        let k = r.firstIndex(of: best) ?? 0
        return Period(frames: minLag + k, correlation: best, best: best)
    }

    // MARK: - Peaks

    /// One index per cycle: repeatedly take the largest unclaimed sample and
    /// suppress its neighbourhood. Stops at the signal's median, below which
    /// nothing can be a plateau.
    static func peakIndices(_ w: [Double?], guardFrames: Int) -> [Int] {
        let finite = w.compactMap { $0 }
        guard !finite.isEmpty else { return [] }
        let floorValue = median(finite)
        var used = [Bool](repeating: false, count: w.count)
        var out: [Int] = []
        while true {
            var bestIndex = -1
            var bestValue = -Double.infinity
            for i in w.indices where !used[i] {
                if let v = w[i], v > bestValue { bestValue = v; bestIndex = i }
            }
            guard bestIndex >= 0, bestValue > floorValue else { break }
            out.append(bestIndex)
            for j in max(0, bestIndex - guardFrames)...min(w.count - 1, bestIndex + guardFrames) {
                used[j] = true
            }
        }
        return out.sorted()
    }
}
