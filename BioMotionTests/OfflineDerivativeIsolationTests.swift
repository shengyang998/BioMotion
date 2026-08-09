import XCTest
@testable import BioMotion

/// The offline derivative filter may see only uninterrupted decoder slots.
/// These tests exercise the pure plan so a Core ML fallback, failed decode, or
/// failed pose cannot be hidden by the compact array of surviving frames.
final class OfflineDerivativeIsolationTests: XCTestCase {

    func testTrustedFramesSplitOnDecoderSlotGaps() {
        let plans = OfflineTemporalPolicy.segmentPlans(
            frameNumbers: [0, 1, 4, 5, 7],
            firstRequestedFrameNumber: 0,
            lastRequestedFrameNumber: 7)

        XCTAssertEqual(plans.map(\.frameIndices), [0..<2, 2..<4, 4..<5])
        XCTAssertEqual(plans.map(\.resetsRealtimeStateBefore), [false, true, true])
        XCTAssertTrue(plans[0].padsHead)
        XCTAssertFalse(plans[0].padsTail)
        XCTAssertFalse(plans[1].padsHead)
        XCTAssertFalse(plans[1].padsTail)
        XCTAssertFalse(plans[2].padsHead)
        XCTAssertTrue(plans[2].padsTail)
    }

    func testOnlyTrueClipEndpointsReceivePadding() {
        let leadingGap = OfflineTemporalPolicy.segmentPlans(
            frameNumbers: [1, 2],
            firstRequestedFrameNumber: 0,
            lastRequestedFrameNumber: 2)
        XCTAssertFalse(leadingGap[0].padsHead,
                       "a known missing first slot must not be invented as a held pose")
        XCTAssertTrue(leadingGap[0].padsTail)

        let trailingGap = OfflineTemporalPolicy.segmentPlans(
            frameNumbers: [0, 1],
            firstRequestedFrameNumber: 0,
            lastRequestedFrameNumber: 2)
        XCTAssertTrue(trailingGap[0].padsHead)
        XCTAssertFalse(trailingGap[0].padsTail,
                       "a trailing fallback/failure must clear the prior endpoint")

        let middleGap = OfflineTemporalPolicy.segmentPlans(
            frameNumbers: [0, 1, 3, 4],
            firstRequestedFrameNumber: 0,
            lastRequestedFrameNumber: 4)
        XCTAssertEqual(middleGap.count, 2)
        XCTAssertTrue(middleGap[0].padsHead)
        XCTAssertFalse(middleGap[0].padsTail)
        XCTAssertFalse(middleGap[1].padsHead)
        XCTAssertTrue(middleGap[1].padsTail)

        let photo = OfflineTemporalPolicy.segmentPlans(
            frameNumbers: [0],
            firstRequestedFrameNumber: 0,
            lastRequestedFrameNumber: 0)
        XCTAssertEqual(photo.count, 1)
        XCTAssertTrue(photo[0].padsHead)
        XCTAssertTrue(photo[0].padsTail)
    }
}
