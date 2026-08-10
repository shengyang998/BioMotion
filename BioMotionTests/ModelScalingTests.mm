#import <XCTest/XCTest.h>

#import "NimbleBridge.h"
#import "NimbleBridge+Internal.h"

#include <cmath>
#include <memory>
#include <string>

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/math/MathTypes.hpp"

using namespace dart;

namespace {

struct ScaleMarkers {
    NSMutableArray<NSNumber *> *positions;
    NSMutableArray<NSString *> *names;
};

static ScaleMarkers markersFromLoadedModel(
    const std::shared_ptr<dynamics::Skeleton>& skeleton,
    double ratio) {
    ScaleMarkers result = {
        [NSMutableArray array],
        [NSMutableArray array],
    };

    auto bodyOrigin = [&](const char *bodyName) -> Eigen::Vector3s {
        dynamics::BodyNode *body = skeleton->getBodyNode(std::string(bodyName));
        XCTAssertTrue(body != nullptr, @"missing body %s", bodyName);
        if (body == nullptr) return Eigen::Vector3s::Zero();
        return body->getWorldTransform().translation();
    };

    const Eigen::Vector3s pelvis = bodyOrigin("pelvis");
    struct MarkerBody {
        const char *marker;
        const char *body;
    };
    static const MarkerBody markerBodies[] = {
        {"PELVIS", "pelvis"},
        {"LHJC", "femur_l"},
        {"RHJC", "femur_r"},
        {"LAJC", "talus_l"},
        {"RAJC", "talus_r"},
        {"LSJC", "humerus_l"},
        {"RSJC", "humerus_r"},
        {"LWJC", "hand_l"},
        {"RWJC", "hand_r"},
    };

    for (const MarkerBody& markerBody : markerBodies) {
        Eigen::Vector3s position = bodyOrigin(markerBody.body);
        position = pelvis + ratio * (position - pelvis);
        [result.names addObject:[NSString stringWithUTF8String:markerBody.marker]];
        [result.positions addObject:@(position.x())];
        [result.positions addObject:@(position.y())];
        [result.positions addObject:@(position.z())];
    }
    return result;
}

static void assertScalesEqual(
    const Eigen::VectorXs& actual,
    const Eigen::VectorXs& expected,
    NSString *context) {
    XCTAssertEqual(actual.size(), expected.size(), @"%@ size", context);
    if (actual.size() != expected.size()) return;
    Eigen::Index worstIndex = 0;
    const double worstDifference = (actual - expected).cwiseAbs().maxCoeff(&worstIndex);
    XCTAssertEqualWithAccuracy(worstDifference, 0.0, 1e-10,
                               @"%@ worst component %ld: actual %.12g expected %.12g",
                               context, (long)worstIndex,
                               actual(worstIndex), expected(worstIndex));
}

} // namespace

@interface ModelScalingTests : XCTestCase
@end

@implementation ModelScalingTests

- (NSString *)modelPath:(NSString *)name {
    NSString *path = [[NSBundle bundleForClass:[self class]] pathForResource:name
                                                                      ofType:@"osim"];
    XCTAssertNotNil(path, @"%@.osim must be present in the test bundle", name);
    return path;
}

- (void)testLoadedModelMeasurementsAreAnIdentityScale {
    NimbleBridge *bridge = [[NimbleBridge alloc] init];
    XCTAssertTrue([bridge loadModelFromPath:[self modelPath:@"Rajagopal2016"]]);
    std::shared_ptr<dynamics::Skeleton> skeleton = [bridge sharedSkeleton];
    XCTAssertTrue(skeleton != nullptr);
    if (skeleton == nullptr) return;

    const Eigen::VectorXs loadedDefaults = skeleton->getBodyScales();
    ScaleMarkers markers = markersFromLoadedModel(skeleton, 1.0);
    XCTAssertTrue([bridge scaleModelWithHeight:1.8
                               markerPositions:markers.positions
                                   markerNames:markers.names]);

    assertScalesEqual(skeleton->getBodyScales(), loadedDefaults,
                      @"model-native measurements must preserve loaded defaults");
}

- (void)testRepeatedScalingUsesTheLoadedBaselineInsteadOfCompounding {
    NimbleBridge *bridge = [[NimbleBridge alloc] init];
    XCTAssertTrue([bridge loadModelFromPath:[self modelPath:@"Rajagopal2016"]]);
    std::shared_ptr<dynamics::Skeleton> skeleton = [bridge sharedSkeleton];
    XCTAssertTrue(skeleton != nullptr);
    if (skeleton == nullptr) return;

    const double ratio = 1.12;
    const Eigen::VectorXs loadedDefaults = skeleton->getBodyScales();
    const Eigen::VectorXs expected = loadedDefaults * ratio;
    ScaleMarkers markers = markersFromLoadedModel(skeleton, ratio);

    XCTAssertTrue([bridge scaleModelWithHeight:1.8 * ratio
                               markerPositions:markers.positions
                                   markerNames:markers.names]);
    const Eigen::VectorXs first = skeleton->getBodyScales();
    assertScalesEqual(first, expected, @"first scale from loaded baseline");

    XCTAssertTrue([bridge scaleModelWithHeight:1.8 * ratio
                               markerPositions:markers.positions
                                   markerNames:markers.names]);
    assertScalesEqual(skeleton->getBodyScales(), first,
                      @"repeating the same subject must be idempotent");
    assertScalesEqual(skeleton->getBodyScales(), expected,
                      @"repeat must not multiply the subject ratio twice");
}

- (void)testReloadingAnotherModelRefreshesEveryScalingBaseline {
    NimbleBridge *bridge = [[NimbleBridge alloc] init];
    XCTAssertTrue([bridge loadModelFromPath:[self modelPath:@"Rajagopal2016"]]);
    std::shared_ptr<dynamics::Skeleton> firstSkeleton = [bridge sharedSkeleton];
    XCTAssertTrue(firstSkeleton != nullptr);
    if (firstSkeleton == nullptr) return;
    ScaleMarkers firstMarkers = markersFromLoadedModel(firstSkeleton, 1.12);
    XCTAssertTrue([bridge scaleModelWithHeight:1.8 * 1.12
                               markerPositions:firstMarkers.positions
                                   markerNames:firstMarkers.names]);

    XCTAssertTrue([bridge loadModelFromPath:[self modelPath:@"FullBody"]]);
    std::shared_ptr<dynamics::Skeleton> reloadedSkeleton = [bridge sharedSkeleton];
    XCTAssertTrue(reloadedSkeleton != nullptr);
    if (reloadedSkeleton == nullptr) return;
    XCTAssertTrue(reloadedSkeleton != firstSkeleton,
                  @"reload must replace the live skeleton instance");

    const Eigen::VectorXs reloadedDefaults = reloadedSkeleton->getBodyScales();
    ScaleMarkers reloadedMarkers = markersFromLoadedModel(reloadedSkeleton, 1.0);
    XCTAssertTrue([bridge scaleModelWithHeight:1.8
                               markerPositions:reloadedMarkers.positions
                                   markerNames:reloadedMarkers.names]);

    assertScalesEqual(reloadedSkeleton->getBodyScales(), reloadedDefaults,
                      @"FullBody reload must use FullBody defaults and references");
}

@end
