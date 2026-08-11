#import <XCTest/XCTest.h>

#import "NimbleBridge.h"
#import "NimbleBridge+Internal.h"

#include <algorithm>
#include <array>
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

enum class ScaleRootMarker {
    Pelvis,
    MHRRoot,
};

static ScaleMarkers markersFromLoadedModel(
    const std::shared_ptr<dynamics::Skeleton>& skeleton,
    double ratio,
    ScaleRootMarker rootMarker = ScaleRootMarker::Pelvis) {
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
    const Eigen::Vector3s hipMidpoint = 0.5 * (
        bodyOrigin("femur_l") + bodyOrigin("femur_r")
    );
    const char *rootName = rootMarker == ScaleRootMarker::MHRRoot
        ? "MHR_ROOT" : "PELVIS";
    const Eigen::Vector3s rootPosition = rootMarker == ScaleRootMarker::MHRRoot
        ? hipMidpoint : pelvis;

    [result.names addObject:[NSString stringWithUTF8String:rootName]];
    const Eigen::Vector3s scaledRoot = pelvis + ratio * (rootPosition - pelvis);
    [result.positions addObject:@(scaledRoot.x())];
    [result.positions addObject:@(scaledRoot.y())];
    [result.positions addObject:@(scaledRoot.z())];

    struct MarkerBody {
        const char *marker;
        const char *body;
    };
    static const MarkerBody markerBodies[] = {
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

static constexpr std::array<const char *, 9> kScaleGeometryBodyNames = {
    "pelvis",
    "femur_l", "femur_r",
    "talus_l", "talus_r",
    "humerus_l", "humerus_r",
    "hand_l", "hand_r",
};

using BodyTransformSnapshot = std::array<Eigen::Matrix4s, 9>;

static BodyTransformSnapshot bodyTransformsAtPose(
    const std::shared_ptr<dynamics::Skeleton>& skeleton,
    const Eigen::VectorXs& pose) {
    skeleton->setPositions(pose);
    BodyTransformSnapshot result;
    for (size_t index = 0; index < kScaleGeometryBodyNames.size(); ++index) {
        const char *bodyName = kScaleGeometryBodyNames[index];
        dynamics::BodyNode *body = skeleton->getBodyNode(std::string(bodyName));
        XCTAssertTrue(body != nullptr, @"missing body %s", bodyName);
        if (body == nullptr) {
            result[index].setIdentity();
        } else {
            result[index] = body->getWorldTransform().matrix();
        }
    }
    return result;
}

static void assertTransformsEqual(
    const BodyTransformSnapshot& actual,
    const BodyTransformSnapshot& expected,
    NSString *context) {
    for (size_t index = 0; index < kScaleGeometryBodyNames.size(); ++index) {
        const double worstDifference =
            (actual[index] - expected[index]).cwiseAbs().maxCoeff();
        XCTAssertEqualWithAccuracy(
            worstDifference,
            0.0,
            1e-10,
            @"%@ body %s transform",
            context,
            kScaleGeometryBodyNames[index]
        );
    }
}

static double maximumTransformDifference(
    const BodyTransformSnapshot& lhs,
    const BodyTransformSnapshot& rhs) {
    double result = 0.0;
    for (size_t index = 0; index < kScaleGeometryBodyNames.size(); ++index) {
        result = std::max(
            result,
            (lhs[index] - rhs[index]).cwiseAbs().maxCoeff()
        );
    }
    return result;
}

static Eigen::VectorXs neutralPoseForSkeleton(
    const std::shared_ptr<dynamics::Skeleton>& skeleton) {
    const Eigen::VectorXs savedPose = skeleton->getPositions();
    skeleton->setPositions(Eigen::VectorXs::Zero(skeleton->getNumDofs()));
    skeleton->clampPositionsToLimits();
    const Eigen::VectorXs neutralPose = skeleton->getPositions();
    skeleton->setPositions(savedPose);
    return neutralPose;
}

static Eigen::VectorXs distinctValidPose(
    const std::shared_ptr<dynamics::Skeleton>& skeleton,
    const Eigen::VectorXs& baseline) {
    Eigen::VectorXs result = baseline;
    const Eigen::VectorXs lowerLimits = skeleton->getPositionLowerLimits();
    const Eigen::VectorXs upperLimits = skeleton->getPositionUpperLimits();
    for (Eigen::Index index = 0; index < result.size(); ++index) {
        const double candidates[] = {
            baseline(index) + 0.05,
            baseline(index) - 0.05,
        };
        for (double candidate : candidates) {
            if (std::isfinite(candidate)
                && candidate >= lowerLimits(index)
                && candidate <= upperLimits(index)) {
                result(index) = candidate;
                return result;
            }
        }
    }
    XCTFail(@"loaded model must expose at least one movable coordinate");
    return result;
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

- (void)testMHRRootScalingUsesTheHipCentreReferenceAndDoesNotFallBackToHeight {
    NimbleBridge *bridge = [[NimbleBridge alloc] init];
    XCTAssertTrue([bridge loadModelFromPath:[self modelPath:@"FullBody"]]);
    std::shared_ptr<dynamics::Skeleton> skeleton = [bridge sharedSkeleton];
    XCTAssertTrue(skeleton != nullptr);
    if (skeleton == nullptr) return;

    const double markerRatio = 1.12;
    const double offlineMarkerRatio = 0.86;
    const Eigen::VectorXs loadedDefaults = skeleton->getBodyScales();
    const Eigen::VectorXs expected = loadedDefaults * markerRatio;
    const Eigen::VectorXs neutralPose = neutralPoseForSkeleton(skeleton);
    const BodyTransformSnapshot loadedDefaultTransforms =
        bodyTransformsAtPose(skeleton, neutralPose);
    ScaleMarkers markers = markersFromLoadedModel(
        skeleton,
        markerRatio,
        ScaleRootMarker::MHRRoot
    );
    ScaleMarkers offlineMarkers = markersFromLoadedModel(
        skeleton,
        offlineMarkerRatio,
        ScaleRootMarker::MHRRoot
    );
    XCTAssertEqualObjects(markers.names.firstObject, @"MHR_ROOT");

    // Height deliberately says 1.0x. If the bridge loses the MHR_ROOT alias,
    // only the trunk falls back to height and this cannot equal 1.12x.
    XCTAssertTrue([bridge scaleModelWithHeight:1.8
                               markerPositions:markers.positions
                                   markerNames:markers.names]);
    const Eigen::VectorXs first = skeleton->getBodyScales();
    assertScalesEqual(first, expected,
                      @"MHR_ROOT measurements must use the HJC-midpoint reference");
    const BodyTransformSnapshot liveTransforms =
        bodyTransformsAtPose(skeleton, neutralPose);

    // An offline subject may temporarily replace the shared skeleton's body
    // geometry. Its scales and neutral transforms must actually differ so the
    // two restoration assertions below cannot pass vacuously.
    XCTAssertTrue([bridge scaleModelWithHeight:1.8
                               markerPositions:offlineMarkers.positions
                                   markerNames:offlineMarkers.names]);
    assertScalesEqual(skeleton->getBodyScales(),
                      loadedDefaults * offlineMarkerRatio,
                      @"offline subject must use its own body geometry");
    const BodyTransformSnapshot offlineTransforms =
        bodyTransformsAtPose(skeleton, neutralPose);
    XCTAssertGreaterThan(maximumTransformDifference(offlineTransforms, liveTransforms),
                         1e-6,
                         @"offline geometry must differ from live geometry");

    // Replaying the value-only live recipe must reconstruct both all body
    // scales and representative bilateral neutral geometry exactly.
    XCTAssertTrue([bridge scaleModelWithHeight:1.8
                               markerPositions:markers.positions
                                   markerNames:markers.names]);
    assertScalesEqual(skeleton->getBodyScales(), first,
                      @"live recipe replay must restore every body scale");
    assertTransformsEqual(bodyTransformsAtPose(skeleton, neutralPose),
                          liveTransforms,
                          @"live recipe replay");

    // With no live recipe, the engine falls back to the exact baseline cached
    // at model load. The bridge restore itself must preserve pose and remain
    // idempotent while restoring the full neutral geometry.
    XCTAssertTrue([bridge scaleModelWithHeight:1.8
                               markerPositions:offlineMarkers.positions
                                   markerNames:offlineMarkers.names]);
    skeleton->setPositions(distinctValidPose(skeleton, neutralPose));
    const Eigen::VectorXs poseBeforeRestore = skeleton->getPositions();
    XCTAssertTrue([bridge restoreLoadedModelBodyScales]);
    assertScalesEqual(skeleton->getPositions(), poseBeforeRestore,
                      @"loaded-default restore must not reset pose");
    assertScalesEqual(skeleton->getBodyScales(), loadedDefaults,
                      @"loaded-default restore must restore every body scale");
    assertTransformsEqual(bodyTransformsAtPose(skeleton, neutralPose),
                          loadedDefaultTransforms,
                          @"loaded-default restore");

    XCTAssertTrue([bridge restoreLoadedModelBodyScales]);
    assertScalesEqual(skeleton->getBodyScales(), loadedDefaults,
                      @"loaded-default restore must be idempotent");
    assertTransformsEqual(bodyTransformsAtPose(skeleton, neutralPose),
                          loadedDefaultTransforms,
                          @"idempotent loaded-default restore");
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
    ScaleMarkers reloadedMarkers = markersFromLoadedModel(
        reloadedSkeleton,
        1.0,
        ScaleRootMarker::MHRRoot
    );
    XCTAssertTrue([bridge scaleModelWithHeight:1.8
                               markerPositions:reloadedMarkers.positions
                                   markerNames:reloadedMarkers.names]);

    assertScalesEqual(reloadedSkeleton->getBodyScales(), reloadedDefaults,
                      @"FullBody reload must use FullBody defaults and MHR_ROOT reference");
}

@end
