#import <CommonCrypto/CommonDigest.h>
#import <XCTest/XCTest.h>

#import "NimbleBridge.h"
#import "NimbleBridge+Internal.h"

#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/math/MathTypes.hpp"

using namespace dart;

namespace {

constexpr const char *kFixtureSource =
    "upstream demo image; shipping SAM3DBodyPose raw joint_coords underlying "
    "OfflineMuscleChainFixture; legacy marker baseline";
constexpr const char *kFixtureGenerator =
    "labs/sam-3d-body/export/e2e_check.py --swift";
constexpr const char *kFixtureSHA256 =
    "a122b84e1052d3f44dc39a5dfdd7fc861e86721ff13137f10140d0ed97bc58ef";

struct FixtureMarker {
    std::string sourceJoint;
    std::string legacyMarker;
    Eigen::Vector3d position;
};

struct CoreMLRootFixture {
    int version = 0;
    std::string source;
    std::string generator;
    std::string payloadSHA256;
    std::vector<FixtureMarker> markers;
    std::string error;

    bool ok() const { return error.empty(); }
};

static CoreMLRootFixture fixtureFailure(const char *message) {
    CoreMLRootFixture fixture;
    fixture.error = message;
    return fixture;
}

static bool hasOnlyPrintableASCII(NSString *value) {
    NSData *data = [value dataUsingEncoding:NSUTF8StringEncoding];
    if (data == nil) return false;
    const uint8_t *bytes = static_cast<const uint8_t *>(data.bytes);
    for (NSUInteger i = 0; i < data.length; ++i) {
        if (bytes[i] < 0x20 || bytes[i] > 0x7e) return false;
    }
    return true;
}

static bool parseStrictUnsigned(NSString *token, int *value) {
    if (token.length == 0 || (token.length > 1 && [token hasPrefix:@"0"])) {
        return false;
    }
    for (NSUInteger i = 0; i < token.length; ++i) {
        const unichar c = [token characterAtIndex:i];
        if (c < '0' || c > '9') return false;
    }
    const long parsed = std::strtol(token.UTF8String, nullptr, 10);
    if (parsed < 0 || parsed > INT_MAX) return false;
    *value = static_cast<int>(parsed);
    return true;
}

static bool parseStrictDecimal(NSString *token, double *value) {
    if (token.length == 0 || !hasOnlyPrintableASCII(token)) return false;

    NSUInteger cursor = [token hasPrefix:@"-"] ? 1 : 0;
    if (cursor == token.length) return false;
    const NSUInteger wholeStart = cursor;
    while (cursor < token.length) {
        const unichar c = [token characterAtIndex:cursor];
        if (c < '0' || c > '9') break;
        ++cursor;
    }
    const NSUInteger wholeLength = cursor - wholeStart;
    if (wholeLength == 0 || cursor >= token.length ||
        [token characterAtIndex:cursor] != '.') {
        return false;
    }
    if (wholeLength > 1 && [token characterAtIndex:wholeStart] == '0') {
        return false;
    }
    ++cursor;
    const NSUInteger fractionStart = cursor;
    while (cursor < token.length) {
        const unichar c = [token characterAtIndex:cursor];
        if (c < '0' || c > '9') return false;
        ++cursor;
    }
    if (cursor == fractionStart) return false;

    char *end = nullptr;
    const double parsed = std::strtod(token.UTF8String, &end);
    if (end == nullptr || *end != '\0' || !std::isfinite(parsed)) return false;
    *value = parsed;
    return true;
}

static bool parseField(NSString *line, NSString *key, NSString **value) {
    NSString *prefix = [key stringByAppendingString:@" "];
    if (![line hasPrefix:prefix]) return false;
    NSString *candidate = [line substringFromIndex:prefix.length];
    if (candidate.length == 0 || !hasOnlyPrintableASCII(candidate)) return false;
    *value = candidate;
    return true;
}

static std::string sha256Hex(NSData *data) {
    unsigned char digest[CC_SHA256_DIGEST_LENGTH];
    CC_SHA256(data.bytes, static_cast<CC_LONG>(data.length), digest);
    static const char digits[] = "0123456789abcdef";
    std::string result;
    result.reserve(CC_SHA256_DIGEST_LENGTH * 2);
    for (unsigned char byte : digest) {
        result.push_back(digits[byte >> 4]);
        result.push_back(digits[byte & 0x0f]);
    }
    return result;
}

/// Reads an intentionally tiny, versioned snapshot of the same shipping Core
/// ML output used by OfflineMuscleChainFixture. No partial rows escape: every
/// schema, provenance, count, numeric, ordering, and digest check succeeds
/// before `markers` is assigned.
static CoreMLRootFixture loadCoreMLRootFixture(Class testClass) {
    NSBundle *bundle = [NSBundle bundleForClass:testClass];
    NSURL *url = [bundle URLForResource:@"mhr_root_coreml"
                          withExtension:@"txt"
                           subdirectory:@"Fixtures"];
    if (url == nil) return fixtureFailure("fixture resource is missing");

    NSData *fileData = [NSData dataWithContentsOfURL:url];
    if (fileData == nil) return fixtureFailure("fixture cannot be read");
    NSString *text = [[NSString alloc] initWithData:fileData
                                           encoding:NSUTF8StringEncoding];
    if (text == nil) return fixtureFailure("fixture is not UTF-8");
    if (![text hasSuffix:@"\n"] || [text containsString:@"\r"]) {
        return fixtureFailure("fixture must use LF lines and end with one LF");
    }

    NSArray<NSString *> *split = [text componentsSeparatedByString:@"\n"];
    if (split.count != 13 || ![split.lastObject isEqualToString:@""]) {
        return fixtureFailure("fixture line count does not match schema v1");
    }
    NSArray<NSString *> *lines = [split subarrayWithRange:NSMakeRange(0, 12)];
    for (NSString *line in lines) {
        if (!hasOnlyPrintableASCII(line)) {
            return fixtureFailure("fixture contains non-printable or non-ASCII data");
        }
    }

    if (![lines[0] isEqualToString:@"format biomotion-mhr-root"]) {
        return fixtureFailure("fixture format is unsupported");
    }

    NSString *versionToken = nil;
    int version = 0;
    if (!parseField(lines[1], @"version", &versionToken) ||
        !parseStrictUnsigned(versionToken, &version) || version != 1) {
        return fixtureFailure("fixture version is unsupported");
    }

    NSString *source = nil;
    NSString *generator = nil;
    NSString *declaredHash = nil;
    NSString *countToken = nil;
    int markerCount = 0;
    if (!parseField(lines[2], @"source", &source) ||
        ![source isEqualToString:[NSString stringWithUTF8String:kFixtureSource]]) {
        return fixtureFailure("fixture source provenance is missing or unexpected");
    }
    if (!parseField(lines[3], @"generator", &generator) ||
        ![generator isEqualToString:[NSString stringWithUTF8String:kFixtureGenerator]]) {
        return fixtureFailure("fixture generator provenance is missing or unexpected");
    }
    if (!parseField(lines[4], @"payload_sha256", &declaredHash) ||
        ![declaredHash isEqualToString:[NSString stringWithUTF8String:kFixtureSHA256]]) {
        return fixtureFailure("fixture digest declaration is unexpected");
    }
    if (!parseField(lines[5], @"marker_count", &countToken) ||
        !parseStrictUnsigned(countToken, &markerCount) || markerCount != 3) {
        return fixtureFailure("fixture marker count is invalid");
    }
    if (![lines[6] isEqualToString:@"columns source_joint legacy_marker x y z"] ||
        ![lines[7] isEqualToString:@"data"] ||
        ![lines[11] isEqualToString:@"end"]) {
        return fixtureFailure("fixture columns or data delimiters are invalid");
    }

    NSMutableString *payload = [NSMutableString string];
    for (NSUInteger i = 8; i < 11; ++i) {
        [payload appendString:lines[i]];
        [payload appendString:@"\n"];
    }
    NSData *payloadData = [payload dataUsingEncoding:NSUTF8StringEncoding];
    if (sha256Hex(payloadData) != declaredHash.UTF8String) {
        return fixtureFailure("fixture payload digest does not match");
    }

    static const char *expectedSourceJoints[] = {
        "hips_joint", "left_upLeg_joint", "right_upLeg_joint",
    };
    static const char *expectedLegacyMarkers[] = {"PELVIS", "LHJC", "RHJC"};
    std::vector<FixtureMarker> parsedMarkers;
    parsedMarkers.reserve(static_cast<size_t>(markerCount));
    for (int row = 0; row < markerCount; ++row) {
        NSArray<NSString *> *fields = [lines[static_cast<NSUInteger>(8 + row)]
            componentsSeparatedByString:@" "];
        if (fields.count != 5 ||
            ![fields[0] isEqualToString:
                [NSString stringWithUTF8String:expectedSourceJoints[row]]] ||
            ![fields[1] isEqualToString:
                [NSString stringWithUTF8String:expectedLegacyMarkers[row]]]) {
            return fixtureFailure("fixture marker ordering or names are invalid");
        }
        double x = 0.0, y = 0.0, z = 0.0;
        if (!parseStrictDecimal(fields[2], &x) ||
            !parseStrictDecimal(fields[3], &y) ||
            !parseStrictDecimal(fields[4], &z)) {
            return fixtureFailure("fixture contains an invalid coordinate");
        }
        parsedMarkers.push_back({
            expectedSourceJoints[row],
            expectedLegacyMarkers[row],
            Eigen::Vector3d(x, y, z),
        });
    }

    CoreMLRootFixture fixture;
    fixture.version = version;
    fixture.source = source.UTF8String;
    fixture.generator = generator.UTF8String;
    fixture.payloadSHA256 = declaredHash.UTF8String;
    fixture.markers = std::move(parsedMarkers);
    return fixture;
}

static void appendPosition(NSMutableArray<NSNumber *> *positions,
                           const Eigen::Vector3d& value) {
    [positions addObject:@(value.x())];
    [positions addObject:@(value.y())];
    [positions addObject:@(value.z())];
}

static NSMutableArray<NSNumber *> *fixturePositions(
    const CoreMLRootFixture& fixture) {
    NSMutableArray<NSNumber *> *positions =
        [NSMutableArray arrayWithCapacity:fixture.markers.size() * 3];
    for (const FixtureMarker& marker : fixture.markers) {
        appendPosition(positions, marker.position);
    }
    return positions;
}

} // namespace

@interface MHRRootMarkerTests : XCTestCase
@end

@implementation MHRRootMarkerTests

- (NSString *)modelPath:(NSString *)name {
    NSString *path = [[NSBundle bundleForClass:[self class]] pathForResource:name
                                                                      ofType:@"osim"];
    XCTAssertNotNil(path, @"%@.osim must be present in the test bundle", name);
    return path;
}

- (void)testBothModelsKeepPelvisAndPlaceMHRRootAtHipJointMidpoint {
    for (NSString *modelName in @[@"FullBody", @"Rajagopal2016"]) {
        NimbleBridge *bridge = [[NimbleBridge alloc] init];
        XCTAssertTrue([bridge loadModelFromPath:[self modelPath:modelName]],
                      @"%@ must load", modelName);
        NSArray<NSString *> *availableMarkers = bridge.markerNames;
        XCTAssertTrue([availableMarkers containsObject:@"PELVIS"],
                      @"%@ must retain the legacy pelvis-body-origin marker", modelName);
        XCTAssertTrue([availableMarkers containsObject:@"MHR_ROOT"],
                      @"%@ must expose the Core ML root marker separately", modelName);

        std::shared_ptr<dynamics::Skeleton> skeleton = [bridge sharedSkeleton];
        XCTAssertTrue(skeleton != nullptr, @"%@ must expose its loaded skeleton", modelName);
        if (skeleton == nullptr) continue;
        skeleton->setPositions(Eigen::VectorXs::Zero(skeleton->getNumDofs()));
        skeleton->clampPositionsToLimits();

        auto bodyOrigin = [&](const char *bodyName) -> Eigen::Vector3d {
            dynamics::BodyNode *body = skeleton->getBodyNode(std::string(bodyName));
            XCTAssertTrue(body != nullptr, @"%@ is missing body %s", modelName, bodyName);
            if (body == nullptr) return Eigen::Vector3d::Zero();
            return body->getWorldTransform().translation().cast<double>();
        };
        const Eigen::Vector3d pelvis = bodyOrigin("pelvis");
        const Eigen::Vector3d leftHJC = bodyOrigin("femur_l");
        const Eigen::Vector3d rightHJC = bodyOrigin("femur_r");
        const Eigen::Vector3d hjcMidpoint = 0.5 * (leftHJC + rightHJC);
        XCTAssertGreaterThan((pelvis - hjcMidpoint).norm(), 0.05,
                             @"%@ fixture must distinguish pelvis origin from HJC midpoint",
                             modelName);

        NSMutableArray<NSNumber *> *positions = [NSMutableArray arrayWithCapacity:12];
        appendPosition(positions, pelvis);
        appendPosition(positions, hjcMidpoint);
        appendPosition(positions, leftHJC);
        appendPosition(positions, rightHJC);
        NSArray<NSString *> *names = @[@"PELVIS", @"MHR_ROOT", @"LHJC", @"RHJC"];

        [bridge resetSessionState];
        NimbleIKResult *result = [bridge solveIKWithMarkerPositions:positions
                                                       markerNames:names];
        XCTAssertNotNil(result, @"%@ exact marker contract must solve", modelName);
        XCTAssertEqual(result.markerCount, 4,
                       @"%@ must resolve all four names; unknown markers are silently skipped",
                       modelName);
        XCTAssertEqual(result.markerErrorsMeters.count, 4,
                       @"%@ must report every resolved marker", modelName);
        XCTAssertLessThan(result.markerRMSMeters, 1e-6,
                          @"%@ MHR_ROOT must be the HJC midpoint, not the pelvis origin",
                          modelName);
        XCTAssertLessThan(result.markerMaxErrorMeters, 1e-6,
                          @"%@ exact marker geometry must be preserved", modelName);
    }
}

- (void)testShippingCoreMLRootMarkerClosesThreePointGeometryMismatch {
    const CoreMLRootFixture fixture = loadCoreMLRootFixture([self class]);
    XCTAssertTrue(fixture.ok(), @"fixture must fail closed: %s", fixture.error.c_str());
    if (!fixture.ok()) return;
    XCTAssertEqual(fixture.version, 1);
    XCTAssertEqual(fixture.markers.size(), static_cast<size_t>(3));
    XCTAssertTrue(fixture.source == kFixtureSource);
    XCTAssertTrue(fixture.generator == kFixtureGenerator);
    XCTAssertTrue(fixture.payloadSHA256 == kFixtureSHA256);

    const Eigen::Vector3d rawRoot = fixture.markers[0].position;
    const Eigen::Vector3d sourceHipMidpoint = 0.5 * (
        fixture.markers[1].position + fixture.markers[2].position
    );
    const double sourceRootOffset = (rawRoot - sourceHipMidpoint).norm();
    XCTAssertGreaterThan(sourceRootOffset, 0.012,
                         @"fixture must prove raw MHR root is not literally the HJC midpoint");
    XCTAssertLessThan(sourceRootOffset, 0.020,
                      @"fixture root-to-midpoint offset drifted beyond its registered range");

    NimbleBridge *bridge = [[NimbleBridge alloc] init];
    XCTAssertTrue([bridge loadModelFromPath:[self modelPath:@"FullBody"]]);
    NSMutableArray<NSNumber *> *positions = fixturePositions(fixture);

    [bridge resetSessionState];
    NimbleIKResult *legacy = [bridge
        solveIKWithMarkerPositions:positions
                       markerNames:@[@"PELVIS", @"LHJC", @"RHJC"]];
    XCTAssertNotNil(legacy);
    XCTAssertEqual(legacy.markerCount, 3,
                   @"legacy baseline must use all three fixture points");
    XCTAssertEqual(legacy.markerErrorsMeters.count, 3,
                   @"legacy baseline must not drop a fixture point");

    [bridge resetSessionState];
    NimbleIKResult *withRoot = [bridge
        solveIKWithMarkerPositions:positions
                       markerNames:@[@"MHR_ROOT", @"LHJC", @"RHJC"]];
    XCTAssertNotNil(withRoot);
    XCTAssertEqual(withRoot.markerCount, 3,
                   @"MHR_ROOT must resolve; the bridge silently skips unknown names");
    XCTAssertEqual(withRoot.markerErrorsMeters.count, 3,
                   @"all three Core ML points must participate in the comparison");
    printf("MHRROOT-METRIC source_root_to_hip_midpoint_m=%.9f legacy_rms_m=%.9f mhr_root_rms_m=%.9f\n",
           sourceRootOffset, legacy.markerRMSMeters, withRoot.markerRMSMeters);
    XCTAssertGreaterThan(legacy.markerRMSMeters, 0.025,
                         @"the shipping fixture must retain the gross legacy mismatch");
    XCTAssertLessThan(withRoot.markerRMSMeters, 0.012,
                      @"the explicit proxy should leave only the disclosed source offset");
    XCTAssertLessThan(withRoot.markerRMSMeters, legacy.markerRMSMeters * 0.40,
                      @"the source-specific proxy must close most of the gross mismatch");
}

@end
