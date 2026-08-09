#import <XCTest/XCTest.h>

#include "dart/math/SimmSpline.hpp"

using dart::math::SimmSpline;

@interface SimmSplineExtrapolationTests : XCTestCase
@end

@implementation SimmSplineExtrapolationTests

- (void)testLinearExtrapolationMatchesOpenSimAtBothEnds {
    SimmSpline spline({0.0, 1.0, 2.0}, {0.0, 1.0, 0.0});
    constexpr double tolerance = 1e-12;

    // The endpoint tangents of this symmetric three-knot spline are +2 and -2.
    XCTAssertEqualWithAccuracy(spline.calcValue(0.0), 0.0, tolerance);
    XCTAssertEqualWithAccuracy(spline.calcDerivative(1, 0.0), 2.0, tolerance);
    XCTAssertEqualWithAccuracy(spline.calcValue(2.0), 0.0, tolerance);
    XCTAssertEqualWithAccuracy(spline.calcDerivative(1, 2.0), -2.0, tolerance);

    // OpenSim continues those tangents outside the knot domain. The linked
    // Nimble archive used to continue the endpoint cubics instead, returning
    // (-3, +4, -2) below and (-3, -4, -2) above for (value, d1, d2).
    XCTAssertEqualWithAccuracy(spline.calcValue(-1.0), -2.0, tolerance);
    XCTAssertEqualWithAccuracy(spline.calcDerivative(1, -1.0), 2.0, tolerance);
    XCTAssertEqualWithAccuracy(spline.calcDerivative(2, -1.0), 0.0, tolerance);

    XCTAssertEqualWithAccuracy(spline.calcValue(3.0), -2.0, tolerance);
    XCTAssertEqualWithAccuracy(spline.calcDerivative(1, 3.0), -2.0, tolerance);
    XCTAssertEqualWithAccuracy(spline.calcDerivative(2, 3.0), 0.0, tolerance);
}

@end
