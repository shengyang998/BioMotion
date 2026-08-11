#import <XCTest/XCTest.h>

#include <string>

#include "dart/biomechanics/C3DLoader.hpp"
#include "dart/biomechanics/OpenSimParser.hpp"

namespace {

using OpenSimC3DConsumer
    = decltype(&dart::biomechanics::OpenSimParser::
                   loadMotAtLowestMarkerRMSERotation);

} // namespace

@interface C3DIOSBoundaryTests : XCTestCase
@end

@implementation C3DIOSBoundaryTests

- (void)testC3DAndForcePlateRemainUsableValueTypes
{
  dart::biomechanics::C3D c3d{};
  c3d.framesPerSecond = 120;
  c3d.timestamps = {0.0, 1.0 / 120.0};
  c3d.markers = {"RASI"};
  c3d.markerTimesteps.resize(2);
  c3d.markerTimesteps[0]["RASI"] = Eigen::Vector3s(1.0, 2.0, 3.0);
  c3d.shuffledMarkersMatrix = Eigen::MatrixXs::Zero(3, 2);
  c3d.shuffledMarkersMatrixMask = Eigen::MatrixXs::Ones(3, 2);
  c3d.dataRotation = Eigen::Matrix3s::Identity();

  dart::biomechanics::ForcePlate plate;
  plate.worldOrigin = Eigen::Vector3s(0.1, 0.2, 0.3);
  plate.timestamps = c3d.timestamps;
  plate.corners = {
      Eigen::Vector3s(-0.2, 0.0, -0.3),
      Eigen::Vector3s(0.2, 0.0, -0.3),
      Eigen::Vector3s(0.2, 0.0, 0.3),
      Eigen::Vector3s(-0.2, 0.0, 0.3)};
  plate.centersOfPressure = {
      Eigen::Vector3s::Zero(), Eigen::Vector3s(0.01, 0.0, 0.02)};
  plate.moments = {
      Eigen::Vector3s::Zero(), Eigen::Vector3s(1.0, 2.0, 3.0)};
  plate.forces = {
      Eigen::Vector3s::Zero(), Eigen::Vector3s(0.0, 100.0, 0.0)};
  c3d.forcePlates.push_back(plate);

  XCTAssertEqual(c3d.framesPerSecond, 120);
  XCTAssertEqual(c3d.timestamps.size(), static_cast<std::size_t>(2));
  XCTAssertEqual(c3d.markerTimesteps.size(), static_cast<std::size_t>(2));
  XCTAssertEqual(c3d.shuffledMarkersMatrix.rows(), 3);
  XCTAssertEqual(c3d.shuffledMarkersMatrix.cols(), 2);
  XCTAssertEqualWithAccuracy(c3d.shuffledMarkersMatrixMask.sum(), 6.0, 1e-12);
  XCTAssertEqual(c3d.forcePlates.size(), static_cast<std::size_t>(1));
  XCTAssertEqual(c3d.forcePlates[0].corners.size(), static_cast<std::size_t>(4));
  XCTAssertEqual(
      c3d.forcePlates[0].centersOfPressure.size(), static_cast<std::size_t>(2));
  XCTAssertEqual(c3d.forcePlates[0].moments.size(), static_cast<std::size_t>(2));
  XCTAssertEqual(c3d.forcePlates[0].forces.size(), static_cast<std::size_t>(2));
  XCTAssertEqualWithAccuracy(c3d.forcePlates[0].worldOrigin.y(), 0.2, 1e-12);
  XCTAssertEqualWithAccuracy(
      c3d.markerTimesteps[0].at("RASI").z(), 3.0, 1e-12);
  XCTAssertTrue(c3d.dataRotation.isApprox(Eigen::Matrix3s::Identity()));
}

- (void)testOpenSimC3DConsumerRemainsLinked
{
  OpenSimC3DConsumer consumer
      = &dart::biomechanics::OpenSimParser::
          loadMotAtLowestMarkerRMSERotation;
  XCTAssertTrue(consumer != nullptr);
}

@end
