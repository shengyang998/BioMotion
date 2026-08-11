#import <XCTest/XCTest.h>

#import "NimbleBridge.h"

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "dart/biomechanics/OpenSimParser.hpp"
#include "dart/common/ResourceRetriever.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace {

constexpr char kOpenSimGeometryUnavailableMessage[]
    = "OpenSim geometry loading is unavailable in this iOS build because "
      "Assimp mesh support is not linked; pass ignoreGeometry=true.";

enum class ParseAttemptKind
{
  ReturnedNormally,
  RuntimeError,
  StandardException,
  NonStandardException
};

struct ParseAttempt
{
  ParseAttemptKind kind;
  std::string detail;
};

const char* parseAttemptKindName(ParseAttemptKind kind)
{
  switch (kind)
  {
    case ParseAttemptKind::ReturnedNormally:
      return "returned normally";
    case ParseAttemptKind::RuntimeError:
      return "std::runtime_error";
    case ParseAttemptKind::StandardException:
      return "other std::exception";
    case ParseAttemptKind::NonStandardException:
      return "non-standard exception";
  }

  return "invalid outcome";
}

/// Only call pure C++ parsing operations from this helper. XCTest assertions
/// and Objective-C messaging remain outside its exception boundary.
template <typename Function>
ParseAttempt captureParseAttempt(Function&& function)
{
  try
  {
    std::forward<Function>(function)();
    return {ParseAttemptKind::ReturnedNormally, "operation returned normally"};
  }
  catch (const std::runtime_error& error)
  {
    return {
        ParseAttemptKind::RuntimeError,
        error.what() == nullptr ? "" : error.what()};
  }
  catch (const std::exception& error)
  {
    return {
        ParseAttemptKind::StandardException,
        error.what() == nullptr ? "" : error.what()};
  }
  catch (...)
  {
    return {
        ParseAttemptKind::NonStandardException,
        "operation threw a non-standard exception"};
  }
}

class NoAccessResourceRetriever final : public dart::common::ResourceRetriever
{
public:
  bool exists(const dart::common::Uri& /*uri*/) override
  {
    ++mAccessCount;
    return false;
  }

  dart::common::ResourcePtr retrieve(
      const dart::common::Uri& /*uri*/) override
  {
    ++mAccessCount;
    return nullptr;
  }

  std::string readAll(const dart::common::Uri& /*uri*/) override
  {
    ++mAccessCount;
    throw std::runtime_error("OpenSim parser attempted resource I/O");
  }

  std::string getFilePath(const dart::common::Uri& /*uri*/) override
  {
    ++mAccessCount;
    return "";
  }

  std::size_t getAccessCount() const
  {
    return mAccessCount;
  }

private:
  std::size_t mAccessCount = 0;
};

std::size_t countMeshShapes(
    const std::shared_ptr<dart::dynamics::Skeleton>& skeleton)
{
  std::size_t count = 0;
  for (std::size_t bodyIndex = 0;
       bodyIndex < skeleton->getNumBodyNodes();
       ++bodyIndex)
  {
    const dart::dynamics::BodyNode* body = skeleton->getBodyNode(bodyIndex);
    for (std::size_t shapeIndex = 0;
         shapeIndex < body->getNumShapeNodes();
         ++shapeIndex)
    {
      const auto& shape = body->getShapeNode(shapeIndex)->getShape();
      if (shape != nullptr
          && shape->getType()
                 == dart::dynamics::MeshShape::getStaticType())
      {
        ++count;
      }
    }
  }
  return count;
}

} // namespace

@interface OpenSimGeometryBoundaryTests : XCTestCase
@end

@implementation OpenSimGeometryBoundaryTests

- (NSString*)modelPath:(NSString*)modelName
{
  NSString* path
      = [[NSBundle bundleForClass:[self class]] pathForResource:modelName
                                                     ofType:@"osim"];
  XCTAssertNotNil(
      path, @"%@.osim must be present in the test bundle", modelName);
  return path;
}

- (void)assertGeometryUnavailableAttempt:(const ParseAttempt&)attempt
                               operation:(const char*)operation
{
  if (attempt.kind != ParseAttemptKind::RuntimeError)
  {
    XCTFail(
        @"%s must throw std::runtime_error; outcome=%s detail=%s",
        operation,
        parseAttemptKindName(attempt.kind),
        attempt.detail.c_str());
    return;
  }

  NSString* actual = [NSString stringWithUTF8String:attempt.detail.c_str()];
  NSString* expected
      = [NSString stringWithUTF8String:kOpenSimGeometryUnavailableMessage];
  XCTAssertEqualObjects(
      actual, expected, @"%s must expose the reviewed diagnostic", operation);
}

- (void)testUriOverloadRejectsGeometryBeforeResourceAccess
{
  auto retriever = std::make_shared<NoAccessResourceRetriever>();
  ParseAttempt attempt = captureParseAttempt([retriever]() {
    (void)dart::biomechanics::OpenSimParser::parseOsim(
        dart::common::Uri("file:///must-not-be-read/missing.osim"),
        "",
        false,
        retriever);
  });

  [self assertGeometryUnavailableAttempt:attempt
                               operation:"parseOsim(Uri, ignoreGeometry=false)"];
  XCTAssertEqual(
      retriever->getAccessCount(),
      static_cast<std::size_t>(0),
      @"the unavailable-geometry boundary must run before resource I/O");
}

- (void)testDocumentOverloadRejectsGeometryBeforeXMLInspection
{
  tinyxml2::XMLDocument emptyDocument;
  auto retriever = std::make_shared<NoAccessResourceRetriever>();
  ParseAttempt attempt = captureParseAttempt([&emptyDocument, retriever]() {
    (void)dart::biomechanics::OpenSimParser::parseOsim(
        emptyDocument, "empty.osim", "", false, retriever);
  });

  [self assertGeometryUnavailableAttempt:
            attempt operation:"parseOsim(XMLDocument, ignoreGeometry=false)"];
  XCTAssertEqual(
      retriever->getAccessCount(),
      static_cast<std::size_t>(0),
      @"the document overload must reject before consulting a retriever");
}

- (void)testFullBodyParsesWithoutInstantiatingGeometry
{
  NSString* path = [self modelPath:@"FullBody"];
  if (path == nil)
    return;

  dart::biomechanics::OpenSimFile parsed
      = dart::biomechanics::OpenSimParser::parseOsim(
          std::string(path.UTF8String), "", true);
  XCTAssertTrue(parsed.skeleton != nullptr);
  if (parsed.skeleton == nullptr)
    return;

  // Fixture contract order: bodies, DOFs, native markers, mesh metadata,
  // mesh-scale metadata, and instantiated MeshShape objects.
  XCTAssertEqual(
      parsed.skeleton->getNumBodyNodes(), static_cast<std::size_t>(80));
  XCTAssertEqual(parsed.skeleton->getNumDofs(), static_cast<std::size_t>(169));
  XCTAssertEqual(parsed.markersMap.size(), static_cast<std::size_t>(57));
  XCTAssertEqual(parsed.meshMap.size(), static_cast<std::size_t>(132));
  XCTAssertEqual(parsed.meshScaleMap.size(), static_cast<std::size_t>(132));
  XCTAssertEqual(countMeshShapes(parsed.skeleton), static_cast<std::size_t>(0));
}

- (void)testRajagopalParsesWithoutInstantiatingGeometry
{
  NSString* path = [self modelPath:@"Rajagopal2016"];
  if (path == nil)
    return;

  dart::biomechanics::OpenSimFile parsed
      = dart::biomechanics::OpenSimParser::parseOsim(
          std::string(path.UTF8String), "", true);
  XCTAssertTrue(parsed.skeleton != nullptr);
  if (parsed.skeleton == nullptr)
    return;

  XCTAssertEqual(
      parsed.skeleton->getNumBodyNodes(), static_cast<std::size_t>(20));
  XCTAssertEqual(parsed.skeleton->getNumDofs(), static_cast<std::size_t>(37));
  XCTAssertEqual(parsed.markersMap.size(), static_cast<std::size_t>(66));
  XCTAssertEqual(parsed.meshMap.size(), static_cast<std::size_t>(79));
  XCTAssertEqual(parsed.meshScaleMap.size(), static_cast<std::size_t>(79));
  XCTAssertEqual(countMeshShapes(parsed.skeleton), static_cast<std::size_t>(0));
}

- (void)testFullBodyBridgeMarkerNamesRemainStable
{
  NSString* path = [self modelPath:@"FullBody"];
  if (path == nil)
    return;

  NimbleBridge* bridge = [[NimbleBridge alloc] init];
  XCTAssertTrue([bridge loadModelFromPath:path]);
  if (!bridge.isModelLoaded)
    return;

  NSArray<NSString*>* names = bridge.markerNames;
  XCTAssertEqual(names.count, static_cast<NSUInteger>(78));
  XCTAssertEqual([NSSet setWithArray:names].count, names.count);
  for (NSString* requiredName in @[
         @"PELVIS", @"MHR_ROOT", @"SPINE_L", @"SPINE_M", @"C7", @"NECK",
         @"HEAD", @"LHJC", @"RHJC", @"LTOE", @"RTOE"
       ])
  {
    XCTAssertTrue(
        [names containsObject:requiredName],
        @"FullBody bridge marker contract is missing %@",
        requiredName);
  }
}

- (void)testRajagopalBridgeMarkerNamesRemainStable
{
  NSString* path = [self modelPath:@"Rajagopal2016"];
  if (path == nil)
    return;

  NimbleBridge* bridge = [[NimbleBridge alloc] init];
  XCTAssertTrue([bridge loadModelFromPath:path]);
  if (!bridge.isModelLoaded)
    return;

  NSArray<NSString*>* names = bridge.markerNames;
  XCTAssertEqual(names.count, static_cast<NSUInteger>(74));
  XCTAssertEqual([NSSet setWithArray:names].count, names.count);
  for (NSString* requiredName in @[
         @"PELVIS", @"MHR_ROOT", @"SPINE_L", @"SPINE_M", @"C7", @"NECK",
         @"HEAD", @"LHJC", @"RHJC", @"LTOE", @"RTOE"
       ])
  {
    XCTAssertTrue(
        [names containsObject:requiredName],
        @"Rajagopal bridge marker contract is missing %@",
        requiredName);
  }
}

@end
