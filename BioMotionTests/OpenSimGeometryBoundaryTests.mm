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
#include "dart/dynamics/CustomJoint.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
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

- (void)testCustomJointPreservesNonIdentityLinearFunctionsAndCoordinateMappings
{
  NSString* sourcePath = [self modelPath:@"Rajagopal2016"];
  if (sourcePath == nil)
    return;

  NSError* readError = nil;
  NSString* source = [NSString stringWithContentsOfFile:sourcePath
                                                encoding:NSUTF8StringEncoding
                                                   error:&readError];
  XCTAssertNotNil(source, @"%@", readError);
  if (source == nil)
    return;

  // The specialized six-axis fast path may only accept identity functions.
  // Force pelvis_tx to 2*q and require the exact CustomJoint fallback to move
  // the pelvis by 0.2 m when q is 0.1 m.
  NSRange translationAnchor
      = [source rangeOfString:@"<coordinates>pelvis_tx</coordinates>"];
  XCTAssertNotEqual(translationAnchor.location, NSNotFound);
  if (translationAnchor.location == NSNotFound)
    return;
  NSRange search = NSMakeRange(
      NSMaxRange(translationAnchor),
      source.length - NSMaxRange(translationAnchor));
  NSRange translationCoefficients
      = [source rangeOfString:@"<coefficients> 1 0</coefficients>"
                      options:0
                        range:search];
  XCTAssertNotEqual(translationCoefficients.location, NSNotFound);
  if (translationCoefficients.location == NSNotFound)
    return;

  NSMutableString* doubled = [source mutableCopy];
  [doubled replaceCharactersInRange:translationCoefficients
                         withString:@"<coefficients> 2 0</coefficients>"];
  NSString* doubledPath = [NSTemporaryDirectory()
      stringByAppendingPathComponent:[NSString
          stringWithFormat:@"nonidentity-linear-%@.osim", NSUUID.UUID.UUIDString]];
  NSError* writeError = nil;
  BOOL didWrite = [doubled writeToFile:doubledPath
                            atomically:YES
                              encoding:NSUTF8StringEncoding
                                 error:&writeError];
  XCTAssertTrue(didWrite, @"%@", writeError);
  if (!didWrite)
    return;

  dart::biomechanics::OpenSimFile doubledFile
      = dart::biomechanics::OpenSimParser::parseOsim(
          std::string(doubledPath.UTF8String), "", true);
  [[NSFileManager defaultManager] removeItemAtPath:doubledPath error:nil];
  XCTAssertTrue(doubledFile.skeleton != nullptr);
  if (doubledFile.skeleton == nullptr)
    return;

  dart::dynamics::DegreeOfFreedom* pelvisTX
      = doubledFile.skeleton->getDof("pelvis_tx");
  dart::dynamics::BodyNode* pelvis
      = doubledFile.skeleton->getBodyNode("pelvis");
  XCTAssertTrue(pelvisTX != nullptr);
  XCTAssertTrue(pelvis != nullptr);
  if (pelvisTX == nullptr || pelvis == nullptr)
    return;

  Eigen::VectorXs q
      = Eigen::VectorXs::Zero(doubledFile.skeleton->getNumDofs());
  doubledFile.skeleton->setPositions(q);
  Eigen::Vector3s before = pelvis->getWorldTransform().translation();
  q(pelvisTX->getIndexInSkeleton()) = 0.1;
  doubledFile.skeleton->setPositions(q);
  Eigen::Vector3s delta
      = pelvis->getWorldTransform().translation() - before;
  XCTAssertEqualWithAccuracy(delta.x(), 0.2, 1e-12);
  XCTAssertEqualWithAccuracy(delta.y(), 0.0, 1e-12);
  XCTAssertEqualWithAccuracy(delta.z(), 0.0, 1e-12);

  // Baking a -1 rotational slope into the axis must negate the intercept too:
  // a*(-q+b) == (-a)*(q-b).
  NSRange tiltAnchor
      = [source rangeOfString:@"<coordinates>pelvis_tilt</coordinates>"];
  XCTAssertNotEqual(tiltAnchor.location, NSNotFound);
  if (tiltAnchor.location == NSNotFound)
    return;
  search = NSMakeRange(
      NSMaxRange(tiltAnchor), source.length - NSMaxRange(tiltAnchor));
  NSRange tiltCoefficients
      = [source rangeOfString:@"<coefficients> 1 0</coefficients>"
                      options:0
                        range:search];
  XCTAssertNotEqual(tiltCoefficients.location, NSNotFound);
  if (tiltCoefficients.location == NSNotFound)
    return;

  NSMutableString* reflected = [source mutableCopy];
  [reflected replaceCharactersInRange:tiltCoefficients
                           withString:@"<coefficients> -1 0.5</coefficients>"];
  NSString* reflectedPath = [NSTemporaryDirectory()
      stringByAppendingPathComponent:[NSString
          stringWithFormat:@"reflected-linear-%@.osim", NSUUID.UUID.UUIDString]];
  writeError = nil;
  didWrite = [reflected writeToFile:reflectedPath
                         atomically:YES
                           encoding:NSUTF8StringEncoding
                              error:&writeError];
  XCTAssertTrue(didWrite, @"%@", writeError);
  if (!didWrite)
    return;

  dart::biomechanics::OpenSimFile reflectedFile
      = dart::biomechanics::OpenSimParser::parseOsim(
          std::string(reflectedPath.UTF8String), "", true);
  [[NSFileManager defaultManager] removeItemAtPath:reflectedPath error:nil];
  XCTAssertTrue(reflectedFile.skeleton != nullptr);
  if (reflectedFile.skeleton == nullptr)
    return;

  auto* reflectedRoot = dynamic_cast<dart::dynamics::CustomJoint<6>*>(
      reflectedFile.skeleton->getJoint("ground_pelvis"));
  XCTAssertTrue(reflectedRoot != nullptr);
  if (reflectedRoot == nullptr)
    return;
  XCTAssertEqualWithAccuracy(reflectedRoot->getFlipAxisMap()(0), -1.0, 1e-12);
  XCTAssertEqualWithAccuracy(
      reflectedRoot->getCustomFunction(0)->calcValue(0.0), -0.5, 1e-12);
  XCTAssertEqualWithAccuracy(
      reflectedRoot->getCustomFunction(0)->calcValue(0.1), -0.4, 1e-12);

  // A specialized joint also requires each TransformAxis to reference its
  // corresponding DOF. Remap X to pelvis_ty and require the exact fallback:
  // pelvis_tx drives neither translation, while pelvis_ty drives X and Y.
  NSMutableString* remapped = [source mutableCopy];
  [remapped replaceCharactersInRange:translationAnchor
                           withString:@"<coordinates>pelvis_ty</coordinates>"];
  NSString* remappedPath = [NSTemporaryDirectory()
      stringByAppendingPathComponent:[NSString
          stringWithFormat:@"remapped-coordinate-%@.osim", NSUUID.UUID.UUIDString]];
  writeError = nil;
  didWrite = [remapped writeToFile:remappedPath
                        atomically:YES
                          encoding:NSUTF8StringEncoding
                             error:&writeError];
  XCTAssertTrue(didWrite, @"%@", writeError);
  if (!didWrite)
    return;

  dart::biomechanics::OpenSimFile remappedFile
      = dart::biomechanics::OpenSimParser::parseOsim(
          std::string(remappedPath.UTF8String), "", true);
  [[NSFileManager defaultManager] removeItemAtPath:remappedPath error:nil];
  XCTAssertTrue(remappedFile.skeleton != nullptr);
  if (remappedFile.skeleton == nullptr)
    return;

  dart::dynamics::DegreeOfFreedom* remappedTX
      = remappedFile.skeleton->getDof("pelvis_tx");
  dart::dynamics::DegreeOfFreedom* remappedTY
      = remappedFile.skeleton->getDof("pelvis_ty");
  dart::dynamics::BodyNode* remappedPelvis
      = remappedFile.skeleton->getBodyNode("pelvis");
  XCTAssertTrue(remappedTX != nullptr);
  XCTAssertTrue(remappedTY != nullptr);
  XCTAssertTrue(remappedPelvis != nullptr);
  if (remappedTX == nullptr || remappedTY == nullptr
      || remappedPelvis == nullptr)
    return;

  q = Eigen::VectorXs::Zero(remappedFile.skeleton->getNumDofs());
  remappedFile.skeleton->setPositions(q);
  before = remappedPelvis->getWorldTransform().translation();
  q(remappedTX->getIndexInSkeleton()) = 0.1;
  remappedFile.skeleton->setPositions(q);
  delta = remappedPelvis->getWorldTransform().translation() - before;
  XCTAssertEqualWithAccuracy(delta.x(), 0.0, 1e-12);
  XCTAssertEqualWithAccuracy(delta.y(), 0.0, 1e-12);
  XCTAssertEqualWithAccuracy(delta.z(), 0.0, 1e-12);

  q.setZero();
  q(remappedTY->getIndexInSkeleton()) = 0.1;
  remappedFile.skeleton->setPositions(q);
  delta = remappedPelvis->getWorldTransform().translation() - before;
  XCTAssertEqualWithAccuracy(delta.x(), 0.1, 1e-12);
  XCTAssertEqualWithAccuracy(delta.y(), 0.1, 1e-12);
  XCTAssertEqualWithAccuracy(delta.z(), 0.0, 1e-12);
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
