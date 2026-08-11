#import <XCTest/XCTest.h>

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/simulation/World.hpp"

namespace {

constexpr char kCollisionUnavailableMessage[] =
    "DARTCollisionDetector is unavailable in this iOS build because "
    "Assimp/libccd collision support is not linked.";

struct FactoryReturnedNull final
{
};

enum class CollisionAttemptKind
{
  ReturnedNormally,
  FactoryReturnedNull,
  RuntimeError,
  StandardException
};

struct CollisionAttempt
{
  CollisionAttemptKind kind;
  bool operationInvoked;
  std::string detail;
};

const char* collisionAttemptKindName(CollisionAttemptKind kind)
{
  switch (kind)
  {
    case CollisionAttemptKind::ReturnedNormally:
      return "returned normally";
    case CollisionAttemptKind::FactoryReturnedNull:
      return "factory returned nullptr";
    case CollisionAttemptKind::RuntimeError:
      return "std::runtime_error";
    case CollisionAttemptKind::StandardException:
      return "other std::exception";
  }

  return "invalid outcome";
}

/// Only wrap pure C++ factory/constructor operations here. XCTest assertions,
/// Objective-C messaging that may raise NSException, and @throw must remain
/// outside this helper.
template <typename Function>
CollisionAttempt captureCollisionAttempt(Function&& function)
{
  try
  {
    std::forward<Function>(function)();
    return {
        CollisionAttemptKind::ReturnedNormally,
        true,
        "operation returned normally"};
  }
  catch (const FactoryReturnedNull&)
  {
    return {
        CollisionAttemptKind::FactoryReturnedNull,
        true,
        "collision detector factory returned nullptr"};
  }
  catch (const std::runtime_error& error)
  {
    return {
        CollisionAttemptKind::RuntimeError,
        true,
        error.what() == nullptr ? "" : error.what()};
  }
  catch (const std::exception& error)
  {
    return {
        CollisionAttemptKind::StandardException,
        true,
        error.what() == nullptr ? "" : error.what()};
  }
}

void invokeDirectDARTCreate()
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  if (detector == nullptr)
    throw FactoryReturnedNull();
}

/// The legacy iOS stub returns nullptr. Calling ConstraintSolver, BoxedLCP or
/// World after that would dereference it in ConstraintSolver's initializer.
/// Only run a consumer after direct create demonstrates the exact new contract.
template <typename Function>
CollisionAttempt captureAfterDirectFactoryPreflight(Function&& function)
{
  CollisionAttempt preflight
      = captureCollisionAttempt([]() { invokeDirectDARTCreate(); });

  if (preflight.kind != CollisionAttemptKind::RuntimeError
      || preflight.detail != kCollisionUnavailableMessage)
  {
    preflight.operationInvoked = false;
    preflight.detail = "direct factory preflight: " + preflight.detail;
    return preflight;
  }

  return captureCollisionAttempt(std::forward<Function>(function));
}

class ConstraintSolverConstructorProbe final
    : public dart::constraint::ConstraintSolver
{
public:
  ConstraintSolverConstructorProbe()
    : dart::constraint::ConstraintSolver()
  {
  }

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif
  explicit ConstraintSolverConstructorProbe(s_t timeStep)
    : dart::constraint::ConstraintSolver(timeStep)
  {
  }
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

  std::vector<s_t*> solveConstrainedGroup(
      dart::constraint::ConstrainedGroup& /*group*/) override
  {
    return {};
  }
};

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif
void constructDeprecatedBoxedLcpConstraintSolver()
{
  dart::constraint::BoxedLcpConstraintSolver solver(
      static_cast<s_t>(0.001));
  (void)solver;
}
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

} // namespace

@interface CollisionFailClosedTests : XCTestCase
@end

@implementation CollisionFailClosedTests

- (void)assertUnavailableAttempt:(const CollisionAttempt&)attempt
                       operation:(const char*)operation
{
  if (!attempt.operationInvoked)
  {
    XCTFail(
        @"%s was not invoked because the safe direct-factory preflight did "
         "not produce the exact fail-closed result; outcome=%s detail=%s",
        operation,
        collisionAttemptKindName(attempt.kind),
        attempt.detail.c_str());
    return;
  }

  if (attempt.kind != CollisionAttemptKind::RuntimeError)
  {
    XCTFail(
        @"%s must throw std::runtime_error; outcome=%s detail=%s",
        operation,
        collisionAttemptKindName(attempt.kind),
        attempt.detail.c_str());
    return;
  }

  NSString* actual = [NSString stringWithUTF8String:attempt.detail.c_str()];
  NSString* expected
      = [NSString stringWithUTF8String:kCollisionUnavailableMessage];
  XCTAssertEqualObjects(
      actual,
      expected,
      @"%s must expose the reviewed diagnostic",
      operation);
}

- (void)testDirectDARTCollisionDetectorCreateFailsClosed
{
  CollisionAttempt attempt
      = captureCollisionAttempt([]() { invokeDirectDARTCreate(); });
  [self assertUnavailableAttempt:attempt
                       operation:"DARTCollisionDetector::create()"];
}

- (void)testCollisionDetectorFactoryRegistersDARTAsExplicitlyUnavailable
{
  // Deliberately no direct-create preflight: this proves the ordinary static
  // archive registry exposes the unavailable backend rather than omitting it.
  auto* factory = dart::collision::CollisionDetector::getFactory();
  XCTAssertTrue(factory != nullptr);
  if (factory == nullptr)
    return;

  const std::string key("dart");
  XCTAssertTrue(
      factory->canCreate(key),
      @"the iOS registry must advertise dart as explicitly unavailable");
  if (!factory->canCreate(key))
    return;

  CollisionAttempt attempt = captureCollisionAttempt([factory, key]() {
    auto detector = factory->create(key);
    if (detector == nullptr)
      throw FactoryReturnedNull();
  });
  [self assertUnavailableAttempt:attempt
                       operation:"CollisionDetector factory create(\"dart\")"];
}

- (void)testConstraintSolverConstructorsFailClosed
{
  CollisionAttempt defaultAttempt
      = captureAfterDirectFactoryPreflight([]() {
          ConstraintSolverConstructorProbe solver;
          (void)solver;
        });
  [self assertUnavailableAttempt:defaultAttempt
                       operation:"ConstraintSolver()"];

  CollisionAttempt timeStepAttempt
      = captureAfterDirectFactoryPreflight([]() {
          ConstraintSolverConstructorProbe solver(static_cast<s_t>(0.001));
          (void)solver;
        });
  [self assertUnavailableAttempt:timeStepAttempt
                       operation:"ConstraintSolver(s_t)"];
}

- (void)testBoxedLcpConstraintSolverConstructorsFailClosed
{
  CollisionAttempt defaultAttempt
      = captureAfterDirectFactoryPreflight([]() {
          dart::constraint::BoxedLcpConstraintSolver solver;
          (void)solver;
        });
  [self assertUnavailableAttempt:defaultAttempt
                       operation:"BoxedLcpConstraintSolver()"];

  CollisionAttempt timeStepAttempt
      = captureAfterDirectFactoryPreflight([]() {
          constructDeprecatedBoxedLcpConstraintSolver();
        });
  [self assertUnavailableAttempt:timeStepAttempt
                       operation:"BoxedLcpConstraintSolver(s_t)"];
}

- (void)testWorldConstructionPathsFailClosed
{
  CollisionAttempt directAttempt
      = captureAfterDirectFactoryPreflight([]() {
          dart::simulation::World world;
          (void)world;
        });
  [self assertUnavailableAttempt:directAttempt operation:"World()"];

  CollisionAttempt factoryAttempt
      = captureAfterDirectFactoryPreflight([]() {
          auto world = dart::simulation::World::create();
          (void)world;
        });
  [self assertUnavailableAttempt:factoryAttempt
                       operation:"World::create()"];
}

@end
