#include <array>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <new>
#include <stdexcept>
#include <string>

#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/WeldJoint.hpp"

namespace {

constexpr std::size_t kTrackedAllocationSize
    = sizeof(dart::dynamics::PointMassNotifier);
constexpr std::size_t kTrackedCapacity = 64;

bool gTrackNotifierSizedAllocations = false;
std::array<void*, kTrackedCapacity> gTrackedPointers{};
std::size_t gTrackedAllocations = 0;
std::size_t gTrackedDeallocations = 0;

void recordAllocation(void* pointer, std::size_t size)
{
  if (!gTrackNotifierSizedAllocations || size != kTrackedAllocationSize)
    return;

  for (void*& slot : gTrackedPointers)
  {
    if (slot == nullptr)
    {
      slot = pointer;
      ++gTrackedAllocations;
      return;
    }
  }

  std::fputs("tracked allocation table overflow\n", stderr);
  std::abort();
}

void recordDeallocation(void* pointer)
{
  if (pointer == nullptr)
    return;

  for (void*& slot : gTrackedPointers)
  {
    if (slot == pointer)
    {
      slot = nullptr;
      ++gTrackedDeallocations;
      return;
    }
  }
}

struct AllocationSummary
{
  std::size_t allocated;
  std::size_t freed;
  std::size_t live;
};

void beginAllocationTracking()
{
  gTrackedPointers.fill(nullptr);
  gTrackedAllocations = 0;
  gTrackedDeallocations = 0;
  gTrackNotifierSizedAllocations = true;
}

AllocationSummary endAllocationTracking()
{
  gTrackNotifierSizedAllocations = false;
  std::size_t live = 0;
  for (void* pointer : gTrackedPointers)
  {
    if (pointer != nullptr)
      ++live;
  }
  return {gTrackedAllocations, gTrackedDeallocations, live};
}

bool allocationTransactionIsBalanced(
    const AllocationSummary& summary, const char* label, int iteration)
{
  if (summary.allocated >= 1 && summary.allocated == summary.freed
      && summary.live == 0)
  {
    return true;
  }

  std::fprintf(
      stderr,
      "%s allocation mismatch at iteration %d: allocated=%zu freed=%zu "
      "live=%zu\n",
      label,
      iteration,
      summary.allocated,
      summary.freed,
      summary.live);
  return false;
}

} // namespace

void* operator new(std::size_t size)
{
  void* pointer = std::malloc(size == 0 ? 1 : size);
  if (pointer == nullptr)
    throw std::bad_alloc();
  recordAllocation(pointer, size);
  return pointer;
}

void* operator new[](std::size_t size)
{
  return ::operator new(size);
}

void operator delete(void* pointer) noexcept
{
  recordDeallocation(pointer);
  std::free(pointer);
}

void operator delete[](void* pointer) noexcept
{
  ::operator delete(pointer);
}

void operator delete(void* pointer, std::size_t) noexcept
{
  ::operator delete(pointer);
}

void operator delete[](void* pointer, std::size_t) noexcept
{
  ::operator delete(pointer);
}

namespace {

constexpr char kExpectedMessage[]
    = "SoftMeshShape is unavailable in this iOS build because Assimp mesh "
      "support is not linked.";
constexpr int kIterationCount = 32;

class InspectableBodyNode final : public dart::dynamics::BodyNode
{
public:
  using Properties = dart::dynamics::BodyNode::Properties;

  InspectableBodyNode(
      dart::dynamics::BodyNode* parent,
      dart::dynamics::Joint* parentJoint,
      const Properties& properties)
    : Entity(dart::dynamics::Frame::World(), false),
      Frame(dart::dynamics::Frame::World()),
      BodyNode(parent, parentJoint, properties)
  {
  }

  std::size_t getTrackedJacobianChildCount() const
  {
    return mChildJacobianNodes.size();
  }
};

[[noreturn]] void throwInjectedMeshRejection()
{
  throw std::runtime_error(kExpectedMessage);
}

template <typename Operation>
bool rejectsWithExpectedMessage(Operation&& operation, const char* label)
{
  try
  {
    operation();
    std::fprintf(stderr, "%s returned normally\n", label);
    return false;
  }
  catch (const std::runtime_error& error)
  {
    if (std::string(error.what()) == kExpectedMessage)
      return true;

    std::fprintf(
        stderr, "%s returned the wrong runtime_error: %s\n", label,
        error.what());
    return false;
  }
  catch (const std::exception& error)
  {
    std::fprintf(
        stderr, "%s returned the wrong std::exception: %s\n", label,
        error.what());
    return false;
  }
  catch (...)
  {
    std::fprintf(stderr, "%s returned a non-standard exception\n", label);
    return false;
  }
}

bool rootStateIsEmpty(const dart::dynamics::SkeletonPtr& skeleton)
{
  return skeleton->getNumBodyNodes() == 0
         && skeleton->getNumRigidBodyNodes() == 0
         && skeleton->getNumSoftBodyNodes() == 0
         && skeleton->getNumJoints() == 0 && skeleton->getNumDofs() == 0
         && skeleton->getNumTrees() == 0;
}

bool runRootTransactions()
{
  auto skeleton = dart::dynamics::Skeleton::create("soft-root-rejection");
  for (int iteration = 0; iteration < kIterationCount; ++iteration)
  {
    beginAllocationTracking();
    const bool rejected = rejectsWithExpectedMessage(
            [&] {
              (void)skeleton->createJointAndBodyNodePair<
                  dart::dynamics::WeldJoint,
                  dart::dynamics::SoftBodyNode>();
            },
            "root SoftBodyNode construction");
    const AllocationSummary allocation = endAllocationTracking();
    if (!rejected
        || !allocationTransactionIsBalanced(allocation, "root", iteration))
    {
      return false;
    }
    if (!rootStateIsEmpty(skeleton))
    {
      std::fprintf(
          stderr, "root rejection mutated skeleton state at iteration %d\n",
          iteration);
      return false;
    }
  }

  std::printf("SOFT_BODY_ROOT_REJECTIONS %d\n", kIterationCount);
  std::printf("SOFT_BODY_ROOT_ALLOCATION_TRANSACTIONS %d\n", kIterationCount);
  return true;
}

bool runChildTransactions()
{
  auto skeleton = dart::dynamics::Skeleton::create("soft-child-rejection");
  auto root = skeleton->createJointAndBodyNodePair<
      dart::dynamics::WeldJoint,
      InspectableBodyNode>();
  InspectableBodyNode* parent = root.second;
  if (root.first == nullptr || parent == nullptr)
  {
    std::fprintf(stderr, "rigid child-test parent construction failed\n");
    return false;
  }

  for (int iteration = 0; iteration < kIterationCount; ++iteration)
  {
    beginAllocationTracking();
    const bool rejected = rejectsWithExpectedMessage(
            [&] {
              (void)skeleton->createJointAndBodyNodePair<
                  dart::dynamics::WeldJoint,
                  dart::dynamics::SoftBodyNode>(parent);
            },
            "child SoftBodyNode construction");
    const AllocationSummary allocation = endAllocationTracking();
    if (!rejected
        || !allocationTransactionIsBalanced(allocation, "child", iteration))
    {
      return false;
    }
    if (skeleton->getNumBodyNodes() != 1
        || skeleton->getNumRigidBodyNodes() != 1
        || skeleton->getNumSoftBodyNodes() != 0
        || skeleton->getNumJoints() != 1 || skeleton->getNumDofs() != 0
        || skeleton->getNumTrees() != 1
        || parent->getNumChildBodyNodes() != 0
        || parent->getTrackedJacobianChildCount() != 0)
    {
      std::fprintf(
          stderr, "child rejection mutated skeleton state at iteration %d\n",
          iteration);
      return false;
    }

    // BodyNode tracks Jacobian children in a separate protected set. Removing a
    // half-constructed child after its dynamic type has unwound can clean the
    // public child list while leaving a dangling JacobianNode pointer behind.
    // The inspector above makes emptiness deterministic; the refresh below
    // additionally exercises the normal public Jacobian path after cleanup.
    const auto& cleanJacobian = parent->getJacobian();
    if (cleanJacobian.rows() != 6 || cleanJacobian.cols() != 0)
    {
      std::fprintf(
          stderr, "unexpected parent Jacobian before iteration %d\n", iteration);
      return false;
    }
    parent->dirtyJacobian();
    const auto& refreshedJacobian = parent->getJacobian();
    if (refreshedJacobian.rows() != 6 || refreshedJacobian.cols() != 0)
    {
      std::fprintf(
          stderr, "unexpected parent Jacobian after iteration %d\n", iteration);
      return false;
    }
  }

  std::printf("SOFT_BODY_CHILD_REJECTIONS %d\n", kIterationCount);
  std::printf("SOFT_BODY_CHILD_ALLOCATION_TRANSACTIONS %d\n", kIterationCount);
  return true;
}

} // namespace

namespace dart {
namespace dynamics {

// BodyNode.cpp carries scale support for ordinary MeshShape instances. These
// metadata-only definitions keep the injected archive link independent from
// MeshShape_ios.cpp.o; the transaction below never constructs a MeshShape.
const std::string& MeshShape::getStaticType()
{
  static const std::string type("MeshShape");
  return type;
}

void MeshShape::setScale(const Eigen::Vector3s& scale)
{
  mScale = scale;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;
  incrementVersion();
}

const Eigen::Vector3s& MeshShape::getScale() const
{
  return mScale;
}

// This deliberately injected backend rejection isolates the transaction in
// SoftBodyNode from the separate MeshShape implementation under test.
SoftMeshShape::SoftMeshShape(SoftBodyNode* softBodyNode)
  : Shape(SOFT_MESH),
    mSoftBodyNode(softBodyNode),
    mAssimpMesh(nullptr)
{
  throwInjectedMeshRejection();
}

SoftMeshShape::~SoftMeshShape() = default;

const std::string& SoftMeshShape::getType() const
{
  return getStaticType();
}

const std::string& SoftMeshShape::getStaticType()
{
  static const std::string type("SoftMeshShape");
  return type;
}

const aiMesh* SoftMeshShape::getAssimpMesh() const
{
  throwInjectedMeshRejection();
}

const SoftBodyNode* SoftMeshShape::getSoftBodyNode() const
{
  throwInjectedMeshRejection();
}

void SoftMeshShape::update()
{
  throwInjectedMeshRejection();
}

Eigen::Matrix3s SoftMeshShape::computeInertia(s_t) const
{
  throwInjectedMeshRejection();
}

ShapePtr SoftMeshShape::clone() const
{
  throwInjectedMeshRejection();
}

void SoftMeshShape::updateBoundingBox() const
{
  throwInjectedMeshRejection();
}

void SoftMeshShape::updateVolume() const
{
  throwInjectedMeshRejection();
}

void SoftMeshShape::_buildMesh()
{
  throwInjectedMeshRejection();
}

} // namespace dynamics
} // namespace dart

int main(int argc, char* argv[])
{
  if (argc == 2 && std::string(argv[1]) == "--positive-control")
  {
    beginAllocationTracking();
    void* leaked = ::operator new(kTrackedAllocationSize);
    asm volatile("" : : "r"(leaked) : "memory");
    const AllocationSummary allocation = endAllocationTracking();
    std::printf(
        "SOFT_BODY_ALLOCATION_POSITIVE_CONTROL allocated=%zu freed=%zu "
        "live=%zu\n",
        allocation.allocated,
        allocation.freed,
        allocation.live);
    if (allocation.allocated != 1 || allocation.freed != 0
        || allocation.live != 1)
    {
      return 30;
    }
    return 86;
  }
  if (argc == 2 && std::string(argv[1]) == "--root-only")
  {
    if (!runRootTransactions())
      return 10;
    std::puts("SOFT_BODY_ROOT_REJECTION_TRANSACTION_REACHED_EXIT");
    return 0;
  }
  if (argc == 2 && std::string(argv[1]) == "--child-only")
  {
    if (!runChildTransactions())
      return 11;
    std::puts("SOFT_BODY_CHILD_REJECTION_TRANSACTION_REACHED_EXIT");
    return 0;
  }
  if (argc != 1)
  {
    std::fprintf(stderr, "unexpected arguments\n");
    return 2;
  }

  if (!runRootTransactions())
    return 12;
  if (!runChildTransactions())
    return 13;

  std::puts("SOFT_BODY_MESH_REJECTION_TRANSACTION_REACHED_EXIT");
  return 0;
}
