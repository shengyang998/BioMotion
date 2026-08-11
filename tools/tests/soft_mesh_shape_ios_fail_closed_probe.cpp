#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include "dart/dynamics/SoftMeshShape.hpp"

namespace {

constexpr char kExpectedMessage[]
    = "SoftMeshShape is unavailable in this iOS build because Assimp mesh "
      "support is not linked.";

int gFailures = 0;

template <typename Operation>
void expectUnavailable(const char* label, Operation&& operation)
{
  try
  {
    operation();
    std::fprintf(stderr, "SILENT_SUCCESS %s\n", label);
    ++gFailures;
  }
  catch (const std::runtime_error& error)
  {
    if (std::string(error.what()) != kExpectedMessage)
    {
      std::fprintf(
          stderr, "WRONG_RUNTIME_ERROR %s: %s\n", label, error.what());
      ++gFailures;
    }
  }
  catch (const std::exception& error)
  {
    std::fprintf(stderr, "WRONG_EXCEPTION %s: %s\n", label, error.what());
    ++gFailures;
  }
  catch (...)
  {
    std::fprintf(stderr, "WRONG_NON_STANDARD_EXCEPTION %s\n", label);
    ++gFailures;
  }
}

} // namespace

int main()
{
  using dart::dynamics::SoftMeshShape;

  if (SoftMeshShape::getStaticType() != "SoftMeshShape")
  {
    std::fprintf(stderr, "WRONG_STATIC_TYPE SoftMeshShape\n");
    ++gFailures;
  }

  expectUnavailable("SoftMeshShape constructor", [] {
    SoftMeshShape mesh(nullptr);
    (void)mesh;
  });

  // The remaining instance API cannot be reached after this rejection. The
  // source contract pins getType/getStaticType/destructors as the complete
  // safe metadata whitelist and rejects every other declaration; no null or
  // raw-storage object is used to manufacture a call.

  if (gFailures != 0)
  {
    std::fprintf(stderr, "SOFT_MESH_FAIL_CLOSED_FAILURES %d\n", gFailures);
    return 1;
  }

  std::puts("SOFT_MESH_SHAPE_IOS_FAIL_CLOSED_PASS");
  return 0;
}
