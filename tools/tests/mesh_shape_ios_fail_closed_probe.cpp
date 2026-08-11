#include <cstdio>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include "dart/dynamics/MeshShape.hpp"

namespace {

constexpr char kExpectedMessage[]
    = "MeshShape is unavailable in this iOS build because Assimp mesh support "
      "is not linked.";

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
  using dart::common::ResourceRetrieverPtr;
  using dart::common::Uri;
  using dart::dynamics::MeshShape;
  using dart::dynamics::SharedMeshWrapper;

  const std::string path("unavailable.obj");
  const Uri uri("file:///unavailable.obj");
  const ResourceRetrieverPtr retriever;

  if (MeshShape::getStaticType() != "MeshShape")
  {
    std::fprintf(stderr, "WRONG_STATIC_TYPE MeshShape\n");
    ++gFailures;
  }

  expectUnavailable("SharedMeshWrapper constructor", [] {
    SharedMeshWrapper wrapper(nullptr);
    (void)wrapper;
  });
  expectUnavailable("MeshShape path constructor", [&] {
    MeshShape mesh(Eigen::Vector3s::Ones(), path, retriever);
    (void)mesh;
  });
  expectUnavailable("MeshShape wrapped-scene constructor", [&] {
    MeshShape mesh(
        Eigen::Vector3s::Ones(),
        std::shared_ptr<SharedMeshWrapper>(),
        uri,
        retriever);
    (void)mesh;
  });

  expectUnavailable("loadMesh(path)", [&] { (void)MeshShape::loadMesh(path); });
  expectUnavailable("loadMesh(path, retriever)", [&] {
    (void)MeshShape::loadMesh(path, retriever);
  });
  expectUnavailable("loadMesh(uri, retriever)", [&] {
    (void)MeshShape::loadMesh(uri, retriever);
  });

  // A rejected constructor makes instance methods unreachable to a conforming
  // consumer. The source contract pins getType/getStaticType/destructors as
  // the complete safe metadata whitelist and requires every other declaration
  // to use the shared rejection path, without manufacturing an object via UB.

  if (gFailures != 0)
  {
    std::fprintf(stderr, "MESH_FAIL_CLOSED_FAILURES %d\n", gFailures);
    return 1;
  }

  std::puts("MESH_SHAPE_IOS_FAIL_CLOSED_PASS");
  return 0;
}
