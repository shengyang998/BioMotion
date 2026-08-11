#include <type_traits>

#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"

namespace {

template <typename T, typename = void>
struct IsComplete : std::false_type
{
};

template <typename T>
struct IsComplete<T, std::void_t<decltype(sizeof(T))>> : std::true_type
{
};

static_assert(
    !IsComplete<aiScene>::value,
    "the no-Assimp iOS MeshShape header must only forward-declare aiScene");
static_assert(
    !IsComplete<aiNode>::value,
    "the no-Assimp iOS MeshShape header must only forward-declare aiNode");
static_assert(
    !IsComplete<aiMesh>::value,
    "the no-Assimp iOS mesh headers must only forward-declare aiMesh");

} // namespace

int main()
{
  return 0;
}
