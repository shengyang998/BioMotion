#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include "dart/collision/CollisionDetector.hpp"

namespace {

constexpr char kExpectedMessage[] =
    "DARTCollisionDetector is unavailable in this iOS build because "
    "Assimp/libccd collision support is not linked.";

} // namespace

int main()
{
  auto* factory = dart::collision::CollisionDetector::getFactory();
  if (factory == nullptr)
  {
    std::fprintf(stderr, "factory singleton is nullptr\n");
    return 10;
  }

  const std::string key("dart");
  if (!factory->canCreate(key))
  {
    std::fprintf(stderr, "factory does not register the dart key\n");
    return 11;
  }

  try
  {
    auto detector = factory->create(key);
    if (detector == nullptr)
    {
      std::fprintf(stderr, "factory returned nullptr\n");
      return 12;
    }

    std::fprintf(stderr, "factory returned a usable detector\n");
    return 13;
  }
  catch (const std::runtime_error& error)
  {
    if (std::string(error.what()) != kExpectedMessage)
    {
      std::fprintf(stderr, "wrong runtime_error: %s\n", error.what());
      return 14;
    }

    std::puts("ARCHIVE_FACTORY_PROBE_PASS");
    return 0;
  }
  catch (const std::exception& error)
  {
    std::fprintf(stderr, "wrong std::exception: %s\n", error.what());
    return 15;
  }
  catch (...)
  {
    std::fprintf(stderr, "wrong non-standard exception\n");
    return 16;
  }
}
