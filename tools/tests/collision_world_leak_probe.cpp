#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>

#include "dart/simulation/World.hpp"

namespace {

constexpr char kExpectedMessage[] =
    "DARTCollisionDetector is unavailable in this iOS build because "
    "Assimp/libccd collision support is not linked.";

constexpr int kIterationCount = 32;

__attribute__((noinline)) void leakForPositiveControl()
{
  void* leaked = std::malloc(123);
  if (leaked == nullptr)
  {
    std::fprintf(stderr, "positive-control allocation failed\n");
    std::exit(30);
  }

  std::memset(leaked, 0xA5, 123);
  asm volatile("" : : "r"(leaked) : "memory");
}

template <typename Function>
bool rejectsWithExpectedMessage(Function&& function, const char* operation)
{
  try
  {
    function();
    std::fprintf(stderr, "%s returned normally\n", operation);
    return false;
  }
  catch (const std::runtime_error& error)
  {
    if (std::string(error.what()) == kExpectedMessage)
      return true;

    std::fprintf(
        stderr, "%s returned the wrong runtime_error: %s\n", operation,
        error.what());
    return false;
  }
  catch (const std::exception& error)
  {
    std::fprintf(
        stderr, "%s returned the wrong std::exception: %s\n", operation,
        error.what());
    return false;
  }
  catch (...)
  {
    std::fprintf(stderr, "%s returned a non-standard exception\n", operation);
    return false;
  }
}

} // namespace

int main(int argc, char* argv[])
{
  if (argc == 2 && std::string(argv[1]) == "--positive-control")
  {
    leakForPositiveControl();
    std::puts("WORLD_LEAK_POSITIVE_CONTROL_REACHED_EXIT");
    return 0;
  }

  if (argc != 1)
  {
    std::fprintf(stderr, "unexpected arguments\n");
    return 2;
  }

  for (int i = 0; i < kIterationCount; ++i)
  {
    if (!rejectsWithExpectedMessage(
            []() {
              dart::simulation::World world;
              (void)world;
            },
            "World()"))
    {
      return 10;
    }

    if (!rejectsWithExpectedMessage(
            []() {
              auto world = dart::simulation::World::create();
              (void)world;
            },
            "World::create()"))
    {
      return 11;
    }
  }

  std::puts("WORLD_COLLISION_REJECTION_NO_LEAK_PATH_REACHED_EXIT");
  return 0;
}
