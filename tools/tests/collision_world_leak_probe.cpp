#include <cerrno>
#include <cstddef>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <pthread.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

#include "dart/simulation/World.hpp"

namespace {

constexpr char kExpectedMessage[] =
    "DARTCollisionDetector is unavailable in this iOS build because "
    "Assimp/libccd collision support is not linked.";

constexpr int kIterationCount = 32;
constexpr std::size_t kDisposableStackSize = 1024 * 1024;

volatile std::sig_atomic_t gTerminationRequested = 0;

extern "C" void handleTerminationSignal(int)
{
  gTerminationRequested = 1;
}

__attribute__((noinline)) bool leakForPositiveControl()
{
  void* leaked = std::malloc(123);
  if (leaked == nullptr)
  {
    std::fprintf(stderr, "positive-control allocation failed\n");
    return false;
  }

  std::memset(leaked, 0xA5, 123);
  asm volatile("" : "+r"(leaked) : : "memory");
  leaked = nullptr;
  asm volatile("" : "+r"(leaked) : : "memory");
  return true;
}

// The leaks tool treats stale pointer-shaped values on the stack as roots. The
// positive control deliberately drops ownership, then overwrites the retired
// helper frame so its allocation is classified as unreachable rather than as a
// reachable allocation.
__attribute__((noinline)) void scrubRetiredStackFrames()
{
  volatile unsigned char scratch[64 * 1024];
  for (std::size_t i = 0; i < sizeof(scratch); ++i)
    scratch[i] = static_cast<unsigned char>(i);

  asm volatile("" : : "r"(&scratch[0]) : "memory");
}

using IsolatedWorkload = int (*)();

struct IsolatedWorkloadContext
{
  IsolatedWorkload workload;
  int status;
};

int runPositiveControlWorkload()
{
  return leakForPositiveControl() ? 0 : 30;
}

void* runIsolatedWorkload(void* rawContext)
{
  auto* context = static_cast<IsolatedWorkloadContext*>(rawContext);
  context->status = context->workload();
  scrubRetiredStackFrames();
  return nullptr;
}

bool runOnDisposableStack(
    const char* workloadName, IsolatedWorkload workload, int& workloadStatus)
{
  // After join(), unmapping the worker stack removes every retired
  // register/stack root before leaks snapshots the still-resident main thread.
  void* threadStack = mmap(
      nullptr, kDisposableStackSize, PROT_READ | PROT_WRITE,
      MAP_PRIVATE | MAP_ANON, -1, 0);
  if (threadStack == MAP_FAILED)
  {
    std::fprintf(
        stderr, "%s stack mmap failed: %s\n", workloadName,
        std::strerror(errno));
    return false;
  }

  pthread_attr_t attributes;
  int status = pthread_attr_init(&attributes);
  if (status != 0)
  {
    std::fprintf(
        stderr, "%s pthread_attr_init failed: %s\n", workloadName,
        std::strerror(status));
    munmap(threadStack, kDisposableStackSize);
    return false;
  }

  status = pthread_attr_setstack(
      &attributes, threadStack, kDisposableStackSize);
  if (status != 0)
  {
    std::fprintf(
        stderr, "%s pthread_attr_setstack failed: %s\n", workloadName,
        std::strerror(status));
    pthread_attr_destroy(&attributes);
    munmap(threadStack, kDisposableStackSize);
    return false;
  }

  IsolatedWorkloadContext context{workload, -1};
  pthread_t thread;
  status = pthread_create(&thread, &attributes, runIsolatedWorkload, &context);
  pthread_attr_destroy(&attributes);
  if (status != 0)
  {
    std::fprintf(
        stderr, "%s pthread_create failed: %s\n", workloadName,
        std::strerror(status));
    munmap(threadStack, kDisposableStackSize);
    return false;
  }

  status = pthread_join(thread, nullptr);
  if (status != 0)
  {
    std::fprintf(
        stderr, "%s pthread_join failed: %s\n", workloadName,
        std::strerror(status));
    return false;
  }

  if (munmap(threadStack, kDisposableStackSize) != 0)
  {
    std::fprintf(
        stderr, "%s stack munmap failed: %s\n", workloadName,
        std::strerror(errno));
    return false;
  }

  workloadStatus = context.status;
  return true;
}

bool reportReadyAndWait(const char* sentinel)
{
  struct sigaction action {};
  action.sa_handler = handleTerminationSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  if (sigaction(SIGTERM, &action, nullptr) != 0
      || sigaction(SIGINT, &action, nullptr) != 0)
  {
    std::perror("sigaction");
    return false;
  }

  if (std::puts(sentinel) == EOF || std::fflush(stdout) != 0)
  {
    std::perror("ready sentinel");
    return false;
  }

  while (gTerminationRequested == 0)
  {
    if (usleep(50'000) != 0 && errno != EINTR)
    {
      std::perror("usleep");
      return false;
    }
  }

  return true;
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

int runWorldRejectionWorkload()
{
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

  return 0;
}

} // namespace

int main(int argc, char* argv[])
{
  if (argc == 2 && std::strcmp(argv[1], "--positive-control") == 0)
  {
    int workloadStatus = 0;
    if (!runOnDisposableStack(
            "positive-control", runPositiveControlWorkload, workloadStatus))
      return 32;
    if (workloadStatus != 0)
      return workloadStatus;
    return reportReadyAndWait("WORLD_LEAK_POSITIVE_CONTROL_READY") ? 0 : 31;
  }

  if (argc != 1)
  {
    std::fprintf(stderr, "unexpected arguments\n");
    return 2;
  }

  int workloadStatus = 0;
  if (!runOnDisposableStack(
          "world-rejection", runWorldRejectionWorkload, workloadStatus))
    return 13;
  if (workloadStatus != 0)
    return workloadStatus;

  return reportReadyAndWait("WORLD_COLLISION_REJECTION_NO_LEAK_READY") ? 0
                                                                       : 12;
}
