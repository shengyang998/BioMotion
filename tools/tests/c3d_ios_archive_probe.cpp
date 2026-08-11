#include <cstdio>
#include <string>

#include "dart/biomechanics/C3DLoader.hpp"
#include "dart/biomechanics/OpenSimParser.hpp"

#if defined(BIOMOTION_C3D_FORCE_HEADER_ONLY) \
    || defined(BIOMOTION_C3D_FORCE_CONVENTION) \
    || defined(BIOMOTION_C3D_FORCE_PLATFORM) \
    || defined(BIOMOTION_C3D_FORCE_PLATFORMS)
#include "dart/biomechanics/C3DForcePlatforms.hpp"
#endif

#if defined(BIOMOTION_C3D_WEIGHTED_METHOD)

int main()
{
  dart::biomechanics::C3D c3d{};
  return c3d.getWeightedDistFromCoPToNearestMarker() == 0.0 ? 0 : 1;
}

#elif defined(BIOMOTION_C3D_LOADER)

int main()
{
  dart::biomechanics::C3DLoader* loader = nullptr;
  return loader == nullptr ? 0 : 1;
}

#elif defined(BIOMOTION_C3D_FORCE_CONVENTION)

int main()
{
  return dart::biomechanics::FORCE_PLATFORM_NUM_CONVENTIONS;
}

#elif defined(BIOMOTION_C3D_FORCE_PLATFORM)

int main()
{
  dart::biomechanics::ForcePlatform* platform = nullptr;
  return platform == nullptr ? 0 : 1;
}

#elif defined(BIOMOTION_C3D_FORCE_PLATFORMS)

int main()
{
  dart::biomechanics::ForcePlatforms* platforms = nullptr;
  return platforms == nullptr ? 0 : 1;
}

#elif defined(BIOMOTION_C3D_FORCE_HEADER_ONLY)

int main()
{
  return 0;
}

#else

namespace {

using OpenSimC3DConsumer
    = decltype(&dart::biomechanics::OpenSimParser::
                   loadMotAtLowestMarkerRMSERotation);
using ForcePlateCopy = dart::biomechanics::ForcePlate (*)(
    const dart::biomechanics::ForcePlate&);

OpenSimC3DConsumer volatile gOpenSimC3DConsumer
    = &dart::biomechanics::OpenSimParser::
        loadMotAtLowestMarkerRMSERotation;
ForcePlateCopy volatile gForcePlateCopy
    = &dart::biomechanics::ForcePlate::copyForcePlate;

} // namespace

int main()
{
  dart::biomechanics::C3D c3d{};
  c3d.framesPerSecond = 100;
  c3d.timestamps = {0.0};
  c3d.markers = {"RASI"};
  c3d.markerTimesteps.resize(1);
  c3d.markerTimesteps[0]["RASI"] = Eigen::Vector3s::Zero();
  c3d.shuffledMarkersMatrix = Eigen::MatrixXs::Zero(3, 1);
  c3d.shuffledMarkersMatrixMask = Eigen::MatrixXs::Ones(3, 1);
  c3d.dataRotation = Eigen::Matrix3s::Identity();

  dart::biomechanics::ForcePlate plate;
  plate.worldOrigin = Eigen::Vector3s::Zero();
  plate.timestamps = c3d.timestamps;
  plate.corners = {Eigen::Vector3s::Zero()};
  plate.centersOfPressure = {Eigen::Vector3s::Zero()};
  plate.moments = {Eigen::Vector3s::Zero()};
  plate.forces = {Eigen::Vector3s::Zero()};
  c3d.forcePlates.push_back(gForcePlateCopy(plate));

  if (gOpenSimC3DConsumer == nullptr || c3d.forcePlates.size() != 1
      || c3d.shuffledMarkersMatrix.rows() != 3
      || c3d.shuffledMarkersMatrixMask.sum() != 3.0
      || c3d.forcePlates[0].corners.size() != 1
      || c3d.forcePlates[0].centersOfPressure.size() != 1
      || c3d.forcePlates[0].moments.size() != 1
      || c3d.forcePlates[0].forces.size() != 1)
  {
    std::fprintf(stderr, "supported C3D consumer surface is unavailable\n");
    return 10;
  }

  std::puts("C3D_IOS_ARCHIVE_PROBE_PASS");
  return 0;
}

#endif
