#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "dart/biomechanics/OpenSimParser.hpp"

#if defined(BIOMOTION_OPEN_SIM_TRANSLATE_MARKERS)

namespace {

using Utility = decltype(
    &dart::biomechanics::OpenSimParser::translateOsimMarkers);
Utility volatile gUtility
    = &dart::biomechanics::OpenSimParser::translateOsimMarkers;

} // namespace

int main()
{
  return gUtility == nullptr ? 1 : 0;
}

#elif defined(BIOMOTION_OPEN_SIM_CONVERT_SDF)

namespace {

using Utility
    = decltype(&dart::biomechanics::OpenSimParser::convertOsimToSDF);
Utility volatile gUtility
    = &dart::biomechanics::OpenSimParser::convertOsimToSDF;

} // namespace

int main()
{
  return gUtility == nullptr ? 1 : 0;
}

#elif defined(BIOMOTION_OPEN_SIM_CONVERT_MJCF)

namespace {

using Utility
    = decltype(&dart::biomechanics::OpenSimParser::convertOsimToMJCF);
Utility volatile gUtility
    = &dart::biomechanics::OpenSimParser::convertOsimToMJCF;

} // namespace

int main()
{
  return gUtility == nullptr ? 1 : 0;
}

#else

namespace {

using Parser = dart::biomechanics::OpenSimParser;
using RetrieverPtr = dart::common::ResourceRetrieverPtr;
using ParseUri = dart::biomechanics::OpenSimFile (*)(
    const dart::common::Uri&, std::string, bool, const RetrieverPtr&);
using ParseDocument = dart::biomechanics::OpenSimFile (*)(
    tinyxml2::XMLDocument&,
    std::string,
    std::string,
    bool,
    const RetrieverPtr&);
using LoadTRC = dart::biomechanics::OpenSimTRC (*)(
    const dart::common::Uri&, const RetrieverPtr&);
using LoadMot = dart::biomechanics::OpenSimMot (*)(
    std::shared_ptr<dart::dynamics::Skeleton>,
    const dart::common::Uri&,
    Eigen::Matrix3s,
    int,
    const RetrieverPtr&);
using LoadGRF = std::vector<dart::biomechanics::ForcePlate> (*)(
    const dart::common::Uri&,
    const std::vector<double>&,
    const RetrieverPtr&);
using C3DConsumer = dart::biomechanics::OpenSimMot (*)(
    dart::biomechanics::OpenSimFile&,
    const dart::common::Uri&,
    dart::biomechanics::C3D&,
    int,
    const RetrieverPtr&);

constexpr char kGeometryMessage[]
    = "OpenSim geometry loading is unavailable in this iOS build because "
      "Assimp mesh support is not linked; pass ignoreGeometry=true.";

ParseUri volatile gParseUri = static_cast<ParseUri>(&Parser::parseOsim);
ParseDocument volatile gParseDocument
    = static_cast<ParseDocument>(&Parser::parseOsim);
LoadTRC volatile gLoadTRC = &Parser::loadTRC;
LoadMot volatile gLoadMot = &Parser::loadMot;
LoadGRF volatile gLoadGRF = &Parser::loadGRF;
C3DConsumer volatile gC3DConsumer
    = &Parser::loadMotAtLowestMarkerRMSERotation;

template <typename Function>
bool rejectsGeometry(Function&& function)
{
  try
  {
    function();
    return false;
  }
  catch (const std::runtime_error& error)
  {
    return std::string(error.what()) == kGeometryMessage;
  }
  catch (...)
  {
    return false;
  }
}

} // namespace

int main()
{
  const bool uriRejected = rejectsGeometry([]() {
    (void)gParseUri(
        dart::common::Uri("file:///must-not-be-read/missing.osim"),
        "",
        false,
        nullptr);
  });
  tinyxml2::XMLDocument emptyDocument;
  const bool documentRejected = rejectsGeometry([&emptyDocument]() {
    (void)gParseDocument(
        emptyDocument, "empty.osim", "", false, nullptr);
  });

  if (!uriRejected || !documentRejected || gParseUri == nullptr
      || gParseDocument == nullptr || gLoadTRC == nullptr
      || gLoadMot == nullptr || gLoadGRF == nullptr
      || gC3DConsumer == nullptr)
  {
    std::fputs(
        "supported OpenSim parser/C3D surface is unavailable\n", stderr);
    return 1;
  }

  std::puts("OPENSIM_UTILS_IOS_BOUNDARY_PASS");
  return 0;
}

#endif
