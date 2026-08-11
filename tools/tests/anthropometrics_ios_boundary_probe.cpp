#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "dart/biomechanics/Anthropometrics.hpp"
#include "dart/biomechanics/IKErrorReport.hpp"
#include "dart/dynamics/FreeJoint.hpp"

using Anthropometrics = dart::biomechanics::Anthropometrics;

#if defined(BIOMOTION_ANTHRO_DEBUG_GUI)
using Forbidden = decltype(&Anthropometrics::debugToGUI);
Forbidden volatile gForbidden = &Anthropometrics::debugToGUI;
#elif defined(BIOMOTION_ANTHRO_DEBUG_VALUES)
using Forbidden = decltype(&Anthropometrics::debugValues);
Forbidden volatile gForbidden = &Anthropometrics::debugValues;
#elif defined(BIOMOTION_ANTHRO_GET_MARKERS)
using Forbidden = decltype(&Anthropometrics::getMarkers);
Forbidden volatile gForbidden = &Anthropometrics::getMarkers;
#elif defined(BIOMOTION_ANTHRO_MEASURE)
using Forbidden = decltype(&Anthropometrics::measure);
Forbidden volatile gForbidden = &Anthropometrics::measure;
#elif defined(BIOMOTION_ANTHRO_GET_PDF)
using Forbidden = decltype(&Anthropometrics::getPDF);
Forbidden volatile gForbidden = &Anthropometrics::getPDF;
#elif defined(BIOMOTION_ANTHRO_GET_LOG_PDF)
using Forbidden = decltype(&Anthropometrics::getLogPDF);
Forbidden volatile gForbidden = &Anthropometrics::getLogPDF;
#elif defined(BIOMOTION_ANTHRO_BODY_GRADIENT)
using Forbidden = decltype(&Anthropometrics::getGradientOfLogPDFWrtBodyScales);
Forbidden volatile gForbidden
    = &Anthropometrics::getGradientOfLogPDFWrtBodyScales;
#elif defined(BIOMOTION_ANTHRO_BODY_FD_GRADIENT)
using Forbidden
    = decltype(&Anthropometrics::finiteDifferenceGradientOfLogPDFWrtBodyScales);
Forbidden volatile gForbidden
    = &Anthropometrics::finiteDifferenceGradientOfLogPDFWrtBodyScales;
#elif defined(BIOMOTION_ANTHRO_GROUP_GRADIENT)
using Forbidden = decltype(&Anthropometrics::getGradientOfLogPDFWrtGroupScales);
Forbidden volatile gForbidden
    = &Anthropometrics::getGradientOfLogPDFWrtGroupScales;
#elif defined(BIOMOTION_ANTHRO_GROUP_FD_GRADIENT)
using Forbidden
    = decltype(&Anthropometrics::finiteDifferenceGradientOfLogPDFWrtGroupScales);
Forbidden volatile gForbidden
    = &Anthropometrics::finiteDifferenceGradientOfLogPDFWrtGroupScales;
#else

namespace {

using RetrieverPtr = dart::common::ResourceRetrieverPtr;
using Load = std::shared_ptr<Anthropometrics> (*)(
    const dart::common::Uri&, const RetrieverPtr&);
using AddMetric = void (Anthropometrics::*)(
    std::string,
    Eigen::VectorXs,
    std::string,
    Eigen::Vector3s,
    std::string,
    Eigen::Vector3s,
    Eigen::Vector3s);
using MetricNames = std::vector<std::string> (Anthropometrics::*)();
using SetDistribution = void (Anthropometrics::*)(
    std::shared_ptr<dart::math::MultivariateGaussian>);
using GetDistribution = std::shared_ptr<dart::math::MultivariateGaussian> (
    Anthropometrics::*)();
using Condition = std::shared_ptr<Anthropometrics> (Anthropometrics::*)(
    const std::map<std::string, ::s_t>&);
using SetPose = void (Anthropometrics::*)(
    std::shared_ptr<dart::dynamics::Skeleton>,
    const dart::biomechanics::AnthroMetric&);

Load volatile gLoad = &Anthropometrics::loadFromFile;
AddMetric volatile gAddMetric = &Anthropometrics::addMetric;
MetricNames volatile gMetricNames = &Anthropometrics::getMetricNames;
SetDistribution volatile gSetDistribution = &Anthropometrics::setDistribution;
GetDistribution volatile gGetDistribution = &Anthropometrics::getDistribution;
Condition volatile gCondition = &Anthropometrics::condition;
SetPose volatile gSetPose = &Anthropometrics::setSkelToMetricPose;

constexpr char kUnavailableMessage[]
    = "Anthropometric IK scoring is unavailable in this iOS build because "
      "mesh-backed anthropometric measurements are not linked.";

} // namespace

int main()
{
  if (gLoad == nullptr || gAddMetric == nullptr || gMetricNames == nullptr
      || gSetDistribution == nullptr || gGetDistribution == nullptr
      || gCondition == nullptr || gSetPose == nullptr)
  {
    std::fputs("supported anthropometric data surface is unavailable\n", stderr);
    return 1;
  }

  auto prior = std::make_shared<Anthropometrics>();
  prior->addMetric(
      "fixture",
      Eigen::VectorXs::Zero(0),
      "mesh-a",
      Eigen::Vector3s::Zero(),
      "mesh-b",
      Eigen::Vector3s::Zero());
  if (prior->getMetricNames() != std::vector<std::string>{"fixture"})
  {
    std::fputs("anthropometric metric data methods changed\n", stderr);
    return 2;
  }

  auto skeleton = dart::dynamics::Skeleton::create("probe");
  skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  Eigen::VectorXs seededPositions
      = Eigen::VectorXs::Zero(skeleton->getNumDofs());
  for (int i = 0; i < seededPositions.size(); i++)
  {
    seededPositions(i) = 0.01 * (i + 1);
  }
  skeleton->setPositions(seededPositions);

  Eigen::VectorXs seededBodyScales
      = Eigen::VectorXs::Ones(skeleton->getNumBodyNodes() * 3);
  seededBodyScales << 1.1, 0.9, 1.2;
  skeleton->setBodyScales(seededBodyScales);

  const Eigen::VectorXs originalPositions = skeleton->getPositions();
  const Eigen::VectorXs originalBodyScales = skeleton->getBodyScales();
  const Eigen::VectorXs originalGroupScales = skeleton->getGroupScales();
  if (originalPositions.size() == 0 || originalBodyScales.size() == 0
      || originalGroupScales.size() == 0)
  {
    std::fputs("skeleton mutation fixture has an empty state surface\n", stderr);
    return 3;
  }

  dart::dynamics::MarkerMap markers;
  Eigen::MatrixXs poses = Eigen::MatrixXs::Zero(skeleton->getNumDofs(), 1);
  std::vector<std::map<std::string, Eigen::Vector3s>> observations(1);

  dart::biomechanics::IKErrorReport supported(
      skeleton, markers, poses, observations);
  if (supported.rootMeanSquaredError.size() != 1)
  {
    std::fputs("prior-free IK error reporting is unavailable\n", stderr);
    return 4;
  }
  if (skeleton->getPositions() != originalPositions
      || skeleton->getBodyScales() != originalBodyScales
      || skeleton->getGroupScales() != originalGroupScales)
  {
    std::fputs("prior-free IK error reporting mutated skeleton state\n", stderr);
    return 5;
  }

  try
  {
    dart::biomechanics::IKErrorReport rejected(
        skeleton, markers, poses, observations, prior);
    (void)rejected;
  }
  catch (const std::runtime_error& error)
  {
    if (std::string(error.what()) != kUnavailableMessage)
    {
      std::fprintf(stderr, "unexpected rejection: %s\n", error.what());
      return 6;
    }
    if (skeleton->getPositions() != originalPositions
        || skeleton->getBodyScales() != originalBodyScales
        || skeleton->getGroupScales() != originalGroupScales)
    {
      std::fputs("rejected prior mutated skeleton state\n", stderr);
      return 7;
    }
    std::puts("ANTHROPOMETRICS_IOS_BOUNDARY_PASS");
    return 0;
  }
  catch (...)
  {
    std::fputs("anthropometric prior threw the wrong exception type\n", stderr);
    return 8;
  }

  std::fputs("anthropometric prior returned a plausible iOS score\n", stderr);
  return 9;
}

#endif

#if defined(BIOMOTION_ANTHRO_DEBUG_GUI) \
    || defined(BIOMOTION_ANTHRO_DEBUG_VALUES) \
    || defined(BIOMOTION_ANTHRO_GET_MARKERS) \
    || defined(BIOMOTION_ANTHRO_MEASURE) \
    || defined(BIOMOTION_ANTHRO_GET_PDF) \
    || defined(BIOMOTION_ANTHRO_GET_LOG_PDF) \
    || defined(BIOMOTION_ANTHRO_BODY_GRADIENT) \
    || defined(BIOMOTION_ANTHRO_BODY_FD_GRADIENT) \
    || defined(BIOMOTION_ANTHRO_GROUP_GRADIENT) \
    || defined(BIOMOTION_ANTHRO_GROUP_FD_GRADIENT)
int main()
{
  return gForbidden == nullptr ? 1 : 0;
}
#endif
