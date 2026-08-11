#include <cerrno>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

#include "dart/utils/XmlHelpers.hpp"

namespace {

int failures = 0;

void fail(const std::string& message)
{
  std::cerr << "FAIL " << message << std::endl;
  failures++;
}

template <typename T>
void expectEqual(const std::string& label, const T& actual, const T& expected)
{
  if (actual != expected)
  {
    std::cerr << "FAIL " << label << ": expected [" << expected
              << "], got [" << actual << "]" << std::endl;
    failures++;
  }
}

void expectNear(
    const std::string& label, double actual, double expected, double tolerance)
{
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
  {
    std::cerr << "FAIL " << label << ": expected " << expected << " +/- "
              << tolerance << ", got " << actual << std::endl;
    failures++;
  }
}

template <typename Callable>
void expectBadCast(const std::string& label, Callable&& callable)
{
  try
  {
    callable();
    fail(label + " did not throw");
  }
  catch (const std::bad_cast&)
  {
  }
  catch (...)
  {
    fail(label + " did not preserve the std::bad_cast base contract");
  }
}

template <typename Callable>
void expectRangeBadCast(const std::string& label, Callable&& callable)
{
  errno = EDOM;
  expectBadCast(label, std::forward<Callable>(callable));
  expectEqual(label + " leaves ERANGE", errno, ERANGE);
}

void checkScalarFormatting()
{
  using dart::utils::toString;

  expectEqual("false formatting", toString(false), std::string("0"));
  expectEqual("true formatting", toString(true), std::string("1"));
  expectEqual("signed integer formatting", toString(-42), std::string("-42"));
  expectEqual(
      "unsigned integer formatting", toString(42u), std::string("42"));
  expectEqual("character formatting", toString('x'), std::string("x"));
  expectEqual("exact float formatting", toString(1.25f), std::string("1.25"));
  expectEqual("exact double formatting", toString(1.25), std::string("1.25"));
  expectEqual(
      "round-trip float formatting",
      toString(0.1f),
      std::string("0.100000001"));
  expectEqual(
      "round-trip double formatting",
      toString(0.1),
      std::string("0.10000000000000001"));
}

void checkScalarParsing()
{
  using namespace dart::utils;

  expectEqual("lowercase true", toBool("true"), true);
  expectEqual("mixed-case true", toBool("TrUe"), true);
  expectEqual("uppercase false", toBool("FALSE"), false);
  expectEqual("numeric true", toBool("1"), true);
  expectEqual("numeric false", toBool("0"), false);
  expectEqual("boolean whitespace is not trimmed", toBool(" true "), false);

  expectEqual("signed integer parsing", toInt("-42"), -42);
  expectEqual("explicit plus integer parsing", toInt("+42"), 42);
  expectEqual("unsigned integer parsing", toUInt("42"), 42u);
  expectEqual(
      "negative text wraps for unsigned parsing",
      toUInt("-1"),
      std::numeric_limits<unsigned int>::max());
  expectEqual(
      "unsigned maximum parsing",
      toUInt("4294967295"),
      std::numeric_limits<unsigned int>::max());
  expectNear("float exponent parsing", toFloat("1.25e-2"), 0.0125, 1e-8);
  expectNear("double exponent parsing", toDouble("-1.25e2"), -125.0, 0.0);
  expectEqual("character parsing", toChar("x"), 'x');
  expectEqual("space character parsing", toChar(" "), ' ');
  expectEqual("tab character parsing", toChar("\t"), '\t');

  expectBadCast("integer leading whitespace", [] { (void)toInt(" 42"); });
  expectBadCast("integer trailing whitespace", [] { (void)toInt("42 "); });
  expectBadCast("integer trailing text", [] { (void)toInt("42x"); });
  expectBadCast("invalid integer exception base", [] { (void)toInt("x"); });
  expectBadCast("integer overflow", [] { (void)toInt("999999999999999"); });
  expectBadCast("unsigned positive overflow", [] {
    (void)toUInt("4294967296");
  });
  expectBadCast("unsigned negative overflow", [] {
    (void)toUInt("-4294967296");
  });
  expectBadCast("unsigned double sign", [] { (void)toUInt("-+1"); });
  expectBadCast("unsigned separated sign", [] { (void)toUInt("- 1"); });
  expectBadCast("unsigned double negative", [] { (void)toUInt("--0"); });
  expectBadCast("double leading whitespace", [] {
    (void)toDouble(" 1.25");
  });
  expectBadCast("double trailing whitespace", [] {
    (void)toDouble("1.25 ");
  });
  expectBadCast("multi-character char", [] { (void)toChar("xy"); });

  const double nan = toDouble("nan");
  if (!std::isnan(nan))
    fail("nan parsing");
  const double infinity = toDouble("inf");
  if (!std::isinf(infinity) || infinity < 0.0)
    fail("positive infinity parsing");
  const double negativeInfinity = toDouble("-infinity");
  if (!std::isinf(negativeInfinity) || negativeInfinity > 0.0)
    fail("negative infinity parsing");
  if (!std::isnan(toDouble("nan(payload)")))
    fail("nan payload parsing");
  expectNear("hex float parsing", toDouble("0x1p2"), 4.0, 0.0);

  errno = EDOM;
  (void)toInt("1");
  expectEqual("integer success preserves errno", errno, EDOM);
  (void)toUInt("1");
  expectEqual("unsigned success preserves errno", errno, EDOM);
  (void)toDouble("1.25");
  expectEqual("double success preserves errno", errno, EDOM);
  (void)toDouble("nan");
  expectEqual("nan success preserves errno", errno, EDOM);
  expectRangeBadCast("float overflow", [] { (void)toFloat("1e5000"); });
  expectRangeBadCast("double overflow", [] { (void)toDouble("1e5000"); });
  expectRangeBadCast("float underflow", [] { (void)toFloat("1e-5000"); });
  expectRangeBadCast("double underflow", [] {
    (void)toDouble("1e-5000");
  });

  const std::vector<float> floatRoundTrips = {
      0.0f,
      -0.0f,
      std::numeric_limits<float>::min(),
      std::numeric_limits<float>::max()};
  for (std::size_t i = 0; i < floatRoundTrips.size(); i++)
  {
    const float value = floatRoundTrips[i];
    const float roundTrip = toFloat(toString(value));
    if (roundTrip != value || std::signbit(roundTrip) != std::signbit(value))
      fail("float round trip " + std::to_string(i));
  }

  const std::vector<double> doubleRoundTrips = {
      0.0,
      -0.0,
      std::numeric_limits<double>::min(),
      std::numeric_limits<double>::max()};
  for (std::size_t i = 0; i < doubleRoundTrips.size(); i++)
  {
    const double value = doubleRoundTrips[i];
    const double roundTrip = toDouble(toString(value));
    if (roundTrip != value || std::signbit(roundTrip) != std::signbit(value))
      fail("double round trip " + std::to_string(i));
  }
  expectRangeBadCast("float subnormal parsing reports range error", [] {
    (void)toFloat(toString(std::numeric_limits<float>::denorm_min()));
  });
  expectRangeBadCast("double subnormal parsing reports range error", [] {
    (void)toDouble(toString(std::numeric_limits<double>::denorm_min()));
  });
}

void checkVectorAndTransformParsing()
{
  using namespace dart::utils;

  const Eigen::Vector2s vector2 = toVector2s("\t1.25    -2.5\n");
  expectNear("Vector2s first value", vector2[0], 1.25, 0.0);
  expectNear("Vector2s second value", vector2[1], -2.5, 0.0);
  expectEqual(
      "Vector2s formatting", toString(vector2), std::string("1.25 -2.5"));

  const Eigen::Vector3s vector3 = toVector3s("1  2   3");
  expectNear("Vector3s first value", vector3[0], 1.0, 0.0);
  expectNear("Vector3s second value", vector3[1], 2.0, 0.0);
  expectNear("Vector3s third value", vector3[2], 3.0, 0.0);
  expectEqual(
      "Vector3s formatting", toString(vector3), std::string("1 2 3"));

  const Eigen::Vector3i vector3i = toVector3i("-1 0 2");
  expectEqual("Vector3i first value", vector3i[0], -1);
  expectEqual("Vector3i second value", vector3i[1], 0);
  expectEqual("Vector3i third value", vector3i[2], 2);
  expectEqual(
      "Vector3i formatting", toString(vector3i), std::string("-1  0  2"));

  const Eigen::Vector6s vector6 = toVector6s("1 2 3 4 5 6");
  expectNear("Vector6s final value", vector6[5], 6.0, 0.0);
  expectEqual(
      "Vector6s formatting", toString(vector6), std::string("1 2 3 4 5 6"));

  const Eigen::VectorXs dynamic = toVectorXs("1 2 3 4");
  expectEqual("VectorXs size", dynamic.size(), Eigen::Index(4));
  expectNear("VectorXs final value", dynamic[3], 4.0, 0.0);
  expectEqual(
      "VectorXs formatting", toString(dynamic), std::string("1 2 3 4"));

  const Eigen::Isometry3s transform = toIsometry3s("1 2 3 0 0 0");
  expectNear("transform x", transform.translation()[0], 1.0, 0.0);
  expectNear("transform y", transform.translation()[1], 2.0, 0.0);
  expectNear("transform z", transform.translation()[2], 3.0, 0.0);
  expectNear(
      "transform identity rotation",
      (transform.linear() - Eigen::Matrix3s::Identity()).norm(),
      0.0,
      1e-12);
  const Eigen::Isometry3s transformRoundTrip = toIsometry3s(toString(transform));
  expectNear(
      "transform formatting round trip",
      (transformRoundTrip.matrix() - transform.matrix()).norm(),
      0.0,
      1e-12);

  const Eigen::Isometry3s extrinsic
      = toIsometry3sWithExtrinsicRotation("1 2 3 0 0 0");
  expectNear(
      "extrinsic identity rotation",
      (extrinsic.linear() - Eigen::Matrix3s::Identity()).norm(),
      0.0,
      1e-12);
}

void checkXMLWrappers()
{
  tinyxml2::XMLDocument document;
  const tinyxml2::XMLError parseResult = document.Parse(
      "<root enabled=\"true\" count=\"7\" point=\"1 2 3\">"
      "<name>sample</name><enabled>TrUe</enabled><count>-7</count>"
      "<weight>1.25e2</weight><point>1 2 3</point></root>");
  expectEqual(
      "fixture XML parse", parseResult, tinyxml2::XML_SUCCESS);
  const tinyxml2::XMLElement* root = document.FirstChildElement("root");
  if (root == nullptr)
  {
    fail("fixture root missing");
    return;
  }

  using namespace dart::utils;
  expectEqual("element presence", hasElement(root, "name"), true);
  expectEqual("missing element", hasElement(root, "missing"), false);
  expectEqual(
      "string child value", getValueString(root, "name"), std::string("sample"));
  expectEqual("bool child value", getValueBool(root, "enabled"), true);
  expectEqual("int child value", getValueInt(root, "count"), -7);
  expectNear("double child value", getValueDouble(root, "weight"), 125.0, 0.0);
  const Eigen::Vector3s point = getValueVector3s(root, "point");
  expectNear("vector child value", point[2], 3.0, 0.0);

  expectEqual("attribute presence", hasAttribute(root, "enabled"), true);
  expectEqual("missing attribute", hasAttribute(root, "missing"), false);
  expectEqual("bool attribute", getAttributeBool(root, "enabled"), true);
  expectEqual("int attribute", getAttributeInt(root, "count"), 7);
  const Eigen::Vector3s attributePoint
      = getAttributeVector3s(root, "point");
  expectNear("vector attribute", attributePoint[1], 2.0, 0.0);
}

} // namespace

int main()
{
  checkScalarFormatting();
  checkScalarParsing();
  checkVectorAndTransformParsing();
  checkXMLWrappers();

  if (failures != 0)
  {
    std::cerr << failures << " characterization failure(s)" << std::endl;
    return 1;
  }

  std::cout << "XML_HELPERS_CHARACTERIZATION_PASS" << std::endl;
  return 0;
}
