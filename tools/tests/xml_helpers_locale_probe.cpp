#include <cmath>
#include <iostream>
#include <locale>
#include <stdexcept>
#include <string>

#include "dart/math/Geometry.hpp"
#include "dart/utils/XmlHelpers.hpp"

namespace {

class HostileNumpunct : public std::numpunct<char>
{
protected:
  char do_decimal_point() const override
  {
    return ',';
  }

  char do_thousands_sep() const override
  {
    return '_';
  }

  std::string do_grouping() const override
  {
    return "\3";
  }
};

int failures = 0;

void fail(const std::string& message)
{
  std::cerr << "FAIL " << message << std::endl;
  failures++;
}

void expectEqual(
    const std::string& label,
    const std::string& actual,
    const std::string& expected)
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
    fail(label + " did not throw through std::bad_cast");
  }
}

} // namespace

int main()
{
  using namespace dart::utils;

  const std::locale originalLocale = std::locale();
  std::locale::global(
      std::locale(std::locale::classic(), new HostileNumpunct()));

  expectEqual(
      "double formatting ignores global locale",
      toString(1234.5),
      "1234.5");
  expectNear(
      "decimal-dot parsing ignores global locale",
      toDouble("1234.5"),
      1234.5,
      0.0);
  expectBadCast("decimal-comma XML is rejected", [] {
    (void)toDouble("1234,5");
  });
  expectBadCast("grouped locale-specific XML is rejected", [] {
    (void)toDouble("1_234,5");
  });

  const Eigen::Vector3s vector(1234.5, 2.5, -3.5);
  const std::string encodedVector = toString(vector);
  if (encodedVector.find(',') != std::string::npos
      || encodedVector.find('_') != std::string::npos)
    fail("Eigen formatting emitted locale punctuation");
  const Eigen::Vector3s roundTripVector = toVector3s(encodedVector);
  expectNear(
      "Eigen classic-locale round trip",
      (roundTripVector - vector).norm(),
      0.0,
      0.0);
  const Eigen::Vector3s parsedVector = toVector3s("1234.5 2.5 -3.5");
  expectNear("Eigen dot parse x", parsedVector[0], 1234.5, 0.0);
  expectNear("Eigen dot parse y", parsedVector[1], 2.5, 0.0);
  expectNear("Eigen dot parse z", parsedVector[2], -3.5, 0.0);

  Eigen::Isometry3s transform = Eigen::Isometry3s::Identity();
  transform.translation() = vector;
  transform.linear()
      = dart::math::eulerXYZToMatrix(Eigen::Vector3s(0.125, -0.25, 0.375));
  const std::string encodedTransform = toString(transform);
  if (encodedTransform.find(',') != std::string::npos
      || encodedTransform.find('_') != std::string::npos)
    fail("transform formatting emitted locale punctuation");
  const Eigen::Isometry3s decodedTransform = toIsometry3s(encodedTransform);
  expectNear(
      "transform classic-locale round trip",
      (decodedTransform.matrix() - transform.matrix()).norm(),
      0.0,
      2e-6);

  std::locale::global(originalLocale);

  if (failures != 0)
  {
    std::cerr << failures << " locale contract failure(s)" << std::endl;
    return 1;
  }

  std::cout << "XML_HELPERS_CLASSIC_LOCALE_PASS" << std::endl;
  return 0;
}
