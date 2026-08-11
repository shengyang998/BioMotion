#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <tinyxml2.h>

#include "dart/biomechanics/OpenSimParser.hpp"
#include "dart/common/ResourceRetriever.hpp"
#include "dart/common/Uri.hpp"

namespace {

constexpr char kExpectedMessage[]
    = "OpenSim geometry loading is unavailable in this iOS build because "
      "Assimp mesh support is not linked; pass ignoreGeometry=true.";

class NoAccessResourceRetriever final : public dart::common::ResourceRetriever
{
public:
  bool exists(const dart::common::Uri&) override
  {
    ++mAccessCount;
    return false;
  }

  dart::common::ResourcePtr retrieve(const dart::common::Uri&) override
  {
    ++mAccessCount;
    return nullptr;
  }

  std::string readAll(const dart::common::Uri&) override
  {
    ++mAccessCount;
    throw std::runtime_error("unexpected OpenSim resource access");
  }

  std::string getFilePath(const dart::common::Uri&) override
  {
    ++mAccessCount;
    return "";
  }

  std::size_t accessCount() const
  {
    return mAccessCount;
  }

private:
  std::size_t mAccessCount = 0;
};

template <typename Function>
bool rejectsExactly(Function&& function, const char* label)
{
  try
  {
    function();
    std::cerr << label << " returned normally" << std::endl;
    return false;
  }
  catch (const std::runtime_error& error)
  {
    if (std::string(error.what()) != kExpectedMessage)
    {
      std::cerr << label << " returned the wrong diagnostic: " << error.what()
                << std::endl;
      return false;
    }
    return true;
  }
  catch (const std::exception& error)
  {
    std::cerr << label << " threw the wrong std::exception: " << error.what()
              << std::endl;
    return false;
  }
  catch (...)
  {
    std::cerr << label << " threw a non-standard exception" << std::endl;
    return false;
  }
}

} // namespace

int main()
{
  auto uriRetriever = std::make_shared<NoAccessResourceRetriever>();
  const bool uriRejected = rejectsExactly(
      [&uriRetriever]() {
        (void)dart::biomechanics::OpenSimParser::parseOsim(
            dart::common::Uri("file:///must-not-be-read/missing.osim"),
            "",
            false,
            uriRetriever);
      },
      "parseOsim(Uri)");

  tinyxml2::XMLDocument emptyDocument;
  auto documentRetriever = std::make_shared<NoAccessResourceRetriever>();
  const bool documentRejected = rejectsExactly(
      [&emptyDocument, &documentRetriever]() {
        (void)dart::biomechanics::OpenSimParser::parseOsim(
            emptyDocument, "empty.osim", "", false, documentRetriever);
      },
      "parseOsim(XMLDocument)");

  if (!uriRejected || !documentRejected || uriRetriever->accessCount() != 0
      || documentRetriever->accessCount() != 0)
  {
    std::cerr << "unexpected retriever access counts: uri="
              << uriRetriever->accessCount()
              << " document=" << documentRetriever->accessCount()
              << std::endl;
    return 1;
  }

  std::cout << "OPENSIM_GEOMETRY_IOS_BOUNDARY_PASS" << std::endl;
  return 0;
}
