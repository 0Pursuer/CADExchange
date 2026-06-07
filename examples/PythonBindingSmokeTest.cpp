#include "../service/accessors/ModelAccessor.h"
#include "../service/api/py/python_api.h"

#include <filesystem>
#include <iostream>

using namespace CADExchange;
using namespace CADExchange::Accessor;
using namespace CADExchange::PythonApi;

namespace {

void Expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "[FAIL] " << message << std::endl;
    std::exit(1);
  }
}

} // namespace

int main() {
  const std::filesystem::path xmlPath =
      std::filesystem::path("tmp") / "cadexchange_chamfer_roundtrip.xml";
  Expect(std::filesystem::exists(xmlPath),
         "Expected MigrationRegressionTest XML fixture to exist.");

  const ModelAccessor model = LoadModelAccessor(xmlPath.string());
  Expect(model.IsValid(), "ModelAccessor should be valid after loading fixture.");
  Expect(model.GetFeatureCount() > 0, "ModelAccessor should expose features.");

  const auto features = GetAllFeatures(model);
  Expect(!features.empty(), "GetAllFeatures should return bound feature list.");

  bool foundChamfer = false;
  for (const auto &feature : features) {
    if (feature.GetFeatureType() == FeatureType::Chamfer) {
      const ChamferAccessor chamfer(feature.GetRaw());
      Expect(chamfer.IsValid(), "Chamfer feature should round-trip via accessors.");
      foundChamfer = true;
      break;
    }
  }

  Expect(foundChamfer, "Fixture should contain a chamfer feature.");
  std::cout << "[PASS] PythonBindingSmokeTest" << std::endl;
  return 0;
}
