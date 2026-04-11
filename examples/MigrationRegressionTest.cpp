#include "../core/UnifiedModel.h"
#include "../service/accessors/RevolveAccessor.h"
#include "../service/builders/EndConditionBuilder.h"
#include "../service/builders/ReferenceBuilder.h"
#include "../service/builders/RevolveBuilder.h"
#include "../service/serialization/CADSerializer.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

using namespace CADExchange;
using namespace CADExchange::Accessor;
using namespace CADExchange::Builder;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kHalfPi = kPi * 0.5;

void Fail(const std::string &message) {
  std::cerr << "[FAIL] " << message << std::endl;
  std::exit(1);
}

void Expect(bool condition, const std::string &message) {
  if (!condition) {
    Fail(message);
  }
}

std::shared_ptr<CSketch> MakeSketch(const std::string &id,
                                    const std::string &name) {
  auto sketch = std::make_shared<CSketch>();
  sketch->featureID = id;
  sketch->featureName = name;
  sketch->referencePlane =
      RefPlaneBuilder(StandardID::PLANE_XY)
          .Origin(StandardID::kOrigin)
          .XDir(StandardID::kAxisX)
          .YDir(StandardID::kAxisY)
          .Normal(StandardID::kPlaneXYNormal)
          .Build();
  sketch->sketchCSys.origin = StandardID::kOrigin;
  sketch->sketchCSys.xDir = StandardID::kAxisX;
  sketch->sketchCSys.yDir = StandardID::kAxisY;
  sketch->sketchCSys.zDir = StandardID::kAxisZ;
  return sketch;
}

void TestRevolveBuilderIgnoresUnknownExtent() {
  UnifiedModel model(UnitType::METER, "builder-red-green");
  model.AddFeature(MakeSketch("SK-1", "Sketch1"));

  SweepExtent invalid;
  invalid.type = SweepExtent::Type::UNKNOWN;

  const std::string featureID =
      RevolveBuilder(model, "RevolveUnknownIgnored")
          .SetProfile("SK-1")
          .SetAxisExplicit(StandardID::kOrigin, StandardID::kAxisZ)
          .SetAngle(kHalfPi)
          .SetExtent1(invalid)
          .SetExtent2(invalid)
          .Build();

  RevolveAccessor revolve(model.GetFeature(featureID));
  Expect(revolve.IsValid(), "RevolveAccessor should be valid.");
  Expect(revolve.GetExtentType1() == SweepExtent::Type::VALUE,
         "UNKNOWN extent1 must not overwrite existing VALUE extent.");
  Expect(std::abs(revolve.GetExtentValue1() - kHalfPi) < 1e-9,
         "Extent1 value should remain pi/2 radians.");
  Expect(!revolve.HasExtent2(),
         "UNKNOWN extent2 must be ignored rather than stored.");

  const std::filesystem::path xmlPath =
      std::filesystem::path("E:/MyProject/tmp/cadexchange_revolve_radians.xml");
  std::filesystem::create_directories(xmlPath.parent_path());
  std::string errorMessage;
  Expect(SaveModel(model, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Saving revolve radian XML should succeed.");

  std::ifstream in(xmlPath, std::ios::binary);
  const std::string xml((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
  Expect(xml.find("Value=\"1.5707963267948966\"") != std::string::npos,
         "Serialized revolve XML should store radians, not degrees.");
}

void TestRevolveAccessorExposesSharedExtentFields() {
  UnifiedModel model(UnitType::METER, "accessor-shared-extent");
  model.AddFeature(MakeSketch("SK-2", "Sketch2"));

  auto planeRef =
      RefPlaneBuilder(StandardID::PLANE_XY)
          .Origin(StandardID::kOrigin)
          .XDir(StandardID::kAxisX)
          .YDir(StandardID::kAxisY)
          .Normal(StandardID::kPlaneXYNormal)
          .Build();

  SweepExtent extent1 = Extent::UpToEntity(planeRef, 1.5);
  extent1.isFlip = true;
  extent1.isFlipMaterialSide = true;

  SweepExtent extent2 = Extent::UpToEntity(planeRef, 2.5);
  extent2.isFlip = true;

  const std::string featureID =
      RevolveBuilder(model, "RevolveExtentFields")
          .SetProfile("SK-2")
          .SetAxisExplicit(StandardID::kOrigin, StandardID::kAxisZ)
          .SetExtent1(extent1)
          .SetExtent2(extent2)
          .Build();

  RevolveAccessor revolve(model.GetFeature(featureID));
  Expect(revolve.HasOffset1(), "Revolve extent1 should expose HasOffset1().");
  Expect(std::abs(revolve.GetExtentOffset1() - 1.5) < 1e-9,
         "Revolve extent1 offset should be readable.");
  Expect(revolve.IsFlip1(), "Revolve extent1 flip should be readable.");
  Expect(revolve.IsFlipMaterialSide1(),
         "Revolve extent1 material-side flip should be readable.");
  Expect(revolve.GetReference1().IsValid(),
         "Revolve extent1 reference should be readable.");

  Expect(revolve.HasExtent2(), "Revolve extent2 should exist.");
  Expect(revolve.HasOffset2(), "Revolve extent2 should expose HasOffset2().");
  Expect(std::abs(revolve.GetExtentOffset2() - 2.5) < 1e-9,
         "Revolve extent2 offset should be readable.");
  Expect(revolve.IsFlip2(), "Revolve extent2 flip should be readable.");
  Expect(!revolve.IsFlipMaterialSide2(),
         "Revolve extent2 material-side flip should default to false.");
  Expect(revolve.GetReference2().IsValid(),
         "Revolve extent2 reference should be readable.");
}

void TestLegacyRevolveXmlRejected() {
  const std::filesystem::path xmlPath =
      std::filesystem::path("E:/MyProject/tmp/cadexchange_legacy_revolve_rejected.xml");
  std::filesystem::create_directories(xmlPath.parent_path());

  const char *xml = R"LEGACYXML(<?xml version="1.0" encoding="UTF-8"?>
<UnifiedModel UnitSystem="Meter" ModelName="legacy-revolve" FeatureCount="2" SchemaVersion="1">
  <Feature ID="SK-3" Name="SketchLegacy" Suppressed="false" Type="Sketch">
    <ReferencePlane TargetFeatureID="STD_DATUM_XY" Origin="(0,0,0)" XDir="(1,0,0)" YDir="(0,1,0)" Normal="(0,0,1)" Type="Plane"/>
    <LocalCSys Origin="(0,0,0)" XDir="(1,0,0)" YDir="(0,1,0)" ZDir="(0,0,1)"/>
    <Segments/>
    <Constraints/>
  </Feature>
  <Feature ID="REV-1" Name="LegacyRevolve" Suppressed="false" Type="Revolve" ProfileSketchID="SK-3" Operation="BOSS" AngleKind="0" PrimaryAngle="180">
    <Axis RefLocalID="" Origin="(0,0,0)" Direction="(0,0,1)"/>
  </Feature>
</UnifiedModel>)LEGACYXML";

  {
    std::ofstream out(xmlPath, std::ios::binary);
    out << xml;
  }

  UnifiedModel model;
  std::string errorMessage;
  const bool ok =
      LoadModel(model, xmlPath, &errorMessage, SerializationFormat::TINYXML);
  Expect(!ok, "Legacy revolve XML without Extent1 should now be rejected.");
  Expect(errorMessage.find("[REVOLVE_002]") != std::string::npos,
         "Legacy revolve rejection should come from unknown extent type.");
}

void TestRevolveSketchAxisSerializesReferenceEntity() {
  UnifiedModel model(UnitType::METER, "revolve-sketch-axis-reference");
  auto sketch = MakeSketch("SK-AX", "SketchAxis");
  auto axisLine = std::make_shared<CSketchLine>();
  axisLine->localID = "L_1";
  axisLine->startPos = CPoint3D{0.0, 0.0, 0.0};
  axisLine->endPos = CPoint3D{0.0, 1.0, 0.0};
  sketch->segments.push_back(axisLine);
  model.AddFeature(sketch);

  const std::string revolveId =
      RevolveBuilder(model, "RevolveSketchAxis")
          .SetProfile("SK-AX")
          .SetAxisFromSketchLine("L_1")
          .SetAxisExplicit(StandardID::kOrigin, StandardID::kAxisZ)
          .SetAngle(kHalfPi)
          .Build();

  const std::filesystem::path xmlPath =
      std::filesystem::path("E:/MyProject/tmp/cadexchange_revolve_sketch_axis_ref.xml");
  std::filesystem::create_directories(xmlPath.parent_path());
  std::string errorMessage;
  Expect(SaveModel(model, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Saving revolve sketch-axis XML should succeed.");

  std::ifstream in(xmlPath, std::ios::binary);
  const std::string xml((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
  Expect(xml.find("<Axis RefLocalID=\"L_1\"") != std::string::npos,
         "Axis should keep RefLocalID for backward compatibility.");
  Expect(xml.find("Type=\"SketchSeg\"") != std::string::npos,
         "Axis should serialize a sketch-segment ReferenceEntity.");
  Expect(xml.find("ParentFeatureID=\"SK-AX\"") != std::string::npos,
         "Sketch-segment reference should carry parent sketch ID.");
  Expect(xml.find("SegmentLocalID=\"L_1\"") != std::string::npos,
         "Sketch-segment reference should carry segment local ID.");

  UnifiedModel loaded;
  errorMessage.clear();
  Expect(LoadModel(loaded, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Loading revolve sketch-axis XML should succeed.");
  RevolveAccessor loadedRevolve(loaded.GetFeature(revolveId));
  Expect(loadedRevolve.IsValid(), "Loaded revolve should be accessible.");
  Expect(loadedRevolve.GetAxisReferenceLocalID() == "L_1",
         "Loaded revolve should keep axis local ID from sketch-segment reference.");
}

} // namespace

int main() {
  TestRevolveBuilderIgnoresUnknownExtent();
  TestRevolveAccessorExposesSharedExtentFields();
  TestLegacyRevolveXmlRejected();
  TestRevolveSketchAxisSerializesReferenceEntity();
  std::cout << "[PASS] MigrationRegressionTest" << std::endl;
  return 0;
}
