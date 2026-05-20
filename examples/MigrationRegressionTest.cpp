#include "../core/UnifiedModel.h"
#include "../service/accessors/RevolveAccessor.h"
#include "../service/accessors/SketchAccessor.h"
#include "../service/accessors/SweepAccessor.h"
#include "../service/accessors/FilletAccessor.h"
#include "../service/accessors/ChamferAccessor.h"
#include "../service/builders/EndConditionBuilder.h"
#include "../service/builders/ExtrudeBuilder.h"
#include "../service/builders/ReferenceBuilder.h"
#include "../service/builders/RevolveBuilder.h"
#include "../service/builders/SketchBuilder.h"
#include "../service/builders/SweepBuilder.h"
#include "../service/builders/FilletBuilder.h"
#include "../service/builders/ChamferBuilder.h"
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

void AddSimpleProfileSegment(const std::shared_ptr<CSketch> &sketch,
                             const std::string &localID = "L_PROFILE") {
  auto line = std::make_shared<CSketchLine>();
  line->localID = localID;
  line->startPos = CPoint3D{0.0, 0.0, 0.0};
  line->endPos = CPoint3D{1.0, 0.0, 0.0};
  sketch->segments.push_back(line);
}

std::string MakeExtrudeFromSketch(UnifiedModel &model,
                                  const std::string &sketchID,
                                  const std::string &name) {
  return ExtrudeBuilder(model, name)
      .SetProfile(sketchID)
      .SetDirection(CVector3D{0.0, 0.0, 1.0})
      .SetEndCondition1(EndCondition::Blind(0.02))
      .Build();
}

void TestRevolveBuilderIgnoresUnknownExtent() {
  UnifiedModel model(UnitType::METER, "builder-red-green");
  auto sketch = MakeSketch("SK-1", "Sketch1");
  AddSimpleProfileSegment(sketch);
  model.AddFeature(sketch);

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
      std::filesystem::path("tmp") / "cadexchange_revolve_radians.xml";
  std::filesystem::create_directories(xmlPath.parent_path());
  std::string errorMessage;
  Expect(SaveModel(model, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Saving revolve radian XML should succeed: " + errorMessage);

  std::ifstream in(xmlPath, std::ios::binary);
  const std::string xml((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
  Expect(xml.find("Value=\"1.5707963267948966\"") != std::string::npos,
         "Serialized revolve XML should store radians, not degrees.");
}

void TestRevolveAccessorExposesSharedExtentFields() {
  UnifiedModel model(UnitType::METER, "accessor-shared-extent");
  auto sketch = MakeSketch("SK-2", "Sketch2");
  AddSimpleProfileSegment(sketch);
  model.AddFeature(sketch);

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
      std::filesystem::path("tmp") / "cadexchange_legacy_revolve_rejected.xml";
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
  auto profileSketch = MakeSketch("SK-PROFILE", "SketchProfile");
  auto profileLine = std::make_shared<CSketchLine>();
  profileLine->localID = "P_1";
  profileLine->startPos = CPoint3D{0.0, 0.0, 0.0};
  profileLine->endPos = CPoint3D{1.0, 0.0, 0.0};
  profileSketch->segments.push_back(profileLine);
  model.AddFeature(profileSketch);

  auto sketch = MakeSketch("SK-AX", "SketchAxis");
  auto axisLine = std::make_shared<CSketchLine>();
  axisLine->localID = "L_1";
  axisLine->startPos = CPoint3D{0.0, 0.0, 0.0};
  axisLine->endPos = CPoint3D{0.0, 1.0, 0.0};
  sketch->segments.push_back(axisLine);
  model.AddFeature(sketch);

  const std::string revolveId =
      RevolveBuilder(model, "RevolveSketchAxis")
          .SetProfile("SK-PROFILE")
          .SetAxisRef(
              static_cast<std::shared_ptr<CRefEntityBase>>(
                  Ref::SketchSegment("SK-AX", "L_1", 0)))
          .SetAxisExplicit(StandardID::kOrigin, StandardID::kAxisZ)
          .SetAngle(kHalfPi)
          .Build();

  const std::filesystem::path xmlPath =
      std::filesystem::path("tmp") / "cadexchange_revolve_sketch_axis_ref.xml";
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
  const auto loadedAxisRef = loadedRevolve.GetAxisReference();
  Expect(loadedAxisRef.IsValid(),
         "Loaded revolve should preserve axis ReferenceEntity.");
  Expect(loadedAxisRef.GetRefType() == RefType::TOPO_SKETCH_SEG,
         "Loaded revolve axis reference type should remain SketchSeg.");
  Expect(loadedAxisRef.GetParentFeatureID() == "SK-AX",
         "Loaded revolve axis sketch reference should keep parent sketch ID.");
  Expect(loadedAxisRef.GetSketchSegmentLocalID() == "L_1",
         "Loaded revolve axis sketch reference should keep local segment ID.");
  Expect(loadedRevolve.GetProfileSketchID() == "SK-PROFILE",
         "Loaded revolve profile sketch should not be overwritten by axis parent.");
  Expect(loadedRevolve.GetAxisReferenceLocalID() == "L_1",
         "Loaded revolve should keep axis local ID from sketch-segment reference.");
}

void TestSweepBuilderAccessorAndXmlRoundTrip() {
  UnifiedModel model(UnitType::METER, "sweep-builder-accessor-xml");

  auto profileSketch = MakeSketch("SK-SWEEP-PROFILE", "SweepProfile");
  auto circle = std::make_shared<CSketchCircle>();
  circle->localID = "C_1";
  circle->center = CPoint3D{0.0, 0.0, 0.0};
  circle->radius = 0.002;
  profileSketch->segments.push_back(circle);
  model.AddFeature(profileSketch);

  auto pathSketch = MakeSketch("SK-SWEEP-PATH", "SweepPathSketch");
  auto pathLine = std::make_shared<CSketchLine>();
  pathLine->localID = "L_PATH";
  pathLine->startPos = CPoint3D{0.0, 0.0, 0.0};
  pathLine->endPos = CPoint3D{0.05, 0.0, 0.0};
  pathSketch->segments.push_back(pathLine);
  auto guideLine = std::make_shared<CSketchLine>();
  guideLine->localID = "L_GUIDE";
  guideLine->startPos = CPoint3D{0.0, 0.01, 0.0};
  guideLine->endPos = CPoint3D{0.05, 0.01, 0.0};
  pathSketch->segments.push_back(guideLine);
  model.AddFeature(pathSketch);

  const std::string sweepId =
      SweepBuilder(model, "SweepRoundTrip")
          .SetProfile("SK-SWEEP-PROFILE")
          .SetOperation(BooleanOp::BOSS)
          .AddPathReference(Ref::SketchSegment("SK-SWEEP-PATH", "L_PATH", 0))
          .SetPathStartPoint(CPoint3D{0.0, 0.0, 0.0})
          .SetPathEndPoint(CPoint3D{0.05, 0.0, 0.0})
          .AddGuidePathReference(
              Ref::SketchSegment("SK-SWEEP-PATH", "L_GUIDE", 1))
          .SetGuidePathStartPoint(0, CPoint3D{0.0, 0.01, 0.0})
          .SetGuidePathEndPoint(0, CPoint3D{0.05, 0.01, 0.0})
          .SetOrientation(SweepPathOrientation::KeepProfileNormal)
          .SetThinWallOffsets(-0.001, 0.001, false)
          .Build();

  SweepAccessor sweep(model.GetFeature(sweepId));
  Expect(sweep.IsValid(), "SweepAccessor should be valid.");
  Expect(sweep.GetProfileSketchID() == "SK-SWEEP-PROFILE",
         "Sweep profile sketch should be readable.");
  Expect(sweep.GetPathReferenceCount() == 1,
         "Sweep path should contain one reference.");
  Expect(sweep.GetPathReference(0).GetRefType() == RefType::TOPO_SKETCH_SEG,
         "Sweep path reference should preserve sketch-segment type.");
  Expect(sweep.GetPathReference(0).GetParentFeatureID() == "SK-SWEEP-PATH",
         "Sweep path reference should preserve path sketch ID.");
  Expect(sweep.GetPathReference(0).GetSketchSegmentLocalID() == "L_PATH",
         "Sweep path reference should preserve path segment local ID.");
  Expect(sweep.HasPathStartPoint() && sweep.HasPathEndPoint(),
         "Sweep path should expose start/end disambiguation points.");
  Expect(sweep.GetGuidePathCount() == 1,
         "Sweep guide path should be optional and readable when present.");
  Expect(sweep.GetGuidePathReferenceCount(0) == 1,
         "Sweep guide path should preserve its reference count.");
  Expect(sweep.GetGuidePathReference(0, 0).GetRefType() ==
             RefType::TOPO_SKETCH_SEG,
         "Sweep guide path reference should preserve sketch-segment type.");
  Expect(sweep.GetGuidePathReference(0, 0).GetSketchSegmentLocalID() ==
             "L_GUIDE",
         "Sweep guide path should preserve guide segment local ID.");
  Expect(sweep.HasGuidePathStartPoint(0) && sweep.HasGuidePathEndPoint(0),
         "Sweep guide path should expose optional start/end points.");
  Expect(sweep.GetOrientation() == SweepPathOrientation::KeepProfileNormal,
         "Sweep orientation should be readable.");
  Expect(sweep.HasThinWall(), "Sweep thin wall should be readable.");
  Expect(std::abs(sweep.GetThinWallThickness() - 0.002) < 1e-9,
         "Sweep thin wall thickness should combine two-sided offsets.");

  auto report = model.Validate();
  Expect(report.isValid, "Sweep model validation should pass.");

  const std::filesystem::path xmlPath =
      std::filesystem::path("tmp") / "cadexchange_sweep_roundtrip.xml";
  std::filesystem::create_directories(xmlPath.parent_path());
  std::string errorMessage;
  Expect(SaveModel(model, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Saving sweep XML should succeed: " + errorMessage);

  std::ifstream in(xmlPath, std::ios::binary);
  const std::string xml((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
  Expect(xml.find("Type=\"Sweep\"") != std::string::npos,
         "Sweep XML should use Feature Type=\"Sweep\".");
  Expect(xml.find("Orientation=\"KeepProfileNormal\"") != std::string::npos,
         "Sweep XML should serialize orientation.");
  Expect(xml.find("SegmentLocalID=\"L_PATH\"") != std::string::npos,
         "Sweep XML should serialize path sketch segment reference.");
  Expect(xml.find("GuidePaths") != std::string::npos,
         "Sweep XML should serialize optional guide paths when present.");
  Expect(xml.find("SegmentLocalID=\"L_GUIDE\"") != std::string::npos,
         "Sweep XML should serialize guide sketch segment reference.");

  UnifiedModel loaded;
  errorMessage.clear();
  Expect(LoadModel(loaded, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Loading sweep XML should succeed: " + errorMessage);
  SweepAccessor loadedSweep(loaded.GetFeature(sweepId));
  Expect(loadedSweep.IsValid(), "Loaded sweep should be accessible.");
  Expect(loadedSweep.GetProfileSketchID() == "SK-SWEEP-PROFILE",
         "Loaded sweep should preserve profile sketch.");
  Expect(loadedSweep.GetPathReferenceCount() == 1,
         "Loaded sweep should preserve path reference count.");
  Expect(loadedSweep.GetPathReference(0).GetSketchSegmentLocalID() == "L_PATH",
         "Loaded sweep should preserve path sketch segment local ID.");
  Expect(loadedSweep.GetGuidePathCount() == 1,
         "Loaded sweep should preserve guide path count.");
  Expect(loadedSweep.GetGuidePathReference(0, 0).GetSketchSegmentLocalID() ==
             "L_GUIDE",
         "Loaded sweep should preserve guide sketch segment local ID.");
  Expect(loadedSweep.GetOrientation() ==
             SweepPathOrientation::KeepProfileNormal,
         "Loaded sweep should preserve orientation.");
}

void TestSweepCircularProfileWithoutSketchRoundTrip() {
  UnifiedModel model(UnitType::METER, "sweep-circular-profile");

  auto pathSketch = MakeSketch("SK-CIRCULAR-SWEEP-PATH", "CircularSweepPath");
  auto pathLine = std::make_shared<CSketchLine>();
  pathLine->localID = "L_PATH";
  pathLine->startPos = CPoint3D{0.0, 0.0, 0.0};
  pathLine->endPos = CPoint3D{0.1, 0.0, 0.0};
  pathSketch->segments.push_back(pathLine);
  model.AddFeature(pathSketch);

  const std::string sweepId =
      SweepBuilder(model, "CircularSweep")
          .SetCircularProfile(0.004, 0.002)
          .SetSectionPlacement(SweepSectionPlacement::PathNormalAtStart)
          .AddPathReference(
              Ref::SketchSegment("SK-CIRCULAR-SWEEP-PATH", "L_PATH", 0))
          .Build();

  SweepAccessor sweep(model.GetFeature(sweepId));
  Expect(sweep.IsValid(), "Circular sweep should be accessible.");
  Expect(sweep.GetProfileKind() == SweepProfileKind::Circular,
         "Circular sweep should expose circular profile kind.");
  Expect(sweep.GetProfileSketchID().empty(),
         "Circular sweep should not require a profile sketch ID.");
  Expect(std::abs(sweep.GetCircularOuterRadius() - 0.004) < 1e-12,
         "Circular sweep should expose outer radius.");
  Expect(std::abs(sweep.GetCircularInnerRadius() - 0.002) < 1e-12,
         "Circular sweep should expose inner radius.");
  Expect(sweep.GetSectionPlacement() ==
             SweepSectionPlacement::PathNormalAtStart,
         "Circular sweep should preserve path-normal section placement.");

  auto report = model.Validate();
  Expect(report.isValid,
         "Circular sweep without profileSketchID should validate.");

  const std::filesystem::path xmlPath =
      std::filesystem::path("tmp") / "cadexchange_sweep_circular.xml";
  std::filesystem::create_directories(xmlPath.parent_path());
  std::string errorMessage;
  Expect(SaveModel(model, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Saving circular sweep XML should succeed: " + errorMessage);

  std::ifstream in(xmlPath, std::ios::binary);
  const std::string xml((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
  Expect(xml.find("<Profile Kind=\"Circular\"") != std::string::npos,
         "Circular sweep XML should serialize profile kind.");
  Expect(xml.find("OuterRadius=\"0.004") != std::string::npos,
         "Circular sweep XML should serialize outer radius.");
  Expect(xml.find("InnerRadius=\"0.002") != std::string::npos,
         "Circular sweep XML should serialize inner radius.");

  UnifiedModel loaded;
  errorMessage.clear();
  Expect(LoadModel(loaded, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Loading circular sweep XML should succeed: " + errorMessage);
  SweepAccessor loadedSweep(loaded.GetFeature(sweepId));
  Expect(loadedSweep.GetProfileKind() == SweepProfileKind::Circular,
         "Loaded circular sweep should preserve profile kind.");
  Expect(std::abs(loadedSweep.GetCircularOuterRadius() - 0.004) < 1e-12,
         "Loaded circular sweep should preserve outer radius.");
  Expect(std::abs(loadedSweep.GetCircularInnerRadius() - 0.002) < 1e-12,
         "Loaded circular sweep should preserve inner radius.");
}

void TestSweepEmbeddedSketchProfileRoundTrip() {
  UnifiedModel model(UnitType::METER, "sweep-embedded-profile");

  CSketch section;
  section.sketchCSys.origin = CPoint3D{0.0, 0.0, 0.0};
  section.sketchCSys.xDir = CVector3D{1.0, 0.0, 0.0};
  section.sketchCSys.yDir = CVector3D{0.0, 1.0, 0.0};
  section.sketchCSys.zDir = CVector3D{0.0, 0.0, 1.0};
  auto bottom = std::make_shared<CSketchLine>();
  bottom->localID = "E_BOTTOM";
  bottom->startPos = CPoint3D{-0.005, -0.005, 0.0};
  bottom->endPos = CPoint3D{0.005, -0.005, 0.0};
  auto right = std::make_shared<CSketchLine>();
  right->localID = "E_RIGHT";
  right->startPos = CPoint3D{0.005, -0.005, 0.0};
  right->endPos = CPoint3D{0.005, 0.005, 0.0};
  auto top = std::make_shared<CSketchLine>();
  top->localID = "E_TOP";
  top->startPos = CPoint3D{0.005, 0.005, 0.0};
  top->endPos = CPoint3D{-0.005, 0.005, 0.0};
  auto left = std::make_shared<CSketchLine>();
  left->localID = "E_LEFT";
  left->startPos = CPoint3D{-0.005, 0.005, 0.0};
  left->endPos = CPoint3D{-0.005, -0.005, 0.0};
  section.segments = {bottom, right, top, left};

  auto pathSketch = MakeSketch("SK-EMBEDDED-SWEEP-PATH", "EmbeddedSweepPath");
  auto pathLine = std::make_shared<CSketchLine>();
  pathLine->localID = "L_PATH";
  pathLine->startPos = CPoint3D{0.0, 0.0, 0.0};
  pathLine->endPos = CPoint3D{0.0, 0.0, 0.1};
  pathSketch->segments.push_back(pathLine);
  model.AddFeature(pathSketch);

  const std::string sweepId =
      SweepBuilder(model, "EmbeddedSweep")
          .SetEmbeddedProfile(section)
          .SetSectionPlacement(SweepSectionPlacement::PathNormalAtStart)
          .AddPathReference(
              Ref::SketchSegment("SK-EMBEDDED-SWEEP-PATH", "L_PATH", 0))
          .Build();

  SweepAccessor sweep(model.GetFeature(sweepId));
  Expect(sweep.GetProfileKind() == SweepProfileKind::EmbeddedSketch,
         "Embedded sweep should expose embedded sketch profile kind.");
  Expect(sweep.HasEmbeddedProfile(),
         "Embedded sweep should expose embedded profile sketch.");
  Expect(sweep.GetEmbeddedProfileSegmentCount() == 4,
         "Embedded sweep should preserve section sketch segments.");
  Expect(sweep.GetProfileSketchID().empty(),
         "Embedded sweep should not require external profileSketchID.");
  Expect(model.Validate().isValid,
         "Embedded sketch sweep without profileSketchID should validate.");

  const std::filesystem::path xmlPath =
      std::filesystem::path("tmp") / "cadexchange_sweep_embedded.xml";
  std::filesystem::create_directories(xmlPath.parent_path());
  std::string errorMessage;
  Expect(SaveModel(model, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Saving embedded sweep XML should succeed: " + errorMessage);

  std::ifstream in(xmlPath, std::ios::binary);
  const std::string xml((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
  Expect(xml.find("<Profile Kind=\"EmbeddedSketch\"") != std::string::npos,
         "Embedded sweep XML should serialize profile kind.");
  Expect(xml.find("LocalID=\"E_BOTTOM\"") != std::string::npos,
         "Embedded sweep XML should serialize embedded section segments.");

  UnifiedModel loaded;
  errorMessage.clear();
  Expect(LoadModel(loaded, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Loading embedded sweep XML should succeed: " + errorMessage);
  SweepAccessor loadedSweep(loaded.GetFeature(sweepId));
  Expect(loadedSweep.GetProfileKind() == SweepProfileKind::EmbeddedSketch,
         "Loaded embedded sweep should preserve profile kind.");
  Expect(loadedSweep.GetEmbeddedProfileSegmentCount() == 4,
         "Loaded embedded sweep should preserve section sketch segments.");
}

void TestSketchConstraintSupportsSubEntityAndExternalReference() {
  UnifiedModel model(UnitType::METER, "sketch-constraint-ref");

  auto sketch = MakeSketch("SK-CONSTRAINT", "SketchConstraint");
  auto line1 = std::make_shared<CSketchLine>();
  line1->localID = "L_1";
  line1->startPos = CPoint3D{0.0, 0.0, 0.0};
  line1->endPos = CPoint3D{1.0, 0.0, 0.0};
  sketch->segments.push_back(line1);

  auto line2 = std::make_shared<CSketchLine>();
  line2->localID = "L_2";
  line2->startPos = CPoint3D{1.0, 0.0, 0.0};
  line2->endPos = CPoint3D{1.0, 1.0, 0.0};
  sketch->segments.push_back(line2);

  CSketchConstraint coincident;
  coincident.type = CSketchConstraint::ConstraintType::COINCIDENT;
  coincident.refs.push_back(
      SketchConstraintRef::ForSketchEntity("L_1", SketchConstraintSubEntity::End));
  coincident.refs.push_back(
      SketchConstraintRef::ForSketchEntity("L_2", SketchConstraintSubEntity::Start));
  sketch->constraints.push_back(coincident);

  CSketchConstraint distance;
  distance.type = CSketchConstraint::ConstraintType::DISTANCE;
  distance.value = 12.5;
  distance.refs.push_back(
      SketchConstraintRef::ForSketchEntity("L_1", SketchConstraintSubEntity::Whole));
  distance.refs.push_back(
      SketchConstraintRef::ForExternalReference(
          RefPlaneBuilder(StandardID::PLANE_XY)
              .Origin(StandardID::kOrigin)
              .XDir(StandardID::kAxisX)
              .YDir(StandardID::kAxisY)
              .Normal(StandardID::kPlaneXYNormal)
              .Build(),
          SketchConstraintSubEntity::Whole));
  sketch->constraints.push_back(distance);

  model.AddFeature(sketch);

  Expect(model.GetFeature("SK-CONSTRAINT") != nullptr,
         "Sketch with constraint refs should be addable to model.");
  Expect(sketch->constraints.size() == 2,
         "Sketch should preserve two constraints.");
  Expect(sketch->constraints[0].refs.size() == 2,
         "Coincident constraint should preserve two refs.");
  Expect(sketch->constraints[0].refs[0].subEntity ==
             SketchConstraintSubEntity::End,
         "First coincident ref should preserve sub-entity End.");
  Expect(sketch->constraints[0].refs[1].subEntity ==
             SketchConstraintSubEntity::Start,
         "Second coincident ref should preserve sub-entity Start.");
  Expect(sketch->constraints[1].refs[1].refEntity != nullptr,
         "External reference should be preserved on sketch constraint.");
}

void TestSketchBuilderAndAccessorSupportRefBasedConstraints() {
  UnifiedModel model(UnitType::METER, "sketch-builder-accessor-ref-based");

  SketchBuilder sk(model, "SketchRefBased");
  sk.SetReferencePlane(Ref::XY())
      .SetCSys(CPoint3D{0, 0, 0}, CVector3D{1, 0, 0}, CVector3D{0, 1, 0},
               CVector3D{0, 0, 1});

  const std::string l1 =
      sk.AddLine(CPoint3D{0.0, 0.0, 0.0}, CPoint3D{1.0, 0.0, 0.0});
  const std::string l2 =
      sk.AddLine(CPoint3D{1.0, 0.0, 0.0}, CPoint3D{1.0, 1.0, 0.0});

  sk.AddCoincident(
        SketchConstraintRef::ForSketchEntity(
            l1, SketchConstraintSubEntity::End),
        SketchConstraintRef::ForSketchEntity(
            l2, SketchConstraintSubEntity::Start))
    .AddDistanceDimension(
        SketchConstraintRef::ForSketchEntity(
            l1, SketchConstraintSubEntity::Whole),
        SketchConstraintRef::ForExternalReference(Ref::XY()),
        0.25);

  const std::string sketchID = sk.Build();

  SketchAccessor sketch(model.GetFeature(sketchID));
  Expect(sketch.IsValid(), "SketchAccessor should be valid for ref-based sketch.");
  Expect(sketch.GetConstraintCount() == 2,
         "Sketch should preserve two ref-based constraints.");

  auto c0 = sketch.GetConstraintAccessor(0);
  Expect(c0.IsValid(), "Constraint accessor should be valid.");
  Expect(c0.GetRefCount() == 2,
         "Coincident constraint should expose two refs.");
  Expect(c0.GetRefSubEntity(0) == SketchConstraintSubEntity::End,
         "Coincident ref[0] should preserve End sub-entity.");
  Expect(c0.GetRefSubEntity(1) == SketchConstraintSubEntity::Start,
         "Coincident ref[1] should preserve Start sub-entity.");

  auto c1 = sketch.GetConstraintAccessor(1);
  Expect(c1.HasValue(), "Distance constraint should expose a numeric value.");
  Expect(std::abs(c1.GetValue() - 0.25) < 1e-12,
         "Distance constraint value should round-trip through accessor.");
  Expect(c1.GetRefKind(1) == SketchConstraintRefKind::ExternalReference,
         "Distance constraint ref[1] should expose external reference kind.");
  Expect(c1.GetReference(1).IsValid(),
         "Distance constraint external ref should be readable via accessor.");
}

void TestSketchConstraintValidationRejectsMissingSketchEntityRef() {
  UnifiedModel model(UnitType::METER, "sketch-constraint-validation");

  auto sketch = MakeSketch("SK-INVALID-CONSTRAINT", "SketchInvalidConstraint");
  auto line = std::make_shared<CSketchLine>();
  line->localID = "L_1";
  line->startPos = CPoint3D{0.0, 0.0, 0.0};
  line->endPos = CPoint3D{1.0, 0.0, 0.0};
  sketch->segments.push_back(line);

  CSketchConstraint invalid;
  invalid.type = CSketchConstraint::ConstraintType::COINCIDENT;
  invalid.refs.push_back(
      SketchConstraintRef::ForSketchEntity("L_1", SketchConstraintSubEntity::End));
  invalid.refs.push_back(
      SketchConstraintRef::ForSketchEntity("L_MISSING", SketchConstraintSubEntity::Start));
  sketch->constraints.push_back(invalid);
  model.AddFeature(sketch);

  const auto report = model.Validate();
  Expect(!report.isValid,
         "Validation should fail when a sketch constraint references a missing localID.");
  bool foundConstraintError = false;
  for (const auto &err : report.errors) {
    if (err.find("[SKETCH_002]") != std::string::npos) {
      foundConstraintError = true;
      break;
    }
  }
  Expect(foundConstraintError,
         "Missing sketch-entity constraint ref should report SKETCH_002.");
}

void TestConvertModelUnitScalesSketchConstraintExternalReferences() {
  UnifiedModel model(UnitType::METER, "sketch-constraint-unit-convert");

  auto sketch = MakeSketch("SK-UNIT-CONSTRAINT", "SketchUnitConstraint");
  auto line = std::make_shared<CSketchLine>();
  line->localID = "L_1";
  line->startPos = CPoint3D{0.0, 0.0, 0.0};
  line->endPos = CPoint3D{0.5, 0.0, 0.0};
  sketch->segments.push_back(line);

  auto planeRef =
      RefPlaneBuilder("DATUM_TEST")
          .Origin(CPoint3D{0.1, 0.2, 0.3})
          .XDir(StandardID::kAxisX)
          .YDir(StandardID::kAxisY)
          .Normal(StandardID::kPlaneXYNormal)
          .Build();

  CSketchConstraint distance;
  distance.type = CSketchConstraint::ConstraintType::DISTANCE;
  distance.value = 0.25;
  distance.refs.push_back(
      SketchConstraintRef::ForSketchEntity("L_1", SketchConstraintSubEntity::Whole));
  distance.refs.push_back(SketchConstraintRef::ForExternalReference(planeRef));
  sketch->constraints.push_back(distance);
  model.AddFeature(sketch);

  std::string errorMessage;
  Expect(ConvertModelUnit(model, UnitType::MILLIMETER, &errorMessage),
         "ConvertModelUnit should succeed for sketch constraint refs: " +
             errorMessage);

  SketchAccessor sketchAcc(model.GetFeature("SK-UNIT-CONSTRAINT"));
  Expect(sketchAcc.IsValid(), "Converted sketch should still be accessible.");
  auto constraint = sketchAcc.GetConstraintAccessor(0);
  Expect(constraint.IsValid(), "Converted constraint should still be accessible.");
  Expect(constraint.HasValue(), "Converted distance constraint should keep value.");
  Expect(std::abs(constraint.GetValue() - 250.0) < 1e-9,
         "Distance constraint value should scale from meters to millimeters.");

  auto ref = constraint.GetReference(1);
  Expect(ref.IsValid(), "Converted external reference should remain readable.");
  CPoint3D origin;
  Expect(ref.GetPlaneOrigin(origin),
         "Converted external plane reference should expose origin.");
  Expect(std::abs(origin.x - 100.0) < 1e-9 &&
             std::abs(origin.y - 200.0) < 1e-9 &&
             std::abs(origin.z - 300.0) < 1e-9,
         "External plane reference origin should scale with model units.");
}

void TestChamferBuilderAccessorAndXmlRoundTrip() {
  UnifiedModel model(UnitType::METER, "chamfer-builder-accessor-xml");
  auto sketch = MakeSketch("SK-CHAMFER", "ChamferSketch");
  auto line1 = std::make_shared<CSketchLine>();
  line1->localID = "L_1";
  line1->startPos = CPoint3D{0.0, 0.0, 0.0};
  line1->endPos = CPoint3D{0.05, 0.0, 0.0};
  sketch->segments.push_back(line1);
  auto line2 = std::make_shared<CSketchLine>();
  line2->localID = "L_2";
  line2->startPos = CPoint3D{0.05, 0.0, 0.0};
  line2->endPos = CPoint3D{0.05, 0.05, 0.0};
  sketch->segments.push_back(line2);
  model.AddFeature(sketch);

  const std::string extrudeID =
      MakeExtrudeFromSketch(model, "SK-CHAMFER", "ChamferBoss");

  const std::string chamferID =
      ChamferBuilder(model, "EdgeChamfer")
          .SetMode(ChamferMode::DISTANCE_ANGLE)
          .SetDistance1(0.003)
          .SetAngle(45.0)
          .SetFirstEndFaceMarker(CPoint3D{0.05, 0.01, 0.02})
          .AddReference(Ref::Edge(extrudeID, 0)
                            .StartPoint(CPoint3D{0.0, 0.0, 0.02})
                            .EndPoint(CPoint3D{0.05, 0.0, 0.02})
                            .MidPoint(CPoint3D{0.025, 0.0, 0.02}))
          .Build();

  ChamferAccessor chamfer(model.GetFeature(chamferID));
  Expect(chamfer.IsValid(), "ChamferAccessor should be valid.");
  Expect(chamfer.GetMode() == ChamferMode::DISTANCE_ANGLE,
         "Chamfer mode should be readable.");
  Expect(chamfer.HasDistance1() && std::abs(chamfer.GetDistance1() - 0.003) < 1e-12,
         "Chamfer distance1 should be readable.");
  Expect(chamfer.HasAngle() && std::abs(chamfer.GetAngle() - 45.0) < 1e-12,
         "Chamfer angle should be readable.");
  Expect(chamfer.HasFirstEndFaceMarker(),
         "Chamfer first end face marker should be readable.");
  const CPoint3D marker = chamfer.GetFirstEndFaceMarker();
  Expect(std::abs(marker.x - 0.05) < 1e-12 &&
             std::abs(marker.y - 0.01) < 1e-12 &&
             std::abs(marker.z - 0.02) < 1e-12,
         "Chamfer first end face marker should preserve coordinates.");
  Expect(chamfer.GetReferenceCount() == 1,
         "Chamfer should preserve one reference.");
  Expect(chamfer.GetReference(0).GetRefType() == RefType::TOPO_EDGE,
         "Chamfer reference type should remain edge.");

  auto report = model.Validate();
  Expect(report.isValid, "Chamfer model validation should pass.");

  const std::filesystem::path xmlPath =
      std::filesystem::path("tmp") / "cadexchange_chamfer_roundtrip.xml";
  std::filesystem::create_directories(xmlPath.parent_path());
  std::string errorMessage;
  Expect(SaveModel(model, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Saving chamfer XML should succeed: " + errorMessage);

  std::ifstream in(xmlPath, std::ios::binary);
  const std::string xml((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
  Expect(xml.find("Type=\"Chamfer\"") != std::string::npos,
         "Chamfer XML should use Feature Type=\"Chamfer\".");
  Expect(xml.find("Mode=\"DistanceAngle\"") != std::string::npos,
         "Chamfer XML should serialize mode.");
  Expect(xml.find("Angle=\"45\"") != std::string::npos,
         "Chamfer XML should serialize angle.");
  Expect(xml.find("FirstEndFaceMarker=\"(0.05,0.01,0.02)\"") != std::string::npos,
         "Chamfer XML should serialize first end face marker.");

  UnifiedModel loaded;
  errorMessage.clear();
  Expect(LoadModel(loaded, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Loading chamfer XML should succeed: " + errorMessage);
  ChamferAccessor loadedChamfer(loaded.GetFeature(chamferID));
  Expect(loadedChamfer.IsValid(), "Loaded chamfer should be accessible.");
  Expect(loadedChamfer.GetMode() == ChamferMode::DISTANCE_ANGLE,
         "Loaded chamfer should preserve mode.");
  Expect(std::abs(loadedChamfer.GetDistance1() - 0.003) < 1e-12,
         "Loaded chamfer should preserve distance1.");
  Expect(std::abs(loadedChamfer.GetAngle() - 45.0) < 1e-12,
         "Loaded chamfer should preserve angle.");
  Expect(loadedChamfer.HasFirstEndFaceMarker(),
         "Loaded chamfer should preserve first end face marker.");
  const CPoint3D loadedMarker = loadedChamfer.GetFirstEndFaceMarker();
  Expect(std::abs(loadedMarker.x - 0.05) < 1e-12 &&
             std::abs(loadedMarker.y - 0.01) < 1e-12 &&
             std::abs(loadedMarker.z - 0.02) < 1e-12,
         "Loaded chamfer should preserve first end face marker coordinates.");
  Expect(loadedChamfer.GetReferenceCount() == 1,
         "Loaded chamfer should preserve reference count.");
}

void TestChamferOffsetModeRoundTrip() {
  UnifiedModel model(UnitType::METER, "chamfer-offset-roundtrip");
  auto sketch = MakeSketch("SK-CHAMFER-OFFSET", "ChamferSketchOffset");
  auto line = std::make_shared<CSketchLine>();
  line->localID = "L_1";
  line->startPos = CPoint3D{0.0, 0.0, 0.0};
  line->endPos = CPoint3D{0.04, 0.0, 0.0};
  sketch->segments.push_back(line);
  model.AddFeature(sketch);

  const std::string extrudeID =
      MakeExtrudeFromSketch(model, "SK-CHAMFER-OFFSET", "ChamferBossOffset");

  const std::string chamferID =
      ChamferBuilder(model, "OffsetChamfer")
          .SetMode(ChamferMode::TWO_OFFSETS)
          .SetOffset1(0.0015)
          .SetOffset2(0.0025)
          .AddReference(Ref::Edge(extrudeID, 0)
                            .StartPoint(CPoint3D{0.0, 0.0, 0.02})
                            .EndPoint(CPoint3D{0.04, 0.0, 0.02})
                            .MidPoint(CPoint3D{0.02, 0.0, 0.02}))
          .Build();

  ChamferAccessor chamfer(model.GetFeature(chamferID));
  Expect(chamfer.GetMode() == ChamferMode::TWO_OFFSETS,
         "Chamfer offset mode should be readable.");
  Expect(chamfer.HasOffset1() && std::abs(chamfer.GetOffset1() - 0.0015) < 1e-12,
         "Chamfer offset1 should be readable.");
  Expect(chamfer.HasOffset2() && std::abs(chamfer.GetOffset2() - 0.0025) < 1e-12,
         "Chamfer offset2 should be readable.");

  const std::filesystem::path xmlPath =
      std::filesystem::path("tmp") / "cadexchange_chamfer_offset_roundtrip.xml";
  std::filesystem::create_directories(xmlPath.parent_path());
  std::string errorMessage;
  Expect(SaveModel(model, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Saving offset chamfer XML should succeed: " + errorMessage);

  std::ifstream in(xmlPath, std::ios::binary);
  const std::string xml((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
  Expect(xml.find("Mode=\"TwoOffsets\"") != std::string::npos,
         "Chamfer XML should serialize offset mode.");
  Expect(xml.find("Offset1=\"0.0015") != std::string::npos,
         "Chamfer XML should serialize offset1.");
  Expect(xml.find("Offset2=\"0.0025") != std::string::npos,
         "Chamfer XML should serialize offset2.");

  UnifiedModel loaded;
  errorMessage.clear();
  Expect(LoadModel(loaded, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Loading offset chamfer XML should succeed: " + errorMessage);
  ChamferAccessor loadedChamfer(loaded.GetFeature(chamferID));
  Expect(loadedChamfer.GetMode() == ChamferMode::TWO_OFFSETS,
         "Loaded chamfer should preserve offset mode.");
  Expect(std::abs(loadedChamfer.GetOffset1() - 0.0015) < 1e-12,
         "Loaded chamfer should preserve offset1.");
  Expect(std::abs(loadedChamfer.GetOffset2() - 0.0025) < 1e-12,
         "Loaded chamfer should preserve offset2.");
}

void TestChamferValidationAndUnitConversion() {
  UnifiedModel validModel(UnitType::METER, "chamfer-unit-convert");
  auto sketch = MakeSketch("SK-CHAMFER-UNIT", "ChamferSketchUnit");
  auto line = std::make_shared<CSketchLine>();
  line->localID = "L_1";
  line->startPos = CPoint3D{0.0, 0.0, 0.0};
  line->endPos = CPoint3D{0.04, 0.0, 0.0};
  sketch->segments.push_back(line);
  validModel.AddFeature(sketch);

  const std::string extrudeID =
      MakeExtrudeFromSketch(validModel, "SK-CHAMFER-UNIT", "ChamferBossUnit");

  const std::string chamferID =
      ChamferBuilder(validModel, "VertexChamfer")
          .SetMode(ChamferMode::VERTEX_3DISTANCES)
          .SetDistance1(0.001)
          .SetDistance2(0.002)
          .SetDistance3(0.003)
          .AddReference(Ref::Vertex(extrudeID, 0).Pos(CPoint3D{0.0, 0.0, 0.02}))
          .Build();

  std::string errorMessage;
  Expect(ConvertModelUnit(validModel, UnitType::MILLIMETER, &errorMessage),
         "ConvertModelUnit should scale chamfer distances: " + errorMessage);

  ChamferAccessor converted(validModel.GetFeature(chamferID));
  Expect(std::abs(converted.GetDistance1() - 1.0) < 1e-9,
         "Chamfer distance1 should scale to millimeters.");
  Expect(std::abs(converted.GetDistance2() - 2.0) < 1e-9,
         "Chamfer distance2 should scale to millimeters.");
  Expect(std::abs(converted.GetDistance3() - 3.0) < 1e-9,
         "Chamfer distance3 should scale to millimeters.");

  UnifiedModel offsetModel(UnitType::METER, "chamfer-offset-unit-convert");
  auto offsetSketch = MakeSketch("SK-CHAMFER-OFFSET-UNIT", "ChamferOffsetSketchUnit");
  auto offsetLine = std::make_shared<CSketchLine>();
  offsetLine->localID = "L_1";
  offsetLine->startPos = CPoint3D{0.0, 0.0, 0.0};
  offsetLine->endPos = CPoint3D{0.04, 0.0, 0.0};
  offsetSketch->segments.push_back(offsetLine);
  offsetModel.AddFeature(offsetSketch);

  const std::string offsetExtrudeID =
      MakeExtrudeFromSketch(offsetModel, "SK-CHAMFER-OFFSET-UNIT", "ChamferBossOffsetUnit");
  const std::string offsetChamferID =
      ChamferBuilder(offsetModel, "OffsetChamferUnit")
          .SetMode(ChamferMode::TWO_OFFSETS)
          .SetOffset1(0.004)
          .SetOffset2(0.005)
          .SetFirstEndFaceMarker(CPoint3D{0.004, 0.005, 0.006})
          .AddReference(Ref::Edge(offsetExtrudeID, 0)
                            .StartPoint(CPoint3D{0.0, 0.0, 0.02})
                            .EndPoint(CPoint3D{0.04, 0.0, 0.02})
                            .MidPoint(CPoint3D{0.02, 0.0, 0.02}))
          .Build();

  errorMessage.clear();
  Expect(ConvertModelUnit(offsetModel, UnitType::CENTIMETER, &errorMessage),
         "ConvertModelUnit should scale chamfer offsets: " + errorMessage);

  ChamferAccessor convertedOffset(offsetModel.GetFeature(offsetChamferID));
  Expect(std::abs(convertedOffset.GetOffset1() - 0.4) < 1e-9,
         "Chamfer offset1 should scale to centimeters.");
  Expect(std::abs(convertedOffset.GetOffset2() - 0.5) < 1e-9,
         "Chamfer offset2 should scale to centimeters.");
  Expect(convertedOffset.HasFirstEndFaceMarker(),
         "Chamfer first end face marker should remain after unit conversion.");
  const CPoint3D convertedMarker = convertedOffset.GetFirstEndFaceMarker();
  Expect(std::abs(convertedMarker.x - 0.4) < 1e-9 &&
             std::abs(convertedMarker.y - 0.5) < 1e-9 &&
             std::abs(convertedMarker.z - 0.6) < 1e-9,
         "Chamfer first end face marker should scale with model units.");

  UnifiedModel invalid(UnitType::METER, "chamfer-invalid");
  invalid.AddFeature(MakeSketch("SK-CHAMFER-BAD", "ChamferBadSketch"));
  auto badChamfer = std::make_shared<CChamfer>();
  badChamfer->featureID = "CH_BAD";
  badChamfer->featureName = "BadChamfer";
  badChamfer->mode = ChamferMode::DISTANCE_ANGLE;
  badChamfer->params.distance1 = 0.002;
  invalid.AddFeature(badChamfer);

  const auto report = invalid.Validate();
  Expect(!report.isValid,
         "Validation should fail when a chamfer misses required fields.");
  bool foundMissingAngle = false;
  bool foundMissingRefs = false;
  for (const auto &err : report.errors) {
    if (err.find("angle") != std::string::npos) {
      foundMissingAngle = true;
    }
    if (err.find("no references") != std::string::npos) {
      foundMissingRefs = true;
    }
  }
  Expect(foundMissingAngle,
         "Missing chamfer angle should be reported by validation.");
  Expect(foundMissingRefs,
         "Missing chamfer references should be reported by validation.");
}

void TestFilletBuilderAccessorAndXmlRoundTrip() {
  UnifiedModel model(UnitType::METER, "fillet-builder-accessor-xml");
  auto sketch = MakeSketch("SK-FILLET", "FilletSketch");
  auto line = std::make_shared<CSketchLine>();
  line->localID = "L_1";
  line->startPos = CPoint3D{0.0, 0.0, 0.0};
  line->endPos = CPoint3D{0.06, 0.0, 0.0};
  sketch->segments.push_back(line);
  model.AddFeature(sketch);

  const std::string extrudeID =
      MakeExtrudeFromSketch(model, "SK-FILLET", "FilletBoss");

  CFilletRadiusItem radiusItem;
  radiusItem.position = 0.25;
  radiusItem.radius1 = 0.002;
  radiusItem.radius2 = 0.003;
  radiusItem.refEdge = Ref::Edge(extrudeID, 0)
                           .StartPoint(CPoint3D{0.0, 0.0, 0.02})
                           .EndPoint(CPoint3D{0.06, 0.0, 0.02})
                           .MidPoint(CPoint3D{0.03, 0.0, 0.02});

  const std::string filletID =
      FilletBuilder(model, "EdgeFillet")
          .SetMode(FilletMode::VARIABLE_RADIUS)
          .SetCrossSection(FilletCrossSection::CONIC)
          .SetReferenceMode(FilletReferenceMode::EDGE_CHAIN)
          .SetDefaultRadius(0.004)
          .SetDefaultRadius2(0.005)
          .SetAsymmetric(true)
          .SetTangentPropagation(true)
          .SetConicValue(0.75, FilletConicValueMode::RHO)
          .SetFirstEndFaceMarker(CPoint3D{0.06, 0.0, 0.02})
          .AddReference(Ref::Edge(extrudeID, 1)
                            .StartPoint(CPoint3D{0.06, 0.0, 0.02})
                            .EndPoint(CPoint3D{0.06, 0.06, 0.02})
                            .MidPoint(CPoint3D{0.06, 0.03, 0.02}))
          .AddRadiusItem(radiusItem)
          .SetSwOverflowType("KeepEdge")
          .SetSwKeepFeatures(true)
          .SetCreoAttachType(2)
          .SetCreoConicDepOption(1)
          .Build();

  FilletAccessor fillet(model.GetFeature(filletID));
  Expect(fillet.IsValid(), "FilletAccessor should be valid.");
  Expect(fillet.GetMode() == FilletMode::VARIABLE_RADIUS,
         "Fillet mode should be readable.");
  Expect(fillet.GetCrossSection() == FilletCrossSection::CONIC,
         "Fillet cross section should be readable.");
  Expect(fillet.GetReferenceMode() == FilletReferenceMode::EDGE_CHAIN,
         "Fillet reference mode should be readable.");
  Expect(fillet.HasDefaultRadius() &&
             std::abs(fillet.GetDefaultRadius() - 0.004) < 1e-12,
         "Fillet default radius should be readable.");
  Expect(fillet.HasDefaultRadius2() &&
             std::abs(fillet.GetDefaultRadius2() - 0.005) < 1e-12,
         "Fillet default radius2 should be readable.");
  Expect(fillet.IsAsymmetric(), "Fillet asymmetric flag should be readable.");
  Expect(fillet.HasTangentPropagation(),
         "Fillet tangent propagation should be readable.");
  Expect(fillet.HasConicValue() &&
             std::abs(fillet.GetConicValue() - 0.75) < 1e-12,
         "Fillet conic value should be readable.");
  Expect(fillet.GetConicValueMode() == FilletConicValueMode::RHO,
         "Fillet conic value mode should be readable.");
  Expect(fillet.HasFirstEndFaceMarker(),
         "Fillet first-end-face marker should be readable.");
  const CPoint3D filletMarker = fillet.GetFirstEndFaceMarker();
  Expect(std::abs(filletMarker.x - 0.06) < 1e-12 &&
             std::abs(filletMarker.y - 0.0) < 1e-12 &&
             std::abs(filletMarker.z - 0.02) < 1e-12,
         "Fillet first-end-face marker should preserve coordinates.");
  Expect(fillet.GetReferences().size() == 1,
         "Fillet should preserve one edge-chain reference.");
  Expect(fillet.GetRadiusItems().size() == 1,
         "Fillet should preserve one radius item.");
  const auto &item = fillet.GetRadiusItems().front();
  Expect(item.position.has_value() && std::abs(*item.position - 0.25) < 1e-12,
         "Fillet radius item position should be readable.");
  Expect(item.radius1.has_value() && std::abs(*item.radius1 - 0.002) < 1e-12,
         "Fillet radius item radius1 should be readable.");
  Expect(item.radius2.has_value() && std::abs(*item.radius2 - 0.003) < 1e-12,
         "Fillet radius item radius2 should be readable.");
  Expect(item.refEdge != nullptr,
         "Fillet radius item refEdge should be preserved.");

  const std::filesystem::path xmlPath =
      std::filesystem::path("tmp") / "cadexchange_fillet_roundtrip.xml";
  std::filesystem::create_directories(xmlPath.parent_path());
  std::string errorMessage;
  Expect(SaveModel(model, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Saving fillet XML should succeed: " + errorMessage);

  std::ifstream in(xmlPath, std::ios::binary);
  const std::string xml((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
  Expect(xml.find("Type=\"Fillet\"") != std::string::npos,
         "Fillet XML should use Feature Type=\"Fillet\".");
  Expect(xml.find("Mode=\"VariableRadius\"") != std::string::npos,
         "Fillet XML should serialize Mode as a readable string.");
  Expect(xml.find("FirstEndFaceMarker=\"(0.06,0,0.02)\"") !=
             std::string::npos,
         "Fillet XML should serialize first-end-face marker.");
  Expect(xml.find("CrossSection=\"Conic\"") != std::string::npos,
         "Fillet XML should serialize CrossSection as a readable string.");
  Expect(xml.find("ReferenceMode=\"EdgeChain\"") != std::string::npos,
         "Fillet XML should serialize ReferenceMode as a readable string.");
  Expect(xml.find("ConicValueMode=\"Rho\"") != std::string::npos,
         "Fillet XML should serialize ConicValueMode only when conic data is present.");
  Expect(xml.find("DefaultRadius=\"0.004") == std::string::npos,
         "Variable-radius fillet XML should not duplicate default radius when RadiusItems are present.");
  Expect(xml.find("DefaultRadius2=\"0.005") == std::string::npos,
         "Variable-radius fillet XML should not duplicate default radius2 when RadiusItems are present.");
  Expect(xml.find("RadiusItem") != std::string::npos,
         "Fillet XML should serialize radius items.");
  Expect(xml.find("VendorExtensions") == std::string::npos,
         "Fillet XML should not serialize vendor-specific extension fields.");
  Expect(xml.find("Mode=\"Chordal\"") != std::string::npos,
         "Fillet XML should serialize chordal mode as a readable string.");

  UnifiedModel loaded;
  errorMessage.clear();
  Expect(LoadModel(loaded, xmlPath, &errorMessage, SerializationFormat::TINYXML),
         "Loading fillet XML should succeed: " + errorMessage);
  FilletAccessor loadedFillet(loaded.GetFeature(filletID));
  Expect(loadedFillet.IsValid(), "Loaded fillet should be accessible.");
  Expect(loadedFillet.GetMode() == FilletMode::VARIABLE_RADIUS,
         "Loaded fillet should preserve mode.");
  Expect(loadedFillet.HasFirstEndFaceMarker(),
         "Loaded fillet should preserve first-end-face marker.");
  const CPoint3D loadedFilletMarker = loadedFillet.GetFirstEndFaceMarker();
  Expect(std::abs(loadedFilletMarker.x - 0.06) < 1e-12 &&
             std::abs(loadedFilletMarker.y - 0.0) < 1e-12 &&
             std::abs(loadedFilletMarker.z - 0.02) < 1e-12,
         "Loaded fillet should preserve first-end-face marker coordinates.");
  Expect(!loadedFillet.HasDefaultRadius(),
         "Loaded variable-radius fillet should omit duplicate default radius when RadiusItems are serialized.");
  Expect(!loadedFillet.HasDefaultRadius2(),
         "Loaded variable-radius fillet should omit duplicate default radius2 when RadiusItems are serialized.");
  Expect(loadedFillet.GetReferences().size() == 1,
         "Loaded fillet should preserve references.");
  Expect(loadedFillet.GetRadiusItems().size() == 1,
         "Loaded fillet should preserve radius items.");
  FilletAccessor loadedChordalFillet(loaded.GetFeature(chordalFilletID));
  Expect(loadedChordalFillet.IsValid(), "Loaded chordal fillet should be accessible.");
  Expect(loadedChordalFillet.GetMode() == FilletMode::CHORDAL,
         "Loaded chordal fillet should preserve mode.");

  const std::string chordalFilletID =
      FilletBuilder(model, "ChordalFillet")
          .SetMode(FilletMode::CHORDAL)
          .SetCrossSection(FilletCrossSection::CONIC)
          .SetReferenceMode(FilletReferenceMode::EDGE_CHAIN)
          .SetDefaultRadius(0.006)
          .AddReference(Ref::Edge(extrudeID, 2)
                            .StartPoint(CPoint3D{0.06, 0.06, 0.02})
                            .EndPoint(CPoint3D{0.0, 0.06, 0.02})
                            .MidPoint(CPoint3D{0.03, 0.06, 0.02}))
          .Build();
  FilletAccessor chordalFillet(model.GetFeature(chordalFilletID));
  Expect(chordalFillet.IsValid(), "Chordal fillet should be accessible.");
  Expect(chordalFillet.GetMode() == FilletMode::CHORDAL,
         "Chordal fillet mode should be readable.");
}

void TestFilletUnitConversion() {
  UnifiedModel model(UnitType::METER, "fillet-unit-convert");
  auto sketch = MakeSketch("SK-FILLET-UNIT", "FilletSketchUnit");
  auto line = std::make_shared<CSketchLine>();
  line->localID = "L_1";
  line->startPos = CPoint3D{0.0, 0.0, 0.0};
  line->endPos = CPoint3D{0.05, 0.0, 0.0};
  sketch->segments.push_back(line);
  model.AddFeature(sketch);

  const std::string extrudeID =
      MakeExtrudeFromSketch(model, "SK-FILLET-UNIT", "FilletBossUnit");

  CFilletRadiusItem radiusItem;
  radiusItem.radius1 = 0.001;
  radiusItem.radius2 = 0.002;
  radiusItem.refEdge = Ref::Edge(extrudeID, 0)
                           .StartPoint(CPoint3D{0.0, 0.0, 0.02})
                           .EndPoint(CPoint3D{0.05, 0.0, 0.02})
                           .MidPoint(CPoint3D{0.025, 0.0, 0.02});

  const std::string filletID =
      FilletBuilder(model, "UnitFillet")
          .SetMode(FilletMode::CONSTANT_RADIUS)
          .SetCrossSection(FilletCrossSection::CONIC)
          .SetReferenceMode(FilletReferenceMode::EDGE_CHAIN)
          .SetDefaultRadius(0.003)
          .SetDefaultRadius2(0.004)
          .SetConicValue(0.005)
          .SetFirstEndFaceMarker(CPoint3D{0.01, 0.02, 0.03})
          .AddReference(Ref::Edge(extrudeID, 1)
                            .StartPoint(CPoint3D{0.05, 0.0, 0.02})
                            .EndPoint(CPoint3D{0.05, 0.05, 0.02})
                            .MidPoint(CPoint3D{0.05, 0.025, 0.02}))
          .AddRadiusItem(radiusItem)
          .Build();

  std::string errorMessage;
  Expect(ConvertModelUnit(model, UnitType::MILLIMETER, &errorMessage),
         "ConvertModelUnit should scale fillet values: " + errorMessage);

  FilletAccessor converted(model.GetFeature(filletID));
  Expect(std::abs(converted.GetDefaultRadius() - 3.0) < 1e-9,
         "Fillet default radius should scale to millimeters.");
  Expect(std::abs(converted.GetDefaultRadius2() - 4.0) < 1e-9,
         "Fillet default radius2 should scale to millimeters.");
  Expect(converted.HasConicValue() &&
             std::abs(converted.GetConicValue() - 5.0) < 1e-9,
         "Fillet conic value should scale with model units.");
  Expect(converted.GetRadiusItems().size() == 1,
         "Converted fillet should preserve radius items.");
  const auto &item = converted.GetRadiusItems().front();
  Expect(item.radius1.has_value() && std::abs(*item.radius1 - 1.0) < 1e-9,
         "Fillet radius item radius1 should scale to millimeters.");
  Expect(item.radius2.has_value() && std::abs(*item.radius2 - 2.0) < 1e-9,
         "Fillet radius item radius2 should scale to millimeters.");
  Expect(converted.HasFirstEndFaceMarker(),
         "Converted fillet should preserve first-end-face marker.");
  const CPoint3D convertedMarker = converted.GetFirstEndFaceMarker();
  Expect(std::abs(convertedMarker.x - 10.0) < 1e-9 &&
             std::abs(convertedMarker.y - 20.0) < 1e-9 &&
             std::abs(convertedMarker.z - 30.0) < 1e-9,
         "Fillet first-end-face marker should scale to millimeters.");
}

} // namespace

int main() {
  TestRevolveBuilderIgnoresUnknownExtent();
  TestRevolveAccessorExposesSharedExtentFields();
  TestLegacyRevolveXmlRejected();
  TestRevolveSketchAxisSerializesReferenceEntity();
  TestSweepBuilderAccessorAndXmlRoundTrip();
  TestSweepCircularProfileWithoutSketchRoundTrip();
  TestSweepEmbeddedSketchProfileRoundTrip();
  TestSketchConstraintSupportsSubEntityAndExternalReference();
  TestSketchBuilderAndAccessorSupportRefBasedConstraints();
  TestSketchConstraintValidationRejectsMissingSketchEntityRef();
  TestConvertModelUnitScalesSketchConstraintExternalReferences();
  TestFilletBuilderAccessorAndXmlRoundTrip();
  TestFilletUnitConversion();
  TestChamferBuilderAccessorAndXmlRoundTrip();
  TestChamferOffsetModeRoundTrip();
  TestChamferValidationAndUnitConversion();
  std::cout << "[PASS] MigrationRegressionTest" << std::endl;
  return 0;
}
