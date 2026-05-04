#include "../core/UnifiedModel.h"
#include "../service/accessors/RevolveAccessor.h"
#include "../service/accessors/SketchAccessor.h"
#include "../service/accessors/SweepAccessor.h"
#include "../service/builders/EndConditionBuilder.h"
#include "../service/builders/ReferenceBuilder.h"
#include "../service/builders/RevolveBuilder.h"
#include "../service/builders/SketchBuilder.h"
#include "../service/builders/SweepBuilder.h"
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
         "Saving revolve radian XML should succeed: " + errorMessage);

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
      std::filesystem::path("E:/MyProject/tmp/cadexchange_sweep_roundtrip.xml");
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
      std::filesystem::path("E:/MyProject/tmp/cadexchange_sweep_circular.xml");
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
      std::filesystem::path("E:/MyProject/tmp/cadexchange_sweep_embedded.xml");
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
  std::cout << "[PASS] MigrationRegressionTest" << std::endl;
  return 0;
}
