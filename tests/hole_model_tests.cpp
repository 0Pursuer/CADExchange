#include "CADSerializer.h"
#include "FeatureFormatter.h"
#include "HoleAccessor.h"
#include "HoleBuilder.h"
#include "HoleGeometry.h"
#include "ReferenceBuilder.h"
#include "cereal/external/rapidjson/document.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace CADExchange;
void Require(bool ok, const std::string &why) {
  if (!ok)
    throw std::runtime_error(why);
}
void Near(double a, double b, const char *what) {
  Require(std::abs(a - b) < 1e-10 * (std::max)(1.0, std::abs(b)), what);
}
std::shared_ptr<CHole> MakeHole(UnifiedModel &model) {
  model.unit = UnitType::MILLIMETER;
  auto support = std::make_shared<CRefPlane>();
  support->targetFeatureID = StandardID::PLANE_XY;
  support->normal = {0, 0, 1};
  support->xDir = {1, 0, 0};
  support->yDir = {0, 1, 0};
  SweepExtent extent;
  extent.type = SweepExtent::Type::VALUE;
  extent.value = 20;
  Builder::HoleBuilder builder(model, "hole");
  builder.SetSupportReference(support)
      .SetDirection({0, 0, -1})
      .AddLocation({10, 15, 0})
      .SetDiameter(10)
      .SetExtent(extent)
      .SetBottomKind(HoleBottomKind::Conical)
      .SetDepthReference(HoleDepthReference::Shoulder)
      .SetDrillPointAngle(118 * std::acos(-1.0) / 180.0);
  builder.Build();
  return builder.GetFeature();
}
int main(int argc, char **argv) {
  try {
    const auto output =
        std::filesystem::path(argc > 1 ? argv[1] : "tmp/hole-tests");
    Accessor::HoleAccessor invalid(std::shared_ptr<const CFeatureBase>{});
    bool rejected = false;
    try {
      (void)invalid.GetDiameter();
    } catch (const std::exception &) {
      rejected = true;
    }
    Require(rejected, "invalid Hole accessor must reject access safely");
    std::filesystem::create_directories(output);
    UnifiedModel model;
    auto hole = MakeHole(model);
    Require(model.Validate().isValid, "valid conical hole");
    HoleResolvedGeometry g;
    std::string error;
    Require(ResolveHoleGeometry(*hole, g, error), error);
    Near(g.tipHeight, 3.004303095137802, "drill tip");
    Near(g.shoulderDepth, 20, "shoulder");
    Near(g.tipDepth, 23.004303095137802, "tip");
    UnifiedModel datumModel;
    auto datum = std::make_shared<CDatumPlane>();
    datum->featureID = "entry_datum";
    datum->method = PlaneMethod::OFFSET;
    datum->projectedOrigin = CPoint3D{0, 0, 40};
    datum->normal = CVector3D{0, 0, 1};
    datum->referenceEntities.push_back(hole->supportRef);
    datumModel.AddFeature(datum);
    auto datumHole = MakeHole(datumModel);
    auto idOnlyPlane = std::make_shared<CRefPlane>();
    idOnlyPlane->targetFeatureID = datum->featureID;
    datumHole->supportRef = idOnlyPlane;
    datumHole->locations[0].entryPoint.z = 40;
    Require(datumModel.Validate().isValid,
            "ID-only support resolves an existing datum plane");
    UnifiedModel forwardModel;
    auto forwardHole = MakeHole(forwardModel);
    forwardHole->supportRef = idOnlyPlane;
    forwardHole->locations[0].entryPoint.z = 40;
    forwardModel.AddFeature(datum);
    Require(!forwardModel.Validate().isValid,
            "forward datum reference remains invalid");
    hole->depthReference = HoleDepthReference::Tip;
    hole->extent.value = g.tipDepth;
    Require(ResolveHoleGeometry(*hole, g, error), error);
    Near(g.shoulderDepth, 20, "tip reference");
    hole->diameter = 6.6;
    hole->countersink = CHoleCountersink{12.6, std::acos(-1.0) / 2};
    Require(ResolveHoleGeometry(*hole, g, error), error);
    Near(g.entryHeight, 3, "countersink height");
    Require(model.Validate().isValid, "valid countersink");
    hole->counterbore = CHoleCounterbore{11, 6};
    Require(!model.Validate().isValid, "mutually exclusive entrances");
    hole->countersink.reset();
    hole->counterbore->depth = 100;
    Require(!model.Validate().isValid, "entrance beyond shoulder");
    hole->counterbore->depth = 6;
    hole->extent.value = 0.1;
    Require(!model.Validate().isValid, "negative cylinder length");
    hole->extent.value = 20;
    hole->depthReference = HoleDepthReference::Unknown;
    Require(!model.Validate().isValid, "unknown blind depth reference");
    hole->extent.type = SweepExtent::Type::THROUGH_ALL;
    hole->extent.value = std::numeric_limits<double>::quiet_NaN();
    hole->drillPointAngle = std::numeric_limits<double>::quiet_NaN();
    Require(model.Validate().isValid, "through ignores inactive bottom/depth");
    const auto throughJson = Builder::FeatureFormatter::ToJson(hole);
    CEREAL_RAPIDJSON_NAMESPACE::Document jsonDocument;
    jsonDocument.Parse(throughJson.c_str());
    Require(!jsonDocument.HasParseError() && jsonDocument.HasMember("Feature"),
            "inactive NaNs must not corrupt cereal JSON");
    auto throughPath = output / "through.cereal.xml";
    Require(SaveModel(model, throughPath, &error, SerializationFormat::CEREAL),
            error);
    UnifiedModel throughLoaded;
    Require(LoadModel(throughLoaded, throughPath, &error,
                      SerializationFormat::CEREAL),
            error);
    auto throughHole =
        std::dynamic_pointer_cast<CHole>(throughLoaded.GetFeatures()[0]);
    Require(std::isfinite(throughHole->extent.value) &&
                !throughHole->drillPointAngle,
            "inactive metadata normalized during persistence");
    hole->locations.push_back(hole->locations.front());
    Require(!model.Validate().isValid, "multiple positions unsupported");
    hole->locations.pop_back();
    hole->direction = {0, 0, -2};
    Require(!model.Validate().isValid, "unit direction required");
    hole->direction = {0, 0, -1};
    hole->locations.front().entryPoint.z = 1;
    Require(!model.Validate().isValid, "entry lies on support");
    hole->locations.front().entryPoint.z = 0;
    hole->diameter = std::numeric_limits<double>::infinity();
    Require(!model.Validate().isValid, "finite diameter");
    hole->diameter = 6.6;
    hole->extent.type = SweepExtent::Type::VALUE;
    hole->extent.value = 20;
    hole->depthReference = HoleDepthReference::Shoulder;
    hole->drillPointAngle = 118 * std::acos(-1.0) / 180;
    UnifiedModel aliasModel;
    auto aliasA = MakeHole(aliasModel);
    auto aliasB = MakeHole(aliasModel);
    auto aliasPlane = std::dynamic_pointer_cast<CRefPlane>(aliasA->supportRef);
    aliasPlane->origin.x = 25.4;
    aliasB->supportRef = aliasPlane;
    Require(ConvertModelUnit(aliasModel, UnitType::INCH, &error), error);
    Near(aliasPlane->origin.x, 1, "shared support must scale once");
    std::string unitError;
    Require(ConvertModelUnit(model, UnitType::INCH, &unitError), unitError);
    Near(hole->diameter, 6.6 / 25.4, "diameter units");
    Near(hole->locations[0].entryPoint.x, 10 / 25.4, "point units");
    Near(*hole->drillPointAngle, 118 * std::acos(-1.0) / 180,
         "angles do not scale");
    Require(ConvertModelUnit(model, UnitType::MILLIMETER, &unitError),
            unitError);
    Near(hole->diameter, 6.6, "unit roundtrip");
    for (auto format :
         {SerializationFormat::TINYXML, SerializationFormat::CEREAL}) {
      auto path =
          output / (format == SerializationFormat::TINYXML ? "hole.tinyxml.xml"
                                                           : "hole.cereal.xml");
      Require(SaveModel(model, path, &error, format), error);
      UnifiedModel loaded;
      Require(LoadModel(loaded, path, &error, format), error);
      Require(loaded.GetFeatures().size() == 1, "hole preserved");
      Accessor::HoleAccessor acc(loaded.GetFeatures()[0]);
      Require(acc.IsValid(), "polymorphic hole");
      Near(acc.GetDiameter(), 6.6, "persisted diameter");
      Near(acc.GetCounterbore()->depth, 6, "persisted cbore");
      Require(acc.GetDepthReference() == HoleDepthReference::Shoulder,
              "persisted reference");
      Near(acc.GetLocations()[0].entryPoint.y, 15, "persisted point");
    }
    hole->bottom = HoleBottomKind::Flat;
    hole->drillPointAngle = std::numeric_limits<double>::quiet_NaN();
    Require(model.Validate().isValid, "flat bottom ignores inactive tip angle");
    jsonDocument.Parse(Builder::FeatureFormatter::ToJson(hole).c_str());
    Require(!jsonDocument.HasParseError() && jsonDocument.HasMember("Feature"),
            "flat inactive angle must not corrupt JSON");
    UnifiedModel pointModel;
    auto pointHole = MakeHole(pointModel);
    pointHole->locations[0].entryPoint = {0, 0, 0};
    auto pointRef = std::make_shared<CRefPoint>();
    pointRef->targetFeatureID = StandardID::ORIGIN;
    pointHole->locations[0].positionRef = pointRef;
    for (auto format :
         {SerializationFormat::TINYXML, SerializationFormat::CEREAL}) {
      auto path = output / (format == SerializationFormat::TINYXML
                                ? "point.tinyxml.xml"
                                : "point.cereal.xml");
      Require(SaveModel(pointModel, path, &error, format), error);
      UnifiedModel loaded;
      Require(LoadModel(loaded, path, &error, format), error);
      auto loadedHole =
          std::dynamic_pointer_cast<CHole>(loaded.GetFeatures()[0]);
      auto loadedPoint = std::dynamic_pointer_cast<CRefPoint>(
          loadedHole->locations[0].positionRef);
      Require(loadedPoint && loadedPoint->targetFeatureID == StandardID::ORIGIN,
              "point reference polymorphic roundtrip");
    }
    pointRef->targetFeatureID = "missing_point";
    Require(!pointModel.Validate().isValid,
            "missing location dependency is rejected");
    const auto legacyPath = output / "legacy-schema-1.xml";
    std::ofstream(legacyPath)
        << R"(<UnifiedModel UnitSystem="Millimeter" ModelName="legacy" FeatureCount="1" SchemaVersion="1">
<Feature ID="old_sketch" Name="legacy sketch" Suppressed="false" Type="Sketch"><Segments/></Feature>
</UnifiedModel>)";
    UnifiedModel oldModel;
    Require(
        LoadModel(oldModel, legacyPath, &error, SerializationFormat::TINYXML),
        error);
    Require(oldModel.GetFeatures().size() == 1 &&
                oldModel.GetFeatures()[0]->featureType == FeatureType::Sketch,
            "pre-hole schema loads without new fields");
    Require(static_cast<int>(FeatureType::MirrorPattern) == 13,
            "old feature enum numbers stable");
    Require(static_cast<int>(FeatureType::Hole) == 14, "hole appended");
    std::cout << "hole model contract tests passed\n";
    return 0;
  } catch (const std::exception &ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  }
}
