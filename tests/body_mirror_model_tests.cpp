#include "BodyGeometry.h"
#include "CADSerializer.h"
#include "PatternAccessor.h"
#include "PatternBuilder.h"
#include "ReferenceBuilder.h"
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>
using namespace CADExchange;
namespace {
void Require(bool ok, const std::string &message) {
  if (!ok)
    throw std::runtime_error(message);
}
void Near(double a, double b, const std::string &message) {
  Require(std::abs(a - b) < 1e-10 * (std::max)(1.0, std::abs(b)), message);
}
std::shared_ptr<CRefBody> Body() {
  return Builder::Ref::Body()
      .BoundsMin(0, 0, 0)
      .BoundsMax(10, 20, 30)
      .Volume(6000)
      .VolumeCentroid(5, 10, 15)
      .SurfaceArea(2200)
      .SurfacePoints({{0, 5, 5}, {10, 5, 5}, {5, 20, 5}, {5, 5, 30}})
      .SourceFeatureIDs({"diagnostic-source-missing-in-target"})
      .Build();
}
std::shared_ptr<CMirrorPattern>
BodyMirror(UnifiedModel &model, const std::shared_ptr<CRefBody> &body,
           bool merge = false) {
  Builder::MirrorPatternBuilder builder(model, "body mirror");
  builder.SetMirrorPlane(Builder::Ref::Plane(StandardID::PLANE_XY))
      .SetScope(PatternScope::BODIES)
      .SetMergeResults(merge)
      .AddSeedObject(body);
  builder.Build();
  return builder.GetFeature();
}
void CheckBody(const CRefBody &body) {
  Near(body.boundsMin.x, 0, "bounds min");
  Near(body.boundsMax.y, 20, "bounds max");
  Near(body.volume, 6000, "volume");
  Near(body.volumeCentroid.z, 15, "centroid");
  Near(body.surfaceArea, 2200, "surface area");
  Require(body.surfacePoints.size() == 4, "point count");
  Near(body.surfacePoints[2].y, 20, "point position");
  Require(body.sourceFeatureIDs ==
              std::vector<std::string>{"diagnostic-source-missing-in-target"},
          "diagnostic IDs");
}
void Roundtrip(const UnifiedModel &model, const std::filesystem::path &out,
               const std::string &name) {
  for (auto format :
       {SerializationFormat::TINYXML, SerializationFormat::CEREAL}) {
    const auto file =
        out / (name + (format == SerializationFormat::TINYXML ? "-tiny.xml"
                                                              : "-cereal.xml"));
    std::string error;
    Require(SaveModel(model, file, &error, format), "save: " + error);
    UnifiedModel loaded;
    Require(LoadModel(loaded, file, &error, format), "load: " + error);
    Require(loaded.GetFeatures().size() == model.GetFeatures().size(),
            "feature count");
    for (size_t i = 0; i < model.GetFeatures().size(); ++i) {
      const auto before =
          std::dynamic_pointer_cast<CMirrorPattern>(model.GetFeatures()[i]);
      const auto after =
          std::dynamic_pointer_cast<CMirrorPattern>(loaded.GetFeatures()[i]);
      Require(after != nullptr, "mirror type");
      Require(after->scope == before->scope &&
                  after->geometryPattern == before->geometryPattern,
              "mirror scope");
      Require(after->mergeResults == before->mergeResults,
              "merge option roundtrip");
      Require(after->affectedBodyMode == before->affectedBodyMode,
              "affected mode roundtrip");
      Require(after->seedObjects.size() == before->seedObjects.size(),
              "seed count roundtrip");
      Require(after->affectedBodies.size() == before->affectedBodies.size(),
              "affected bodies count roundtrip");
      for (const auto &ref : after->seedObjects)
        if (auto body = std::dynamic_pointer_cast<CRefBody>(ref))
          CheckBody(*body);
      for (const auto &body : after->affectedBodies)
        CheckBody(*body);
    }
  }
}
void XmlReject(const std::filesystem::path &source,
               const std::filesystem::path &out,
               const std::function<void(tinyxml2::XMLElement *)> &mutate,
               const std::string &name) {
  tinyxml2::XMLDocument doc;
  Require(doc.LoadFile(source.string().c_str()) == tinyxml2::XML_SUCCESS,
          "load fixture");
  auto *feature =
      doc.FirstChildElement("UnifiedModel")->FirstChildElement("Feature");
  mutate(feature);
  const auto file = out / ("bad-" + name + ".xml");
  Require(doc.SaveFile(file.string().c_str()) == tinyxml2::XML_SUCCESS,
          "save malformed fixture");
  std::string error;
  UnifiedModel loaded;
  Require(!LoadModel(loaded, file, &error, SerializationFormat::TINYXML),
          "must reject malformed XML: " + name);
  Require(!error.empty(), "must explain malformed XML: " + name);
}
void CerealRejectMissing(const std::filesystem::path &source,
                         const std::filesystem::path &out, const char *field,
                         bool bodyField) {
  tinyxml2::XMLDocument doc;
  Require(doc.LoadFile(source.string().c_str()) == tinyxml2::XML_SUCCESS,
          "load cereal fixture");
  auto *data = doc.FirstChildElement("cereal")
                   ->FirstChildElement("Feature")
                   ->FirstChildElement("ptr_wrapper")
                   ->FirstChildElement("data");
  if (bodyField)
    data = data->FirstChildElement("SeedObjects")
               ->FirstChildElement("value0")
               ->FirstChildElement("ptr_wrapper")
               ->FirstChildElement("data");
  Require(data->FirstChildElement(field) != nullptr,
          "cereal field exists before deletion");
  data->DeleteChild(data->FirstChildElement(field));
  auto file = out / (std::string("bad-cereal-") + field + ".xml");
  Require(doc.SaveFile(file.string().c_str()) == tinyxml2::XML_SUCCESS,
          "save cereal fixture");
  UnifiedModel loaded;
  std::string error;
  Require(!LoadModel(loaded, file, &error, SerializationFormat::CEREAL),
          std::string("cereal must reject absent field: ") + field);
}
} // namespace
int main(int argc, char **argv) {
  try {
    static_assert(static_cast<int>(RefType::UNKNOWN) == 8,
                  "existing RefType must stay stable");
    static_assert(static_cast<int>(RefType::TOPO_BODY) == 9,
                  "append Body RefType");
    static_assert(!std::is_base_of<CRefFeature, CRefBody>::value,
                  "Body must not inherit feature ID");
    static_assert(!std::is_base_of<CRefSubTopo, CRefBody>::value,
                  "Body must not require a parent feature");
    const auto out = std::filesystem::path(
        argc > 1 ? argv[1] : "../tmp/body-mirror-model-tests");
    std::filesystem::create_directories(out);
    auto body = Body();
    std::string detail;
    Require(ValidateBodyReference(*body, detail), detail);
    Accessor::ReferenceAccessor ref(body);
    auto bodyAccessor = ref.AsBody();
    Require(bodyAccessor.IsValid() && ref.GetParentFeatureID().empty() &&
                ref.GetTargetFeatureID().empty(),
            "Body accessor identity");
    Near(bodyAccessor.GetVolume(), 6000, "Body accessor volume");
    Accessor::BodyRefAccessor invalid;
    bool thrown = false;
    try {
      (void)invalid.GetVolume();
    } catch (const std::exception &) {
      thrown = true;
    }
    Require(thrown, "invalid Body accessor must reject access");
    for (int i = 0; i < 9; ++i) {
      auto bad = *body;
      if (i == 0)
        bad.volume = 0;
      if (i == 1)
        bad.volume = std::numeric_limits<double>::quiet_NaN();
      if (i == 2)
        bad.surfaceArea = -1;
      if (i == 3)
        bad.boundsMax.x = -1;
      if (i == 4)
        bad.volumeCentroid.x = std::numeric_limits<double>::infinity();
      if (i == 5)
        bad.surfacePoints = {{1, 1, 1}, {1, 1, 1}, {2, 2, 2}};
      if (i == 6)
        bad.surfacePoints = {{0, 0, 0}, {1, 1, 1}, {2, 2, 2}};
      if (i == 7)
        bad.surfacePoints[0].x = 100;
      if (i == 8)
        bad.refType = RefType::TOPO_FACE;
      Require(!ValidateBodyReference(bad, detail),
              "invalid body fingerprint " + std::to_string(i));
    }
    UnifiedModel model(UnitType::MILLIMETER);
    auto mirror = BodyMirror(model, body);
    Require(model.Validate().isValid,
            "valid Body mirror (source IDs diagnostic only)");
    Accessor::MirrorPatternAccessor acc(mirror);
    Require(acc.GetMergeResults() == std::optional<bool>(false),
            "accessor merge false presence");
    Roundtrip(model, out, "body");
    for (const auto &field : {"BoundsMin", "BoundsMax", "Volume",
                              "VolumeCentroid", "SurfaceArea", "SurfacePoints"})
      CerealRejectMissing(out / "body-cereal.xml", out, field, true);
    CerealRejectMissing(out / "body-cereal.xml", out, "MergeResults", false);
    auto saved = *mirror;
    auto validBody = *body;
    body->volume = -1;
    Require(!model.Validate().isValid,
            "Body geometry invalidity propagates to model validation");
    std::string rejectedError;
    Require(!SaveModel(model, out / "invalid-save.xml", &rejectedError,
                       SerializationFormat::TINYXML),
            "invalid Body must fail default saving");
    *body = validBody;
    mirror->seedObjects.clear();
    Require(!model.Validate().isValid, "empty mirror seeds");
    *mirror = saved;
    mirror->mergeResults.reset();
    Require(!model.Validate().isValid, "missing merge policy");
    *mirror = saved;
    mirror->seedObjects = {std::make_shared<CRefFeature>()};
    Require(!model.Validate().isValid, "feature seed is not body");
    *mirror = saved;
    mirror->seedObjects.push_back(nullptr);
    Require(!model.Validate().isValid, "partial null seeds rejected");
    *mirror = saved;
    mirror->seedObjects.push_back(body);
    Require(!model.Validate().isValid, "duplicate Body seed");
    *mirror = saved;
    mirror->scope = PatternScope::FACES;
    Require(!model.Validate().isValid, "body seed cannot be face");
    *mirror = saved;
    mirror->geometryPattern = true;
    Require(!model.Validate().isValid, "inactive body geometry mode");
    *mirror = saved;
    mirror->affectedBodyMode = MirrorAffectedBodyMode::AllBodies;
    Require(!model.Validate().isValid, "inactive affected options");
    *mirror = saved;
    mirror->mirrorPlaneRef = body;
    Require(!model.Validate().isValid, "body is not mirror plane");
    *mirror = saved;
    mirror->scope = static_cast<PatternScope>(500);
    Require(!model.Validate().isValid, "invalid mirror scope enum");
    *mirror = saved;
    const auto fixture = out / "body-tiny.xml";
    for (const auto &field :
         {"BoundsMin", "BoundsMax", "Volume", "VolumeCentroid", "SurfaceArea"})
      XmlReject(
          fixture, out,
          [&](auto *f) {
            f->FirstChildElement("SeedObjects")
                ->FirstChildElement("ReferenceEntity")
                ->DeleteAttribute(field);
          },
          field);
    for (const auto &field : {"MergeResults", "Scope", "GeometryPattern"})
      XmlReject(
          fixture, out, [&](auto *f) { f->DeleteAttribute(field); }, field);
    XmlReject(
        fixture, out,
        [](auto *f) { f->SetAttribute("MergeResults", "garbage"); },
        "merge-invalid");
    XmlReject(
        fixture, out, [](auto *f) { f->SetAttribute("Scope", "TYPO"); },
        "scope-invalid");
    XmlReject(
        fixture, out,
        [](auto *f) {
          auto *r = f->FirstChildElement("SeedObjects")
                        ->FirstChildElement("ReferenceEntity");
          r->DeleteChild(r->FirstChildElement("SurfacePoints"));
        },
        "points-missing");
    XmlReject(
        fixture, out,
        [](auto *f) {
          f->FirstChildElement("SeedObjects")
              ->FirstChildElement("ReferenceEntity")
              ->SetAttribute("BoundsMin", "(0,0,0)garbage");
        },
        "point-malformed");
    XmlReject(
        fixture, out,
        [](auto *f) {
          auto *seed = f->GetDocument()->NewElement("ReferenceEntity");
          seed->SetAttribute("Type", "UnknownBody");
          f->FirstChildElement("SeedObjects")->InsertEndChild(seed);
        },
        "partial-seeds");
    auto second = BodyMirror(model, body, true);
    Builder::MirrorPatternBuilder geom(model, "geometry mirror");
    auto featureSeed = std::make_shared<CRefFeature>();
    featureSeed->targetFeatureID = mirror->featureID;
    geom.SetMirrorPlane(Builder::Ref::Plane(StandardID::PLANE_XY))
        .AddSeedObject(featureSeed)
        .SetGeometryPattern(true)
        .SetAffectedBodyMode(MirrorAffectedBodyMode::SelectedBodiesOnly)
        .AddAffectedBody(body);
    geom.Build();
    auto geometry = geom.GetFeature();
    Require(model.Validate().isValid, "geometry selected affected-body scope");
    auto validFeatureSeed = *featureSeed;
    featureSeed->targetFeatureID = "missing-seed";
    Require(!model.Validate().isValid, "missing mirror feature dependency");
    *featureSeed = validFeatureSeed;
    geometry->mergeResults = false;
    Require(!model.Validate().isValid, "inactive feature mirror merge option");
    geometry->mergeResults.reset();
    geometry->affectedBodies.push_back(body);
    Require(!model.Validate().isValid, "duplicate affected body");
    geometry->affectedBodies.pop_back();
    Roundtrip(model, out, "selected-only");
    geometry->affectedBodyMode =
        MirrorAffectedBodyMode::SelectedBodiesWithAutoSelect;
    Roundtrip(model, out, "selected-auto");
    geometry->affectedBodyMode = MirrorAffectedBodyMode::AllBodies;
    geometry->affectedBodies.clear();
    Roundtrip(model, out, "all-bodies");
    geometry->affectedBodies.push_back(body);
    Require(!model.Validate().isValid, "all bodies forbids list");
    geometry->affectedBodyMode.reset();
    Require(!model.Validate().isValid, "geometry needs affected mode");
    geometry->affectedBodyMode = MirrorAffectedBodyMode::SelectedBodiesOnly;
    geometry->affectedBodies.clear();
    Require(!model.Validate().isValid, "selected scope requires bodies");
    geometry->affectedBodies.push_back(body);
    XmlReject(
        out / "selected-only-tiny.xml", out,
        [](auto *f) {
          while (f->NextSiblingElement("Feature"))
            f = f->NextSiblingElement("Feature");
          f->DeleteAttribute("AffectedBodyMode");
        },
        "affected-mode-missing");
    Require(ConvertModelUnit(model, UnitType::METER, &detail), detail);
    Near(body->volume, 0.000006, "shared body volume scales once");
    Near(body->surfaceArea, 0.0022, "body area squared scale");
    Near(body->boundsMax.y, 0.02, "shared body bounds scale once");
    Near(body->surfacePoints[3].z, 0.03, "surface point scale");
    Require(ConvertModelUnit(model, UnitType::MILLIMETER, &detail), detail);
    CheckBody(*body);
    Require(model.Validate().isValid, "unit roundtrip mirror validation");
    UnifiedModel faceModel(UnitType::MILLIMETER);
    Builder::MirrorPatternBuilder faceBuilder(faceModel, "face mirror");
    auto face = std::make_shared<CRefFace>();
    face->normal = {0, 0, 1};
    faceBuilder.SetMirrorPlane(Builder::Ref::Plane(StandardID::PLANE_XY))
        .SetScope(PatternScope::FACES)
        .AddSeedObject(face);
    faceBuilder.Build();
    Require(faceModel.Validate().isValid, "nongeometry face mirror");
    Roundtrip(faceModel, out, "face");
    face->surfaceType = CGeoSurfaceType::CYLINDER;
    face->normal = {0, 0, 0};
    Require(faceModel.Validate().isValid,
            "Valid cylindrical mirror seed may have no constant face normal");
    Roundtrip(faceModel, out, "curved-zero-normal");
    face->surfaceType = CGeoSurfaceType::PLANE;
    face->normal = {0, 0, 1};

    const auto faceFixture = out / "face-tiny.xml";
    XmlReject(
        faceFixture, out,
        [](auto *f) {
          f->FirstChildElement("SeedObjects")
              ->FirstChildElement("ReferenceEntity")
              ->DeleteAttribute("Center");
        },
        "face-seed-center-missing");
    XmlReject(
        faceFixture, out,
        [](auto *f) {
          f->FirstChildElement("SeedObjects")
              ->FirstChildElement("ReferenceEntity")
              ->SetAttribute("Center", "(0,0,0)garbage");
        },
        "face-seed-center-malformed");
    XmlReject(
        faceFixture, out,
        [](auto *f) {
          f->FirstChildElement("SeedObjects")
              ->FirstChildElement("ReferenceEntity")
              ->DeleteAttribute("SurfaceType");
        },
        "face-seed-type-missing");
    XmlReject(
        faceFixture, out,
        [](auto *f) {
          f->FirstChildElement("SeedObjects")
              ->FirstChildElement("ReferenceEntity")
              ->SetAttribute("SurfaceType", "unknown");
        },
        "face-seed-type-unknown");
    XmlReject(
        faceFixture, out,
        [](auto *f) {
          f->FirstChildElement("SeedObjects")
              ->FirstChildElement("ReferenceEntity")
              ->SetAttribute("Normal", "(0,0,1)garbage");
        },
        "face-seed-normal-malformed");
    auto faceMirror = faceBuilder.GetFeature();
    faceMirror->mirrorPlaneRef = face;
    Roundtrip(faceModel, out, "face-plane");
    const auto facePlaneFixture = out / "face-plane-tiny.xml";
    for (const auto &field : {"Center", "Normal", "SurfaceType"}) {
      XmlReject(
          facePlaneFixture, out,
          [&](auto *f) {
            f->FirstChildElement("MirrorPlaneReference")
                ->DeleteAttribute(field);
          },
          std::string("face-plane-missing-") + field);
    }
    XmlReject(
        facePlaneFixture, out,
        [](auto *f) {
          f->FirstChildElement("MirrorPlaneReference")
              ->SetAttribute("Center", "(0,0,0)garbage");
        },
        "face-plane-center-malformed");
    XmlReject(
        facePlaneFixture, out,
        [](auto *f) {
          f->FirstChildElement("MirrorPlaneReference")
              ->SetAttribute("Normal", "(0,0,1)garbage");
        },
        "face-plane-normal-malformed");
    XmlReject(
        facePlaneFixture, out,
        [](auto *f) {
          f->FirstChildElement("MirrorPlaneReference")
              ->SetAttribute("Normal", "(0,0,0)");
        },
        "face-plane-normal-zero");
    for (const auto &field : {"Origin", "Normal"}) {
      XmlReject(
          fixture, out,
          [&](auto *f) {
            f->FirstChildElement("MirrorPlaneReference")
                ->DeleteAttribute(field);
          },
          std::string("datum-plane-missing-") + field);
      XmlReject(
          fixture, out,
          [&](auto *f) {
            f->FirstChildElement("MirrorPlaneReference")
                ->SetAttribute(field, "(0,0,1)garbage");
          },
          std::string("datum-plane-malformed-") + field);
    }

    std::cout << "PASS Body/Mirror validation, accessors, units, strict "
                 "TinyXML and cereal roundtrips\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL " << e.what() << "\n";
    return 1;
  }
}
