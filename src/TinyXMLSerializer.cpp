#include "TinyXMLSerializer.h"
#include <algorithm>
#include <cctype>
#include <functional>
#include <optional>
#include <sstream>

namespace CADExchange {

using namespace tinyxml2;

namespace {
std::string FormatTriple(double x, double y, double z) {
  std::ostringstream ss;
  ss << '(' << x << ',' << y << ',' << z << ')';
  return ss.str();
}

bool TryParseTriple(const char *text, double &x, double &y, double &z) {
  if (!text)
    return false;
  std::string str = text;
  if (!str.empty() && str.front() == '(' && str.back() == ')') {
    str = str.substr(1, str.size() - 2);
  }
  size_t first = str.find(',');
  if (first == std::string::npos)
    return false;
  size_t second = str.find(',', first + 1);
  if (second == std::string::npos)
    return false;

  try {
    x = std::stod(str.substr(0, first));
    y = std::stod(str.substr(first + 1, second - first - 1));
    z = std::stod(str.substr(second + 1));
  } catch (...) {
    return false;
  }
  return true;
}

std::string ToLower(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return str;
}

std::string UnitTypeToString(UnitType type) {
  switch (type) {
  case UnitType::METER:
    return "Meter";
  case UnitType::CENTIMETER:
    return "Centimeter";
  case UnitType::MILLIMETER:
    return "Millimeter";
  case UnitType::INCH:
    return "Inch";
  case UnitType::FOOT:
    return "Foot";
  }
  return "Unknown";
}

std::optional<UnitType> UnitTypeFromString(const char *text) {
  if (!text)
    return std::nullopt;
  std::string value = ToLower(text);
  if (value == "meter")
    return UnitType::METER;
  if (value == "centimeter")
    return UnitType::CENTIMETER;
  if (value == "millimeter")
    return UnitType::MILLIMETER;
  if (value == "inch")
    return UnitType::INCH;
  if (value == "foot")
    return UnitType::FOOT;
  return std::nullopt;
}

std::string BooleanOpToString(BooleanOp op) {
  switch (op) {
  case BooleanOp::BOSS:
    return "BOSS";
  case BooleanOp::CUT:
    return "Cut";
  case BooleanOp::MERGE:
    return "Merge";
  }
  return "Unknown";
}

std::optional<BooleanOp> BooleanOpFromString(const char *text) {
  if (!text)
    return std::nullopt;
  std::string value = ToLower(text);
  if (value == "boss")
    return BooleanOp::BOSS;
  if (value == "cut")
    return BooleanOp::CUT;
  if (value == "merge")
    return BooleanOp::MERGE;
  return std::nullopt;
}

std::string ExtrudeEndConditionTypeToString(ExtrudeEndCondition::Type type) {
  switch (type) {
  case ExtrudeEndCondition::Type::BLIND:
    return "Blind";
  case ExtrudeEndCondition::Type::THROUGH_ALL:
    return "ThroughAll";
  case ExtrudeEndCondition::Type::UP_TO_NEXT:
    return "UpToNext";
  case ExtrudeEndCondition::Type::UP_TO_FACE:
    return "UpToFace";
  case ExtrudeEndCondition::Type::UP_TO_VERTEX:
    return "UpToVertex";
  case ExtrudeEndCondition::Type::MID_PLANE:
    return "MidPlane";
  }
  return "Unknown";
}

std::optional<ExtrudeEndCondition::Type>
ExtrudeEndConditionTypeFromString(const char *text) {
  if (!text)
    return std::nullopt;
  std::string value = ToLower(text);
  if (value == "blind")
    return ExtrudeEndCondition::Type::BLIND;
  if (value == "throughall")
    return ExtrudeEndCondition::Type::THROUGH_ALL;
  if (value == "uptonext")
    return ExtrudeEndCondition::Type::UP_TO_NEXT;
  if (value == "uptoface")
    return ExtrudeEndCondition::Type::UP_TO_FACE;
  if (value == "uptovertex")
    return ExtrudeEndCondition::Type::UP_TO_VERTEX;
  if (value == "midplane")
    return ExtrudeEndCondition::Type::MID_PLANE;
  return std::nullopt;
}
} // namespace

// =================================================================================================
// Save Implementation
// =================================================================================================

bool TinyXMLSerializer::Save(const UnifiedModel &model,
                             const std::filesystem::path &filePath,
                             std::string *errorMessage) {
  XMLDocument doc;

  // Declaration
  doc.InsertFirstChild(doc.NewDeclaration());

  // Root Element: UnifiedModel
  XMLElement *root = doc.NewElement("UnifiedModel");
  doc.InsertEndChild(root);

  // Attributes
  root->SetAttribute("UnitSystem", UnitTypeToString(model.unit).c_str());
  root->SetAttribute("ModelName", model.modelName.c_str());
  root->SetAttribute("FeatureCount",
                     static_cast<int64_t>(model.GetFeatures().size()));

  // Features
  for (const auto &feature : model.GetFeatures()) {
    SaveFeature(doc, root, feature);
  }

  XMLError result = doc.SaveFile(filePath.string().c_str());
  if (result != XML_SUCCESS) {
    if (errorMessage)
      *errorMessage = doc.ErrorStr();
    return false;
  }
  return true;
}

void TinyXMLSerializer::SavePoint3D(XMLElement *element, const char *name,
                                    const CPoint3D &pt) {
  std::string value = FormatTriple(pt.x, pt.y, pt.z);
  element->SetAttribute(name, value.c_str());
}

void TinyXMLSerializer::SaveVector3D(XMLElement *element, const char *name,
                                     const CVector3D &vec) {
  std::string value = FormatTriple(vec.x, vec.y, vec.z);
  element->SetAttribute(name, value.c_str());
}

std::string FormatPoint(const CPoint3D &pt) {
  return FormatTriple(pt.x, pt.y, pt.z);
}

std::string FormatVector(const CVector3D &vec) {
  return FormatTriple(vec.x, vec.y, vec.z);
}

CPoint3D ParsePointAttribute(XMLElement *element, const char *name) {
  CPoint3D pt;
  double x, y, z;
  if (TryParseTriple(element->Attribute(name), x, y, z)) {
    pt.x = x;
    pt.y = y;
    pt.z = z;
  }
  return pt;
}

CVector3D ParseVectorAttribute(XMLElement *element, const char *name) {
  CVector3D vec;
  double x, y, z;
  if (TryParseTriple(element->Attribute(name), x, y, z)) {
    vec.x = x;
    vec.y = y;
    vec.z = z;
  }
  return vec;
}

std::shared_ptr<CRefFeature> LoadFeatureReference(XMLElement *element,
                                                  RefType refType) {
  auto reference = std::make_shared<CRefFeature>(refType);
  if (const char *tid = element->Attribute("TargetFeatureID"))
    reference->targetFeatureID = tid;
  return reference;
}

void SaveFeatureReference(XMLElement *element,
                          const std::shared_ptr<CRefEntityBase> &ref) {
  if (auto feature = std::dynamic_pointer_cast<CRefFeature>(ref)) {
    element->SetAttribute("TargetFeatureID", feature->targetFeatureID.c_str());
  }
}

CVector3D ComputePlaneYAxis(const CVector3D &normal, const CVector3D &xDir) {
  CVector3D yDir = CVector3D::Cross(normal, xDir);
  yDir.Normalize();
  return yDir;
}

struct RefSerializerEntry {
  using RefSaveFn = std::function<void(
      XMLElement *, const std::shared_ptr<CRefEntityBase> &)>;
  using RefLoadFn =
      std::function<std::shared_ptr<CRefEntityBase>(XMLElement *)>;

  RefType type;
  const char *name;
  const char *lowerName;
  RefSaveFn save;
  RefLoadFn load;
};
/**
 * @brief 引用实体序列化注册表。
 * 通过 RefType 注册对应的序列化函数。
 */
static const RefSerializerEntry kRefSerializerEntries[] = {
    {RefType::FEATURE_DATUM_PLANE, "Plane", "plane",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       if (auto plane = std::dynamic_pointer_cast<CRefPlane>(ref)) {
         element->SetAttribute("TargetFeatureID",
                               plane->targetFeatureID.c_str());
         element->SetAttribute("Origin", FormatPoint(plane->origin).c_str());
         element->SetAttribute("XDir", FormatVector(plane->xDir).c_str());
         element->SetAttribute("YDir", FormatVector(plane->yDir).c_str());
         element->SetAttribute("Normal", FormatVector(plane->normal).c_str());
       }
     },
     [](XMLElement *element) {
       auto plane = std::make_shared<CRefPlane>();
       if (const char *tid = element->Attribute("TargetFeatureID"))
         plane->targetFeatureID = tid;
       plane->origin = ParsePointAttribute(element, "Origin");
       plane->xDir = ParseVectorAttribute(element, "XDir");
       plane->normal = ParseVectorAttribute(element, "Normal");
       if (element->Attribute("YDir")) {
         plane->yDir = ParseVectorAttribute(element, "YDir");
       } else {
         plane->yDir = ComputePlaneYAxis(plane->normal, plane->xDir);
       }
       return plane;
     }},
    {RefType::FEATURE_DATUM_AXIS, "Axis", "axis",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       SaveFeatureReference(element, ref);
     },
     [](XMLElement *element) {
       return LoadFeatureReference(element, RefType::FEATURE_DATUM_AXIS);
     }},
    {RefType::FEATURE_DATUM_POINT, "Point", "point",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       SaveFeatureReference(element, ref);
     },
     [](XMLElement *element) {
       return LoadFeatureReference(element, RefType::FEATURE_DATUM_POINT);
     }},
    {RefType::FEATURE_WHOLE_SKETCH, "Sketch", "sketch",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       if (auto sketch = std::dynamic_pointer_cast<CRefSketch>(ref)) {
         element->SetAttribute("TargetFeatureID",
                               sketch->targetFeatureID.c_str());
       }
     },
     [](XMLElement *element) {
       auto sketch = std::make_shared<CRefSketch>();
       if (const char *tid = element->Attribute("TargetFeatureID"))
         sketch->targetFeatureID = tid;
       return sketch;
     }},
    {RefType::TOPO_FACE, "Face", "face",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       if (auto face = std::dynamic_pointer_cast<CRefFace>(ref)) {
         element->SetAttribute("ParentFeatureID",
                               face->parentFeatureID.c_str());
         element->SetAttribute("TopologyIndex", face->topologyIndex);
         element->SetAttribute("U", FormatVector(face->uDir).c_str());
         element->SetAttribute("V", FormatVector(face->vDir).c_str());
         element->SetAttribute("Normal", FormatVector(face->normal).c_str());
         element->SetAttribute("Center", FormatPoint(face->centroid).c_str());
       }
     },
     [](XMLElement *element) {
       auto face = std::make_shared<CRefFace>();
       if (const char *parent = element->Attribute("ParentFeatureID"))
         face->parentFeatureID = parent;
       element->QueryIntAttribute("TopologyIndex", &face->topologyIndex);
       face->uDir = ParseVectorAttribute(element, "U");
       face->vDir = ParseVectorAttribute(element, "V");
       face->normal = ParseVectorAttribute(element, "Normal");
       face->centroid = ParsePointAttribute(element, "Center");
       return face;
     }},
    {RefType::TOPO_EDGE, "Edge", "edge",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       if (auto edge = std::dynamic_pointer_cast<CRefEdge>(ref)) {
         element->SetAttribute("ParentFeatureID",
                               edge->parentFeatureID.c_str());
         element->SetAttribute("TopologyIndex", edge->topologyIndex);
         element->SetAttribute("MidPoint", FormatPoint(edge->midPoint).c_str());
       }
     },
     [](XMLElement *element) {
       auto edge = std::make_shared<CRefEdge>();
       if (const char *parent = element->Attribute("ParentFeatureID"))
         edge->parentFeatureID = parent;
       element->QueryIntAttribute("TopologyIndex", &edge->topologyIndex);
       edge->midPoint = ParsePointAttribute(element, "MidPoint");
       return edge;
     }},
    {RefType::TOPO_VERTEX, "Vertex", "vertex",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       if (auto vertex = std::dynamic_pointer_cast<CRefVertex>(ref)) {
         element->SetAttribute("ParentFeatureID",
                               vertex->parentFeatureID.c_str());
         element->SetAttribute("TopologyIndex", vertex->topologyIndex);
         element->SetAttribute("Position", FormatPoint(vertex->pos).c_str());
       }
     },
     [](XMLElement *element) {
       auto vertex = std::make_shared<CRefVertex>();
       if (const char *parent = element->Attribute("ParentFeatureID"))
         vertex->parentFeatureID = parent;
       element->QueryIntAttribute("TopologyIndex", &vertex->topologyIndex);
       vertex->pos = ParsePointAttribute(element, "Position");
       return vertex;
     }},
    {RefType::TOPO_SKETCH_SEG, "SketchSeg", "sketchseg",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       if (auto seg = std::dynamic_pointer_cast<CRefSketchSeg>(ref)) {
         element->SetAttribute("ParentFeatureID", seg->parentFeatureID.c_str());
         element->SetAttribute("TopologyIndex", seg->topologyIndex);
         if (!seg->segmentLocalID.empty())
           element->SetAttribute("SegmentLocalID", seg->segmentLocalID.c_str());
       }
     },
     [](XMLElement *element) {
       auto seg = std::make_shared<CRefSketchSeg>();
       if (const char *parent = element->Attribute("ParentFeatureID"))
         seg->parentFeatureID = parent;
       element->QueryIntAttribute("TopologyIndex", &seg->topologyIndex);
       if (const char *lid = element->Attribute("SegmentLocalID"))
         seg->segmentLocalID = lid;
       return seg;
     }}};

const RefSerializerEntry *FindRefEntry(RefType type) {
  for (const auto &entry : kRefSerializerEntries) {
    if (entry.type == type)
      return &entry;
  }
  return nullptr;
}

const RefSerializerEntry *FindRefEntryByName(const std::string &value) {
  for (const auto &entry : kRefSerializerEntries) {
    if (value == entry.lowerName)
      return &entry;
  }
  return nullptr;
}

std::string RefTypeToString(RefType type) {
  if (auto entry = FindRefEntry(type))
    return entry->name;
  return "Unknown";
}

std::optional<RefType> RefTypeFromString(const char *text) {
  if (!text)
    return std::nullopt;
  std::string value = ToLower(text);
  if (auto entry = FindRefEntryByName(value))
    return entry->type;
  return std::nullopt;
}

void TinyXMLSerializer::SaveRefEntity(
    XMLDocument &doc, XMLElement *parent, const std::string &name,
    const std::shared_ptr<CRefEntityBase> &ref) {
  if (!ref)
    return;

  XMLElement *refElem = doc.NewElement(name.c_str());
  parent->InsertEndChild(refElem);

  if (auto entry = FindRefEntry(ref->refType)) {
    entry->save(refElem, ref);
    refElem->SetAttribute("Type", entry->name);
  } else {
    SaveFeatureReference(refElem, ref);
    refElem->SetAttribute("Type", RefTypeToString(ref->refType).c_str());
  }
}

void TinyXMLSerializer::SaveFeature(
    XMLDocument &doc, XMLElement *parent,
    const std::shared_ptr<CFeatureBase> &feature) {
  if (!feature)
    return;

  XMLElement *featElem = doc.NewElement("Feature");
  parent->InsertEndChild(featElem);

  if (auto sketch = std::dynamic_pointer_cast<CSketch>(feature)) {
    featElem->SetAttribute("Type", "Sketch");
    SaveSketch(doc, featElem, sketch);
  } else if (auto extrude = std::dynamic_pointer_cast<CExtrude>(feature)) {
    featElem->SetAttribute("Type", "Extrude");
    SaveExtrude(doc, featElem, extrude);
  } else if (auto revolve = std::dynamic_pointer_cast<CRevolve>(feature)) {
    featElem->SetAttribute("Type", "Revolve");
    SaveRevolve(doc, featElem, revolve);
  } else {
    featElem->SetAttribute("Type", "Unknown");
  }

  featElem->SetAttribute("ID", feature->featureID.c_str());
  featElem->SetAttribute("Name", feature->featureName.c_str());
  featElem->SetAttribute("Suppressed", feature->isSuppressed);
}

void TinyXMLSerializer::SaveSketch(XMLDocument &doc, XMLElement *element,
                                   const std::shared_ptr<CSketch> &sketch) {
  SaveRefEntity(doc, element, "ReferencePlane", sketch->referencePlane);

  XMLElement *segsElem = doc.NewElement("Segments");
  element->InsertEndChild(segsElem);
  for (const auto &seg : sketch->segments) {
    SaveSketchSeg(doc, segsElem, seg);
  }

  XMLElement *consElem = doc.NewElement("Constraints");
  element->InsertEndChild(consElem);
  for (const auto &con : sketch->constraints) {
    SaveConstraint(doc, consElem, con);
  }
}

void TinyXMLSerializer::SaveSketchSeg(XMLDocument &doc, XMLElement *parent,
                                      const std::shared_ptr<CSketchSeg> &seg) {
  if (!seg)
    return;
  XMLElement *segElem = doc.NewElement("Segment");
  parent->InsertEndChild(segElem);

  segElem->SetAttribute("LocalID", seg->localID.c_str());
  if (seg->type != CSketchSeg::SegType::POINT) {
    segElem->SetAttribute("Construction", seg->isConstruction);
  }

  if (auto line = std::dynamic_pointer_cast<CSketchLine>(seg)) {
    segElem->SetAttribute("Type", "Line");
    SavePoint3D(segElem, "Start", line->startPos);
    SavePoint3D(segElem, "End", line->endPos);
  } else if (auto circle = std::dynamic_pointer_cast<CSketchCircle>(seg)) {
    segElem->SetAttribute("Type", "Circle");
    SavePoint3D(segElem, "Center", circle->center);
    segElem->SetAttribute("Radius", circle->radius);
  } else if (auto arc = std::dynamic_pointer_cast<CSketchArc>(seg)) {
    segElem->SetAttribute("Type", "Arc");
    SavePoint3D(segElem, "Center", arc->center);
    segElem->SetAttribute("Radius", arc->radius);
    segElem->SetAttribute("StartAngle", arc->startAngle);
    segElem->SetAttribute("EndAngle", arc->endAngle);
    segElem->SetAttribute("Clockwise", arc->isClockwise);
  } else if (auto pt = std::dynamic_pointer_cast<CSketchPoint>(seg)) {
    segElem->SetAttribute("Type", "Point");
    SavePoint3D(segElem, "Position", pt->position);
  }
}

void TinyXMLSerializer::SaveConstraint(XMLDocument &doc, XMLElement *parent,
                                       const CSketchConstraint &constraint) {
  XMLElement *conElem = doc.NewElement("Constraint");
  parent->InsertEndChild(conElem);
  conElem->SetAttribute("Type", static_cast<int>(constraint.type));
  conElem->SetAttribute("Dimension", constraint.dimensionValue);

  std::string ids;
  for (size_t i = 0; i < constraint.entityLocalIDs.size(); ++i) {
    ids += constraint.entityLocalIDs[i];
    if (i < constraint.entityLocalIDs.size() - 1)
      ids += ",";
  }
  conElem->SetAttribute("Entities", ids.c_str());
}

void TinyXMLSerializer::SaveExtrude(XMLDocument &doc, XMLElement *element,
                                    const std::shared_ptr<CExtrude> &extrude) {
  if (extrude->sketchProfile) {
    XMLElement *profileElem = doc.NewElement("ProfileSketchID");
    profileElem->SetAttribute("Value",
                              extrude->sketchProfile->featureID.c_str());
    element->InsertEndChild(profileElem);
  }

  XMLElement *directionElem = doc.NewElement("Direction");
  SaveVector3D(directionElem, "Value", extrude->direction);
  element->InsertEndChild(directionElem);

  element->SetAttribute("Operation",
                        BooleanOpToString(extrude->operation).c_str());

  // EndCondition1
  XMLElement *ec1 = doc.NewElement("EndCondition1");
  element->InsertEndChild(ec1);
  ec1->SetAttribute(
      "Type",
      ExtrudeEndConditionTypeToString(extrude->endCondition1.type).c_str());
  ec1->SetAttribute("Depth", extrude->endCondition1.depth);
  ec1->SetAttribute("Offset", extrude->endCondition1.offset);
  ec1->SetAttribute("HasOffset", extrude->endCondition1.hasOffset);
  ec1->SetAttribute("Flip", extrude->endCondition1.isFlip);
  ec1->SetAttribute("FlipMaterialSide",
                    extrude->endCondition1.isFlipMaterialSide);
  SaveRefEntity(doc, ec1, "ReferenceEntity",
                extrude->endCondition1.referenceEntity);

  // EndCondition2
  if (extrude->endCondition2) {
    XMLElement *ec2 = doc.NewElement("EndCondition2");
    element->InsertEndChild(ec2);
    ec2->SetAttribute(
        "Type",
        ExtrudeEndConditionTypeToString(extrude->endCondition2->type).c_str());
    ec2->SetAttribute("Depth", extrude->endCondition2->depth);
    ec2->SetAttribute("HasOffset", extrude->endCondition2->hasOffset);
    ec2->SetAttribute("Offset", extrude->endCondition2->offset);

    // ... other fields
  }
}

void TinyXMLSerializer::SaveRevolve(XMLDocument &doc, XMLElement *element,
                                    const std::shared_ptr<CRevolve> &revolve) {
  element->SetAttribute("ProfileSketchID", revolve->profileSketchID.c_str());
  element->SetAttribute("AngleKind", static_cast<int>(revolve->angleKind));
  element->SetAttribute("PrimaryAngle", revolve->primaryAngle);
  element->SetAttribute("SecondaryAngle", revolve->secondaryAngle);

  XMLElement *axisElem = doc.NewElement("Axis");
  element->InsertEndChild(axisElem);
  axisElem->SetAttribute("Kind", static_cast<int>(revolve->axis.kind));
  axisElem->SetAttribute("RefLocalID", revolve->axis.referenceLocalID.c_str());
  SavePoint3D(axisElem, "Origin", revolve->axis.origin);
  SaveVector3D(axisElem, "Direction", revolve->axis.direction);
  SaveRefEntity(doc, axisElem, "ReferenceEntity",
                revolve->axis.referenceEntity);
}

// =================================================================================================
// Load Implementation
// =================================================================================================

bool TinyXMLSerializer::Load(UnifiedModel &model,
                             const std::filesystem::path &filePath,
                             std::string *errorMessage) {
  XMLDocument doc;
  XMLError result = doc.LoadFile(filePath.string().c_str());
  if (result != XML_SUCCESS) {
    if (errorMessage)
      *errorMessage = doc.ErrorStr();
    return false;
  }

  XMLElement *root = doc.FirstChildElement("UnifiedModel");
  if (!root) {
    if (errorMessage)
      *errorMessage = "Missing UnifiedModel root element";
    return false;
  }

  const char *unitText = root->Attribute("UnitSystem");
  if (auto unitOpt = UnitTypeFromString(unitText)) {
    model.unit = *unitOpt;
  }

  const char *name = root->Attribute("ModelName");
  if (name)
    model.modelName = name;

  model.Clear();

  XMLElement *featElem = root->FirstChildElement("Feature");
  while (featElem) {
    auto feature = LoadFeature(featElem);
    if (feature) {
      model.AddFeature(feature);
    }
    featElem = featElem->NextSiblingElement("Feature");
  }

  return true;
}

CPoint3D TinyXMLSerializer::LoadPoint3D(XMLElement *element, const char *name) {
  CPoint3D pt;
  double x, y, z;
  if (TryParseTriple(element->Attribute(name), x, y, z)) {
    pt.x = x;
    pt.y = y;
    pt.z = z;
  }
  return pt;
}

CVector3D TinyXMLSerializer::LoadVector3D(XMLElement *element,
                                          const char *name) {
  CVector3D vec;
  double x, y, z;
  if (TryParseTriple(element->Attribute(name), x, y, z)) {
    vec.x = x;
    vec.y = y;
    vec.z = z;
  }
  return vec;
}

std::shared_ptr<CRefEntityBase>
TinyXMLSerializer::LoadRefEntity(XMLElement *element) {
  if (!element)
    return nullptr;

  const char *typeStr = element->Attribute("Type");
  if (!typeStr)
    return nullptr;
  std::string normalizedType = ToLower(typeStr);
  std::shared_ptr<CRefEntityBase> ref;
  std::optional<RefType> resolvedType = RefTypeFromString(typeStr);

  if (resolvedType) {
    if (auto entry = FindRefEntry(*resolvedType)) {
      ref = entry->load(element);
    } else if (*resolvedType == RefType::FEATURE_DATUM_AXIS ||
               *resolvedType == RefType::FEATURE_DATUM_POINT) {
      ref = LoadFeatureReference(element, *resolvedType);
    }
  }

  if (!ref && normalizedType == "feature")
    ref = LoadFeatureReference(element, RefType::FEATURE_DATUM_PLANE);

  if (ref && resolvedType)
    ref->refType = *resolvedType;
  return ref;
}

std::shared_ptr<CFeatureBase>
TinyXMLSerializer::LoadFeature(XMLElement *element) {
  const char *typeStr = element->Attribute("Type");
  if (!typeStr)
    return nullptr;
  std::string type = typeStr;

  std::shared_ptr<CFeatureBase> feature;

  if (type == "Sketch") {
    auto sketch = std::make_shared<CSketch>();
    LoadSketch(element, sketch);
    feature = sketch;
  } else if (type == "Extrude") {
    auto extrude = std::make_shared<CExtrude>();
    LoadExtrude(element, extrude);
    feature = extrude;
  } else if (type == "Revolve") {
    auto revolve = std::make_shared<CRevolve>();
    LoadRevolve(element, revolve);
    feature = revolve;
  }

  if (feature) {
    const char *id = element->Attribute("ID");
    if (id)
      feature->featureID = id;
    const char *name = element->Attribute("Name");
    if (name)
      feature->featureName = name;
    bool suppressed = false;
    element->QueryBoolAttribute("Suppressed", &suppressed);
    feature->isSuppressed = suppressed;
  }
  return feature;
}

void TinyXMLSerializer::LoadSketch(XMLElement *element,
                                   std::shared_ptr<CSketch> &sketch) {
  sketch->referencePlane =
      LoadRefEntity(element->FirstChildElement("ReferencePlane"));

  XMLElement *segsElem = element->FirstChildElement("Segments");
  if (segsElem) {
    XMLElement *segElem = segsElem->FirstChildElement("Segment");
    while (segElem) {
      auto seg = LoadSketchSeg(segElem);
      if (seg)
        sketch->segments.push_back(seg);
      segElem = segElem->NextSiblingElement("Segment");
    }
  }

  XMLElement *consElem = element->FirstChildElement("Constraints");
  if (consElem) {
    XMLElement *conElem = consElem->FirstChildElement("Constraint");
    while (conElem) {
      sketch->constraints.push_back(LoadConstraint(conElem));
      conElem = conElem->NextSiblingElement("Constraint");
    }
  }
}

std::shared_ptr<CSketchSeg>
TinyXMLSerializer::LoadSketchSeg(XMLElement *element) {
  const char *typeStr = element->Attribute("Type");
  if (!typeStr)
    return nullptr;
  std::string type = typeStr;

  std::shared_ptr<CSketchSeg> seg;
  if (type == "Line") {
    auto line = std::make_shared<CSketchLine>();
    line->startPos = LoadPoint3D(element, "Start");
    line->endPos = LoadPoint3D(element, "End");
    seg = line;
  } else if (type == "Circle") {
    auto circle = std::make_shared<CSketchCircle>();
    circle->center = LoadPoint3D(element, "Center");
    element->QueryDoubleAttribute("Radius", &circle->radius);
    seg = circle;
  }
  // ... others

  if (seg) {
    const char *lid = element->Attribute("LocalID");
    if (lid)
      seg->localID = lid;
    bool cons = false;
    element->QueryBoolAttribute("Construction", &cons);
    seg->isConstruction = cons;
  }
  return seg;
}

CSketchConstraint TinyXMLSerializer::LoadConstraint(XMLElement *element) {
  CSketchConstraint con;
  int typeVal = 0;
  element->QueryIntAttribute("Type", &typeVal);
  con.type = static_cast<CSketchConstraint::ConstraintType>(typeVal);
  element->QueryDoubleAttribute("Dimension", &con.dimensionValue);

  const char *ents = element->Attribute("Entities");
  if (ents) {
    std::stringstream ss(ents);
    std::string item;
    while (std::getline(ss, item, ',')) {
      con.entityLocalIDs.push_back(item);
    }
  }
  return con;
}

void TinyXMLSerializer::LoadExtrude(XMLElement *element,
                                    std::shared_ptr<CExtrude> &extrude) {
  XMLElement *profileElem = element->FirstChildElement("ProfileSketchID");
  // Note: In a real scenario, we might need to resolve this ID to a pointer,
  // but CExtrude stores a shared_ptr.
  // The current LoadModel logic in Cereal deserializes the pointer directly if
  // it was serialized as such. Here we only have the ID. We might need a second
  // pass to link pointers or store the ID temporarily. For this simplified
  // implementation, we'll assume the caller or a post-process step handles
  // linking, OR we just create a dummy sketch with that ID if we can't find it.
  // However, CExtrude definition has std::shared_ptr<CSketch> sketchProfile.
  // We can't easily fill this without a lookup map.
  // For now, let's create a placeholder sketch with the ID so we don't lose the
  // info.
  if (profileElem) {
    const char *value = profileElem->Attribute("Value");
    if (value) {
      auto placeholder = std::make_shared<CSketch>();
      placeholder->featureID = value;
      extrude->sketchProfile = placeholder;
    }
  }

  XMLElement *directionElem = element->FirstChildElement("Direction");
  if (directionElem) {
    extrude->direction = LoadVector3D(directionElem, "Value");
  }

  if (auto opOpt = BooleanOpFromString(element->Attribute("Operation"))) {
    extrude->operation = *opOpt;
  }

  XMLElement *ec1 = element->FirstChildElement("EndCondition1");
  if (ec1) {
    if (auto typeOpt =
            ExtrudeEndConditionTypeFromString(ec1->Attribute("Type"))) {
      extrude->endCondition1.type = *typeOpt;
    }
    ec1->QueryDoubleAttribute("Depth", &extrude->endCondition1.depth);
    // ...
  }
  XMLElement *ec2 = element->FirstChildElement("EndCondition2");
  if (ec2) {
    ExtrudeEndCondition cond2;
    if (auto typeOpt =
            ExtrudeEndConditionTypeFromString(ec2->Attribute("Type"))) {
      cond2.type = *typeOpt;
    }
    ec2->QueryDoubleAttribute("Depth", &cond2.depth);
    // ...
    extrude->endCondition2 = cond2;
  }
}

void TinyXMLSerializer::LoadRevolve(XMLElement *element,
                                    std::shared_ptr<CRevolve> &revolve) {
  // Implementation similar to Extrude
}

} // namespace CADExchange
