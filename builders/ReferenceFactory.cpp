#include "ReferenceFactory.h"

#include <optional>

namespace CADExchange {
namespace Builder {

namespace {

constexpr const char *kAttrParentFeatureID = "parentFeatureID";
constexpr const char *kAttrTopologyIndex = "topologyIndex";
constexpr const char *kAttrNormal = "normal";
constexpr const char *kAttrCentroid = "centroid";
constexpr const char *kAttrUDir = "uDir";
constexpr const char *kAttrVDir = "vDir";
constexpr const char *kAttrMidPoint = "midPoint";
constexpr const char *kAttrPos = "pos";
constexpr const char *kAttrSegmentID = "segmentLocalID";
constexpr const char *kAttrTargetFeatureID = "targetFeatureID";
constexpr const char *kAttrOrigin = "origin";
constexpr const char *kAttrXDir = "xDir";
constexpr const char *kAttrYDir = "yDir";

std::optional<std::string> ToString(const ReferenceAttributeMap &attrs,
                                    const char *key) {
  auto it = attrs.find(key);
  if (it == attrs.end())
    return std::nullopt;
  if (auto val = it->second.GetAs<std::string>(); val) {
    return *val;
  }
  return std::nullopt;
}

std::optional<int> ToInt(const ReferenceAttributeMap &attrs, const char *key) {
  auto it = attrs.find(key);
  if (it == attrs.end())
    return std::nullopt;
  if (auto val = it->second.GetAs<int>(); val) {
    return *val;
  }
  return std::nullopt;
}

std::optional<double> ToDouble(const ReferenceAttributeMap &attrs,
                               const char *key) {
  auto it = attrs.find(key);
  if (it == attrs.end())
    return std::nullopt;
  if (auto val = it->second.GetAs<double>(); val) {
    return *val;
  }
  return std::nullopt;
}

std::optional<Point3D> ToPoint(const ReferenceAttributeMap &attrs,
                               const char *key) {
  auto it = attrs.find(key);
  if (it == attrs.end())
    return std::nullopt;
  if (auto val = it->second.GetAs<Point3D>(); val) {
    return *val;
  }
  return std::nullopt;
}

std::optional<Vector3D> ToVector(const ReferenceAttributeMap &attrs,
                                 const char *key) {
  auto it = attrs.find(key);
  if (it == attrs.end())
    return std::nullopt;
  if (auto val = it->second.GetAs<Vector3D>(); val) {
    return *val;
  }
  return std::nullopt;
}

CPoint3D ToCPoint(const Point3D &value) { return {value.x, value.y, value.z}; }

CVector3D ToCVector(const Vector3D &value) {
  return {value.x, value.y, value.z};
}

std::shared_ptr<CRefFace>
CreateFaceReference(const ReferenceAttributeMap &attrs) {
  auto face = std::make_shared<CRefFace>();
  if (auto parent = ToString(attrs, kAttrParentFeatureID))
    face->parentFeatureID = *parent;
  if (auto index = ToInt(attrs, kAttrTopologyIndex))
    face->topologyIndex = *index;
  if (auto centroid = ToPoint(attrs, kAttrCentroid))
    face->centroid = ToCPoint(*centroid);
  if (auto normal = ToVector(attrs, kAttrNormal))
    face->normal = ToCVector(*normal);
  if (auto uDir = ToVector(attrs, kAttrUDir))
    face->uDir = ToCVector(*uDir);
  if (auto vDir = ToVector(attrs, kAttrVDir))
    face->vDir = ToCVector(*vDir);

  return face;
}

std::shared_ptr<CRefEdge>
CreateEdgeReference(const ReferenceAttributeMap &attrs) {
  auto edge = std::make_shared<CRefEdge>();
  if (auto parent = ToString(attrs, kAttrParentFeatureID))
    edge->parentFeatureID = *parent;
  if (auto index = ToInt(attrs, kAttrTopologyIndex))
    edge->topologyIndex = *index;
  if (auto mid = ToPoint(attrs, kAttrMidPoint))
    edge->midPoint = ToCPoint(*mid);
  return edge;
}

std::shared_ptr<CRefVertex>
CreateVertexReference(const ReferenceAttributeMap &attrs) {
  auto vertex = std::make_shared<CRefVertex>();
  if (auto parent = ToString(attrs, kAttrParentFeatureID))
    vertex->parentFeatureID = *parent;
  if (auto index = ToInt(attrs, kAttrTopologyIndex))
    vertex->topologyIndex = *index;
  if (auto pos = ToPoint(attrs, kAttrPos))
    vertex->pos = ToCPoint(*pos);
  return vertex;
}

std::shared_ptr<CRefSketchSeg>
CreateSketchSegmentReference(const ReferenceAttributeMap &attrs) {
  auto segment = std::make_shared<CRefSketchSeg>();
  if (auto parent = ToString(attrs, kAttrParentFeatureID))
    segment->parentFeatureID = *parent;
  if (auto index = ToInt(attrs, kAttrTopologyIndex))
    segment->topologyIndex = *index;
  if (auto id = ToString(attrs, kAttrSegmentID))
    segment->segmentLocalID = *id;
  return segment;
}

std::shared_ptr<CRefSketch>
CreateSketchReference(const ReferenceAttributeMap &attrs) {
  auto sketch = std::make_shared<CRefSketch>();
  if (auto tid = ToString(attrs, kAttrTargetFeatureID))
    sketch->targetFeatureID = *tid;
  return sketch;
}

std::shared_ptr<CRefPlane>
CreatePlaneReference(const ReferenceAttributeMap &attrs) {
  auto plane = std::make_shared<CRefPlane>();
  if (auto tid = ToString(attrs, kAttrTargetFeatureID))
    plane->targetFeatureID = *tid;
  if (auto origin = ToPoint(attrs, kAttrOrigin))
    plane->origin = ToCPoint(*origin);
  if (auto xdir = ToVector(attrs, kAttrXDir))
    plane->xDir = ToCVector(*xdir);
  if (auto normal = ToVector(attrs, kAttrNormal))
    plane->normal = ToCVector(*normal);
  if (auto ydir = ToVector(attrs, kAttrYDir)) {
    plane->yDir = ToCVector(*ydir);
  } else {
    plane->yDir = CVector3D::Cross(plane->normal, plane->xDir);
    plane->yDir.Normalize();
  }
  return plane;
}

std::shared_ptr<CRefEntityBase>
CreateFeatureReference(RefType refType, const ReferenceAttributeMap &attrs) {
  auto ref = std::make_shared<CRefFeature>(refType);
  if (auto tid = ToString(attrs, kAttrTargetFeatureID))
    ref->targetFeatureID = *tid;
  return ref;
}

} // namespace

std::shared_ptr<CRefEntityBase>
ReferenceFactory::Create(ReferenceKind kind,
                         const ReferenceAttributeMap &attributes,
                         std::string *outError) {
  switch (kind) {
  case ReferenceKind::Face:
    return CreateFaceReference(attributes);
  case ReferenceKind::Edge:
    return CreateEdgeReference(attributes);
  case ReferenceKind::Vertex:
    return CreateVertexReference(attributes);
  case ReferenceKind::SketchSegment:
    return CreateSketchSegmentReference(attributes);
  case ReferenceKind::Sketch:
    return CreateSketchReference(attributes);
  case ReferenceKind::DatumPlane:
    return CreatePlaneReference(attributes);
  case ReferenceKind::DatumAxis:
    return CreateFeatureReference(RefType::FEATURE_DATUM_AXIS, attributes);
  case ReferenceKind::DatumPoint:
    return CreateFeatureReference(RefType::FEATURE_DATUM_POINT, attributes);
  }

  if (outError) {
    *outError = "Unsupported reference kind";
  }
  return nullptr;
}

} // namespace Builder
} // namespace CADExchange
