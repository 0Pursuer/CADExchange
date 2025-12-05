#pragma once
#include "../core/UnifiedFeatures.h"
#include "../core/UnifiedTypes.h"

namespace CADExchange {
namespace Builder {

// Helper Builder for Face References
class RefFaceBuilder {
  std::shared_ptr<CRefFace> m_ptr;

public:
  RefFaceBuilder(const std::string &parentID, int index = 0) {
    m_ptr = std::make_shared<CRefFace>();
    m_ptr->parentFeatureID = parentID;
    m_ptr->topologyIndex = index;
  }
  RefFaceBuilder &Centroid(double x, double y, double z) {
    m_ptr->centroid = {x, y, z};
    return *this;
  }
  RefFaceBuilder &Normal(double x, double y, double z) {
    m_ptr->normal = {x, y, z};
    return *this;
  }
  RefFaceBuilder &UDir(double x, double y, double z) {
    m_ptr->uDir = {x, y, z};
    return *this;
  }
  RefFaceBuilder &VDir(double x, double y, double z) {
    m_ptr->vDir = {x, y, z};
    return *this;
  }
  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; }
  operator std::shared_ptr<CRefFace>() const { return m_ptr; }
};

// Helper Builder for Vertex References
class RefVertexBuilder {
  std::shared_ptr<CRefVertex> m_ptr;

public:
  RefVertexBuilder(const std::string &parentID, int index = 0) {
    m_ptr = std::make_shared<CRefVertex>();
    m_ptr->parentFeatureID = parentID;
    m_ptr->topologyIndex = index;
  }
  RefVertexBuilder &Pos(double x, double y, double z) {
    m_ptr->pos = {x, y, z};
    return *this;
  }
  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; }
  operator std::shared_ptr<CRefVertex>() const { return m_ptr; }
};

// Helper Builder for Edge References
class RefEdgeBuilder {
  std::shared_ptr<CRefEdge> m_ptr;

public:
  RefEdgeBuilder(const std::string &parentID, int index = 0) {
    m_ptr = std::make_shared<CRefEdge>();
    m_ptr->parentFeatureID = parentID;
    m_ptr->topologyIndex = index;
  }
  RefEdgeBuilder &MidPoint(double x, double y, double z) {
    m_ptr->midPoint = {x, y, z};
    return *this;
  }
  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; }
  operator std::shared_ptr<CRefEdge>() const { return m_ptr; }
};

// Static Facade
class Ref {
public:
  static RefFaceBuilder Face(const std::string &parentID, int index = 0) {
    return RefFaceBuilder(parentID, index);
  }
  static RefVertexBuilder Vertex(const std::string &parentID, int index = 0) {
    return RefVertexBuilder(parentID, index);
  }
  static RefEdgeBuilder Edge(const std::string &parentID, int index = 0) {
    return RefEdgeBuilder(parentID, index);
  }

  static std::shared_ptr<CRefFeature> Plane(const std::string &planeID) {
    auto p = std::make_shared<CRefFeature>(RefType::FEATURE_DATUM_PLANE);
    p->targetFeatureID = planeID;
    return p;
  }

  // Standard Planes
  static std::shared_ptr<CRefFeature> XY() {
    return Plane(StandardID::PLANE_XY);
  }
  static std::shared_ptr<CRefFeature> YZ() {
    return Plane(StandardID::PLANE_YZ);
  }
  static std::shared_ptr<CRefFeature> ZX() {
    return Plane(StandardID::PLANE_ZX);
  }
};

} // namespace Builder
} // namespace CADExchange
