#pragma once

#include "../../core/UnifiedFeatures.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <utility>
#include <vector>
#include <map>
#include <string>
#include <cmath>

#include "../../thirdParty/cereal/archives/json.hpp"
#include "../../thirdParty/cereal/types/vector.hpp"
#include "../../thirdParty/cereal/types/map.hpp"
#include "../../thirdParty/cereal/types/string.hpp"

namespace CADExchange {

// ---------------------------------------------------------
// Non-intrusive cereal serialization for geometric primitives
// ---------------------------------------------------------
template <class Archive>
void serialize(Archive &ar, CPoint3D &pt) {
  ar(cereal::make_nvp("x", pt.x),
     cereal::make_nvp("y", pt.y),
     cereal::make_nvp("z", pt.z));
}

template <class Archive>
void serialize(Archive &ar, CVector3D &vec) {
  ar(cereal::make_nvp("x", vec.x),
     cereal::make_nvp("y", vec.y),
     cereal::make_nvp("z", vec.z));
}

template <class Archive>
void serialize(Archive &ar, CSketchCSys &csys) {
  ar(cereal::make_nvp("origin", csys.origin),
     cereal::make_nvp("xDir", csys.xDir),
     cereal::make_nvp("yDir", csys.yDir),
     cereal::make_nvp("zDir", csys.zDir));
}

template <class Archive>
void serialize(Archive &ar, CGeoDatumPlane &plane) {
  ar(cereal::make_nvp("targetFeatureID", plane.targetFeatureID),
     cereal::make_nvp("type", plane.type),
     cereal::make_nvp("localCSys", plane.localCSys));
}

template <class Archive>
void serialize(Archive &ar, CRefEdge &edge) {
  int curveTypeVal = static_cast<int>(edge.curveType);
  ar(cereal::make_nvp("parentFeatureID", edge.parentFeatureID),
     cereal::make_nvp("topologyIndex", edge.topologyIndex),
     cereal::make_nvp("curveType", curveTypeVal),
     cereal::make_nvp("startPoint", edge.startPoint),
     cereal::make_nvp("endPoint", edge.endPoint),
     cereal::make_nvp("midPoint", edge.midPoint));
  edge.curveType = static_cast<CGeoCurveType>(curveTypeVal);
}

namespace Geometry {

/**
 * @brief CRTP 基类：统一管理几何边与辅助基准面容器
 */
template <typename Derived, typename EdgeT = CRefEdge>
class GeometryCollectorBase {
public:
  using EdgeType = EdgeT;
  using DatumPlaneType = CGeoDatumPlane;

  template <typename... Args> auto Collect(Args &&...args) {
    Clear();
    return DerivedSelf().CollectImpl(std::forward<Args>(args)...);
  }

  const std::vector<EdgeType> &GetEdges() const noexcept { return m_edges; }
  const std::vector<DatumPlaneType> &GetDatumPlanes() const noexcept {
    return m_datumPlanes;
  }
  std::size_t EdgeCount() const noexcept { return m_edges.size(); }
  std::size_t DatumPlaneCount() const noexcept { return m_datumPlanes.size(); }
  void Clear() noexcept {
    m_edges.clear();
    m_datumPlanes.clear();
  }

  // Serialization for the collector itself
  template <class Archive>
  void serialize(Archive &ar) {
    ar(cereal::make_nvp("edges", m_edges),
       cereal::make_nvp("datumPlanes", m_datumPlanes));
  }

  // Equivalency check for runtime validation
  bool IsEquivalent(const GeometryCollectorBase& other, double tol = 2e-3) const {
    if (m_edges.size() != other.m_edges.size()) {
      return false;
    }
    if (m_datumPlanes.size() != other.m_datumPlanes.size()) return false;

    // Verify all edges in 'this' exist in 'other'
    for (size_t i = 0; i < m_edges.size(); ++i) {
      const auto& myEdge = m_edges[i];
      bool found = false;
      for (const auto& otherEdge : other.m_edges) {
        if (myEdge.curveType == otherEdge.curveType) {
          double dx = myEdge.midPoint.x - otherEdge.midPoint.x;
          double dy = myEdge.midPoint.y - otherEdge.midPoint.y;
          double dz = myEdge.midPoint.z - otherEdge.midPoint.z;
          double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
          if (dist <= tol) {
            found = true;
            break;
          }
        }
      }
      if (!found) {
        std::cout << "[DEBUG] IsEquivalent FAIL: myEdge type=" << (int)myEdge.curveType 
                  << " mid=(" << std::fixed << std::setprecision(8) << myEdge.midPoint.x << "," << myEdge.midPoint.y << "," << myEdge.midPoint.z << ") NOT FOUND" << std::endl;
        
        std::cout << "[DEBUG] Reference edges (count=" << other.m_edges.size() << "):" << std::endl;
        for (const auto& oe : other.m_edges) {
           std::cout << "  - type=" << (int)oe.curveType << " mid=(" << oe.midPoint.x << "," << oe.midPoint.y << "," << oe.midPoint.z << ")" << std::endl;
        }

        double minD = 1e9;
        int bestType = -1;
        for (const auto& oe : other.m_edges) {
          double d = std::sqrt(std::pow(myEdge.midPoint.x - oe.midPoint.x, 2) + 
                               std::pow(myEdge.midPoint.y - oe.midPoint.y, 2) + 
                               std::pow(myEdge.midPoint.z - oe.midPoint.z, 2));
          if (d < minD) {
              minD = d;
              bestType = (int)oe.curveType;
          }
        }
        std::cout << "[DEBUG] Best match (any type): dist=" << minD << " type=" << bestType << " (tol=" << tol << ")" << std::endl;
        return false;
      }
    }
    return true;
  }

protected:
  void AddEdge(const EdgeType &edge) { m_edges.push_back(edge); }
  void AddEdge(EdgeType &&edge) { m_edges.emplace_back(std::move(edge)); }
  void AddDatumPlane(const DatumPlaneType &plane) { m_datumPlanes.push_back(plane); }
  void AddDatumPlane(DatumPlaneType &&plane) { m_datumPlanes.emplace_back(std::move(plane)); }

  Derived &DerivedSelf() noexcept { return static_cast<Derived &>(*this); }

private:
  std::vector<EdgeType> m_edges;
  std::vector<DatumPlaneType> m_datumPlanes;
};

/**
 * @brief 包含多个特征级 GeometryCollector 的模型级几何集合。
 * 提供方便的基于 cereal 的序列化和反序列化 JSON。
 */
template <typename CollectorT>
class ModelGeometrySet {
public:
  std::map<std::string, CollectorT> features;

  template <class Archive>
  void serialize(Archive &ar) {
    ar(cereal::make_nvp("features", features));
  }

  bool SaveToJson(const std::filesystem::path &filePath,
                  std::string *errorMessage = nullptr) const {
    std::ofstream out(filePath, std::ios::trunc);
    if (!out.is_open()) {
      if (errorMessage) {
        *errorMessage = "Unable to open geometry json output: " + filePath.string();
      }
      return false;
    }
    try {
      cereal::JSONOutputArchive archive(out);
      archive(cereal::make_nvp("ModelGeometry", *this));
      return true;
    } catch (const std::exception& e) {
      if (errorMessage) {
        *errorMessage = "Failed to write geometry json: " + std::string(e.what());
      }
      return false;
    }
  }

  bool LoadFromJson(const std::filesystem::path &filePath,
                    std::string *errorMessage = nullptr) {
    std::ifstream in(filePath);
    if (!in.is_open()) {
      if (errorMessage) {
        *errorMessage = "Unable to open geometry json input: " + filePath.string();
      }
      return false;
    }
    try {
      cereal::JSONInputArchive archive(in);
      archive(cereal::make_nvp("ModelGeometry", *this));
      return true;
    } catch (const std::exception& e) {
      if (errorMessage) {
        *errorMessage = "Failed to parse geometry json: " + std::string(e.what());
      }
      return false;
    }
  }
  
  std::size_t TotalEdgeCount() const {
    std::size_t total = 0;
    for (const auto& pair : features) {
      total += pair.second.EdgeCount();
    }
    return total;
  }
  
  std::size_t TotalDatumPlaneCount() const {
    std::size_t total = 0;
    for (const auto& pair : features) {
      total += pair.second.DatumPlaneCount();
    }
    return total;
  }
};

} // namespace Geometry
} // namespace CADExchange
