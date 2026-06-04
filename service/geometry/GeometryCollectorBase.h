#pragma once

#include "../../core/UnifiedFeatures.h"
#include "../../core/UnifiedModel.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../../thirdParty/json/single_include/nlohmann/json.hpp"

namespace CADExchange {

namespace Geometry {

namespace detail {

using json = nlohmann::json;

inline json PointToJson(const CPoint3D &pt) {
  return json{{"x", pt.x}, {"y", pt.y}, {"z", pt.z}};
}

inline json VectorToJson(const CVector3D &vec) {
  return json{{"x", vec.x}, {"y", vec.y}, {"z", vec.z}};
}

inline bool TryReadPoint(const json &node, CPoint3D &pt) {
  if (!node.is_object() || !node.contains("x") || !node.contains("y") ||
      !node.contains("z")) {
    return false;
  }
  pt.x = node.at("x").get<double>();
  pt.y = node.at("y").get<double>();
  pt.z = node.at("z").get<double>();
  return true;
}

inline bool TryReadVector(const json &node, CVector3D &vec) {
  if (!node.is_object() || !node.contains("x") || !node.contains("y") ||
      !node.contains("z")) {
    return false;
  }
  vec.x = node.at("x").get<double>();
  vec.y = node.at("y").get<double>();
  vec.z = node.at("z").get<double>();
  return true;
}

inline std::string CurveTypeToString(CGeoCurveType type) {
  switch (type) {
  case CGeoCurveType::LINE:
    return "Line";
  case CGeoCurveType::CIRCLE:
    return "Circle";
  case CGeoCurveType::ELLIPSE:
    return "Ellipse";
  case CGeoCurveType::INTERSECTION:
    return "Intersection";
  case CGeoCurveType::BCURVE:
    return "BCurve";
  case CGeoCurveType::SPCURVE:
    return "SPCurve";
  case CGeoCurveType::CONSTPARAM:
    return "ConstParam";
  case CGeoCurveType::TRIMMED:
    return "Trimmed";
  case CGeoCurveType::UNKNOWN:
  default:
    return "Unknown";
  }
}

inline CGeoCurveType CurveTypeFromString(const std::string &typeName) {
  if (typeName == "Line") return CGeoCurveType::LINE;
  if (typeName == "Circle") return CGeoCurveType::CIRCLE;
  if (typeName == "Ellipse") return CGeoCurveType::ELLIPSE;
  if (typeName == "Intersection") return CGeoCurveType::INTERSECTION;
  if (typeName == "BCurve") return CGeoCurveType::BCURVE;
  if (typeName == "SPCurve") return CGeoCurveType::SPCURVE;
  if (typeName == "ConstParam") return CGeoCurveType::CONSTPARAM;
  if (typeName == "Trimmed") return CGeoCurveType::TRIMMED;
  return CGeoCurveType::UNKNOWN;
}

} // namespace detail

template <typename Derived, typename EdgeT = CRefEdge>
class GeometryCollectorBase {
public:
  using EdgeType = EdgeT;
  using DatumPlaneType = CGeoDatumPlane;

  struct ComparisonResult {
    bool equivalent = true;
    std::vector<std::string> diagnostics;
  };

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

  void Scale(double factor) noexcept {
    for (auto &edge : m_edges) {
      edge.startPoint.x *= factor;
      edge.startPoint.y *= factor;
      edge.startPoint.z *= factor;
      edge.endPoint.x *= factor;
      edge.endPoint.y *= factor;
      edge.endPoint.z *= factor;
      edge.midPoint.x *= factor;
      edge.midPoint.y *= factor;
      edge.midPoint.z *= factor;
    }
  }

  bool SaveEdgesToJson(const std::filesystem::path &filePath,
                       std::string *errorMessage = nullptr,
                       const std::string &lengthUnit = "") const {
    try {
      detail::json root;
      root["schema_version"] = 1;
      if (!lengthUnit.empty()) {
        root["length_unit"] = lengthUnit;
      }
      root["edge_count"] = m_edges.size();
      root["datum_plane_count"] = m_datumPlanes.size();
      root["edges"] = detail::json::array();
      for (const auto &edge : m_edges) {
        root["edges"].push_back(detail::json{{"parentFeatureID", edge.parentFeatureID},
                                              {"topologyIndex", edge.topologyIndex},
                                              {"curveType", detail::CurveTypeToString(edge.curveType)},
                                              {"curveTypeValue", static_cast<int>(edge.curveType)},
                                              {"startPoint", detail::PointToJson(edge.startPoint)},
                                              {"endPoint", detail::PointToJson(edge.endPoint)},
                                              {"midPoint", detail::PointToJson(edge.midPoint)}});
      }
      root["datum_planes"] = detail::json::array();
      for (const auto &plane : m_datumPlanes) {
        root["datum_planes"].push_back(detail::json{{"targetFeatureID", plane.targetFeatureID},
                                                     {"type", plane.type},
                                                     {"origin", detail::PointToJson(plane.localCSys.origin)},
                                                     {"xDir", detail::VectorToJson(plane.localCSys.xDir)},
                                                     {"yDir", detail::VectorToJson(plane.localCSys.yDir)},
                                                     {"normal", detail::VectorToJson(plane.localCSys.zDir)}});
      }
      std::ofstream out(filePath, std::ios::trunc);
      if (!out.is_open()) {
        if (errorMessage) {
          *errorMessage = "Unable to open geometry json output: " + filePath.string();
        }
        return false;
      }
      out << root.dump(2) << '\n';
      return true;
    } catch (const std::exception &e) {
      if (errorMessage) {
        *errorMessage = "Failed to write geometry json: " + std::string(e.what());
      }
      return false;
    }
  }

  detail::json ToJsonValue() const {
    detail::json geometry;
    geometry["edges"] = detail::json::array();
    for (const auto &edge : m_edges) {
      geometry["edges"].push_back(detail::json{{"parentFeatureID", edge.parentFeatureID},
                                                {"topologyIndex", edge.topologyIndex},
                                                {"curveType", static_cast<int>(edge.curveType)},
                                                {"startPoint", detail::PointToJson(edge.startPoint)},
                                                {"endPoint", detail::PointToJson(edge.endPoint)},
                                                {"midPoint", detail::PointToJson(edge.midPoint)}});
    }
    geometry["datumPlanes"] = detail::json::array();
    for (const auto &plane : m_datumPlanes) {
      geometry["datumPlanes"].push_back(detail::json{{"targetFeatureID", plane.targetFeatureID},
                                                      {"type", plane.type},
                                                      {"localCSys", detail::json{{"origin", detail::PointToJson(plane.localCSys.origin)},
                                                                                  {"xDir", detail::VectorToJson(plane.localCSys.xDir)},
                                                                                  {"yDir", detail::VectorToJson(plane.localCSys.yDir)},
                                                                                  {"zDir", detail::VectorToJson(plane.localCSys.zDir)}}}});
    }
    return geometry;
  }

  bool LoadFromJsonValue(const detail::json &geometry,
                         std::string *errorMessage = nullptr) {
    try {
      const auto edgesIt = geometry.find("edges");
      if (edgesIt == geometry.end() || !edgesIt->is_array()) {
        if (errorMessage) {
          *errorMessage = "geometry json missing edges array";
        }
        return false;
      }

      m_edges.clear();
      m_edges.reserve(edgesIt->size());
      for (const auto &edgeJson : *edgesIt) {
        EdgeType edge;
        edge.parentFeatureID = edgeJson.value("parentFeatureID", "");
        edge.topologyIndex = edgeJson.value("topologyIndex", -1);
        if (edgeJson.contains("curveTypeValue")) {
          edge.curveType = static_cast<CGeoCurveType>(edgeJson.at("curveTypeValue").get<int>());
        } else if (edgeJson.contains("curveType")) {
          const auto &curveTypeNode = edgeJson.at("curveType");
          if (curveTypeNode.is_number_integer()) {
            edge.curveType = static_cast<CGeoCurveType>(curveTypeNode.get<int>());
          } else if (curveTypeNode.is_string()) {
            edge.curveType = detail::CurveTypeFromString(curveTypeNode.get<std::string>());
          }
        }
        if (!edgeJson.contains("startPoint") || !edgeJson.contains("endPoint") ||
            !edgeJson.contains("midPoint") ||
            !detail::TryReadPoint(edgeJson.at("startPoint"), edge.startPoint) ||
            !detail::TryReadPoint(edgeJson.at("endPoint"), edge.endPoint) ||
            !detail::TryReadPoint(edgeJson.at("midPoint"), edge.midPoint)) {
          if (errorMessage) {
            *errorMessage = "geometry json contains invalid edge point payload";
          }
          return false;
        }
        m_edges.push_back(std::move(edge));
      }

      const auto planesIt = geometry.find("datumPlanes");
      const auto flatPlanesIt = geometry.find("datum_planes");
      const detail::json *planesJson = nullptr;
      if (planesIt != geometry.end() && planesIt->is_array()) {
        planesJson = &(*planesIt);
      } else if (flatPlanesIt != geometry.end() && flatPlanesIt->is_array()) {
        planesJson = &(*flatPlanesIt);
      }

      m_datumPlanes.clear();
      if (planesJson != nullptr) {
        m_datumPlanes.reserve(planesJson->size());
        for (const auto &planeJson : *planesJson) {
          DatumPlaneType plane;
          plane.targetFeatureID = planeJson.value("targetFeatureID", "");
          plane.type = planeJson.value("type", "Plane");
          const auto localCSysIt = planeJson.find("localCSys");
          if (localCSysIt != planeJson.end() && localCSysIt->is_object()) {
            if (!localCSysIt->contains("origin") || !localCSysIt->contains("xDir") ||
                !localCSysIt->contains("yDir") || !localCSysIt->contains("zDir") ||
                !detail::TryReadPoint(localCSysIt->at("origin"), plane.localCSys.origin) ||
                !detail::TryReadVector(localCSysIt->at("xDir"), plane.localCSys.xDir) ||
                !detail::TryReadVector(localCSysIt->at("yDir"), plane.localCSys.yDir) ||
                !detail::TryReadVector(localCSysIt->at("zDir"), plane.localCSys.zDir)) {
              if (errorMessage) {
                *errorMessage = "geometry json contains invalid datum plane csys payload";
              }
              return false;
            }
          } else {
            if (!planeJson.contains("origin") || !planeJson.contains("xDir") ||
                !planeJson.contains("yDir") || !planeJson.contains("normal") ||
                !detail::TryReadPoint(planeJson.at("origin"), plane.localCSys.origin) ||
                !detail::TryReadVector(planeJson.at("xDir"), plane.localCSys.xDir) ||
                !detail::TryReadVector(planeJson.at("yDir"), plane.localCSys.yDir) ||
                !detail::TryReadVector(planeJson.at("normal"), plane.localCSys.zDir)) {
              if (errorMessage) {
                *errorMessage = "geometry json contains invalid flat datum plane payload";
              }
              return false;
            }
          }
          m_datumPlanes.push_back(std::move(plane));
        }
      }
      return true;
    } catch (const std::exception &e) {
      if (errorMessage) {
        *errorMessage = "Failed to parse geometry json: " + std::string(e.what());
      }
      return false;
    }
  }

  struct HalfStructurePointGroup {
    CPoint3D center{};
    double radius = 0;
    std::vector<CPoint3D> points;
  };

  static std::vector<HalfStructurePointGroup> ExtractHalfStructureGroups(
      const std::vector<EdgeType>& edges, double tol) {
    std::vector<EdgeType> open;
    std::vector<NormalizedArc> arcs;
    std::vector<CircleType> circles;
    int warn = 0;
    ClassifyEdges(edges, open, arcs, circles, warn, tol);
    std::vector<CircleType> promoted;
    std::vector<HalfStructurePointGroup> groups;
    MergeArcs(arcs, tol, promoted, &groups);
    return groups;
  }

  ComparisonResult CompareDetailed(const GeometryCollectorBase& other,
                                   double tol = 2e-3,
                                   const std::vector<HalfStructurePointGroup>* global_src_half_groups = nullptr,
                                   const std::vector<HalfStructurePointGroup>* global_dst_half_groups = nullptr) const {
    ComparisonResult result;
    if (m_datumPlanes.size() != other.m_datumPlanes.size()) {
      result.equivalent = false;
      result.diagnostics.push_back("DATUM plane count mismatch: SRC=" + std::to_string(m_datumPlanes.size()) +
                                   " DST=" + std::to_string(other.m_datumPlanes.size()));
      return result;
    }

    std::vector<EdgeType> src_open, dst_open;
    std::vector<NormalizedArc> src_arcs, dst_arcs;
    std::vector<CircleType> src_circles, dst_circles;
    std::vector<HalfStructurePointGroup> src_half_structure_groups, dst_half_structure_groups;
    int src_warn = 0, dst_warn = 0;
    ClassifyEdges(m_edges, src_open, src_arcs, src_circles, src_warn, tol);
    ClassifyEdges(other.m_edges, dst_open, dst_arcs, dst_circles, dst_warn, tol);

    std::vector<CircleType> promoted_src, promoted_dst;
    src_arcs = MergeArcs(src_arcs, tol, promoted_src, &src_half_structure_groups);
    dst_arcs = MergeArcs(dst_arcs, tol, promoted_dst, &dst_half_structure_groups);
    for (auto &p : promoted_src) src_circles.push_back(p);
    for (auto &p : promoted_dst) dst_circles.push_back(p);

    // Simplify circles and arcs on both sides
    SimplifyCirclesAndArcs(src_circles, src_arcs, tol);
    SimplifyCirclesAndArcs(dst_circles, dst_arcs, tol);

    const auto* src_groups = global_src_half_groups ? global_src_half_groups : &src_half_structure_groups;
    const auto* dst_groups = global_dst_half_groups ? global_dst_half_groups : &dst_half_structure_groups;
    FilterHalfStructureEdges(src_open, *src_groups, tol);
    FilterHalfStructureEdges(dst_open, *dst_groups, tol);


    std::vector<CircleType> src_unmatched_circles;
    std::vector<CircleType> dst_unmatched_circles;
    std::vector<bool> dst_circle_used(dst_circles.size(), false);
    for (const auto& sc : src_circles) {
      bool found = false;
      for (size_t j = 0; j < dst_circles.size(); ++j) {
        if (dst_circle_used[j]) continue;
        if (PtDist(sc.center, dst_circles[j].center) <= tol && 
            std::abs(sc.radius - dst_circles[j].radius) <= tol) {
          dst_circle_used[j] = true;
          found = true;
          break;
        }
      }
      if (!found) {
        src_unmatched_circles.push_back(sc);
      }
    }
    for (size_t j = 0; j < dst_circles.size(); ++j) {
      if (!dst_circle_used[j]) {
        dst_unmatched_circles.push_back(dst_circles[j]);
      }
    }

    std::vector<NormalizedArc> src_unmatched_arcs;
    std::vector<NormalizedArc> dst_unmatched_arcs;
    std::vector<CPoint3D> matched_vertices;
    std::vector<bool> dst_arc_used(dst_arcs.size(), false);
    for (const auto& sa : src_arcs) {
      bool found = false;
      for (size_t j = 0; j < dst_arcs.size(); ++j) {
        if (dst_arc_used[j]) continue;
        const auto& da = dst_arcs[j];
        if (PtDist(sa.center, da.center) <= tol && std::abs(sa.radius - da.radius) <= tol) {
          double fwd = (std::max)(PtDist(sa.startPt, da.startPt), PtDist(sa.endPt, da.endPt));
          double rev = (std::max)(PtDist(sa.startPt, da.endPt), PtDist(sa.endPt, da.startPt));
          if ((std::min)(fwd, rev) <= tol) {
            dst_arc_used[j] = true;
            found = true;
            matched_vertices.push_back(sa.startPt);
            matched_vertices.push_back(sa.endPt);
            break;
          }
        }
      }
      if (!found) {
        src_unmatched_arcs.push_back(sa);
      }
    }
    for (size_t j = 0; j < dst_arcs.size(); ++j) {
      if (!dst_arc_used[j]) {
        dst_unmatched_arcs.push_back(dst_arcs[j]);
      }
    }

    std::vector<EdgeType> src_unmatched_open;
    std::vector<EdgeType> dst_unmatched_open;
    std::vector<bool> dst_open_used(dst_open.size(), false);
    for (const auto& se : src_open) {
      bool found = false;
      for (size_t j = 0; j < dst_open.size(); ++j) {
        if (dst_open_used[j]) continue;
        const auto& de = dst_open[j];
        if (se.curveType == de.curveType && PtDist(se.midPoint, de.midPoint) <= tol) {
          double fwd = (std::max)(PtDist(se.startPoint, de.startPoint), PtDist(se.endPoint, de.endPoint));
          double rev = (std::max)(PtDist(se.startPoint, de.endPoint), PtDist(se.endPoint, de.startPoint));
          if ((std::min)(fwd, rev) <= tol) {
            dst_open_used[j] = true;
            found = true;
            matched_vertices.push_back(se.startPoint);
            matched_vertices.push_back(se.endPoint);
            break;
          }
        }
      }
      if (!found) {
        src_unmatched_open.push_back(se);
      }
    }
    for (size_t j = 0; j < dst_open.size(); ++j) {
      if (!dst_open_used[j]) {
        dst_unmatched_open.push_back(dst_open[j]);
      }
    }

    auto is_vertex_matched = [&](const CPoint3D& pt) -> bool {
      for (const auto& mv : matched_vertices) {
        if (PtDist(pt, mv) <= tol) return true;
      }
      return false;
    };
    auto is_redundant_division_open = [&](const EdgeType& edge) -> bool {
      return is_vertex_matched(edge.startPoint) && is_vertex_matched(edge.endPoint);
    };
    auto is_redundant_division_arc = [&](const NormalizedArc& arc) -> bool {
      return is_vertex_matched(arc.startPt) && is_vertex_matched(arc.endPt);
    };

    src_unmatched_open.erase(
        std::remove_if(src_unmatched_open.begin(), src_unmatched_open.end(),
                       is_redundant_division_open),
        src_unmatched_open.end());
    dst_unmatched_open.erase(
        std::remove_if(dst_unmatched_open.begin(), dst_unmatched_open.end(),
                       is_redundant_division_open),
        dst_unmatched_open.end());

    src_unmatched_arcs.erase(
        std::remove_if(src_unmatched_arcs.begin(), src_unmatched_arcs.end(),
                       is_redundant_division_arc),
        src_unmatched_arcs.end());
    dst_unmatched_arcs.erase(
        std::remove_if(dst_unmatched_arcs.begin(), dst_unmatched_arcs.end(),
                       is_redundant_division_arc),
        dst_unmatched_arcs.end());

    result.equivalent = true;
    for (const auto& sc : src_unmatched_circles) {
      if (IsWarnOnlyEdge(sc.curveType)) continue;
      result.equivalent = false;
      result.diagnostics.push_back("SRC unmatched TRUE_CIRCLE " + FormatCircle(sc.center, sc.radius));
    }
    for (const auto& dc : dst_unmatched_circles) {
      if (IsWarnOnlyEdge(dc.curveType)) continue;
      result.equivalent = false;
      result.diagnostics.push_back("DST extra TRUE_CIRCLE " + FormatCircle(dc.center, dc.radius));
    }
    for (const auto& sa : src_unmatched_arcs) {
      if (IsWarnOnlyEdge(sa.curveType)) continue;
      result.equivalent = false;
      result.diagnostics.push_back("SRC unmatched ARC " + FormatArc(sa));
    }
    for (const auto& da : dst_unmatched_arcs) {
      if (IsWarnOnlyEdge(da.curveType)) continue;
      result.equivalent = false;
      result.diagnostics.push_back("DST extra ARC " + FormatArc(da));
    }
    for (const auto& se : src_unmatched_open) {
      if (IsWarnOnlyEdge(se.curveType)) continue;
      result.equivalent = false;
      result.diagnostics.push_back("SRC unmatched OPEN_EDGE " + FormatOpenEdge(se));
    }
    for (const auto& de : dst_unmatched_open) {
      if (IsWarnOnlyEdge(de.curveType)) continue;
      result.equivalent = false;
      result.diagnostics.push_back("DST extra OPEN_EDGE " + FormatOpenEdge(de));
    }

    if (src_warn != dst_warn) {
      result.diagnostics.push_back("WARN-ONLY edge count mismatch: SRC=" + std::to_string(src_warn) +
                                   " DST=" + std::to_string(dst_warn));
    }
    return result;
  }

  bool IsEquivalent(const GeometryCollectorBase& other, double tol = 2e-3) const {
    ComparisonResult result = CompareDetailed(other, tol);
    for (const auto &line : result.diagnostics) {
      std::cout << "[DEBUG] IsEquivalent: " << line << "\n";
    }
    return result.equivalent;
  }

protected:
  void AddEdge(const EdgeType &edge) { m_edges.push_back(edge); }
  void AddEdge(EdgeType &&edge) { m_edges.emplace_back(std::move(edge)); }
  void AddDatumPlane(const DatumPlaneType &plane) { m_datumPlanes.push_back(plane); }
  void AddDatumPlane(DatumPlaneType &&plane) { m_datumPlanes.emplace_back(std::move(plane)); }

  Derived &DerivedSelf() noexcept { return static_cast<Derived &>(*this); }

private:
  struct NormalizedArc {
    CPoint3D center{};
    double radius = 0;
    CPoint3D startPt{};
    CPoint3D endPt{};
    CGeoCurveType curveType = CGeoCurveType::UNKNOWN;
  };

  struct CircleType {
    CPoint3D center{};
    double radius = 0;
    CGeoCurveType curveType = CGeoCurveType::UNKNOWN;
  };



  static std::string FormatPoint(const CPoint3D &pt) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "(" << pt.x << "," << pt.y << "," << pt.z << ")";
    return oss.str();
  }

  static std::string FormatCircle(const CPoint3D &center, double radius) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "center=" << FormatPoint(center) << " r=" << radius;
    return oss.str();
  }

  static std::string FormatArc(const NormalizedArc &arc) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "center=" << FormatPoint(arc.center)
        << " r=" << arc.radius
        << " start=" << FormatPoint(arc.startPt)
        << " end=" << FormatPoint(arc.endPt);
    return oss.str();
  }

  static std::string FormatOpenEdge(const EdgeType &edge) {
    std::ostringstream oss;
    oss << "type=" << static_cast<int>(edge.curveType)
        << " start=" << FormatPoint(edge.startPoint)
        << " mid=" << FormatPoint(edge.midPoint)
        << " end=" << FormatPoint(edge.endPoint);
    return oss.str();
  }

  static bool PointsNear(const CPoint3D& a, const CPoint3D& b,
                         double tol) noexcept {
    return PtDist(a, b) <= tol;
  }

  static bool PointInGroup(const CPoint3D& pt,
                           const HalfStructurePointGroup& group,
                           double tol) noexcept {
    for (const auto& candidate : group.points) {
      if (PointsNear(pt, candidate, tol)) {
        return true;
      }
    }
    return false;
  }

  static bool IsHalfStructureRedundantEdge(
      const EdgeType& edge,
      const std::vector<HalfStructurePointGroup>& groups,
      double tol) noexcept {
    for (const auto& group : groups) {
      if (PointInGroup(edge.startPoint, group, tol) ||
          PointInGroup(edge.endPoint, group, tol)) {
        return true;
      }
    }
    return false;
  }

  static void FilterHalfStructureEdges(
      std::vector<EdgeType>& open_edges,
      const std::vector<HalfStructurePointGroup>& groups,
      double tol) {
    if (groups.empty() || open_edges.empty()) {
      return;
    }

    open_edges.erase(
        std::remove_if(open_edges.begin(), open_edges.end(),
                       [&](const EdgeType& edge) {
                         return IsHalfStructureRedundantEdge(edge, groups, tol);
                       }),
        open_edges.end());
  }


  static double PtDist(const CPoint3D& a, const CPoint3D& b) noexcept {
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  static bool IsOpenEdge(CGeoCurveType t) noexcept {
    return t == CGeoCurveType::LINE || t == CGeoCurveType::ELLIPSE ||
           t == CGeoCurveType::BCURVE || t == CGeoCurveType::TRIMMED;
  }

  static bool IsWarnOnlyEdge(CGeoCurveType t) noexcept {
    return t == CGeoCurveType::INTERSECTION || t == CGeoCurveType::SPCURVE ||
           t == CGeoCurveType::CONSTPARAM;
  }

  static bool ComputeCircumcenter(const CPoint3D& p1, const CPoint3D& p2,
                                  const CPoint3D& p3, CPoint3D& center,
                                  double& radius) noexcept {
    double ax = p2.x-p1.x, ay = p2.y-p1.y, az = p2.z-p1.z;
    double bx = p3.x-p1.x, by = p3.y-p1.y, bz = p3.z-p1.z;
    double uu = ax*ax + ay*ay + az*az;
    double uv = ax*bx + ay*by + az*bz;
    double vv = bx*bx + by*by + bz*bz;
    double det = uu*vv - uv*uv;
    if (std::abs(det) < 1e-20) return false;
    double inv2 = 0.5 / det;
    double s = vv*(uu - uv)*inv2;
    double t = uu*(vv - uv)*inv2;
    center.x = p1.x + s*ax + t*bx;
    center.y = p1.y + s*ay + t*by;
    center.z = p1.z + s*az + t*bz;
    double rx = center.x-p1.x, ry = center.y-p1.y, rz = center.z-p1.z;
    radius = std::sqrt(rx*rx + ry*ry + rz*rz);
    return true;
  }

  static void ClassifyEdges(const std::vector<EdgeType>& edges,
                            std::vector<EdgeType>& open_out,
                            std::vector<NormalizedArc>& arc_out,
                            std::vector<CircleType>& circle_out,
                            int& warn_count,
                            double tol) {
    for (const auto& e : edges) {
      if (e.curveType == CGeoCurveType::UNKNOWN) continue;
      if (IsWarnOnlyEdge(e.curveType)) { ++warn_count; }

      // LINE is strictly a straight line segment.
      if (e.curveType == CGeoCurveType::LINE) {
        open_out.push_back(e);
        continue;
      }

      double se_dist = PtDist(e.startPoint, e.endPoint);
      double sm_dist = PtDist(e.startPoint, e.midPoint);
      if (se_dist <= tol) {
        if (sm_dist <= tol) continue;
        CPoint3D cen;
        cen.x = (e.startPoint.x + e.midPoint.x) * 0.5;
        cen.y = (e.startPoint.y + e.midPoint.y) * 0.5;
        cen.z = (e.startPoint.z + e.midPoint.z) * 0.5;
        circle_out.push_back({cen, sm_dist * 0.5, e.curveType});
      } else {
        NormalizedArc arc;
        if (!ComputeCircumcenter(e.startPoint, e.midPoint, e.endPoint, arc.center, arc.radius)) {
          // If circumcenter fit fails (collinear), it's a straight segment, treat as open edge
          open_out.push_back(e);
          continue;
        }
        arc.startPt = e.startPoint;
        arc.endPt = e.endPoint;
        arc.curveType = e.curveType;
        arc_out.push_back(arc);
      }
    }
  }

  static std::vector<NormalizedArc> MergeArcs(const std::vector<NormalizedArc>& arcs,
                                              double tol,
                                              std::vector<CircleType>& promoted_circles,
                                              std::vector<HalfStructurePointGroup>* half_structure_groups = nullptr) {
    std::vector<NormalizedArc> current_arcs = arcs;
    bool changed = true;
    while (changed) {
      changed = false;
      std::vector<bool> used(current_arcs.size(), false);
      std::vector<NormalizedArc> next_arcs;
      for (size_t i = 0; i < current_arcs.size(); ++i) {
        if (used[i]) continue;
        bool found_partner = false;
        for (size_t j = i + 1; j < current_arcs.size(); ++j) {
          if (used[j]) continue;
          if (PtDist(current_arcs[i].center, current_arcs[j].center) > tol) continue;
          if (std::abs(current_arcs[i].radius - current_arcs[j].radius) > tol) continue;
          bool loop_fwd = PtDist(current_arcs[i].endPt, current_arcs[j].startPt) <= tol &&
                          PtDist(current_arcs[j].endPt, current_arcs[i].startPt) <= tol;
          bool loop_rev = PtDist(current_arcs[i].endPt, current_arcs[j].endPt) <= tol &&
                          PtDist(current_arcs[i].startPt, current_arcs[j].startPt) <= tol;
          if (loop_fwd || loop_rev) {
            CGeoCurveType promotedType = current_arcs[i].curveType;
            if (current_arcs[i].curveType != current_arcs[j].curveType) {
              if (IsWarnOnlyEdge(current_arcs[i].curveType) || IsWarnOnlyEdge(current_arcs[j].curveType)) {
                promotedType = CGeoCurveType::INTERSECTION;
              }
            }
            promoted_circles.push_back({current_arcs[i].center, current_arcs[i].radius, promotedType});
            if (half_structure_groups) {
              HalfStructurePointGroup group;
              group.center = current_arcs[i].center;
              group.radius = current_arcs[i].radius;
              group.points.push_back(current_arcs[i].startPt);
              group.points.push_back(current_arcs[i].endPt);
              group.points.push_back(current_arcs[j].startPt);
              group.points.push_back(current_arcs[j].endPt);
              half_structure_groups->push_back(std::move(group));
            }
            used[i] = used[j] = true;
            found_partner = true;
            changed = true;
            break;
          }
          if (PtDist(current_arcs[i].endPt, current_arcs[j].startPt) <= tol) {
            CGeoCurveType mergedType = current_arcs[i].curveType;
            if (IsWarnOnlyEdge(current_arcs[i].curveType) || IsWarnOnlyEdge(current_arcs[j].curveType)) {
              mergedType = CGeoCurveType::INTERSECTION;
            }
            NormalizedArc merged{current_arcs[i].center, current_arcs[i].radius, current_arcs[i].startPt, current_arcs[j].endPt, mergedType};
            if (half_structure_groups) {
              HalfStructurePointGroup group;
              group.center = current_arcs[i].center;
              group.radius = current_arcs[i].radius;
              group.points.push_back(current_arcs[i].endPt);
              group.points.push_back(current_arcs[j].startPt);
              half_structure_groups->push_back(std::move(group));
            }
            next_arcs.push_back(merged);
            used[i] = used[j] = true;
            found_partner = true;
            changed = true;
            break;
          }
          if (PtDist(current_arcs[j].endPt, current_arcs[i].startPt) <= tol) {
            CGeoCurveType mergedType = current_arcs[i].curveType;
            if (IsWarnOnlyEdge(current_arcs[i].curveType) || IsWarnOnlyEdge(current_arcs[j].curveType)) {
              mergedType = CGeoCurveType::INTERSECTION;
            }
            NormalizedArc merged{current_arcs[i].center, current_arcs[i].radius, current_arcs[j].startPt, current_arcs[i].endPt, mergedType};
            if (half_structure_groups) {
              HalfStructurePointGroup group;
              group.center = current_arcs[i].center;
              group.radius = current_arcs[i].radius;
              group.points.push_back(current_arcs[j].endPt);
              group.points.push_back(current_arcs[i].startPt);
              half_structure_groups->push_back(std::move(group));
            }
            next_arcs.push_back(merged);
            used[i] = used[j] = true;
            found_partner = true;
            changed = true;
            break;
          }
        }
        if (!found_partner) {
          next_arcs.push_back(current_arcs[i]);
        }
      }
      current_arcs = std::move(next_arcs);
    }
    return current_arcs;
  }

  static void SimplifyCirclesAndArcs(std::vector<CircleType>& circles,
                                     std::vector<NormalizedArc>& arcs,
                                     double tol) {
    for (auto circle_it = circles.begin(); circle_it != circles.end(); ) {
      bool circle_simplified = false;
      for (auto arc_it = arcs.begin(); arc_it != arcs.end(); ++arc_it) {
        if (PtDist(circle_it->center, arc_it->center) <= tol &&
            std::abs(circle_it->radius - arc_it->radius) <= tol) {
          NormalizedArc comp_arc;
          comp_arc.center = arc_it->center;
          comp_arc.radius = arc_it->radius;
          comp_arc.startPt = arc_it->endPt;
          comp_arc.endPt = arc_it->startPt;
          comp_arc.curveType = arc_it->curveType;
          
          *arc_it = comp_arc;
          circle_it = circles.erase(circle_it);
          circle_simplified = true;
          break;
        }
      }
      if (!circle_simplified) {
        ++circle_it;
      }
    }
  }

  static bool MatchCircles(const std::vector<CircleType>& src,
                           const std::vector<CircleType>& dst,
                           double tol,
                           std::vector<std::string>* diagnostics) {
    bool ok = true;
    std::vector<bool> used(dst.size(), false);
    for (const auto& sc : src) {
      bool found = false;
      for (size_t j = 0; j < dst.size(); ++j) {
        if (used[j]) continue;
        if (PtDist(sc.center, dst[j].center) <= tol && std::abs(sc.radius - dst[j].radius) <= tol) {
          used[j] = true;
          found = true;
          break;
        }
      }
      if (!found) {
        ok = false;
        if (diagnostics) diagnostics->push_back("SRC unmatched TRUE_CIRCLE " + FormatCircle(sc.center, sc.radius));
      }
    }
    for (size_t j = 0; j < dst.size(); ++j) {
      if (!used[j]) {
        ok = false;
        if (diagnostics) diagnostics->push_back("DST extra TRUE_CIRCLE " + FormatCircle(dst[j].center, dst[j].radius));
      }
    }
    return ok;
  }

  static bool MatchArcs(const std::vector<NormalizedArc>& src,
                        const std::vector<NormalizedArc>& dst,
                        double tol,
                        std::vector<std::string>* diagnostics) {
    bool ok = true;
    std::vector<bool> used(dst.size(), false);
    for (const auto& sa : src) {
      bool found = false;
      for (size_t j = 0; j < dst.size(); ++j) {
        if (used[j]) continue;
        const auto& da = dst[j];
        if (PtDist(sa.center, da.center) > tol) continue;
        if (std::abs(sa.radius - da.radius) > tol) continue;
        double fwd = (std::max)(PtDist(sa.startPt, da.startPt), PtDist(sa.endPt, da.endPt));
        double rev = (std::max)(PtDist(sa.startPt, da.endPt), PtDist(sa.endPt, da.startPt));
        if ((std::min)(fwd, rev) <= tol) {
          used[j] = true;
          found = true;
          break;
        }
      }
      if (!found) {
        ok = false;
        if (diagnostics) diagnostics->push_back("SRC unmatched ARC " + FormatArc(sa));
      }
    }
    for (size_t j = 0; j < dst.size(); ++j) {
      if (!used[j]) {
        ok = false;
        if (diagnostics) diagnostics->push_back("DST extra ARC " + FormatArc(dst[j]));
      }
    }
    return ok;
  }

  static bool MatchOpenEdges(const std::vector<EdgeType>& src,
                             const std::vector<EdgeType>& dst,
                             double tol,
                             std::vector<std::string>* diagnostics) {
    bool ok = true;
    std::vector<bool> used(dst.size(), false);
    for (const auto& se : src) {
      bool found = false;
      for (size_t j = 0; j < dst.size(); ++j) {
        if (used[j]) continue;
        const auto& de = dst[j];
        if (se.curveType != de.curveType) continue;
        if (PtDist(se.midPoint, de.midPoint) > tol) continue;
        double fwd = (std::max)(PtDist(se.startPoint, de.startPoint), PtDist(se.endPoint, de.endPoint));
        double rev = (std::max)(PtDist(se.startPoint, de.endPoint), PtDist(se.endPoint, de.startPoint));
        if ((std::min)(fwd, rev) <= tol) {
          used[j] = true;
          found = true;
          break;
        }
      }
      if (!found) {
        ok = false;
        if (diagnostics) diagnostics->push_back("SRC unmatched OPEN_EDGE " + FormatOpenEdge(se));
      }
    }
    for (size_t j = 0; j < dst.size(); ++j) {
      if (!used[j]) {
        ok = false;
        if (diagnostics) diagnostics->push_back("DST extra OPEN_EDGE " + FormatOpenEdge(dst[j]));
      }
    }
    return ok;
  }

  std::vector<EdgeType> m_edges;
  std::vector<DatumPlaneType> m_datumPlanes;
};

template <typename CollectorT>
class ModelGeometrySet {
public:
  std::map<std::string, CollectorT> features;
  std::string length_unit;

  bool SaveToJson(const std::filesystem::path &filePath,
                  std::string *errorMessage = nullptr) const {
    try {
      detail::json featuresJson = detail::json::array();
      for (const auto &[featureId, collector] : features) {
        featuresJson.push_back(detail::json{{"key", featureId}, {"value", collector.ToJsonValue()}});
      }
      detail::json root{{"ModelGeometry", detail::json{{"features", std::move(featuresJson)}}}};
      if (!length_unit.empty()) {
        root["length_unit"] = length_unit;
      }
      std::ofstream out(filePath, std::ios::trunc);
      if (!out.is_open()) {
        if (errorMessage) *errorMessage = "Unable to open geometry json output: " + filePath.string();
        return false;
      }
      out << root.dump(2) << '\n';
      return true;
    } catch (const std::exception &e) {
      if (errorMessage) *errorMessage = "Failed to write geometry json: " + std::string(e.what());
      return false;
    }
  }

  // Load geometry from JSON.  If `target_unit` is non-empty and differs from
  // the unit recorded in the file's `length_unit` field, all edge coordinates
  // are automatically scaled so that the loaded data is expressed in
  // `target_unit`.  Pass an empty string (the default) to skip conversion.
  // After a successful load, `this->length_unit` is set to the effective unit
  // (either the file's unit or `target_unit` if conversion was applied).
  bool LoadFromJson(const std::filesystem::path &filePath,
                    std::string *errorMessage = nullptr,
                    const std::string &target_unit = "") {
    try {
      std::ifstream in(filePath);
      if (!in.is_open()) {
        if (errorMessage) *errorMessage = "Unable to open geometry json input: " + filePath.string();
        return false;
      }
      detail::json root = detail::json::parse(in);

      // Parse length_unit if present
      std::string file_unit;
      const auto unitIt = root.find("length_unit");
      if (unitIt != root.end() && unitIt->is_string()) {
        file_unit = unitIt->get<std::string>();
      }

      const auto modelIt = root.find("ModelGeometry");
      if (modelIt == root.end() || !modelIt->is_object()) {
        if (errorMessage) *errorMessage = "geometry json missing ModelGeometry object";
        return false;
      }
      const auto featuresIt = modelIt->find("features");
      if (featuresIt == modelIt->end() || !featuresIt->is_array()) {
        if (errorMessage) *errorMessage = "geometry json missing features array";
        return false;
      }
      features.clear();
      for (const auto &entry : *featuresIt) {
        if (!entry.is_object() || !entry.contains("key") || !entry.contains("value")) {
          if (errorMessage) *errorMessage = "geometry json contains malformed feature entry";
          return false;
        }
        CollectorT collector;
        std::string featureError;
        const std::string featureId = entry.at("key").get<std::string>();
        if (!collector.LoadFromJsonValue(entry.at("value"), &featureError)) {
          if (errorMessage) *errorMessage = "feature geometry parse failed for " + featureId + ": " + featureError;
          return false;
        }
        features.emplace(featureId, std::move(collector));
      }

      // Optionally convert coordinates from the file's unit to target_unit.
      // Uses CADExchange::TryParseUnitType / TryGetUnitConversionFactor so that
      // all unit arithmetic stays in the canonical CADExchange unit system.
      if (!target_unit.empty() && !file_unit.empty() &&
          target_unit != file_unit) {
        UnitType srcUnit{}, dstUnit{};
        if (TryParseUnitType(file_unit, srcUnit) &&
            TryParseUnitType(target_unit, dstUnit)) {
          double factor = 1.0;
          if (TryGetUnitConversionFactor(srcUnit, dstUnit, factor) &&
              std::abs(factor - 1.0) > 1e-12) {
            for (auto &[id, collector] : features) {
              collector.Scale(factor);
            }
          }
        }
      }

      // Record the effective unit after any conversion.
      length_unit = target_unit.empty() ? file_unit : target_unit;
      return true;
    } catch (const std::exception &e) {
      if (errorMessage) *errorMessage = "Failed to parse geometry json: " + std::string(e.what());
      return false;
    }
  }

  std::size_t TotalEdgeCount() const {
    std::size_t total = 0;
    for (const auto& pair : features) total += pair.second.EdgeCount();
    return total;
  }

  std::size_t TotalDatumPlaneCount() const {
    std::size_t total = 0;
    for (const auto& pair : features) total += pair.second.DatumPlaneCount();
    return total;
  }
};

} // namespace Geometry
} // namespace CADExchange
