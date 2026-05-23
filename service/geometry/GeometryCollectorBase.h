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

namespace Geometry {

namespace detail {

struct SerializablePoint {
  double x = 0;
  double y = 0;
  double z = 0;

  SerializablePoint() = default;
  SerializablePoint(const CPoint3D &pt) : x(pt.x), y(pt.y), z(pt.z) {}
  SerializablePoint(const CVector3D &vec) : x(vec.x), y(vec.y), z(vec.z) {}

  template <class Archive>
  void serialize(Archive &ar) {
    ar(cereal::make_nvp("x", x),
       cereal::make_nvp("y", y),
       cereal::make_nvp("z", z));
  }
};

struct SerializableCSys {
  SerializablePoint origin;
  SerializablePoint xDir;
  SerializablePoint yDir;
  SerializablePoint zDir;

  SerializableCSys() = default;
  SerializableCSys(const CSketchCSys &csys)
      : origin(csys.origin), xDir(csys.xDir), yDir(csys.yDir), zDir(csys.zDir) {}

  template <class Archive>
  void serialize(Archive &ar) {
    ar(cereal::make_nvp("origin", origin),
       cereal::make_nvp("xDir", xDir),
       cereal::make_nvp("yDir", yDir),
       cereal::make_nvp("zDir", zDir));
  }
};

struct SerializableRefEdge {
  std::string parentFeatureID;
  int topologyIndex = -1;
  int curveType = 0;
  SerializablePoint startPoint;
  SerializablePoint endPoint;
  SerializablePoint midPoint;

  SerializableRefEdge() = default;
  SerializableRefEdge(const CRefEdge &edge)
      : parentFeatureID(edge.parentFeatureID),
        topologyIndex(edge.topologyIndex),
        curveType(static_cast<int>(edge.curveType)),
        startPoint(edge.startPoint),
        endPoint(edge.endPoint),
        midPoint(edge.midPoint) {}

  template <class Archive>
  void serialize(Archive &ar) {
    ar(cereal::make_nvp("parentFeatureID", parentFeatureID),
       cereal::make_nvp("topologyIndex", topologyIndex),
       cereal::make_nvp("curveType", curveType),
       cereal::make_nvp("startPoint", startPoint),
       cereal::make_nvp("endPoint", endPoint),
       cereal::make_nvp("midPoint", midPoint));
  }
};

struct SerializableDatumPlane {
  std::string targetFeatureID;
  SerializableCSys localCSys;
  std::string type = "Plane";

  SerializableDatumPlane() = default;
  SerializableDatumPlane(const CGeoDatumPlane &plane)
      : targetFeatureID(plane.targetFeatureID),
        localCSys(plane.localCSys),
        type(plane.type) {}

  template <class Archive>
  void serialize(Archive &ar) {
    ar(cereal::make_nvp("targetFeatureID", targetFeatureID),
       cereal::make_nvp("type", type),
       cereal::make_nvp("localCSys", localCSys));
  }
};

} // namespace detail

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

  bool SaveEdgesToJson(const std::filesystem::path &filePath,
                       std::string *errorMessage = nullptr,
                       const std::string &lengthUnit = "") const {
    std::ofstream out(filePath, std::ios::trunc);
    if (!out.is_open()) {
      if (errorMessage) {
        *errorMessage = "Unable to open geometry json output: " + filePath.string();
      }
      return false;
    }

    out << "{\n";
    out << "  \"schema_version\": 1,\n";
    if (!lengthUnit.empty()) {
      out << "  \"length_unit\": \"" << EscapeJson(lengthUnit) << "\",\n";
    }
    out << "  \"edge_count\": " << m_edges.size() << ",\n";
    out << "  \"datum_plane_count\": " << m_datumPlanes.size() << ",\n";
    out << "  \"edges\": [\n";
    for (std::size_t i = 0; i < m_edges.size(); ++i) {
      const auto &edge = m_edges[i];
      out << "    {\n";
      out << "      \"parentFeatureID\": \""
          << EscapeJson(edge.parentFeatureID) << "\",\n";
      out << "      \"topologyIndex\": " << edge.topologyIndex << ",\n";
      out << "      \"curveType\": \"" << CurveTypeToString(edge.curveType)
          << "\",\n";
      out << "      \"curveTypeValue\": " << static_cast<int>(edge.curveType)
          << ",\n";
      out << "      \"startPoint\": " << FormatPoint(edge.startPoint) << ",\n";
      out << "      \"endPoint\": " << FormatPoint(edge.endPoint) << ",\n";
      out << "      \"midPoint\": " << FormatPoint(edge.midPoint) << "\n";
      out << "    }";
      if (i + 1 < m_edges.size()) {
        out << ",";
      }
      out << "\n";
    }
    out << "  ],\n";
    out << "  \"datum_planes\": [\n";
    for (std::size_t i = 0; i < m_datumPlanes.size(); ++i) {
      const auto &plane = m_datumPlanes[i];
      out << "    {\n";
      out << "      \"targetFeatureID\": \""
          << EscapeJson(plane.targetFeatureID) << "\",\n";
      out << "      \"type\": \"" << EscapeJson(plane.type) << "\",\n";
      out << "      \"origin\": " << FormatPoint(plane.localCSys.origin) << ",\n";
      out << "      \"xDir\": " << FormatVector(plane.localCSys.xDir) << ",\n";
      out << "      \"yDir\": " << FormatVector(plane.localCSys.yDir) << ",\n";
      out << "      \"normal\": " << FormatVector(plane.localCSys.zDir) << "\n";
      out << "    }";
      if (i + 1 < m_datumPlanes.size()) {
        out << ",";
      }
      out << "\n";
    }
    out << "  ]\n";
    out << "}\n";

    if (!out.good()) {
      if (errorMessage) {
        *errorMessage = "Failed to write geometry json: " + filePath.string();
      }
      return false;
    }
    return true;
  }

  // Serialization for the collector itself
  template <class Archive>
  void save(Archive &ar) const {
    std::vector<detail::SerializableRefEdge> tempEdges;
    tempEdges.reserve(m_edges.size());
    for (const auto &edge : m_edges) {
      tempEdges.emplace_back(edge);
    }

    std::vector<detail::SerializableDatumPlane> tempPlanes;
    tempPlanes.reserve(m_datumPlanes.size());
    for (const auto &plane : m_datumPlanes) {
      tempPlanes.emplace_back(plane);
    }

    ar(cereal::make_nvp("edges", tempEdges),
       cereal::make_nvp("datumPlanes", tempPlanes));
  }

  template <class Archive>
  void load(Archive &ar) {
    std::vector<detail::SerializableRefEdge> tempEdges;
    std::vector<detail::SerializableDatumPlane> tempPlanes;

    ar(cereal::make_nvp("edges", tempEdges),
       cereal::make_nvp("datumPlanes", tempPlanes));

    m_edges.clear();
    m_edges.reserve(tempEdges.size());
    for (const auto &te : tempEdges) {
      CRefEdge edge;
      edge.parentFeatureID = te.parentFeatureID;
      edge.topologyIndex = te.topologyIndex;
      edge.curveType = static_cast<CGeoCurveType>(te.curveType);
      edge.startPoint = CPoint3D{te.startPoint.x, te.startPoint.y, te.startPoint.z};
      edge.endPoint = CPoint3D{te.endPoint.x, te.endPoint.y, te.endPoint.z};
      edge.midPoint = CPoint3D{te.midPoint.x, te.midPoint.y, te.midPoint.z};
      m_edges.push_back(std::move(edge));
    }

    m_datumPlanes.clear();
    m_datumPlanes.reserve(tempPlanes.size());
    for (const auto &tp : tempPlanes) {
      CGeoDatumPlane plane;
      plane.targetFeatureID = tp.targetFeatureID;
      plane.type = tp.type;
      plane.localCSys.origin = CPoint3D{tp.localCSys.origin.x, tp.localCSys.origin.y, tp.localCSys.origin.z};
      plane.localCSys.xDir = CVector3D{tp.localCSys.xDir.x, tp.localCSys.xDir.y, tp.localCSys.xDir.z};
      plane.localCSys.yDir = CVector3D{tp.localCSys.yDir.x, tp.localCSys.yDir.y, tp.localCSys.yDir.z};
      plane.localCSys.zDir = CVector3D{tp.localCSys.zDir.x, tp.localCSys.zDir.y, tp.localCSys.zDir.z};
      m_datumPlanes.push_back(std::move(plane));
    }
  }

  // Equivalency check for runtime validation.
  //
  // Edge classification strategy:
  //   CIRCLE (3002) with start==end, mid!=start  → TRUE CIRCLE  (center=(start+mid)/2, r=dist/2)
  //   CIRCLE (3002) with start!=end              → ARC          (circumcenter of start/mid/end)
  //   CIRCLE (3002) with start==end==mid         → DEGENERATE   (skip)
  //   LINE/ELLIPSE/BCURVE/TRIMMED               → open edge    (mid pre-filter + endpoint check)
  //   INTERSECTION/SPCURVE/CONSTPARAM           → warn-only    (count mismatch → WARN, not FAIL)
  //
  // ARC merging: arcs with identical center+radius that connect end-to-end within the same
  // JSON are merged; a closed loop of arcs is promoted to a TRUE CIRCLE.
  //
  // Matching:
  //   TRUE CIRCLEs  : center + radius
  //   ARCs          : center + radius + endpoints (forward or reversed)
  //   Open edges    : curveType + mid (pre-filter) + endpoints (forward or reversed)
  bool IsEquivalent(const GeometryCollectorBase& other, double tol = 2e-3) const {
    if (m_datumPlanes.size() != other.m_datumPlanes.size()) {
      std::cout << "[DEBUG] IsEquivalent FAIL: datum plane count "
                << m_datumPlanes.size() << " vs " << other.m_datumPlanes.size() << "\n";
      return false;
    }

    // ── Step 1+2: classify and parameterise every edge ───────────────────
    std::vector<EdgeType>                        src_open, dst_open;
    std::vector<NormalizedArc>                   src_arcs, dst_arcs;
    std::vector<std::pair<CPoint3D, double>>     src_circles, dst_circles;
    int src_warn = 0, dst_warn = 0;
    ClassifyEdges(m_edges,       src_open, src_arcs, src_circles, src_warn, tol);
    ClassifyEdges(other.m_edges, dst_open, dst_arcs, dst_circles, dst_warn, tol);

    // ── Step 3: merge connected arcs within each side ────────────────────
    std::vector<std::pair<CPoint3D, double>> promoted_src, promoted_dst;
    src_arcs = MergeArcs(src_arcs, tol, promoted_src);
    dst_arcs = MergeArcs(dst_arcs, tol, promoted_dst);
    for (auto& p : promoted_src) src_circles.push_back(p);
    for (auto& p : promoted_dst) dst_circles.push_back(p);

    // ── Step 4A: TRUE CIRCLE matching (center + radius) ──────────────────
    if (!MatchCircles(src_circles, dst_circles, tol)) return false;

    // ── Step 4B: ARC matching (center + radius + endpoints) ──────────────
    if (!MatchArcs(src_arcs, dst_arcs, tol)) return false;

    // ── Step 4C: open-edge matching (mid pre-filter + endpoint check) ────
    if (!MatchOpenEdges(src_open, dst_open, tol)) return false;

    // ── Step 4D: warn-only types (INTERSECTION / SPCURVE / CONSTPARAM) ──
    if (src_warn != dst_warn) {
      std::cout << "[DEBUG] IsEquivalent WARN-ONLY edge count mismatch: "
                << src_warn << " vs " << dst_warn << " (not a failure)\n";
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
  static std::string EscapeJson(const std::string &value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (unsigned char ch : value) {
      switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (ch < 0x20) {
          static const char kHex[] = "0123456789ABCDEF";
          out += "\\u00";
          out.push_back(kHex[(ch >> 4) & 0xF]);
          out.push_back(kHex[ch & 0xF]);
        } else {
          out.push_back(static_cast<char>(ch));
        }
        break;
      }
    }
    return out;
  }

  static std::string FormatPoint(const CPoint3D &pt) {
    return "{\"x\":" + FormatNumber(pt.x) + ",\"y\":" +
           FormatNumber(pt.y) + ",\"z\":" + FormatNumber(pt.z) + "}";
  }

  static std::string FormatVector(const CVector3D &vec) {
    return "{\"x\":" + FormatNumber(vec.x) + ",\"y\":" +
           FormatNumber(vec.y) + ",\"z\":" + FormatNumber(vec.z) + "}";
  }

  static std::string CurveTypeToString(CGeoCurveType type) {
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

  static std::string FormatNumber(double value) {
    std::ostringstream oss;
    oss.imbue(std::locale::classic());
    oss << std::setprecision(std::numeric_limits<double>::max_digits10)
        << value;
    return oss.str();
  }

  // ═══════════════════════════════════════════════════════════════════════
  // Private helpers for IsEquivalent
  // ═══════════════════════════════════════════════════════════════════════

  /// Intermediate representation of a CIRCLE edge after parameterisation.
  struct NormalizedArc {
    CPoint3D center{};    ///< circumcenter (or (start+mid)/2 for true circles)
    double   radius = 0;  ///< circumradius
    CPoint3D startPt{};   ///< original startPoint
    CPoint3D endPt{};     ///< original endPoint
  };

  static double PtDist(const CPoint3D& a, const CPoint3D& b) noexcept {
    double dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  static bool IsOpenEdge(CGeoCurveType t) noexcept {
    return t == CGeoCurveType::LINE    || t == CGeoCurveType::ELLIPSE ||
           t == CGeoCurveType::BCURVE  || t == CGeoCurveType::TRIMMED;
  }

  static bool IsWarnOnlyEdge(CGeoCurveType t) noexcept {
    return t == CGeoCurveType::INTERSECTION ||
           t == CGeoCurveType::SPCURVE      ||
           t == CGeoCurveType::CONSTPARAM;
  }

  /// Compute the circumcenter of three points in 3-D.
  /// Uses the formula: C = p1 + s*(p2-p1) + t*(p3-p1) where
  ///   s = |v|^2*(|u|^2 - u.v) / (2*(|u|^2|v|^2 - (u.v)^2))
  ///   t = |u|^2*(|v|^2 - u.v) / (2*(|u|^2|v|^2 - (u.v)^2))
  ///   u = p2-p1,  v = p3-p1
  /// Returns false if the three points are (nearly) collinear.
  static bool ComputeCircumcenter(const CPoint3D& p1, const CPoint3D& p2, const CPoint3D& p3,
                                   CPoint3D& center, double& radius) noexcept {
    double ax = p2.x-p1.x, ay = p2.y-p1.y, az = p2.z-p1.z; // u
    double bx = p3.x-p1.x, by = p3.y-p1.y, bz = p3.z-p1.z; // v
    double uu = ax*ax + ay*ay + az*az;
    double uv = ax*bx + ay*by + az*bz;
    double vv = bx*bx + by*by + bz*bz;
    double det = uu*vv - uv*uv;          // = |u x v|^2
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

  /// Classify each edge into one of four buckets:
  ///   open_out     – LINE / ELLIPSE / BCURVE / TRIMMED
  ///   arc_out      – CIRCLE with start != end  (non-degenerate arc)
  ///   circle_out   – CIRCLE with start == end, mid != start (true circle)
  ///   warn_count   – INTERSECTION / SPCURVE / CONSTPARAM  (warn-only)
  ///   truly degenerate edges (all three sample points coincide) are silently dropped.
  static void ClassifyEdges(
      const std::vector<EdgeType>&              edges,
      std::vector<EdgeType>&                    open_out,
      std::vector<NormalizedArc>&               arc_out,
      std::vector<std::pair<CPoint3D,double>>&  circle_out,
      int&                                      warn_count,
      double                                    tol)
  {
    for (const auto& e : edges) {
      if (e.curveType == CGeoCurveType::UNKNOWN)  continue;
      if (IsWarnOnlyEdge(e.curveType))            { ++warn_count; continue; }
      if (IsOpenEdge(e.curveType))                { open_out.push_back(e); continue; }
      if (e.curveType != CGeoCurveType::CIRCLE)   continue;

      double se_dist = PtDist(e.startPoint, e.endPoint);
      double sm_dist = PtDist(e.startPoint, e.midPoint);

      if (se_dist <= tol) {
        // start == end
        if (sm_dist <= tol) {
          // All three sample points coincide → truly degenerate, skip.
          continue;
        }
        // start == end, mid != start → TRUE CIRCLE.
        // start and mid are antipodal points; diameter = dist(start, mid).
        CPoint3D cen;
        cen.x = (e.startPoint.x + e.midPoint.x) * 0.5;
        cen.y = (e.startPoint.y + e.midPoint.y) * 0.5;
        cen.z = (e.startPoint.z + e.midPoint.z) * 0.5;
        circle_out.emplace_back(cen, sm_dist * 0.5);
      } else {
        // start != end → ARC.  Compute circumcenter from (start, mid, end).
        NormalizedArc arc;
        if (!ComputeCircumcenter(e.startPoint, e.midPoint, e.endPoint,
                                 arc.center, arc.radius)) {
          continue; // collinear sample points – skip
        }
        arc.startPt = e.startPoint;
        arc.endPt   = e.endPoint;
        arc_out.push_back(arc);
      }
    }
  }

  /// Within one side's arc list, merge arcs that share the same
  /// circumcircle (center + radius within tol) and are connected end-to-end.
  /// A chain that closes into a full loop is promoted to a true circle.
  static std::vector<NormalizedArc> MergeArcs(
      const std::vector<NormalizedArc>&          arcs,
      double                                     tol,
      std::vector<std::pair<CPoint3D,double>>&   promoted_circles)
  {
    std::vector<bool>         used(arcs.size(), false);
    std::vector<NormalizedArc> result;

    for (size_t i = 0; i < arcs.size(); ++i) {
      if (used[i]) continue;
      bool found_partner = false;

      for (size_t j = i+1; j < arcs.size(); ++j) {
        if (used[j]) continue;
        if (PtDist(arcs[i].center, arcs[j].center) > tol)     continue;
        if (std::abs(arcs[i].radius - arcs[j].radius)  > tol) continue;

        // Check whether the two arcs together form a closed loop:
        //   i.end → j.start and j.end → i.start  (forward)
        //   i.end → j.end   and j.start → i.start (reversed adjacency)
        bool loop_fwd = PtDist(arcs[i].endPt,   arcs[j].startPt) <= tol &&
                        PtDist(arcs[j].endPt,   arcs[i].startPt) <= tol;
        bool loop_rev = PtDist(arcs[i].endPt,   arcs[j].endPt)   <= tol &&
                        PtDist(arcs[i].startPt, arcs[j].startPt) <= tol;
        if (loop_fwd || loop_rev) {
          promoted_circles.emplace_back(arcs[i].center, arcs[i].radius);
          used[i] = used[j] = true;
          found_partner = true;
          break;
        }

        // Chain i → j  (i.end ≈ j.start)
        if (PtDist(arcs[i].endPt, arcs[j].startPt) <= tol) {
          NormalizedArc merged;
          merged.center  = arcs[i].center;
          merged.radius  = arcs[i].radius;
          merged.startPt = arcs[i].startPt;
          merged.endPt   = arcs[j].endPt;
          result.push_back(merged);
          used[i] = used[j] = true;
          found_partner = true;
          break;
        }
        // Chain j → i  (j.end ≈ i.start)
        if (PtDist(arcs[j].endPt, arcs[i].startPt) <= tol) {
          NormalizedArc merged;
          merged.center  = arcs[i].center;
          merged.radius  = arcs[i].radius;
          merged.startPt = arcs[j].startPt;
          merged.endPt   = arcs[i].endPt;
          result.push_back(merged);
          used[i] = used[j] = true;
          found_partner = true;
          break;
        }
      }

      if (!found_partner) result.push_back(arcs[i]);
    }
    return result;
  }

  /// Match TRUE CIRCLEs: each src circle must find a dst circle with the
  /// same center (within tol) and radius (within tol).
  static bool MatchCircles(
      const std::vector<std::pair<CPoint3D,double>>& src,
      const std::vector<std::pair<CPoint3D,double>>& dst,
      double tol)
  {
    std::vector<bool> used(dst.size(), false);
    for (const auto& [sc, sr] : src) {
      bool found = false;
      for (size_t j = 0; j < dst.size(); ++j) {
        if (used[j]) continue;
        if (PtDist(sc, dst[j].first) <= tol &&
            std::abs(sr - dst[j].second) <= tol) {
          used[j] = true; found = true; break;
        }
      }
      if (!found) {
        std::cout << std::fixed << std::setprecision(6)
                  << "[DEBUG] IsEquivalent: unmatched TRUE CIRCLE"
                  << " center=(" << sc.x << "," << sc.y << "," << sc.z
                  << ") r=" << sr << "\n";
        return false;
      }
    }
    for (size_t j = 0; j < dst.size(); ++j) {
      if (!used[j]) {
        const auto& [dc, dr] = dst[j];
        std::cout << std::fixed << std::setprecision(6)
                  << "[DEBUG] IsEquivalent: extra TRUE CIRCLE in rebuilt"
                  << " center=(" << dc.x << "," << dc.y << "," << dc.z
                  << ") r=" << dr << "\n";
        return false;
      }
    }
    return true;
  }

  /// Match ARCs: center + radius + endpoints (forward or reversed orientation).
  static bool MatchArcs(
      const std::vector<NormalizedArc>& src,
      const std::vector<NormalizedArc>& dst,
      double tol)
  {
    std::vector<bool> used(dst.size(), false);
    for (const auto& sa : src) {
      bool found = false;
      for (size_t j = 0; j < dst.size(); ++j) {
        if (used[j]) continue;
        const auto& da = dst[j];
        if (PtDist(sa.center, da.center) > tol)        continue;
        if (std::abs(sa.radius - da.radius) > tol)     continue;
        double fwd = (std::max)(PtDist(sa.startPt, da.startPt),
                                PtDist(sa.endPt,   da.endPt));
        double rev = (std::max)(PtDist(sa.startPt, da.endPt),
                                PtDist(sa.endPt,   da.startPt));
        if ((std::min)(fwd, rev) <= tol) {
          used[j] = true; found = true; break;
        }
      }
      if (!found) {
        std::cout << std::fixed << std::setprecision(6)
                  << "[DEBUG] IsEquivalent: unmatched ARC"
                  << " center=(" << sa.center.x << "," << sa.center.y
                  << "," << sa.center.z << ") r=" << sa.radius
                  << " start=(" << sa.startPt.x << "," << sa.startPt.y
                  << "," << sa.startPt.z << ")"
                  << " end=("   << sa.endPt.x   << "," << sa.endPt.y
                  << "," << sa.endPt.z   << ")\n";
        return false;
      }
    }
    for (size_t j = 0; j < dst.size(); ++j) {
      if (!used[j]) {
        const auto& da = dst[j];
        std::cout << std::fixed << std::setprecision(6)
                  << "[DEBUG] IsEquivalent: extra ARC in rebuilt"
                  << " center=(" << da.center.x << "," << da.center.y
                  << "," << da.center.z << ") r=" << da.radius << "\n";
        return false;
      }
    }
    return true;
  }

  /// Match open edges (LINE/ELLIPSE/BCURVE/TRIMMED).
  /// Uses midPoint as a fast spatial pre-filter, then verifies endpoints
  /// in both forward and reversed orientations.
  static bool MatchOpenEdges(
      const std::vector<EdgeType>& src,
      const std::vector<EdgeType>& dst,
      double tol)
  {
    std::vector<bool> used(dst.size(), false);
    for (const auto& se : src) {
      bool found = false;
      for (size_t j = 0; j < dst.size(); ++j) {
        if (used[j]) continue;
        const auto& de = dst[j];
        if (se.curveType != de.curveType)              continue;
        if (PtDist(se.midPoint, de.midPoint) > tol)    continue; // spatial pre-filter
        double fwd = (std::max)(PtDist(se.startPoint, de.startPoint),
                                PtDist(se.endPoint,   de.endPoint));
        double rev = (std::max)(PtDist(se.startPoint, de.endPoint),
                                PtDist(se.endPoint,   de.startPoint));
        if ((std::min)(fwd, rev) <= tol) {
          used[j] = true; found = true; break;
        }
      }
      if (!found) {
        std::cout << std::fixed << std::setprecision(6)
                  << "[DEBUG] IsEquivalent: unmatched open edge type="
                  << static_cast<int>(se.curveType)
                  << " mid=(" << se.midPoint.x << "," << se.midPoint.y
                  << "," << se.midPoint.z << ")\n";
        return false;
      }
    }
    for (size_t j = 0; j < dst.size(); ++j) {
      if (!used[j]) {
        const auto& de = dst[j];
        std::cout << std::fixed << std::setprecision(6)
                  << "[DEBUG] IsEquivalent: extra open edge in rebuilt type="
                  << static_cast<int>(de.curveType)
                  << " mid=(" << de.midPoint.x << "," << de.midPoint.y
                  << "," << de.midPoint.z << ")\n";
        return false;
      }
    }
    return true;
  }

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
