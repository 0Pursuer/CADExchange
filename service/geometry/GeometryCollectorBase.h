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

namespace CADExchange {
namespace Geometry {

/**
 * @brief CRTP 基类：统一管理几何边与辅助基准面容器，派生类负责具体 CAD 接口读取。
 *
 * 用法：
 * 1) 派生类实现 `CollectImpl(...)`
 * 2) 通过基类 `Collect(...)` 触发采集（先清空，再调用派生实现）
 */
template <typename Derived, typename EdgeT = CGeoEdge>
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
                       std::string *errorMessage = nullptr) const {
    std::ofstream out(filePath, std::ios::trunc);
    if (!out.is_open()) {
      if (errorMessage) {
        *errorMessage = "Unable to open geometry json output: " +
                        filePath.string();
      }
      return false;
    }

    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10);

    out << "{\n";
    out << "  \"schema_version\": 1,\n";
    out << "  \"edge_count\": " << m_edges.size() << ",\n";
    out << "  \"datum_plane_count\": " << m_datumPlanes.size() << ",\n";
    out << "  \"edges\": [\n";
    for (std::size_t i = 0; i < m_edges.size(); ++i) {
      const auto &edge = m_edges[i];
      out << "    {\n";
      out << "      \"parentFeatureID\": \"" << EscapeJson(edge.parentFeatureID)
          << "\",\n";
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

protected:
  void AddEdge(const EdgeType &edge) { m_edges.push_back(edge); }
  void AddEdge(EdgeType &&edge) { m_edges.emplace_back(std::move(edge)); }
  void AddDatumPlane(const DatumPlaneType &plane) { m_datumPlanes.push_back(plane); }
  void AddDatumPlane(DatumPlaneType &&plane) { m_datumPlanes.emplace_back(std::move(plane)); }

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

  Derived &DerivedSelf() noexcept { return static_cast<Derived &>(*this); }

private:
  std::vector<EdgeType> m_edges;
  std::vector<DatumPlaneType> m_datumPlanes;
};

} // namespace Geometry
} // namespace CADExchange
