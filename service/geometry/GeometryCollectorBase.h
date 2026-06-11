#pragma once

#include "../../core/UnifiedFeatures.h"
#include "../../core/UnifiedModel.h"
#include "GeometryTypes.h"
#include "GeometryCompareHelpers.h"

#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <iostream>

namespace CADExchange {
namespace Geometry {

template <typename Derived, typename EdgeT = CRefEdge>
class GeometryCollectorBase {
public:
  using EdgeType = EdgeT;
  using DatumPlaneType = CGeoDatumPlane;
  using ComparisonResult = Geometry::ComparisonResult;

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
    detail::ScaleEdges(m_edges, factor);
  }

  bool SaveEdgesToJson(const std::filesystem::path &filePath,
                       std::string *errorMessage = nullptr,
                       const std::string &lengthUnit = "") const {
    return detail::SaveGeometryToJson(filePath, m_edges, m_datumPlanes, errorMessage, lengthUnit);
  }

  detail::json ToJsonValue() const {
    return detail::GeometryToJson(m_edges, m_datumPlanes);
  }

  bool LoadFromJsonValue(const detail::json &geometry,
                         std::string *errorMessage = nullptr) {
    return detail::LoadGeometryFromJson(geometry, m_edges, m_datumPlanes, errorMessage);
  }

  static std::vector<HalfStructurePointGroup> ExtractHalfStructureGroups(
      const std::vector<EdgeType>& edges, double tol) {
    return Geometry::ExtractHalfStructureGroups(edges, tol);
  }

  static std::vector<HalfStructurePointGroup> ExtractHalfStructureLineGroups(
      const std::vector<EdgeType>& edges, double tol) {
    return Geometry::ExtractHalfStructureLineGroups(edges, tol);
  }

  ComparisonResult CompareDetailed(const GeometryCollectorBase& other,
                                   double tol = 2e-3,
                                   const std::vector<HalfStructurePointGroup>* global_src_half_groups = nullptr,
                                   const std::vector<HalfStructurePointGroup>* global_dst_half_groups = nullptr,
                                   const std::vector<HalfStructurePointGroup>* global_src_line_groups = nullptr,
                                   const std::vector<HalfStructurePointGroup>* global_dst_line_groups = nullptr) const {
    return detail::CompareDetailedImpl(m_edges, m_datumPlanes, other.m_edges, other.m_datumPlanes,
                                       tol, global_src_half_groups, global_dst_half_groups,
                                       global_src_line_groups, global_dst_line_groups);
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
  std::vector<EdgeType> m_edges;
  std::vector<DatumPlaneType> m_datumPlanes;
};

// Dummy derived class to support instantiation for schema reading/verification
class GeometryCollectorBaseDummyDerived : public GeometryCollectorBase<GeometryCollectorBaseDummyDerived> {
public:
  template <typename... Args>
  bool CollectImpl(Args &&...) { return true; }
};

template <typename CollectorT>
class ModelGeometrySet {
public:
  std::map<std::string, CollectorT> features;
  std::string length_unit;

  bool SaveToJson(const std::filesystem::path &filePath,
                  std::string *errorMessage = nullptr) const {
    std::vector<std::pair<std::string, detail::json>> featureList;
    for (const auto &[featureId, collector] : features) {
      featureList.emplace_back(featureId, collector.ToJsonValue());
    }
    return detail::SaveModelGeometryToJson(filePath, featureList, length_unit, errorMessage);
  }

  bool LoadFromJson(const std::filesystem::path &filePath,
                    std::string *errorMessage = nullptr,
                    const std::string &target_unit = "") {
    std::vector<std::pair<std::string, detail::json>> featureList;
    std::string file_unit;
    if (!detail::LoadModelGeometryFromJson(filePath, featureList, file_unit, errorMessage)) {
      return false;
    }
    features.clear();
    for (auto &[featureId, entryJson] : featureList) {
      CollectorT collector;
      std::string featureError;
      if (!collector.LoadFromJsonValue(entryJson, &featureError)) {
        if (errorMessage) *errorMessage = "feature geometry parse failed for " + featureId + ": " + featureError;
        return false;
      }
      features.emplace(featureId, std::move(collector));
    }

    // Optionally convert coordinates from the file's unit to target_unit.
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

    length_unit = target_unit.empty() ? file_unit : target_unit;
    return true;
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

using GeometrySet = ModelGeometrySet<GeometryCollectorBaseDummyDerived>;

} // namespace Geometry
} // namespace CADExchange
