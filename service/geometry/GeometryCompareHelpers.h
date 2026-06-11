#pragma once

#include "GeometryTypes.h"
#include <vector>
#include <string>
#include <filesystem>

namespace CADExchange {
namespace Geometry {

// Forward declarations
struct ComparisonResult {
  bool equivalent = true;
  std::vector<std::string> diagnostics;
};

// Declarations of non-template helpers
double PtDist(const CPoint3D& a, const CPoint3D& b) noexcept;
bool PointsNear(const CPoint3D& a, const CPoint3D& b, double tol) noexcept;
bool PointInGroup(const CPoint3D& pt, const HalfStructurePointGroup& group, double tol) noexcept;
bool IsOpenEdge(CGeoCurveType t) noexcept;
bool IsWarnOnlyEdge(CGeoCurveType t) noexcept;
bool ComputeCircumcenter(const CPoint3D& p1, const CPoint3D& p2,
                         const CPoint3D& p3, CPoint3D& center,
                         double& radius) noexcept;

std::vector<NormalizedArc> MergeArcs(const std::vector<NormalizedArc>& arcs,
                                     double tol,
                                     std::vector<CircleType>& promoted_circles,
                                     std::vector<HalfStructurePointGroup>* half_structure_groups = nullptr);

void SimplifyCirclesAndArcs(std::vector<CircleType>& circles,
                            std::vector<NormalizedArc>& arcs,
                            double tol);

std::string FormatPoint(const CPoint3D &pt);
std::string FormatCircle(const CPoint3D &center, double radius);
std::string FormatArc(const NormalizedArc &arc);

bool MatchCircles(const std::vector<CircleType>& src,
                  const std::vector<CircleType>& dst,
                  double tol,
                  std::vector<std::string>* diagnostics);

bool MatchArcs(const std::vector<NormalizedArc>& src,
                const std::vector<NormalizedArc>& dst,
                double tol,
                std::vector<std::string>* diagnostics);

bool IsHalfStructureRedundantEdge(const CRefEdge& edge, const std::vector<HalfStructurePointGroup>& groups, double tol) noexcept;

void FilterHalfStructureEdges(std::vector<CRefEdge>& open_edges, const std::vector<HalfStructurePointGroup>& groups, double tol);

void ClassifyEdges(const std::vector<CRefEdge>& edges,
                  std::vector<CRefEdge>& open_out,
                  std::vector<NormalizedArc>& arc_out,
                  std::vector<CircleType>& circle_out,
                  int& warn_count,
                  double tol);

bool AreCollinear(const CRefEdge& e1, const CRefEdge& e2, double tol,
                  CPoint3D& shared_pt, CPoint3D& new_start, CPoint3D& new_end);

std::vector<CRefEdge> MergeCollinearLines(
    const std::vector<CRefEdge>& lines,
    double tol,
    std::vector<HalfStructurePointGroup>& line_half_groups);

std::string FormatOpenEdge(const CRefEdge &edge);

bool MatchOpenEdges(const std::vector<CRefEdge>& src,
                    const std::vector<CRefEdge>& dst,
                    double tol,
                    std::vector<std::string>* diagnostics);

std::vector<HalfStructurePointGroup> ExtractHalfStructureGroups(
    const std::vector<CRefEdge>& edges, double tol);

std::vector<HalfStructurePointGroup> ExtractHalfStructureLineGroups(
    const std::vector<CRefEdge>& edges, double tol);

namespace detail {
  void ScaleEdges(std::vector<CRefEdge>& edges, double factor) noexcept;
  
  bool SaveGeometryToJson(const std::filesystem::path &filePath,
                          const std::vector<CRefEdge>& edges,
                          const std::vector<CGeoDatumPlane>& datumPlanes,
                          std::string *errorMessage,
                          const std::string &lengthUnit);
                          
  json GeometryToJson(const std::vector<CRefEdge>& edges,
                      const std::vector<CGeoDatumPlane>& datumPlanes);
                      
  bool LoadGeometryFromJson(const json &geometry,
                            std::vector<CRefEdge>& edges,
                            std::vector<CGeoDatumPlane>& datumPlanes,
                            std::string *errorMessage);
                            
  ComparisonResult CompareDetailedImpl(const std::vector<CRefEdge>& src_edges,
                                       const std::vector<CGeoDatumPlane>& src_datumPlanes,
                                       const std::vector<CRefEdge>& dst_edges,
                                       const std::vector<CGeoDatumPlane>& dst_datumPlanes,
                                       double tol,
                                       const std::vector<HalfStructurePointGroup>* global_src_half_groups,
                                       const std::vector<HalfStructurePointGroup>* global_dst_half_groups,
                                       const std::vector<HalfStructurePointGroup>* global_src_line_groups,
                                       const std::vector<HalfStructurePointGroup>* global_dst_line_groups);

  bool SaveModelGeometryToJson(const std::filesystem::path &filePath,
                               const std::vector<std::pair<std::string, json>>& featureList,
                               const std::string &length_unit,
                               std::string *errorMessage);

  bool LoadModelGeometryFromJson(const std::filesystem::path &filePath,
                                 std::vector<std::pair<std::string, json>>& featureList,
                                 std::string &file_unit,
                                 std::string *errorMessage);
} // namespace detail

} // namespace Geometry
} // namespace CADExchange
