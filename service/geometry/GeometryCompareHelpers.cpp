#include "GeometryCompareHelpers.h"
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace CADExchange {
namespace Geometry {

double PtDist(const CPoint3D& a, const CPoint3D& b) noexcept {
  double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

bool PointsNear(const CPoint3D& a, const CPoint3D& b, double tol) noexcept {
  return PtDist(a, b) <= tol;
}

bool PointInGroup(const CPoint3D& pt, const HalfStructurePointGroup& group, double tol) noexcept {
  for (const auto& candidate : group.points) {
    if (PointsNear(pt, candidate, tol)) {
      return true;
    }
  }
  return false;
}

bool IsOpenEdge(CGeoCurveType t) noexcept {
  return t == CGeoCurveType::LINE || t == CGeoCurveType::ELLIPSE ||
         t == CGeoCurveType::BCURVE || t == CGeoCurveType::TRIMMED;
}

bool IsWarnOnlyEdge(CGeoCurveType t) noexcept {
  return t == CGeoCurveType::INTERSECTION || t == CGeoCurveType::SPCURVE ||
         t == CGeoCurveType::CONSTPARAM;
}

bool ComputeCircumcenter(const CPoint3D& p1, const CPoint3D& p2,
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

std::vector<NormalizedArc> MergeArcs(const std::vector<NormalizedArc>& arcs,
                                     double tol,
                                     std::vector<CircleType>& promoted_circles,
                                     std::vector<HalfStructurePointGroup>* half_structure_groups) {
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

void SimplifyCirclesAndArcs(std::vector<CircleType>& circles,
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

std::string FormatPoint(const CPoint3D &pt) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "(" << pt.x << "," << pt.y << "," << pt.z << ")";
  return oss.str();
}

std::string FormatCircle(const CPoint3D &center, double radius) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "center=" << FormatPoint(center) << " r=" << radius;
  return oss.str();
}

std::string FormatArc(const NormalizedArc &arc) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "center=" << FormatPoint(arc.center)
      << " r=" << arc.radius
      << " start=" << FormatPoint(arc.startPt)
      << " end=" << FormatPoint(arc.endPt);
  return oss.str();
}

bool MatchCircles(const std::vector<CircleType>& src,
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

bool MatchArcs(const std::vector<NormalizedArc>& src,
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

bool IsHalfStructureRedundantEdge(const CRefEdge& edge, const std::vector<HalfStructurePointGroup>& groups, double tol) noexcept {
  for (const auto& group : groups) {
    if (PointInGroup(edge.startPoint, group, tol) ||
        PointInGroup(edge.endPoint, group, tol)) {
      return true;
    }
  }
  return false;
}

void FilterHalfStructureEdges(std::vector<CRefEdge>& open_edges, const std::vector<HalfStructurePointGroup>& groups, double tol) {
  if (groups.empty() || open_edges.empty()) {
    return;
  }
  open_edges.erase(
      std::remove_if(open_edges.begin(), open_edges.end(),
                     [&](const CRefEdge& edge) {
                       return IsHalfStructureRedundantEdge(edge, groups, tol);
                     }),
      open_edges.end());
}

void ClassifyEdges(const std::vector<CRefEdge>& edges,
                  std::vector<CRefEdge>& open_out,
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

bool AreCollinear(const CRefEdge& e1, const CRefEdge& e2, double tol,
                  CPoint3D& shared_pt, CPoint3D& new_start, CPoint3D& new_end) {
  if (e1.curveType != CGeoCurveType::LINE || e2.curveType != CGeoCurveType::LINE) {
    return false;
  }
  
  // Check adjacency of endpoints
  bool adjacent = false;
  CPoint3D p1_start = e1.startPoint;
  CPoint3D p1_end = e1.endPoint;
  CPoint3D p2_start = e2.startPoint;
  CPoint3D p2_end = e2.endPoint;
  
  if (PtDist(p1_end, p2_start) <= tol) {
    adjacent = true;
    shared_pt = p1_end;
    new_start = p1_start;
    new_end = p2_end;
  } else if (PtDist(p1_end, p2_end) <= tol) {
    adjacent = true;
    shared_pt = p1_end;
    new_start = p1_start;
    new_end = p2_start;
  } else if (PtDist(p1_start, p2_start) <= tol) {
    adjacent = true;
    shared_pt = p1_start;
    new_start = p1_end;
    new_end = p2_end;
  } else if (PtDist(p1_start, p2_end) <= tol) {
    adjacent = true;
    shared_pt = p1_start;
    new_start = p1_end;
    new_end = p2_start;
  }
  
  if (!adjacent) {
    return false;
  }
  
  // Check collinearity
  // Vector of segment 1
  CVector3D v1 = p1_end - p1_start;
  // Vector of segment 2
  CVector3D v2 = p2_end - p2_start;
  
  // Normalize directions
  double len1 = std::sqrt(v1.x*v1.x + v1.y*v1.y + v1.z*v1.z);
  double len2 = std::sqrt(v2.x*v2.x + v2.y*v2.y + v2.z*v2.z);
  
  if (len1 < 1e-9 || len2 < 1e-9) {
    return false;
  }
  
  CVector3D u1{v1.x / len1, v1.y / len1, v1.z / len1};
  CVector3D u2{v2.x / len2, v2.y / len2, v2.z / len2};
  
  // For collinearity, the direction vectors should be parallel (dot product close to 1 or -1)
  double dot = u1.Dot(u2);
  if (std::abs(std::abs(dot) - 1.0) > 1e-4) {
    return false;
  }
  
  // Additionally, check if the combined segment new_start -> new_end is parallel to u1
  CVector3D v_comb = new_end - new_start;
  double len_comb = std::sqrt(v_comb.x*v_comb.x + v_comb.y*v_comb.y + v_comb.z*v_comb.z);
  if (len_comb > 1e-9) {
    CVector3D u_comb{v_comb.x / len_comb, v_comb.y / len_comb, v_comb.z / len_comb};
    double dot_comb = u1.Dot(u_comb);
    if (std::abs(std::abs(dot_comb) - 1.0) > 1e-4) {
      return false;
    }
  }
  
  return true;
}

std::vector<CRefEdge> MergeCollinearLines(
    const std::vector<CRefEdge>& lines,
    double tol,
    std::vector<HalfStructurePointGroup>& line_half_groups) {
  std::vector<CRefEdge> current_lines = lines;
  bool changed = true;
  while (changed) {
    changed = false;
    std::vector<bool> used(current_lines.size(), false);
    std::vector<CRefEdge> next_lines;
    for (size_t i = 0; i < current_lines.size(); ++i) {
      if (used[i]) continue;
      bool found_partner = false;
      for (size_t j = i + 1; j < current_lines.size(); ++j) {
        if (used[j]) continue;
        
        CPoint3D shared_pt;
        CPoint3D new_start;
        CPoint3D new_end;
        if (AreCollinear(current_lines[i], current_lines[j], tol, shared_pt, new_start, new_end)) {
          // Merge them!
          CRefEdge merged = current_lines[i]; // copy properties
          merged.startPoint = new_start;
          merged.endPoint = new_end;
          merged.midPoint = CPoint3D{(new_start.x + new_end.x) * 0.5,
                                    (new_start.y + new_end.y) * 0.5,
                                    (new_start.z + new_end.z) * 0.5};
          
          // Add to half structure groups
          HalfStructurePointGroup group;
          group.center = CPoint3D{0, 0, 0};
          group.radius = 0.0;
          group.points.push_back(shared_pt);
          line_half_groups.push_back(std::move(group));
          
          next_lines.push_back(merged);
          used[i] = used[j] = true;
          found_partner = true;
          changed = true;
          break;
        }
      }
      if (!found_partner) {
        next_lines.push_back(current_lines[i]);
      }
    }
    current_lines = std::move(next_lines);
  }
  return current_lines;
}

std::string FormatOpenEdge(const CRefEdge &edge) {
  std::ostringstream oss;
  oss << "type=" << static_cast<int>(edge.curveType)
      << " start=" << FormatPoint(edge.startPoint)
      << " mid=" << FormatPoint(edge.midPoint)
      << " end=" << FormatPoint(edge.endPoint);
  return oss.str();
}

bool MatchOpenEdges(const std::vector<CRefEdge>& src,
                    const std::vector<CRefEdge>& dst,
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

std::vector<HalfStructurePointGroup> ExtractHalfStructureGroups(
    const std::vector<CRefEdge>& edges, double tol) {
  std::vector<CRefEdge> open;
  std::vector<NormalizedArc> arcs;
  std::vector<CircleType> circles;
  int warn = 0;
  ClassifyEdges(edges, open, arcs, circles, warn, tol);
  std::vector<CircleType> promoted;
  std::vector<HalfStructurePointGroup> groups;
  MergeArcs(arcs, tol, promoted, &groups);
  return groups;
}

std::vector<HalfStructurePointGroup> ExtractHalfStructureLineGroups(
    const std::vector<CRefEdge>& edges, double tol) {
  std::vector<CRefEdge> open;
  std::vector<NormalizedArc> arcs;
  std::vector<CircleType> circles;
  int warn = 0;
  ClassifyEdges(edges, open, arcs, circles, warn, tol);
  std::vector<HalfStructurePointGroup> line_groups;
  open = MergeCollinearLines(open, tol, line_groups);
  return line_groups;
}

namespace detail {

json PointToJson(const CPoint3D &pt) {
  return json{{"x", pt.x}, {"y", pt.y}, {"z", pt.z}};
}

json VectorToJson(const CVector3D &vec) {
  return json{{"x", vec.x}, {"y", vec.y}, {"z", vec.z}};
}

bool TryReadPoint(const json &node, CPoint3D &pt) {
  if (!node.is_object() || !node.contains("x") || !node.contains("y") ||
      !node.contains("z")) {
    return false;
  }
  pt.x = node.at("x").get<double>();
  pt.y = node.at("y").get<double>();
  pt.z = node.at("z").get<double>();
  return true;
}

bool TryReadVector(const json &node, CVector3D &vec) {
  if (!node.is_object() || !node.contains("x") || !node.contains("y") ||
      !node.contains("z")) {
    return false;
  }
  vec.x = node.at("x").get<double>();
  vec.y = node.at("y").get<double>();
  vec.z = node.at("z").get<double>();
  return true;
}

std::string CurveTypeToString(CGeoCurveType type) {
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

CGeoCurveType CurveTypeFromString(const std::string &typeName) {
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

void ScaleEdges(std::vector<CRefEdge>& edges, double factor) noexcept {
  for (auto &edge : edges) {
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

bool SaveGeometryToJson(const std::filesystem::path &filePath,
                        const std::vector<CRefEdge>& edges,
                        const std::vector<CGeoDatumPlane>& datumPlanes,
                        std::string *errorMessage,
                        const std::string &lengthUnit) {
  try {
    json fileRoot;
    fileRoot["schema_version"] = 1;
    if (!lengthUnit.empty()) {
      fileRoot["length_unit"] = lengthUnit;
    }
    fileRoot["edge_count"] = edges.size();
    fileRoot["datum_plane_count"] = datumPlanes.size();
    fileRoot["edges"] = json::array();
    for (const auto &edge : edges) {
      fileRoot["edges"].push_back(json{{"parentFeatureID", edge.parentFeatureID},
                                        {"topologyIndex", edge.topologyIndex},
                                        {"curveType", CurveTypeToString(edge.curveType)},
                                        {"curveTypeValue", static_cast<int>(edge.curveType)},
                                        {"startPoint", PointToJson(edge.startPoint)},
                                        {"endPoint", PointToJson(edge.endPoint)},
                                        {"midPoint", PointToJson(edge.midPoint)}});
    }
    fileRoot["datum_planes"] = json::array();
    for (const auto &plane : datumPlanes) {
      fileRoot["datum_planes"].push_back(json{{"targetFeatureID", plane.targetFeatureID},
                                               {"type", plane.type},
                                               {"origin", PointToJson(plane.localCSys.origin)},
                                               {"xDir", VectorToJson(plane.localCSys.xDir)},
                                               {"yDir", VectorToJson(plane.localCSys.yDir)},
                                               {"normal", VectorToJson(plane.localCSys.zDir)}});
    }

    std::ofstream out(filePath, std::ios::trunc);
    if (!out.is_open()) {
      if (errorMessage) {
        *errorMessage = "Unable to open geometry json output: " + filePath.string();
      }
      return false;
    }
    out << fileRoot.dump(2) << '\n';
    return true;
  } catch (const std::exception &e) {
    if (errorMessage) {
      *errorMessage = "Failed to write geometry json: " + std::string(e.what());
    }
    return false;
  }
}

json GeometryToJson(const std::vector<CRefEdge>& edges,
                    const std::vector<CGeoDatumPlane>& datumPlanes) {
  json geometry;
  geometry["edges"] = json::array();
  for (const auto &edge : edges) {
    geometry["edges"].push_back(json{{"parentFeatureID", edge.parentFeatureID},
                                      {"topologyIndex", edge.topologyIndex},
                                      {"curveType", static_cast<int>(edge.curveType)},
                                      {"startPoint", PointToJson(edge.startPoint)},
                                      {"endPoint", PointToJson(edge.endPoint)},
                                      {"midPoint", PointToJson(edge.midPoint)}});
  }
  geometry["datumPlanes"] = json::array();
  for (const auto &plane : datumPlanes) {
    geometry["datumPlanes"].push_back(json{{"targetFeatureID", plane.targetFeatureID},
                                            {"type", plane.type},
                                            {"localCSys", json{{"origin", PointToJson(plane.localCSys.origin)},
                                                                {"xDir", VectorToJson(plane.localCSys.xDir)},
                                                                {"yDir", VectorToJson(plane.localCSys.yDir)},
                                                                {"zDir", VectorToJson(plane.localCSys.zDir)}}}});
  }
  return geometry;
}

bool LoadGeometryFromJson(const json &geometry,
                          std::vector<CRefEdge>& edges,
                          std::vector<CGeoDatumPlane>& datumPlanes,
                          std::string *errorMessage) {
  try {
    const auto edgesIt = geometry.find("edges");
    if (edgesIt == geometry.end() || !edgesIt->is_array()) {
      if (errorMessage) {
        *errorMessage = "geometry json missing edges array";
      }
      return false;
    }

    edges.clear();
    edges.reserve(edgesIt->size());
    for (const auto &edgeJson : *edgesIt) {
      CRefEdge edge;
      edge.parentFeatureID = edgeJson.value("parentFeatureID", "");
      edge.topologyIndex = edgeJson.value("topologyIndex", -1);
      if (edgeJson.contains("curveTypeValue")) {
        edge.curveType = static_cast<CGeoCurveType>(edgeJson.at("curveTypeValue").get<int>());
      } else if (edgeJson.contains("curveType")) {
        const auto &curveTypeNode = edgeJson.at("curveType");
        if (curveTypeNode.is_number_integer()) {
          edge.curveType = static_cast<CGeoCurveType>(curveTypeNode.get<int>());
        } else if (curveTypeNode.is_string()) {
          edge.curveType = CurveTypeFromString(curveTypeNode.get<std::string>());
        }
      }
      if (!edgeJson.contains("startPoint") || !edgeJson.contains("endPoint") ||
          !edgeJson.contains("midPoint") ||
          !TryReadPoint(edgeJson.at("startPoint"), edge.startPoint) ||
          !TryReadPoint(edgeJson.at("endPoint"), edge.endPoint) ||
          !TryReadPoint(edgeJson.at("midPoint"), edge.midPoint)) {
        if (errorMessage) {
          *errorMessage = "geometry json contains invalid edge point payload";
        }
        return false;
      }
      edges.push_back(std::move(edge));
    }

    const auto planesIt = geometry.find("datumPlanes");
    const auto flatPlanesIt = geometry.find("datum_planes");
    const json *planesJson = nullptr;
    if (planesIt != geometry.end() && planesIt->is_array()) {
      planesJson = &(*planesIt);
    } else if (flatPlanesIt != geometry.end() && flatPlanesIt->is_array()) {
      planesJson = &(*flatPlanesIt);
    }

    datumPlanes.clear();
    if (planesJson != nullptr) {
      datumPlanes.reserve(planesJson->size());
      for (const auto &planeJson : *planesJson) {
        CGeoDatumPlane plane;
        plane.targetFeatureID = planeJson.value("targetFeatureID", "");
        plane.type = planeJson.value("type", "Plane");
        const auto localCSysIt = planeJson.find("localCSys");
        if (localCSysIt != planeJson.end() && localCSysIt->is_object()) {
          if (!localCSysIt->contains("origin") || !localCSysIt->contains("xDir") ||
              !localCSysIt->contains("yDir") || !localCSysIt->contains("zDir") ||
              !TryReadPoint(localCSysIt->at("origin"), plane.localCSys.origin) ||
              !TryReadVector(localCSysIt->at("xDir"), plane.localCSys.xDir) ||
              !TryReadVector(localCSysIt->at("yDir"), plane.localCSys.yDir) ||
              !TryReadVector(localCSysIt->at("zDir"), plane.localCSys.zDir)) {
            if (errorMessage) {
              *errorMessage = "geometry json contains invalid datum plane csys payload";
            }
            return false;
          }
        } else {
          if (!planeJson.contains("origin") || !planeJson.contains("xDir") ||
              !planeJson.contains("yDir") || !planeJson.contains("normal") ||
              !TryReadPoint(planeJson.at("origin"), plane.localCSys.origin) ||
              !TryReadVector(planeJson.at("xDir"), plane.localCSys.xDir) ||
              !TryReadVector(planeJson.at("yDir"), plane.localCSys.yDir) ||
              !TryReadVector(planeJson.at("normal"), plane.localCSys.zDir)) {
            if (errorMessage) {
              *errorMessage = "geometry json contains invalid flat datum plane payload";
            }
            return false;
          }
        }
        datumPlanes.push_back(std::move(plane));
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

ComparisonResult CompareDetailedImpl(const std::vector<CRefEdge>& src_edges,
                                     const std::vector<CGeoDatumPlane>& src_datumPlanes,
                                     const std::vector<CRefEdge>& dst_edges,
                                     const std::vector<CGeoDatumPlane>& dst_datumPlanes,
                                     double tol,
                                     const std::vector<HalfStructurePointGroup>* global_src_half_groups,
                                     const std::vector<HalfStructurePointGroup>* global_dst_half_groups,
                                     const std::vector<HalfStructurePointGroup>* global_src_line_groups,
                                     const std::vector<HalfStructurePointGroup>* global_dst_line_groups) {
  ComparisonResult result;
  if (src_datumPlanes.size() != dst_datumPlanes.size()) {
    result.equivalent = false;
    result.diagnostics.push_back("DATUM plane count mismatch: SRC=" + std::to_string(src_datumPlanes.size()) +
                                 " DST=" + std::to_string(dst_datumPlanes.size()));
    return result;
  }

  std::vector<CRefEdge> src_open, dst_open;
  std::vector<NormalizedArc> src_arcs, dst_arcs;
  std::vector<CircleType> src_circles, dst_circles;
  std::vector<HalfStructurePointGroup> src_half_structure_groups, dst_half_structure_groups;
  int src_warn = 0, dst_warn = 0;
  ClassifyEdges(src_edges, src_open, src_arcs, src_circles, src_warn, tol);
  ClassifyEdges(dst_edges, dst_open, dst_arcs, dst_circles, dst_warn, tol);

  std::vector<CircleType> promoted_src, promoted_dst;
  src_arcs = MergeArcs(src_arcs, tol, promoted_src, &src_half_structure_groups);
  dst_arcs = MergeArcs(dst_arcs, tol, promoted_dst, &dst_half_structure_groups);
  for (auto &p : promoted_src) src_circles.push_back(p);
  for (auto &p : promoted_dst) dst_circles.push_back(p);

  // Simplify circles and arcs on both sides
  SimplifyCirclesAndArcs(src_circles, src_arcs, tol);
  SimplifyCirclesAndArcs(dst_circles, dst_arcs, tol);

  std::vector<HalfStructurePointGroup> src_line_half_groups, dst_line_half_groups;
  src_open = MergeCollinearLines(src_open, tol, src_line_half_groups);
  dst_open = MergeCollinearLines(dst_open, tol, dst_line_half_groups);

  const auto* src_line_groups_to_use = global_src_line_groups ? global_src_line_groups : &src_line_half_groups;
  const auto* dst_line_groups_to_use = global_dst_line_groups ? global_dst_line_groups : &dst_line_half_groups;

  // Filter by line groups first, then by arc groups
  FilterHalfStructureEdges(src_open, *src_line_groups_to_use, tol);
  FilterHalfStructureEdges(dst_open, *dst_line_groups_to_use, tol);

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

  std::vector<CRefEdge> src_unmatched_open;
  std::vector<CRefEdge> dst_unmatched_open;
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
  auto is_redundant_division_open = [&](const CRefEdge& edge) -> bool {
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

bool SaveModelGeometryToJson(const std::filesystem::path &filePath,
                             const std::vector<std::pair<std::string, json>>& featureList,
                             const std::string &length_unit,
                             std::string *errorMessage) {
  try {
    json featuresJson = json::array();
    for (const auto &[featureId, collectorJson] : featureList) {
      featuresJson.push_back(json{{"key", featureId}, {"value", collectorJson}});
    }
    json root{{"ModelGeometry", json{{"features", std::move(featuresJson)}}}};
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

bool LoadModelGeometryFromJson(const std::filesystem::path &filePath,
                               std::vector<std::pair<std::string, json>>& featureList,
                               std::string &file_unit,
                               std::string *errorMessage) {
  try {
    std::ifstream in(filePath);
    if (!in.is_open()) {
      if (errorMessage) *errorMessage = "Unable to open geometry json input: " + filePath.string();
      return false;
    }
    json root = json::parse(in);

    // Parse length_unit if present
    if (root.contains("length_unit")) {
      const auto &unitNode = root.at("length_unit");
      if (unitNode.is_string()) {
        file_unit = unitNode.get<std::string>();
      }
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
    featureList.clear();
    for (const auto &entry : *featuresIt) {
      if (!entry.is_object() || !entry.contains("key") || !entry.contains("value")) {
        if (errorMessage) *errorMessage = "geometry json contains malformed feature entry";
        return false;
      }
      featureList.emplace_back(entry.at("key").get<std::string>(), entry.at("value"));
    }
    return true;
  } catch (const std::exception &e) {
    if (errorMessage) *errorMessage = "Failed to parse geometry json: " + std::string(e.what());
    return false;
  }
}

} // namespace detail

} // namespace Geometry
} // namespace CADExchange
