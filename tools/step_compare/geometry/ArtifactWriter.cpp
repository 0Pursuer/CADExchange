// STL / BREP writers and the VTP visualisation writers.

#include "geometry/ArtifactWriter.h"
#include "domain/GeometryMath.h"
#include "geometry/ShapeAudit.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <Poly_Triangulation.hxx>
#include <StlAPI_Writer.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <system_error>
#include <vector>

namespace cadstep {
namespace detail {

int SurfaceTypeCode(const std::string &type) {
  if (type == "PLANE") return 0;
  if (type == "CYLINDER") return 1;
  if (type == "CONE") return 2;
  if (type == "SPHERE") return 3;
  if (type == "TORUS") return 4;
  if (type == "BSPLINE") return 5;
  return 6;
}

int CurveTypeCode(const std::string &type) {
  if (type == "LINE") return 0;
  if (type == "CIRCLE") return 1;
  if (type == "ELLIPSE") return 2;
  if (type == "BSPLINE") return 3;
  return 4;
}

int MatchStatusCode(MatchStatus status) {
  switch (status) {
    case MatchStatus::Matched: return 0;
    case MatchStatus::Unmatched: return 1;
    case MatchStatus::Ambiguous: return 2;
    default: return 3;
  }
}

// The CellData carries a match-status code per entity. Both writers derive it the
// same way: the entity id ends in the visual index, and entities the matcher
// never compared fall back to "unmatched" at the call site.
std::map<int, int> BuildMatchStatusByVisualIndex(const MatchCollection &matches,
                                                 EntitySide side) {
  std::map<int, int> statusByIndex;
  for (const auto &item : matches.items) {
    const std::string id =
        (side == EntitySide::Reference) ? item.referenceId : item.candidateId;
    if (id.empty()) {
      continue;
    }
    int index = 0;
    const std::size_t pos = id.find_last_of(':');
    if (pos != std::string::npos) {
      try {
        index = std::stoi(id.substr(pos + 1));
      } catch (...) {
      }
    }
    if (index > 0) {
      statusByIndex[index] = MatchStatusCode(item.status);
    }
  }
  return statusByIndex;
}

template <typename Info>
std::map<int, Info> IndexByVisualIndex(const std::vector<Info> &infos) {
  std::map<int, Info> byIndex;
  for (const auto &info : infos) {
    byIndex[info.visualIndex] = info;
  }
  return byIndex;
}

bool ExportFacesVtp(const TopoDS_Shape &solid,
                    const TopTools_IndexedMapOfShape &normalizedFaces,
                    const std::vector<NormalizedFaceInfo> &faceInfos,
                    const MatchCollection &faceMatches,
                    EntitySide side,
                    const std::filesystem::path &outputPath) {
  if (solid.IsNull() || normalizedFaces.IsEmpty()) return false;

  // Kept non-const deliberately: the CellData loops below index with
  // `count(i) ? map[i] : default`, which relies on operator[]; making these
  // const is a compile error rather than a silent behaviour change.
  std::map<int, int> matchStatusMap = BuildMatchStatusByVisualIndex(faceMatches, side);
  std::map<int, NormalizedFaceInfo> infoMap = IndexByVisualIndex(faceInfos);

  BRepMesh_IncrementalMesh mesh(solid, 0.05, Standard_False, 0.5, Standard_True);

  struct Point3D { double x, y, z; };
  struct Triangle { int p0, p1, p2; };

  std::vector<Point3D> points;
  std::vector<Triangle> polys;
  std::vector<int> cellEntityIdx;
  std::vector<int> cellMatchStatus;
  std::vector<int> cellGeomType;
  std::vector<int> cellSourceCount;
  std::vector<int> cellSideCode;

  int sideCodeVal = (side == EntitySide::Reference) ? 0 : 1;

  for (int i = 1; i <= normalizedFaces.Extent(); ++i) {
    const TopoDS_Face face = TopoDS::Face(normalizedFaces(i));
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull() || tri->NbTriangles() == 0) continue;

    const gp_Trsf trsf = loc.Transformation();
    const bool isReversed = (face.Orientation() == TopAbs_REVERSED);

    int startPtIdx = static_cast<int>(points.size());
    for (int p = 1; p <= tri->NbNodes(); ++p) {
      gp_Pnt pt = tri->Node(p).Transformed(trsf);
      points.push_back({pt.X(), pt.Y(), pt.Z()});
    }

    int matchStatus = matchStatusMap.count(i) ? matchStatusMap[i] : 1;
    int geomTypeCode = infoMap.count(i) ? SurfaceTypeCode(infoMap[i].surfaceType) : 6;
    int sourceCount = infoMap.count(i) ? infoMap[i].sourceCount : 1;

    for (int t = 1; t <= tri->NbTriangles(); ++t) {
      int n1, n2, n3;
      tri->Triangle(t).Get(n1, n2, n3);
      if (isReversed) {
        std::swap(n2, n3);
      }
      polys.push_back({startPtIdx + n1 - 1, startPtIdx + n2 - 1, startPtIdx + n3 - 1});
      cellEntityIdx.push_back(i);
      cellMatchStatus.push_back(matchStatus);
      cellGeomType.push_back(geomTypeCode);
      cellSourceCount.push_back(sourceCount);
      cellSideCode.push_back(sideCodeVal);
    }
  }

  if (points.empty()) return false;

  std::ofstream out(outputPath);
  if (!out.is_open()) return false;

  out << "<?xml version=\"1.0\"?>\n";
  out << "<VTKFile type=\"PolyData\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
  out << "  <PolyData>\n";
  out << "    <Piece NumberOfPoints=\"" << points.size() << "\" NumberOfVerts=\"0\" NumberOfLines=\"0\" NumberOfStrips=\"0\" NumberOfPolys=\"" << polys.size() << "\">\n";
  out << "      <Points>\n";
  out << "        <DataArray type=\"Float64\" Name=\"Points\" NumberOfComponents=\"3\" format=\"ascii\">\n";
  for (const auto &p : points) {
    out << p.x << " " << p.y << " " << p.z << "\n";
  }
  out << "        </DataArray>\n";
  out << "      </Points>\n";

  out << "      <Polys>\n";
  out << "        <DataArray type=\"Int64\" Name=\"connectivity\" format=\"ascii\">\n";
  for (const auto &tri : polys) {
    out << tri.p0 << " " << tri.p1 << " " << tri.p2 << "\n";
  }
  out << "        </DataArray>\n";
  out << "        <DataArray type=\"Int64\" Name=\"offsets\" format=\"ascii\">\n";
  for (size_t c = 1; c <= polys.size(); ++c) {
    out << (c * 3) << "\n";
  }
  out << "        </DataArray>\n";
  out << "      </Polys>\n";

  out << "      <CellData>\n";
  out << "        <DataArray type=\"Int32\" Name=\"entity_index\" format=\"ascii\">\n";
  for (int v : cellEntityIdx) out << v << "\n";
  out << "        </DataArray>\n";

  out << "        <DataArray type=\"Int32\" Name=\"match_status_code\" format=\"ascii\">\n";
  for (int v : cellMatchStatus) out << v << "\n";
  out << "        </DataArray>\n";

  out << "        <DataArray type=\"Int32\" Name=\"geometry_type_code\" format=\"ascii\">\n";
  for (int v : cellGeomType) out << v << "\n";
  out << "        </DataArray>\n";

  out << "        <DataArray type=\"Int32\" Name=\"source_count\" format=\"ascii\">\n";
  for (int v : cellSourceCount) out << v << "\n";
  out << "        </DataArray>\n";

  out << "        <DataArray type=\"Int32\" Name=\"side_code\" format=\"ascii\">\n";
  for (int v : cellSideCode) out << v << "\n";
  out << "        </DataArray>\n";

  out << "      </CellData>\n";
  out << "    </Piece>\n";
  out << "  </PolyData>\n";
  out << "</VTKFile>\n";

  return true;
}

bool ExportEdgesVtp(const TopoDS_Shape &solid,
                    const TopTools_IndexedMapOfShape &normalizedEdges,
                    const std::vector<NormalizedEdgeInfo> &edgeInfos,
                    const MatchCollection &edgeMatches,
                    EntitySide side,
                    const std::filesystem::path &outputPath) {
  if (solid.IsNull() || normalizedEdges.IsEmpty()) return false;

  std::map<int, int> matchStatusMap = BuildMatchStatusByVisualIndex(edgeMatches, side);
  std::map<int, NormalizedEdgeInfo> infoMap = IndexByVisualIndex(edgeInfos);

  struct Point3D { double x, y, z; };
  struct LineCell { std::vector<int> ptIndices; };

  std::vector<Point3D> points;
  std::vector<LineCell> lines;
  std::vector<int> cellEntityIdx;
  std::vector<int> cellMatchStatus;
  std::vector<int> cellGeomType;
  std::vector<int> cellSourceCount;
  std::vector<int> cellComparable;
  std::vector<int> cellRoleCode;

  for (int i = 1; i <= normalizedEdges.Extent(); ++i) {
    const TopoDS_Edge edge = TopoDS::Edge(normalizedEdges(i));
    if (edge.IsNull()) continue;

    BRepAdaptor_Curve curve(edge);
    GCPnts_QuasiUniformDeflection sampler(curve, 0.01);
    if (!sampler.IsDone() || sampler.NbPoints() < 2) continue;

    int startPtIdx = static_cast<int>(points.size());
    LineCell line;
    for (int p = 1; p <= sampler.NbPoints(); ++p) {
      gp_Pnt pt = sampler.Value(p);
      points.push_back({pt.X(), pt.Y(), pt.Z()});
      line.ptIndices.push_back(startPtIdx + p - 1);
    }
    lines.push_back(line);

    bool isComp = infoMap.count(i) ? infoMap[i].comparable : true;
    int matchStatus = 3; // 3 = NOT_COMPARED
    if (isComp) {
      matchStatus = matchStatusMap.count(i) ? matchStatusMap[i] : 1; // 1 = UNMATCHED
    } else {
      matchStatus = 3; // 3 = NOT_COMPARED
    }

    int geomTypeCode = infoMap.count(i) ? CurveTypeCode(infoMap[i].curveType) : 4;
    int sourceCount = infoMap.count(i) ? infoMap[i].sourceCount : 1;
    int comparable = isComp ? 1 : 0;

    int roleCode = 0;
    if (infoMap.count(i)) {
      switch (infoMap[i].comparisonRole) {
        case EdgeComparisonRole::Comparable: roleCode = 0; break;
        case EdgeComparisonRole::PeriodicSeam: roleCode = 1; break;
        case EdgeComparisonRole::Degenerated: roleCode = 2; break;
        default: roleCode = 3; break;
      }
    }

    cellEntityIdx.push_back(i);
    cellMatchStatus.push_back(matchStatus);
    cellGeomType.push_back(geomTypeCode);
    cellSourceCount.push_back(sourceCount);
    cellComparable.push_back(comparable);
    cellRoleCode.push_back(roleCode);
  }

  if (points.empty() || lines.empty()) return false;

  std::ofstream out(outputPath);
  if (!out.is_open()) return false;

  out << "<?xml version=\"1.0\"?>\n";
  out << "<VTKFile type=\"PolyData\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
  out << "  <PolyData>\n";
  out << "    <Piece NumberOfPoints=\"" << points.size() << "\" NumberOfVerts=\"0\" NumberOfLines=\"" << lines.size() << "\" NumberOfStrips=\"0\" NumberOfPolys=\"0\">\n";
  out << "      <Points>\n";
  out << "        <DataArray type=\"Float64\" Name=\"Points\" NumberOfComponents=\"3\" format=\"ascii\">\n";
  for (const auto &p : points) {
    out << p.x << " " << p.y << " " << p.z << "\n";
  }
  out << "        </DataArray>\n";
  out << "      </Points>\n";

  out << "      <Lines>\n";
  out << "        <DataArray type=\"Int64\" Name=\"connectivity\" format=\"ascii\">\n";
  for (const auto &l : lines) {
    for (int idx : l.ptIndices) {
      out << idx << " ";
    }
    out << "\n";
  }
  out << "        </DataArray>\n";
  out << "        <DataArray type=\"Int64\" Name=\"offsets\" format=\"ascii\">\n";
  size_t offset = 0;
  for (const auto &l : lines) {
    offset += l.ptIndices.size();
    out << offset << "\n";
  }
  out << "        </DataArray>\n";
  out << "      </Lines>\n";

  out << "      <CellData>\n";
  out << "        <DataArray type=\"Int32\" Name=\"entity_index\" format=\"ascii\">\n";
  for (int v : cellEntityIdx) out << v << "\n";
  out << "        </DataArray>\n";

  out << "        <DataArray type=\"Int32\" Name=\"match_status_code\" format=\"ascii\">\n";
  for (int v : cellMatchStatus) out << v << "\n";
  out << "        </DataArray>\n";

  out << "        <DataArray type=\"Int32\" Name=\"comparison_role_code\" format=\"ascii\">\n";
  for (int v : cellRoleCode) out << v << "\n";
  out << "        </DataArray>\n";

  out << "        <DataArray type=\"Int32\" Name=\"geometry_type_code\" format=\"ascii\">\n";
  for (int v : cellGeomType) out << v << "\n";
  out << "        </DataArray>\n";

  out << "        <DataArray type=\"Int32\" Name=\"source_count\" format=\"ascii\">\n";
  for (int v : cellSourceCount) out << v << "\n";
  out << "        </DataArray>\n";

  out << "        <DataArray type=\"Int32\" Name=\"comparable\" format=\"ascii\">\n";
  for (int v : cellComparable) out << v << "\n";
  out << "        </DataArray>\n";

  out << "      </CellData>\n";
  out << "    </Piece>\n";
  out << "  </PolyData>\n";
  out << "</VTKFile>\n";

  return true;
}

void RemoveOldArtifacts(const std::filesystem::path &outputDirectory) {
  std::error_code ec;
  std::filesystem::remove(outputDirectory / "reference_base.stl", ec);
  std::filesystem::remove(outputDirectory / "candidate_base.stl", ec);
  std::filesystem::remove(outputDirectory / "missing_material.stl", ec);
  std::filesystem::remove(outputDirectory / "added_material.stl", ec);
  std::filesystem::remove(outputDirectory / "reference_original.brep", ec);
  std::filesystem::remove(outputDirectory / "candidate_original.brep", ec);
  std::filesystem::remove(outputDirectory / "reference_normalized.brep", ec);
  std::filesystem::remove(outputDirectory / "candidate_normalized.brep", ec);
  std::filesystem::remove(outputDirectory / "result.json", ec);
  std::filesystem::remove(outputDirectory / "result.json.tmp", ec);
  std::filesystem::remove_all(outputDirectory / "visualization", ec);
}

bool ExportShapeStl(const TopoDS_Shape &shape, const std::filesystem::path &stlPath) {
  if (shape.IsNull()) {
    return false;
  }
  try {
    const double diagScale = BoundsDiagonal(ComputeBounds(shape));
    const double deflection = std::max(0.001, 0.001 * diagScale);
    BRepMesh_IncrementalMesh mesh(shape, deflection);
    mesh.Perform();

    StlAPI_Writer writer;
    writer.ASCIIMode() = false;
    return writer.Write(shape, stlPath.string().c_str()) != 0;
  } catch (...) {
    return false;
  }
}

bool ExportShapeBrep(const TopoDS_Shape &shape, const std::filesystem::path &brepPath) {
  if (shape.IsNull()) {
    return false;
  }
  try {
    return BRepTools::Write(shape, brepPath.string().c_str()) != 0;
  } catch (...) {
    return false;
  }
}

} // namespace detail
} // namespace cadstep
