#include "StepCompare.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRepTools_History.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_PrintCount.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <StlAPI_Writer.hxx>
#include <Standard_Failure.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <gp_Pnt.hxx>
#include <json/single_include/nlohmann/json.hpp>

#include <GCPnts_QuasiUniformDeflection.hxx>
#include <Poly_Triangulation.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <system_error>
#include <tuple>

namespace cadstep {

using json = nlohmann::ordered_json;

int FindShapeIndex(const TopTools_IndexedMapOfShape &shapeMap, const TopoDS_Shape &target) {
  for (int i = 1; i <= shapeMap.Extent(); ++i) {
    if (shapeMap(i).IsSame(target)) {
      return i;
    }
  }
  return 0;
}

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

bool ExportFacesVtp(const TopoDS_Shape &solid,
                    const TopTools_IndexedMapOfShape &normalizedFaces,
                    const std::vector<NormalizedFaceInfo> &faceInfos,
                    const MatchCollection &faceMatches,
                    EntitySide side,
                    const std::filesystem::path &outputPath) {
  if (solid.IsNull() || normalizedFaces.IsEmpty()) return false;

  std::map<int, int> matchStatusMap;
  for (const auto &item : faceMatches.items) {
    std::string id = (side == EntitySide::Reference) ? item.referenceId : item.candidateId;
    if (!id.empty()) {
      int idx = 0;
      size_t pos = id.find_last_of(':');
      if (pos != std::string::npos) {
        try {
          idx = std::stoi(id.substr(pos + 1));
        } catch (...) {}
      }
      if (idx > 0) {
        matchStatusMap[idx] = MatchStatusCode(item.status);
      }
    }
  }

  std::map<int, NormalizedFaceInfo> infoMap;
  for (const auto &info : faceInfos) {
    infoMap[info.visualIndex] = info;
  }

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

  std::map<int, int> matchStatusMap;
  for (const auto &item : edgeMatches.items) {
    std::string id = (side == EntitySide::Reference) ? item.referenceId : item.candidateId;
    if (!id.empty()) {
      int idx = 0;
      size_t pos = id.find_last_of(':');
      if (pos != std::string::npos) {
        try {
          idx = std::stoi(id.substr(pos + 1));
        } catch (...) {}
      }
      if (idx > 0) {
        matchStatusMap[idx] = MatchStatusCode(item.status);
      }
    }
  }

  std::map<int, NormalizedEdgeInfo> infoMap;
  for (const auto &info : edgeInfos) {
    infoMap[info.visualIndex] = info;
  }

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

const char *ToString(EdgeComparisonRole role) {
  switch (role) {
    case EdgeComparisonRole::Comparable: return "COMPARABLE";
    case EdgeComparisonRole::PeriodicSeam: return "PERIODIC_SEAM";
    case EdgeComparisonRole::Degenerated: return "DEGENERATED";
    case EdgeComparisonRole::Unsupported: return "UNSUPPORTED";
    default: return "UNKNOWN";
  }
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

using json = nlohmann::ordered_json;

namespace {

enum class LoadClass {
  Ready,
  Invalid,
  Unsupported,
  InternalError,
};

struct LoadedSolidItem {
  int index = 0;
  std::string id;
  InputAudit audit;
  TopoDS_Solid solid;
};

struct LoadedStepModel {
  LoadClass classification = LoadClass::InternalError;
  InputAudit compositeAudit;
  std::vector<LoadedSolidItem> solids;
  std::string reason;
};

struct FaceDescriptor {
  std::string entityId;
  int visualIndex = 0;
  TopoDS_Face face;
  std::string typeName;
  double areaMm2 = 0.0;
  Point3 centroidMm;
  Bounds3 boundsMm;
};

struct EdgeDescriptor {
  std::string entityId;
  int visualIndex = 0;
  TopoDS_Edge edge;
  std::string typeName;
  double lengthMm = 0.0;
  Point3 centroidMm;
  Bounds3 boundsMm;
};

struct OriginalTopologyIndex {
  TopTools_IndexedMapOfShape faces;
  TopTools_IndexedMapOfShape edges;
  std::vector<std::string> faceIds;
  std::vector<std::string> edgeIds;
};

struct NormalizedSolidInternal {
  TopoDS_Solid solid;
  NormalizationAudit audit;
  TopTools_IndexedMapOfShape normalizedFaces;
  TopTools_IndexedMapOfShape normalizedEdges;
};

struct BooleanDifferenceResult {
  DifferenceAudit audit;
  TopoDS_Shape shape;
};

std::string PathText(const std::filesystem::path &path) {
#ifdef _WIN32
  const auto utf8 = path.u8string();
  return std::string(utf8.begin(), utf8.end());
#else
  return path.string();
#endif
}

const char *ReadStatusText(IFSelect_ReturnStatus status) {
  switch (status) {
  case IFSelect_RetVoid:
    return "void";
  case IFSelect_RetDone:
    return "done";
  case IFSelect_RetError:
    return "error";
  case IFSelect_RetFail:
    return "fail";
  case IFSelect_RetStop:
    return "stop";
  }
  return "unknown";
}

int CountSubShapes(const TopoDS_Shape &shape, TopAbs_ShapeEnum type) {
  int count = 0;
  for (TopExp_Explorer explorer(shape, type); explorer.More();
       explorer.Next()) {
    ++count;
  }
  return count;
}

int CountUniqueSubShapes(const TopoDS_Shape &shape, TopAbs_ShapeEnum type) {
  TopTools_IndexedMapOfShape shapeMap;
  TopExp::MapShapes(shape, type, shapeMap);
  return shapeMap.Extent();
}

double VectorLength(const Point3 &p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

double PointDistance(const Point3 &p1, const Point3 &p2) {
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double BoundsDiagonal(const Bounds3 &bounds) {
  if (bounds.isVoid) {
    return 0.0;
  }
  return PointDistance(bounds.minimum, bounds.maximum);
}

double ComputeBoundsDifference(const Bounds3 &b1, const Bounds3 &b2) {
  if (b1.isVoid && b2.isVoid) {
    return 0.0;
  }
  if (b1.isVoid || b2.isVoid) {
    return std::numeric_limits<double>::infinity();
  }
  const double minDiff = PointDistance(b1.minimum, b2.minimum);
  const double maxDiff = PointDistance(b1.maximum, b2.maximum);
  return std::max(minDiff, maxDiff);
}

Bounds3 ComputeBounds(const TopoDS_Shape &shape) {
  Bounds3 bounds;
  Bnd_Box box;
  BRepBndLib::AddOptimal(shape, box);
  if (box.IsVoid()) {
    BRepBndLib::Add(shape, box);
  }
  if (box.IsVoid()) {
    bounds.isVoid = true;
    return bounds;
  }

  double xmin = 0.0, ymin = 0.0, zmin = 0.0;
  double xmax = 0.0, ymax = 0.0, zmax = 0.0;
  box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

  bounds.isVoid = false;
  bounds.minimum = {xmin, ymin, zmin};
  bounds.maximum = {xmax, ymax, zmax};
  return bounds;
}

std::string SurfaceTypeName(GeomAbs_SurfaceType type) {
  switch (type) {
  case GeomAbs_Plane:
    return "PLANE";
  case GeomAbs_Cylinder:
    return "CYLINDER";
  case GeomAbs_Cone:
    return "CONE";
  case GeomAbs_Sphere:
    return "SPHERE";
  case GeomAbs_Torus:
    return "TORUS";
  case GeomAbs_BezierSurface:
    return "BEZIER";
  case GeomAbs_BSplineSurface:
    return "BSPLINE";
  case GeomAbs_SurfaceOfRevolution:
    return "REVOLUTION";
  case GeomAbs_SurfaceOfExtrusion:
    return "EXTRUSION";
  case GeomAbs_OffsetSurface:
    return "OFFSET";
  case GeomAbs_OtherSurface:
    return "OTHER";
  }
  return "UNKNOWN";
}

std::string CurveTypeName(GeomAbs_CurveType type) {
  switch (type) {
  case GeomAbs_Line:
    return "LINE";
  case GeomAbs_Circle:
    return "CIRCLE";
  case GeomAbs_Ellipse:
    return "ELLIPSE";
  case GeomAbs_Hyperbola:
    return "HYPERBOLA";
  case GeomAbs_Parabola:
    return "PARABOLA";
  case GeomAbs_BezierCurve:
    return "BEZIER";
  case GeomAbs_BSplineCurve:
    return "BSPLINE";
  case GeomAbs_OffsetCurve:
    return "OFFSET";
  case GeomAbs_OtherCurve:
    return "OTHER";
  }
  return "UNKNOWN";
}

template <typename Clock = std::chrono::high_resolution_clock>
double ElapsedMs(const std::chrono::time_point<Clock> &start) {
  const auto finish = Clock::now();
  return std::chrono::duration<double, std::milli>(finish - start).count();
}

std::vector<TypeStatistics> SummarizeMap(const std::map<std::string, std::pair<int, double>> &countMap) {
  std::vector<TypeStatistics> result;
  result.reserve(countMap.size());
  for (const auto &pair : countMap) {
    TypeStatistics stat;
    stat.type = pair.first;
    stat.count = pair.second.first;
    stat.totalMeasure = pair.second.second;
    result.push_back(stat);
  }
  return result;
}

OriginalTopologyIndex BuildOriginalTopologyIndex(const TopoDS_Solid &solid, EntitySide side) {
  OriginalTopologyIndex index;
  TopExp::MapShapes(solid, TopAbs_FACE, index.faces);
  TopExp::MapShapes(solid, TopAbs_EDGE, index.edges);

  index.faceIds.reserve(index.faces.Extent());
  for (int i = 1; i <= index.faces.Extent(); ++i) {
    index.faceIds.push_back(MakeEntityId(side, EntityKind::OriginalFace, i));
  }

  index.edgeIds.reserve(index.edges.Extent());
  for (int i = 1; i <= index.edges.Extent(); ++i) {
    index.edgeIds.push_back(MakeEntityId(side, EntityKind::OriginalEdge, i));
  }
  return index;
}

LoadedStepModel LoadStepModel(const std::filesystem::path &stepPath, const CompareConfig &config, EntitySide side) {
  LoadedStepModel result;
  result.compositeAudit.path = stepPath.string();

  if (!std::filesystem::exists(stepPath)) {
    result.classification = LoadClass::Invalid;
    result.reason = "file does not exist: " + stepPath.string();
    return result;
  }

  STEPControl_Reader reader;
  const IFSelect_ReturnStatus readStatus = reader.ReadFile(stepPath.string().c_str());

  std::ostringstream loadDiag;
  loadDiag << "ReadFile status=" << ReadStatusText(readStatus);
  result.compositeAudit.loadDiagnostics = loadDiag.str();

  if (readStatus != IFSelect_RetDone) {
    result.classification = LoadClass::Invalid;
    result.reason = "failed to parse STEP file (read_status=" + std::string(ReadStatusText(readStatus)) + ")";
    return result;
  }

  TColStd_SequenceOfAsciiString lengthUnits;
  TColStd_SequenceOfAsciiString planeAngleUnits;
  TColStd_SequenceOfAsciiString solidAngleUnits;
  reader.FileUnits(lengthUnits, planeAngleUnits, solidAngleUnits);
  for (int i = 1; i <= lengthUnits.Length(); ++i) {
    result.compositeAudit.fileLengthUnits.push_back(lengthUnits.Value(i).ToCString());
  }

  const int rootsCount = reader.TransferRoots();
  std::ostringstream transferDiag;
  transferDiag << "transferred_roots=" << rootsCount;
  result.compositeAudit.transferDiagnostics = transferDiag.str();

  const TopoDS_Shape compositeShape = reader.OneShape();
  if (compositeShape.IsNull()) {
    result.classification = LoadClass::Invalid;
    result.reason = "STEP file contains no geometry";
    return result;
  }

  result.compositeAudit.solidCount = CountUniqueSubShapes(compositeShape, TopAbs_SOLID);
  result.compositeAudit.shellCount = CountUniqueSubShapes(compositeShape, TopAbs_SHELL);
  result.compositeAudit.faceCount = CountUniqueSubShapes(compositeShape, TopAbs_FACE);
  result.compositeAudit.edgeCount = CountUniqueSubShapes(compositeShape, TopAbs_EDGE);

  if (result.compositeAudit.solidCount == 0) {
    result.classification = LoadClass::Unsupported;
    result.reason = "STEP file contains no 3D solids (shell_count=" + std::to_string(result.compositeAudit.shellCount) +
                    ", face_count=" + std::to_string(result.compositeAudit.faceCount) + ")";
    return result;
  }

  if (result.compositeAudit.solidCount > 1 && !config.allowMultipleSolids && config.multiSolidPolicy == MultiSolidPolicy::Strict) {
    result.classification = LoadClass::Unsupported;
    result.reason = "multiple 3D solids are not supported (solid_count=" + std::to_string(result.compositeAudit.solidCount) + ")";
    return result;
  }

  TopExp_Explorer solidExplorer(compositeShape, TopAbs_SOLID);
  int solidIndex = 0;
  double totalVolume = 0.0;
  double totalSurfaceArea = 0.0;
  gp_Vec weightedCentroidSum(0.0, 0.0, 0.0);
  Bounds3 compositeBounds;

  while (solidExplorer.More()) {
    TopoDS_Solid s = TopoDS::Solid(solidExplorer.Current());
    solidExplorer.Next();
    if (s.IsNull()) continue;

    LoadedSolidItem item;
    item.index = solidIndex;
    item.id = (side == EntitySide::Reference ? "ref_solid_" : "cand_solid_") + std::to_string(solidIndex);
    item.solid = s;

    item.audit.path = stepPath.string();
    item.audit.fileLengthUnits = result.compositeAudit.fileLengthUnits;
    item.audit.loadDiagnostics = result.compositeAudit.loadDiagnostics;
    item.audit.transferDiagnostics = result.compositeAudit.transferDiagnostics;
    item.audit.solidCount = 1;
    item.audit.shellCount = CountUniqueSubShapes(s, TopAbs_SHELL);
    item.audit.faceCount = CountUniqueSubShapes(s, TopAbs_FACE);
    item.audit.edgeCount = CountUniqueSubShapes(s, TopAbs_EDGE);

    BRepCheck_Analyzer analyzer(s);
    item.audit.brepValid = analyzer.IsValid() != 0;
    item.audit.closed = (s.Closed() != 0) || (CountSubShapes(s, TopAbs_SHELL) > 0);

    if (!item.audit.brepValid) {
      result.classification = LoadClass::Invalid;
      result.reason = "solid fail BRepCheck validation";
      return result;
    }

    GProp_GProps systemProps;
    BRepGProp::VolumeProperties(s, systemProps);
    item.audit.signedVolumeMm3 = systemProps.Mass();

    GProp_GProps surfaceProps;
    BRepGProp::SurfaceProperties(s, surfaceProps);
    item.audit.surfaceAreaMm2 = surfaceProps.Mass();

    const gp_Pnt centroidPnt = systemProps.CentreOfMass();
    item.audit.centroidMm = {centroidPnt.X(), centroidPnt.Y(), centroidPnt.Z()};
    item.audit.boundsMm = ComputeBounds(s);

    totalVolume += item.audit.signedVolumeMm3;
    totalSurfaceArea += item.audit.surfaceAreaMm2;
    weightedCentroidSum += gp_Vec(centroidPnt.X(), centroidPnt.Y(), centroidPnt.Z()) * item.audit.signedVolumeMm3;

    if (compositeBounds.isVoid) {
      compositeBounds = item.audit.boundsMm;
    } else {
      compositeBounds.minimum.x = std::min(compositeBounds.minimum.x, item.audit.boundsMm.minimum.x);
      compositeBounds.minimum.y = std::min(compositeBounds.minimum.y, item.audit.boundsMm.minimum.y);
      compositeBounds.minimum.z = std::min(compositeBounds.minimum.z, item.audit.boundsMm.minimum.z);
      compositeBounds.maximum.x = std::max(compositeBounds.maximum.x, item.audit.boundsMm.maximum.x);
      compositeBounds.maximum.y = std::max(compositeBounds.maximum.y, item.audit.boundsMm.maximum.y);
      compositeBounds.maximum.z = std::max(compositeBounds.maximum.z, item.audit.boundsMm.maximum.z);
    }

    result.solids.push_back(item);
    solidIndex++;
  }

  if (result.solids.empty()) {
    result.classification = LoadClass::Invalid;
    result.reason = "failed to extract solid shape";
    return result;
  }

  TopTools_IndexedMapOfShape solidFacesMap;
  TopTools_IndexedMapOfShape solidEdgesMap;
  for (const auto &item : result.solids) {
    TopExp::MapShapes(item.solid, TopAbs_FACE, solidFacesMap);
    TopExp::MapShapes(item.solid, TopAbs_EDGE, solidEdgesMap);
  }

  if (result.compositeAudit.faceCount > solidFacesMap.Extent() ||
      result.compositeAudit.edgeCount > solidEdgesMap.Extent()) {
    result.classification = LoadClass::Unsupported;
    result.reason = "STEP file contains non-solid topological entities (free curves or surface faces)";
    return result;
  }

  result.compositeAudit.signedVolumeMm3 = totalVolume;
  result.compositeAudit.surfaceAreaMm2 = totalSurfaceArea;
  if (totalVolume > 0.0) {
    result.compositeAudit.centroidMm = {weightedCentroidSum.X() / totalVolume, weightedCentroidSum.Y() / totalVolume, weightedCentroidSum.Z() / totalVolume};
  } else {
    result.compositeAudit.centroidMm = result.solids[0].audit.centroidMm;
  }
  result.compositeAudit.boundsMm = compositeBounds;
  result.compositeAudit.brepValid = true;
  result.compositeAudit.closed = true;

  result.classification = LoadClass::Ready;
  return result;
}

std::vector<EdgeDescriptor> CollectEdgeDescriptors(const TopoDS_Shape &shape,
                                                   const TopTools_IndexedMapOfShape &normalizedEdges,
                                                   EntitySide side);

struct EdgeComparisonClassification {
  EdgeComparisonRole role = EdgeComparisonRole::Unsupported;
  bool comparable = false;
  std::string exclusionReason;
};

EdgeComparisonClassification ClassifyNormalizedEdge(
    const TopoDS_Edge &edge,
    const TopTools_IndexedDataMapOfShapeListOfShape &edgeFacesMap) {
  if (BRep_Tool::Degenerated(edge)) {
    return {EdgeComparisonRole::Degenerated, false, "DEGENERATED"};
  }

  if (edgeFacesMap.Contains(edge)) {
    const auto &faces = edgeFacesMap.FindFromKey(edge);
    for (const TopoDS_Shape &s : faces) {
      const TopoDS_Face face = TopoDS::Face(s);
      if (BRepTools::IsReallyClosed(edge, face)) {
        return {EdgeComparisonRole::PeriodicSeam, false, "PERIODIC_SEAM"};
      }
    }
  }

  return {EdgeComparisonRole::Comparable, true, ""};
}

std::vector<EdgeDescriptor> CollectEdgeDescriptors(const TopTools_IndexedMapOfShape &normalizedEdges,
                                                   const std::vector<NormalizedEdgeInfo> &edgeInfos,
                                                   EntitySide side);

NormalizedSolidInternal NormalizeSameDomain(const TopoDS_Solid &input,
                                             const OriginalTopologyIndex &originalIndex,
                                             EntitySide side,
                                             const CompareConfig &config) {
  NormalizedSolidInternal result;
  result.audit.enabled = config.enableSameDomainNormalization;
  result.audit.faceCountBefore = CountUniqueSubShapes(input, TopAbs_FACE);
  result.audit.edgeCountBefore = CountUniqueSubShapes(input, TopAbs_EDGE);
  result.audit.comparableEdgeCountBefore = result.audit.edgeCountBefore;

  GProp_GProps inputProps;
  BRepGProp::VolumeProperties(input, inputProps);
  result.audit.volumeBeforeMm3 = inputProps.Mass();

  if (!config.enableSameDomainNormalization) {
    result.solid = input;
    result.audit.succeeded = true;
    result.audit.usedNormalizedShape = false;
    result.audit.faceCountAfter = result.audit.faceCountBefore;
    result.audit.edgeCountAfter = result.audit.edgeCountBefore;
    result.audit.volumeAfterMm3 = result.audit.volumeBeforeMm3;
    result.audit.faceMappingComplete = true;
    result.audit.edgeMappingComplete = true;
    result.audit.mappingComplete = true;

    TopExp::MapShapes(result.solid, TopAbs_FACE, result.normalizedFaces);
    TopExp::MapShapes(result.solid, TopAbs_EDGE, result.normalizedEdges);

    TopTools_IndexedDataMapOfShapeListOfShape edgeFacesMap;
    TopExp::MapShapesAndAncestors(result.solid, TopAbs_EDGE, TopAbs_FACE, edgeFacesMap);

    int compIdx = 1;
    for (int i = 1; i <= result.normalizedEdges.Extent(); ++i) {
      const TopoDS_Edge e = TopoDS::Edge(result.normalizedEdges(i));
      NormalizedEdgeInfo info;
      info.id = MakeEntityId(side, EntityKind::NormalizedEdge, i);
      info.visualIndex = i;

      BRepAdaptor_Curve curve(e);
      info.curveType = CurveTypeName(curve.GetType());

      GProp_GProps edgeProps;
      BRepGProp::LinearProperties(e, edgeProps);
      info.lengthMm = edgeProps.Mass();

      const gp_Pnt edgeCentroid = edgeProps.CentreOfMass();
      info.centroidMm = {edgeCentroid.X(), edgeCentroid.Y(), edgeCentroid.Z()};
      info.boundsMm = ComputeBounds(e);
      info.closed = (BRep_Tool::IsClosed(e) != 0);

      const auto classification = ClassifyNormalizedEdge(e, edgeFacesMap);
      info.comparisonRole = classification.role;
      info.comparable = classification.comparable;
      info.exclusionReason = classification.exclusionReason;
      if (info.comparable) {
        info.comparableIndex = compIdx++;
      } else {
        info.comparableIndex = 0;
      }
      result.audit.edges.push_back(info);
    }
    result.audit.comparableEdgeCountBefore = compIdx - 1;
    result.audit.comparableEdgeCountAfter = compIdx - 1;
    return result;
  }

  const auto normStart = std::chrono::high_resolution_clock::now();
  Handle(BRepTools_History) history;
  try {
    ShapeUpgrade_UnifySameDomain unify(input, Standard_True, Standard_True, Standard_False);
    unify.SetSafeInputMode(Standard_True);
    unify.AllowInternalEdges(Standard_False);
    unify.SetLinearTolerance(config.normalizationLinearToleranceMm);
    unify.SetAngularTolerance(config.normalizationAngularToleranceRad);
    unify.Build();
    history = unify.History();

    const TopoDS_Shape unifiedShape = unify.Shape();
    if (!unifiedShape.IsNull() && unifiedShape.ShapeType() == TopAbs_SOLID) {
      result.solid = TopoDS::Solid(unifiedShape);
      result.audit.succeeded = true;
    } else if (!unifiedShape.IsNull() && CountSubShapes(unifiedShape, TopAbs_SOLID) == 1) {
      TopExp_Explorer expl(unifiedShape, TopAbs_SOLID);
      result.solid = TopoDS::Solid(expl.Current());
      result.audit.succeeded = true;
    } else {
      result.solid = input;
      result.audit.succeeded = false;
      result.audit.warning = "unify_same_domain produced non-solid shape; fallback to original";
    }
  } catch (const Standard_Failure &e) {
    result.solid = input;
    result.audit.succeeded = false;
    result.audit.warning = std::string("unify_same_domain exception: ") + e.GetMessageString();
  } catch (...) {
    result.solid = input;
    result.audit.succeeded = false;
    result.audit.warning = "unify_same_domain unknown exception; fallback to original";
  }

  result.audit.elapsedMs = ElapsedMs(normStart);
  if (!result.solid.IsNull()) {
    GProp_GProps outputProps;
    BRepGProp::VolumeProperties(result.solid, outputProps);
    result.audit.volumeAfterMm3 = outputProps.Mass();

    const double denom = std::abs(result.audit.volumeBeforeMm3);
    result.audit.relativeVolumeDrift =
        denom > 0.0 ? std::abs(result.audit.volumeAfterMm3 - result.audit.volumeBeforeMm3) / denom : 0.0;
  }

  const bool volumeSafe = result.audit.relativeVolumeDrift <= config.relativeVolumeTolerance;
  if (!volumeSafe && result.audit.succeeded) {
    result.audit.warning = "normalization volume drift exceeds safety threshold; fallback to original";
    result.solid = input;
    result.audit.succeeded = false;
  }

  result.audit.usedNormalizedShape = result.audit.succeeded;
  result.audit.faceCountAfter = CountUniqueSubShapes(result.solid, TopAbs_FACE);
  result.audit.edgeCountAfter = CountUniqueSubShapes(result.solid, TopAbs_EDGE);

  TopExp::MapShapes(result.solid, TopAbs_FACE, result.normalizedFaces);
  TopExp::MapShapes(result.solid, TopAbs_EDGE, result.normalizedEdges);

  // Build faces detail list
  std::map<int, NormalizedFaceInfo> normFaceMap;
  for (int i = 1; i <= result.normalizedFaces.Extent(); ++i) {
    const TopoDS_Face f = TopoDS::Face(result.normalizedFaces(i));
    NormalizedFaceInfo info;
    info.id = MakeEntityId(side, EntityKind::NormalizedFace, i);
    info.visualIndex = i;

    BRepAdaptor_Surface surface(f);
    info.surfaceType = SurfaceTypeName(surface.GetType());

    GProp_GProps faceProps;
    BRepGProp::SurfaceProperties(f, faceProps);
    info.areaMm2 = faceProps.Mass();

    const gp_Pnt faceCentroid = faceProps.CentreOfMass();
    info.centroidMm = {faceCentroid.X(), faceCentroid.Y(), faceCentroid.Z()};
    info.boundsMm = ComputeBounds(f);

    if (surface.GetType() == GeomAbs_Cylinder) {
      info.radiusMm = surface.Cylinder().Radius();
      const gp_Pnt loc = surface.Cylinder().Location();
      const gp_Dir dir = surface.Cylinder().Axis().Direction();
      info.axisOriginMm = Point3{loc.X(), loc.Y(), loc.Z()};
      info.axisDirection = Point3{dir.X(), dir.Y(), dir.Z()};
    }

    TopTools_IndexedMapOfShape faceEdges;
    TopExp::MapShapes(f, TopAbs_EDGE, faceEdges);
    for (int j = 1; j <= faceEdges.Extent(); ++j) {
      const int edgeIdx = FindShapeIndex(result.normalizedEdges, faceEdges(j));
      if (edgeIdx > 0) {
        info.boundaryEdgeIds.push_back(MakeEntityId(side, EntityKind::NormalizedEdge, edgeIdx));
      }
    }

    normFaceMap[i] = info;
  }

  // Build edges detail list
  std::map<int, NormalizedEdgeInfo> normEdgeMap;
  for (int i = 1; i <= result.normalizedEdges.Extent(); ++i) {
    const TopoDS_Edge e = TopoDS::Edge(result.normalizedEdges(i));
    NormalizedEdgeInfo info;
    info.id = MakeEntityId(side, EntityKind::NormalizedEdge, i);
    info.visualIndex = i;

    BRepAdaptor_Curve curve(e);
    info.curveType = CurveTypeName(curve.GetType());

    GProp_GProps edgeProps;
    BRepGProp::LinearProperties(e, edgeProps);
    info.lengthMm = edgeProps.Mass();

    const gp_Pnt edgeCentroid = edgeProps.CentreOfMass();
    info.centroidMm = {edgeCentroid.X(), edgeCentroid.Y(), edgeCentroid.Z()};
    info.boundsMm = ComputeBounds(e);
    info.closed = (BRep_Tool::IsClosed(e) != 0);

    normEdgeMap[i] = info;
  }

  // Map original topology to normalized topology using history
  bool faceMappingComplete = true;
  for (int i = 1; i <= originalIndex.faces.Extent(); ++i) {
    const TopoDS_Face origF = TopoDS::Face(originalIndex.faces(i));
    const std::string origId = originalIndex.faceIds[i - 1];

    bool mapped = false;
    if (!history.IsNull()) {
      const TopTools_ListOfShape &modified = history->Modified(origF);
      for (const TopoDS_Shape &s : modified) {
        int idx = FindShapeIndex(result.normalizedFaces, s);
        if (idx > 0) {
          normFaceMap[idx].sourceFaceIds.push_back(origId);
          mapped = true;
        }
      }
      const TopTools_ListOfShape &generated = history->Generated(origF);
      for (const TopoDS_Shape &s : generated) {
        int idx = FindShapeIndex(result.normalizedFaces, s);
        if (idx > 0) {
          normFaceMap[idx].sourceFaceIds.push_back(origId);
          mapped = true;
        }
      }
    }

    if (!mapped) {
      int sameIdx = FindShapeIndex(result.normalizedFaces, origF);
      if (sameIdx > 0) {
        normFaceMap[sameIdx].sourceFaceIds.push_back(origId);
        mapped = true;
      }
    }

    if (!mapped) {
      faceMappingComplete = false;
    }
  }

  for (int j = 1; j <= result.normalizedFaces.Extent(); ++j) {
    auto &info = normFaceMap[j];
    std::sort(info.sourceFaceIds.begin(), info.sourceFaceIds.end());
    info.sourceFaceIds.erase(std::unique(info.sourceFaceIds.begin(), info.sourceFaceIds.end()), info.sourceFaceIds.end());
    info.sourceCount = static_cast<int>(info.sourceFaceIds.size());
    info.merged = info.sourceCount > 1;
    result.audit.faces.push_back(info);
  }

  TopTools_IndexedDataMapOfShapeListOfShape edgeFacesMap;
  TopExp::MapShapesAndAncestors(result.solid, TopAbs_EDGE, TopAbs_FACE, edgeFacesMap);

  bool edgeMappingComplete = true;
  for (int i = 1; i <= originalIndex.edges.Extent(); ++i) {
    const TopoDS_Edge origE = TopoDS::Edge(originalIndex.edges(i));
    const std::string origId = originalIndex.edgeIds[i - 1];

    bool mapped = false;
    if (!history.IsNull()) {
      const TopTools_ListOfShape &modified = history->Modified(origE);
      for (const TopoDS_Shape &s : modified) {
        int idx = FindShapeIndex(result.normalizedEdges, s);
        if (idx > 0) {
          normEdgeMap[idx].sourceEdgeIds.push_back(origId);
          mapped = true;
        }
      }
      const TopTools_ListOfShape &generated = history->Generated(origE);
      for (const TopoDS_Shape &s : generated) {
        int idx = FindShapeIndex(result.normalizedEdges, s);
        if (idx > 0) {
          normEdgeMap[idx].sourceEdgeIds.push_back(origId);
          mapped = true;
        }
      }
    }

    if (!mapped) {
      int sameIdx = FindShapeIndex(result.normalizedEdges, origE);
      if (sameIdx > 0) {
        normEdgeMap[sameIdx].sourceEdgeIds.push_back(origId);
        mapped = true;
      }
    }

    if (!mapped) {
      if (!history.IsNull() && history->IsRemoved(origE)) {
        RemovedEdgeInfo removed;
        removed.sourceEdgeId = origId;
        if (BRep_Tool::Degenerated(origE)) {
          removed.reason = "DEGENERATED";
        } else if (edgeFacesMap.Contains(origE)) {
          const auto &faces = edgeFacesMap.FindFromKey(origE);
          if (faces.Extent() == 1 && BRepTools::IsReallyClosed(origE, TopoDS::Face(faces.First()))) {
            removed.reason = "PERIODIC_SEAM";
          } else if (faces.Extent() >= 2) {
            removed.reason = "SAME_DOMAIN_INTERNAL_EDGE";
          } else {
            removed.reason = "REMOVED_BY_NORMALIZATION";
          }
        } else {
          removed.reason = "REMOVED_BY_NORMALIZATION";
        }
        result.audit.removedEdges.push_back(removed);
        mapped = true;
      }
    }

    if (!mapped) {
      edgeMappingComplete = false;
    }
  }

  int compIdx = 1;
  for (int j = 1; j <= result.normalizedEdges.Extent(); ++j) {
    auto &info = normEdgeMap[j];
    std::sort(info.sourceEdgeIds.begin(), info.sourceEdgeIds.end());
    info.sourceEdgeIds.erase(std::unique(info.sourceEdgeIds.begin(), info.sourceEdgeIds.end()), info.sourceEdgeIds.end());
    info.sourceCount = static_cast<int>(info.sourceEdgeIds.size());
    info.merged = info.sourceCount > 1;

    const TopoDS_Edge edge = TopoDS::Edge(result.normalizedEdges(j));
    const auto classification = ClassifyNormalizedEdge(edge, edgeFacesMap);
    info.comparisonRole = classification.role;
    info.comparable = classification.comparable;
    info.exclusionReason = classification.exclusionReason;
    if (info.comparable) {
      info.comparableIndex = compIdx++;
    } else {
      info.comparableIndex = 0;
    }
    result.audit.edges.push_back(info);
  }

  result.audit.comparableEdgeCountAfter = compIdx - 1;

  TopTools_IndexedDataMapOfShapeListOfShape origEdgeFacesMap;
  TopExp::MapShapesAndAncestors(input, TopAbs_EDGE, TopAbs_FACE, origEdgeFacesMap);
  int origCompCount = 0;
  for (int i = 1; i <= originalIndex.edges.Extent(); ++i) {
    const TopoDS_Edge origE = TopoDS::Edge(originalIndex.edges(i));
    if (ClassifyNormalizedEdge(origE, origEdgeFacesMap).comparable) {
      origCompCount++;
    }
  }
  result.audit.comparableEdgeCountBefore = origCompCount;

  result.audit.faceMappingComplete = faceMappingComplete;
  result.audit.edgeMappingComplete = edgeMappingComplete;
  result.audit.mappingComplete = faceMappingComplete && edgeMappingComplete;

  // Type Statistics
  std::map<std::string, std::pair<int, double>> faceTypeStats;
  for (const auto &info : result.audit.faces) {
    auto &item = faceTypeStats[info.surfaceType];
    item.first += 1;
    item.second += info.areaMm2;
  }
  result.audit.faceTypes = SummarizeMap(faceTypeStats);

  std::map<std::string, std::pair<int, double>> edgeTypeStats;
  for (const auto &info : result.audit.edges) {
    auto &item = edgeTypeStats[info.curveType];
    item.first += 1;
    item.second += info.lengthMm;
  }
  result.audit.edgeTypes = SummarizeMap(edgeTypeStats);

  return result;
}

std::vector<FaceDescriptor> CollectFaceDescriptors(const TopoDS_Shape &shape, EntitySide side) {
  std::vector<FaceDescriptor> descriptors;
  TopTools_IndexedMapOfShape faceMap;
  TopExp::MapShapes(shape, TopAbs_FACE, faceMap);

  descriptors.reserve(faceMap.Extent());
  for (int i = 1; i <= faceMap.Extent(); ++i) {
    const TopoDS_Face face = TopoDS::Face(faceMap(i));
    FaceDescriptor descriptor;
    descriptor.entityId = MakeEntityId(side, EntityKind::NormalizedFace, i);
    descriptor.visualIndex = i;
    descriptor.face = face;

    BRepAdaptor_Surface surface(face);
    descriptor.typeName = SurfaceTypeName(surface.GetType());

    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    descriptor.areaMm2 = props.Mass();

    const gp_Pnt centroidPnt = props.CentreOfMass();
    descriptor.centroidMm = {centroidPnt.X(), centroidPnt.Y(), centroidPnt.Z()};
    descriptor.boundsMm = ComputeBounds(face);

    descriptors.push_back(descriptor);
  }

  return descriptors;
}

std::vector<EdgeDescriptor> CollectEdgeDescriptors(const TopTools_IndexedMapOfShape &normalizedEdges,
                                                   const std::vector<NormalizedEdgeInfo> &edgeInfos,
                                                   EntitySide side) {
  std::vector<EdgeDescriptor> descriptors;

  for (const auto &info : edgeInfos) {
    if (!info.comparable) {
      continue;
    }

    const int visualIndex = info.visualIndex;
    if (visualIndex <= 0 || visualIndex > normalizedEdges.Extent()) {
      continue;
    }

    const TopoDS_Edge edge = TopoDS::Edge(normalizedEdges(visualIndex));
    if (edge.IsNull()) continue;

    EdgeDescriptor descriptor;
    descriptor.entityId = MakeEntityId(side, EntityKind::NormalizedEdge, visualIndex);
    descriptor.visualIndex = visualIndex;
    descriptor.edge = edge;

    BRepAdaptor_Curve curve(edge);
    descriptor.typeName = CurveTypeName(curve.GetType());

    GProp_GProps props;
    BRepGProp::LinearProperties(edge, props);
    descriptor.lengthMm = props.Mass();

    const gp_Pnt centroidPnt = props.CentreOfMass();
    descriptor.centroidMm = {centroidPnt.X(), centroidPnt.Y(), centroidPnt.Z()};
    descriptor.boundsMm = ComputeBounds(edge);

    descriptors.push_back(descriptor);
  }

  return descriptors;
}

double ComputeDescriptorScore(double measureErr, double centroidErr, double boundsErr) {
  double score = 1.0 - 0.4 * measureErr - 0.3 * centroidErr - 0.3 * boundsErr;
  return std::clamp(score, 0.0, 1.0);
}

MatchCollection MatchFaceDescriptors(const std::vector<FaceDescriptor> &referenceFaces,
                                     const std::vector<FaceDescriptor> &candidateFaces,
                                     const CompareConfig &config,
                                     double characteristicScale) {
  MatchCollection collection;
  collection.attempted = true;
  collection.referenceCount = static_cast<int>(referenceFaces.size());
  collection.candidateCount = static_cast<int>(candidateFaces.size());

  const double distTol = std::max(config.distanceToleranceMm, 1.0e-4 * characteristicScale);

  std::map<std::string, int> refHistogram;
  for (const auto &f : referenceFaces) {
    refHistogram[f.typeName]++;
  }
  std::map<std::string, int> candHistogram;
  for (const auto &f : candidateFaces) {
    candHistogram[f.typeName]++;
  }
  collection.typeHistogramEqual = (refHistogram == candHistogram);

  std::vector<bool> candidateUsed(candidateFaces.size(), false);
  int matchPairIdx = 1;

  for (size_t i = 0; i < referenceFaces.size(); ++i) {
    const auto &refFace = referenceFaces[i];

    int bestCandidateIdx = -1;
    double bestScore = -1.0;
    int secondBestIdx = -1;
    double secondBestScore = -1.0;
    MatchMetrics bestMetrics;

    for (size_t j = 0; j < candidateFaces.size(); ++j) {
      if (candidateUsed[j]) {
        continue;
      }
      const auto &candFace = candidateFaces[j];
      if (refFace.typeName != candFace.typeName) {
        continue;
      }

      const double areaDiff = std::abs(refFace.areaMm2 - candFace.areaMm2);
      const double relAreaDiff = refFace.areaMm2 > 0.0 ? (areaDiff / refFace.areaMm2) : 0.0;
      const double centroidDist = PointDistance(refFace.centroidMm, candFace.centroidMm);
      const double boundsDiff = ComputeBoundsDifference(refFace.boundsMm, candFace.boundsMm);

      const bool areaPass = relAreaDiff <= config.relativeVolumeTolerance * 100.0;
      const bool distPass = centroidDist <= distTol;
      const bool boundsPass = boundsDiff <= distTol;

      if (areaPass && distPass && boundsPass) {
        const double normAreaErr = std::min(1.0, relAreaDiff);
        const double normCentroidErr = characteristicScale > 0.0 ? std::min(1.0, centroidDist / characteristicScale) : 0.0;
        const double normBoundsErr = characteristicScale > 0.0 ? std::min(1.0, boundsDiff / characteristicScale) : 0.0;
        const double score = ComputeDescriptorScore(normAreaErr, normCentroidErr, normBoundsErr);

        if (score > bestScore) {
          secondBestScore = bestScore;
          secondBestIdx = bestCandidateIdx;
          bestScore = score;
          bestCandidateIdx = static_cast<int>(j);
          bestMetrics = {areaDiff, relAreaDiff, centroidDist, boundsDiff};
        } else if (score > secondBestScore) {
          secondBestScore = score;
          secondBestIdx = static_cast<int>(j);
        }
      }
    }

    EntityMatch item;
    item.id = "face-match:" + MakeEntityId(EntitySide::Reference, EntityKind::NormalizedFace, matchPairIdx++);
    item.referenceId = refFace.entityId;
    item.geometryType = refFace.typeName;
    item.verificationLevel = VerificationLevel::Descriptor;

    if (bestCandidateIdx >= 0) {
      const bool isAmbiguous = (secondBestIdx >= 0) && ((bestScore - secondBestScore) < config.ambiguousMatchMargin);
      if (isAmbiguous) {
        item.status = MatchStatus::Ambiguous;
        item.candidateId = candidateFaces[bestCandidateIdx].entityId;
        item.score = bestScore;
        item.metrics = bestMetrics;
        item.reasonCodes.push_back("MULTIPLE_SIMILAR_CANDIDATES");
        collection.ambiguousCount++;
      } else {
        item.status = MatchStatus::Matched;
        item.candidateId = candidateFaces[bestCandidateIdx].entityId;
        item.score = bestScore;
        item.metrics = bestMetrics;
        candidateUsed[bestCandidateIdx] = true;
        collection.matchedCount++;
      }
    } else {
      item.status = MatchStatus::Unmatched;
      item.candidateId = "";
      item.score = std::nullopt;
      item.metrics = std::nullopt;
      item.reasonCodes.push_back("NO_ONE_TO_ONE_CANDIDATE");
      collection.unmatchedReferenceIds.push_back(refFace.entityId);
    }
    collection.items.push_back(item);
  }

  for (size_t j = 0; j < candidateFaces.size(); ++j) {
    if (!candidateUsed[j]) {
      collection.unmatchedCandidateIds.push_back(candidateFaces[j].entityId);
    }
  }

  collection.allMatched = (collection.matchedCount == collection.referenceCount) &&
                          (collection.referenceCount == collection.candidateCount);
  return collection;
}

MatchCollection MatchEdgeDescriptors(const std::vector<EdgeDescriptor> &referenceEdges,
                                     const std::vector<EdgeDescriptor> &candidateEdges,
                                     const CompareConfig &config,
                                     double characteristicScale) {
  MatchCollection collection;
  collection.attempted = true;
  collection.referenceCount = static_cast<int>(referenceEdges.size());
  collection.candidateCount = static_cast<int>(candidateEdges.size());

  const double distTol = std::max(config.distanceToleranceMm, 1.0e-4 * characteristicScale);

  std::map<std::string, int> refHistogram;
  for (const auto &e : referenceEdges) {
    refHistogram[e.typeName]++;
  }
  std::map<std::string, int> candHistogram;
  for (const auto &e : candidateEdges) {
    candHistogram[e.typeName]++;
  }
  collection.typeHistogramEqual = (refHistogram == candHistogram);

  std::vector<bool> candidateUsed(candidateEdges.size(), false);
  int matchPairIdx = 1;

  for (size_t i = 0; i < referenceEdges.size(); ++i) {
    const auto &refEdge = referenceEdges[i];

    int bestCandidateIdx = -1;
    double bestScore = -1.0;
    int secondBestIdx = -1;
    double secondBestScore = -1.0;
    MatchMetrics bestMetrics;

    for (size_t j = 0; j < candidateEdges.size(); ++j) {
      if (candidateUsed[j]) {
        continue;
      }
      const auto &candEdge = candidateEdges[j];
      if (refEdge.typeName != candEdge.typeName) {
        continue;
      }

      const double lenDiff = std::abs(refEdge.lengthMm - candEdge.lengthMm);
      const double relLenDiff = refEdge.lengthMm > 0.0 ? (lenDiff / refEdge.lengthMm) : 0.0;
      const double centroidDist = PointDistance(refEdge.centroidMm, candEdge.centroidMm);
      const double boundsDiff = ComputeBoundsDifference(refEdge.boundsMm, candEdge.boundsMm);

      const bool lenPass = lenDiff <= distTol;
      const bool distPass = centroidDist <= distTol;
      const bool boundsPass = boundsDiff <= distTol;

      if (lenPass && distPass && boundsPass) {
        const double normLenErr = characteristicScale > 0.0 ? std::min(1.0, lenDiff / characteristicScale) : 0.0;
        const double normCentroidErr = characteristicScale > 0.0 ? std::min(1.0, centroidDist / characteristicScale) : 0.0;
        const double normBoundsErr = characteristicScale > 0.0 ? std::min(1.0, boundsDiff / characteristicScale) : 0.0;
        const double score = ComputeDescriptorScore(normLenErr, normCentroidErr, normBoundsErr);

        if (score > bestScore) {
          secondBestScore = bestScore;
          secondBestIdx = bestCandidateIdx;
          bestScore = score;
          bestCandidateIdx = static_cast<int>(j);
          bestMetrics = {lenDiff, relLenDiff, centroidDist, boundsDiff};
        } else if (score > secondBestScore) {
          secondBestScore = score;
          secondBestIdx = static_cast<int>(j);
        }
      }
    }

    EntityMatch item;
    item.id = "edge-match:" + MakeEntityId(EntitySide::Reference, EntityKind::NormalizedEdge, matchPairIdx++);
    item.referenceId = refEdge.entityId;
    item.geometryType = refEdge.typeName;
    item.verificationLevel = VerificationLevel::Descriptor;

    if (bestCandidateIdx >= 0) {
      const bool isAmbiguous = (secondBestIdx >= 0) && ((bestScore - secondBestScore) < config.ambiguousMatchMargin);
      if (isAmbiguous) {
        item.status = MatchStatus::Ambiguous;
        item.candidateId = candidateEdges[bestCandidateIdx].entityId;
        item.score = bestScore;
        item.metrics = bestMetrics;
        item.reasonCodes.push_back("MULTIPLE_SIMILAR_CANDIDATES");
        collection.ambiguousCount++;
      } else {
        item.status = MatchStatus::Matched;
        item.candidateId = candidateEdges[bestCandidateIdx].entityId;
        item.score = bestScore;
        item.metrics = bestMetrics;
        candidateUsed[bestCandidateIdx] = true;
        collection.matchedCount++;
      }
    } else {
      item.status = MatchStatus::Unmatched;
      item.candidateId = "";
      item.score = std::nullopt;
      item.metrics = std::nullopt;
      item.reasonCodes.push_back("NO_ONE_TO_ONE_CANDIDATE");
      collection.unmatchedReferenceIds.push_back(refEdge.entityId);
    }
    collection.items.push_back(item);
  }

  for (size_t j = 0; j < candidateEdges.size(); ++j) {
    if (!candidateUsed[j]) {
      collection.unmatchedCandidateIds.push_back(candidateEdges[j].entityId);
    }
  }

  collection.allMatched = (collection.matchedCount == collection.referenceCount) &&
                          (collection.referenceCount == collection.candidateCount);
  return collection;
}

BooleanDifferenceResult CutSolids(const TopoDS_Solid &argument, const TopoDS_Solid &tool, double fuzzyTolerance) {
  BooleanDifferenceResult result;
  if (argument.IsNull()) {
    result.audit.succeeded = false;
    result.audit.report = "argument solid is null";
    return result;
  }
  if (tool.IsNull()) {
    result.audit.succeeded = true;
    result.audit.volumeMm3 = 0.0;
    result.audit.componentCount = 0;
    result.audit.report = "tool solid is null";
    return result;
  }

  try {
    TopTools_ListOfShape argList;
    argList.Append(argument);
    TopTools_ListOfShape toolList;
    toolList.Append(tool);

    BRepAlgoAPI_Cut cutAlgo;
    cutAlgo.SetArguments(argList);
    cutAlgo.SetTools(toolList);
    cutAlgo.SetFuzzyValue(fuzzyTolerance);
    cutAlgo.SetRunParallel(true);
    cutAlgo.SetUseOBB(true);
    cutAlgo.SetToFillHistory(false);
    cutAlgo.SetNonDestructive(true);
    cutAlgo.Build();

    if (cutAlgo.HasErrors()) {
      result.audit.succeeded = false;
      result.audit.report = "BRepAlgoAPI_Cut failed";
      return result;
    }

    result.shape = cutAlgo.Shape();
    if (result.shape.IsNull()) {
      result.audit.succeeded = true;
      result.audit.volumeMm3 = 0.0;
      result.audit.componentCount = 0;
      return result;
    }

    GProp_GProps props;
    BRepGProp::VolumeProperties(result.shape, props);
    result.audit.succeeded = true;
    result.audit.volumeMm3 = std::abs(props.Mass());
    result.audit.componentCount = CountUniqueSubShapes(result.shape, TopAbs_SOLID);
  } catch (const Standard_Failure &e) {
    result.audit.succeeded = false;
    result.audit.report = std::string("BRepAlgoAPI_Cut exception: ") + e.GetMessageString();
  } catch (...) {
    result.audit.succeeded = false;
    result.audit.report = "BRepAlgoAPI_Cut unknown exception";
  }

  return result;
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

} // namespace

std::string MakeEntityId(EntitySide side, EntityKind kind, int index) {
  std::ostringstream ss;
  ss << (side == EntitySide::Reference ? "ref:" : "cand:");
  switch (kind) {
  case EntityKind::OriginalFace:
    ss << "face:";
    break;
  case EntityKind::OriginalEdge:
    ss << "edge:";
    break;
  case EntityKind::NormalizedFace:
    ss << "nface:";
    break;
  case EntityKind::NormalizedEdge:
    ss << "nedge:";
    break;
  }
  ss << std::setw(6) << std::setfill('0') << index;
  return ss.str();
}

const char *ToString(CompareStatus status) {
  switch (status) {
  case CompareStatus::Equal:
    return "EQUAL";
  case CompareStatus::LikelyEqual:
    return "LIKELY_EQUAL";
  case CompareStatus::Different:
    return "DIFFERENT";
  case CompareStatus::InvalidInput:
    return "INVALID_INPUT";
  case CompareStatus::UnsupportedShape:
    return "UNSUPPORTED_SHAPE";
  case CompareStatus::Indeterminate:
    return "INDETERMINATE";
  case CompareStatus::InternalError:
    return "INTERNAL_ERROR";
  }
  return "UNKNOWN";
}

const char *ToString(MatchStatus status) {
  switch (status) {
  case MatchStatus::Matched:
    return "MATCHED";
  case MatchStatus::Unmatched:
    return "UNMATCHED";
  case MatchStatus::Ambiguous:
    return "AMBIGUOUS";
  case MatchStatus::Unsupported:
    return "UNSUPPORTED";
  }
  return "UNKNOWN";
}

const char *ToString(VerificationLevel level) {
  switch (level) {
  case VerificationLevel::TypeOnly:
    return "TYPE_ONLY";
  case VerificationLevel::Descriptor:
    return "DESCRIPTOR";
  case VerificationLevel::AnalyticSupport:
    return "ANALYTIC_SUPPORT";
  case VerificationLevel::DistanceVerified:
    return "DISTANCE_VERIFIED";
  case VerificationLevel::BooleanVerified:
    return "BOOLEAN_VERIFIED";
  }
  return "UNKNOWN";
}

int ExitCode(CompareStatus status) {
  switch (status) {
  case CompareStatus::Equal:
    return 0;
  case CompareStatus::LikelyEqual:
    return 3;
  case CompareStatus::Different:
    return 1;
  case CompareStatus::InvalidInput:
    return 2;
  case CompareStatus::UnsupportedShape:
    return 2;
  case CompareStatus::Indeterminate:
    return 4;
  case CompareStatus::InternalError:
    return 5;
  }
  return 5;
}

CompareStatus detail::ClassifyClosedSolidComparison(
    bool volumePass, bool centroidPass, bool boundsPass, bool booleanPass,
    const BooleanConsistencyMetrics &consistency) {
  if (volumePass && centroidPass && boundsPass && booleanPass) {
    return CompareStatus::Equal;
  }
  return CompareStatus::Different;
}

EdgeAuditValidation ValidateEdgeAudit(const NormalizationAudit &refNorm,
                                     const NormalizationAudit &candNorm,
                                     const MatchCollection &edgeMatches) {
  EdgeAuditValidation validation;

  auto checkNorm = [&](const NormalizationAudit &norm, const std::string &sideName) {
    if (static_cast<int>(norm.edges.size()) != norm.edgeCountAfter) {
      validation.valid = false;
      validation.errors.push_back(sideName + "_EDGE_COUNT_MISMATCH: edges.size() != edgeCountAfter");
    }

    int compCount = 0;
    std::set<int> compIndices;
    for (const auto &info : norm.edges) {
      if (info.comparable) {
        compCount++;
        compIndices.insert(info.comparableIndex);
      } else {
        if (info.comparableIndex != 0) {
          validation.valid = false;
          validation.errors.push_back(sideName + "_NON_COMPARABLE_EDGE_HAS_INDEX: " + info.id);
        }
      }
    }

    if (compCount != norm.comparableEdgeCountAfter) {
      validation.valid = false;
      validation.errors.push_back(sideName + "_COMPARABLE_COUNT_MISMATCH: count=" +
                                  std::to_string(compCount) + " audit=" + std::to_string(norm.comparableEdgeCountAfter));
    }

    if (compCount > 0) {
      if (*compIndices.begin() != 1 || *compIndices.rbegin() != compCount || static_cast<int>(compIndices.size()) != compCount) {
        validation.valid = false;
        validation.errors.push_back(sideName + "_COMPARABLE_INDICES_NOT_CONTIGUOUS");
      }
    }
  };

  checkNorm(refNorm, "REF");
  checkNorm(candNorm, "CAND");

  if (edgeMatches.attempted) {
    if (edgeMatches.referenceCount != refNorm.comparableEdgeCountAfter) {
      validation.valid = false;
      validation.errors.push_back("EDGE_MATCH_REF_COUNT_MISMATCH: matchRef=" +
                                  std::to_string(edgeMatches.referenceCount) + " normRef=" + std::to_string(refNorm.comparableEdgeCountAfter));
    }
    if (edgeMatches.candidateCount != candNorm.comparableEdgeCountAfter) {
      validation.valid = false;
      validation.errors.push_back("EDGE_MATCH_CAND_COUNT_MISMATCH: matchCand=" +
                                  std::to_string(edgeMatches.candidateCount) + " normCand=" + std::to_string(candNorm.comparableEdgeCountAfter));
    }
  }

  return validation;
}

const char *ToString(MultiSolidPolicy policy) {
  switch (policy) {
  case MultiSolidPolicy::Strict:
    return "strict";
  case MultiSolidPolicy::CollectionOnly:
    return "collection";
  case MultiSolidPolicy::Pairwise:
    return "pairwise";
  }
  return "unknown";
}

CompareResult CompareSolidPairInternal(const TopoDS_Solid &solidRef,
                                       const InputAudit &auditRef,
                                       const TopoDS_Solid &solidCand,
                                       const InputAudit &auditCand,
                                       const CompareConfig &config,
                                       const std::filesystem::path &outputDirectory,
                                       const std::string &artifactSuffix) {
  const auto totalStart = std::chrono::high_resolution_clock::now();
  CompareResult result;
  result.thresholds = config;
  result.reference = auditRef;
  result.candidate = auditCand;

  const OriginalTopologyIndex refOriginalIndex = BuildOriginalTopologyIndex(solidRef, EntitySide::Reference);
  const OriginalTopologyIndex candOriginalIndex = BuildOriginalTopologyIndex(solidCand, EntitySide::Candidate);

  const auto normRefStart = std::chrono::high_resolution_clock::now();
  const NormalizedSolidInternal normRef = NormalizeSameDomain(solidRef, refOriginalIndex, EntitySide::Reference, config);
  result.timings.normalizeReferenceMs = ElapsedMs(normRefStart);
  result.referenceNormalization = normRef.audit;

  const auto normCandStart = std::chrono::high_resolution_clock::now();
  const NormalizedSolidInternal normCand = NormalizeSameDomain(solidCand, candOriginalIndex, EntitySide::Candidate, config);
  result.timings.normalizeCandidateMs = ElapsedMs(normCandStart);
  result.candidateNormalization = normCand.audit;

  const bool normalizationPairUsable = result.referenceNormalization.succeeded && result.candidateNormalization.succeeded;

  const TopoDS_Solid &compareSolidRef = normalizationPairUsable ? normRef.solid : solidRef;
  const TopoDS_Solid &compareSolidCand = normalizationPairUsable ? normCand.solid : solidCand;

  result.decisionPath = normalizationPairUsable ? "boolean_after_normalization" : "boolean_after_original";

  const double diagScale = BoundsDiagonal(result.reference.boundsMm);
  const double effectiveDistTol = std::max(config.distanceToleranceMm, 1.0e-4 * diagScale);
  const double refVol = result.reference.signedVolumeMm3;
  const double effectiveAbsVolTol = std::max(config.absoluteVolumeToleranceMm3, config.relativeVolumeTolerance * refVol);

  result.absoluteInputVolumeDifferenceMm3 = std::abs(result.reference.signedVolumeMm3 - result.candidate.signedVolumeMm3);
  const double volDenom = std::abs(result.reference.signedVolumeMm3);
  result.relativeInputVolumeDifference = volDenom > 0.0 ? (result.absoluteInputVolumeDifferenceMm3 / volDenom) : 0.0;

  result.centroidDistanceMm = PointDistance(result.reference.centroidMm, result.candidate.centroidMm);
  result.maximumBoundsDifferenceMm = ComputeBoundsDifference(result.reference.boundsMm, result.candidate.boundsMm);
  result.globalMetricsExecuted = true;

  const bool volumePass = (result.absoluteInputVolumeDifferenceMm3 <= effectiveAbsVolTol) &&
                          (result.relativeInputVolumeDifference <= config.relativeVolumeTolerance);
  const bool centroidPass = result.centroidDistanceMm <= effectiveDistTol;
  const bool boundsPass = result.maximumBoundsDifferenceMm <= effectiveDistTol;

  if (!volumePass || !centroidPass || !boundsPass) {
    result.status = CompareStatus::Different;
    result.reason = "geometry thresholds failed: input_volume centroid symmetric_difference";
    result.decisionPath = "global_metrics_failed";
  }

  // Descriptors & Matching
  if (normalizationPairUsable) {
    const auto descStart = std::chrono::high_resolution_clock::now();
    const std::vector<FaceDescriptor> refFaceDescs = CollectFaceDescriptors(compareSolidRef, EntitySide::Reference);
    const std::vector<FaceDescriptor> candFaceDescs = CollectFaceDescriptors(compareSolidCand, EntitySide::Candidate);
    const std::vector<EdgeDescriptor> refEdgeDescs = CollectEdgeDescriptors(normRef.normalizedEdges, normRef.audit.edges, EntitySide::Reference);
    const std::vector<EdgeDescriptor> candEdgeDescs = CollectEdgeDescriptors(normCand.normalizedEdges, normCand.audit.edges, EntitySide::Candidate);
    result.timings.descriptorBuildMs = ElapsedMs(descStart);

    const auto faceMatchStart = std::chrono::high_resolution_clock::now();
    result.normalizedTopology.faces = MatchFaceDescriptors(refFaceDescs, candFaceDescs, config, diagScale);
    result.timings.faceMatchMs = ElapsedMs(faceMatchStart);
    result.normalizedTopology.faces.elapsedMs = result.timings.faceMatchMs;

    const auto edgeMatchStart = std::chrono::high_resolution_clock::now();
    result.normalizedTopology.edges = MatchEdgeDescriptors(refEdgeDescs, candEdgeDescs, config, diagScale);
    result.timings.edgeMatchMs = ElapsedMs(edgeMatchStart);
    result.normalizedTopology.edges.elapsedMs = result.timings.edgeMatchMs;

    const auto edgeValidation = ValidateEdgeAudit(normRef.audit, normCand.audit, result.normalizedTopology.edges);
    result.normalizedTopology.edgeAuditConsistent = edgeValidation.valid;
    result.normalizedTopology.edgeAuditErrors = edgeValidation.errors;

    result.normalizedTopology.attempted = true;
    result.normalizedTopology.normalizedTopologyMatch =
        result.normalizedTopology.faces.allMatched && result.normalizedTopology.edges.allMatched &&
        (result.normalizedTopology.faces.ambiguousCount == 0) && (result.normalizedTopology.edges.ambiguousCount == 0) &&
        result.normalizedTopology.faces.typeHistogramEqual && result.normalizedTopology.edges.typeHistogramEqual &&
        result.normalizedTopology.edgeAuditConsistent;
    result.normalizedTopology.elapsedMs = result.timings.descriptorBuildMs + result.timings.faceMatchMs + result.timings.edgeMatchMs;

    result.normalizedTopology.fastPath.enabled = config.enableNormalizedFastPath;
    result.normalizedTopology.fastPath.eligible = result.normalizedTopology.normalizedTopologyMatch;
    if (!result.normalizedTopology.edgeAuditConsistent) {
      result.normalizedTopology.fastPath.blockReasons.push_back("EDGE_AUDIT_INCONSISTENT");
    }
    if (!result.normalizedTopology.faces.allMatched) {
      result.normalizedTopology.fastPath.blockReasons.push_back(
          "FACE_DESCRIPTOR_UNMATCHED:" + std::to_string(result.normalizedTopology.faces.referenceCount - result.normalizedTopology.faces.matchedCount));
    }
    if (!result.normalizedTopology.edges.allMatched) {
      result.normalizedTopology.fastPath.blockReasons.push_back(
          "EDGE_DESCRIPTOR_UNMATCHED:" + std::to_string(result.normalizedTopology.edges.referenceCount - result.normalizedTopology.edges.matchedCount));
    }

    if (config.enableNormalizedFastPath && result.normalizedTopology.normalizedTopologyMatch &&
        (result.status != CompareStatus::Different)) {
      result.status = CompareStatus::Equal;
      result.reason = "closed solids pass configured thresholds via normalized fast path";
      result.decisionPath = "normalized_topology_fast_path";
      result.booleanExecuted = false;
      result.normalizedTopology.fastPath.used = true;
    }
  } else {
    result.normalizedTopology.attempted = false;
    result.normalizedTopology.skipReason = "normalization pair is not usable";
  }

  // Boolean difference if not skipped by fast path
  BooleanDifferenceResult missingRes;
  BooleanDifferenceResult addedRes;
  if (result.decisionPath != "normalized_topology_fast_path") {
    result.booleanExecuted = true;
    const auto boolAbStart = std::chrono::high_resolution_clock::now();
    missingRes = CutSolids(compareSolidRef, compareSolidCand, config.booleanFuzzyToleranceMm);
    result.timings.booleanAbMs = ElapsedMs(boolAbStart);
    result.missingMaterial = missingRes.audit;

    const auto boolBaStart = std::chrono::high_resolution_clock::now();
    addedRes = CutSolids(compareSolidCand, compareSolidRef, config.booleanFuzzyToleranceMm);
    result.timings.booleanBaMs = ElapsedMs(boolBaStart);
    result.addedMaterial = addedRes.audit;

    if (!result.missingMaterial.succeeded || !result.addedMaterial.succeeded) {
      result.status = CompareStatus::Indeterminate;
      result.reason = "boolean cut operation failed: " + result.missingMaterial.report + " " + result.addedMaterial.report;
      result.decisionPath = "boolean_failed";
    } else {
      result.symmetricDifferenceVolumeMm3 = result.missingMaterial.volumeMm3 + result.addedMaterial.volumeMm3;
      result.symmetricDifferenceRelative = volDenom > 0.0 ? (result.symmetricDifferenceVolumeMm3 / volDenom) : 0.0;

      bool booleanPass = (result.missingMaterial.volumeMm3 <= effectiveAbsVolTol) &&
                         (result.addedMaterial.volumeMm3 <= effectiveAbsVolTol);

      if (!booleanPass && normalizationPairUsable && volumePass && centroidPass && boundsPass) {
        BooleanDifferenceResult origMissing = CutSolids(solidRef, solidCand, config.booleanFuzzyToleranceMm);
        BooleanDifferenceResult origAdded = CutSolids(solidCand, solidRef, config.booleanFuzzyToleranceMm);
        if (origMissing.audit.succeeded && origAdded.audit.succeeded) {
          const bool origBooleanPass = (origMissing.audit.volumeMm3 <= effectiveAbsVolTol) &&
                                       (origAdded.audit.volumeMm3 <= effectiveAbsVolTol);
          if (origBooleanPass) {
            missingRes = origMissing;
            addedRes = origAdded;
            result.missingMaterial = origMissing.audit;
            result.addedMaterial = origAdded.audit;
            result.symmetricDifferenceVolumeMm3 = result.missingMaterial.volumeMm3 + result.addedMaterial.volumeMm3;
            result.symmetricDifferenceRelative = volDenom > 0.0 ? (result.symmetricDifferenceVolumeMm3 / volDenom) : 0.0;
            booleanPass = true;
            result.decisionPath = "boolean_after_original_fallback";
          }
        }
      }
      auto &consistency = result.booleanConsistency;
      consistency.cutReferenceMinusCandidateSucceeded = result.missingMaterial.succeeded;
      consistency.cutCandidateMinusReferenceSucceeded = result.addedMaterial.succeeded;
      consistency.signedInputVolumeDiffMm3 =
          result.reference.signedVolumeMm3 - result.candidate.signedVolumeMm3;
      consistency.signedBooleanVolumeDiffMm3 =
          result.missingMaterial.volumeMm3 - result.addedMaterial.volumeMm3;
      consistency.conservationErrorMm3 = std::abs(
          consistency.signedInputVolumeDiffMm3 - consistency.signedBooleanVolumeDiffMm3);
      const double conservationScale = std::max(
          {1.0, std::abs(result.reference.signedVolumeMm3),
           std::abs(result.candidate.signedVolumeMm3)});
      consistency.relativeConservationError =
          consistency.conservationErrorMm3 / conservationScale;
      consistency.conservationPassed =
          consistency.relativeConservationError <= config.booleanConservationRelativeTolerance;
      consistency.booleanResultValid =
          consistency.cutReferenceMinusCandidateSucceeded &&
          consistency.cutCandidateMinusReferenceSucceeded && consistency.conservationPassed;
      if (!consistency.booleanResultValid) {
        consistency.invalidReason = "boolean volume conservation check failed";
      }

      const CompareStatus classifiedStatus =
          detail::ClassifyClosedSolidComparison(
              volumePass, centroidPass, boundsPass, booleanPass, consistency);
      if (classifiedStatus == CompareStatus::Equal) {
        result.status = CompareStatus::Equal;
        result.reason = (result.decisionPath == "boolean_after_original_fallback") ?
                        "closed solids pass configured thresholds (via original solid boolean fallback)" :
                        "closed solids pass configured thresholds";
      } else if (classifiedStatus == CompareStatus::LikelyEqual) {
        result.status = CompareStatus::LikelyEqual;
        result.reason = "geometry thresholds passed but boolean conservation is invalid";
        result.decisionPath = "boolean_conservation_invalid";
      } else {
        result.status = CompareStatus::Different;
        result.reason = "geometry thresholds failed: input_volume centroid symmetric_difference";
        result.decisionPath = "boolean_difference";
      }
    }
  }

  if (!outputDirectory.empty()) {
    const auto exportStart = std::chrono::high_resolution_clock::now();
    std::error_code ec;
    std::filesystem::create_directories(outputDirectory, ec);
    if (artifactSuffix.empty()) {
      RemoveOldArtifacts(outputDirectory);
    }

    const std::string sfx = artifactSuffix;
    if (config.exportStl) {
      const auto refStl = outputDirectory / ("reference_base" + sfx + ".stl");
      if (ExportShapeStl(compareSolidRef, refStl)) {
        result.artifacts.items.push_back({"reference_base" + sfx, "reference_base" + sfx + ".stl", "STL", true, ""});
      }
      const auto candStl = outputDirectory / ("candidate_base" + sfx + ".stl");
      if (ExportShapeStl(compareSolidCand, candStl)) {
        result.artifacts.items.push_back({"candidate_base" + sfx, "candidate_base" + sfx + ".stl", "STL", true, ""});
      }
      if (result.booleanExecuted && result.missingMaterial.volumeMm3 > 0.0) {
        const auto missStl = outputDirectory / ("missing_material" + sfx + ".stl");
        if (ExportShapeStl(missingRes.shape, missStl)) {
          result.artifacts.items.push_back({"missing_material" + sfx, "missing_material" + sfx + ".stl", "STL", true, ""});
        }
      }
      if (result.booleanExecuted && result.addedMaterial.volumeMm3 > 0.0) {
        const auto addStl = outputDirectory / ("added_material" + sfx + ".stl");
        if (ExportShapeStl(addedRes.shape, addStl)) {
          result.artifacts.items.push_back({"added_material" + sfx, "added_material" + sfx + ".stl", "STL", true, ""});
        }
      }
    }

    if (config.exportBrep) {
      ExportShapeBrep(solidRef, outputDirectory / ("reference_original" + sfx + ".brep"));
      ExportShapeBrep(solidCand, outputDirectory / ("candidate_original" + sfx + ".brep"));
      if (normalizationPairUsable) {
        ExportShapeBrep(normRef.solid, outputDirectory / ("reference_normalized" + sfx + ".brep"));
        ExportShapeBrep(normCand.solid, outputDirectory / ("candidate_normalized" + sfx + ".brep"));
      }
    }

    const auto visDir = outputDirectory / "visualization";
    std::filesystem::create_directories(visDir, ec);

    if (ExportFacesVtp(normRef.solid, normRef.normalizedFaces, normRef.audit.faces, result.normalizedTopology.faces, EntitySide::Reference, visDir / ("reference_faces" + sfx + ".vtp"))) {
      result.artifacts.items.push_back({"reference_faces" + sfx, "visualization/reference_faces" + sfx + ".vtp", "VTP", true, "entity_index"});
    }
    if (ExportFacesVtp(normCand.solid, normCand.normalizedFaces, normCand.audit.faces, result.normalizedTopology.faces, EntitySide::Candidate, visDir / ("candidate_faces" + sfx + ".vtp"))) {
      result.artifacts.items.push_back({"candidate_faces" + sfx, "visualization/candidate_faces" + sfx + ".vtp", "VTP", true, "entity_index"});
    }
    if (ExportEdgesVtp(normRef.solid, normRef.normalizedEdges, normRef.audit.edges, result.normalizedTopology.edges, EntitySide::Reference, visDir / ("reference_edges" + sfx + ".vtp"))) {
      result.artifacts.items.push_back({"reference_edges" + sfx, "visualization/reference_edges" + sfx + ".vtp", "VTP", true, "entity_index"});
    }
    if (ExportEdgesVtp(normCand.solid, normCand.normalizedEdges, normCand.audit.edges, result.normalizedTopology.edges, EntitySide::Candidate, visDir / ("candidate_edges" + sfx + ".vtp"))) {
      result.artifacts.items.push_back({"candidate_edges" + sfx, "visualization/candidate_edges" + sfx + ".vtp", "VTP", true, "entity_index"});
    }

    result.timings.artifactExportMs = ElapsedMs(exportStart);
  }

  result.timings.totalMs = ElapsedMs(totalStart);
  return result;
}

CompareResult CompareStepFiles(const std::filesystem::path &reference,
                               const std::filesystem::path &candidate,
                               const CompareConfig &config,
                               const std::filesystem::path &outputDirectory) {
  const auto totalStart = std::chrono::high_resolution_clock::now();
  CompareResult result;
  result.thresholds = config;

  const auto loadRefStart = std::chrono::high_resolution_clock::now();
  const LoadedStepModel loadedRef = LoadStepModel(reference, config, EntitySide::Reference);
  result.timings.loadReferenceMs = ElapsedMs(loadRefStart);
  result.reference = loadedRef.compositeAudit;

  const auto loadCandStart = std::chrono::high_resolution_clock::now();
  const LoadedStepModel loadedCand = LoadStepModel(candidate, config, EntitySide::Candidate);
  result.timings.loadCandidateMs = ElapsedMs(loadCandStart);
  result.candidate = loadedCand.compositeAudit;

  if (loadedRef.classification == LoadClass::Invalid || loadedCand.classification == LoadClass::Invalid) {
    result.status = CompareStatus::InvalidInput;
    result.reason = loadedRef.classification == LoadClass::Invalid ? loadedRef.reason : loadedCand.reason;
    result.decisionPath = "input_invalid";
    result.timings.totalMs = ElapsedMs(totalStart);
    return result;
  }

  if (loadedRef.classification == LoadClass::Unsupported || loadedCand.classification == LoadClass::Unsupported) {
    result.status = CompareStatus::UnsupportedShape;
    result.reason = loadedRef.classification == LoadClass::Unsupported ? loadedRef.reason : loadedCand.reason;
    result.decisionPath = "input_unsupported";
    result.timings.totalMs = ElapsedMs(totalStart);
    return result;
  }

  const bool referenceIsMulti = loadedRef.solids.size() > 1;
  const bool candidateIsMulti = loadedCand.solids.size() > 1;
  const bool actuallyMultiSolid = referenceIsMulti || candidateIsMulti;

  if (!actuallyMultiSolid) {
    CompareResult pairRes = CompareSolidPairInternal(
        loadedRef.solids[0].solid, loadedRef.solids[0].audit,
        loadedCand.solids[0].solid, loadedCand.solids[0].audit,
        config, outputDirectory, "");

    pairRes.reference = loadedRef.compositeAudit;
    pairRes.candidate = loadedCand.compositeAudit;
    pairRes.thresholds = config;
    pairRes.timings.loadReferenceMs = result.timings.loadReferenceMs;
    pairRes.timings.loadCandidateMs = result.timings.loadCandidateMs;
    pairRes.timings.totalMs = ElapsedMs(totalStart);

    pairRes.multiSolid.allowed = config.allowMultipleSolids;
    pairRes.multiSolid.executed = false;
    pairRes.multiSolid.enabled = false;
    pairRes.multiSolid.policy = config.multiSolidPolicy;
    pairRes.multiSolid.referenceSolidCount = 1;
    pairRes.multiSolid.candidateSolidCount = 1;
    pairRes.multiSolid.matchedSolidCount = 1;
    pairRes.multiSolid.unmatchedReferenceSolidCount = 0;
    pairRes.multiSolid.unmatchedCandidateSolidCount = 0;

    SolidMatchRecord matchRec;
    matchRec.referenceSolidId = loadedRef.solids[0].id;
    matchRec.candidateSolidId = loadedCand.solids[0].id;
    matchRec.referenceIndex = 0;
    matchRec.candidateIndex = 0;
    matchRec.matchStatus = MatchStatus::Matched;
    matchRec.status = pairRes.status;
    matchRec.reason = pairRes.reason;
    matchRec.volumeDifferenceMm3 = pairRes.absoluteInputVolumeDifferenceMm3;
    matchRec.relativeVolumeDifference = pairRes.relativeInputVolumeDifference;
    matchRec.centroidDistanceMm = pairRes.centroidDistanceMm;
    matchRec.boundsDifferenceMm = pairRes.maximumBoundsDifferenceMm;
    matchRec.volumeEligible = true;
    matchRec.centroidEligible = true;
    matchRec.boundsEligible = true;
    matchRec.volumeTolerance = config.solidMatchVolumeRelTol;
    matchRec.centroidToleranceMm = config.solidMatchCentroidTolMm;
    matchRec.boundsToleranceMm = config.solidMatchBoundsTolMm;
    pairRes.multiSolid.solidMatches.push_back(matchRec);

    return pairRes;
  }

  if (!config.allowMultipleSolids || config.multiSolidPolicy == MultiSolidPolicy::Strict) {
    result.status = CompareStatus::UnsupportedShape;
    result.reason = "multiple 3D solids detected (reference_solids=" + std::to_string(loadedRef.solids.size()) +
                    ", candidate_solids=" + std::to_string(loadedCand.solids.size()) +
                    "); allowMultipleSolids=false or policy=strict";
    result.decisionPath = "input_unsupported";
    result.multiSolid.allowed = config.allowMultipleSolids;
    result.multiSolid.executed = false;
    result.multiSolid.enabled = false;
    result.multiSolid.policy = config.multiSolidPolicy;
    result.multiSolid.referenceSolidCount = static_cast<int>(loadedRef.solids.size());
    result.multiSolid.candidateSolidCount = static_cast<int>(loadedCand.solids.size());
    result.timings.totalMs = ElapsedMs(totalStart);
    return result;
  }

  if (config.multiSolidPolicy == MultiSolidPolicy::CollectionOnly) {
    result.status = CompareStatus::UnsupportedShape;
    result.reason = "multi-solid collection policy is not implemented yet";
    result.decisionPath = "input_unsupported_policy";
    result.multiSolid.allowed = config.allowMultipleSolids;
    result.multiSolid.executed = false;
    result.multiSolid.enabled = false;
    result.multiSolid.policy = config.multiSolidPolicy;
    result.multiSolid.referenceSolidCount = static_cast<int>(loadedRef.solids.size());
    result.multiSolid.candidateSolidCount = static_cast<int>(loadedCand.solids.size());
    result.timings.totalMs = ElapsedMs(totalStart);
    return result;
  }

  result.multiSolid.allowed = true;
  result.multiSolid.executed = true;
  result.multiSolid.enabled = true;
  result.multiSolid.policy = config.multiSolidPolicy;
  result.multiSolid.referenceSolidCount = static_cast<int>(loadedRef.solids.size());
  result.multiSolid.candidateSolidCount = static_cast<int>(loadedCand.solids.size());
  result.globalMetricsExecuted = false;

  const double diagScale = BoundsDiagonal(result.reference.boundsMm);
  const double scaleTol = std::max(config.distanceToleranceMm, 1.0e-4 * diagScale);
  const double effectiveCentroidTol = std::max(config.solidMatchCentroidTolMm, scaleTol);
  const double effectiveBoundsTol = std::max(config.solidMatchBoundsTolMm, scaleTol);

  std::vector<bool> candMatched(loadedCand.solids.size(), false);
  int matchedCount = 0;
  std::vector<CompareResult> pairResults;

  for (std::size_t i = 0; i < loadedRef.solids.size(); ++i) {
    const auto &refItem = loadedRef.solids[i];
    int bestCandIdx = -1;
    double bestCost = 1.0e12;
    double bestVolDiff = 0.0;
    double bestRelVolDiff = 0.0;
    double bestCentDist = 0.0;
    double bestBoundsDiff = 0.0;

    int bestRejectedIdx = -1;
    double bestRejectedCost = 1.0e12;
    double bestRejectedVolDiff = 0.0;
    double bestRejectedRelVolDiff = 0.0;
    double bestRejectedCentDist = 0.0;
    double bestRejectedBoundsDiff = 0.0;
    bool bestRejectedVolEligible = false;
    bool bestRejectedCentEligible = false;
    bool bestRejectedBoundsEligible = false;

    for (std::size_t j = 0; j < loadedCand.solids.size(); ++j) {
      if (candMatched[j]) continue;
      const auto &candItem = loadedCand.solids[j];

      const double volDiff = std::abs(refItem.audit.signedVolumeMm3 - candItem.audit.signedVolumeMm3);
      const double relVolDiff = refItem.audit.signedVolumeMm3 > 0.0 ? (volDiff / refItem.audit.signedVolumeMm3) : 0.0;
      const double centDist = PointDistance(refItem.audit.centroidMm, candItem.audit.centroidMm);
      const double boundsDiff = ComputeBoundsDifference(refItem.audit.boundsMm, candItem.audit.boundsMm);

      const bool volEligible = relVolDiff <= config.solidMatchVolumeRelTol;
      const bool centEligible = centDist <= effectiveCentroidTol;
      const bool bdsEligible = boundsDiff <= effectiveBoundsTol ||
                               (volEligible && centEligible && relVolDiff <= config.solidMatchVolumeRelTol && centDist <= effectiveCentroidTol * 0.1);
      const bool isEligible = volEligible && centEligible && bdsEligible;

      const double cost = relVolDiff * 100.0 + centDist + boundsDiff;

      if (isEligible) {
        if (cost < bestCost) {
          bestCost = cost;
          bestCandIdx = static_cast<int>(j);
          bestVolDiff = volDiff;
          bestRelVolDiff = relVolDiff;
          bestCentDist = centDist;
          bestBoundsDiff = boundsDiff;
        }
      } else {
        if (cost < bestRejectedCost) {
          bestRejectedCost = cost;
          bestRejectedIdx = static_cast<int>(j);
          bestRejectedVolDiff = volDiff;
          bestRejectedRelVolDiff = relVolDiff;
          bestRejectedCentDist = centDist;
          bestRejectedBoundsDiff = boundsDiff;
          bestRejectedVolEligible = volEligible;
          bestRejectedCentEligible = centEligible;
          bestRejectedBoundsEligible = bdsEligible;
        }
      }
    }

    if (bestCandIdx >= 0) {
      candMatched[bestCandIdx] = true;
      matchedCount++;

      std::string pairSuffix = "_pair_" + std::to_string(i) + "_" + std::to_string(bestCandIdx);
      CompareResult pairRes = CompareSolidPairInternal(
          refItem.solid, refItem.audit,
          loadedCand.solids[bestCandIdx].solid, loadedCand.solids[bestCandIdx].audit,
          config, outputDirectory, pairSuffix);
      pairResults.push_back(pairRes);

      SolidMatchRecord matchRec;
      matchRec.referenceSolidId = refItem.id;
      matchRec.candidateSolidId = loadedCand.solids[bestCandIdx].id;
      matchRec.referenceIndex = static_cast<int>(i);
      matchRec.candidateIndex = bestCandIdx;
      matchRec.matchStatus = MatchStatus::Matched;
      matchRec.status = pairRes.status;
      matchRec.reason = pairRes.reason;
      matchRec.volumeDifferenceMm3 = bestVolDiff;
      matchRec.relativeVolumeDifference = bestRelVolDiff;
      matchRec.centroidDistanceMm = bestCentDist;
      matchRec.boundsDifferenceMm = bestBoundsDiff;
      matchRec.volumeEligible = true;
      matchRec.centroidEligible = true;
      matchRec.boundsEligible = true;
      matchRec.volumeTolerance = config.solidMatchVolumeRelTol;
      matchRec.centroidToleranceMm = effectiveCentroidTol;
      matchRec.boundsToleranceMm = effectiveBoundsTol;
      result.multiSolid.solidMatches.push_back(matchRec);
    } else {
      SolidMatchRecord matchRec;
      matchRec.referenceSolidId = refItem.id;
      matchRec.referenceIndex = static_cast<int>(i);
      matchRec.matchStatus = MatchStatus::Unmatched;
      matchRec.status = CompareStatus::Different;
      matchRec.volumeTolerance = config.solidMatchVolumeRelTol;
      matchRec.centroidToleranceMm = effectiveCentroidTol;
      matchRec.boundsToleranceMm = effectiveBoundsTol;

      if (bestRejectedIdx >= 0) {
        matchRec.candidateSolidId = loadedCand.solids[bestRejectedIdx].id;
        matchRec.candidateIndex = bestRejectedIdx;
        matchRec.volumeDifferenceMm3 = bestRejectedVolDiff;
        matchRec.relativeVolumeDifference = bestRejectedRelVolDiff;
        matchRec.centroidDistanceMm = bestRejectedCentDist;
        matchRec.boundsDifferenceMm = bestRejectedBoundsDiff;
        matchRec.volumeEligible = bestRejectedVolEligible;
        matchRec.centroidEligible = bestRejectedCentEligible;
        matchRec.boundsEligible = bestRejectedBoundsEligible;

        if (!bestRejectedVolEligible) matchRec.rejectReasons.push_back("VOLUME_THRESHOLD_EXCEEDED");
        if (!bestRejectedCentEligible) matchRec.rejectReasons.push_back("CENTROID_THRESHOLD_EXCEEDED");
        if (!bestRejectedBoundsEligible) matchRec.rejectReasons.push_back("BOUNDS_THRESHOLD_EXCEEDED");

        matchRec.reason = "best candidate rejected due to threshold failure";
      } else {
        matchRec.candidateSolidId = "";
        matchRec.candidateIndex = -1;
        matchRec.reason = "no available candidate solid for matching";
      }

      result.multiSolid.solidMatches.push_back(matchRec);
      result.multiSolid.unmatchedReferenceSolidIds.push_back(refItem.id);
    }
  }

  for (std::size_t j = 0; j < loadedCand.solids.size(); ++j) {
    if (!candMatched[j]) {
      const auto &candItem = loadedCand.solids[j];
      SolidMatchRecord matchRec;
      matchRec.referenceSolidId = "";
      matchRec.candidateSolidId = candItem.id;
      matchRec.referenceIndex = -1;
      matchRec.candidateIndex = static_cast<int>(j);
      matchRec.matchStatus = MatchStatus::Unmatched;
      matchRec.status = CompareStatus::Different;
      matchRec.reason = "unmatched candidate solid";
      matchRec.volumeTolerance = config.solidMatchVolumeRelTol;
      matchRec.centroidToleranceMm = effectiveCentroidTol;
      matchRec.boundsToleranceMm = effectiveBoundsTol;
      result.multiSolid.solidMatches.push_back(matchRec);
      result.multiSolid.unmatchedCandidateSolidIds.push_back(candItem.id);
    }
  }

  result.multiSolid.matchedSolidCount = matchedCount;
  result.multiSolid.unmatchedReferenceSolidCount = static_cast<int>(result.multiSolid.unmatchedReferenceSolidIds.size());
  result.multiSolid.unmatchedCandidateSolidCount = static_cast<int>(result.multiSolid.unmatchedCandidateSolidIds.size());

  if (result.multiSolid.unmatchedReferenceSolidCount > 0 || result.multiSolid.unmatchedCandidateSolidCount > 0) {
    result.status = CompareStatus::Different;
    result.reason = "multi-solid matching failed: reference_solids=" + std::to_string(result.multiSolid.referenceSolidCount) +
                    ", candidate_solids=" + std::to_string(result.multiSolid.candidateSolidCount) +
                    ", matched=" + std::to_string(matchedCount);
    result.decisionPath = "multi_solid_unmatched";
  } else {
    bool allEqual = true;
    bool anyDifferent = false;
    bool anyLikelyEqual = false;
    std::string firstDiffReason;

    for (const auto &pairRes : pairResults) {
      if (pairRes.status != CompareStatus::Equal) {
        allEqual = false;
      }
      if (pairRes.status == CompareStatus::Different) {
        anyDifferent = true;
        if (firstDiffReason.empty()) firstDiffReason = pairRes.reason;
      } else if (pairRes.status == CompareStatus::LikelyEqual) {
        anyLikelyEqual = true;
      }
    }

    if (allEqual) {
      result.status = CompareStatus::Equal;
      result.reason = "all " + std::to_string(matchedCount) + " solid pairs are EQUAL";
      result.decisionPath = "multi_solid_pairs_equal";
    } else if (anyDifferent) {
      result.status = CompareStatus::Different;
      result.reason = "solid pair comparison failed: " + firstDiffReason;
      result.decisionPath = "multi_solid_pair_different";
    } else if (anyLikelyEqual) {
      result.status = CompareStatus::LikelyEqual;
      result.reason = "all solid pairs matched (with likely equal pairs)";
      result.decisionPath = "multi_solid_pairs_likely_equal";
    } else {
      result.status = CompareStatus::Indeterminate;
      result.reason = "solid pair comparison indeterminate";
      result.decisionPath = "multi_solid_pairs_indeterminate";
    }
  }

  if (config.exportStl && !outputDirectory.empty()) {
    BRep_Builder builder;
    TopoDS_Compound compRef;
    builder.MakeCompound(compRef);
    for (const auto &item : loadedRef.solids) {
      if (!item.solid.IsNull()) builder.Add(compRef, item.solid);
    }

    TopoDS_Compound compCand;
    builder.MakeCompound(compCand);
    for (const auto &item : loadedCand.solids) {
      if (!item.solid.IsNull()) builder.Add(compCand, item.solid);
    }

    const auto refStl = outputDirectory / "reference_base.stl";
    if (ExportShapeStl(compRef, refStl)) {
      result.artifacts.items.push_back({"reference_base", "reference_base.stl", "STL", true, ""});
    }

    const auto candStl = outputDirectory / "candidate_base.stl";
    if (ExportShapeStl(compCand, candStl)) {
      result.artifacts.items.push_back({"candidate_base", "candidate_base.stl", "STL", true, ""});
    }

    for (const auto &pairRes : pairResults) {
      for (const auto &art : pairRes.artifacts.items) {
        result.artifacts.items.push_back(art);
      }
    }
  }

  result.timings.totalMs = ElapsedMs(totalStart);
  return result;
}

std::string ToJson(const CompareResult &result) {
  json root = json::object();
  root["schema_version"] = 2;

  // 1. overall
  json overall = json::object();
  overall["status"] = ToString(result.status);
  overall["exit_code"] = ExitCode(result.status);
  overall["reason"] = result.reason;
  overall["decision_path"] = result.decisionPath;
  overall["boolean_executed"] = result.booleanExecuted;
  root["overall"] = overall;

  // 2. configuration
  const double diagScale = BoundsDiagonal(result.reference.boundsMm);
  const double effectiveDistTol = std::max(result.thresholds.distanceToleranceMm, 1.0e-4 * diagScale);
  const double refVol = result.reference.signedVolumeMm3;
  const double effectiveAbsVolTol = std::max(result.thresholds.absoluteVolumeToleranceMm3, result.thresholds.relativeVolumeTolerance * refVol);

  json configJson = json::object();
  configJson["distance_tolerance_mm"] = result.thresholds.distanceToleranceMm;
  configJson["effective_distance_tolerance_mm"] = effectiveDistTol;
  configJson["absolute_volume_tolerance_mm3"] = result.thresholds.absoluteVolumeToleranceMm3;
  configJson["relative_volume_tolerance"] = result.thresholds.relativeVolumeTolerance;
  configJson["effective_volume_tolerance_mm3"] = effectiveAbsVolTol;
  configJson["boolean_fuzzy_tolerance_mm"] = result.thresholds.booleanFuzzyToleranceMm;
  configJson["normalization_enabled"] = result.thresholds.enableSameDomainNormalization;
  configJson["normalization_linear_tolerance_mm"] = result.thresholds.normalizationLinearToleranceMm;
  configJson["normalization_angular_tolerance_rad"] = result.thresholds.normalizationAngularToleranceRad;
  configJson["normalized_fast_path_enabled"] = result.thresholds.enableNormalizedFastPath;
  configJson["ambiguous_match_margin"] = result.thresholds.ambiguousMatchMargin;
  configJson["allow_multiple_solids"] = result.thresholds.allowMultipleSolids;
  configJson["multi_solid_policy"] = ToString(result.thresholds.multiSolidPolicy);
  configJson["solid_match_volume_rel_tol"] = result.thresholds.solidMatchVolumeRelTol;
  configJson["solid_match_centroid_tol_mm"] = result.thresholds.solidMatchCentroidTolMm;
  configJson["solid_match_bounds_tol_mm"] = result.thresholds.solidMatchBoundsTolMm;
  root["configuration"] = configJson;

  // 3. multi_solid
  json multiNode = json::object();
  multiNode["allowed"] = result.multiSolid.allowed;
  multiNode["executed"] = result.multiSolid.executed;
  multiNode["enabled"] = result.multiSolid.enabled;
  multiNode["policy"] = ToString(result.multiSolid.policy);
  multiNode["reference_solid_count"] = result.multiSolid.referenceSolidCount;
  multiNode["candidate_solid_count"] = result.multiSolid.candidateSolidCount;
  multiNode["matched_solid_count"] = result.multiSolid.matchedSolidCount;
  multiNode["unmatched_reference_solid_count"] = result.multiSolid.unmatchedReferenceSolidCount;
  multiNode["unmatched_candidate_solid_count"] = result.multiSolid.unmatchedCandidateSolidCount;

  json matchesArray = json::array();
  for (const auto &m : result.multiSolid.solidMatches) {
    json item = json::object();
    item["reference_solid_id"] = m.referenceSolidId;
    item["candidate_solid_id"] = m.candidateSolidId;
    item["reference_index"] = m.referenceIndex;
    item["candidate_index"] = m.candidateIndex;
    item["match_status"] = ToString(m.matchStatus);
    item["status"] = ToString(m.status);
    item["reason"] = m.reason;
    item["volume_difference_mm3"] = m.volumeDifferenceMm3;
    item["relative_volume_difference"] = m.relativeVolumeDifference;
    item["centroid_distance_mm"] = m.centroidDistanceMm;
    item["bounds_difference_mm"] = m.boundsDifferenceMm;
    item["volume_eligible"] = m.volumeEligible;
    item["centroid_eligible"] = m.centroidEligible;
    item["bounds_eligible"] = m.boundsEligible;
    item["volume_tolerance"] = m.volumeTolerance;
    item["centroid_tolerance_mm"] = m.centroidToleranceMm;
    item["bounds_tolerance_mm"] = m.boundsToleranceMm;
    item["reject_reasons"] = m.rejectReasons;
    matchesArray.push_back(item);
  }
  multiNode["solid_matches"] = matchesArray;
  multiNode["unmatched_reference_solid_ids"] = result.multiSolid.unmatchedReferenceSolidIds;
  multiNode["unmatched_candidate_solid_ids"] = result.multiSolid.unmatchedCandidateSolidIds;
  root["multi_solid"] = multiNode;

  // 4. inputs
  root["global_metrics_executed"] = result.globalMetricsExecuted;

  // 4. inputs
  auto MakeInputNode = [](const InputAudit &audit) {
    json node = json::object();
    node["path"] = audit.path;
    node["filename"] = std::filesystem::path(audit.path).filename().string();
    node["file_length_units"] = audit.fileLengthUnits;
    node["solid_count"] = audit.solidCount;
    node["shell_count"] = audit.shellCount;
    node["face_count"] = audit.faceCount;
    node["edge_count"] = audit.edgeCount;
    node["brep_valid"] = audit.brepValid;
    node["closed"] = audit.closed;
    node["volume_mm3"] = audit.signedVolumeMm3;
    node["surface_area_mm2"] = audit.surfaceAreaMm2;
    node["centroid_mm"] = {{"x", audit.centroidMm.x}, {"y", audit.centroidMm.y}, {"z", audit.centroidMm.z}};
    node["bounds_mm"] = {
        {"is_void", audit.boundsMm.isVoid},
        {"minimum", {{"x", audit.boundsMm.minimum.x}, {"y", audit.boundsMm.minimum.y}, {"z", audit.boundsMm.minimum.z}}},
        {"maximum", {{"x", audit.boundsMm.maximum.x}, {"y", audit.boundsMm.maximum.y}, {"z", audit.boundsMm.maximum.z}}}};
    node["load_diagnostics"] = audit.loadDiagnostics;
    node["transfer_diagnostics"] = audit.transferDiagnostics;
    return node;
  };
  json inputs = json::object();
  inputs["reference"] = MakeInputNode(result.reference);
  inputs["candidate"] = MakeInputNode(result.candidate);
  root["inputs"] = inputs;

  // 5. normalization
  auto MakeNormNode = [](const NormalizationAudit &audit) {
    json node = json::object();
    node["enabled"] = audit.enabled;
    node["succeeded"] = audit.succeeded;
    node["used_normalized_shape"] = audit.usedNormalizedShape;
    node["faces_before"] = audit.faceCountBefore;
    node["faces_after"] = audit.faceCountAfter;
    node["edges_before"] = audit.edgeCountBefore;
    node["edges_after"] = audit.edgeCountAfter;
    node["comparable_edges_before"] = audit.comparableEdgeCountBefore;
    node["comparable_edges_after"] = audit.comparableEdgeCountAfter;
    node["volume_before_mm3"] = audit.volumeBeforeMm3;
    node["volume_after_mm3"] = audit.volumeAfterMm3;
    node["relative_volume_drift"] = audit.relativeVolumeDrift;
    node["face_mapping_complete"] = audit.faceMappingComplete;
    node["edge_mapping_complete"] = audit.edgeMappingComplete;
    node["mapping_complete"] = audit.mappingComplete;

    json faceStats = json::array();
    for (const auto &st : audit.faceTypes) {
      faceStats.push_back({{"type", st.type}, {"count", st.count}, {"total_measure", st.totalMeasure}});
    }
    node["face_type_statistics"] = faceStats;

    json edgeStats = json::array();
    for (const auto &st : audit.edgeTypes) {
      edgeStats.push_back({{"type", st.type}, {"count", st.count}, {"total_measure", st.totalMeasure}});
    }
    node["edge_type_statistics"] = edgeStats;

    json facesArr = json::array();
    for (const auto &f : audit.faces) {
      facesArr.push_back({
          {"id", f.id},
          {"visual_index", f.visualIndex},
          {"surface_type", f.surfaceType},
          {"area_mm2", f.areaMm2},
          {"centroid_mm", {{"x", f.centroidMm.x}, {"y", f.centroidMm.y}, {"z", f.centroidMm.z}}},
          {"bounds_mm",
           {{"is_void", f.boundsMm.isVoid},
            {"minimum", {{"x", f.boundsMm.minimum.x}, {"y", f.boundsMm.minimum.y}, {"z", f.boundsMm.minimum.z}}},
            {"maximum", {{"x", f.boundsMm.maximum.x}, {"y", f.boundsMm.maximum.y}, {"z", f.boundsMm.maximum.z}}}}},
          {"source_face_ids", f.sourceFaceIds},
          {"boundary_edge_ids", f.boundaryEdgeIds},
          {"source_count", f.sourceCount},
          {"merged", f.merged},
      });
    }
    node["faces"] = facesArr;

    json edgesArr = json::array();
    for (const auto &e : audit.edges) {
      edgesArr.push_back({
          {"id", e.id},
          {"visual_index", e.visualIndex},
          {"curve_type", e.curveType},
          {"length_mm", e.lengthMm},
          {"centroid_mm", {{"x", e.centroidMm.x}, {"y", e.centroidMm.y}, {"z", e.centroidMm.z}}},
          {"bounds_mm",
           {{"is_void", e.boundsMm.isVoid},
            {"minimum", {{"x", e.boundsMm.minimum.x}, {"y", e.boundsMm.minimum.y}, {"z", e.boundsMm.minimum.z}}},
            {"maximum", {{"x", e.boundsMm.maximum.x}, {"y", e.boundsMm.maximum.y}, {"z", e.boundsMm.maximum.z}}}}},
          {"source_edge_ids", e.sourceEdgeIds},
          {"source_count", e.sourceCount},
          {"merged", e.merged},
          {"closed", e.closed},
          {"comparable", e.comparable},
          {"comparable_index", e.comparable ? json(e.comparableIndex) : json(nullptr)},
          {"comparison_role", ToString(e.comparisonRole)},
          {"exclusion_reason", e.exclusionReason.empty() ? json(nullptr) : json(e.exclusionReason)},
      });
    }
    node["edges"] = edgesArr;

    json removedArr = json::array();
    for (const auto &re : audit.removedEdges) {
      removedArr.push_back({{"source_edge_id", re.sourceEdgeId}, {"reason", re.reason}});
    }
    node["removed_edges"] = removedArr;

    node["elapsed_ms"] = audit.elapsedMs;
    node["warning"] = audit.warning;
    return node;
  };

  json normJson = json::object();
  const bool pairUsable = result.referenceNormalization.succeeded && result.candidateNormalization.succeeded;
  normJson["pair_usable"] = pairUsable;
  normJson["reference"] = MakeNormNode(result.referenceNormalization);
  normJson["candidate"] = MakeNormNode(result.candidateNormalization);
  root["normalization"] = normJson;

  // 6. matches
  auto MakeMatchCollectionNode = [](const MatchCollection &col) {
    json node = json::object();
    json summary = json::object();
    summary["attempted"] = col.attempted;
    summary["reference_count"] = col.referenceCount;
    summary["candidate_count"] = col.candidateCount;
    summary["matched_count"] = col.matchedCount;
    summary["ambiguous_count"] = col.ambiguousCount;
    summary["unmatched_reference_count"] = col.unmatchedReferenceIds.size();
    summary["unmatched_candidate_count"] = col.unmatchedCandidateIds.size();
    summary["type_histogram_equal"] = col.typeHistogramEqual;
    summary["all_matched"] = col.allMatched;
    summary["elapsed_ms"] = col.elapsedMs;
    node["summary"] = summary;

    json itemsArr = json::array();
    for (const auto &item : col.items) {
      json itemNode = {
          {"id", item.id},
          {"reference_id", item.referenceId},
          {"candidate_id", item.candidateId},
          {"geometry_type", item.geometryType},
          {"status", ToString(item.status)},
          {"verification_level", ToString(item.verificationLevel)},
          {"reason_codes", item.reasonCodes},
      };
      if (item.score.has_value()) {
        itemNode["score"] = *item.score;
      } else {
        itemNode["score"] = nullptr;
      }
      if (item.metrics.has_value()) {
        itemNode["metrics"] = {
            {"measure_difference", item.metrics->measureDifference},
            {"relative_measure_difference", item.metrics->relativeMeasureDifference},
            {"centroid_distance_mm", item.metrics->centroidDistanceMm},
            {"bounds_difference_mm", item.metrics->boundsDifferenceMm}};
      } else {
        itemNode["metrics"] = nullptr;
      }
      itemsArr.push_back(itemNode);
    }
    node["items"] = itemsArr;
    node["unmatched_reference_ids"] = col.unmatchedReferenceIds;
    node["unmatched_candidate_ids"] = col.unmatchedCandidateIds;
    return node;
  };

  json matchesJson = json::object();
  matchesJson["faces"] = MakeMatchCollectionNode(result.normalizedTopology.faces);
  matchesJson["edges"] = MakeMatchCollectionNode(result.normalizedTopology.edges);
  matchesJson["edge_audit_errors"] = result.normalizedTopology.edgeAuditErrors;
  matchesJson["fast_path"] = {
      {"enabled", result.normalizedTopology.fastPath.enabled},
      {"eligible", result.normalizedTopology.fastPath.eligible},
      {"used", result.normalizedTopology.fastPath.used},
      {"block_reasons", result.normalizedTopology.fastPath.blockReasons}};
  root["matches"] = matchesJson;

  // 6. differences
  json diffs = json::object();
  diffs["missing_material"] = {
      {"succeeded", result.missingMaterial.succeeded},
      {"volume_mm3", result.missingMaterial.volumeMm3},
      {"component_count", result.missingMaterial.componentCount},
      {"kernel_report", result.missingMaterial.report},
  };
  diffs["added_material"] = {
      {"succeeded", result.addedMaterial.succeeded},
      {"volume_mm3", result.addedMaterial.volumeMm3},
      {"component_count", result.addedMaterial.componentCount},
      {"kernel_report", result.addedMaterial.report},
  };
  root["differences"] = diffs;

  // 7. metrics
  json metricsJson = json::object();
  metricsJson["absolute_input_volume_difference_mm3"] = result.absoluteInputVolumeDifferenceMm3;
  metricsJson["relative_input_volume_difference"] = result.relativeInputVolumeDifference;
  metricsJson["centroid_distance_mm"] = result.centroidDistanceMm;
  metricsJson["maximum_bounds_difference_mm"] = result.maximumBoundsDifferenceMm;
  metricsJson["symmetric_difference_volume_mm3"] = result.symmetricDifferenceVolumeMm3;
  metricsJson["symmetric_difference_relative"] = result.symmetricDifferenceRelative;
  root["metrics"] = metricsJson;

  json boolCons = json::object();
  boolCons["signed_input_volume_diff_mm3"] = result.booleanConsistency.signedInputVolumeDiffMm3;
  boolCons["signed_boolean_volume_diff_mm3"] = result.booleanConsistency.signedBooleanVolumeDiffMm3;
  boolCons["conservation_error_mm3"] = result.booleanConsistency.conservationErrorMm3;
  boolCons["relative_conservation_error"] = result.booleanConsistency.relativeConservationError;
  boolCons["conservation_passed"] = result.booleanConsistency.conservationPassed;
  boolCons["boolean_result_valid"] = result.booleanConsistency.booleanResultValid;
  boolCons["invalid_reason"] = result.booleanConsistency.invalidReason;
  root["boolean_consistency"] = boolCons;

  // 8. checks
  const bool volPass = (result.absoluteInputVolumeDifferenceMm3 <= effectiveAbsVolTol) &&
                       (result.relativeInputVolumeDifference <= result.thresholds.relativeVolumeTolerance);
  const bool centroidPass = result.centroidDistanceMm <= effectiveDistTol;
  const bool boundsPass = result.maximumBoundsDifferenceMm <= effectiveDistTol;
  const bool boolPass = (!result.booleanExecuted) ||
                        (result.missingMaterial.volumeMm3 <= effectiveAbsVolTol && result.addedMaterial.volumeMm3 <= effectiveAbsVolTol);

  const auto &faceCol = result.normalizedTopology.faces;
  const auto &edgeCol = result.normalizedTopology.edges;

  auto FormatDouble = [](double val, int precision = 4) -> std::string {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << val;
    return ss.str();
  };

  json checks = json::array();
  checks.push_back({
      {"id", "input_volume_difference"},
      {"label", "输入实体体积差"},
      {"actual", result.absoluteInputVolumeDifferenceMm3},
      {"actual_text", FormatDouble(result.absoluteInputVolumeDifferenceMm3, 4) + " mm³"},
      {"unit", "mm3"},
      {"relative", result.relativeInputVolumeDifference},
      {"relative_percent", result.relativeInputVolumeDifference * 100.0},
      {"limit", effectiveAbsVolTol},
      {"limit_text", FormatDouble(effectiveAbsVolTol, 4) + " mm³"},
      {"limit_unit", "mm3"},
      {"available", true},
      {"diagnostic", false},
      {"passed", volPass},
  });
  checks.push_back({
      {"id", "centroid_distance"},
      {"label", "质心距离"},
      {"actual", result.centroidDistanceMm},
      {"actual_text", FormatDouble(result.centroidDistanceMm, 4) + " mm"},
      {"unit", "mm"},
      {"relative", nullptr},
      {"relative_percent", nullptr},
      {"limit", effectiveDistTol},
      {"limit_text", FormatDouble(effectiveDistTol, 4) + " mm"},
      {"limit_unit", "mm"},
      {"available", true},
      {"diagnostic", false},
      {"passed", centroidPass},
  });
  checks.push_back({
      {"id", "bounds_difference"},
      {"label", "包围盒尺寸差异"},
      {"actual", result.maximumBoundsDifferenceMm},
      {"actual_text", FormatDouble(result.maximumBoundsDifferenceMm, 4) + " mm"},
      {"unit", "mm"},
      {"relative", nullptr},
      {"relative_percent", nullptr},
      {"limit", effectiveDistTol},
      {"limit_text", FormatDouble(effectiveDistTol, 4) + " mm"},
      {"limit_unit", "mm"},
      {"available", true},
      {"diagnostic", false},
      {"passed", boundsPass},
  });
  checks.push_back({
      {"id", "face_descriptor_match"},
      {"label", "归一化面描述符匹配"},
      {"actual", faceCol.matchedCount},
      {"actual_text", std::to_string(faceCol.matchedCount) + " / " + std::to_string(faceCol.referenceCount)},
      {"unit", "faces"},
      {"relative", faceCol.referenceCount > 0 ? (static_cast<double>(faceCol.matchedCount) / faceCol.referenceCount) : 1.0},
      {"relative_percent", faceCol.referenceCount > 0 ? (faceCol.matchedCount * 100.0 / faceCol.referenceCount) : 100.0},
      {"limit", faceCol.referenceCount},
      {"limit_text", std::to_string(faceCol.referenceCount) + " / " + std::to_string(faceCol.referenceCount)},
      {"limit_unit", "faces"},
      {"available", faceCol.attempted},
      {"diagnostic", true},
      {"passed", faceCol.allMatched},
  });
  checks.push_back({
      {"id", "edge_descriptor_match"},
      {"label", "归一化边描述符匹配"},
      {"actual", edgeCol.matchedCount},
      {"actual_text", std::to_string(edgeCol.matchedCount) + " / " + std::to_string(edgeCol.referenceCount)},
      {"unit", "edges"},
      {"relative", edgeCol.referenceCount > 0 ? (static_cast<double>(edgeCol.matchedCount) / edgeCol.referenceCount) : 1.0},
      {"relative_percent", edgeCol.referenceCount > 0 ? (edgeCol.matchedCount * 100.0 / edgeCol.referenceCount) : 100.0},
      {"limit", edgeCol.referenceCount},
      {"limit_text", std::to_string(edgeCol.referenceCount) + " / " + std::to_string(edgeCol.referenceCount)},
      {"limit_unit", "edges"},
      {"available", edgeCol.attempted},
      {"diagnostic", true},
      {"passed", edgeCol.allMatched},
  });
  checks.push_back({
      {"id", "boolean_cut_residual"},
      {"label", "布尔减法残留体积"},
      {"actual", result.symmetricDifferenceVolumeMm3},
      {"actual_text", FormatDouble(result.symmetricDifferenceVolumeMm3, 4) + " mm³"},
      {"unit", "mm3"},
      {"relative", result.symmetricDifferenceRelative},
      {"relative_percent", result.symmetricDifferenceRelative * 100.0},
      {"limit", effectiveAbsVolTol},
      {"limit_text", FormatDouble(effectiveAbsVolTol, 4) + " mm³"},
      {"limit_unit", "mm3"},
      {"available", result.booleanExecuted},
      {"diagnostic", false},
      {"passed", boolPass},
  });
  root["checks"] = checks;

  // 9. timings_ms
  json timingsJson = json::object();
  timingsJson["load_reference"] = result.timings.loadReferenceMs;
  timingsJson["load_candidate"] = result.timings.loadCandidateMs;
  timingsJson["normalize_reference"] = result.timings.normalizeReferenceMs;
  timingsJson["normalize_candidate"] = result.timings.normalizeCandidateMs;
  timingsJson["descriptor_build"] = result.timings.descriptorBuildMs;
  timingsJson["face_match"] = result.timings.faceMatchMs;
  timingsJson["edge_match"] = result.timings.edgeMatchMs;
  timingsJson["boolean_ab"] = result.timings.booleanAbMs;
  timingsJson["boolean_ba"] = result.timings.booleanBaMs;
  timingsJson["artifact_export"] = result.timings.artifactExportMs;
  timingsJson["total"] = result.timings.totalMs;
  root["timings_ms"] = timingsJson;

  // 10. artifacts
  json artJson = json::object();
  for (const auto &item : result.artifacts.items) {
    json info = json::object();
    info["path"] = item.relativePath;
    info["format"] = item.format;
    info["available"] = item.available;
    if (!item.entityIndexArray.empty()) {
      info["entity_index_array"] = item.entityIndexArray;
    }
    artJson[item.key] = info;
  }
  root["artifacts"] = artJson;

  return root.dump(2) + '\n';
}

std::string ToHumanSummary(const CompareResult &result) {
  std::ostringstream ss;

  const double diagScale = BoundsDiagonal(result.reference.boundsMm);
  const double effectiveDistTol = std::max(result.thresholds.distanceToleranceMm, 1.0e-4 * diagScale);
  const double refVol = result.reference.signedVolumeMm3;
  const double effectiveAbsVolTol = std::max(result.thresholds.absoluteVolumeToleranceMm3, result.thresholds.relativeVolumeTolerance * refVol);

  const std::string refName = std::filesystem::path(result.reference.path).filename().string();
  const std::string candName = std::filesystem::path(result.candidate.path).filename().string();

  const bool volPass = (result.absoluteInputVolumeDifferenceMm3 <= effectiveAbsVolTol) &&
                       (result.relativeInputVolumeDifference <= result.thresholds.relativeVolumeTolerance);
  const bool centroidPass = result.centroidDistanceMm <= effectiveDistTol;
  const bool boundsPass = result.maximumBoundsDifferenceMm <= effectiveDistTol;
  const bool boolPass = (!result.booleanExecuted) ||
                        (result.missingMaterial.volumeMm3 <= effectiveAbsVolTol && result.addedMaterial.volumeMm3 <= effectiveAbsVolTol);

  const bool pairUsable = result.referenceNormalization.succeeded && result.candidateNormalization.succeeded;
  const bool detectedMulti = (result.reference.solidCount > 1 || result.candidate.solidCount > 1);

  ss << "============================================================\n"
     << "STEP GEOMETRY COMPARISON\n"
     << "============================================================\n\n"
     << "COMPARE MODE\n"
     << "  Detected    : " << (detectedMulti ? "multi-solid" : "single-solid") << "\n"
     << "  Allow multi : " << (result.thresholds.allowMultipleSolids ? "YES" : "NO") << "\n"
     << "  Selected    : " << (result.multiSolid.executed ? ToString(result.multiSolid.policy) : (detectedMulti ? "strict" : "single-solid")) << "\n\n"
     << "INPUT\n"
     << "  Reference : " << refName << "\n"
     << "  Candidate : " << candName << "\n\n";

  if (result.multiSolid.executed) {
    ss << "MULTI-SOLID MATCH\n"
       << "  Policy     : " << ToString(result.multiSolid.policy) << "\n"
       << "  Reference  : " << result.multiSolid.referenceSolidCount << " solids\n"
       << "  Candidate  : " << result.multiSolid.candidateSolidCount << " solids\n"
       << "  Matched    : " << result.multiSolid.matchedSolidCount << "\n"
       << "  Unmatched  : ref=" << result.multiSolid.unmatchedReferenceSolidCount
       << ", cand=" << result.multiSolid.unmatchedCandidateSolidCount << "\n";
    for (const auto &m : result.multiSolid.solidMatches) {
      if (m.matchStatus == MatchStatus::Unmatched && !m.rejectReasons.empty()) {
        ss << "  [Match Rejected] Ref: " << m.referenceSolidId << ", Best Cand: " << m.candidateSolidId << "\n"
           << "    Vol rel diff: " << (m.relativeVolumeDifference * 100.0) << "% <= " << (m.volumeTolerance * 100.0) << "% [" << (m.volumeEligible ? "PASS" : "FAIL") << "]\n"
           << "    Centroid dist: " << m.centroidDistanceMm << " mm <= " << m.centroidToleranceMm << " mm [" << (m.centroidEligible ? "PASS" : "FAIL") << "]\n"
           << "    Bounds diff  : " << m.boundsDifferenceMm << " mm <= " << m.boundsToleranceMm << " mm [" << (m.boundsEligible ? "PASS" : "FAIL") << "]\n";
      }
    }
    ss << "\n";
  }

  ss << "NORMALIZATION\n"
     << "  Pair usable    : " << (pairUsable ? "YES" : "NO") << "\n"
     << "  Reference face : " << result.reference.faceCount << " -> " << result.referenceNormalization.faceCountAfter << "\n"
     << "  Candidate face : " << result.candidate.faceCount << " -> " << result.candidateNormalization.faceCountAfter << "\n"
     << "  Reference edge : " << result.reference.edgeCount << " -> " << result.referenceNormalization.edgeCountAfter << "\n"
     << "  Candidate edge : " << result.candidate.edgeCount << " -> " << result.candidateNormalization.edgeCountAfter << "\n"
     << "  Mapping        : " << (result.referenceNormalization.mappingComplete ? "COMPLETE" : "INCOMPLETE") << "\n\n"
     << "DESCRIPTOR MATCH\n"
     << "  Faces     : " << result.normalizedTopology.faces.matchedCount << " / " << result.normalizedTopology.faces.referenceCount << "\n"
     << "  Edges     : " << result.normalizedTopology.edges.matchedCount << " / " << result.normalizedTopology.edges.referenceCount << "\n"
     << "  Ambiguous : faces=" << result.normalizedTopology.faces.ambiguousCount << ", edges=" << result.normalizedTopology.edges.ambiguousCount << "\n"
     << "  Unmatched : faces=" << result.normalizedTopology.faces.unmatchedReferenceIds.size()
     << ", edges=" << result.normalizedTopology.edges.unmatchedReferenceIds.size() << "\n\n"
     << "GLOBAL METRICS\n";

  if (result.globalMetricsExecuted) {
    ss << "  Volume diff : " << result.absoluteInputVolumeDifferenceMm3 << " mm³ ("
       << (result.relativeInputVolumeDifference * 100.0) << "%) [" << (volPass ? "PASS" : "FAIL") << "]\n"
       << "  Centroid dist : " << result.centroidDistanceMm << " mm [" << (centroidPass ? "PASS" : "FAIL") << "]\n"
       << "  Bounds diff   : " << result.maximumBoundsDifferenceMm << " mm [" << (boundsPass ? "PASS" : "FAIL") << "]\n\n";
  } else {
    ss << "  Volume diff   : N/A [NOT EXECUTED]\n"
       << "  Centroid dist : N/A [NOT EXECUTED]\n"
       << "  Bounds diff   : N/A [NOT EXECUTED]\n\n";
  }

  ss << "BOOLEAN\n"
     << "  Executed   : " << (result.booleanExecuted ? "YES" : "NO") << "\n";
  if (result.booleanExecuted) {
    ss << "  A-B volume : " << result.missingMaterial.volumeMm3 << " mm³\n"
       << "  B-A volume : " << result.addedMaterial.volumeMm3 << " mm³\n"
       << "  Residual   : " << result.symmetricDifferenceVolumeMm3 << " mm³ [" << (boolPass ? "PASS" : "FAIL") << "]\n\n";
  } else {
    ss << "  A-B volume : N/A\n"
       << "  B-A volume : N/A\n"
       << "  Residual   : N/A\n\n";
  }

  ss << "TIMING\n"
     << "  Load       : " << (result.timings.loadReferenceMs + result.timings.loadCandidateMs) << " ms\n"
     << "  Normalize  : " << (result.timings.normalizeReferenceMs + result.timings.normalizeCandidateMs) << " ms\n"
     << "  Match      : " << result.normalizedTopology.elapsedMs << " ms\n"
     << "  Boolean    : " << (result.timings.booleanAbMs + result.timings.booleanBaMs) << " ms\n"
     << "  Export     : " << result.timings.artifactExportMs << " ms\n"
     << "  Total      : " << result.timings.totalMs << " ms\n\n"
     << "RESULT\n"
     << "  Status        : " << ToString(result.status) << "\n"
     << "  Decision path : " << result.decisionPath << "\n"
     << "  Reason        : " << result.reason << "\n"
     << "============================================================\n";

  return ss.str();
}

bool WriteResultJson(const std::filesystem::path &outputDirectory,
                     const CompareResult &result, std::string &error) {
  try {
    std::error_code ec;
    std::filesystem::create_directories(outputDirectory, ec);

    const auto finalPath = outputDirectory / "result.json";
    const auto tempPath = outputDirectory / "result.json.tmp";

    {
      std::ofstream out(tempPath, std::ios::binary | std::ios::trunc);
      if (!out.is_open()) {
        error = "cannot open temporary result file: " + tempPath.string();
        return false;
      }

      out << ToJson(result);
      out.flush();

      if (!out.good()) {
        error = "failed to write temporary result file: " + tempPath.string();
        return false;
      }
    }

    std::filesystem::remove(finalPath, ec);
    ec.clear();
    std::filesystem::rename(tempPath, finalPath, ec);

    if (ec) {
      error = "failed to replace result.json: " + ec.message();
      return false;
    }

    return true;
  } catch (const std::exception &ex) {
    error = ex.what();
    return false;
  }
}

} // namespace cadstep
