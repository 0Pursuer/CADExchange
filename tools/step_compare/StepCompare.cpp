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
#include <gp_Pnt.hxx>
#include <json/single_include/nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <system_error>
#include <tuple>

namespace cadstep {

using json = nlohmann::ordered_json;

namespace {

enum class LoadClass {
  Ready,
  Invalid,
  Unsupported,
  InternalError,
};

struct LoadedSolid {
  LoadClass classification = LoadClass::InternalError;
  TopoDS_Solid solid;
  InputAudit audit;
  std::string reason;
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
  TopTools_IndexedMapOfShape shapes;
  TopExp::MapShapes(shape, type, shapes);
  return shapes.Extent();
}

bool AllShellsClosed(const TopoDS_Solid &solid) {
  bool foundShell = false;
  for (TopExp_Explorer explorer(solid, TopAbs_SHELL); explorer.More();
       explorer.Next()) {
    foundShell = true;
    if (!BRep_Tool::IsClosed(TopoDS::Shell(explorer.Current()))) {
      return false;
    }
  }
  return foundShell;
}

std::vector<std::string>
ToStrings(const TColStd_SequenceOfAsciiString &values) {
  std::vector<std::string> result;
  result.reserve(static_cast<std::size_t>(values.Length()));
  for (Standard_Integer index = 1; index <= values.Length(); ++index) {
    result.emplace_back(values.Value(index).ToCString());
  }
  return result;
}

Point3 ToPoint3(const gp_Pnt &point) {
  return Point3{point.X(), point.Y(), point.Z()};
}

Bounds3 ComputeBounds(const TopoDS_Shape &shape) {
  Bnd_Box box;
  BRepBndLib::AddOptimal(shape, box, Standard_False, Standard_False);

  Bounds3 result;
  result.isVoid = box.IsVoid();
  if (!result.isVoid) {
    Standard_Real xMin = 0.0;
    Standard_Real yMin = 0.0;
    Standard_Real zMin = 0.0;
    Standard_Real xMax = 0.0;
    Standard_Real yMax = 0.0;
    Standard_Real zMax = 0.0;
    box.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    result.minimum = Point3{xMin, yMin, zMin};
    result.maximum = Point3{xMax, yMax, zMax};
  }
  return result;
}

void ComputeProperties(const TopoDS_Solid &solid, InputAudit &audit) {
  GProp_GProps volumeProperties;
  BRepGProp::VolumeProperties(solid, volumeProperties, Standard_True,
                              Standard_False, Standard_False);
  audit.signedVolumeMm3 = volumeProperties.Mass();
  audit.centroidMm = ToPoint3(volumeProperties.CentreOfMass());

  GProp_GProps surfaceProperties;
  BRepGProp::SurfaceProperties(solid, surfaceProperties, Standard_False,
                               Standard_False);
  audit.surfaceAreaMm2 = surfaceProperties.Mass();
  audit.boundsMm = ComputeBounds(solid);
}

LoadedSolid LoadSingleSolid(const std::filesystem::path &path) {
  LoadedSolid loaded;
  loaded.audit.path = PathText(path);

  std::error_code pathError;
  if (!std::filesystem::exists(path, pathError) || pathError) {
    loaded.classification = LoadClass::InternalError;
    loaded.reason = "STEP path does not exist: " + loaded.audit.path;
    return loaded;
  }
  if (!std::filesystem::is_regular_file(path, pathError) || pathError) {
    loaded.classification = LoadClass::InternalError;
    loaded.reason = "STEP path is not a regular file: " + loaded.audit.path;
    return loaded;
  }

  STEPControl_Reader reader;
  const std::string pathUtf8 = PathText(path);
  const IFSelect_ReturnStatus readStatus = reader.ReadFile(pathUtf8.c_str());
  std::ostringstream loadDiagnostics;
  loadDiagnostics << "ReadFile status=" << ReadStatusText(readStatus);
  if (readStatus == IFSelect_RetDone) {
    reader.PrintCheckLoad(loadDiagnostics, Standard_False,
                          IFSelect_ItemsByEntity);
  }
  loaded.audit.loadDiagnostics = loadDiagnostics.str();
  if (readStatus != IFSelect_RetDone) {
    loaded.classification = LoadClass::Invalid;
    loaded.reason = "OCCT could not read STEP file: " + pathUtf8;
    return loaded;
  }

  TColStd_SequenceOfAsciiString lengthUnits;
  TColStd_SequenceOfAsciiString angleUnits;
  TColStd_SequenceOfAsciiString solidAngleUnits;
  reader.FileUnits(lengthUnits, angleUnits, solidAngleUnits);
  loaded.audit.fileLengthUnits = ToStrings(lengthUnits);

  // OCCT expresses the system length unit in millimetres. 1.0 therefore
  // normalizes every transferred STEP shape to millimetres.
  reader.SetSystemLengthUnit(1.0);
  const Standard_Integer transferredRoots = reader.TransferRoots();
  std::ostringstream transferDiagnostics;
  transferDiagnostics << "transferred_roots=" << transferredRoots;
  reader.PrintCheckTransfer(transferDiagnostics, Standard_False,
                            IFSelect_ItemsByEntity);
  loaded.audit.transferDiagnostics = transferDiagnostics.str();
  if (transferredRoots <= 0) {
    loaded.classification = LoadClass::Invalid;
    loaded.reason = "STEP roots could not be transferred: " + pathUtf8;
    return loaded;
  }

  const TopoDS_Shape transferred = reader.OneShape();
  if (transferred.IsNull()) {
    loaded.classification = LoadClass::Invalid;
    loaded.reason = "STEP transfer produced an empty shape: " + pathUtf8;
    return loaded;
  }

  loaded.audit.solidCount = CountUniqueSubShapes(transferred, TopAbs_SOLID);
  loaded.audit.shellCount = CountUniqueSubShapes(transferred, TopAbs_SHELL);
  loaded.audit.faceCount = CountUniqueSubShapes(transferred, TopAbs_FACE);
  loaded.audit.edgeCount = CountUniqueSubShapes(transferred, TopAbs_EDGE);

  if (loaded.audit.solidCount != 1) {
    loaded.classification = LoadClass::Unsupported;
    loaded.reason = "MVP requires exactly one solid; found " +
                    std::to_string(loaded.audit.solidCount) + " in " + pathUtf8;
    return loaded;
  }

  TopExp_Explorer solidExplorer(transferred, TopAbs_SOLID);
  loaded.solid = TopoDS::Solid(solidExplorer.Current());
  const bool containsOnlyTheSolid =
      loaded.audit.shellCount == CountUniqueSubShapes(loaded.solid, TopAbs_SHELL) &&
      loaded.audit.faceCount == CountUniqueSubShapes(loaded.solid, TopAbs_FACE) &&
      loaded.audit.edgeCount == CountUniqueSubShapes(loaded.solid, TopAbs_EDGE) &&
      CountUniqueSubShapes(transferred, TopAbs_VERTEX) ==
          CountUniqueSubShapes(loaded.solid, TopAbs_VERTEX);
  if (!containsOnlyTheSolid) {
    loaded.classification = LoadClass::Unsupported;
    loaded.reason =
        "MVP does not accept geometry outside the single solid: " + pathUtf8;
    return loaded;
  }

  loaded.audit.brepValid =
      BRepCheck_Analyzer(loaded.solid, Standard_True).IsValid();
  loaded.audit.closed = AllShellsClosed(loaded.solid);
  if (!loaded.audit.closed) {
    loaded.classification = LoadClass::Unsupported;
    loaded.reason = "MVP requires a closed solid: " + pathUtf8;
    return loaded;
  }
  if (!loaded.audit.brepValid) {
    loaded.classification = LoadClass::Invalid;
    loaded.reason = "OCCT B-Rep validation failed: " + pathUtf8;
    return loaded;
  }

  ComputeProperties(loaded.solid, loaded.audit);
  if (!std::isfinite(loaded.audit.signedVolumeMm3) ||
      loaded.audit.signedVolumeMm3 <= 0.0 || loaded.audit.boundsMm.isVoid) {
    loaded.classification = LoadClass::Invalid;
    loaded.reason =
        "solid is empty or has an invalid/reversed volume orientation: " +
        pathUtf8;
    return loaded;
  }

  loaded.classification = LoadClass::Ready;
  return loaded;
}

using Clock = std::chrono::steady_clock;

double ElapsedMs(const Clock::time_point &start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

const char *SurfaceTypeText(GeomAbs_SurfaceType type) {
  switch (type) {
  case GeomAbs_Plane: return "PLANE";
  case GeomAbs_Cylinder: return "CYLINDER";
  case GeomAbs_Cone: return "CONE";
  case GeomAbs_Sphere: return "SPHERE";
  case GeomAbs_Torus: return "TORUS";
  case GeomAbs_BezierSurface: return "BEZIER";
  case GeomAbs_BSplineSurface: return "BSPLINE";
  case GeomAbs_SurfaceOfRevolution: return "REVOLUTION";
  case GeomAbs_SurfaceOfExtrusion: return "EXTRUSION";
  default: return "OTHER";
  }
}

const char *CurveTypeText(GeomAbs_CurveType type) {
  switch (type) {
  case GeomAbs_Line: return "LINE";
  case GeomAbs_Circle: return "CIRCLE";
  case GeomAbs_Ellipse: return "ELLIPSE";
  case GeomAbs_Hyperbola: return "HYPERBOLA";
  case GeomAbs_Parabola: return "PARABOLA";
  case GeomAbs_BezierCurve: return "BEZIER";
  case GeomAbs_BSplineCurve: return "BSPLINE";
  default: return "OTHER";
  }
}

bool IsSeamEdge(const TopoDS_Edge &edge, const TopTools_ListOfShape &faces) {
  for (TopTools_ListOfShape::Iterator it(faces); it.More(); it.Next()) {
    const TopoDS_Face face = TopoDS::Face(it.Value());
    if (BRepTools::IsReallyClosed(edge, face)) return true;
  }
  return false;
}

std::vector<TopoDS_Edge> CollectComparableEdges(const TopoDS_Shape &shape) {
  TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
  TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
  std::vector<TopoDS_Edge> result;
  for (int index = 1; index <= edgeFaces.Extent(); ++index) {
    const TopoDS_Edge edge = TopoDS::Edge(edgeFaces.FindKey(index));
    if (BRep_Tool::Degenerated(edge)) continue;
    if (IsSeamEdge(edge, edgeFaces.FindFromIndex(index))) continue;
    result.push_back(edge);
  }
  return result;
}

std::vector<TypeStatistics> CollectFaceTypeStatistics(const TopoDS_Shape &shape) {
  std::map<std::string, TypeStatistics> statsMap;
  TopTools_IndexedMapOfShape faces;
  TopExp::MapShapes(shape, TopAbs_FACE, faces);
  for (int index = 1; index <= faces.Extent(); ++index) {
    const TopoDS_Face face = TopoDS::Face(faces(index));
    BRepAdaptor_Surface surface(face, Standard_True);
    const std::string type = SurfaceTypeText(surface.GetType());
    GProp_GProps properties;
    BRepGProp::SurfaceProperties(face, properties, Standard_False, Standard_False);
    auto &item = statsMap[type];
    item.type = type;
    ++item.count;
    item.totalMeasure += std::abs(properties.Mass());
  }
  std::vector<TypeStatistics> result;
  for (const auto &pair : statsMap) {
    result.push_back(pair.second);
  }
  return result;
}

std::vector<TypeStatistics> CollectEdgeTypeStatistics(const TopoDS_Shape &shape) {
  std::map<std::string, TypeStatistics> statsMap;
  for (const TopoDS_Edge &edge : CollectComparableEdges(shape)) {
    BRepAdaptor_Curve curve(edge);
    const std::string type = CurveTypeText(curve.GetType());
    GProp_GProps properties;
    BRepGProp::LinearProperties(edge, properties);
    auto &item = statsMap[type];
    item.type = type;
    ++item.count;
    item.totalMeasure += std::abs(properties.Mass());
  }
  std::vector<TypeStatistics> result;
  for (const auto &pair : statsMap) {
    result.push_back(pair.second);
  }
  return result;
}

struct NormalizedSolid {
  TopoDS_Solid solid;
  NormalizationAudit audit;
};

double SumSolidVolumes(const TopoDS_Shape &shape) {
  if (shape.IsNull()) return 0.0;
  double total = 0.0;
  for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More();
       explorer.Next()) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(explorer.Current(), properties, true,
                                false, false);
    total += std::abs(properties.Mass());
  }
  return total;
}

NormalizedSolid NormalizeSameDomain(const TopoDS_Solid &input, const CompareConfig &config) {
  NormalizedSolid result;
  result.solid = input;
  NormalizationAudit &audit = result.audit;
  audit.enabled = config.enableSameDomainNormalization;
  audit.faceCountBefore = CountUniqueSubShapes(input, TopAbs_FACE);
  audit.edgeCountBefore = CountUniqueSubShapes(input, TopAbs_EDGE);
  audit.comparableEdgeCountBefore = static_cast<int>(CollectComparableEdges(input).size());
  audit.volumeBeforeMm3 = SumSolidVolumes(input);

  if (!config.enableSameDomainNormalization) {
    audit.warning = "normalization disabled";
    audit.faceCountAfter = audit.faceCountBefore;
    audit.edgeCountAfter = audit.edgeCountBefore;
    audit.comparableEdgeCountAfter = audit.comparableEdgeCountBefore;
    audit.volumeAfterMm3 = audit.volumeBeforeMm3;
    return result;
  }

  const auto started = Clock::now();
  ShapeUpgrade_UnifySameDomain unify(input, Standard_True, Standard_True, Standard_False);
  unify.SetSafeInputMode(Standard_True);
  unify.SetLinearTolerance(config.normalizationLinearToleranceMm);
  unify.SetAngularTolerance(config.normalizationAngularToleranceRad);
  unify.AllowInternalEdges(Standard_False);
  unify.Build();
  audit.elapsedMs = ElapsedMs(started);

  const TopoDS_Shape normalized = unify.Shape();
  if (normalized.IsNull()) {
    audit.warning = "ShapeUpgrade_UnifySameDomain returned null shape";
    audit.faceCountAfter = audit.faceCountBefore;
    audit.edgeCountAfter = audit.edgeCountBefore;
    audit.comparableEdgeCountAfter = audit.comparableEdgeCountBefore;
    audit.volumeAfterMm3 = audit.volumeBeforeMm3;
    return result;
  }

  TopTools_IndexedMapOfShape solids;
  TopExp::MapShapes(normalized, TopAbs_SOLID, solids);
  if (solids.Extent() != 1) {
    audit.warning = "normalization did not preserve one-solid topology";
    audit.faceCountAfter = audit.faceCountBefore;
    audit.edgeCountAfter = audit.edgeCountBefore;
    audit.comparableEdgeCountAfter = audit.comparableEdgeCountBefore;
    audit.volumeAfterMm3 = audit.volumeBeforeMm3;
    return result;
  }

  const TopoDS_Solid normalizedSolid = TopoDS::Solid(solids(1));
  if (!AllShellsClosed(normalizedSolid)) {
    audit.warning = "normalized solid is not closed";
    audit.faceCountAfter = audit.faceCountBefore;
    audit.edgeCountAfter = audit.edgeCountBefore;
    audit.comparableEdgeCountAfter = audit.comparableEdgeCountBefore;
    audit.volumeAfterMm3 = audit.volumeBeforeMm3;
    return result;
  }

  if (!BRepCheck_Analyzer(normalizedSolid, Standard_True).IsValid()) {
    audit.warning = "normalized solid failed B-Rep validation";
    audit.faceCountAfter = audit.faceCountBefore;
    audit.edgeCountAfter = audit.edgeCountBefore;
    audit.comparableEdgeCountAfter = audit.comparableEdgeCountBefore;
    audit.volumeAfterMm3 = audit.volumeBeforeMm3;
    return result;
  }

  audit.volumeAfterMm3 = SumSolidVolumes(normalizedSolid);
  const double volumeScale = std::max(audit.volumeBeforeMm3, 1.0);
  audit.relativeVolumeDrift = std::abs(audit.volumeAfterMm3 - audit.volumeBeforeMm3) / volumeScale;

  if (audit.relativeVolumeDrift > 1.0e-10) {
    audit.warning = "normalization volume drift exceeds safety threshold";
    audit.faceCountAfter = audit.faceCountBefore;
    audit.edgeCountAfter = audit.edgeCountBefore;
    audit.comparableEdgeCountAfter = audit.comparableEdgeCountBefore;
    return result;
  }

  audit.faceCountAfter = CountUniqueSubShapes(normalizedSolid, TopAbs_FACE);
  audit.edgeCountAfter = CountUniqueSubShapes(normalizedSolid, TopAbs_EDGE);
  audit.comparableEdgeCountAfter = static_cast<int>(CollectComparableEdges(normalizedSolid).size());
  audit.succeeded = true;
  audit.usedNormalizedShape = true;
  result.solid = normalizedSolid;
  return result;
}

TopologyMatchAudit MatchNormalizedTopology(const TopoDS_Solid &refSolid,
                                           const TopoDS_Solid &candSolid,
                                           const CompareConfig &config);

double PointDistance(const Point3 &left, const Point3 &right) {
  return std::hypot(std::hypot(left.x - right.x, left.y - right.y),
                    left.z - right.z);
}

double MaximumBoundsDifference(const Bounds3 &left, const Bounds3 &right) {
  if (left.isVoid || right.isVoid) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max({std::abs(left.minimum.x - right.minimum.x),
                   std::abs(left.minimum.y - right.minimum.y),
                   std::abs(left.minimum.z - right.minimum.z),
                   std::abs(left.maximum.x - right.maximum.x),
                   std::abs(left.maximum.y - right.maximum.y),
                   std::abs(left.maximum.z - right.maximum.z)});
}

Bounds3 ShapeBounds(const TopoDS_Shape &shape) {
  Bounds3 result;
  if (shape.IsNull()) return result;
  Bnd_Box box;
  BRepBndLib::Add(shape, box);
  if (box.IsVoid()) return result;
  double xMin = 0.0, yMin = 0.0, zMin = 0.0;
  double xMax = 0.0, yMax = 0.0, zMax = 0.0;
  box.Get(xMin, yMin, zMin, xMax, yMax, zMax);
  result.minimum = Point3{xMin, yMin, zMin};
  result.maximum = Point3{xMax, yMax, zMax};
  result.isVoid = false;
  return result;
}

struct FaceDescriptor {
  int index = 0;
  std::string typeName;
  double areaMm2 = 0.0;
  Point3 centroidMm;
  Bounds3 boundsMm;
  bool matched = false;
};

struct EdgeDescriptor {
  int index = 0;
  std::string typeName;
  double lengthMm = 0.0;
  Point3 centroidMm;
  Bounds3 boundsMm;
  bool matched = false;
};

std::vector<FaceDescriptor> CollectFaceDescriptors(const TopoDS_Shape &shape) {
  std::vector<FaceDescriptor> result;
  TopTools_IndexedMapOfShape faces;
  TopExp::MapShapes(shape, TopAbs_FACE, faces);
  for (int index = 1; index <= faces.Extent(); ++index) {
    const TopoDS_Face face = TopoDS::Face(faces(index));
    BRepAdaptor_Surface surface(face, Standard_True);
    FaceDescriptor desc;
    desc.index = index;
    desc.typeName = SurfaceTypeText(surface.GetType());
    GProp_GProps properties;
    BRepGProp::SurfaceProperties(face, properties, Standard_False, Standard_False);
    desc.areaMm2 = std::abs(properties.Mass());
    const gp_Pnt center = properties.CentreOfMass();
    desc.centroidMm = Point3{center.X(), center.Y(), center.Z()};
    desc.boundsMm = ShapeBounds(face);
    result.push_back(desc);
  }
  return result;
}

std::vector<EdgeDescriptor> CollectEdgeDescriptors(const TopoDS_Shape &shape) {
  std::vector<EdgeDescriptor> result;
  const auto edges = CollectComparableEdges(shape);
  for (std::size_t index = 0; index < edges.size(); ++index) {
    const TopoDS_Edge edge = edges[index];
    BRepAdaptor_Curve curve(edge);
    EdgeDescriptor desc;
    desc.index = static_cast<int>(index + 1);
    desc.typeName = CurveTypeText(curve.GetType());
    GProp_GProps properties;
    BRepGProp::LinearProperties(edge, properties);
    desc.lengthMm = std::abs(properties.Mass());
    const gp_Pnt center = properties.CentreOfMass();
    desc.centroidMm = Point3{center.X(), center.Y(), center.Z()};
    desc.boundsMm = ShapeBounds(edge);
    result.push_back(desc);
  }
  return result;
}

bool FaceMatches(const FaceDescriptor &ref, const FaceDescriptor &cand, const CompareConfig &config) {
  if (ref.typeName != cand.typeName) return false;

  const double allowedAreaDiff = std::max(1.0e-3 * ref.areaMm2, 0.01);
  if (std::abs(ref.areaMm2 - cand.areaMm2) > allowedAreaDiff) return false;

  const double centroidDist = PointDistance(ref.centroidMm, cand.centroidMm);
  if (centroidDist > config.distanceToleranceMm) return false;

  const double boundsDiff = MaximumBoundsDifference(ref.boundsMm, cand.boundsMm);
  if (boundsDiff > config.distanceToleranceMm) return false;

  return true;
}

bool EdgeMatches(const EdgeDescriptor &ref, const EdgeDescriptor &cand, const CompareConfig &config) {
  if (ref.typeName != cand.typeName) return false;

  const double allowedLengthDiff = std::max(1.0e-3 * ref.lengthMm, 0.005);
  if (std::abs(ref.lengthMm - cand.lengthMm) > allowedLengthDiff) return false;

  const double centroidDist = PointDistance(ref.centroidMm, cand.centroidMm);
  if (centroidDist > config.distanceToleranceMm) return false;

  const double boundsDiff = MaximumBoundsDifference(ref.boundsMm, cand.boundsMm);
  if (boundsDiff > config.distanceToleranceMm) return false;

  return true;
}

int MatchFaceDescriptors(const std::vector<FaceDescriptor> &refFaces,
                         std::vector<FaceDescriptor> &candFaces,
                         const CompareConfig &config) {
  int matched = 0;
  for (const auto &ref : refFaces) {
    int bestCandIndex = -1;
    double bestDistance = std::numeric_limits<double>::max();

    for (std::size_t cIdx = 0; cIdx < candFaces.size(); ++cIdx) {
      if (candFaces[cIdx].matched) continue;
      if (FaceMatches(ref, candFaces[cIdx], config)) {
        const double dist = PointDistance(ref.centroidMm, candFaces[cIdx].centroidMm);
        if (dist < bestDistance) {
          bestDistance = dist;
          bestCandIndex = static_cast<int>(cIdx);
        }
      }
    }

    if (bestCandIndex >= 0) {
      candFaces[bestCandIndex].matched = true;
      ++matched;
    }
  }
  return matched;
}

int MatchEdgeDescriptors(const std::vector<EdgeDescriptor> &refEdges,
                         std::vector<EdgeDescriptor> &candEdges,
                         const CompareConfig &config) {
  int matched = 0;
  for (const auto &ref : refEdges) {
    int bestCandIndex = -1;
    double bestDistance = std::numeric_limits<double>::max();

    for (std::size_t cIdx = 0; cIdx < candEdges.size(); ++cIdx) {
      if (candEdges[cIdx].matched) continue;
      if (EdgeMatches(ref, candEdges[cIdx], config)) {
        const double dist = PointDistance(ref.centroidMm, candEdges[cIdx].centroidMm);
        if (dist < bestDistance) {
          bestDistance = dist;
          bestCandIndex = static_cast<int>(cIdx);
        }
      }
    }

    if (bestCandIndex >= 0) {
      candEdges[bestCandIndex].matched = true;
      ++matched;
    }
  }
  return matched;
}

TopologyMatchAudit MatchNormalizedTopology(const TopoDS_Solid &refSolid,
                                           const TopoDS_Solid &candSolid,
                                           const CompareConfig &config) {
  const auto started = Clock::now();
  TopologyMatchAudit audit;
  audit.attempted = true;

  const auto refFaceTypes = CollectFaceTypeStatistics(refSolid);
  const auto candFaceTypes = CollectFaceTypeStatistics(candSolid);
  const auto refEdgeTypes = CollectEdgeTypeStatistics(refSolid);
  const auto candEdgeTypes = CollectEdgeTypeStatistics(candSolid);

  const auto refFaceDescs = CollectFaceDescriptors(refSolid);
  auto candFaceDescs = CollectFaceDescriptors(candSolid);
  const auto refEdgeDescs = CollectEdgeDescriptors(refSolid);
  auto candEdgeDescs = CollectEdgeDescriptors(candSolid);

  audit.referenceFaceCount = static_cast<int>(refFaceDescs.size());
  audit.candidateFaceCount = static_cast<int>(candFaceDescs.size());
  audit.referenceEdgeCount = static_cast<int>(refEdgeDescs.size());
  audit.candidateEdgeCount = static_cast<int>(candEdgeDescs.size());

  auto MapsEqual = [](const std::vector<TypeStatistics> &left, const std::vector<TypeStatistics> &right) {
    std::map<std::string, int> lMap, rMap;
    for (const auto &item : left) lMap[item.type] = item.count;
    for (const auto &item : right) rMap[item.type] = item.count;
    return lMap == rMap;
  };

  audit.faceTypeHistogramEqual = MapsEqual(refFaceTypes, candFaceTypes);
  audit.edgeTypeHistogramEqual = MapsEqual(refEdgeTypes, candEdgeTypes);

  audit.matchedFaceCount = MatchFaceDescriptors(refFaceDescs, candFaceDescs, config);
  audit.matchedEdgeCount = MatchEdgeDescriptors(refEdgeDescs, candEdgeDescs, config);

  audit.unmatchedReferenceFaces = audit.referenceFaceCount - audit.matchedFaceCount;
  audit.unmatchedCandidateFaces = audit.candidateFaceCount - audit.matchedFaceCount;
  audit.unmatchedReferenceEdges = audit.referenceEdgeCount - audit.matchedEdgeCount;
  audit.unmatchedCandidateEdges = audit.candidateEdgeCount - audit.matchedEdgeCount;

  audit.allFacesMatched = (audit.matchedFaceCount == audit.referenceFaceCount) &&
                          (audit.matchedFaceCount == audit.candidateFaceCount);

  audit.allEdgesMatched = (audit.matchedEdgeCount == audit.referenceEdgeCount) &&
                          (audit.matchedEdgeCount == audit.candidateEdgeCount);

  audit.normalizedTopologyMatch = audit.allFacesMatched && audit.allEdgesMatched &&
                                  audit.faceTypeHistogramEqual && audit.edgeTypeHistogramEqual;

  audit.elapsedMs = ElapsedMs(started);
  return audit;
}

void ExportShapeStl(const TopoDS_Shape &shape, const std::filesystem::path &outPath) {
  if (shape.IsNull()) return;
  try {
    BRepTools::Clean(shape);
    BRepMesh_IncrementalMesh mesh(shape, 0.05, Standard_False, 0.5, Standard_True);
    mesh.Perform();
    StlAPI_Writer writer;
    writer.Write(shape, outPath.string().c_str());
  } catch (...) {}
}

DifferenceAudit Cut(const TopoDS_Solid &argument, const TopoDS_Solid &tool,
                    double fuzzyTolerance, const std::filesystem::path &stlOutPath = "") {
  DifferenceAudit result;
  BRepAlgoAPI_Cut cut;
  TopTools_ListOfShape arguments;
  TopTools_ListOfShape tools;
  arguments.Append(argument);
  tools.Append(tool);
  cut.SetArguments(arguments);
  cut.SetTools(tools);
  cut.SetNonDestructive(Standard_True);
  if (fuzzyTolerance > 0.0) {
    cut.SetFuzzyValue(fuzzyTolerance);
  }
  cut.Build();

  std::ostringstream report;
  if (cut.HasErrors()) {
    cut.DumpErrors(report);
  }
  if (cut.HasWarnings()) {
    cut.DumpWarnings(report);
  }
  result.report = report.str();
  result.succeeded = !cut.HasErrors() && cut.IsDone();
  if (!result.succeeded) {
    return result;
  }

  const TopoDS_Shape difference = cut.Shape();
  if (difference.IsNull()) {
    return result;
  }

  result.componentCount = CountSubShapes(difference, TopAbs_SOLID);
  if (result.componentCount > 0) {
    result.volumeMm3 = SumSolidVolumes(difference);

    if (!stlOutPath.empty()) {
      ExportShapeStl(difference, stlOutPath);
    }
  }
  return result;
}

json PointJson(const Point3 &point) {
  json obj = json::object();
  obj["x"] = point.x;
  obj["y"] = point.y;
  obj["z"] = point.z;
  return obj;
}

json BoundsJson(const Bounds3 &bounds) {
  json obj = json::object();
  obj["is_void"] = bounds.isVoid;
  obj["minimum_mm"] = PointJson(bounds.minimum);
  obj["maximum_mm"] = PointJson(bounds.maximum);
  return obj;
}

json InputJson(const InputAudit &audit) {
  json obj = json::object();
  obj["path"] = audit.path;
  obj["file_length_units"] = audit.fileLengthUnits;
  obj["load_diagnostics"] = audit.loadDiagnostics;
  obj["transfer_diagnostics"] = audit.transferDiagnostics;
  obj["solid_count"] = audit.solidCount;
  obj["shell_count"] = audit.shellCount;
  obj["face_count"] = audit.faceCount;
  obj["edge_count"] = audit.edgeCount;
  obj["brep_valid"] = audit.brepValid;
  obj["closed"] = audit.closed;
  obj["signed_volume_mm3"] = audit.signedVolumeMm3;
  obj["surface_area_mm2"] = audit.surfaceAreaMm2;
  obj["centroid_mm"] = PointJson(audit.centroidMm);
  obj["bounds"] = BoundsJson(audit.boundsMm);
  return obj;
}

json DifferenceJson(const DifferenceAudit &audit) {
  json obj = json::object();
  obj["succeeded"] = audit.succeeded;
  obj["volume_mm3"] = audit.volumeMm3;
  obj["component_count"] = audit.componentCount;
  obj["kernel_report"] = audit.report;
  return obj;
}

json TypeStatsJson(const std::vector<TypeStatistics> &stats) {
  json arr = json::array();
  for (const auto &item : stats) {
    json itemObj = json::object();
    itemObj["type"] = item.type;
    itemObj["count"] = item.count;
    itemObj["total_measure"] = item.totalMeasure;
    arr.push_back(itemObj);
  }
  return arr;
}

json NormalizationAuditJson(const NormalizationAudit &audit) {
  json obj = json::object();
  obj["enabled"] = audit.enabled;
  obj["succeeded"] = audit.succeeded;
  obj["used_normalized_shape"] = audit.usedNormalizedShape;
  obj["faces_before"] = audit.faceCountBefore;
  obj["faces_after"] = audit.faceCountAfter;
  obj["edges_before"] = audit.edgeCountBefore;
  obj["edges_after"] = audit.edgeCountAfter;
  obj["comparable_edges_before"] = audit.comparableEdgeCountBefore;
  obj["comparable_edges_after"] = audit.comparableEdgeCountAfter;
  obj["volume_before_mm3"] = audit.volumeBeforeMm3;
  obj["volume_after_mm3"] = audit.volumeAfterMm3;
  obj["relative_volume_drift"] = audit.relativeVolumeDrift;
  obj["elapsed_ms"] = audit.elapsedMs;
  obj["warning"] = audit.warning;
  obj["face_types"] = TypeStatsJson(audit.faceTypes);
  obj["edge_types"] = TypeStatsJson(audit.edgeTypes);
  return obj;
}

json TopologyMatchAuditJson(const TopologyMatchAudit &audit) {
  json obj = json::object();
  obj["attempted"] = audit.attempted;
  obj["reference_faces"] = audit.referenceFaceCount;
  obj["candidate_faces"] = audit.candidateFaceCount;
  obj["matched_faces"] = audit.matchedFaceCount;
  obj["reference_edges"] = audit.referenceEdgeCount;
  obj["candidate_edges"] = audit.candidateEdgeCount;
  obj["matched_edges"] = audit.matchedEdgeCount;
  obj["unmatched_reference_faces"] = audit.unmatchedReferenceFaces;
  obj["unmatched_candidate_faces"] = audit.unmatchedCandidateFaces;
  obj["unmatched_reference_edges"] = audit.unmatchedReferenceEdges;
  obj["unmatched_candidate_edges"] = audit.unmatchedCandidateEdges;
  obj["face_type_histogram_equal"] = audit.faceTypeHistogramEqual;
  obj["edge_type_histogram_equal"] = audit.edgeTypeHistogramEqual;
  obj["all_faces_matched"] = audit.allFacesMatched;
  obj["all_edges_matched"] = audit.allEdgesMatched;
  obj["match"] = audit.normalizedTopologyMatch;
  obj["elapsed_ms"] = audit.elapsedMs;
  return obj;
}

json TimingsJson(const TimingAudit &timings) {
  json obj = json::object();
  obj["load_reference_ms"] = timings.loadReferenceMs;
  obj["load_candidate_ms"] = timings.loadCandidateMs;
  obj["normalize_reference_ms"] = timings.normalizeReferenceMs;
  obj["normalize_candidate_ms"] = timings.normalizeCandidateMs;
  obj["topology_match_ms"] = timings.topologyMatchMs;
  obj["boolean_ab_ms"] = timings.booleanAbMs;
  obj["boolean_ba_ms"] = timings.booleanBaMs;
  obj["artifact_export_ms"] = timings.artifactExportMs;
  obj["total_ms"] = timings.totalMs;
  return obj;
}

CompareStatus FailedLoadStatus(LoadClass classification) {
  switch (classification) {
  case LoadClass::Invalid:
    return CompareStatus::InvalidInput;
  case LoadClass::Unsupported:
    return CompareStatus::UnsupportedShape;
  case LoadClass::InternalError:
    return CompareStatus::InternalError;
  case LoadClass::Ready:
    break;
  }
  return CompareStatus::InternalError;
}

bool VolumePass(double absoluteDiff, double referenceScale, double absTol, double relTol) {
  const double effectiveAbsTol = std::max(absTol, relTol * referenceScale);
  const bool absPass = absoluteDiff <= effectiveAbsTol;
  const bool relPass = (referenceScale > 0.0) ? (absoluteDiff / referenceScale <= relTol) : true;
  return absPass && relPass;
}

} // namespace

CompareResult CompareStepFiles(const std::filesystem::path &reference,
                               const std::filesystem::path &candidate,
                               const CompareConfig &config,
                               const std::filesystem::path &outputDirectory) {
  const auto totalStart = Clock::now();
  CompareResult result;
  result.thresholds = config;
  result.reference.path = PathText(reference);
  result.candidate.path = PathText(candidate);

  if (!std::isfinite(config.distanceToleranceMm) ||
      !std::isfinite(config.absoluteVolumeToleranceMm3) ||
      !std::isfinite(config.relativeVolumeTolerance) ||
      config.distanceToleranceMm < 0.0 ||
      config.absoluteVolumeToleranceMm3 < 0.0 ||
      config.relativeVolumeTolerance < 0.0) {
    result.status = CompareStatus::InternalError;
    result.reason = "comparison tolerances must be finite and non-negative";
    return result;
  }

  try {
    const auto loadRefStart = Clock::now();
    LoadedSolid referenceSolid = LoadSingleSolid(reference);
    result.timings.loadReferenceMs = ElapsedMs(loadRefStart);

    const auto loadCandStart = Clock::now();
    LoadedSolid candidateSolid = LoadSingleSolid(candidate);
    result.timings.loadCandidateMs = ElapsedMs(loadCandStart);

    result.reference = referenceSolid.audit;
    result.candidate = candidateSolid.audit;

    if (referenceSolid.classification != LoadClass::Ready) {
      result.status = FailedLoadStatus(referenceSolid.classification);
      result.reason =
          "reference shape load failed: " + referenceSolid.audit.loadDiagnostics;
      result.timings.totalMs = ElapsedMs(totalStart);
      return result;
    }
    if (candidateSolid.classification != LoadClass::Ready) {
      result.status = FailedLoadStatus(candidateSolid.classification);
      result.reason =
          "candidate shape load failed: " + candidateSolid.audit.loadDiagnostics;
      result.timings.totalMs = ElapsedMs(totalStart);
      return result;
    }

    // Normalization phase
    const auto normRefStart = Clock::now();
    NormalizedSolid normalizedReference = NormalizeSameDomain(referenceSolid.solid, config);
    result.timings.normalizeReferenceMs = ElapsedMs(normRefStart);

    const auto normCandStart = Clock::now();
    NormalizedSolid normalizedCandidate = NormalizeSameDomain(candidateSolid.solid, config);
    result.timings.normalizeCandidateMs = ElapsedMs(normCandStart);

    result.referenceNormalization = normalizedReference.audit;
    result.candidateNormalization = normalizedCandidate.audit;

    const TopoDS_Solid &referenceCompareSolid =
        normalizedReference.audit.usedNormalizedShape
            ? normalizedReference.solid
            : referenceSolid.solid;

    const TopoDS_Solid &candidateCompareSolid =
        normalizedCandidate.audit.usedNormalizedShape
            ? normalizedCandidate.solid
            : candidateSolid.solid;

    result.referenceNormalization.faceTypes = CollectFaceTypeStatistics(referenceCompareSolid);
    result.candidateNormalization.faceTypes = CollectFaceTypeStatistics(candidateCompareSolid);
    result.referenceNormalization.edgeTypes = CollectEdgeTypeStatistics(referenceCompareSolid);
    result.candidateNormalization.edgeTypes = CollectEdgeTypeStatistics(candidateCompareSolid);

    // Topology matching phase
    const auto topMatchStart = Clock::now();
    result.normalizedTopology = MatchNormalizedTopology(referenceCompareSolid, candidateCompareSolid, config);
    result.timings.topologyMatchMs = ElapsedMs(topMatchStart);

    const double referenceVolume = referenceSolid.audit.signedVolumeMm3;
    const double candidateVolume = candidateSolid.audit.signedVolumeMm3;
    const double volumeScale = std::max(referenceVolume, candidateVolume);

    result.absoluteInputVolumeDifferenceMm3 =
        std::abs(referenceVolume - candidateVolume);
    result.relativeInputVolumeDifference =
        result.absoluteInputVolumeDifferenceMm3 / volumeScale;
    result.centroidDistanceMm = PointDistance(referenceSolid.audit.centroidMm,
                                              candidateSolid.audit.centroidMm);
    result.maximumBoundsDifferenceMm = MaximumBoundsDifference(
        referenceSolid.audit.boundsMm, candidateSolid.audit.boundsMm);

    const bool inputVolumePass = VolumePass(
        result.absoluteInputVolumeDifferenceMm3, volumeScale,
        config.absoluteVolumeToleranceMm3, config.relativeVolumeTolerance);
    const bool centroidPass =
        result.centroidDistanceMm <= config.distanceToleranceMm;
    const bool boundsPass =
        result.maximumBoundsDifferenceMm <= config.distanceToleranceMm;

    const bool globalMetricsPass = inputVolumePass && centroidPass && boundsPass;
    const bool normalizedMatch = result.normalizedTopology.normalizedTopologyMatch;

    // Optional fast-path decision
    if (config.enableNormalizedFastPath && globalMetricsPass && normalizedMatch) {
      result.status = CompareStatus::Equal;
      result.booleanExecuted = false;
      result.decisionPath = "normalized_topology_fast_path";
      result.reason = "global metrics and normalized face/edge matching passed";
      result.timings.totalMs = ElapsedMs(totalStart);
      return result;
    }

    // Boolean verification phase
    result.booleanExecuted = true;
    result.decisionPath = normalizedReference.audit.usedNormalizedShape
                              ? "boolean_after_normalization"
                              : "boolean_original";

    const double fuzzyTolerance =
        config.booleanFuzzyToleranceMm > 0.0 ? config.booleanFuzzyToleranceMm : config.distanceToleranceMm;

    const auto boolAbStart = Clock::now();
    if (!outputDirectory.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(outputDirectory, ec);
      ExportShapeStl(referenceCompareSolid, outputDirectory / "reference_base.stl");
      result.missingMaterial = Cut(referenceCompareSolid, candidateCompareSolid, fuzzyTolerance, outputDirectory / "missing_material.stl");
    } else {
      result.missingMaterial = Cut(referenceCompareSolid, candidateCompareSolid, fuzzyTolerance);
    }
    result.timings.booleanAbMs = ElapsedMs(boolAbStart);

    const auto boolBaStart = Clock::now();
    if (!outputDirectory.empty()) {
      result.addedMaterial = Cut(candidateCompareSolid, referenceCompareSolid, fuzzyTolerance, outputDirectory / "added_material.stl");
    } else {
      result.addedMaterial = Cut(candidateCompareSolid, referenceCompareSolid, fuzzyTolerance);
    }
    result.timings.booleanBaMs = ElapsedMs(boolBaStart);

    if (!result.missingMaterial.succeeded || !result.addedMaterial.succeeded) {
      result.status = CompareStatus::Indeterminate;
      result.reason =
          "OCCT boolean subtraction failed; geometry equality is unknown";
      result.timings.totalMs = ElapsedMs(totalStart);
      return result;
    }

    result.symmetricDifferenceVolumeMm3 =
        result.missingMaterial.volumeMm3 + result.addedMaterial.volumeMm3;
    result.symmetricDifferenceRelative =
        result.symmetricDifferenceVolumeMm3 / volumeScale;

    // Check for suspicious boolean cut result ("near-zero overlap illusion" where global metrics are highly similar but Cut returned full inputs)
    const bool booleanClaimsNoOverlap =
        result.missingMaterial.volumeMm3 > 0.95 * referenceVolume &&
        result.addedMaterial.volumeMm3 > 0.95 * candidateVolume;

    if (globalMetricsPass && booleanClaimsNoOverlap) {
      result.status = CompareStatus::Indeterminate;
      result.reason =
          "boolean cut claims near-zero overlap despite highly similar global volume, centroid, and bounds metrics";
      result.timings.totalMs = ElapsedMs(totalStart);
      return result;
    }

    const bool symmetricDifferencePass = VolumePass(
        result.symmetricDifferenceVolumeMm3, volumeScale,
        config.absoluteVolumeToleranceMm3, config.relativeVolumeTolerance);

    if (globalMetricsPass && symmetricDifferencePass) {
      result.status = CompareStatus::Equal;
      result.reason =
          "closed solids pass volume, centroid, bounds, and symmetric difference thresholds";
    } else {
      result.status = CompareStatus::Different;
      std::ostringstream reason;
      reason << "geometry thresholds failed:";
      if (!inputVolumePass) {
        reason << " input_volume";
      }
      if (!centroidPass) {
        reason << " centroid";
      }
      if (!boundsPass) {
        reason << " bounds";
      }
      if (!symmetricDifferencePass) {
        reason << " symmetric_difference";
      }
      result.reason = reason.str();
    }
  } catch (const Standard_Failure &failure) {
    result.status = CompareStatus::InternalError;
    result.reason =
        std::string("OCCT exception: ") +
        (failure.GetMessageString() ? failure.GetMessageString() : "unknown");
  } catch (const std::exception &error) {
    result.status = CompareStatus::InternalError;
    result.reason = std::string("program exception: ") + error.what();
  }
  result.timings.totalMs = ElapsedMs(totalStart);
  return result;
}

const char *ToString(CompareStatus status) {
  switch (status) {
  case CompareStatus::Equal:
    return "EQUAL";
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
  return "INTERNAL_ERROR";
}

int ExitCode(CompareStatus status) {
  switch (status) {
  case CompareStatus::Equal:
    return 0;
  case CompareStatus::Different:
    return 1;
  case CompareStatus::InvalidInput:
  case CompareStatus::UnsupportedShape:
    return 2;
  case CompareStatus::Indeterminate:
    return 3;
  case CompareStatus::InternalError:
    return 4;
  }
  return 4;
}

std::string ToJson(const CompareResult &result) {
  json root = json::object();
  root["status"] = ToString(result.status);
  root["exit_code"] = ExitCode(result.status);
  root["reason"] = result.reason;

  json decision = json::object();
  decision["path"] = result.decisionPath;
  decision["boolean_executed"] = result.booleanExecuted;
  root["decision"] = decision;

  json thresholds = json::object();
  thresholds["distance_tolerance_mm"] = result.thresholds.distanceToleranceMm;
  thresholds["absolute_volume_tolerance_mm3"] = result.thresholds.absoluteVolumeToleranceMm3;
  thresholds["relative_volume_tolerance"] = result.thresholds.relativeVolumeTolerance;
  thresholds["boolean_fuzzy_tolerance_mm"] = result.thresholds.booleanFuzzyToleranceMm;
  thresholds["normalization_linear_tolerance_mm"] = result.thresholds.normalizationLinearToleranceMm;
  thresholds["normalization_angular_tolerance_rad"] = result.thresholds.normalizationAngularToleranceRad;
  thresholds["enable_normalized_fast_path"] = result.thresholds.enableNormalizedFastPath;
  root["thresholds"] = thresholds;

  root["reference"] = InputJson(result.reference);
  root["candidate"] = InputJson(result.candidate);

  json norm = json::object();
  norm["reference"] = NormalizationAuditJson(result.referenceNormalization);
  norm["candidate"] = NormalizationAuditJson(result.candidateNormalization);
  root["normalization"] = norm;

  root["normalized_topology"] = TopologyMatchAuditJson(result.normalizedTopology);
  root["missing_material"] = DifferenceJson(result.missingMaterial);
  root["added_material"] = DifferenceJson(result.addedMaterial);

  json metrics = json::object();
  metrics["absolute_input_volume_difference_mm3"] = result.absoluteInputVolumeDifferenceMm3;
  metrics["relative_input_volume_difference"] = result.relativeInputVolumeDifference;
  metrics["centroid_distance_mm"] = result.centroidDistanceMm;
  metrics["maximum_bounds_difference_mm"] = result.maximumBoundsDifferenceMm;
  metrics["symmetric_difference_volume_mm3"] = result.symmetricDifferenceVolumeMm3;
  metrics["symmetric_difference_relative"] = result.symmetricDifferenceRelative;
  root["decision_metrics"] = metrics;

  root["timings_ms"] = TimingsJson(result.timings);
  return root.dump(2) + '\n';
}

std::string ToHumanSummary(const CompareResult &result) {
  std::ostringstream ss;
  ss << "============================================================\n"
     << " CAD STEP GEOMETRY COMPARISON\n"
     << "============================================================\n\n"
     << "[1] INPUT\n"
     << "  Reference : " << result.reference.path << "\n"
     << "  Candidate : " << result.candidate.path << "\n"
     << "  Reference : solids=" << result.reference.solidCount
     << " faces=" << result.reference.faceCount
     << " edges=" << result.reference.edgeCount << "\n"
     << "  Candidate : solids=" << result.candidate.solidCount
     << " faces=" << result.candidate.faceCount
     << " edges=" << result.candidate.edgeCount << "\n\n"
     << "[2] SAME-DOMAIN NORMALIZATION\n"
     << "  Linear tolerance : " << result.thresholds.normalizationLinearToleranceMm << " mm\n"
     << "  Angular tolerance: " << result.thresholds.normalizationAngularToleranceRad << " rad\n"
     << "  Reference\n"
     << "    status       : " << (result.referenceNormalization.succeeded ? "SUCCESS" : "SKIPPED/WARNING") << "\n"
     << "    faces        : " << result.referenceNormalization.faceCountBefore << " -> " << result.referenceNormalization.faceCountAfter << "\n"
     << "    edges        : " << result.referenceNormalization.edgeCountBefore << " -> " << result.referenceNormalization.edgeCountAfter << "\n"
     << "    elapsed      : " << result.referenceNormalization.elapsedMs << " ms\n"
     << "  Candidate\n"
     << "    status       : " << (result.candidateNormalization.succeeded ? "SUCCESS" : "SKIPPED/WARNING") << "\n"
     << "    faces        : " << result.candidateNormalization.faceCountBefore << " -> " << result.candidateNormalization.faceCountAfter << "\n"
     << "    edges        : " << result.candidateNormalization.edgeCountBefore << " -> " << result.candidateNormalization.edgeCountAfter << "\n"
     << "    elapsed      : " << result.candidateNormalization.elapsedMs << " ms\n\n"
     << "[3] TOPOLOGY MATCH\n"
     << "  Faces : " << result.normalizedTopology.matchedFaceCount << " / " << result.normalizedTopology.referenceFaceCount << " matched\n"
     << "  Edges : " << result.normalizedTopology.matchedEdgeCount << " / " << result.normalizedTopology.referenceEdgeCount << " matched\n"
     << "  Normalized topology match : " << (result.normalizedTopology.normalizedTopologyMatch ? "YES" : "NO") << "\n\n"
     << "[4] GLOBAL GEOMETRY\n"
     << "  Absolute volume difference : " << result.absoluteInputVolumeDifferenceMm3 << " mm3\n"
     << "  Relative volume difference : " << result.relativeInputVolumeDifference << "\n"
     << "  Centroid distance          : " << result.centroidDistanceMm << " mm\n"
     << "  Maximum bounds difference  : " << result.maximumBoundsDifferenceMm << " mm\n\n"
     << "[5] BOOLEAN VERIFICATION\n"
     << "  Executed       : " << (result.booleanExecuted ? "YES" : "NO") << "\n"
     << "  Decision path  : " << result.decisionPath << "\n"
     << "  A - B time     : " << result.timings.booleanAbMs << " ms\n"
     << "  B - A time     : " << result.timings.booleanBaMs << " ms\n\n"
     << "[6] RESULT\n"
     << "  Status        : " << ToString(result.status) << "\n"
     << "  Reason        : " << result.reason << "\n"
     << "============================================================\n";
  return ss.str();
}

bool WriteResultJson(const std::filesystem::path &outputDirectory,
                     const CompareResult &result, std::string &error) {
  try {
    std::filesystem::create_directories(outputDirectory);
    const std::filesystem::path resultPath = outputDirectory / "result.json";
    std::ofstream output(resultPath, std::ios::binary | std::ios::trunc);
    if (!output.is_open()) {
      error = "unable to open result file: " + PathText(resultPath);
      return false;
    }
    output << ToJson(result);
    if (!output.good()) {
      error = "failed to write result file: " + PathText(resultPath);
      return false;
    }
    return true;
  } catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

} // namespace cadstep
