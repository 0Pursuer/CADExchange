// Reads a STEP file into OCCT shapes and audits every solid it contains.

#include "geometry/StepLoader.h"
#include "geometry/ShapeAudit.h"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

namespace cadstep {
namespace detail {

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

} // namespace detail
} // namespace cadstep
