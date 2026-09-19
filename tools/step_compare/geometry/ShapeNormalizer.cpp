// Same-domain normalisation plus the edge-role classification and the
// original-to-normalised topology mapping it produces.

#include "geometry/ShapeNormalizer.h"
#include "domain/Timing.h"
#include "geometry/ShapeAudit.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRepTools_History.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace cadstep {
namespace detail {

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
      const auto geometry = SummarizeEdge(e, curve);
      info.curveType = geometry.curveType;
      info.lengthMm = geometry.lengthMm;
      info.centroidMm = geometry.centroidMm;
      info.boundsMm = geometry.boundsMm;
      info.closed = geometry.closed;

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

    const auto descriptorStart = std::chrono::high_resolution_clock::now();
    result.comparableEdgeDescriptors.reserve(result.audit.edges.size());
    for (const auto &info : result.audit.edges) {
      if (info.comparable) {
        result.comparableEdgeDescriptors.push_back(
            {info.id, info.curveType, info.lengthMm, info.centroidMm, info.boundsMm});
      }
    }

    // audit.faces deliberately stays empty on this path, but matching still needs
    // every face: the audit additionally carries source-mapping detail that only
    // the normalisation path can produce, so the two cannot be the same list.
    result.faceDescriptors.reserve(static_cast<std::size_t>(result.normalizedFaces.Extent()));
    for (int i = 1; i <= result.normalizedFaces.Extent(); ++i) {
      const TopoDS_Face f = TopoDS::Face(result.normalizedFaces(i));
      BRepAdaptor_Surface surface(f);
      const auto geometry = SummarizeFace(f, surface);
      result.faceDescriptors.push_back({MakeEntityId(side, EntityKind::NormalizedFace, i),
                                        geometry.surfaceType, geometry.areaMm2,
                                        geometry.centroidMm, geometry.boundsMm});
    }
    result.descriptorBuildMs = ElapsedMs(descriptorStart);
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
    const auto geometry = SummarizeFace(f, surface);
    info.surfaceType = geometry.surfaceType;
    info.areaMm2 = geometry.areaMm2;
    info.centroidMm = geometry.centroidMm;
    info.boundsMm = geometry.boundsMm;

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
    const auto geometry = SummarizeEdge(e, curve);
    info.curveType = geometry.curveType;
    info.lengthMm = geometry.lengthMm;
    info.centroidMm = geometry.centroidMm;
    info.boundsMm = geometry.boundsMm;
    info.closed = geometry.closed;

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

  // Descriptors are projected from the audit records above: the per-entity
  // geometry is measured once, so all that remains here is the projection.
  const auto descriptorStart = std::chrono::high_resolution_clock::now();
  result.faceDescriptors.reserve(result.audit.faces.size());
  for (const auto &info : result.audit.faces) {
    result.faceDescriptors.push_back(
        {info.id, info.surfaceType, info.areaMm2, info.centroidMm, info.boundsMm});
  }
  result.comparableEdgeDescriptors.reserve(result.audit.edges.size());
  for (const auto &info : result.audit.edges) {
    if (info.comparable) {
      result.comparableEdgeDescriptors.push_back(
          {info.id, info.curveType, info.lengthMm, info.centroidMm, info.boundsMm});
    }
  }
  result.descriptorBuildMs = ElapsedMs(descriptorStart);

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

} // namespace detail
} // namespace cadstep
