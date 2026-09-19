#pragma once

#include "StepCompare.h"
#include "domain/ComparisonModel.h"

#include <TopoDS_Solid.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace cadstep {
namespace detail {

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

  // Matching inputs, produced by the same geometry pass that fills
  // audit.faces / audit.edges. They are held separately because audit.faces
  // deliberately stays empty when same-domain normalisation is disabled, while
  // matching still needs every face.
  std::vector<DescriptorView> faceDescriptors;
  std::vector<DescriptorView> comparableEdgeDescriptors;

  // Time spent assembling the descriptors themselves. This is only the
  // projection over the records built above (plus, on the disabled path, the
  // face geometry pass there is no audit record for) - the per-entity geometry
  // is measured once and is charged to the normalisation bucket.
  double descriptorBuildMs = 0.0;
};

// Runs UnifySameDomain over `input` and records the audit trail, the resulting
// face/edge maps, the matching descriptors, and the mapping back to the original
// topology. Falls back to `input` unchanged whenever the result would not be a
// single valid closed solid with a tolerable volume drift.
NormalizedSolidInternal NormalizeSameDomain(const TopoDS_Solid &input,
                                            const OriginalTopologyIndex &originalIndex,
                                            EntitySide side,
                                            const CompareConfig &config);

// Indexes the original solid's faces/edges and their entity ids.
OriginalTopologyIndex BuildOriginalTopologyIndex(const TopoDS_Solid &solid, EntitySide side);

std::vector<TypeStatistics> SummarizeMap(
    const std::map<std::string, std::pair<int, double>> &countMap);


} // namespace detail
} // namespace cadstep
