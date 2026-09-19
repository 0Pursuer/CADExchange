#pragma once

#include "StepCompare.h"

#include <string>
#include <vector>

namespace cadstep {
namespace detail {

// Minimal geometry view consumed by the descriptor matcher.
//
// Deliberately free of OCCT types and of audit-only bookkeeping: faces and edges
// both reduce to (type, one measure, centroid, bounds), which is what allows a
// single matching algorithm to serve both. The audit trail keeps its own richer
// records (NormalizedFaceInfo / NormalizedEdgeInfo); these views are produced
// from the same geometry pass so nothing is measured twice.
struct DescriptorView {
  std::string id;        // written to EntityMatch::referenceId / candidateId
  std::string typeName;  // surface or curve type string
  double measure = 0.0;  // area (mm^2) for faces, length (mm) for edges
  Point3 centroidMm;
  Bounds3 boundsMm;
};

} // namespace detail
} // namespace cadstep
