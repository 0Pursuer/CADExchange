#pragma once

#include "StepCompare.h"
#include "domain/ComparisonModel.h"
#include "domain/TolerancePolicy.h"

#include <vector>

namespace cadstep {
namespace detail {

// Greedy 1-to-1 descriptor matching.
//
// Faces and edges matched types and counts, so they share one implementation and
// differ only in how "the same size" is judged:
//   faces -> area compared relative to the reference face (a relative gate),
//   edges -> length compared absolutely against the distance tolerance.
// Everything else (type histogram, best/second-best scoring, ambiguity margin,
// picking order) is common and was previously duplicated verbatim.
MatchCollection MatchFaceDescriptors(const std::vector<DescriptorView> &reference,
                                     const std::vector<DescriptorView> &candidate,
                                     const CompareConfig &config,
                                     double characteristicScale,
                                     const TolerancePolicy &tolerances);

MatchCollection MatchEdgeDescriptors(const std::vector<DescriptorView> &reference,
                                     const std::vector<DescriptorView> &candidate,
                                     const CompareConfig &config,
                                     double characteristicScale,
                                     const TolerancePolicy &tolerances);

} // namespace detail
} // namespace cadstep
