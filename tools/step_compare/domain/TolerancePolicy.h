#pragma once

#include "StepCompare.h"

namespace cadstep {
namespace detail {

// Single source of truth for the adaptive tolerance model described in
// CADExchange/doc/STEP终态实体几何对比规范与自适应容差说明_20260804.md
// ("static floor + characteristic-scale relative ratio").
//
// The thresholds and the pass/fail gates derived from them are consumed both by
// the decision flow and by the two report writers. Historically each of those
// three places re-implemented the formulas, which allowed the rendered
// passed/failed flags to drift away from the flags that produced the verdict.
class TolerancePolicy {
public:
  static TolerancePolicy Derive(const CompareConfig &config,
                                const InputAudit &reference);

  double distanceMm() const { return distanceToleranceMm_; }
  double absoluteVolumeMm3() const { return absoluteVolumeToleranceMm3_; }
  double relativeVolume() const { return relativeVolumeTolerance_; }

private:
  double distanceToleranceMm_ = 0.0;
  double absoluteVolumeToleranceMm3_ = 0.0;
  double relativeVolumeTolerance_ = 0.0;
};

// Outcome of the three global-metric gates.
//
// In the decision flow a failure here latches: it blocks the normalized fast
// path but does not terminate the comparison. The report writers render the very
// same flags, which is why they are computed here rather than at each site.
struct GeometryPassFlags {
  bool volume = false;
  bool centroid = false;
  bool bounds = false;
};

GeometryPassFlags EvaluateGeometryPasses(const CompareResult &result,
                                         const TolerancePolicy &tolerances);

// Boolean residual gate. A comparison that never executed the boolean step is
// not treated as a failure, matching the historical `!booleanExecuted || ...`
// rule used by the report writers.
bool EvaluateBooleanPass(const CompareResult &result,
                         const TolerancePolicy &tolerances);

} // namespace detail
} // namespace cadstep
