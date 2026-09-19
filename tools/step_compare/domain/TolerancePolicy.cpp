#include "domain/TolerancePolicy.h"

#include "domain/GeometryMath.h"

#include <algorithm>

namespace cadstep {
namespace detail {

TolerancePolicy TolerancePolicy::Derive(const CompareConfig &config,
                                        const InputAudit &reference) {
  TolerancePolicy policy;
  policy.distanceToleranceMm_ =
      std::max(config.distanceToleranceMm, 1.0e-4 * BoundsDiagonal(reference.boundsMm));
  policy.absoluteVolumeToleranceMm3_ =
      std::max(config.absoluteVolumeToleranceMm3,
               config.relativeVolumeTolerance * reference.signedVolumeMm3);
  policy.relativeVolumeTolerance_ = config.relativeVolumeTolerance;
  return policy;
}

GeometryPassFlags EvaluateGeometryPasses(const CompareResult &result,
                                         const TolerancePolicy &tolerances) {
  GeometryPassFlags flags;
  flags.volume =
      (result.absoluteInputVolumeDifferenceMm3 <= tolerances.absoluteVolumeMm3()) &&
      (result.relativeInputVolumeDifference <= tolerances.relativeVolume());
  flags.centroid = result.centroidDistanceMm <= tolerances.distanceMm();
  flags.bounds = result.maximumBoundsDifferenceMm <= tolerances.distanceMm();
  return flags;
}

bool EvaluateBooleanPass(const CompareResult &result,
                         const TolerancePolicy &tolerances) {
  if (!result.booleanExecuted) {
    return true;
  }
  return result.missingMaterial.volumeMm3 <= tolerances.absoluteVolumeMm3() &&
         result.addedMaterial.volumeMm3 <= tolerances.absoluteVolumeMm3();
}

} // namespace detail
} // namespace cadstep
