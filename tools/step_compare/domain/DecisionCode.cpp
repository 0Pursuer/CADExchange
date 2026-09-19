#include "domain/DecisionCode.h"

namespace cadstep {
namespace detail {

const char *ToString(DecisionCode code) {
  switch (code) {
  case DecisionCode::BooleanAfterNormalization:
    return "boolean_after_normalization";
  case DecisionCode::BooleanAfterOriginal:
    return "boolean_after_original";
  case DecisionCode::GlobalMetricsFailed:
    return "global_metrics_failed";
  case DecisionCode::NormalizedTopologyFastPath:
    return "normalized_topology_fast_path";
  case DecisionCode::BooleanFailed:
    return "boolean_failed";
  case DecisionCode::BooleanAfterOriginalFallback:
    return "boolean_after_original_fallback";
  case DecisionCode::BooleanConservationInvalid:
    return "boolean_conservation_invalid";
  case DecisionCode::BooleanDifference:
    return "boolean_difference";
  case DecisionCode::InputInvalid:
    return "input_invalid";
  case DecisionCode::InputUnsupported:
    return "input_unsupported";
  case DecisionCode::InputUnsupportedPolicy:
    return "input_unsupported_policy";
  case DecisionCode::MultiSolidUnmatched:
    return "multi_solid_unmatched";
  case DecisionCode::MultiSolidPairsEqual:
    return "multi_solid_pairs_equal";
  case DecisionCode::MultiSolidPairDifferent:
    return "multi_solid_pair_different";
  case DecisionCode::MultiSolidPairsLikelyEqual:
    return "multi_solid_pairs_likely_equal";
  case DecisionCode::MultiSolidPairsIndeterminate:
    return "multi_solid_pairs_indeterminate";
  }
  return "unknown";
}

} // namespace detail
} // namespace cadstep
