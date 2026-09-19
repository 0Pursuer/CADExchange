#pragma once

namespace cadstep {
namespace detail {

// Identifies which branch of the comparison decision flow produced the final
// CompareResult::status.
//
// The spelling returned by ToString() is written verbatim into result.json
// ("overall.decision_path") and into the human summary, so it is part of the
// observable output contract and must not change.
//
// Note that GlobalMetricsFailed is an intermediate latch rather than a terminal
// branch: a failing global-metrics check blocks the normalized fast path but
// execution continues into boolean verification, which then overwrites the
// code. It is modelled here so the existing behaviour can be preserved without
// relying on free-form strings.
enum class DecisionCode {
  BooleanAfterNormalization,
  BooleanAfterOriginal,
  GlobalMetricsFailed,
  NormalizedTopologyFastPath,
  BooleanFailed,
  BooleanAfterOriginalFallback,
  BooleanConservationInvalid,
  BooleanDifference,
  InputInvalid,
  InputUnsupported,
  InputUnsupportedPolicy,
  MultiSolidUnmatched,
  MultiSolidPairsEqual,
  MultiSolidPairDifferent,
  MultiSolidPairsLikelyEqual,
  MultiSolidPairsIndeterminate,
};

// Returns the spelling emitted into result.json for the given code.
const char *ToString(DecisionCode code);

} // namespace detail
} // namespace cadstep
