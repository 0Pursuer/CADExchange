#include "StepCompare.h"

#include <iomanip>
#include <sstream>
#include <string>

namespace cadstep {

const char *ToString(EdgeComparisonRole role) {
  switch (role) {
    case EdgeComparisonRole::Comparable: return "COMPARABLE";
    case EdgeComparisonRole::PeriodicSeam: return "PERIODIC_SEAM";
    case EdgeComparisonRole::Degenerated: return "DEGENERATED";
    case EdgeComparisonRole::Unsupported: return "UNSUPPORTED";
    default: return "UNKNOWN";
  }
}

std::string MakeEntityId(EntitySide side, EntityKind kind, int index) {
  std::ostringstream ss;
  ss << (side == EntitySide::Reference ? "ref:" : "cand:");
  switch (kind) {
  case EntityKind::OriginalFace:
    ss << "face:";
    break;
  case EntityKind::OriginalEdge:
    ss << "edge:";
    break;
  case EntityKind::NormalizedFace:
    ss << "nface:";
    break;
  case EntityKind::NormalizedEdge:
    ss << "nedge:";
    break;
  }
  ss << std::setw(6) << std::setfill('0') << index;
  return ss.str();
}

const char *ToString(CompareStatus status) {
  switch (status) {
  case CompareStatus::Equal:
    return "EQUAL";
  case CompareStatus::LikelyEqual:
    return "LIKELY_EQUAL";
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
  return "UNKNOWN";
}

const char *ToString(MatchStatus status) {
  switch (status) {
  case MatchStatus::Matched:
    return "MATCHED";
  case MatchStatus::Unmatched:
    return "UNMATCHED";
  case MatchStatus::Ambiguous:
    return "AMBIGUOUS";
  case MatchStatus::Unsupported:
    return "UNSUPPORTED";
  }
  return "UNKNOWN";
}

const char *ToString(VerificationLevel level) {
  switch (level) {
  case VerificationLevel::TypeOnly:
    return "TYPE_ONLY";
  case VerificationLevel::Descriptor:
    return "DESCRIPTOR";
  case VerificationLevel::AnalyticSupport:
    return "ANALYTIC_SUPPORT";
  case VerificationLevel::DistanceVerified:
    return "DISTANCE_VERIFIED";
  case VerificationLevel::BooleanVerified:
    return "BOOLEAN_VERIFIED";
  }
  return "UNKNOWN";
}

int ExitCode(CompareStatus status) {
  switch (status) {
  case CompareStatus::Equal:
    return 0;
  case CompareStatus::LikelyEqual:
    return 3;
  case CompareStatus::Different:
    return 1;
  case CompareStatus::InvalidInput:
    return 2;
  case CompareStatus::UnsupportedShape:
    return 2;
  case CompareStatus::Indeterminate:
    return 4;
  case CompareStatus::InternalError:
    return 5;
  }
  return 5;
}

CompareStatus detail::ClassifyClosedSolidComparison(
    bool volumePass, bool centroidPass, bool boundsPass, bool booleanPass,
    const BooleanConsistencyMetrics &consistency) {
  if (volumePass && centroidPass && boundsPass && booleanPass) {
    return CompareStatus::Equal;
  }
  return CompareStatus::Different;
}

const char *ToString(MultiSolidPolicy policy) {
  switch (policy) {
  case MultiSolidPolicy::Strict:
    return "strict";
  case MultiSolidPolicy::CollectionOnly:
    return "collection";
  case MultiSolidPolicy::Pairwise:
    return "pairwise";
  }
  return "unknown";
}

} // namespace cadstep
