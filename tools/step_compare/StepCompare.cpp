#include "StepCompare.h"

#include <iomanip>
#include <sstream>
#include <string>

namespace cadstep {

// 各 ToString() 重载返回的拼写会原样写入 result.json，属输出契约，不得改动。

const char *ToString(EdgeComparisonRole role) {
  switch (role) {
    case EdgeComparisonRole::Comparable: return "COMPARABLE";
    case EdgeComparisonRole::PeriodicSeam: return "PERIODIC_SEAM";
    case EdgeComparisonRole::Degenerated: return "DEGENERATED";
    case EdgeComparisonRole::Unsupported: return "UNSUPPORTED";
    default: return "UNKNOWN";
  }
}

// id 前缀（ref:/cand:）+ 种类段（face:/edge:/nface:/nedge:）+ 6 位零填充序号。
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

// 退出码与 CompareStatus 枚举声明顺序对应；UI/CI 依赖这套数值，不得改动。
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

// 有意忽略 consistency 参数（上游提交 01f2556 的行为）：四项判据全过即
// Equal，否则 Different，LikelyEqual 分支不可达；配套测试有一条因此按原样失败。
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
