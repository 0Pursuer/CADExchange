#pragma once
#include "UnifiedFeatures.h"
#include <cmath>
#include <cstdint>
#include <string>

namespace CADExchange {
// Canonical coordinates include the seed (0,0). CAD-specific flattening belongs
// to the native adapter; a second circular direction here is radial, not angular.
inline bool IsPatternIndexInDomain(const CPatternIndex &index, int count1,
                                   int count2, bool seedOnly) {
  return count1 > 0 && count2 > 0 && index.dir1Index >= 0 &&
         index.dir1Index < count1 && index.dir2Index >= 0 &&
         index.dir2Index < count2 &&
         (!seedOnly || index.dir1Index == 0 || index.dir2Index == 0);
}

inline std::uint64_t PatternInstanceCount(int count1, int count2, bool seedOnly) {
  if (count1 < 1 || count2 < 1) return 0;
  const auto first=static_cast<std::uint64_t>(count1);
  const auto second=static_cast<std::uint64_t>(count2);
  return seedOnly ? first+second-1 : first*second;
}

inline bool ResolveLinearPatternPitch(const CLinearPatternDir &dir,
                                      double &pitch, std::string &detail) {
  detail.clear();pitch=0;
  if (dir.count < 1 || !std::isfinite(dir.spacing) || dir.spacing < 0 ||
      (dir.count > 1 && dir.spacing <= 0)) {
    detail="Linear count must be positive and repeated spacing finite and positive";
    return false;
  }
  if (dir.spacingType != PatternSpacingType::PITCH_AND_COUNT &&
      dir.spacingType != PatternSpacingType::SPAN_AND_COUNT) {
    detail="Invalid linear spacing type";return false;
  }
  if (dir.count > 1)
    pitch=dir.spacingType==PatternSpacingType::SPAN_AND_COUNT
              ?dir.spacing/(dir.count-1):dir.spacing;
  return true;
}

inline bool ResolveCircularPatternPitch(const CCircularPatternDir &dir,
                                        double &pitch, std::string &detail) {
  detail.clear();pitch=0;
  if (dir.count < 1 || !std::isfinite(dir.angle) || dir.angle < 0 ||
      (dir.count > 1 && dir.angle <= 0)) {
    detail="Circular count must be positive and repeated angle finite and positive";
    return false;
  }
  if (dir.spacingType != PatternSpacingType::PITCH_AND_COUNT &&
      dir.spacingType != PatternSpacingType::SPAN_AND_COUNT) {
    detail="Invalid circular spacing type";return false;
  }
  if (dir.count > 1) {
    const bool closed=std::abs(dir.angle-2*GeoUtils::PI)<=1e-10;
    pitch=dir.spacingType==PatternSpacingType::SPAN_AND_COUNT
              ?dir.angle/(closed?dir.count:dir.count-1):dir.angle;
  }
  return true;
}
} // namespace CADExchange
