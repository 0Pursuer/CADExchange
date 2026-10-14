#pragma once
#include "UnifiedFeatures.h"
#include <cmath>
#include <string>

namespace CADExchange {
struct HoleResolvedGeometry {
  double shoulderDepth = 0.0;
  double tipDepth = 0.0;
  double tipHeight = 0.0;
  double entryHeight = 0.0;
};

// Resolves active dimensions only. A through hole has no finite end geometry.
inline bool ResolveHoleGeometry(const CHole &hole, HoleResolvedGeometry &out,
                                std::string &detail) {
  out = {};
  detail.clear();
  const auto positive = [](double x) { return std::isfinite(x) && x > 0; };
  const auto angle = [&](double x) {
    return positive(x) && x < std::acos(-1.0);
  };
  const auto fail = [&](const char *message) {
    detail = message;
    return false;
  };
  if (!positive(hole.diameter))
    return fail("main diameter must be finite and positive");
  if (hole.counterbore && hole.countersink)
    return fail("counterbore and countersink are mutually exclusive");
  if (hole.counterbore) {
    if (!positive(hole.counterbore->diameter) ||
        hole.counterbore->diameter <= hole.diameter ||
        !positive(hole.counterbore->depth))
      return fail("invalid counterbore diameter/depth");
    out.entryHeight = hole.counterbore->depth;
  }
  if (hole.countersink) {
    if (!positive(hole.countersink->diameter) ||
        hole.countersink->diameter <= hole.diameter ||
        !angle(hole.countersink->includedAngle))
      return fail("invalid countersink diameter/full angle");
    out.entryHeight = (hole.countersink->diameter - hole.diameter) /
                      (2 * std::tan(hole.countersink->includedAngle / 2));
    if (!positive(out.entryHeight))
      return fail("non-finite countersink height");
  }
  if (hole.extent.isFlip || hole.extent.isFlipMaterialSide ||
      hole.extent.hasOffset || !std::isfinite(hole.extent.offset) ||
      hole.extent.offset != 0 || hole.extent.referenceEntity ||
      hole.extent.helperPoint)
    return fail("hole v1 does not support extent flips/offsets/references");
  if (hole.extent.type == SweepExtent::Type::THROUGH_ALL)
    return true;
  if (hole.extent.type != SweepExtent::Type::VALUE)
    return fail("hole v1 supports VALUE and THROUGH_ALL only");
  if (!positive(hole.extent.value))
    return fail("blind depth must be finite and positive");
  if (hole.bottom == HoleBottomKind::Flat) {
    out.shoulderDepth = out.tipDepth = hole.extent.value;
  } else if (hole.bottom == HoleBottomKind::Conical) {
    if (!hole.drillPointAngle || !angle(*hole.drillPointAngle))
      return fail("conical bottom needs a valid full tip angle");
    out.tipHeight = hole.diameter / (2 * std::tan(*hole.drillPointAngle / 2));
    if (!positive(out.tipHeight))
      return fail("non-finite drill tip height");
    if (hole.depthReference == HoleDepthReference::Shoulder) {
      out.shoulderDepth = hole.extent.value;
      out.tipDepth = hole.extent.value + out.tipHeight;
    } else if (hole.depthReference == HoleDepthReference::Tip) {
      out.tipDepth = hole.extent.value;
      out.shoulderDepth = hole.extent.value - out.tipHeight;
    } else
      return fail("blind conical hole depth reference is unresolved");
  } else
    return fail("blind hole bottom is unresolved");
  if (!positive(out.shoulderDepth) || !positive(out.tipDepth))
    return fail("blind hole needs a positive cylindrical depth");
  if (out.entryHeight >= out.shoulderDepth)
    return fail("entrance consumes the complete main cylinder");
  return true;
}
} // namespace CADExchange
