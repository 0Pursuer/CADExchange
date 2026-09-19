#pragma once

#include "StepCompare.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cadstep {
namespace detail {

// Pure arithmetic over the public Point3 / Bounds3 value types.
//
// These deliberately live outside the OCCT adapter layer: they carry no shape
// handle and are shared by the tolerance policy, the descriptor matcher and the
// reports. Previously they were three separate internal-linkage definitions
// inside StepCompare.cpp, which let the tolerance formulas drift between the
// decision flow and the report writers.

inline double PointDistance(const Point3 &lhs, const Point3 &rhs) {
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double BoundsDiagonal(const Bounds3 &bounds) {
  if (bounds.isVoid) {
    return 0.0;
  }
  return PointDistance(bounds.minimum, bounds.maximum);
}

inline double ComputeBoundsDifference(const Bounds3 &lhs, const Bounds3 &rhs) {
  if (lhs.isVoid && rhs.isVoid) {
    return 0.0;
  }
  if (lhs.isVoid || rhs.isVoid) {
    return std::numeric_limits<double>::infinity();
  }
  const double minDiff = PointDistance(lhs.minimum, rhs.minimum);
  const double maxDiff = PointDistance(lhs.maximum, rhs.maximum);
  return std::max(minDiff, maxDiff);
}

} // namespace detail
} // namespace cadstep
