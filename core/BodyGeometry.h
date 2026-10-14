#pragma once
#include "UnifiedFeatures.h"
#include <algorithm>
#include <cmath>
#include <string>

namespace CADExchange {
// Geometry fingerprint only: the native CAD must additionally verify trimmed
// surface membership for every point when locating the unique solid.
inline bool ValidateBodyReference(const CRefBody &body, std::string &detail) {
  detail.clear();
  const auto fail = [&](const char *message) {
    detail = message;
    return false;
  };
  const auto finite = [](const CPoint3D &p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  };
  if (body.refType != RefType::TOPO_BODY)
    return fail("Body reference has wrong RefType");
  if (!finite(body.boundsMin) || !finite(body.boundsMax) ||
      !finite(body.volumeCentroid))
    return fail("Body bounds and volume centroid must be finite");
  const auto &lo = body.boundsMin;
  const auto &hi = body.boundsMax;
  if (!(lo.x < hi.x && lo.y < hi.y && lo.z < hi.z))
    return fail("Closed solid bounds must have positive extent in all axes");
  if (!std::isfinite(body.volume) || body.volume <= 0 ||
      !std::isfinite(body.surfaceArea) || body.surfaceArea <= 0)
    return fail(
        "Closed solid volume and surface area must be finite and positive");
  const double scale = (std::max)({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z});
  if (!std::isfinite(scale))
    return fail("Body bounds span must be finite");
  const double tolerance = scale * 1e-10;
  const auto inBounds = [&](const CPoint3D &p) {
    return p.x >= lo.x - tolerance && p.x <= hi.x + tolerance &&
           p.y >= lo.y - tolerance && p.y <= hi.y + tolerance &&
           p.z >= lo.z - tolerance && p.z <= hi.z + tolerance;
  };
  if (!inBounds(body.volumeCentroid))
    return fail("Volume centroid must be inside body bounds");
  if (body.surfacePoints.size() < 3)
    return fail("Body needs at least three surface points");
  for (const auto &p : body.surfacePoints)
    if (!finite(p) || !inBounds(p))
      return fail("Body surface points must be finite and inside bounds");
  const auto &a = body.surfacePoints.front();
  for (size_t i = 1; i < body.surfacePoints.size(); ++i) {
    const auto &b = body.surfacePoints[i];
    CVector3D u{(b.x - a.x) / scale, (b.y - a.y) / scale, (b.z - a.z) / scale};
    const double u2 = u.Dot(u);
    if (u2 <= 1e-20)
      continue;
    for (size_t j = i + 1; j < body.surfacePoints.size(); ++j) {
      const auto &c = body.surfacePoints[j];
      CVector3D v{(c.x - a.x) / scale, (c.y - a.y) / scale,
                  (c.z - a.z) / scale};
      const auto cross = u.Cross(v);
      if (cross.Dot(cross) > 1e-20 * u2 * v.Dot(v))
        return true;
    }
  }
  return fail(
      "Body surface points must contain three distinct noncollinear points");
}
} // namespace CADExchange
