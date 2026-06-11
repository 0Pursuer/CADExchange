#pragma once

#include "../../core/UnifiedFeatures.h"
#include "../../core/UnifiedModel.h"
#include "../../thirdParty/json/single_include/nlohmann/json.hpp"
#include <string>
#include <vector>

namespace CADExchange {
namespace Geometry {

using json = nlohmann::json;

namespace detail {

using json = nlohmann::json;

json PointToJson(const CPoint3D &pt);
json VectorToJson(const CVector3D &vec);
bool TryReadPoint(const json &node, CPoint3D &pt);
bool TryReadVector(const json &node, CVector3D &vec);
std::string CurveTypeToString(CGeoCurveType type);
CGeoCurveType CurveTypeFromString(const std::string &typeName);

} // namespace detail

struct HalfStructurePointGroup {
  CPoint3D center{};
  double radius = 0;
  std::vector<CPoint3D> points;
};

// Normalized arc definition used for merging and comparison
struct NormalizedArc {
  CPoint3D center{};
  double radius = 0;
  CPoint3D startPt{};
  CPoint3D endPt{};
  CGeoCurveType curveType = CGeoCurveType::UNKNOWN;
};

struct CircleType {
  CPoint3D center{};
  double radius = 0;
  CGeoCurveType curveType = CGeoCurveType::UNKNOWN;
};

} // namespace Geometry
} // namespace CADExchange
