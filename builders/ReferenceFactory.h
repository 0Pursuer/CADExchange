#pragma once

#include "../core/UnifiedFeatures.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>

namespace CADExchange {
namespace Builder {

// ==========================================
// 引用类型定义 (Moved from API)
// ==========================================
struct Point3D {
  double x, y, z;
};
struct Vector3D {
  double x, y, z;
};

enum class ReferenceKind {
  Face,
  Edge,
  Vertex,
  SketchSegment,
  Sketch,
  DatumPlane,
  DatumAxis,
  DatumPoint
};

struct ReferenceAttributeValue {
  using VariantType = std::variant<std::string, int, double, Point3D, Vector3D>;

  ReferenceAttributeValue(const std::string &value) : value(value) {}
  ReferenceAttributeValue(const char *value) : value(std::string(value)) {}
  ReferenceAttributeValue(int value) : value(value) {}
  ReferenceAttributeValue(double value) : value(value) {}
  ReferenceAttributeValue(Point3D value) : value(value) {}
  ReferenceAttributeValue(Vector3D value) : value(value) {}

  template <typename T> const T *GetAs() const {
    return std::get_if<T>(&value);
  }

  VariantType value;
};

using ReferenceAttributeMap =
    std::unordered_map<std::string, ReferenceAttributeValue>;

class ReferenceFactory {
public:
  static std::shared_ptr<CRefEntityBase>
  Create(ReferenceKind kind, const ReferenceAttributeMap &attributes,
         std::string *outError = nullptr);
};

} // namespace Builder
} // namespace CADExchange
