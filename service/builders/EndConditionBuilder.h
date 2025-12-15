#pragma once
#include "../../core/UnifiedFeatures.h"
#include "ReferenceBuilder.h"
#include <cmath>
#include <memory>

namespace CADExchange {
namespace Builder {

class EndCondition {
public:
  // 盲孔 (给定深度)
  static ExtrudeEndCondition Blind(double depth) {
    ExtrudeEndCondition c;
    c.type = ExtrudeEndCondition::Type::BLIND;
    c.depth = depth;
    return c;
  }

  // 贯穿
  static ExtrudeEndCondition ThroughAll() {
    ExtrudeEndCondition c;
    c.type = ExtrudeEndCondition::Type::THROUGH_ALL;
    return c;
  }

  // 拉伸到面，实体面
  static ExtrudeEndCondition
  UpToFace(const std::shared_ptr<CRefEntityBase> &ref, double offset = 0.0) {
    ExtrudeEndCondition c;
    c.type = ExtrudeEndCondition::Type::UP_TO_FACE;
    c.referenceEntity = ref;
    c.offset = offset;
    c.hasOffset = (std::abs(offset) > 1e-9);
    return c;
  }

  // 拉伸到基准面
  static ExtrudeEndCondition UpToRefPlane(const std::shared_ptr<CRefPlane> &ref,
                                         double offset = 0.0) {
    ExtrudeEndCondition c;
    c.type = ExtrudeEndCondition::Type::UP_TO_FACE;
    c.referenceEntity = ref;
    c.offset = offset;
    c.hasOffset = (std::abs(offset) > 1e-9);
    return c;
  }

  // 拉伸到顶点, 实体顶点
  static ExtrudeEndCondition
  UpToVertex(const std::shared_ptr<CRefEntityBase> &ref, double offset = 0.0) {
    ExtrudeEndCondition c;
    c.type = ExtrudeEndCondition::Type::UP_TO_VERTEX;
    c.referenceEntity = ref;
    c.offset = offset;
    c.hasOffset = (std::abs(offset) > 1e-9);
    return c;
  }
  // 拉伸到顶点, 基准点
  static ExtrudeEndCondition
  UpToRefPoint(const std::shared_ptr<CRefPoint> &ref, double offset = 0.0) {
    ExtrudeEndCondition c;
    c.type = ExtrudeEndCondition::Type::UP_TO_VERTEX;
    c.referenceEntity = ref;
    c.offset = offset;
    c.hasOffset = (std::abs(offset) > 1e-9);
    return c;
  }

  // 拉伸到下一面
  static ExtrudeEndCondition UpToNext() {
    ExtrudeEndCondition c;
    c.type = ExtrudeEndCondition::Type::UP_TO_NEXT;
    return c;
  }
};

} // namespace Builder
} // namespace CADExchange
