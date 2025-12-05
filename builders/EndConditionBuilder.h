#pragma once
#include "../core/UnifiedFeatures.h"
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

  // 拉伸到面 (支持 Ref 工厂生成的引用)
  static ExtrudeEndCondition
  UpToSurface(const std::shared_ptr<CRefEntityBase> &ref, double offset = 0.0) {
    ExtrudeEndCondition c;
    c.type = ExtrudeEndCondition::Type::UP_TO_FACE;
    c.referenceEntity = ref;
    c.offset = offset;
    c.hasOffset = (std::abs(offset) > 1e-9);
    return c;
  }

  // 拉伸到顶点
  static ExtrudeEndCondition
  UpToVertex(const std::shared_ptr<CRefEntityBase> &ref, double offset = 0.0) {
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
