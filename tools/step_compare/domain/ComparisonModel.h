#pragma once

#include "StepCompare.h"

#include <string>
#include <vector>

namespace cadstep {
namespace detail {

/**
 * @brief 描述符匹配器消费的最小几何视图。
 *
 * 刻意不含 OCCT 类型，也不含仅供审计使用的记账字段：面与边在此都退化为
 * 「类型 + 一个度量 + 质心 + 包围盒」四元组，这正是面和边能共用同一套匹配
 * 算法的前提。审计明细（NormalizedFaceInfo / NormalizedEdgeInfo）保留自己
 * 更丰富的记录；本视图由同一次几何采集投影而来，因此任何实体的几何都只被
 * 测量一次。
 *
 * @note 聚合初始化顺序与成员声明顺序一致：
 *       {id, typeName, measure, centroidMm, boundsMm}。
 *       顺序写错会静默地把 id 和度量互换，编译器不会报错。
 */
struct DescriptorView {
  std::string id;        ///< 实体 id，写入 EntityMatch::referenceId / candidateId
  std::string typeName;  ///< 表面类型串（面）或曲线类型串（边），如 "PLANE" / "LINE"
  double measure = 0.0;  ///< 度量值：面为面积 mm^2，边为长度 mm
  Point3 centroidMm;     ///< 质心（毫米）
  Bounds3 boundsMm;      ///< 轴对齐包围盒（毫米）
};

} // namespace detail
} // namespace cadstep
