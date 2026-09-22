#pragma once

#include "StepCompare.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cadstep {
namespace detail {

/**
 * @defgroup geometry_math 几何纯算术
 * @brief 对公共值类型 Point3 / Bounds3 做纯算术运算的一组内联函数。
 *
 * 这些函数刻意放在 OCCT 适配层之外：不携带任何 shape 句柄，可被容差策略、
 * 描述符匹配器与报告层共用。此前它们是 StepCompare.cpp 里三份各自为政的
 * 内部链接定义，导致判定流程与报告层之间的容差公式产生了漂移。
 * @{
 */

/**
 * @brief 计算两点间的欧氏距离（单位与输入一致，本工程内为毫米）。
 * @param lhs 第一个点
 * @param rhs 第二个点
 * @return 两点间距离；输入为同一点时返回 0
 */
inline double PointDistance(const Point3 &lhs, const Point3 &rhs) {
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 计算包围盒的空间对角线长度，用作尺度归一化基准。
 * @param bounds 待测包围盒
 * @return 对角线长度；包围盒为空（isVoid）时返回 0.0
 *
 * @note 该值常作为各误差量的归一化分母，因此 isVoid 必须归零而不是返回
 *       无穷大，否则分母为无穷会掩盖真实的度量误差。
 */
inline double BoundsDiagonal(const Bounds3 &bounds) {
  if (bounds.isVoid) {
    return 0.0;
  }
  return PointDistance(bounds.minimum, bounds.maximum);
}

/**
 * @brief 比较两个包围盒的差异程度。
 * @param lhs 参考包围盒
 * @param rhs 候选包围盒
 * @return 两个包围盒对应角点距离的较大者（即"最坏方向"上的位移量）；
 *         两者都为空视为完全一致返回 0.0；仅一方为空视为不可比返回正无穷
 *
 * @note 取 max 而非角点距离的欧氏范数，是为了让单个角点的严重偏移
 *       不会被另一个角点的良好对齐所抵消。
 */
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

/** @} */ // end of geometry_math

} // namespace detail
} // namespace cadstep
