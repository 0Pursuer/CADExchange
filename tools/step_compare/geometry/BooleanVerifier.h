#pragma once

#include "StepCompare.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>

namespace cadstep {
namespace detail {

/**
 * @brief 一次布尔差分的产物：差分形状与写进审计的执行明细。
 */
struct BooleanDifferenceResult {
  DifferenceAudit audit; ///< 执行结果与体积/报告明细
  TopoDS_Shape shape;    ///< 差分结果形状；失败或结果为空时保持 null
};

/**
 * @brief 计算布尔差分 `argument - tool`，用于细粒度材料对比。
 * @param argument 被减的实体
 * @param tool 减去的工具实体
 * @param fuzzyTolerance 传给 BRepAlgoAPI_Cut 的模糊容差
 * @return 结果与审计明细
 *
 * 行为要点：
 *  - argument 为 null 视为失败；tool 为 null 视为**成功且体积为 0**（不是失败）；
 *  - BRepAlgoAPI_Cut 报错或抛出异常（含 Standard_Failure）时捕获原因写入
 *    audit.report 并置 succeeded=false；
 *  - 体积取 BRepGProp::VolumeProperties 的 Mass 绝对值。
 */
BooleanDifferenceResult CutSolids(const TopoDS_Solid &argument,
                                  const TopoDS_Solid &tool,
                                  double fuzzyTolerance);


} // namespace detail
} // namespace cadstep
