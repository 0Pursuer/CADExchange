#pragma once

#include "StepCompare.h"

#include <TopoDS_Solid.hxx>

#include <filesystem>
#include <string>

namespace cadstep {
namespace detail {

/**
 * @brief 对一对实体跑完整的四阶段比较流程，返回已填充的 CompareResult。
 *
 * 四个阶段依次执行、逐级向下传递，只有归一化快路径会提前终止：
 *   1. 归一化 + 全局度量     —— 可能置位 GlobalMetricsFailed 闩锁
 *   2. 描述符匹配            —— 可能经快路径直接判等并提前返回
 *   3. 布尔校验              —— 快路径已判定时跳过
 *   4. 产物导出（STL/BREP/VTP）
 *
 * @param solidRef 参考侧原始实体
 * @param auditRef 参考侧输入审计（体积/质心/包围盒等，供容差推导与全局度量）
 * @param solidCand 候选侧原始实体
 * @param auditCand 候选侧输入审计
 * @param config 比较配置
 * @param outputDirectory 产物输出目录；为空则跳过阶段 4
 * @param artifactSuffix 产物文件名后缀，多实体路径用它区分各配对的产物；
 *                       单实体路径传空串
 * @return 已填充判定结论、审计明细、计时与产物清单的 CompareResult
 *
 * @note 判定结论在本函数内部以 ComparisonDecision 累积，函数末尾一次性写入
 *       result.status / result.reason / result.decisionPath，中途不回读。
 */
CompareResult CompareSolidPairInternal(const TopoDS_Solid &solidRef,
                                       const InputAudit &auditRef,
                                       const TopoDS_Solid &solidCand,
                                       const InputAudit &auditCand,
                                       const CompareConfig &config,
                                       const std::filesystem::path &outputDirectory,
                                       const std::string &artifactSuffix);


} // namespace detail
} // namespace cadstep
