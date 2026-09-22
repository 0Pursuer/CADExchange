#pragma once

#include "StepCompare.h"
#include "domain/ComparisonModel.h"
#include "domain/TolerancePolicy.h"

#include <vector>

namespace cadstep {
namespace detail {

/**
 * @defgroup descriptor_matching 描述符贪心一对一匹配
 * @brief 面与边共用的匹配算法；两者只在"尺寸是否相当"的判据上不同。
 *
 * 面和边的匹配在类型直方图、最优/次优打分、歧义裕度、挑选顺序上完全一致，
 * 此前是两份逐字重复的 114 行实现。现在共享同一个内部实现，仅用
 * MeasureGate 参数化尺寸判据：
 *   - 面：面积相对参考面比较（相对判据）
 *   - 边：长度与距离容差做绝对比较（绝对判据）
 * @{
 */

/**
 * @brief 对参考面与候选面做贪心一对一匹配。
 * @param reference 参考侧描述符（来自归一化流程的投影）
 * @param candidate 候选侧描述符
 * @param config 提供歧义匹配裕度与相对体积容差（面判据用到）
 * @param characteristicScale 特征尺度，用于把质心/包围盒误差归一化到 [0,1]
 * @param tolerances 由 Derive() 得到的容差，提供质心与包围盒的距离判据
 * @return 匹配集合，含每对实体的度量、得分、状态与未匹配 id 列表
 *
 * @note 歧义（Ambiguous）匹配**不会**占用候选实体，候选仍可被后续参考实体选中；
 *       只有 Matched 才标记 candidateUsed。
 */
MatchCollection MatchFaceDescriptors(const std::vector<DescriptorView> &reference,
                                     const std::vector<DescriptorView> &candidate,
                                     const CompareConfig &config,
                                     double characteristicScale,
                                     const TolerancePolicy &tolerances);

/**
 * @brief 对参考边与候选边做贪心一对一匹配。
 * @param reference 参考侧描述符（仅含 comparable 的边，按 visualIndex 升序）
 * @param candidate 候选侧描述符
 * @param config 提供歧义匹配裕度
 * @param characteristicScale 特征尺度，用于归一化各误差项
 * @param tolerances 由 Derive() 得到的容差，提供长度/质心/包围盒的距离判据
 * @return 匹配集合，结构与面匹配一致
 *
 * @note 与面匹配唯一的算法差异是尺寸判据：边用绝对长度差对距离容差比较，
 *       而面用相对面积差对 `relativeVolumeTolerance * 100` 比较。
 */
MatchCollection MatchEdgeDescriptors(const std::vector<DescriptorView> &reference,
                                     const std::vector<DescriptorView> &candidate,
                                     const CompareConfig &config,
                                     double characteristicScale,
                                     const TolerancePolicy &tolerances);

/** @} */ // end of descriptor_matching

} // namespace detail
} // namespace cadstep
