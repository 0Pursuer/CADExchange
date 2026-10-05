#pragma once

namespace cadstep {
namespace detail {

/**
 * @brief 标识比较判定流程中最终产生 CompareResult::status 的那条分支。
 *
 * ToString() 返回的拼写会原样写入 result.json 的 overall.decision_path
 * 以及人类可读摘要，因此它属于**可观察输出契约**，任何一项的字符串都不得改动。
 *
 * @note 全局度量失败后仍执行布尔校验；若布尔证据不可用，
 *       GlobalMetricsFailed 保留由全局度量证明的 DIFFERENT 结论。
 */
enum class DecisionCode {
  BooleanAfterNormalization,      ///< 对归一化后的实体做布尔差分得出结论
  BooleanAfterOriginal,           ///< 归一化不可用时对原始实体做布尔差分
  GlobalMetricsFailed,            ///< 全局度量不通过
  NormalizedTopologyFastPath,     ///< 归一化拓扑全匹配，走快路径直接判等，唯一会提前返回的分支
  BooleanFailed,                  ///< 布尔差分运算本身失败，结论为 Indeterminate
  BooleanAfterOriginalFallback,   ///< 归一化实体差分超差、但原始实体差分通过，判定相等
  BooleanConservationInvalid,     ///< 全局度量通过，但布尔体积守恒校验失败
  BooleanDifference,              ///< 几何或布尔判据不通过，结论为 Different
  InputInvalid,                   ///< 输入文件无法读取或不是合法 STEP
  InputUnsupported,               ///< 输入形状不受支持（非单一封闭实体等）
  InputUnsupportedPolicy,         ///< 多实体策略为 CollectionOnly（尚未实现）
  MultiSolidUnmatched,            ///< 多实体配对存在未匹配的实体
  MultiSolidPairsEqual,           ///< 多实体：全部配对均为 EQUAL
  MultiSolidPairDifferent,        ///< 多实体：存在配对为 DIFFERENT
  MultiSolidPairsLikelyEqual,     ///< 多实体：全部配对成功但存在 LIKELY_EQUAL
  MultiSolidPairsIndeterminate,   ///< 多实体：全部配对成功但结论不确定
};

/**
 * @brief 返回指定判定码写入 result.json 的字符串拼写。
 * @param code 判定码
 * @return 静态字符串，如 "boolean_after_normalization"；拼写属于输出契约
 * @see DecisionCode 关于哪些值实际可达的说明
 */
const char *ToString(DecisionCode code);

} // namespace detail
} // namespace cadstep
