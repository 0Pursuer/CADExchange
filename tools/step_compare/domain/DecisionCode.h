#pragma once

namespace cadstep {
namespace detail {

/**
 * @brief 标识比较判定流程中最终产生 CompareResult::status 的那条分支。
 *
 * ToString() 返回的拼写会原样写入 result.json 的 overall.decision_path
 * 以及人类可读摘要，因此它属于**可观察输出契约**，任何一项的字符串都不得改动。
 *
 * @note 重要的语义：GlobalMetricsFailed 是**中间闩锁**而非终态分支——
 *       全局度量不通过只会阻止归一化快路径，执行仍会继续进入布尔校验，
 *       并由后者覆盖该 code。把它建模成枚举是为了在不依赖自由字符串的
 *       前提下原样保留既有行为。
 *
 * @warning 既有行为中并非所有枚举值都可达：GlobalMetricsFailed 与
 *          BooleanConservationInvalid 在最终输出中不会出现（前者总是被
 *          布尔阶段的结论覆盖，后者因 ClassifyClosedSolidComparison 只返回
 *          Equal/Different 而不可达）。保留它们是为了如实反映流程的分支。
 */
enum class DecisionCode {
  BooleanAfterNormalization,      ///< 对归一化后的实体做布尔差分得出结论
  BooleanAfterOriginal,           ///< 归一化不可用时对原始实体做布尔差分
  GlobalMetricsFailed,            ///< 【中间闩锁】全局度量不通过（见类说明）
  NormalizedTopologyFastPath,     ///< 归一化拓扑全匹配，走快路径直接判等，唯一会提前返回的分支
  BooleanFailed,                  ///< 布尔差分运算本身失败，结论为 Indeterminate
  BooleanAfterOriginalFallback,   ///< 归一化实体差分超差、但原始实体差分通过，判定相等
  BooleanConservationInvalid,     ///< 布尔体积守恒校验失败（当前实现中不可达）
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
