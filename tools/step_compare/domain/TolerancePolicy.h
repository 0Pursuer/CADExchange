#pragma once

#include "StepCompare.h"

namespace cadstep {
namespace detail {

/**
 * @brief 自适应容差模型的唯一真源。
 *
 * 模型为「静态下限 + 特征尺度相对比例」，规范见
 * CADExchange/doc/STEP终态实体几何对比规范与自适应容差说明_20260804.md。
 *
 * 由它派生的阈值与通过/不通过判据同时被判定流程和两个报告层消费。历史上
 * 这三处各自实现了一遍公式，导致报告里渲染的 passed/failed 标志与真正产生
 * 判定结论的标志发生漂移——本类就是为了消灭这种漂移。
 *
 * @note 判定流程只应通过 Derive() 取得实例，不应自行组合 config 字段。
 */
class TolerancePolicy {
public:
  /**
   * @brief 依据比较配置与参考实体的输入审计推导出一组容差。
   * @param config 比较配置（提供静态下限与相对比例等开关）
   * @param reference 参考实体的输入审计（提供特征尺度所需的体积/包围盒）
   * @return 已填充好的容差策略实例
   */
  static TolerancePolicy Derive(const CompareConfig &config,
                                const InputAudit &reference);

  /// @brief 距离类判据的容差（毫米），用于质心距离、包围盒差与边长度差
  double distanceMm() const { return distanceToleranceMm_; }
  /// @brief 体积差判据的绝对容差（立方毫米），用于布尔差分残差
  double absoluteVolumeMm3() const { return absoluteVolumeToleranceMm3_; }
  /// @brief 体积差判据的相对容差（无量纲），用于输入体积相对差
  double relativeVolume() const { return relativeVolumeTolerance_; }

private:
  double distanceToleranceMm_ = 0.0;
  double absoluteVolumeToleranceMm3_ = 0.0;
  double relativeVolumeTolerance_ = 0.0;
};

/**
 * @brief 三项全局度量判据（体积、质心、包围盒）各自的通过标志。
 *
 * 在判定流程中这里的失败是**闩锁**而非终止：它会阻止归一化快路径，但不会
 * 提前结束比较。报告层渲染的正是同一组标志，因此标志在此统一计算，
 * 而不是在每个使用点各算一遍。
 */
struct GeometryPassFlags {
  bool volume = false;   ///< 输入体积差是否在容差内
  bool centroid = false; ///< 质心距离是否在容差内
  bool bounds = false;   ///< 包围盒差是否在容差内
};

/**
 * @brief 依据全局度量结果评估三项判据。
 * @param result 已填充 globalMetricsExecuted 与各度量值的比较结果
 * @param tolerances 由 Derive() 得到的容差
 * @return 三项判据各自的通过标志
 */
GeometryPassFlags EvaluateGeometryPasses(const CompareResult &result,
                                         const TolerancePolicy &tolerances);

/**
 * @brief 布尔残差判据：两个方向的布尔差分体积都须落在绝对容差内。
 * @param result 含 booleanExecuted 与两个方向的残差体积
 * @param tolerances 由 Derive() 得到的容差
 * @return 是否通过；未执行布尔步骤时返回 true 而非失败
 *
 * @note 「未执行即通过」对应报告层历史上 `!booleanExecuted || ...` 的写法，
 *       二者必须保持一致，否则报告的 passed 标志会与判定结论脱节。
 */
bool EvaluateBooleanPass(const CompareResult &result,
                         const TolerancePolicy &tolerances);

} // namespace detail
} // namespace cadstep
