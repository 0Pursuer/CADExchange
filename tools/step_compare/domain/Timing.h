#pragma once

#include <chrono>

namespace cadstep {
namespace detail {

/**
 * @brief 计算自 start 起经过的毫秒数（浮点，保留亚毫秒精度）。
 * @tparam Clock 时钟类型，默认 std::chrono::high_resolution_clock
 * @param start 计时起点，通常在函数入口处用 Clock::now() 取得
 * @return 经过的毫秒数，double 类型
 *
 * @note 这是 timings_ms 各分桶唯一的取值方式。各桶之间存在明确的归属约定
 *       （例如归一化桶要减去其中包含的描述符投影耗时），修改计时点位置
 *       会改变对外报告的时间分布，属可观察行为变化。
 */
template <typename Clock = std::chrono::high_resolution_clock>
double ElapsedMs(const std::chrono::time_point<Clock> &start) {
  const auto finish = Clock::now();
  return std::chrono::duration<double, std::milli>(finish - start).count();
}

} // namespace detail
} // namespace cadstep
