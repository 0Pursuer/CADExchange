#pragma once

#include <chrono>

namespace cadstep {
namespace detail {

template <typename Clock = std::chrono::high_resolution_clock>
double ElapsedMs(const std::chrono::time_point<Clock> &start) {
  const auto finish = Clock::now();
  return std::chrono::duration<double, std::milli>(finish - start).count();
}

} // namespace detail
} // namespace cadstep
