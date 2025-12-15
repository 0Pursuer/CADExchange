#pragma once
// clang-format off
#include "UnifiedTypes.h"
#include <type_traits>
// clang-format on
namespace CADExchange {

/**
 * @brief PointAdapter traits for converting user-defined point types to
 * CPoint3D. Users should specialize this template for their own point types.
 */
template <typename T, typename Enable = void> struct PointAdapter {
  // Default implementation assumes x, y, z members
  static CPoint3D Convert(const T &pt) {
    return {static_cast<double>(pt.x), static_cast<double>(pt.y),
            static_cast<double>(pt.z)};
  }
};

// Specialization for CPoint3D itself
template <> struct PointAdapter<CPoint3D> {
  static CPoint3D Convert(const CPoint3D &pt) { return pt; }
};

/**
 * @brief 对于数组类型到点类型的特化
 */
template <typename T, std::size_t N> struct PointAdapter<T[N],
                                    typename std::enable_if<N == 3>::type> {
  static CPoint3D Convert(const T (&arr)[N]) {
    return {static_cast<double>(arr[0]), static_cast<double>(arr[1]),
            static_cast<double>(arr[2])};
  }
};


/**
 * @brief VectorAdapter traits for converting user-defined vector types to
 * CVector3D. Users should specialize this template for their own vector types.
 */
template <typename T, typename Enable = void> struct VectorAdapter {
  // Default implementation assumes x, y, z members
  static CVector3D Convert(const T &vec) {
    return {static_cast<double>(vec.x), static_cast<double>(vec.y),
            static_cast<double>(vec.z)};
  }
};

// Specialization for CVector3D itself
template <> struct VectorAdapter<CVector3D> {
  static CVector3D Convert(const CVector3D &vec) { return vec; }
};

} // namespace CADExchange
