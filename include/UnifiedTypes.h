#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace CADExchange {

/**
 * @brief Layer 4: 几何助手 (GeoUtils)，放在基础类型头文件中以便全局使用。
 */
namespace GeoUtils {
/// 几何比较容差，用于浮点判断。
constexpr double EPSILON = 1e-6;
/// PI 常数。
constexpr double PI = 3.14159265358979323846;

inline double DegreesToRadians(double degrees) { return degrees * PI / 180.0; }

inline double RadiansToDegrees(double radians) { return radians * 180.0 / PI; }
} // namespace GeoUtils

/**
 * @brief 单位枚举，供几何数据统一使用。
 */
enum class UnitType {
  METER,      ///< 米
  CENTIMETER, ///< 厘米
  MILLIMETER, ///< 毫米
  INCH,       ///< 英寸
  FOOT        ///< 英尺
};

struct CPoint3D {
  double x{};
  double y{};
  double z{};

  bool operator==(const CPoint3D &other) const {
    return std::abs(x - other.x) < GeoUtils::EPSILON &&
           std::abs(y - other.y) < GeoUtils::EPSILON &&
           std::abs(z - other.z) < GeoUtils::EPSILON;
  }
};

struct CVector3D {
  double x{};
  double y{};
  double z{};

  void Normalize() {
    double len = std::sqrt(x * x + y * y + z * z);
    if (len > GeoUtils::EPSILON) {
      x /= len;
      y /= len;
      z /= len;
    }
  }

  static CVector3D CreateNormalized(double x, double y, double z) {
    CVector3D v{x, y, z};
    v.Normalize();
    return v;
  }

  static CVector3D Cross(const CVector3D &a, const CVector3D &b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
  }
};

/**
 * @brief 统一的标准基准 ID 及辅助匹配逻辑，用于屏蔽不同 CAD 系统默认平面差异。
 */
namespace StandardID {
inline constexpr const char *PLANE_XY = "STD_DATUM_XY"; ///< 法向(0,0,1)
inline constexpr const char *PLANE_YZ = "STD_DATUM_YZ"; ///< 法向(1,0,0)
inline constexpr const char *PLANE_ZX = "STD_DATUM_ZX"; ///< 法向(0,1,0)

inline constexpr const char *AXIS_X = "STD_AXIS_X";
inline constexpr const char *AXIS_Y = "STD_AXIS_Y";
inline constexpr const char *AXIS_Z = "STD_AXIS_Z";

inline constexpr const char *ORIGIN = "STD_POINT_ORIGIN";

inline constexpr CVector3D kPlaneXYNormal{0.0, 0.0, 1.0};
inline constexpr CVector3D kPlaneYZNormal{1.0, 0.0, 0.0};
inline constexpr CVector3D kPlaneZXNormal{0.0, 1.0, 0.0};

inline constexpr CVector3D kAxisX{1.0, 0.0, 0.0};
inline constexpr CVector3D kAxisY{0.0, 1.0, 0.0};
inline constexpr CVector3D kAxisZ{0.0, 0.0, 1.0};

/**
 * @brief 计算两个向量的点积。
 */
inline double Dot(const CVector3D &a, const CVector3D &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * @brief 判断两个向量是否平行或反向。
 */
inline bool IsParallel(const CVector3D &a, const CVector3D &b) {
  CVector3D na = a;
  CVector3D nb = b;
  na.Normalize();
  nb.Normalize();
  return std::abs(std::abs(Dot(na, nb)) - 1.0) < (GeoUtils::EPSILON * 10);
}

/**
 * @brief 将法向量映射为标准基准面 ID。
 *
 * @return 识别成功返回标准 ID，否则返回 nullopt。
 */
inline std::optional<std::string> MatchPlane(const CVector3D &normal) {
  if (IsParallel(normal, kPlaneXYNormal)) {
    return PLANE_XY;
  }
  if (IsParallel(normal, kPlaneYZNormal)) {
    return PLANE_YZ;
  }
  if (IsParallel(normal, kPlaneZXNormal)) {
    return PLANE_ZX;
  }
  return std::nullopt;
}

/**
 * @brief 将轴向向量映射为标准方向 ID。
 */
inline std::optional<std::string> MatchAxis(const CVector3D &direction) {
  if (IsParallel(direction, kAxisX)) {
    return AXIS_X;
  }
  if (IsParallel(direction, kAxisY)) {
    return AXIS_Y;
  }
  if (IsParallel(direction, kAxisZ)) {
    return AXIS_Z;
  }
  return std::nullopt;
}
} // namespace StandardID

/**
 * @brief 引用实体类型枚举，用于区分拓扑引用对象。
 */
enum class RefEntityType {
  POINT,
  EDGE,
  FACE,
  SKETCH,
  SKETCH_POINT,
  SKETCH_LINE,
  DATUM_PLANE,
  DATUM_AXIS,
  DATUM_POINT,
  UNKNOWN
};

} // namespace CADExchange
