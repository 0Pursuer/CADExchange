#pragma once

/**
 * @file BuilderMacros.h
 * @brief Builder类通用宏定义，用于快速生成链式API方法。
 * 
 * 这个文件提供了宏，可以为Builder类快速添加支持多种输入类型的setter方法。
 * 包括三参数直接版本和模板版本，利用TypeAdapters自动转换不同类型。
 * 
 * 使用场景：
 * - 为任何需要点或向量属性的Builder类添加链式setter
 * - 支持三参数(double x, y, z)和模板版本(任意点/向量类型)
 * - 自动利用TypeAdapters进行类型转换
 * 
 * @see TypeAdapters.h for point/vector type conversion
 */

namespace CADExchange {
namespace Builder {

/**
 * @brief 为Builder类快速添加点属性的setter（三参数 + 模板版本）
 * 
 * 生成两个重载方法：
 * 1. 接受三个double参数的直接版本 - 用于简单的坐标赋值
 * 2. 接受任意点类型的模板版本 - 通过PointAdapter自动转换
 * 
 * 使用示例：
 * @code
 *   class MyBuilder {
 *       std::shared_ptr<MyData> m_ptr;
 *   public:
 *       // 在类定义中使用宏
 *       BUILDER_ADD_POINT_SETTER(MyBuilder, origin, Origin)
 *       // 自动生成：
 *       // MyBuilder &Origin(double x, double y, double z) { ... }
 *       // template <typename PointT> MyBuilder &Origin(const PointT &pt) { ... }
 *   };
 * @endcode
 * 
 * @param ClassName 声明这个方法的Builder类名
 * @param MemberName 类内的数据成员名（必须是CPoint3D或兼容类型）
 * @param MethodName 生成的setter方法名
 * 
 * 生成的代码支持以下调用：
 * - builder.Origin(0, 0, 0)                    // 三参数
 * - builder.Origin(CPoint3D{0, 0, 0})         // CPoint3D
 * - builder.Origin(std::array<double, 3>{})   // std::array
 * - builder.Origin(Eigen::Vector3d{})         // Eigen向量等
 */
#define BUILDER_ADD_POINT_SETTER(ClassName, MemberName, MethodName) \
  ClassName &MethodName(double x, double y, double z) { \
    m_ptr->MemberName = {x, y, z}; \
    return *this; \
  } \
  template <typename PointT> \
  ClassName &MethodName(const PointT &pt) { \
    m_ptr->MemberName = PointAdapter<PointT>::Convert(pt); \
    return *this; \
  }

/**
 * @brief 为Builder类快速添加向量属性的setter（三参数 + 模板版本）
 * 
 * 生成两个重载方法：
 * 1. 接受三个double参数的直接版本 - 用于简单的方向赋值
 * 2. 接受任意向量类型的模板版本 - 通过VectorAdapter自动转换
 * 
 * 使用示例：
 * @code
 *   class MyBuilder {
 *       std::shared_ptr<MyData> m_ptr;
 *   public:
 *       // 在类定义中使用宏
 *       BUILDER_ADD_VECTOR_SETTER(MyBuilder, normal, Normal)
 *       // 自动生成：
 *       // MyBuilder &Normal(double x, double y, double z) { ... }
 *       // template <typename VectorT> MyBuilder &Normal(const VectorT &dir) { ... }
 *   };
 * @endcode
 * 
 * @param ClassName 声明这个方法的Builder类名
 * @param MemberName 类内的数据成员名（必须是CVector3D或兼容类型）
 * @param MethodName 生成的setter方法名
 * 
 * 生成的代码支持以下调用：
 * - builder.Normal(0, 0, 1)                    // 三参数
 * - builder.Normal(CVector3D{0, 0, 1})        // CVector3D
 * - builder.Normal(std::array<double, 3>{})   // std::array
 * - builder.Normal(Eigen::Vector3d{})         // Eigen向量等
 */
#define BUILDER_ADD_VECTOR_SETTER(ClassName, MemberName, MethodName) \
  ClassName &MethodName(double x, double y, double z) { \
    m_ptr->MemberName = {x, y, z}; \
    return *this; \
  } \
  template <typename VectorT> \
  ClassName &MethodName(const VectorT &dir) { \
    m_ptr->MemberName = VectorAdapter<VectorT>::Convert(dir); \
    return *this; \
  }

} // namespace Builder
} // namespace CADExchange
