#pragma once
#include "../../core/UnifiedModel.h"
#include "../../core/UnifiedFeatures.h"
#include "ReferenceBuilder.h"
#include <cmath>
#include <memory>
#include <string>
#include <optional>

namespace CADExchange {
namespace Builder {

class Extent {
public:
  static SweepExtent Value(double value) {
    SweepExtent c;
    c.type = SweepExtent::Type::VALUE;
    c.value = value;
    return c;
  }

  static SweepExtent Angle(double value) { return Value(value); }

  static SweepExtent Symmetric(double value) {
    SweepExtent c;
    c.type = SweepExtent::Type::SYMMETRIC;
    c.value = value;
    return c;
  }

  static SweepExtent ThroughAll() {
    SweepExtent c;
    c.type = SweepExtent::Type::THROUGH_ALL;
    return c;
  }

  static SweepExtent ThroughAllBothSides() {
    SweepExtent c;
    c.type = SweepExtent::Type::THROUGH_ALL_BOTH_SIDES;
    return c;
  }

  static SweepExtent
  UpToEntity(const std::shared_ptr<CRefEntityBase> &ref, double offset = 0.0) {
    SweepExtent c;
    c.type = SweepExtent::Type::UP_TO_ENTITY;
    c.referenceEntity = ref;
    c.offset = offset;
    c.hasOffset = (std::abs(offset) > 1e-9);
    return c;
  }

  static SweepExtent UpToFace(const std::shared_ptr<CRefEntityBase> &ref,
                              double offset = 0.0) {
    return UpToEntity(ref, offset);
  }

  static SweepExtent UpToRefPlane(const std::shared_ptr<CRefPlane> &ref,
                                  double offset = 0.0) {
    return UpToEntity(ref, offset);
  }

  static SweepExtent UpToVertex(const std::shared_ptr<CRefEntityBase> &ref,
                                double offset = 0.0) {
    return UpToEntity(ref, offset);
  }

  static SweepExtent UpToRefPoint(const std::shared_ptr<CRefPoint> &ref,
                                  double offset = 0.0) {
    return UpToEntity(ref, offset);
  }

  static SweepExtent UpToNext() {
    SweepExtent c;
    c.type = SweepExtent::Type::UP_TO_NEXT;
    return c;
  }

};

class EndCondition {
public:
  // 盲孔 (给定深度)
  static SweepExtent Blind(double depth) {
    return Extent::Value(depth);
  }

  // 贯穿
  static SweepExtent ThroughAll() { return Extent::ThroughAll(); }

  // 拉伸到面，实体面
  static SweepExtent
  UpToFace(const std::shared_ptr<CRefEntityBase> &ref, double offset = 0.0) {
    return Extent::UpToFace(ref, offset);
  }

  // 拉伸到基准面
  static SweepExtent UpToRefPlane(const std::shared_ptr<CRefPlane> &ref,
                                  double offset = 0.0) {
    return Extent::UpToRefPlane(ref, offset);
  }

  // 拉伸到顶点, 实体顶点
  static SweepExtent
  UpToVertex(const std::shared_ptr<CRefEntityBase> &ref, double offset = 0.0) {
    return Extent::UpToVertex(ref, offset);
  }
  // 拉伸到顶点, 基准点
  static SweepExtent
  UpToRefPoint(const std::shared_ptr<CRefPoint> &ref, double offset = 0.0) {
    return Extent::UpToRefPoint(ref, offset);
  }

  // 拉伸到下一面
  static SweepExtent UpToNext() { return Extent::UpToNext(); }

  // 双向贯穿
  static SweepExtent ThroughAllBothSides() {
    return Extent::ThroughAllBothSides();
  }

};

/**
 * @brief 终止条件构造的便利工厂类。
 * 
 * 提供静态工厂方法，用于快速创建常见的拉伸终止条件，
 * 特别是在处理引用实体（顶点、面、基准面）时。
 * 
 * 使用示例：
 * @code
 *   using namespace CADExchange::Builder;
 *   auto endCond = EndCondition::UpToVertex(
 *       model, 
 *       "ParentFeatureId", 
 *       vertexPoint);
 * @endcode
 */
class EndConditionHelper {
public:
  /**
   * @brief 构造"拉伸到顶点"的终止条件。
   * 
   * @param model UnifiedModel 引用
   * @param parentFeatureId 顶点所属特征的 ID
   * @param vertexPoint 顶点的坐标
   * @param topologyIndex 顶点的拓扑索引（默认 0）
   * @param offset 偏移量（默认 0）
   * @return 构造好的 SweepExtent
   */
  static SweepExtent UpToVertex(
      UnifiedModel &model,
      const std::string &parentFeatureId,
      const CPoint3D &vertexPoint,
      int topologyIndex = 0,
      double offset = 0.0) {
    
    auto refVertex = RefVertexBuilder(parentFeatureId, topologyIndex)
        .Pos(vertexPoint)
        .Build();
    
    return EndCondition::UpToVertex(refVertex, offset);
  }

  /**
   * @brief 构造"拉伸到面"的终止条件。
   * 
   * @param parentFeatureId 面所属特征的 ID
   * @param topologyIndex 面的拓扑索引（默认 0）
   * @param offset 偏移量（默认 0）
   * @return 构造好的 SweepExtent
   */
  static SweepExtent UpToFace(
      const std::string &parentFeatureId,
      int topologyIndex = 0,
      double offset = 0.0) {
    
    auto refFace = RefFaceBuilder(parentFeatureId, topologyIndex)
        .Build();
    
    return EndCondition::UpToFace(refFace, offset);
  }

  /**
   * @brief 构造"拉伸到基准面"的终止条件。
   * 
   * @param model UnifiedModel 引用
   * @param planeFeatureId 基准面的特征 ID
   * @param origin 基准面原点
   * @param normal 基准面法向量
   * @param xDir 基准面 X 方向
   * @param offset 偏移量（默认 0）
   * @return 构造好的 SweepExtent
   */
  static SweepExtent UpToRefPlane(
      UnifiedModel &model,
      const std::string &planeFeatureId,
      const CPoint3D &origin,
      const CVector3D &normal,
      const CVector3D &xDir,
      double offset = 0.0) {
    
    auto refPlane = RefPlaneBuilder(planeFeatureId)
        .Origin(origin)
        .Normal(normal)
        .XDir(xDir)
        .Build();
    
    return EndCondition::UpToRefPlane(refPlane, offset);
  }

  /**
   * @brief 尝试将终止条件从一种类型转换为另一种。
   * 
   * @param source 源终止条件
   * @return 转换后的条件，如果无法转换则返回 std::nullopt
   */
  static std::optional<SweepExtent>
  SafeConvert(const SweepExtent &source) {
    if (source.type == SweepExtent::Type::UNKNOWN) {
      return std::nullopt;
    }
    return source;
  }
};

} // namespace Builder
} // namespace CADExchange
