#pragma once
// clang-format off
#include "../../core/TypeAdapters.h"
#include "FeatureBuilderBase.h"
#include "ReferenceBuilder.h"
#include <initializer_list>
#include <stdexcept>
#include <string>
// clang-format on
namespace CADExchange {
namespace Builder {

/**
 * @brief 草图构建器，隐藏内部对象创建，返回 LocalID 供约束使用。
 */
class SketchBuilder : public FeatureBuilderBase<CSketch> {
public:
  SketchBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  /**
   * @brief 设置草图的参考面。
   *
   * 支持两种参考面：
   * 1. 标准平面（内置）：Ref::XY()、Ref::YZ()、Ref::ZX()
   * 2. 自定义基准面：Ref::Plane("DatumPlane_001").Origin(0,0,0).XDir(1,0,0)
   * 3. 实体面：Ref::Face(...)
   *
   * @param ref 参考面引用（CRefPlane 或 CRefEntityBase 的任意派生类）
   * @return 自身引用，支持链式调用
   */
  SketchBuilder &SetReferencePlane(const std::shared_ptr<CRefEntityBase> &ref) {
    if (!ref) {
      throw std::invalid_argument("Reference plane cannot be null");
    }

    ValidateReference(ref);

    m_feature->referencePlane = ref;
    return *this;
  }

  /**
   * @brief 添加直线。
   */
  template <typename PointT>
  std::string AddLine(const PointT &start, const PointT &end,
                      bool isConstruction = false) {
    auto line = std::make_shared<CSketchLine>();
    line->startPos = PointAdapter<PointT>::Convert(start);
    line->endPos = PointAdapter<PointT>::Convert(end);
    line->localID = GenerateLocalID("L");
    line->isConstruction = isConstruction;
    m_feature->segments.push_back(line);
    return line->localID;
  }

  /**
   * @brief 添加圆。
   */
  template <typename PointT>
  std::string AddCircle(const PointT &center, double radius,
                        bool isConstruction = false) {
    if (radius <= 0) {
      throw std::invalid_argument("radius must be positive");
    }
    auto circle = std::make_shared<CSketchCircle>();
    circle->center = PointAdapter<PointT>::Convert(center);
    circle->radius = radius;
    circle->localID = GenerateLocalID("C");
    circle->isConstruction = isConstruction;
    m_feature->segments.push_back(circle);
    return circle->localID;
  }

  /**
   * @brief 添加圆弧。
   */
  template <typename PointT>
  std::string AddArc(const PointT &center, double radius, double startAngle,
                     double endAngle, bool isClockwise = false,
                     bool isConstruction = false) {
    if (radius <= 0) {
      throw std::invalid_argument("radius must be positive");
    }
    auto arc = std::make_shared<CSketchArc>();
    arc->center = PointAdapter<PointT>::Convert(center);
    arc->radius = radius;
    arc->startAngle = startAngle;
    arc->endAngle = endAngle;
    arc->isClockwise = isClockwise;
    arc->isConstruction = isConstruction;
    arc->localID = GenerateLocalID("A");
    m_feature->segments.push_back(arc);
    return arc->localID;
  }

  /**
   * @brief 添加点。
   */
  template <typename PointT> std::string AddPoint(const PointT &pos) {
    auto point = std::make_shared<CSketchPoint>();
    point->position = PointAdapter<PointT>::Convert(pos);
    point->localID = GenerateLocalID("P");
    m_feature->segments.push_back(point);
    return point->localID;
  }

  /**
   * @brief 添加重合约束。
   */
  SketchBuilder &AddCoincident(const std::string &entityID1,
                               const std::string &entityID2) {
    return AddConstraint(CSketchConstraint::ConstraintType::COINCIDENT,
                         {entityID1, entityID2});
  }

  /**
   * @brief 添加水平约束。
   */
  SketchBuilder &AddHorizontal(const std::string &lineID) {
    return AddConstraint(CSketchConstraint::ConstraintType::HORIZONTAL,
                         {lineID});
  }

  /**
   * @brief 添加垂直约束。
   */
  SketchBuilder &AddVertical(const std::string &lineID) {
    return AddConstraint(CSketchConstraint::ConstraintType::VERTICAL, {lineID});
  }

  /**
   * @brief 添加相切约束。
   */
  SketchBuilder &AddTangent(const std::string &entityID1,
                            const std::string &entityID2) {
    return AddConstraint(CSketchConstraint::ConstraintType::TANGENT,
                         {entityID1, entityID2});
  }

  /**
   * @brief 添加尺寸约束。
   */
  SketchBuilder &AddDistanceDimension(const std::string &entityID1,
                                      const std::string &entityID2,
                                      double value) {
    return AddConstraint(CSketchConstraint::ConstraintType::DIMENSIONAL,
                         {entityID1, entityID2}, value);
  }

private:
  int m_localCounter = 0;

  std::string GenerateLocalID(const std::string &prefix) {
    return prefix + "_" + std::to_string(++m_localCounter);
  }

  SketchBuilder &AddConstraint(CSketchConstraint::ConstraintType type,
                               std::initializer_list<std::string> ids,
                               double value = 0.0) {
    CSketchConstraint constraint;
    constraint.type = type;
    constraint.entityLocalIDs.assign(ids);
    constraint.dimensionValue = value;
    m_feature->constraints.push_back(constraint);
    return *this;
  }
};

} // namespace Builder
} // namespace CADExchange
