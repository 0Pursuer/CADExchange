#pragma once

#include "FeatureBuilderBase.h"
#include <cmath>
#include <stdexcept>

namespace CADExchange {
namespace Builder {

/**
 * @brief 旋转构建器，用于描述旋转轴与角度。
 */
class RevolveBuilder : public FeatureBuilderBase<CRevolve> {
public:
  RevolveBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  RevolveBuilder &SetProfile(const std::string &sketchID) {
    auto sketch = m_model.GetFeatureAs<CSketch>(sketchID);
    if (!sketch) {
      throw std::runtime_error("Sketch profile not found: " + sketchID);
    }
    m_feature->profileSketchID = sketchID;
    return *this;
  }

  RevolveBuilder &SetAxisFromSketchLine(const std::string &sketchLineID) {

    m_feature->axis.referenceLocalID = sketchLineID;
    return *this;
  }

  RevolveBuilder &SetAxisExplicit(const CPoint3D &origin,
                                  const CVector3D &direction) {
    double axisLength = std::sqrt(direction.x * direction.x +
                                  direction.y * direction.y +
                                  direction.z * direction.z);
    if (axisLength < 1e-9) {
      throw std::runtime_error("Axis direction vector is too small (near zero).");
    }
    m_feature->axis.origin = origin;
    m_feature->axis.direction = direction;
    m_feature->axis.direction.Normalize();
    return *this;
  }

  RevolveBuilder &SetAxisRef(std::shared_ptr<CRefEntityBase> edgeRef) {
    // 验证引用实体的合法性
    ValidateReference(edgeRef);

    m_feature->axis.referenceEntity = std::move(edgeRef);
    return *this;
  }

  RevolveBuilder &SetOperation(BooleanOp op) {
    m_feature->operation = op;
    return *this;
  }

  RevolveBuilder &SetThinWall(double thickness, bool isOneSided = true,
                              bool isOutward = false, bool isCovered = false) {
    if (thickness <= 0) {
      throw std::runtime_error("Thickness must be positive.");
    }
    if (isOneSided) {
      return SetThinWallOffsets(isOutward ? 0.0 : thickness,
                                isOutward ? thickness : 0.0,
                                isCovered);
    }
    return SetThinWallOffsets(thickness, thickness, isCovered);
  }

  RevolveBuilder &SetThinWallOffsets(double startOffset, double endOffset,
                                     bool isCovered = false) {
    const double absStart = std::fabs(startOffset);
    const double absEnd = std::fabs(endOffset);
    if (absStart <= 1e-9 && absEnd <= 1e-9) {
      throw std::runtime_error("Thin wall offsets must not both be zero.");
    }
    ThinWallOption tw;
    tw.isCovered = isCovered;
    tw.startOffset = startOffset;
    tw.endOffset = endOffset;
    m_feature->thinWall = tw;
    return *this;
  }

  RevolveBuilder &SetAngle(double angle) {
    m_feature->extent1.type = SweepExtent::Type::VALUE;
    m_feature->extent1.value = angle;
    m_feature->extent2.reset();
    return *this;
  }

  RevolveBuilder &SetTwoWayAngle(double angle1, double angle2) {
    m_feature->extent1.type = SweepExtent::Type::VALUE;
    m_feature->extent1.value = angle1;
    m_feature->extent2 = SweepExtent{};
    m_feature->extent2->type = SweepExtent::Type::VALUE;
    m_feature->extent2->value = angle2;
    return *this;
  }

  RevolveBuilder &SetSymmetricAngle(double totalAngle) {
    m_feature->extent1.type = SweepExtent::Type::SYMMETRIC;
    m_feature->extent1.value = totalAngle;
    m_feature->extent2.reset();
    return *this;
  }

  RevolveBuilder &SetExtent1(const SweepExtent &extent) {
    if (extent.type == SweepExtent::Type::UNKNOWN) {
      return *this;
    }
    ValidateReference(extent.referenceEntity);
    m_feature->extent1 = extent;
    return *this;
  }

  RevolveBuilder &SetExtent2(const SweepExtent &extent) {
    if (extent.type == SweepExtent::Type::UNKNOWN) {
      return *this;
    }
    ValidateReference(extent.referenceEntity);
    m_feature->extent2 = extent;
    return *this;
  }
};

} // namespace Builder
} // namespace CADExchange
