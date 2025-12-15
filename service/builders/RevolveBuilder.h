#pragma once

#include "FeatureBuilderBase.h"
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

  RevolveBuilder &SetProfileByExternalID(const std::string &externalID) {
    auto sketch = m_model.GetFeatureByExternalIDAs<CSketch>(externalID);
    if (!sketch) {
      throw std::runtime_error("Sketch profile not found by external ID: " +
                               externalID);
    }
    m_feature->profileSketchID = sketch->featureID;
    return *this;
  }

  RevolveBuilder &SetAxisFromSketchLine(const std::string &sketchLineID) {

    m_feature->axis.referenceLocalID = sketchLineID;
    return *this;
  }

  RevolveBuilder &SetAxisExplicit(const CPoint3D &origin,
                                  const CVector3D &direction) {

    m_feature->axis.origin = origin;
    m_feature->axis.direction = direction;
    m_feature->axis.direction.Normalize();
    return *this;
  }

  RevolveBuilder &SetAxisRef(std::shared_ptr<CRefEntityBase> edgeRef) {

    m_feature->axis.referenceEntity = std::move(edgeRef);
    return *this;
  }

  RevolveBuilder &SetAngle(double angle) {
    m_feature->angleKind = CRevolve::AngleKind::Single;
    m_feature->primaryAngle = angle;
    m_feature->secondaryAngle = 0.0;
    return *this;
  }

  RevolveBuilder &SetTwoWayAngle(double angle1, double angle2) {
    m_feature->angleKind = CRevolve::AngleKind::TwoWay;
    m_feature->primaryAngle = angle1;
    m_feature->secondaryAngle = angle2;
    return *this;
  }

  RevolveBuilder &SetSymmetricAngle(double totalAngle) {
    m_feature->angleKind = CRevolve::AngleKind::Symmetric;
    m_feature->primaryAngle = totalAngle;
    m_feature->secondaryAngle = totalAngle;
    return *this;
  }
};

} // namespace Builder
} // namespace CADExchange
