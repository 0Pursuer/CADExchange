#pragma once
#include "../../core/HoleGeometry.h"
#include "BuilderMacros.h"
#include "FeatureBuilderBase.h"

namespace CADExchange::Builder {
class HoleBuilder : public FeatureBuilderBase<CHole> {
public:
  HoleBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}
  HoleBuilder &SetSupportReference(std::shared_ptr<CRefEntityBase> ref) {
    ValidateReference(ref);
    m_feature->supportRef = std::move(ref);
    return *this;
  }
  HoleBuilder &SetDirection(const CVector3D &v) {
    m_feature->direction = v;
    return *this;
  }
  HoleBuilder &AddLocation(const CPoint3D &p,
                           std::shared_ptr<CRefEntityBase> ref = nullptr) {
    ValidateReference(ref);
    m_feature->locations.push_back({p, std::move(ref)});
    return *this;
  }
  HoleBuilder &SetDiameter(double v) {
    m_feature->diameter = v;
    return *this;
  }
  HoleBuilder &SetExtent(const SweepExtent &v) {
    m_feature->extent = v;
    return *this;
  }
  HoleBuilder &SetBottomKind(HoleBottomKind v) {
    m_feature->bottom = v;
    return *this;
  }
  HoleBuilder &SetDepthReference(HoleDepthReference v) {
    m_feature->depthReference = v;
    return *this;
  }
  HoleBuilder &SetDrillPointAngle(double v) {
    m_feature->drillPointAngle = v;
    return *this;
  }
  HoleBuilder &SetCounterbore(double diameter, double depth) {
    m_feature->counterbore = CHoleCounterbore{diameter, depth};
    return *this;
  }
  HoleBuilder &SetCountersink(double diameter, double angle) {
    m_feature->countersink = CHoleCountersink{diameter, angle};
    return *this;
  }
};
} // namespace CADExchange::Builder
