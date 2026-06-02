#pragma once

#include "FeatureBuilderBase.h"

namespace CADExchange {
namespace Builder {

class RibBuilder : public FeatureBuilderBase<CRib> {
public:
  RibBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  RibBuilder &SetSectionSketch(const std::string &sketchID) {
    if (!m_model.GetFeatureAs<CSketch>(sketchID)) {
      throw std::runtime_error("Sketch profile not found: " + sketchID);
    }
    m_feature->sectionSketchID = sketchID;
    return *this;
  }

  RibBuilder &SetThickness(double thickness) {
    if (thickness <= 0.0) {
      throw std::runtime_error("Rib thickness must be positive.");
    }
    m_feature->thickness = thickness;
    return *this;
  }

  RibBuilder &SetThicknessSideMode(RibThicknessSideMode mode) {
    m_feature->thicknessSideMode = mode;
    return *this;
  }

  RibBuilder &SetMaterialSide(RibMaterialSide side) {
    m_feature->materialSide = side;
    return *this;
  }

  RibBuilder &SetTargetBody(std::shared_ptr<CRefEntityBase> ref) {
    ValidateReference(ref);
    m_feature->targetBody = std::move(ref);
    return *this;
  }
};

} // namespace Builder
} // namespace CADExchange
