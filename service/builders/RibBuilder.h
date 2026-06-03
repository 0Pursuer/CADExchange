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
    m_feature->sketchID = sketchID;
    return *this;
  }

  RibBuilder &SetThicknessOption(const RibThicknessOption &opt) {
    m_feature->thicknessOption = opt;
    return *this;
  }

  RibBuilder &SetMaterialOption(const RibMaterialOption &opt) {
    m_feature->materialOption = opt;
    return *this;
  }
};

} // namespace Builder
} // namespace CADExchange
