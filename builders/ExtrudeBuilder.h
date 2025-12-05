#pragma once
#include "../core/TypeAdapters.h"
#include "EndConditionBuilder.h"
#include "FeatureBuilderBase.h"
#include "ReferenceBuilder.h"

#include <stdexcept>

namespace CADExchange {
namespace Builder {

/**
 * @brief 拉伸构建器，封装方向、结束条件、拔模与薄壁等属性。
 */
class ExtrudeBuilder : public FeatureBuilderBase<CExtrude> {
public:
  ExtrudeBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {
    m_feature->direction = {0, 0, 1};
  }

  ExtrudeBuilder &SetProfile(const std::string &sketchID) {
    auto sketch = m_model.GetFeatureAs<CSketch>(sketchID);
    if (!sketch) {
      throw std::runtime_error("Sketch profile not found: " + sketchID);
    }
    m_feature->sketchProfile = sketch;
    return *this;
  }

  ExtrudeBuilder &SetProfileByExternalID(const std::string &externalID) {
    auto sketch = m_model.GetFeatureByExternalIDAs<CSketch>(externalID);
    if (!sketch) {
      throw std::runtime_error("Sketch profile not found by external ID: " +
                               externalID);
    }
    m_feature->sketchProfile = sketch;
    return *this;
  }

  template <typename VectorT> ExtrudeBuilder &SetDirection(const VectorT &dir) {
    CVector3D normalized = VectorAdapter<VectorT>::Convert(dir);
    normalized.Normalize();
    m_feature->direction = normalized;
    return *this;
  }

  ExtrudeBuilder &SetOperation(BooleanOp op) {
    m_feature->operation = op;
    return *this;
  }

  ExtrudeBuilder &SetEndCondition1(const ExtrudeEndCondition &cond) {
    m_feature->endCondition1 = cond;
    return *this;
  }
  ExtrudeBuilder &SetEndCondition2(const ExtrudeEndCondition &cond) {
    m_feature->endCondition2 = cond;
    return *this;
  }

  ExtrudeBuilder &SetDraft(double angle, bool outward = false) {
    if (angle < 0) {
      throw std::runtime_error("Draft angle must be non-negative.");
    }
    m_feature->draft = DraftOption{angle, outward};
    return *this;
  }

  ExtrudeBuilder &SetThinWall(double thickness, bool isOneSided = true,
                              bool isCovered = false) {
    if (thickness <= 0) {
      throw std::runtime_error("Thickness must be positive.");
    }
    m_feature->thinWall = ThinWallOption{thickness, isOneSided, isCovered};
    return *this;
  }
};

} // namespace Builder
} // namespace CADExchange
