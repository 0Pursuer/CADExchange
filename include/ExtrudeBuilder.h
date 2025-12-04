#pragma once

#include "FeatureBuilderBase.h"
#include <cmath>
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

  ExtrudeBuilder &SetDirection(const CVector3D &dir) {
    CVector3D normalized = dir;
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

  ExtrudeBuilder &SetDepth(double depth) {
    if (depth < 0) {
      throw std::runtime_error("Depth must be non-negative.");
    }
    m_feature->endCondition1.type = ExtrudeEndCondition::Type::BLIND;
    m_feature->endCondition1.depth = depth;
    return *this;
  }

  ExtrudeBuilder &SetThroughAll() {
    m_feature->endCondition1.type = ExtrudeEndCondition::Type::THROUGH_ALL;
    return *this;
  }

  ExtrudeBuilder &SetUpToSurface(const CRefFace &faceInfo,
                                 double offset = 0.0) {
    auto targetFace = std::make_shared<CRefFace>(faceInfo);
    m_feature->endCondition1.type = ExtrudeEndCondition::Type::UP_TO_FACE;
    m_feature->endCondition1.referenceEntity = targetFace;
    m_feature->endCondition1.hasOffset = std::abs(offset) > 1e-6;
    m_feature->endCondition1.offset = offset;
    return *this;
  }

  ExtrudeBuilder &SetUpToNext() {
    m_feature->endCondition1.type = ExtrudeEndCondition::Type::UP_TO_NEXT;
    return *this;
  }

  ExtrudeBuilder &SetDirection2Depth(double depth) {
    if (depth < 0) {
      throw std::runtime_error("Direction2 depth must be non-negative.");
    }
    m_feature->endCondition2.emplace();
    m_feature->endCondition2->type = ExtrudeEndCondition::Type::BLIND;
    m_feature->endCondition2->depth = depth;
    return *this;
  }

  ExtrudeBuilder &SetDirection2ThroughAll() {
    m_feature->endCondition2.emplace();
    m_feature->endCondition2->type = ExtrudeEndCondition::Type::THROUGH_ALL;
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
