#pragma once

#include "FeatureBuilderBase.h"
#include "ReferenceBuilder.h"
#include <stdexcept>
#include <type_traits>

namespace CADExchange {
namespace Builder {

/**
 * @brief 拔模特征的 Builder。
 */
class DraftBuilder : public FeatureBuilderBase<CDraft> {
public:
  DraftBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  DraftBuilder &SetDraftType(DraftType type) {
    if (type == DraftType::Unknown) {
      throw std::runtime_error("Draft type must not be Unknown.");
    }
    m_feature->draftType = type;
    return *this;
  }

  DraftBuilder &SetPullDirection(const std::shared_ptr<CRefEntityBase> &ref) {
    if (ref) {
      ValidateReference(ref);
    }
    m_feature->pullDirectionRef = ref;
    return *this;
  }

  DraftBuilder &SetReversePullDirection(bool reverse) {
    m_feature->reversePullDirection = reverse;
    return *this;
  }

  DraftBuilder &AddDraftFace(const std::shared_ptr<CRefFace> &ref) {
    if (!ref) {
      throw std::runtime_error("Draft face must not be null.");
    }
    ValidateReference(ref);
    m_feature->draftFaces.push_back(ref);
    return *this;
  }

  DraftBuilder &SetNeutralPlane(const std::shared_ptr<CRefEntityBase> &ref) {
    if (ref) {
      ValidateReference(ref);
    }
    m_feature->neutralPlaneRef = ref;
    return *this;
  }

  DraftBuilder &AddPartingLine(const std::shared_ptr<CRefEntityBase> &ref) {
    if (!ref) {
      throw std::runtime_error("Parting line must not be null.");
    }
    ValidateReference(ref);
    m_feature->partingLines.push_back(ref);
    return *this;
  }

  DraftBuilder &SetDraftAngle(double angle) {
    m_feature->draftAngle = angle;
    return *this;
  }

  DraftBuilder &SetIsTwoSided(bool twoSided) {
    m_feature->isTwoSided = twoSided;
    return *this;
  }

  DraftBuilder &SetDraftAngleSide2(double angle) {
    m_feature->draftAngleSide2 = angle;
    return *this;
  }
};

} // namespace Builder
} // namespace CADExchange
