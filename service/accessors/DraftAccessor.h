#pragma once

#include "AccessorMacros.h"
#include "FeatureAccessorBase.h"
#include <memory>
#include <vector>

namespace CADExchange {
namespace Accessor {

/**
 * @brief 拔模特征的只读访问器。
 */
class DraftAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CDraft> m_draft;

public:
  explicit DraftAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_draft = std::dynamic_pointer_cast<const CDraft>(feat);
  }

  explicit DraftAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_draft = std::dynamic_pointer_cast<const CDraft>(other.GetRaw());
  }

  explicit DraftAccessor(std::shared_ptr<FeatureAccessorBase> feat)
      : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
    if (feat) {
      m_draft = std::dynamic_pointer_cast<const CDraft>(feat->GetRaw());
    }
  }

  const CDraft *Data() const { return m_draft.get(); }
  const CDraft *operator->() const { return m_draft.get(); }
  bool IsValid() const override { return m_draft != nullptr; }

  DraftType GetDraftType() const {
    return IsValid() ? Data()->draftType : DraftType::Unknown;
  }

  std::shared_ptr<CRefEntityBase> GetPullDirectionRef() const {
    return IsValid() ? Data()->pullDirectionRef : nullptr;
  }

  ACCESSOR_GETTER(bool, ReversePullDirection, reversePullDirection, false)

  const std::vector<std::shared_ptr<CRefFace>> &GetDraftFaces() const {
    static const std::vector<std::shared_ptr<CRefFace>> kEmpty;
    return IsValid() ? Data()->draftFaces : kEmpty;
  }

  size_t GetDraftFacesCount() const {
    return IsValid() ? Data()->draftFaces.size() : 0;
  }

  std::shared_ptr<CRefEntityBase> GetNeutralPlaneRef() const {
    return IsValid() ? Data()->neutralPlaneRef : nullptr;
  }

  const std::vector<std::shared_ptr<CRefEntityBase>> &GetPartingLines() const {
    static const std::vector<std::shared_ptr<CRefEntityBase>> kEmpty;
    return IsValid() ? Data()->partingLines : kEmpty;
  }

  size_t GetPartingLinesCount() const {
    return IsValid() ? Data()->partingLines.size() : 0;
  }

  std::shared_ptr<CRefEntityBase> GetPartingSplitSketchRef() const {
    return IsValid() ? Data()->partingSplitSketchRef : nullptr;
  }

  const std::vector<std::shared_ptr<CRefFace>> &GetPartingSplitTargetFaces() const {
    static const std::vector<std::shared_ptr<CRefFace>> kEmpty;
    return IsValid() ? Data()->partingSplitTargetFaces : kEmpty;
  }

  size_t GetPartingSplitTargetFacesCount() const {
    return IsValid() ? Data()->partingSplitTargetFaces.size() : 0;
  }

  ACCESSOR_GETTER(bool, PartingSplitSingleDirection, partingSplitSingleDirection, false)
  ACCESSOR_GETTER(bool, PartingSplitReverseDirection, partingSplitReverseDirection, false)

  ACCESSOR_GETTER(double, DraftAngle, draftAngle, 0.0)
  ACCESSOR_GETTER(bool, IsTwoSided, isTwoSided, false)
  ACCESSOR_GETTER(double, DraftAngleSide2, draftAngleSide2, 0.0)
};

} // namespace Accessor
} // namespace CADExchange
