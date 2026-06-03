#pragma once

#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"

namespace CADExchange {
namespace Accessor {

class RibAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CRib> m_rib;

public:
  explicit RibAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_rib = std::dynamic_pointer_cast<const CRib>(feat);
  }

  explicit RibAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_rib = std::dynamic_pointer_cast<const CRib>(other.GetRaw());
  }

  explicit RibAccessor(std::shared_ptr<FeatureAccessorBase> feat)
      : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
    if (feat) {
      m_rib = std::dynamic_pointer_cast<const CRib>(feat->GetRaw());
    }
  }

  const CRib *Data() const { return m_rib.get(); }
  const CRib *operator->() const { return m_rib.get(); }
  bool IsValid() const override { return m_rib != nullptr; }

  std::string GetSectionSketchID() const {
    return IsValid() ? Data()->sketchID : "";
  }

  RibThicknessOption GetThicknessOption() const {
    return IsValid() ? Data()->thicknessOption : RibThicknessOption{};
  }

  RibMaterialOption GetMaterialOption() const {
    return IsValid() ? Data()->materialOption : RibMaterialOption{};
  }
};

} // namespace Accessor
} // namespace CADExchange
