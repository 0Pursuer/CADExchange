#pragma once

#include "AccessorMacros.h"
#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"
#include <memory>
#include <vector>

namespace CADExchange {
namespace Accessor {

/**
 * @brief Datum plane accessor for read-only method/constraints/references.
 */
class DatumPlaneAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CDatumPlane> m_plane;

public:
  explicit DatumPlaneAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_plane = std::dynamic_pointer_cast<const CDatumPlane>(feat);
  }

  explicit DatumPlaneAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_plane = std::dynamic_pointer_cast<const CDatumPlane>(other.GetRaw());
  }

  explicit DatumPlaneAccessor(std::shared_ptr<FeatureAccessorBase> feat)
      : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
    if (feat) {
      m_plane = std::dynamic_pointer_cast<const CDatumPlane>(feat->GetRaw());
    }
  }

  const CDatumPlane *Data() const { return m_plane.get(); }

  const CDatumPlane *operator->() const { return m_plane.get(); }

  bool IsValid() const override { return m_plane != nullptr; }

  CPoint3D GetProjectedOrigin() const {
    return IsValid() ? Data()->projectedOrigin : CPoint3D{};
  }

  CVector3D GetNormal() const {
    return IsValid() ? Data()->normal : CVector3D{};
  }

  ACCESSOR_GETTER(PlaneMethod, Method, method, PlaneMethod::UNKNOWN)

  bool IsLineMethod() const {
    return IsValid() && Data()->method == PlaneMethod::LINE;
  }

  bool HasConstraints() const {
    return IsValid() && !Data()->constraints.empty();
  }

  int GetConstraintCount() const {
    return IsValid() ? static_cast<int>(Data()->constraints.size()) : 0;
  }

  PlaneConstraint GetConstraint(int index) const {
    if (!IsValid() || index < 0 ||
        index >= static_cast<int>(Data()->constraints.size())) {
      return PlaneConstraint{};
    }
    return Data()->constraints[static_cast<size_t>(index)];
  }

  const std::vector<PlaneConstraint> &GetConstraints() const {
    static const std::vector<PlaneConstraint> kEmpty;
    return IsValid() ? Data()->constraints : kEmpty;
  }

  bool HasReferences() const {
    return IsValid() && !Data()->referenceEntities.empty();
  }

  int GetReferenceCount() const {
    return IsValid() ? static_cast<int>(Data()->referenceEntities.size()) : 0;
  }

  ReferenceAccessor GetReference(int index) const {
    if (!IsValid() || index < 0 ||
        index >= static_cast<int>(Data()->referenceEntities.size())) {
      return ReferenceAccessor(nullptr);
    }
    return ReferenceAccessor(
        Data()->referenceEntities[static_cast<size_t>(index)]);
  }

  const std::vector<std::shared_ptr<CRefEntityBase>> &
  GetReferenceEntities() const {
    static const std::vector<std::shared_ptr<CRefEntityBase>> kEmpty;
    return IsValid() ? Data()->referenceEntities : kEmpty;
  }
};

} // namespace Accessor
} // namespace CADExchange
