#pragma once

#include "AccessorMacros.h"
#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"
#include <memory>
#include <vector>

namespace CADExchange {
namespace Accessor {

/**
 * @brief Read-only accessor for unified chamfer features.
 */
class ChamferAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CChamfer> m_chamfer;

public:
  explicit ChamferAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_chamfer = std::dynamic_pointer_cast<const CChamfer>(feat);
  }

  explicit ChamferAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_chamfer = std::dynamic_pointer_cast<const CChamfer>(other.GetRaw());
  }

  explicit ChamferAccessor(std::shared_ptr<FeatureAccessorBase> feat)
      : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
    if (feat) {
      m_chamfer = std::dynamic_pointer_cast<const CChamfer>(feat->GetRaw());
    }
  }

  const CChamfer *Data() const { return m_chamfer.get(); }

  const CChamfer *operator->() const { return m_chamfer.get(); }

  bool IsValid() const override { return m_chamfer != nullptr; }

  ACCESSOR_GETTER(ChamferMode, Mode, mode, ChamferMode::UNKNOWN)

  bool HasDistance1() const {
    return IsValid() && Data()->params.distance1.has_value();
  }

  double GetDistance1() const {
    return HasDistance1() ? *Data()->params.distance1 : 0.0;
  }

  bool HasDistance2() const {
    return IsValid() && Data()->params.distance2.has_value();
  }

  double GetDistance2() const {
    return HasDistance2() ? *Data()->params.distance2 : 0.0;
  }

  bool HasDistance3() const {
    return IsValid() && Data()->params.distance3.has_value();
  }

  double GetDistance3() const {
    return HasDistance3() ? *Data()->params.distance3 : 0.0;
  }

  bool HasOffset1() const {
    return IsValid() && Data()->params.offset1.has_value();
  }

  double GetOffset1() const {
    return HasOffset1() ? *Data()->params.offset1 : 0.0;
  }

  bool HasOffset2() const {
    return IsValid() && Data()->params.offset2.has_value();
  }

  double GetOffset2() const {
    return HasOffset2() ? *Data()->params.offset2 : 0.0;
  }

  bool HasAngle() const { return IsValid() && Data()->params.angle.has_value(); }

  double GetAngle() const { return HasAngle() ? *Data()->params.angle : 0.0; }

  bool HasFirstEndFaceMarker() const {
    return IsValid() && Data()->firstEndFaceMarker.has_value();
  }

  CPoint3D GetFirstEndFaceMarker() const {
    return HasFirstEndFaceMarker() ? *Data()->firstEndFaceMarker
                                   : CPoint3D{0.0, 0.0, 0.0};
  }

  int GetReferenceCount() const {
    return IsValid() ? static_cast<int>(Data()->references.size()) : 0;
  }

  ReferenceAccessor GetReference(int index) const {
    if (!IsValid() || index < 0 ||
        index >= static_cast<int>(Data()->references.size())) {
      return ReferenceAccessor(nullptr);
    }
    return ReferenceAccessor(Data()->references[static_cast<size_t>(index)]);
  }

  const std::vector<std::shared_ptr<CRefEntityBase>> &GetReferences() const {
    static const std::vector<std::shared_ptr<CRefEntityBase>> kEmpty;
    return IsValid() ? Data()->references : kEmpty;
  }
};

} // namespace Accessor
} // namespace CADExchange
