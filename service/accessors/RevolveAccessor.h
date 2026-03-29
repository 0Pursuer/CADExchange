#pragma once
#include "AccessorMacros.h"
#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"
#include <memory>
#include <string>

namespace CADExchange {
namespace Accessor {

/**
 * @brief 旋转特征访问器。
 *
 * 对应 Builder 层的 RevolveBuilder，提供对旋转特征的只读访问。
 * 包括轮廓草图、旋转轴和 SweepExtent 配置。
 */
class RevolveAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CRevolve> m_revolve;

public:
  explicit RevolveAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_revolve = std::dynamic_pointer_cast<const CRevolve>(feat);
  }

  explicit RevolveAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_revolve = std::dynamic_pointer_cast<const CRevolve>(other.GetRaw());
  }

  explicit RevolveAccessor(std::shared_ptr<FeatureAccessorBase> feat)
      : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
    if (feat)
      m_revolve = std::dynamic_pointer_cast<const CRevolve>(feat->GetRaw());
  }

  const CRevolve *Data() const { return m_revolve.get(); }

  const CRevolve *operator->() const { return m_revolve.get(); }

  bool IsValid() const override { return m_revolve != nullptr; }

  // --- 基本属性 ---
  ACCESSOR_GETTER(std::string, ProfileSketchID, profileSketchID, "")
  ACCESSOR_GETTER(BooleanOp, Operation, operation, BooleanOp::BOSS)

  // --- 旋转轴 ---
  // Axis structure
  CPoint3D GetAxisOrigin() const {
    return IsValid() ? Data()->axis.origin : CPoint3D{0, 0, 0};
  }

  CVector3D GetAxisDirection() const {
    return IsValid() ? Data()->axis.direction : CVector3D{0, 0, 1};
  }

  ReferenceAccessor GetAxisReference() const {
    if (IsValid()) {
      return ReferenceAccessor(Data()->axis.referenceEntity);
    }
    return ReferenceAccessor(nullptr);
  }

  // --- 扫掠终止 ---
  SweepExtent::Type GetExtentType1() const {
    return IsValid() ? Data()->extent1.type : SweepExtent::Type::VALUE;
  }

  double GetExtentValue1() const {
    return IsValid() ? Data()->extent1.value : 0.0;
  }

  double GetExtentOffset1() const {
    return IsValid() ? Data()->extent1.offset : 0.0;
  }

  bool HasExtent2() const { return IsValid() && Data()->extent2.has_value(); }

  SweepExtent::Type GetExtentType2() const {
    if (!HasExtent2())
      return SweepExtent::Type::VALUE;
    return Data()->extent2->type;
  }

  double GetExtentValue2() const {
    if (!HasExtent2())
      return 0.0;
    return Data()->extent2->value;
  }

  bool HasThinWall() const {
    return IsValid() && Data()->thinWall.has_value();
  }

  double GetThinWallThickness() const {
    return HasThinWall() ? Data()->thinWall->thickness : 0.0;
  }

  bool IsThinWallOneSided() const {
    return HasThinWall() ? Data()->thinWall->isOneSided : true;
  }

  bool IsThinWallOutward() const {
    return HasThinWall() && Data()->thinWall->isOutward;
  }

  bool IsThinWallCovered() const {
    return HasThinWall() && Data()->thinWall->isCovered;
  }

};

} // namespace Accessor
} // namespace CADExchange
