#pragma once
#include "../../core/TypeAdapters.h"
#include "AccessorMacros.h"
#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"
#include <optional>

namespace CADExchange {
namespace Accessor {

/**
 * @brief 拉伸特征访问器，提供对拉伸特征的只读访问。
 *
 * 对应 Builder 层的 ExtrudeBuilder，提供与其相反的操作：
 * - Builder: SetProfile() + SetDirection() + Build()
 * - Accessor: GetProfileSketchID() + GetDirection() + IsValid()
 *
 * 使用 HasXXX() + GetXXX() 模式处理 optional 字段。
 */
class ExtrudeAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CExtrude> m_extrude;

public:
  explicit ExtrudeAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_extrude = std::dynamic_pointer_cast<const CExtrude>(feat);
  }

  explicit ExtrudeAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_extrude = std::dynamic_pointer_cast<const CExtrude>(other.GetRaw());
  }

  explicit ExtrudeAccessor(std::shared_ptr<FeatureAccessorBase> feat)
      : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
    if (feat)
      m_extrude = std::dynamic_pointer_cast<const CExtrude>(feat->GetRaw());
  }

  const CExtrude *Data() const { return m_extrude.get(); }

  const CExtrude *operator->() const { return m_extrude.get(); }

  bool IsValid() const { return m_extrude != nullptr; }

  // --- 核心属性 ---
  std::string GetProfileSketchID() const {
    // Access via Data() or operator->
    if (!IsValid() || !Data()->sketchProfile)
      return "";
    return Data()->sketchProfile->featureID;
  }

  // 使用宏定义简化代码
  ACCESSOR_GETTER(CVector3D, Direction, direction,
                  (CVector3D{0, 0,
                             1})) // 相当于 GetDirection() { return IsValid() ?
                                  // Data()->direction : CVector3D{0, 0, 1}; }
  ACCESSOR_GETTER(BooleanOp, Operation, operation, BooleanOp::BOSS)

  template <typename VectorT> VectorT GetDirectionAs() const {
    CVector3D dir = GetDirection();
    return VectorWriter<VectorT>::Convert(dir);
  }

  // --- 拉伸方向 1（主方向） ---
  // Note: m_extrude->endCondition1 is a struct, not a pointer, so we access
  // fields directly via . But our macro expects m_data->MemberName. My macro:
  // return IsValid() ? Data()->MemberName : DefaultValue; For endCondition1
  // which is a struct member of CExtrude, we can't use the simple macro
  // directly for its sub-fields unless we treat EndCondition1 as the member.
  // Let's manually refactor these to use Data() to show the pattern A mixed
  // with B where possible.

  ExtrudeEndCondition::Type GetEndType1() const {
    return IsValid() ? Data()->endCondition1.type
                     : ExtrudeEndCondition::Type::BLIND;
  }

  double GetDepth1() const {
    return IsValid() ? Data()->endCondition1.depth : 0.0;
  }

  double GetOffset1() const {
    return IsValid() ? Data()->endCondition1.offset : 0.0;
  }

  bool HasOffset1() const {
    return IsValid() && Data()->endCondition1.hasOffset;
  }

  bool IsFlip1() const { return IsValid() && Data()->endCondition1.isFlip; }

  bool IsFlipMaterialSide1() const {
    return IsValid() && Data()->endCondition1.isFlipMaterialSide;
  }

  ReferenceAccessor GetReference1() const {
    if (IsValid()) {
      return ReferenceAccessor(Data()->endCondition1.referenceEntity);
    }
    return ReferenceAccessor(nullptr);
  }

  // --- 拉伸方向 2（可选） ---
  // endCondition2 is std::optional<ExtrudeEndCondition>

  bool HasDirection2() const {
    return IsValid() && Data()->endCondition2.has_value();
  }

  ExtrudeEndCondition::Type GetEndType2() const {
    if (!HasDirection2())
      return ExtrudeEndCondition::Type::BLIND;
    return Data()->endCondition2->type;
  }

  double GetDepth2() const {
    if (!HasDirection2())
      return 0.0;
    return Data()->endCondition2->depth;
  }

  double GetOffset2() const {
    if (!HasDirection2())
      return 0.0;
    return Data()->endCondition2->offset;
  }

  bool HasOffset2() const {
    return HasDirection2() && Data()->endCondition2->hasOffset;
  }

  bool IsFlip2() const {
    return HasDirection2() && Data()->endCondition2->isFlip;
  }

  bool IsFlipMaterialSide2() const {
    return HasDirection2() && Data()->endCondition2->isFlipMaterialSide;
  }

  ReferenceAccessor GetReference2() const {
    if (HasDirection2()) {
      return ReferenceAccessor(Data()->endCondition2->referenceEntity);
    }
    return ReferenceAccessor(nullptr);
  }

  // --- 拔模（可选） ---
  ACCESSOR_HAS_GETTER(Draft, draft)
  ACCESSOR_OPTIONAL_GETTER(double, DraftAngle, draft, angle, 0.0)
  // ACCESSOR_OPTIONAL_GETTER(bool, IsDraftOutward, draft, outward, false) //
  // 宏名字冲突？ No, macro is generic. But IsDraftOutward implies boolean
  // return.

  bool IsDraftOutward() const {
    if (!HasDraft())
      return false;
    return Data()->draft->outward;
  }

  // --- 薄壁（可选） ---
  ACCESSOR_HAS_GETTER(ThinWall, thinWall)
  ACCESSOR_OPTIONAL_GETTER(double, ThinWallThickness, thinWall, thickness, 0.0)

  bool IsThinWallOneSided() const {
    if (!HasThinWall())
      return false;
    return Data()->thinWall->isOneSided;
  }

  bool IsThinWallCovered() const {
    if (!HasThinWall())
      return false;
    return Data()->thinWall->isCovered;
  }
};

} // namespace Accessor
} // namespace CADExchange
