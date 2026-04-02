#pragma once
#include "../../core/TypeAdapters.h"
#include "AccessorMacros.h"
#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"
#include <cmath>
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

  bool IsValid() const override { return m_extrude != nullptr; }

  // --- 核心属性 ---
  std::string GetProfileSketchID() const {
    if (!IsValid())
      return "";
    return Data()->profileSketchID;
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
  SweepExtent::Type GetEndType1() const {
    return IsValid() ? Data()->extent1.type : SweepExtent::Type::VALUE;
  }

  double GetDepth1() const {
    return IsValid() ? Data()->extent1.value : 0.0;
  }

  double GetOffset1() const {
    return IsValid() ? Data()->extent1.offset : 0.0;
  }

  bool HasOffset1() const {
    return IsValid() && Data()->extent1.hasOffset;
  }

  bool IsFlip1() const { return IsValid() && Data()->extent1.isFlip; }

  bool IsFlipMaterialSide1() const {
    return IsValid() && Data()->extent1.isFlipMaterialSide;
  }

  ReferenceAccessor GetReference1() const {
    if (IsValid()) {
      return ReferenceAccessor(Data()->extent1.referenceEntity);
    }
    return ReferenceAccessor(nullptr);
  }

  // --- 拉伸方向 2（可选） ---
  // extent2 is std::optional<SweepExtent>

  bool HasDirection2() const {
    return IsValid() && Data()->extent2.has_value();
  }

  SweepExtent::Type GetEndType2() const {
    if (!HasDirection2())
      return SweepExtent::Type::VALUE;
    return Data()->extent2->type;
  }

  double GetDepth2() const {
    if (!HasDirection2())
      return 0.0;
    return Data()->extent2->value;
  }

  double GetOffset2() const {
    if (!HasDirection2())
      return 0.0;
    return Data()->extent2->offset;
  }

  bool HasOffset2() const {
    return HasDirection2() && Data()->extent2->hasOffset;
  }

  bool IsFlip2() const {
    return HasDirection2() && Data()->extent2->isFlip;
  }

  bool IsFlipMaterialSide2() const {
    return HasDirection2() && Data()->extent2->isFlipMaterialSide;
  }

  ReferenceAccessor GetReference2() const {
    if (HasDirection2()) {
      return ReferenceAccessor(Data()->extent2->referenceEntity);
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
  double GetThinWallThickness() const {
    if (!HasThinWall())
      return 0.0;
    const double absStart = std::fabs(Data()->thinWall->startOffset);
    const double absEnd = std::fabs(Data()->thinWall->endOffset);
    return absStart > absEnd ? absStart : absEnd;
  }

  bool IsThinWallOneSided() const {
    if (!HasThinWall())
      return false;
    return std::fabs(Data()->thinWall->startOffset) <= 1e-9 ||
           std::fabs(Data()->thinWall->endOffset) <= 1e-9;
  }

  /// 仅 isOneSided=true 时有意义：向外(true) vs 向内(false,默认)
  bool IsThinWallOutward() const {
    if (!HasThinWall())
      return false;
    return std::fabs(Data()->thinWall->endOffset) >
           std::fabs(Data()->thinWall->startOffset);
  }

  bool IsThinWallCovered() const {
    if (!HasThinWall())
      return false;
    return Data()->thinWall->isCovered;
  }

  bool HasThinWallStartOffset() const {
    return HasThinWall();
  }

  bool HasThinWallEndOffset() const {
    return HasThinWall();
  }

  double GetThinWallStartOffset() const {
    return HasThinWall() ? Data()->thinWall->startOffset : 0.0;
  }

  double GetThinWallEndOffset() const {
    return HasThinWall() ? Data()->thinWall->endOffset : 0.0;
  }
};

} // namespace Accessor
} // namespace CADExchange
