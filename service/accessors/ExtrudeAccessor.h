#pragma once
#include "FeatureAccessor.h"
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

    explicit ExtrudeAccessor(const FeatureAccessorBase& other)
        : FeatureAccessorBase(other.GetRaw()) {
        m_extrude = std::dynamic_pointer_cast<const CExtrude>(other.GetRaw());
    }

    explicit ExtrudeAccessor(std::shared_ptr<FeatureAccessorBase> feat)
        : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
        if (feat) m_extrude = std::dynamic_pointer_cast<const CExtrude>(feat->GetRaw());
    }

    bool IsValid() const {
        return m_extrude != nullptr;
    }

    // --- 核心属性 ---
    std::string GetProfileSketchID() const {
        if (!IsValid() || !m_extrude->sketchProfile) return "";
        return m_extrude->sketchProfile->featureID;
    }

    CVector3D GetDirection() const {
        return IsValid() ? m_extrude->direction : CVector3D{0, 0, 1};
    }

    BooleanOp GetOperation() const {
        return IsValid() ? m_extrude->operation : BooleanOp::BOSS;
    }

    // --- 拉伸方向 1（主方向） ---
    ExtrudeEndCondition::Type GetEndType1() const {
        return IsValid() ? m_extrude->endCondition1.type : ExtrudeEndCondition::Type::BLIND;
    }

    double GetDepth1() const {
        return IsValid() ? m_extrude->endCondition1.depth : 0.0;
    }

    double GetOffset1() const {
        return IsValid() ? m_extrude->endCondition1.offset : 0.0;
    }

    bool HasOffset1() const {
        return IsValid() && m_extrude->endCondition1.hasOffset;
    }

    bool IsFlip1() const {
        return IsValid() && m_extrude->endCondition1.isFlip;
    }

    bool IsFlipMaterialSide1() const {
        return IsValid() && m_extrude->endCondition1.isFlipMaterialSide;
    }

    ReferenceAccessor GetReference1() const {
        if (IsValid()) {
            return ReferenceAccessor(m_extrude->endCondition1.referenceEntity);
        }
        return ReferenceAccessor(nullptr);
    }

    // --- 拉伸方向 2（可选） ---
    bool HasDirection2() const {
        return IsValid() && m_extrude->endCondition2.has_value();
    }

    ExtrudeEndCondition::Type GetEndType2() const {
        if (!HasDirection2()) return ExtrudeEndCondition::Type::BLIND;
        return m_extrude->endCondition2->type;
    }

    double GetDepth2() const {
        if (!HasDirection2()) return 0.0;
        return m_extrude->endCondition2->depth;
    }

    double GetOffset2() const {
        if (!HasDirection2()) return 0.0;
        return m_extrude->endCondition2->offset;
    }

    bool HasOffset2() const {
        return HasDirection2() && m_extrude->endCondition2->hasOffset;
    }

    bool IsFlip2() const {
        return HasDirection2() && m_extrude->endCondition2->isFlip;
    }

    bool IsFlipMaterialSide2() const {
        return HasDirection2() && m_extrude->endCondition2->isFlipMaterialSide;
    }

    ReferenceAccessor GetReference2() const {
        if (HasDirection2()) {
            return ReferenceAccessor(m_extrude->endCondition2->referenceEntity);
        }
        return ReferenceAccessor(nullptr);
    }

    // --- 拔模（可选） ---
    bool HasDraft() const {
        return IsValid() && m_extrude->draft.has_value();
    }

    double GetDraftAngle() const {
        if (!HasDraft()) return 0.0;
        return m_extrude->draft->angle;
    }

    bool IsDraftOutward() const {
        if (!HasDraft()) return false;
        return m_extrude->draft->outward;
    }

    // --- 薄壁（可选） ---
    bool HasThinWall() const {
        return IsValid() && m_extrude->thinWall.has_value();
    }

    double GetThinWallThickness() const {
        if (!HasThinWall()) return 0.0;
        return m_extrude->thinWall->thickness;
    }

    bool IsThinWallOneSided() const {
        if (!HasThinWall()) return false;
        return m_extrude->thinWall->isOneSided;
    }

    bool IsThinWallCovered() const {
        if (!HasThinWall()) return false;
        return m_extrude->thinWall->isCovered;
    }
};

} // namespace Accessor
} // namespace CADExchange
