#pragma once
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
 * 包括轮廓草图、旋转轴和角度配置。
 */
class RevolveAccessor : public FeatureAccessorBase {
private:
    std::shared_ptr<const CRevolve> m_revolve;

public:
    explicit RevolveAccessor(std::shared_ptr<const CFeatureBase> feat)
        : FeatureAccessorBase(feat) {
        m_revolve = std::dynamic_pointer_cast<const CRevolve>(feat);
    }

    explicit RevolveAccessor(const FeatureAccessorBase& other)
        : FeatureAccessorBase(other.GetRaw()) {
        m_revolve = std::dynamic_pointer_cast<const CRevolve>(other.GetRaw());
    }

    explicit RevolveAccessor(std::shared_ptr<FeatureAccessorBase> feat)
        : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
        if (feat) m_revolve = std::dynamic_pointer_cast<const CRevolve>(feat->GetRaw());
    }

    bool IsValid() const {
        return m_revolve != nullptr;
    }

    // --- 基本属性 ---
    std::string GetProfileSketchID() const {
        if (!IsValid()) return "";
        return m_revolve->profileSketchID;
    }

    // --- 旋转轴 ---

    CPoint3D GetAxisOrigin() const {
        return IsValid() ? m_revolve->axis.origin : CPoint3D{0, 0, 0};
    }

    CVector3D GetAxisDirection() const {
        return IsValid() ? m_revolve->axis.direction : CVector3D{0, 0, 1};
    }

    ReferenceAccessor GetAxisReference() const {
        if (IsValid()) {
            return ReferenceAccessor(m_revolve->axis.referenceEntity);
        }
        return ReferenceAccessor(nullptr);
    }

    // --- 角度配置 ---
    CRevolve::AngleKind GetAngleKind() const {
        return IsValid() ? m_revolve->angleKind : CRevolve::AngleKind::Single;
    }

    double GetPrimaryAngle() const {
        return IsValid() ? m_revolve->primaryAngle : 0.0;
    }

    double GetSecondaryAngle() const {
        return IsValid() ? m_revolve->secondaryAngle : 0.0;
    }
};

} // namespace Accessor
} // namespace CADExchange
