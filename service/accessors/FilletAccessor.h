#pragma once

#include "AccessorMacros.h"
#include "FeatureAccessorBase.h"
#include <memory>
#include <vector>

namespace CADExchange {
namespace Accessor {

class FilletAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CFillet> m_fillet;

public:
  explicit FilletAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_fillet = std::dynamic_pointer_cast<const CFillet>(feat);
  }

  explicit FilletAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_fillet = std::dynamic_pointer_cast<const CFillet>(other.GetRaw());
  }

  explicit FilletAccessor(std::shared_ptr<FeatureAccessorBase> feat)
      : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
    if (feat) {
      m_fillet = std::dynamic_pointer_cast<const CFillet>(feat->GetRaw());
    }
  }

  const CFillet *Data() const { return m_fillet.get(); }
  const CFillet *operator->() const { return m_fillet.get(); }
  bool IsValid() const override { return m_fillet != nullptr; }

  ACCESSOR_GETTER(FilletMode, Mode, mode, FilletMode::UNKNOWN)
  ACCESSOR_GETTER(FilletCrossSection, CrossSection, params.crossSection,
                  FilletCrossSection::UNKNOWN)
  ACCESSOR_GETTER(FilletReferenceMode, ReferenceMode, referenceMode,
                  FilletReferenceMode::UNKNOWN)

  bool HasDefaultRadius() const {
    return IsValid() && Data()->params.defaultRadius.has_value();
  }

  double GetDefaultRadius() const {
    return HasDefaultRadius() ? *Data()->params.defaultRadius : 0.0;
  }

  bool HasDefaultRadius2() const {
    return IsValid() && Data()->params.defaultRadius2.has_value();
  }

  double GetDefaultRadius2() const {
    return HasDefaultRadius2() ? *Data()->params.defaultRadius2 : 0.0;
  }

  bool IsAsymmetric() const {
    return IsValid() && Data()->params.isAsymmetric;
  }

  bool HasTangentPropagation() const {
    return IsValid() && Data()->params.tangentPropagation;
  }

  bool IsCurvatureContinuous() const {
    return IsValid() &&
           Data()->params.crossSection ==
               FilletCrossSection::CURVATURE_CONTINUOUS;
  }

  bool HasConicValue() const {
    return IsValid() && Data()->params.conicValue.has_value();
  }

  double GetConicValue() const {
    return HasConicValue() ? *Data()->params.conicValue : 0.0;
  }

  FilletConicValueMode GetConicValueMode() const {
    return IsValid() ? Data()->params.conicValueMode
                     : FilletConicValueMode::NONE;
  }

  const std::vector<CFilletRadiusPoint> &GetRadiusPoints() const {
    static const std::vector<CFilletRadiusPoint> kEmpty;
    return IsValid() ? Data()->params.radiusPoints : kEmpty;
  }

  bool HasFirstEndFaceMarker() const {
    return IsValid() && Data()->firstEndFaceMarker.has_value();
  }

  CPoint3D GetFirstEndFaceMarker() const {
    return HasFirstEndFaceMarker() ? *Data()->firstEndFaceMarker
                                   : CPoint3D{};
  }

  const std::vector<std::shared_ptr<CRefEntityBase>> &GetReferences() const {
    static const std::vector<std::shared_ptr<CRefEntityBase>> kEmpty;
    return IsValid() ? Data()->references : kEmpty;
  }

  const std::vector<std::shared_ptr<CRefFace>> &GetSide1Faces() const {
    static const std::vector<std::shared_ptr<CRefFace>> kEmpty;
    return IsValid() ? Data()->side1Faces : kEmpty;
  }

  const std::vector<std::shared_ptr<CRefFace>> &GetSide2Faces() const {
    static const std::vector<std::shared_ptr<CRefFace>> kEmpty;
    return IsValid() ? Data()->side2Faces : kEmpty;
  }

  const std::vector<std::shared_ptr<CRefFace>> &GetCenterFaces() const {
    static const std::vector<std::shared_ptr<CRefFace>> kEmpty;
    return IsValid() ? Data()->centerFaces : kEmpty;
  }
};

} // namespace Accessor
} // namespace CADExchange
