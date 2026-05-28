#pragma once

#include "FeatureBuilderBase.h"
#include "ReferenceBuilder.h"
#include <stdexcept>
#include <type_traits>

namespace CADExchange {
namespace Builder {

class FilletBuilder : public FeatureBuilderBase<CFillet> {
public:
  FilletBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  FilletBuilder &SetMode(FilletMode mode) {
    if (mode == FilletMode::UNKNOWN) {
      throw std::runtime_error("Fillet mode must not be UNKNOWN.");
    }
    m_feature->mode = mode;
    return *this;
  }

  FilletBuilder &SetCrossSection(FilletCrossSection crossSection) {
    m_feature->params.crossSection = crossSection;
    return *this;
  }

  FilletBuilder &SetDriveType(FilletDriveType driveType) {
    m_feature->params.driveType = driveType;
    return *this;
  }

  FilletBuilder &SetReferenceMode(FilletReferenceMode referenceMode) {
    m_feature->referenceMode = referenceMode;
    return *this;
  }

  FilletBuilder &SetPrimaryValue(double value) {
    m_feature->params.primaryValue = value;
    return *this;
  }

  FilletBuilder &SetSecondValue(double value) {
    m_feature->params.secondValue = value;
    return *this;
  }

  FilletBuilder &SetTangentPropagation(bool value = true) {
    m_feature->params.tangentPropagation = value;
    return *this;
  }

  FilletBuilder &SetCurvatureContinuous(bool value = true) {
    if (value) {
      m_feature->params.crossSection =
          FilletCrossSection::CURVATURE_CONTINUOUS;
    } else if (m_feature->params.crossSection ==
               FilletCrossSection::CURVATURE_CONTINUOUS) {
      m_feature->params.crossSection = FilletCrossSection::UNKNOWN;
    }
    return *this;
  }

  FilletBuilder &SetConicValue(
      double value,
      FilletConicValueMode mode = FilletConicValueMode::GENERIC_VALUE) {
    m_feature->params.conicValue = value;
    m_feature->params.conicValueMode = mode;
    if (m_feature->params.crossSection == FilletCrossSection::UNKNOWN) {
      m_feature->params.crossSection = FilletCrossSection::CONIC;
    }
    return *this;
  }

  FilletBuilder &AddReference(const std::shared_ptr<CRefEntityBase> &ref) {
    if (!ref) {
      throw std::runtime_error("Fillet reference must not be null.");
    }
    ValidateReference(ref);
    m_feature->references.push_back(ref);
    return *this;
  }

  template <typename RefBuilderT>
  FilletBuilder &AddReference(const RefBuilderT &refBuilder) {
    return AddReference(ToReference(refBuilder));
  }

  FilletBuilder &AddSide1Face(const std::shared_ptr<CRefFace> &ref) {
    return AddFace(m_feature->side1Faces, ref, "Fillet side1 face");
  }

  FilletBuilder &AddSide2Face(const std::shared_ptr<CRefFace> &ref) {
    return AddFace(m_feature->side2Faces, ref, "Fillet side2 face");
  }

  FilletBuilder &AddCenterFace(const std::shared_ptr<CRefFace> &ref) {
    return AddFace(m_feature->centerFaces, ref, "Fillet center face");
  }

  FilletBuilder &AddRadiusPoint(const CFilletRadiusPoint &point) {
    CFilletRadiusPoint normalizedPoint = point;
    m_feature->params.radiusPoints.push_back(std::move(normalizedPoint));
    return *this;
  }

  FilletBuilder &SetFirstEndFaceMarker(const CPoint3D &point) {
    m_feature->firstEndFaceMarker = point;
    return *this;
  }

  FilletBuilder &ClearFirstEndFaceMarker() {
    m_feature->firstEndFaceMarker.reset();
    return *this;
  }

  FilletBuilder &SetSwOverflowType(const std::string &value) {
    m_feature->swOverflowType = value;
    return *this;
  }

  FilletBuilder &SetSwKeepFeatures(bool value = true) {
    m_feature->swKeepFeatures = value;
    return *this;
  }

  FilletBuilder &SetCreoAttachType(int value) {
    m_feature->creoAttachType = value;
    return *this;
  }

  FilletBuilder &SetCreoConicDepOption(int value) {
    m_feature->creoConicDepOption = value;
    return *this;
  }

private:
  template <typename RefBuilderT>
  static std::shared_ptr<CRefEntityBase>
  ToReference(const RefBuilderT &refBuilder) {
    static_assert(std::is_convertible_v<RefBuilderT,
                                        std::shared_ptr<CRefEntityBase>>,
                  "Reference must be built by CADExchange::Builder::Ref::* wrappers.");
    return static_cast<std::shared_ptr<CRefEntityBase>>(refBuilder);
  }

  FilletBuilder &AddFace(std::vector<std::shared_ptr<CRefFace>> &target,
                         const std::shared_ptr<CRefFace> &ref,
                         const char *label) {
    if (!ref) {
      throw std::runtime_error(std::string(label) + " must not be null.");
    }
    ValidateReference(ref);
    target.push_back(ref);
    return *this;
  }
};

} // namespace Builder
} // namespace CADExchange
