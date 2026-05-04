#pragma once

#include "FeatureBuilderBase.h"
#include "ReferenceBuilder.h"
#include <stdexcept>
#include <type_traits>

namespace CADExchange {
namespace Builder {

/**
 * @brief Builder for unified solid chamfer features.
 */
class ChamferBuilder : public FeatureBuilderBase<CChamfer> {
public:
  ChamferBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  /**
   * @brief Set chamfer parameter mode.
   */
  ChamferBuilder &SetMode(ChamferMode mode) {
    if (mode == ChamferMode::UNKNOWN) {
      throw std::runtime_error("Chamfer mode must not be UNKNOWN.");
    }
    m_feature->mode = mode;
    return *this;
  }

  /**
   * @brief Set the first chamfer distance.
   */
  ChamferBuilder &SetDistance1(double value) {
    m_feature->params.distance1 = value;
    return *this;
  }

  /**
   * @brief Set the second chamfer distance.
   */
  ChamferBuilder &SetDistance2(double value) {
    m_feature->params.distance2 = value;
    return *this;
  }

  /**
   * @brief Set the third chamfer distance.
   */
  ChamferBuilder &SetDistance3(double value) {
    m_feature->params.distance3 = value;
    return *this;
  }

  /**
   * @brief Set the chamfer angle.
   */
  ChamferBuilder &SetAngle(double value) {
    m_feature->params.angle = value;
    return *this;
  }

  /**
   * @brief Add one reference participating in chamfer creation.
   */
  template <typename RefBuilderT>
  ChamferBuilder &AddReference(const RefBuilderT &refBuilder) {
    return AddReferenceRaw(ToReference(refBuilder));
  }

  /**
   * @brief Replace all chamfer references.
   */
  template <typename... RefBuilderTs>
  ChamferBuilder &SetReferences(const RefBuilderTs &...references) {
    m_feature->references.clear();
    m_feature->references.reserve(sizeof...(references));
    (AddReference(references), ...);
    return *this;
  }

  /**
   * @brief Remove all references.
   */
  ChamferBuilder &ClearReferences() {
    m_feature->references.clear();
    return *this;
  }

private:
  ChamferBuilder &AddReferenceRaw(const std::shared_ptr<CRefEntityBase> &ref) {
    if (!ref) {
      throw std::runtime_error("Chamfer reference must not be null.");
    }
    ValidateReference(ref);
    m_feature->references.push_back(ref);
    return *this;
  }

  template <typename RefBuilderT>
  static std::shared_ptr<CRefEntityBase>
  ToReference(const RefBuilderT &refBuilder) {
    static_assert(std::is_convertible_v<RefBuilderT,
                                        std::shared_ptr<CRefEntityBase>>,
                  "Reference must be built by CADExchange::Builder::Ref::* wrappers.");
    return static_cast<std::shared_ptr<CRefEntityBase>>(refBuilder);
  }
};

} // namespace Builder
} // namespace CADExchange
