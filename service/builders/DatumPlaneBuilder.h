#pragma once

#include "ReferenceBuilder.h"
#include "FeatureBuilderBase.h"
#include <optional>
#include <stdexcept>
#include <type_traits>

namespace CADExchange {
namespace Builder {

/**
 * @brief Constraint factory for datum plane, similar to EndCondition factory.
 */
class PlaneConstraintBuilder {
public:
  static PlaneConstraint Parallel(int ref, bool reversed = false) {
    PlaneConstraint c;
    c.type = PlaneConstraintType::PARALLEL;
    c.ref = ref;
    c.reversed = reversed;
    return c;
  }

  static PlaneConstraint Perpendicular(int ref, bool reversed = false) {
    PlaneConstraint c;
    c.type = PlaneConstraintType::PERPENDICULAR;
    c.ref = ref;
    c.reversed = reversed;
    return c;
  }

  static PlaneConstraint Coincident(int ref, bool reversed = false) {
    PlaneConstraint c;
    c.type = PlaneConstraintType::COINCIDENT;
    c.ref = ref;
    c.reversed = reversed;
    return c;
  }

  static PlaneConstraint Distance(int ref, double distance,
                                  std::optional<CVector3D> defaultDir =
                                      std::nullopt,
                                  bool reversed = false) {
    PlaneConstraint c;
    c.type = PlaneConstraintType::DISTANCE;
    c.ref = ref;
    c.value = distance;
    c.defaultDir = defaultDir;
    c.reversed = reversed;
    return c;
  }

  static PlaneConstraint Angle(int ref, double angle,
                               std::optional<CVector3D> defaultDir =
                                   std::nullopt,
                               bool reversed = false) {
    PlaneConstraint c;
    c.type = PlaneConstraintType::ANGLE;
    c.ref = ref;
    c.value = angle;
    c.defaultDir = defaultDir;
    c.reversed = reversed;
    return c;
  }

  static PlaneConstraint Symmetric(int ref, bool reversed = false) {
    PlaneConstraint c;
    c.type = PlaneConstraintType::SYMMETRIC;
    c.ref = ref;
    c.reversed = reversed;
    return c;
  }

  static PlaneConstraint Tangent(int ref, bool reversed = false) {
    PlaneConstraint c;
    c.type = PlaneConstraintType::TANGENT;
    c.ref = ref;
    c.reversed = reversed;
    return c;
  }

  static PlaneConstraint Projection(int ref, bool reversed = false) {
    PlaneConstraint c;
    c.type = PlaneConstraintType::PROJECTION;
    c.ref = ref;
    c.reversed = reversed;
    return c;
  }
};

/**
 * @brief Datum plane builder for method, constraints and references.
 */
class DatumPlaneBuilder : public FeatureBuilderBase<CDatumPlane> {
public:
  DatumPlaneBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  /**
   * @brief Set datum plane construction method.
   */
  DatumPlaneBuilder &SetMethod(PlaneMethod method) {
    if (method == PlaneMethod::UNKNOWN) {
      throw std::runtime_error("Datum plane method must not be UNKNOWN.");
    }
    m_feature->method = method;
    return *this;
  }

  /**
   * @brief Convenience helper for line-based datum planes.
   *
   * Typical usage:
   *   SetLineMethod()
   *     .AddReference(Ref::Edge(...))
   *     .AddReference(Ref::Edge(...))
   *     .AddConstraintByFactory(PlaneConstraintBuilder::Coincident(0))
   *     .AddConstraintByFactory(PlaneConstraintBuilder::Coincident(1))
   */
  DatumPlaneBuilder &SetLineMethod() { return SetMethod(PlaneMethod::LINE); }

  DatumPlaneBuilder &SetProjectedOrigin(const CPoint3D &origin) {
    m_feature->projectedOrigin = origin;
    return *this;
  }

  DatumPlaneBuilder &SetNormal(const CVector3D &normal) {
    m_feature->normal = normal;
    return *this;
  }

  /**
   * @brief Add one reference from ReferenceBuilder facade/builder objects.
   *
   * Example:
   *   AddReference(Ref::XY())
   *   AddReference(Ref::Plane(model, "PlaneA"))
   *   AddReference(Ref::Face(parentId, 0))
   */
  template <typename RefBuilderT>
  DatumPlaneBuilder &AddReference(const RefBuilderT &refBuilder) {
    return AddReferenceRaw(ToReference(refBuilder));
  }

  /**
   * @brief Set all references (overwrite existing list).
   */
  template <typename... RefBuilderTs>
  DatumPlaneBuilder &SetReferences(const RefBuilderTs &...references) {
    m_feature->referenceEntities.clear();
    m_feature->referenceEntities.reserve(sizeof...(references));
    (AddReference(references), ...);
    return *this;
  }

  /**
   * @brief Add one reference and return its index for subsequent constraints.
   */
  template <typename RefBuilderT>
  int AddReferenceAndGetIndex(const RefBuilderT &refBuilder) {
    AddReference(refBuilder);
    return static_cast<int>(m_feature->referenceEntities.size()) - 1;
  }

  /**
   * @brief Add one constraint.
   */
  DatumPlaneBuilder &AddConstraint(const PlaneConstraint &constraint) {
    if (constraint.type == PlaneConstraintType::UNKNOWN) {
      throw std::runtime_error(
          "Datum plane constraint type must not be UNKNOWN.");
    }
    if (constraint.ref < 0) {
      throw std::runtime_error("Datum plane constraint ref must be >= 0.");
    }
    if (constraint.ref >=
        static_cast<int>(m_feature->referenceEntities.size())) {
      throw std::runtime_error(
          "Datum plane constraint ref index is out of range.");
    }
    m_feature->constraints.push_back(constraint);
    return *this;
  }

  /**
   * @brief Convenience overload to add one constraint.
   */
  DatumPlaneBuilder &
  AddConstraint(PlaneConstraintType type, int ref, double value = 0.0,
                std::optional<CVector3D> defaultDir = std::nullopt,
                bool reversed = false) {
    PlaneConstraint constraint;
    constraint.type = type;
    constraint.ref = ref;
    constraint.value = value;
    constraint.defaultDir = defaultDir;
    constraint.reversed = reversed;
    return AddConstraint(constraint);
  }

  /**
   * @brief Add a constraint by factory helper.
   */
  DatumPlaneBuilder &AddConstraintByFactory(const PlaneConstraint &constraint) {
    return AddConstraint(constraint);
  }

  /**
   * @brief Add a reference and one matching constraint in one call.
   */
  template <typename RefBuilderT>
  DatumPlaneBuilder &AddReferenceWithConstraint(const RefBuilderT &refBuilder,
                                                PlaneConstraintType type,
                                                double value = 0.0,
                                                std::optional<CVector3D> defaultDir = std::nullopt,
                                                bool reversed = false) {
    int refIndex = AddReferenceAndGetIndex(refBuilder);
    return AddConstraint(type, refIndex, value, defaultDir, reversed);
  }

  /**
   * @brief Clear all constraints.
   */
  DatumPlaneBuilder &ClearConstraints() {
    m_feature->constraints.clear();
    return *this;
  }

  /**
   * @brief Clear all references.
   */
  DatumPlaneBuilder &ClearReferences() {
    m_feature->referenceEntities.clear();
    return *this;
  }

private:
  DatumPlaneBuilder &
  AddReferenceRaw(const std::shared_ptr<CRefEntityBase> &reference) {
    if (!reference) {
      throw std::runtime_error("Datum plane reference must not be null.");
    }
    ValidateReference(reference);
    m_feature->referenceEntities.push_back(reference);
    return *this;
  }

  template <typename RefBuilderT>
  static std::shared_ptr<CRefEntityBase>
  ToReference(const RefBuilderT &refBuilder) {
    static_assert(std::is_convertible_v<RefBuilderT, std::shared_ptr<CRefEntityBase>>,
                  "Reference must be built by CADExchange::Builder::Ref::* wrappers.");
    return static_cast<std::shared_ptr<CRefEntityBase>>(refBuilder);
  }
};

} // namespace Builder
} // namespace CADExchange
