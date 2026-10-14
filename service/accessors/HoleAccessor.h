#pragma once
#include "../../core/HoleGeometry.h"
#include "FeatureAccessorBase.h"
#include <stdexcept>

namespace CADExchange::Accessor {
class HoleAccessor : public FeatureAccessorBase {
  std::shared_ptr<const CHole> hole_;
  const CHole &Checked() const {
    if (!hole_)
      throw std::invalid_argument("invalid Hole accessor");
    return *hole_;
  }

public:
  explicit HoleAccessor(std::shared_ptr<const CFeatureBase> feature)
      : FeatureAccessorBase(feature),
        hole_(std::dynamic_pointer_cast<const CHole>(feature)) {}
  explicit HoleAccessor(const FeatureAccessorBase &feature)
      : HoleAccessor(feature.GetRaw()) {}
  const CHole *Data() const { return hole_.get(); }
  bool IsValid() const override { return hole_ != nullptr; }
  std::shared_ptr<CRefEntityBase> GetSupportReference() const {
    return Checked().supportRef;
  }
  CVector3D GetDirection() const { return Checked().direction; }
  const std::vector<CHoleLocation> &GetLocations() const {
    return Checked().locations;
  }
  double GetDiameter() const { return Checked().diameter; }
  const SweepExtent &GetExtent() const { return Checked().extent; }
  HoleBottomKind GetBottomKind() const { return Checked().bottom; }
  HoleDepthReference GetDepthReference() const {
    return Checked().depthReference;
  }
  std::optional<double> GetDrillPointAngle() const {
    return Checked().drillPointAngle;
  }
  std::optional<CHoleCounterbore> GetCounterbore() const {
    return Checked().counterbore;
  }
  std::optional<CHoleCountersink> GetCountersink() const {
    return Checked().countersink;
  }
  bool ResolveGeometry(HoleResolvedGeometry &geometry,
                       std::string &detail) const {
    if (!hole_) {
      detail = "invalid Hole accessor";
      return false;
    }
    return ResolveHoleGeometry(*hole_, geometry, detail);
  }
};
} // namespace CADExchange::Accessor
