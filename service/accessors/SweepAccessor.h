#pragma once
#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"

#include <cmath>
#include <memory>
#include <string>

namespace CADExchange {
namespace Accessor {

/**
 * @brief Read-only accessor for solid sweep features.
 */
class SweepAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CSweep> m_sweep;

public:
  explicit SweepAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_sweep = std::dynamic_pointer_cast<const CSweep>(feat);
  }

  explicit SweepAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_sweep = std::dynamic_pointer_cast<const CSweep>(other.GetRaw());
  }

  explicit SweepAccessor(std::shared_ptr<FeatureAccessorBase> feat)
      : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
    if (feat) {
      m_sweep = std::dynamic_pointer_cast<const CSweep>(feat->GetRaw());
    }
  }

  const CSweep *Data() const { return m_sweep.get(); }

  const CSweep *operator->() const { return m_sweep.get(); }

  bool IsValid() const override { return m_sweep != nullptr; }

  std::string GetProfileSketchID() const {
    if (!IsValid()) {
      return "";
    }
    return !Data()->profile.sketchID.empty() ? Data()->profile.sketchID
                                             : Data()->profileSketchID;
  }

  SweepProfileKind GetProfileKind() const {
    return IsValid() ? Data()->profile.kind : SweepProfileKind::SketchReference;
  }

  SweepSectionPlacement GetSectionPlacement() const {
    return IsValid() ? Data()->sectionPlacement
                     : SweepSectionPlacement::ExistingProfilePlane;
  }

  bool HasProfilePathAngleCos() const {
    return IsValid() && Data()->profilePathAngleCos.has_value();
  }

  double GetProfilePathAngleCos() const {
    return HasProfilePathAngleCos() ? *Data()->profilePathAngleCos : 0.0;
  }

  bool HasEmbeddedProfile() const {
    return IsValid() && Data()->profile.embedded.has_value();
  }

  int GetEmbeddedProfileSegmentCount() const {
    return HasEmbeddedProfile()
               ? static_cast<int>(Data()->profile.embedded->sketch.segments.size())
               : 0;
  }

  const CSketch *GetEmbeddedProfileSketch() const {
    return HasEmbeddedProfile() ? &Data()->profile.embedded->sketch : nullptr;
  }

  bool HasCircularProfile() const {
    return IsValid() && Data()->profile.circular.has_value();
  }

  double GetCircularOuterRadius() const {
    return HasCircularProfile() ? Data()->profile.circular->outerRadius : 0.0;
  }

  double GetCircularInnerRadius() const {
    return HasCircularProfile() ? Data()->profile.circular->innerRadius : 0.0;
  }

  BooleanOp GetOperation() const {
    return IsValid() ? Data()->operation : BooleanOp::BOSS;
  }

  SweepPathOrientation GetOrientation() const {
    return IsValid() ? Data()->orientation : SweepPathOrientation::FollowPath;
  }

  int GetPathReferenceCount() const {
    return IsValid() ? static_cast<int>(Data()->path.references.size()) : 0;
  }

  ReferenceAccessor GetPathReference(int index) const {
    if (!IsValid() || index < 0 ||
        index >= static_cast<int>(Data()->path.references.size())) {
      return ReferenceAccessor(nullptr);
    }
    return ReferenceAccessor(Data()->path.references[static_cast<size_t>(index)]);
  }

  bool IsPathClosed() const { return IsValid() && Data()->path.isClosed; }

  bool HasPathStartPoint() const {
    return IsValid() && Data()->path.startPoint.has_value();
  }

  CPoint3D GetPathStartPoint() const {
    return HasPathStartPoint() ? *Data()->path.startPoint : CPoint3D{0, 0, 0};
  }

  bool HasPathEndPoint() const {
    return IsValid() && Data()->path.endPoint.has_value();
  }

  CPoint3D GetPathEndPoint() const {
    return HasPathEndPoint() ? *Data()->path.endPoint : CPoint3D{0, 0, 0};
  }

  int GetGuidePathCount() const {
    return IsValid() ? static_cast<int>(Data()->guidePaths.size()) : 0;
  }

  int GetGuidePathReferenceCount(int guidePathIndex) const {
    if (!IsValid() || guidePathIndex < 0 ||
        guidePathIndex >= static_cast<int>(Data()->guidePaths.size())) {
      return 0;
    }
    return static_cast<int>(
        Data()->guidePaths[static_cast<size_t>(guidePathIndex)]
            .references.size());
  }

  ReferenceAccessor GetGuidePathReference(int guidePathIndex,
                                          int referenceIndex) const {
    if (!IsValid() || guidePathIndex < 0 ||
        guidePathIndex >= static_cast<int>(Data()->guidePaths.size())) {
      return ReferenceAccessor(nullptr);
    }
    const auto &guidePath =
        Data()->guidePaths[static_cast<size_t>(guidePathIndex)];
    if (referenceIndex < 0 ||
        referenceIndex >= static_cast<int>(guidePath.references.size())) {
      return ReferenceAccessor(nullptr);
    }
    return ReferenceAccessor(
        guidePath.references[static_cast<size_t>(referenceIndex)]);
  }

  bool IsGuidePathClosed(int guidePathIndex) const {
    return IsValid() && guidePathIndex >= 0 &&
           guidePathIndex < static_cast<int>(Data()->guidePaths.size()) &&
           Data()->guidePaths[static_cast<size_t>(guidePathIndex)].isClosed;
  }

  bool HasGuidePathStartPoint(int guidePathIndex) const {
    return IsValid() && guidePathIndex >= 0 &&
           guidePathIndex < static_cast<int>(Data()->guidePaths.size()) &&
           Data()->guidePaths[static_cast<size_t>(guidePathIndex)]
               .startPoint.has_value();
  }

  CPoint3D GetGuidePathStartPoint(int guidePathIndex) const {
    return HasGuidePathStartPoint(guidePathIndex)
               ? *Data()->guidePaths[static_cast<size_t>(guidePathIndex)]
                      .startPoint
               : CPoint3D{0, 0, 0};
  }

  bool HasGuidePathEndPoint(int guidePathIndex) const {
    return IsValid() && guidePathIndex >= 0 &&
           guidePathIndex < static_cast<int>(Data()->guidePaths.size()) &&
           Data()->guidePaths[static_cast<size_t>(guidePathIndex)]
               .endPoint.has_value();
  }

  CPoint3D GetGuidePathEndPoint(int guidePathIndex) const {
    return HasGuidePathEndPoint(guidePathIndex)
               ? *Data()->guidePaths[static_cast<size_t>(guidePathIndex)]
                      .endPoint
               : CPoint3D{0, 0, 0};
  }

  bool HasThinWall() const {
    return IsValid() && Data()->thinWall.has_value();
  }

  double GetThinWallThickness() const {
    if (!HasThinWall()) {
      return 0.0;
    }
    const double start = Data()->thinWall->startOffset;
    const double end = Data()->thinWall->endOffset;
    const double absStart = std::fabs(start);
    const double absEnd = std::fabs(end);
    if (start < -1e-9 && end > 1e-9) {
      return absStart + absEnd;
    }
    return absStart > absEnd ? absStart : absEnd;
  }

  bool IsThinWallOneSided() const {
    return HasThinWall()
               ? (std::fabs(Data()->thinWall->startOffset) <= 1e-9 ||
                  std::fabs(Data()->thinWall->endOffset) <= 1e-9)
               : false;
  }

  bool IsThinWallOutward() const {
    return HasThinWall() &&
           std::fabs(Data()->thinWall->startOffset) <= 1e-9 &&
           Data()->thinWall->endOffset > 1e-9;
  }

  bool IsThinWallCovered() const {
    return HasThinWall() && Data()->thinWall->isCovered;
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
