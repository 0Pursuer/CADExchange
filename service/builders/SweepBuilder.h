#pragma once
#include "FeatureBuilderBase.h"

#include <cmath>
#include <stdexcept>

namespace CADExchange {
namespace Builder {

/**
 * @brief Builder for solid sweep features.
 */
class SweepBuilder : public FeatureBuilderBase<CSweep> {
public:
  SweepBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  /**
   * @brief Set the sweep profile by sketch feature ID.
   */
  SweepBuilder &SetProfile(const std::string &sketchID) {
    if (!m_model.GetFeatureAs<CSketch>(sketchID)) {
      throw std::runtime_error("Sketch profile not found: " + sketchID);
    }
    m_feature->profileSketchID = sketchID;
    m_feature->profile.kind = SweepProfileKind::SketchReference;
    m_feature->profile.sketchID = sketchID;
    m_feature->profile.embedded.reset();
    m_feature->profile.circular.reset();
    return *this;
  }

  /**
   * @brief Set the sweep profile by sketch feature name.
   */
  SweepBuilder &SetProfileByName(const std::string &sketchName) {
    auto sketchId = m_model.GetFeatureIdByName(sketchName);
    if (sketchId.empty() || sketchId == "UnknownSketchId") {
      throw std::runtime_error("Sketch not found by name: " + sketchName);
    }
    return SetProfile(sketchId);
  }

  SweepBuilder &SetEmbeddedProfile(const CSketch &sketch) {
    if (sketch.segments.empty()) {
      throw std::runtime_error(
          "Sweep embedded profile must contain at least one sketch segment.");
    }
    CSweepEmbeddedProfile embedded;
    embedded.sketch = sketch;
    m_feature->profile.kind = SweepProfileKind::EmbeddedSketch;
    m_feature->profile.sketchID.clear();
    m_feature->profile.embedded = embedded;
    m_feature->profile.circular.reset();
    m_feature->profileSketchID.clear();
    return *this;
  }

  SweepBuilder &SetCircularProfile(double outerRadius,
                                   double innerRadius = 0.0) {
    if (outerRadius <= 0.0) {
      throw std::runtime_error(
          "Sweep circular profile outer radius must be positive.");
    }
    if (innerRadius < 0.0 || innerRadius >= outerRadius) {
      throw std::runtime_error(
          "Sweep circular profile inner radius must be non-negative and less "
          "than outer radius.");
    }
    CSweepCircularProfile circular;
    circular.outerRadius = outerRadius;
    circular.innerRadius = innerRadius;
    m_feature->profile.kind = SweepProfileKind::Circular;
    m_feature->profile.sketchID.clear();
    m_feature->profile.embedded.reset();
    m_feature->profile.circular = circular;
    m_feature->profileSketchID.clear();
    return *this;
  }

  /**
   * @brief Set the Boolean operation.
   */
  SweepBuilder &SetOperation(BooleanOp op) {
    m_feature->operation = op;
    return *this;
  }

  /**
   * @brief Add one path reference. The path can reference sketches, sketch
   * segments, model edges, or other curve-like reference entities.
   */
  SweepBuilder &
  AddPathReference(const std::shared_ptr<CRefEntityBase> &reference) {
    if (!reference) {
      throw std::runtime_error("Sweep path reference must not be null.");
    }
    ValidatePathReference(reference);
    m_feature->path.references.push_back(reference);
    return *this;
  }

  /**
   * @brief Replace the full ordered path reference chain.
   */
  SweepBuilder &
  SetPathReferences(const std::vector<std::shared_ptr<CRefEntityBase>> &refs) {
    m_feature->path.references.clear();
    for (const auto &ref : refs) {
      AddPathReference(ref);
    }
    return *this;
  }

  SweepBuilder &SetPathStartPoint(const CPoint3D &point) {
    m_feature->path.startPoint = point;
    return *this;
  }

  SweepBuilder &SetPathEndPoint(const CPoint3D &point) {
    m_feature->path.endPoint = point;
    return *this;
  }

  SweepBuilder &SetPathClosed(bool isClosed = true) {
    m_feature->path.isClosed = isClosed;
    return *this;
  }

  SweepBuilder &
  AddGuidePathReference(const std::shared_ptr<CRefEntityBase> &reference,
                        size_t guidePathIndex = 0) {
    if (!reference) {
      throw std::runtime_error("Sweep guide path reference must not be null.");
    }
    ValidatePathReference(reference);
    EnsureGuidePath(guidePathIndex);
    m_feature->guidePaths[guidePathIndex].references.push_back(reference);
    return *this;
  }

  SweepBuilder &AddGuidePath(const CSweepPath &guidePath) {
    for (const auto &ref : guidePath.references) {
      if (!ref) {
        throw std::runtime_error("Sweep guide path reference must not be null.");
      }
      ValidatePathReference(ref);
    }
    m_feature->guidePaths.push_back(guidePath);
    return *this;
  }

  SweepBuilder &SetGuidePathStartPoint(size_t guidePathIndex,
                                       const CPoint3D &point) {
    EnsureGuidePath(guidePathIndex);
    m_feature->guidePaths[guidePathIndex].startPoint = point;
    return *this;
  }

  SweepBuilder &SetGuidePathEndPoint(size_t guidePathIndex,
                                     const CPoint3D &point) {
    EnsureGuidePath(guidePathIndex);
    m_feature->guidePaths[guidePathIndex].endPoint = point;
    return *this;
  }

  SweepBuilder &SetGuidePathClosed(size_t guidePathIndex,
                                   bool isClosed = true) {
    EnsureGuidePath(guidePathIndex);
    m_feature->guidePaths[guidePathIndex].isClosed = isClosed;
    return *this;
  }

  SweepBuilder &SetOrientation(SweepPathOrientation orientation) {
    m_feature->orientation = orientation;
    return *this;
  }

  SweepBuilder &SetSectionPlacement(SweepSectionPlacement placement) {
    m_feature->sectionPlacement = placement;
    return *this;
  }

  SweepBuilder &SetProfilePathAngleCos(double value) {
    if (value < -1.0 || value > 1.0) {
      throw std::runtime_error(
          "Sweep profile/path angle cosine must be within [-1, 1].");
    }
    m_feature->profilePathAngleCos = value;
    return *this;
  }

  SweepBuilder &SetThinWall(double thickness, bool isOneSided = true,
                            bool isOutward = false, bool isCovered = false) {
    if (thickness <= 0) {
      throw std::runtime_error("Thickness must be positive.");
    }
    if (isOneSided) {
      return SetThinWallOffsets(isOutward ? 0.0 : -thickness,
                                isOutward ? thickness : 0.0, isCovered);
    }
    return SetThinWallOffsets(-thickness * 0.5, thickness * 0.5, isCovered);
  }

  SweepBuilder &SetThinWallOffsets(double startOffset, double endOffset,
                                   bool isCovered = false) {
    if (std::fabs(startOffset) <= 1e-9 && std::fabs(endOffset) <= 1e-9) {
      throw std::runtime_error("Thin wall offsets must not both be zero.");
    }
    ThinWallOption tw;
    tw.isCovered = isCovered;
    tw.startOffset = startOffset;
    tw.endOffset = endOffset;
    m_feature->thinWall = tw;
    return *this;
  }

private:
  void EnsureGuidePath(size_t guidePathIndex) {
    if (m_feature->guidePaths.size() <= guidePathIndex) {
      m_feature->guidePaths.resize(guidePathIndex + 1);
    }
  }

  void ValidatePathReference(const std::shared_ptr<CRefEntityBase> &ref) const {
    ValidateReference(ref);

    if (auto sketch = std::dynamic_pointer_cast<CRefSketch>(ref)) {
      if (!m_model.GetFeatureAs<CSketch>(sketch->targetFeatureID)) {
        throw std::runtime_error("Sweep path sketch not found: " +
                                 sketch->targetFeatureID);
      }
      return;
    }

    if (auto seg = std::dynamic_pointer_cast<CRefSketchSeg>(ref)) {
      auto sketch = m_model.GetFeatureAs<CSketch>(seg->parentFeatureID);
      if (!sketch) {
        throw std::runtime_error("Sweep path sketch segment parent not found: " +
                                 seg->parentFeatureID);
      }
      if (seg->segmentLocalID.empty()) {
        throw std::runtime_error(
            "Sweep path sketch segment local ID must not be empty.");
      }
      return;
    }

    if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(ref)) {
      if (!subTopo->parentFeatureID.empty() &&
          !m_model.GetFeature(subTopo->parentFeatureID)) {
        throw std::runtime_error("Sweep path parent feature not found: " +
                                 subTopo->parentFeatureID);
      }
    }
  }
};

} // namespace Builder
} // namespace CADExchange
