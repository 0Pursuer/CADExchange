// clang-format off
#include "ModelValidator.h"
#include "UnifiedFeatures.h"
#include <cmath>
#include <string>
#include <unordered_set>
// clang-format on

namespace CADExchange {

// One-liner delegation defined here to avoid including ModelValidator.h
// from UnifiedModel.h (which would create a circular dependency).
ValidationReport UnifiedModel::Validate() const {
  return ModelValidator::Validate(*this);
}

ValidationReport ModelValidator::Validate(const UnifiedModel &model) {
  ValidationReport report;

  // Collect sketchIDs referenced by Extrude/Revolve for SKETCH_001
  std::unordered_set<std::string> referencedSketchIDs;
  for (const auto &f : model.GetFeatures()) {
    if (auto ex = std::dynamic_pointer_cast<CExtrude>(f))
      if (!ex->profileSketchID.empty())
        referencedSketchIDs.insert(ex->profileSketchID);
    if (auto rv = std::dynamic_pointer_cast<CRevolve>(f))
      if (!rv->profileSketchID.empty())
        referencedSketchIDs.insert(rv->profileSketchID);
  }

  // depth magnitude threshold (convert to meters)
  auto toMeter = [&](double v) -> double {
    switch (model.unit) {
      case UnitType::MILLIMETER:  return v * 1e-3;
      case UnitType::CENTIMETER:  return v * 1e-2;
      case UnitType::INCH:        return v * 0.0254;
      case UnitType::FOOT:        return v * 0.3048;
      default:                    return v; // METER
    }
  };

  auto isZeroVec = [](const CVector3D &v) -> bool {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z) < GeoUtils::EPSILON;
  };

  auto addError = [&](const std::string &msg) {
    report.isValid = false;
    report.errors.push_back(msg);
  };
  auto addWarn = [&](const std::string &msg) {
    report.warnings.push_back(msg);
  };

  auto checkEC = [&](const ExtrudeEndCondition &ec,
                     const std::string &extrudeID,
                     const std::string &side,
                     const std::unordered_set<std::string> &seen) {
    using ECType = ExtrudeEndCondition::Type;

    if (ec.type == ECType::BLIND && ec.depth <= 0.0) {
      addError("[EXTRUDE_002] Extrude '" + extrudeID + "' " + side +
               " type=BLIND but depth=" + std::to_string(ec.depth) + " (must be > 0).");
    }
    if ((ec.type == ECType::UP_TO_FACE || ec.type == ECType::UP_TO_VERTEX) &&
        !ec.referenceEntity) {
      addError("[EXTRUDE_003] Extrude '" + extrudeID + "' " + side +
               " type=" + (ec.type == ECType::UP_TO_FACE ? "UP_TO_FACE" : "UP_TO_VERTEX") +
               " but referenceEntity is null.");
    }
    if (ec.type == ECType::UP_TO_FACE && ec.referenceEntity) {
      if (!std::dynamic_pointer_cast<CRefFace>(ec.referenceEntity)) {
        addError("[EXTRUDE_004] Extrude '" + extrudeID + "' " + side +
                 " type=UP_TO_FACE but referenceEntity is not CRefFace.");
      } else {
        auto face = std::dynamic_pointer_cast<CRefFace>(ec.referenceEntity);
        if (isZeroVec(face->normal)) {
          addError("[GEOM_002] Extrude '" + extrudeID + "' " + side +
                   " CRefFace normal is zero vector.");
        }
        CPoint3D zero{};
        if (face->centroid == zero) {
          addWarn("[GEOM_005] Extrude '" + extrudeID + "' " + side +
                  " CRefFace centroid is (0,0,0) -- may be unset.");
        }
      }
    }
    if (ec.type == ECType::UP_TO_VERTEX && ec.referenceEntity) {
      if (!std::dynamic_pointer_cast<CRefVertex>(ec.referenceEntity)) {
        addError("[EXTRUDE_005] Extrude '" + extrudeID + "' " + side +
                 " type=UP_TO_VERTEX but referenceEntity is not CRefVertex.");
      }
    }
    if (ec.type == ECType::BLIND) {
      double depthM = toMeter(ec.depth);
      if (depthM > 0.0 && (depthM < 1e-6 || depthM > 100.0)) {
        addWarn("[SCALE_001] Extrude '" + extrudeID + "' " + side +
                " BLIND depth=" + std::to_string(ec.depth) +
                " (~" + std::to_string(depthM * 1000.0) + "mm) is out of normal range -- "
                "check unit system.");
      }
    }
    if (ec.referenceEntity) {
      if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(ec.referenceEntity)) {
        if (!subTopo->parentFeatureID.empty() &&
            seen.find(subTopo->parentFeatureID) == seen.end()) {
          addWarn("[REF_002] Extrude '" + extrudeID + "' " + side +
                  " references feature '" + subTopo->parentFeatureID +
                  "' which has not been defined yet.");
        }
      }
    }
  };

  // Main validation loop
  std::unordered_set<std::string> seen;
  std::unordered_set<std::string> seenIDs;

  for (const auto &feature : model.GetFeatures()) {
    // MODEL_001
    if (feature->featureID.empty()) {
      addError("[MODEL_001] A feature has an empty featureID.");
      seen.insert("");
      continue;
    }
    // MODEL_002
    if (seenIDs.count(feature->featureID)) {
      addError("[MODEL_002] Duplicate featureID '" + feature->featureID + "'.");
    }
    seenIDs.insert(feature->featureID);

    // ---- CExtrude ----
    if (auto extrude = std::dynamic_pointer_cast<CExtrude>(feature)) {
      // EXTRUDE_001
      if (extrude->profileSketchID.empty()) {
        addError("[EXTRUDE_001] Extrude '" + extrude->featureID +
                 "' has empty profileSketchID.");
      }
      // REF_001
      else if (seen.find(extrude->profileSketchID) == seen.end()) {
        addError("[REF_001] Extrude '" + extrude->featureID +
                 "' references sketch '" + extrude->profileSketchID +
                 "' which has not been defined yet.");
      }
      // GEOM_001
      if (isZeroVec(extrude->direction)) {
        addError("[GEOM_001] Extrude '" + extrude->featureID +
                 "' direction is zero vector.");
      } else {
        double len = std::sqrt(extrude->direction.x * extrude->direction.x +
                               extrude->direction.y * extrude->direction.y +
                               extrude->direction.z * extrude->direction.z);
        if (std::abs(len - 1.0) > 0.01) {
          addWarn("[GEOM_006] Extrude '" + extrude->featureID +
                  "' direction length=" + std::to_string(len) +
                  " is not normalized (expected ~1.0).");
        }
      }
      checkEC(extrude->endCondition1, extrude->featureID, "EC1", seen);
      if (extrude->endCondition2)
        checkEC(*extrude->endCondition2, extrude->featureID, "EC2", seen);
      // EXTRUDE_006
      if (extrude->thinWall.has_value() && extrude->thinWall->thickness <= 0.0) {
        addError("[EXTRUDE_006] Extrude '" + extrude->featureID +
                 "' has ThinWall but Thickness=" +
                 std::to_string(extrude->thinWall->thickness) + " (must be > 0).");
      }
    }

    // ---- CSketch ----
    else if (auto sketch = std::dynamic_pointer_cast<CSketch>(feature)) {
      // SKETCH_001
      if (sketch->segments.empty() &&
          referencedSketchIDs.count(sketch->featureID)) {
        addWarn("[SKETCH_001] Sketch '" + sketch->featureID +
                "' is referenced by Extrude/Revolve but has no segments.");
      }
      // GEOM_003
      if (referencedSketchIDs.count(sketch->featureID) &&
          !sketch->sketchCSys.IsValid()) {
        addError("[GEOM_003] Sketch '" + sketch->featureID +
                 "' sketchCSys is not orthogonal.");
      }
      // REF_003
      if (sketch->referencePlane) {
        if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(
                sketch->referencePlane)) {
          if (!subTopo->parentFeatureID.empty() &&
              seen.find(subTopo->parentFeatureID) == seen.end()) {
            addWarn("[REF_003] Sketch '" + sketch->featureID +
                    "' referencePlane parent '" + subTopo->parentFeatureID +
                    "' has not been defined yet.");
          }
        }
      }
    }

    // ---- CRevolve ----
    else if (auto revolve = std::dynamic_pointer_cast<CRevolve>(feature)) {
      // REVOLVE_001
      if (revolve->profileSketchID.empty()) {
        addError("[REVOLVE_001] Revolve '" + revolve->featureID +
                 "' has empty profileSketchID.");
      }
      // REF_001 (revolve)
      else if (seen.find(revolve->profileSketchID) == seen.end()) {
        addError("[REF_001] Revolve '" + revolve->featureID +
                 "' references sketch '" + revolve->profileSketchID +
                 "' which has not been defined yet.");
      }
      // REVOLVE_002
      if (revolve->primaryAngle <= 0.0) {
        addError("[REVOLVE_002] Revolve '" + revolve->featureID +
                 "' primaryAngle=" + std::to_string(revolve->primaryAngle) +
                 " (must be > 0).");
      }
      // GEOM_004
      if (isZeroVec(revolve->axis.direction)) {
        addError("[GEOM_004] Revolve '" + revolve->featureID +
                 "' axis direction is zero vector.");
      }
    }

    seen.insert(feature->featureID);
  }

  return report;
}

} // namespace CADExchange

