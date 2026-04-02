// clang-format off
#include "ModelValidator.h"
#include "UnifiedFeatures.h"
#include <cmath>
#include <string>
#include <unordered_set>
// clang-format on

namespace CADExchange {

namespace {

bool IsBuiltinStandardDatumID(const std::string& id) {
  return id == StandardID::PLANE_XY ||
         id == StandardID::PLANE_YZ ||
         id == StandardID::PLANE_ZX;
}

} // namespace

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

  // length magnitude threshold (convert to meters)
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
  auto vecLen = [](const CVector3D &v) -> double {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  };

  auto addError = [&](const std::string &msg) {
    report.isValid = false;
    report.errors.push_back(msg);
  };
  auto addWarn = [&](const std::string &msg) {
    report.warnings.push_back(msg);
  };

  auto checkExtent = [&](const SweepExtent &extent,
                         const std::string &featureID,
                         const std::string &side,
                         const std::unordered_set<std::string> &seen,
                         const char *kind,
                         bool valueIsLength) {
    if (extent.type == SweepExtent::Type::UNKNOWN) {
      addError(std::string("[") + kind + "_002] " + kind + " '" + featureID +
               "' " + side + " extent type is UNKNOWN.");
      return;
    }

    if ((extent.type == SweepExtent::Type::VALUE ||
         extent.type == SweepExtent::Type::SYMMETRIC) &&
        extent.value <= 0.0) {
      addError(std::string("[") + kind + "_003] " + kind + " '" + featureID +
               "' " + side + " extent value=" + std::to_string(extent.value) +
               " (must be > 0).");
    }

    if (extent.type == SweepExtent::Type::UP_TO_ENTITY &&
        !extent.referenceEntity) {
      addError(std::string("[") + kind + "_004] " + kind + " '" + featureID +
               "' " + side + " extent requires referenceEntity.");
    }

    if (extent.referenceEntity) {
      if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(extent.referenceEntity)) {
        if (!subTopo->parentFeatureID.empty() &&
            seen.find(subTopo->parentFeatureID) == seen.end()) {
          addWarn(std::string("[REF_002] ") + kind + " '" + featureID + "' " + side +
                  " references feature '" + subTopo->parentFeatureID +
                  "' which has not been defined yet.");
        }
      }
    }

    if (valueIsLength) {
      double valueM = toMeter(extent.value);
      if (valueM > 0.0 && (valueM < 1e-6 || valueM > 100.0)) {
        addWarn(std::string("[SCALE_001] ") + kind + " '" + featureID + "' " + side +
                " extent value=" + std::to_string(extent.value) +
                " (~" + std::to_string(valueM * 1000.0) + "mm) is out of normal range -- "
                "check unit system.");
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
        double len = vecLen(extrude->direction);
        if (std::abs(len - 1.0) > 0.01) {
          addWarn("[GEOM_006] Extrude '" + extrude->featureID +
                  "' direction length=" + std::to_string(len) +
                  " is not normalized (expected ~1.0).");
        }
      }
      checkExtent(extrude->extent1, extrude->featureID, "Extent1", seen, "EXTRUDE", true);
      if (extrude->extent2)
        checkExtent(*extrude->extent2, extrude->featureID, "Extent2", seen, "EXTRUDE", true);
      if (extrude->thinWall.has_value() &&
          std::fabs(extrude->thinWall->startOffset) <= 1e-9 &&
          std::fabs(extrude->thinWall->endOffset) <= 1e-9) {
        addError("[EXTRUDE_006] Extrude '" + extrude->featureID +
                 "' has ThinWall but StartOffset/EndOffset are both zero.");
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
              !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
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
      // GEOM_004
      if (isZeroVec(revolve->axis.direction)) {
        addError("[GEOM_004] Revolve '" + revolve->featureID +
                 "' axis direction is zero vector.");
      }
      checkExtent(revolve->extent1, revolve->featureID, "Extent1", seen, "REVOLVE", false);
      if (revolve->extent2)
        checkExtent(*revolve->extent2, revolve->featureID, "Extent2", seen, "REVOLVE", false);
      if (revolve->thinWall.has_value() &&
          std::fabs(revolve->thinWall->startOffset) <= 1e-9 &&
          std::fabs(revolve->thinWall->endOffset) <= 1e-9) {
        addError("[REVOLVE_006] Revolve '" + revolve->featureID +
                 "' has ThinWall but StartOffset/EndOffset are both zero.");
      }
    }
    // ---- CDatumPlane ----
    else if (auto datumPlane = std::dynamic_pointer_cast<CDatumPlane>(feature)) {
      if (datumPlane->method == PlaneMethod::UNKNOWN) {
        addError("[DATUM_001] DatumPlane '" + datumPlane->featureID +
                 "' method is UNKNOWN.");
      }
      if (datumPlane->referenceEntities.empty()) {
        addError("[DATUM_002] DatumPlane '" + datumPlane->featureID +
                 "' has no referenceEntities.");
      }
      if (datumPlane->constraints.empty()) {
        addWarn("[DATUM_003] DatumPlane '" + datumPlane->featureID +
                "' has no constraints.");
      }

      bool hasDistance = false;
      bool hasAngle = false;
      for (size_t i = 0; i < datumPlane->constraints.size(); ++i) {
        const auto &constraint = datumPlane->constraints[i];
        const std::string idx = std::to_string(i);

        if (constraint.type == PlaneConstraintType::UNKNOWN) {
          addError("[DATUM_004] DatumPlane '" + datumPlane->featureID +
                   "' constraint[" + idx + "] type is UNKNOWN.");
        }
        if (constraint.ref < 0 ||
            constraint.ref >= static_cast<int>(datumPlane->referenceEntities.size())) {
          addError("[DATUM_005] DatumPlane '" + datumPlane->featureID +
                   "' constraint[" + idx + "] ref=" + std::to_string(constraint.ref) +
                   " is out of range [0, " +
                   std::to_string(datumPlane->referenceEntities.empty()
                                      ? 0
                                      : datumPlane->referenceEntities.size() - 1) +
                   "].");
        }

        if (constraint.type == PlaneConstraintType::DISTANCE) {
          hasDistance = true;
          if (std::abs(constraint.value) < GeoUtils::EPSILON) {
            addWarn("[DATUM_006] DatumPlane '" + datumPlane->featureID +
                    "' constraint[" + idx +
                    "] DISTANCE value is near zero.");
          }
        }
        if (constraint.type == PlaneConstraintType::ANGLE) {
          hasAngle = true;
          if (std::abs(constraint.value) < GeoUtils::EPSILON) {
            addWarn("[DATUM_007] DatumPlane '" + datumPlane->featureID +
                    "' constraint[" + idx + "] ANGLE value is near zero.");
          }
        }

        if (constraint.defaultDir.has_value()) {
          double len = vecLen(*constraint.defaultDir);
          if (len < GeoUtils::EPSILON) {
            addError("[GEOM_007] DatumPlane '" + datumPlane->featureID +
                     "' constraint[" + idx + "] defaultDir is zero vector.");
          } else if (std::abs(len - 1.0) > 0.01) {
            addWarn("[GEOM_008] DatumPlane '" + datumPlane->featureID +
                    "' constraint[" + idx + "] defaultDir length=" +
                    std::to_string(len) + " is not normalized.");
          }
        }

        if (constraint.ref >= 0 &&
            constraint.ref < static_cast<int>(datumPlane->referenceEntities.size())) {
          const auto &ref = datumPlane->referenceEntities[constraint.ref];
          if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(ref)) {
            if (!subTopo->parentFeatureID.empty() &&
                seen.find(subTopo->parentFeatureID) == seen.end()) {
              addWarn("[REF_004] DatumPlane '" + datumPlane->featureID +
                      "' constraint[" + idx + "] references parent feature '" +
                      subTopo->parentFeatureID +
                      "' which has not been defined yet.");
            }
          }
        }
      }

      if (datumPlane->method == PlaneMethod::OFFSET && !hasDistance) {
        addWarn("[DATUM_008] DatumPlane '" + datumPlane->featureID +
                "' method=OFFSET but no DISTANCE constraint found.");
      }
      if (datumPlane->method == PlaneMethod::ANGLE && !hasAngle) {
        addWarn("[DATUM_009] DatumPlane '" + datumPlane->featureID +
                "' method=ANGLE but no ANGLE constraint found.");
      }
      if (datumPlane->method == PlaneMethod::THREE_POINTS &&
          datumPlane->referenceEntities.size() != 3) {
        addWarn("[DATUM_010] DatumPlane '" + datumPlane->featureID +
                "' method=THREE_POINTS expects 3 references, actual=" +
                std::to_string(datumPlane->referenceEntities.size()) + ".");
      }
      if (datumPlane->method == PlaneMethod::MID_PLANE &&
          datumPlane->referenceEntities.size() < 2) {
        addWarn("[DATUM_011] DatumPlane '" + datumPlane->featureID +
                "' method=MID_PLANE typically requires at least 2 references.");
      }
      if (datumPlane->method == PlaneMethod::LINE &&
          datumPlane->referenceEntities.size() < 1) {
        addWarn("[DATUM_012] DatumPlane '" + datumPlane->featureID +
                "' method=LINE typically requires one or more linear references.");
      }
    }

    seen.insert(feature->featureID);
  }

  return report;
}

} // namespace CADExchange

