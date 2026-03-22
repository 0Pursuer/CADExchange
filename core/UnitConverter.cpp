#include "UnifiedModel.h"
#include <unordered_set>

namespace CADExchange {

namespace {

bool IsSupportedUnitForConversion(UnitType unit) {
  return unit == UnitType::METER || unit == UnitType::CENTIMETER ||
         unit == UnitType::MILLIMETER || unit == UnitType::INCH ||
         unit == UnitType::FOOT;
}

const char *UnitTypeToString(UnitType unit) {
  switch (unit) {
  case UnitType::METER:
    return "METER";
  case UnitType::MILLIMETER:
    return "MILLIMETER";
  case UnitType::INCH:
    return "INCH";
  case UnitType::CENTIMETER:
    return "CENTIMETER";
  case UnitType::FOOT:
    return "FOOT";
  default:
    return "UNKNOWN";
  }
}

bool TryGetMeterScale(UnitType unit, double &scaleToMeter) {
  switch (unit) {
  case UnitType::METER:
    scaleToMeter = 1.0;
    return true;
  case UnitType::CENTIMETER:
    scaleToMeter = 1e-2;
    return true;
  case UnitType::MILLIMETER:
    scaleToMeter = 1e-3;
    return true;
  case UnitType::INCH:
    scaleToMeter = 0.0254;
    return true;
  case UnitType::FOOT:
    scaleToMeter = 0.3048;
    return true;
  default:
    return false;
  }
}

inline void ScalePoint(CPoint3D &p, double factor) {
  p.x *= factor;
  p.y *= factor;
  p.z *= factor;
}

struct UnitScaleContext {
  std::unordered_set<const CRefEntityBase *> scaledRefs;
};

void ScaleRefEntity(const std::shared_ptr<CRefEntityBase> &ref, double factor,
                    UnitScaleContext &ctx) {
  if (!ref) {
    return;
  }
  const CRefEntityBase *key = ref.get();
  if (ctx.scaledRefs.find(key) != ctx.scaledRefs.end()) {
    return;
  }
  ctx.scaledRefs.insert(key);

  if (auto plane = std::dynamic_pointer_cast<CRefPlane>(ref)) {
    ScalePoint(plane->origin, factor);
    return;
  }
  if (auto face = std::dynamic_pointer_cast<CRefFace>(ref)) {
    ScalePoint(face->centroid, factor);
    return;
  }
  if (auto edge = std::dynamic_pointer_cast<CRefEdge>(ref)) {
    ScalePoint(edge->startPoint, factor);
    ScalePoint(edge->endPoint, factor);
    ScalePoint(edge->midPoint, factor);
    return;
  }
  if (auto vertex = std::dynamic_pointer_cast<CRefVertex>(ref)) {
    ScalePoint(vertex->pos, factor);
    return;
  }
  if (auto axis = std::dynamic_pointer_cast<CRefAxis>(ref)) {
    ScalePoint(axis->origin, factor);
    return;
  }
  if (auto point = std::dynamic_pointer_cast<CRefPoint>(ref)) {
    ScalePoint(point->position, factor);
    return;
  }
}

void ScaleSketch(CSketch &sketch, double factor, UnitScaleContext &ctx) {
  ScaleRefEntity(sketch.referencePlane, factor, ctx);
  ScalePoint(sketch.sketchCSys.origin, factor);

  for (auto &seg : sketch.segments) {
    if (!seg) {
      continue;
    }
    if (auto line = std::dynamic_pointer_cast<CSketchLine>(seg)) {
      ScalePoint(line->startPos, factor);
      ScalePoint(line->endPos, factor);
      continue;
    }
    if (auto circle = std::dynamic_pointer_cast<CSketchCircle>(seg)) {
      ScalePoint(circle->center, factor);
      circle->radius *= factor;
      continue;
    }
    if (auto arc = std::dynamic_pointer_cast<CSketchArc>(seg)) {
      ScalePoint(arc->center, factor);
      arc->radius *= factor;
      continue;
    }
    if (auto point = std::dynamic_pointer_cast<CSketchPoint>(seg)) {
      ScalePoint(point->position, factor);
      continue;
    }
  }

  for (auto &constraint : sketch.constraints) {
    if (constraint.type == CSketchConstraint::ConstraintType::DIMENSIONAL) {
      constraint.dimensionValue *= factor;
    }
  }
}

void ScaleExtrudeEndCondition(ExtrudeEndCondition &endCondition, double factor,
                              UnitScaleContext &ctx) {
  endCondition.depth *= factor;
  endCondition.offset *= factor;
  ScaleRefEntity(endCondition.referenceEntity, factor, ctx);
}

void ScaleExtrude(CExtrude &extrude, double factor, UnitScaleContext &ctx) {
  ScaleExtrudeEndCondition(extrude.endCondition1, factor, ctx);
  if (extrude.endCondition2.has_value()) {
    ScaleExtrudeEndCondition(*extrude.endCondition2, factor, ctx);
  }
  if (extrude.thinWall.has_value()) {
    extrude.thinWall->thickness *= factor;
  }
}

void ScaleRevolve(CRevolve &revolve, double factor, UnitScaleContext &ctx) {
  ScalePoint(revolve.axis.origin, factor);
  ScaleRefEntity(revolve.axis.referenceEntity, factor, ctx);
}

} // namespace

bool ConvertModelUnit(UnifiedModel &model, UnitType targetUnit,
                      std::string *errorMessage) {
  if (model.unit == targetUnit) {
    if (errorMessage) {
      errorMessage->clear();
    }
    return true;
  }

  if (!IsSupportedUnitForConversion(model.unit) ||
      !IsSupportedUnitForConversion(targetUnit)) {
    if (errorMessage) {
      *errorMessage = "ConvertModelUnit only supports METER/CENTIMETER/MILLIMETER/INCH/FOOT "
                      "(source=" +
                      std::string(UnitTypeToString(model.unit)) + ", target=" +
                      std::string(UnitTypeToString(targetUnit)) + ")";
    }
    return false;
  }

  double sourceToMeter = 1.0;
  double targetToMeter = 1.0;
  if (!TryGetMeterScale(model.unit, sourceToMeter) ||
      !TryGetMeterScale(targetUnit, targetToMeter)) {
    if (errorMessage) {
      *errorMessage = "ConvertModelUnit failed to resolve unit scales.";
    }
    return false;
  }

  const double factor = sourceToMeter / targetToMeter;
  UnitScaleContext ctx;

  model.ForEachMutable([&](std::shared_ptr<CFeatureBase> &feature) {
    if (!feature)
      return;
    switch (feature->featureType) {
      case FeatureType::Sketch:
        ScaleSketch(*std::static_pointer_cast<CSketch>(feature), factor, ctx);
        break;
      case FeatureType::Extrude:
        ScaleExtrude(*std::static_pointer_cast<CExtrude>(feature), factor, ctx);
        break;
      case FeatureType::Revolve:
        ScaleRevolve(*std::static_pointer_cast<CRevolve>(feature), factor, ctx);
        break;
      default:
        break;
    }
  });

  model.unit = targetUnit;
  if (errorMessage) {
    errorMessage->clear();
  }
  return true;
}

} // namespace CADExchange
