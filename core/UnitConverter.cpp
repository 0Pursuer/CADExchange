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
    for (auto &ref : constraint.refs) {
      if (ref.kind == SketchConstraintRefKind::ExternalReference) {
        ScaleRefEntity(ref.refEntity, factor, ctx);
      }
    }
    if (!constraint.value.has_value()) {
      continue;
    }
    switch (constraint.type) {
    case CSketchConstraint::ConstraintType::DISTANCE:
    case CSketchConstraint::ConstraintType::RADIUS:
    case CSketchConstraint::ConstraintType::DIAMETER:
      *constraint.value *= factor;
      break;
    default:
      break;
    }
  }
}

void ScaleSweepExtent(SweepExtent &extent, double factor,
                      UnitScaleContext &ctx, bool scaleValue) {
  if (scaleValue) {
    extent.value *= factor;
  }
  extent.offset *= factor;
  ScaleRefEntity(extent.referenceEntity, factor, ctx);
  if (extent.helperPoint.has_value()) {
    ScalePoint(*extent.helperPoint, factor);
  }
}

void ScaleExtrude(CExtrude &extrude, double factor, UnitScaleContext &ctx) {
  ScaleSweepExtent(extrude.extent1, factor, ctx, true);
  if (extrude.extent2.has_value()) {
    ScaleSweepExtent(*extrude.extent2, factor, ctx, true);
  }
  if (extrude.thinWall.has_value()) {
    extrude.thinWall->startOffset *= factor;
    extrude.thinWall->endOffset *= factor;
  }
}

void ScaleRevolve(CRevolve &revolve, double factor, UnitScaleContext &ctx) {
  ScalePoint(revolve.axis.origin, factor);
  ScaleRefEntity(revolve.axis.referenceEntity, factor, ctx);
  // Revolve extent.value expresses angles and must not be unit-scaled.
  ScaleSweepExtent(revolve.extent1, factor, ctx, false);
  if (revolve.extent2.has_value()) {
    ScaleSweepExtent(*revolve.extent2, factor, ctx, false);
  }
  if (revolve.thinWall.has_value()) {
    revolve.thinWall->startOffset *= factor;
    revolve.thinWall->endOffset *= factor;
  }
}

void ScaleSweep(CSweep &sweep, double factor, UnitScaleContext &ctx) {
  if (sweep.profile.embedded.has_value()) {
    ScaleSketch(sweep.profile.embedded->sketch, factor, ctx);
  }
  if (sweep.profile.circular.has_value()) {
    sweep.profile.circular->outerRadius *= factor;
    sweep.profile.circular->innerRadius *= factor;
  }
  for (auto &ref : sweep.path.references) {
    ScaleRefEntity(ref, factor, ctx);
  }
  if (sweep.path.startPoint.has_value()) {
    ScalePoint(*sweep.path.startPoint, factor);
  }
  if (sweep.path.endPoint.has_value()) {
    ScalePoint(*sweep.path.endPoint, factor);
  }
  for (auto &guidePath : sweep.guidePaths) {
    for (auto &ref : guidePath.references) {
      ScaleRefEntity(ref, factor, ctx);
    }
    if (guidePath.startPoint.has_value()) {
      ScalePoint(*guidePath.startPoint, factor);
    }
    if (guidePath.endPoint.has_value()) {
      ScalePoint(*guidePath.endPoint, factor);
    }
  }
  if (sweep.thinWall.has_value()) {
    sweep.thinWall->startOffset *= factor;
    sweep.thinWall->endOffset *= factor;
  }
}

void ScaleChamfer(CChamfer &chamfer, double factor, UnitScaleContext &ctx) {
  if (chamfer.params.distance1.has_value()) {
    *chamfer.params.distance1 *= factor;
  }
  if (chamfer.params.distance2.has_value()) {
    *chamfer.params.distance2 *= factor;
  }
  if (chamfer.params.distance3.has_value()) {
    *chamfer.params.distance3 *= factor;
  }
  if (chamfer.params.offset1.has_value()) {
    *chamfer.params.offset1 *= factor;
  }
  if (chamfer.params.offset2.has_value()) {
    *chamfer.params.offset2 *= factor;
  }
  if (chamfer.firstEndFaceMarker.has_value()) {
    ScalePoint(*chamfer.firstEndFaceMarker, factor);
  }
  for (auto &ref : chamfer.references) {
    ScaleRefEntity(ref, factor, ctx);
  }
}

void ScaleFillet(CFillet &fillet, double factor, UnitScaleContext &ctx) {
  if (fillet.params.primaryValue.has_value()) {
    *fillet.params.primaryValue *= factor;
  }
  if (fillet.params.secondValue.has_value()) {
    *fillet.params.secondValue *= factor;
  }
  if (fillet.params.conicValue.has_value() &&
      fillet.params.conicValueMode != FilletConicValueMode::RHO &&
      fillet.params.conicValueMode != FilletConicValueMode::GENERIC_VALUE) {
    *fillet.params.conicValue *= factor;
  }
  if (fillet.firstEndFaceMarker.has_value()) {
    ScalePoint(*fillet.firstEndFaceMarker, factor);
  }
  for (auto &point : fillet.params.radiusPoints) {
    if (point.primaryValue.has_value()) {
      *point.primaryValue *= factor;
    }
    if (point.secondValue.has_value()) {
      *point.secondValue *= factor;
    }
    if (point.edgeMidPoint.has_value()) {
      ScalePoint(*point.edgeMidPoint, factor);
    }
  }
  for (auto &ref : fillet.references) {
    ScaleRefEntity(ref, factor, ctx);
  }
  for (auto &face : fillet.side1Faces) {
    ScaleRefEntity(face, factor, ctx);
  }
  for (auto &face : fillet.side2Faces) {
    ScaleRefEntity(face, factor, ctx);
  }
  for (auto &face : fillet.centerFaces) {
    ScaleRefEntity(face, factor, ctx);
  }
}

void ScaleDatumPlane(CDatumPlane &datumPlane, double factor, UnitScaleContext &ctx) {
  for (auto &ref : datumPlane.referenceEntities) {
    ScaleRefEntity(ref, factor, ctx);
  }
  for (auto &constraint : datumPlane.constraints) {
    if (constraint.type == PlaneConstraintType::DISTANCE) {
      constraint.value *= factor;
    }
  }
}

void ScaleRib(CRib &rib, double factor, UnitScaleContext &ctx) {
  rib.thicknessOption.thickness *= factor;
  ScalePoint(rib.materialOption.referencePoint, factor);
}

void ScaleShell(CShell &shell, double factor, UnitScaleContext &ctx) {
  shell.thickness *= factor;
  for (auto &face : shell.facesToRemove) {
    ScaleRefEntity(face, factor, ctx);
  }
  for (auto &item : shell.thicknessFaces) {
    item.thickness *= factor;
    ScaleRefEntity(item.face, factor, ctx);
  }
  if (shell.targetBody) {
    ScaleRefEntity(shell.targetBody, factor, ctx);
  }
  for (auto &face : shell.excludedFaces) {
    ScaleRefEntity(face, factor, ctx);
  }
}

void ScaleDraft(CDraft &draft, double factor, UnitScaleContext &ctx) {
  ScaleRefEntity(draft.pullDirectionRef, factor, ctx);
  for (auto &face : draft.draftFaces) {
    ScaleRefEntity(face, factor, ctx);
  }
  ScaleRefEntity(draft.neutralPlaneRef, factor, ctx);
  for (auto &line : draft.partingLines) {
    ScaleRefEntity(line, factor, ctx);
  }
}

void ScaleLinearPatternDir(CLinearPatternDir &dir, double factor, UnitScaleContext &ctx) {
  ScaleRefEntity(dir.directionRef, factor, ctx);
  dir.spacing *= factor;
}

void ScaleLinearPattern(CLinearPattern &pattern, double factor, UnitScaleContext &ctx) {
  ScaleLinearPatternDir(pattern.dir1, factor, ctx);
  if (pattern.dir2) {
    ScaleLinearPatternDir(*pattern.dir2, factor, ctx);
  }
  for (auto &seed : pattern.seedObjects) {
    ScaleRefEntity(seed, factor, ctx);
  }
}

void ScaleCircularPatternDir(CCircularPatternDir &dir, double factor, UnitScaleContext &ctx) {
  ScaleRefEntity(dir.axisRef, factor, ctx);
}

void ScaleCircularPattern(CCircularPattern &pattern, double factor, UnitScaleContext &ctx) {
  ScaleCircularPatternDir(pattern.dir1, factor, ctx);
  if (pattern.dir2) {
    ScaleLinearPatternDir(*pattern.dir2, factor, ctx);
  }
  for (auto &seed : pattern.seedObjects) {
    ScaleRefEntity(seed, factor, ctx);
  }
}

void ScaleMirrorPattern(CMirrorPattern &pattern, double factor, UnitScaleContext &ctx) {
  ScaleRefEntity(pattern.mirrorPlaneRef, factor, ctx);
  for (auto &seed : pattern.seedObjects) {
    ScaleRefEntity(seed, factor, ctx);
  }
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
      case FeatureType::Sweep:
        ScaleSweep(*std::static_pointer_cast<CSweep>(feature), factor, ctx);
        break;
      case FeatureType::Fillet:
        ScaleFillet(*std::static_pointer_cast<CFillet>(feature), factor, ctx);
        break;
      case FeatureType::Chamfer:
        ScaleChamfer(*std::static_pointer_cast<CChamfer>(feature), factor, ctx);
        break;
      case FeatureType::DatumPlane:
        ScaleDatumPlane(*std::static_pointer_cast<CDatumPlane>(feature), factor, ctx);
        break;
      case FeatureType::Rib:
        ScaleRib(*std::static_pointer_cast<CRib>(feature), factor, ctx);
        break;
      case FeatureType::Shell:
        ScaleShell(*std::static_pointer_cast<CShell>(feature), factor, ctx);
        break;
      case FeatureType::Draft:
        ScaleDraft(*std::static_pointer_cast<CDraft>(feature), factor, ctx);
        break;
      case FeatureType::LinearPattern:
        ScaleLinearPattern(*std::static_pointer_cast<CLinearPattern>(feature), factor, ctx);
        break;
      case FeatureType::CircularPattern:
        ScaleCircularPattern(*std::static_pointer_cast<CCircularPattern>(feature), factor, ctx);
        break;
      case FeatureType::MirrorPattern:
        ScaleMirrorPattern(*std::static_pointer_cast<CMirrorPattern>(feature), factor, ctx);
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

bool TryParseUnitType(const std::string &unitStr, UnitType &out) {
  // Build a lower-case copy for case-insensitive matching.
  std::string lower;
  lower.reserve(unitStr.size());
  for (unsigned char c : unitStr) {
    lower.push_back(static_cast<char>(
        (c >= 'A' && c <= 'Z') ? (c + 32) : c));
  }
  if (lower == "m" || lower == "meter" || lower == "metre") {
    out = UnitType::METER; return true;
  }
  if (lower == "mm" || lower == "millimeter" || lower == "millimetre") {
    out = UnitType::MILLIMETER; return true;
  }
  if (lower == "cm" || lower == "centimeter" || lower == "centimetre") {
    out = UnitType::CENTIMETER; return true;
  }
  if (lower == "in" || lower == "inch") {
    out = UnitType::INCH; return true;
  }
  if (lower == "ft" || lower == "foot" || lower == "feet") {
    out = UnitType::FOOT; return true;
  }
  return false;
}

bool TryGetUnitConversionFactor(UnitType src, UnitType dst, double &factor) {
  if (src == dst) {
    factor = 1.0;
    return true;
  }
  double srcToMeter = 1.0;
  double dstToMeter = 1.0;
  if (!TryGetMeterScale(src, srcToMeter) ||
      !TryGetMeterScale(dst, dstToMeter)) {
    return false;
  }
  factor = srcToMeter / dstToMeter;
  return true;
}

} // namespace CADExchange
