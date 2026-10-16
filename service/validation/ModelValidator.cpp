// clang-format off
#include "ModelValidator.h"
#include "UnifiedFeatures.h"
#include "HoleGeometry.h"
#include "BodyGeometry.h"
#include "PatternGeometry.h"
#include <cmath>
#include <functional>
#include <set>
#include <string>
#include <type_traits>
#include <unordered_set>
// clang-format on

namespace CADExchange {

namespace {

bool IsBuiltinStandardDatumID(const std::string& id) {
  return id == StandardID::PLANE_XY ||
         id == StandardID::PLANE_YZ ||
         id == StandardID::PLANE_ZX;
}

template<class T> bool PatternFinite3(const T &value) {
  return std::isfinite(value.x)&&std::isfinite(value.y)&&std::isfinite(value.z);
}

bool PatternUnitDirection(const CVector3D &direction) {
  return PatternFinite3(direction)&&std::abs(direction.Dot(direction)-1)<=1e-6;
}

using PatternError=std::function<void(const std::string &)>;

void CheckPatternFace(const CRefFace &face,const PatternError &error) {
  if(face.refType!=RefType::TOPO_FACE||!PatternFinite3(face.centroid)||
     !PatternFinite3(face.normal)||!PatternFinite3(face.uDir)||!PatternFinite3(face.vDir))
    error("Face reference has invalid type or nonfinite geometry");
  switch(face.surfaceType) {
    case CGeoSurfaceType::PLANE:
      if(!PatternUnitDirection(face.normal)) error("Planar face needs a unit normal");
      break;
    case CGeoSurfaceType::CYLINDER:case CGeoSurfaceType::CONE:
    case CGeoSurfaceType::SPHERE:case CGeoSurfaceType::TORUS:
    case CGeoSurfaceType::BSURFACE:break;
    default:error("Unknown pattern face SurfaceType");
  }
}

void CheckPatternDirectionReference(const std::shared_ptr<CRefEntityBase> &ref,
                                    const std::unordered_set<std::string> &seen,
                                    const PatternError &error) {
  if(!ref) {error("Missing direction/axis reference");return;}
  if(auto axis=std::dynamic_pointer_cast<CRefAxis>(ref)) {
    if(axis->refType!=RefType::FEATURE_DATUM_AXIS||!PatternFinite3(axis->origin)||
       !PatternFinite3(axis->direction)) error("Invalid datum axis reference geometry");
    if(axis->targetFeatureID.empty()||
       (!StandardID::IsStandardAxis(axis->targetFeatureID)&&!seen.count(axis->targetFeatureID)))
      error("Missing or forward direction axis feature '"+axis->targetFeatureID+"'");
  } else if(auto plane=std::dynamic_pointer_cast<CRefPlane>(ref)) {
    if(plane->refType!=RefType::FEATURE_DATUM_PLANE||!PatternFinite3(plane->origin)||
       !PatternFinite3(plane->normal)||!PatternFinite3(plane->xDir)||
       !PatternFinite3(plane->yDir)) error("Invalid datum plane direction reference geometry");
    if(plane->targetFeatureID.empty()||
       (!StandardID::IsStandardPlane(plane->targetFeatureID)&&!seen.count(plane->targetFeatureID)))
      error("Missing or forward direction plane feature '"+plane->targetFeatureID+"'");
  } else if(auto edge=std::dynamic_pointer_cast<CRefEdge>(ref)) {
    if(edge->refType!=RefType::TOPO_EDGE||!PatternFinite3(edge->startPoint)||
       !PatternFinite3(edge->endPoint)||!PatternFinite3(edge->midPoint)||
       !std::isfinite(edge->length)||edge->length<0)
      error("Invalid direction edge geometry");
    switch(edge->curveType) {
      case CGeoCurveType::LINE:case CGeoCurveType::CIRCLE:
      case CGeoCurveType::ELLIPSE:case CGeoCurveType::INTERSECTION:
      case CGeoCurveType::BCURVE:case CGeoCurveType::SPCURVE:
      case CGeoCurveType::CONSTPARAM:case CGeoCurveType::TRIMMED:break;
      default:error("Unknown direction edge CurveType");
    }
  } else if(auto face=std::dynamic_pointer_cast<CRefFace>(ref)) {
    CheckPatternFace(*face,error);
  } else if(auto segment=std::dynamic_pointer_cast<CRefSketchSeg>(ref)) {
    if(segment->refType!=RefType::TOPO_SKETCH_SEG||segment->segmentLocalID.empty()||
       segment->parentFeatureID.empty()||!seen.count(segment->parentFeatureID))
      error("Missing or forward direction sketch segment");
  } else error("Unsupported direction/axis reference type");
}

template<class Pattern>
void CheckOrdinaryPattern(const Pattern &pattern,
                          const std::unordered_set<std::string> &seen,
                          const PatternError &addError) {
  const PatternError error=[&](const std::string &detail) {
    addError("[PATTERN] '"+pattern.featureID+"': "+detail);
  };
  const auto checkLinear=[&](const CLinearPatternDir &dir) {
    std::string detail;double pitch=0;
    if(!ResolveLinearPatternPitch(dir,pitch,detail)) error(detail);
    if(!PatternUnitDirection(dir.direction)) error("Linear direction must be a finite unit vector");
    CheckPatternDirectionReference(dir.directionRef,seen,error);
  };
  if constexpr(std::is_same<Pattern,CLinearPattern>::value) {
    checkLinear(pattern.dir1);
  } else {
    std::string detail;double pitch=0;
    if(!ResolveCircularPatternPitch(pattern.dir1,pitch,detail)) error(detail);
    if(!PatternUnitDirection(pattern.dir1.direction)||!PatternFinite3(pattern.dir1.origin))
      error("Circular axis needs finite origin and unit direction");
    CheckPatternDirectionReference(pattern.dir1.axisRef,seen,error);
  }
  if(pattern.dir2) checkLinear(*pattern.dir2);
  const int count2=pattern.dir2?pattern.dir2->count:1;
  const auto instanceCount=PatternInstanceCount(pattern.dir1.count,count2,pattern.patternSeedOnly);
  if(instanceCount<=1) error("Pattern must generate at least one instance beyond its seed");
  if(pattern.patternSeedOnly&&(!pattern.dir2||count2<=1))
    error("PatternSeedOnly requires an active second direction");
  std::set<std::pair<int,int>> skips;
  for(const auto &index:pattern.skippedInstances) {
    if(!IsPatternIndexInDomain(index,pattern.dir1.count,count2,pattern.patternSeedOnly)||
       (index.dir1Index==0&&index.dir2Index==0)) error("Skipped index is outside the generated instance domain");
    if(!skips.emplace(index.dir1Index,index.dir2Index).second) error("Duplicate skipped instance index");
  }
  if(instanceCount>1&&pattern.skippedInstances.size()>=instanceCount-1)
    error("Pattern must retain at least one generated instance");
  if(pattern.scope!=PatternScope::FEATURES&&pattern.scope!=PatternScope::FACES&&pattern.scope!=PatternScope::BODIES)
    error("Invalid pattern scope");
  if(pattern.seedObjects.empty()) error("Pattern seeds must not be empty");
  std::unordered_set<const CRefEntityBase*> seeds;
  std::unordered_set<std::string> featureSeeds;
  for(const auto &ref:pattern.seedObjects) {
    if(!ref) {error("Pattern seed is null");continue;}
    if(!seeds.insert(ref.get()).second) error("Duplicate pattern seed reference");
    if(pattern.scope==PatternScope::BODIES) {
      auto body=std::dynamic_pointer_cast<CRefBody>(ref);std::string detail;
      if(!body) error("BODIES requires Body references");
      else if(!ValidateBodyReference(*body,detail)) error(detail);
    } else if(pattern.scope==PatternScope::FEATURES) {
      const auto seed=std::dynamic_pointer_cast<CRefFeature>(ref);
      if(!seed||typeid(*ref)!=typeid(CRefFeature)||ref->refType!=RefType::UNKNOWN)
        error("FEATURES requires whole-feature references");
      else {
        if(seed->targetFeatureID.empty()||!seen.count(seed->targetFeatureID))
          error("Missing or forward seed feature '"+seed->targetFeatureID+"'");
        if(!featureSeeds.insert(seed->targetFeatureID).second) error("Duplicate seed feature ID");
      }
    } else if(pattern.scope==PatternScope::FACES) {
      const auto face=std::dynamic_pointer_cast<CRefFace>(ref);
      if(!face) error("FACES requires face references");else CheckPatternFace(*face,error);
    }
  }
  if(pattern.scope==PatternScope::BODIES&&pattern.geometryPattern)
    error("GeometryPattern is inactive for body arrays");
  const bool affectedActive=pattern.scope==PatternScope::FEATURES&&pattern.geometryPattern;
  if(affectedActive) {
    if(!pattern.affectedBodyMode) error("Geometry feature pattern requires AffectedBodyMode");
    else switch(*pattern.affectedBodyMode) {
      case PatternAffectedBodyMode::AllBodies:case PatternAffectedBodyMode::AutoSelect:
        if(!pattern.affectedBodies.empty()) error("AllBodies/AutoSelect forbids a fixed affected Body list");
        break;
      case PatternAffectedBodyMode::SelectedBodies:
        if(pattern.affectedBodies.empty()) error("SelectedBodies requires affected Body references");
        break;
      default:error("Invalid AffectedBodyMode");
    }
  } else if(pattern.affectedBodyMode||!pattern.affectedBodies.empty())
    error("Affected Body options are only active for FEATURES geometry patterns");
  std::unordered_set<const CRefBody*> affected;
  for(const auto &body:pattern.affectedBodies) {
    std::string detail;
    if(!body) error("Affected Body reference is null");
    else {
      if(!ValidateBodyReference(*body,detail)) error(detail);
      if(!affected.insert(body.get()).second) error("Duplicate affected Body reference");
    }
  }
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
    if (auto sw = std::dynamic_pointer_cast<CSweep>(f)) {
      const std::string sketchID = !sw->profile.sketchID.empty()
                                       ? sw->profile.sketchID
                                       : sw->profileSketchID;
      if (sw->profile.kind == SweepProfileKind::SketchReference &&
          !sketchID.empty()) {
        referencedSketchIDs.insert(sketchID);
      }
    }
    if (auto rib = std::dynamic_pointer_cast<CRib>(f))
      if (!rib->sketchID.empty())
        referencedSketchIDs.insert(rib->sketchID);
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

  auto checkPositiveDistance = [&](const std::optional<double> &value,
                                   const std::string &featureID,
                                   const char *label) {
    if (!value.has_value()) {
      addError(std::string("[CHAMFER_003] Chamfer '") + featureID +
               "' missing required parameter " + label + ".");
      return;
    }
    if (*value <= 0.0) {
      addError(std::string("[CHAMFER_004] Chamfer '") + featureID + "' " +
               label + "=" + std::to_string(*value) + " (must be > 0).");
      return;
    }
    const double valueM = toMeter(*value);
    if (valueM < 1e-6 || valueM > 100.0) {
      addWarn(std::string("[SCALE_002] Chamfer '") + featureID + "' " + label +
              "=" + std::to_string(*value) + " (~" +
              std::to_string(valueM * 1000.0) +
              "mm) is out of normal range -- check unit system.");
    }
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

  auto checkSweepPathReference =
      [&](const std::shared_ptr<CRefEntityBase> &ref,
          const std::string &featureID,
          const std::string &role,
          size_t index,
          const std::unordered_set<std::string> &seen) {
        const std::string idx = std::to_string(index);
        if (!ref) {
          addError("[SWEEP_003] Sweep '" + featureID + "' " + role +
                   " reference[" + idx + "] is null.");
          return;
        }
        if (auto sketch = std::dynamic_pointer_cast<CRefSketch>(ref)) {
          if (seen.find(sketch->targetFeatureID) == seen.end()) {
            addError("[REF_001] Sweep '" + featureID + "' " + role +
                     " reference[" + idx + "] sketch '" +
                     sketch->targetFeatureID +
                     "' has not been defined yet.");
          }
          return;
        }
        if (auto sketchSeg = std::dynamic_pointer_cast<CRefSketchSeg>(ref)) {
          if (sketchSeg->parentFeatureID.empty() ||
              sketchSeg->segmentLocalID.empty()) {
            addError("[SWEEP_004] Sweep '" + featureID + "' " + role +
                     " reference[" + idx +
                     "] sketch segment requires parentFeatureID and segmentLocalID.");
          } else if (seen.find(sketchSeg->parentFeatureID) == seen.end()) {
            addError("[REF_001] Sweep '" + featureID + "' " + role +
                     " reference[" + idx + "] sketch '" +
                     sketchSeg->parentFeatureID +
                     "' has not been defined yet.");
          }
          return;
        }
        if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(ref)) {
          if (!subTopo->parentFeatureID.empty() &&
              seen.find(subTopo->parentFeatureID) == seen.end()) {
            addWarn("[REF_002] Sweep '" + featureID + "' " + role +
                    " reference[" + idx + "] parent feature '" +
                    subTopo->parentFeatureID +
                    "' has not been defined yet.");
          }
        }
      };

  auto isAllowedRibTargetBodyRef =
      [&](const std::shared_ptr<CRefEntityBase> &ref) -> bool {
        if (!ref) {
          return false;
        }
        return std::dynamic_pointer_cast<CRefFeature>(ref) != nullptr ||
               std::dynamic_pointer_cast<CRefFace>(ref) != nullptr;
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

    if(auto linear=std::dynamic_pointer_cast<CLinearPattern>(feature))
      CheckOrdinaryPattern(*linear,seen,addError);
    if(auto circular=std::dynamic_pointer_cast<CCircularPattern>(feature))
      CheckOrdinaryPattern(*circular,seen,addError);

    if (auto mirror = std::dynamic_pointer_cast<CMirrorPattern>(feature)) {
      const std::string prefix = "[MIRROR] '" + mirror->featureID + "': ";
      const auto error = [&](const std::string &message) { addError(prefix + message); };
      const auto checkBody = [&](const std::shared_ptr<CRefBody> &body) {
        std::string detail;
        if (!body) error("Body reference is null");
        else if (!ValidateBodyReference(*body, detail)) error(detail);
      };
      const auto checkDependency = [&](const std::string &id) {
        if (id.empty() || (!StandardID::IsStandardPlane(id) && !seen.count(id)))
          error("Missing or forward feature reference '" + id + "'");
      };
      if (auto plane = std::dynamic_pointer_cast<CRefPlane>(mirror->mirrorPlaneRef)) {
        if (plane->refType != RefType::FEATURE_DATUM_PLANE) error("Wrong plane RefType");
        const auto &origin = plane->origin; const auto &normal = plane->normal;
        if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z) ||
            !std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z))
          error("Invalid mirror datum plane geometry");
        checkDependency(plane->targetFeatureID);
      } else if (auto face = std::dynamic_pointer_cast<CRefFace>(mirror->mirrorPlaneRef)) {
        if (face->refType != RefType::TOPO_FACE || face->surfaceType != CGeoSurfaceType::PLANE)
          error("Mirror face must be planar");
        if (!std::isfinite(face->normal.x) || !std::isfinite(face->normal.y) ||
            !std::isfinite(face->normal.z) || isZeroVec(face->normal) ||
            !std::isfinite(face->centroid.x) || !std::isfinite(face->centroid.y) ||
            !std::isfinite(face->centroid.z)) error("Invalid mirror face geometry");
      } else error("Mirror plane must be a datum plane or planar face");
      if (mirror->scope != PatternScope::FEATURES && mirror->scope != PatternScope::FACES &&
          mirror->scope != PatternScope::BODIES) error("Invalid mirror seed scope");
      if (mirror->seedObjects.empty()) error("Mirror seeds must not be empty");
      std::unordered_set<const CRefEntityBase *> seeds;
      std::unordered_set<std::string> seedIDs;
      for (const auto &ref : mirror->seedObjects) {
        if (!ref) { error("Mirror seed is null"); continue; }
        if (!seeds.insert(ref.get()).second) error("Duplicate mirror seed reference");
        if (mirror->scope == PatternScope::BODIES) {
          auto body = std::dynamic_pointer_cast<CRefBody>(ref);
          if (!body) error("BODIES scope requires Body references"); else checkBody(body);
        } else if (mirror->scope == PatternScope::FACES) {
          const auto face = std::dynamic_pointer_cast<CRefFace>(ref);
          if (!face || ref->refType != RefType::TOPO_FACE) error("FACES scope requires face references");
          else {
            const auto &center = face->centroid; const auto &normal = face->normal;
            if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z) ||
                !std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z))
              error("Invalid mirror seed face geometry");
            if (face->surfaceType == CGeoSurfaceType::PLANE &&
                normal.x == 0 && normal.y == 0 && normal.z == 0)
              error("Mirror planar seed face Normal must be nonzero");
            switch (face->surfaceType) {
              case CGeoSurfaceType::PLANE: case CGeoSurfaceType::CYLINDER:
              case CGeoSurfaceType::CONE: case CGeoSurfaceType::SPHERE:
              case CGeoSurfaceType::TORUS: case CGeoSurfaceType::BSURFACE: break;
              default: error("Invalid mirror seed face SurfaceType");
            }
          }
        } else if (mirror->scope == PatternScope::FEATURES) {
          auto seed = std::dynamic_pointer_cast<CRefFeature>(ref);
          if (!seed || typeid(*ref) != typeid(CRefFeature) || ref->refType != RefType::UNKNOWN)
            error("FEATURES scope requires whole-feature references");
          else {
            checkDependency(seed->targetFeatureID);
            if (!seedIDs.insert(seed->targetFeatureID).second) error("Duplicate seed feature ID");
          }
        }
      }
      if (mirror->scope == PatternScope::BODIES) {
        if (!mirror->mergeResults.has_value()) error("Body mirror requires MergeResults");
        if (mirror->geometryPattern) error("GeometryPattern is inactive for body mirrors");
      } else if (mirror->mergeResults.has_value()) error("MergeResults is only valid for body mirrors");
      const bool affectedActive = mirror->scope != PatternScope::BODIES && mirror->geometryPattern;
      if (affectedActive) {
        if (!mirror->affectedBodyMode) error("Geometry mirror requires AffectedBodyMode");
        else if (*mirror->affectedBodyMode == MirrorAffectedBodyMode::AllBodies) {
          if (!mirror->affectedBodies.empty()) error("AllBodies forbids an explicit affected-body list");
        } else if (*mirror->affectedBodyMode == MirrorAffectedBodyMode::SelectedBodiesWithAutoSelect ||
                   *mirror->affectedBodyMode == MirrorAffectedBodyMode::SelectedBodiesOnly) {
          if (mirror->affectedBodies.empty()) error("Selected-body scope needs affected Bodies");
        } else error("Invalid AffectedBodyMode");
      } else if (mirror->affectedBodyMode || !mirror->affectedBodies.empty())
        error("Affected body options are only valid for feature/face geometry mirrors");
      std::unordered_set<const CRefBody *> affected;
      for (const auto &body : mirror->affectedBodies) {
        checkBody(body);
        if (body && !affected.insert(body.get()).second) error("Duplicate affected Body reference");
      }
    }

    if (auto hole = std::dynamic_pointer_cast<CHole>(feature)) {
      const auto prefix = "[HOLE] '" + hole->featureID + "': ";
      HoleResolvedGeometry geometry; std::string detail;
      if (!ResolveHoleGeometry(*hole,geometry,detail)) addError(prefix+detail);
      const auto finite3 = [](const auto& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
      };
      if (!finite3(hole->direction) || std::abs(vecLen(hole->direction)-1) > 1e-6)
        addError(prefix+"direction must be a finite inward unit vector");
      if (hole->locations.size()!=1) addError(prefix+"v1 requires exactly one location");
      CPoint3D origin{}; CVector3D normal{}; bool planar=false;
      if (auto plane=std::dynamic_pointer_cast<CRefPlane>(hole->supportRef)) {
        origin=plane->origin; normal=plane->normal;planar=true;
        // ID-only datum references are an existing shared-model contract.
        // Resolve missing geometry without relaxing dependency-order validation.
        if (finite3(origin) && finite3(normal) && vecLen(normal)<GeoUtils::EPSILON) {
          if (StandardID::IsStandardPlane(plane->targetFeatureID)) {
            origin={};
            if(plane->targetFeatureID==StandardID::PLANE_XY) normal={0,0,1};
            else if(plane->targetFeatureID==StandardID::PLANE_YZ) normal={1,0,0};
            else if(plane->targetFeatureID==StandardID::PLANE_ZX) normal={0,1,0};
          } else if(auto datum=model.GetFeatureAs<CDatumPlane>(plane->targetFeatureID)) {
            if(datum->projectedOrigin && datum->normal) {
              origin=*datum->projectedOrigin;normal=*datum->normal;
            }
          }
        }
      } else if (auto face=std::dynamic_pointer_cast<CRefFace>(hole->supportRef)) {
        origin=face->centroid;normal=face->normal;planar=face->surfaceType==CGeoSurfaceType::PLANE;
      }
      const double nLength=vecLen(normal);
      if (!planar || !finite3(origin) || !finite3(normal) || nLength < GeoUtils::EPSILON)
        addError(prefix+"support must resolve to a finite plane");
      else {
        const double dot=hole->direction.x*normal.x+hole->direction.y*normal.y+hole->direction.z*normal.z;
        if (!std::isfinite(dot) || std::abs(std::abs(dot/nLength)-1)>1e-6)
          addError(prefix+"hole axis must be normal to the entry plane");
        for (const auto& location:hole->locations) {
          const auto& p=location.entryPoint;
          const double distance=((p.x-origin.x)*normal.x+(p.y-origin.y)*normal.y+(p.z-origin.z)*normal.z)/nLength;
          if (!finite3(p) || !std::isfinite(distance) || std::abs(toMeter(distance))>1e-7)
            addError(prefix+"entry point must be finite and lie on the entry plane");
        }
      }
      auto checkHoleRef = [&](const std::shared_ptr<CRefEntityBase>& ref) {
        if (!ref) return;
        std::string id;
        if (auto r=std::dynamic_pointer_cast<CRefFeature>(ref)) id=r->targetFeatureID;
        if (auto r=std::dynamic_pointer_cast<CRefSubTopo>(ref)) id=r->parentFeatureID;
        if (!id.empty() && !StandardID::IsStandardPlane(id) && !StandardID::IsStandardAxis(id) &&
            !StandardID::IsStandardPoint(id) && !seen.count(id))
          addError(prefix+"missing or forward reference '"+id+"'");
        if (ref->refType==RefType::UNKNOWN) addError(prefix+"unknown reference type");
      };
      checkHoleRef(hole->supportRef);
      for(const auto& location:hole->locations) checkHoleRef(location.positionRef);
    }

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
                "' is referenced by a profiled feature but has no segments.");
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

      std::unordered_set<std::string> segmentIDs;
      for (const auto &seg : sketch->segments) {
        if (seg && !seg->localID.empty()) {
          segmentIDs.insert(seg->localID);
        }
      }

      for (size_t i = 0; i < sketch->constraints.size(); ++i) {
        const auto &constraint = sketch->constraints[i];
        const std::string idx = std::to_string(i);

        if (constraint.type == CSketchConstraint::ConstraintType::UNKNOWN) {
          addError("[SKETCH_004] Sketch '" + sketch->featureID +
                   "' constraint[" + idx + "] type is UNKNOWN.");
        }

        if (constraint.refs.empty()) {
          addError("[SKETCH_003] Sketch '" + sketch->featureID +
                   "' constraint[" + idx + "] has no refs.");
        }

        for (size_t refIndex = 0; refIndex < constraint.refs.size(); ++refIndex) {
          const auto &ref = constraint.refs[refIndex];
          const std::string refIdx = std::to_string(refIndex);
          if (ref.kind == SketchConstraintRefKind::SketchEntity) {
            if (ref.sketchEntityLocalID.empty() ||
                segmentIDs.find(ref.sketchEntityLocalID) == segmentIDs.end()) {
              addError("[SKETCH_002] Sketch '" + sketch->featureID +
                       "' constraint[" + idx + "] ref[" + refIdx +
                       "] references missing sketch entity localID '" +
                       ref.sketchEntityLocalID + "'.");
            }
          } else if (!ref.refEntity) {
            addError("[SKETCH_005] Sketch '" + sketch->featureID +
                     "' constraint[" + idx + "] ref[" + refIdx +
                     "] external reference is null.");
          } else if (auto subTopo =
                         std::dynamic_pointer_cast<CRefSubTopo>(ref.refEntity)) {
            if (!subTopo->parentFeatureID.empty() &&
                !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
                seen.find(subTopo->parentFeatureID) == seen.end()) {
              addWarn("[REF_005] Sketch '" + sketch->featureID +
                      "' constraint[" + idx + "] ref[" + refIdx +
                      "] parent feature '" + subTopo->parentFeatureID +
                      "' has not been defined yet.");
            }
          }
        }

        const bool requiresValue =
            constraint.type == CSketchConstraint::ConstraintType::DISTANCE ||
            constraint.type == CSketchConstraint::ConstraintType::ANGLE ||
            constraint.type == CSketchConstraint::ConstraintType::RADIUS ||
            constraint.type == CSketchConstraint::ConstraintType::DIAMETER;
        if (requiresValue && !constraint.value.has_value()) {
          addError("[SKETCH_006] Sketch '" + sketch->featureID +
                   "' constraint[" + idx + "] requires numeric value.");
        }
        if (!requiresValue && constraint.value.has_value()) {
          addWarn("[SKETCH_007] Sketch '" + sketch->featureID +
                  "' constraint[" + idx +
                  "] stores numeric value but type is non-dimensional.");
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
    // ---- CSweep ----
    else if (auto sweep = std::dynamic_pointer_cast<CSweep>(feature)) {
      if (sweep->profile.kind == SweepProfileKind::SketchReference) {
        const std::string sketchID = !sweep->profile.sketchID.empty()
                                         ? sweep->profile.sketchID
                                         : sweep->profileSketchID;
        if (sketchID.empty()) {
          addError("[SWEEP_001] Sweep '" + sweep->featureID +
                   "' has empty sketch profile reference.");
        } else if (seen.find(sketchID) == seen.end()) {
          addError("[REF_001] Sweep '" + sweep->featureID +
                   "' references sketch '" + sketchID +
                   "' which has not been defined yet.");
        }
      } else if (sweep->profile.kind == SweepProfileKind::EmbeddedSketch) {
        if (!sweep->profile.embedded.has_value()) {
          addError("[SWEEP_007] Sweep '" + sweep->featureID +
                   "' uses EmbeddedSketch profile but has no embedded sketch.");
        } else if (sweep->profile.embedded->sketch.segments.empty()) {
          addError("[SWEEP_008] Sweep '" + sweep->featureID +
                   "' embedded profile sketch has no segments.");
        }
      } else if (sweep->profile.kind == SweepProfileKind::Circular) {
        if (!sweep->profile.circular.has_value()) {
          addError("[SWEEP_009] Sweep '" + sweep->featureID +
                   "' uses Circular profile but has no circular parameters.");
        } else {
          const auto &circular = *sweep->profile.circular;
          if (circular.outerRadius <= 0.0) {
            addError("[SWEEP_010] Sweep '" + sweep->featureID +
                     "' circular outer radius must be positive.");
          }
          if (circular.innerRadius < 0.0 ||
              circular.innerRadius >= circular.outerRadius) {
            addError("[SWEEP_011] Sweep '" + sweep->featureID +
                     "' circular inner radius must be non-negative and less "
                     "than outer radius.");
          }
        }
      }

      if (sweep->path.references.empty()) {
        addError("[SWEEP_002] Sweep '" + sweep->featureID +
                 "' has no path references.");
      }
      if (sweep->profilePathAngleCos &&
          (*sweep->profilePathAngleCos < -1.0 ||
           *sweep->profilePathAngleCos > 1.0)) {
        addError("[SWEEP_012] Sweep '" + sweep->featureID +
                 "' profilePathAngleCos must be within [-1, 1].");
      }

      for (size_t i = 0; i < sweep->path.references.size(); ++i) {
        checkSweepPathReference(sweep->path.references[i], sweep->featureID,
                                "path", i, seen);
      }

      for (size_t guideIndex = 0; guideIndex < sweep->guidePaths.size();
           ++guideIndex) {
        const auto &guidePath = sweep->guidePaths[guideIndex];
        if (guidePath.references.empty()) {
          addError("[SWEEP_005] Sweep '" + sweep->featureID +
                   "' guidePath[" + std::to_string(guideIndex) +
                   "] has no references.");
        }
        const std::string role =
            "guidePath[" + std::to_string(guideIndex) + "]";
        for (size_t i = 0; i < guidePath.references.size(); ++i) {
          checkSweepPathReference(guidePath.references[i], sweep->featureID,
                                  role, i, seen);
        }
      }

      if (sweep->thinWall.has_value() &&
          std::fabs(sweep->thinWall->startOffset) <= 1e-9 &&
          std::fabs(sweep->thinWall->endOffset) <= 1e-9) {
        addError("[SWEEP_006] Sweep '" + sweep->featureID +
                 "' has ThinWall but StartOffset/EndOffset are both zero.");
      }
    }
    // ---- CChamfer ----
    else if (auto chamfer = std::dynamic_pointer_cast<CChamfer>(feature)) {
      if (chamfer->mode == ChamferMode::UNKNOWN) {
        addError("[CHAMFER_001] Chamfer '" + chamfer->featureID +
                 "' mode is UNKNOWN.");
      }
      if (chamfer->references.empty()) {
        addError("[CHAMFER_002] Chamfer '" + chamfer->featureID +
                 "' has no references.");
      }

      switch (chamfer->mode) {
      case ChamferMode::EQUAL_DISTANCE:
        checkPositiveDistance(chamfer->params.distance1, chamfer->featureID,
                              "distance1");
        break;
      case ChamferMode::TWO_DISTANCES:
        checkPositiveDistance(chamfer->params.distance1, chamfer->featureID,
                              "distance1");
        checkPositiveDistance(chamfer->params.distance2, chamfer->featureID,
                              "distance2");
        break;
      case ChamferMode::TWO_OFFSETS:
        checkPositiveDistance(chamfer->params.offset1, chamfer->featureID,
                              "offset1");
        checkPositiveDistance(chamfer->params.offset2, chamfer->featureID,
                              "offset2");
        break;
      case ChamferMode::DISTANCE_ANGLE:
        checkPositiveDistance(chamfer->params.distance1, chamfer->featureID,
                              "distance1");
        if (!chamfer->params.angle.has_value()) {
          addError("[CHAMFER_003] Chamfer '" + chamfer->featureID +
                   "' missing required parameter angle.");
        } else if (std::abs(*chamfer->params.angle) < GeoUtils::EPSILON) {
          addWarn("[CHAMFER_005] Chamfer '" + chamfer->featureID +
                  "' angle is near zero.");
        }
        break;
      case ChamferMode::VERTEX_3DISTANCES:
        checkPositiveDistance(chamfer->params.distance1, chamfer->featureID,
                              "distance1");
        checkPositiveDistance(chamfer->params.distance2, chamfer->featureID,
                              "distance2");
        checkPositiveDistance(chamfer->params.distance3, chamfer->featureID,
                              "distance3");
        break;
      case ChamferMode::UNKNOWN:
        break;
      }

      for (size_t i = 0; i < chamfer->references.size(); ++i) {
        const auto &ref = chamfer->references[i];
        if (!ref) {
          addError("[CHAMFER_006] Chamfer '" + chamfer->featureID +
                   "' reference[" + std::to_string(i) + "] is null.");
          continue;
        }
        if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(ref)) {
          if (!subTopo->parentFeatureID.empty() &&
              !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
              seen.find(subTopo->parentFeatureID) == seen.end()) {
            addWarn("[REF_006] Chamfer '" + chamfer->featureID +
                    "' reference[" + std::to_string(i) +
                    "] parent feature '" + subTopo->parentFeatureID +
                    "' has not been defined yet.");
          }
        }
        if (auto edge = std::dynamic_pointer_cast<CRefEdge>(ref)) {
          const bool hasGeometry =
              std::fabs(edge->startPoint.x - edge->endPoint.x) > GeoUtils::EPSILON ||
              std::fabs(edge->startPoint.y - edge->endPoint.y) > GeoUtils::EPSILON ||
              std::fabs(edge->startPoint.z - edge->endPoint.z) > GeoUtils::EPSILON ||
              std::fabs(edge->startPoint.x - edge->midPoint.x) > GeoUtils::EPSILON ||
              std::fabs(edge->startPoint.y - edge->midPoint.y) > GeoUtils::EPSILON ||
              std::fabs(edge->startPoint.z - edge->midPoint.z) > GeoUtils::EPSILON;
          if (hasGeometry && edge->curveType == CGeoCurveType::UNKNOWN) {
            addWarn("[REF_007] Chamfer '" + chamfer->featureID +
                    "' reference[" + std::to_string(i) +
                    "] is an edge with geometry fingerprint but curveType is UNKNOWN.");
          }
        }
      }
    }
    // ---- CRib ----
    // ---- CRib ----
    else if (auto rib = std::dynamic_pointer_cast<CRib>(feature)) {
      if (rib->sketchID.empty()) {
        addError("[RIB_001] Rib '" + rib->featureID +
                 "' has empty sketchID.");
      } else if (seen.find(rib->sketchID) == seen.end()) {
        addError("[RIB_002] Rib '" + rib->featureID +
                 "' references sketch '" + rib->sketchID +
                 "' which has not been defined yet.");
      }

      if (rib->thicknessOption.thickness <= 0.0) {
        addError("[RIB_003] Rib '" + rib->featureID +
                 "' thickness=" + std::to_string(rib->thicknessOption.thickness) +
                 " (must be > 0).");
      } else {
        const double thicknessM = toMeter(rib->thicknessOption.thickness);
        if (thicknessM < 1e-6 || thicknessM > 100.0) {
          addWarn("[SCALE_003] Rib '" + rib->featureID + "' thickness=" +
                  std::to_string(rib->thicknessOption.thickness) + " (~" +
                  std::to_string(thicknessM * 1000.0) +
                  "mm) is out of normal range -- check unit system.");
        }
      }

      if (!rib->thicknessOption.symmetric) {
        if (!rib->thicknessOption.direction.has_value()) {
          addError("[RIB_004] Rib '" + rib->featureID +
                   "' is asymmetric but has no thickness direction.");
        } else if (isZeroVec(*rib->thicknessOption.direction)) {
          addError("[RIB_004] Rib '" + rib->featureID +
                   "' thickness direction is zero vector.");
        }
      }

      if (isZeroVec(rib->materialOption.direction)) {
        addError("[RIB_005] Rib '" + rib->featureID +
                 "' material direction is zero vector.");
      }
    }
    // ---- CShell ----
    else if (auto shell = std::dynamic_pointer_cast<CShell>(feature)) {
      // SHELL_001: thickness must be positive
      if (shell->thickness <= 0.0) {
        addError("[SHELL_001] Shell '" + shell->featureID +
                 "' thickness=" + std::to_string(shell->thickness) +
                 " (must be > 0).");
      } else {
        const double thicknessM = toMeter(shell->thickness);
        if (thicknessM < 1e-6 || thicknessM > 100.0) {
          addWarn("[SCALE_004] Shell '" + shell->featureID + "' thickness=" +
                  std::to_string(shell->thickness) + " (~" +
                  std::to_string(thicknessM * 1000.0) +
                  "mm) is out of normal range -- check unit system.");
        }
      }

      // SHELL_002: direction must be specified
      if (shell->direction == ShellThicknessDirection::Unknown) {
        addError("[SHELL_002] Shell '" + shell->featureID +
                 "' direction is Unknown.");
      }

      // SHELL_003: facesToRemove is typically non-empty for a meaningful shell
      if (shell->facesToRemove.empty() && shell->thicknessFaces.empty()) {
        addWarn("[SHELL_003] Shell '" + shell->featureID +
                "' has no facesToRemove and no thicknessFaces -- "
                "shell may be a no-op.");
      }

      // Validate references in facesToRemove
      for (size_t i = 0; i < shell->facesToRemove.size(); ++i) {
        const auto &ref = shell->facesToRemove[i];
        if (!ref) {
          addError("[SHELL_004] Shell '" + shell->featureID +
                   "' facesToRemove[" + std::to_string(i) + "] is null.");
        } else if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(ref)) {
          if (!subTopo->parentFeatureID.empty() &&
              !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
              seen.find(subTopo->parentFeatureID) == seen.end()) {
            addWarn("[REF_008] Shell '" + shell->featureID +
                    "' facesToRemove[" + std::to_string(i) +
                    "] parent feature '" + subTopo->parentFeatureID +
                    "' has not been defined yet.");
          }
        }
      }

      // Validate references in thicknessFaces
      for (size_t i = 0; i < shell->thicknessFaces.size(); ++i) {
        const auto &item = shell->thicknessFaces[i];
        if (!item.face) {
          addError("[SHELL_005] Shell '" + shell->featureID +
                   "' thicknessFaces[" + std::to_string(i) + "] face is null.");
        } else {
          if (item.thickness <= 0.0) {
            addError("[SHELL_006] Shell '" + shell->featureID +
                     "' thicknessFaces[" + std::to_string(i) +
                     "].thickness=" + std::to_string(item.thickness) +
                     " (must be > 0).");
          }
          if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(item.face)) {
            if (!subTopo->parentFeatureID.empty() &&
                !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
                seen.find(subTopo->parentFeatureID) == seen.end()) {
              addWarn("[REF_009] Shell '" + shell->featureID +
                      "' thicknessFaces[" + std::to_string(i) +
                      "] parent feature '" + subTopo->parentFeatureID +
                      "' has not been defined yet.");
            }
          }
        }
      }

      // Validate references in excludedFaces (optional, Creo-specific)
      for (size_t i = 0; i < shell->excludedFaces.size(); ++i) {
        const auto &ref = shell->excludedFaces[i];
        if (!ref) {
          addError("[SHELL_007] Shell '" + shell->featureID +
                   "' excludedFaces[" + std::to_string(i) + "] is null.");
        } else if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(ref)) {
          if (!subTopo->parentFeatureID.empty() &&
              !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
              seen.find(subTopo->parentFeatureID) == seen.end()) {
            addWarn("[REF_010] Shell '" + shell->featureID +
                    "' excludedFaces[" + std::to_string(i) +
                    "] parent feature '" + subTopo->parentFeatureID +
                    "' has not been defined yet.");
          }
        }
      }
    }
    // ---- CDraft ----
    else if (auto draft = std::dynamic_pointer_cast<CDraft>(feature)) {
      if (draft->draftType == DraftType::Unknown) {
        addError("[DRAFT_001] Draft '" + draft->featureID + "' draftType is Unknown.");
      }

      if (!draft->pullDirectionRef) {
        addError("[DRAFT_002] Draft '" + draft->featureID + "' pullDirectionRef is null.");
      } else if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(draft->pullDirectionRef)) {
        if (!subTopo->parentFeatureID.empty() &&
            !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
            seen.find(subTopo->parentFeatureID) == seen.end()) {
          addWarn("[REF_002] Draft '" + draft->featureID + "' references pullDirectionRef parentFeatureID '" +
                  subTopo->parentFeatureID + "' which is not defined yet.");
        }
      }

      if (draft->draftType == DraftType::NeutralPlane && draft->draftFaces.empty()) {
        addError("[DRAFT_003] Draft '" + draft->featureID + "' has no draftFaces.");
      } else {
        for (size_t i = 0; i < draft->draftFaces.size(); ++i) {
          const auto &face = draft->draftFaces[i];
          if (!face) {
            addError("[DRAFT_004] Draft '" + draft->featureID + "' draftFaces[" + std::to_string(i) + "] is null.");
          } else if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(face)) {
            if (!subTopo->parentFeatureID.empty() &&
                !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
                seen.find(subTopo->parentFeatureID) == seen.end()) {
              addWarn("[REF_002] Draft '" + draft->featureID + "' references draftFace parentFeatureID '" +
                      subTopo->parentFeatureID + "' which is not defined yet.");
            }
          }
        }
      }

      if (draft->draftType == DraftType::NeutralPlane) {
        if (!draft->neutralPlaneRef) {
          addError("[DRAFT_005] Draft '" + draft->featureID + "' of type NeutralPlane missing neutralPlaneRef.");
        } else if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(draft->neutralPlaneRef)) {
          if (!subTopo->parentFeatureID.empty() &&
              !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
              seen.find(subTopo->parentFeatureID) == seen.end()) {
            addWarn("[REF_002] Draft '" + draft->featureID + "' references neutralPlaneRef parentFeatureID '" +
                    subTopo->parentFeatureID + "' which is not defined yet.");
          }
        }
      } else if (draft->draftType == DraftType::PartingLine) {
        if (draft->partingLines.empty()) {
          addError("[DRAFT_006] Draft '" + draft->featureID + "' of type PartingLine has no partingLines.");
        } else {
          for (size_t i = 0; i < draft->partingLines.size(); ++i) {
            const auto &line = draft->partingLines[i];
            if (!line) {
              addError("[DRAFT_007] Draft '" + draft->featureID + "' partingLines[" + std::to_string(i) + "] is null.");
            } else if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(line)) {
              if (!subTopo->parentFeatureID.empty() &&
                  !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
                  seen.find(subTopo->parentFeatureID) == seen.end()) {
                addWarn("[REF_002] Draft '" + draft->featureID + "' references partingLine parentFeatureID '" +
                        subTopo->parentFeatureID + "' which is not defined yet.");
              }
            }
          }
        }
      }

      if (draft->draftAngle <= 0.0) {
        addError("[DRAFT_008] Draft '" + draft->featureID + "' draftAngle=" + std::to_string(draft->draftAngle) + " (must be > 0).");
      } else if (draft->draftAngle > 1.0) {
        addWarn("[SCALE_005] Draft '" + draft->featureID + "' draftAngle=" + std::to_string(draft->draftAngle) + " is very large.");
      }

      if (draft->isTwoSided) {
        if (draft->draftAngleSide2 <= 0.0) {
          addError("[DRAFT_009] Draft '" + draft->featureID + "' draftAngleSide2=" + std::to_string(draft->draftAngleSide2) + " (must be > 0 when isTwoSided is true).");
        } else if (draft->draftAngleSide2 > 1.0) {
          addWarn("[SCALE_005] Draft '" + draft->featureID + "' draftAngleSide2=" + std::to_string(draft->draftAngleSide2) + " is very large.");
        }
      }
    }
    // ---- CFillet ----
    else if (auto fillet = std::dynamic_pointer_cast<CFillet>(feature)) {
      if (fillet->mode == FilletMode::UNKNOWN) {
        addError("[FILLET_001] Fillet '" + fillet->featureID +
                 "' mode is UNKNOWN.");
      }
      if (fillet->referenceMode == FilletReferenceMode::UNKNOWN) {
        addError("[FILLET_002] Fillet '" + fillet->featureID +
                 "' referenceMode is UNKNOWN.");
      }

      switch (fillet->referenceMode) {
        case FilletReferenceMode::EDGE_CHAIN:
          if (fillet->references.empty()) {
            addError("[FILLET_003] Fillet '" + fillet->featureID +
                     "' has no edge references.");
          }
          break;
        case FilletReferenceMode::FACE_FACE:
          if (fillet->side1Faces.empty() || fillet->side2Faces.empty()) {
            addError("[FILLET_004] Fillet '" + fillet->featureID +
                     "' face-face mode requires side1Faces and side2Faces.");
          }
          break;
        case FilletReferenceMode::FULL_ROUND_THREE_FACES:
          if (fillet->side1Faces.empty() || fillet->side2Faces.empty() ||
              fillet->centerFaces.empty()) {
            addError("[FILLET_005] Fillet '" + fillet->featureID +
                     "' full-round mode requires side1Faces, centerFaces, and side2Faces.");
          }
          break;
        case FilletReferenceMode::UNKNOWN:
          break;
      }

      if (fillet->mode == FilletMode::CONSTANT_RADIUS) {
        const auto &primaryValue = fillet->params.primaryValue;
        if (primaryValue.has_value()) {
          if (*primaryValue <= 0.0) {
            addError("[FILLET_007] Fillet '" + fillet->featureID +
                     "' primaryValue=" + std::to_string(*primaryValue) +
                     " (must be > 0).");
          }
        } else if (fillet->params.radiusPoints.empty()) {
          addError("[FILLET_006] Fillet '" + fillet->featureID +
                   "' missing required primaryValue.");
        }
      }

      if (fillet->mode == FilletMode::CHORDAL) {
        const auto &primaryValue = fillet->params.primaryValue;
        if (primaryValue.has_value()) {
          if (*primaryValue <= 0.0) {
            addError("[FILLET_007] Fillet '" + fillet->featureID +
                     "' primaryValue=" + std::to_string(*primaryValue) +
                     " (must be > 0).");
          }
        } else if (fillet->params.radiusPoints.empty()) {
          addError("[FILLET_006] Fillet '" + fillet->featureID +
                   "' missing required primaryValue.");
        }
      }

      if (fillet->mode == FilletMode::VARIABLE_RADIUS &&
          fillet->params.radiusPoints.empty()) {
        addError("[FILLET_008] Fillet '" + fillet->featureID +
                 "' variable-radius mode requires radiusPoints.");
      }

      const auto &secondValue = fillet->params.secondValue;
      if (secondValue.has_value() && *secondValue <= 0.0) {
        addWarn("[FILLET_009] Fillet '" + fillet->featureID +
                "' secondValue=" + std::to_string(*secondValue) +
                " (ignored because it is not positive).");
      }

      for (size_t i = 0; i < fillet->params.radiusPoints.size(); ++i) {
        const auto& point = fillet->params.radiusPoints[i];
        const bool hasPositivePrimary =
            point.primaryValue.has_value() && *point.primaryValue > 0.0;
        if (!hasPositivePrimary) {
          addError("[FILLET_011] Fillet '" + fillet->featureID +
                   "' radiusPoints[" + std::to_string(i) +
                   "] missing positive primaryValue.");
        }
        const auto &pointSecondValue = point.secondValue;
        if (pointSecondValue.has_value() && *pointSecondValue <= 0.0) {
          addError("[FILLET_012] Fillet '" + fillet->featureID +
                   "' radiusPoints[" + std::to_string(i) + "].secondValue=" +
                   std::to_string(*pointSecondValue) + " (must be > 0).");
        }
        if (point.position < 0.0 || point.position > 1.0) {
          addWarn("[FILLET_013] Fillet '" + fillet->featureID +
                  "' radiusPoints[" + std::to_string(i) + "].position=" +
                  std::to_string(point.position) +
                  " is outside [0, 1] -- check edge parameter normalization.");
        }
      }

      auto checkFilletRef = [&](const std::shared_ptr<CRefEntityBase>& ref,
                                const std::string& role,
                                size_t index) {
        if (!ref) {
          addError("[FILLET_014] Fillet '" + fillet->featureID + "' " + role +
                   "[" + std::to_string(index) + "] is null.");
          return;
        }
        if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(ref)) {
          if (!subTopo->parentFeatureID.empty() &&
              !IsBuiltinStandardDatumID(subTopo->parentFeatureID) &&
              seen.find(subTopo->parentFeatureID) == seen.end()) {
            addWarn("[REF_007] Fillet '" + fillet->featureID + "' " + role +
                    "[" + std::to_string(index) + "] parent feature '" +
                    subTopo->parentFeatureID + "' has not been defined yet.");
          }
        }
      };

      for (size_t i = 0; i < fillet->references.size(); ++i) {
        checkFilletRef(fillet->references[i], "references", i);
      }
      for (size_t i = 0; i < fillet->side1Faces.size(); ++i) {
        checkFilletRef(fillet->side1Faces[i], "side1Faces", i);
      }
      for (size_t i = 0; i < fillet->side2Faces.size(); ++i) {
        checkFilletRef(fillet->side2Faces[i], "side2Faces", i);
      }
      for (size_t i = 0; i < fillet->centerFaces.size(); ++i) {
        checkFilletRef(fillet->centerFaces[i], "centerFaces", i);
      }
    }
    // ---- CDatumPlane ----
    else if (auto datumPlane = std::dynamic_pointer_cast<CDatumPlane>(feature)) {
      if (datumPlane->method == PlaneMethod::UNKNOWN) {
        addError("[DATUM_001] DatumPlane '" + datumPlane->featureID +
                 "' method is UNKNOWN.");
      }
      const bool hasProjectedOrigin = datumPlane->projectedOrigin.has_value();
      const bool hasNormal = datumPlane->normal.has_value();
      if (hasProjectedOrigin && !hasNormal) {
        addWarn("[GEOM_009] DatumPlane '" + datumPlane->featureID +
                "' has projectedOrigin but missing normal.");
      }
      if (hasNormal) {
        const double normalLen = vecLen(*datumPlane->normal);
        if (normalLen < GeoUtils::EPSILON) {
          addWarn("[GEOM_010] DatumPlane '" + datumPlane->featureID +
                  "' normal is zero.");
        } else if (std::abs(normalLen - 1.0) > 0.01) {
          addWarn("[GEOM_011] DatumPlane '" + datumPlane->featureID +
                  "' normal length=" + std::to_string(normalLen) +
                  " is not normalized.");
        }
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

