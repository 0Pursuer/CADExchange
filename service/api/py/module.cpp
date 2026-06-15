#include "python_api.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace CADExchange;
using namespace CADExchange::Accessor;
using namespace CADExchange::PythonApi;

namespace {

FeatureAccessorBase InvalidFeature() { return FeatureAccessorBase(nullptr); }

SketchAccessor FeatureAsSketch(const FeatureAccessorBase &feature) {
  return SketchAccessor(feature.GetRaw());
}

ExtrudeAccessor FeatureAsExtrude(const FeatureAccessorBase &feature) {
  return ExtrudeAccessor(feature.GetRaw());
}

RevolveAccessor FeatureAsRevolve(const FeatureAccessorBase &feature) {
  return RevolveAccessor(feature.GetRaw());
}

ChamferAccessor FeatureAsChamfer(const FeatureAccessorBase &feature) {
  return ChamferAccessor(feature.GetRaw());
}

DatumPlaneAccessor FeatureAsDatumPlane(const FeatureAccessorBase &feature) {
  return DatumPlaneAccessor(feature.GetRaw());
}

std::vector<double> GetLineStart(const SketchSegmentAccessor &segment) {
  CPoint3D start;
  CPoint3D end;
  if (!segment.GetLineCoords(start, end)) {
    return {};
  }
  return PointToVector(start);
}

std::vector<double> GetLineEnd(const SketchSegmentAccessor &segment) {
  CPoint3D start;
  CPoint3D end;
  if (!segment.GetLineCoords(start, end)) {
    return {};
  }
  return PointToVector(end);
}

std::vector<double> GetCircleCenter(const SketchSegmentAccessor &segment) {
  CPoint3D center;
  double radius = 0.0;
  if (!segment.GetCircleParams(center, radius)) {
    return {};
  }
  return PointToVector(center);
}

double GetCircleRadius(const SketchSegmentAccessor &segment) {
  CPoint3D center;
  double radius = 0.0;
  if (!segment.GetCircleParams(center, radius)) {
    return 0.0;
  }
  return radius;
}

py::dict GetArcData(const SketchSegmentAccessor &segment) {
  CPoint3D center;
  double startAngle = 0.0;
  double endAngle = 0.0;
  double radius = 0.0;
  bool clockwise = false;
  py::dict result;
  if (!segment.GetArcParams(center, startAngle, endAngle, radius, clockwise)) {
    return result;
  }
  result["center"] = PointToVector(center);
  result["start_angle"] = startAngle;
  result["end_angle"] = endAngle;
  result["radius"] = radius;
  result["clockwise"] = clockwise;
  return result;
}

std::vector<double> GetPointCoord(const SketchSegmentAccessor &segment) {
  CPoint3D point;
  if (!segment.GetPointCoord(point)) {
    return {};
  }
  return PointToVector(point);
}

std::vector<double> GetAxisOriginVector(const RevolveAccessor &revolve) {
  return PointToVector(revolve.GetAxisOrigin());
}

std::vector<double> GetAxisDirectionVector(const RevolveAccessor &revolve) {
  return VectorToVector(revolve.GetAxisDirection());
}

std::vector<double> GetDirectionVector(const ExtrudeAccessor &extrude) {
  return VectorToVector(extrude.GetDirection());
}

py::dict GetPlaneConstraintData(const PlaneConstraint &constraint) {
  py::dict result;
  result["type"] = constraint.type;
  result["ref"] = constraint.ref;
  result["value"] = constraint.value;
  result["reversed"] = constraint.reversed;
  if (constraint.defaultDir.has_value()) {
    result["default_dir"] = VectorToVector(*constraint.defaultDir);
  } else {
    result["default_dir"] = py::none();
  }
  return result;
}

} // namespace

PYBIND11_MODULE(cadexchange_py, m) {
  m.doc() = "CADExchange Python bindings built on service accessors.";

  py::enum_<UnitType>(m, "UnitType")
      .value("METER", UnitType::METER)
      .value("CENTIMETER", UnitType::CENTIMETER)
      .value("MILLIMETER", UnitType::MILLIMETER)
      .value("INCH", UnitType::INCH)
      .value("FOOT", UnitType::FOOT);

  py::enum_<FeatureType>(m, "FeatureType")
      .value("Unknown", FeatureType::Unknown)
      .value("Extrude", FeatureType::Extrude)
      .value("Revolve", FeatureType::Revolve)
      .value("Sweep", FeatureType::Sweep)
      .value("Fillet", FeatureType::Fillet)
      .value("Chamfer", FeatureType::Chamfer)
      .value("Rib", FeatureType::Rib)
      .value("Shell", FeatureType::Shell)
      .value("Sketch", FeatureType::Sketch)
      .value("DatumPlane", FeatureType::DatumPlane)
      .value("Draft", FeatureType::Draft);

  py::enum_<BooleanOp>(m, "BooleanOp")
      .value("BOSS", BooleanOp::BOSS)
      .value("CUT", BooleanOp::CUT)
      .value("MERGE", BooleanOp::MERGE);

  py::enum_<RefType>(m, "RefType")
      .value("FEATURE_DATUM_PLANE", RefType::FEATURE_DATUM_PLANE)
      .value("FEATURE_DATUM_AXIS", RefType::FEATURE_DATUM_AXIS)
      .value("FEATURE_DATUM_POINT", RefType::FEATURE_DATUM_POINT)
      .value("FEATURE_WHOLE_SKETCH", RefType::FEATURE_WHOLE_SKETCH)
      .value("TOPO_FACE", RefType::TOPO_FACE)
      .value("TOPO_EDGE", RefType::TOPO_EDGE)
      .value("TOPO_VERTEX", RefType::TOPO_VERTEX)
      .value("TOPO_SKETCH_SEG", RefType::TOPO_SKETCH_SEG)
      .value("UNKNOWN", RefType::UNKNOWN);

  py::enum_<CGeoCurveType>(m, "CGeoCurveType")
      .value("UNKNOWN", CGeoCurveType::UNKNOWN)
      .value("LINE", CGeoCurveType::LINE)
      .value("CIRCLE", CGeoCurveType::CIRCLE)
      .value("ELLIPSE", CGeoCurveType::ELLIPSE)
      .value("INTERSECTION", CGeoCurveType::INTERSECTION)
      .value("BCURVE", CGeoCurveType::BCURVE)
      .value("SPCURVE", CGeoCurveType::SPCURVE)
      .value("CONSTPARAM", CGeoCurveType::CONSTPARAM)
      .value("TRIMMED", CGeoCurveType::TRIMMED);

  py::enum_<ChamferMode>(m, "ChamferMode")
      .value("UNKNOWN", ChamferMode::UNKNOWN)
      .value("EQUAL_DISTANCE", ChamferMode::EQUAL_DISTANCE)
      .value("TWO_DISTANCES", ChamferMode::TWO_DISTANCES)
      .value("TWO_OFFSETS", ChamferMode::TWO_OFFSETS)
      .value("DISTANCE_ANGLE", ChamferMode::DISTANCE_ANGLE)
      .value("VERTEX_3DISTANCES", ChamferMode::VERTEX_3DISTANCES);

  py::enum_<PlaneMethod>(m, "PlaneMethod")
      .value("UNKNOWN", PlaneMethod::UNKNOWN)
      .value("OFFSET", PlaneMethod::OFFSET)
      .value("FIXED", PlaneMethod::FIXED)
      .value("ANGLE", PlaneMethod::ANGLE)
      .value("PARALLEL", PlaneMethod::PARALLEL)
      .value("PERPENDICULAR", PlaneMethod::PERPENDICULAR)
      .value("MID_PLANE", PlaneMethod::MID_PLANE)
      .value("THREE_POINTS", PlaneMethod::THREE_POINTS)
      .value("LINE", PlaneMethod::LINE)
      .value("TANGENT", PlaneMethod::TANGENT);

  py::enum_<PlaneConstraintType>(m, "PlaneConstraintType")
      .value("UNKNOWN", PlaneConstraintType::UNKNOWN)
      .value("PARALLEL", PlaneConstraintType::PARALLEL)
      .value("PERPENDICULAR", PlaneConstraintType::PERPENDICULAR)
      .value("COINCIDENT", PlaneConstraintType::COINCIDENT)
      .value("DISTANCE", PlaneConstraintType::DISTANCE)
      .value("ANGLE", PlaneConstraintType::ANGLE)
      .value("SYMMETRIC", PlaneConstraintType::SYMMETRIC)
      .value("TANGENT", PlaneConstraintType::TANGENT)
      .value("PROJECTION", PlaneConstraintType::PROJECTION);

  py::enum_<SweepExtent::Type>(m, "SweepExtentType")
      .value("VALUE", SweepExtent::Type::VALUE)
      .value("SYMMETRIC", SweepExtent::Type::SYMMETRIC)
      .value("THROUGH_ALL", SweepExtent::Type::THROUGH_ALL)
      .value("THROUGH_ALL_BOTH_SIDES", SweepExtent::Type::THROUGH_ALL_BOTH_SIDES)
      .value("UP_TO_NEXT", SweepExtent::Type::UP_TO_NEXT)
      .value("UP_TO_ENTITY", SweepExtent::Type::UP_TO_ENTITY)
      .value("UP_TO_EXTENDED", SweepExtent::Type::UP_TO_EXTENDED)
      .value("THRU_POINT", SweepExtent::Type::THRU_POINT)
      .value("UNKNOWN", SweepExtent::Type::UNKNOWN);

  py::enum_<CSketchSeg::SegType>(m, "SketchSegType")
      .value("LINE", CSketchSeg::SegType::LINE)
      .value("CIRCLE", CSketchSeg::SegType::CIRCLE)
      .value("ARC", CSketchSeg::SegType::ARC)
      .value("SPLINE", CSketchSeg::SegType::SPLINE)
      .value("POINT", CSketchSeg::SegType::POINT);

  py::enum_<SketchConstraintRefKind>(m, "SketchConstraintRefKind")
      .value("SketchEntity", SketchConstraintRefKind::SketchEntity)
      .value("ExternalReference", SketchConstraintRefKind::ExternalReference);

  py::enum_<SketchConstraintSubEntity>(m, "SketchConstraintSubEntity")
      .value("Whole", SketchConstraintSubEntity::Whole)
      .value("Start", SketchConstraintSubEntity::Start)
      .value("End", SketchConstraintSubEntity::End)
      .value("Center", SketchConstraintSubEntity::Center)
      .value("Midpoint", SketchConstraintSubEntity::Midpoint);

  py::enum_<CSketchConstraint::ConstraintType>(m, "SketchConstraintType")
      .value("COINCIDENT", CSketchConstraint::ConstraintType::COINCIDENT)
      .value("USEEDGE", CSketchConstraint::ConstraintType::USEEDGE)
      .value("HORIZONTAL", CSketchConstraint::ConstraintType::HORIZONTAL)
      .value("VERTICAL", CSketchConstraint::ConstraintType::VERTICAL)
      .value("PARALLEL", CSketchConstraint::ConstraintType::PARALLEL)
      .value("PERPENDICULAR", CSketchConstraint::ConstraintType::PERPENDICULAR)
      .value("TANGENT", CSketchConstraint::ConstraintType::TANGENT)
      .value("CONCENTRIC", CSketchConstraint::ConstraintType::CONCENTRIC)
      .value("EQUAL", CSketchConstraint::ConstraintType::EQUAL)
      .value("DISTANCE", CSketchConstraint::ConstraintType::DISTANCE)
      .value("ANGLE", CSketchConstraint::ConstraintType::ANGLE)
      .value("RADIUS", CSketchConstraint::ConstraintType::RADIUS)
      .value("DIAMETER", CSketchConstraint::ConstraintType::DIAMETER)
      .value("SYMMETRIC", CSketchConstraint::ConstraintType::SYMMETRIC)
      .value("MIDPOINT", CSketchConstraint::ConstraintType::MIDPOINT)
      .value("COLLINEAR", CSketchConstraint::ConstraintType::COLLINEAR)
      .value("FIXED", CSketchConstraint::ConstraintType::FIXED)
      .value("UNKNOWN", CSketchConstraint::ConstraintType::UNKNOWN);

  py::class_<ReferenceAccessor>(m, "ReferenceAccessor")
      .def("is_valid", &ReferenceAccessor::IsValid)
      .def_property_readonly("ref_type", &ReferenceAccessor::GetRefType)
      .def_property_readonly("parent_feature_id", &ReferenceAccessor::GetParentFeatureID)
      .def_property_readonly("target_feature_id", &ReferenceAccessor::GetTargetFeatureID)
      .def_property_readonly("topology_index", &ReferenceAccessor::GetTopologyIndex)
      .def_property_readonly("sketch_segment_local_id",
                             &ReferenceAccessor::GetSketchSegmentLocalID)
      .def_property_readonly("is_standard", &ReferenceAccessor::IsStandard)
      .def("point_hint", &GetReferencePointOrEmpty)
      .def("direction_hint", &GetReferenceDirectionOrEmpty);

  py::class_<SketchConstraintAccessor>(m, "SketchConstraintAccessor")
      .def("is_valid", &SketchConstraintAccessor::IsValid)
      .def_property_readonly("type", &SketchConstraintAccessor::GetType)
      .def_property_readonly("ref_count", &SketchConstraintAccessor::GetRefCount)
      .def("get_ref_kind", &SketchConstraintAccessor::GetRefKind)
      .def("get_ref_sub_entity", &SketchConstraintAccessor::GetRefSubEntity)
      .def("get_sketch_entity_local_id",
           &SketchConstraintAccessor::GetSketchEntityLocalID)
      .def("get_reference", &SketchConstraintAccessor::GetReference)
      .def_property_readonly("has_value", &SketchConstraintAccessor::HasValue)
      .def("get_value", &SketchConstraintAccessor::GetValue,
           py::arg("default_value") = 0.0);

  py::class_<SketchSegmentAccessor>(m, "SketchSegmentAccessor")
      .def("is_valid", &SketchSegmentAccessor::IsValid)
      .def_property_readonly("type", &SketchSegmentAccessor::GetType)
      .def_property_readonly("local_id", &SketchSegmentAccessor::GetLocalID)
      .def_property_readonly("is_construction",
                             &SketchSegmentAccessor::IsConstruction)
      .def("line_start", &GetLineStart)
      .def("line_end", &GetLineEnd)
      .def("circle_center", &GetCircleCenter)
      .def("circle_radius", &GetCircleRadius)
      .def("arc_data", &GetArcData)
      .def("point_coord", &GetPointCoord);

  py::class_<FeatureAccessorBase>(m, "FeatureAccessorBase")
      .def("is_valid", &FeatureAccessorBase::IsValid)
      .def_property_readonly("id", &FeatureAccessorBase::GetID)
      .def_property_readonly("name", &FeatureAccessorBase::GetName)
      .def_property_readonly("is_suppressed", &FeatureAccessorBase::IsSuppressed)
      .def_property_readonly("feature_type", &FeatureAccessorBase::GetFeatureType)
      .def("as_sketch", &FeatureAsSketch)
      .def("as_extrude", &FeatureAsExtrude)
      .def("as_revolve", &FeatureAsRevolve)
      .def("as_chamfer", &FeatureAsChamfer)
      .def("as_datum_plane", &FeatureAsDatumPlane);

  py::class_<ModelAccessor>(m, "ModelAccessor")
      .def("is_valid", &ModelAccessor::IsValid)
      .def_property_readonly("feature_count", &ModelAccessor::GetFeatureCount)
      .def("get_feature", &GetFeature)
      .def("get_feature_by_id", &GetFeatureById)
      .def("get_all_features", &GetAllFeatures)
      .def_property_readonly("unit", [](const ModelAccessor &m) {
        return m.Data()->unit;
      })
      .def_property_readonly("model_name", [](const ModelAccessor &m) {
        return m.Data()->modelName;
      });

  py::class_<SketchAccessor>(m, "SketchAccessor")
      .def("is_valid", &SketchAccessor::IsValid)
      .def_property_readonly("reference_plane", &SketchAccessor::GetReferencePlane)
      .def_property_readonly("has_reference_plane", &SketchAccessor::HasReferencePlane)
      .def_property_readonly("origin", &GetSketchOrigin)
      .def_property_readonly("x_axis", &GetSketchXAxis)
      .def_property_readonly("y_axis", &GetSketchYAxis)
      .def_property_readonly("z_axis", &GetSketchZAxis)
      .def_property_readonly("segment_count", &SketchAccessor::GetSegmentCount)
      .def("get_segment", &SketchAccessor::GetSegment)
      .def("get_segment_by_local_id", &SketchAccessor::GetSegmentByLocalID)
      .def_property_readonly("constraint_count", &SketchAccessor::GetConstraintCount)
      .def("get_constraint", &SketchAccessor::GetConstraintAccessor);

  py::class_<ExtrudeAccessor>(m, "ExtrudeAccessor")
      .def("is_valid", &ExtrudeAccessor::IsValid)
      .def_property_readonly("profile_sketch_id", &ExtrudeAccessor::GetProfileSketchID)
      .def_property_readonly("direction", &GetDirectionVector)
      .def_property_readonly("operation", &ExtrudeAccessor::GetOperation)
      .def_property_readonly("end_type1", &ExtrudeAccessor::GetEndType1)
      .def_property_readonly("depth1", &ExtrudeAccessor::GetDepth1)
      .def_property_readonly("offset1", &ExtrudeAccessor::GetOffset1)
      .def_property_readonly("has_offset1", &ExtrudeAccessor::HasOffset1)
      .def_property_readonly("is_flip1", &ExtrudeAccessor::IsFlip1)
      .def_property_readonly("is_flip_material_side1",
                             &ExtrudeAccessor::IsFlipMaterialSide1)
      .def_property_readonly("reference1", &ExtrudeAccessor::GetReference1)
      .def_property_readonly("has_direction2", &ExtrudeAccessor::HasDirection2)
      .def_property_readonly("end_type2", &ExtrudeAccessor::GetEndType2)
      .def_property_readonly("depth2", &ExtrudeAccessor::GetDepth2)
      .def_property_readonly("offset2", &ExtrudeAccessor::GetOffset2)
      .def_property_readonly("has_offset2", &ExtrudeAccessor::HasOffset2)
      .def_property_readonly("is_flip2", &ExtrudeAccessor::IsFlip2)
      .def_property_readonly("is_flip_material_side2",
                             &ExtrudeAccessor::IsFlipMaterialSide2)
      .def_property_readonly("reference2", &ExtrudeAccessor::GetReference2)
      .def_property_readonly("has_draft", &ExtrudeAccessor::HasDraft)
      .def_property_readonly("draft_angle", &ExtrudeAccessor::GetDraftAngle)
      .def_property_readonly("is_draft_outward", &ExtrudeAccessor::IsDraftOutward)
      .def_property_readonly("has_thin_wall", &ExtrudeAccessor::HasThinWall)
      .def_property_readonly("thin_wall_thickness",
                             &ExtrudeAccessor::GetThinWallThickness);

  py::class_<RevolveAccessor>(m, "RevolveAccessor")
      .def("is_valid", &RevolveAccessor::IsValid)
      .def_property_readonly("profile_sketch_id", &RevolveAccessor::GetProfileSketchID)
      .def_property_readonly("operation", &RevolveAccessor::GetOperation)
      .def_property_readonly("axis_origin", &GetAxisOriginVector)
      .def_property_readonly("axis_direction", &GetAxisDirectionVector)
      .def_property_readonly("axis_reference", &RevolveAccessor::GetAxisReference)
      .def_property_readonly("axis_reference_local_id",
                             &RevolveAccessor::GetAxisReferenceLocalID)
      .def_property_readonly("extent_type1", &RevolveAccessor::GetExtentType1)
      .def_property_readonly("extent_value1", &RevolveAccessor::GetExtentValue1)
      .def_property_readonly("extent_offset1", &RevolveAccessor::GetExtentOffset1)
      .def_property_readonly("has_offset1", &RevolveAccessor::HasOffset1)
      .def_property_readonly("is_flip1", &RevolveAccessor::IsFlip1)
      .def_property_readonly("is_flip_material_side1",
                             &RevolveAccessor::IsFlipMaterialSide1)
      .def_property_readonly("reference1", &RevolveAccessor::GetReference1)
      .def_property_readonly("has_extent2", &RevolveAccessor::HasExtent2)
      .def_property_readonly("extent_type2", &RevolveAccessor::GetExtentType2)
      .def_property_readonly("extent_value2", &RevolveAccessor::GetExtentValue2)
      .def_property_readonly("extent_offset2", &RevolveAccessor::GetExtentOffset2)
      .def_property_readonly("has_offset2", &RevolveAccessor::HasOffset2)
      .def_property_readonly("is_flip2", &RevolveAccessor::IsFlip2)
      .def_property_readonly("is_flip_material_side2",
                             &RevolveAccessor::IsFlipMaterialSide2)
      .def_property_readonly("reference2", &RevolveAccessor::GetReference2)
      .def_property_readonly("has_thin_wall", &RevolveAccessor::HasThinWall)
      .def_property_readonly("thin_wall_thickness",
                             &RevolveAccessor::GetThinWallThickness);

  py::class_<ChamferAccessor>(m, "ChamferAccessor")
      .def("is_valid", &ChamferAccessor::IsValid)
      .def_property_readonly("mode", &ChamferAccessor::GetMode)
      .def_property_readonly("has_distance1", &ChamferAccessor::HasDistance1)
      .def_property_readonly("distance1", &ChamferAccessor::GetDistance1)
      .def_property_readonly("has_distance2", &ChamferAccessor::HasDistance2)
      .def_property_readonly("distance2", &ChamferAccessor::GetDistance2)
      .def_property_readonly("has_distance3", &ChamferAccessor::HasDistance3)
      .def_property_readonly("distance3", &ChamferAccessor::GetDistance3)
      .def_property_readonly("has_offset1", &ChamferAccessor::HasOffset1)
      .def_property_readonly("offset1", &ChamferAccessor::GetOffset1)
      .def_property_readonly("has_offset2", &ChamferAccessor::HasOffset2)
      .def_property_readonly("offset2", &ChamferAccessor::GetOffset2)
      .def_property_readonly("has_angle", &ChamferAccessor::HasAngle)
      .def_property_readonly("angle", &ChamferAccessor::GetAngle)
      .def_property_readonly("has_first_end_face_marker",
                             &ChamferAccessor::HasFirstEndFaceMarker)
      .def_property_readonly("first_end_face_marker", [](const ChamferAccessor &a) {
        return PointToVector(a.GetFirstEndFaceMarker());
      })
      .def_property_readonly("reference_count", &ChamferAccessor::GetReferenceCount)
      .def("get_reference", &ChamferAccessor::GetReference);

  py::class_<DatumPlaneAccessor>(m, "DatumPlaneAccessor")
      .def("is_valid", &DatumPlaneAccessor::IsValid)
      .def_property_readonly("method", &DatumPlaneAccessor::GetMethod)
      .def_property_readonly("projected_origin",
                             [](const DatumPlaneAccessor &a) {
                               return PointToVector(a.GetProjectedOrigin());
                             })
      .def_property_readonly("normal", [](const DatumPlaneAccessor &a) {
        return VectorToVector(a.GetNormal());
      })
      .def_property_readonly("is_line_method", &DatumPlaneAccessor::IsLineMethod)
      .def_property_readonly("has_constraints", &DatumPlaneAccessor::HasConstraints)
      .def_property_readonly("constraint_count",
                             &DatumPlaneAccessor::GetConstraintCount)
      .def("get_constraint", [](const DatumPlaneAccessor &a, int index) {
        return GetPlaneConstraintData(a.GetConstraint(index));
      })
      .def_property_readonly("has_references", &DatumPlaneAccessor::HasReferences)
      .def_property_readonly("reference_count", &DatumPlaneAccessor::GetReferenceCount)
      .def("get_reference", &DatumPlaneAccessor::GetReference);

  m.def("load_model", &LoadModelAccessor, py::arg("path"));
}
