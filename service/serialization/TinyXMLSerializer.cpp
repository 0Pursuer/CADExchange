#include "TinyXMLSerializer.h"
#include <algorithm>
#include <cctype>
#include <functional>
#include <optional>
#include <cstdio>
#include <sstream>
#include <cmath>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

namespace CADExchange {

using namespace tinyxml2;

namespace {
// Helper to format a single double value with max 6 decimals, treating near-zero values as 0
double CleanupZero(double value) {
  if (std::abs(value) < 1e-10)
    return 0.0;
  return value;
}

// Format one component with %.6g: strips trailing zeros per-component.
// e.g. 1.0->"1", 0.5->"0.5", 3.141593->"3.14159"
std::string FormatDouble(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", CleanupZero(v));
  return buf;
}

// Format a (x,y,z) triple; each component formatted independently.
std::string FormatTriple(double x, double y, double z) {
  return "(" + FormatDouble(x) + "," + FormatDouble(y) + "," + FormatDouble(z) + ")";
}

bool TryParseTriple(const char *text, double &x, double &y, double &z) {
  if (!text)
    return false;
  std::string str = text;
  if (!str.empty() && str.front() == '(' && str.back() == ')') {
    str = str.substr(1, str.size() - 2);
  }
  size_t first = str.find(',');
  if (first == std::string::npos)
    return false;
  size_t second = str.find(',', first + 1);
  if (second == std::string::npos)
    return false;

  try {
    x = std::stod(str.substr(0, first));
    y = std::stod(str.substr(first + 1, second - first - 1));
    z = std::stod(str.substr(second + 1));
  } catch (...) {
    return false;
  }
  return true;
}

std::string ToLower(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return str;
}

std::string UnitTypeToString(UnitType type) {
  switch (type) {
  case UnitType::METER:
    return "Meter";
  case UnitType::CENTIMETER:
    return "Centimeter";
  case UnitType::MILLIMETER:
    return "Millimeter";
  case UnitType::INCH:
    return "Inch";
  case UnitType::FOOT:
    return "Foot";
  }
  return "Unknown";
}

std::optional<UnitType> UnitTypeFromString(const char *text) {
  if (!text)
    return std::nullopt;
  std::string value = ToLower(text);
  if (value == "meter")
    return UnitType::METER;
  if (value == "centimeter")
    return UnitType::CENTIMETER;
  if (value == "millimeter")
    return UnitType::MILLIMETER;
  if (value == "inch")
    return UnitType::INCH;
  if (value == "foot")
    return UnitType::FOOT;
  return std::nullopt;
}

std::string BooleanOpToString(BooleanOp op) {
  switch (op) {
  case BooleanOp::BOSS:
    return "BOSS";
  case BooleanOp::CUT:
    return "Cut";
  case BooleanOp::MERGE:
    return "Merge";
  }
  return "Unknown";
}

std::optional<BooleanOp> BooleanOpFromString(const char *text) {
  if (!text)
    return std::nullopt;
  std::string value = ToLower(text);
  if (value == "boss")
    return BooleanOp::BOSS;
  if (value == "cut")
    return BooleanOp::CUT;
  if (value == "merge")
    return BooleanOp::MERGE;
  return std::nullopt;
}

std::string CurveTypeToString(CGeoCurveType type) {
  switch (type) {
  case CGeoCurveType::LINE:
    return "Line";
  case CGeoCurveType::CIRCLE:
    return "Circle";
  case CGeoCurveType::ELLIPSE:
    return "Ellipse";
  case CGeoCurveType::INTERSECTION:
    return "Intersection";
  case CGeoCurveType::BCURVE:
    return "BCurve";
  case CGeoCurveType::SPCURVE:
    return "SPCurve";
  case CGeoCurveType::CONSTPARAM:
    return "ConstParam";
  case CGeoCurveType::TRIMMED:
    return "Trimmed";
  case CGeoCurveType::UNKNOWN:
  default:
    return "Unknown";
  }
}

std::optional<CGeoCurveType> CurveTypeFromString(const char *text) {
  if (!text) {
    return std::nullopt;
  }
  const std::string value = ToLower(text);
  if (value == "line")
    return CGeoCurveType::LINE;
  if (value == "circle")
    return CGeoCurveType::CIRCLE;
  if (value == "ellipse")
    return CGeoCurveType::ELLIPSE;
  if (value == "intersection")
    return CGeoCurveType::INTERSECTION;
  if (value == "bcurve")
    return CGeoCurveType::BCURVE;
  if (value == "spcurve")
    return CGeoCurveType::SPCURVE;
  if (value == "constparam")
    return CGeoCurveType::CONSTPARAM;
  if (value == "trimmed")
    return CGeoCurveType::TRIMMED;
  if (value == "unknown")
    return CGeoCurveType::UNKNOWN;
  return std::nullopt;
}

std::string SegTypeToString(CSketchSeg::SegType type) {
  switch (type) {
  case CSketchSeg::SegType::LINE:
    return "Line";
  case CSketchSeg::SegType::CIRCLE:
    return "Circle";
  case CSketchSeg::SegType::ARC:
    return "Arc";
  case CSketchSeg::SegType::SPLINE:
    return "Spline";
  case CSketchSeg::SegType::POINT:
    return "Point";
  }
  return "Unknown";
}

CSketchSeg::SegType SegTypeFromString(const char *text) {
  if (!text)
    return CSketchSeg::SegType::LINE; // Default
  std::string value = ToLower(text);
  if (value == "line")
    return CSketchSeg::SegType::LINE;
  if (value == "circle")
    return CSketchSeg::SegType::CIRCLE;
  if (value == "arc")
    return CSketchSeg::SegType::ARC;
  if (value == "spline")
    return CSketchSeg::SegType::SPLINE;
  if (value == "point")
    return CSketchSeg::SegType::POINT;
  return CSketchSeg::SegType::LINE; // Default
}

std::string SweepExtentTypeToString(SweepExtent::Type type) {
  switch (type) {
  case SweepExtent::Type::VALUE:
    return "Value";
  case SweepExtent::Type::SYMMETRIC:
    return "Symmetric";
  case SweepExtent::Type::THROUGH_ALL:
    return "ThroughAll";
  case SweepExtent::Type::THROUGH_ALL_BOTH_SIDES:
    return "ThroughAllBothSides";
  case SweepExtent::Type::UP_TO_NEXT:
    return "UpToNext";
  case SweepExtent::Type::UP_TO_ENTITY:
    return "UpToEntity";
  case SweepExtent::Type::UP_TO_EXTENDED:
    return "UpToExtended";
  case SweepExtent::Type::THRU_POINT:
    return "ThruPoint";
  case SweepExtent::Type::UNKNOWN:
    return "Unknown";
  }
  return "Unknown";
}

std::optional<SweepExtent::Type>
SweepExtentTypeFromString(const char *text) {
  if (!text)
    return std::nullopt;
  std::string value = ToLower(text);
  if (value == "value" || value == "blind")
    return SweepExtent::Type::VALUE;
  if (value == "symmetric")
    return SweepExtent::Type::SYMMETRIC;
  if (value == "throughall")
    return SweepExtent::Type::THROUGH_ALL;
  if (value == "throughallbothsides")
    return SweepExtent::Type::THROUGH_ALL_BOTH_SIDES;
  if (value == "uptonext")
    return SweepExtent::Type::UP_TO_NEXT;
  if (value == "uptoentity" || value == "uptoface" || value == "uptovertex")
    return SweepExtent::Type::UP_TO_ENTITY;
  if (value == "uptoextended")
    return SweepExtent::Type::UP_TO_EXTENDED;
  if (value == "thrupoint")
    return SweepExtent::Type::THRU_POINT;
  return std::nullopt;
}

std::string SweepPathOrientationToString(SweepPathOrientation orientation) {
  switch (orientation) {
  case SweepPathOrientation::FollowPath:
    return "FollowPath";
  case SweepPathOrientation::KeepProfileNormal:
    return "KeepProfileNormal";
  }
  return "FollowPath";
}

std::optional<SweepPathOrientation>
SweepPathOrientationFromString(const char *text) {
  if (!text) {
    return std::nullopt;
  }
  std::string value = ToLower(text);
  if (value == "followpath") {
    return SweepPathOrientation::FollowPath;
  }
  if (value == "keepprofilenormal") {
    return SweepPathOrientation::KeepProfileNormal;
  }
  return std::nullopt;
}

std::string SweepProfileKindToString(SweepProfileKind kind) {
  switch (kind) {
  case SweepProfileKind::SketchReference:
    return "SketchReference";
  case SweepProfileKind::EmbeddedSketch:
    return "EmbeddedSketch";
  case SweepProfileKind::Circular:
    return "Circular";
  }
  return "SketchReference";
}

std::optional<SweepProfileKind> SweepProfileKindFromString(const char *text) {
  if (!text) {
    return std::nullopt;
  }
  std::string value = ToLower(text);
  if (value == "sketchreference" || value == "sketch") {
    return SweepProfileKind::SketchReference;
  }
  if (value == "embeddedsketch" || value == "embedded") {
    return SweepProfileKind::EmbeddedSketch;
  }
  if (value == "circular") {
    return SweepProfileKind::Circular;
  }
  return std::nullopt;
}

std::string ChamferModeToString(ChamferMode mode) {
  switch (mode) {
  case ChamferMode::EQUAL_DISTANCE:
    return "EqualDistance";
  case ChamferMode::TWO_DISTANCES:
    return "TwoDistances";
  case ChamferMode::TWO_OFFSETS:
    return "TwoOffsets";
  case ChamferMode::DISTANCE_ANGLE:
    return "DistanceAngle";
  case ChamferMode::VERTEX_3DISTANCES:
    return "Vertex3Distances";
  case ChamferMode::UNKNOWN:
    return "Unknown";
  }
  return "Unknown";
}

std::optional<ChamferMode> ChamferModeFromString(const char *text) {
  if (!text) {
    return std::nullopt;
  }
  const std::string value = ToLower(text);
  if (value == "equaldistance") {
    return ChamferMode::EQUAL_DISTANCE;
  }
  if (value == "twodistances") {
    return ChamferMode::TWO_DISTANCES;
  }
  if (value == "twooffsets") {
    return ChamferMode::TWO_OFFSETS;
  }
  if (value == "distanceangle") {
    return ChamferMode::DISTANCE_ANGLE;
  }
  if (value == "vertex3distances") {
    return ChamferMode::VERTEX_3DISTANCES;
  }
  return std::nullopt;
}

std::string FilletModeToString(FilletMode mode) {
  switch (mode) {
  case FilletMode::CONSTANT_RADIUS:
    return "Constant";
  case FilletMode::VARIABLE_RADIUS:
    return "Variable";
  case FilletMode::FACE_FILLET:
    return "FaceFillet";
  case FilletMode::FULL_ROUND:
    return "FullRound";
  case FilletMode::CHORDAL:
    return "Chordal";
  case FilletMode::UNKNOWN:
    return "Unknown";
  }
  return "Unknown";
}

std::optional<FilletMode> FilletModeFromString(const char *text) {
  if (!text) {
    return std::nullopt;
  }
  const std::string value = ToLower(text);
  if (value == "constant") {
    return FilletMode::CONSTANT_RADIUS;
  }
  if (value == "constantradius") {
    return FilletMode::CONSTANT_RADIUS;
  }
  if (value == "variable") {
    return FilletMode::VARIABLE_RADIUS;
  }
  if (value == "variableradius") {
    return FilletMode::VARIABLE_RADIUS;
  }
  if (value == "facefillet") {
    return FilletMode::FACE_FILLET;
  }
  if (value == "fullround") {
    return FilletMode::FULL_ROUND;
  }
  if (value == "chordal") {
    return FilletMode::CHORDAL;
  }
  if (value == "unknown") {
    return FilletMode::UNKNOWN;
  }
  return std::nullopt;
}

std::string FilletCrossSectionToString(FilletCrossSection crossSection) {
  switch (crossSection) {
  case FilletCrossSection::CIRCULAR:
    return "Circular";
  case FilletCrossSection::CONIC:
    return "Conic";
  case FilletCrossSection::CURVATURE_CONTINUOUS:
    return "CurvatureContinuous";
  case FilletCrossSection::UNKNOWN:
    return "Unknown";
  }
  return "Unknown";
}

std::optional<FilletCrossSection>
FilletCrossSectionFromString(const char *text) {
  if (!text) {
    return std::nullopt;
  }
  const std::string value = ToLower(text);
  if (value == "circular") {
    return FilletCrossSection::CIRCULAR;
  }
  if (value == "conic") {
    return FilletCrossSection::CONIC;
  }
  if (value == "curvaturecontinuous") {
    return FilletCrossSection::CURVATURE_CONTINUOUS;
  }
  if (value == "unknown") {
    return FilletCrossSection::UNKNOWN;
  }
  return std::nullopt;
}

std::string FilletReferenceModeToString(FilletReferenceMode referenceMode) {
  switch (referenceMode) {
  case FilletReferenceMode::EDGE_CHAIN:
    return "EdgeChain";
  case FilletReferenceMode::FACE_FACE:
    return "FaceFace";
  case FilletReferenceMode::FULL_ROUND_THREE_FACES:
    return "FullRoundThreeFaces";
  case FilletReferenceMode::UNKNOWN:
    return "Unknown";
  }
  return "Unknown";
}

std::optional<FilletReferenceMode>
FilletReferenceModeFromString(const char *text) {
  if (!text) {
    return std::nullopt;
  }
  const std::string value = ToLower(text);
  if (value == "edgechain") {
    return FilletReferenceMode::EDGE_CHAIN;
  }
  if (value == "faceface") {
    return FilletReferenceMode::FACE_FACE;
  }
  if (value == "fullroundthreefaces") {
    return FilletReferenceMode::FULL_ROUND_THREE_FACES;
  }
  if (value == "unknown") {
    return FilletReferenceMode::UNKNOWN;
  }
  return std::nullopt;
}

std::string FilletConicValueModeToString(FilletConicValueMode mode) {
  switch (mode) {
  case FilletConicValueMode::RHO:
    return "Rho";
  case FilletConicValueMode::RADIUS:
    return "Radius";
  case FilletConicValueMode::GENERIC_VALUE:
    return "GenericValue";
  case FilletConicValueMode::NONE:
    return "None";
  }
  return "None";
}

std::optional<FilletConicValueMode>
FilletConicValueModeFromString(const char *text) {
  if (!text) {
    return std::nullopt;
  }
  const std::string value = ToLower(text);
  if (value == "rho") {
    return FilletConicValueMode::RHO;
  }
  if (value == "radius") {
    return FilletConicValueMode::RADIUS;
  }
  if (value == "genericvalue") {
    return FilletConicValueMode::GENERIC_VALUE;
  }
  if (value == "none") {
    return FilletConicValueMode::NONE;
  }
  return std::nullopt;
}

std::string FilletDriveTypeToString(FilletDriveType driveType) {
  switch (driveType) {
  case FilletDriveType::RADIUS:
    return "Radius";
  case FilletDriveType::SINGLE_DISTANCE:
  case FilletDriveType::TWO_DISTANCES:
    return "Distances";
  case FilletDriveType::UNKNOWN:
    return "Unknown";
  }
  return "Unknown";
}

std::optional<FilletDriveType> FilletDriveTypeFromString(const char *text) {
  if (!text) {
    return std::nullopt;
  }
  const std::string value = ToLower(text);
  if (value == "radius") {
    return FilletDriveType::RADIUS;
  }
  if (value == "distances") {
    return FilletDriveType::SINGLE_DISTANCE;
  }
  if (value == "singledistance") {
    return FilletDriveType::SINGLE_DISTANCE;
  }
  if (value == "twodistances") {
    return FilletDriveType::TWO_DISTANCES;
  }
  if (value == "unknown") {
    return FilletDriveType::UNKNOWN;
  }
  return std::nullopt;
}

std::string SweepSectionPlacementToString(SweepSectionPlacement placement) {
  switch (placement) {
  case SweepSectionPlacement::ExistingProfilePlane:
    return "ExistingProfilePlane";
  case SweepSectionPlacement::PathNormalAtStart:
    return "PathNormalAtStart";
  }
  return "ExistingProfilePlane";
}

std::optional<SweepSectionPlacement>
SweepSectionPlacementFromString(const char *text) {
  if (!text) {
    return std::nullopt;
  }
  std::string value = ToLower(text);
  if (value == "existingprofileplane" || value == "existing") {
    return SweepSectionPlacement::ExistingProfilePlane;
  }
  if (value == "pathnormalatstart" || value == "pathnormal") {
    return SweepSectionPlacement::PathNormalAtStart;
  }
  return std::nullopt;
}

std::string PlaneMethodToString(PlaneMethod method) {
  switch (method) {
  case PlaneMethod::OFFSET:
    return "Offset";
  case PlaneMethod::FIXED:
    return "Fixed";
  case PlaneMethod::ANGLE:
    return "Angle";
  case PlaneMethod::PARALLEL:
    return "Parallel";
  case PlaneMethod::PERPENDICULAR:
    return "Perpendicular";
  case PlaneMethod::MID_PLANE:
    return "MidPlane";
  case PlaneMethod::THREE_POINTS:
    return "ThreePoints";
  case PlaneMethod::LINE:
    return "Line";
  case PlaneMethod::TANGENT:
    return "Tangent";
  case PlaneMethod::UNKNOWN:
  default:
    return "Unknown";
  }
}

std::optional<PlaneMethod> PlaneMethodFromString(const char *text) {
  if (!text)
    return std::nullopt;
  std::string value = ToLower(text);
  if (value == "offset")
    return PlaneMethod::OFFSET;
  if (value == "fixed")
    return PlaneMethod::FIXED;
  if (value == "angle")
    return PlaneMethod::ANGLE;
  if (value == "parallel")
    return PlaneMethod::PARALLEL;
  if (value == "perpendicular")
    return PlaneMethod::PERPENDICULAR;
  if (value == "midplane")
    return PlaneMethod::MID_PLANE;
  if (value == "threepoints")
    return PlaneMethod::THREE_POINTS;
  if (value == "line")
    return PlaneMethod::LINE;
  if (value == "tangent")
    return PlaneMethod::TANGENT;
  return std::nullopt;
}

std::string PlaneConstraintTypeToString(PlaneConstraintType type) {
  switch (type) {
  case PlaneConstraintType::PARALLEL:
    return "Parallel";
  case PlaneConstraintType::PERPENDICULAR:
    return "Perpendicular";
  case PlaneConstraintType::COINCIDENT:
    return "Coincident";
  case PlaneConstraintType::DISTANCE:
    return "Distance";
  case PlaneConstraintType::ANGLE:
    return "Angle";
  case PlaneConstraintType::SYMMETRIC:
    return "Symmetric";
  case PlaneConstraintType::TANGENT:
    return "Tangent";
  case PlaneConstraintType::PROJECTION:
    return "Projection";
  case PlaneConstraintType::UNKNOWN:
  default:
    return "Unknown";
  }
}

std::optional<PlaneConstraintType>
PlaneConstraintTypeFromString(const char *text) {
  if (!text)
    return std::nullopt;
  std::string value = ToLower(text);
  if (value == "parallel")
    return PlaneConstraintType::PARALLEL;
  if (value == "perpendicular")
    return PlaneConstraintType::PERPENDICULAR;
  if (value == "coincident")
    return PlaneConstraintType::COINCIDENT;
  if (value == "distance")
    return PlaneConstraintType::DISTANCE;
  if (value == "angle")
    return PlaneConstraintType::ANGLE;
  if (value == "symmetric")
    return PlaneConstraintType::SYMMETRIC;
  if (value == "tangent")
    return PlaneConstraintType::TANGENT;
  if (value == "projection")
    return PlaneConstraintType::PROJECTION;
  return std::nullopt;
}

// ─── Internal helpers moved into anon-ns; not exported ──────────────────────
std::string FormatPoint(const CPoint3D &pt) {
  return FormatTriple(pt.x, pt.y, pt.z);
}

std::string FormatVector(const CVector3D &vec) {
  return FormatTriple(vec.x, vec.y, vec.z);
}

CPoint3D ParsePointAttribute(XMLElement *element, const char *name) {
  CPoint3D pt;
  double x, y, z;
  if (TryParseTriple(element->Attribute(name), x, y, z)) {
    pt.x = x; pt.y = y; pt.z = z;
  }
  return pt;
}

CVector3D ParseVectorAttribute(XMLElement *element, const char *name) {
  CVector3D vec;
  double x, y, z;
  if (TryParseTriple(element->Attribute(name), x, y, z)) {
    vec.x = x; vec.y = y; vec.z = z;
  }
  return vec;
}

std::shared_ptr<CRefFeature> LoadFeatureReference(XMLElement *element, RefType refType) {
  auto reference = std::make_shared<CRefFeature>(refType);
  if (const char *tid = element->Attribute("TargetFeatureID"))
    reference->targetFeatureID = tid;
  return reference;
}

void SaveFeatureReference(XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
  if (auto feature = std::dynamic_pointer_cast<CRefFeature>(ref))
    element->SetAttribute("TargetFeatureID", feature->targetFeatureID.c_str());
}

CVector3D ComputePlaneYAxis(const CVector3D &normal, const CVector3D &xDir) {
  CVector3D yDir = Cross(normal, xDir);
  yDir.Normalize();
  return yDir;
}

// ConstraintType <-> string (human-readable; integer fallback for old files).
const char *ConstraintTypeToString(CSketchConstraint::ConstraintType t) {
  switch (t) {
  case CSketchConstraint::ConstraintType::COINCIDENT:    return "Coincident";
  case CSketchConstraint::ConstraintType::USEEDGE:       return "UseEdge";
  case CSketchConstraint::ConstraintType::HORIZONTAL:    return "Horizontal";
  case CSketchConstraint::ConstraintType::VERTICAL:      return "Vertical";
  case CSketchConstraint::ConstraintType::PARALLEL:      return "Parallel";
  case CSketchConstraint::ConstraintType::PERPENDICULAR: return "Perpendicular";
  case CSketchConstraint::ConstraintType::TANGENT:       return "Tangent";
  case CSketchConstraint::ConstraintType::CONCENTRIC:    return "Concentric";
  case CSketchConstraint::ConstraintType::EQUAL:         return "Equal";
  case CSketchConstraint::ConstraintType::DISTANCE:      return "Distance";
  case CSketchConstraint::ConstraintType::ANGLE:         return "Angle";
  case CSketchConstraint::ConstraintType::RADIUS:        return "Radius";
  case CSketchConstraint::ConstraintType::DIAMETER:      return "Diameter";
  case CSketchConstraint::ConstraintType::SYMMETRIC:     return "Symmetric";
  case CSketchConstraint::ConstraintType::MIDPOINT:      return "Midpoint";
  case CSketchConstraint::ConstraintType::COLLINEAR:     return "Collinear";
  case CSketchConstraint::ConstraintType::FIXED:         return "Fixed";
  case CSketchConstraint::ConstraintType::UNKNOWN:       return "Unknown";
  default:                                               return "Unknown";
  }
}

CSketchConstraint::ConstraintType ConstraintTypeFromString(const char *text) {
  if (!text) return CSketchConstraint::ConstraintType::UNKNOWN;
  std::string v = ToLower(text);
  if (v == "coincident")    return CSketchConstraint::ConstraintType::COINCIDENT;
  if (v == "useedge")       return CSketchConstraint::ConstraintType::USEEDGE;
  if (v == "horizontal")    return CSketchConstraint::ConstraintType::HORIZONTAL;
  if (v == "vertical")      return CSketchConstraint::ConstraintType::VERTICAL;
  if (v == "parallel")      return CSketchConstraint::ConstraintType::PARALLEL;
  if (v == "perpendicular") return CSketchConstraint::ConstraintType::PERPENDICULAR;
  if (v == "tangent")       return CSketchConstraint::ConstraintType::TANGENT;
  if (v == "concentric")    return CSketchConstraint::ConstraintType::CONCENTRIC;
  if (v == "equal")         return CSketchConstraint::ConstraintType::EQUAL;
  if (v == "distance" || v == "dimensional") {
    return CSketchConstraint::ConstraintType::DISTANCE;
  }
  if (v == "angle")         return CSketchConstraint::ConstraintType::ANGLE;
  if (v == "radius")        return CSketchConstraint::ConstraintType::RADIUS;
  if (v == "diameter")      return CSketchConstraint::ConstraintType::DIAMETER;
  if (v == "symmetric")     return CSketchConstraint::ConstraintType::SYMMETRIC;
  if (v == "midpoint")      return CSketchConstraint::ConstraintType::MIDPOINT;
  if (v == "collinear")     return CSketchConstraint::ConstraintType::COLLINEAR;
  if (v == "fixed")         return CSketchConstraint::ConstraintType::FIXED;
  if (v == "unknown")       return CSketchConstraint::ConstraintType::UNKNOWN;
  // Backward-compat: integer fallback for files written by older versions.
  try { return static_cast<CSketchConstraint::ConstraintType>(std::stoi(text)); }
  catch (...) {}
  return CSketchConstraint::ConstraintType::UNKNOWN;
}

const char *SketchConstraintRefKindToString(SketchConstraintRefKind kind) {
  switch (kind) {
  case SketchConstraintRefKind::SketchEntity:      return "SketchEntity";
  case SketchConstraintRefKind::ExternalReference: return "ExternalReference";
  default:                                         return "SketchEntity";
  }
}

SketchConstraintRefKind SketchConstraintRefKindFromString(const char *text) {
  if (!text) return SketchConstraintRefKind::SketchEntity;
  return ToLower(text) == "externalreference"
             ? SketchConstraintRefKind::ExternalReference
             : SketchConstraintRefKind::SketchEntity;
}

const char *SketchConstraintSubEntityToString(SketchConstraintSubEntity sub) {
  switch (sub) {
  case SketchConstraintSubEntity::Whole:    return "Whole";
  case SketchConstraintSubEntity::Start:    return "Start";
  case SketchConstraintSubEntity::End:      return "End";
  case SketchConstraintSubEntity::Center:   return "Center";
  case SketchConstraintSubEntity::Midpoint: return "Midpoint";
  default:                                  return "Whole";
  }
}

SketchConstraintSubEntity SketchConstraintSubEntityFromString(const char *text) {
  if (!text) return SketchConstraintSubEntity::Whole;
  std::string v = ToLower(text);
  if (v == "start") return SketchConstraintSubEntity::Start;
  if (v == "end") return SketchConstraintSubEntity::End;
  if (v == "center") return SketchConstraintSubEntity::Center;
  if (v == "midpoint") return SketchConstraintSubEntity::Midpoint;
  return SketchConstraintSubEntity::Whole;
}
} // namespace

// =================================================================================================
// Save Implementation
// =================================================================================================

bool TinyXMLSerializer::Save(const UnifiedModel &model,
                             const std::filesystem::path &filePath,
                             std::string *errorMessage) {
  XMLDocument doc;

  // Declaration
  doc.InsertFirstChild(doc.NewDeclaration());

  // Root Element: UnifiedModel
  XMLElement *root = doc.NewElement("UnifiedModel");
  doc.InsertEndChild(root);

  // Attributes
  root->SetAttribute("UnitSystem", UnitTypeToString(model.unit).c_str());
  root->SetAttribute("ModelName", model.modelName.c_str());
  root->SetAttribute("FeatureCount",
                     static_cast<int64_t>(model.GetFeatures().size()));
  root->SetAttribute("SchemaVersion", 1);

  // Features
  for (const auto &feature : model.GetFeatures()) {
    SaveFeature(doc, root, feature);
  }

  XMLError result = doc.SaveFile(filePath.string().c_str());
  if (result != XML_SUCCESS) {
    if (errorMessage)
      *errorMessage = doc.ErrorStr();
    return false;
  }
  return true;
}

void TinyXMLSerializer::SavePoint3D(XMLElement *element, const char *name,
                                    const CPoint3D &pt) {
  std::string value = FormatTriple(pt.x, pt.y, pt.z);
  element->SetAttribute(name, value.c_str());
}

void TinyXMLSerializer::SaveVector3D(XMLElement *element, const char *name,
                                     const CVector3D &vec) {
  std::string value = FormatTriple(vec.x, vec.y, vec.z);
  element->SetAttribute(name, value.c_str());
}



struct RefSerializerEntry {
  using RefSaveFn = std::function<void(
      XMLElement *, const std::shared_ptr<CRefEntityBase> &)>;
  using RefLoadFn =
      std::function<std::shared_ptr<CRefEntityBase>(XMLElement *)>;

  RefType type;
  const char *name;
  const char *lowerName;
  RefSaveFn save;
  RefLoadFn load;
};
/**
 * @brief 引用实体序列化注册表。
 * 通过 RefType 注册对应的序列化函数。
 */
static const RefSerializerEntry kRefSerializerEntries[] = {
    {RefType::FEATURE_DATUM_PLANE, "Plane", "plane",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       if (auto plane = std::dynamic_pointer_cast<CRefPlane>(ref)) {
         element->SetAttribute("TargetFeatureID",
                               plane->targetFeatureID.c_str());
         element->SetAttribute("Origin", FormatPoint(plane->origin).c_str());
         element->SetAttribute("XDir", FormatVector(plane->xDir).c_str());
         element->SetAttribute("YDir", FormatVector(plane->yDir).c_str());
         element->SetAttribute("Normal", FormatVector(plane->normal).c_str());
       }
     },
     [](XMLElement *element) {
       auto plane = std::make_shared<CRefPlane>();
       if (const char *tid = element->Attribute("TargetFeatureID"))
         plane->targetFeatureID = tid;
       plane->origin = ParsePointAttribute(element, "Origin");
       plane->xDir = ParseVectorAttribute(element, "XDir");
       plane->normal = ParseVectorAttribute(element, "Normal");
       if (element->Attribute("YDir")) {
         plane->yDir = ParseVectorAttribute(element, "YDir");
       } else {
         plane->yDir = ComputePlaneYAxis(plane->normal, plane->xDir);
       }
       return plane;
     }},
    {RefType::FEATURE_DATUM_AXIS, "Axis", "axis",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       SaveFeatureReference(element, ref);
     },
     [](XMLElement *element) {
       return LoadFeatureReference(element, RefType::FEATURE_DATUM_AXIS);
     }},
    {RefType::FEATURE_DATUM_POINT, "Point", "point",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       SaveFeatureReference(element, ref);
     },
     [](XMLElement *element) {
       return LoadFeatureReference(element, RefType::FEATURE_DATUM_POINT);
     }},
    {RefType::FEATURE_WHOLE_SKETCH, "Sketch", "sketch",
     [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
       if (auto sketch = std::dynamic_pointer_cast<CRefSketch>(ref)) {
         element->SetAttribute("TargetFeatureID",
                               sketch->targetFeatureID.c_str());
       }
     },
     [](XMLElement *element) {
       auto sketch = std::make_shared<CRefSketch>();
       if (const char *tid = element->Attribute("TargetFeatureID"))
         sketch->targetFeatureID = tid;
       return sketch;
     }},
    {RefType::TOPO_FACE, "Face", "face",
      [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
        if (auto face = std::dynamic_pointer_cast<CRefFace>(ref)) {
          // Legacy compatibility only: TopologyIndex is preserved for old data
          // streams, but new matching logic should rely on geometry and owner IDs.
          element->SetAttribute("ParentFeatureID",
                                face->parentFeatureID.c_str());
#ifdef _MSC_VER
#pragma warning(suppress : 4996)
#endif
          element->SetAttribute("TopologyIndex", face->topologyIndex);
         element->SetAttribute("U", FormatVector(face->uDir).c_str());
         element->SetAttribute("V", FormatVector(face->vDir).c_str());
         element->SetAttribute("Normal", FormatVector(face->normal).c_str());
         element->SetAttribute("Center", FormatPoint(face->centroid).c_str());
       }
     },
     [](XMLElement *element) {
        auto face = std::make_shared<CRefFace>();
        if (const char *parent = element->Attribute("ParentFeatureID"))
          face->parentFeatureID = parent;
#ifdef _MSC_VER
#pragma warning(suppress : 4996)
#endif
        element->QueryIntAttribute("TopologyIndex", &face->topologyIndex);
       face->uDir = ParseVectorAttribute(element, "U");
       face->vDir = ParseVectorAttribute(element, "V");
       face->normal = ParseVectorAttribute(element, "Normal");
       face->centroid = ParsePointAttribute(element, "Center");
       return face;
     }},
    {RefType::TOPO_EDGE, "Edge", "edge",
      [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
        if (auto edge = std::dynamic_pointer_cast<CRefEdge>(ref)) {
          // Legacy compatibility only: TopologyIndex is preserved for old data
          // streams, but new matching logic should rely on geometry and owner IDs.
          element->SetAttribute("ParentFeatureID",
                                edge->parentFeatureID.c_str());
#ifdef _MSC_VER
#pragma warning(suppress : 4996)
#endif
          element->SetAttribute("TopologyIndex", edge->topologyIndex);
          element->SetAttribute("StartPoint", FormatPoint(edge->startPoint).c_str());
          element->SetAttribute("EndPoint", FormatPoint(edge->endPoint).c_str());
          element->SetAttribute("MidPoint", FormatPoint(edge->midPoint).c_str());
          element->SetAttribute("CurveType",
                                CurveTypeToString(edge->curveType).c_str());
        }
      },
      [](XMLElement *element) {
        auto edge = std::make_shared<CRefEdge>();
        if (const char *parent = element->Attribute("ParentFeatureID"))
          edge->parentFeatureID = parent;
#ifdef _MSC_VER
#pragma warning(suppress : 4996)
#endif
        element->QueryIntAttribute("TopologyIndex", &edge->topologyIndex);
        edge->startPoint = ParsePointAttribute(element, "StartPoint");
        edge->endPoint = ParsePointAttribute(element, "EndPoint");
        edge->midPoint = ParsePointAttribute(element, "MidPoint");
        if (const char *curveTypeText = element->Attribute("CurveType")) {
          if (auto mapped = CurveTypeFromString(curveTypeText)) {
            edge->curveType = *mapped;
          } else {
            char *end = nullptr;
            const long curveTypeValue = std::strtol(curveTypeText, &end, 10);
            if (end != curveTypeText && end != nullptr && *end == '\0') {
              edge->curveType = static_cast<CGeoCurveType>(curveTypeValue);
            }
          }
        }
        return edge;
      }},
    {RefType::TOPO_VERTEX, "Vertex", "vertex",
      [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
        if (auto vertex = std::dynamic_pointer_cast<CRefVertex>(ref)) {
          // Legacy compatibility only: TopologyIndex is preserved for old data
          // streams, but new matching logic should rely on geometry and owner IDs.
          element->SetAttribute("ParentFeatureID",
                                vertex->parentFeatureID.c_str());
#ifdef _MSC_VER
#pragma warning(suppress : 4996)
#endif
          element->SetAttribute("TopologyIndex", vertex->topologyIndex);
         element->SetAttribute("Position", FormatPoint(vertex->pos).c_str());
       }
     },
     [](XMLElement *element) {
        auto vertex = std::make_shared<CRefVertex>();
        if (const char *parent = element->Attribute("ParentFeatureID"))
          vertex->parentFeatureID = parent;
#ifdef _MSC_VER
#pragma warning(suppress : 4996)
#endif
        element->QueryIntAttribute("TopologyIndex", &vertex->topologyIndex);
       vertex->pos = ParsePointAttribute(element, "Position");
       return vertex;
     }},
    {RefType::TOPO_SKETCH_SEG, "SketchSeg", "sketchseg",
      [](XMLElement *element, const std::shared_ptr<CRefEntityBase> &ref) {
        if (auto seg = std::dynamic_pointer_cast<CRefSketchSeg>(ref)) {
          // Legacy compatibility only: TopologyIndex is preserved for old data
          // streams, but new matching logic should rely on geometry and owner IDs.
          element->SetAttribute("ParentFeatureID", seg->parentFeatureID.c_str());
#ifdef _MSC_VER
#pragma warning(suppress : 4996)
#endif
          element->SetAttribute("TopologyIndex", seg->topologyIndex);
         if (!seg->segmentLocalID.empty())
           element->SetAttribute("SegmentLocalID", seg->segmentLocalID.c_str());
       }
     },
     [](XMLElement *element) {
        auto seg = std::make_shared<CRefSketchSeg>();
        if (const char *parent = element->Attribute("ParentFeatureID"))
          seg->parentFeatureID = parent;
#ifdef _MSC_VER
#pragma warning(suppress : 4996)
#endif
        element->QueryIntAttribute("TopologyIndex", &seg->topologyIndex);
       if (const char *lid = element->Attribute("SegmentLocalID"))
         seg->segmentLocalID = lid;
       return seg;
     }}};

const RefSerializerEntry *FindRefEntry(RefType type) {
  for (const auto &entry : kRefSerializerEntries) {
    if (entry.type == type)
      return &entry;
  }
  return nullptr;
}

const RefSerializerEntry *FindRefEntryByName(const std::string &value) {
  for (const auto &entry : kRefSerializerEntries) {
    if (value == entry.lowerName)
      return &entry;
  }
  return nullptr;
}

std::string RefTypeToString(RefType type) {
  if (auto entry = FindRefEntry(type))
    return entry->name;
  return "Unknown";
}

std::optional<RefType> RefTypeFromString(const char *text) {
  if (!text)
    return std::nullopt;
  std::string value = ToLower(text);
  if (auto entry = FindRefEntryByName(value))
    return entry->type;
  return std::nullopt;
}

void TinyXMLSerializer::SaveRefEntity(
    XMLDocument &doc, XMLElement *parent, const std::string &name,
    const std::shared_ptr<CRefEntityBase> &ref) {
  if (!ref)
    return;

  XMLElement *refElem = doc.NewElement(name.c_str());
  parent->InsertEndChild(refElem);

  if (auto entry = FindRefEntry(ref->refType)) {
    entry->save(refElem, ref);
    refElem->SetAttribute("Type", entry->name);
  } else {
    SaveFeatureReference(refElem, ref);
    refElem->SetAttribute("Type", RefTypeToString(ref->refType).c_str());
  }
}

void TinyXMLSerializer::SaveFeature(
    XMLDocument &doc, XMLElement *parent,
    const std::shared_ptr<CFeatureBase> &feature) {
  if (!feature)
    return;

  std::fprintf(stderr, "[TinyXMLSerializer] SaveFeature begin type=%d id=%s\n",
               static_cast<int>(feature->featureType), feature->featureID.c_str());

  XMLElement *featElem = doc.NewElement("Feature");
  parent->InsertEndChild(featElem);

  // Identity attrs first (matches XMLHelper::CreateFeatureNode convention).
  featElem->SetAttribute("ID", feature->featureID.c_str());
  featElem->SetAttribute("Name", feature->featureName.c_str());
  featElem->SetAttribute("Suppressed", feature->isSuppressed);

  switch (feature->featureType) {
    case FeatureType::Sketch:
      featElem->SetAttribute("Type", "Sketch");
      SaveSketch(doc, featElem, std::static_pointer_cast<CSketch>(feature));
      break;
    case FeatureType::Extrude:
      featElem->SetAttribute("Type", "Extrude");
      SaveExtrude(doc, featElem, std::static_pointer_cast<CExtrude>(feature));
      break;
    case FeatureType::Revolve:
      featElem->SetAttribute("Type", "Revolve");
      SaveRevolve(doc, featElem, std::static_pointer_cast<CRevolve>(feature));
      break;
    case FeatureType::Sweep:
      featElem->SetAttribute("Type", "Sweep");
      SaveSweep(doc, featElem, std::static_pointer_cast<CSweep>(feature));
      break;
    case FeatureType::Fillet:
      featElem->SetAttribute("Type", "Fillet");
      SaveFillet(doc, featElem, std::static_pointer_cast<CFillet>(feature));
      break;
    case FeatureType::Chamfer:
      featElem->SetAttribute("Type", "Chamfer");
      SaveChamfer(doc, featElem, std::static_pointer_cast<CChamfer>(feature));
      break;
    case FeatureType::DatumPlane:
      featElem->SetAttribute("Type", "DatumPlane");
      SaveDatumPlane(doc, featElem,
                     std::static_pointer_cast<CDatumPlane>(feature));
      break;
    default:
      featElem->SetAttribute("Type", "Unknown");
      break;
  }

  std::fprintf(stderr, "[TinyXMLSerializer] SaveFeature done type=%d id=%s\n",
               static_cast<int>(feature->featureType), feature->featureID.c_str());

}

void TinyXMLSerializer::SaveSketch(XMLDocument &doc, XMLElement *element,
                                   const std::shared_ptr<CSketch> &sketch) {
  SaveRefEntity(doc, element, "ReferencePlane", sketch->referencePlane);

  XMLElement *csysElem = doc.NewElement("LocalCSys");
  element->InsertEndChild(csysElem);
  SavePoint3D(csysElem, "Origin", sketch->sketchCSys.origin);
  SaveVector3D(csysElem, "XDir", sketch->sketchCSys.xDir);
  SaveVector3D(csysElem, "YDir", sketch->sketchCSys.yDir);
  SaveVector3D(csysElem, "ZDir", sketch->sketchCSys.zDir);

  XMLElement *segsElem = doc.NewElement("Segments");
  element->InsertEndChild(segsElem);
  for (const auto &seg : sketch->segments) {
    SaveSketchSeg(doc, segsElem, seg);
  }

  XMLElement *consElem = doc.NewElement("Constraints");
  element->InsertEndChild(consElem);
  for (const auto &con : sketch->constraints) {
    SaveConstraint(doc, consElem, con);
  }
}

void TinyXMLSerializer::SaveSketchSeg(XMLDocument &doc, XMLElement *parent,
                                      const std::shared_ptr<CSketchSeg> &seg) {
  if (!seg)
    return;
  XMLElement *segElem = doc.NewElement("Segment");
  parent->InsertEndChild(segElem);

  segElem->SetAttribute("LocalID", seg->localID.c_str());
  segElem->SetAttribute("Type", SegTypeToString(seg->type).c_str());

  if (seg->type != CSketchSeg::SegType::POINT) {
    segElem->SetAttribute("Construction", seg->isConstruction);
  }

  if (auto line = std::dynamic_pointer_cast<CSketchLine>(seg)) {
    SavePoint3D(segElem, "Start", line->startPos);
    SavePoint3D(segElem, "End", line->endPos);
  } else if (auto circle = std::dynamic_pointer_cast<CSketchCircle>(seg)) {
    SavePoint3D(segElem, "Center", circle->center);
    segElem->SetAttribute("Radius", circle->radius);
  } else if (auto arc = std::dynamic_pointer_cast<CSketchArc>(seg)) {
    SavePoint3D(segElem, "Center", arc->center);
    segElem->SetAttribute("Radius", arc->radius);
    segElem->SetAttribute("StartAngle", arc->startAngle);
    segElem->SetAttribute("EndAngle", arc->endAngle);
    segElem->SetAttribute("Clockwise", arc->isClockwise);
  } else if (auto pt = std::dynamic_pointer_cast<CSketchPoint>(seg)) {
    SavePoint3D(segElem, "Position", pt->position);
  }
}

void TinyXMLSerializer::SaveConstraint(XMLDocument &doc, XMLElement *parent,
                                       const CSketchConstraint &constraint) {
  XMLElement *conElem = doc.NewElement("Constraint");
  parent->InsertEndChild(conElem);
  conElem->SetAttribute("Type", ConstraintTypeToString(constraint.type));
  if (constraint.value.has_value()) {
    conElem->SetAttribute("Value", *constraint.value);
  }

  XMLElement *refsElem = doc.NewElement("Refs");
  conElem->InsertEndChild(refsElem);
  for (const auto &ref : constraint.refs) {
    XMLElement *refElem = doc.NewElement("Ref");
    refElem->SetAttribute("Kind", SketchConstraintRefKindToString(ref.kind));
    refElem->SetAttribute("SubEntity",
                          SketchConstraintSubEntityToString(ref.subEntity));
    if (ref.kind == SketchConstraintRefKind::SketchEntity) {
      refElem->SetAttribute("SketchEntityLocalID", ref.sketchEntityLocalID.c_str());
    } else if (ref.refEntity) {
      SaveRefEntity(doc, refElem, "ReferenceEntity", ref.refEntity);
    }
    refsElem->InsertEndChild(refElem);
  }
}

void TinyXMLSerializer::SaveExtrude(XMLDocument &doc, XMLElement *element,
                                    const std::shared_ptr<CExtrude> &extrude) {
  if (!extrude->profileSketchID.empty())
    element->SetAttribute("ProfileSketchID", extrude->profileSketchID.c_str());

  XMLElement *directionElem = doc.NewElement("Direction");
  SaveVector3D(directionElem, "Value", extrude->direction);
  element->InsertEndChild(directionElem);

  element->SetAttribute("Operation",
                        BooleanOpToString(extrude->operation).c_str());

  // 统一写 extent 结构，保持 Extrude/Revolve 的共享语义。
  auto saveExtent = [&](const char *tag, const SweepExtent &extent) {
    XMLElement *elem = doc.NewElement(tag);
    elem->SetAttribute("Type", SweepExtentTypeToString(extent.type).c_str());
    elem->SetAttribute("Value", extent.value);
    elem->SetAttribute("Offset", extent.offset);
    elem->SetAttribute("HasOffset", extent.hasOffset);
    elem->SetAttribute("Flip", extent.isFlip);
    elem->SetAttribute("FlipMaterialSide", extent.isFlipMaterialSide);
    if (extent.helperPoint) {
      XMLElement *helperElem = doc.NewElement("HelperPoint");
      SavePoint3D(helperElem, "Value", *extent.helperPoint);
      elem->InsertEndChild(helperElem);
    }
    SaveRefEntity(doc, elem, "ReferenceEntity", extent.referenceEntity);
    element->InsertEndChild(elem);
  };

  saveExtent("Extent1", extrude->extent1);
  if (extrude->extent2) {
    saveExtent("Extent2", *extrude->extent2);
  }

  // 薄壁参数（可选）
  if (extrude->thinWall.has_value()) {
    XMLElement *twElem = doc.NewElement("ThinWall");
    twElem->SetAttribute("Covered",   extrude->thinWall->isCovered);
    twElem->SetAttribute("StartOffset", extrude->thinWall->startOffset);
    twElem->SetAttribute("EndOffset", extrude->thinWall->endOffset);
    element->InsertEndChild(twElem);
  }
}

void TinyXMLSerializer::SaveRevolve(XMLDocument &doc, XMLElement *element,
                                    const std::shared_ptr<CRevolve> &revolve) {
  if (!revolve->profileSketchID.empty())
    element->SetAttribute("ProfileSketchID", revolve->profileSketchID.c_str());
  element->SetAttribute("Operation",
                        BooleanOpToString(revolve->operation).c_str());

  auto saveExtent = [&](const char *tag, const SweepExtent &extent) {
    XMLElement *elem = doc.NewElement(tag);
    elem->SetAttribute("Type", SweepExtentTypeToString(extent.type).c_str());
    elem->SetAttribute("Value", extent.value);
    elem->SetAttribute("Offset", extent.offset);
    elem->SetAttribute("HasOffset", extent.hasOffset);
    elem->SetAttribute("Flip", extent.isFlip);
    elem->SetAttribute("FlipMaterialSide", extent.isFlipMaterialSide);
    if (extent.helperPoint) {
      XMLElement *helperElem = doc.NewElement("HelperPoint");
      SavePoint3D(helperElem, "Value", *extent.helperPoint);
      elem->InsertEndChild(helperElem);
    }
    SaveRefEntity(doc, elem, "ReferenceEntity", extent.referenceEntity);
    element->InsertEndChild(elem);
  };

  saveExtent("Extent1", revolve->extent1);
  if (revolve->extent2) {
    saveExtent("Extent2", *revolve->extent2);
  }

  XMLElement *axisElem = doc.NewElement("Axis");
  element->InsertEndChild(axisElem);
  std::string axisLocalID = revolve->axis.referenceLocalID;
  if (axisLocalID.empty()) {
    if (auto sketchSeg =
            std::dynamic_pointer_cast<CRefSketchSeg>(revolve->axis.referenceEntity)) {
      axisLocalID = sketchSeg->segmentLocalID;
    }
  }
  axisElem->SetAttribute("RefLocalID", axisLocalID.c_str());
  SavePoint3D(axisElem, "Origin", revolve->axis.origin);
  SaveVector3D(axisElem, "Direction", revolve->axis.direction);
  std::shared_ptr<CRefEntityBase> axis_ref = revolve->axis.referenceEntity;
  if (!axis_ref && !revolve->axis.referenceLocalID.empty() &&
      !revolve->profileSketchID.empty()) {
    auto sketch_seg_ref = std::make_shared<CRefSketchSeg>();
    sketch_seg_ref->parentFeatureID = revolve->profileSketchID;
    sketch_seg_ref->segmentLocalID = revolve->axis.referenceLocalID;
#ifdef _MSC_VER
#pragma warning(suppress : 4996)
#endif
    sketch_seg_ref->topologyIndex = -1;
    axis_ref = sketch_seg_ref;
  }
  SaveRefEntity(doc, axisElem, "ReferenceEntity", axis_ref);

  if (revolve->thinWall.has_value()) {
    XMLElement *twElem = doc.NewElement("ThinWall");
    twElem->SetAttribute("Covered",   revolve->thinWall->isCovered);
    twElem->SetAttribute("StartOffset", revolve->thinWall->startOffset);
    twElem->SetAttribute("EndOffset", revolve->thinWall->endOffset);
    element->InsertEndChild(twElem);
  }
}

void TinyXMLSerializer::SaveSweep(XMLDocument &doc, XMLElement *element,
                                  const std::shared_ptr<CSweep> &sweep) {
  if (!sweep->profileSketchID.empty()) {
    element->SetAttribute("ProfileSketchID", sweep->profileSketchID.c_str());
  }
  element->SetAttribute("Operation",
                        BooleanOpToString(sweep->operation).c_str());
  element->SetAttribute(
      "Orientation", SweepPathOrientationToString(sweep->orientation).c_str());
  element->SetAttribute(
      "SectionPlacement",
      SweepSectionPlacementToString(sweep->sectionPlacement).c_str());
  if (sweep->profilePathAngleCos) {
    element->SetAttribute("ProfilePathAngleCos", *sweep->profilePathAngleCos);
  }

  XMLElement *profileElem = doc.NewElement("Profile");
  profileElem->SetAttribute("Kind",
                            SweepProfileKindToString(sweep->profile.kind).c_str());
  switch (sweep->profile.kind) {
  case SweepProfileKind::SketchReference:
    profileElem->SetAttribute("SketchID",
                              (!sweep->profile.sketchID.empty()
                                   ? sweep->profile.sketchID
                                   : sweep->profileSketchID)
                                  .c_str());
    break;
  case SweepProfileKind::EmbeddedSketch:
    if (sweep->profile.embedded) {
      XMLElement *sketchElem = doc.NewElement("EmbeddedSketch");
      auto sketchCopy =
          std::make_shared<CSketch>(sweep->profile.embedded->sketch);
      SaveSketch(doc, sketchElem, sketchCopy);
      profileElem->InsertEndChild(sketchElem);
    }
    break;
  case SweepProfileKind::Circular:
    if (sweep->profile.circular) {
      profileElem->SetAttribute("OuterRadius",
                                sweep->profile.circular->outerRadius);
      profileElem->SetAttribute("InnerRadius",
                                sweep->profile.circular->innerRadius);
    }
    break;
  }
  element->InsertEndChild(profileElem);

  XMLElement *pathElem = doc.NewElement("Path");
  pathElem->SetAttribute("Closed", sweep->path.isClosed);
  element->InsertEndChild(pathElem);
  if (sweep->path.startPoint) {
    XMLElement *pointElem = doc.NewElement("StartPoint");
    SavePoint3D(pointElem, "Value", *sweep->path.startPoint);
    pathElem->InsertEndChild(pointElem);
  }
  if (sweep->path.endPoint) {
    XMLElement *pointElem = doc.NewElement("EndPoint");
    SavePoint3D(pointElem, "Value", *sweep->path.endPoint);
    pathElem->InsertEndChild(pointElem);
  }
  for (const auto &ref : sweep->path.references) {
    SaveRefEntity(doc, pathElem, "ReferenceEntity", ref);
  }

  if (!sweep->guidePaths.empty()) {
    XMLElement *guidesElem = doc.NewElement("GuidePaths");
    element->InsertEndChild(guidesElem);
    for (const auto &guidePath : sweep->guidePaths) {
      XMLElement *guideElem = doc.NewElement("GuidePath");
      guideElem->SetAttribute("Closed", guidePath.isClosed);
      guidesElem->InsertEndChild(guideElem);
      if (guidePath.startPoint) {
        XMLElement *pointElem = doc.NewElement("StartPoint");
        SavePoint3D(pointElem, "Value", *guidePath.startPoint);
        guideElem->InsertEndChild(pointElem);
      }
      if (guidePath.endPoint) {
        XMLElement *pointElem = doc.NewElement("EndPoint");
        SavePoint3D(pointElem, "Value", *guidePath.endPoint);
        guideElem->InsertEndChild(pointElem);
      }
      for (const auto &ref : guidePath.references) {
        SaveRefEntity(doc, guideElem, "ReferenceEntity", ref);
      }
    }
  }

  if (sweep->thinWall.has_value()) {
    XMLElement *twElem = doc.NewElement("ThinWall");
    twElem->SetAttribute("Covered", sweep->thinWall->isCovered);
    twElem->SetAttribute("StartOffset", sweep->thinWall->startOffset);
    twElem->SetAttribute("EndOffset", sweep->thinWall->endOffset);
    element->InsertEndChild(twElem);
  }
}

void TinyXMLSerializer::SaveDatumPlane(
    XMLDocument &doc, XMLElement *element,
    const std::shared_ptr<CDatumPlane> &datumPlane) {
  element->SetAttribute("Method", PlaneMethodToString(datumPlane->method).c_str());

  XMLElement *refsElem = doc.NewElement("ReferenceEntities");
  element->InsertEndChild(refsElem);
  for (const auto &ref : datumPlane->referenceEntities) {
    SaveRefEntity(doc, refsElem, "ReferenceEntity", ref);
  }

  XMLElement *constraintsElem = doc.NewElement("Constraints");
  element->InsertEndChild(constraintsElem);
  for (const auto &constraint : datumPlane->constraints) {
    XMLElement *constraintElem = doc.NewElement("Constraint");
    constraintElem->SetAttribute(
        "Type", PlaneConstraintTypeToString(constraint.type).c_str());
    constraintElem->SetAttribute("Ref", constraint.ref);
    constraintElem->SetAttribute("Value", constraint.value);
    constraintElem->SetAttribute("Reversed", constraint.reversed);
    if (constraint.defaultDir.has_value()) {
      SaveVector3D(constraintElem, "DefaultDir", *constraint.defaultDir);
    }
    constraintsElem->InsertEndChild(constraintElem);
  }
}

void TinyXMLSerializer::SaveChamfer(XMLDocument &doc, XMLElement *element,
                                    const std::shared_ptr<CChamfer> &chamfer) {
  element->SetAttribute("Mode", ChamferModeToString(chamfer->mode).c_str());
  if (chamfer->firstEndFaceMarker.has_value()) {
    SavePoint3D(element, "FirstEndFaceMarker", *chamfer->firstEndFaceMarker);
  }

  XMLElement *paramsElem = doc.NewElement("Parameters");
  element->InsertEndChild(paramsElem);
  switch (chamfer->mode) {
  case ChamferMode::EQUAL_DISTANCE:
    if (chamfer->params.distance1.has_value()) {
      paramsElem->SetAttribute("Distance1", *chamfer->params.distance1);
    }
    break;
  case ChamferMode::TWO_DISTANCES:
    if (chamfer->params.distance1.has_value()) {
      paramsElem->SetAttribute("Distance1", *chamfer->params.distance1);
    }
    if (chamfer->params.distance2.has_value()) {
      paramsElem->SetAttribute("Distance2", *chamfer->params.distance2);
    }
    break;
  case ChamferMode::TWO_OFFSETS:
    if (chamfer->params.offset1.has_value()) {
      paramsElem->SetAttribute("Offset1", *chamfer->params.offset1);
    }
    if (chamfer->params.offset2.has_value()) {
      paramsElem->SetAttribute("Offset2", *chamfer->params.offset2);
    }
    break;
  case ChamferMode::DISTANCE_ANGLE:
    if (chamfer->params.distance1.has_value()) {
      paramsElem->SetAttribute("Distance1", *chamfer->params.distance1);
    }
    if (chamfer->params.angle.has_value()) {
      paramsElem->SetAttribute("Angle", *chamfer->params.angle);
    }
    break;
  case ChamferMode::VERTEX_3DISTANCES:
    if (chamfer->params.distance1.has_value()) {
      paramsElem->SetAttribute("Distance1", *chamfer->params.distance1);
    }
    if (chamfer->params.distance2.has_value()) {
      paramsElem->SetAttribute("Distance2", *chamfer->params.distance2);
    }
    if (chamfer->params.distance3.has_value()) {
      paramsElem->SetAttribute("Distance3", *chamfer->params.distance3);
    }
    break;
  case ChamferMode::UNKNOWN:
  default:
    if (chamfer->params.distance1.has_value()) {
      paramsElem->SetAttribute("Distance1", *chamfer->params.distance1);
    }
    if (chamfer->params.distance2.has_value()) {
      paramsElem->SetAttribute("Distance2", *chamfer->params.distance2);
    }
    if (chamfer->params.distance3.has_value()) {
      paramsElem->SetAttribute("Distance3", *chamfer->params.distance3);
    }
    if (chamfer->params.offset1.has_value()) {
      paramsElem->SetAttribute("Offset1", *chamfer->params.offset1);
    }
    if (chamfer->params.offset2.has_value()) {
      paramsElem->SetAttribute("Offset2", *chamfer->params.offset2);
    }
    if (chamfer->params.angle.has_value()) {
      paramsElem->SetAttribute("Angle", *chamfer->params.angle);
    }
    break;
  }

  XMLElement *refsElem = doc.NewElement("References");
  element->InsertEndChild(refsElem);
  for (const auto &ref : chamfer->references) {
    SaveRefEntity(doc, refsElem, "ReferenceEntity", ref);
  }
}

void TinyXMLSerializer::SaveFillet(XMLDocument &doc, XMLElement *element,
                                   const std::shared_ptr<CFillet> &fillet) {
  std::fprintf(stderr,
               "[TinyXMLSerializer] SaveFillet begin id=%s mode=%d refMode=%d refs=%zu radiusPoints=%zu\n",
               fillet->featureID.c_str(), static_cast<int>(fillet->mode),
               static_cast<int>(fillet->referenceMode), fillet->references.size(),
               fillet->params.radiusPoints.size());
  element->SetAttribute("Mode", FilletModeToString(fillet->mode).c_str());
  if (fillet->referenceMode != FilletReferenceMode::UNKNOWN &&
      fillet->referenceMode != FilletReferenceMode::EDGE_CHAIN) {
    element->SetAttribute(
        "ReferenceMode",
        FilletReferenceModeToString(fillet->referenceMode).c_str());
  }
  if (fillet->params.secondValue.has_value() &&
      fillet->firstEndFaceMarker.has_value()) {
    SavePoint3D(element, "FirstEndFaceMarker",
                *fillet->firstEndFaceMarker);
  }
  const bool emitDefaultRadius = fillet->mode != FilletMode::VARIABLE_RADIUS;

  XMLElement *paramsElem = doc.NewElement("Parameters");
  paramsElem->SetAttribute("DriveType",
                           FilletDriveTypeToString(fillet->params.driveType).c_str());
  if (fillet->params.primaryValue.has_value()) {
    paramsElem->SetAttribute("PrimaryValue", *fillet->params.primaryValue);
  }
  if (fillet->params.secondValue.has_value()) {
    paramsElem->SetAttribute("SecondValue", *fillet->params.secondValue);
  }
  if (fillet->params.crossSection != FilletCrossSection::UNKNOWN) {
    paramsElem->SetAttribute(
        "CrossSection",
        FilletCrossSectionToString(fillet->params.crossSection).c_str());
  }
  const bool emitConicValue =
      fillet->params.crossSection == FilletCrossSection::CONIC ||
      fillet->params.crossSection ==
          FilletCrossSection::CURVATURE_CONTINUOUS;
  if (emitConicValue &&
      fillet->params.conicValueMode != FilletConicValueMode::NONE) {
    paramsElem->SetAttribute(
        "ConicValueMode",
        FilletConicValueModeToString(fillet->params.conicValueMode).c_str());
  }
  if (emitConicValue && fillet->params.conicValue.has_value()) {
    paramsElem->SetAttribute("ConicValue", *fillet->params.conicValue);
  }
  element->InsertEndChild(paramsElem);

  if (fillet->mode == FilletMode::VARIABLE_RADIUS) {
    XMLElement *radiusPointsElem = doc.NewElement("RadiusPoints");
    for (const auto &point : fillet->params.radiusPoints) {
      XMLElement *pointElem = doc.NewElement("RadiusPoint");
      pointElem->SetAttribute("Position", point.position);
      if (point.primaryValue.has_value()) {
        pointElem->SetAttribute("PrimaryValue", *point.primaryValue);
      }
      if (point.secondValue.has_value()) {
        pointElem->SetAttribute("SecondValue", *point.secondValue);
      }
      if (point.edgeRef) {
        SaveRefEntity(doc, pointElem, "ReferenceEntity", point.edgeRef);
      }
      radiusPointsElem->InsertEndChild(pointElem);
    }
    if (radiusPointsElem->FirstChildElement("RadiusPoint") != nullptr) {
      element->InsertEndChild(radiusPointsElem);
    } else {
      doc.DeleteNode(radiusPointsElem);
    }
  }

  XMLElement *refsElem = doc.NewElement("References");
  for (const auto &ref : fillet->references) {
    SaveRefEntity(doc, refsElem, "ReferenceEntity", ref);
  }
  if (refsElem->FirstChildElement("ReferenceEntity") != nullptr) {
    element->InsertEndChild(refsElem);
  } else {
    doc.DeleteNode(refsElem);
  }

  auto saveFaceGroup = [&](const char *name,
                           const std::vector<std::shared_ptr<CRefFace>> &faces) {
    XMLElement *groupElem = doc.NewElement(name);
    for (const auto &face : faces) {
      SaveRefEntity(doc, groupElem, "ReferenceEntity", face);
    }
    if (groupElem->FirstChildElement("ReferenceEntity") != nullptr) {
      element->InsertEndChild(groupElem);
    } else {
      doc.DeleteNode(groupElem);
    }
  };
  saveFaceGroup("Side1Faces", fillet->side1Faces);
  saveFaceGroup("Side2Faces", fillet->side2Faces);
  saveFaceGroup("CenterFaces", fillet->centerFaces);
  std::fprintf(stderr, "[TinyXMLSerializer] SaveFillet done id=%s\n",
               fillet->featureID.c_str());
}

// =================================================================================================
// Load Implementation
// =================================================================================================

bool TinyXMLSerializer::Load(UnifiedModel &model,
                             const std::filesystem::path &filePath,
                             std::string *errorMessage) {
  XMLDocument doc;
  XMLError result = doc.LoadFile(filePath.string().c_str());
  if (result != XML_SUCCESS) {
    if (errorMessage)
      *errorMessage = doc.ErrorStr();
    return false;
  }

  XMLElement *root = doc.FirstChildElement("UnifiedModel");
  if (!root) {
    if (errorMessage)
      *errorMessage = "Missing UnifiedModel root element";
    return false;
  }

  // SchemaVersion 检查（warn but continue for forward compatibility）
  int schemaVersion = 0;
  if (root->QueryIntAttribute("SchemaVersion", &schemaVersion) != XML_SUCCESS) {
    std::cerr << "[TinyXMLSerializer][WARN] Missing SchemaVersion attribute — "
                 "file may have been created by an older version.\n";
  } else if (schemaVersion != 1) {
    std::cerr << "[TinyXMLSerializer][WARN] SchemaVersion=" << schemaVersion
              << " (expected 1) — compatibility not guaranteed.\n";
  }

  const char *unitText = root->Attribute("UnitSystem");
  if (auto unitOpt = UnitTypeFromString(unitText)) {
    model.unit = *unitOpt;
  }

  const char *name = root->Attribute("ModelName");
  if (name)
    model.modelName = name;

  model.Clear();

  XMLElement *featElem = root->FirstChildElement("Feature");
  while (featElem) {
    auto feature = LoadFeature(featElem);
    if (feature) {
      model.AddFeature(feature);
    } else {
      // 严格模式：记录跳过原因（Type 未知或 ID 缺失）
      const char *typeStr = featElem->Attribute("Type");
      const char *idStr   = featElem->Attribute("ID");
      std::cerr << "[TinyXMLSerializer][WARN] Skipped Feature"
                << " Type=" << (typeStr ? typeStr : "<missing>")
                << " ID="   << (idStr   ? idStr   : "<missing>")
                << " — unknown type or missing ID.\n";
    }
    featElem = featElem->NextSiblingElement("Feature");
  }

  return true;
}

CPoint3D TinyXMLSerializer::LoadPoint3D(XMLElement *element, const char *name) {
  CPoint3D pt;
  double x, y, z;
  if (TryParseTriple(element->Attribute(name), x, y, z)) {
    pt.x = x;
    pt.y = y;
    pt.z = z;
  }
  return pt;
}

CVector3D TinyXMLSerializer::LoadVector3D(XMLElement *element,
                                          const char *name) {
  CVector3D vec;
  double x, y, z;
  if (TryParseTriple(element->Attribute(name), x, y, z)) {
    vec.x = x;
    vec.y = y;
    vec.z = z;
  }
  return vec;
}

std::shared_ptr<CRefEntityBase>
TinyXMLSerializer::LoadRefEntity(XMLElement *element) {
  if (!element)
    return nullptr;

  const char *typeStr = element->Attribute("Type");
  if (!typeStr)
    return nullptr;
  std::string normalizedType = ToLower(typeStr);
  std::shared_ptr<CRefEntityBase> ref;
  std::optional<RefType> resolvedType = RefTypeFromString(typeStr);

  if (resolvedType) {
    if (auto entry = FindRefEntry(*resolvedType)) {
      ref = entry->load(element);
    } else if (*resolvedType == RefType::FEATURE_DATUM_AXIS ||
               *resolvedType == RefType::FEATURE_DATUM_POINT) {
      ref = LoadFeatureReference(element, *resolvedType);
    }
  }

  if (!ref && normalizedType == "feature")
    ref = LoadFeatureReference(element, RefType::FEATURE_DATUM_PLANE);

  if (ref && resolvedType)
    ref->refType = *resolvedType;
  return ref;
}

std::shared_ptr<CFeatureBase>
TinyXMLSerializer::LoadFeature(XMLElement *element) {
  const char *typeStr = element->Attribute("Type");
  if (!typeStr)
    return nullptr; // 无 Type，调用方会打印 warn
  std::string type = typeStr;

  std::shared_ptr<CFeatureBase> feature;

  if (type == "Sketch") {
    auto sketch = std::make_shared<CSketch>();
    LoadSketch(element, sketch);
    feature = sketch;
  } else if (type == "Extrude") {
    auto extrude = std::make_shared<CExtrude>();
    LoadExtrude(element, extrude);
    feature = extrude;
  } else if (type == "Revolve") {
    auto revolve = std::make_shared<CRevolve>();
    LoadRevolve(element, revolve);
    feature = revolve;
  } else if (type == "Sweep") {
    auto sweep = std::make_shared<CSweep>();
    LoadSweep(element, sweep);
    feature = sweep;
  } else if (type == "Fillet") {
    auto fillet = std::make_shared<CFillet>();
    LoadFillet(element, fillet);
    feature = fillet;
  } else if (type == "Chamfer") {
    auto chamfer = std::make_shared<CChamfer>();
    LoadChamfer(element, chamfer);
    feature = chamfer;
  } else if (type == "DatumPlane") {
    auto datumPlane = std::make_shared<CDatumPlane>();
    LoadDatumPlane(element, datumPlane);
    feature = datumPlane;
  } else {
    return nullptr; // 未知 Type，调用方会打印 warn
  }

  if (feature) {
    const char *id = element->Attribute("ID");
    if (!id || std::string(id).empty()) {
      // 严格模式：ID 缺失或为空，丢弃该特征
      std::cerr << "[TinyXMLSerializer][WARN] Feature Type=" << type
                << " has missing or empty ID — skipped.\n";
      return nullptr;
    }
    feature->featureID = id;
    const char *name = element->Attribute("Name");
    if (name)
      feature->featureName = name;
    bool suppressed = false;
    element->QueryBoolAttribute("Suppressed", &suppressed);
    feature->isSuppressed = suppressed;
  }
  return feature;
}

void TinyXMLSerializer::LoadSketch(XMLElement *element,
                                   std::shared_ptr<CSketch> &sketch) {
  sketch->referencePlane =
      LoadRefEntity(element->FirstChildElement("ReferencePlane"));

  XMLElement *csysElem = element->FirstChildElement("LocalCSys");
  if (csysElem) {
    sketch->sketchCSys.origin = LoadPoint3D(csysElem, "Origin");
    sketch->sketchCSys.xDir = LoadVector3D(csysElem, "XDir");
    sketch->sketchCSys.yDir = LoadVector3D(csysElem, "YDir");
    sketch->sketchCSys.zDir = LoadVector3D(csysElem, "ZDir");
  }

  XMLElement *segsElem = element->FirstChildElement("Segments");
  if (segsElem) {
    XMLElement *segElem = segsElem->FirstChildElement("Segment");
    while (segElem) {
      auto seg = LoadSketchSeg(segElem);
      if (seg)
        sketch->segments.push_back(seg);
      segElem = segElem->NextSiblingElement("Segment");
    }
  }

  XMLElement *consElem = element->FirstChildElement("Constraints");
  if (consElem) {
    XMLElement *conElem = consElem->FirstChildElement("Constraint");
    while (conElem) {
      sketch->constraints.push_back(LoadConstraint(conElem));
      conElem = conElem->NextSiblingElement("Constraint");
    }
  }
}

std::shared_ptr<CSketchSeg>
TinyXMLSerializer::LoadSketchSeg(XMLElement *element) {
  const char *typeStr = element->Attribute("Type");
  if (!typeStr)
    return nullptr;
  std::string type = typeStr;

  std::shared_ptr<CSketchSeg> seg;
  if (type == "Line") {
    auto line = std::make_shared<CSketchLine>();
    line->startPos = LoadPoint3D(element, "Start");
    line->endPos = LoadPoint3D(element, "End");
    seg = line;
  } else if (type == "Circle") {
    auto circle = std::make_shared<CSketchCircle>();
    circle->center = LoadPoint3D(element, "Center");
    element->QueryDoubleAttribute("Radius", &circle->radius);
    seg = circle;
  } else if(type == "Arc") {
    auto arc = std::make_shared<CSketchArc>();
    arc->center = LoadPoint3D(element, "Center");
    element->QueryDoubleAttribute("Radius", &arc->radius);
    element->QueryDoubleAttribute("StartAngle", &arc->startAngle);
    element->QueryDoubleAttribute("EndAngle", &arc->endAngle);
    bool clockwise = false;
    element->QueryBoolAttribute("Clockwise", &clockwise);
    arc->isClockwise = clockwise;
    seg = arc;
  } else if (type == "Point") {
    auto pt = std::make_shared<CSketchPoint>();
    pt->position = LoadPoint3D(element, "Position");
    seg = pt;
  }
  // ... others

  if (seg) {
    const char *lid = element->Attribute("LocalID");
    if (lid)
      seg->localID = lid;
    bool cons = false;
    element->QueryBoolAttribute("Construction", &cons);
    seg->isConstruction = cons;
  }
  return seg;
}

CSketchConstraint TinyXMLSerializer::LoadConstraint(XMLElement *element) {
  CSketchConstraint con;
  con.type = ConstraintTypeFromString(element->Attribute("Type"));

  double value = 0.0;
  if (element->QueryDoubleAttribute("Value", &value) == XML_SUCCESS) {
    con.value = value;
  } else if (element->QueryDoubleAttribute("Dimension", &value) == XML_SUCCESS) {
    // Backward compatibility with legacy dimensional constraints.
    con.value = value;
  }

  if (XMLElement *refsElem = element->FirstChildElement("Refs")) {
    for (XMLElement *refElem = refsElem->FirstChildElement("Ref"); refElem;
         refElem = refElem->NextSiblingElement("Ref")) {
      SketchConstraintRef ref;
      ref.kind = SketchConstraintRefKindFromString(refElem->Attribute("Kind"));
      ref.subEntity =
          SketchConstraintSubEntityFromString(refElem->Attribute("SubEntity"));
      if (ref.kind == SketchConstraintRefKind::SketchEntity) {
        if (const char *localID = refElem->Attribute("SketchEntityLocalID")) {
          ref.sketchEntityLocalID = localID;
        }
      } else if (XMLElement *refEntityElem =
                     refElem->FirstChildElement("ReferenceEntity")) {
        ref.refEntity = LoadRefEntity(refEntityElem);
      }
      con.refs.push_back(std::move(ref));
    }
  } else {
    const char *ents = element->Attribute("Entities");
    if (ents) {
      std::stringstream ss(ents);
      std::string item;
      while (std::getline(ss, item, ',')) {
        con.refs.push_back(SketchConstraintRef::ForSketchEntity(item));
      }
    }
  }
  return con;
}

void TinyXMLSerializer::LoadExtrude(XMLElement *element,
                                    std::shared_ptr<CExtrude> &extrude) {
  // ProfileSketchID is now a direct attribute (changed from child element in P3).
  if (const char *v = element->Attribute("ProfileSketchID"))
    extrude->profileSketchID = v;

  XMLElement *directionElem = element->FirstChildElement("Direction");
  if (directionElem) {
    extrude->direction = LoadVector3D(directionElem, "Value");
  }

  if (auto opOpt = BooleanOpFromString(element->Attribute("Operation"))) {
    extrude->operation = *opOpt;
  }

  auto loadExtent = [&](XMLElement *ec, const char *tag) -> SweepExtent {
    SweepExtent extent;
    if (auto t = SweepExtentTypeFromString(ec->Attribute("Type")))
      extent.type = *t;
    else
      std::cerr << "[TinyXMLSerializer][WARN] Extrude '" << extrude->featureID
                << "' " << tag << " has missing or unknown Type attribute.\n";
    ec->QueryDoubleAttribute("Value", &extent.value);
    if (extent.value == 0.0) {
      ec->QueryDoubleAttribute("Depth", &extent.value);
    }
    ec->QueryDoubleAttribute("Offset",           &extent.offset);
    ec->QueryBoolAttribute  ("HasOffset",        &extent.hasOffset);
    ec->QueryBoolAttribute  ("Flip",             &extent.isFlip);
    ec->QueryBoolAttribute  ("FlipMaterialSide", &extent.isFlipMaterialSide);
    if (auto *helper = ec->FirstChildElement("HelperPoint")) {
      extent.helperPoint = LoadPoint3D(helper, "Value");
    }
    if (auto *ref = ec->FirstChildElement("ReferenceEntity"))
      extent.referenceEntity = LoadRefEntity(ref);
    return extent;
  };

  if (auto *ec1 = element->FirstChildElement("Extent1"))
    extrude->extent1 = loadExtent(ec1, "Extent1");
  else if (auto *ec1 = element->FirstChildElement("EndCondition1"))
    extrude->extent1 = loadExtent(ec1, "EndCondition1");

  if (auto *ec2 = element->FirstChildElement("Extent2"))
    extrude->extent2 = loadExtent(ec2, "Extent2");
  else if (auto *ec2 = element->FirstChildElement("EndCondition2"))
    extrude->extent2 = loadExtent(ec2, "EndCondition2");

  // 薄壁参数（可选）
  XMLElement *twElem = element->FirstChildElement("ThinWall");
  if (twElem) {
    ThinWallOption tw;
    twElem->QueryBoolAttribute("Covered",     &tw.isCovered);
    twElem->QueryDoubleAttribute("StartOffset", &tw.startOffset);
    twElem->QueryDoubleAttribute("EndOffset", &tw.endOffset);
    if (std::fabs(tw.startOffset) > 1e-9 || std::fabs(tw.endOffset) > 1e-9) {
      extrude->thinWall = tw;
    } else {
      std::cerr << "[TinyXMLSerializer][WARN] Extrude '" << extrude->featureID
                << "' ThinWall.StartOffset/EndOffset are both zero or missing — skipping.\n";
    }
  }
}

void TinyXMLSerializer::LoadRevolve(XMLElement *element,
                                    std::shared_ptr<CRevolve> &revolve) {
  if (const char *v = element->Attribute("ProfileSketchID"))
    revolve->profileSketchID = v;

  if (auto opOpt = BooleanOpFromString(element->Attribute("Operation"))) {
    revolve->operation = *opOpt;
  }

  auto loadExtent = [&](XMLElement *ec, const char *tag) -> SweepExtent {
    SweepExtent extent;
    if (auto t = SweepExtentTypeFromString(ec->Attribute("Type")))
      extent.type = *t;
    else
      std::cerr << "[TinyXMLSerializer][WARN] Revolve '" << revolve->featureID
                << "' " << tag << " has missing or unknown Type attribute.\n";
    ec->QueryDoubleAttribute("Value", &extent.value);
    ec->QueryDoubleAttribute("Offset",           &extent.offset);
    ec->QueryBoolAttribute  ("HasOffset",        &extent.hasOffset);
    ec->QueryBoolAttribute  ("Flip",             &extent.isFlip);
    ec->QueryBoolAttribute  ("FlipMaterialSide", &extent.isFlipMaterialSide);
    if (auto *helper = ec->FirstChildElement("HelperPoint")) {
      extent.helperPoint = LoadPoint3D(helper, "Value");
    }
    if (auto *ref = ec->FirstChildElement("ReferenceEntity"))
      extent.referenceEntity = LoadRefEntity(ref);
    return extent;
  };

  if (auto *e1 = element->FirstChildElement("Extent1")) {
    revolve->extent1 = loadExtent(e1, "Extent1");
  }

  if (auto *e2 = element->FirstChildElement("Extent2")) {
    revolve->extent2 = loadExtent(e2, "Extent2");
  }

  XMLElement *axisElem = element->FirstChildElement("Axis");
  if (axisElem) {
    if (const char *id = axisElem->Attribute("RefLocalID"))
      revolve->axis.referenceLocalID = id;
    revolve->axis.origin = LoadPoint3D(axisElem, "Origin");
    revolve->axis.direction = LoadVector3D(axisElem, "Direction");
    if (auto *ref = axisElem->FirstChildElement("ReferenceEntity")) {
      auto loaded_axis_ref = LoadRefEntity(ref);
      if (auto sketch_seg =
              std::dynamic_pointer_cast<CRefSketchSeg>(loaded_axis_ref)) {
        if (revolve->axis.referenceLocalID.empty()) {
          revolve->axis.referenceLocalID = sketch_seg->segmentLocalID;
        }
        if (revolve->profileSketchID.empty()) {
          revolve->profileSketchID = sketch_seg->parentFeatureID;
        }
      }
      // Keep the explicit axis ReferenceEntity so cross-sketch axis bindings
      // are not collapsed into profileSketchID + RefLocalID.
      revolve->axis.referenceEntity = std::move(loaded_axis_ref);
    }
  }

  XMLElement *twElem = element->FirstChildElement("ThinWall");
  if (twElem) {
    ThinWallOption tw;
    twElem->QueryBoolAttribute("Covered",     &tw.isCovered);
    twElem->QueryDoubleAttribute("StartOffset", &tw.startOffset);
    twElem->QueryDoubleAttribute("EndOffset", &tw.endOffset);
    if (std::fabs(tw.startOffset) > 1e-9 || std::fabs(tw.endOffset) > 1e-9) {
      revolve->thinWall = tw;
    } else {
      std::cerr << "[TinyXMLSerializer][WARN] Revolve '" << revolve->featureID
                << "' ThinWall.StartOffset/EndOffset are both zero or missing — skipping.\n";
    }
  }
}

void TinyXMLSerializer::LoadSweep(XMLElement *element,
                                  std::shared_ptr<CSweep> &sweep) {
  if (const char *v = element->Attribute("ProfileSketchID")) {
    sweep->profileSketchID = v;
    sweep->profile.kind = SweepProfileKind::SketchReference;
    sweep->profile.sketchID = v;
  }

  if (auto opOpt = BooleanOpFromString(element->Attribute("Operation"))) {
    sweep->operation = *opOpt;
  }
  if (auto orientation =
          SweepPathOrientationFromString(element->Attribute("Orientation"))) {
    sweep->orientation = *orientation;
  }
  if (auto placement =
          SweepSectionPlacementFromString(element->Attribute("SectionPlacement"))) {
    sweep->sectionPlacement = *placement;
  }
  double angleCos = 0.0;
  if (element->QueryDoubleAttribute("ProfilePathAngleCos", &angleCos) ==
      XML_SUCCESS) {
    sweep->profilePathAngleCos = angleCos;
  }

  if (auto *profileElem = element->FirstChildElement("Profile")) {
    if (auto kind = SweepProfileKindFromString(profileElem->Attribute("Kind"))) {
      sweep->profile.kind = *kind;
    }
    switch (sweep->profile.kind) {
    case SweepProfileKind::SketchReference:
      if (const char *id = profileElem->Attribute("SketchID")) {
        sweep->profile.sketchID = id;
        sweep->profileSketchID = id;
      }
      break;
    case SweepProfileKind::EmbeddedSketch:
      if (auto *sketchElem = profileElem->FirstChildElement("EmbeddedSketch")) {
        auto embeddedSketch = std::make_shared<CSketch>();
        LoadSketch(sketchElem, embeddedSketch);
        CSweepEmbeddedProfile embedded;
        embedded.sketch = *embeddedSketch;
        sweep->profile.embedded = embedded;
      }
      break;
    case SweepProfileKind::Circular: {
      CSweepCircularProfile circular;
      profileElem->QueryDoubleAttribute("OuterRadius", &circular.outerRadius);
      profileElem->QueryDoubleAttribute("InnerRadius", &circular.innerRadius);
      sweep->profile.circular = circular;
      break;
    }
    }
  }

  if (auto *pathElem = element->FirstChildElement("Path")) {
    bool closed = false;
    pathElem->QueryBoolAttribute("Closed", &closed);
    sweep->path.isClosed = closed;

    if (auto *startElem = pathElem->FirstChildElement("StartPoint")) {
      sweep->path.startPoint = LoadPoint3D(startElem, "Value");
    }
    if (auto *endElem = pathElem->FirstChildElement("EndPoint")) {
      sweep->path.endPoint = LoadPoint3D(endElem, "Value");
    }

    for (auto *refElem = pathElem->FirstChildElement("ReferenceEntity");
         refElem != nullptr;
         refElem = refElem->NextSiblingElement("ReferenceEntity")) {
      sweep->path.references.push_back(LoadRefEntity(refElem));
    }
  }

  if (auto *guidesElem = element->FirstChildElement("GuidePaths")) {
    for (auto *guideElem = guidesElem->FirstChildElement("GuidePath");
         guideElem != nullptr;
         guideElem = guideElem->NextSiblingElement("GuidePath")) {
      CSweepPath guidePath;
      bool closed = false;
      guideElem->QueryBoolAttribute("Closed", &closed);
      guidePath.isClosed = closed;
      if (auto *startElem = guideElem->FirstChildElement("StartPoint")) {
        guidePath.startPoint = LoadPoint3D(startElem, "Value");
      }
      if (auto *endElem = guideElem->FirstChildElement("EndPoint")) {
        guidePath.endPoint = LoadPoint3D(endElem, "Value");
      }
      for (auto *refElem = guideElem->FirstChildElement("ReferenceEntity");
           refElem != nullptr;
           refElem = refElem->NextSiblingElement("ReferenceEntity")) {
        guidePath.references.push_back(LoadRefEntity(refElem));
      }
      sweep->guidePaths.push_back(std::move(guidePath));
    }
  }

  XMLElement *twElem = element->FirstChildElement("ThinWall");
  if (twElem) {
    ThinWallOption tw;
    twElem->QueryBoolAttribute("Covered", &tw.isCovered);
    twElem->QueryDoubleAttribute("StartOffset", &tw.startOffset);
    twElem->QueryDoubleAttribute("EndOffset", &tw.endOffset);
    if (std::fabs(tw.startOffset) > 1e-9 ||
        std::fabs(tw.endOffset) > 1e-9) {
      sweep->thinWall = tw;
    } else {
      std::cerr << "[TinyXMLSerializer][WARN] Sweep '" << sweep->featureID
                << "' ThinWall.StartOffset/EndOffset are both zero or missing -- skipping.\n";
    }
  }
}

void TinyXMLSerializer::LoadDatumPlane(
    XMLElement *element, std::shared_ptr<CDatumPlane> &datumPlane) {
  if (auto method = PlaneMethodFromString(element->Attribute("Method"))) {
    datumPlane->method = *method;
  }

  if (XMLElement *refsElem = element->FirstChildElement("ReferenceEntities")) {
    XMLElement *refElem = refsElem->FirstChildElement("ReferenceEntity");
    while (refElem) {
      auto ref = LoadRefEntity(refElem);
      if (ref) {
        datumPlane->referenceEntities.push_back(ref);
      }
      refElem = refElem->NextSiblingElement("ReferenceEntity");
    }
  }

  if (XMLElement *constraintsElem = element->FirstChildElement("Constraints")) {
    XMLElement *constraintElem = constraintsElem->FirstChildElement("Constraint");
    while (constraintElem) {
      PlaneConstraint constraint{};
      if (auto t = PlaneConstraintTypeFromString(constraintElem->Attribute("Type"))) {
        constraint.type = *t;
      }
      constraintElem->QueryIntAttribute("Ref", &constraint.ref);
      constraintElem->QueryDoubleAttribute("Value", &constraint.value);
      constraintElem->QueryBoolAttribute("Reversed", &constraint.reversed);
      if (constraintElem->Attribute("DefaultDir")) {
        constraint.defaultDir = LoadVector3D(constraintElem, "DefaultDir");
      }
      datumPlane->constraints.push_back(constraint);
      constraintElem = constraintElem->NextSiblingElement("Constraint");
    }
  }
}

void TinyXMLSerializer::LoadChamfer(XMLElement *element,
                                    std::shared_ptr<CChamfer> &chamfer) {
  if (auto mode = ChamferModeFromString(element->Attribute("Mode"))) {
    chamfer->mode = *mode;
  }
  if (element->Attribute("FirstEndFaceMarker")) {
    chamfer->firstEndFaceMarker =
        LoadPoint3D(element, "FirstEndFaceMarker");
  }

  if (XMLElement *paramsElem = element->FirstChildElement("Parameters")) {
    double value = 0.0;
    if (paramsElem->QueryDoubleAttribute("Distance1", &value) == XML_SUCCESS) {
      chamfer->params.distance1 = value;
    }
    if (paramsElem->QueryDoubleAttribute("Distance2", &value) == XML_SUCCESS) {
      chamfer->params.distance2 = value;
    }
    if (paramsElem->QueryDoubleAttribute("Distance3", &value) == XML_SUCCESS) {
      chamfer->params.distance3 = value;
    }
    if (paramsElem->QueryDoubleAttribute("Offset1", &value) == XML_SUCCESS) {
      chamfer->params.offset1 = value;
    }
    if (paramsElem->QueryDoubleAttribute("Offset2", &value) == XML_SUCCESS) {
      chamfer->params.offset2 = value;
    }
    if (paramsElem->QueryDoubleAttribute("Angle", &value) == XML_SUCCESS) {
      chamfer->params.angle = value;
    }
  }

  if (XMLElement *refsElem = element->FirstChildElement("References")) {
    XMLElement *refElem = refsElem->FirstChildElement("ReferenceEntity");
    while (refElem) {
      auto ref = LoadRefEntity(refElem);
      if (ref) {
        chamfer->references.push_back(ref);
      }
      refElem = refElem->NextSiblingElement("ReferenceEntity");
    }
  }
}

void TinyXMLSerializer::LoadFillet(XMLElement *element,
                                   std::shared_ptr<CFillet> &fillet) {
  int intValue = 0;
  if (auto mode = FilletModeFromString(element->Attribute("Mode"))) {
    fillet->mode = *mode;
  } else if (element->QueryIntAttribute("Mode", &intValue) == XML_SUCCESS) {
    fillet->mode = static_cast<FilletMode>(intValue);
  }
  if (auto referenceMode =
          FilletReferenceModeFromString(element->Attribute("ReferenceMode"))) {
    fillet->referenceMode = *referenceMode;
  } else if (element->QueryIntAttribute("ReferenceMode", &intValue) ==
             XML_SUCCESS) {
    fillet->referenceMode = static_cast<FilletReferenceMode>(intValue);
  }
  if (element->Attribute("FirstEndFaceMarker")) {
    fillet->firstEndFaceMarker =
        LoadPoint3D(element, "FirstEndFaceMarker");
  }

  if (XMLElement *paramsElem = element->FirstChildElement("Parameters")) {
    if (auto driveType =
            FilletDriveTypeFromString(paramsElem->Attribute("DriveType"))) {
      fillet->params.driveType = *driveType;
    } else if (fillet->mode == FilletMode::CHORDAL) {
      fillet->params.driveType = FilletDriveType::SINGLE_DISTANCE;
    }
    if (auto crossSection =
            FilletCrossSectionFromString(paramsElem->Attribute("CrossSection"))) {
      fillet->params.crossSection = *crossSection;
    } else if (paramsElem->QueryIntAttribute("CrossSection", &intValue) ==
               XML_SUCCESS) {
      fillet->params.crossSection = static_cast<FilletCrossSection>(intValue);
    }
    paramsElem->QueryBoolAttribute("TangentPropagation",
                                   &fillet->params.tangentPropagation);
    if (auto conicValueMode =
            FilletConicValueModeFromString(paramsElem->Attribute("ConicValueMode"))) {
      fillet->params.conicValueMode = *conicValueMode;
    } else if (paramsElem->QueryIntAttribute("ConicValueMode", &intValue) ==
               XML_SUCCESS) {
      fillet->params.conicValueMode =
          static_cast<FilletConicValueMode>(intValue);
    }
    double doubleValue = 0.0;
    if (paramsElem->QueryDoubleAttribute("PrimaryValue", &doubleValue) ==
        XML_SUCCESS) {
      fillet->params.primaryValue = doubleValue;
    }
    if (paramsElem->QueryDoubleAttribute("SecondValue", &doubleValue) ==
        XML_SUCCESS) {
      fillet->params.secondValue = doubleValue;
      if (fillet->params.driveType == FilletDriveType::SINGLE_DISTANCE) {
        fillet->params.driveType = FilletDriveType::TWO_DISTANCES;
      }
    }
    if (paramsElem->QueryDoubleAttribute("DefaultRadius", &doubleValue) ==
        XML_SUCCESS) {
      if (!fillet->params.primaryValue.has_value()) {
        fillet->params.primaryValue = doubleValue;
      }
    }
    if (paramsElem->QueryDoubleAttribute("DefaultDistance", &doubleValue) ==
        XML_SUCCESS) {
      if (!fillet->params.primaryValue.has_value()) {
        fillet->params.primaryValue = doubleValue;
      }
    }
    if (paramsElem->QueryDoubleAttribute("DefaultDistance2", &doubleValue) ==
        XML_SUCCESS) {
      if (!fillet->params.secondValue.has_value()) {
        fillet->params.secondValue = doubleValue;
      }
      if (fillet->params.driveType == FilletDriveType::SINGLE_DISTANCE) {
        fillet->params.driveType = FilletDriveType::TWO_DISTANCES;
      }
    } else if (paramsElem->QueryDoubleAttribute("DefaultRadius2", &doubleValue) ==
               XML_SUCCESS) {
      if (!fillet->params.secondValue.has_value()) {
        fillet->params.secondValue = doubleValue;
      }
      if (fillet->params.driveType == FilletDriveType::SINGLE_DISTANCE) {
        fillet->params.driveType = FilletDriveType::TWO_DISTANCES;
      }
    }
    if (fillet->params.driveType == FilletDriveType::UNKNOWN) {
      if (fillet->params.secondValue.has_value()) {
        fillet->params.driveType = FilletDriveType::TWO_DISTANCES;
      } else if (fillet->params.primaryValue.has_value()) {
        fillet->params.driveType = FilletDriveType::SINGLE_DISTANCE;
      }
    }
    if (paramsElem->QueryDoubleAttribute("ConicValue", &doubleValue) ==
        XML_SUCCESS) {
      fillet->params.conicValue = doubleValue;
    }
    if (fillet->referenceMode == FilletReferenceMode::UNKNOWN) {
      if (auto legacyReferenceMode = FilletReferenceModeFromString(
              paramsElem->Attribute("ReferenceMode"))) {
        fillet->referenceMode = *legacyReferenceMode;
      } else if (paramsElem->QueryIntAttribute("ReferenceMode", &intValue) ==
                 XML_SUCCESS) {
        fillet->referenceMode = static_cast<FilletReferenceMode>(intValue);
      }
    }
    bool curvatureContinuous = false;
    if (paramsElem->QueryBoolAttribute("CurvatureContinuous",
                                       &curvatureContinuous) == XML_SUCCESS &&
        curvatureContinuous) {
      fillet->params.crossSection =
          FilletCrossSection::CURVATURE_CONTINUOUS;
    }
  }

  if (fillet->referenceMode == FilletReferenceMode::UNKNOWN) {
    if (fillet->mode == FilletMode::FULL_ROUND) {
      fillet->referenceMode = FilletReferenceMode::FULL_ROUND_THREE_FACES;
    } else {
      fillet->referenceMode = FilletReferenceMode::EDGE_CHAIN;
    }
  }

  if (!fillet->params.secondValue.has_value()) {
    fillet->firstEndFaceMarker.reset();
  }

  XMLElement *pointsElem = element->FirstChildElement("RadiusPoints");
  bool legacyRadiusItems = false;
  if (pointsElem == nullptr) {
    pointsElem = element->FirstChildElement("RadiusItems");
    legacyRadiusItems = pointsElem != nullptr;
  }
  if (pointsElem != nullptr) {
    const char *pointTag = legacyRadiusItems ? "RadiusItem" : "RadiusPoint";
    for (XMLElement *pointElem = pointsElem->FirstChildElement(pointTag);
         pointElem != nullptr;
         pointElem = pointElem->NextSiblingElement(pointTag)) {
      CFilletRadiusPoint point;
      double doubleValue = 0.0;
      if (pointElem->QueryDoubleAttribute("Position", &doubleValue) ==
          XML_SUCCESS) {
        point.position = doubleValue;
      }
      if (pointElem->QueryDoubleAttribute("PrimaryValue", &doubleValue) ==
          XML_SUCCESS) {
        point.primaryValue = doubleValue;
      }
      if (pointElem->QueryDoubleAttribute("SecondValue", &doubleValue) ==
          XML_SUCCESS) {
        point.secondValue = doubleValue;
      }
      if (pointElem->QueryDoubleAttribute("Radius1", &doubleValue) ==
          XML_SUCCESS) {
        point.primaryValue = doubleValue;
      }
      if (pointElem->QueryDoubleAttribute("Radius", &doubleValue) ==
          XML_SUCCESS) {
        point.primaryValue = doubleValue;
      }
      if (pointElem->QueryDoubleAttribute("Distance1", &doubleValue) ==
          XML_SUCCESS) {
        point.primaryValue = doubleValue;
      }
      if (pointElem->QueryDoubleAttribute("Distance", &doubleValue) ==
          XML_SUCCESS) {
        if (!point.primaryValue.has_value()) {
          point.primaryValue = doubleValue;
        }
      }
      if (pointElem->QueryDoubleAttribute("Distance2", &doubleValue) ==
          XML_SUCCESS) {
        if (!point.secondValue.has_value()) {
          point.secondValue = doubleValue;
        }
      } else if (pointElem->QueryDoubleAttribute("Radius2", &doubleValue) ==
          XML_SUCCESS) {
        if (!point.secondValue.has_value()) {
          point.secondValue = doubleValue;
        }
      }
      if (auto *edgeElem = pointElem->FirstChildElement("ReferenceEntity")) {
        point.edgeRef =
            std::dynamic_pointer_cast<CRefEdge>(LoadRefEntity(edgeElem));
      }
      fillet->params.radiusPoints.push_back(std::move(point));
    }
  }

  if (XMLElement *refsElem = element->FirstChildElement("References")) {
    for (XMLElement *refElem = refsElem->FirstChildElement("ReferenceEntity");
         refElem != nullptr;
         refElem = refElem->NextSiblingElement("ReferenceEntity")) {
      if (auto ref = LoadRefEntity(refElem)) {
        fillet->references.push_back(ref);
      }
    }
  }

  auto loadFaceGroup = [&](const char *name,
                           std::vector<std::shared_ptr<CRefFace>> &target) {
    if (XMLElement *groupElem = element->FirstChildElement(name)) {
      for (XMLElement *faceElem = groupElem->FirstChildElement("ReferenceEntity");
           faceElem != nullptr; faceElem = faceElem->NextSiblingElement("ReferenceEntity")) {
        auto face = std::dynamic_pointer_cast<CRefFace>(LoadRefEntity(faceElem));
        if (face) {
          target.push_back(std::move(face));
        }
      }
    }
  };
  loadFaceGroup("Side1Faces", fillet->side1Faces);
  loadFaceGroup("Side2Faces", fillet->side2Faces);
  loadFaceGroup("CenterFaces", fillet->centerFaces);

  if (XMLElement *extElem = element->FirstChildElement("VendorExtensions")) {
    extElem->QueryBoolAttribute("SwKeepFeatures", &fillet->swKeepFeatures);
    if (const char *text = extElem->Attribute("SwOverflowType")) {
      fillet->swOverflowType = text;
    }
    if (extElem->QueryIntAttribute("CreoAttachType", &intValue) == XML_SUCCESS) {
      fillet->creoAttachType = intValue;
    }
    if (extElem->QueryIntAttribute("CreoConicDepOption", &intValue) ==
        XML_SUCCESS) {
      fillet->creoConicDepOption = intValue;
    }
  }
}

} // namespace CADExchange

#ifdef _MSC_VER
#pragma warning(pop)
#endif
