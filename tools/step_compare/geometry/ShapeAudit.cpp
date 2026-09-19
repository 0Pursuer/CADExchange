#include "geometry/ShapeAudit.h"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Pnt.hxx>

namespace cadstep {
namespace detail {
namespace {

Point3 ToPoint3(const gp_Pnt &point) {
  return {point.X(), point.Y(), point.Z()};
}

} // namespace

Bounds3 ComputeBounds(const TopoDS_Shape &shape) {
  Bounds3 bounds;
  Bnd_Box box;
  BRepBndLib::AddOptimal(shape, box);
  if (box.IsVoid()) {
    BRepBndLib::Add(shape, box);
  }
  if (box.IsVoid()) {
    bounds.isVoid = true;
    return bounds;
  }

  double xmin = 0.0, ymin = 0.0, zmin = 0.0;
  double xmax = 0.0, ymax = 0.0, zmax = 0.0;
  box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

  bounds.isVoid = false;
  bounds.minimum = {xmin, ymin, zmin};
  bounds.maximum = {xmax, ymax, zmax};
  return bounds;
}

std::string SurfaceTypeName(GeomAbs_SurfaceType type) {
  switch (type) {
  case GeomAbs_Plane:
    return "PLANE";
  case GeomAbs_Cylinder:
    return "CYLINDER";
  case GeomAbs_Cone:
    return "CONE";
  case GeomAbs_Sphere:
    return "SPHERE";
  case GeomAbs_Torus:
    return "TORUS";
  case GeomAbs_BezierSurface:
    return "BEZIER";
  case GeomAbs_BSplineSurface:
    return "BSPLINE";
  case GeomAbs_SurfaceOfRevolution:
    return "REVOLUTION";
  case GeomAbs_SurfaceOfExtrusion:
    return "EXTRUSION";
  case GeomAbs_OffsetSurface:
    return "OFFSET";
  case GeomAbs_OtherSurface:
    return "OTHER";
  }
  return "UNKNOWN";
}

std::string CurveTypeName(GeomAbs_CurveType type) {
  switch (type) {
  case GeomAbs_Line:
    return "LINE";
  case GeomAbs_Circle:
    return "CIRCLE";
  case GeomAbs_Ellipse:
    return "ELLIPSE";
  case GeomAbs_Hyperbola:
    return "HYPERBOLA";
  case GeomAbs_Parabola:
    return "PARABOLA";
  case GeomAbs_BezierCurve:
    return "BEZIER";
  case GeomAbs_BSplineCurve:
    return "BSPLINE";
  case GeomAbs_OffsetCurve:
    return "OFFSET";
  case GeomAbs_OtherCurve:
    return "OTHER";
  }
  return "UNKNOWN";
}

FaceGeometrySummary SummarizeFace(const TopoDS_Face &face,
                                  const BRepAdaptor_Surface &surface) {
  FaceGeometrySummary summary;
  summary.surfaceType = SurfaceTypeName(surface.GetType());

  GProp_GProps props;
  BRepGProp::SurfaceProperties(face, props);
  summary.areaMm2 = props.Mass();
  summary.centroidMm = ToPoint3(props.CentreOfMass());
  summary.boundsMm = ComputeBounds(face);
  return summary;
}

EdgeGeometrySummary SummarizeEdge(const TopoDS_Edge &edge,
                                  const BRepAdaptor_Curve &curve) {
  EdgeGeometrySummary summary;
  summary.curveType = CurveTypeName(curve.GetType());

  GProp_GProps props;
  BRepGProp::LinearProperties(edge, props);
  summary.lengthMm = props.Mass();
  summary.centroidMm = ToPoint3(props.CentreOfMass());
  summary.boundsMm = ComputeBounds(edge);
  summary.closed = (BRep_Tool::IsClosed(edge) != 0);
  return summary;
}

int FindShapeIndex(const TopTools_IndexedMapOfShape &shapeMap,
                   const TopoDS_Shape &target) {
  for (int i = 1; i <= shapeMap.Extent(); ++i) {
    if (shapeMap(i).IsSame(target)) {
      return i;
    }
  }
  return 0;
}

int CountSubShapes(const TopoDS_Shape &shape, TopAbs_ShapeEnum type) {
  int count = 0;
  for (TopExp_Explorer explorer(shape, type); explorer.More();
       explorer.Next()) {
    ++count;
  }
  return count;
}

int CountUniqueSubShapes(const TopoDS_Shape &shape, TopAbs_ShapeEnum type) {
  TopTools_IndexedMapOfShape shapeMap;
  TopExp::MapShapes(shape, type, shapeMap);
  return shapeMap.Extent();
}

} // namespace detail
} // namespace cadstep
