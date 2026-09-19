#pragma once

#include "StepCompare.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <string>

namespace cadstep {
namespace detail {

// Optimal (tight) axis-aligned bounds of a shape, falling back to the coarse
// box when the optimal one is void.
Bounds3 ComputeBounds(const TopoDS_Shape &shape);

// Index of `target` in `shapeMap`, or 0 when absent.
int FindShapeIndex(const TopTools_IndexedMapOfShape &shapeMap,
                   const TopoDS_Shape &target);

// Counts every sub-shape occurrence / unique sub-shapes of the given type.
// (Unique counting is what makes a "face count" meaningful on a shape that
// shares edges between faces.)
int CountSubShapes(const TopoDS_Shape &shape, TopAbs_ShapeEnum type);
int CountUniqueSubShapes(const TopoDS_Shape &shape, TopAbs_ShapeEnum type);

std::string SurfaceTypeName(GeomAbs_SurfaceType type);
std::string CurveTypeName(GeomAbs_CurveType type);

// Per-entity geometry descriptors. These are the values the audit trail, the
// descriptor matcher and the VTP writer all need; they used to be recomputed
// independently in five places (two branches of the same-domain normalisation,
// the face/edge collector functions, and the descriptor collectors).
struct FaceGeometrySummary {
  std::string surfaceType;
  double areaMm2 = 0.0;
  Point3 centroidMm;
  Bounds3 boundsMm;
};

struct EdgeGeometrySummary {
  std::string curveType;
  double lengthMm = 0.0;
  Point3 centroidMm;
  Bounds3 boundsMm;
  bool closed = false;
};

// The adaptors are supplied by the caller: the call sites already build them,
// and some of them additionally need the raw adaptor (e.g. the cylinder radius
// and axis recorded during same-domain normalisation).
FaceGeometrySummary SummarizeFace(const TopoDS_Face &face,
                                  const BRepAdaptor_Surface &surface);
EdgeGeometrySummary SummarizeEdge(const TopoDS_Edge &edge,
                                  const BRepAdaptor_Curve &curve);

} // namespace detail
} // namespace cadstep
