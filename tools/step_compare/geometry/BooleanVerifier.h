#pragma once

#include "StepCompare.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>

namespace cadstep {
namespace detail {

struct BooleanDifferenceResult {
  DifferenceAudit audit;
  TopoDS_Shape shape;
};

// argument - tool. A null tool yields an empty (successful) result.
BooleanDifferenceResult CutSolids(const TopoDS_Solid &argument,
                                  const TopoDS_Solid &tool,
                                  double fuzzyTolerance);


} // namespace detail
} // namespace cadstep
