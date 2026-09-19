// Boolean difference used for the fine-grained material comparison.

#include "geometry/BooleanVerifier.h"
#include "geometry/ShapeAudit.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS_Solid.hxx>

#include <cmath>
#include <string>

namespace cadstep {
namespace detail {

BooleanDifferenceResult CutSolids(const TopoDS_Solid &argument, const TopoDS_Solid &tool, double fuzzyTolerance) {
  BooleanDifferenceResult result;
  if (argument.IsNull()) {
    result.audit.succeeded = false;
    result.audit.report = "argument solid is null";
    return result;
  }
  if (tool.IsNull()) {
    result.audit.succeeded = true;
    result.audit.volumeMm3 = 0.0;
    result.audit.componentCount = 0;
    result.audit.report = "tool solid is null";
    return result;
  }

  try {
    TopTools_ListOfShape argList;
    argList.Append(argument);
    TopTools_ListOfShape toolList;
    toolList.Append(tool);

    BRepAlgoAPI_Cut cutAlgo;
    cutAlgo.SetArguments(argList);
    cutAlgo.SetTools(toolList);
    cutAlgo.SetFuzzyValue(fuzzyTolerance);
    cutAlgo.SetRunParallel(true);
    cutAlgo.SetUseOBB(true);
    cutAlgo.SetToFillHistory(false);
    cutAlgo.SetNonDestructive(true);
    cutAlgo.Build();

    if (cutAlgo.HasErrors()) {
      result.audit.succeeded = false;
      result.audit.report = "BRepAlgoAPI_Cut failed";
      return result;
    }

    result.shape = cutAlgo.Shape();
    if (result.shape.IsNull()) {
      result.audit.succeeded = true;
      result.audit.volumeMm3 = 0.0;
      result.audit.componentCount = 0;
      return result;
    }

    GProp_GProps props;
    BRepGProp::VolumeProperties(result.shape, props);
    result.audit.succeeded = true;
    result.audit.volumeMm3 = std::abs(props.Mass());
    result.audit.componentCount = CountUniqueSubShapes(result.shape, TopAbs_SOLID);
  } catch (const Standard_Failure &e) {
    result.audit.succeeded = false;
    result.audit.report = std::string("BRepAlgoAPI_Cut exception: ") + e.GetMessageString();
  } catch (...) {
    result.audit.succeeded = false;
    result.audit.report = "BRepAlgoAPI_Cut unknown exception";
  }

  return result;
}

} // namespace detail
} // namespace cadstep
