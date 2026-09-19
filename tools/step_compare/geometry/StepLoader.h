#pragma once

#include "StepCompare.h"

#include <TopoDS_Solid.hxx>

#include <filesystem>
#include <string>
#include <vector>

namespace cadstep {
namespace detail {

enum class LoadClass {
  Ready,
  Invalid,
  Unsupported,
  InternalError,
};

struct LoadedSolidItem {
  int index = 0;
  std::string id;
  InputAudit audit;
  TopoDS_Solid solid;
};

struct LoadedStepModel {
  LoadClass classification = LoadClass::InternalError;
  InputAudit compositeAudit;
  std::vector<LoadedSolidItem> solids;
  std::string reason;
};

// Reads a STEP file and audits every solid it contains. The composite audit
// aggregates the individual solids (volume-weighted centroid, union bounds).
LoadedStepModel LoadStepModel(const std::filesystem::path &stepPath,
                              const CompareConfig &config,
                              EntitySide side);


} // namespace detail
} // namespace cadstep
