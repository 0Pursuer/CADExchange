#pragma once

#include "StepCompare.h"

#include <TopoDS_Solid.hxx>

#include <filesystem>
#include <string>

namespace cadstep {
namespace detail {

// Runs the full four-phase comparison for one pair of solids and returns the
// populated CompareResult. `artifactSuffix` disambiguates per-pair artefacts on
// the multi-solid path (empty for the single-solid path).
CompareResult CompareSolidPairInternal(const TopoDS_Solid &solidRef,
                                       const InputAudit &auditRef,
                                       const TopoDS_Solid &solidCand,
                                       const InputAudit &auditCand,
                                       const CompareConfig &config,
                                       const std::filesystem::path &outputDirectory,
                                       const std::string &artifactSuffix);


} // namespace detail
} // namespace cadstep
