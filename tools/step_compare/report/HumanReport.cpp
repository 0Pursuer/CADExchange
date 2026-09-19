// Rendering of the comparison result as a human-readable summary.

#include "StepCompare.h"

#include "domain/TolerancePolicy.h"

#include <algorithm>
#include <filesystem>
#include <sstream>
#include <string>

namespace cadstep {

std::string ToHumanSummary(const CompareResult &result) {
  std::ostringstream ss;

  const detail::TolerancePolicy tolerances =
      detail::TolerancePolicy::Derive(result.thresholds, result.reference);
  const double effectiveDistTol = tolerances.distanceMm();
  const double effectiveAbsVolTol = tolerances.absoluteVolumeMm3();

  const std::string refName = std::filesystem::path(result.reference.path).filename().string();
  const std::string candName = std::filesystem::path(result.candidate.path).filename().string();

  const detail::GeometryPassFlags passes =
      detail::EvaluateGeometryPasses(result, tolerances);
  const bool volPass = passes.volume;
  const bool centroidPass = passes.centroid;
  const bool boundsPass = passes.bounds;
  const bool boolPass = detail::EvaluateBooleanPass(result, tolerances);

  const bool pairUsable = result.referenceNormalization.succeeded && result.candidateNormalization.succeeded;
  const bool detectedMulti = (result.reference.solidCount > 1 || result.candidate.solidCount > 1);

  ss << "============================================================\n"
     << "STEP GEOMETRY COMPARISON\n"
     << "============================================================\n\n"
     << "COMPARE MODE\n"
     << "  Detected    : " << (detectedMulti ? "multi-solid" : "single-solid") << "\n"
     << "  Allow multi : " << (result.thresholds.allowMultipleSolids ? "YES" : "NO") << "\n"
     << "  Selected    : " << (result.multiSolid.executed ? ToString(result.multiSolid.policy) : (detectedMulti ? "strict" : "single-solid")) << "\n\n"
     << "INPUT\n"
     << "  Reference : " << refName << "\n"
     << "  Candidate : " << candName << "\n\n";

  if (result.multiSolid.executed) {
    ss << "MULTI-SOLID MATCH\n"
       << "  Policy     : " << ToString(result.multiSolid.policy) << "\n"
       << "  Reference  : " << result.multiSolid.referenceSolidCount << " solids\n"
       << "  Candidate  : " << result.multiSolid.candidateSolidCount << " solids\n"
       << "  Matched    : " << result.multiSolid.matchedSolidCount << "\n"
       << "  Unmatched  : ref=" << result.multiSolid.unmatchedReferenceSolidCount
       << ", cand=" << result.multiSolid.unmatchedCandidateSolidCount << "\n";
    for (const auto &m : result.multiSolid.solidMatches) {
      if (m.matchStatus == MatchStatus::Unmatched && !m.rejectReasons.empty()) {
        ss << "  [Match Rejected] Ref: " << m.referenceSolidId << ", Best Cand: " << m.candidateSolidId << "\n"
           << "    Vol rel diff: " << (m.relativeVolumeDifference * 100.0) << "% <= " << (m.volumeTolerance * 100.0) << "% [" << (m.volumeEligible ? "PASS" : "FAIL") << "]\n"
           << "    Centroid dist: " << m.centroidDistanceMm << " mm <= " << m.centroidToleranceMm << " mm [" << (m.centroidEligible ? "PASS" : "FAIL") << "]\n"
           << "    Bounds diff  : " << m.boundsDifferenceMm << " mm <= " << m.boundsToleranceMm << " mm [" << (m.boundsEligible ? "PASS" : "FAIL") << "]\n";
      }
    }
    ss << "\n";
  }

  ss << "NORMALIZATION\n"
     << "  Pair usable    : " << (pairUsable ? "YES" : "NO") << "\n"
     << "  Reference face : " << result.reference.faceCount << " -> " << result.referenceNormalization.faceCountAfter << "\n"
     << "  Candidate face : " << result.candidate.faceCount << " -> " << result.candidateNormalization.faceCountAfter << "\n"
     << "  Reference edge : " << result.reference.edgeCount << " -> " << result.referenceNormalization.edgeCountAfter << "\n"
     << "  Candidate edge : " << result.candidate.edgeCount << " -> " << result.candidateNormalization.edgeCountAfter << "\n"
     << "  Mapping        : " << (result.referenceNormalization.mappingComplete ? "COMPLETE" : "INCOMPLETE") << "\n\n"
     << "DESCRIPTOR MATCH\n"
     << "  Faces     : " << result.normalizedTopology.faces.matchedCount << " / " << result.normalizedTopology.faces.referenceCount << "\n"
     << "  Edges     : " << result.normalizedTopology.edges.matchedCount << " / " << result.normalizedTopology.edges.referenceCount << "\n"
     << "  Ambiguous : faces=" << result.normalizedTopology.faces.ambiguousCount << ", edges=" << result.normalizedTopology.edges.ambiguousCount << "\n"
     << "  Unmatched : faces=" << result.normalizedTopology.faces.unmatchedReferenceIds.size()
     << ", edges=" << result.normalizedTopology.edges.unmatchedReferenceIds.size() << "\n\n"
     << "GLOBAL METRICS\n";

  if (result.globalMetricsExecuted) {
    ss << "  Volume diff : " << result.absoluteInputVolumeDifferenceMm3 << " mm³ ("
       << (result.relativeInputVolumeDifference * 100.0) << "%) [" << (volPass ? "PASS" : "FAIL") << "]\n"
       << "  Centroid dist : " << result.centroidDistanceMm << " mm [" << (centroidPass ? "PASS" : "FAIL") << "]\n"
       << "  Bounds diff   : " << result.maximumBoundsDifferenceMm << " mm [" << (boundsPass ? "PASS" : "FAIL") << "]\n\n";
  } else {
    ss << "  Volume diff   : N/A [NOT EXECUTED]\n"
       << "  Centroid dist : N/A [NOT EXECUTED]\n"
       << "  Bounds diff   : N/A [NOT EXECUTED]\n\n";
  }

  ss << "BOOLEAN\n"
     << "  Executed   : " << (result.booleanExecuted ? "YES" : "NO") << "\n";
  if (result.booleanExecuted) {
    ss << "  A-B volume : " << result.missingMaterial.volumeMm3 << " mm³\n"
       << "  B-A volume : " << result.addedMaterial.volumeMm3 << " mm³\n"
       << "  Residual   : " << result.symmetricDifferenceVolumeMm3 << " mm³ [" << (boolPass ? "PASS" : "FAIL") << "]\n\n";
  } else {
    ss << "  A-B volume : N/A\n"
       << "  B-A volume : N/A\n"
       << "  Residual   : N/A\n\n";
  }

  ss << "TIMING\n"
     << "  Load       : " << (result.timings.loadReferenceMs + result.timings.loadCandidateMs) << " ms\n"
     << "  Normalize  : " << (result.timings.normalizeReferenceMs + result.timings.normalizeCandidateMs) << " ms\n"
     << "  Match      : " << result.normalizedTopology.elapsedMs << " ms\n"
     << "  Boolean    : " << (result.timings.booleanAbMs + result.timings.booleanBaMs) << " ms\n"
     << "  Export     : " << result.timings.artifactExportMs << " ms\n"
     << "  Total      : " << result.timings.totalMs << " ms\n\n"
     << "RESULT\n"
     << "  Status        : " << ToString(result.status) << "\n"
     << "  Decision path : " << result.decisionPath << "\n"
     << "  Reason        : " << result.reason << "\n"
     << "============================================================\n";

  return ss.str();
}
} // namespace cadstep
