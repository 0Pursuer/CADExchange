// Entry point: loads both files, routes single- vs multi-solid, and rolls
// the per-pair verdicts up into the overall result.

#include "StepCompare.h"
#include "domain/DecisionCode.h"
#include "domain/GeometryMath.h"
#include "domain/Timing.h"
#include "domain/TolerancePolicy.h"
#include "geometry/ArtifactWriter.h"
#include "geometry/ShapeAudit.h"
#include "geometry/StepLoader.h"
#include "pipeline/SolidPairPipeline.h"

#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace cadstep {
namespace detail {

struct SolidPairCandidate {
  int index = -1;
  double cost = 1.0e12;
  double volumeDifferenceMm3 = 0.0;
  double relativeVolumeDifference = 0.0;
  double centroidDistanceMm = 0.0;
  double boundsDifferenceMm = 0.0;
  bool volumeEligible = false;
  bool centroidEligible = false;
  bool boundsEligible = false;
};

struct SolidCandidateSelection {
  SolidPairCandidate best;
  SolidPairCandidate bestRejected;
};

SolidCandidateSelection SelectSolidCandidate(
    const LoadedSolidItem &refItem,
    const std::vector<LoadedSolidItem> &candidates,
    const std::vector<bool> &candidateTaken,
    const CompareConfig &config,
    double centroidToleranceMm,
    double boundsToleranceMm) {
  SolidCandidateSelection selection;

  for (std::size_t j = 0; j < candidates.size(); ++j) {
    if (candidateTaken[j]) {
      continue;
    }
    const LoadedSolidItem &candItem = candidates[j];

    const double volDiff =
        std::abs(refItem.audit.signedVolumeMm3 - candItem.audit.signedVolumeMm3);
    const double relVolDiff = refItem.audit.signedVolumeMm3 > 0.0
                                  ? (volDiff / refItem.audit.signedVolumeMm3)
                                  : 0.0;
    const double centDist =
        PointDistance(refItem.audit.centroidMm, candItem.audit.centroidMm);
    const double boundsDiff =
        ComputeBoundsDifference(refItem.audit.boundsMm, candItem.audit.boundsMm);

    const bool volEligible = relVolDiff <= config.solidMatchVolumeRelTol;
    const bool centEligible = centDist <= centroidToleranceMm;
    const bool bdsEligible =
        boundsDiff <= boundsToleranceMm ||
        (volEligible && centEligible && relVolDiff <= config.solidMatchVolumeRelTol &&
         centDist <= centroidToleranceMm * 0.1);
    const bool isEligible = volEligible && centEligible && bdsEligible;

    // The cost function is unchanged: relative volume (scaled) dominates, with
    // centroid and bounds distances as tie-breaks.
    const double cost = relVolDiff * 100.0 + centDist + boundsDiff;

    SolidPairCandidate &slot = isEligible ? selection.best : selection.bestRejected;
    // Strict '<' keeps the first candidate on ties, as before.
    if (cost < slot.cost) {
      slot.index = static_cast<int>(j);
      slot.cost = cost;
      slot.volumeDifferenceMm3 = volDiff;
      slot.relativeVolumeDifference = relVolDiff;
      slot.centroidDistanceMm = centDist;
      slot.boundsDifferenceMm = boundsDiff;
      slot.volumeEligible = volEligible;
      slot.centroidEligible = centEligible;
      slot.boundsEligible = bdsEligible;
    }
  }

  return selection;
}

void AggregateMultiSolidStatus(CompareResult &result,
                               const std::vector<CompareResult> &pairResults,
                               int matchedCount) {
  if (result.multiSolid.unmatchedReferenceSolidCount > 0 ||
      result.multiSolid.unmatchedCandidateSolidCount > 0) {
    result.status = CompareStatus::Different;
    result.reason = "multi-solid matching failed: reference_solids=" +
                    std::to_string(result.multiSolid.referenceSolidCount) +
                    ", candidate_solids=" +
                    std::to_string(result.multiSolid.candidateSolidCount) +
                    ", matched=" + std::to_string(matchedCount);
    result.decisionPath = ToString(DecisionCode::MultiSolidUnmatched);
    return;
  }

  bool allEqual = true;
  bool anyDifferent = false;
  bool anyLikelyEqual = false;
  std::string firstDiffReason;

  for (const auto &pairRes : pairResults) {
    if (pairRes.status != CompareStatus::Equal) {
      allEqual = false;
    }
    if (pairRes.status == CompareStatus::Different) {
      anyDifferent = true;
      if (firstDiffReason.empty()) firstDiffReason = pairRes.reason;
    } else if (pairRes.status == CompareStatus::LikelyEqual) {
      anyLikelyEqual = true;
    }
  }

  if (allEqual) {
    result.status = CompareStatus::Equal;
    result.reason = "all " + std::to_string(matchedCount) + " solid pairs are EQUAL";
    result.decisionPath = ToString(DecisionCode::MultiSolidPairsEqual);
  } else if (anyDifferent) {
    result.status = CompareStatus::Different;
    result.reason = "solid pair comparison failed: " + firstDiffReason;
    result.decisionPath = ToString(DecisionCode::MultiSolidPairDifferent);
  } else if (anyLikelyEqual) {
    result.status = CompareStatus::LikelyEqual;
    result.reason = "all solid pairs matched (with likely equal pairs)";
    result.decisionPath = ToString(DecisionCode::MultiSolidPairsLikelyEqual);
  } else {
    result.status = CompareStatus::Indeterminate;
    result.reason = "solid pair comparison indeterminate";
    result.decisionPath = ToString(DecisionCode::MultiSolidPairsIndeterminate);
  }
}

void ExportCompositeArtifacts(const std::filesystem::path &outputDirectory,
                              const CompareConfig &config,
                              const LoadedStepModel &loadedRef,
                              const LoadedStepModel &loadedCand,
                              const std::vector<CompareResult> &pairResults,
                              CompareResult &result) {
  BRep_Builder builder;
  TopoDS_Compound compRef;
  builder.MakeCompound(compRef);
  for (const auto &item : loadedRef.solids) {
    if (!item.solid.IsNull()) builder.Add(compRef, item.solid);
  }

  TopoDS_Compound compCand;
  builder.MakeCompound(compCand);
  for (const auto &item : loadedCand.solids) {
    if (!item.solid.IsNull()) builder.Add(compCand, item.solid);
  }

  const auto refStl = outputDirectory / "reference_base.stl";
  if (ExportShapeStl(compRef, refStl)) {
    result.artifacts.items.push_back({"reference_base", "reference_base.stl", "STL", true, ""});
  }

  const auto candStl = outputDirectory / "candidate_base.stl";
  if (ExportShapeStl(compCand, candStl)) {
    result.artifacts.items.push_back({"candidate_base", "candidate_base.stl", "STL", true, ""});
  }

  for (const auto &pairRes : pairResults) {
    for (const auto &art : pairRes.artifacts.items) {
      result.artifacts.items.push_back(art);
    }
  }
}

} // namespace detail
} // namespace cadstep

namespace cadstep {

// CompareStepFiles is part of the public header, so it lives in cadstep
// rather than cadstep::detail; these bring the pipeline helpers into scope.
using detail::AggregateMultiSolidStatus;
using detail::CompareSolidPairInternal;
using detail::DecisionCode;
using detail::ElapsedMs;
using detail::ExportCompositeArtifacts;
using detail::LoadClass;
using detail::LoadedStepModel;
using detail::LoadStepModel;
using detail::SelectSolidCandidate;
using detail::SolidCandidateSelection;
using detail::SolidPairCandidate;
using detail::TolerancePolicy;
using detail::ToString;

CompareResult CompareStepFiles(const std::filesystem::path &reference,
                               const std::filesystem::path &candidate,
                               const CompareConfig &config,
                               const std::filesystem::path &outputDirectory) {
  const auto totalStart = std::chrono::high_resolution_clock::now();
  CompareResult result;
  result.thresholds = config;

  const auto loadRefStart = std::chrono::high_resolution_clock::now();
  const LoadedStepModel loadedRef = LoadStepModel(reference, config, EntitySide::Reference);
  result.timings.loadReferenceMs = ElapsedMs(loadRefStart);
  result.reference = loadedRef.compositeAudit;

  const auto loadCandStart = std::chrono::high_resolution_clock::now();
  const LoadedStepModel loadedCand = LoadStepModel(candidate, config, EntitySide::Candidate);
  result.timings.loadCandidateMs = ElapsedMs(loadCandStart);
  result.candidate = loadedCand.compositeAudit;

  if (loadedRef.classification == LoadClass::Invalid || loadedCand.classification == LoadClass::Invalid) {
    result.status = CompareStatus::InvalidInput;
    result.reason = loadedRef.classification == LoadClass::Invalid ? loadedRef.reason : loadedCand.reason;
    result.decisionPath = ToString(DecisionCode::InputInvalid);
    result.timings.totalMs = ElapsedMs(totalStart);
    return result;
  }

  if (loadedRef.classification == LoadClass::Unsupported || loadedCand.classification == LoadClass::Unsupported) {
    result.status = CompareStatus::UnsupportedShape;
    result.reason = loadedRef.classification == LoadClass::Unsupported ? loadedRef.reason : loadedCand.reason;
    result.decisionPath = ToString(DecisionCode::InputUnsupported);
    result.timings.totalMs = ElapsedMs(totalStart);
    return result;
  }

  const bool referenceIsMulti = loadedRef.solids.size() > 1;
  const bool candidateIsMulti = loadedCand.solids.size() > 1;
  const bool actuallyMultiSolid = referenceIsMulti || candidateIsMulti;

  if (!actuallyMultiSolid) {
    CompareResult pairRes = CompareSolidPairInternal(
        loadedRef.solids[0].solid, loadedRef.solids[0].audit,
        loadedCand.solids[0].solid, loadedCand.solids[0].audit,
        config, outputDirectory, "");

    pairRes.reference = loadedRef.compositeAudit;
    pairRes.candidate = loadedCand.compositeAudit;
    pairRes.thresholds = config;
    pairRes.timings.loadReferenceMs = result.timings.loadReferenceMs;
    pairRes.timings.loadCandidateMs = result.timings.loadCandidateMs;
    pairRes.timings.totalMs = ElapsedMs(totalStart);

    pairRes.multiSolid.allowed = config.allowMultipleSolids;
    pairRes.multiSolid.executed = false;
    pairRes.multiSolid.enabled = false;
    pairRes.multiSolid.policy = config.multiSolidPolicy;
    pairRes.multiSolid.referenceSolidCount = 1;
    pairRes.multiSolid.candidateSolidCount = 1;
    pairRes.multiSolid.matchedSolidCount = 1;
    pairRes.multiSolid.unmatchedReferenceSolidCount = 0;
    pairRes.multiSolid.unmatchedCandidateSolidCount = 0;

    SolidMatchRecord matchRec;
    matchRec.referenceSolidId = loadedRef.solids[0].id;
    matchRec.candidateSolidId = loadedCand.solids[0].id;
    matchRec.referenceIndex = 0;
    matchRec.candidateIndex = 0;
    matchRec.matchStatus = MatchStatus::Matched;
    matchRec.status = pairRes.status;
    matchRec.reason = pairRes.reason;
    matchRec.volumeDifferenceMm3 = pairRes.absoluteInputVolumeDifferenceMm3;
    matchRec.relativeVolumeDifference = pairRes.relativeInputVolumeDifference;
    matchRec.centroidDistanceMm = pairRes.centroidDistanceMm;
    matchRec.boundsDifferenceMm = pairRes.maximumBoundsDifferenceMm;
    matchRec.volumeEligible = true;
    matchRec.centroidEligible = true;
    matchRec.boundsEligible = true;
    matchRec.volumeTolerance = config.solidMatchVolumeRelTol;
    matchRec.centroidToleranceMm = config.solidMatchCentroidTolMm;
    matchRec.boundsToleranceMm = config.solidMatchBoundsTolMm;
    pairRes.multiSolid.solidMatches.push_back(matchRec);

    return pairRes;
  }

  if (!config.allowMultipleSolids || config.multiSolidPolicy == MultiSolidPolicy::Strict) {
    result.status = CompareStatus::UnsupportedShape;
    result.reason = "multiple 3D solids detected (reference_solids=" + std::to_string(loadedRef.solids.size()) +
                    ", candidate_solids=" + std::to_string(loadedCand.solids.size()) +
                    "); allowMultipleSolids=false or policy=strict";
    result.decisionPath = ToString(DecisionCode::InputUnsupported);
    result.multiSolid.allowed = config.allowMultipleSolids;
    result.multiSolid.executed = false;
    result.multiSolid.enabled = false;
    result.multiSolid.policy = config.multiSolidPolicy;
    result.multiSolid.referenceSolidCount = static_cast<int>(loadedRef.solids.size());
    result.multiSolid.candidateSolidCount = static_cast<int>(loadedCand.solids.size());
    result.timings.totalMs = ElapsedMs(totalStart);
    return result;
  }

  if (config.multiSolidPolicy == MultiSolidPolicy::CollectionOnly) {
    result.status = CompareStatus::UnsupportedShape;
    result.reason = "multi-solid collection policy is not implemented yet";
    result.decisionPath = ToString(DecisionCode::InputUnsupportedPolicy);
    result.multiSolid.allowed = config.allowMultipleSolids;
    result.multiSolid.executed = false;
    result.multiSolid.enabled = false;
    result.multiSolid.policy = config.multiSolidPolicy;
    result.multiSolid.referenceSolidCount = static_cast<int>(loadedRef.solids.size());
    result.multiSolid.candidateSolidCount = static_cast<int>(loadedCand.solids.size());
    result.timings.totalMs = ElapsedMs(totalStart);
    return result;
  }

  result.multiSolid.allowed = true;
  result.multiSolid.executed = true;
  result.multiSolid.enabled = true;
  result.multiSolid.policy = config.multiSolidPolicy;
  result.multiSolid.referenceSolidCount = static_cast<int>(loadedRef.solids.size());
  result.multiSolid.candidateSolidCount = static_cast<int>(loadedCand.solids.size());
  result.globalMetricsExecuted = false;

  const detail::TolerancePolicy tolerances =
      detail::TolerancePolicy::Derive(config, result.reference);
  const double scaleTol = tolerances.distanceMm();
  const double effectiveCentroidTol = std::max(config.solidMatchCentroidTolMm, scaleTol);
  const double effectiveBoundsTol = std::max(config.solidMatchBoundsTolMm, scaleTol);

  std::vector<bool> candMatched(loadedCand.solids.size(), false);
  int matchedCount = 0;
  std::vector<CompareResult> pairResults;

  for (std::size_t i = 0; i < loadedRef.solids.size(); ++i) {
    const auto &refItem = loadedRef.solids[i];
    const SolidCandidateSelection selection =
        SelectSolidCandidate(refItem, loadedCand.solids, candMatched, config,
                             effectiveCentroidTol, effectiveBoundsTol);
    const SolidPairCandidate &best = selection.best;
    const SolidPairCandidate &rejected = selection.bestRejected;

    if (best.index >= 0) {
      candMatched[best.index] = true;
      matchedCount++;

      std::string pairSuffix = "_pair_" + std::to_string(i) + "_" + std::to_string(best.index);
      CompareResult pairRes = CompareSolidPairInternal(
          refItem.solid, refItem.audit,
          loadedCand.solids[best.index].solid, loadedCand.solids[best.index].audit,
          config, outputDirectory, pairSuffix);
      pairResults.push_back(pairRes);

      SolidMatchRecord matchRec;
      matchRec.referenceSolidId = refItem.id;
      matchRec.candidateSolidId = loadedCand.solids[best.index].id;
      matchRec.referenceIndex = static_cast<int>(i);
      matchRec.candidateIndex = best.index;
      matchRec.matchStatus = MatchStatus::Matched;
      matchRec.status = pairRes.status;
      matchRec.reason = pairRes.reason;
      matchRec.volumeDifferenceMm3 = best.volumeDifferenceMm3;
      matchRec.relativeVolumeDifference = best.relativeVolumeDifference;
      matchRec.centroidDistanceMm = best.centroidDistanceMm;
      matchRec.boundsDifferenceMm = best.boundsDifferenceMm;
      matchRec.volumeEligible = true;
      matchRec.centroidEligible = true;
      matchRec.boundsEligible = true;
      matchRec.volumeTolerance = config.solidMatchVolumeRelTol;
      matchRec.centroidToleranceMm = effectiveCentroidTol;
      matchRec.boundsToleranceMm = effectiveBoundsTol;
      result.multiSolid.solidMatches.push_back(matchRec);
    } else {
      SolidMatchRecord matchRec;
      matchRec.referenceSolidId = refItem.id;
      matchRec.referenceIndex = static_cast<int>(i);
      matchRec.matchStatus = MatchStatus::Unmatched;
      matchRec.status = CompareStatus::Different;
      matchRec.volumeTolerance = config.solidMatchVolumeRelTol;
      matchRec.centroidToleranceMm = effectiveCentroidTol;
      matchRec.boundsToleranceMm = effectiveBoundsTol;

      if (rejected.index >= 0) {
        matchRec.candidateSolidId = loadedCand.solids[rejected.index].id;
        matchRec.candidateIndex = rejected.index;
        matchRec.volumeDifferenceMm3 = rejected.volumeDifferenceMm3;
        matchRec.relativeVolumeDifference = rejected.relativeVolumeDifference;
        matchRec.centroidDistanceMm = rejected.centroidDistanceMm;
        matchRec.boundsDifferenceMm = rejected.boundsDifferenceMm;
        matchRec.volumeEligible = rejected.volumeEligible;
        matchRec.centroidEligible = rejected.centroidEligible;
        matchRec.boundsEligible = rejected.boundsEligible;

        if (!rejected.volumeEligible) matchRec.rejectReasons.push_back("VOLUME_THRESHOLD_EXCEEDED");
        if (!rejected.centroidEligible) matchRec.rejectReasons.push_back("CENTROID_THRESHOLD_EXCEEDED");
        if (!rejected.boundsEligible) matchRec.rejectReasons.push_back("BOUNDS_THRESHOLD_EXCEEDED");

        matchRec.reason = "best candidate rejected due to threshold failure";
      } else {
        matchRec.candidateSolidId = "";
        matchRec.candidateIndex = -1;
        matchRec.reason = "no available candidate solid for matching";
      }

      result.multiSolid.solidMatches.push_back(matchRec);
      result.multiSolid.unmatchedReferenceSolidIds.push_back(refItem.id);
    }
  }

  for (std::size_t j = 0; j < loadedCand.solids.size(); ++j) {
    if (!candMatched[j]) {
      const auto &candItem = loadedCand.solids[j];
      SolidMatchRecord matchRec;
      matchRec.referenceSolidId = "";
      matchRec.candidateSolidId = candItem.id;
      matchRec.referenceIndex = -1;
      matchRec.candidateIndex = static_cast<int>(j);
      matchRec.matchStatus = MatchStatus::Unmatched;
      matchRec.status = CompareStatus::Different;
      matchRec.reason = "unmatched candidate solid";
      matchRec.volumeTolerance = config.solidMatchVolumeRelTol;
      matchRec.centroidToleranceMm = effectiveCentroidTol;
      matchRec.boundsToleranceMm = effectiveBoundsTol;
      result.multiSolid.solidMatches.push_back(matchRec);
      result.multiSolid.unmatchedCandidateSolidIds.push_back(candItem.id);
    }
  }

  result.multiSolid.matchedSolidCount = matchedCount;
  result.multiSolid.unmatchedReferenceSolidCount = static_cast<int>(result.multiSolid.unmatchedReferenceSolidIds.size());
  result.multiSolid.unmatchedCandidateSolidCount = static_cast<int>(result.multiSolid.unmatchedCandidateSolidIds.size());

  AggregateMultiSolidStatus(result, pairResults, matchedCount);

  if (config.exportStl && !outputDirectory.empty()) {
    ExportCompositeArtifacts(outputDirectory, config, loadedRef, loadedCand, pairResults, result);
  }

  result.timings.totalMs = ElapsedMs(totalStart);
  return result;
}

} // namespace cadstep
