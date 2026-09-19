// The decision flow for one solid pair: normalise, match, verify, decide.

#include "pipeline/SolidPairPipeline.h"
#include "domain/DecisionCode.h"
#include "domain/DescriptorMatching.h"
#include "domain/GeometryMath.h"
#include "domain/Timing.h"
#include "domain/TolerancePolicy.h"
#include "geometry/ArtifactWriter.h"
#include "geometry/BooleanVerifier.h"
#include "geometry/ShapeAudit.h"
#include "geometry/ShapeNormalizer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <set>
#include <string>
#include <vector>

namespace cadstep {
namespace detail {

struct ComparisonDecision {
  CompareStatus status = CompareStatus::InternalError;
  DecisionCode code = DecisionCode::BooleanAfterOriginal;
  std::string reason;
};

void SetDecision(ComparisonDecision &decision, CompareStatus status,
                 DecisionCode code, std::string reason) {
  decision.status = status;
  decision.code = code;
  decision.reason = reason;
}

EdgeAuditValidation ValidateEdgeAudit(const NormalizationAudit &refNorm,
                                     const NormalizationAudit &candNorm,
                                     const MatchCollection &edgeMatches) {
  EdgeAuditValidation validation;

  auto checkNorm = [&](const NormalizationAudit &norm, const std::string &sideName) {
    if (static_cast<int>(norm.edges.size()) != norm.edgeCountAfter) {
      validation.valid = false;
      validation.errors.push_back(sideName + "_EDGE_COUNT_MISMATCH: edges.size() != edgeCountAfter");
    }

    int compCount = 0;
    std::set<int> compIndices;
    for (const auto &info : norm.edges) {
      if (info.comparable) {
        compCount++;
        compIndices.insert(info.comparableIndex);
      } else {
        if (info.comparableIndex != 0) {
          validation.valid = false;
          validation.errors.push_back(sideName + "_NON_COMPARABLE_EDGE_HAS_INDEX: " + info.id);
        }
      }
    }

    if (compCount != norm.comparableEdgeCountAfter) {
      validation.valid = false;
      validation.errors.push_back(sideName + "_COMPARABLE_COUNT_MISMATCH: count=" +
                                  std::to_string(compCount) + " audit=" + std::to_string(norm.comparableEdgeCountAfter));
    }

    if (compCount > 0) {
      if (*compIndices.begin() != 1 || *compIndices.rbegin() != compCount || static_cast<int>(compIndices.size()) != compCount) {
        validation.valid = false;
        validation.errors.push_back(sideName + "_COMPARABLE_INDICES_NOT_CONTIGUOUS");
      }
    }
  };

  checkNorm(refNorm, "REF");
  checkNorm(candNorm, "CAND");

  if (edgeMatches.attempted) {
    if (edgeMatches.referenceCount != refNorm.comparableEdgeCountAfter) {
      validation.valid = false;
      validation.errors.push_back("EDGE_MATCH_REF_COUNT_MISMATCH: matchRef=" +
                                  std::to_string(edgeMatches.referenceCount) + " normRef=" + std::to_string(refNorm.comparableEdgeCountAfter));
    }
    if (edgeMatches.candidateCount != candNorm.comparableEdgeCountAfter) {
      validation.valid = false;
      validation.errors.push_back("EDGE_MATCH_CAND_COUNT_MISMATCH: matchCand=" +
                                  std::to_string(edgeMatches.candidateCount) + " normCand=" + std::to_string(candNorm.comparableEdgeCountAfter));
    }
  }

  return validation;
}

void ExportPairArtifacts(const std::filesystem::path &outputDirectory,
                         const std::string &artifactSuffix,
                         const CompareConfig &config,
                         const TopoDS_Solid &solidRef,
                         const TopoDS_Solid &solidCand,
                         const TopoDS_Solid &compareRef,
                         const TopoDS_Solid &compareCand,
                         const NormalizedSolidInternal &normRef,
                         const NormalizedSolidInternal &normCand,
                         bool normalizationPairUsable,
                         const BooleanDifferenceResult &missingRes,
                         const BooleanDifferenceResult &addedRes,
                         CompareResult &result) {
  std::error_code ec;
  std::filesystem::create_directories(outputDirectory, ec);
  if (artifactSuffix.empty()) {
    RemoveOldArtifacts(outputDirectory);
  }

  const std::string sfx = artifactSuffix;
  if (config.exportStl) {
    const auto refStl = outputDirectory / ("reference_base" + sfx + ".stl");
    if (ExportShapeStl(compareRef, refStl)) {
      result.artifacts.items.push_back({"reference_base" + sfx, "reference_base" + sfx + ".stl", "STL", true, ""});
    }
    const auto candStl = outputDirectory / ("candidate_base" + sfx + ".stl");
    if (ExportShapeStl(compareCand, candStl)) {
      result.artifacts.items.push_back({"candidate_base" + sfx, "candidate_base" + sfx + ".stl", "STL", true, ""});
    }
    if (result.booleanExecuted && result.missingMaterial.volumeMm3 > 0.0) {
      const auto missStl = outputDirectory / ("missing_material" + sfx + ".stl");
      if (ExportShapeStl(missingRes.shape, missStl)) {
        result.artifacts.items.push_back({"missing_material" + sfx, "missing_material" + sfx + ".stl", "STL", true, ""});
      }
    }
    if (result.booleanExecuted && result.addedMaterial.volumeMm3 > 0.0) {
      const auto addStl = outputDirectory / ("added_material" + sfx + ".stl");
      if (ExportShapeStl(addedRes.shape, addStl)) {
        result.artifacts.items.push_back({"added_material" + sfx, "added_material" + sfx + ".stl", "STL", true, ""});
      }
    }
  }

  if (config.exportBrep) {
    ExportShapeBrep(solidRef, outputDirectory / ("reference_original" + sfx + ".brep"));
    ExportShapeBrep(solidCand, outputDirectory / ("candidate_original" + sfx + ".brep"));
    if (normalizationPairUsable) {
      ExportShapeBrep(normRef.solid, outputDirectory / ("reference_normalized" + sfx + ".brep"));
      ExportShapeBrep(normCand.solid, outputDirectory / ("candidate_normalized" + sfx + ".brep"));
    }
  }

  const auto visDir = outputDirectory / "visualization";
  std::filesystem::create_directories(visDir, ec);

  if (ExportFacesVtp(normRef.solid, normRef.normalizedFaces, normRef.audit.faces, result.normalizedTopology.faces, EntitySide::Reference, visDir / ("reference_faces" + sfx + ".vtp"))) {
    result.artifacts.items.push_back({"reference_faces" + sfx, "visualization/reference_faces" + sfx + ".vtp", "VTP", true, "entity_index"});
  }
  if (ExportFacesVtp(normCand.solid, normCand.normalizedFaces, normCand.audit.faces, result.normalizedTopology.faces, EntitySide::Candidate, visDir / ("candidate_faces" + sfx + ".vtp"))) {
    result.artifacts.items.push_back({"candidate_faces" + sfx, "visualization/candidate_faces" + sfx + ".vtp", "VTP", true, "entity_index"});
  }
  if (ExportEdgesVtp(normRef.solid, normRef.normalizedEdges, normRef.audit.edges, result.normalizedTopology.edges, EntitySide::Reference, visDir / ("reference_edges" + sfx + ".vtp"))) {
    result.artifacts.items.push_back({"reference_edges" + sfx, "visualization/reference_edges" + sfx + ".vtp", "VTP", true, "entity_index"});
  }
  if (ExportEdgesVtp(normCand.solid, normCand.normalizedEdges, normCand.audit.edges, result.normalizedTopology.edges, EntitySide::Candidate, visDir / ("candidate_edges" + sfx + ".vtp"))) {
    result.artifacts.items.push_back({"candidate_edges" + sfx, "visualization/candidate_edges" + sfx + ".vtp", "VTP", true, "entity_index"});
  }
}

CompareResult CompareSolidPairInternal(const TopoDS_Solid &solidRef,
                                       const InputAudit &auditRef,
                                       const TopoDS_Solid &solidCand,
                                       const InputAudit &auditCand,
                                       const CompareConfig &config,
                                       const std::filesystem::path &outputDirectory,
                                       const std::string &artifactSuffix) {
  const auto totalStart = std::chrono::high_resolution_clock::now();
  CompareResult result;
  result.thresholds = config;
  result.reference = auditRef;
  result.candidate = auditCand;

  // Decision flow for one solid pair. Each phase fills its own section of
  // `result` and updates `decision`; the phases fall through one to the next.
  // Only the normalised fast path terminates the flow early.
  //   1. normalisation + global metrics  - may latch GlobalMetricsFailed
  //   2. descriptor matching             - may early-exit on the fast path
  //   3. boolean verification            - skipped when the fast path decided
  //   4. artefact export
  const OriginalTopologyIndex refOriginalIndex = BuildOriginalTopologyIndex(solidRef, EntitySide::Reference);
  const OriginalTopologyIndex candOriginalIndex = BuildOriginalTopologyIndex(solidCand, EntitySide::Candidate);

  const auto normRefStart = std::chrono::high_resolution_clock::now();
  const NormalizedSolidInternal normRef = NormalizeSameDomain(solidRef, refOriginalIndex, EntitySide::Reference, config);
  // The descriptor projection happens inside NormalizeSameDomain but is reported
  // separately, so keep the two buckets disjoint.
  result.timings.normalizeReferenceMs = ElapsedMs(normRefStart) - normRef.descriptorBuildMs;
  result.referenceNormalization = normRef.audit;

  const auto normCandStart = std::chrono::high_resolution_clock::now();
  const NormalizedSolidInternal normCand = NormalizeSameDomain(solidCand, candOriginalIndex, EntitySide::Candidate, config);
  result.timings.normalizeCandidateMs = ElapsedMs(normCandStart) - normCand.descriptorBuildMs;
  result.candidateNormalization = normCand.audit;

  const bool normalizationPairUsable = result.referenceNormalization.succeeded && result.candidateNormalization.succeeded;

  const TopoDS_Solid &compareSolidRef = normalizationPairUsable ? normRef.solid : solidRef;
  const TopoDS_Solid &compareSolidCand = normalizationPairUsable ? normCand.solid : solidCand;

  ComparisonDecision decision;
  decision.code = normalizationPairUsable ? DecisionCode::BooleanAfterNormalization
                                          : DecisionCode::BooleanAfterOriginal;

  const double diagScale = BoundsDiagonal(result.reference.boundsMm);
  const detail::TolerancePolicy tolerances =
      detail::TolerancePolicy::Derive(config, result.reference);
  const double effectiveAbsVolTol = tolerances.absoluteVolumeMm3();

  result.absoluteInputVolumeDifferenceMm3 = std::abs(result.reference.signedVolumeMm3 - result.candidate.signedVolumeMm3);
  const double volDenom = std::abs(result.reference.signedVolumeMm3);
  result.relativeInputVolumeDifference = volDenom > 0.0 ? (result.absoluteInputVolumeDifferenceMm3 / volDenom) : 0.0;

  result.centroidDistanceMm = PointDistance(result.reference.centroidMm, result.candidate.centroidMm);
  result.maximumBoundsDifferenceMm = ComputeBoundsDifference(result.reference.boundsMm, result.candidate.boundsMm);
  result.globalMetricsExecuted = true;

  const detail::GeometryPassFlags passes =
      detail::EvaluateGeometryPasses(result, tolerances);

  // Latch, not an early exit: a failing metrics check blocks the fast path below
  // but execution continues into boolean verification.
  if (!passes.volume || !passes.centroid || !passes.bounds) {
    SetDecision(decision, CompareStatus::Different, DecisionCode::GlobalMetricsFailed,
                "geometry thresholds failed: input_volume centroid symmetric_difference");
  }

  // Phase 2: descriptor matching.
  if (normalizationPairUsable) {
    // Descriptors come straight out of the normalisation pass, so the matched
    // entities are exactly the geometry the audit already reported: no second
    // measurement pass over the shape. Their cost is measured where they are
    // built, per side.
    const std::vector<DescriptorView> &refFaceDescs = normRef.faceDescriptors;
    const std::vector<DescriptorView> &candFaceDescs = normCand.faceDescriptors;
    const std::vector<DescriptorView> &refEdgeDescs = normRef.comparableEdgeDescriptors;
    const std::vector<DescriptorView> &candEdgeDescs = normCand.comparableEdgeDescriptors;
    result.timings.descriptorBuildMs =
        normRef.descriptorBuildMs + normCand.descriptorBuildMs;

    const auto faceMatchStart = std::chrono::high_resolution_clock::now();
    result.normalizedTopology.faces =
        MatchFaceDescriptors(refFaceDescs, candFaceDescs, config, diagScale, tolerances);
    result.timings.faceMatchMs = ElapsedMs(faceMatchStart);
    result.normalizedTopology.faces.elapsedMs = result.timings.faceMatchMs;

    const auto edgeMatchStart = std::chrono::high_resolution_clock::now();
    result.normalizedTopology.edges =
        MatchEdgeDescriptors(refEdgeDescs, candEdgeDescs, config, diagScale, tolerances);
    result.timings.edgeMatchMs = ElapsedMs(edgeMatchStart);
    result.normalizedTopology.edges.elapsedMs = result.timings.edgeMatchMs;

    const auto edgeValidation = ValidateEdgeAudit(normRef.audit, normCand.audit, result.normalizedTopology.edges);
    result.normalizedTopology.edgeAuditConsistent = edgeValidation.valid;
    result.normalizedTopology.edgeAuditErrors = edgeValidation.errors;

    result.normalizedTopology.attempted = true;
    result.normalizedTopology.normalizedTopologyMatch =
        result.normalizedTopology.faces.allMatched && result.normalizedTopology.edges.allMatched &&
        (result.normalizedTopology.faces.ambiguousCount == 0) && (result.normalizedTopology.edges.ambiguousCount == 0) &&
        result.normalizedTopology.faces.typeHistogramEqual && result.normalizedTopology.edges.typeHistogramEqual &&
        result.normalizedTopology.edgeAuditConsistent;
    result.normalizedTopology.elapsedMs = result.timings.descriptorBuildMs + result.timings.faceMatchMs + result.timings.edgeMatchMs;

    result.normalizedTopology.fastPath.enabled = config.enableNormalizedFastPath;
    result.normalizedTopology.fastPath.eligible = result.normalizedTopology.normalizedTopologyMatch;
    if (!result.normalizedTopology.edgeAuditConsistent) {
      result.normalizedTopology.fastPath.blockReasons.push_back("EDGE_AUDIT_INCONSISTENT");
    }
    if (!result.normalizedTopology.faces.allMatched) {
      result.normalizedTopology.fastPath.blockReasons.push_back(
          "FACE_DESCRIPTOR_UNMATCHED:" + std::to_string(result.normalizedTopology.faces.referenceCount - result.normalizedTopology.faces.matchedCount));
    }
    if (!result.normalizedTopology.edges.allMatched) {
      result.normalizedTopology.fastPath.blockReasons.push_back(
          "EDGE_DESCRIPTOR_UNMATCHED:" + std::to_string(result.normalizedTopology.edges.referenceCount - result.normalizedTopology.edges.matchedCount));
    }

    if (config.enableNormalizedFastPath && result.normalizedTopology.normalizedTopologyMatch &&
        (decision.status != CompareStatus::Different)) {
      SetDecision(decision, CompareStatus::Equal, DecisionCode::NormalizedTopologyFastPath,
                  "closed solids pass configured thresholds via normalized fast path");
      result.booleanExecuted = false;
      result.normalizedTopology.fastPath.used = true;
    }
  } else {
    result.normalizedTopology.attempted = false;
    result.normalizedTopology.skipReason = "normalization pair is not usable";
  }

  // Phase 3: boolean verification, unless the fast path already decided.
  BooleanDifferenceResult missingRes;
  BooleanDifferenceResult addedRes;
  if (decision.code != DecisionCode::NormalizedTopologyFastPath) {
    result.booleanExecuted = true;
    const auto boolAbStart = std::chrono::high_resolution_clock::now();
    missingRes = CutSolids(compareSolidRef, compareSolidCand, config.booleanFuzzyToleranceMm);
    result.timings.booleanAbMs = ElapsedMs(boolAbStart);
    result.missingMaterial = missingRes.audit;

    const auto boolBaStart = std::chrono::high_resolution_clock::now();
    addedRes = CutSolids(compareSolidCand, compareSolidRef, config.booleanFuzzyToleranceMm);
    result.timings.booleanBaMs = ElapsedMs(boolBaStart);
    result.addedMaterial = addedRes.audit;

    if (!result.missingMaterial.succeeded || !result.addedMaterial.succeeded) {
      SetDecision(decision, CompareStatus::Indeterminate, DecisionCode::BooleanFailed,
                  "boolean cut operation failed: " + result.missingMaterial.report + " " +
                      result.addedMaterial.report);
    } else {
      result.symmetricDifferenceVolumeMm3 = result.missingMaterial.volumeMm3 + result.addedMaterial.volumeMm3;
      result.symmetricDifferenceRelative = volDenom > 0.0 ? (result.symmetricDifferenceVolumeMm3 / volDenom) : 0.0;

      // Deliberately NOT detail::EvaluateBooleanPass: this is a mutable latch,
      // not the rendered flag. The original-solid fallback below may flip it to
      // true, and the result feeds ClassifyClosedSolidComparison. The report
      // writers use the helper (which also short-circuits on !booleanExecuted);
      // the two agree here because booleanExecuted is true at this point.
      bool booleanPass = (result.missingMaterial.volumeMm3 <= effectiveAbsVolTol) &&
                         (result.addedMaterial.volumeMm3 <= effectiveAbsVolTol);

      if (!booleanPass && normalizationPairUsable && passes.volume && passes.centroid &&
          passes.bounds) {
        BooleanDifferenceResult origMissing = CutSolids(solidRef, solidCand, config.booleanFuzzyToleranceMm);
        BooleanDifferenceResult origAdded = CutSolids(solidCand, solidRef, config.booleanFuzzyToleranceMm);
        if (origMissing.audit.succeeded && origAdded.audit.succeeded) {
          const bool origBooleanPass = (origMissing.audit.volumeMm3 <= effectiveAbsVolTol) &&
                                       (origAdded.audit.volumeMm3 <= effectiveAbsVolTol);
          if (origBooleanPass) {
            missingRes = origMissing;
            addedRes = origAdded;
            result.missingMaterial = origMissing.audit;
            result.addedMaterial = origAdded.audit;
            result.symmetricDifferenceVolumeMm3 = result.missingMaterial.volumeMm3 + result.addedMaterial.volumeMm3;
            result.symmetricDifferenceRelative = volDenom > 0.0 ? (result.symmetricDifferenceVolumeMm3 / volDenom) : 0.0;
            booleanPass = true;
            decision.code = DecisionCode::BooleanAfterOriginalFallback;
          }
        }
      }
      auto &consistency = result.booleanConsistency;
      consistency.cutReferenceMinusCandidateSucceeded = result.missingMaterial.succeeded;
      consistency.cutCandidateMinusReferenceSucceeded = result.addedMaterial.succeeded;
      consistency.signedInputVolumeDiffMm3 =
          result.reference.signedVolumeMm3 - result.candidate.signedVolumeMm3;
      consistency.signedBooleanVolumeDiffMm3 =
          result.missingMaterial.volumeMm3 - result.addedMaterial.volumeMm3;
      consistency.conservationErrorMm3 = std::abs(
          consistency.signedInputVolumeDiffMm3 - consistency.signedBooleanVolumeDiffMm3);
      const double conservationScale = std::max(
          {1.0, std::abs(result.reference.signedVolumeMm3),
           std::abs(result.candidate.signedVolumeMm3)});
      consistency.relativeConservationError =
          consistency.conservationErrorMm3 / conservationScale;
      consistency.conservationPassed =
          consistency.relativeConservationError <= config.booleanConservationRelativeTolerance;
      consistency.booleanResultValid =
          consistency.cutReferenceMinusCandidateSucceeded &&
          consistency.cutCandidateMinusReferenceSucceeded && consistency.conservationPassed;
      if (!consistency.booleanResultValid) {
        consistency.invalidReason = "boolean volume conservation check failed";
      }

      const CompareStatus classifiedStatus = detail::ClassifyClosedSolidComparison(
          passes.volume, passes.centroid, passes.bounds, booleanPass, consistency);
      if (classifiedStatus == CompareStatus::Equal) {
        // The code is intentionally left as whatever produced this verdict
        // (after normalization / after original / original-solid fallback).
        decision.status = CompareStatus::Equal;
        decision.reason = (decision.code == DecisionCode::BooleanAfterOriginalFallback)
                              ? "closed solids pass configured thresholds (via original solid boolean fallback)"
                              : "closed solids pass configured thresholds";
      } else if (classifiedStatus == CompareStatus::LikelyEqual) {
        SetDecision(decision, CompareStatus::LikelyEqual,
                    DecisionCode::BooleanConservationInvalid,
                    "geometry thresholds passed but boolean conservation is invalid");
      } else {
        SetDecision(decision, CompareStatus::Different, DecisionCode::BooleanDifference,
                    "geometry thresholds failed: input_volume centroid symmetric_difference");
      }
    }
  }

  // Phase 4: artefact export.
  if (!outputDirectory.empty()) {
    const auto exportStart = std::chrono::high_resolution_clock::now();
    ExportPairArtifacts(outputDirectory, artifactSuffix, config, solidRef, solidCand,
                        compareSolidRef, compareSolidCand, normRef, normCand,
                        normalizationPairUsable, missingRes, addedRes, result);
    result.timings.artifactExportMs = ElapsedMs(exportStart);
  }

  result.status = decision.status;
  result.reason = decision.reason;
  result.decisionPath = ToString(decision.code);

  result.timings.totalMs = ElapsedMs(totalStart);
  return result;
}

} // namespace detail
} // namespace cadstep
