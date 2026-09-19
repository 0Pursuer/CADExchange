// Serialisation of the comparison result as result.json.
//
// The key insertion order matters: result.json is built with ordered_json and is
// read by GuiApp/src/step_compare_result.py, so this is the single place that
// knows the on-disk schema.

#include "StepCompare.h"

#include "domain/TolerancePolicy.h"

#include <json/single_include/nlohmann/json.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>

namespace cadstep {

using json = nlohmann::ordered_json;

std::string ToJson(const CompareResult &result) {
  json root = json::object();
  root["schema_version"] = 2;

  // 1. overall
  json overall = json::object();
  overall["status"] = ToString(result.status);
  overall["exit_code"] = ExitCode(result.status);
  overall["reason"] = result.reason;
  overall["decision_path"] = result.decisionPath;
  overall["boolean_executed"] = result.booleanExecuted;
  root["overall"] = overall;

  // 2. configuration
  const detail::TolerancePolicy tolerances =
      detail::TolerancePolicy::Derive(result.thresholds, result.reference);
  const double effectiveDistTol = tolerances.distanceMm();
  const double effectiveAbsVolTol = tolerances.absoluteVolumeMm3();

  json configJson = json::object();
  configJson["distance_tolerance_mm"] = result.thresholds.distanceToleranceMm;
  configJson["effective_distance_tolerance_mm"] = effectiveDistTol;
  configJson["absolute_volume_tolerance_mm3"] = result.thresholds.absoluteVolumeToleranceMm3;
  configJson["relative_volume_tolerance"] = result.thresholds.relativeVolumeTolerance;
  configJson["effective_volume_tolerance_mm3"] = effectiveAbsVolTol;
  configJson["boolean_fuzzy_tolerance_mm"] = result.thresholds.booleanFuzzyToleranceMm;
  configJson["normalization_enabled"] = result.thresholds.enableSameDomainNormalization;
  configJson["normalization_linear_tolerance_mm"] = result.thresholds.normalizationLinearToleranceMm;
  configJson["normalization_angular_tolerance_rad"] = result.thresholds.normalizationAngularToleranceRad;
  configJson["normalized_fast_path_enabled"] = result.thresholds.enableNormalizedFastPath;
  configJson["ambiguous_match_margin"] = result.thresholds.ambiguousMatchMargin;
  configJson["allow_multiple_solids"] = result.thresholds.allowMultipleSolids;
  configJson["multi_solid_policy"] = ToString(result.thresholds.multiSolidPolicy);
  configJson["solid_match_volume_rel_tol"] = result.thresholds.solidMatchVolumeRelTol;
  configJson["solid_match_centroid_tol_mm"] = result.thresholds.solidMatchCentroidTolMm;
  configJson["solid_match_bounds_tol_mm"] = result.thresholds.solidMatchBoundsTolMm;
  root["configuration"] = configJson;

  // 3. multi_solid
  json multiNode = json::object();
  multiNode["allowed"] = result.multiSolid.allowed;
  multiNode["executed"] = result.multiSolid.executed;
  multiNode["enabled"] = result.multiSolid.enabled;
  multiNode["policy"] = ToString(result.multiSolid.policy);
  multiNode["reference_solid_count"] = result.multiSolid.referenceSolidCount;
  multiNode["candidate_solid_count"] = result.multiSolid.candidateSolidCount;
  multiNode["matched_solid_count"] = result.multiSolid.matchedSolidCount;
  multiNode["unmatched_reference_solid_count"] = result.multiSolid.unmatchedReferenceSolidCount;
  multiNode["unmatched_candidate_solid_count"] = result.multiSolid.unmatchedCandidateSolidCount;

  json matchesArray = json::array();
  for (const auto &m : result.multiSolid.solidMatches) {
    json item = json::object();
    item["reference_solid_id"] = m.referenceSolidId;
    item["candidate_solid_id"] = m.candidateSolidId;
    item["reference_index"] = m.referenceIndex;
    item["candidate_index"] = m.candidateIndex;
    item["match_status"] = ToString(m.matchStatus);
    item["status"] = ToString(m.status);
    item["reason"] = m.reason;
    item["volume_difference_mm3"] = m.volumeDifferenceMm3;
    item["relative_volume_difference"] = m.relativeVolumeDifference;
    item["centroid_distance_mm"] = m.centroidDistanceMm;
    item["bounds_difference_mm"] = m.boundsDifferenceMm;
    item["volume_eligible"] = m.volumeEligible;
    item["centroid_eligible"] = m.centroidEligible;
    item["bounds_eligible"] = m.boundsEligible;
    item["volume_tolerance"] = m.volumeTolerance;
    item["centroid_tolerance_mm"] = m.centroidToleranceMm;
    item["bounds_tolerance_mm"] = m.boundsToleranceMm;
    item["reject_reasons"] = m.rejectReasons;
    matchesArray.push_back(item);
  }
  multiNode["solid_matches"] = matchesArray;
  multiNode["unmatched_reference_solid_ids"] = result.multiSolid.unmatchedReferenceSolidIds;
  multiNode["unmatched_candidate_solid_ids"] = result.multiSolid.unmatchedCandidateSolidIds;
  root["multi_solid"] = multiNode;

  // 4. inputs
  root["global_metrics_executed"] = result.globalMetricsExecuted;

  // 4. inputs
  auto MakeInputNode = [](const InputAudit &audit) {
    json node = json::object();
    node["path"] = audit.path;
    node["filename"] = std::filesystem::path(audit.path).filename().string();
    node["file_length_units"] = audit.fileLengthUnits;
    node["solid_count"] = audit.solidCount;
    node["shell_count"] = audit.shellCount;
    node["face_count"] = audit.faceCount;
    node["edge_count"] = audit.edgeCount;
    node["brep_valid"] = audit.brepValid;
    node["closed"] = audit.closed;
    node["volume_mm3"] = audit.signedVolumeMm3;
    node["surface_area_mm2"] = audit.surfaceAreaMm2;
    node["centroid_mm"] = {{"x", audit.centroidMm.x}, {"y", audit.centroidMm.y}, {"z", audit.centroidMm.z}};
    node["bounds_mm"] = {
        {"is_void", audit.boundsMm.isVoid},
        {"minimum", {{"x", audit.boundsMm.minimum.x}, {"y", audit.boundsMm.minimum.y}, {"z", audit.boundsMm.minimum.z}}},
        {"maximum", {{"x", audit.boundsMm.maximum.x}, {"y", audit.boundsMm.maximum.y}, {"z", audit.boundsMm.maximum.z}}}};
    node["load_diagnostics"] = audit.loadDiagnostics;
    node["transfer_diagnostics"] = audit.transferDiagnostics;
    return node;
  };
  json inputs = json::object();
  inputs["reference"] = MakeInputNode(result.reference);
  inputs["candidate"] = MakeInputNode(result.candidate);
  root["inputs"] = inputs;

  // 5. normalization
  auto MakeNormNode = [](const NormalizationAudit &audit) {
    json node = json::object();
    node["enabled"] = audit.enabled;
    node["succeeded"] = audit.succeeded;
    node["used_normalized_shape"] = audit.usedNormalizedShape;
    node["faces_before"] = audit.faceCountBefore;
    node["faces_after"] = audit.faceCountAfter;
    node["edges_before"] = audit.edgeCountBefore;
    node["edges_after"] = audit.edgeCountAfter;
    node["comparable_edges_before"] = audit.comparableEdgeCountBefore;
    node["comparable_edges_after"] = audit.comparableEdgeCountAfter;
    node["volume_before_mm3"] = audit.volumeBeforeMm3;
    node["volume_after_mm3"] = audit.volumeAfterMm3;
    node["relative_volume_drift"] = audit.relativeVolumeDrift;
    node["face_mapping_complete"] = audit.faceMappingComplete;
    node["edge_mapping_complete"] = audit.edgeMappingComplete;
    node["mapping_complete"] = audit.mappingComplete;

    json faceStats = json::array();
    for (const auto &st : audit.faceTypes) {
      faceStats.push_back({{"type", st.type}, {"count", st.count}, {"total_measure", st.totalMeasure}});
    }
    node["face_type_statistics"] = faceStats;

    json edgeStats = json::array();
    for (const auto &st : audit.edgeTypes) {
      edgeStats.push_back({{"type", st.type}, {"count", st.count}, {"total_measure", st.totalMeasure}});
    }
    node["edge_type_statistics"] = edgeStats;

    json facesArr = json::array();
    for (const auto &f : audit.faces) {
      facesArr.push_back({
          {"id", f.id},
          {"visual_index", f.visualIndex},
          {"surface_type", f.surfaceType},
          {"area_mm2", f.areaMm2},
          {"centroid_mm", {{"x", f.centroidMm.x}, {"y", f.centroidMm.y}, {"z", f.centroidMm.z}}},
          {"bounds_mm",
           {{"is_void", f.boundsMm.isVoid},
            {"minimum", {{"x", f.boundsMm.minimum.x}, {"y", f.boundsMm.minimum.y}, {"z", f.boundsMm.minimum.z}}},
            {"maximum", {{"x", f.boundsMm.maximum.x}, {"y", f.boundsMm.maximum.y}, {"z", f.boundsMm.maximum.z}}}}},
          {"source_face_ids", f.sourceFaceIds},
          {"boundary_edge_ids", f.boundaryEdgeIds},
          {"source_count", f.sourceCount},
          {"merged", f.merged},
      });
    }
    node["faces"] = facesArr;

    json edgesArr = json::array();
    for (const auto &e : audit.edges) {
      edgesArr.push_back({
          {"id", e.id},
          {"visual_index", e.visualIndex},
          {"curve_type", e.curveType},
          {"length_mm", e.lengthMm},
          {"centroid_mm", {{"x", e.centroidMm.x}, {"y", e.centroidMm.y}, {"z", e.centroidMm.z}}},
          {"bounds_mm",
           {{"is_void", e.boundsMm.isVoid},
            {"minimum", {{"x", e.boundsMm.minimum.x}, {"y", e.boundsMm.minimum.y}, {"z", e.boundsMm.minimum.z}}},
            {"maximum", {{"x", e.boundsMm.maximum.x}, {"y", e.boundsMm.maximum.y}, {"z", e.boundsMm.maximum.z}}}}},
          {"source_edge_ids", e.sourceEdgeIds},
          {"source_count", e.sourceCount},
          {"merged", e.merged},
          {"closed", e.closed},
          {"comparable", e.comparable},
          {"comparable_index", e.comparable ? json(e.comparableIndex) : json(nullptr)},
          {"comparison_role", ToString(e.comparisonRole)},
          {"exclusion_reason", e.exclusionReason.empty() ? json(nullptr) : json(e.exclusionReason)},
      });
    }
    node["edges"] = edgesArr;

    json removedArr = json::array();
    for (const auto &re : audit.removedEdges) {
      removedArr.push_back({{"source_edge_id", re.sourceEdgeId}, {"reason", re.reason}});
    }
    node["removed_edges"] = removedArr;

    node["elapsed_ms"] = audit.elapsedMs;
    node["warning"] = audit.warning;
    return node;
  };

  json normJson = json::object();
  const bool pairUsable = result.referenceNormalization.succeeded && result.candidateNormalization.succeeded;
  normJson["pair_usable"] = pairUsable;
  normJson["reference"] = MakeNormNode(result.referenceNormalization);
  normJson["candidate"] = MakeNormNode(result.candidateNormalization);
  root["normalization"] = normJson;

  // 6. matches
  auto MakeMatchCollectionNode = [](const MatchCollection &col) {
    json node = json::object();
    json summary = json::object();
    summary["attempted"] = col.attempted;
    summary["reference_count"] = col.referenceCount;
    summary["candidate_count"] = col.candidateCount;
    summary["matched_count"] = col.matchedCount;
    summary["ambiguous_count"] = col.ambiguousCount;
    summary["unmatched_reference_count"] = col.unmatchedReferenceIds.size();
    summary["unmatched_candidate_count"] = col.unmatchedCandidateIds.size();
    summary["type_histogram_equal"] = col.typeHistogramEqual;
    summary["all_matched"] = col.allMatched;
    summary["elapsed_ms"] = col.elapsedMs;
    node["summary"] = summary;

    json itemsArr = json::array();
    for (const auto &item : col.items) {
      json itemNode = {
          {"id", item.id},
          {"reference_id", item.referenceId},
          {"candidate_id", item.candidateId},
          {"geometry_type", item.geometryType},
          {"status", ToString(item.status)},
          {"verification_level", ToString(item.verificationLevel)},
          {"reason_codes", item.reasonCodes},
      };
      if (item.score.has_value()) {
        itemNode["score"] = *item.score;
      } else {
        itemNode["score"] = nullptr;
      }
      if (item.metrics.has_value()) {
        itemNode["metrics"] = {
            {"measure_difference", item.metrics->measureDifference},
            {"relative_measure_difference", item.metrics->relativeMeasureDifference},
            {"centroid_distance_mm", item.metrics->centroidDistanceMm},
            {"bounds_difference_mm", item.metrics->boundsDifferenceMm}};
      } else {
        itemNode["metrics"] = nullptr;
      }
      itemsArr.push_back(itemNode);
    }
    node["items"] = itemsArr;
    node["unmatched_reference_ids"] = col.unmatchedReferenceIds;
    node["unmatched_candidate_ids"] = col.unmatchedCandidateIds;
    return node;
  };

  json matchesJson = json::object();
  matchesJson["faces"] = MakeMatchCollectionNode(result.normalizedTopology.faces);
  matchesJson["edges"] = MakeMatchCollectionNode(result.normalizedTopology.edges);
  matchesJson["edge_audit_errors"] = result.normalizedTopology.edgeAuditErrors;
  matchesJson["fast_path"] = {
      {"enabled", result.normalizedTopology.fastPath.enabled},
      {"eligible", result.normalizedTopology.fastPath.eligible},
      {"used", result.normalizedTopology.fastPath.used},
      {"block_reasons", result.normalizedTopology.fastPath.blockReasons}};
  root["matches"] = matchesJson;

  // 6. differences
  json diffs = json::object();
  diffs["missing_material"] = {
      {"succeeded", result.missingMaterial.succeeded},
      {"volume_mm3", result.missingMaterial.volumeMm3},
      {"component_count", result.missingMaterial.componentCount},
      {"kernel_report", result.missingMaterial.report},
  };
  diffs["added_material"] = {
      {"succeeded", result.addedMaterial.succeeded},
      {"volume_mm3", result.addedMaterial.volumeMm3},
      {"component_count", result.addedMaterial.componentCount},
      {"kernel_report", result.addedMaterial.report},
  };
  root["differences"] = diffs;

  // 7. metrics
  json metricsJson = json::object();
  metricsJson["absolute_input_volume_difference_mm3"] = result.absoluteInputVolumeDifferenceMm3;
  metricsJson["relative_input_volume_difference"] = result.relativeInputVolumeDifference;
  metricsJson["centroid_distance_mm"] = result.centroidDistanceMm;
  metricsJson["maximum_bounds_difference_mm"] = result.maximumBoundsDifferenceMm;
  metricsJson["symmetric_difference_volume_mm3"] = result.symmetricDifferenceVolumeMm3;
  metricsJson["symmetric_difference_relative"] = result.symmetricDifferenceRelative;
  root["metrics"] = metricsJson;

  json boolCons = json::object();
  boolCons["signed_input_volume_diff_mm3"] = result.booleanConsistency.signedInputVolumeDiffMm3;
  boolCons["signed_boolean_volume_diff_mm3"] = result.booleanConsistency.signedBooleanVolumeDiffMm3;
  boolCons["conservation_error_mm3"] = result.booleanConsistency.conservationErrorMm3;
  boolCons["relative_conservation_error"] = result.booleanConsistency.relativeConservationError;
  boolCons["conservation_passed"] = result.booleanConsistency.conservationPassed;
  boolCons["boolean_result_valid"] = result.booleanConsistency.booleanResultValid;
  boolCons["invalid_reason"] = result.booleanConsistency.invalidReason;
  root["boolean_consistency"] = boolCons;

  // 8. checks
  const detail::GeometryPassFlags passes =
      detail::EvaluateGeometryPasses(result, tolerances);
  const bool volPass = passes.volume;
  const bool centroidPass = passes.centroid;
  const bool boundsPass = passes.bounds;
  const bool boolPass = detail::EvaluateBooleanPass(result, tolerances);

  const auto &faceCol = result.normalizedTopology.faces;
  const auto &edgeCol = result.normalizedTopology.edges;

  auto FormatDouble = [](double val, int precision = 4) -> std::string {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << val;
    return ss.str();
  };

  json checks = json::array();
  checks.push_back({
      {"id", "input_volume_difference"},
      {"label", "输入实体体积差"},
      {"actual", result.absoluteInputVolumeDifferenceMm3},
      {"actual_text", FormatDouble(result.absoluteInputVolumeDifferenceMm3, 4) + " mm³"},
      {"unit", "mm3"},
      {"relative", result.relativeInputVolumeDifference},
      {"relative_percent", result.relativeInputVolumeDifference * 100.0},
      {"limit", effectiveAbsVolTol},
      {"limit_text", FormatDouble(effectiveAbsVolTol, 4) + " mm³"},
      {"limit_unit", "mm3"},
      {"available", true},
      {"diagnostic", false},
      {"passed", volPass},
  });
  checks.push_back({
      {"id", "centroid_distance"},
      {"label", "质心距离"},
      {"actual", result.centroidDistanceMm},
      {"actual_text", FormatDouble(result.centroidDistanceMm, 4) + " mm"},
      {"unit", "mm"},
      {"relative", nullptr},
      {"relative_percent", nullptr},
      {"limit", effectiveDistTol},
      {"limit_text", FormatDouble(effectiveDistTol, 4) + " mm"},
      {"limit_unit", "mm"},
      {"available", true},
      {"diagnostic", false},
      {"passed", centroidPass},
  });
  checks.push_back({
      {"id", "bounds_difference"},
      {"label", "包围盒尺寸差异"},
      {"actual", result.maximumBoundsDifferenceMm},
      {"actual_text", FormatDouble(result.maximumBoundsDifferenceMm, 4) + " mm"},
      {"unit", "mm"},
      {"relative", nullptr},
      {"relative_percent", nullptr},
      {"limit", effectiveDistTol},
      {"limit_text", FormatDouble(effectiveDistTol, 4) + " mm"},
      {"limit_unit", "mm"},
      {"available", true},
      {"diagnostic", false},
      {"passed", boundsPass},
  });
  checks.push_back({
      {"id", "face_descriptor_match"},
      {"label", "归一化面描述符匹配"},
      {"actual", faceCol.matchedCount},
      {"actual_text", std::to_string(faceCol.matchedCount) + " / " + std::to_string(faceCol.referenceCount)},
      {"unit", "faces"},
      {"relative", faceCol.referenceCount > 0 ? (static_cast<double>(faceCol.matchedCount) / faceCol.referenceCount) : 1.0},
      {"relative_percent", faceCol.referenceCount > 0 ? (faceCol.matchedCount * 100.0 / faceCol.referenceCount) : 100.0},
      {"limit", faceCol.referenceCount},
      {"limit_text", std::to_string(faceCol.referenceCount) + " / " + std::to_string(faceCol.referenceCount)},
      {"limit_unit", "faces"},
      {"available", faceCol.attempted},
      {"diagnostic", true},
      {"passed", faceCol.allMatched},
  });
  checks.push_back({
      {"id", "edge_descriptor_match"},
      {"label", "归一化边描述符匹配"},
      {"actual", edgeCol.matchedCount},
      {"actual_text", std::to_string(edgeCol.matchedCount) + " / " + std::to_string(edgeCol.referenceCount)},
      {"unit", "edges"},
      {"relative", edgeCol.referenceCount > 0 ? (static_cast<double>(edgeCol.matchedCount) / edgeCol.referenceCount) : 1.0},
      {"relative_percent", edgeCol.referenceCount > 0 ? (edgeCol.matchedCount * 100.0 / edgeCol.referenceCount) : 100.0},
      {"limit", edgeCol.referenceCount},
      {"limit_text", std::to_string(edgeCol.referenceCount) + " / " + std::to_string(edgeCol.referenceCount)},
      {"limit_unit", "edges"},
      {"available", edgeCol.attempted},
      {"diagnostic", true},
      {"passed", edgeCol.allMatched},
  });
  checks.push_back({
      {"id", "boolean_cut_residual"},
      {"label", "布尔减法残留体积"},
      {"actual", result.symmetricDifferenceVolumeMm3},
      {"actual_text", FormatDouble(result.symmetricDifferenceVolumeMm3, 4) + " mm³"},
      {"unit", "mm3"},
      {"relative", result.symmetricDifferenceRelative},
      {"relative_percent", result.symmetricDifferenceRelative * 100.0},
      {"limit", effectiveAbsVolTol},
      {"limit_text", FormatDouble(effectiveAbsVolTol, 4) + " mm³"},
      {"limit_unit", "mm3"},
      {"available", result.booleanExecuted},
      {"diagnostic", false},
      {"passed", boolPass},
  });
  root["checks"] = checks;

  // 9. timings_ms
  json timingsJson = json::object();
  timingsJson["load_reference"] = result.timings.loadReferenceMs;
  timingsJson["load_candidate"] = result.timings.loadCandidateMs;
  timingsJson["normalize_reference"] = result.timings.normalizeReferenceMs;
  timingsJson["normalize_candidate"] = result.timings.normalizeCandidateMs;
  timingsJson["descriptor_build"] = result.timings.descriptorBuildMs;
  timingsJson["face_match"] = result.timings.faceMatchMs;
  timingsJson["edge_match"] = result.timings.edgeMatchMs;
  timingsJson["boolean_ab"] = result.timings.booleanAbMs;
  timingsJson["boolean_ba"] = result.timings.booleanBaMs;
  timingsJson["artifact_export"] = result.timings.artifactExportMs;
  timingsJson["total"] = result.timings.totalMs;
  root["timings_ms"] = timingsJson;

  // 10. artifacts
  json artJson = json::object();
  for (const auto &item : result.artifacts.items) {
    json info = json::object();
    info["path"] = item.relativePath;
    info["format"] = item.format;
    info["available"] = item.available;
    if (!item.entityIndexArray.empty()) {
      info["entity_index_array"] = item.entityIndexArray;
    }
    artJson[item.key] = info;
  }
  root["artifacts"] = artJson;

  return root.dump(2) + '\n';
}
} // namespace cadstep
