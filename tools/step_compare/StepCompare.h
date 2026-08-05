#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace cadstep {

enum class CompareStatus {
  Equal,
  Different,
  InvalidInput,
  UnsupportedShape,
  Indeterminate,
  InternalError,
};

enum class MatchStatus {
  Matched,
  Unmatched,
  Ambiguous,
  Unsupported,
};

enum class VerificationLevel {
  TypeOnly,
  Descriptor,
  AnalyticSupport,
  DistanceVerified,
  BooleanVerified,
};

enum class EntitySide {
  Reference,
  Candidate,
};

enum class EntityKind {
  OriginalFace,
  OriginalEdge,
  NormalizedFace,
  NormalizedEdge,
};

enum class EdgeComparisonRole {
  Comparable,
  PeriodicSeam,
  Degenerated,
  Unsupported,
};

const char *ToString(EdgeComparisonRole role);

struct CompareConfig {
  double distanceToleranceMm = 0.01;
  double absoluteVolumeToleranceMm3 = 0.001;
  double relativeVolumeTolerance = 0.001;
  double booleanFuzzyToleranceMm = 0.01;

  bool enableSameDomainNormalization = true;
  double normalizationLinearToleranceMm = 0.001;
  double normalizationAngularToleranceRad = 1.0e-6;
  bool enableNormalizedFastPath = false;
  double ambiguousMatchMargin = 0.02;

  bool exportStl = true;
  bool exportBrep = true;
  bool exportEntityVtp = false;
  bool writeEntityDetails = true;

  bool printHumanSummary = true;
};

struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Bounds3 {
  Point3 minimum;
  Point3 maximum;
  bool isVoid = true;
};

struct TypeStatistics {
  std::string type;
  int count = 0;
  double totalMeasure = 0.0;
};

struct NormalizedFaceInfo {
  std::string id;
  int visualIndex = 0;

  std::string surfaceType;

  double areaMm2 = 0.0;
  Point3 centroidMm;
  Bounds3 boundsMm;

  std::vector<std::string> sourceFaceIds;
  int sourceCount = 0;
  bool merged = false;

  std::optional<double> radiusMm;
  std::optional<Point3> axisOriginMm;
  std::optional<Point3> axisDirection;
};

struct NormalizedEdgeInfo {
  std::string id;
  int visualIndex = 0;

  std::string curveType;

  double lengthMm = 0.0;
  Point3 centroidMm;
  Bounds3 boundsMm;

  std::vector<std::string> sourceEdgeIds;
  int sourceCount = 0;
  bool merged = false;
  bool closed = false;
  bool comparable = false;
  int comparableIndex = 0;

  EdgeComparisonRole comparisonRole = EdgeComparisonRole::Comparable;
  std::string exclusionReason;
};

struct RemovedEdgeInfo {
  std::string sourceEdgeId;
  std::string reason; // PERIODIC_SEAM, DEGENERATED, SAME_DOMAIN_INTERNAL_EDGE, REMOVED_BY_NORMALIZATION, UNKNOWN
};

struct MatchMetrics {
  double measureDifference = 0.0;
  double relativeMeasureDifference = 0.0;
  double centroidDistanceMm = 0.0;
  double boundsDifferenceMm = 0.0;
};

struct EntityMatch {
  std::string id;

  std::string referenceId;
  std::string candidateId;

  MatchStatus status = MatchStatus::Unmatched;
  VerificationLevel verificationLevel = VerificationLevel::Descriptor;

  std::string geometryType;

  std::optional<double> score;
  std::optional<MatchMetrics> metrics;

  std::vector<std::string> reasonCodes;
};

struct MatchCollection {
  bool attempted = false;

  int referenceCount = 0;
  int candidateCount = 0;
  int matchedCount = 0;
  int ambiguousCount = 0;

  bool typeHistogramEqual = false;
  bool allMatched = false;

  double elapsedMs = 0.0;

  std::vector<EntityMatch> items;

  std::vector<std::string> unmatchedReferenceIds;
  std::vector<std::string> unmatchedCandidateIds;
};

struct NormalizationAudit {
  bool enabled = false;
  bool succeeded = false;
  bool usedNormalizedShape = false;

  int faceCountBefore = 0;
  int faceCountAfter = 0;

  int edgeCountBefore = 0;
  int edgeCountAfter = 0;

  int comparableEdgeCountBefore = 0;
  int comparableEdgeCountAfter = 0;

  double volumeBeforeMm3 = 0.0;
  double volumeAfterMm3 = 0.0;
  double relativeVolumeDrift = 0.0;

  bool faceMappingComplete = false;
  bool edgeMappingComplete = false;
  bool mappingComplete = false;

  double elapsedMs = 0.0;
  std::string warning;

  std::vector<TypeStatistics> faceTypes;
  std::vector<TypeStatistics> edgeTypes;

  std::vector<NormalizedFaceInfo> faces;
  std::vector<NormalizedEdgeInfo> edges;
  std::vector<RemovedEdgeInfo> removedEdges;
};

struct FastPathAudit {
  bool enabled = false;
  bool eligible = false;
  bool used = false;
  std::vector<std::string> blockReasons;
};

struct EdgeAuditValidation {
  bool valid = true;
  std::vector<std::string> errors;
};

struct TopologyMatchAudit {
  bool attempted = false;
  std::string skipReason;

  MatchCollection faces;
  MatchCollection edges;

  FastPathAudit fastPath;

  bool normalizedTopologyMatch = false;
  bool edgeAuditConsistent = true;
  std::vector<std::string> edgeAuditErrors;
  double elapsedMs = 0.0;
};

struct ArtifactInfo {
  std::string key;
  std::string relativePath;
  std::string format;
  bool available = false;
  std::string entityIndexArray;
};

struct ArtifactAudit {
  std::vector<ArtifactInfo> items;
};

struct TimingAudit {
  double loadReferenceMs = 0.0;
  double loadCandidateMs = 0.0;
  double normalizeReferenceMs = 0.0;
  double normalizeCandidateMs = 0.0;
  double descriptorBuildMs = 0.0;
  double faceMatchMs = 0.0;
  double edgeMatchMs = 0.0;
  double booleanAbMs = 0.0;
  double booleanBaMs = 0.0;
  double artifactExportMs = 0.0;
  double totalMs = 0.0;
};

struct InputAudit {
  std::string path;
  std::vector<std::string> fileLengthUnits;
  std::string loadDiagnostics;
  std::string transferDiagnostics;
  int solidCount = 0;
  int shellCount = 0;
  int faceCount = 0;
  int edgeCount = 0;
  bool brepValid = false;
  bool closed = false;
  double signedVolumeMm3 = 0.0;
  double surfaceAreaMm2 = 0.0;
  Point3 centroidMm;
  Bounds3 boundsMm;
};

struct DifferenceAudit {
  bool succeeded = false;
  double volumeMm3 = 0.0;
  int componentCount = 0;
  std::string report;
};

struct CompareResult {
  CompareStatus status = CompareStatus::InternalError;
  std::string reason;
  CompareConfig thresholds;
  InputAudit reference;
  InputAudit candidate;
  DifferenceAudit missingMaterial;
  DifferenceAudit addedMaterial;
  NormalizationAudit referenceNormalization;
  NormalizationAudit candidateNormalization;
  TopologyMatchAudit normalizedTopology;
  ArtifactAudit artifacts;
  TimingAudit timings;
  bool booleanExecuted = false;
  std::string decisionPath;
  double absoluteInputVolumeDifferenceMm3 = 0.0;
  double relativeInputVolumeDifference = 0.0;
  double centroidDistanceMm = 0.0;
  double maximumBoundsDifferenceMm = 0.0;
  double symmetricDifferenceVolumeMm3 = 0.0;
  double symmetricDifferenceRelative = 0.0;
};

std::string MakeEntityId(EntitySide side, EntityKind kind, int index);

CompareResult CompareStepFiles(const std::filesystem::path &reference,
                               const std::filesystem::path &candidate,
                               const CompareConfig &config,
                               const std::filesystem::path &outputDirectory = "");

const char *ToString(CompareStatus status);
const char *ToString(MatchStatus status);
const char *ToString(VerificationLevel level);

int ExitCode(CompareStatus status);
std::string ToJson(const CompareResult &result);
std::string ToHumanSummary(const CompareResult &result);
bool WriteResultJson(const std::filesystem::path &outputDirectory,
                     const CompareResult &result, std::string &error);

} // namespace cadstep
