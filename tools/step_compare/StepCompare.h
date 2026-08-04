#pragma once

#include <filesystem>
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

struct CompareConfig {
  double distanceToleranceMm = 0.01;
  double absoluteVolumeToleranceMm3 = 0.000001;
  double relativeVolumeTolerance = 1.0e-8;
  double booleanFuzzyToleranceMm = 0.01;

  bool enableSameDomainNormalization = true;
  double normalizationLinearToleranceMm = 0.001;
  double normalizationAngularToleranceRad = 1.0e-6;
  bool enableNormalizedFastPath = false;
  bool printHumanSummary = true;
};

struct TypeStatistics {
  std::string type;
  int count = 0;
  double totalMeasure = 0.0;
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
  double elapsedMs = 0.0;
  std::string warning;
  std::vector<TypeStatistics> faceTypes;
  std::vector<TypeStatistics> edgeTypes;
};

struct TopologyMatchAudit {
  bool attempted = false;
  int referenceFaceCount = 0;
  int candidateFaceCount = 0;
  int matchedFaceCount = 0;
  int referenceEdgeCount = 0;
  int candidateEdgeCount = 0;
  int matchedEdgeCount = 0;
  int unmatchedReferenceFaces = 0;
  int unmatchedCandidateFaces = 0;
  int unmatchedReferenceEdges = 0;
  int unmatchedCandidateEdges = 0;
  bool faceTypeHistogramEqual = false;
  bool edgeTypeHistogramEqual = false;
  bool allFacesMatched = false;
  bool allEdgesMatched = false;
  bool normalizedTopologyMatch = false;
  double elapsedMs = 0.0;
};

struct TimingAudit {
  double loadReferenceMs = 0.0;
  double loadCandidateMs = 0.0;
  double normalizeReferenceMs = 0.0;
  double normalizeCandidateMs = 0.0;
  double topologyMatchMs = 0.0;
  double booleanAbMs = 0.0;
  double booleanBaMs = 0.0;
  double artifactExportMs = 0.0;
  double totalMs = 0.0;
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

CompareResult CompareStepFiles(const std::filesystem::path &reference,
                               const std::filesystem::path &candidate,
                               const CompareConfig &config,
                               const std::filesystem::path &outputDirectory = "");

const char *ToString(CompareStatus status);
int ExitCode(CompareStatus status);
std::string ToJson(const CompareResult &result);
std::string ToHumanSummary(const CompareResult &result);
bool WriteResultJson(const std::filesystem::path &outputDirectory,
                     const CompareResult &result, std::string &error);

} // namespace cadstep
