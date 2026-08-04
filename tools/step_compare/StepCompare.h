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
bool WriteResultJson(const std::filesystem::path &outputDirectory,
                     const CompareResult &result, std::string &error);

} // namespace cadstep
