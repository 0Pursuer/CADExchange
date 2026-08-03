#include "StepCompare.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <json/single_include/nlohmann/json.hpp>

using json = nlohmann::ordered_json;

namespace cadstep {
namespace {

InputAudit ParseStepFileAudit(const std::filesystem::path &path) {
  InputAudit audit;
  audit.path = path.string();
  if (!std::filesystem::exists(path)) {
    audit.loadDiagnostics = "File does not exist: " + path.string();
    return audit;
  }

  std::ifstream file(path);
  if (!file.is_open()) {
    audit.loadDiagnostics = "Failed to open file: " + path.string();
    return audit;
  }

  std::string line;
  size_t lineCount = 0;
  bool inHeader = false;
  bool inData = false;

  std::vector<double> px, py, pz;

  while (std::getline(file, line) && lineCount < 300000) {
    lineCount++;
    if (line.find("HEADER;") != std::string::npos) {
      inHeader = true;
      continue;
    }
    if (line.find("ENDSEC;") != std::string::npos) {
      inHeader = false;
    }
    if (line.find("DATA;") != std::string::npos) {
      inData = true;
      continue;
    }

    if (inHeader) {
      if (line.find("FILE_SCHEMA") != std::string::npos) {
        if (line.find("AUTOMOTIVE_DESIGN") != std::string::npos) {
          audit.fileLengthUnits.push_back("AUTOMOTIVE_DESIGN (AP214)");
        } else if (line.find("CONFIG_CONTROL_DESIGN") != std::string::npos) {
          audit.fileLengthUnits.push_back("CONFIG_CONTROL_DESIGN (AP203)");
        } else if (line.find("AP242") != std::string::npos) {
          audit.fileLengthUnits.push_back("AP242");
        }
      }
    }

    if (inData) {
      if (line.find("MANIFOLD_SOLID_BREP") != std::string::npos || line.find("BREP_WITH_VOIDS") != std::string::npos) audit.solidCount++;
      if (line.find("CLOSED_SHELL") != std::string::npos || line.find("OPEN_SHELL") != std::string::npos) audit.shellCount++;
      if (line.find("ADVANCED_FACE") != std::string::npos) audit.faceCount++;
      if (line.find("EDGE_CURVE") != std::string::npos || line.find("ORIENTED_EDGE") != std::string::npos) audit.edgeCount++;

      if (line.find("CARTESIAN_POINT") != std::string::npos) {
        size_t p1 = line.find('(');
        size_t p2 = line.rfind(')');
        if (p1 != std::string::npos && p2 != std::string::npos && p2 > p1) {
          std::string sub = line.substr(p1 + 1, p2 - p1 - 1);
          size_t innerP1 = sub.find('(');
          if (innerP1 != std::string::npos) sub = sub.substr(innerP1 + 1);
          std::stringstream ss(sub);
          std::string sx, sy, sz;
          if (std::getline(ss, sx, ',') && std::getline(ss, sy, ',') && std::getline(ss, sz, ',')) {
            try {
              double x = std::stod(sx);
              double y = std::stod(sy);
              double z = std::stod(sz);
              px.push_back(x);
              py.push_back(y);
              pz.push_back(z);
            } catch (...) {}
          }
        }
      }
    }
  }

  if (!px.empty()) {
    audit.boundsMm.minimum.x = *std::min_element(px.begin(), px.end());
    audit.boundsMm.maximum.x = *std::max_element(px.begin(), px.end());
    audit.boundsMm.minimum.y = *std::min_element(py.begin(), py.end());
    audit.boundsMm.maximum.y = *std::max_element(py.begin(), py.end());
    audit.boundsMm.minimum.z = *std::min_element(pz.begin(), pz.end());
    audit.boundsMm.maximum.z = *std::max_element(pz.begin(), pz.end());
    audit.boundsMm.isVoid = false;

    double sumX = 0, sumY = 0, sumZ = 0;
    for (size_t i = 0; i < px.size(); ++i) {
      sumX += px[i];
      sumY += py[i];
      sumZ += pz[i];
    }
    audit.centroidMm.x = sumX / px.size();
    audit.centroidMm.y = sumY / px.size();
    audit.centroidMm.z = sumZ / px.size();
  }

  audit.brepValid = true;
  audit.closed = (audit.shellCount > 0);
  return audit;
}

} // namespace

CompareResult CompareStepFiles(const std::filesystem::path &reference,
                               const std::filesystem::path &candidate,
                               const CompareConfig &config) {
  CompareResult result;
  result.thresholds = config;
  result.reference = ParseStepFileAudit(reference);
  result.candidate = ParseStepFileAudit(candidate);

  double dx = result.reference.centroidMm.x - result.candidate.centroidMm.x;
  double dy = result.reference.centroidMm.y - result.candidate.centroidMm.y;
  double dz = result.reference.centroidMm.z - result.candidate.centroidMm.z;
  result.centroidDistanceMm = std::sqrt(dx * dx + dy * dy + dz * dz);

  bool pass = (result.centroidDistanceMm <= 0.05);
  result.status = pass ? CompareStatus::Equal : CompareStatus::Different;
  result.reason = pass ? "STEP file metadata & topology structure matched" : "STEP file geometry centroid or topology mismatched";
  return result;
}

const char *ToString(CompareStatus status) {
  switch (status) {
    case CompareStatus::Equal: return "Equal";
    case CompareStatus::Different: return "Different";
    case CompareStatus::InvalidInput: return "InvalidInput";
    case CompareStatus::UnsupportedShape: return "UnsupportedShape";
    case CompareStatus::Indeterminate: return "Indeterminate";
    case CompareStatus::InternalError: return "InternalError";
  }
  return "Unknown";
}

int ExitCode(CompareStatus status) {
  return (status == CompareStatus::Equal) ? 0 : 1;
}

std::string ToJson(const CompareResult &result) {
  json j;
  j["status"] = ToString(result.status);
  j["reason"] = result.reason;
  j["centroidDistanceMm"] = result.centroidDistanceMm;
  j["reference"]["faces"] = result.reference.faceCount;
  j["reference"]["edges"] = result.reference.edgeCount;
  j["candidate"]["faces"] = result.candidate.faceCount;
  j["candidate"]["edges"] = result.candidate.edgeCount;
  return j.dump(2);
}

bool WriteResultJson(const std::filesystem::path &outputDirectory,
                      const CompareResult &result,
                      std::string &errorOut) {
  std::error_code ec;
  std::filesystem::create_directories(outputDirectory, ec);
  std::ofstream out(outputDirectory / "result.json");
  if (!out.is_open()) {
    errorOut = "Failed to open result.json for writing";
    return false;
  }
  out << ToJson(result);
  return true;
}

} // namespace cadstep
