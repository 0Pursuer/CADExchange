#include "../service/geometry/GeometryCollectorBase.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class DummyCollector
    : public CADExchange::Geometry::GeometryCollectorBase<DummyCollector,
                                                          CADExchange::CRefEdge> {
public:
  int CollectImpl() { return 0; }
};

namespace {

using Collector = DummyCollector;
using GeometrySet = CADExchange::Geometry::ModelGeometrySet<Collector>;

struct CompareOptions {
  std::filesystem::path srcPath;
  std::filesystem::path dstPath;
  double tol = 2e-3;
};

std::string QuoteJson(const std::string &value) {
  std::string out;
  out.reserve(value.size() + 8);
  for (unsigned char ch : value) {
    switch (ch) {
    case '\\':
      out += "\\\\";
      break;
    case '"':
      out += "\\\"";
      break;
    case '\b':
      out += "\\b";
      break;
    case '\f':
      out += "\\f";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      if (ch < 0x20) {
        static const char kHex[] = "0123456789ABCDEF";
        out += "\\u00";
        out.push_back(kHex[(ch >> 4) & 0xF]);
        out.push_back(kHex[ch & 0xF]);
      } else {
        out.push_back(static_cast<char>(ch));
      }
      break;
    }
  }
  return out;
}

bool StartsWith(const std::string &value, const std::string &prefix) {
  return value.size() >= prefix.size() &&
         value.compare(0, prefix.size(), prefix) == 0;
}

bool ParseArgs(int argc, char *argv[], CompareOptions &out,
               std::string &errorMessage) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "--src" || arg == "--src-geometry") && i + 1 < argc) {
      out.srcPath = argv[++i];
      continue;
    }
    if ((arg == "--dst" || arg == "--dst-geometry") && i + 1 < argc) {
      out.dstPath = argv[++i];
      continue;
    }
    if (arg == "--tol" && i + 1 < argc) {
      try {
        out.tol = std::stod(argv[++i]);
      } catch (const std::exception &) {
        errorMessage = "invalid --tol value";
        return false;
      }
      continue;
    }
    if (arg == "--help" || arg == "-h") {
      errorMessage =
          "usage: test_geom --src <geometry.json> --dst <geometry.json> [--tol <double>]";
      return false;
    }

    errorMessage = "unknown argument: " + arg;
    return false;
  }

  if (out.srcPath.empty()) {
    errorMessage = "missing --src";
    return false;
  }
  if (out.dstPath.empty()) {
    errorMessage = "missing --dst";
    return false;
  }
  return true;
}

bool TryLoadGeometrySet(const std::filesystem::path &path, GeometrySet &set,
                        std::string &errorMessage) {
  return set.LoadFromJson(path, &errorMessage);
}

bool TryLoadSingleCollector(const std::filesystem::path &path, Collector &collector,
                            std::string &errorMessage) {
  std::ifstream in(path);
  if (!in.is_open()) {
    errorMessage = "Unable to open geometry json input: " + path.string();
    return false;
  }
  try {
    cereal::JSONInputArchive archive(in);
    archive(cereal::make_nvp("Geometry", collector));
    return true;
  } catch (const std::exception &e) {
    errorMessage = "Failed to parse single-collector geometry json: " +
                   std::string(e.what());
    return false;
  }
}

bool TryLoadGeometry(const std::filesystem::path &path, GeometrySet &set,
                     bool &isFlatGeometry, std::string &errorMessage) {
  if (TryLoadGeometrySet(path, set, errorMessage)) {
    isFlatGeometry = false;
    return true;
  }

  Collector collector;
  std::string flatError;
  if (!TryLoadSingleCollector(path, collector, flatError)) {
    errorMessage += "; fallback failed: " + flatError;
    return false;
  }

  set.features.clear();
  set.features.emplace("__flat__", std::move(collector));
  isFlatGeometry = true;
  errorMessage.clear();
  return true;
}

std::string BuildSummaryJson(bool equivalent, const GeometrySet &srcSet,
                             const GeometrySet &dstSet,
                             const std::vector<std::string> &diffs) {
  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"equivalent\": " << (equivalent ? "true" : "false") << ",\n";
  oss << "  \"source_feature_count\": " << srcSet.features.size() << ",\n";
  oss << "  \"target_feature_count\": " << dstSet.features.size() << ",\n";
  oss << "  \"diffs\": [";
  if (!diffs.empty()) {
    oss << "\n";
    for (std::size_t i = 0; i < diffs.size(); ++i) {
      oss << "    \"" << QuoteJson(diffs[i]) << "\"";
      if (i + 1 < diffs.size()) {
        oss << ",";
      }
      oss << "\n";
    }
    oss << "  ]\n";
  } else {
    oss << "]\n";
  }
  oss << "}\n";
  return oss.str();
}

bool CompareSets(const GeometrySet &srcSet, const GeometrySet &dstSet,
                 double tol, std::vector<std::string> &diffs) {
  bool equivalent = true;

  for (const auto &[featureId, srcCollector] : srcSet.features) {
    auto dstIt = dstSet.features.find(featureId);
    if (dstIt == dstSet.features.end()) {
      diffs.push_back("missing target feature: " + featureId);
      equivalent = false;
      continue;
    }
    if (!srcCollector.IsEquivalent(dstIt->second, tol)) {
      diffs.push_back("feature mismatch: " + featureId);
      equivalent = false;
    }
  }

  for (const auto &[featureId, unusedCollector] : dstSet.features) {
    (void)unusedCollector;
    if (srcSet.features.find(featureId) == srcSet.features.end()) {
      diffs.push_back("unexpected target feature: " + featureId);
      equivalent = false;
    }
  }

  return equivalent;
}

} // namespace

int main(int argc, char *argv[]) {
  CompareOptions options;
  std::string parseError;
  if (!ParseArgs(argc, argv, options, parseError)) {
    std::cerr << parseError << std::endl;
    return StartsWith(parseError, "usage:") ? 0 : 2;
  }

  GeometrySet srcSet;
  GeometrySet dstSet;
  bool srcFlat = false;
  bool dstFlat = false;
  std::string loadError;
  if (!TryLoadGeometry(options.srcPath, srcSet, srcFlat, loadError)) {
    std::cerr << loadError << std::endl;
    return 2;
  }
  if (!TryLoadGeometry(options.dstPath, dstSet, dstFlat, loadError)) {
    std::cerr << loadError << std::endl;
    return 2;
  }

  if (srcFlat != dstFlat) {
    std::vector<std::string> diffs;
    diffs.push_back("geometry payload shape mismatch: one side flat, the other model-level");
    std::cout << BuildSummaryJson(false, srcSet, dstSet, diffs);
    return 1;
  }

  std::vector<std::string> diffs;
  const bool equivalent = CompareSets(srcSet, dstSet, options.tol, diffs);
  std::cout << BuildSummaryJson(equivalent, srcSet, dstSet, diffs);
  return equivalent ? 0 : 1;
}
