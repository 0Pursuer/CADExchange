#include "../service/geometry/GeometryCollectorBase.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
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
using ComparisonResult = CADExchange::Geometry::GeometryCollectorBase<DummyCollector, CADExchange::CRefEdge>::ComparisonResult;
using json = nlohmann::json;

struct CompareOptions {
  std::filesystem::path srcPath;
  std::filesystem::path dstPath;
  std::string srcUnit; // optional: convert src geometry to this unit on load
  std::string dstUnit; // optional: convert dst geometry to this unit on load
  double tol = 2e-3;
};

struct DumpOptions {
  std::filesystem::path srcPath;
  std::filesystem::path outPath;
};

struct FeatureDiff {
  std::string featureId;
  std::vector<std::string> edgeDiffs;
};

std::string QuoteJson(const std::string &value) {
  std::string out;
  out.reserve(value.size() + 8);
  for (unsigned char ch : value) {
    switch (ch) {
    case '\\': out += "\\\\"; break;
    case '"': out += "\\\""; break;
    case '\b': out += "\\b"; break;
    case '\f': out += "\\f"; break;
    case '\n': out += "\\n"; break;
    case '\r': out += "\\r"; break;
    case '\t': out += "\\t"; break;
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

bool ParseCompareArgs(int argc, char *argv[], CompareOptions &out,
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
    if (arg == "--src-unit" && i + 1 < argc) {
      out.srcUnit = argv[++i];
      continue;
    }
    if (arg == "--dst-unit" && i + 1 < argc) {
      out.dstUnit = argv[++i];
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
          "usage: test_geom --src <geometry.json> --dst <geometry.json>"
          " [--src-unit <unit>] [--dst-unit <unit>] [--tol <double>]\n"
          "  unit examples: m, mm, cm, in, ft";
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

bool ParseDumpArgs(int argc, char *argv[], DumpOptions &out,
                   std::string &errorMessage) {
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "--src" || arg == "--src-geometry") && i + 1 < argc) {
      out.srcPath = argv[++i];
      continue;
    }
    if ((arg == "--out" || arg == "--dst") && i + 1 < argc) {
      out.outPath = argv[++i];
      continue;
    }
    errorMessage = "unknown argument: " + arg;
    return false;
  }
  if (out.srcPath.empty()) {
    errorMessage = "missing --src";
    return false;
  }
  if (out.outPath.empty()) {
    errorMessage = "missing --out";
    return false;
  }
  return true;
}

bool TryLoadGeometrySet(const std::filesystem::path &path, GeometrySet &set,
                        std::string &errorMessage,
                        const std::string &target_unit = "") {
  return set.LoadFromJson(path, &errorMessage, target_unit);
}

bool TryLoadSingleCollector(const std::filesystem::path &path, Collector &collector,
                            std::string &errorMessage) {
  try {
    std::ifstream in(path);
    if (!in.is_open()) {
      errorMessage = "Unable to open geometry json input: " + path.string();
      return false;
    }
    json root = json::parse(in);
    const json *payload = &root;
    const auto geometryIt = root.find("Geometry");
    if (geometryIt != root.end() && geometryIt->is_object()) {
      payload = &(*geometryIt);
    }
    return collector.LoadFromJsonValue(*payload, &errorMessage);
  } catch (const std::exception &e) {
    errorMessage = "Failed to parse single-collector geometry json: " +
                   std::string(e.what());
    return false;
  }
}

bool TryLoadGeometry(const std::filesystem::path &path, GeometrySet &set,
                     bool &isFlatGeometry, std::string &errorMessage,
                     const std::string &target_unit = "") {
  if (TryLoadGeometrySet(path, set, errorMessage, target_unit)) {
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
                             const std::vector<std::string> &diffs,
                             const std::vector<FeatureDiff> &featureDiffs) {
  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"equivalent\": " << (equivalent ? "true" : "false") << ",\n";
  oss << "  \"source_feature_count\": " << srcSet.features.size() << ",\n";
  oss << "  \"target_feature_count\": " << dstSet.features.size() << ",\n";
  oss << "  \"failed_feature_count\": " << featureDiffs.size() << ",\n";
  oss << "  \"diffs\": [";
  if (!diffs.empty()) {
    oss << "\n";
    for (std::size_t i = 0; i < diffs.size(); ++i) {
      oss << "    \"" << QuoteJson(diffs[i]) << "\"";
      if (i + 1 < diffs.size()) oss << ",";
      oss << "\n";
    }
    oss << "  ],\n";
  } else {
    oss << "],\n";
  }
  oss << "  \"feature_diffs\": [";
  if (!featureDiffs.empty()) {
    oss << "\n";
    for (std::size_t i = 0; i < featureDiffs.size(); ++i) {
      const auto &fd = featureDiffs[i];
      oss << "    {\n";
      oss << "      \"feature_id\": \"" << QuoteJson(fd.featureId) << "\",\n";
      oss << "      \"edge_diffs\": [";
      if (!fd.edgeDiffs.empty()) {
        oss << "\n";
        for (std::size_t j = 0; j < fd.edgeDiffs.size(); ++j) {
          oss << "        \"" << QuoteJson(fd.edgeDiffs[j]) << "\"";
          if (j + 1 < fd.edgeDiffs.size()) oss << ",";
          oss << "\n";
        }
        oss << "      ]\n";
      } else {
        oss << "]\n";
      }
      oss << "    }";
      if (i + 1 < featureDiffs.size()) oss << ",";
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
                 double tol, std::vector<std::string> &diffs,
                 std::vector<FeatureDiff> &featureDiffs) {
  bool equivalent = true;

  std::vector<CADExchange::CRefEdge> all_src_edges, all_dst_edges;
  for (const auto& [featureId, srcCollector] : srcSet.features) {
    for (const auto& edge : srcCollector.GetEdges()) {
      all_src_edges.push_back(edge);
    }
  }
  for (const auto& [featureId, dstCollector] : dstSet.features) {
    for (const auto& edge : dstCollector.GetEdges()) {
      all_dst_edges.push_back(edge);
    }
  }

  auto global_src_groups = Collector::ExtractHalfStructureGroups(all_src_edges, tol);
  auto global_dst_groups = Collector::ExtractHalfStructureGroups(all_dst_edges, tol);

  for (const auto &[featureId, srcCollector] : srcSet.features) {
    auto dstIt = dstSet.features.find(featureId);
    if (dstIt == dstSet.features.end()) {
      diffs.push_back("missing target feature: " + featureId);
      featureDiffs.push_back(FeatureDiff{featureId, {"DST missing feature collector"}});
      equivalent = false;
      continue;
    }

    ComparisonResult comparison = srcCollector.CompareDetailed(dstIt->second, tol, &global_src_groups, &global_dst_groups);
    if (!comparison.equivalent) {
      diffs.push_back("feature mismatch: " + featureId);
      featureDiffs.push_back(FeatureDiff{featureId, std::move(comparison.diagnostics)});
      equivalent = false;
    }
  }


  for (const auto &[featureId, unusedCollector] : dstSet.features) {
    (void)unusedCollector;
    if (srcSet.features.find(featureId) == srcSet.features.end()) {
      diffs.push_back("unexpected target feature: " + featureId);
      featureDiffs.push_back(FeatureDiff{featureId, {"DST has extra feature collector"}});
      equivalent = false;
    }
  }

  std::sort(featureDiffs.begin(), featureDiffs.end(), [](const FeatureDiff &a, const FeatureDiff &b) {
    return a.featureId < b.featureId;
  });
  return equivalent;
}

bool DumpGeometrySet(const DumpOptions &options, std::string &errorMessage) {
  GeometrySet set;
  if (!TryLoadGeometrySet(options.srcPath, set, errorMessage)) {
    return false;
  }
  return set.SaveToJson(options.outPath, &errorMessage);
}

bool DumpFlatGeometry(const DumpOptions &options, std::string &errorMessage) {
  Collector collector;
  if (!TryLoadSingleCollector(options.srcPath, collector, errorMessage)) {
    return false;
  }
  try {
    json root{{"Geometry", collector.ToJsonValue()}};
    std::ofstream out(options.outPath, std::ios::trunc);
    if (!out.is_open()) {
      errorMessage = "Unable to open output: " + options.outPath.string();
      return false;
    }
    out << root.dump(2) << '\n';
    return true;
  } catch (const std::exception &e) {
    errorMessage = e.what();
    return false;
  }
}

} // namespace

int main(int argc, char *argv[]) {
  if (argc >= 2) {
    const std::string command = argv[1];
    if (command == "dump" || command == "dump-flat") {
      DumpOptions dumpOptions;
      std::string dumpParseError;
      if (!ParseDumpArgs(argc, argv, dumpOptions, dumpParseError)) {
        std::cerr << dumpParseError << std::endl;
        return 2;
      }
      std::string ioError;
      const bool ok = (command == "dump") ? DumpGeometrySet(dumpOptions, ioError)
                                            : DumpFlatGeometry(dumpOptions, ioError);
      if (!ok) {
        std::cerr << ioError << std::endl;
        return 2;
      }
      std::cout << (command == "dump" ? "dumped" : "dumped-flat") << std::endl;
      return 0;
    }
  }

  CompareOptions options;
  std::string parseError;
  if (!ParseCompareArgs(argc, argv, options, parseError)) {
    std::cerr << parseError << std::endl;
    return StartsWith(parseError, "usage:") ? 0 : 2;
  }

  GeometrySet srcSet;
  GeometrySet dstSet;
  bool srcFlat = false;
  bool dstFlat = false;
  std::string loadError;
  if (!TryLoadGeometry(options.srcPath, srcSet, srcFlat, loadError, options.srcUnit)) {
    std::cerr << loadError << std::endl;
    return 2;
  }
  if (!TryLoadGeometry(options.dstPath, dstSet, dstFlat, loadError, options.dstUnit)) {
    std::cerr << loadError << std::endl;
    return 2;
  }

  if (srcFlat != dstFlat) {
    std::vector<std::string> diffs;
    std::vector<FeatureDiff> featureDiffs;
    diffs.push_back("geometry payload shape mismatch: one side flat, the other model-level");
    std::cout << BuildSummaryJson(false, srcSet, dstSet, diffs, featureDiffs);
    return 1;
  }

  std::vector<std::string> diffs;
  std::vector<FeatureDiff> featureDiffs;
  const bool equivalent = CompareSets(srcSet, dstSet, options.tol, diffs, featureDiffs);
  std::cout << BuildSummaryJson(equivalent, srcSet, dstSet, diffs, featureDiffs);
  return equivalent ? 0 : 1;
}
