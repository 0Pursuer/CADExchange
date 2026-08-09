#include "StepCompare.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct CliOptions {
  std::filesystem::path reference;
  std::filesystem::path candidate;
  std::filesystem::path output;
  cadstep::CompareConfig config;
  bool printJsonStdout = false;
  bool help = false;
};

void PrintUsage() {
  std::cout << "Usage: cad_step_compare"
               " --reference <source.step>"
               " --candidate <target.step>"
               " --output <directory>"
               " [--distance-tol-mm 0.01]"
               " [--abs-volume-tol-mm3 0.000001]"
               " [--rel-volume-tol 1e-8]"
               " [--boolean-fuzzy-tol-mm 0.01]"
               " [--normalize-same-domain]"
               " [--no-normalize-same-domain]"
               " [--normalize-linear-tol-mm 0.001]"
               " [--normalize-angular-tol-rad 1e-6]"
               " [--normalized-fast-path]"
               " [--allow-multi-solid]"
               " [--multi-solid-policy strict|collection|pairwise]"
               " [--solid-match-volume-rel-tol 1e-4]"
               " [--solid-match-centroid-tol-mm 0.1]"
               " [--solid-match-bounds-tol-mm 0.1]"
               " [--quiet]\n";
}

double ParseNumber(const std::wstring &text, const char *name) {
  std::size_t parsed = 0;
  const double value = std::stod(text, &parsed);
  if (parsed != text.size()) {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
  return value;
}

CliOptions ParseArguments(const std::vector<std::wstring> &arguments) {
  CliOptions options;
  for (std::size_t index = 0; index < arguments.size(); ++index) {
    const std::wstring &argument = arguments[index];
    const auto requireValue = [&]() -> const std::wstring & {
      if (index + 1 >= arguments.size()) {
        throw std::invalid_argument("missing value for command-line option");
      }
      return arguments[++index];
    };

    if (argument == L"--reference") {
      options.reference = requireValue();
    } else if (argument == L"--candidate") {
      options.candidate = requireValue();
    } else if (argument == L"--output") {
      options.output = requireValue();
    } else if (argument == L"--distance-tol-mm") {
      options.config.distanceToleranceMm =
          ParseNumber(requireValue(), "--distance-tol-mm");
    } else if (argument == L"--abs-volume-tol-mm3") {
      options.config.absoluteVolumeToleranceMm3 =
          ParseNumber(requireValue(), "--abs-volume-tol-mm3");
    } else if (argument == L"--rel-volume-tol") {
      options.config.relativeVolumeTolerance =
          ParseNumber(requireValue(), "--rel-volume-tol");
    } else if (argument == L"--boolean-fuzzy-tol-mm") {
      options.config.booleanFuzzyToleranceMm =
          ParseNumber(requireValue(), "--boolean-fuzzy-tol-mm");
    } else if (argument == L"--normalize-same-domain") {
      options.config.enableSameDomainNormalization = true;
    } else if (argument == L"--no-normalize-same-domain") {
      options.config.enableSameDomainNormalization = false;
    } else if (argument == L"--normalize-linear-tol-mm") {
      options.config.normalizationLinearToleranceMm =
          ParseNumber(requireValue(), "--normalize-linear-tol-mm");
    } else if (argument == L"--normalize-angular-tol-rad") {
      options.config.normalizationAngularToleranceRad =
          ParseNumber(requireValue(), "--normalize-angular-tol-rad");
    } else if (argument == L"--normalized-fast-path") {
      options.config.enableNormalizedFastPath = true;
    } else if (argument == L"--allow-multi-solid" || argument == L"--allow-multiple-solids") {
      options.config.allowMultipleSolids = true;
    } else if (argument == L"--no-allow-multi-solid") {
      options.config.allowMultipleSolids = false;
    } else if (argument == L"--multi-solid-policy") {
      const std::wstring polStr = requireValue();
      if (polStr == L"strict") {
        options.config.multiSolidPolicy = cadstep::MultiSolidPolicy::Strict;
      } else if (polStr == L"collection") {
        throw std::invalid_argument("multi-solid collection policy is not implemented yet");
      } else if (polStr == L"pairwise") {
        options.config.multiSolidPolicy = cadstep::MultiSolidPolicy::Pairwise;
      } else {
        throw std::invalid_argument("invalid --multi-solid-policy (must be strict, collection, or pairwise)");
      }
    } else if (argument == L"--solid-match-volume-rel-tol") {
      options.config.solidMatchVolumeRelTol =
          ParseNumber(requireValue(), "--solid-match-volume-rel-tol");
    } else if (argument == L"--solid-match-centroid-tol-mm") {
      options.config.solidMatchCentroidTolMm =
          ParseNumber(requireValue(), "--solid-match-centroid-tol-mm");
    } else if (argument == L"--solid-match-bounds-tol-mm") {
      options.config.solidMatchBoundsTolMm =
          ParseNumber(requireValue(), "--solid-match-bounds-tol-mm");
    } else if (argument == L"--export-stl") {
      options.config.exportStl = true;
    } else if (argument == L"--no-export-stl") {
      options.config.exportStl = false;
    } else if (argument == L"--export-brep") {
      options.config.exportBrep = true;
    } else if (argument == L"--no-export-brep") {
      options.config.exportBrep = false;
    } else if (argument == L"--export-entity-vtp") {
      options.config.exportEntityVtp = true;
    } else if (argument == L"--no-export-entity-vtp") {
      options.config.exportEntityVtp = false;
    } else if (argument == L"--write-entity-details") {
      options.config.writeEntityDetails = true;
    } else if (argument == L"--no-write-entity-details") {
      options.config.writeEntityDetails = false;
    } else if (argument == L"--ambiguous-match-margin") {
      options.config.ambiguousMatchMargin =
          ParseNumber(requireValue(), "--ambiguous-match-margin");
    } else if (argument == L"--quiet") {
      options.config.printHumanSummary = false;
    } else if (argument == L"--json-stdout") {
      options.printJsonStdout = true;
    } else if (argument == L"--help" || argument == L"-h") {
      options.help = true;
    } else {
      throw std::invalid_argument("unknown command-line option");
    }
  }

  if (!options.help && (options.reference.empty() ||
                        options.candidate.empty() || options.output.empty())) {
    throw std::invalid_argument(
        "--reference, --candidate, and --output are required");
  }
  return options;
}

int Run(const std::vector<std::wstring> &arguments) {
  CliOptions options;
  try {
    options = ParseArguments(arguments);
  } catch (const std::exception &error) {
    std::cerr << "INTERNAL_ERROR: " << error.what() << '\n';
    PrintUsage();
    return cadstep::ExitCode(cadstep::CompareStatus::InternalError);
  }

  if (options.help) {
    PrintUsage();
    return 0;
  }

  cadstep::CompareResult result = cadstep::CompareStepFiles(
      options.reference, options.candidate, options.config, options.output);
  std::string writeError;
  if (!cadstep::WriteResultJson(options.output, result, writeError)) {
    std::cerr << "INTERNAL_ERROR: " << writeError << '\n';
    return cadstep::ExitCode(cadstep::CompareStatus::InternalError);
  }

  if (options.printJsonStdout) {
    std::cout << cadstep::ToJson(result) << '\n';
  } else if (options.config.printHumanSummary) {
    std::cout << cadstep::ToHumanSummary(result);
  }
  return cadstep::ExitCode(result.status);
}

} // namespace

#ifdef _WIN32
int wmain(int argc, wchar_t **argv) {
  std::vector<std::wstring> arguments;
  arguments.reserve(static_cast<std::size_t>(std::max(0, argc - 1)));
  for (int index = 1; index < argc; ++index) {
    arguments.emplace_back(argv[index]);
  }
  return Run(arguments);
}
#else
int main(int argc, char **argv) {
  std::vector<std::wstring> arguments;
  arguments.reserve(static_cast<std::size_t>(std::max(0, argc - 1)));
  for (int index = 1; index < argc; ++index) {
    const std::string value = argv[index];
    arguments.emplace_back(value.begin(), value.end());
  }
  return Run(arguments);
}
#endif
