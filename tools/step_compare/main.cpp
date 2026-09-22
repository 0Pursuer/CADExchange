#include "StepCompare.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/**
 * @brief CLI 解析的中间结果：路径、比较配置与输出行为开关。
 */
struct CliOptions {
  /// 参考 STEP 文件路径（--reference）
  std::filesystem::path reference;
  /// 候选 STEP 文件路径（--candidate）
  std::filesystem::path candidate;
  /// 工件与 result.json 的输出目录（--output）
  std::filesystem::path output;
  /// 由各命令行选项填充的比较配置
  cadstep::CompareConfig config;
  /// 是否把 result.json 的内容整体打印到 stdout（--json-stdout）
  bool printJsonStdout = false;
  /// 是否只打印用法说明（--help / -h）
  bool help = false;
};

/**
 * @brief 打印命令行用法说明到 stdout。
 */
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

/**
 * @brief 把宽字符串解析为浮点数，并要求整串都是合法数字。
 * @param text 待解析的宽字符串
 * @param name 对应的命令行选项名，用于拼装错误信息
 * @return 解析出的数值
 * @throws std::invalid_argument 字符串存在未消费的尾部字符时抛出
 */
double ParseNumber(const std::wstring &text, const char *name) {
  std::size_t parsed = 0;
  const double value = std::stod(text, &parsed);
  if (parsed != text.size()) {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
  return value;
}

/**
 * @brief 解析命令行参数并组装 CliOptions。
 * @param arguments 去掉程序名后的参数列表
 * @return 填充好的选项结构
 * @throws std::invalid_argument 选项缺值、取值非法或遇到未知选项时抛出
 *
 * @note --reference / --candidate / --output 三项为必需（--help 除外）。
 *       --multi-solid-policy collection 因尚未实现而直接拒绝。
 */
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
    } else if (argument == L"--no-allow-multi-solid" || argument == L"--no-allow-multiple-solids" || argument == L"--disallow-multi-solid") {
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

/**
 * @brief CLI 主流程：解析参数 → 对比 → 写 result.json → 输出 → 返回退出码。
 * @param arguments 去掉程序名后的参数列表
 * @return 进程退出码，取自 cadstep::ExitCode()；参数解析失败或写文件失败时
 *         返回 INTERNAL_ERROR 对应的退出码
 */
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

// Windows 入口：使用宽字符 argv，避免非 ASCII 路径的编码损失。
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
// 非 Windows 入口：把窄字符 argv 逐字节扩为宽字符串后复用同一主流程。
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
