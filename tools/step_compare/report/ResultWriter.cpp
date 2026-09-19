// Atomic writer for result.json.

#include "StepCompare.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace cadstep {

bool WriteResultJson(const std::filesystem::path &outputDirectory,
                     const CompareResult &result, std::string &error) {
  try {
    std::error_code ec;
    std::filesystem::create_directories(outputDirectory, ec);

    const auto finalPath = outputDirectory / "result.json";
    const auto tempPath = outputDirectory / "result.json.tmp";

    {
      std::ofstream out(tempPath, std::ios::binary | std::ios::trunc);
      if (!out.is_open()) {
        error = "cannot open temporary result file: " + tempPath.string();
        return false;
      }

      out << ToJson(result);
      out.flush();

      if (!out.good()) {
        error = "failed to write temporary result file: " + tempPath.string();
        return false;
      }
    }

    std::filesystem::remove(finalPath, ec);
    ec.clear();
    std::filesystem::rename(tempPath, finalPath, ec);

    if (ec) {
      error = "failed to replace result.json: " + ec.message();
      return false;
    }

    return true;
  } catch (const std::exception &ex) {
    error = ex.what();
    return false;
  }
}
} // namespace cadstep
