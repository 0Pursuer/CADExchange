// result.json 的原子写入实现（WriteResultJson 的定义，Doxygen 文档见
// StepCompare.h 声明处）。
//
// 原子性策略：先写入 result.json.tmp 并 flush 校验，成功后再 rename 覆盖
// 最终文件；任何一步失败都通过 error 返回原因并返回 false，
// 已存在的 result.json 不会被半写状态破坏。

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

    // Windows 上 rename 不允许目标已存在，故先移除旧文件；移除失败可容忍
    //（首次运行时目标不存在），后续 rename 的 ec 才是判定依据。
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
