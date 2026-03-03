#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "../../core/UnifiedModel.h"
#include "TinyXMLSerializer.h"

// Only include cereal when actually needed (not when using TINYXML)
// This avoids compile-time static assertions from cereal on types that don't support it
#ifdef ENABLE_CEREAL_SERIALIZATION
  // Undefine placeholder CEREAL_NVP before including cereal to allow it to define the real one
  #ifdef CEREAL_NVP
  #undef CEREAL_NVP
  #endif

  #include "../../thirdParty/cereal/archives/xml.hpp"
  #include "UnifiedSerialization.h"
#endif

namespace CADExchange {
void RegisterSerializationTypes();

enum class SerializationFormat { CEREAL, TINYXML };

} // namespace CADExchange
/**
 * @file CADSerializer.h
 * @brief 封装 UnifiedModel 的保存与加载逻辑。
 */

namespace CADExchange {

/**
 * @brief 将 UnifiedModel 序列化为 XML 文件。
 *
 * 默认在保存前自动执行 Validate()：有 error 则阻断保存并将错误写入
 * errorMessage。skipValidation=true 可绕过校验（仅用于 debug 路径）。
 *
 * @param model 要保存的统一模型。
 * @param filePath 目标输出路径。
 * @param errorMessage 可选的错误消息输出地址。
 * @param format 序列化格式 (默认 CEREAL)。
 * @param skipValidation 为 true 时跳过 Validate()（debug 用途）。
 * @return 保存成功返回 true，否则返回 false。
 */
inline bool
SaveModel(const UnifiedModel &model, const std::filesystem::path &filePath,
          std::string *errorMessage = nullptr,
          SerializationFormat format = SerializationFormat::CEREAL,
          bool skipValidation = false) {
  if (!skipValidation) {
    const auto report = model.Validate();
    if (!report.isValid) {
      if (errorMessage) {
        std::string msg = "Model validation failed before saving:";
        for (const auto &e : report.errors) {
          msg += "\n  " + e;
        }
        *errorMessage = msg;
      }
      return false;
    }
    // warnings 写入 stderr（不阻断）
    for (const auto &w : report.warnings) {
      std::cerr << "[CADSerializer][WARN] " << w << "\n";
    }
  }

  if (format == SerializationFormat::TINYXML) {
    return TinyXMLSerializer::Save(model, filePath, errorMessage);
  }

#ifdef ENABLE_CEREAL_SERIALIZATION
  RegisterSerializationTypes();
  std::ofstream output(filePath, std::ios::binary);
  if (!output) {
    if (errorMessage) {
      *errorMessage = "Could not open output file.";
    }
    return false;
  }

  try {
    cereal::XMLOutputArchive archive(output);
    // Use the save function defined in UnifiedSerialization.h
    save(archive, model);
  } catch (const std::exception &ex) {
    if (errorMessage) {
      *errorMessage = ex.what();
    }
    return false;
  }

  return true;
#else
  if (errorMessage) {
    *errorMessage = "CEREAL serialization not enabled. Please compile with ENABLE_CEREAL_SERIALIZATION flag.";
  }
  return false;
#endif
}

/**
 * @brief 从 XML 文件加载 UnifiedModel，加载后自动执行 Validate()。
 *
 * 加载完成后自动执行 Validate()：有 error 则返回 false 并写入 errorMessage，
 * warnings 输出到 stderr。
 *
 * @param model 用于接收数据的模型对象引用。
 * @param filePath 源文件路径。
 * @param errorMessage 可选错误文本输出。
 * @param format 序列化格式 (默认 CEREAL)。
 * @return 加载且验证均成功返回 true，否则返回 false。
 */
inline bool
LoadModel(UnifiedModel &model, const std::filesystem::path &filePath,
          std::string *errorMessage = nullptr,
          SerializationFormat format = SerializationFormat::CEREAL) {
  bool loadOk = false;
  if (format == SerializationFormat::TINYXML) {
    loadOk = TinyXMLSerializer::Load(model, filePath, errorMessage);
  }

#ifdef ENABLE_CEREAL_SERIALIZATION
  else {
    RegisterSerializationTypes();
    std::ifstream input(filePath, std::ios::binary);
    if (!input) {
      if (errorMessage) {
        *errorMessage = "Could not open input file.";
      }
      return false;
    }
    try {
      cereal::XMLInputArchive archive(input);
      load(archive, model);
      loadOk = true;
    } catch (const std::exception &ex) {
      if (errorMessage) {
        *errorMessage = ex.what();
      }
      return false;
    }
  }
#else
  else {
    if (errorMessage) {
      *errorMessage = "CEREAL serialization not enabled. Please compile with ENABLE_CEREAL_SERIALIZATION flag.";
    }
    return false;
  }
#endif

  if (!loadOk) {
    return false;
  }

  // 加载完成后自动校验
  const auto report = model.Validate();
  for (const auto &w : report.warnings) {
    std::cerr << "[CADSerializer][WARN] " << w << "\n";
  }
  if (!report.isValid) {
    if (errorMessage) {
      std::string msg = "Model validation failed after loading:";
      for (const auto &e : report.errors) {
        msg += "\n  " + e;
      }
      *errorMessage = msg;
    }
    return false;
  }
  return true;
}

} // namespace CADExchange
