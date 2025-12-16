#pragma once

#include <filesystem>
#include <fstream>
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
 * @param model 要保存的统一模型。
 * @param filePath 目标输出路径。
 * @param format 序列化格式 (默认 CEREAL)。
 * @param errorMessage 可选的错误消息输出地址。
 * @return 保存成功返回 true，否则返回 false。
 */
inline bool
SaveModel(const UnifiedModel &model, const std::filesystem::path &filePath,
          std::string *errorMessage = nullptr,
          SerializationFormat format = SerializationFormat::CEREAL) {
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
 * @brief 从 XML 文件加载 UnifiedModel。
 *
 * @param model 用于接收数据的模型对象引用。
 * @param filePath 源文件路径。
 * @param errorMessage 可选错误文本输出。
 * @param format 序列化格式 (默认 CEREAL)。
 * @return 加载成功返回 true，否则返回 false。
 */
inline bool
LoadModel(UnifiedModel &model, const std::filesystem::path &filePath,
          std::string *errorMessage = nullptr,
          SerializationFormat format = SerializationFormat::CEREAL) {
  if (format == SerializationFormat::TINYXML) {
    return TinyXMLSerializer::Load(model, filePath, errorMessage);
  }

#ifdef ENABLE_CEREAL_SERIALIZATION
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
    // Use the load function defined in UnifiedSerialization.h
    load(archive, model);
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

} // namespace CADExchange
