#pragma once

#include "../../core/UnifiedFeatures.h"
#include "UnifiedSerialization.h"
#include "../../thirdParty/cereal/archives/json.hpp"
#include "CADSerializer.h"
#include <sstream>
#include <string>


namespace CADExchange {
namespace Builder {

/**
 * @brief Helper class to format feature information as a JSON string.
 */
class FeatureFormatter {
public:
  template <typename T>
  static std::string ToJson(const std::shared_ptr<T> &feature) {
    if (!feature)
      return "{}";

    try {
      RegisterSerializationTypes();
      std::stringstream ss;
      {
        cereal::JSONOutputArchive archive(ss);
        archive(cereal::make_nvp("Feature", *feature));
      }
      return ss.str();
    } catch (const std::exception &ex) {
      std::stringstream ss;
      ss << "{\"error\":\"" << ex.what() << "\"}";
      return ss.str();
    }
  }
};

} // namespace Builder
} // namespace CADExchange
