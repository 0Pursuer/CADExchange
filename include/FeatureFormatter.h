#pragma once

#include "../thirdParty/cereal/archives/json.hpp"
#include "UnifiedFeatures.h"
#include "UnifiedSerialization.h"
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

    std::stringstream ss;
    {
      cereal::JSONOutputArchive archive(ss);
      // We need to cast to the base type or specific type that has a serialize
      // function Since serialize functions are defined for specific types like
      // CSketch, CExtrude, etc. we can try to serialize the object directly.
      // However, cereal expects an NVP or the object itself.
      // For better formatting, we wrap it in a root object if needed,
      // but here we just want the feature data.

      // Note: We need to ensure the correct serialize function is picked up.
      // The serialize functions are in CADExchange namespace.
      // We are in CADExchange::Builder namespace.
      // ADL should find them if T is in CADExchange namespace.

      archive(cereal::make_nvp("Feature", *feature));
    }
    return ss.str();
  }
};

} // namespace Builder
} // namespace CADExchange
