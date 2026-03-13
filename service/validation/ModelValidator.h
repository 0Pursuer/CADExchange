#pragma once
#include "../../core/UnifiedModel.h"

namespace CADExchange {

/**
 * @brief Validates a UnifiedModel and returns a structured report.
 *
 * Extracted from UnifiedModel::Validate() to keep the core model class
 * focused on storage and indexing rather than domain validation logic.
 * Rules and RuleIDs are documented in UnifiedModel.h.
 */
class ModelValidator {
public:
  static ValidationReport Validate(const UnifiedModel &model);
};

} // namespace CADExchange
