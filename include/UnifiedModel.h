#pragma once

#include "UnifiedFeatures.h"
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace CADExchange {

struct ValidationReport {
  bool isValid = true;
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
};

/**
 * @brief 封装所有构建特征的容器，同时记录单位、名称等元数据。
 */
class UnifiedModel {
public:
  UnitType unit = UnitType::METER; ///< 当前使用的单位系统。
  std::string modelName;           ///< 可选的模型名称。

  UnifiedModel() = default;

  /**
   * @brief 将给定特征注册到模型，并同步索引。
   *
   * @param feature 需要保存的特征对象，若为空则忽略。
   */
  void AddFeature(const std::shared_ptr<CFeatureBase> &feature) {
    if (!feature) {
      return;
    }
    m_features.push_back(feature);
    m_index[feature->featureID] = feature;
    if (!feature->externalID.empty()) {
      m_externalIndex[feature->externalID] = feature;
    }
  }

  /**
   * @brief 根据标识符获取对应的特征对象。
   *
   * @param featureID 特征的 UUID。
   * @return 若存在则返回 shared_ptr，否则返回 nullptr。
   */
  std::shared_ptr<CFeatureBase> GetFeature(const std::string &featureID) const {
    if (auto it = m_index.find(featureID); it != m_index.end()) {
      return it->second;
    }
    return nullptr;
  }

  /**
   * @brief 根据外部 ID 获取对应的特征对象。
   *
   * @param externalID 外部系统的 ID。
   * @return 若存在则返回 shared_ptr，否则返回 nullptr。
   */
  std::shared_ptr<CFeatureBase>
  GetFeatureByExternalID(const std::string &externalID) const {
    if (auto it = m_externalIndex.find(externalID);
        it != m_externalIndex.end()) {
      return it->second;
    }
    return nullptr;
  }

  /**
   * @brief 尝试将 ID 对应的特征安全地转换为指定类型。
   *
   * @tparam T 期望的派生类型。
   * @param featureID 需要查询的特征 ID。
   * @return 转换成功则返回特定类型的 shared_ptr，否则 nullptr。
   */
  template <typename T>
  std::shared_ptr<T> GetFeatureAs(const std::string &featureID) const {
    static_assert(std::is_base_of<CFeatureBase, T>::value,
                  "T must derive from CFeatureBase");
    auto base = GetFeature(featureID);
    if (!base) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<T>(base);
  }

  /**
   * @brief 尝试将外部 ID 对应的特征安全地转换为指定类型。
   */
  template <typename T>
  std::shared_ptr<T>
  GetFeatureByExternalIDAs(const std::string &externalID) const {
    static_assert(std::is_base_of<CFeatureBase, T>::value,
                  "T must derive from CFeatureBase");
    auto base = GetFeatureByExternalID(externalID);
    if (!base) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<T>(base);
  }

  /**
   * @brief 获取当前模型持有的所有特征列表。
   *
   * @return 包含所有特征的 const 引用。
   */
  const std::vector<std::shared_ptr<CFeatureBase>> &GetFeatures() const {
    return m_features;
  }

  /**
   * @brief 非 const 访问版本，允许在外部修改特征集合。
   */
  std::vector<std::shared_ptr<CFeatureBase>> &Features() { return m_features; }

  /**
   * @brief 清空模型中的所有特征和索引。
   */
  void Clear() {
    m_features.clear();
    m_index.clear();
    m_externalIndex.clear();
  }

  /**
   * @brief 验证模型完整性。
   */
  ValidationReport Validate() const {
    ValidationReport report;
    for (const auto &feature : m_features) {
      if (feature->featureID.empty()) {
        report.isValid = false;
        report.errors.push_back("Feature with empty ID found.");
      }
      // Check for duplicate IDs (implicit in map, but good to check list)
    }
    // More complex checks can be added here
    return report;
  }

private:
  std::vector<std::shared_ptr<CFeatureBase>> m_features; ///< 特征列表
  std::unordered_map<std::string, std::shared_ptr<CFeatureBase>>
      m_index; ///< ID 索引
  std::unordered_map<std::string, std::shared_ptr<CFeatureBase>>
      m_externalIndex; ///< 外部 ID 索引
};

} // namespace CADExchange
