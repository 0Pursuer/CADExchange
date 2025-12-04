#pragma once

#include "UnifiedModel.h"
#include <atomic>
#include <memory>
#include <string>
#include <type_traits>

namespace CADExchange {
namespace Builder {

/**
 * @brief 全局 ID 生成器，确保不同类型的 Builder 生成的 ID 也是全局唯一的。
 */
class IDGenerator {
public:
  static std::string Generate() {
    static std::atomic<int> counter{0};
    return "FB-" +
           std::to_string(counter.fetch_add(1, std::memory_order_relaxed) + 1);
  }
};

/**
 * @brief Builder 体系的基类模板，提供生命周期管理和唯一 ID 分配。
 *
 * @tparam T 具体的特征类型，必须继承自 CFeatureBase。
 */
template <typename T> class FeatureBuilderBase {
  static_assert(std::is_base_of<CFeatureBase, T>::value,
                "FeatureBuilderBase can only be instantiated with a "
                "CFeatureBase derivative.");

public:
  /**
   * @brief 构造器，初始化特征并绑定到模型。
   *
   * @param model 目标 UnifiedModel，用于注册构建出的特征。
   * @param name 赋予特征的可读名称。
   */
  FeatureBuilderBase(UnifiedModel &model, const std::string &name)
      : m_model(model) {
    m_feature = std::make_shared<T>();
    m_feature->featureName = name;
    m_feature->featureID = IDGenerator::Generate();
  }

  /**
   * @brief 默认虚析构函数，确保派生类析构行为正确。
   */
  virtual ~FeatureBuilderBase() = default;

  /**
   * @brief 设置特征的抑制状态（是否参与求解）。
   */
  FeatureBuilderBase &SetSuppressed(bool isSuppressed) {
    m_feature->isSuppressed = isSuppressed;
    return *this;
  }

  /**
   * @brief 设置外部系统 ID。
   */
  FeatureBuilderBase &SetExternalID(const std::string &externalID) {
    m_feature->externalID = externalID;
    return *this;
  }

  /**
   * @brief 将构建完成的特征写入模型并返回其标识符。
   */
  std::string Build() {
    m_model.AddFeature(m_feature);
    return m_feature->featureID;
  }

protected:
  std::shared_ptr<T> m_feature;
  UnifiedModel &m_model;
};

} // namespace Builder
} // namespace CADExchange
