#pragma once

#include "../../core/UnifiedModel.h"
#include "StringHelper.h"
#include <atomic>
#include <memory>
#include <string>
#include <type_traits>

namespace CADExchange {
namespace Builder {

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
    m_feature->featureID = StringHelper::GenerateUUID();
  }

  /**
   * @brief 默认虚析构函数，确保派生类析构行为正确。
   */
  virtual ~FeatureBuilderBase() = default;

  /**
   * @brief 获取关联的模型。
   */
  UnifiedModel* GetModel() { return &m_model; }

  /**
   * @brief 获取正在构建的特征。
   */
  std::shared_ptr<T> GetFeature() { return m_feature; }

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
