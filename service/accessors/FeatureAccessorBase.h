#pragma once
#include "../../core/UnifiedFeatures.h"
#include <memory>
#include <optional>
#include <string>

namespace CADExchange {
namespace Accessor {

/**
 * @brief 特征访问器基类，对应 Builder 层的 FeatureBuilderBase。
 *
 * 提供所有特征的通用只读属性访问。
 *
 * 与 Builder 的对称性设计：
 * - Builder: SetID() + Build() → 写入
 * - Accessor: GetID() + IsValid() → 读取
 */
class FeatureAccessorBase {
protected:
  std::shared_ptr<const CFeatureBase> m_feature;

public:
  explicit FeatureAccessorBase(std::shared_ptr<const CFeatureBase> feat)
      : m_feature(std::move(feat)) {}

  virtual ~FeatureAccessorBase() = default;

  // --- 有效性检查 ---
  virtual bool IsValid() const { return m_feature != nullptr; }

  /**
   * @brief 转换为特定类型的 Accessor (语法糖)
   *
   * 使用示例:
   * if (auto extrude = feat->As<ExtrudeAccessor>()) {
   *     extrude->GetDepth1();
   * }
   */
  template <typename AccessorT> std::optional<AccessorT> As() const {
    // AccessorT 必须能从 shared_ptr<const CFeatureBase> 构造
    AccessorT accessor(m_feature);
    if (accessor.IsValid()) {
      return accessor;
    }
    return std::nullopt;
  }

  // --- 通用属性（对应 Builder 的 SetName/SetExternalID 等） ---
  std::string GetID() const { return IsValid() ? m_feature->featureID : ""; }

  std::string GetName() const {
    return IsValid() ? m_feature->featureName : "";
  }

  std::string GetExternalID() const {
    return IsValid() ? m_feature->externalID : "";
  }

  bool IsSuppressed() const {
    return IsValid() ? m_feature->isSuppressed : false;
  }

  // --- 类型查询 ---
  FeatureType GetFeatureType() const {
    return IsValid() ? m_feature->featureType : FeatureType::Unknown;
  }

  /**
   * @brief 判断特征是否为特定类型（模板方法）。
   *
   * 使用示例：
   *   if (accessor.IsType<CExtrude>()) { ... }
   */
  template <typename FeatureT> bool IsType() const {
    return IsValid() &&
           std::dynamic_pointer_cast<const FeatureT>(m_feature) != nullptr;
  }

  // --- 底层指针访问（供子类使用） ---
  // 推荐使用 Data() 替代 m_feature 直接访问
  const CFeatureBase *Data() const { return m_feature.get(); }

  const CFeatureBase *operator->() const { return m_feature.get(); }

  std::shared_ptr<const CFeatureBase> GetRaw() const { return m_feature; }
};

} // namespace Accessor
} // namespace CADExchange
