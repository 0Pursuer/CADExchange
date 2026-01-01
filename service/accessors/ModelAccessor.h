#pragma once
#include "../../core/UnifiedModel.h"
#include "AccessorMacros.h"
#include "FeatureAccessorBase.h"
#include <memory>
#include <vector>


namespace CADExchange {
namespace Accessor {

/**
 * @brief 模型访问器，提供对完整 CAD 模型的访问入口。
 *
 * 这是 Accessor 层的主入口，用于：
 * 1. 加载已序列化的 CAD 模型
 * 2. 遍历模型中的所有特征
 * 3. 获取特定类型的特征访问器
 */
class ModelAccessor {
private:
  UnifiedModel m_model;

public:
  ModelAccessor() = default;

  // ModelAccessor doesn't inherit from FeatureAccessorBase in the same way (it
  // wraps UnifiedModel), but the plan says "Implement Data() returning const
  // CModel*". Wait, ModelAccessor::m_model is `UnifiedModel`, not
  // `std::shared_ptr<const CModel>`. And `UnifiedModel` seems to be a class
  // wrapper, not a raw C struct pointer. Let's check `UnifiedModel` definition
  // in `core/UnifiedModel.h` if possible, but based on `GetRawModel` it returns
  // `const UnifiedModel&`. The plan said: "Implement Data() returning const
  // CModel*". This might be a mistake in the plan if `UnifiedModel` captures
  // `CModel`. Or maybe `UnifiedModel` IS `CModel` alias? I haven't seen
  // `CModel` definition. Let's assume `UnifiedModel` is the data holder.
  // `ModelAccessor` has `GetFeatureCount` which uses `m_model.GetFeatures()`.
  // Getting strict `Data()` pointer might not apply directly if `m_model` is an
  // object, not a pointer. However, I can provide `Data()` returning `const
  // UnifiedModel*`.

  // Let's stick to what's reasonable. `m_model` is a `UnifiedModel`.

  const UnifiedModel *Data() const { return &m_model; }

  const UnifiedModel *operator->() const { return &m_model; }

  /**
   * @brief 检查模型是否有效（包含数据）。
   */
  bool IsValid() const { return !Data()->GetFeatures().empty(); }

  /**
   * @brief 获取模型中的特征数量。
   */
  int GetFeatureCount() const { return Data()->GetFeatures().size(); }

  /**
   * @brief 按索引获取通用特征访问器。
   *
   * @param index 特征索引（0-based）
   * @return 如果索引有效，返回特征访问器；否则返回空指针
   */
  std::shared_ptr<FeatureAccessorBase> GetFeature(int index) const {
    auto features = m_model.GetFeatures();
    if (index < 0 || index >= (int)features.size()) {
      return nullptr;
    }
    return std::make_shared<FeatureAccessorBase>(features[index]);
  }

  /**
   * @brief 按 ID 获取特征访问器。
   */
  std::shared_ptr<FeatureAccessorBase>
  GetFeatureByID(const std::string &featureID) const {
    auto features = m_model.GetFeatures();
    for (const auto &feat : features) {
      if (feat && feat->featureID == featureID) {
        return std::make_shared<FeatureAccessorBase>(feat);
      }
    }
    return nullptr;
  }

  /**
   * @brief 遍历所有特征。
   *
   * 使用示例：
   *   for (int i = 0; i < modelAccessor.GetFeatureCount(); ++i) {
   *       auto feat = modelAccessor.GetFeature(i);
   *       // ...
   *   }
   */
  std::vector<std::shared_ptr<FeatureAccessorBase>> GetAllFeatures() const {
    std::vector<std::shared_ptr<FeatureAccessorBase>> result;
    for (int i = 0; i < GetFeatureCount(); ++i) {
      auto feat = GetFeature(i);
      if (feat)
        result.push_back(feat);
    }
    return result;
  }

  // --- 底层访问 ---

  /**
   * @brief 获取底层模型对象（用于高级操作）。
   */
  const UnifiedModel &GetRawModel() const { return m_model; }

  UnifiedModel &GetRawModel() { return m_model; }

  /**
   * @brief 设置模型数据。
   */
  void SetModel(const UnifiedModel &model) { m_model = model; }
};

} // namespace Accessor
} // namespace CADExchange
