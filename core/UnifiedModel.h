#pragma once
// clang-format off
#include "UnifiedFeatures.h"
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>
// clang-format on
namespace CADExchange {

/**
 * @brief 模型验证报告，包含错误和警告信息。
 *
 * 每条错误/警告均以 "[RULE_ID] 描述" 格式输出，便于日志定位。
 *
 * Rule ID 命名规范：
 *   MODEL_xxx  — 模型级（重复ID、空ID）
 *   SKETCH_xxx — 草图特征规则
 *   EXTRUDE_xxx— 拉伸特征规则
 *   REVOLVE_xxx— 旋转特征规则
 *   DATUM_xxx  — 基准面特征规则
 *   GEOM_xxx   — 几何合法性（向量/坐标系）
 *   REF_xxx    — 引用顺序/类型匹配
 *   SCALE_xxx  — 数值量级（单位一致性）
 */
struct ValidationReport {
  bool isValid = true;
  std::vector<std::string> errors;   ///< 阻断级：有 error 时不应保存或重建
  std::vector<std::string> warnings; ///< 警告级：记录但不阻断
};

/**
 * @brief 封装所有构建特征的容器，同时记录单位、名称等元数据。
 */
class UnifiedModel {
public:
  UnitType unit = UnitType::METER; ///< 当前使用的单位系统。
  std::string modelName;           ///< 可选的模型名称。

  UnifiedModel() = default;
  UnifiedModel(UnitType unitType, const std::string &name = "")
      : unit(unitType), modelName(name) {}

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
   * @brief 根据特征名称查找特征 ID。
   *
   * @param name 特征名称。
   * @return 若找到则返回特征 ID，否则返回空字符串。
   */
  std::string GetFeatureIdByName(const std::string &name) const {
    for (const auto &feature : m_features) {
      if (feature->featureName == name) {
        return feature->featureID;
      }
    }
    return "";
  }

  /**
   * @brief  获取特征的index。
   * 
   */
  int GetFeatureIndexByID(const std::string &featureID) const {
    for (size_t i = 0; i < m_features.size(); ++i) {
      if (m_features[i]->featureID == featureID) {
        return static_cast<int>(i);
      }
    }
    return -1; // 未找到
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
   * @brief 获取当前模型持有的所有特征列表。
   *
   * @return 包含所有特征的 const 引用。
   */
  const std::vector<std::shared_ptr<CFeatureBase>> &GetFeatures() const {
    return m_features;
  }

  /**
   * @brief 遍历所有特征并允许原地修改字段值（如单位缩放）。
   *
   * 回调签名: void(std::shared_ptr<CFeatureBase>&)
   * 注意: 不允许在回调内增删特征，否则索引将失效。
   */
  template <typename Fn>
  void ForEachMutable(Fn &&fn) {
    for (auto &f : m_features)
      fn(f);
  }

  /**
   * @brief 非 const 访问版本（已废弃）。
   * @deprecated 使用 ForEachMutable() 进行原地修改；使用 GetFeatures() 进行只读访问。
   *             直接暴露 vector 引用会绕过 AddFeature() 破坏索引一致性。
   */
  [[deprecated("Use ForEachMutable() for in-place mutation; use GetFeatures() "
               "for read-only access")]]
  std::vector<std::shared_ptr<CFeatureBase>> &Features() { return m_features; }

  /**
   * @brief 清空模型中的所有特征和索引。
   */
  void Clear() {
    m_features.clear();
    m_index.clear();
  }

  /**
   * @brief 验证模型完整性。
   *
   * 委托给 ModelValidator::Validate(*this)。实现位于
   * service/validation/ModelValidator.cpp，规则说明见 ModelValidator.h。
   *
   * 检查分为 error（阻断）和 warning（提示）两级，每条均带 RuleID：
   *   MODEL_xxx / SKETCH_xxx / EXTRUDE_xxx / REVOLVE_xxx / DATUM_xxx
   *   GEOM_xxx / REF_xxx / SCALE_xxx
   */
  ValidationReport Validate() const;

private:
  std::vector<std::shared_ptr<CFeatureBase>> m_features; ///< 特征列表
  std::unordered_map<std::string, std::shared_ptr<CFeatureBase>>
      m_index; ///< ID 索引
};

bool ConvertModelUnit(UnifiedModel &model, UnitType targetUnit,
                      std::string *errorMessage = nullptr);

} // namespace CADExchange
