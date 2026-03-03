#pragma once
// clang-format off
#include "UnifiedFeatures.h"
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>
// clang-format on
namespace CADExchange {

/**
 * @brief 模型验证报告，包含错误和警告信息。
 */
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
   * @brief 非 const 访问版本，允许在外部修改特征集合。
   */
  std::vector<std::shared_ptr<CFeatureBase>> &Features() { return m_features; }

  /**
   * @brief 清空模型中的所有特征和索引。
   */
  void Clear() {
    m_features.clear();
    m_index.clear();
  }

  /**
   * @brief 验证模型完整性，包括特征顺序（引用的父特征必须在当前特征之前出现）。
   */
  ValidationReport Validate() const {
    ValidationReport report;
    std::unordered_set<std::string> seen;

    for (const auto &feature : m_features) {
      if (feature->featureID.empty()) {
        report.isValid = false;
        report.errors.push_back("Feature with empty ID found.");
      }

      // 检查拉伸特征的草图引用是否在其之前已定义
      if (auto extrude = std::dynamic_pointer_cast<CExtrude>(feature)) {
        if (!extrude->profileSketchID.empty() &&
            seen.find(extrude->profileSketchID) == seen.end()) {
          report.isValid = false;
          report.errors.push_back(
              "Extrude '" + extrude->featureID +
              "' references sketch '" + extrude->profileSketchID +
              "' which has not been defined yet (wrong order).");
        }
        // 检查 EndCondition 面/顶点引用的父特征是否已定义
        auto checkECRef = [&](const ExtrudeEndCondition &ec) {
          if (!ec.referenceEntity) return;
          if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(
                  ec.referenceEntity)) {
            if (!subTopo->parentFeatureID.empty() &&
                seen.find(subTopo->parentFeatureID) == seen.end()) {
              report.warnings.push_back(
                  "Extrude '" + extrude->featureID +
                  "' end condition references feature '" +
                  subTopo->parentFeatureID +
                  "' which has not been defined yet (wrong order).");
            }
          }
        };
        checkECRef(extrude->endCondition1);
        if (extrude->endCondition2)
          checkECRef(*extrude->endCondition2);
      }

      // 检查草图参考平面的父特征是否已定义
      if (auto sketch = std::dynamic_pointer_cast<CSketch>(feature)) {
        if (sketch->referencePlane) {
          if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(
                  sketch->referencePlane)) {
            if (!subTopo->parentFeatureID.empty() &&
                seen.find(subTopo->parentFeatureID) == seen.end()) {
              report.warnings.push_back(
                  "Sketch '" + sketch->featureID +
                  "' reference plane parent '" + subTopo->parentFeatureID +
                  "' has not been defined yet (wrong order).");
            }
          }
        }
      }

      seen.insert(feature->featureID);
    }
    return report;
  }

private:
  std::vector<std::shared_ptr<CFeatureBase>> m_features; ///< 特征列表
  std::unordered_map<std::string, std::shared_ptr<CFeatureBase>>
      m_index; ///< ID 索引
};

} // namespace CADExchange
