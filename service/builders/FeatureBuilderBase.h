#pragma once

#include "../../core/UnifiedModel.h"
#include "StringHelper.h"
#include <atomic>
#include <memory>
#include <stdexcept>
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
  UnifiedModel *GetModel() { return &m_model; }

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
  /**
   * @brief 验证引用实体是否合法。
   *
   * 对于基准面、基准轴等引用，如果不是标准 ID，则必须在模型中存在对应特征。
   *
   * @param ref 需要验证的引用实体指针
   * @throws std::runtime_error 当引用特征在模型中不存在时抛出
   */
  void ValidateReference(const std::shared_ptr<CRefEntityBase> &ref) const {
    if (!ref)
      return;

    // 处理基准面引用
    if (ref->refType == RefType::FEATURE_DATUM_PLANE) {
      if (auto plane = std::dynamic_pointer_cast<const CRefPlane>(ref)) {
        if (!StandardID::IsStandardPlane(plane->targetFeatureID)) {
          if (!m_model.GetFeature(plane->targetFeatureID)) {
            throw std::runtime_error(
                "Reference plane feature not found in model: " +
                plane->targetFeatureID);
          }
        }
      }
    }
    // 处理基准轴引用
    else if (ref->refType == RefType::FEATURE_DATUM_AXIS) {
      if (auto axis = std::dynamic_pointer_cast<const CRefAxis>(ref)) {
        if (!StandardID::IsStandardAxis(axis->targetFeatureID)) {
          if (!m_model.GetFeature(axis->targetFeatureID)) {
            throw std::runtime_error(
                "Reference axis feature not found in model: " +
                axis->targetFeatureID);
          }
        }
      }
    }
    // 处理基准点引用
    else if (ref->refType == RefType::FEATURE_DATUM_POINT) {
      if (auto pnt = std::dynamic_pointer_cast<const CRefPoint>(ref)) {
        if (!StandardID::IsStandardPoint(pnt->targetFeatureID)) {
          if (!m_model.GetFeature(pnt->targetFeatureID)) {
            throw std::runtime_error(
                "Reference point feature not found in model: " +
                pnt->targetFeatureID);
          }
        }
      }
    }
  }

  std::shared_ptr<T> m_feature;
  UnifiedModel &m_model;
};

} // namespace Builder
} // namespace CADExchange
