#pragma once

#include "FeatureBuilderBase.h"
#include "ReferenceBuilder.h"
#include <stdexcept>
#include <type_traits>

namespace CADExchange {
namespace Builder {

/**
 * @brief 壳特征（抽壳）的 Builder。
 *
 * 壳特征属于非草图驱动的面操作类特征，直接引用拓扑面（CRefFace）。
 * 参照 FilletBuilder / ChamferBuilder 模式，不依赖轮廓草图。
 */
class ShellBuilder : public FeatureBuilderBase<CShell> {
public:
  ShellBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  /**
   * @brief 设置默认壳壁厚度。
   * @param thickness 厚度值，必须 > 0。
   */
  ShellBuilder &SetThickness(double thickness) {
    if (thickness <= 0.0) {
      throw std::runtime_error("Shell thickness must be > 0.");
    }
    m_feature->thickness = thickness;
    return *this;
  }

  /**
   * @brief 设置抽壳方向（内侧 / 外侧）。
   */
  ShellBuilder &SetDirection(ShellThicknessDirection direction) {
    if (direction == ShellThicknessDirection::Unknown) {
      throw std::runtime_error("Shell direction must not be Unknown.");
    }
    m_feature->direction = direction;
    return *this;
  }

  /**
   * @brief 添加一个被移除的面（开口面）。
   * @param ref 拓扑面引用，必须非空。
   */
  ShellBuilder &AddFaceToRemove(const std::shared_ptr<CRefFace> &ref) {
    if (!ref) {
      throw std::runtime_error("Shell faceToRemove must not be null.");
    }
    ValidateReference(ref);
    m_feature->facesToRemove.push_back(ref);
    return *this;
  }

  template <typename RefBuilderT>
  ShellBuilder &AddFaceToRemove(const RefBuilderT &refBuilder) {
    return AddFaceToRemove(ToReference(refBuilder));
  }

  /**
   * @brief 添加一个具有独立（非默认）厚度的面。
   * @param ref 拓扑面引用，必须非空。
   * @param thickness 该面的独立厚度值，必须 > 0。
   */
  ShellBuilder &AddThicknessFace(const std::shared_ptr<CRefFace> &ref,
                                  double thickness) {
    if (!ref) {
      throw std::runtime_error("Shell thickness face must not be null.");
    }
    if (thickness <= 0.0) {
      throw std::runtime_error("Shell thickness face thickness must be > 0.");
    }
    ValidateReference(ref);
    CShellThicknessFace item;
    item.face = ref;
    item.thickness = thickness;
    m_feature->thicknessFaces.push_back(std::move(item));
    return *this;
  }

  template <typename RefBuilderT>
  ShellBuilder &AddThicknessFace(const RefBuilderT &refBuilder,
                                  double thickness) {
    return AddThicknessFace(ToReference(refBuilder), thickness);
  }

  /**
   * @brief 设置目标实体引用（Creo PRO_E_BODY_SELECT，Phase-1 可选）。
   */
  ShellBuilder &SetTargetBody(const std::shared_ptr<CRefEntityBase> &ref) {
    if (ref) {
      ValidateReference(ref);
    }
    m_feature->targetBody = ref;
    return *this;
  }

  /**
   * @brief 添加一个排除曲面（Creo PRO_E_STD_SURF_COLLECTION_APPL，Phase-1 可选）。
   */
  ShellBuilder &AddExcludedFace(const std::shared_ptr<CRefFace> &ref) {
    if (!ref) {
      throw std::runtime_error("Shell excluded face must not be null.");
    }
    ValidateReference(ref);
    m_feature->excludedFaces.push_back(ref);
    return *this;
  }

  template <typename RefBuilderT>
  ShellBuilder &AddExcludedFace(const RefBuilderT &refBuilder) {
    return AddExcludedFace(ToReference(refBuilder));
  }

private:
  template <typename RefBuilderT>
  static std::shared_ptr<CRefEntityBase>
  ToReference(const RefBuilderT &refBuilder) {
    static_assert(std::is_convertible_v<RefBuilderT,
                                        std::shared_ptr<CRefEntityBase>>,
                  "Reference must be built by CADExchange::Builder::Ref::* wrappers.");
    return static_cast<std::shared_ptr<CRefEntityBase>>(refBuilder);
  }
};

} // namespace Builder
} // namespace CADExchange
