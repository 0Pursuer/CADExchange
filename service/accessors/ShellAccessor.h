#pragma once

#include "AccessorMacros.h"
#include "FeatureAccessorBase.h"
#include <memory>
#include <vector>

namespace CADExchange {
namespace Accessor {

/**
 * @brief 壳特征（抽壳）的只读访问器。
 *
 * 参照 FilletAccessor / ChamferAccessor 模式，提供对 CShell 各字段的
 * 安全只读访问。
 */
class ShellAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CShell> m_shell;

public:
  explicit ShellAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_shell = std::dynamic_pointer_cast<const CShell>(feat);
  }

  explicit ShellAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_shell = std::dynamic_pointer_cast<const CShell>(other.GetRaw());
  }

  explicit ShellAccessor(std::shared_ptr<FeatureAccessorBase> feat)
      : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
    if (feat) {
      m_shell = std::dynamic_pointer_cast<const CShell>(feat->GetRaw());
    }
  }

  const CShell *Data() const { return m_shell.get(); }
  const CShell *operator->() const { return m_shell.get(); }
  bool IsValid() const override { return m_shell != nullptr; }

  ACCESSOR_GETTER(double, Thickness, thickness, 0.0)

  ShellThicknessDirection GetDirection() const {
    return IsValid() ? Data()->direction
                     : ShellThicknessDirection::Unknown;
  }

  bool IsInward() const {
    return IsValid() && Data()->direction == ShellThicknessDirection::Inward;
  }

  bool IsOutward() const {
    return IsValid() && Data()->direction == ShellThicknessDirection::Outward;
  }

  const std::vector<std::shared_ptr<CRefFace>> &GetFacesToRemove() const {
    static const std::vector<std::shared_ptr<CRefFace>> kEmpty;
    return IsValid() ? Data()->facesToRemove : kEmpty;
  }

  size_t GetFacesToRemoveCount() const {
    return IsValid() ? Data()->facesToRemove.size() : 0;
  }

  const std::vector<CShellThicknessFace> &GetThicknessFaces() const {
    static const std::vector<CShellThicknessFace> kEmpty;
    return IsValid() ? Data()->thicknessFaces : kEmpty;
  }

  size_t GetThicknessFacesCount() const {
    return IsValid() ? Data()->thicknessFaces.size() : 0;
  }

  bool HasTargetBody() const {
    return IsValid() && Data()->targetBody != nullptr;
  }

  std::shared_ptr<CRefEntityBase> GetTargetBody() const {
    return IsValid() ? Data()->targetBody : nullptr;
  }

  const std::vector<std::shared_ptr<CRefFace>> &GetExcludedFaces() const {
    static const std::vector<std::shared_ptr<CRefFace>> kEmpty;
    return IsValid() ? Data()->excludedFaces : kEmpty;
  }

  size_t GetExcludedFacesCount() const {
    return IsValid() ? Data()->excludedFaces.size() : 0;
  }
};

} // namespace Accessor
} // namespace CADExchange
