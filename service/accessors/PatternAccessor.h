#pragma once

#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"
#include <memory>
#include <vector>
#include <optional>

namespace CADExchange {
namespace Accessor {

/**
 * @brief 线性阵列访问器
 */
class LinearPatternAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CLinearPattern> m_pat;

public:
  explicit LinearPatternAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_pat = std::dynamic_pointer_cast<const CLinearPattern>(feat);
  }

  explicit LinearPatternAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_pat = std::dynamic_pointer_cast<const CLinearPattern>(other.GetRaw());
  }

  const CLinearPattern *Data() const { return m_pat.get(); }
  const CLinearPattern *operator->() const { return m_pat.get(); }
  bool IsValid() const override { return m_pat != nullptr; }

  const CLinearPatternDir &GetDir1() const { return Data()->dir1; }
  
  bool HasDir2() const { return IsValid() && Data()->dir2.has_value(); }
  const CLinearPatternDir &GetDir2() const {
    static const CLinearPatternDir kEmpty{};
    return HasDir2() ? *Data()->dir2 : kEmpty;
  }

  bool IsPatternSeedOnly() const { return IsValid() && Data()->patternSeedOnly; }
  PatternScope GetScope() const { return IsValid() ? Data()->scope : PatternScope::FEATURES; }
  
  const std::vector<std::shared_ptr<CRefEntityBase>> &GetSeedObjects() const {
    static const std::vector<std::shared_ptr<CRefEntityBase>> kEmpty;
    return IsValid() ? Data()->seedObjects : kEmpty;
  }

  const std::vector<CPatternIndex> &GetSkippedInstances() const {
    static const std::vector<CPatternIndex> kEmpty;
    return IsValid() ? Data()->skippedInstances : kEmpty;
  }

  bool IsGeometryPattern() const { return IsValid() && Data()->geometryPattern; }
};

/**
 * @brief 圆周阵列访问器
 */
class CircularPatternAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CCircularPattern> m_pat;

public:
  explicit CircularPatternAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_pat = std::dynamic_pointer_cast<const CCircularPattern>(feat);
  }

  explicit CircularPatternAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_pat = std::dynamic_pointer_cast<const CCircularPattern>(other.GetRaw());
  }

  const CCircularPattern *Data() const { return m_pat.get(); }
  const CCircularPattern *operator->() const { return m_pat.get(); }
  bool IsValid() const override { return m_pat != nullptr; }

  const CCircularPatternDir &GetDir1() const { return Data()->dir1; }

  bool HasDir2() const { return IsValid() && Data()->dir2.has_value(); }
  const CLinearPatternDir &GetDir2() const {
    static const CLinearPatternDir kEmpty{};
    return HasDir2() ? *Data()->dir2 : kEmpty;
  }

  bool IsPatternSeedOnly() const { return IsValid() && Data()->patternSeedOnly; }
  PatternScope GetScope() const { return IsValid() ? Data()->scope : PatternScope::FEATURES; }

  const std::vector<std::shared_ptr<CRefEntityBase>> &GetSeedObjects() const {
    static const std::vector<std::shared_ptr<CRefEntityBase>> kEmpty;
    return IsValid() ? Data()->seedObjects : kEmpty;
  }

  const std::vector<CPatternIndex> &GetSkippedInstances() const {
    static const std::vector<CPatternIndex> kEmpty;
    return IsValid() ? Data()->skippedInstances : kEmpty;
  }

  bool IsGeometryPattern() const { return IsValid() && Data()->geometryPattern; }
};

/**
 * @brief 镜像访问器
 */
class MirrorPatternAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CMirrorPattern> m_pat;

public:
  explicit MirrorPatternAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_pat = std::dynamic_pointer_cast<const CMirrorPattern>(feat);
  }

  explicit MirrorPatternAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_pat = std::dynamic_pointer_cast<const CMirrorPattern>(other.GetRaw());
  }

  const CMirrorPattern *Data() const { return m_pat.get(); }
  const CMirrorPattern *operator->() const { return m_pat.get(); }
  bool IsValid() const override { return m_pat != nullptr; }

  std::shared_ptr<CRefEntityBase> GetMirrorPlaneRef() const {
    return IsValid() ? Data()->mirrorPlaneRef : nullptr;
  }

  PatternScope GetScope() const { return IsValid() ? Data()->scope : PatternScope::FEATURES; }

  const std::vector<std::shared_ptr<CRefEntityBase>> &GetSeedObjects() const {
    static const std::vector<std::shared_ptr<CRefEntityBase>> kEmpty;
    return IsValid() ? Data()->seedObjects : kEmpty;
  }

  bool IsGeometryPattern() const { return IsValid() && Data()->geometryPattern; }
};

} // namespace Accessor
} // namespace CADExchange
