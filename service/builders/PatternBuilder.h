#pragma once

#include "FeatureBuilderBase.h"
#include <memory>
#include <vector>
#include <string>

namespace CADExchange {
namespace Builder {

/**
 * @brief 线性阵列构建器
 */
class LinearPatternBuilder : public FeatureBuilderBase<CLinearPattern> {
public:
  LinearPatternBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  LinearPatternBuilder &SetDir1(const CLinearPatternDir &dir) {
    m_feature->dir1 = dir;
    return *this;
  }

  LinearPatternBuilder &SetDir2(const CLinearPatternDir &dir) {
    m_feature->dir2 = dir;
    return *this;
  }

  LinearPatternBuilder &SetPatternSeedOnly(bool seedOnly) {
    m_feature->patternSeedOnly = seedOnly;
    return *this;
  }

  LinearPatternBuilder &SetScope(PatternScope scope) {
    m_feature->scope = scope;
    return *this;
  }

  LinearPatternBuilder &AddSeedObject(const std::shared_ptr<CRefEntityBase> &obj) {
    m_feature->seedObjects.push_back(obj);
    return *this;
  }

  LinearPatternBuilder &AddSkippedInstance(const CPatternIndex &idx) {
    m_feature->skippedInstances.push_back(idx);
    return *this;
  }

  LinearPatternBuilder &SetGeometryPattern(bool geomPattern) {
    m_feature->geometryPattern = geomPattern;
    return *this;
  }
};

/**
 * @brief 圆周阵列构建器
 */
class CircularPatternBuilder : public FeatureBuilderBase<CCircularPattern> {
public:
  CircularPatternBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  CircularPatternBuilder &SetDir1(const CCircularPatternDir &dir) {
    m_feature->dir1 = dir;
    return *this;
  }

  CircularPatternBuilder &SetDir2(const CLinearPatternDir &dir) {
    m_feature->dir2 = dir;
    return *this;
  }

  CircularPatternBuilder &SetPatternSeedOnly(bool seedOnly) {
    m_feature->patternSeedOnly = seedOnly;
    return *this;
  }

  CircularPatternBuilder &SetScope(PatternScope scope) {
    m_feature->scope = scope;
    return *this;
  }

  CircularPatternBuilder &AddSeedObject(const std::shared_ptr<CRefEntityBase> &obj) {
    m_feature->seedObjects.push_back(obj);
    return *this;
  }

  CircularPatternBuilder &AddSkippedInstance(const CPatternIndex &idx) {
    m_feature->skippedInstances.push_back(idx);
    return *this;
  }

  CircularPatternBuilder &SetGeometryPattern(bool geomPattern) {
    m_feature->geometryPattern = geomPattern;
    return *this;
  }
};

/**
 * @brief 镜像构建器
 */
class MirrorPatternBuilder : public FeatureBuilderBase<CMirrorPattern> {
public:
  MirrorPatternBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  MirrorPatternBuilder &SetMirrorPlane(const std::shared_ptr<CRefEntityBase> &planeRef) {
    m_feature->mirrorPlaneRef = planeRef;
    return *this;
  }

  MirrorPatternBuilder &SetScope(PatternScope scope) {
    m_feature->scope = scope;
    return *this;
  }

  MirrorPatternBuilder &AddSeedObject(const std::shared_ptr<CRefEntityBase> &obj) {
    m_feature->seedObjects.push_back(obj);
    return *this;
  }

  MirrorPatternBuilder &SetGeometryPattern(bool geomPattern) {
    m_feature->geometryPattern = geomPattern;
    return *this;
  }
};

} // namespace Builder
} // namespace CADExchange
