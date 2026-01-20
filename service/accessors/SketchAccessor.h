#pragma once
#include "../../core/TypeAdapters.h"
#include "AccessorMacros.h"
#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"
#include <memory>
#include <vector>

namespace CADExchange {
namespace Accessor {

/**
 * @brief 草图几何段访问器（辅助类）。
 *
 * 用于逐个访问草图中的各个几何段（直线、圆、圆弧、点等）。
 * 对应 Builder 层中 SketchBuilder 的 AddLine/AddCircle 等操作的反向。
 */
class SketchSegmentAccessor {
private:
  std::shared_ptr<const CSketchSeg> m_seg;

public:
  explicit SketchSegmentAccessor(
      std::shared_ptr<const CSketchSeg> seg = nullptr)
      : m_seg(std::move(seg)) {}

  bool IsValid() const { return m_seg != nullptr; }

  // --- 基本属性 ---
  CSketchSeg::SegType GetType() const {
    return IsValid() ? m_seg->type : CSketchSeg::SegType::LINE;
  }

  std::string GetLocalID() const { return IsValid() ? m_seg->localID : ""; }

  bool IsConstruction() const {
    return IsValid() ? m_seg->isConstruction : false;
  }

  // --- 直线特有方法 ---
  bool GetLineCoords(CPoint3D &outStart, CPoint3D &outEnd) const {
    if (auto line = std::dynamic_pointer_cast<const CSketchLine>(m_seg)) {
      outStart = line->startPos;
      outEnd = line->endPos;
      return true;
    }
    return false;
  }

  template <typename StartT, typename EndT>
  bool GetLineCoords(StartT &outStart, EndT &outEnd) const {
    CPoint3D s, e;
    if (!GetLineCoords(s, e))
      return false;
    outStart = PointWriter<StartT>::Convert(s);
    outEnd = PointWriter<EndT>::Convert(e);
    return true;
  }

  // --- 圆特有方法 ---
  bool GetCircleParams(CPoint3D &outCenter, double &outRadius) const {
    if (auto circle = std::dynamic_pointer_cast<const CSketchCircle>(m_seg)) {
      outCenter = circle->center;
      outRadius = circle->radius;
      return true;
    }
    return false;
  }

  template <typename PointT>
  bool GetCircleParams(PointT &outCenter, double &outRadius) const {
    CPoint3D c;
    double r;
    if (!GetCircleParams(c, r))
      return false;
    outCenter = PointWriter<PointT>::Convert(c);
    outRadius = r;
    return true;
  }

  // --- 圆弧特有方法 ---
  bool GetArcParams(CPoint3D &outCenter, double &outStart, double &outEnd,
                    double &outRadius, bool &outClockWise) const {
    if (auto arc = std::dynamic_pointer_cast<const CSketchArc>(m_seg)) {
      outCenter = arc->center;
      outStart = arc->startAngle;
      outEnd = arc->endAngle;
      outRadius = arc->radius;
      outClockWise = arc->isClockwise;
      return true;
    }
    return false;
  }

  template <typename PointT>
  bool GetArcParams(PointT &outCenter, double &outStart, double &outEnd,
                    double &outRadius, bool &outClockWise) const {
    CPoint3D c;
    double s, e, r;
    bool cw;
    if (!GetArcParams(c, s, e, r, cw))
      return false;
    outCenter = PointWriter<PointT>::Convert(c);
    outStart = s;
    outEnd = e;
    outRadius = r;
    outClockWise = cw;
    return true;
  }

  // --- 点特有方法 ---
  bool GetPointCoord(CPoint3D &outPos) const {
    if (auto point = std::dynamic_pointer_cast<const CSketchPoint>(m_seg)) {
      outPos = point->position;
      return true;
    }
    return false;
  }

  template <typename PointT> bool GetPointCoord(PointT &outPos) const {
    CPoint3D p;
    if (!GetPointCoord(p))
      return false;
    outPos = PointWriter<PointT>::Convert(p);
    return true;
  }

  /**
   * @brief 泛型类型转换（优化访问体验）。
   *
   * 允许用户直接获取底层特定类型的指针，避免使用繁琐的 GetXXX(out) 模式。
   * 使用示例：
   *   if (auto circle = seg.As<CSketchCircle>()) {
   *       std::cout << circle->radius;
   *   }
   */
  template <typename T> std::shared_ptr<const T> As() const {
    return std::dynamic_pointer_cast<const T>(m_seg);
  }

  // --- 底层访问 ---
  std::shared_ptr<const CSketchSeg> GetRaw() const { return m_seg; }
};

/**
 * @brief 草图特征访问器。
 *
 * 对应 Builder 层的 SketchBuilder，提供对草图的只读访问。
 * 包括参考平面和几何段的遍历。
 */
class SketchAccessor : public FeatureAccessorBase {
private:
  std::shared_ptr<const CSketch> m_sketch;

public:
  explicit SketchAccessor(std::shared_ptr<const CFeatureBase> feat)
      : FeatureAccessorBase(feat) {
    m_sketch = std::dynamic_pointer_cast<const CSketch>(feat);
  }

  explicit SketchAccessor(const FeatureAccessorBase &other)
      : FeatureAccessorBase(other.GetRaw()) {
    m_sketch = std::dynamic_pointer_cast<const CSketch>(other.GetRaw());
  }

  explicit SketchAccessor(std::shared_ptr<FeatureAccessorBase> feat)
      : FeatureAccessorBase(feat ? feat->GetRaw() : nullptr) {
    if (feat)
      m_sketch = std::dynamic_pointer_cast<const CSketch>(feat->GetRaw());
  }

  const CSketch *Data() const { return m_sketch.get(); }

  const CSketch *operator->() const { return m_sketch.get(); }

  bool IsValid() const override { return m_sketch != nullptr; }

  // --- 参考平面 ---
  ACCESSOR_REF_GETTER(ReferenceAccessor, ReferencePlane, referencePlane)

  bool HasReferencePlane() const {
    return IsValid() && Data()->referencePlane != nullptr;
  }

  // --- 局部坐标系 ---
  bool GetCSys(CPoint3D &outOrigin, CVector3D &outXDir, CVector3D &outYDir,
               CVector3D &outZDir) const {
    if (!IsValid())
      return false;
    outOrigin = m_sketch->sketchCSys.origin;
    outXDir = m_sketch->sketchCSys.xDir;
    outYDir = m_sketch->sketchCSys.yDir;
    outZDir = m_sketch->sketchCSys.zDir;
    return true;
  }

  template <typename PointT, typename VectorT>
  bool GetCSys(PointT &outOrigin, VectorT &outXDir, VectorT &outYDir,
               VectorT &outZDir) const {
    CPoint3D o;
    CVector3D x, y, z;
    if (!GetCSys(o, x, y, z))
      return false;
    outOrigin = PointWriter<PointT>::Convert(o);
    outXDir = VectorWriter<VectorT>::Convert(x);
    outYDir = VectorWriter<VectorT>::Convert(y);
    outZDir = VectorWriter<VectorT>::Convert(z);
    return true;
  }

  // --- 几何段访问 ---
  int GetSegmentCount() const {
    if (!IsValid())
      return 0;
    return m_sketch->segments.size();
  }

  SketchSegmentAccessor GetSegment(int index) const {
    if (!IsValid() || index < 0 || index >= (int)m_sketch->segments.size()) {
      return SketchSegmentAccessor(nullptr);
    }
    return SketchSegmentAccessor(m_sketch->segments[index]);
  }

  SketchSegmentAccessor GetSegmentByLocalID(const std::string &localID) const {
    if (!IsValid())
      return SketchSegmentAccessor(nullptr);
    for (const auto &seg : m_sketch->segments) {
      if (seg && seg->localID == localID) {
        return SketchSegmentAccessor(seg);
      }
    }
    return SketchSegmentAccessor(nullptr);
  }

  // --- 约束访问 ---
  int GetConstraintCount() const {
    if (!IsValid())
      return 0;
    return m_sketch->constraints.size();
  }

  const CSketchConstraint *GetConstraint(int index) const {
    if (!IsValid() || index < 0 || index >= (int)m_sketch->constraints.size()) {
      return nullptr;
    }
    return &m_sketch->constraints[index];
  }
};

} // namespace Accessor
} // namespace CADExchange
