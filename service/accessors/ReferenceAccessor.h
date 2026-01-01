#pragma once
#include "../../core/TypeAdapters.h"
#include "../../core/UnifiedFeatures.h"
#include <memory>
#include <string>

namespace CADExchange {
namespace Accessor {

/**
 * @brief 引用实体通用访问器，对应 Builder 层的 ReferenceBuilder 体系。
 *
 * 这是一个"万能钥匙"，可以处理任何 CRefEntityBase 子类。
 * 通过尝试性的 dynamic_cast 来进行多态访问。
 *
 * 设计哲学："安全地拆包"
 * - 先问"你是什么类型？"(GetRefType)
 * - 再问"你有法向量吗？"(GetFaceNormal)
 * - 最后取数据
 */
class ReferenceAccessor {
private:
  std::shared_ptr<const CRefEntityBase> m_ref;

public:
  explicit ReferenceAccessor(
      std::shared_ptr<const CRefEntityBase> ref = nullptr)
      : m_ref(std::move(ref)) {}

  // --- 基本检查 ---
  bool IsValid() const { return m_ref != nullptr; }

  // --- 类型识别 ---
  RefType GetRefType() const {
    if (!IsValid())
      return RefType::FEATURE_DATUM_PLANE;
    return m_ref->refType;
  }

  // --- 通用属性（尝试转换） ---

  /**
   * @brief 获取父特征 ID（适用于 SubTopo 类型的引用）。
   * 面、边、顶点等都有父特征 ID。
   */
  std::string GetParentFeatureID() const {
    if (auto topo = std::dynamic_pointer_cast<const CRefSubTopo>(m_ref)) {
      return topo->parentFeatureID;
    }
    return "";
  }

  /**
   * @brief 获取目标特征 ID（适用于 Feature Reference 类型）。
   * 基准平面、基准轴、基准点等指向目标特征。
   */
  std::string GetTargetFeatureID() const {
    if (auto feat = std::dynamic_pointer_cast<const CRefFeature>(m_ref)) {
      return feat->targetFeatureID;
    }
    return "";
  }

  /**
   * @brief 判断当前引用是否指向标准基准（面、轴或原点）。
   */
  bool IsStandard() const {
    if (auto feat = std::dynamic_pointer_cast<const CRefFeature>(m_ref)) {
      const std::string &id = feat->targetFeatureID;
      return StandardID::IsStandardPlane(id) ||
             StandardID::IsStandardAxis(id) || StandardID::IsStandardPoint(id);
    }
    return false;
  }

  /**
   * @brief 获取拓扑索引（用于面、边、顶点）。
   */
  int GetTopologyIndex() const {
    if (auto topo = std::dynamic_pointer_cast<const CRefSubTopo>(m_ref)) {
      return topo->topologyIndex;
    }
    return -1;
  }

  // --- 几何指纹获取（扁平化接口） ---

  /**
   * @brief 尝试获取面法向（如果是面引用）。
   * @return true 表示成功获取，false 表示不是面或无法获取
   */
  bool GetFaceNormal(CVector3D &outNormal) const {
    if (auto face = std::dynamic_pointer_cast<const CRefFace>(m_ref)) {
      outNormal = face->normal;
      return true;
    }
    return false;
  }

  template <typename VecT> bool GetFaceNormal(VecT &outNormal) const {
    CVector3D tmp;
    if (!GetFaceNormal(tmp))
      return false;
    outNormal = VectorWriter<VecT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取面质心（如果是面引用）。
   */
  bool GetFaceCentroid(CPoint3D &outCentroid) const {
    if (auto face = std::dynamic_pointer_cast<const CRefFace>(m_ref)) {
      outCentroid = face->centroid;
      return true;
    }
    return false;
  }

  template <typename PointT> bool GetFaceCentroid(PointT &outCentroid) const {
    CPoint3D tmp;
    if (!GetFaceCentroid(tmp))
      return false;
    outCentroid = PointWriter<PointT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取面 U 方向（如果是面引用）。
   */
  bool GetFaceUDir(CVector3D &outUDir) const {
    if (auto face = std::dynamic_pointer_cast<const CRefFace>(m_ref)) {
      outUDir = face->uDir;
      return true;
    }
    return false;
  }

  template <typename VecT> bool GetFaceUDir(VecT &outUDir) const {
    CVector3D tmp;
    if (!GetFaceUDir(tmp))
      return false;
    outUDir = VectorWriter<VecT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取面 V 方向（如果是面引用）。
   */
  bool GetFaceVDir(CVector3D &outVDir) const {
    if (auto face = std::dynamic_pointer_cast<const CRefFace>(m_ref)) {
      outVDir = face->vDir;
      return true;
    }
    return false;
  }

  template <typename VecT> bool GetFaceVDir(VecT &outVDir) const {
    CVector3D tmp;
    if (!GetFaceVDir(tmp))
      return false;
    outVDir = VectorWriter<VecT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取边的起点（如果是边引用）。
   */
  bool GetEdgeStartPoint(CPoint3D &outStart) const {
    if (auto edge = std::dynamic_pointer_cast<const CRefEdge>(m_ref)) {
      outStart = edge->startPoint;
      return true;
    }
    return false;
  }

  template <typename PointT> bool GetEdgeStartPoint(PointT &outStart) const {
    CPoint3D tmp;
    if (!GetEdgeStartPoint(tmp))
      return false;
    outStart = PointWriter<PointT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取边的终点（如果是边引用）。
   */
  bool GetEdgeEndPoint(CPoint3D &outEnd) const {
    if (auto edge = std::dynamic_pointer_cast<const CRefEdge>(m_ref)) {
      outEnd = edge->endPoint;
      return true;
    }
    return false;
  }

  template <typename PointT> bool GetEdgeEndPoint(PointT &outEnd) const {
    CPoint3D tmp;
    if (!GetEdgeEndPoint(tmp))
      return false;
    outEnd = PointWriter<PointT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取边的中点（如果是边引用）。
   */
  bool GetEdgeMidPoint(CPoint3D &outMid) const {
    if (auto edge = std::dynamic_pointer_cast<const CRefEdge>(m_ref)) {
      outMid = edge->midPoint;
      return true;
    }
    return false;
  }

  template <typename PointT> bool GetEdgeMidPoint(PointT &outMid) const {
    CPoint3D tmp;
    if (!GetEdgeMidPoint(tmp))
      return false;
    outMid = PointWriter<PointT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取顶点位置（如果是顶点引用）。
   */
  bool GetVertexPosition(CPoint3D &outPos) const {
    if (auto vertex = std::dynamic_pointer_cast<const CRefVertex>(m_ref)) {
      outPos = vertex->pos;
      return true;
    }
    return false;
  }

  template <typename PointT> bool GetVertexPosition(PointT &outPos) const {
    CPoint3D tmp;
    if (!GetVertexPosition(tmp))
      return false;
    outPos = PointWriter<PointT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取平面原点（如果是基准平面）。
   */
  bool GetPlaneOrigin(CPoint3D &outOrigin) const {
    if (auto plane = std::dynamic_pointer_cast<const CRefPlane>(m_ref)) {
      outOrigin = plane->origin;
      return true;
    }
    return false;
  }

  template <typename PointT> bool GetPlaneOrigin(PointT &outOrigin) const {
    CPoint3D tmp;
    if (!GetPlaneOrigin(tmp))
      return false;
    outOrigin = PointWriter<PointT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取平面法向（如果是基准平面）。
   */
  bool GetPlaneNormal(CVector3D &outNormal) const {
    if (auto plane = std::dynamic_pointer_cast<const CRefPlane>(m_ref)) {
      outNormal = plane->normal;
      return true;
    }
    return false;
  }

  template <typename VecT> bool GetPlaneNormal(VecT &outNormal) const {
    CVector3D tmp;
    if (!GetPlaneNormal(tmp))
      return false;
    outNormal = VectorWriter<VecT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取平面 X 方向（如果是基准平面）。
   */
  bool GetPlaneXDir(CVector3D &outXDir) const {
    if (auto plane = std::dynamic_pointer_cast<const CRefPlane>(m_ref)) {
      outXDir = plane->xDir;
      return true;
    }
    return false;
  }

  template <typename VecT> bool GetPlaneXDir(VecT &outXDir) const {
    CVector3D tmp;
    if (!GetPlaneXDir(tmp))
      return false;
    outXDir = VectorWriter<VecT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取平面 Y 方向（如果是基准平面）。
   */
  bool GetPlaneYDir(CVector3D &outYDir) const {
    if (auto plane = std::dynamic_pointer_cast<const CRefPlane>(m_ref)) {
      outYDir = plane->yDir;
      return true;
    }
    return false;
  }

  template <typename VecT> bool GetPlaneYDir(VecT &outYDir) const {
    CVector3D tmp;
    if (!GetPlaneYDir(tmp))
      return false;
    outYDir = VectorWriter<VecT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取轴原点（如果是基准轴）。
   */
  bool GetAxisOrigin(CPoint3D &outOrigin) const {
    if (auto axis = std::dynamic_pointer_cast<const CRefAxis>(m_ref)) {
      outOrigin = axis->origin;
      return true;
    }
    return false;
  }

  template <typename PointT> bool GetAxisOrigin(PointT &outOrigin) const {
    CPoint3D tmp;
    if (!GetAxisOrigin(tmp))
      return false;
    outOrigin = PointWriter<PointT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取轴方向（如果是基准轴）。
   */
  bool GetAxisDirection(CVector3D &outDir) const {
    if (auto axis = std::dynamic_pointer_cast<const CRefAxis>(m_ref)) {
      outDir = axis->direction;
      return true;
    }
    return false;
  }

  template <typename VecT> bool GetAxisDirection(VecT &outDir) const {
    CVector3D tmp;
    if (!GetAxisDirection(tmp))
      return false;
    outDir = VectorWriter<VecT>::Convert(tmp);
    return true;
  }

  /**
   * @brief 尝试获取点位置（如果是基准点）。
   */
  bool GetPointPosition(CPoint3D &outPos) const {
    if (auto point = std::dynamic_pointer_cast<const CRefPoint>(m_ref)) {
      outPos = point->position;
      return true;
    }
    return false;
  }

  template <typename PointT> bool GetPointPosition(PointT &outPos) const {
    CPoint3D tmp;
    if (!GetPointPosition(tmp))
      return false;
    outPos = PointWriter<PointT>::Convert(tmp);
    return true;
  }

  // --- 底层访问 ---
  const CRefEntityBase *Data() const { return m_ref.get(); }

  const CRefEntityBase *operator->() const { return m_ref.get(); }

  std::shared_ptr<const CRefEntityBase> GetRaw() const { return m_ref; }

  /**
   * @brief 获取具体类型的指针（模板方法）。
   */
  template <typename RefT> std::shared_ptr<const RefT> GetAs() const {
    return std::dynamic_pointer_cast<const RefT>(m_ref);
  }
};

} // namespace Accessor
} // namespace CADExchange
