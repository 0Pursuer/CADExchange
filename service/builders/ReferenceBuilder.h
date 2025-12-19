#pragma once
// clang-format off
#include "../../core/UnifiedFeatures.h"
#include "../../core/UnifiedTypes.h"
#include "../../core/TypeAdapters.h"
#include "BuilderMacros.h"
// clang-format on
namespace CADExchange {
namespace Builder {

/**
 * @brief 拓扑面引用构建器辅助类。
 * 
 */
class RefFaceBuilder {
  std::shared_ptr<CRefFace> m_ptr;

public:

  /**
   * @brief 构造一个新的面引用构建器。
   * 
   * @param parentID  父特征 ID
   * @param index   拓扑索引，默认为 0
   */
  RefFaceBuilder(const std::string &parentID, int index = 0) {
    m_ptr = std::make_shared<CRefFace>();
    m_ptr->parentFeatureID = parentID;
    m_ptr->topologyIndex = index;
  }
  /**
   * @brief 设置面引用的质心位置。
   */
  BUILDER_ADD_POINT_SETTER(RefFaceBuilder, centroid, Centroid)

  /**
   * @brief 设置面引用的法向量。
   */
  BUILDER_ADD_VECTOR_SETTER(RefFaceBuilder, normal, Normal)

  /**
   * @brief 设置面引用的U方向。
   */
  BUILDER_ADD_VECTOR_SETTER(RefFaceBuilder, uDir, UDir)

  /**
   * @brief 设置面引用的V方向。
   */
  BUILDER_ADD_VECTOR_SETTER(RefFaceBuilder, vDir, VDir)
  
  /**
   * @brief 构建完成，返回面引用指针。
   */
  std::shared_ptr<CRefFace> Build() const { return m_ptr; }
  
  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; } // 转换为基类指针
  operator std::shared_ptr<CRefFace>() const { return m_ptr; } // 转换为具体类型指针
};

// Helper Builder for Vertex References
class RefVertexBuilder {
  std::shared_ptr<CRefVertex> m_ptr;

public:
  RefVertexBuilder(const std::string &parentID, int index = 0) {
    m_ptr = std::make_shared<CRefVertex>();
    m_ptr->parentFeatureID = parentID;
    m_ptr->topologyIndex = index;
  }
  
  /**
   * @brief 设置顶点位置。
   */
  BUILDER_ADD_POINT_SETTER(RefVertexBuilder, pos, Pos)

  /**
   * @brief 构建完成，返回顶点引用指针。
   */
  std::shared_ptr<CRefVertex> Build() const { return m_ptr; }

  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; }
  operator std::shared_ptr<CRefVertex>() const { return m_ptr; }
};

// Helper Builder for Edge References
class RefEdgeBuilder {
  std::shared_ptr<CRefEdge> m_ptr;

public:
  RefEdgeBuilder(const std::string &parentID, int index = 0) {
    m_ptr = std::make_shared<CRefEdge>();
    m_ptr->parentFeatureID = parentID;
    m_ptr->topologyIndex = index;
  }
  
  /**
   * @brief 设置边的起点。
   */
  BUILDER_ADD_POINT_SETTER(RefEdgeBuilder, startPoint, StartPoint)

  /**
   * @brief 设置边的终点。
   */
  BUILDER_ADD_POINT_SETTER(RefEdgeBuilder, endPoint, EndPoint)

  /**
   * @brief 设置边的中点。
   */
  BUILDER_ADD_POINT_SETTER(RefEdgeBuilder, midPoint, MidPoint)

  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; }
  operator std::shared_ptr<CRefEdge>() const { return m_ptr; }
};

// Helper Builder for Sketch References
class RefSketchBuilder {
  std::shared_ptr<CRefSketch> m_ptr;

public:
  /**
   * @brief 构造一个新的草图引用构建器。
   * 
   * @param sketchID 草图特征 ID
   */
  RefSketchBuilder(const std::string &sketchID) {
    m_ptr = std::make_shared<CRefSketch>();
    m_ptr->targetFeatureID = sketchID;
  }
  
  /**
   * @brief 构建完成，返回草图引用指针。
   */
  std::shared_ptr<CRefSketch> Build() const { return m_ptr; }
  
  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; }
  operator std::shared_ptr<CRefSketch>() const { return m_ptr; }
};

// Helper Builder for Plane References (Datum Plane)
class RefPlaneBuilder {
  std::shared_ptr<CRefPlane> m_ptr;

public:
  /**
   * @brief 构造一个新的基准面引用构建器。
   * 
   * @param planeID 基准面特征 ID
   */
  RefPlaneBuilder(const std::string &planeID) {
    m_ptr = std::make_shared<CRefPlane>();
    m_ptr->targetFeatureID = planeID;
  }
  
  /**
   * @brief 设置基准面的原点。
   */
  BUILDER_ADD_POINT_SETTER(RefPlaneBuilder, origin, Origin)

  /**
   * @brief 设置基准面的 X 方向。
   */
  BUILDER_ADD_VECTOR_SETTER(RefPlaneBuilder, xDir, XDir)
  
  /**
   * @brief 设置基准面的 Y 方向。
   */
  BUILDER_ADD_VECTOR_SETTER(RefPlaneBuilder, yDir, YDir)
  
  /**
   * @brief 设置基准面的法向量。
   */
  BUILDER_ADD_VECTOR_SETTER(RefPlaneBuilder, normal, Normal)

  /**
   * @brief 构建完成，返回基准面引用指针。
   */
  std::shared_ptr<CRefPlane> Build() const { return m_ptr; }
  
  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; }
  operator std::shared_ptr<CRefPlane>() const { return m_ptr; }
};

// Helper Builder for Axis References (Datum Axis)
class RefAxisBuilder {
  std::shared_ptr<CRefAxis> m_ptr;

public:
  /**
   * @brief 构造一个新的基准轴引用构建器。
   * 
   * @param axisID 基准轴特征 ID
   */
  RefAxisBuilder(const std::string &axisID) {
    m_ptr = std::make_shared<CRefAxis>();
    m_ptr->targetFeatureID = axisID;
  }
  
  /**
   * @brief 设置基准轴的原点。
   */
  BUILDER_ADD_POINT_SETTER(RefAxisBuilder, origin, Origin)
  
  /**
   * @brief 设置基准轴的方向向量。
   */
  BUILDER_ADD_VECTOR_SETTER(RefAxisBuilder, direction, Direction)
  
  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; }
  operator std::shared_ptr<CRefAxis>() const { return m_ptr; }
};

// Helper Builder for Point References (Datum Point)
class RefPointBuilder {
  std::shared_ptr<CRefPoint> m_ptr;

public:
  /**
   * @brief 构造一个新的基准点引用构建器。
   * 
   * @param pointID 基准点特征 ID
   */
  RefPointBuilder(const std::string &pointID) {
    m_ptr = std::make_shared<CRefPoint>();
    m_ptr->targetFeatureID = pointID;
  }
  
  /**
   * @brief 设置基准点的位置。
   */
  BUILDER_ADD_POINT_SETTER(RefPointBuilder, position, Position)
  
  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; }
  operator std::shared_ptr<CRefPoint>() const { return m_ptr; }
};

// Helper Builder for Sketch Segment References
class RefSketchSegBuilder {
  std::shared_ptr<CRefSketchSeg> m_ptr;

public:
  /**
   * @brief 构造一个新的草图段引用构建器。
   * 
   * @param parentSketchID 父草图特征 ID
   * @param segmentLocalID 草图段的局部 ID
   * @param index 拓扑索引，默认为 0
   */
  RefSketchSegBuilder(const std::string &parentSketchID, 
                      const std::string &segmentLocalID, 
                      int index = 0) {
    m_ptr = std::make_shared<CRefSketchSeg>();
    m_ptr->parentFeatureID = parentSketchID;
    m_ptr->segmentLocalID = segmentLocalID;
    m_ptr->topologyIndex = index;
  }
  
  operator std::shared_ptr<CRefEntityBase>() const { return m_ptr; }
  operator std::shared_ptr<CRefSketchSeg>() const { return m_ptr; }
};




// Static Facade
class Ref {
public:
  /**
   * @brief 创建拓扑面引用。
   */
  static RefFaceBuilder Face(const std::string &parentID, int index = 0) {
    return RefFaceBuilder(parentID, index);
  }
  
  /**
   * @brief 创建拓扑顶点引用。
   */
  static RefVertexBuilder Vertex(const std::string &parentID, int index = 0) {
    return RefVertexBuilder(parentID, index);
  }
  
  /**
   * @brief 创建拓扑边引用。
   */
  static RefEdgeBuilder Edge(const std::string &parentID, int index = 0) {
    return RefEdgeBuilder(parentID, index);
  }

  /**
   * @brief 创建基准面引用。
   */
  static RefPlaneBuilder Plane(const std::string &planeID) {
    return RefPlaneBuilder(planeID);
  }

  /**
   * @brief 创建基准轴引用。
   */
  static RefAxisBuilder Axis(const std::string &axisID) {
    return RefAxisBuilder(axisID);
  }

  /**
   * @brief 创建基准点引用。
   */
  static RefPointBuilder Point(const std::string &pointID) {
    return RefPointBuilder(pointID);
  }

  /**
   * @brief 创建草图引用。
   */
  static RefSketchBuilder Sketch(const std::string &sketchID) {
    return RefSketchBuilder(sketchID);
  }

  /**
   * @brief 创建草图段引用。
   */
  static RefSketchSegBuilder SketchSegment(const std::string &parentSketchID,
                                          const std::string &segmentLocalID,
                                          int index = 0) {
    return RefSketchSegBuilder(parentSketchID, segmentLocalID, index);
  }

  // Standard Planes
  static RefPlaneBuilder XY() {
    return RefPlaneBuilder(StandardID::PLANE_XY)
      .Origin(StandardID::kOrigin)
      .XDir(StandardID::kAxisX)
      .YDir(StandardID::kAxisY)
      .Normal(StandardID::kPlaneXYNormal);
  }
  
  static RefPlaneBuilder YZ() {
    return RefPlaneBuilder(StandardID::PLANE_YZ)
      .Origin(StandardID::kOrigin)
      .XDir(StandardID::kAxisY)
      .YDir(StandardID::kAxisZ)
      .Normal(StandardID::kPlaneYZNormal);
  }
  
  static RefPlaneBuilder ZX() {
    return RefPlaneBuilder(StandardID::PLANE_ZX)
      .Origin(StandardID::kOrigin)
      .XDir(StandardID::kAxisZ)
      .YDir(StandardID::kAxisX)
      .Normal(StandardID::kPlaneZXNormal);
  }
};

} // namespace Builder
} // namespace CADExchange
