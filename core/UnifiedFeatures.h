#pragma once

#include "UnifiedTypes.h"
#include <memory>
#include <optional>
#include <string>
#include <vector>

#ifndef CEREAL_NVP
#define CEREAL_NVP(x) x
#endif

namespace CADExchange {

/**
 * @brief 描述特征之间的布尔运算类型。
 */
enum class BooleanOp { BOSS, CUT, MERGE };

/**
 * @brief 所有特征的基类。
 */
struct CFeatureBase {
  std::string featureID;             ///< 全局唯一标识符 (UUID)
  std::string featureName;           ///< 用户可见名称
  std::string externalID;            ///< 外部系统 ID (可选)
  bool isSuppressed = false;         ///< 是否被抑制，不参与求解
  virtual ~CFeatureBase() = default; ///< 虚析构函数，避免对象截断
};

/**
 * @brief 引用系统 (Topological References)，用于存放拓扑指纹。
 */
enum class RefType {
  FEATURE_DATUM_PLANE,  // 基准面
  FEATURE_DATUM_AXIS,   // 基准轴
  FEATURE_DATUM_POINT,  // 基准点
  FEATURE_WHOLE_SKETCH, // 整体草图
  TOPO_FACE,            // 拓扑面
  TOPO_EDGE,            // 拓扑边
  TOPO_VERTEX,          // 拓扑顶点
  TOPO_SKETCH_SEG       // 拓扑草图段
};

struct CRefEntityBase {
  RefType refType = RefType::FEATURE_DATUM_PLANE;
  virtual ~CRefEntityBase() = default;
};

struct CRefFeature : public CRefEntityBase {
  std::string targetFeatureID;

  CRefFeature(RefType type = RefType::FEATURE_DATUM_PLANE) { refType = type; }
};

struct CRefSubTopo : public CRefEntityBase {
  std::string parentFeatureID;
  int topologyIndex = -1;

  CRefSubTopo(RefType type = RefType::FEATURE_DATUM_PLANE) { refType = type; }
};

struct CRefPlane : public CRefFeature {
  CPoint3D origin{};
  CVector3D xDir{};
  CVector3D yDir{};
  CVector3D normal{};

  CRefPlane() : CRefFeature(RefType::FEATURE_DATUM_PLANE) {}
};

struct CRefSketch : public CRefFeature {
  CRefSketch() : CRefFeature(RefType::FEATURE_WHOLE_SKETCH) {}
};

struct CRefFace : public CRefSubTopo {
  CVector3D normal;
  CPoint3D centroid; // 几何中心
  CVector3D uDir{1, 0, 0};
  CVector3D vDir{0, 1, 0};

  CRefFace() : CRefSubTopo(RefType::TOPO_FACE) {}
};

struct CRefEdge : public CRefSubTopo {
  CPoint3D midPoint;

  CRefEdge() : CRefSubTopo(RefType::TOPO_EDGE) {}
};

struct CRefVertex : public CRefSubTopo {
  CPoint3D pos;

  CRefVertex() : CRefSubTopo(RefType::TOPO_VERTEX) {}
};

struct CRefSketchSeg : public CRefSubTopo {
  std::string segmentLocalID;

  CRefSketchSeg() : CRefSubTopo(RefType::TOPO_SKETCH_SEG) {}
};

/**
 * @brief 草图元素基类。
 */
struct CSketchSeg {
  enum class SegType { LINE, CIRCLE, ARC, SPLINE, POINT } type;
  std::string localID;         ///< 元素局部ID
  bool isConstruction = false; ///< 是否为参考几何
  virtual ~CSketchSeg() = default;
};

struct CSketchLine : public CSketchSeg {
  CPoint3D startPos;
  CPoint3D endPos;
  CSketchLine() { type = SegType::LINE; }
};

struct CSketchCircle : public CSketchSeg {
  CPoint3D center;
  double radius = 0.0;
  CSketchCircle() { type = SegType::CIRCLE; }
};

struct CSketchArc : public CSketchSeg {
  CPoint3D center;
  double radius = 0.0;
  double startAngle = 0.0;
  double endAngle = 0.0;
  bool isClockwise = false;
  CSketchArc() { type = SegType::ARC; }
};

struct CSketchPoint : public CSketchSeg {
  CPoint3D position;
  CSketchPoint() { type = SegType::POINT; }
};

/**
 * @brief 约束类型定义。
 */
struct CSketchConstraint {
  enum class ConstraintType {
    HORIZONTAL, // 水平约束
    VERTICAL,
    COINCIDENT,
    CONCENTRIC,
    TANGENT,
    EQUAL,
    PARALLEL,
    PERPENDICULAR,
    DIMENSIONAL
  } type;
  std::vector<std::string> entityLocalIDs;
  double dimensionValue = 0.0;
};

/**
 * @brief 草图特征。
 */
struct CSketch : public CFeatureBase {
  std::shared_ptr<CRefEntityBase> referencePlane;
  std::vector<std::shared_ptr<CSketchSeg>> segments;
  std::vector<CSketchConstraint> constraints;
};

/**
 * @brief 薄壁选项。
 */
struct DraftOption {
  double angle = 0.0;
  bool outward = false;
};

struct ThinWallOption {
  double thickness = 0.0;
  bool isOneSided = true;
  bool isCovered = false;
};

/**
 * @brief 拉伸结束条件。
 */
struct ExtrudeEndCondition {
  enum class Type {
    BLIND,        // 盲孔
    THROUGH_ALL,  // 贯穿
    UP_TO_NEXT,   // 到下一面
    UP_TO_FACE,   // 到指定面
    UP_TO_VERTEX, // 到指定顶点
    MID_PLANE     // 中间平面
  } type = Type::BLIND;
  double depth = 0.0;
  double offset = 0.0;
  bool hasOffset = false;                          ///< 是否启用偏移
  std::shared_ptr<CRefEntityBase> referenceEntity; ///< 参考实体
  bool isFlip = false;                             ///< 是否反转方向
  bool isFlipMaterialSide = false;                 ///< 是否反转材料侧
};

/**
 * @brief 拉伸特征。
 */
struct CExtrude : public CFeatureBase {
  std::shared_ptr<CSketch> sketchProfile;
  CVector3D direction = {0, 0, 1};
  ExtrudeEndCondition endCondition1;
  std::optional<ExtrudeEndCondition> endCondition2;
  BooleanOp operation = BooleanOp::BOSS; ///< 默认新建实体
  std::optional<DraftOption> draft;
  std::optional<ThinWallOption> thinWall;
};

/**
 * @brief 旋转轴描述。
 */
struct CRevolveAxis {
  enum class Kind { SketchLine, Explicit, Reference } kind = Kind::Explicit;
  std::string referenceLocalID;
  std::shared_ptr<CRefEntityBase> referenceEntity;
  CPoint3D origin;
  CVector3D direction;
};

/**
 * @brief 旋转特征。
 */
struct CRevolve : public CFeatureBase {
  std::string profileSketchID;
  CRevolveAxis axis;
  enum class AngleKind {
    Single,
    TwoWay,
    Symmetric
  } angleKind = AngleKind::Single;
  double primaryAngle = 0.0;
  double secondaryAngle = 0.0;
};

} // namespace CADExchange
