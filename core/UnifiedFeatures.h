#pragma once
// clang-format off
#include "UnifiedTypes.h"
#include <memory>
#include <optional>
#include <string>
#include <vector>
// clang-format on

// Placeholder macro - will be overridden by cereal if included later
#ifndef CEREAL_NVP
#define CEREAL_NVP(x) x // 占位宏，避免编译错误
#endif

namespace CADExchange {

/**
 * @brief 描述特征之间的布尔运算类型。
 */
enum class BooleanOp { BOSS, CUT, MERGE };

/**
 * @brief 特征类型枚举
 */
enum class FeatureType { Unknown, Extrude, Revolve, Sketch, DatumPlane };

/**
 * @brief 所有特征的基类。
 */
struct CFeatureBase {
  std::string featureID;     ///< 全局唯一标识符 (UUID)
  std::string featureName;   ///< 用户可见名称
  bool isSuppressed = false; ///< 是否被抑制，不参与求解
  FeatureType featureType =
      FeatureType::Unknown;          ///< 特征类型，避免 dynamic_cast
  virtual ~CFeatureBase() = default; ///< 虚析构函数，避免对象截断
};

// ------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

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
  TOPO_SKETCH_SEG,      // 拓扑草图段
  UNKNOWN               // 未知/未初始化类型
};

struct CRefEntityBase {
  RefType refType = RefType::UNKNOWN;
  virtual ~CRefEntityBase() = default;
};

struct CRefFeature : public CRefEntityBase {
  std::string targetFeatureID;

  CRefFeature(RefType type = RefType::UNKNOWN) { refType = type; }
};

struct CRefSubTopo : public CRefEntityBase {
  std::string parentFeatureID;
  /**
   * @deprecated Legacy topology hint kept only for backward compatibility.
   * New read/write logic should not rely on this value for reference matching.
   */
  [[deprecated("Legacy compatibility only; do not use TopologyIndex for new references.")]]
  int topologyIndex = -1;

  CRefSubTopo(RefType type = RefType::UNKNOWN) { refType = type; }
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
  /// 面上采样点：SolidWorks 二次开发中取三角剖分 (GetTessTriangles)
  /// 第一个三角形的重心， 保证该点落在面的几何表面内，用于 SelectByRay
  /// 选面。并非该面的几何质心。
  CPoint3D centroid;
  CVector3D uDir{1, 0, 0};
  CVector3D vDir{0, 1, 0};

  CRefFace() : CRefSubTopo(RefType::TOPO_FACE) {}
};

struct CRefEdge : public CRefSubTopo {
  CPoint3D startPoint;
  CPoint3D endPoint;
  CPoint3D midPoint;

  CRefEdge() : CRefSubTopo(RefType::TOPO_EDGE) {}
};

/**
 * @brief 边的几何属性扩展：在拓扑边基础上附加曲线类型。
 *
 * 用于几何调试输出和可视化，避免边在 JSON 中只剩下三个点而丢失
 * 原始曲线语义。
 */
enum class CGeoCurveType {
  UNKNOWN = 0,
  LINE = 3001,
  CIRCLE = 3002,
  ELLIPSE = 3003,
  INTERSECTION = 3004,
  BCURVE = 3005,
  SPCURVE = 3006,
  CONSTPARAM = 3008,
  TRIMMED = 3009
};

struct CGeoEdge : public CRefEdge {
  CGeoCurveType curveType = CGeoCurveType::UNKNOWN;

  CGeoEdge() = default;
};

struct CRefVertex : public CRefSubTopo {
  CPoint3D pos;

  CRefVertex() : CRefSubTopo(RefType::TOPO_VERTEX) {}
};

struct CRefSketchSeg : public CRefSubTopo {
  std::string segmentLocalID;

  CRefSketchSeg() : CRefSubTopo(RefType::TOPO_SKETCH_SEG) {}
};

struct CRefAxis : public CRefFeature {
  CPoint3D origin{};
  CVector3D direction{};

  CRefAxis() : CRefFeature(RefType::FEATURE_DATUM_AXIS) {}
};

struct CRefPoint : public CRefFeature {
  CPoint3D position{};

  CRefPoint() : CRefFeature(RefType::FEATURE_DATUM_POINT) {}
};

// ------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

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
 * @brief 草图局部坐标系
 */
struct CSketchCSys {
  CPoint3D origin;
  CVector3D xDir;
  CVector3D yDir;
  CVector3D zDir;

  bool IsValid() const {
    CVector3D zCross = Cross(xDir, yDir);
    zCross.Normalize();
    CVector3D zDirNorm = zDir;
    zDirNorm.Normalize();
    return zCross.IsParallel(zDirNorm);
  }
};

/**
 * @brief 几何调试输出中的辅助基准面记录。
 *
 * 只保存位置与局部坐标系，不承载完整的基准面约束/引用树。
 */
struct CGeoDatumPlane {
  std::string targetFeatureID;
  CSketchCSys localCSys;
  std::string type = "Plane";
};

/**
 * @brief 草图特征。
 */
struct CSketch : public CFeatureBase {
  std::shared_ptr<CRefEntityBase> referencePlane;
  std::vector<std::shared_ptr<CSketchSeg>> segments;
  std::vector<CSketchConstraint> constraints;
  CSketchCSys sketchCSys;

  CSketch() { featureType = FeatureType::Sketch; }
};

/**
 * @brief 拔模、薄壁选项。
 */
struct DraftOption {
  double angle = 0.0;
  bool outward = false;
};

struct ThinWallOption {
  double startOffset = 0.0; ///< 相对草图轮廓的内侧偏置，向内为负
  double endOffset = 0.0;   ///< 相对草图轮廓的外侧偏置，向外为正
  bool isCovered = false;   ///< 薄壁端部是否有盖
};

/**
 * @brief 共享扫掠终止条件。
 *
 * 用于统一 Extrude / Revolve 的单向、双向、对称以及参考终止表达。
 */
struct SweepExtent {
  enum class Type {
    VALUE,                  // 数值（Extrude: 深度 / Revolve: 角度）
    SYMMETRIC,              // 对称
    THROUGH_ALL,            // 贯穿
    THROUGH_ALL_BOTH_SIDES, // 双向贯穿（保留给 Extrude 兼容）
    UP_TO_NEXT,             // 到下一面
    UP_TO_ENTITY,           // 到指定参考实体
    UP_TO_EXTENDED,         // 延伸到参考实体
    THRU_POINT,             // 过点
    UNKNOWN
  } type = Type::UNKNOWN;

  double value = 0.0;
  double offset = 0.0;
  bool hasOffset = false;                          ///< 是否启用偏移
  std::shared_ptr<CRefEntityBase> referenceEntity; ///< 参考实体
  bool isFlip = false;                             ///< 是否反转方向
  bool isFlipMaterialSide = false;                 ///< 是否反转材料侧
  std::optional<CPoint3D> helperPoint;             ///< 多解时用于消歧
};

/**
 * @brief 轮廓驱动特征的共享基类。
 */
struct CProfiledFeatureBase : public CFeatureBase {
  std::string profileSketchID; ///< 草图轮廓的特征 ID（引用，不嵌入）
  BooleanOp operation = BooleanOp::BOSS;
  std::optional<ThinWallOption> thinWall;
};

/**
 * @brief 拉伸特征。
 */
struct CExtrude : public CProfiledFeatureBase {
  CVector3D direction = {0, 0, 1};
  SweepExtent extent1;
  std::optional<SweepExtent> extent2;
  std::optional<DraftOption> draft;

  CExtrude() { featureType = FeatureType::Extrude; }
};

/**
 * @brief 旋转轴描述。
 */
struct CRevolveAxis {
  std::string referenceLocalID;
  std::shared_ptr<CRefEntityBase> referenceEntity;
  CPoint3D origin;
  CVector3D direction;
};

/**
 * @brief 旋转特征。
 */
struct CRevolve : public CProfiledFeatureBase {
  CRevolveAxis axis;
  SweepExtent extent1;
  std::optional<SweepExtent> extent2;

  CRevolve() { featureType = FeatureType::Revolve; }
};

enum class PlaneMethod {
  UNKNOWN = 0,

  // P0
  OFFSET, // 偏置平面
  FIXED,  // 点+法向 / 固定平面

  // P1
  ANGLE,         // 角度平面
  PARALLEL,      // 平行平面
  PERPENDICULAR, // 垂直相关
  MID_PLANE,     // 中间面
  THREE_POINTS,  // 三点面
  LINE,          // 过线/两线定义
  TANGENT        // 相切面
};

enum class PlaneConstraintType {
  UNKNOWN = 0,

  PARALLEL,      // 平行
  PERPENDICULAR, // 垂直
  COINCIDENT,    // 重合
  DISTANCE,      // 距离
  ANGLE,         // 角度
  SYMMETRIC,     // 对称
  TANGENT,       // 相切
  PROJECTION     // 投影
};

struct PlaneConstraint {
  PlaneConstraintType type{PlaneConstraintType::UNKNOWN};

  // 指向哪个引用实体
  int ref{-1};

  // 参数：
  // - DISTANCE: 线性距离，随模型单位缩放后写入/读取
  // - ANGLE: 角度，内部统一用弧度
  double value{0.0};

  // 参数默认方向
  std::optional<CVector3D> defaultDir;

  // 是否反转（注意：是局部的）
  bool reversed{false};
};

/**
 * @brief 基准面特征。
 */
struct CDatumPlane : public CFeatureBase {
  PlaneMethod method{PlaneMethod::UNKNOWN};
  std::vector<PlaneConstraint> constraints;
  std::vector<std::shared_ptr<CRefEntityBase>> referenceEntities;

  CDatumPlane() { featureType = FeatureType::DatumPlane; }
};

} // namespace CADExchange
