#pragma once
// clang-format off
#include "UnifiedTypes.h"
#include <memory>
#include <optional>
#include <string>
#include <utility>
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
enum class FeatureType {
  Unknown,
  Extrude,
  Revolve,
  Sweep,
  Fillet,
  Chamfer,
  Sketch,
  DatumPlane
};

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

/**
 * @brief 拓扑边的曲线类型。
 *
 * 用于引用边、几何 sidecar 输出以及后续 CAD 写回时保留原始曲线语义。
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

struct CRefEdge : public CRefSubTopo {
  CPoint3D startPoint;
  CPoint3D endPoint;
  CPoint3D midPoint;
  CGeoCurveType curveType = CGeoCurveType::UNKNOWN;

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
 * @brief 草图约束引用的来源类别。
 *
 * `CSketchConstraint` 的每个参与对象都通过 `SketchConstraintRef` 表达。
 * 这里区分“本草图内部图元”与“外部引用实体”两大来源：
 *
 * - `SketchEntity`
 *   表示引用当前草图中的某个图元，使用 `sketchEntityLocalID` 定位。
 * - `ExternalReference`
 *   表示引用草图外部对象，统一复用 `CRefEntityBase` 体系表达，
 *   可指向外部边、面、轴、点、基准面、其他草图段等。
 */
enum class SketchConstraintRefKind {
  SketchEntity,      ///< 当前草图内部图元，由 `sketchEntityLocalID` 定位
  ExternalReference  ///< 草图外部引用对象，由 `refEntity` 定位
};

/**
 * @brief 约束引用落在实体上的哪个子位置。
 *
 * 仅保存“对象身份”不足以表达草图约束语义，因为很多约束并不是作用于
 * 整个实体，而是作用于实体上的特定位置，例如线段起点、终点、圆心、
 * 中点等。
 *
 * 该枚举用于补充这些位点级语义：
 *
 * - `Whole`
 *   约束作用于整个实体，例如水平、垂直、平行、相切中的整条线/整段圆弧。
 * - `Start` / `End`
 *   约束作用于曲线端点，常用于点重合、点在线上等端点级约束。
 * - `Center`
 *   约束作用于圆/圆弧的圆心，常用于同心、半径/直径等场景。
 * - `Midpoint`
 *   约束作用于线段中点，常用于中点约束或某些对称辅助语义。
 */
enum class SketchConstraintSubEntity {
  Whole,     ///< 整个实体
  Start,     ///< 起点
  End,       ///< 终点
  Center,    ///< 圆心/中心点
  Midpoint   ///< 中点
};

/**
 * @brief 草图约束中的单个参与引用。
 *
 * 这是草图约束引用的统一载体，用来表达：
 *
 * 1. 约束引用的是“本草图内部图元”还是“外部引用对象”
 * 2. 若引用的是某个对象，约束具体落在该对象的哪一部分
 *
 * 设计原则：
 *
 * - 内部草图图元：
 *   使用 `kind=SketchEntity` + `sketchEntityLocalID`
 * - 外部引用对象：
 *   使用 `kind=ExternalReference` + `refEntity`
 * - 子实体定位：
 *   始终通过 `subEntity` 明确约束作用位点
 *
 * 这样可以统一覆盖如下场景：
 *
 * - 草图内两条线端点重合
 * - 草图线与外部边重合/平行/垂直
 * - 圆弧与外部轴/边相切
 * - 圆与圆的同心
 * - 中点、圆心、端点等位点级约束
 */
struct SketchConstraintRef {
  SketchConstraintRefKind kind{SketchConstraintRefKind::SketchEntity};
  ///< 引用来源类别：内部草图图元或外部引用实体。

  SketchConstraintSubEntity subEntity{SketchConstraintSubEntity::Whole};
  ///< 约束落在该对象上的哪个子位置。

  std::string sketchEntityLocalID;
  ///< `kind == SketchEntity` 时使用的草图局部图元 ID。

  std::shared_ptr<CRefEntityBase> refEntity;
  ///< `kind == ExternalReference` 时使用的外部引用对象。

  /**
   * @brief 创建一个指向当前草图内部图元的约束引用。
   * @param localID 草图图元 localID。
   * @param subEntity 约束落点，默认为整个实体。
   * @return 构造好的 `SketchConstraintRef`。
   */
  static SketchConstraintRef ForSketchEntity(
      std::string localID,
      SketchConstraintSubEntity subEntity = SketchConstraintSubEntity::Whole) {
    SketchConstraintRef ref;
    ref.kind = SketchConstraintRefKind::SketchEntity;
    ref.subEntity = subEntity;
    ref.sketchEntityLocalID = std::move(localID);
    return ref;
  }

  /**
   * @brief 创建一个指向草图外部引用对象的约束引用。
   * @param reference 外部引用对象，通常为 `CRefEntityBase` 派生类实例。
   * @param subEntity 约束落点，默认为整个实体。
   * @return 构造好的 `SketchConstraintRef`。
   */
  static SketchConstraintRef ForExternalReference(
      std::shared_ptr<CRefEntityBase> reference,
      SketchConstraintSubEntity subEntity = SketchConstraintSubEntity::Whole) {
    SketchConstraintRef ref;
    ref.kind = SketchConstraintRefKind::ExternalReference;
    ref.subEntity = subEntity;
    ref.refEntity = std::move(reference);
    return ref;
  }
};

/**
 * @brief 统一草图约束定义。
 *
 * `CSketchConstraint` 是 CADExchange 草图约束的统一表达，目标是同时支持：
 *
 * - 草图内部图元之间的约束
 * - 草图图元与外部引用实体之间的约束
 * - 常见几何约束
 * - 常见尺寸约束
 * - 原生 CAD 系统附加元数据保真
 *
 * 该结构的核心字段为：
 *
 * - `type`
 *   统一语义层的约束类型
 * - `refs`
 *   参与该约束的一个或多个引用对象
 * - `value`
 *   尺寸类约束的数值参数；非尺寸类通常为空
 * 当前结构只保留统一语义本身，不再额外保存 CAD 原生系统级元数据。
 */
struct CSketchConstraint {
  /**
   * @brief 统一约束类型枚举。
   *
   * 设计上将“几何关系约束”与“尺寸约束”分开，而不是继续使用一个粗粒度
   * 的 `DIMENSIONAL`。这样更有利于对接 SolidWorks、Creo、UG/NX
   * 各自的原生 API 能力。
   */
  enum class ConstraintType {
    COINCIDENT,    ///< 重合
    USEEDGE,       ///< 使用边线/转换实体
    HORIZONTAL,    ///< 水平
    VERTICAL,      ///< 垂直（方向约束）
    PARALLEL,      ///< 平行
    PERPENDICULAR, ///< 垂直（两对象关系）
    TANGENT,       ///< 相切
    CONCENTRIC,    ///< 同心
    EQUAL,         ///< 等值（如等长、等半径等统一入口）
    DISTANCE,      ///< 距离尺寸
    ANGLE,         ///< 角度尺寸
    RADIUS,        ///< 半径尺寸
    DIAMETER,      ///< 直径尺寸
    SYMMETRIC,     ///< 对称
    MIDPOINT,      ///< 中点
    COLLINEAR,     ///< 共线
    FIXED,         ///< 固定
    UNKNOWN        ///< 未知/未归一化约束
  };

  ConstraintType type{ConstraintType::UNKNOWN};
  ///< 统一层约束类型。

  std::vector<SketchConstraintRef> refs;
  ///< 参与该约束的引用对象列表。

  std::optional<double> value;
  ///< 尺寸类约束的数值；非尺寸类一般为空。
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
    VALUE,                  // 数值（Extrude: 深度 / Revolve: 角度，内部统一用弧度）
    SYMMETRIC,              // 对称
    THROUGH_ALL,            // 贯穿
    THROUGH_ALL_BOTH_SIDES, // 双向贯穿（保留给 Extrude 兼容）
    UP_TO_NEXT,             // 到下一面
    UP_TO_ENTITY,           // 到指定参考实体
    UP_TO_EXTENDED,         // 延伸到参考实体
    THRU_POINT,             // 过点
    UNKNOWN
  } type = Type::UNKNOWN;

  // 共享数值参数：
  // - Extrude: 线性长度，跟随模型单位
  // - Revolve: 角度，内部统一使用弧度
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
  CVector3D direction; ///< 统一语义：特征正旋转方向；ReferenceEntity 保留原始几何方向
};

/**
 * @brief 旋转特征。
 */
struct CRevolve : public CProfiledFeatureBase {
  CRevolveAxis axis;
  SweepExtent extent1;              ///< Revolve 角度值统一使用弧度
  std::optional<SweepExtent> extent2; ///< Revolve 第二方向角度值统一使用弧度

  CRevolve() { featureType = FeatureType::Revolve; }
};

/**
 * @brief 扫掠路径引用链。
 *
 * 路径不限定为草图：可以引用草图、草图段、实体边等可解析为曲线链的实体。
 * references 的顺序表达链顺序；起止点用于消除跨 CAD 选择方向差异。
 */
struct CSweepPath {
  std::vector<std::shared_ptr<CRefEntityBase>> references;
  std::optional<CPoint3D> startPoint;
  std::optional<CPoint3D> endPoint;
  bool isClosed = false;
};

/**
 * @brief 扫掠截面相对路径的方向控制。
 */
enum class SweepPathOrientation {
  FollowPath,        ///< 使用各 CAD 默认随路径更新截面方向的实体扫掠行为
  KeepProfileNormal, ///< 尽量保持截面法向，不承诺三系统高级选项完全等价
};

enum class SweepProfileKind {
  SketchReference, ///< 引用模型中特征树上的独立草图
  EmbeddedSketch,  ///< 扫掠特征内部截面草绘，坐标解释依赖 sectionPlacement
  Circular         ///< 参数圆/管截面，不需要独立草图引用
};

struct CSweepCircularProfile {
  double outerRadius = 0.0;
  double innerRadius = 0.0;
};

struct CSweepEmbeddedProfile {
  CSketch sketch;
};

struct CSweepProfile {
  SweepProfileKind kind = SweepProfileKind::SketchReference;
  std::string sketchID;
  std::optional<CSweepEmbeddedProfile> embedded;
  std::optional<CSweepCircularProfile> circular;
};

enum class SweepSectionPlacement {
  ExistingProfilePlane, ///< 截面来自已有草图平面，可能不垂直路径起点切线
  PathNormalAtStart     ///< 截面位于路径起点法平面，适配 Creo/UG 变化扫掠/管道
};

/**
 * @brief 扫掠特征。
 */
struct CSweep : public CProfiledFeatureBase {
  CSweepProfile profile;
  CSweepPath path;
  std::vector<CSweepPath> guidePaths;
  SweepPathOrientation orientation = SweepPathOrientation::FollowPath;
  SweepSectionPlacement sectionPlacement =
      SweepSectionPlacement::ExistingProfilePlane;
  std::optional<double> profilePathAngleCos;

  CSweep() { featureType = FeatureType::Sweep; }
};

enum class FilletMode {
  UNKNOWN = 0,
  CONSTANT_RADIUS,
  VARIABLE_RADIUS,
  FACE_FILLET,
  FULL_ROUND,
  CHORDAL
};

enum class FilletCrossSection {
  UNKNOWN = 0,
  CIRCULAR,
  CONIC,
  CURVATURE_CONTINUOUS
};

enum class FilletReferenceMode {
  UNKNOWN = 0,
  EDGE_CHAIN,
  FACE_FACE,
  FULL_ROUND_THREE_FACES
};

enum class FilletConicValueMode {
  NONE = 0,
  RHO,
  RADIUS,
  GENERIC_VALUE
};

enum class FilletDriveType {
  UNKNOWN = 0,
  RADIUS,
  SINGLE_DISTANCE,
  TWO_DISTANCES
};

struct CFilletRadiusPoint {
  double position = 0.0;
  std::optional<double> primaryValue;
  std::optional<double> secondValue;
  std::shared_ptr<CRefEdge> edgeRef;
};

struct CFilletParams {
  FilletDriveType driveType{FilletDriveType::UNKNOWN};
  std::optional<double> primaryValue;
  std::optional<double> secondValue;
  std::vector<CFilletRadiusPoint> radiusPoints;
  FilletCrossSection crossSection{FilletCrossSection::UNKNOWN};
  FilletConicValueMode conicValueMode{FilletConicValueMode::NONE};
  std::optional<double> conicValue;
  bool tangentPropagation = false;
};

struct CFillet : public CFeatureBase {
  FilletMode mode{FilletMode::UNKNOWN};
  FilletReferenceMode referenceMode{FilletReferenceMode::UNKNOWN};
  CFilletParams params;
  std::vector<std::shared_ptr<CRefEntityBase>> references;
  std::vector<std::shared_ptr<CRefFace>> side1Faces;
  std::vector<std::shared_ptr<CRefFace>> side2Faces;
  std::vector<std::shared_ptr<CRefFace>> centerFaces;
  std::optional<CPoint3D> firstEndFaceMarker;
  std::optional<std::string> swOverflowType;
  bool swKeepFeatures = false;
  std::optional<int> creoAttachType;
  std::optional<int> creoConicDepOption;

  CFillet() { featureType = FeatureType::Fillet; }
};

/**
 * @brief 统一倒角参数模式。
 *
 * 第一版 unified chamfer 仅保留跨系统共同参数语义，不直接镜像各 CAD
 * 原生 feature family 或行为控制选项。
 *
 * 当前统一规则：
 * - EQUAL_DISTANCE: 等距倒角
 * - TWO_DISTANCES: 双距离倒角
 * - TWO_OFFSETS: 双偏移倒角
 * - DISTANCE_ANGLE: 距离 + 角度倒角
 * - VERTEX_3DISTANCES: 顶点倒角，三边距离
 *
 * 其中 Creo 原生 schema 建议在读侧归一化映射为上述通用模式，例如：
 * - PRO_CHM_45_X_D -> DISTANCE_ANGLE, 且 angle = 45deg
 * - PRO_CHM_D_X_D / PRO_CHM_D1_X_D2 -> TWO_DISTANCES
 * - PRO_CHM_ANG_X_D -> DISTANCE_ANGLE
 * - PRO_CHM_O_X_O / PRO_CHM_O1_X_O2 -> TWO_OFFSETS
 */
enum class ChamferMode {
  UNKNOWN = 0,
  EQUAL_DISTANCE,    ///< 等距倒角
  TWO_DISTANCES,     ///< 双距离倒角
  TWO_OFFSETS,       ///< 双偏移倒角
  DISTANCE_ANGLE,    ///< 距离 + 角度倒角
  VERTEX_3DISTANCES  ///< 顶点倒角，三边距离
};

/**
 * @brief 统一倒角数值参数。
 *
 * 字段是否生效由 `ChamferMode` 决定：
 * - EQUAL_DISTANCE: distance1
 * - TWO_DISTANCES: distance1 + distance2
 * - TWO_OFFSETS: offset1 + offset2
 * - DISTANCE_ANGLE: distance1 + angle
 * - VERTEX_3DISTANCES: distance1 + distance2 + distance3
 */
struct CChamferParams {
  std::optional<double> distance1;
  std::optional<double> distance2;
  std::optional<double> distance3;
  std::optional<double> offset1;
  std::optional<double> offset2;
  std::optional<double> angle;
};

/**
 * @brief 倒角特征。
 *
 * 第一版统一结构只保留：
 * - `mode`：参数解释方式
 * - `params`：统一数值参数
 * - `references`：参与倒角定义的引用对象数组
 * - `firstEndFaceMarker`：用于标识倒角第一端面的几何点
 *
 * 当前不引入 reversed / flipped、tangent propagation、shape/method 等
 * 更偏 CAD 原生行为控制的字段，避免将写回补偿语义提前混入统一模型。
 */
struct CChamfer : public CFeatureBase {
  ChamferMode mode{ChamferMode::UNKNOWN};
  CChamferParams params;
  std::vector<std::shared_ptr<CRefEntityBase>> references;
  std::optional<CPoint3D> firstEndFaceMarker;

  CChamfer() { featureType = FeatureType::Chamfer; }
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
