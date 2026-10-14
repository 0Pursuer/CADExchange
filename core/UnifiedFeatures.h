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
  Rib,
  Shell,
  Sketch,
  DatumPlane,
  Draft,
  LinearPattern,
  CircularPattern,
  MirrorPattern,
  Hole ///< 原生减材孔特征，由 CHole 描述孔位、深度、孔底及入口形状
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
  /**
   * @brief 源端拓扑元素的来源特征标识。
   *
   * 该字段仅用于来源追踪、日志和设计历史展示。
   * 不同 CAD 系统可能将同一几何元素归属于不同特征，
   * 因此目标端拓扑匹配不得要求 parentFeatureID 相等，
   * 也不得使用该字段限制候选边或候选面集合。
   */
  std::string parentFeatureID;
  /**
   * @deprecated Legacy topology hint kept only for backward compatibility.
   * New read/write logic should not rely on this value for reference matching.
   */
  [[deprecated("Legacy compatibility only; do not use TopologyIndex for new "
               "references.")]]
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

/**
 * @brief 拓扑面的曲面类型。
 */
enum class CGeoSurfaceType {
  UNKNOWN = 0,
  PLANE = 4001,
  CYLINDER = 4002,
  CONE = 4003,
  SPHERE = 4004,
  TORUS = 4005,
  BSURFACE = 4006
};

struct CRefFace : public CRefSubTopo {
  CVector3D normal;
  /// 面上采样点：SolidWorks 二次开发中取三角剖分 (GetTessTriangles)
  /// 第一个三角形的重心， 保证该点落在面的几何表面内，用于 SelectByRay
  /// 选面。并非该面的几何质心。
  CPoint3D centroid;
  CVector3D uDir{1, 0, 0};
  CVector3D vDir{0, 1, 0};
  CGeoSurfaceType surfaceType = CGeoSurfaceType::PLANE;

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
  CPoint3D midPoint; ///< 裁剪边参数区间中间位置，不是弦中点。
  CGeoCurveType curveType = CGeoCurveType::UNKNOWN;
  double length = 0.0;

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
  enum class SegType { LINE, CIRCLE, ARC, ELLIPSE, SPLINE, POINT } type;
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

struct CSketchEllipse : public CSketchSeg {
  CPoint3D center;
  double majorRadius = 0.0;
  double minorRadius = 0.0;
  double rotation = 0.0;    ///< 长轴相对局部X轴的旋转角 (弧度)
  double startAngle = 0.0;  ///< 起始参数角 (弧度)
  double endAngle = 0.0;    ///< 终止参数角 (弧度)
  bool isClockwise = false;
  CSketchEllipse() { type = SegType::ELLIPSE; }
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
  SketchEntity,     ///< 当前草图内部图元，由 `sketchEntityLocalID` 定位
  ExternalReference ///< 草图外部引用对象，由 `refEntity` 定位
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
  Whole,   ///< 整个实体
  Start,   ///< 起点
  End,     ///< 终点
  Center,  ///< 圆心/中心点
  Midpoint ///< 中点
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
  bool valid = false;

  bool IsValid() const {
    if (!valid)
      return false;
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
 * CHole 也复用此结构，但首版仅支持 VALUE 定深和 THROUGH_ALL 单向贯穿。
 */
struct SweepExtent {
  enum class Type {
    VALUE,       // 数值（Extrude: 深度 / Revolve: 角度，内部统一用弧度）
    SYMMETRIC,   // 对称
    THROUGH_ALL, // 贯穿
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
 * @brief 盲孔底部的几何形状。
 *
 * 与孔口的柱形沉孔 / 锥形沉头相互独立：本枚举描述主孔的末端，
 * counterbore 和 countersink 描述入口。贯穿孔没有有限孔底，本枚举不生效。
 */
enum class HoleBottomKind {
  Unknown, ///< 尚未解析孔底；盲孔必须明确孔底，贯穿孔保存时规范为此值
  Flat,    ///< 平底盲孔；没有钻尖，钻肩深度与总深度相同
  Conical  ///< 锥底盲孔；必须给出有效的 drillPointAngle 和 depthReference
};

/**
 * @brief 盲孔深度数值所对应的轴向终点。
 *
 * 起点始终是实际入口平面，本枚举只决定 CHole::extent.value 量到钻肩还是钻尖。
 * “钻肩”是主孔圆柱段与孔底锥面的交界，不是孔口沉孔的台阶。
 * 平底盲孔统一使用 Shoulder；贯穿孔不使用此枚举。
 */
enum class HoleDepthReference {
  Unknown,  ///< 深度基准未解析；锥底盲孔不能据此推测深度，贯穿保存时使用此值
  Shoulder, ///< 从实际入口到主孔圆柱段末端（钻肩）；不包含钻尖高度
  Tip       ///< 从实际入口到锥底钻尖；包含主孔圆柱段及钻尖高度
};

/**
 * @brief 一个孔位的入口坐标及可选定位依赖。
 *
 * 几何位置由 entryPoint 表达，positionRef 用于保留源模型中能重建的定位关联。
 * 首版 CHole 只允许一个孔位；列表形式为后续多孔位表达保留数据结构。
 */
struct CHoleLocation {
  /**
   * @brief 孔轴与实际入口平面的交点。
   *
   * 使用模型全局坐标系，坐标长度随模型单位变化；必须为有限值并位于
   * CHole::supportRef 对应的平面上。写入 CAD 时以此点定位孔轴，
   * 不能直接填入定位草图的局部坐标或孔口沉孔底面的坐标。
   */
  CPoint3D entryPoint;

  /**
   * @brief 可选的孔位定位引用，空指针表示仅用 entryPoint 定位。
   *
   * 复用 CRef* 引用体系保存点、顶点、草图段等源定位依赖，供目标桥接解析
   * 并建立原生关联；引用不能替代 entryPoint，也不承载完整的草图约束树。
   * 非空引用必须有效，其目标须为标准引用或模型中先于本孔定义的特征。
   * 目标系统能否保留该关联由桥接能力决定，不能仅凭此字段保证无损重建。
   */
  std::shared_ptr<CRefEntityBase> positionRef;
};

/**
 * @brief 孔口柱形沉孔参数，描述用于容纳螺钉头等的同轴大径圆柱凹部。
 *
 * 仅在 CHole::counterbore 有值时生效，不能与锥形沉头同时启用。
 * 两个长度都使用模型单位；盲孔的主孔有效圆柱长度为钻肩深度减去 depth。
 */
struct CHoleCounterbore {
  /**
   * @brief 孔口沉孔的大径，而非主孔径。
   *
   * 必须为有限正值且严格大于 CHole::diameter，用于重建入口的大径圆柱段。
   */
  double diameter = 0.0;

  /**
   * @brief 从实际入口沿孔的向内方向量到沉孔台阶的轴向深度。
   *
   * 必须为有限正值；对盲孔还须严格小于钻肩深度，以保留正长度的主孔圆柱段。
   * 主孔深度也从实际入口起算，不能把此值作为主孔深度的起点偏移。
   */
  double depth = 0.0;
};

/**
 * @brief 孔口锥形沉头参数，描述入口大径到主孔径之间的同轴锥面。
 *
 * 仅在 CHole::countersink 有值时生效，不能与柱形沉孔同时启用。
 * 不另存锥面轴向高度，而由大径 D、主孔径 d、完整夹角 alpha 计算：
 * @f$ h_{csink} = (D-d)/(2\tan(\alpha/2)) @f$。
 * 对盲孔，该高度须严格小于钻肩深度，以保留正长度的主孔圆柱段。
 */
struct CHoleCountersink {
  /**
   * @brief 实际入口平面处沉头锥面的最大直径，使用模型长度单位。
   *
   * 必须为有限正值且严格大于 CHole::diameter；与 includedAngle 一起决定
   * 沉头锥面的轴向高度及其与主孔圆柱的交界位置。
   */
  double diameter = 0.0;

  /**
   * @brief 沉头锥面的完整夹角，单位为弧度。
   *
   * 为锥面轴向截面中两条母线的夹角，不是母线与孔轴的半角；
   * 必须为有限值并满足 0 < includedAngle < pi。
   * 例如 90 度沉头应保存为 pi/2，而不是 90 或 pi/4。
   */
  double includedAngle = 0.0;
};

/**
 * @brief 跨 CAD 系统统一的原生减材孔特征。
 *
 * 通过主孔径、终止条件和孔底描述普通直孔，并可选择一种入口扩孔：
 * 柱形沉孔或锥形沉头。继承 CFeatureBase 的特征 ID、名称及抑制状态；
 * 孔固定为减材语义，不需要外部轮廓草图或额外的布尔操作字段。
 *
 * 所有孔位使用模型全局坐标系，所有长度使用模型单位，所有角度为弧度及
 * 完整夹角。所有轴向深度均从实际入口平面沿 direction 向材料内部量起，
 * 即使有沉孔 / 沉头，也不能改从入口扩孔的底部起算主孔深度。
 *
 * @note 首版限定单孔位、单实体、平面入口、孔轴垂直入口以及单向定深或贯穿。
 *       默认构造不会产生有效孔，仍需填写活动几何参数并执行模型校验。
 * @see CHoleLocation, CHoleCounterbore, CHoleCountersink, ResolveHoleGeometry
 */
struct CHole : CFeatureBase {
  /**
   * @brief 孔入口的支撑引用，用于选择原生放置平面并保留特征依赖。
   *
   * 必须能解析为 CRefPlane 基准平面或 surfaceType 为 PLANE 的 CRefFace，
   * 并与 locations 中的入口点共面、与 direction 垂直。
   * 引用目标须为标准基准或模型中先于本孔定义的特征；不得为空。
   */
  std::shared_ptr<CRefEntityBase> supportRef;

  /**
   * @brief 从实际入口指向材料内部的孔轴单位向量，使用模型全局坐标系。
   *
   * 分量必须有限，向量长度为 1，并垂直于入口平面；本字段无长度单位，
   * 单位转换时不缩放。平面法向可能与其同向或反向，不能直接代替向内方向。
   * 默认零向量无效，调用者必须明确提供实际钻孔方向。
   */
  CVector3D direction{};

  /**
   * @brief 孔位列表；每项保存入口点及可选定位引用。
   *
   * 所有孔位共用本特征的孔径、孔轴、终止条件、孔底和入口参数。
   * 首版必须恰好包含一项，空列表和多孔位均不支持，不能静默省略孔位。
   */
  std::vector<CHoleLocation> locations;

  /**
   * @brief 主孔圆柱段的直径，使用模型长度单位。
   *
   * 必须为有限正值；不包含沉孔 / 沉头的大径。
   * 锥底盲孔还用此值与 drillPointAngle 共同计算钻尖高度。
   */
  double diameter = 0.0;

  /**
   * @brief 沿 direction 的单向终止条件，复用共享 SweepExtent 结构。
   *
   * - VALUE：盲孔，value 为有限正深度，长度随模型单位变化；
   *   深度终点由 depthReference 决定，起点始终为实际入口平面。
   * - THROUGH_ALL：保持原生贯穿语义，value 不生效，不能用猜测长度替代贯穿。
   *
   * 首版不支持其它终止类型。hasOffset、isFlip、isFlipMaterialSide 须为 false，
   * offset 须为有限的 0，referenceEntity 和 helperPoint 须为空；
   * 孔轴方向只由 direction 表达，不再通过 extent 翻转。
   */
  SweepExtent extent;

  /**
   * @brief 盲孔 extent.value 的深度基准，用于区分到钻肩与到钻尖的尺寸标注。
   *
   * 对锥底盲孔须明确为 Shoulder 或 Tip，未知基准不能用默认值猜测。
   * 钻尖高度 h_tip = diameter / (2 * tan(drillPointAngle / 2))，
   * 到尖深度 = 到肩深度 + h_tip；同一几何可采用这两种等价深度标注。
   * 平底盲孔规范为 Shoulder，贯穿孔此字段不生效，保存时规范为 Unknown。
   */
  HoleDepthReference depthReference = HoleDepthReference::Unknown;

  /**
   * @brief 主孔的孔底形状，仅 VALUE 盲孔生效。
   *
   * Flat 表示平底，Conical 表示带钻尖的锥底；盲孔不能为 Unknown。
   * 贯穿孔不使用孔底参数，保存时规范为 Unknown；入口沉头由 countersink 表达。
   */
  HoleBottomKind bottom = HoleBottomKind::Unknown;

  /**
   * @brief 锥底盲孔的钻尖完整夹角，单位为弧度。
   *
   * 仅 extent.type 为 VALUE 且 bottom 为 Conical 时必须有值并生效，
   * 有效范围为有限的 0 < drillPointAngle < pi，用于重建孔底及换算肩 / 尖深度。
   * 平底或贯穿孔忽略此字段，保存时清空；它与孔口沉头的 includedAngle 独立。
   */
  std::optional<double> drillPointAngle;

  /**
   * @brief 可选柱形沉孔入口；有值时按大径和轴向深度重建入口台阶。
   *
   * 普通直孔为空；定深和贯穿主孔均可携带此入口描述。
   * 与 countersink 互斥，盲孔须保证沉孔台阶之后仍有正长度的主孔圆柱段。
   */
  std::optional<CHoleCounterbore> counterbore;

  /**
   * @brief 可选锥形沉头入口；有值时按大径和完整夹角重建入口锥面。
   *
   * 普通直孔为空；定深和贯穿主孔均可携带此入口描述。
   * 与 counterbore 互斥；由 CHoleCountersink 的公式推导入口高度，
   * 盲孔须保证锥面之后仍有正长度的主孔圆柱段。
   */
  std::optional<CHoleCountersink> countersink;

  /** @brief 初始化特征类型为 Hole；不自动补齐孔的几何参数或执行有效性校验。 */
  CHole() { featureType = FeatureType::Hole; }
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
  CVector3D
      direction; ///< 统一语义：特征正旋转方向；ReferenceEntity 保留原始几何方向
};

/**
 * @brief 旋转特征。
 */
struct CRevolve : public CProfiledFeatureBase {
  CRevolveAxis axis;
  SweepExtent extent1;                ///< Revolve 角度值统一使用弧度
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

enum class FilletConicValueMode { NONE = 0, RHO, RADIUS, GENERIC_VALUE };

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
  std::optional<CPoint3D> edgeMidPoint;
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
  EQUAL_DISTANCE,   ///< 等距倒角
  TWO_DISTANCES,    ///< 双距离倒角
  TWO_OFFSETS,      ///< 双偏移倒角
  DISTANCE_ANGLE,   ///< 距离 + 角度倒角
  VERTEX_3DISTANCES ///< 顶点倒角，三边距离
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

// 厚度情况
struct RibThicknessOption {
  bool symmetric = true;              // 是否对称
  double thickness = 0.0;             // 厚度值
  std::optional<CVector3D> direction; // 如果不对称，提供具体方向值
};

// 材料情况
struct RibMaterialOption {
  CVector3D direction; // 材料方向
  CPoint3D
      referencePoint; // 标记面，方向值是该点处的法向相反方向，辅助确定材料方向
};

// Rib特征定义
struct CRib : public CFeatureBase {
  std::string sketchID;               // 草图ID
  RibThicknessOption thicknessOption; // 厚度情况
  RibMaterialOption materialOption;   // 材料情况

  CRib() { featureType = FeatureType::Rib; }
};

/**
 * @brief 壳特征（抽壳）的厚度方向模式。
 *
 * 与方向取反 (Flip) 不同：这是壳特征自身的厚度偏置语义。
 */
enum class ShellThicknessDirection {
  Unknown = 0,
  Inward, ///< 内侧抽壳：向实体内部偏移厚度
  Outward ///< 外侧抽壳：向实体外部偏移厚度
};

/**
 * @brief 壳特征中具有独立厚度的面。
 *
 * 例如 Creo `PRO_E_ST_SHELL_LOCL_LIST` 中的单条多厚度记录，
 * 或 SW `IShellFeatureData` 中 `MultipleThicknessFaces` 与
 * `GetMultipleThicknessAtIndex` 的组合。
 */
struct CShellThicknessFace {
  std::shared_ptr<CRefFace> face; ///< 具有独立厚度的面引用
  double thickness = 0.0;         ///< 该面的独立厚度值（跟随模型单位）
};

/**
 * @brief 壳特征（抽壳）。
 *
 * 壳特征属于非草图驱动的面操作类特征，直接引用拓扑面（`CRefFace`），
 * 与 `CFillet` / `CChamfer` 同属一类（继承 `CFeatureBase`，不继承
 * `CProfiledFeatureBase`）。
 *
 * 核心参数（跨系统共同）：
 * - `thickness` + `direction`：默认壳壁厚度与偏移方向
 * - `facesToRemove`：被移除的面（开口面）
 * - `thicknessFaces`：具有非默认厚度的面列表
 *
 * 系统专有扩展（可选）：
 * - `targetBody`：目标实体引用（Creo `PRO_E_BODY_SELECT`）
 * - `excludedFaces`：排除曲面集合（Creo `PRO_E_STD_SURF_COLLECTION_APPL`）
 */
struct CShell : public CFeatureBase {
  // ---- 核心参数（跨系统共同） ----
  double thickness = 0.0; ///< 默认壳壁厚度
  ShellThicknessDirection direction{
      ShellThicknessDirection::Unknown}; ///< 抽壳方向

  std::vector<std::shared_ptr<CRefFace>>
      facesToRemove;                               ///< 被移除的面（开口面）
  std::vector<CShellThicknessFace> thicknessFaces; ///< 非默认厚度的面

  // ---- 系统专有扩展（Phase-1 可选） ----
  std::shared_ptr<CRefEntityBase>
      targetBody; ///< 目标实体引用（Creo: PRO_E_BODY_SELECT）
  std::vector<std::shared_ptr<CRefFace>>
      excludedFaces; ///< 排除曲面（Creo: PRO_E_STD_SURF_COLLECTION_APPL）

  CShell() { featureType = FeatureType::Shell; }
};

/**
 * @brief 拔模类型枚举（最小标准结构）。
 */
enum class DraftType {
  Unknown = 0,
  NeutralPlane, ///< 中性面拔模 (SW: Neutral Plane, Creo: Hinge == PLANE)
  PartingLine   ///< 分型线拔模 (SW: Parting Line, Creo: Hinge == CURVE)
};

/**
 * @brief 拔模特征 (CDraft)。
 *
 * 仅包含 SolidWorks 和 Creo
 * 均能原生、无损支持的几何特征参数（通用最小标准结构）。
 */
struct CDraft : public CFeatureBase {
  DraftType draftType{DraftType::Unknown}; ///< 拔模类型

  std::shared_ptr<CRefEntityBase> pullDirectionRef; ///< 拔模/拉伸开模方向参考
  bool reversePullDirection = false;                ///< 方向是否反向

  std::vector<std::shared_ptr<CRefFace>> draftFaces; ///< 被拔模的目标面集合

  std::shared_ptr<CRefEntityBase>
      neutralPlaneRef; ///< 中性面参考 (用于中性面拔模)
  std::vector<std::shared_ptr<CRefEntityBase>>
      partingLines; ///< 分型线参考 (用于分型线/台阶拔模)

  double draftAngle = 0.0; ///< 第一侧/主侧拔模角度 (弧度制)

  bool isTwoSided = false;      ///< 是否为双侧拔模
  double draftAngleSide2 = 0.0; ///< 第二侧拔模角度 (弧度制)

  std::shared_ptr<CRefEntityBase> partingSplitSketchRef;
  std::vector<std::shared_ptr<CRefFace>> partingSplitTargetFaces;
  bool partingSplitSingleDirection = false;
  bool partingSplitReverseDirection = false;

  CDraft() { featureType = FeatureType::Draft; }
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
  // Optional geometry supplement extracted from source CAD. Absence means the
  // source path did not provide stable plane geometry; zero values remain
  // valid.
  std::optional<CPoint3D> projectedOrigin;
  std::optional<CVector3D> normal;

  CDatumPlane() { featureType = FeatureType::DatumPlane; }
};

/**
 * @brief 阵列作用域：特征、拓扑面、还是整个实体
 */
enum class PatternScope {
  FEATURES, ///< 针对特征列表阵列
  FACES,    ///< 针对指定的一组面阵列
  BODIES    ///< 针对多实体环境下的体阵列
};

/**
 * @brief 阵列间距/分布模式
 */
enum class PatternSpacingType {
  PITCH_AND_COUNT, // 给定 数量(Count) + 间距(Pitch)
  SPAN_AND_COUNT,  // 给定 数量(Count) + 总跨度(Span)
};

/**
 * @brief 网格坐标，用于精确标记需要跳过（抑制）的阵列实例
 */
struct CPatternIndex {
  int dir1Index = 0; // 方向 1 索引（从0开始）
  int dir2Index = 0; // 方向 2 索引（如果不启用方向2，则为0）

  bool operator==(const CPatternIndex &other) const {
    return dir1Index == other.dir1Index && dir2Index == other.dir2Index;
  }
};

/**
 * @brief 线性阵列的方向控制参数
 */
struct CLinearPatternDir {
  std::shared_ptr<CRefEntityBase>
      directionRef;                   ///< 方向参考（边、轴、面法线等）
  CVector3D direction{0.0, 0.0, 1.0}; ///< 绝对方向物理向量 (归一化)

  PatternSpacingType spacingType = PatternSpacingType::PITCH_AND_COUNT;
  double spacing =
      0.0;       ///< 线性间距值（按模型单位，SW提取时需从米转为模型单位）
  int count = 1; ///< 包含源特征在内的实例总数
};

/**
 * @brief 统一线性阵列特征
 */
struct CLinearPattern : public CFeatureBase {
  CLinearPatternDir dir1;
  std::optional<CLinearPatternDir> dir2;

  bool patternSeedOnly =
      false; ///< 方向二是否仅对“源特征”进行阵列（而不阵列方向1产生的拷贝）

  PatternScope scope = PatternScope::FEATURES;
  std::vector<std::shared_ptr<CRefEntityBase>>
      seedObjects; ///< 特征、面、或实体的引用列表

  std::vector<CPatternIndex> skippedInstances; ///< 跳过的阵列实例索引列表
  bool geometryPattern = false;                ///< 是否启用几何阵列（加速复制）

  CLinearPattern() { featureType = FeatureType::LinearPattern; }
};

/**
 * @brief 圆周阵列的旋转方向参数
 */
struct CCircularPatternDir {
  std::shared_ptr<CRefEntityBase>
      axisRef; ///< 旋转轴参考（基准轴、线性边、圆柱面等）
  CVector3D direction{0.0, 0.0, 1.0}; ///< 绝对旋转轴物理向量 (归一化)
  CPoint3D origin{0.0, 0.0, 0.0};     ///< 旋转轴原点（绝对坐标）

  PatternSpacingType spacingType = PatternSpacingType::PITCH_AND_COUNT;
  double angle = 0.0; ///< 旋转角（统一使用弧度制，Creo和UG的度需转换）
  int count = 1;      ///< 实例数量
};

/**
 * @brief 统一圆周阵列特征
 */
struct CCircularPattern : public CFeatureBase {
  CCircularPatternDir dir1;
  std::optional<CLinearPatternDir> dir2; // 径向线性平移扩展（同心圆环阵列）
  bool patternSeedOnly = false;

  PatternScope scope = PatternScope::FEATURES;
  std::vector<std::shared_ptr<CRefEntityBase>> seedObjects;

  std::vector<CPatternIndex> skippedInstances;
  bool geometryPattern = false;

  CCircularPattern() { featureType = FeatureType::CircularPattern; }
};

/**
 * @brief 统一镜像特征
 */
struct CMirrorPattern : public CFeatureBase {
  std::shared_ptr<CRefEntityBase>
      mirrorPlaneRef; ///< 镜像平面（基准面、实体平面）

  PatternScope scope = PatternScope::FEATURES;
  std::vector<std::shared_ptr<CRefEntityBase>> seedObjects;

  bool geometryPattern = false;

  CMirrorPattern() { featureType = FeatureType::MirrorPattern; }
};

} // namespace CADExchange
