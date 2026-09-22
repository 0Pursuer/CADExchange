#pragma once

#include "StepCompare.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <string>

namespace cadstep {
namespace detail {

/**
 * @file ShapeAudit.h
 * @brief 逐实体几何测量与形状统计的基础工具。
 *
 * 本模块是包围盒、子形状计数、曲面/曲线类型名与逐实体几何摘要的唯一出处，
 * 被归一化流程、描述符匹配器与 VTP 可视化写出器共同消费。
 */

/**
 * @brief 计算形状的紧致轴对齐包围盒。
 * @param shape 待测形状
 * @return 包围盒；两种方法都得到空盒时返回 isVoid=true 的空包围盒
 *
 * 先用 BRepBndLib::AddOptimal 取紧致盒，结果为空则退回 BRepBndLib::Add
 * 的粗盒（带控制点外扩），仍为空才判定为 void。
 */
Bounds3 ComputeBounds(const TopoDS_Shape &shape);

/**
 * @brief 在形状索引图中查找目标形状的下标。
 * @param shapeMap 待查的索引图（下标从 1 开始）
 * @param target 目标形状
 * @return 按 TopoDS_Shape::IsSame 判等命中的下标（1-based）；找不到返回 0
 *
 * @note 调用方以返回值 >0 判定存在，0 同时充当"未找到"哨兵。
 */
int FindShapeIndex(const TopTools_IndexedMapOfShape &shapeMap,
                   const TopoDS_Shape &target);

/**
 * @brief 数给定类型子形状的**出现次数**。
 * @param shape 待统计的形状
 * @param type 子形状类型（TopAbs_FACE、TopAbs_EDGE 等）
 * @return 用 TopExp_Explorer 遍历到的出现次数（不去重）
 */
int CountSubShapes(const TopoDS_Shape &shape, TopAbs_ShapeEnum type);

/**
 * @brief 数给定类型子形状的**唯一数量**。
 * @param shape 待统计的形状
 * @param type 子形状类型（TopAbs_FACE、TopAbs_EDGE 等）
 * @return 用 TopExp::MapShapes 去重后的数量
 *
 * @note 与 CountSubShapes 的差别：拓扑共享（如相邻面共享同一条边）会让
 *       "出现次数"大于"唯一数"。统计面数等语义计数时应使用本函数。
 */
int CountUniqueSubShapes(const TopoDS_Shape &shape, TopAbs_ShapeEnum type);

/**
 * @brief 把 OCCT 曲面类型枚举翻成写入 JSON 审计与 VTP CellData 的类型名。
 * @param type OCCT 曲面类型枚举
 * @return "PLANE"/"CYLINDER"/"CONE"/"SPHERE"/"TORUS"/"BEZIER"/"BSPLINE"/
 *         "REVOLUTION"/"EXTRUSION"/"OFFSET"/"OTHER"；无法识别时返回 "UNKNOWN"
 *
 * @warning 返回的字符串属于输出契约：VTP 侧的 SurfaceTypeCode 按同一套
 *          字符串映射整数编码，改动会破坏可视化拾取。
 */
std::string SurfaceTypeName(GeomAbs_SurfaceType type);

/**
 * @brief 把 OCCT 曲线类型枚举翻成写入 JSON 审计与 VTP CellData 的类型名。
 * @param type OCCT 曲线类型枚举
 * @return "LINE"/"CIRCLE"/"ELLIPSE"/"HYPERBOLA"/"PARABOLA"/"BEZIER"/
 *         "BSPLINE"/"OFFSET"/"OTHER"；无法识别时返回 "UNKNOWN"
 *
 * @warning 返回的字符串属于输出契约：VTP 侧的 CurveTypeCode 按同一套
 *          字符串映射整数编码，改动会破坏可视化拾取。
 */
std::string CurveTypeName(GeomAbs_CurveType type);

/**
 * @brief 单个面的逐实体几何摘要。
 *
 * 这些量是审计明细、描述符匹配与 VTP 写出共用的唯一真源；重构前同样的
 * 计算散落在 5 处（同域归一化的两个分支、面/边收集函数与描述符收集器），
 * 相互之间会产生漂移。
 */
struct FaceGeometrySummary {
  std::string surfaceType; ///< SurfaceTypeName() 产出的曲面类型名
  double areaMm2 = 0.0;    ///< 表面积（BRepGProp::SurfaceProperties 的 Mass）
  Point3 centroidMm;       ///< 面的质心（毫米）
  Bounds3 boundsMm;        ///< 面的包围盒（毫米）
};

/**
 * @brief 单条边的逐实体几何摘要。字段含义与 FaceGeometrySummary 类似。
 */
struct EdgeGeometrySummary {
  std::string curveType; ///< CurveTypeName() 产出的曲线类型名
  double lengthMm = 0.0; ///< 边长（BRepGProp::LinearProperties 的 Mass）
  Point3 centroidMm;     ///< 边的质心（毫米）
  Bounds3 boundsMm;      ///< 边的包围盒（毫米）
  bool closed = false;   ///< BRep_Tool::IsClosed 判定的封闭性
};

/**
 * @brief 汇总单个面的几何摘要。
 * @param face 待测量的面（供 BRepGProp 与包围盒计算使用）
 * @param surface 调用方构建好的曲面适配器
 * @return 面几何摘要
 *
 * @note 适配器由调用方传入而非内部构建：调用点通常已持有适配器，且同域
 *       归一化流程还要用同一个适配器读取圆柱半径与轴线。
 */
FaceGeometrySummary SummarizeFace(const TopoDS_Face &face,
                                  const BRepAdaptor_Surface &surface);

/**
 * @brief 汇总单条边的几何摘要。
 * @param edge 待测量的边
 * @param curve 调用方构建好的曲线适配器
 * @return 边几何摘要
 *
 * @note 与 SummarizeFace 的差别：边额外计算 BRep_Tool::IsClosed 的封闭标志。
 */
EdgeGeometrySummary SummarizeEdge(const TopoDS_Edge &edge,
                                  const BRepAdaptor_Curve &curve);

} // namespace detail
} // namespace cadstep
