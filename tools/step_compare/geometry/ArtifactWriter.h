#pragma once

#include "StepCompare.h"

#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <filesystem>
#include <vector>

namespace cadstep {
namespace detail {

/**
 * @brief 把形状导出为二进制 STL。
 * @param shape 待导出的形状
 * @param stlPath 目标文件路径
 * @return 是否成功；shape 为 null 或写盘异常时返回 false
 *
 * 网格偏差按包围盒对角线自适应（对角线的 0.1%，下限 0.001mm）。
 */
bool ExportShapeStl(const TopoDS_Shape &shape, const std::filesystem::path &stlPath);

/**
 * @brief 把形状导出为 OCCT BREP 文件。
 * @param shape 待导出的形状
 * @param brepPath 目标文件路径
 * @return 是否成功；shape 为 null 或写盘异常时返回 false
 */
bool ExportShapeBrep(const TopoDS_Shape &shape, const std::filesystem::path &brepPath);

/**
 * @brief 把归一化实体导出为面网格 VTP，供 GuiApp 可视化与拾取联动。
 * @param solid 待三角化的实体
 * @param normalizedFaces 实体的面索引图（决定 entity_index 的取值）
 * @param faceInfos 归一化面的审计明细
 * @param faceMatches 面匹配集合
 * @param side 实体侧别（参考侧/候选侧）
 * @param outputPath 目标文件路径
 * @return 是否成功；无有效三角网格或文件打不开时返回 false
 *
 * @note 审计明细与匹配结果写入 PolyData 的 CellData（列名含 entity_index、
 *       match_status_code、geometry_type_code、source_count 等）。CellData 里
 *       的实体下标与 result.json 中的实体下标一致，GuiApp 靠它做拾取联动。
 */
bool ExportFacesVtp(const TopoDS_Shape &solid,
                    const TopTools_IndexedMapOfShape &normalizedFaces,
                    const std::vector<NormalizedFaceInfo> &faceInfos,
                    const MatchCollection &faceMatches,
                    EntitySide side,
                    const std::filesystem::path &outputPath);

/**
 * @brief 把归一化实体导出为边折线 VTP，供 GuiApp 可视化与拾取联动。
 * @param solid 供采样上下文使用的实体
 * @param normalizedEdges 实体的边索引图（决定 entity_index 的取值）
 * @param edgeInfos 归一化边的审计明细
 * @param edgeMatches 边匹配集合
 * @param side 实体侧别（参考侧/候选侧）
 * @param outputPath 目标文件路径
 * @return 是否成功；无有效采样折线或文件打不开时返回 false
 *
 * @note 与面 writer 一样把审计明细与匹配结果写进 CellData；边额外携带
 *       comparison_role_code 与 comparable 列。
 */
bool ExportEdgesVtp(const TopoDS_Shape &solid,
                    const TopTools_IndexedMapOfShape &normalizedEdges,
                    const std::vector<NormalizedEdgeInfo> &edgeInfos,
                    const MatchCollection &edgeMatches,
                    EntitySide side,
                    const std::filesystem::path &outputPath);

/**
 * @brief 清理输出目录里上一轮比较遗留的所有工件文件。
 * @param outputDirectory 输出目录
 */
void RemoveOldArtifacts(const std::filesystem::path &outputDirectory);


} // namespace detail
} // namespace cadstep
