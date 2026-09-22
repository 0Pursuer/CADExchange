#pragma once

#include "StepCompare.h"
#include "domain/ComparisonModel.h"

#include <TopoDS_Solid.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace cadstep {
namespace detail {

/**
 * @brief 原始（归一化前）实体的拓扑索引。
 *
 * 供归一化流程把原拓扑映射回归一化拓扑使用：先按 IsSame 在映射里查找，
 * 再用下标取 faceIds / edgeIds 中对应的实体 id。
 */
struct OriginalTopologyIndex {
  TopTools_IndexedMapOfShape faces;   ///< 原实体的面索引图（1-based）
  TopTools_IndexedMapOfShape edges;   ///< 原实体的边索引图（1-based）
  std::vector<std::string> faceIds;   ///< 与 faces 下标一一对应的实体 id
  std::vector<std::string> edgeIds;   ///< 与 edges 下标一一对应的实体 id
};

/**
 * @brief 归一化流程的内部产物：实体、审计明细与匹配输入。
 */
struct NormalizedSolidInternal {
  TopoDS_Solid solid;                                ///< 归一化（或回退后的原）实体
  NormalizationAudit audit;                          ///< 本次归一化的完整审计明细
  TopTools_IndexedMapOfShape normalizedFaces;        ///< 结果实体的面索引图
  TopTools_IndexedMapOfShape normalizedEdges;        ///< 结果实体的边索引图

  // 匹配输入，与 audit.faces / audit.edges 出自同一次 SummarizeFace /
  // SummarizeEdge 几何测量，不会二次测量。单独持有是因为归一化关闭时
  // audit.faces 刻意保持为空，而匹配仍然需要全部面的描述符。
  std::vector<DescriptorView> faceDescriptors;             ///< 全部面的描述符
  std::vector<DescriptorView> comparableEdgeDescriptors;   ///< 仅 comparable 边的描述符

  // 组装描述符本身耗时。这只是对上述明细的一次投影（在关闭归一化的路径上
  // 还包括该路径没有审计记录的面几何测量）；逐实体几何只测一次，计入归一化耗时。
  double descriptorBuildMs = 0.0;
};

/**
 * @brief 对输入实体执行同域归一化（本模块核心入口）。
 * @param input 归一化前的原始实体
 * @param originalIndex 由 BuildOriginalTopologyIndex() 预构建的原拓扑索引
 * @param side 实体侧别（参考侧/候选侧），用于生成实体 id
 * @param config 比较配置（提供开关、线性/角度容差与体积相对容差）
 * @return 归一化结果，含实体、审计明细、面/边索引图与匹配描述符
 *
 * 行为要点：
 *  - `config.enableSameDomainNormalization == false` 时不做归一化，直接返回
 *    原实体；此时 audit.faces **保持为空**（既有怪癖，故意的），但
 *    faceDescriptors 仍会填满全部面供匹配使用；边的 sourceCount=0、merged=false
 *    （另一条既有怪癖）。
 *  - 启用时走 ShapeUpgrade_UnifySameDomain，并产出 BRepTools_History 把原拓扑
 *    映射回归一化拓扑；无法映射且未被删除的原边会置空对应 mappingComplete 标志。
 *  - 失败回退：结果不是单一 SOLID、或体积漂移超过 `relativeVolumeTolerance`，
 *    都回退为原实体并把 audit.succeeded=false，原因写入 audit.warning。
 *  - 描述符与审计明细出自同一次 SummarizeFace/SummarizeEdge 调用。
 *  - comparableIndex 只发给 comparable 的边，编号必须连续（1,2,3…），
 *    不可比的边为 0。
 */
NormalizedSolidInternal NormalizeSameDomain(const TopoDS_Solid &input,
                                            const OriginalTopologyIndex &originalIndex,
                                            EntitySide side,
                                            const CompareConfig &config);

/**
 * @brief 为原实体建立面/边索引图及对应实体 id。
 * @param solid 待索引的原始实体
 * @param side 实体侧别，用于生成实体 id（OriginalFace/OriginalEdge 种类）
 * @return 索引结构，faces/edges 与 faceIds/edgeIds 按下标一一对应
 */
OriginalTopologyIndex BuildOriginalTopologyIndex(const TopoDS_Solid &solid, EntitySide side);

/**
 * @brief 把 (类型名 → (出现次数, 度量累计)) 的统计映射转成 TypeStatistics 列表。
 * @param countMap 面/边类型统计映射（度量为面积累计或长度累计）
 * @return 按映射顺序（类型名字典序）排列的统计列表
 */
std::vector<TypeStatistics> SummarizeMap(
    const std::map<std::string, std::pair<int, double>> &countMap);


} // namespace detail
} // namespace cadstep
