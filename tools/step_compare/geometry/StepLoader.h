#pragma once

#include "StepCompare.h"

#include <TopoDS_Solid.hxx>

#include <filesystem>
#include <string>
#include <vector>

namespace cadstep {
namespace detail {

/**
 * @brief STEP 文件加载后的分类结论。
 */
enum class LoadClass {
  Ready,         ///< 加载成功，全部实体通过 BRepCheck 校验且配置允许
  Invalid,       ///< 文件不存在、解析失败、无几何、实体校验失败或实体提取失败
  Unsupported,   ///< 有几何但本工具不支持（无 3D 实体、多实体被配置禁止、含非实体拓扑）
  InternalError, ///< 默认初值；加载流程未产出任何分类结论时保持该值
};

/**
 * @brief 从 STEP 文件里提取出的单个实体及其输入审计。
 */
struct LoadedSolidItem {
  int index = 0;        ///< 实体在文件中的出现序号（0-based）
  std::string id;       ///< 实体 id，形如 "ref_solid_0" / "cand_solid_1"
  InputAudit audit;     ///< 该实体自身的输入审计（体积、质心、包围盒等）
  TopoDS_Solid solid;   ///< 提取出的实体
};

/**
 * @brief 一次 STEP 加载的完整产物：分类结论、复合审计与全部实体。
 */
struct LoadedStepModel {
  LoadClass classification = LoadClass::InternalError; ///< 加载分类结论
  InputAudit compositeAudit; ///< 复合实体的输入审计：把多个实体聚合（体积加权
                             ///< 质心、并集包围盒）；单实体时就是该实体自身
  std::vector<LoadedSolidItem> solids; ///< 文件内逐个实体的加载结果
  std::string reason;                  ///< 分类不是 Ready 时的人类可读原因
};

/**
 * @brief 读取 STEP 文件并逐个审计其中包含的实体。
 * @param stepPath STEP 文件路径
 * @param config 比较配置（决定多实体是否被允许）
 * @param side 实体侧别（参考侧/候选侧），用于生成实体 id
 * @return 加载结果；复合审计对多实体做聚合（体积加权质心、并集包围盒），
 *         单实体时就是该实体自身
 *
 * 加载过程中逐个实体做 BRepCheck_Analyzer 有效性检查等，并据此决定
 * LoadClass；任一实体校验失败即整体判 Invalid。
 */
LoadedStepModel LoadStepModel(const std::filesystem::path &stepPath,
                              const CompareConfig &config,
                              EntitySide side);


} // namespace detail
} // namespace cadstep
