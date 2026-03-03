#pragma once
// clang-format off
#include "UnifiedFeatures.h"
#include <cmath>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>
// clang-format on
namespace CADExchange {

/**
 * @brief 模型验证报告，包含错误和警告信息。
 *
 * 每条错误/警告均以 "[RULE_ID] 描述" 格式输出，便于日志定位。
 *
 * Rule ID 命名规范：
 *   MODEL_xxx  — 模型级（重复ID、空ID）
 *   SKETCH_xxx — 草图特征规则
 *   EXTRUDE_xxx— 拉伸特征规则
 *   REVOLVE_xxx— 旋转特征规则
 *   GEOM_xxx   — 几何合法性（向量/坐标系）
 *   REF_xxx    — 引用顺序/类型匹配
 *   SCALE_xxx  — 数值量级（单位一致性）
 */
struct ValidationReport {
  bool isValid = true;
  std::vector<std::string> errors;   ///< 阻断级：有 error 时不应保存或重建
  std::vector<std::string> warnings; ///< 警告级：记录但不阻断
};

/**
 * @brief 封装所有构建特征的容器，同时记录单位、名称等元数据。
 */
class UnifiedModel {
public:
  UnitType unit = UnitType::METER; ///< 当前使用的单位系统。
  std::string modelName;           ///< 可选的模型名称。

  UnifiedModel() = default;
  UnifiedModel(UnitType unitType, const std::string &name = "")
      : unit(unitType), modelName(name) {}

  /**
   * @brief 将给定特征注册到模型，并同步索引。
   *
   * @param feature 需要保存的特征对象，若为空则忽略。
   */
  void AddFeature(const std::shared_ptr<CFeatureBase> &feature) {
    if (!feature) {
      return;
    }
    m_features.push_back(feature);
    m_index[feature->featureID] = feature;
  }

  /**
   * @brief 根据标识符获取对应的特征对象。
   *
   * @param featureID 特征的 UUID。
   * @return 若存在则返回 shared_ptr，否则返回 nullptr。
   */
  std::shared_ptr<CFeatureBase> GetFeature(const std::string &featureID) const {
    if (auto it = m_index.find(featureID); it != m_index.end()) {
      return it->second;
    }
    return nullptr;
  }

  /**
   * @brief 根据特征名称查找特征 ID。
   *
   * @param name 特征名称。
   * @return 若找到则返回特征 ID，否则返回空字符串。
   */
  std::string GetFeatureIdByName(const std::string &name) const {
    for (const auto &feature : m_features) {
      if (feature->featureName == name) {
        return feature->featureID;
      }
    }
    return "";
  }

  /**
   * @brief  获取特征的index。
   * 
   */
  int GetFeatureIndexByID(const std::string &featureID) const {
    for (size_t i = 0; i < m_features.size(); ++i) {
      if (m_features[i]->featureID == featureID) {
        return static_cast<int>(i);
      }
    }
    return -1; // 未找到
  }

  /**
   * @brief 尝试将 ID 对应的特征安全地转换为指定类型。
   *
   * @tparam T 期望的派生类型。
   * @param featureID 需要查询的特征 ID。
   * @return 转换成功则返回特定类型的 shared_ptr，否则 nullptr。
   */
  template <typename T>
  std::shared_ptr<T> GetFeatureAs(const std::string &featureID) const {
    static_assert(std::is_base_of<CFeatureBase, T>::value,
                  "T must derive from CFeatureBase");
    auto base = GetFeature(featureID);
    if (!base) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<T>(base);
  }

  /**
   * @brief 获取当前模型持有的所有特征列表。
   *
   * @return 包含所有特征的 const 引用。
   */
  const std::vector<std::shared_ptr<CFeatureBase>> &GetFeatures() const {
    return m_features;
  }

  /**
   * @brief 非 const 访问版本，允许在外部修改特征集合。
   */
  std::vector<std::shared_ptr<CFeatureBase>> &Features() { return m_features; }

  /**
   * @brief 清空模型中的所有特征和索引。
   */
  void Clear() {
    m_features.clear();
    m_index.clear();
  }

  /**
   * @brief 验证模型完整性。
   *
   * 检查内容分为 error（阻断）和 warning（提示）两级，每条均带 RuleID。
   *
   * Error 规则（有任一 error 时 isValid=false）：
   *   MODEL_001  featureID 为空
   *   MODEL_002  featureID 重复
   *   REF_001    Extrude 引用的 profileSketchID 未在前面定义
   *   REF_002    EC 引用的 parentFeatureID 未在前面定义
   *   REF_003    Sketch referencePlane parentFeatureID 未在前面定义
   *   EXTRUDE_001 Extrude::profileSketchID 为空
   *   EXTRUDE_002 EC type=BLIND 但 depth ≤ 0
   *   EXTRUDE_003 EC type=UP_TO_FACE/UP_TO_VERTEX 但 referenceEntity 为 null
   *   EXTRUDE_004 EC type=UP_TO_FACE 但 referenceEntity 不是 CRefFace
   *   EXTRUDE_005 EC type=UP_TO_VERTEX 但 referenceEntity 不是 CRefVertex
   *   EXTRUDE_006 ThinWall 设置但 thickness ≤ 0
   *   REVOLVE_001 Revolve::profileSketchID 为空
   *   REVOLVE_002 Revolve::primaryAngle ≤ 0
   *   GEOM_001   CExtrude::direction 为零向量
   *   GEOM_002   CRefFace::normal 为零向量
   *   GEOM_003   CSketchCSys 非正交（对被引用的草图）
   *   GEOM_004   CRefAxis::direction 为零向量
   *
   * Warning 规则：
   *   SKETCH_001 草图无 segments 但被 Extrude/Revolve 引用
   *   GEOM_005   CRefFace::centroid = (0,0,0)（可能未填写）
   *   GEOM_006   CExtrude::direction 未归一化（长度偏差 > 1%）
   *   SCALE_001  BLIND depth 量级超出合理范围（换算后 < 1µm 或 > 100m）
   */
  ValidationReport Validate() const {
    ValidationReport report;

    // 收集被 Extrude/Revolve 引用的 sketchID，用于 SKETCH_001
    std::unordered_set<std::string> referencedSketchIDs;
    for (const auto &f : m_features) {
      if (auto ex = std::dynamic_pointer_cast<CExtrude>(f))
        if (!ex->profileSketchID.empty())
          referencedSketchIDs.insert(ex->profileSketchID);
      if (auto rv = std::dynamic_pointer_cast<CRevolve>(f))
        if (!rv->profileSketchID.empty())
          referencedSketchIDs.insert(rv->profileSketchID);
    }

    // depth 量级阈值（统一换算为 meter）
    auto toMeter = [&](double v) -> double {
      switch (unit) {
        case UnitType::MILLIMETER:  return v * 1e-3;
        case UnitType::CENTIMETER:  return v * 1e-2;
        case UnitType::INCH:        return v * 0.0254;
        case UnitType::FOOT:        return v * 0.3048;
        default:                    return v; // METER
      }
    };

    auto isZeroVec = [](const CVector3D &v) -> bool {
      return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z) < GeoUtils::EPSILON;
    };

    // ---- EC 检查 helper ----
    auto addError = [&](const std::string &msg) {
      report.isValid = false;
      report.errors.push_back(msg);
    };
    auto addWarn = [&](const std::string &msg) {
      report.warnings.push_back(msg);
    };

    auto checkEC = [&](const ExtrudeEndCondition &ec,
                       const std::string &extrudeID,
                       const std::string &side,
                       const std::unordered_set<std::string> &seen) {
      using ECType = ExtrudeEndCondition::Type;

      if (ec.type == ECType::BLIND && ec.depth <= 0.0) {
        addError("[EXTRUDE_002] Extrude '" + extrudeID + "' " + side +
                 " type=BLIND but depth=" + std::to_string(ec.depth) + " (must be > 0).");
      }
      if ((ec.type == ECType::UP_TO_FACE || ec.type == ECType::UP_TO_VERTEX) &&
          !ec.referenceEntity) {
        addError("[EXTRUDE_003] Extrude '" + extrudeID + "' " + side +
                 " type=" + (ec.type == ECType::UP_TO_FACE ? "UP_TO_FACE" : "UP_TO_VERTEX") +
                 " but referenceEntity is null.");
      }
      if (ec.type == ECType::UP_TO_FACE && ec.referenceEntity) {
        if (!std::dynamic_pointer_cast<CRefFace>(ec.referenceEntity)) {
          addError("[EXTRUDE_004] Extrude '" + extrudeID + "' " + side +
                   " type=UP_TO_FACE but referenceEntity is not CRefFace.");
        } else {
          auto face = std::dynamic_pointer_cast<CRefFace>(ec.referenceEntity);
          if (isZeroVec(face->normal)) {
            addError("[GEOM_002] Extrude '" + extrudeID + "' " + side +
                     " CRefFace normal is zero vector.");
          }
          CPoint3D zero{};
          if (face->centroid == zero) {
            addWarn("[GEOM_005] Extrude '" + extrudeID + "' " + side +
                    " CRefFace centroid is (0,0,0) — may be unset.");
          }
        }
      }
      if (ec.type == ECType::UP_TO_VERTEX && ec.referenceEntity) {
        if (!std::dynamic_pointer_cast<CRefVertex>(ec.referenceEntity)) {
          addError("[EXTRUDE_005] Extrude '" + extrudeID + "' " + side +
                   " type=UP_TO_VERTEX but referenceEntity is not CRefVertex.");
        }
      }
      if (ec.type == ECType::BLIND) {
        double depthM = toMeter(ec.depth);
        if (depthM > 0.0 && (depthM < 1e-6 || depthM > 100.0)) {
          addWarn("[SCALE_001] Extrude '" + extrudeID + "' " + side +
                  " BLIND depth=" + std::to_string(ec.depth) +
                  " (≈" + std::to_string(depthM * 1000.0) + "mm) is out of normal range — "
                  "check unit system.");
        }
      }
      if (ec.referenceEntity) {
        if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(ec.referenceEntity)) {
          if (!subTopo->parentFeatureID.empty() &&
              seen.find(subTopo->parentFeatureID) == seen.end()) {
            addWarn("[REF_002] Extrude '" + extrudeID + "' " + side +
                    " references feature '" + subTopo->parentFeatureID +
                    "' which has not been defined yet.");
          }
        }
      }
    };

    // ---- 主循环 ----
    std::unordered_set<std::string> seen;
    std::unordered_set<std::string> seenIDs;

    for (const auto &feature : m_features) {
      // MODEL_001
      if (feature->featureID.empty()) {
        addError("[MODEL_001] A feature has an empty featureID.");
        seen.insert("");
        continue;
      }
      // MODEL_002
      if (seenIDs.count(feature->featureID)) {
        addError("[MODEL_002] Duplicate featureID '" + feature->featureID + "'.");
      }
      seenIDs.insert(feature->featureID);

      // ---- CExtrude ----
      if (auto extrude = std::dynamic_pointer_cast<CExtrude>(feature)) {
        // EXTRUDE_001
        if (extrude->profileSketchID.empty()) {
          addError("[EXTRUDE_001] Extrude '" + extrude->featureID +
                   "' has empty profileSketchID.");
        }
        // REF_001
        else if (seen.find(extrude->profileSketchID) == seen.end()) {
          addError("[REF_001] Extrude '" + extrude->featureID +
                   "' references sketch '" + extrude->profileSketchID +
                   "' which has not been defined yet.");
        }
        // GEOM_001
        if (isZeroVec(extrude->direction)) {
          addError("[GEOM_001] Extrude '" + extrude->featureID +
                   "' direction is zero vector.");
        } else {
          double len = std::sqrt(extrude->direction.x * extrude->direction.x +
                                 extrude->direction.y * extrude->direction.y +
                                 extrude->direction.z * extrude->direction.z);
          if (std::abs(len - 1.0) > 0.01) {
            addWarn("[GEOM_006] Extrude '" + extrude->featureID +
                    "' direction length=" + std::to_string(len) +
                    " is not normalized (expected ~1.0).");
          }
        }
        checkEC(extrude->endCondition1, extrude->featureID, "EC1", seen);
        if (extrude->endCondition2)
          checkEC(*extrude->endCondition2, extrude->featureID, "EC2", seen);
        // EXTRUDE_006
        if (extrude->thinWall.has_value() && extrude->thinWall->thickness <= 0.0) {
          addError("[EXTRUDE_006] Extrude '" + extrude->featureID +
                   "' has ThinWall but Thickness=" +
                   std::to_string(extrude->thinWall->thickness) + " (must be > 0).");
        }
      }

      // ---- CSketch ----
      else if (auto sketch = std::dynamic_pointer_cast<CSketch>(feature)) {
        // SKETCH_001
        if (sketch->segments.empty() &&
            referencedSketchIDs.count(sketch->featureID)) {
          addWarn("[SKETCH_001] Sketch '" + sketch->featureID +
                  "' is referenced by Extrude/Revolve but has no segments.");
        }
        // GEOM_003
        if (referencedSketchIDs.count(sketch->featureID) &&
            !sketch->sketchCSys.IsValid()) {
          addError("[GEOM_003] Sketch '" + sketch->featureID +
                   "' sketchCSys is not orthogonal.");
        }
        // REF_003
        if (sketch->referencePlane) {
          if (auto subTopo = std::dynamic_pointer_cast<CRefSubTopo>(
                  sketch->referencePlane)) {
            if (!subTopo->parentFeatureID.empty() &&
                seen.find(subTopo->parentFeatureID) == seen.end()) {
              addWarn("[REF_003] Sketch '" + sketch->featureID +
                      "' referencePlane parent '" + subTopo->parentFeatureID +
                      "' has not been defined yet.");
            }
          }
        }
      }

      // ---- CRevolve ----
      else if (auto revolve = std::dynamic_pointer_cast<CRevolve>(feature)) {
        // REVOLVE_001
        if (revolve->profileSketchID.empty()) {
          addError("[REVOLVE_001] Revolve '" + revolve->featureID +
                   "' has empty profileSketchID.");
        }
        // REF_001 (revolve)
        else if (seen.find(revolve->profileSketchID) == seen.end()) {
          addError("[REF_001] Revolve '" + revolve->featureID +
                   "' references sketch '" + revolve->profileSketchID +
                   "' which has not been defined yet.");
        }
        // REVOLVE_002
        if (revolve->primaryAngle <= 0.0) {
          addError("[REVOLVE_002] Revolve '" + revolve->featureID +
                   "' primaryAngle=" + std::to_string(revolve->primaryAngle) +
                   " (must be > 0).");
        }
        // GEOM_004
        if (isZeroVec(revolve->axis.direction)) {
          addError("[GEOM_004] Revolve '" + revolve->featureID +
                   "' axis direction is zero vector.");
        }
      }

      seen.insert(feature->featureID);
    }

    return report;
  }

private:
  std::vector<std::shared_ptr<CFeatureBase>> m_features; ///< 特征列表
  std::unordered_map<std::string, std::shared_ptr<CFeatureBase>>
      m_index; ///< ID 索引
};

} // namespace CADExchange
