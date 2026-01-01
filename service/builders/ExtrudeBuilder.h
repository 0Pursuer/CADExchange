#pragma once
#include "../../core/TypeAdapters.h"
#include "EndConditionBuilder.h"
#include "FeatureBuilderBase.h"

#include <optional>
#include <stdexcept>


namespace CADExchange {
namespace Builder {

/**
 * @brief 拉伸构建器，封装方向、结束条件、拔模与薄壁等属性。
 *
 * 支持链式调用，例如：
 * @code
 *   ExtrudeBuilder(model, "MyExtrude")
 *       .SetProfileByName("MySketch")
 *       .SetOperation(BooleanOp::BOSS)
 *       .SetEndCondition1(EndCondition::Blind(10.0))
 *       .Build();
 * @endcode
 */
class ExtrudeBuilder : public FeatureBuilderBase<CExtrude> {
public:
  ExtrudeBuilder(UnifiedModel &model, const std::string &name)
      : FeatureBuilderBase(model, name) {}

  /**
   * @brief 通过 sketch ID 设置草图轮廓。
   *
   * @param sketchID 草图的特征 ID
   * @return 返回 *this 以支持链式调用
   * @throws std::runtime_error 当草图不存在时抛出
   */
  ExtrudeBuilder &SetProfile(const std::string &sketchID) {
    auto sketch = m_model.GetFeatureAs<CSketch>(sketchID);
    if (!sketch) {
      throw std::runtime_error("Sketch profile not found: " + sketchID);
    }
    m_feature->sketchProfile = sketch;
    return *this;
  }

  /**
   * @brief 通过 sketch 名称设置草图轮廓。
   *
   * 此方法会自动将 sketch 名称转换为 ID，然后调用 SetProfile。
   * 在处理从其他系统转换过来的特征名称时特别有用。
   *
   * @param sketchName 草图的可读名称
   * @return 返回 *this 以支持链式调用
   * @throws std::runtime_error 当草图不存在时抛出
   */
  ExtrudeBuilder &SetProfileByName(const std::string &sketchName) {
    auto sketchId = m_model.GetFeatureIdByName(sketchName);
    if (sketchId.empty() || sketchId == "UnknownSketchId") {
      throw std::runtime_error("Sketch not found by name: " + sketchName);
    }
    return SetProfile(sketchId);
  }

  /**
   * @brief 通过外部系统 ID 设置草图轮廓。
   *
   * @param externalID 外部系统的草图 ID
   * @return 返回 *this 以支持链式调用
   * @throws std::runtime_error 当草图不存在时抛出
   */
  ExtrudeBuilder &SetProfileByExternalID(const std::string &externalID) {
    auto sketch = m_model.GetFeatureByExternalIDAs<CSketch>(externalID);
    if (!sketch) {
      throw std::runtime_error("Sketch profile not found by external ID: " +
                               externalID);
    }
    m_feature->sketchProfile = sketch;
    return *this;
  }

  /**
   * @brief 设置拉伸方向向量（自动归一化）。
   *
   * 支持任意向量类型（CVector3D、std::array<double,3>、Eigen::Vector3d 等）。
   *
   * @tparam VectorT 向量类型，需要 VectorAdapter 支持
   * @param dir 方向向量
   * @return 返回 *this 以支持链式调用
   * @throws std::runtime_error 当向量长度为 0 时抛出
   */
  template <typename VectorT> ExtrudeBuilder &SetDirection(const VectorT &dir) {
    CVector3D normalized = VectorAdapter<VectorT>::Convert(dir);
    double length =
        std::sqrt(normalized.x * normalized.x + normalized.y * normalized.y +
                  normalized.z * normalized.z);
    if (length < 1e-9) {
      throw std::runtime_error("Direction vector is too small (near zero).");
    }
    normalized.Normalize();
    m_feature->direction = normalized;
    return *this;
  }

  /**
   * @brief 设置布尔运算类型（加料/减料）。
   *
   * @param op 布尔运算类型 (BOSS 加料 / CUT 减料)
   * @return 返回 *this 以支持链式调用
   */
  ExtrudeBuilder &SetOperation(BooleanOp op) {
    m_feature->operation = op;
    return *this;
  }

  /**
   * @brief 设置第一结束条件。
   *
   * 只有类型不为 UNKNOWN 的条件才会被设置。
   *
   * @param cond 结束条件
   * @return 返回 *this 以支持链式调用
   */
  ExtrudeBuilder &SetEndCondition1(const ExtrudeEndCondition &cond) {
    if (cond.type == ExtrudeEndCondition::Type::UNKNOWN) {
      return *this;
    }
    // 验证引用实体的合法性
    ValidateReference(cond.referenceEntity);

    m_feature->endCondition1 = cond;
    return *this;
  }

  /**
   * @brief 设置第二结束条件（可选，用于两个方向的拉伸）。
   *
   * 只有类型不为 UNKNOWN 的条件才会被设置。
   *
   * @param cond 结束条件
   * @return 返回 *this 以支持链式调用
   */
  ExtrudeBuilder &SetEndCondition2(const ExtrudeEndCondition &cond) {
    if (cond.type == ExtrudeEndCondition::Type::UNKNOWN) {
      return *this;
    }
    // 验证引用实体的合法性
    ValidateReference(cond.referenceEntity);

    m_feature->endCondition2 = cond;
    return *this;
  }

  /**
   * @brief 设置拔模参数。
   *
   * @param angle 拔模角度（度数），必须非负
   * @param outward 拔模方向：true 为外侧，false 为内侧
   * @return 返回 *this 以支持链式调用
   * @throws std::runtime_error 当角度为负时抛出
   */
  ExtrudeBuilder &SetDraft(double angle, bool outward = false) {
    if (angle < 0) {
      throw std::runtime_error("Draft angle must be non-negative.");
    }
    m_feature->draft = DraftOption{angle, outward};
    return *this;
  }

  /**
   * @brief 设置薄壁参数。
   *
   * @param thickness 壁厚，必须为正
   * @param isOneSided true 则为单向薄壁，false 为双向薄壁
   * @param isCovered 薄壁是否有盖
   * @return 返回 *this 以支持链式调用
   * @throws std::runtime_error 当厚度不为正时抛出
   */
  ExtrudeBuilder &SetThinWall(double thickness, bool isOneSided = true,
                              bool isCovered = false) {
    if (thickness <= 0) {
      throw std::runtime_error("Thickness must be positive.");
    }
    m_feature->thinWall = ThinWallOption{thickness, isOneSided, isCovered};
    return *this;
  }
};

} // namespace Builder
} // namespace CADExchange
