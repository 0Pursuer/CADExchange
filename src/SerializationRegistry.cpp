#include "../include/UnifiedSerialization.h"
#include "../thirdParty/cereal/archives/json.hpp"
#include "../thirdParty/cereal/types/polymorphic.hpp"

using namespace CADExchange;

/**
 * @file SerializationRegistry.cpp
 * @brief 注册所有多态类型，使 Cereal 在序列化/反序列化阶段能识别具体派生类。
 */

// ==========================================
// 引用系统注册
// ==========================================
/**
 * @brief 引用系统相关类及其继承关系。
 */
CEREAL_REGISTER_TYPE(CRefPlane)
CEREAL_REGISTER_TYPE(CRefFace)
CEREAL_REGISTER_TYPE(CRefEdge)
CEREAL_REGISTER_TYPE(CRefVertex)
CEREAL_REGISTER_TYPE(CRefSketch)
CEREAL_REGISTER_TYPE(CRefSketchSeg)
CEREAL_REGISTER_TYPE(CRefFeature)
CEREAL_REGISTER_TYPE(CRefSubTopo)

CEREAL_REGISTER_POLYMORPHIC_RELATION(CRefEntityBase, CRefPlane)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CRefEntityBase, CRefSketch)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CRefEntityBase, CRefFace)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CRefEntityBase, CRefEdge)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CRefEntityBase, CRefVertex)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CRefEntityBase, CRefSketchSeg)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CRefEntityBase, CRefFeature)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CRefEntityBase, CRefSubTopo)

// ==========================================
// 草图元素注册
// ==========================================
/**
 * @brief 草图几何段及其派生类型。
 */
CEREAL_REGISTER_TYPE(CSketchLine)
CEREAL_REGISTER_TYPE(CSketchCircle)
CEREAL_REGISTER_TYPE(CSketchArc)
CEREAL_REGISTER_TYPE(CSketchPoint)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CSketchSeg, CSketchLine)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CSketchSeg, CSketchCircle)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CSketchSeg, CSketchArc)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CSketchSeg, CSketchPoint)

// ==========================================
// 特征系统注册
// ==========================================
/**
 * @brief 特征系统的多态类注册。
 */
CEREAL_REGISTER_TYPE(CSketch)
CEREAL_REGISTER_TYPE(CExtrude)
CEREAL_REGISTER_TYPE(CRevolve)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CFeatureBase, CSketch)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CFeatureBase, CExtrude)
CEREAL_REGISTER_POLYMORPHIC_RELATION(CFeatureBase, CRevolve)

namespace CADExchange {
void RegisterSerializationTypes() {}
} // namespace CADExchange
