#pragma once

#include "../thirdParty/cereal/archives/xml.hpp"
#include "../thirdParty/cereal/types/base_class.hpp"
#include "../thirdParty/cereal/types/memory.hpp"
#include "../thirdParty/cereal/types/optional.hpp"
#include "../thirdParty/cereal/types/polymorphic.hpp"
#include "../thirdParty/cereal/types/string.hpp"
#include "../thirdParty/cereal/types/vector.hpp"

#include "../core/UnifiedFeatures.h"
#include "../core/UnifiedModel.h"

/**
 * @file UnifiedSerialization.h
 * @brief 通过 Cereal 提供 CAD 统一模型的 XML 序列化规则。
 */

namespace CADExchange {

// ==========================================
// 基础几何类型
// ==========================================
/**
 * @brief 序列化三维点坐标，独立记录 x/y/z 分量。
 */
template <class Archive> void serialize(Archive &ar, CPoint3D &p) {
  ar(cereal::make_nvp("x", p.x), cereal::make_nvp("y", p.y),
     cereal::make_nvp("z", p.z));
}

/**
 * @brief 序列化三维向量，保留方向分量以重建方向。
 */
template <class Archive> void serialize(Archive &ar, CVector3D &v) {
  ar(cereal::make_nvp("x", v.x), cereal::make_nvp("y", v.y),
     cereal::make_nvp("z", v.z));
}

// ==========================================
// 引用系统序列化 (多态)
// ==========================================
/**
 * @brief 序列化引用基类，仅记录引用类型枚举。
 */
template <class Archive> void serialize(Archive &ar, CRefEntityBase &ref) {
  ar(cereal::make_nvp("RefType", ref.refType));
}

/**
 * @brief 序列化特征引用，保存目标特征 ID。
 */
template <class Archive> void serialize(Archive &ar, CRefFeature &ref) {
  ar(cereal::base_class<CRefEntityBase>(&ref),
     cereal::make_nvp("TargetFeatureID", ref.targetFeatureID));
}

/**
 * @brief 序列化子拓扑引用，持有父特征 ID 与索引。
 */
template <class Archive> void serialize(Archive &ar, CRefSubTopo &ref) {
  ar(cereal::base_class<CRefEntityBase>(&ref),
     cereal::make_nvp("ParentFeature", ref.parentFeatureID),
     cereal::make_nvp("TopologyIndex", ref.topologyIndex));
}

/**
 * @brief 序列化参考平面位置与方向。
 */
template <class Archive> void serialize(Archive &ar, CRefPlane &plane) {
  ar(cereal::base_class<CRefFeature>(&plane),
     cereal::make_nvp("Origin", plane.origin),
     cereal::make_nvp("XDir", plane.xDir), cereal::make_nvp("YDir", plane.yDir),
     cereal::make_nvp("Normal", plane.normal));
}

/**
 * @brief 序列化草图引用，扩展特征引用。
 */
template <class Archive> void serialize(Archive &ar, CRefSketch &sketch) {
  ar(cereal::base_class<CRefFeature>(&sketch));
}

/**
 * @brief 序列化面拓扑的法线与中心。
 */
template <class Archive> void serialize(Archive &ar, CRefFace &face) {
  ar(cereal::base_class<CRefSubTopo>(&face),
     cereal::make_nvp("Normal", face.normal),
     cereal::make_nvp("Centroid", face.centroid));
}

/**
 * @brief 序列化边界的中点。
 */
template <class Archive> void serialize(Archive &ar, CRefEdge &edge) {
  ar(cereal::base_class<CRefSubTopo>(&edge),
     cereal::make_nvp("MidPoint", edge.midPoint));
}

/**
 * @brief 序列化顶点位置。
 */
template <class Archive> void serialize(Archive &ar, CRefVertex &vertex) {
  ar(cereal::base_class<CRefSubTopo>(&vertex),
     cereal::make_nvp("Position", vertex.pos));
}

/**
 * @brief 序列化草图段引用的本地 ID。
 */
template <class Archive>
void serialize(Archive &ar, CRefSketchSeg &segmentRef) {
  ar(cereal::base_class<CRefSubTopo>(&segmentRef),
     cereal::make_nvp("SegmentLocalID", segmentRef.segmentLocalID));
}

// ==========================================
// 草图几何序列化
// ==========================================
/**
 * @brief 序列化草图线段的公共字段。
 */
template <class Archive> void serialize(Archive &ar, CSketchSeg &seg) {
  ar(cereal::make_nvp("LocalID", seg.localID),
     cereal::make_nvp("Construction", seg.isConstruction));
}

/**
 * @brief 序列化线段的起始/结束点。
 */
template <class Archive> void serialize(Archive &ar, CSketchLine &line) {
  ar(cereal::base_class<CSketchSeg>(&line),
     cereal::make_nvp("Start", line.startPos),
     cereal::make_nvp("End", line.endPos));
}

/**
 * @brief 序列化圆，记录圆心与半径。
 */
template <class Archive> void serialize(Archive &ar, CSketchCircle &circle) {
  ar(cereal::base_class<CSketchSeg>(&circle),
     cereal::make_nvp("Center", circle.center),
     cereal::make_nvp("Radius", circle.radius));
}

/**
 * @brief 序列化圆弧，包括起止角度与顺时针标识。
 */
template <class Archive> void serialize(Archive &ar, CSketchArc &arc) {
  ar(cereal::base_class<CSketchSeg>(&arc),
     cereal::make_nvp("Center", arc.center),
     cereal::make_nvp("Radius", arc.radius),
     cereal::make_nvp("StartAngle", arc.startAngle),
     cereal::make_nvp("EndAngle", arc.endAngle),
     cereal::make_nvp("Clockwise", arc.isClockwise));
}

/**
 * @brief 序列化点草图段，记录位置。
 */
template <class Archive> void serialize(Archive &ar, CSketchPoint &point) {
  ar(cereal::base_class<CSketchSeg>(&point),
     cereal::make_nvp("Position", point.position));
}

/**
 * @brief 序列化草图约束，包含约束类型、参与实体和尺寸值。
 */
template <class Archive>
void serialize(Archive &ar, CSketchConstraint &constraint) {
  ar(cereal::make_nvp("Type", constraint.type),
     cereal::make_nvp("Entities", constraint.entityLocalIDs),
     cereal::make_nvp("Dimension", constraint.dimensionValue));
}

// ==========================================
// 特征系统序列化
// ==========================================
/**
 * @brief 序列化拔模选项，保留角度与方向。
 */
template <class Archive> void serialize(Archive &ar, DraftOption &draft) {
  ar(cereal::make_nvp("Angle", draft.angle),
     cereal::make_nvp("Outward", draft.outward));
}

/**
 * @brief 序列化薄壁选项，记录厚度与覆盖状态。
 */
template <class Archive> void serialize(Archive &ar, ThinWallOption &thinWall) {
  ar(cereal::make_nvp("Thickness", thinWall.thickness),
     cereal::make_nvp("OneSided", thinWall.isOneSided),
     cereal::make_nvp("Covered", thinWall.isCovered));
}

/**
 * @brief 序列化挤出端条件，包括深度、偏移及参照。
 */
template <class Archive>
void serialize(Archive &ar, ExtrudeEndCondition &cond) {
  ar(cereal::make_nvp("Type", cond.type), cereal::make_nvp("Depth", cond.depth),
     cereal::make_nvp("Offset", cond.offset),
     cereal::make_nvp("HasOffset", cond.hasOffset),
     cereal::make_nvp("Reference", cond.referenceEntity),
     cereal::make_nvp("Flip", cond.isFlip),
     cereal::make_nvp("FlipMaterialSide", cond.isFlipMaterialSide));
}

/**
 * @brief 序列化特征基类，共享 ID/名称/抑制状态。
 */
template <class Archive> void serialize(Archive &ar, CFeatureBase &feat) {
  ar(cereal::make_nvp("ID", feat.featureID),
     cereal::make_nvp("Name", feat.featureName),
     cereal::make_nvp("Suppressed", feat.isSuppressed));
}

/**
 * @brief 序列化草图特征的引用平面、段和约束列表。
 */
template <class Archive> void serialize(Archive &ar, CSketch &sk) {
  ar(cereal::base_class<CFeatureBase>(&sk),
     cereal::make_nvp("Plane", sk.referencePlane),
     cereal::make_nvp("Segments", sk.segments),
     cereal::make_nvp("Constraints", sk.constraints));
}

/**
 * @brief 序列化挤出特征，包括草图侧面、方向、操作与附加选项。
 */
template <class Archive> void serialize(Archive &ar, CExtrude &ext) {
  ar(cereal::base_class<CFeatureBase>(&ext),
     cereal::make_nvp("Profile", ext.sketchProfile),
     cereal::make_nvp("Direction", ext.direction),
     cereal::make_nvp("Operation", ext.operation),
     cereal::make_nvp("EndCondition1", ext.endCondition1),
     cereal::make_nvp("EndCondition2", ext.endCondition2),
     cereal::make_nvp("Draft", ext.draft),
     cereal::make_nvp("ThinWall", ext.thinWall));
}

/**
 * @brief 序列化旋转轴信息，包括参照与方向。
 */
template <class Archive> void serialize(Archive &ar, CRevolveAxis &axis) {
  ar(cereal::make_nvp("Kind", axis.kind),
     cereal::make_nvp("ReferenceLocalID", axis.referenceLocalID),
     cereal::make_nvp("ReferenceEntity", axis.referenceEntity),
     cereal::make_nvp("Origin", axis.origin),
     cereal::make_nvp("Direction", axis.direction));
}

/**
 * @brief 序列化旋转特征的轮廓、角度与轴信息。
 */
template <class Archive> void serialize(Archive &ar, CRevolve &rev) {
  ar(cereal::base_class<CFeatureBase>(&rev),
     cereal::make_nvp("ProfileSketchID", rev.profileSketchID),
     cereal::make_nvp("Axis", rev.axis),
     cereal::make_nvp("AngleKind", rev.angleKind),
     cereal::make_nvp("PrimaryAngle", rev.primaryAngle),
     cereal::make_nvp("SecondaryAngle", rev.secondaryAngle));
}

// ==========================================
// 顶层模型序列化
// ==========================================
/**
 * @brief 序列化 UnifiedModel，包含单位、名称与特征树。
 *        拆分为 save/load 以支持扁平化结构。
 */
template <class Archive> void save(Archive &ar, const UnifiedModel &model) {
  ar(cereal::make_nvp("UnitSystem", model.unit),
     cereal::make_nvp("ModelName", model.modelName));

  // 记录特征数量
  size_t count = model.GetFeatures().size();
  ar(cereal::make_nvp("FeatureCount", count));

  // 扁平化输出特征
  for (const auto &feat : model.GetFeatures()) {
    ar(cereal::make_nvp("Feature", feat));
  }
}

template <class Archive> void load(Archive &ar, UnifiedModel &model) {
  ar(cereal::make_nvp("UnitSystem", model.unit),
     cereal::make_nvp("ModelName", model.modelName));

  size_t count = 0;
  ar(cereal::make_nvp("FeatureCount", count));

  model.Clear();
  for (size_t i = 0; i < count; ++i) {
    std::shared_ptr<CFeatureBase> feat;
    ar(cereal::make_nvp("Feature", feat));
    model.AddFeature(feat);
  }
}

} // namespace CADExchange
