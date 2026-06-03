#pragma once

#include "../../thirdParty/cereal/archives/xml.hpp"
#include "../../thirdParty/cereal/types/base_class.hpp"
#include "../../thirdParty/cereal/types/memory.hpp"
#include "../../thirdParty/cereal/types/optional.hpp"
#include "../../thirdParty/cereal/types/polymorphic.hpp"
#include "../../thirdParty/cereal/types/string.hpp"
#include "../../thirdParty/cereal/types/vector.hpp"

#include "../../core/UnifiedFeatures.h"
#include "../../core/UnifiedModel.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

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
 *
 * TopologyIndex 仅作为历史兼容字段保留，不应再作为新的引用判定键。
 */
template <class Archive> void serialize(Archive &ar, CRefSubTopo &ref) {
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
  ar(cereal::base_class<CRefEntityBase>(&ref),
     cereal::make_nvp("ParentFeature", ref.parentFeatureID),
     cereal::make_nvp("TopologyIndex", ref.topologyIndex));
#ifdef _MSC_VER
#pragma warning(pop)
#endif
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
 * @brief 序列化边界的端点与中点。
 */
template <class Archive> void serialize(Archive &ar, CRefEdge &edge) {
  ar(cereal::base_class<CRefSubTopo>(&edge),
     cereal::make_nvp("StartPoint", edge.startPoint),
     cereal::make_nvp("EndPoint", edge.endPoint),
     cereal::make_nvp("MidPoint", edge.midPoint),
     cereal::make_nvp("CurveType", edge.curveType));
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
void serialize(Archive &ar, SketchConstraintRef &ref) {
  ar(cereal::make_nvp("Kind", ref.kind),
     cereal::make_nvp("SubEntity", ref.subEntity),
     cereal::make_nvp("SketchEntityLocalID", ref.sketchEntityLocalID),
     cereal::make_nvp("ReferenceEntity", ref.refEntity));
}

template <class Archive>
void serialize(Archive &ar, CSketchConstraint &constraint) {
  ar(cereal::make_nvp("Type", constraint.type),
     cereal::make_nvp("Refs", constraint.refs),
     cereal::make_nvp("Value", constraint.value));
}

/**
 * @brief 序列化草图局部坐标系。
 */
template <class Archive> void serialize(Archive &ar, CSketchCSys &csys) {
  ar(cereal::make_nvp("Origin", csys.origin),
     cereal::make_nvp("XDir", csys.xDir), cereal::make_nvp("YDir", csys.yDir),
     cereal::make_nvp("ZDir", csys.zDir));
}

/**
 * @brief 序列化几何调试输出中的辅助基准面记录。
 */
template <class Archive> void serialize(Archive &ar, CGeoDatumPlane &plane) {
  ar(cereal::make_nvp("TargetFeatureID", plane.targetFeatureID),
     cereal::make_nvp("Type", plane.type),
     cereal::make_nvp("LocalCSys", plane.localCSys));
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
  ar(cereal::make_nvp("StartOffset", thinWall.startOffset),
     cereal::make_nvp("EndOffset", thinWall.endOffset),
     cereal::make_nvp("Covered", thinWall.isCovered));
}

/**
 * @brief 序列化共享轮廓特征基类的公共字段。
 */
template <class Archive>
void serialize(Archive &ar, CProfiledFeatureBase &feat) {
  ar(cereal::base_class<CFeatureBase>(&feat),
     cereal::make_nvp("ProfileSketchID", feat.profileSketchID),
     cereal::make_nvp("Operation", feat.operation),
     cereal::make_nvp("ThinWall", feat.thinWall));
}

/**
 * @brief 序列化共享扫掠终止条件。
 */
template <class Archive> void serialize(Archive &ar, SweepExtent &extent) {
  ar(cereal::make_nvp("Type", extent.type),
     cereal::make_nvp("Value", extent.value),
     cereal::make_nvp("Offset", extent.offset),
     cereal::make_nvp("HasOffset", extent.hasOffset),
     cereal::make_nvp("Reference", extent.referenceEntity),
     cereal::make_nvp("Flip", extent.isFlip),
     cereal::make_nvp("FlipMaterialSide", extent.isFlipMaterialSide),
     cereal::make_nvp("HelperPoint", extent.helperPoint));
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
     cereal::make_nvp("CSys", sk.sketchCSys),
     cereal::make_nvp("Segments", sk.segments),
     cereal::make_nvp("Constraints", sk.constraints));
}

/**
 * @brief 序列化挤出特征，包括草图侧面、方向、操作与附加选项。
 */
template <class Archive> void serialize(Archive &ar, CExtrude &ext) {
  ar(cereal::base_class<CProfiledFeatureBase>(&ext),
     cereal::make_nvp("Direction", ext.direction),
     cereal::make_nvp("Extent1", ext.extent1),
     cereal::make_nvp("Extent2", ext.extent2),
     cereal::make_nvp("Draft", ext.draft));
}

/**
 * @brief 序列化旋转轴信息，包括参照与方向。
 */
template <class Archive> void serialize(Archive &ar, CRevolveAxis &axis) {
  ar(cereal::make_nvp("ReferenceLocalID", axis.referenceLocalID),
     cereal::make_nvp("ReferenceEntity", axis.referenceEntity),
     cereal::make_nvp("Origin", axis.origin),
     cereal::make_nvp("Direction", axis.direction));
}

/**
 * @brief 序列化旋转特征的轮廓、角度与轴信息。
 */
template <class Archive> void serialize(Archive &ar, CRevolve &rev) {
  ar(cereal::base_class<CProfiledFeatureBase>(&rev),
     cereal::make_nvp("Axis", rev.axis),
     cereal::make_nvp("Extent1", rev.extent1),
     cereal::make_nvp("Extent2", rev.extent2));
}

/**
 * @brief Serialize a sweep path reference chain.
 */
template <class Archive> void serialize(Archive &ar, CSweepPath &path) {
  ar(cereal::make_nvp("References", path.references),
     cereal::make_nvp("StartPoint", path.startPoint),
     cereal::make_nvp("EndPoint", path.endPoint),
     cereal::make_nvp("Closed", path.isClosed));
}

template <class Archive>
void serialize(Archive &ar, CSweepCircularProfile &profile) {
  ar(cereal::make_nvp("OuterRadius", profile.outerRadius),
     cereal::make_nvp("InnerRadius", profile.innerRadius));
}

template <class Archive>
void serialize(Archive &ar, CSweepEmbeddedProfile &profile) {
  ar(cereal::make_nvp("Sketch", profile.sketch));
}

template <class Archive> void serialize(Archive &ar, CSweepProfile &profile) {
  ar(cereal::make_nvp("Kind", profile.kind),
     cereal::make_nvp("SketchID", profile.sketchID),
     cereal::make_nvp("Embedded", profile.embedded),
     cereal::make_nvp("Circular", profile.circular));
}

/**
 * @brief Serialize a sweep feature.
 */
template <class Archive> void serialize(Archive &ar, CSweep &sweep) {
  ar(cereal::base_class<CProfiledFeatureBase>(&sweep),
     cereal::make_nvp("Profile", sweep.profile),
     cereal::make_nvp("Path", sweep.path),
     cereal::make_nvp("GuidePaths", sweep.guidePaths),
     cereal::make_nvp("Orientation", sweep.orientation),
     cereal::make_nvp("SectionPlacement", sweep.sectionPlacement),
     cereal::make_nvp("ProfilePathAngleCos", sweep.profilePathAngleCos));
}

template <class Archive> void serialize(Archive &ar, CFilletRadiusPoint &point) {
  ar(cereal::make_nvp("Position", point.position),
     cereal::make_nvp("PrimaryValue", point.primaryValue),
     cereal::make_nvp("SecondValue", point.secondValue),
     cereal::make_nvp("EdgeMidPoint", point.edgeMidPoint));
}

template <class Archive> void serialize(Archive &ar, CFilletParams &params) {
  ar(cereal::make_nvp("DriveType", params.driveType),
     cereal::make_nvp("PrimaryValue", params.primaryValue),
     cereal::make_nvp("SecondValue", params.secondValue),
     cereal::make_nvp("RadiusPoints", params.radiusPoints),
     cereal::make_nvp("CrossSection", params.crossSection),
     cereal::make_nvp("ConicValueMode", params.conicValueMode),
     cereal::make_nvp("ConicValue", params.conicValue),
     cereal::make_nvp("TangentPropagation", params.tangentPropagation));
}

template <class Archive> void serialize(Archive &ar, CFillet &fillet) {
  ar(cereal::base_class<CFeatureBase>(&fillet),
     cereal::make_nvp("Mode", fillet.mode),
     cereal::make_nvp("ReferenceMode", fillet.referenceMode),
     cereal::make_nvp("Params", fillet.params),
     cereal::make_nvp("References", fillet.references),
     cereal::make_nvp("Side1Faces", fillet.side1Faces),
     cereal::make_nvp("Side2Faces", fillet.side2Faces),
     cereal::make_nvp("CenterFaces", fillet.centerFaces),
     cereal::make_nvp("FirstEndFaceMarker", fillet.firstEndFaceMarker),
     cereal::make_nvp("SwOverflowType", fillet.swOverflowType),
     cereal::make_nvp("SwKeepFeatures", fillet.swKeepFeatures),
     cereal::make_nvp("CreoAttachType", fillet.creoAttachType),
     cereal::make_nvp("CreoConicDepOption", fillet.creoConicDepOption));
}

/**
 * @brief 序列化倒角参数。
 */
template <class Archive> void serialize(Archive &ar, CChamferParams &params) {
  ar(cereal::make_nvp("Distance1", params.distance1),
     cereal::make_nvp("Distance2", params.distance2),
     cereal::make_nvp("Distance3", params.distance3),
     cereal::make_nvp("Offset1", params.offset1),
     cereal::make_nvp("Offset2", params.offset2),
     cereal::make_nvp("Angle", params.angle));
}

/**
 * @brief 序列化倒角特征。
 */
template <class Archive> void serialize(Archive &ar, CChamfer &chamfer) {
  ar(cereal::base_class<CFeatureBase>(&chamfer),
     cereal::make_nvp("Mode", chamfer.mode),
     cereal::make_nvp("Params", chamfer.params),
     cereal::make_nvp("References", chamfer.references),
     cereal::make_nvp("FirstEndFaceMarker", chamfer.firstEndFaceMarker));
}

template <class Archive>
void serialize(Archive &ar, RibThicknessOption &opt) {
  ar(cereal::make_nvp("Symmetric", opt.symmetric),
     cereal::make_nvp("Thickness", opt.thickness),
     cereal::make_nvp("Direction", opt.direction));
}

template <class Archive>
void serialize(Archive &ar, RibMaterialOption &opt) {
  ar(cereal::make_nvp("Direction", opt.direction),
     cereal::make_nvp("ReferencePoint", opt.referencePoint));
}

template <class Archive> void serialize(Archive &ar, CRib &rib) {
  ar(cereal::base_class<CFeatureBase>(&rib),
     cereal::make_nvp("SketchID", rib.sketchID),
     cereal::make_nvp("ThicknessOption", rib.thicknessOption),
     cereal::make_nvp("MaterialOption", rib.materialOption));
}

/**
 * @brief 序列化基准面约束，记录约束类型、引用索引和参数。
 */
template <class Archive>
void serialize(Archive &ar, PlaneConstraint &constraint) {
  ar(cereal::make_nvp("Type", constraint.type),
     cereal::make_nvp("Ref", constraint.ref),
     cereal::make_nvp("Value", constraint.value),
     cereal::make_nvp("DefaultDir", constraint.defaultDir),
     cereal::make_nvp("Reversed", constraint.reversed));
}

/**
 * @brief 序列化基准面特征。
 */
template <class Archive> void serialize(Archive &ar, CDatumPlane &datumPlane) {
  ar(cereal::base_class<CFeatureBase>(&datumPlane),
     cereal::make_nvp("Method", datumPlane.method),
     cereal::make_nvp("Constraints", datumPlane.constraints),
     cereal::make_nvp("ReferenceEntities", datumPlane.referenceEntities));
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

/**
 * @brief 反序列化 UnifiedModel，重建单位、名称与特征集合。
 */
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

#ifdef _MSC_VER
#pragma warning(pop)
#endif
