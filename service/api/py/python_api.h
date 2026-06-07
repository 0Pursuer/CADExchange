#pragma once

#include "../../accessors/ChamferAccessor.h"
#include "../../accessors/DatumPlaneAccessor.h"
#include "../../accessors/ExtrudeAccessor.h"
#include "../../accessors/FeatureAccessorBase.h"
#include "../../accessors/ModelAccessor.h"
#include "../../accessors/ReferenceAccessor.h"
#include "../../accessors/RevolveAccessor.h"
#include "../../accessors/SketchAccessor.h"
#include "../../serialization/CADSerializer.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace CADExchange::PythonApi {

inline Accessor::ModelAccessor LoadModelAccessor(const std::string &path) {
  UnifiedModel model;
  std::string error;
  if (!LoadModel(model, path, &error, SerializationFormat::TINYXML)) {
    throw std::runtime_error(error.empty() ? "Failed to load model." : error);
  }
  Accessor::ModelAccessor accessor;
  accessor.SetModel(model);
  return accessor;
}

inline std::vector<Accessor::FeatureAccessorBase>
GetAllFeatures(const Accessor::ModelAccessor &modelAccessor) {
  std::vector<Accessor::FeatureAccessorBase> features;
  for (const auto &feature : modelAccessor.GetAllFeatures()) {
    if (feature) {
      features.emplace_back(*feature);
    }
  }
  return features;
}

inline Accessor::FeatureAccessorBase
GetFeature(const Accessor::ModelAccessor &modelAccessor, int index) {
  auto feature = modelAccessor.GetFeature(index);
  return Accessor::FeatureAccessorBase(feature ? feature->GetRaw() : nullptr);
}

inline Accessor::FeatureAccessorBase
GetFeatureById(const Accessor::ModelAccessor &modelAccessor,
               const std::string &featureID) {
  auto feature = modelAccessor.GetFeatureByID(featureID);
  return Accessor::FeatureAccessorBase(feature ? feature->GetRaw() : nullptr);
}

inline std::vector<double> PointToVector(const CPoint3D &point) {
  return {point.x, point.y, point.z};
}

inline std::vector<double> VectorToVector(const CVector3D &vector) {
  return {vector.x, vector.y, vector.z};
}

inline std::vector<double> GetSketchOrigin(const Accessor::SketchAccessor &sketch) {
  CPoint3D origin;
  CVector3D xDir;
  CVector3D yDir;
  CVector3D zDir;
  if (!sketch.GetCSys(origin, xDir, yDir, zDir)) {
    return {};
  }
  return PointToVector(origin);
}

inline std::vector<double> GetSketchXAxis(const Accessor::SketchAccessor &sketch) {
  CPoint3D origin;
  CVector3D xDir;
  CVector3D yDir;
  CVector3D zDir;
  if (!sketch.GetCSys(origin, xDir, yDir, zDir)) {
    return {};
  }
  return VectorToVector(xDir);
}

inline std::vector<double> GetSketchYAxis(const Accessor::SketchAccessor &sketch) {
  CPoint3D origin;
  CVector3D xDir;
  CVector3D yDir;
  CVector3D zDir;
  if (!sketch.GetCSys(origin, xDir, yDir, zDir)) {
    return {};
  }
  return VectorToVector(yDir);
}

inline std::vector<double> GetSketchZAxis(const Accessor::SketchAccessor &sketch) {
  CPoint3D origin;
  CVector3D xDir;
  CVector3D yDir;
  CVector3D zDir;
  if (!sketch.GetCSys(origin, xDir, yDir, zDir)) {
    return {};
  }
  return VectorToVector(zDir);
}

inline std::vector<double>
GetReferencePointOrEmpty(const Accessor::ReferenceAccessor &ref) {
  CPoint3D point;
  if (ref.GetPointPosition(point) || ref.GetVertexPosition(point) ||
      ref.GetPlaneOrigin(point) || ref.GetAxisOrigin(point) ||
      ref.GetFaceCentroid(point) || ref.GetEdgeMidPoint(point)) {
    return PointToVector(point);
  }
  return {};
}

inline std::vector<double>
GetReferenceDirectionOrEmpty(const Accessor::ReferenceAccessor &ref) {
  CVector3D dir;
  if (ref.GetPlaneNormal(dir) || ref.GetAxisDirection(dir) ||
      ref.GetFaceNormal(dir)) {
    return VectorToVector(dir);
  }
  return {};
}

} // namespace CADExchange::PythonApi
