#include "CADSerializer.h"
#include "UnifiedFeatures.h"
#include "UnifiedModel.h"
#include "UnifiedTypes.h"
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace CADExchange;

// =========================================================
// Mock SW API
// 模拟 SolidWorks COM 接口调用，用于展示写入流程
// =========================================================
namespace MockSW {
void SelectByID(const std::string &name, const std::string &type) {
  std::cout << "  [SW API] SelectByID2(\"" << name << "\", \"" << type
            << "\", ...)" << std::endl;
}
void InsertSketch() {
  std::cout << "  [SW API] InsertSketch(TRUE) - Toggle Sketch Mode"
            << std::endl;
}
void CreateLine(const CPoint3D &p1, const CPoint3D &p2) {
  std::cout << "  [SW API] CreateLine(" << p1.x << "," << p1.y << "," << p1.z
            << ", " << p2.x << "," << p2.y << "," << p2.z << ")" << std::endl;
}
void CreateCircle(const CPoint3D &center, double radius) {
  std::cout << "  [SW API] CreateCircleByRadius(" << center.x << "," << center.y
            << "," << center.z << ", " << radius << ")" << std::endl;
}
void FeatureExtrusion(double depth, bool isCut, bool isBlind) {
  std::cout << "  [SW API] FeatureExtrusion(Depth=" << depth
            << ", IsCut=" << (isCut ? "True" : "False")
            << ", Type=" << (isBlind ? "Blind" : "Other") << ")" << std::endl;
}
} // namespace MockSW

/**
 * @brief 根据 UnifiedModel 中的引用数据，派生出 SolidWorks 中的平面选择名称。
 *
 * SwRead/src/SketchInfo.cpp 中的 CreateSketchFeature 先解析
 * CSketch::referencePlane 中的
 * CRefFeature，判断是否是标准平面（XY/YZ/ZX），否则会使用实体或几何引用。
 * 这里假设 UnifiedModel 会通过 targetFeatureID 保存标准平面引用。
 */
static std::string
ResolveReferencePlane(const std::shared_ptr<CRefEntityBase> &ref) {
  if (!ref)
    return "UnknownPlane";

  if (auto plane = std::dynamic_pointer_cast<CRefPlane>(ref)) {
    if (plane->targetFeatureID == StandardID::PLANE_XY)
      return "Front Plane";
    if (plane->targetFeatureID == StandardID::PLANE_YZ)
      return "Right Plane";
    if (plane->targetFeatureID == StandardID::PLANE_ZX)
      return "Top Plane";
  }

  if (auto feature = std::dynamic_pointer_cast<CRefFeature>(ref)) {
    return feature->targetFeatureID.empty() ? "UnknownPlane"
                                            : feature->targetFeatureID;
  }

  return "UnknownPlane";
}

/**
 * @brief 模拟将 UnifiedModel 写入 SolidWorks 的过程 (Creation)
 *
 * 参考 SwRead 项目中的:
 * - CSketchInfo::CreateSketchFeature (SwRead/src/SketchInfo.cpp)
 * - ExtrudeInfo::CreateExtrudeFeature (SwRead/src/ExtrudeInfo.cpp)
 *
 * 此函数遍历 UnifiedModel 的特征树，并模拟调用 SW API 重建模型。
 */
void SimulateWriteToSolidWorks(const UnifiedModel &model) {
  std::cout << "[SwWrite Simulation] Starting to write part to SolidWorks..."
            << std::endl;

  const auto &features = model.GetFeatures();
  for (const auto &feature : features) {

    // ---------------------------------------------------------
    // 处理草图特征
    // 对应 CSketchInfo::CreateSketchFeature
    // ---------------------------------------------------------
    if (auto sketch = std::dynamic_pointer_cast<CSketch>(feature)) {
      std::cout << "[SwWrite Simulation] Creating Sketch: "
                << sketch->featureName << std::endl;

      // 1. 选择参考平面
      // SwRead 中的 CreateSketchFeature 会读取 referencePlane 的标准 ID
      // 或实体引用 UnifiedModel 通过 CRefFeature::targetFeatureID/Name
      // 保存这些信息
      std::string planeName = ResolveReferencePlane(sketch->referencePlane);
      MockSW::SelectByID(planeName, "PLANE");

      // 2. 进入草图模式
      MockSW::InsertSketch();

      // 3. 创建草图段
      for (const auto &seg : sketch->segments) {
        if (auto line = std::dynamic_pointer_cast<CSketchLine>(seg)) {
          MockSW::CreateLine(line->startPos, line->endPos);
        } else if (auto circle =
                       std::dynamic_pointer_cast<CSketchCircle>(seg)) {
          MockSW::CreateCircle(circle->center, circle->radius);
        }
        // ... 其他类型 (Arc, Spline)
      }

      // 4. 添加约束 (略)
      // CSketchInfo::AddConstraint(sketchRelationMgr)

      // 5. 退出草图
      MockSW::InsertSketch();
      std::cout << "[SwWrite Simulation] Sketch created." << std::endl;
    }

    // ---------------------------------------------------------
    // 处理拉伸特征
    // 对应 ExtrudeInfo::CreateExtrudeFeature
    // ---------------------------------------------------------
    else if (auto extrude = std::dynamic_pointer_cast<CExtrude>(feature)) {
      std::cout << "[SwWrite Simulation] Creating Extrude: "
                << extrude->featureName << std::endl;

      // 1. 选择轮廓草图
      std::string sketchName = "UnknownSketch";
      if (extrude->sketchProfile) {
        sketchName = extrude->sketchProfile->featureName;
      }

      MockSW::SelectByID(sketchName, "SKETCH");

      // 2. 准备拉伸参数
      double depth = extrude->endCondition1.depth;
      bool isCut = (extrude->operation == BooleanOp::CUT);
      bool isBlind =
          (extrude->endCondition1.type == ExtrudeEndCondition::Type::BLIND);

      // 3. 执行拉伸命令
      MockSW::FeatureExtrusion(depth, isCut, isBlind);

      std::cout << "[SwWrite Simulation] Extrude created." << std::endl;
    }
  }

  std::cout << "[SwWrite Simulation] Part writing completed." << std::endl;
}

int main() {
  // 尝试加载 ReadPartTest 生成的文件
  UnifiedModel model;
  std::string inputPath = "SimulationPart.xml";
  std::string error;

  if (LoadModel(model, inputPath, &error)) {
    std::cout << "Loaded model from " << inputPath << std::endl;
  } else {
    std::cout << "Could not load " << inputPath << " (" << error
              << "). Creating dummy model." << std::endl;

    // 如果文件不存在，手动创建一个简单的模型用于测试
    auto sketch = std::make_shared<CSketch>();
    sketch->featureID = "SK-DUMMY";
    sketch->featureName = "DummySketch";
    auto dummyPlane = std::make_shared<CRefPlane>();
    dummyPlane->targetFeatureID = StandardID::PLANE_XY;
    sketch->referencePlane = dummyPlane;

    auto line = std::make_shared<CSketchLine>();
    line->startPos = {0, 0, 0};
    line->endPos = {10, 10, 0};
    sketch->segments.push_back(line);

    model.AddFeature(sketch);

    auto extrude = std::make_shared<CExtrude>();
    extrude->featureID = "EX-DUMMY";
    extrude->featureName = "DummyExtrude";
    extrude->sketchProfile = sketch;
    extrude->endCondition1.depth = 50.0;
    extrude->endCondition1.type = ExtrudeEndCondition::Type::BLIND;
    extrude->operation = BooleanOp::NEW_BODY;

    model.AddFeature(extrude);
  }

  SimulateWriteToSolidWorks(model);

  return 0;
}
