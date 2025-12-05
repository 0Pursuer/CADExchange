#include "../builders/ExtrudeBuilder.h"
#include "../builders/RevolveBuilder.h"
#include "../builders/SketchBuilder.h"
#include "../core/TypeAdapters.h"
#include "../core/UnifiedModel.h"
#include "../core/UnifiedTypes.h"
#include "../serialization/CADSerializer.h"
#include <cassert>
#include <iostream>
#include <vector>


using namespace CADExchange;

// =========================================================
// User defined types (Simulating client-side types)
// =========================================================
struct MyPoint {
  float x, y, z;
};

struct MyVector {
  double x, y, z;
};

// =========================================================
// Specialize adapters
// =========================================================
namespace CADExchange {
template <> struct PointAdapter<MyPoint> {
  static CPoint3D Convert(const MyPoint &pt) {
    return {(double)pt.x, (double)pt.y, (double)pt.z};
  }
};

template <> struct VectorAdapter<MyVector> {
  static CVector3D Convert(const MyVector &vec) {
    return {vec.x, vec.y, vec.z};
  }
};
} // namespace CADExchange

/**
 * @brief 模拟从 SolidWorks 读取数据的过程，但使用新特性：
 * 1. ExternalID: 模拟 SW 的持久 ID
 * 2. TypeAdapters: 直接使用 MyPoint/MyVector
 * 3. Validation: 构建后验证模型完整性
 */
void SimulateReadFromSolidWorks(UnifiedModel &model) {
  std::cout << "[SwRead Simulation] Starting to read part (Advanced Mode)..."
            << std::endl;

  // =========================================================
  // 1. 模拟读取草图 "Sketch1"
  // =========================================================
  std::cout << "[SwRead Simulation] Analyzing Sketch 'Sketch1'..." << std::endl;

  Builder::SketchBuilder sketchBuilder(model, "Sketch1");
  sketchBuilder.SetExternalID("SW-Sketch1"); // Set External ID
  sketchBuilder.SetReferencePlane(StandardID::PLANE_XY);

  std::cout << "[SwRead Simulation] Extracting segments using MyPoint..."
            << std::endl;

  // 使用 MyPoint 直接添加几何
  sketchBuilder.AddPoint(MyPoint{0, 0, 0});
  sketchBuilder.AddPoint(MyPoint{100, 0, 0});
  sketchBuilder.AddPoint(MyPoint{100, 50, 0});
  sketchBuilder.AddPoint(MyPoint{0, 50, 0});

  sketchBuilder.AddLine(MyPoint{0, 0, 0}, MyPoint{100, 0, 0});
  sketchBuilder.AddLine(MyPoint{100, 0, 0}, MyPoint{100, 50, 0});
  sketchBuilder.AddLine(MyPoint{100, 50, 0}, MyPoint{0, 50, 0});
  sketchBuilder.AddLine(MyPoint{0, 50, 0}, MyPoint{0, 0, 0});

  sketchBuilder.AddCircle(MyPoint{50, 25, 0}, 15.0);

  std::string sketchID = sketchBuilder.Build();
  std::cout << "[SwRead Simulation] Sketch built. Internal ID: " << sketchID
            << ", External ID: SW-Sketch1" << std::endl;

  // =========================================================
  // 2. 模拟读取拉伸 "Boss-Extrude1"
  // =========================================================
  std::cout << "[SwRead Simulation] Analyzing Extrude 'Boss-Extrude1'..."
            << std::endl;

  Builder::ExtrudeBuilder extrudeBuilder(model, "Boss-Extrude1");
  extrudeBuilder.SetExternalID("SW-Extrude1");

  // 使用 External ID 关联轮廓
  extrudeBuilder.SetProfileByExternalID("SW-Sketch1");

  // 使用 MyVector 设置方向
  extrudeBuilder.SetDirection(MyVector{0, 0, 1});
  extrudeBuilder.SetOperation(BooleanOp::BOSS);

  ExtrudeEndCondition endCond1;
  endCond1.type = ExtrudeEndCondition::Type::BLIND;
  endCond1.depth = 20.0;
  extrudeBuilder.SetEndCondition1(endCond1);

  ExtrudeEndCondition endCond2;
  endCond2.type = ExtrudeEndCondition::Type::BLIND;
  endCond2.depth = 10.0;
  extrudeBuilder.SetEndCondition2(endCond2);

  std::string extrudeID = extrudeBuilder.Build();
  std::cout << "[SwRead Simulation] Extrude built. Internal ID: " << extrudeID
            << ", External ID: SW-Extrude1" << std::endl;

  // =========================================================
  // 3. 基于拉伸面创建第二个草图
  // =========================================================
  std::cout << "[SwRead Simulation] Building Sketch2 on extruded face..."
            << std::endl;

  // 引用面目前仍需使用 Internal ID (parentFeatureID)
  CRefFace faceRef;
  faceRef.parentFeatureID = extrudeID;
  faceRef.topologyIndex = 0;
  faceRef.normal = {0, 0, 1};
  faceRef.centroid = {50, 25, 20};

  Builder::SketchBuilder sketchBuilder2(model, "Sketch2");
  sketchBuilder2.SetExternalID("SW-Sketch2");
  sketchBuilder2.SetReferenceFace(faceRef);
  sketchBuilder2.AddCircle(MyPoint{50, 25, 20}, 10.0);

  std::string sketch2ID = sketchBuilder2.Build();
  std::cout << "[SwRead Simulation] Sketch2 built. External ID: SW-Sketch2"
            << std::endl;

  // =========================================================
  // 4. 向上切割拉伸
  // =========================================================
  std::cout << "[SwRead Simulation] Adding Cut-Extrude1..." << std::endl;
  Builder::ExtrudeBuilder cutBuilder(model, "Cut-Extrude1");
  cutBuilder.SetExternalID("SW-CutExtrude1");
  cutBuilder.SetProfileByExternalID("SW-Sketch2");
  cutBuilder.SetDirection(MyVector{0, 0, 1});
  cutBuilder.SetOperation(BooleanOp::CUT);
  cutBuilder.SetThroughAll();

  std::string cutID = cutBuilder.Build();
  std::cout
      << "[SwRead Simulation] Cut extrude built. External ID: SW-CutExtrude1"
      << std::endl;

  // =========================================================
  // 5. 更多草图/参考
  // =========================================================
  std::cout << "[SwRead Simulation] Building Sketch3 on cut face..."
            << std::endl;
  CRefFace cutFaceRef;
  cutFaceRef.parentFeatureID = cutID;
  cutFaceRef.topologyIndex = 1;
  cutFaceRef.normal = {0, 0, -1};
  cutFaceRef.centroid = {50, 25, 0};

  Builder::SketchBuilder sketchBuilder3(model, "Sketch3");
  sketchBuilder3.SetExternalID("SW-Sketch3");
  sketchBuilder3.SetReferenceFace(cutFaceRef);
  auto sketch3Line =
      sketchBuilder3.AddLine(MyPoint{10, 25, 0}, MyPoint{90, 25, 0});
  auto sketch3Circle = sketchBuilder3.AddCircle(MyPoint{50, 25, 0}, 8.0);
  sketchBuilder3.AddDistanceDimension(sketch3Line, sketch3Circle, 30.0);

  std::string sketch3ID = sketchBuilder3.Build();
  std::cout << "[SwRead Simulation] Sketch3 built. External ID: SW-Sketch3"
            << std::endl;

  // =========================================================
  // 6. 旋转特征
  // =========================================================
  std::cout << "[SwRead Simulation] Adding Revolve-SketchRef..." << std::endl;
  Builder::RevolveBuilder revolveSketchRef(model, "Revolve-SketchRef");
  revolveSketchRef.SetExternalID("SW-Revolve1");

  // 演示：通过 ExternalID 查找 InternalID
  auto sketch3Feature = model.GetFeatureByExternalID("SW-Sketch3");
  if (sketch3Feature) {
    revolveSketchRef.SetProfile(sketch3Feature->featureID);
  }

  auto sketchRef = std::make_shared<CRefSketch>();
  sketchRef->targetFeatureID = sketchID; // Internal ID of Sketch1
  revolveSketchRef.SetAxisRef(sketchRef);
  revolveSketchRef.SetAngle(360.0);

  std::string revolveSketchRefID = revolveSketchRef.Build();
  std::cout << "[SwRead Simulation] Revolve-SketchRef built." << std::endl;

  // =========================================================
  // 7. 顶点驱动的拉伸
  // =========================================================
  std::cout << "[SwRead Simulation] Adding VertexCut extrude..." << std::endl;
  auto vertexRef = std::make_shared<CRefVertex>();
  vertexRef->parentFeatureID = revolveSketchRefID;
  vertexRef->topologyIndex = 0;
  vertexRef->pos = {50, 25, 5};

  ExtrudeEndCondition vertexEnd;
  vertexEnd.type = ExtrudeEndCondition::Type::UP_TO_VERTEX;
  vertexEnd.referenceEntity = vertexRef;

  Builder::ExtrudeBuilder vertexCut(model, "VertexCut");
  vertexCut.SetExternalID("SW-VertexCut");
  vertexCut.SetProfileByExternalID("SW-Sketch3");
  vertexCut.SetOperation(BooleanOp::CUT);
  vertexCut.SetDirection(MyVector{0, 0, -1});
  vertexCut.SetEndCondition1(vertexEnd);

  std::string vertexCutID = vertexCut.Build();
  std::cout << "[SwRead Simulation] VertexCut built." << std::endl;

  // =========================================================
  // 8. 验证模型 (Validation)
  // =========================================================
  std::cout << "[SwRead Simulation] Validating model..." << std::endl;
  auto report = model.Validate();
  if (report.isValid) {
    std::cout << "[Validation] Model is VALID." << std::endl;
  } else {
    std::cerr << "[Validation] Model is INVALID!" << std::endl;
    for (const auto &err : report.errors) {
      std::cerr << " - " << err << std::endl;
    }
  }

  std::cout << "[SwRead Simulation] Part reading completed." << std::endl;
}

int main() {
  // 初始化统一模型
  UnifiedModel model;
  model.modelName = "SimulationPart_Advanced";
  model.unit = UnitType::MILLIMETER;

  // 执行模拟读取
  SimulateReadFromSolidWorks(model);

  // 序列化保存 (Cereal)
  std::string outputPath = "SimulationPart_Advanced.xml";
  std::string error;
  if (SaveModel(model, outputPath, &error)) {
    std::cout << "Successfully serialized model (Cereal) to " << outputPath
              << std::endl;
  } else {
    std::cerr << "Failed to save model (Cereal): " << error << std::endl;
    return 1;
  }

  return 0;
}
