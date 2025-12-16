// clangd-format off
#include "../../service/builders/EndConditionBuilder.h"
#include "../../service/builders/ExtrudeBuilder.h"
#include "../../service/builders/ReferenceBuilder.h"
#include "../../service/builders/RevolveBuilder.h"
#include "../../service/builders/SketchBuilder.h"
#include "../../core/UnifiedModel.h"

#include "../../service/serialization/CADSerializer.h"
#include "UnifiedTypes.h"
#include <iostream>
#include <vector>
// clangd-format on

using namespace CADExchange;

/**
 * @brief 模拟从 SolidWorks 读取数据的过程 (Extraction)
 *
 * 参考 SwRead 项目中的:
 * - CSketchInfo::AnalyzeSketch (SwRead/src/SketchInfo.cpp)
 * - ExtrudeInfo::AnalyzeExtrude (SwRead/src/ExtrudeInfo.cpp)
 *
 * 此函数模拟了通过 SW API 获取几何数据，并将其转换为 UnifiedModel 的过程。
 */
void SimulateReadFromSolidWorks(UnifiedModel &model) {
  std::cout << "[SwRead Simulation] Starting to read part..." << std::endl;

  // =========================================================
  // 1. 模拟读取草图 "Sketch1"
  // 对应 CSketchInfo::AnalyzeSketch -> ExtractSketchSegments
  // =========================================================
  std::cout << "[SwRead Simulation] Analyzing Sketch 'Sketch1'..." << std::endl;

  // 1.1 提取参考平面 (ExtractRefFaceInfo)
  // 假设 SW API 返回了 Front Plane
  std::string refPlaneID = StandardID::PLANE_XY; // 映射到标准 XY 平面

  Builder::SketchBuilder sketchBuilder(model, "Sketch1");
  sketchBuilder.SetReferencePlane(Builder::RefPlaneBuilder(refPlaneID));

  // 1.2 提取草图段 (ExtractSketchSegments)
  // 假设我们遍历 ISketchSegment 得到了以下几何数据
  std::cout << "[SwRead Simulation] Extracting segments (Lines & Arcs)..."
            << std::endl;

  // 模拟读取一个 100x50 的矩形
  // Point 1
  sketchBuilder.AddPoint(CPoint3D{0, 0, 0});
  // Point 2
  sketchBuilder.AddPoint(CPoint3D{100, 0, 0});
  // Point 3
  sketchBuilder.AddPoint(CPoint3D{100, 50, 0});
  // Point 4
  sketchBuilder.AddPoint(CPoint3D{0, 50, 0});
  // Line 1
  sketchBuilder.AddLine(CPoint3D{0, 0, 0}, CPoint3D{100, 0, 0});
  // Line 2
  sketchBuilder.AddLine(CPoint3D{100, 0, 0}, CPoint3D{100, 50, 0});
  // Line 3
  sketchBuilder.AddLine(CPoint3D{100, 50, 0}, CPoint3D{0, 50, 0});
  // Line 4
  sketchBuilder.AddLine(CPoint3D{0, 50, 0}, CPoint3D{0, 0, 0});

  // 模拟读取一个中心圆
  // Circle
  sketchBuilder.AddCircle(CPoint3D{50, 25, 0}, 15.0);

  // 1.3 提取约束 (ExtractConstraintInfo)
  // 假设读取到了水平/垂直约束
  // sketchBuilder.AddConstraint(...) // 暂略

  std::string sketchID = sketchBuilder.Build();
  std::cout << "[SwRead Simulation] Sketch built and added to model. ID: "
            << sketchID << std::endl;

  // std::cout << "[SwRead Simulation] show sketch JSON: " <<
  // sketchBuilder.Show()
  //           << std::endl;

  // =========================================================
  // 2. 模拟读取拉伸 "Boss-Extrude1"
  // 对应 ExtrudeInfo::AnalyzeExtrude -> ExtractDirAndEndInfo
  // =========================================================
  std::cout << "[SwRead Simulation] Analyzing Extrude 'Boss-Extrude1'..."
            << std::endl;

  Builder::ExtrudeBuilder extrudeBuilder(model, "Boss-Extrude1");

  // 2.1 关联轮廓草图
  // 在 SW 中是通过 GetParents 获取草图特征
  extrudeBuilder.SetProfile(sketchID);

  // 2.2 提取方向和深度 (ExtractDirAndEndInfo)
  // 假设读取到: Blind, Depth=20mm, Direction=(0,0,1)
  double readDepth = 20.0;
  CVector3D readDirection{0, 0, 1};

  extrudeBuilder.SetDirection(readDirection);
  extrudeBuilder.SetOperation(BooleanOp::BOSS); // IsBossFeature = true

  extrudeBuilder.SetEndCondition1(Builder::EndCondition::Blind(readDepth));
  extrudeBuilder.SetEndCondition2(
      Builder::EndCondition::Blind(readDepth / 2.0));

  // 2.3 提取薄壁特征 (IsThinFeature)
  // 假设 IsThin = false

  std::string extrudeID = extrudeBuilder.Build();
  std::cout << "[SwRead Simulation] Extrude built and added to model. ID: "
            << extrudeID << std::endl;

  // =========================================================
  // 3. 基于拉伸面创建第二个草图
  // =========================================================
  std::cout << "[SwRead Simulation] Building Sketch2 on extruded face..."
            << std::endl;
  auto faceRef = Builder::Ref::Face(extrudeID, 0)
                     .Normal(0, 0, 1)
                     .Centroid(50, 25, 20)
                     .UDir(1, 0, 0)
                     .VDir(0, -1, 0);

  Builder::SketchBuilder sketchBuilder2(model, "Sketch2");
  sketchBuilder2.SetReferencePlane(faceRef);
  sketchBuilder2.AddCircle(CPoint3D{50, 25, 20}, 10.0);
  std::string sketch2ID = sketchBuilder2.Build();
  std::cout << "[SwRead Simulation] Sketch2 built. ID: " << sketch2ID
            << std::endl;

  // =========================================================
  // 4. 向上切割拉伸
  // =========================================================
  std::cout << "[SwRead Simulation] Adding Cut-Extrude1 based on Sketch2..."
            << std::endl;
  Builder::ExtrudeBuilder cutBuilder(model, "Cut-Extrude1");
  cutBuilder.SetProfile(sketch2ID);
  cutBuilder.SetDirection(CVector3D{0, 0, 1});
  cutBuilder.SetOperation(BooleanOp::CUT);
  cutBuilder.SetEndCondition1(Builder::EndCondition::ThroughAll());

  std::string cutID = cutBuilder.Build();
  std::cout << "[SwRead Simulation] Cut extrude built. ID: " << cutID
            << std::endl;

  // =========================================================
  // 5. 更多草图/参考，覆盖各种引用类型 (面、边、点、草图)
  // =========================================================
  std::cout << "[SwRead Simulation] Building Sketch3 on cut face..."
            << std::endl;
  auto cutFaceRef = Builder::Ref::Face(cutID, 1)
                        .Normal(0, 0, -1)
                        .Centroid(50, 25, 0)
                        .UDir(1, 0, 0)
                        .VDir(0, 1, 0);

  Builder::SketchBuilder sketchBuilder3(model, "Sketch3");
  sketchBuilder3.SetReferencePlane(cutFaceRef);
  auto sketch3Line =
      sketchBuilder3.AddLine(CPoint3D{10, 25, 0}, CPoint3D{90, 25, 0});
  auto sketch3Circle = sketchBuilder3.AddCircle(CPoint3D{50, 25, 0}, 8.0);
  sketchBuilder3.AddDistanceDimension(sketch3Line, sketch3Circle, 30.0);
  std::string sketch3ID = sketchBuilder3.Build();
  std::cout << "[SwRead Simulation] Sketch3 built. ID: " << sketch3ID
            << std::endl;

  // =========================================================
  // 6. 旋转特征：展示引用草图/特征/边（Revolve Axis）
  // =========================================================
  std::cout << "[SwRead Simulation] Adding Revolve-SketchRef..." << std::endl;
  Builder::RevolveBuilder revolveSketchRef(model, "Revolve-SketchRef");
  revolveSketchRef.SetProfile(sketch3ID);
  auto sketchRef = std::make_shared<CRefSketch>();
  sketchRef->targetFeatureID = sketchID;
  revolveSketchRef.SetAxisRef(sketchRef);
  revolveSketchRef.SetAngle(360.0);
  std::string revolveSketchRefID = revolveSketchRef.Build();
  std::cout << "[SwRead Simulation] Revolve-SketchRef built. ID: "
            << revolveSketchRefID << std::endl;

  std::cout << "[SwRead Simulation] Adding Revolve-EdgeRef..." << std::endl;
  Builder::RevolveBuilder revolveEdgeRef(model, "Revolve-EdgeRef");
  revolveEdgeRef.SetProfile(sketch3ID);
  auto edgeRef = Builder::Ref::Edge(cutID, 2).MidPoint(50, 25, 2.5);
  revolveEdgeRef.SetAxisRef(edgeRef);
  revolveEdgeRef.SetAngle(180.0);
  std::string revolveEdgeRefID = revolveEdgeRef.Build();
  std::cout << "[SwRead Simulation] Revolve-EdgeRef built. ID: "
            << revolveEdgeRefID << std::endl;

  std::cout << "[SwRead Simulation] Adding Revolve-FeatureRef..." << std::endl;
  Builder::RevolveBuilder revolveFeatureRef(model, "Revolve-FeatureRef");
  revolveFeatureRef.SetProfile(sketch3ID);
  auto featureRef =
      std::make_shared<CRefFeature>(RefType::FEATURE_WHOLE_SKETCH);
  featureRef->targetFeatureID = extrudeID;
  revolveFeatureRef.SetAxisRef(featureRef);
  revolveFeatureRef.SetAngle(90.0);
  std::string revolveFeatureRefID = revolveFeatureRef.Build();
  std::cout << "[SwRead Simulation] Revolve-FeatureRef built. ID: "
            << revolveFeatureRefID << std::endl;

  // =========================================================
  // 7. 顶点驱动的拉伸 (用于验证 CRefVertex 引用)
  // =========================================================
  std::cout << "[SwRead Simulation] Adding VertexCut extrude..." << std::endl;
  auto vertexRef = Builder::Ref::Vertex(revolveFeatureRefID, 0).Pos(50, 25, 5);

  Builder::ExtrudeBuilder vertexCut(model, "VertexCut");
  vertexCut.SetProfile(sketch3ID);
  vertexCut.SetOperation(BooleanOp::CUT);
  vertexCut.SetDirection(CVector3D{0, 0, -1});
  vertexCut.SetEndCondition1(Builder::EndCondition::UpToVertex(vertexRef));
  std::string vertexCutID = vertexCut.Build();
  std::cout << "[SwRead Simulation] VertexCut built. ID: " << vertexCutID
            << std::endl;

  std::cout << "[SwRead Simulation] Part reading completed." << std::endl;
}

int main() {
  // 初始化统一模型
  UnifiedModel model;
  model.modelName = "SimulationPart";
  model.unit = UnitType::MILLIMETER;

  // 执行模拟读取
  SimulateReadFromSolidWorks(model);

  // 序列化保存 (Cereal)
  std::string outputPath = "SimulationPart.xml";
  std::string error;
  if (SaveModel(model, outputPath, &error)) {
    std::cout << "Successfully serialized model (Cereal) to " << outputPath
              << std::endl;
  } else {
    std::cerr << "Failed to save model (Cereal): " << error << std::endl;
    return 1;
  }

  // 序列化保存 (TinyXML2)
  std::string outputPathTiny = "SimulationPart_Tiny.xml";
  if (SaveModel(model, outputPathTiny, &error, SerializationFormat::TINYXML)) {
    std::cout << "Successfully serialized model (TinyXML) to " << outputPathTiny
              << std::endl;
  } else {
    std::cerr << "Failed to save model (TinyXML): " << error << std::endl;
    return 1;
  }

  return 0;
}
