/**
 * @file RecommendedApproach.cpp
 * @brief Demonstrates the recommended approach for using CAD builders
 * 
 * Shows best practices:
 * 1. Use individual builders (SketchBuilder, ExtrudeBuilder, RevolveBuilder)
 * 2. Leverage factory patterns (Ref::*, EndCondition::*)
 * 3. Use fluent interface for method chaining
 * 4. Combine builder convenience with raw API when needed
 */

#include <iostream>
#include <memory>
#include "../service/builders/SketchBuilder.h"
#include "../service/builders/ExtrudeBuilder.h"
#include "../service/builders/RevolveBuilder.h"
#include "../service/builders/ReferenceBuilder.h"
#include "../service/builders/EndConditionBuilder.h"
#include "../core/UnifiedModel.h"
#include "../service/serialization/TinyXMLSerializer.h"
#include "../service/serialization/CADSerializer.h"

// CADSerializer.h includes cereal which redefines CEREAL_NVP properly
// so we don't need the placeholder macro after this point

using namespace CADExchange::Builder;
using namespace CADExchange;

void PrintResult(const std::string& featureName, const std::string& id) {
    if (!id.empty()) {
        std::cout << "✓ Created " << featureName << " (ID: " << id << ")\n";
    } else {
        std::cout << "✗ Failed to create " << featureName << "\n";
    }
}

/**
 * @brief 演示改进的 ExtrudeBuilder 用法（新方法）
 * 
 * 使用新的便利方法：
 * - SetProfileByName()：直接接受草图名称，自动 ID 转换
 * - 链式调用：流畅的方法链
 * - EndConditionHelper：简化终止条件构造
 */
std::string DemoImprovedExtrudeBuilder(UnifiedModel& model) {
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "IMPROVED ExtrudeBuilder DEMONSTRATION\n";
    std::cout << std::string(70, '=') << "\n";

    try {
        // 方法 1: 使用 SetProfileByName() - 自动查找并转换 ID
        std::cout << "\n[Demo 1] Using SetProfileByName() for convenience:\n";
        auto extrudeId1 = ExtrudeBuilder(model, "ImprovedExtrude_1")
            .SetProfileByName("BaseSketch")              // 直接用名字，自动转换 ID
            .SetOperation(BooleanOp::BOSS)
            .SetDirection(CVector3D{0, 0, 1})            // 指定类型以支持模板推导
            .SetEndCondition1(EndCondition::Blind(15.0))
            .Build();
        PrintResult("ImprovedExtrude_1", extrudeId1);

        // 方法 2: 使用 EndConditionHelper 简化复杂终止条件
        std::cout << "\n[Demo 2] Using EndConditionHelper for complex conditions:\n";
        
        // 创建一个参考顶点用于演示
        CPoint3D demoVertex{50.0, 25.0, 30.0};
        
        auto extrudeId2 = ExtrudeBuilder(model, "ImprovedExtrude_2")
            .SetProfileByName("BaseSketch")
            .SetOperation(BooleanOp::BOSS)
            .SetDirection(CVector3D{0, 0, 1})
            // 使用 Helper 简化拉伸到顶点的条件创建
            .SetEndCondition1(
                EndConditionHelper::UpToVertex(
                    model,
                    extrudeId1,              // 参考特征 ID
                    demoVertex,              // 顶点坐标
                    0,                       // 拓扑索引
                    0.0))                    // 偏移
            .Build();
        PrintResult("ImprovedExtrude_2", extrudeId2);

        // 方法 3: 两向拉伸（高级用法）
        std::cout << "\n[Demo 3] BiDirectional extrude with two end conditions:\n";
        auto extrudeId3 = ExtrudeBuilder(model, "BiDirectionalExtrude")
            .SetProfileByName("BaseSketch")
            .SetOperation(BooleanOp::BOSS)
            .SetDirection(CVector3D{0, 0, 1})
            .SetEndCondition1(EndCondition::Blind(10.0))    // 第一方向：盲孔 10mm
            .SetEndCondition2(EndCondition::Blind(-5.0))    // 第二方向：向下 5mm
            .SetDraft(2.0, true)                            // 拔模 2 度
            .Build();
        PrintResult("BiDirectionalExtrude", extrudeId3);

        // 方法 4: 使用参考平面的高级示例
        std::cout << "\n[Demo 4] Extrude with reference plane (Advanced):\n";
        auto extrudeId4 = ExtrudeBuilder(model, "ExtrudeToPlane")
            .SetProfileByName("BaseSketch")
            .SetOperation(BooleanOp::BOSS)
            .SetDirection(CVector3D{0, 0, 1})
            .SetEndCondition1(
                EndConditionHelper::UpToRefPlane(
                    model,
                    StandardID::PLANE_XY,
                    StandardID::kOrigin,
                    StandardID::kPlaneXYNormal,
                    StandardID::kAxisX,
                    5.0))  // 偏移 5 单位
            .Build();
        PrintResult("ExtrudeToPlane", extrudeId4);

        std::cout << "\n✓ All improved extrude examples completed successfully!\n";
        return extrudeId1;

    } catch (const std::exception& e) {
        std::cerr << "✗ Error in improved extrude demo: " << e.what() << "\n";
        return "";
    }
}

std::string BuildBaseSketch(UnifiedModel& model) {
    SketchBuilder sketch(model, "BaseSketch");
    sketch.SetReferencePlane(Ref::XY());
    sketch.AddLine(CPoint3D{0.0, 0.0, 0.0}, CPoint3D{100.0, 0.0, 0.0});
    sketch.AddLine(CPoint3D{100.0, 0.0, 0.0}, CPoint3D{100.0, 50.0, 0.0});
    sketch.AddLine(CPoint3D{100.0, 50.0, 0.0}, CPoint3D{0.0, 50.0, 0.0});
    sketch.AddLine(CPoint3D{0.0, 50.0, 0.0}, CPoint3D{0.0, 0.0, 0.0});
    sketch.AddCircle(CPoint3D{50.0, 25.0, 0.0}, 15.0);
    return sketch.Build();
}

std::string CreateFaceCenteredSketch(UnifiedModel& model,
                                     const std::string& name,
                                     std::shared_ptr<CRefEntityBase> ref,
                                     double minX, double minY, double maxX,
                                     double maxY) {
    SketchBuilder sketch(model, name);
    sketch.SetReferencePlane(ref);
    sketch.AddLine(CPoint3D{minX, minY, 20.0}, CPoint3D{maxX, minY, 20.0});
    sketch.AddLine(CPoint3D{maxX, minY, 20.0}, CPoint3D{maxX, maxY, 20.0});
    sketch.AddLine(CPoint3D{maxX, maxY, 20.0}, CPoint3D{minX, maxY, 20.0});
    sketch.AddLine(CPoint3D{minX, maxY, 20.0}, CPoint3D{minX, minY, 20.0});
    return sketch.Build();
}

std::string BuildRevolveProfileSketch(UnifiedModel& model) {
    SketchBuilder sketch(model, "RevolveProfile");
    sketch.SetReferencePlane(Ref::YZ());
    sketch.AddLine(CPoint3D{0.0, 40.0, 0.0}, CPoint3D{0.0, 40.0, 25.0});
    sketch.AddLine(CPoint3D{0.0, 40.0, 25.0}, CPoint3D{0.0, 50.0, 25.0});
    sketch.AddLine(CPoint3D{0.0, 50.0, 25.0}, CPoint3D{0.0, 50.0, 0.0});
    return sketch.Build();
}

std::string BuildExtrudeFeature(UnifiedModel& model,
                                const std::string& featureName,
                                const std::string& sketchID,
                                BooleanOp op,
                                const ExtrudeEndCondition& condition) {
    ExtrudeBuilder builder(model, featureName);
    builder.SetProfile(sketchID)
    .SetOperation(op)
    .SetEndCondition1(condition);
    return builder.Build();
}

int main() {
    try {
        // Initialize model
        UnifiedModel model(UnitType::METER, "model RecommendedApproach");

        std::cout << "Creating part with recommended builder approach...\n\n";

        // =========================================================================
        // FEATURE 1: Base Sketch on XY plane
        // =========================================================================
        std::cout << "[1] Creating base sketch on XY plane...\n";
        const auto sketchID = BuildBaseSketch(model);
        PrintResult("Base Sketch", sketchID);

        // =========================================================================
        // FEATURE 2: Extrude the sketch (Pad)
        // =========================================================================
        std::cout << "\n[2] Creating extrude feature...\n";
        const auto extrudeID = BuildExtrudeFeature(model, "Extrude_Pad", sketchID,
                                                   BooleanOp::BOSS,
                                                   EndCondition::Blind(20.0));
        PrintResult("Extrude (Pad)", extrudeID);

        // =========================================================================
        // FEATURE 3: Sketch on top face of extrude (for cut)
        // =========================================================================
        std::cout << "\n[3] Creating sketch on top face of extrude...\n";
        const auto topFaceRef = Ref::Face(extrudeID, 0)
                             .Normal(0, 0, 1)
                             .Centroid(50.0, 25.0, 20.0)
                             .UDir(1, 0, 0)
                             .VDir(0, 1, 0);
        const auto sketch2ID = CreateFaceCenteredSketch(model, "SketchOnFace",
                                                       topFaceRef, 35.0, 15.0,
                                                       65.0, 35.0);
        PrintResult("Sketch on Face", sketch2ID);

        // =========================================================================
        // FEATURE 4: Cut extrude (subtractive extrude)
        // =========================================================================
        std::cout << "\n[4] Creating cut extrude feature...\n";
        const auto cutID = BuildExtrudeFeature(model, "Extrude_Cut", sketch2ID,
                                               BooleanOp::CUT,
                                               EndCondition::ThroughAll());
        PrintResult("Cut Extrude", cutID);

        // =========================================================================
        // FEATURE 5: Sketch for revolve feature
        // =========================================================================
        std::cout << "\n[5] Creating profile sketch for revolve...\n";
        {
            const auto profileID = BuildRevolveProfileSketch(model);
            PrintResult("Revolve Profile Sketch", profileID);

            // =========================================================================
            // FEATURE 6: Revolve feature
            // =========================================================================
            std::cout << "\n[6] Creating revolve feature...\n";
            {
                RevolveBuilder revolve(model, "RevolveFeature");
                revolve.SetProfile(profileID);
                revolve.SetAxisRef(Ref::Axis(StandardID::AXIS_Z)
                                       .Origin(StandardID::kOrigin)
                                       .Direction(StandardID::kAxisZ));
                revolve.SetAngle(360.0);      // Full 360 degree revolution
                
                std::string revolveID = revolve.Build();
                PrintResult("Revolve Feature", revolveID);
            }
        }

        // =========================================================================
        // FEATURE 7: Demonstrate improved ExtrudeBuilder with new convenience methods
        // =========================================================================
        std::cout << "\n[7] Demonstrating improved ExtrudeBuilder methods...\n";
        DemoImprovedExtrudeBuilder(model);

        // =========================================================================
        // Save the model
        // =========================================================================
        std::cout << "\n[8] Saving model...\n";
        try {
            std::string errorMsg;
            bool saveOk = SaveModel(model, "RecommendedApproach_Output.xml", &errorMsg, SerializationFormat::TINYXML);
            if (saveOk) {
                std::cout << "✓ Model saved to RecommendedApproach_Output.xml\n";
            } else {
                std::cout << "✗ Failed to save model: " << errorMsg << "\n";
            }
        } catch (const std::exception& e) {
            std::cout << "✗ Failed to save model: " << e.what() << "\n";
        }

        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "Summary of Recommended Approach:\n";
        std::cout << std::string(70, '=') << "\n";
        std::cout << "1. Use individual builders (SketchBuilder, ExtrudeBuilder, etc.)\n";
        std::cout << "2. Leverage factory patterns:\n";
        std::cout << "   - Ref::XY(), Ref::YZ(), Ref::ZX() for standard planes\n";
        std::cout << "   - Ref::Face(id, index) for topology references\n";
        std::cout << "   - Ref::Axis(id) for axis references\n";
        std::cout << "   - Ref::Edge(id, index), Ref::Vertex(id, index) as needed\n";
        std::cout << "3. Use end condition factories:\n";
        std::cout << "   - EndCondition::Blind(depth)\n";
        std::cout << "   - EndCondition::ThroughAll()\n";
        std::cout << "   - EndCondition::UpToFace(ref, offset)\n";
        std::cout << "4. Use improved ExtrudeBuilder convenience methods:\n";
        std::cout << "   - SetProfileByName(name) - direct name-based lookup\n";
        std::cout << "   - EndConditionHelper::UpToVertex() - simplified vertex refs\n";
        std::cout << "   - EndConditionHelper::UpToRefPlane() - simplified plane refs\n";
        std::cout << "5. Chain builder calls for fluent interface\n";
        std::cout << "6. Use convenience methods (AddLine, AddCircle, etc.) for sketches\n";
        std::cout << "\nAdvantages:\n";
        std::cout << "- Type-safe: Each builder handles its own type\n";
        std::cout << "- Fluent: Easy to read and write\n";
        std::cout << "- Convenient: Helper functions save boilerplate code\n";
        std::cout << "- Flexible: Mix builders with direct API when needed\n";
        std::cout << "- Maintainable: Clean separation of concerns\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}
