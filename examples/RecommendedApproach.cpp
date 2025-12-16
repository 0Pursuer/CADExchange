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
        // Save the model
        // =========================================================================
        std::cout << "\n[7] Saving model...\n";
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
        std::cout << "4. Chain builder calls for fluent interface\n";
        std::cout << "5. Use convenience methods (AddLine, AddCircle, etc.) for sketches\n";
        std::cout << "\nAdvantages:\n";
        std::cout << "- Type-safe: Each builder handles its own type\n";
        std::cout << "- Fluent: Easy to read and write\n";
        std::cout << "- Convenient: Helper functions like AddLine() save boilerplate\n";
        std::cout << "- Flexible: Mix builders with direct API when needed\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}
