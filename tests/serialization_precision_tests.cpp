#include "SketchBuilder.h"
#include "TinyXMLSerializer.h"

#include <filesystem>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace CADExchange;
using namespace CADExchange::Builder;

void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
  try {
    Require(argc == 2, "expected a temporary output directory");
    const auto output = std::filesystem::path(argv[1]) / "precision.xml";
    std::filesystem::create_directories(output.parent_path());
    UnifiedModel model(UnitType::MILLIMETER, "precision");
    SketchBuilder sketch(model, "precise sketch");
    sketch.SetReferencePlane(Ref::XY());
    // Actual coordinates from the 3d7 STEP: decimal display is not a CAD tolerance.
    const CPoint3D point{181.8653347947, 165.25662235829003, 4.5e-12};
    const double axis = std::sqrt(0.5);
    sketch.SetCSys(point, CVector3D{axis, axis, 0},
                   CVector3D{-axis, axis, 0}, CVector3D{0, 0, 1});
    sketch.AddLine(point, CPoint3D{221.8653347947, -80.00000000000001, -4.5e-12});
    const auto id = sketch.Build();
    Require(!id.empty(), "sketch build failed");
    std::string detail;
    Require(TinyXMLSerializer::Save(model, output, &detail), "save failed");
    UnifiedModel loaded;
    Require(TinyXMLSerializer::Load(loaded, output, &detail), "load failed");
    const auto read = loaded.GetFeatureAs<CSketch>(id);
    Require(read != nullptr, "missing reloaded sketch");
    Require(read->sketchCSys.origin.x == point.x &&
            read->sketchCSys.origin.y == point.y &&
            read->sketchCSys.origin.z == point.z, "CSys origin lost double precision");
    Require(read->sketchCSys.xDir.x == axis && read->sketchCSys.yDir.x == -axis,
            "CSys directions lost double precision");
    const auto line = std::dynamic_pointer_cast<CSketchLine>(read->segments.at(0));
    Require(line != nullptr && line->startPos.x == point.x &&
            line->startPos.y == point.y && line->startPos.z == point.z &&
            line->endPos.x == 221.8653347947 && line->endPos.z == -4.5e-12,
            "sketch endpoint lost double precision");
    std::cout << "PASS: TinyXML preserves coordinate and direction doubles\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
