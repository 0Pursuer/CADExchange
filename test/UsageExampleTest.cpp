#include "../include/ExtrudeBuilder.h"
#include "../include/SketchBuilder.h"
#include "../include/TypeAdapters.h"
#include "../include/UnifiedModel.h"
#include <cassert>
#include <iostream>

// User defined types
struct MyPoint {
  float x, y, z;
};

struct MyVector {
  double x, y, z;
};

// Specialize adapters
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

void TestExternalID() {
  CADExchange::UnifiedModel model;

  // Create Sketch with External ID "SW-101"
  CADExchange::Builder::SketchBuilder sketchBuilder(model, "Sketch1");
  sketchBuilder.SetExternalID("SW-101");
  sketchBuilder.SetPlane(MyPoint{0, 0, 0}, MyVector{1, 0, 0},
                         MyVector{0, 0, 1});
  sketchBuilder.AddCircle(MyPoint{0, 0, 0}, 10.0);
  sketchBuilder.Build();

  // Verify lookup
  auto sketch = model.GetFeatureByExternalID("SW-101");
  assert(sketch != nullptr);
  assert(sketch->featureName == "Sketch1");

  // Create Extrude referencing Sketch by External ID
  CADExchange::Builder::ExtrudeBuilder extrudeBuilder(model, "Extrude1");
  extrudeBuilder.SetProfileByExternalID("SW-101");
  extrudeBuilder.SetDirection(MyVector{0, 0, 1});
  extrudeBuilder.SetDepth(5.0);
  extrudeBuilder.Build();

  auto extrude = std::dynamic_pointer_cast<CADExchange::CExtrude>(
      model.GetFeatures().back());
  assert(extrude != nullptr);
  assert(extrude->sketchProfile == sketch);

  std::cout << "TestExternalID Passed" << std::endl;
}

void TestTypeAdapters() {
  CADExchange::UnifiedModel model;
  CADExchange::Builder::SketchBuilder builder(model, "Sketch2");

  MyPoint p1{0, 0, 0};
  MyPoint p2{10, 10, 0};

  // Directly pass MyPoint
  builder.AddLine(p1, p2);

  builder.Build();

  auto sketch = std::dynamic_pointer_cast<CADExchange::CSketch>(
      model.GetFeatures().back());
  auto line =
      std::dynamic_pointer_cast<CADExchange::CSketchLine>(sketch->segments[0]);

  assert(std::abs(line->endPos.x - 10.0) < 1e-6);

  std::cout << "TestTypeAdapters Passed" << std::endl;
}

void TestValidation() {
  CADExchange::UnifiedModel model;
  CADExchange::Builder::SketchBuilder builder(model, "Sketch3");
  // Intentionally not setting ID (though builder does it automatically)
  // Let's manually corrupt it for test
  auto id = builder.Build();
  auto feature = model.GetFeature(id);
  feature->featureID = ""; // Corrupt ID

  auto report = model.Validate();
  assert(!report.isValid);
  assert(report.errors.size() > 0);

  std::cout << "TestValidation Passed" << std::endl;
}

void TestShow() {
  CADExchange::UnifiedModel model;
  CADExchange::Builder::SketchBuilder builder(model, "SketchShow");
  builder.SetPlane(MyPoint{0, 0, 0}, MyVector{1, 0, 0}, MyVector{0, 0, 1});
  builder.AddCircle(MyPoint{5, 5, 0}, 10.0);

  std::string json = builder.Show();
  std::cout << "Show() Output:\n" << json << std::endl;

  assert(json.find("SketchShow") != std::string::npos);
  assert(json.find("Radius") != std::string::npos);

  std::cout << "TestShow Passed" << std::endl;
}

int main() {
  TestExternalID();
  TestTypeAdapters();
  TestValidation();
  TestShow();
  return 0;
}
