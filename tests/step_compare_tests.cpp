#include "StepCompare.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Builder.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void Expect(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void WriteStep(const TopoDS_Shape &shape, const std::filesystem::path &path) {
  STEPControl_Writer writer;
  Expect(writer.Transfer(shape, STEPControl_AsIs) == IFSelect_RetDone,
         "STEP transfer failed for " + path.string());
  Expect(writer.Write(path.string().c_str()) == IFSelect_RetDone,
         "STEP write failed for " + path.string());
}

void WriteStepWithUnit(const TopoDS_Shape &shape,
                       const std::filesystem::path &path, const char *unit) {
  const std::string previousUnit = Interface_Static::CVal("write.step.unit");
  Expect(Interface_Static::SetCVal("write.step.unit", unit),
         "failed to set STEP output unit");
  try {
    WriteStep(shape, path);
  } catch (...) {
    Interface_Static::SetCVal("write.step.unit", previousUnit.c_str());
    throw;
  }
  Expect(Interface_Static::SetCVal("write.step.unit", previousUnit.c_str()),
         "failed to restore STEP output unit");
}

TopoDS_Shape Translated(const TopoDS_Shape &shape, double xMm) {
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(xMm, 0.0, 0.0));
  return BRepBuilderAPI_Transform(shape, transform, true).Shape();
}

void TestEqualBoxes(const std::filesystem::path &root) {
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 30.0, 40.0).Shape();
  const auto reference = root / "equal_reference.step";
  const auto candidate = root / "equal_candidate.step";
  WriteStep(box, reference);
  WriteStep(box, candidate);

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::Equal,
         "identical boxes must be EQUAL, got " +
             std::string(cadstep::ToString(result.status)));
  Expect(cadstep::ExitCode(result.status) == 0, "EQUAL must use exit code 0");
  Expect(cadstep::ToJson(result).find("\"status\": \"EQUAL\"") !=
             std::string::npos,
         "result JSON must contain the EQUAL status");
}

void TestBooleanConservationMetrics(const std::filesystem::path &root) {
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 30.0, 40.0).Shape();
  const auto reference = root / "conservation_reference.step";
  const auto candidate = root / "conservation_candidate.step";
  WriteStep(box, reference);
  WriteStep(box, candidate);

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, cadstep::CompareConfig{});
  Expect(result.booleanExecuted,
         "conservation fixture must execute boolean comparison");
  Expect(result.booleanConsistency.booleanResultValid,
         "successful boolean cuts must produce valid conservation evidence");
  Expect(result.booleanConsistency.conservationPassed,
         "identical solids must pass boolean conservation");
  Expect(result.booleanConsistency.conservationErrorMm3 <= 1.0e-9,
         "identical solids must have zero conservation error");
}

void TestLikelyEqualExitCode() {
  Expect(cadstep::ExitCode(cadstep::CompareStatus::LikelyEqual) == 3,
         "LIKELY_EQUAL must use exit code 3");
}

void TestInvalidBooleanConservationIsLikelyEqual() {
  cadstep::BooleanConsistencyMetrics consistency;
  consistency.cutReferenceMinusCandidateSucceeded = true;
  consistency.cutCandidateMinusReferenceSucceeded = true;

  const auto status = cadstep::detail::ClassifyClosedSolidComparison(
      true, true, true, true, consistency);
  Expect(status == cadstep::CompareStatus::LikelyEqual,
         "invalid boolean conservation must produce LIKELY_EQUAL");
}

void TestTranslatedBoxIsDifferent(const std::filesystem::path &root) {
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 30.0, 40.0).Shape();
  const auto reference = root / "translated_reference.step";
  const auto candidate = root / "translated_candidate.step";
  WriteStep(box, reference);
  WriteStep(Translated(box, 0.02), candidate);

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::Different,
         "0.02 mm translation must be DIFFERENT");
  Expect(cadstep::ExitCode(result.status) == 1,
         "DIFFERENT must use exit code 1");
  Expect(result.centroidDistanceMm > 0.01,
         "translation must be visible in centroid diagnostics");
}

void TestDimensionChangeIsDifferent(const std::filesystem::path &root) {
  const auto reference = root / "dimension_reference.step";
  const auto candidate = root / "dimension_candidate.step";
  WriteStep(BRepPrimAPI_MakeBox(20.0, 30.0, 40.0).Shape(), reference);
  WriteStep(BRepPrimAPI_MakeBox(20.02, 30.0, 40.0).Shape(), candidate);

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::Different,
         "0.02 mm dimension change must be DIFFERENT");
  Expect(result.symmetricDifferenceVolumeMm3 > 0.0,
         "dimension change must produce a non-zero symmetric difference");
}

void TestRelativeVolumeThresholdIsIndependent(
    const std::filesystem::path &root) {
  const auto reference = root / "relative_volume_reference.step";
  const auto candidate = root / "relative_volume_candidate.step";
  WriteStep(BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape(), reference);
  WriteStep(BRepPrimAPI_MakeBox(1.0000005, 1.0, 1.0).Shape(), candidate);

  cadstep::CompareConfig config;
  config.absoluteVolumeToleranceMm3 = 0.001;
  config.relativeVolumeTolerance = 1.0e-8;

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, config);
  Expect(result.absoluteInputVolumeDifferenceMm3 < 0.001,
         "relative-volume fixture must pass the absolute threshold");
  Expect(result.relativeInputVolumeDifference > 1.0e-8,
         "relative-volume fixture must fail the relative threshold");
  Expect(result.status == cadstep::CompareStatus::Different,
         "relative volume threshold must be enforced independently");
}

void TestAbsoluteVolumeThresholdIsIndependent(
    const std::filesystem::path &root) {
  const auto reference = root / "absolute_volume_reference.step";
  const auto candidate = root / "absolute_volume_candidate.step";
  WriteStep(BRepPrimAPI_MakeBox(1000.0, 1000.0, 1000.0).Shape(), reference);
  WriteStep(BRepPrimAPI_MakeBox(1000.00000001, 1000.0, 1000.0).Shape(),
            candidate);

  cadstep::CompareConfig config;
  config.absoluteVolumeToleranceMm3 = 0.000001;
  config.relativeVolumeTolerance = 0.0;

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, config);
  Expect(result.absoluteInputVolumeDifferenceMm3 > 0.000001,
         "absolute-volume fixture must fail the absolute threshold");
  Expect(result.status == cadstep::CompareStatus::Different,
         "absolute volume threshold must be enforced independently");
}

void TestMissingHoleIsDifferent(const std::filesystem::path &root) {
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(30.0, 30.0, 20.0).Shape();
  const gp_Ax2 holeAxis(gp_Pnt(15.0, 15.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
  const TopoDS_Shape hole =
      BRepPrimAPI_MakeCylinder(holeAxis, 3.0, 20.0).Shape();
  BRepAlgoAPI_Cut cut(box, hole);
  Expect(cut.IsDone(), "failed to construct through-hole test solid");

  const auto reference = root / "hole_reference.step";
  const auto candidate = root / "hole_candidate.step";
  WriteStep(cut.Shape(), reference);
  WriteStep(box, candidate);

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::Different,
         "missing through hole must be DIFFERENT");
  Expect(result.addedMaterial.volumeMm3 > 0.0,
         "missing hole must be reported as added candidate material");
}

void TestSplitPlanarFacesRemainEqual(const std::filesystem::path &root) {
  const TopoDS_Shape referenceBox =
      BRepPrimAPI_MakeBox(20.0, 30.0, 40.0).Shape();
  const TopoDS_Shape left = BRepPrimAPI_MakeBox(10.0, 30.0, 40.0).Shape();
  const TopoDS_Shape right =
      Translated(BRepPrimAPI_MakeBox(10.0, 30.0, 40.0).Shape(), 10.0);
  BRepAlgoAPI_Fuse fuse(left, right);
  Expect(fuse.IsDone(), "failed to construct split-face test solid");

  const auto reference = root / "split_face_reference.step";
  const auto candidate = root / "split_face_candidate.step";
  WriteStep(referenceBox, reference);
  WriteStep(fuse.Shape(), candidate);

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::Equal,
         "same solid with split coplanar faces must be EQUAL");
}

void TestSplitCylinderFacesRemainEqual(const std::filesystem::path &root) {
  constexpr double pi = 3.14159265358979323846;
  const TopoDS_Shape referenceCylinder =
      BRepPrimAPI_MakeCylinder(10.0, 30.0).Shape();
  const TopoDS_Shape firstHalf =
      BRepPrimAPI_MakeCylinder(10.0, 30.0, pi).Shape();
  const gp_Ax2 secondAxis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0),
                          gp_Dir(-1.0, 0.0, 0.0));
  const TopoDS_Shape secondHalf =
      BRepPrimAPI_MakeCylinder(secondAxis, 10.0, 30.0, pi).Shape();
  BRepAlgoAPI_Fuse fuse(firstHalf, secondHalf);
  Expect(fuse.IsDone(), "failed to construct split-cylinder test solid");

  const auto reference = root / "cylinder_reference.step";
  const auto candidate = root / "cylinder_split_candidate.step";
  WriteStep(referenceCylinder, reference);
  WriteStep(fuse.Shape(), candidate);

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::Equal,
         "same cylinder with split periodic faces must be EQUAL");
  Expect(result.candidate.faceCount > result.reference.faceCount,
         "split-cylinder fixture must retain additional candidate faces");
}

void TestEquivalentMillimetreAndInchSteps(const std::filesystem::path &root) {
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(25.4, 50.8, 76.2).Shape();
  const auto millimetreStep = root / "unit_mm.step";
  const auto inchStep = root / "unit_inch.step";
  WriteStepWithUnit(box, millimetreStep, "MM");
  WriteStepWithUnit(box, inchStep, "INCH");

  const auto result = cadstep::CompareStepFiles(millimetreStep, inchStep,
                                                cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::Equal,
         "equivalent MM and INCH STEP solids must be EQUAL");
  Expect(!result.reference.fileLengthUnits.empty() &&
             !result.candidate.fileLengthUnits.empty(),
         "STEP audits must report file length units");
}

void TestMalformedStepIsInvalid(const std::filesystem::path &root) {
  const auto malformed = root / "malformed.step";
  const auto candidate = root / "malformed_candidate.step";
  {
    std::ofstream output(malformed, std::ios::binary | std::ios::trunc);
    output << "this is not a STEP file\n";
  }
  WriteStep(BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(), candidate);

  const auto result =
      cadstep::CompareStepFiles(malformed, candidate, cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::InvalidInput,
         "malformed STEP must be INVALID_INPUT");
  Expect(cadstep::ExitCode(result.status) == 2,
         "INVALID_INPUT must use exit code 2");
}

void TestMultipleSolidsAreUnsupported(const std::filesystem::path &root) {
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  builder.Add(compound, BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape());
  builder.Add(compound,
              Translated(BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape(), 20.0));

  const auto reference = root / "multi_solid.step";
  const auto candidate = root / "single_solid.step";
  WriteStep(compound, reference);
  WriteStep(BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(), candidate);

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::UnsupportedShape,
         "multiple solids must be UNSUPPORTED_SHAPE");
  Expect(cadstep::ExitCode(result.status) == 2,
         "UNSUPPORTED_SHAPE must use exit code 2");
}

void TestSolidWithFreeCurveIsUnsupported(const std::filesystem::path &root) {
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  builder.Add(compound, BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape());
  builder.Add(compound, BRepBuilderAPI_MakeEdge(gp_Pnt(20.0, 0.0, 0.0),
                                                gp_Pnt(25.0, 0.0, 0.0))
                            .Shape());

  const auto reference = root / "solid_with_curve.step";
  const auto candidate = root / "solid_without_curve.step";
  WriteStep(compound, reference);
  WriteStep(BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(), candidate);

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::UnsupportedShape,
         "solid plus free curve must be UNSUPPORTED_SHAPE");
}

void TestSurfaceModelIsUnsupported(const std::filesystem::path &root) {
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
  TopExp_Explorer faceExplorer(box, TopAbs_FACE);
  Expect(faceExplorer.More(), "box has no face");
  const TopoDS_Face face = TopoDS::Face(faceExplorer.Current());

  const auto reference = root / "surface.step";
  const auto candidate = root / "surface_candidate.step";
  WriteStep(face, reference);
  WriteStep(face, candidate);

  const auto result =
      cadstep::CompareStepFiles(reference, candidate, cadstep::CompareConfig{});
  Expect(result.status == cadstep::CompareStatus::UnsupportedShape,
         "surface-only STEP must be UNSUPPORTED_SHAPE");
}

void TestBooleanFuzzyToleranceOption(const std::filesystem::path &root) {
  const TopoDS_Shape box1 = BRepPrimAPI_MakeBox(20.0, 30.0, 40.0).Shape();
  const TopoDS_Shape box2 = Translated(box1, 0.005);
  const auto reference = root / "fuzzy_reference.step";
  const auto candidate = root / "fuzzy_candidate.step";
  WriteStep(box1, reference);
  WriteStep(box2, candidate);

  cadstep::CompareConfig config;
  config.distanceToleranceMm = 0.01;
  config.booleanFuzzyToleranceMm = 0.02;

  const auto result = cadstep::CompareStepFiles(reference, candidate, config);
  Expect(result.status == cadstep::CompareStatus::Equal,
         "small translation within fuzzy tolerance must be EQUAL");
}

void TestSameDomainNormalizationAndFastPath(const std::filesystem::path &root) {
  const TopoDS_Shape referenceBox =
      BRepPrimAPI_MakeBox(20.0, 30.0, 40.0).Shape();
  const TopoDS_Shape left = BRepPrimAPI_MakeBox(10.0, 30.0, 40.0).Shape();
  const TopoDS_Shape right =
      Translated(BRepPrimAPI_MakeBox(10.0, 30.0, 40.0).Shape(), 10.0);
  BRepAlgoAPI_Fuse fuse(left, right);
  Expect(fuse.IsDone(), "failed to construct split-face test solid");

  const auto reference = root / "norm_reference.step";
  const auto candidate = root / "norm_candidate.step";
  WriteStep(referenceBox, reference);
  WriteStep(fuse.Shape(), candidate);

  cadstep::CompareConfig config;
  config.enableSameDomainNormalization = true;
  config.enableNormalizedFastPath = true;

  const auto result = cadstep::CompareStepFiles(reference, candidate, config);
  Expect(result.status == cadstep::CompareStatus::Equal,
         "normalization + fast path must yield EQUAL");
  Expect(result.referenceNormalization.succeeded,
         "reference normalization must succeed");
  Expect(result.candidateNormalization.succeeded,
         "candidate normalization must succeed");
  Expect(result.candidateNormalization.faceCountAfter == 6,
         "split-box after normalization must have 6 faces");
  Expect(result.normalizedTopology.faces.matchedCount == 6,
         "all 6 normalized faces must match 1-to-1");
  Expect(result.normalizedTopology.faces.unmatchedReferenceIds.empty(),
         "unmatched reference faces count must be 0");
  Expect(result.normalizedTopology.faces.unmatchedCandidateIds.empty(),
         "unmatched candidate faces count must be 0");
  Expect(result.normalizedTopology.faces.allMatched,
         "allFacesMatched flag must be true");
  Expect(result.normalizedTopology.edges.allMatched,
         "allEdgesMatched flag must be true");
  Expect(result.normalizedTopology.normalizedTopologyMatch,
         "normalized topology must match");
  Expect(result.decisionPath == "normalized_topology_fast_path",
         "fast-path decision path must be selected");
  Expect(!result.booleanExecuted,
         "boolean cut must be skipped on fast-path");
}

void TestMultiSolidBoxesEqual(const std::filesystem::path &root) {
  const TopoDS_Shape boxA = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
  const TopoDS_Shape boxB = Translated(BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape(), 50.0);

  BRep_Builder builder;
  TopoDS_Compound compRef;
  builder.MakeCompound(compRef);
  builder.Add(compRef, boxA);
  builder.Add(compRef, boxB);

  TopoDS_Compound compCand;
  builder.MakeCompound(compCand);
  builder.Add(compCand, boxA);
  builder.Add(compCand, boxB);

  const auto reference = root / "multisolid_ref.step";
  const auto candidate = root / "multisolid_cand.step";
  WriteStep(compRef, reference);
  WriteStep(compCand, candidate);

  cadstep::CompareConfig config;
  config.allowMultipleSolids = true;

  const auto result = cadstep::CompareStepFiles(reference, candidate, config);
  Expect(result.status == cadstep::CompareStatus::Equal,
         "multi-solid identical pairs must be EQUAL");
  Expect(result.multiSolid.enabled, "multiSolid.enabled must be true");
  Expect(result.multiSolid.referenceSolidCount == 2, "reference solid count must be 2");
  Expect(result.multiSolid.candidateSolidCount == 2, "candidate solid count must be 2");
  Expect(result.multiSolid.matchedSolidCount == 2, "matched solid count must be 2");
  Expect(result.multiSolid.unmatchedReferenceSolidCount == 0, "unmatched ref count must be 0");
  Expect(result.multiSolid.unmatchedCandidateSolidCount == 0, "unmatched cand count must be 0");
}

void TestMultiSolidBoxesPermutationEqual(const std::filesystem::path &root) {
  const TopoDS_Shape boxA = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
  const TopoDS_Shape boxB = Translated(BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape(), 50.0);

  BRep_Builder builder;
  TopoDS_Compound compRef;
  builder.MakeCompound(compRef);
  builder.Add(compRef, boxA);
  builder.Add(compRef, boxB);

  TopoDS_Compound compCand;
  builder.MakeCompound(compCand);
  builder.Add(compCand, boxB);
  builder.Add(compCand, boxA);

  const auto reference = root / "multisolid_perm_ref.step";
  const auto candidate = root / "multisolid_perm_cand.step";
  WriteStep(compRef, reference);
  WriteStep(compCand, candidate);

  cadstep::CompareConfig config;
  config.allowMultipleSolids = true;

  const auto result = cadstep::CompareStepFiles(reference, candidate, config);
  Expect(result.status == cadstep::CompareStatus::Equal,
         "multi-solid permuted solid order must be EQUAL");
  Expect(result.multiSolid.matchedSolidCount == 2, "matched solid count must be 2");
}

void TestMultiSolidUnmatchedSolidsIsDifferent(const std::filesystem::path &root) {
  const TopoDS_Shape boxA = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
  const TopoDS_Shape boxB = Translated(BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape(), 50.0);

  BRep_Builder builder;
  TopoDS_Compound compRef;
  builder.MakeCompound(compRef);
  builder.Add(compRef, boxA);
  builder.Add(compRef, boxB);

  const auto reference = root / "multisolid_unmatched_ref.step";
  const auto candidate = root / "multisolid_unmatched_cand.step";
  WriteStep(compRef, reference);
  WriteStep(boxA, candidate);

  cadstep::CompareConfig config;
  config.allowMultipleSolids = true;

  const auto result = cadstep::CompareStepFiles(reference, candidate, config);
  Expect(result.status == cadstep::CompareStatus::Different,
         "multi-solid unmatched solid must be DIFFERENT");
  Expect(result.multiSolid.referenceSolidCount == 2, "reference solid count must be 2");
  Expect(result.multiSolid.candidateSolidCount == 1, "candidate solid count must be 1");
  Expect(result.multiSolid.matchedSolidCount == 1, "matched solid count must be 1");
  Expect(result.multiSolid.unmatchedReferenceSolidCount == 1, "unmatched ref count must be 1");
}

} // namespace

int main(int argc, char **argv) {
  try {
    const std::filesystem::path root =
        argc > 1
            ? std::filesystem::path(argv[1])
            : std::filesystem::temp_directory_path() / "cad_step_compare_tests";
    std::filesystem::create_directories(root);

    TestEqualBoxes(root);
    TestTranslatedBoxIsDifferent(root);
    TestBooleanConservationMetrics(root);
    TestLikelyEqualExitCode();
    TestInvalidBooleanConservationIsLikelyEqual();
    TestDimensionChangeIsDifferent(root);
    TestRelativeVolumeThresholdIsIndependent(root);
    TestAbsoluteVolumeThresholdIsIndependent(root);
    TestMissingHoleIsDifferent(root);
    TestSplitPlanarFacesRemainEqual(root);
    TestSplitCylinderFacesRemainEqual(root);
    TestEquivalentMillimetreAndInchSteps(root);
    TestMalformedStepIsInvalid(root);
    TestMultipleSolidsAreUnsupported(root);
    TestSolidWithFreeCurveIsUnsupported(root);
    TestSurfaceModelIsUnsupported(root);
    TestBooleanFuzzyToleranceOption(root);
    TestSameDomainNormalizationAndFastPath(root);
    TestMultiSolidBoxesEqual(root);
    TestMultiSolidBoxesPermutationEqual(root);
    TestMultiSolidUnmatchedSolidsIsDifferent(root);
    std::cout << "cad_step_compare_tests: PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "cad_step_compare_tests: FAIL: " << error.what() << '\n';
    return 1;
  }
}
