#include "StepCompare.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_PrintCount.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pnt.hxx>
#include <json/single_include/nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <system_error>

namespace cadstep {
namespace {

using json = nlohmann::ordered_json;

enum class LoadClass {
  Ready,
  Invalid,
  Unsupported,
  InternalError,
};

struct LoadedSolid {
  LoadClass classification = LoadClass::InternalError;
  TopoDS_Solid solid;
  InputAudit audit;
  std::string reason;
};

std::string PathText(const std::filesystem::path &path) {
#ifdef _WIN32
  const auto utf8 = path.u8string();
  return std::string(utf8.begin(), utf8.end());
#else
  return path.string();
#endif
}

const char *ReadStatusText(IFSelect_ReturnStatus status) {
  switch (status) {
  case IFSelect_RetVoid:
    return "void";
  case IFSelect_RetDone:
    return "done";
  case IFSelect_RetError:
    return "error";
  case IFSelect_RetFail:
    return "fail";
  case IFSelect_RetStop:
    return "stop";
  }
  return "unknown";
}

int CountSubShapes(const TopoDS_Shape &shape, TopAbs_ShapeEnum type) {
  int count = 0;
  for (TopExp_Explorer explorer(shape, type); explorer.More();
       explorer.Next()) {
    ++count;
  }
  return count;
}

bool AllShellsClosed(const TopoDS_Solid &solid) {
  bool foundShell = false;
  for (TopExp_Explorer explorer(solid, TopAbs_SHELL); explorer.More();
       explorer.Next()) {
    foundShell = true;
    if (!BRep_Tool::IsClosed(TopoDS::Shell(explorer.Current()))) {
      return false;
    }
  }
  return foundShell;
}

std::vector<std::string>
ToStrings(const TColStd_SequenceOfAsciiString &values) {
  std::vector<std::string> result;
  result.reserve(static_cast<std::size_t>(values.Length()));
  for (Standard_Integer index = 1; index <= values.Length(); ++index) {
    result.emplace_back(values.Value(index).ToCString());
  }
  return result;
}

Point3 ToPoint3(const gp_Pnt &point) {
  return Point3{point.X(), point.Y(), point.Z()};
}

Bounds3 ComputeBounds(const TopoDS_Shape &shape) {
  Bnd_Box box;
  BRepBndLib::AddOptimal(shape, box, Standard_False, Standard_False);

  Bounds3 result;
  result.isVoid = box.IsVoid();
  if (!result.isVoid) {
    Standard_Real xMin = 0.0;
    Standard_Real yMin = 0.0;
    Standard_Real zMin = 0.0;
    Standard_Real xMax = 0.0;
    Standard_Real yMax = 0.0;
    Standard_Real zMax = 0.0;
    box.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    result.minimum = Point3{xMin, yMin, zMin};
    result.maximum = Point3{xMax, yMax, zMax};
  }
  return result;
}

void ComputeProperties(const TopoDS_Solid &solid, InputAudit &audit) {
  GProp_GProps volumeProperties;
  BRepGProp::VolumeProperties(solid, volumeProperties, Standard_True,
                              Standard_False, Standard_False);
  audit.signedVolumeMm3 = volumeProperties.Mass();
  audit.centroidMm = ToPoint3(volumeProperties.CentreOfMass());

  GProp_GProps surfaceProperties;
  BRepGProp::SurfaceProperties(solid, surfaceProperties, Standard_False,
                               Standard_False);
  audit.surfaceAreaMm2 = surfaceProperties.Mass();
  audit.boundsMm = ComputeBounds(solid);
}

LoadedSolid LoadSingleSolid(const std::filesystem::path &path) {
  LoadedSolid loaded;
  loaded.audit.path = PathText(path);

  std::error_code pathError;
  if (!std::filesystem::exists(path, pathError) || pathError) {
    loaded.classification = LoadClass::InternalError;
    loaded.reason = "STEP path does not exist: " + loaded.audit.path;
    return loaded;
  }
  if (!std::filesystem::is_regular_file(path, pathError) || pathError) {
    loaded.classification = LoadClass::InternalError;
    loaded.reason = "STEP path is not a regular file: " + loaded.audit.path;
    return loaded;
  }

  STEPControl_Reader reader;
  const std::string pathUtf8 = PathText(path);
  const IFSelect_ReturnStatus readStatus = reader.ReadFile(pathUtf8.c_str());
  std::ostringstream loadDiagnostics;
  loadDiagnostics << "ReadFile status=" << ReadStatusText(readStatus);
  if (readStatus == IFSelect_RetDone) {
    reader.PrintCheckLoad(loadDiagnostics, Standard_False,
                          IFSelect_ItemsByEntity);
  }
  loaded.audit.loadDiagnostics = loadDiagnostics.str();
  if (readStatus != IFSelect_RetDone) {
    loaded.classification = LoadClass::Invalid;
    loaded.reason = "OCCT could not read STEP file: " + pathUtf8;
    return loaded;
  }

  TColStd_SequenceOfAsciiString lengthUnits;
  TColStd_SequenceOfAsciiString angleUnits;
  TColStd_SequenceOfAsciiString solidAngleUnits;
  reader.FileUnits(lengthUnits, angleUnits, solidAngleUnits);
  loaded.audit.fileLengthUnits = ToStrings(lengthUnits);

  // OCCT expresses the system length unit in millimetres. 1.0 therefore
  // normalizes every transferred STEP shape to millimetres.
  reader.SetSystemLengthUnit(1.0);
  const Standard_Integer transferredRoots = reader.TransferRoots();
  std::ostringstream transferDiagnostics;
  transferDiagnostics << "transferred_roots=" << transferredRoots;
  reader.PrintCheckTransfer(transferDiagnostics, Standard_False,
                            IFSelect_ItemsByEntity);
  loaded.audit.transferDiagnostics = transferDiagnostics.str();
  if (transferredRoots <= 0) {
    loaded.classification = LoadClass::Invalid;
    loaded.reason = "STEP roots could not be transferred: " + pathUtf8;
    return loaded;
  }

  const TopoDS_Shape transferred = reader.OneShape();
  if (transferred.IsNull()) {
    loaded.classification = LoadClass::Invalid;
    loaded.reason = "STEP transfer produced an empty shape: " + pathUtf8;
    return loaded;
  }

  loaded.audit.solidCount = CountSubShapes(transferred, TopAbs_SOLID);
  loaded.audit.shellCount = CountSubShapes(transferred, TopAbs_SHELL);
  loaded.audit.faceCount = CountSubShapes(transferred, TopAbs_FACE);
  loaded.audit.edgeCount = CountSubShapes(transferred, TopAbs_EDGE);

  if (loaded.audit.solidCount != 1) {
    loaded.classification = LoadClass::Unsupported;
    loaded.reason = "MVP requires exactly one solid; found " +
                    std::to_string(loaded.audit.solidCount) + " in " + pathUtf8;
    return loaded;
  }

  TopExp_Explorer solidExplorer(transferred, TopAbs_SOLID);
  loaded.solid = TopoDS::Solid(solidExplorer.Current());
  const bool containsOnlyTheSolid =
      loaded.audit.shellCount == CountSubShapes(loaded.solid, TopAbs_SHELL) &&
      loaded.audit.faceCount == CountSubShapes(loaded.solid, TopAbs_FACE) &&
      loaded.audit.edgeCount == CountSubShapes(loaded.solid, TopAbs_EDGE) &&
      CountSubShapes(transferred, TopAbs_VERTEX) ==
          CountSubShapes(loaded.solid, TopAbs_VERTEX);
  if (!containsOnlyTheSolid) {
    loaded.classification = LoadClass::Unsupported;
    loaded.reason =
        "MVP does not accept geometry outside the single solid: " + pathUtf8;
    return loaded;
  }

  loaded.audit.brepValid =
      BRepCheck_Analyzer(loaded.solid, Standard_True).IsValid();
  loaded.audit.closed = AllShellsClosed(loaded.solid);
  if (!loaded.audit.closed) {
    loaded.classification = LoadClass::Unsupported;
    loaded.reason = "MVP requires a closed solid: " + pathUtf8;
    return loaded;
  }
  if (!loaded.audit.brepValid) {
    loaded.classification = LoadClass::Invalid;
    loaded.reason = "OCCT B-Rep validation failed: " + pathUtf8;
    return loaded;
  }

  ComputeProperties(loaded.solid, loaded.audit);
  if (!std::isfinite(loaded.audit.signedVolumeMm3) ||
      loaded.audit.signedVolumeMm3 <= 0.0 || loaded.audit.boundsMm.isVoid) {
    loaded.classification = LoadClass::Invalid;
    loaded.reason =
        "solid is empty or has an invalid/reversed volume orientation: " +
        pathUtf8;
    return loaded;
  }

  loaded.classification = LoadClass::Ready;
  return loaded;
}

double PointDistance(const Point3 &left, const Point3 &right) {
  return std::hypot(std::hypot(left.x - right.x, left.y - right.y),
                    left.z - right.z);
}

double MaximumBoundsDifference(const Bounds3 &left, const Bounds3 &right) {
  if (left.isVoid || right.isVoid) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max({std::abs(left.minimum.x - right.minimum.x),
                   std::abs(left.minimum.y - right.minimum.y),
                   std::abs(left.minimum.z - right.minimum.z),
                   std::abs(left.maximum.x - right.maximum.x),
                   std::abs(left.maximum.y - right.maximum.y),
                   std::abs(left.maximum.z - right.maximum.z)});
}

DifferenceAudit Cut(const TopoDS_Solid &argument, const TopoDS_Solid &tool) {
  DifferenceAudit result;
  BRepAlgoAPI_Cut cut;
  TopTools_ListOfShape arguments;
  TopTools_ListOfShape tools;
  arguments.Append(argument);
  tools.Append(tool);
  cut.SetArguments(arguments);
  cut.SetTools(tools);
  cut.Build();

  std::ostringstream report;
  if (cut.HasErrors()) {
    cut.DumpErrors(report);
  }
  if (cut.HasWarnings()) {
    cut.DumpWarnings(report);
  }
  result.report = report.str();
  result.succeeded = !cut.HasErrors() && cut.IsDone();
  if (!result.succeeded) {
    return result;
  }

  const TopoDS_Shape difference = cut.Shape();
  if (difference.IsNull()) {
    return result;
  }

  result.componentCount = CountSubShapes(difference, TopAbs_SOLID);
  if (result.componentCount > 0) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(difference, properties, Standard_True,
                                Standard_False, Standard_False);
    result.volumeMm3 = std::abs(properties.Mass());
  }
  return result;
}

json PointJson(const Point3 &point) {
  return {{"x", point.x}, {"y", point.y}, {"z", point.z}};
}

json BoundsJson(const Bounds3 &bounds) {
  return {{"is_void", bounds.isVoid},
          {"minimum_mm", PointJson(bounds.minimum)},
          {"maximum_mm", PointJson(bounds.maximum)}};
}

json InputJson(const InputAudit &audit) {
  return {{"path", audit.path},
          {"file_length_units", audit.fileLengthUnits},
          {"load_diagnostics", audit.loadDiagnostics},
          {"transfer_diagnostics", audit.transferDiagnostics},
          {"solid_count", audit.solidCount},
          {"shell_count", audit.shellCount},
          {"face_count", audit.faceCount},
          {"edge_count", audit.edgeCount},
          {"brep_valid", audit.brepValid},
          {"closed", audit.closed},
          {"signed_volume_mm3", audit.signedVolumeMm3},
          {"surface_area_mm2", audit.surfaceAreaMm2},
          {"centroid_mm", PointJson(audit.centroidMm)},
          {"bounds", BoundsJson(audit.boundsMm)}};
}

json DifferenceJson(const DifferenceAudit &audit) {
  return {{"succeeded", audit.succeeded},
          {"volume_mm3", audit.volumeMm3},
          {"component_count", audit.componentCount},
          {"kernel_report", audit.report}};
}

CompareStatus FailedLoadStatus(LoadClass classification) {
  switch (classification) {
  case LoadClass::Invalid:
    return CompareStatus::InvalidInput;
  case LoadClass::Unsupported:
    return CompareStatus::UnsupportedShape;
  case LoadClass::InternalError:
    return CompareStatus::InternalError;
  case LoadClass::Ready:
    break;
  }
  return CompareStatus::InternalError;
}

} // namespace

CompareResult CompareStepFiles(const std::filesystem::path &reference,
                               const std::filesystem::path &candidate,
                               const CompareConfig &config) {
  CompareResult result;
  result.thresholds = config;
  result.reference.path = PathText(reference);
  result.candidate.path = PathText(candidate);

  if (!std::isfinite(config.distanceToleranceMm) ||
      !std::isfinite(config.absoluteVolumeToleranceMm3) ||
      !std::isfinite(config.relativeVolumeTolerance) ||
      config.distanceToleranceMm < 0.0 ||
      config.absoluteVolumeToleranceMm3 < 0.0 ||
      config.relativeVolumeTolerance < 0.0) {
    result.status = CompareStatus::InternalError;
    result.reason = "comparison tolerances must be finite and non-negative";
    return result;
  }

  try {
    LoadedSolid referenceSolid = LoadSingleSolid(reference);
    LoadedSolid candidateSolid = LoadSingleSolid(candidate);
    result.reference = referenceSolid.audit;
    result.candidate = candidateSolid.audit;

    if (referenceSolid.classification != LoadClass::Ready) {
      result.status = FailedLoadStatus(referenceSolid.classification);
      result.reason = "reference: " + referenceSolid.reason;
      return result;
    }
    if (candidateSolid.classification != LoadClass::Ready) {
      result.status = FailedLoadStatus(candidateSolid.classification);
      result.reason = "candidate: " + candidateSolid.reason;
      return result;
    }

    const double referenceVolume = referenceSolid.audit.signedVolumeMm3;
    const double candidateVolume = candidateSolid.audit.signedVolumeMm3;
    const double volumeScale = std::max(referenceVolume, candidateVolume);

    result.absoluteInputVolumeDifferenceMm3 =
        std::abs(referenceVolume - candidateVolume);
    result.relativeInputVolumeDifference =
        result.absoluteInputVolumeDifferenceMm3 / volumeScale;
    result.centroidDistanceMm = PointDistance(referenceSolid.audit.centroidMm,
                                              candidateSolid.audit.centroidMm);
    result.maximumBoundsDifferenceMm = MaximumBoundsDifference(
        referenceSolid.audit.boundsMm, candidateSolid.audit.boundsMm);

    result.missingMaterial = Cut(referenceSolid.solid, candidateSolid.solid);
    result.addedMaterial = Cut(candidateSolid.solid, referenceSolid.solid);
    if (!result.missingMaterial.succeeded || !result.addedMaterial.succeeded) {
      result.status = CompareStatus::Indeterminate;
      result.reason =
          "OCCT boolean subtraction failed; geometry equality is unknown";
      return result;
    }

    result.symmetricDifferenceVolumeMm3 =
        result.missingMaterial.volumeMm3 + result.addedMaterial.volumeMm3;
    result.symmetricDifferenceRelative =
        result.symmetricDifferenceVolumeMm3 / volumeScale;

    const bool inputVolumeAbsolutePass =
        result.absoluteInputVolumeDifferenceMm3 <=
        config.absoluteVolumeToleranceMm3;
    const bool inputVolumeRelativePass =
        result.relativeInputVolumeDifference <= config.relativeVolumeTolerance;
    const bool centroidPass =
        result.centroidDistanceMm <= config.distanceToleranceMm;
    const bool boundsPass =
        result.maximumBoundsDifferenceMm <= config.distanceToleranceMm;
    const bool symmetricDifferenceAbsolutePass =
        result.symmetricDifferenceVolumeMm3 <=
        config.absoluteVolumeToleranceMm3;
    const bool symmetricDifferenceRelativePass =
        result.symmetricDifferenceRelative <= config.relativeVolumeTolerance;

    if (inputVolumeAbsolutePass && inputVolumeRelativePass && centroidPass &&
        boundsPass && symmetricDifferenceAbsolutePass &&
        symmetricDifferenceRelativePass) {
      result.status = CompareStatus::Equal;
      result.reason =
          "closed solids pass absolute and relative volume, centroid, bounds, "
          "and symmetric difference thresholds";
    } else {
      result.status = CompareStatus::Different;
      std::ostringstream reason;
      reason << "geometry thresholds failed:";
      if (!inputVolumeAbsolutePass) {
        reason << " input_volume_absolute";
      }
      if (!inputVolumeRelativePass) {
        reason << " input_volume_relative";
      }
      if (!centroidPass) {
        reason << " centroid";
      }
      if (!boundsPass) {
        reason << " bounds";
      }
      if (!symmetricDifferenceAbsolutePass) {
        reason << " symmetric_difference_absolute";
      }
      if (!symmetricDifferenceRelativePass) {
        reason << " symmetric_difference_relative";
      }
      result.reason = reason.str();
    }
  } catch (const Standard_Failure &failure) {
    result.status = CompareStatus::InternalError;
    result.reason =
        std::string("OCCT exception: ") +
        (failure.GetMessageString() ? failure.GetMessageString() : "unknown");
  } catch (const std::exception &error) {
    result.status = CompareStatus::InternalError;
    result.reason = std::string("program exception: ") + error.what();
  }
  return result;
}

const char *ToString(CompareStatus status) {
  switch (status) {
  case CompareStatus::Equal:
    return "EQUAL";
  case CompareStatus::Different:
    return "DIFFERENT";
  case CompareStatus::InvalidInput:
    return "INVALID_INPUT";
  case CompareStatus::UnsupportedShape:
    return "UNSUPPORTED_SHAPE";
  case CompareStatus::Indeterminate:
    return "INDETERMINATE";
  case CompareStatus::InternalError:
    return "INTERNAL_ERROR";
  }
  return "INTERNAL_ERROR";
}

int ExitCode(CompareStatus status) {
  switch (status) {
  case CompareStatus::Equal:
    return 0;
  case CompareStatus::Different:
    return 1;
  case CompareStatus::InvalidInput:
  case CompareStatus::UnsupportedShape:
    return 2;
  case CompareStatus::Indeterminate:
    return 3;
  case CompareStatus::InternalError:
    return 4;
  }
  return 4;
}

std::string ToJson(const CompareResult &result) {
  const json root = {
      {"status", ToString(result.status)},
      {"exit_code", ExitCode(result.status)},
      {"reason", result.reason},
      {"thresholds",
       {{"distance_tolerance_mm", result.thresholds.distanceToleranceMm},
        {"absolute_volume_tolerance_mm3",
         result.thresholds.absoluteVolumeToleranceMm3},
        {"relative_volume_tolerance",
         result.thresholds.relativeVolumeTolerance}}},
      {"reference", InputJson(result.reference)},
      {"candidate", InputJson(result.candidate)},
      {"missing_material", DifferenceJson(result.missingMaterial)},
      {"added_material", DifferenceJson(result.addedMaterial)},
      {"decision_metrics",
       {{"absolute_input_volume_difference_mm3",
         result.absoluteInputVolumeDifferenceMm3},
        {"relative_input_volume_difference",
         result.relativeInputVolumeDifference},
        {"centroid_distance_mm", result.centroidDistanceMm},
        {"maximum_bounds_difference_mm", result.maximumBoundsDifferenceMm},
        {"symmetric_difference_volume_mm3",
         result.symmetricDifferenceVolumeMm3},
        {"symmetric_difference_relative",
         result.symmetricDifferenceRelative}}}};
  return root.dump(2) + '\n';
}

bool WriteResultJson(const std::filesystem::path &outputDirectory,
                     const CompareResult &result, std::string &error) {
  try {
    std::filesystem::create_directories(outputDirectory);
    const std::filesystem::path resultPath = outputDirectory / "result.json";
    std::ofstream output(resultPath, std::ios::binary | std::ios::trunc);
    if (!output.is_open()) {
      error = "unable to open result file: " + PathText(resultPath);
      return false;
    }
    output << ToJson(result);
    if (!output.good()) {
      error = "failed to write result file: " + PathText(resultPath);
      return false;
    }
    return true;
  } catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

} // namespace cadstep
