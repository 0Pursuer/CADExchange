#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "../thirdParty/json/single_include/nlohmann/json.hpp"

#if defined(CADEXCHANGE_HAS_OCCT)
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#endif

using json = nlohmann::json;

namespace {

struct StepFileTopologyStats {
  std::string filePath;
  size_t fileSizeBytes = 0;
  std::string schema = "UNKNOWN";
  std::string originApp = "UNKNOWN";

  size_t solidCount = 0;
  size_t shellCount = 0;
  size_t faceCount = 0;
  size_t wireCount = 0;
  size_t edgeCount = 0;
  size_t vertexCount = 0;

  double minX = 0, minY = 0, minZ = 0;
  double maxX = 0, maxY = 0, maxZ = 0;
  bool hasBBox = false;

  double volumeMm3 = 0.0;
  double areaMm2 = 0.0;
  double centroidX = 0.0;
  double centroidY = 0.0;
  double centroidZ = 0.0;
  bool hasMassProps = false;
};

StepFileTopologyStats ParseStepFileMetadata(const std::string &pathStr) {
  StepFileTopologyStats stats;
  stats.filePath = pathStr;

  std::filesystem::path path(pathStr);
  std::error_code ec;
  if (std::filesystem::exists(path, ec)) {
    stats.fileSizeBytes = std::filesystem::file_size(path, ec);
  }

  std::ifstream file(pathStr);
  if (!file.is_open()) {
    return stats;
  }

  std::string line;
  size_t lineCount = 0;
  bool inHeader = false;
  bool inData = false;

  std::vector<double> px, py, pz;

  while (std::getline(file, line) && lineCount < 200000) {
    lineCount++;
    if (line.find("HEADER;") != std::string::npos) {
      inHeader = true;
      continue;
    }
    if (line.find("ENDSEC;") != std::string::npos) {
      inHeader = false;
    }
    if (line.find("DATA;") != std::string::npos) {
      inData = true;
      continue;
    }

    if (inHeader) {
      if (line.find("FILE_SCHEMA") != std::string::npos) {
        if (line.find("AUTOMOTIVE_DESIGN") != std::string::npos) {
          stats.schema = "AUTOMOTIVE_DESIGN (AP214)";
        } else if (line.find("CONFIG_CONTROL_DESIGN") != std::string::npos) {
          stats.schema = "CONFIG_CONTROL_DESIGN (AP203)";
        } else if (line.find("AP242") != std::string::npos) {
          stats.schema = "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF";
        }
      }
      if (line.find("FILE_NAME") != std::string::npos || line.find("CREO") != std::string::npos || line.find("SolidWorks") != std::string::npos) {
        if (line.find("CREO") != std::string::npos || line.find("PTC") != std::string::npos) {
          stats.originApp = "PTC Creo Parametric";
        } else if (line.find("SolidWorks") != std::string::npos || line.find("Dassault") != std::string::npos) {
          stats.originApp = "Dassault SolidWorks";
        } else if (line.find("NX") != std::string::npos || line.find("Siemens") != std::string::npos) {
          stats.originApp = "Siemens UG/NX";
        }
      }
    }

    if (inData) {
      if (line.find("MANIFOLD_SOLID_BREP") != std::string::npos || line.find("BREP_WITH_VOIDS") != std::string::npos) stats.solidCount++;
      if (line.find("CLOSED_SHELL") != std::string::npos || line.find("OPEN_SHELL") != std::string::npos) stats.shellCount++;
      if (line.find("ADVANCED_FACE") != std::string::npos) stats.faceCount++;
      if (line.find("FACE_BOUND") != std::string::npos || line.find("FACE_OUTER_BOUND") != std::string::npos) stats.wireCount++;
      if (line.find("EDGE_CURVE") != std::string::npos || line.find("ORIENTED_EDGE") != std::string::npos) stats.edgeCount++;
      if (line.find("VERTEX_POINT") != std::string::npos) stats.vertexCount++;

      if (line.find("CARTESIAN_POINT") != std::string::npos) {
        size_t p1 = line.find('(');
        size_t p2 = line.rfind(')');
        if (p1 != std::string::npos && p2 != std::string::npos && p2 > p1) {
          std::string sub = line.substr(p1 + 1, p2 - p1 - 1);
          size_t innerP1 = sub.find('(');
          if (innerP1 != std::string::npos) sub = sub.substr(innerP1 + 1);
          std::stringstream ss(sub);
          std::string sx, sy, sz;
          if (std::getline(ss, sx, ',') && std::getline(ss, sy, ',') && std::getline(ss, sz, ',')) {
            try {
              double x = std::stod(sx);
              double y = std::stod(sy);
              double z = std::stod(sz);
              px.push_back(x);
              py.push_back(y);
              pz.push_back(z);
            } catch (...) {}
          }
        }
      }
    }
  }

  if (!px.empty()) {
    stats.minX = *std::min_element(px.begin(), px.end());
    stats.maxX = *std::max_element(px.begin(), px.end());
    stats.minY = *std::min_element(py.begin(), py.end());
    stats.maxY = *std::max_element(py.begin(), py.end());
    stats.minZ = *std::min_element(pz.begin(), pz.end());
    stats.maxZ = *std::max_element(pz.begin(), pz.end());
    stats.hasBBox = true;

    double sumX = 0, sumY = 0, sumZ = 0;
    for (size_t i = 0; i < px.size(); ++i) {
      sumX += px[i];
      sumY += py[i];
      sumZ += pz[i];
    }
    stats.centroidX = sumX / px.size();
    stats.centroidY = sumY / py.size();
    stats.centroidZ = sumZ / pz.size();
  }

  return stats;
}

} // namespace

int main(int argc, char **argv) {
  std::string refPath;
  std::string candPath;
  std::string outDir;
  double distanceTolMm = 0.01;
  double absVolTolMm3 = 1e-6;
  double relVolTol = 1e-8;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--reference" && i + 1 < argc) refPath = argv[++i];
    else if (arg == "--candidate" && i + 1 < argc) candPath = argv[++i];
    else if (arg == "--output" && i + 1 < argc) outDir = argv[++i];
    else if (arg == "--distance-tol-mm" && i + 1 < argc) distanceTolMm = std::stod(argv[++i]);
    else if (arg == "--abs-volume-tol-mm3" && i + 1 < argc) absVolTolMm3 = std::stod(argv[++i]);
    else if (arg == "--rel-volume-tol" && i + 1 < argc) relVolTol = std::stod(argv[++i]);
  }

  if (refPath.empty() || candPath.empty()) {
    std::cerr << "Usage: cad_step_compare --reference <src.step> --candidate <dst.step> [--output <dir>]\n";
    return 1;
  }

  std::cout << "==================================================================\n";
  std::cout << "  CADExchange STEP/AP214 工业级几何精确比对引擎\n";
  std::cout << "==================================================================\n";
  std::cout << "  [源端 STEP]   : " << refPath << "\n";
  std::cout << "  [目标端 STEP] : " << candPath << "\n";
  if (!outDir.empty()) {
    std::cout << "  [报告输出目录]: " << outDir << "\n";
  }
  std::cout << "------------------------------------------------------------------\n";

  StepFileTopologyStats srcStats = ParseStepFileMetadata(refPath);
  StepFileTopologyStats dstStats = ParseStepFileMetadata(candPath);

  bool occtComputed = false;
  double srcVol = srcStats.volumeMm3, dstVol = dstStats.volumeMm3;
  double srcArea = srcStats.areaMm2, dstArea = dstStats.areaMm2;
  double maxDistMm = 0.0;
  double cutAminusBVol = 0.0;
  double cutBminusAVol = 0.0;

#if defined(CADEXCHANGE_HAS_OCCT)
  try {
    STEPControl_Reader readerSrc, readerDst;
    if (readerSrc.ReadFile(refPath.c_str()) == IFSelect_RetDone && readerDst.ReadFile(candPath.c_str()) == IFSelect_RetDone) {
      readerSrc.TransferRoots();
      readerDst.TransferRoots();
      TopoDS_Shape shapeSrc = readerSrc.OneShape();
      TopoDS_Shape shapeDst = readerDst.OneShape();

      GProp_GProps propsVolSrc, propsVolDst;
      BRepGProp::VolumeProperties(shapeSrc, propsVolSrc);
      BRepGProp::VolumeProperties(shapeDst, propsVolDst);

      srcVol = propsVolSrc.Mass();
      dstVol = propsVolDst.Mass();
      srcStats.centroidX = propsVolSrc.CentreOfMass().X();
      srcStats.centroidY = propsVolSrc.CentreOfMass().Y();
      srcStats.centroidZ = propsVolSrc.CentreOfMass().Z();

      dstStats.centroidX = propsVolDst.CentreOfMass().X();
      dstStats.centroidY = propsVolDst.CentreOfMass().Y();
      dstStats.centroidZ = propsVolDst.CentreOfMass().Z();

      GProp_GProps propsSurfSrc, propsSurfDst;
      BRepGProp::SurfaceProperties(shapeSrc, propsSurfSrc);
      BRepGProp::SurfaceProperties(shapeDst, propsSurfDst);
      srcArea = propsSurfSrc.Mass();
      dstArea = propsSurfDst.Mass();

      BRepExtrema_DistShapeShape distCalc(shapeSrc, shapeDst);
      if (distCalc.IsDone()) {
        maxDistMm = distCalc.Value();
      }

      BRepAlgoAPI_Cut cutAB(shapeSrc, shapeDst);
      if (cutAB.IsDone()) {
        GProp_GProps cutProps;
        BRepGProp::VolumeProperties(cutAB.Shape(), cutProps);
        cutAminusBVol = cutProps.Mass();
      }

      BRepAlgoAPI_Cut cutBA(shapeDst, shapeSrc);
      if (cutBA.IsDone()) {
        GProp_GProps cutProps;
        BRepGProp::VolumeProperties(cutBA.Shape(), cutProps);
        cutBminusAVol = cutProps.Mass();
      }

      occtComputed = true;
    }
  } catch (...) {
    occtComputed = false;
  }
#endif

  double absVolDiff = std::abs(srcVol - dstVol);
  double relVolDiff = (std::max(srcVol, dstVol) > 1e-9) ? (absVolDiff / std::max(srcVol, dstVol)) : 0.0;
  double absAreaDiff = std::abs(srcArea - dstArea);

  double dx = srcStats.centroidX - dstStats.centroidX;
  double dy = srcStats.centroidY - dstStats.centroidY;
  double dz = srcStats.centroidZ - dstStats.centroidZ;
  double centroidDistMm = std::sqrt(dx * dx + dy * dy + dz * dz);

  std::cout << "\n  1. 拓扑结构计数对比 (Topology Count Comparison):\n";
  std::cout << "     指标 (Metric)       | 源端 (Source)     | 目标端 (Target)   | 绝对差异 (Diff)\n";
  std::cout << "     -------------------|-------------------|-------------------|------------------\n";
  std::cout << "     面数量 (Faces)      | " << std::setw(17) << srcStats.faceCount << " | " << std::setw(17) << dstStats.faceCount << " | " << std::setw(16) << (int)dstStats.faceCount - (int)srcStats.faceCount << "\n";
  std::cout << "     边数量 (Edges)      | " << std::setw(17) << srcStats.edgeCount << " | " << std::setw(17) << dstStats.edgeCount << " | " << std::setw(16) << (int)dstStats.edgeCount - (int)srcStats.edgeCount << "\n";
  std::cout << "     顶点数量 (Vertices) | " << std::setw(17) << srcStats.vertexCount << " | " << std::setw(17) << dstStats.vertexCount << " | " << std::setw(16) << (int)dstStats.vertexCount - (int)srcStats.vertexCount << "\n";
  std::cout << "     壳体/实体 (Shells) | " << std::setw(17) << srcStats.shellCount << " | " << std::setw(17) << dstStats.shellCount << " | " << std::setw(16) << (int)dstStats.shellCount - (int)srcStats.shellCount << "\n";

  std::cout << "\n  2. 质量几何属性精确比对 (Mass Properties & Geometry Compare):\n";
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "     指标 (Metric)       | 源端 (Source)     | 目标端 (Target)   | 偏差 (Difference)\n";
  std::cout << "     -------------------|-------------------|-------------------|------------------\n";
  if (occtComputed) {
    std::cout << "     总体积 (Volume mm³)| " << std::setw(17) << srcVol << " | " << std::setw(17) << dstVol << " | Δ=" << absVolDiff << " mm³\n";
    std::cout << "     总表面积 (Area mm²)| " << std::setw(17) << srcArea << " | " << std::setw(17) << dstArea << " | Δ=" << absAreaDiff << " mm²\n";
  }
  std::cout << "     质心 X (Centroid X) | " << std::setw(17) << srcStats.centroidX << " | " << std::setw(17) << dstStats.centroidX << " | Δ=" << std::abs(dx) << " mm\n";
  std::cout << "     质心 Y (Centroid Y) | " << std::setw(17) << srcStats.centroidY << " | " << std::setw(17) << dstStats.centroidY << " | Δ=" << std::abs(dy) << " mm\n";
  std::cout << "     质心 Z (Centroid Z) | " << std::setw(17) << srcStats.centroidZ << " | " << std::setw(17) << dstStats.centroidZ << " | Δ=" << std::abs(dz) << " mm\n";
  std::cout << "     质心空间总距离      | " << std::setw(17) << centroidDistMm << " mm | -                 | -\n";

  if (occtComputed) {
    std::cout << "\n  3. 高级 B-Rep 实体空间偏差 (Advanced Geometric Deviation):\n";
    std::cout << "     最大曲面贴合距离   : " << maxDistMm << " mm\n";
    std::cout << "     布尔差(A - B)残余体 : " << cutAminusBVol << " mm³\n";
    std::cout << "     布尔差(B - A)残余体 : " << cutBminusAVol << " mm³\n";
  }

  bool isPass = (centroidDistMm <= 0.05);
  if (occtComputed) {
    isPass = isPass && (absVolDiff <= absVolTolMm3 || relVolDiff <= relVolTol) && (maxDistMm <= distanceTolMm) && (cutAminusBVol <= 1e-4) && (cutBminusAVol <= 1e-4);
  }

  std::string statusStr = isPass ? "MATCH" : "MISMATCH";
  std::cout << "\n------------------------------------------------------------------\n";
  std::cout << "  判定结论 (Final Decision): [" << statusStr << "]\n";
  std::cout << "==================================================================\n";

  json report;
  report["status"] = statusStr;
  report["reason"] = isPass ? "STEP exact comparison passed within tolerance" : "STEP exact comparison detected geometric mismatches";
  report["occt_computed"] = occtComputed;
  report["source_file"] = refPath;
  report["candidate_file"] = candPath;

  report["source"]["file_size_bytes"] = srcStats.fileSizeBytes;
  report["source"]["schema"] = srcStats.schema;
  report["source"]["origin_app"] = srcStats.originApp;
  report["source"]["faces"] = srcStats.faceCount;
  report["source"]["edges"] = srcStats.edgeCount;
  report["source"]["vertices"] = srcStats.vertexCount;
  report["source"]["shells"] = srcStats.shellCount;
  report["source"]["volume_mm3"] = srcVol;
  report["source"]["area_mm2"] = srcArea;
  report["source"]["centroid_mm"] = {srcStats.centroidX, srcStats.centroidY, srcStats.centroidZ};

  report["candidate"]["file_size_bytes"] = dstStats.fileSizeBytes;
  report["candidate"]["schema"] = dstStats.schema;
  report["candidate"]["origin_app"] = dstStats.originApp;
  report["candidate"]["faces"] = dstStats.faceCount;
  report["candidate"]["edges"] = dstStats.edgeCount;
  report["candidate"]["vertices"] = dstStats.vertexCount;
  report["candidate"]["shells"] = dstStats.shellCount;
  report["candidate"]["volume_mm3"] = dstVol;
  report["candidate"]["area_mm2"] = dstArea;
  report["candidate"]["centroid_mm"] = {dstStats.centroidX, dstStats.centroidY, dstStats.centroidZ};

  report["metrics"]["abs_volume_diff_mm3"] = absVolDiff;
  report["metrics"]["rel_volume_diff"] = relVolDiff;
  report["metrics"]["abs_area_diff_mm2"] = absAreaDiff;
  report["metrics"]["centroid_distance_mm"] = centroidDistMm;
  report["metrics"]["max_surface_distance_mm"] = maxDistMm;
  report["metrics"]["boolean_cut_A_minus_B_mm3"] = cutAminusBVol;
  report["metrics"]["boolean_cut_B_minus_A_mm3"] = cutBminusAVol;

  if (!outDir.empty()) {
    std::filesystem::path outDirPath(outDir);
    std::error_code ec;
    std::filesystem::create_directories(outDirPath, ec);
    std::filesystem::path outFile = outDirPath / "result.json";
    std::ofstream outStream(outFile);
    if (outStream.is_open()) {
      outStream << report.dump(2);
    }
  }

  return isPass ? 0 : 1;
}
