#include <iostream>
#include <iomanip>
#include "../service/serialization/CADSerializer.h"
#include "../service/accessors/ModelAccessor.h"
#include "../service/accessors/FeatureAccessor.h"
#include "../service/accessors/ExtrudeAccessor.h"
#include "../service/accessors/RevolveAccessor.h"
#include "../service/accessors/SketchAccessor.h"
#include "../service/accessors/ReferenceAccessor.h"

using namespace CADExchange;
using namespace CADExchange::Accessor;

// ============================================================================
// 辅助输出函数
// ============================================================================

void PrintSeparator(const std::string& title = "") {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    if (!title.empty()) {
        std::cout << "  " << title << std::endl;
        std::cout << std::string(80, '=') << std::endl;
    }
}

void PrintSubseparator(const std::string& title = "") {
    std::cout << "\n" << std::string(80, '-') << std::endl;
    if (!title.empty()) {
        std::cout << "  " << title << std::endl;
        std::cout << std::string(80, '-') << std::endl;
    }
}

// ============================================================================
// 第一部分：加载 XML 并显示模型信息
// ============================================================================

void LoadAndDisplayModelInfo(const std::string& xmlPath) {
    PrintSeparator("第一部分：加载 XML 文件");
    
    std::cout << "加载文件: " << xmlPath << std::endl;
    
    UnifiedModel model;
    std::string errorMsg;
    
    // 使用 CADSerializer 加载 XML (指定使用 TinyXML 格式)
    bool loaded = LoadModel(model, xmlPath, &errorMsg, SerializationFormat::TINYXML);
    
    if (!loaded) {
        std::cerr << "❌ 加载失败: " << errorMsg << std::endl;
        return;
    }
    
    std::cout << "✅ 加载成功!" << std::endl;
    
    // 创建访问器
    ModelAccessor modelAccessor;
    modelAccessor.SetModel(model);
    
    if (!modelAccessor.IsValid()) {
        std::cerr << "❌ 模型无效" << std::endl;
        return;
    }
    
    // 显示模型基本信息
    std::cout << "\n模型信息:" << std::endl;
    std::cout << "  - 特征总数: " << modelAccessor.GetFeatureCount() << std::endl;
}

// ============================================================================
// 第二部分：遍历所有特征并显示其类型
// ============================================================================

void TraverseAndDisplayFeatures(const std::string& xmlPath) {
    PrintSeparator("第二部分：遍历特征");
    
    UnifiedModel model;
    LoadModel(model, xmlPath, nullptr, SerializationFormat::TINYXML);
    
    ModelAccessor modelAccessor;
    modelAccessor.SetModel(model);
    
    std::cout << std::left << std::setw(4) << "序号"
              << std::setw(12) << "特征ID"
              << std::setw(15) << "特征名称"
              << std::setw(12) << "类型"
              << std::setw(12) << "是否抑制"
              << std::endl;
    std::cout << std::string(55, '-') << std::endl;
    
    for (int i = 0; i < modelAccessor.GetFeatureCount(); ++i) {
        auto feat = modelAccessor.GetFeature(i);
        
        if (!feat || !feat->IsValid()) continue;
        
        std::string typeStr;
        std::string extraInfo;

        // 演示：使用 As<T>() 语法糖进行类型判断和转换
        if (auto sketch = feat->As<SketchAccessor>()) {
            typeStr = "Sketch";
            extraInfo = std::to_string(sketch->GetSegmentCount()) + " segs";
        } else if (auto extrude = feat->As<ExtrudeAccessor>()) {
            typeStr = "Extrude";
            extraInfo = "D=" + std::to_string(extrude->GetDepth1());
        } else if (auto revolve = feat->As<RevolveAccessor>()) {
            typeStr = "Revolve";
            extraInfo = "Angle=" + std::to_string(revolve->GetPrimaryAngle());
        } else {
            typeStr = "Other";
        }
        
        std::cout << std::left 
                  << std::setw(4) << (i + 1)
                  << std::setw(12) << feat->GetID()
                  << std::setw(15) << feat->GetName()
                  << std::setw(12) << typeStr
                  << std::setw(12) << (feat->IsSuppressed() ? "Yes" : "No")
                  << extraInfo
                  << std::endl;
    }
}

// ============================================================================
// 第三部分：详细提取草图数据
// ============================================================================

void ExtractSketchData(const std::string& xmlPath) {
    PrintSeparator("第三部分：提取草图数据");
    
    UnifiedModel model;
    LoadModel(model, xmlPath, nullptr, SerializationFormat::TINYXML);
    
    ModelAccessor modelAccessor;
    modelAccessor.SetModel(model);
    
    for (int i = 0; i < modelAccessor.GetFeatureCount(); ++i) {
        auto feat = modelAccessor.GetFeature(i);
        
        if (auto sketch = feat->As<SketchAccessor>()) {
            PrintSubseparator("草图: " + sketch->GetName());
            
            // 获取参考面信息
            std::cout << "参考面信息:" << std::endl;
            if (sketch->HasReferencePlane()) {
                auto refPlane = sketch->GetReferencePlane();
                
                // 检查引用类型并调用相应的方法
                RefType refType = refPlane.GetRefType();
                
                if (refType == RefType::FEATURE_DATUM_PLANE) {
                    // 基准平面
                    CPoint3D origin;
                    CVector3D normal, xDir, yDir;
                    
                    if (refPlane.GetPlaneOrigin(origin)) {
                        std::cout << "  原点: (" << origin.x << ", " << origin.y << ", " 
                                  << origin.z << ")" << std::endl;
                    }
                    
                    if (refPlane.GetPlaneNormal(normal)) {
                        std::cout << "  法向: (" << normal.x << ", " << normal.y << ", " 
                                  << normal.z << ")" << std::endl;
                    }
                    
                    if (refPlane.GetPlaneXDir(xDir)) {
                        std::cout << "  X向: (" << xDir.x << ", " << xDir.y << ", " 
                                  << xDir.z << ")" << std::endl;
                    }
                } else if (refType == RefType::TOPO_FACE) {
                    // 拓扑面
                    CPoint3D centroid;
                    CVector3D normal, uDir, vDir;
                    
                    if (refPlane.GetFaceCentroid(centroid)) {
                        std::cout << "  质心: (" << centroid.x << ", " << centroid.y << ", " 
                                  << centroid.z << ")" << std::endl;
                    }
                    
                    if (refPlane.GetFaceNormal(normal)) {
                        std::cout << "  法向: (" << normal.x << ", " << normal.y << ", " 
                                  << normal.z << ")" << std::endl;
                    }
                    
                    if (refPlane.GetFaceUDir(uDir)) {
                        std::cout << "  U向: (" << uDir.x << ", " << uDir.y << ", " 
                                  << uDir.z << ")" << std::endl;
                    }
                } else {
                    std::cout << "  (未知的引用类型: " << static_cast<int>(refType) << ")" << std::endl;
                }
            } else {
                std::cout << "  无参考面" << std::endl;
            }
            
            // 遍历几何段
            std::cout << "\n几何段信息 (总计 " << sketch->GetSegmentCount() << " 条):" << std::endl;
            
            for (int j = 0; j < sketch->GetSegmentCount(); ++j) {
                auto seg = sketch->GetSegment(j);
                
                if (!seg.IsValid()) continue;
                
                std::cout << "  [段 " << j << "] ";
                std::cout << "ID=" << seg.GetLocalID() << ", ";
                std::cout << "构造线=" << (seg.IsConstruction() ? "是" : "否") << ", ";
                
                switch (seg.GetType()) {
                    case CSketchSeg::SegType::LINE: {
                        // 优化方式：直接访问底层数据结构
                        if (auto line = seg.As<CSketchLine>()) {
                            std::cout << "Line: (" << line->startPos.x << "," << line->startPos.y << ") -> "
                                      << "(" << line->endPos.x << "," << line->endPos.y << ")";
                        }
                        break;
                    }
                    case CSketchSeg::SegType::CIRCLE: {
                        // 优化方式：直接访问底层数据结构
                        if (auto circle = seg.As<CSketchCircle>()) {
                            std::cout << "Circle: Center=(" << circle->center.x << "," << circle->center.y 
                                      << "), R=" << circle->radius;
                        }
                        break;
                    }
                    case CSketchSeg::SegType::ARC: {
                        // 优化方式：直接访问底层数据结构
                        if (auto arc = seg.As<CSketchArc>()) {
                            std::cout << "Arc: C=(" << arc->center.x << "," << arc->center.y 
                                      << "), S=" << arc->startAngle 
                                      << ", E=" << arc->endAngle;
                        }
                        break;
                    }
                    case CSketchSeg::SegType::POINT: {
                        // 优化方式：直接访问底层数据结构
                        if (auto pt = seg.As<CSketchPoint>()) {
                            std::cout << "Point: (" << pt->position.x << "," << pt->position.y << ")";
                        }
                        break;
                    }
                    default:
                        std::cout << "Unknown type";
                }
                std::cout << std::endl;
            }
        }
    }
}

// ============================================================================
// 第四部分：详细提取拉伸数据
// ============================================================================

void ExtractExtrudeData(const std::string& xmlPath) {
    PrintSeparator("第四部分：提取拉伸特征数据");
    
    UnifiedModel model;
    LoadModel(model, xmlPath, nullptr, SerializationFormat::TINYXML);
    
    ModelAccessor modelAccessor;
    modelAccessor.SetModel(model);
    
    for (int i = 0; i < modelAccessor.GetFeatureCount(); ++i) {
        auto feat = modelAccessor.GetFeature(i);
        
        if (auto extrude = feat->As<ExtrudeAccessor>()) {
            PrintSubseparator("拉伸: " + extrude->GetName());
            
            // 核心参数
            std::cout << "核心参数:" << std::endl;
            
            std::string profileID = extrude->GetProfileSketchID();
            std::cout << "  轮廓草图 ID: " << profileID << std::endl;
            
            // 在模型中查找这个草图
            auto profileFeat = modelAccessor.GetFeatureByID(profileID);
            if (profileFeat) {
                std::cout << "  轮廓草图名: " << profileFeat->GetName() << std::endl;
            }
            
            CVector3D dir = extrude->GetDirection();
            std::cout << "  拉伸方向: (" << dir.x << ", " << dir.y << ", " << dir.z << ")" << std::endl;
            
            std::cout << "  操作类型: ";
            switch (extrude->GetOperation()) {
                case BooleanOp::BOSS: std::cout << "BOSS (凸出)"; break;
                case BooleanOp::CUT: std::cout << "CUT (凹陷)"; break;
                case BooleanOp::MERGE: std::cout << "MERGE (合并)"; break;
                default: std::cout << "Unknown";
            }
            std::cout << std::endl;
            
            // 第一方向
            std::cout << "\n第一方向参数:" << std::endl;
            
            auto endType1 = extrude->GetEndType1();
            std::cout << "  端面类型: ";
            switch (endType1) {
                case ExtrudeEndCondition::Type::BLIND:
                    std::cout << "BLIND (指定深度)";
                    break;
                case ExtrudeEndCondition::Type::THROUGH_ALL:
                    std::cout << "THROUGH_ALL (穿透全部)";
                    break;
                case ExtrudeEndCondition::Type::UP_TO_FACE:
                    std::cout << "UP_TO_FACE (至面)";
                    break;
                case ExtrudeEndCondition::Type::UP_TO_VERTEX:
                    std::cout << "UP_TO_VERTEX (至顶点)";
                    break;
                case ExtrudeEndCondition::Type::UP_TO_NEXT:
                    std::cout << "UP_TO_NEXT (至下一面)";
                    break;
                case ExtrudeEndCondition::Type::MID_PLANE:
                    std::cout << "MID_PLANE (中间平面)";
                    break;
                default:
                    std::cout << "Unknown";
            }
            std::cout << std::endl;
            
            // 根据端面类型显示相关参数
            if (endType1 == ExtrudeEndCondition::Type::BLIND || 
                endType1 == ExtrudeEndCondition::Type::MID_PLANE) {
                std::cout << "  深度: " << extrude->GetDepth1() << " mm" << std::endl;
            }
            
            if (extrude->HasOffset1()) {
                std::cout << "  偏移: " << extrude->GetOffset1() << " mm" << std::endl;
            }
            
            if (extrude->IsFlip1()) {
                std::cout << "  反转方向: 是" << std::endl;
            }
            
            if (extrude->IsFlipMaterialSide1()) {
                std::cout << "  反转材料侧: 是" << std::endl;
            }
            
            // 检查第一方向的参考
            auto ref1 = extrude->GetReference1();
            if (endType1 == ExtrudeEndCondition::Type::UP_TO_FACE || 
                endType1 == ExtrudeEndCondition::Type::UP_TO_VERTEX ||
                endType1 == ExtrudeEndCondition::Type::UP_TO_NEXT) {
                
                if (ref1.IsValid()) {
                    std::cout << "  参考实体: 存在" << std::endl;
                    
                    // 检查引用类型并调用相应的方法
                    RefType refType = ref1.GetRefType();
                    
                    if (refType == RefType::TOPO_VERTEX) {
                        CPoint3D pos;
                        if (ref1.GetVertexPosition(pos)) {
                            std::cout << "    顶点位置: (" << pos.x << ", " << pos.y << ", " 
                                      << pos.z << ")" << std::endl;
                        }
                    } else if (refType == RefType::TOPO_FACE) {
                        CPoint3D centroid;
                        CVector3D normal, uDir;
                        
                        if (ref1.GetFaceCentroid(centroid)) {
                            std::cout << "    面质心: (" << centroid.x << ", " << centroid.y << ", " 
                                      << centroid.z << ")" << std::endl;
                        }
                        
                        if (ref1.GetFaceNormal(normal)) {
                            std::cout << "    面法向: (" << normal.x << ", " << normal.y << ", " 
                                      << normal.z << ")" << std::endl;
                        }
                        
                        if (ref1.GetFaceUDir(uDir)) {
                            std::cout << "    面U向: (" << uDir.x << ", " << uDir.y << ", " 
                                      << uDir.z << ")" << std::endl;
                        }
                    } else if (refType == RefType::TOPO_EDGE) {
                        CPoint3D midPoint;
                        if (ref1.GetEdgeMidPoint(midPoint)) {
                            std::cout << "    边中点: (" << midPoint.x << ", " << midPoint.y << ", " 
                                      << midPoint.z << ")" << std::endl;
                        }
                    } else if (refType == RefType::FEATURE_DATUM_PLANE) {
                        CPoint3D origin;
                        CVector3D normal, xDir;
                        
                        if (ref1.GetPlaneOrigin(origin)) {
                            std::cout << "    平面原点: (" << origin.x << ", " << origin.y << ", " 
                                      << origin.z << ")" << std::endl;
                        }
                        
                        if (ref1.GetPlaneNormal(normal)) {
                            std::cout << "    平面法向: (" << normal.x << ", " << normal.y << ", " 
                                      << normal.z << ")" << std::endl;
                        }
                    } else {
                        std::cout << "    (未知的引用类型: " << static_cast<int>(refType) << ")" << std::endl;
                    }
                } else {
                    std::cout << "  参考实体: (无效或未设置)" << std::endl;
                }
            }
            
            // 第二方向（可选）
            if (extrude->HasDirection2()) {
                std::cout << "\n第二方向参数:" << std::endl;
                
                auto endType2 = extrude->GetEndType2();
                std::cout << "  端面类型: ";
                switch (endType2) {
                    case ExtrudeEndCondition::Type::BLIND:
                        std::cout << "BLIND (指定深度)";
                        break;
                    case ExtrudeEndCondition::Type::THROUGH_ALL:
                        std::cout << "THROUGH_ALL (穿透全部)";
                        break;
                    case ExtrudeEndCondition::Type::UP_TO_FACE:
                        std::cout << "UP_TO_FACE (至面)";
                        break;
                    case ExtrudeEndCondition::Type::UP_TO_VERTEX:
                        std::cout << "UP_TO_VERTEX (至顶点)";
                        break;
                    case ExtrudeEndCondition::Type::UP_TO_NEXT:
                        std::cout << "UP_TO_NEXT (至下一面)";
                        break;
                    case ExtrudeEndCondition::Type::MID_PLANE:
                        std::cout << "MID_PLANE (中间平面)";
                        break;
                    default:
                        std::cout << "Other";
                }
                std::cout << std::endl;
                
                // 根据端面类型显示相关参数
                if (endType2 == ExtrudeEndCondition::Type::BLIND || 
                    endType2 == ExtrudeEndCondition::Type::MID_PLANE) {
                    std::cout << "  深度: " << extrude->GetDepth2() << " mm" << std::endl;
                }
                
                if (extrude->HasOffset2()) {
                    std::cout << "  偏移: " << extrude->GetOffset2() << " mm" << std::endl;
                }
                
                if (extrude->IsFlip2()) {
                    std::cout << "  反转方向: 是" << std::endl;
                }
                
                if (extrude->IsFlipMaterialSide2()) {
                    std::cout << "  反转材料侧: 是" << std::endl;
                }
                
                // 检查第二方向的参考
                auto ref2 = extrude->GetReference2();
                if (endType2 == ExtrudeEndCondition::Type::UP_TO_FACE || 
                    endType2 == ExtrudeEndCondition::Type::UP_TO_VERTEX ||
                    endType2 == ExtrudeEndCondition::Type::UP_TO_NEXT) {
                    
                    if (ref2.IsValid()) {
                        std::cout << "  参考实体: 存在" << std::endl;
                        
                        // 检查引用类型并调用相应的方法
                        RefType refType = ref2.GetRefType();
                        
                        if (refType == RefType::TOPO_VERTEX) {
                            CPoint3D pos;
                            if (ref2.GetVertexPosition(pos)) {
                                std::cout << "    顶点位置: (" << pos.x << ", " << pos.y << ", " 
                                          << pos.z << ")" << std::endl;
                            }
                        } else if (refType == RefType::TOPO_FACE) {
                            CPoint3D centroid;
                            CVector3D normal;
                            
                            if (ref2.GetFaceCentroid(centroid)) {
                                std::cout << "    面质心: (" << centroid.x << ", " << centroid.y << ", " 
                                          << centroid.z << ")" << std::endl;
                            }
                            
                            if (ref2.GetFaceNormal(normal)) {
                                std::cout << "    面法向: (" << normal.x << ", " << normal.y << ", " 
                                          << normal.z << ")" << std::endl;
                            }
                        } else if (refType == RefType::TOPO_EDGE) {
                            CPoint3D midPoint;
                            if (ref2.GetEdgeMidPoint(midPoint)) {
                                std::cout << "    边中点: (" << midPoint.x << ", " << midPoint.y << ", " 
                                          << midPoint.z << ")" << std::endl;
                            }
                        }
                    } else {
                        std::cout << "  参考实体: (无效或未设置)" << std::endl;
                    }
                }
            }
            
            // 可选参数：拔模
            if (extrude->HasDraft()) {
                std::cout << "\n拔模参数:" << std::endl;
                std::cout << "  拔模角: " << extrude->GetDraftAngle() << "°" << std::endl;
            }
            
            // 可选参数：薄壁
            if (extrude->HasThinWall()) {
                std::cout << "\n薄壁参数:" << std::endl;
                std::cout << "  厚度: " << extrude->GetThinWallThickness() << " mm" << std::endl;
            }
        }
    }
}

// ============================================================================
// 第五部分：分析特征依赖关系
// ============================================================================

void AnalyzeDependencies(const std::string& xmlPath) {
    PrintSeparator("第五部分：特征依赖关系分析");
    
    UnifiedModel model;
    LoadModel(model, xmlPath, nullptr, SerializationFormat::TINYXML);
    
    ModelAccessor modelAccessor;
    modelAccessor.SetModel(model);
    
    // 构建依赖图
    std::map<std::string, std::vector<std::string>> dependencies;
    
    for (int i = 0; i < modelAccessor.GetFeatureCount(); ++i) {
        auto feat = modelAccessor.GetFeature(i);
        if (!feat || !feat->IsValid()) continue;
        
        std::string featID = feat->GetID();
        dependencies[featID] = {};
        
        // 草图依赖分析
        if (auto sketch = feat->As<SketchAccessor>()) {
            if (sketch->HasReferencePlane()) {
                auto refPlane = sketch->GetReferencePlane();
                
                // 通过 ReferenceAccessor 获取依赖的特征 ID
                std::string depID;
                RefType refType = refPlane.GetRefType();
                
                if (refType == RefType::FEATURE_DATUM_PLANE) {
                    // 基准平面引用
                    depID = refPlane.GetTargetFeatureID();
                } else if (refType == RefType::TOPO_FACE) {
                    // 拓扑面引用
                    depID = refPlane.GetParentFeatureID();
                }
                
                if (!depID.empty()) {
                    dependencies[featID].push_back(depID);
                }
            }
        }
        // 拉伸依赖分析
        else if (auto extrude = feat->As<ExtrudeAccessor>()) {
            // 依赖于轮廓草图
            std::string profileID = extrude->GetProfileSketchID();
            if (!profileID.empty()) {
                dependencies[featID].push_back(profileID);
            }
            
            // 检查 EndCondition1 的参考实体
            auto ref1 = extrude->GetReference1();
            if (ref1.IsValid()) {
                std::string depID = ref1.GetTargetFeatureID();
                if (depID.empty()) {
                    depID = ref1.GetParentFeatureID();
                }
                if (!depID.empty()) {
                    dependencies[featID].push_back(depID);
                }
            }
            
            // 检查 EndCondition2 的参考实体
            if (extrude->HasDirection2()) {
                auto ref2 = extrude->GetReference2();
                if (ref2.IsValid()) {
                    std::string depID = ref2.GetTargetFeatureID();
                    if (depID.empty()) {
                        depID = ref2.GetParentFeatureID();
                    }
                    if (!depID.empty()) {
                        dependencies[featID].push_back(depID);
                    }
                }
            }
        }
        // 旋转依赖分析
        else if (auto revolve = feat->As<RevolveAccessor>()) {
            // 依赖于轮廓草图
            std::string profileID = revolve->GetProfileSketchID();
            if (!profileID.empty()) {
                dependencies[featID].push_back(profileID);
            }
            
            // 检查轴的参考实体
            auto axisRef = revolve->GetAxisReference();
            if (axisRef.IsValid()) {
                std::string depID = axisRef.GetTargetFeatureID();
                if (depID.empty()) {
                    depID = axisRef.GetParentFeatureID();
                }
                if (!depID.empty()) {
                    dependencies[featID].push_back(depID);
                }
            }
        }
    }
    
    // 显示依赖关系
    std::cout << "依赖关系图:" << std::endl;
    for (const auto& [featID, deps] : dependencies) {
        auto feat = modelAccessor.GetFeatureByID(featID);
        if (feat) {
            std::cout << "\n  " << feat->GetName() << " (" << featID << ")";
            
            if (deps.empty()) {
                std::cout << " → [无依赖]";
            } else {
                std::cout << " ← {";
                for (size_t i = 0; i < deps.size(); ++i) {
                    if (i > 0) std::cout << ", ";
                    auto depFeat = modelAccessor.GetFeatureByID(deps[i]);
                    if (depFeat) {
                        std::cout << depFeat->GetName();
                    }
                }
                std::cout << "}";
            }
            std::cout << std::endl;
        }
    }
    
    // 拓扑排序：确定重建顺序（带死锁检测）
    std::cout << "\n建议的重建顺序:" << std::endl;
    
    std::set<std::string> processed;
    int order = 1;
    int maxIterations = dependencies.size() * dependencies.size();
    int iterations = 0;
    
    while (processed.size() < dependencies.size() && iterations < maxIterations) {
        size_t previousSize = processed.size();
        
        for (const auto& [featID, deps] : dependencies) {
            if (processed.count(featID)) continue;
            
            // 检查所有依赖是否都已处理
            bool allDepProcessed = true;
            for (const auto& dep : deps) {
                if (!processed.count(dep)) {
                    allDepProcessed = false;
                    break;
                }
            }
            
            if (allDepProcessed) {
                auto feat = modelAccessor.GetFeatureByID(featID);
                if (feat) {
                    std::cout << "  " << order << ". " << feat->GetName() << std::endl;
                }
                processed.insert(featID);
                order++;
            }
        }
        
        // 检测死锁：如果这一轮没有处理任何新特征，说明有循环依赖或未识别的依赖
        if (processed.size() == previousSize) {
            std::cout << "\n⚠️ 警告：检测到循环依赖或未识别的依赖关系" << std::endl;
            std::cout << "未能排序的特征:" << std::endl;
            for (const auto& [featID, deps] : dependencies) {
                if (!processed.count(featID)) {
                    auto feat = modelAccessor.GetFeatureByID(featID);
                    if (feat) {
                        std::cout << "  - " << feat->GetName() << " (ID: " << featID << ")" << std::endl;
                        if (!deps.empty()) {
                            std::cout << "    依赖于: ";
                            for (size_t i = 0; i < deps.size(); ++i) {
                                if (i > 0) std::cout << ", ";
                                auto depFeat = modelAccessor.GetFeatureByID(deps[i]);
                                if (depFeat) {
                                    std::cout << depFeat->GetName();
                                }
                            }
                            std::cout << std::endl;
                        }
                    }
                }
            }
            break;
        }
        
        iterations++;
    }
}

// ============================================================================
// 第六部分：模拟 CAD 零件重建过程
// ============================================================================

void SimulatePartReconstruction(const std::string& xmlPath) {
    PrintSeparator("第六部分：模拟 CAD 零件重建");
    
    UnifiedModel model;
    LoadModel(model, xmlPath, nullptr, SerializationFormat::TINYXML);
    
    ModelAccessor modelAccessor;
    modelAccessor.SetModel(model);
    
    std::cout << "\n开始重建零件..." << std::endl;
    std::cout << "\n重建步骤:" << std::endl;
    
    int stepNum = 1;
    
    for (int i = 0; i < modelAccessor.GetFeatureCount(); ++i) {
        auto feat = modelAccessor.GetFeature(i);
        if (!feat || !feat->IsValid()) continue;
        
        // 第一步：检查特征类型
        std::cout << "\n【步骤 " << stepNum << "】处理特征: " << feat->GetName() << std::endl;
        
        if (auto sketch = feat->As<SketchAccessor>()) {
            // 重建草图
            std::cout << "  ✓ 识别为草图" << std::endl;
            std::cout << "  ✓ 建立坐标系...";
            
            if (sketch->HasReferencePlane()) {
                std::cout << " 绑定参考面成功" << std::endl;
            } else {
                std::cout << " (默认XY平面)" << std::endl;
            }
            
            std::cout << "  ✓ 添加几何段...";
            int segCount = sketch->GetSegmentCount();
            std::cout << " 添加 " << segCount << " 条几何" << std::endl;
            
            // 逐条添加几何
            for (int j = 0; j < segCount; ++j) {
                auto seg = sketch->GetSegment(j);
                if (!seg.IsValid()) continue;
                
                switch (seg.GetType()) {
                    case CSketchSeg::SegType::LINE:
                        std::cout << "      - 添加直线 " << seg.GetLocalID() << std::endl;
                        break;
                    case CSketchSeg::SegType::CIRCLE:
                        std::cout << "      - 添加圆 " << seg.GetLocalID() << std::endl;
                        break;
                    case CSketchSeg::SegType::ARC:
                        std::cout << "      - 添加圆弧 " << seg.GetLocalID() << std::endl;
                        break;
                    case CSketchSeg::SegType::POINT:
                        std::cout << "      - 添加点 " << seg.GetLocalID() << std::endl;
                        break;
                    default:
                        break;
                }
            }
            
            std::cout << "  ✓ 完成草图定义" << std::endl;
            
        } else if (auto extrude = feat->As<ExtrudeAccessor>()) {
            // 重建拉伸
            std::cout << "  ✓ 识别为拉伸特征" << std::endl;
            
            // 查找轮廓草图
            std::string profileID = extrude->GetProfileSketchID();
            auto profileFeat = modelAccessor.GetFeatureByID(profileID);
            if (profileFeat) {
                std::cout << "  ✓ 选择轮廓: " << profileFeat->GetName() << std::endl;
            }
            
            // 获取拉伸参数
            std::cout << "  ✓ 设置参数:" << std::endl;
            
            CVector3D dir = extrude->GetDirection();
            std::cout << "      - 方向: (" << dir.x << ", " << dir.y << ", " << dir.z << ")" << std::endl;
            std::cout << "      - 深度: " << extrude->GetDepth1() << " mm" << std::endl;
            
            std::cout << "  ✓ 应用布尔运算: ";
            switch (extrude->GetOperation()) {
                case BooleanOp::BOSS:
                    std::cout << "BOSS (凸出)" << std::endl;
                    break;
                case BooleanOp::CUT:
                    std::cout << "CUT (凹陷)" << std::endl;
                    break;
                default:
                    std::cout << "其他" << std::endl;
            }
            
            // 应用可选参数
            if (extrude->HasDraft()) {
                std::cout << "  ✓ 应用拔模: " << extrude->GetDraftAngle() << "°" << std::endl;
            }
            
            if (extrude->HasThinWall()) {
                std::cout << "  ✓ 应用薄壁: " << extrude->GetThinWallThickness() << " mm" << std::endl;
            }
            
            std::cout << "  ✓ 完成拉伸操作" << std::endl;
            
        } else if (auto revolve = feat->As<RevolveAccessor>()) {
            std::cout << "  ✓ 识别为旋转特征" << std::endl;
            std::cout << "  ✓ 完成旋转操作" << std::endl;
        }
        
        // 检查抑制状态
        if (feat->IsSuppressed()) {
            std::cout << "  ⚠ 特征已被抑制 (不会参与重建)" << std::endl;
        }
        
        std::cout << "  ✓ 特征完成" << std::endl;
        stepNum++;
    }
    
    std::cout << "\n" << std::string(80, '-') << std::endl;
    std::cout << "✅ 零件重建完成！" << std::endl;
    std::cout << "   总特征数: " << modelAccessor.GetFeatureCount() << std::endl;
}

// ============================================================================
// 主函数
// ============================================================================

#ifdef _WIN32
#include <windows.h>
#endif

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(65001); // 设置控制台代码页为 UTF-8
#endif

    std::cout << std::string(80, '=') << std::endl;
    std::cout << "  CAD 零件重建完整演示" << std::endl;
    std::cout << "  从 XML 导入 → Accessor 访问 → 零件重建" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    // XML 文件路径 - 从命令行参数或使用默认路径
    std::string xmlPath;
    if (argc > 1) {
        xmlPath = argv[1];
        std::cout << "\n使用命令行指定的文件路径: " << xmlPath << std::endl;
    } else {
        xmlPath = "build_msvc\\Release\\AdvancedPart.xml";
        std::cout << "\n使用默认文件路径: " << xmlPath << std::endl;
        std::cout << "提示: 可以通过命令行参数指定文件路径" << std::endl;
        std::cout << "用法: PartReconstructionDemo.exe <xml_file_path>" << std::endl;
    }
    
    try {
        // 第一部分：加载 XML
        LoadAndDisplayModelInfo(xmlPath);
        
        // 第二部分：遍历特征
        TraverseAndDisplayFeatures(xmlPath);
        
        // 第三部分：提取草图数据
        ExtractSketchData(xmlPath);
        
        // 第四部分：提取拉伸数据
        ExtractExtrudeData(xmlPath);
        
        // 第五部分：分析依赖关系
        AnalyzeDependencies(xmlPath);
        
        // 第六部分：模拟重建
        SimulatePartReconstruction(xmlPath);
        
        PrintSeparator("演示完成");
        std::cout << "\n程序执行成功！\n" << std::endl;
        
        return 0;
        
    } catch (const std::exception& ex) {
        std::cerr << "\n❌ 错误: " << ex.what() << std::endl;
        return 1;
    }
}
