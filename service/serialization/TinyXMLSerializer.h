#pragma once

#include "../../thirdParty/tinyxml2/tinyxml2.h"
#include "../../core/UnifiedFeatures.h"
#include "../../core/UnifiedModel.h"
#include <filesystem>
#include <iostream>
namespace CADExchange {

/**
 * @file TinyXMLSerializer.h
 * @brief 使用 tinyxml2 将 UnifiedModel 序列化 / 反序列化为 XML 的接口声明。
 *
 * 本文件声明了 `TinyXMLSerializer`，它负责把内存中的 `UnifiedModel`
 * 导出为轻量的 XML 表示（Save），以及从磁盘上的 XML 重新构建
 * `UnifiedModel`（Load）。序列化格式面向人类可读并兼顾向后兼容性，
 * 主要用于测试、导入导出以及简单持久化场景。
 */

/**
 * @class TinyXMLSerializer
 * @brief 提供静态方法以读写 `UnifiedModel` 到 XML 文件。
 *
 * 所有方法均为静态，类无状态；内部实现使用 tinyxml2 操作 DOM。
 */
class TinyXMLSerializer {
public:
  /**
   * @brief 将 `UnifiedModel` 保存为一个 XML 文件。
   *
   * @param model 要保存的模型引用（只读）。
   * @param filePath 目标文件路径（支持绝对或相对路径）。
   * @param errorMessage 若非空，出错时会写入错误描述。
   * @return 成功返回 true，失败返回 false 并在 `errorMessage`
   * 中返回原因（若提供）。
   */
  static bool Save(const UnifiedModel &model,
                   const std::filesystem::path &filePath,
                   std::string *errorMessage = nullptr);

  /**
   * @brief 从 XML 文件加载 `UnifiedModel` 并填充到传入的 model。
   *
   * @param model
   * 输出参数，函数返回时包含加载得到的要素（若加载失败则保持未定义或已清空）。
   * @param filePath 要加载的 XML 文件路径。
   * @param errorMessage 若非空，出错时会写入错误描述。
   * @return 成功返回 true，失败返回 false 并在 `errorMessage`
   * 中返回原因（若提供）。
   */
  static bool Load(UnifiedModel &model, const std::filesystem::path &filePath,
                   std::string *errorMessage = nullptr);

private:
  // Helpers for Save
  /**
   * @brief 将单个特征写入到父 XML 元素下（Feature 节点）。
   *
   * 此函数根据特征的运行时类型选择合适的子序列化函数（例如
   * Sketch/Extrude/Revolve）。
   * @param doc 当前 XML 文档对象（用于创建元素节点）。
   * @param parent 父 XML 元素，新的 Feature 元素将被插入到此节点下。
   * @param feature 要保存的特征指针。
   */
  static void SaveFeature(tinyxml2::XMLDocument &doc,
                          tinyxml2::XMLElement *parent,
                          const std::shared_ptr<CFeatureBase> &feature);

  /**
   * @brief 将 `CSketch` 类型的特征序列化到给定的元素下。
   * @param doc 当前 XML 文档对象。
   * @param element 用于接收草图数据的元素（通常为新创建的 Feature 节点）。
   * @param sketch 要序列化的草图对象指针。
   */
  static void SaveSketch(tinyxml2::XMLDocument &doc,
                         tinyxml2::XMLElement *element,
                         const std::shared_ptr<CSketch> &sketch);

  /**
   * @brief 将 `CExtrude` 类型的特征序列化到给定的元素下。
   */
  static void SaveExtrude(tinyxml2::XMLDocument &doc,
                          tinyxml2::XMLElement *element,
                          const std::shared_ptr<CExtrude> &extrude);

  /**
   * @brief 将 `CRevolve` 类型的特征序列化到给定的元素下。
   */
  static void SaveRevolve(tinyxml2::XMLDocument &doc,
                          tinyxml2::XMLElement *element,
                          const std::shared_ptr<CRevolve> &revolve);

  /**
   * @brief 序列化草图的单个段（线、圆、点等）。
   * @param doc 当前 XML 文档对象。
   * @param parent 段元素将被插入到的父节点（通常是 Segments 节点）。
   * @param seg 要序列化的草图段指针。
   */
  static void SaveSketchSeg(tinyxml2::XMLDocument &doc,
                            tinyxml2::XMLElement *parent,
                            const std::shared_ptr<CSketchSeg> &seg);

  /**
   * @brief 序列化草图约束。
   */
  static void SaveConstraint(tinyxml2::XMLDocument &doc,
                             tinyxml2::XMLElement *parent,
                             const CSketchConstraint &constraint);

  /**
   * @brief 将引用实体（平面/边/顶点/草图段等）序列化为名为 `name` 的子元素。
   *
   * 引用实体使用统一的 Type/属性格式，便于反序列化恢复引用的具体信息。
   * @param doc XML 文档上下文。
   * @param parent 父元素。
   * @param name 插入的子元素名（例如 "ReferencePlane"、"Reference" 等）。
   * @param ref 要序列化的引用实体指针（可为空表示不存在）。
   */
  static void SaveRefEntity(tinyxml2::XMLDocument &doc,
                            tinyxml2::XMLElement *parent,
                            const std::string &name,
                            const std::shared_ptr<CRefEntityBase> &ref);

  /**
   * @brief 将三维点写入元素属性（格式 (x,y,z)）。
   */
  static void SavePoint3D(tinyxml2::XMLElement *element, const char *name,
                          const CPoint3D &pt);

  /**
   * @brief 将三维向量写入元素属性（格式 (x,y,z)）。
   */
  static void SaveVector3D(tinyxml2::XMLElement *element, const char *name,
                           const CVector3D &vec);

  // Helpers for Load
  /**
   * @brief 从 Feature 节点构建对应的 CFeatureBase 派生对象。
   * @param element Feature 节点。
   * @return 返回填充好的特征指针，失败返回 nullptr。
   */
  static std::shared_ptr<CFeatureBase>
  LoadFeature(tinyxml2::XMLElement *element);

  /**
   * @brief 从 XML 元素恢复 `CSketch` 内容到传入的 shared_ptr。
   * @param element 包含草图数据的元素。
   * @param sketch 输出的草图对象引用。
   */
  static void LoadSketch(tinyxml2::XMLElement *element,
                         std::shared_ptr<CSketch> &sketch);

  /**
   * @brief 从 XML 元素恢复 `CExtrude`。
   */
  static void LoadExtrude(tinyxml2::XMLElement *element,
                          std::shared_ptr<CExtrude> &extrude);

  /**
   * @brief 从 XML 元素恢复 `CRevolve`。
   */
  static void LoadRevolve(tinyxml2::XMLElement *element,
                          std::shared_ptr<CRevolve> &revolve);

  /**
   * @brief 从 Segment 元素构建 `CSketchSeg`
   * 的具体派生实例（Line/Circle/Arc/Point）。
   */
  static std::shared_ptr<CSketchSeg>
  LoadSketchSeg(tinyxml2::XMLElement *element);

  /**
   * @brief 从 Constraint 元素构建 `CSketchConstraint`。
   */
  static CSketchConstraint LoadConstraint(tinyxml2::XMLElement *element);

  /**
   * @brief 从序列化的引用子元素恢复 `CRefEntityBase` 派生对象。
   * @param element 引用元素（如 ReferencePlane / Reference /
   * ReferenceEntity）。
   * @return 成功返回对应的引用对象指针，失败返回 nullptr。
   */
  static std::shared_ptr<CRefEntityBase>
  LoadRefEntity(tinyxml2::XMLElement *element);

  /**
   * @brief 从元素属性解析三维点，返回 CPoint3D。
   */
  static CPoint3D LoadPoint3D(tinyxml2::XMLElement *element, const char *name);

  /**
   * @brief 从元素属性解析三维向量，返回 CVector3D。
   */
  static CVector3D LoadVector3D(tinyxml2::XMLElement *element,
                                const char *name);
};

} // namespace CADExchange
