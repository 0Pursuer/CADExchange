<!--
CADExchange
-------------
统一 CAD 概念模型与 TinyXML/Cereal 两套序列化的参考实现。
-->

# CADExchange

`CADExchange` 是一个轻量级的跨 CAD 平台统一模型框架，核心目标是
用统一数据结构（`UnifiedModel` + `UnifiedFeatures`）封装草图、拉伸、旋转
等常见几何特征，并提供多种序列化路径（TinyXML 为人类可读、
Cereal 用于高性能持久化）。

## 项目结构

- `include/`
	- `UnifiedTypes.*`、`UnifiedFeatures.*`：定义几何/引用、特征、草图等基础数据结构。
	- `TinyXMLSerializer.*` + `UnifiedSerialization.*`：分别实现 TinyXML 和 Cereal 的序列化逻辑。
	- `FeatureBuilder*` 与 `CADSerializer`：提供构建、保存、加载统一模型的 helper。
- `src/`
	- `TinyXMLSerializer.cpp`：TinyXML 的具体实现，包括平面/边/顶点等引用的 entry 表与读取/写入回调。
	- `SerializationRegistry.cpp`：注册 Cereal 的多态关系。
- `test/SolidWorks/`：模拟 SolidWorks (SwRead/SwWrite) 的读写流程，专用于验证序列化互操作性 (`ReadPartTest.exe`/`WritePartTest.exe`)。
- `build/`：CMake 生成的构建产物，可直接运行 `ReadPartTest`、`WritePartTest`、并产出示例 XML（`SimulationPart.xml` / `SimulationPart_Tiny.xml`）。

## 快速开始

```powershell
cmake -S . -B build -G "Ninja"
cmake --build build --config Release
```

以上命令会编译库与测试程序。编译完成后可在 `build/Release` 目录下找到：

- `ReadPartTest.exe`：读取Cereal/TinyXML生成的 XML 并模拟 SolidWorks 导入流程。
- `WritePartTest.exe`：加载 XML（默认 `SimulationPart.xml`）并模拟 SolidWorks 写入流程。
- `SimulationPart.xml` / `_Tiny.xml`：示例模型输出，`Tiny` 版本使用 TinyXML 序列化。

## 序列化说明

### TinyXML

- `TinyXMLSerializer` 使用 `RefSerializerEntry` 表按 `RefType` 注册各类引用的保存/加载逻辑，确保 `Type` 属性与它们的属性（如 `TargetFeatureID`、`ParentFeatureID`、`TopologyIndex`、`YDir` 等）一致。
- 引用实体统一写入 `SaveRefEntity`，读取时先查表然后调用对应 lambda，留下 `ReferenceEntity` 标签兼容性降级（旧 `<Reference>` 仍能解析）。
- 所有方向信息（`XDir`、`YDir`、`Normal`）通过 helper 计算并序列化，不存在 `YDir` 时会在读取端根据正交性重建。

### Cereal

- `UnifiedSerialization` 中的 serialize 函数负责 Cereal XML 序列化，已注册多态关系，支持在 `CADSerializer` 中自由切换格式（`SerializationFormat::CEREAL` vs `::TINYXML`）。
- `SaveModel` / `LoadModel` 是入口，TinyXML 的 `Save`/`Load` 会在 XML 可读格式下表现出相同的特征结构；`ReadPartTest`/`WritePartTest` 演示了如何加载并对外部系统模拟读取。

## 运行示例

运行 TinyXML 测试：

```powershell
cd build/Release
.
WritePartTest.exe SimulationPart_Tiny.xml
```

运行 Cereal 测试（默认）：

```powershell
cd build/Release
.
ReadPartTest.exe
```

## 拓展建议

1. **更多引用类型**：可以在 `RefSerializerEntry` 添加 `CRefSketchSeg` 的本地 ID 等属性。  
2. **XML Schema/Validation**：提供一个 `UnifiedModel.xsd` 以便外部工具检查 generated XML。  
3. **高性能序列化**：结合 `cereal::BinaryOutputArchive` 支持大模型快速读写。

## 贡献与许可

欢迎提交 issue/PR，项目遵循 MIT 风格许可（请根据实际仓库设置补充）。

