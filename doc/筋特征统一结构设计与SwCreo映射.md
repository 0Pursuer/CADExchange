# 筋特征统一结构设计与 Sw/Creo 映射

## 1. 文档目的

本文用于约束 `CADExchange` 第一阶段筋特征实现的统一结构设计范围，仅覆盖：

- `FeatureType::Rib`
- `CRib`
- `TinyXMLSerializer::SaveRib / LoadRib`
- `ModelValidator` 的基础校验

本文不讨论第二阶段的实际 CAD 读写实现细节，例如：

- `SwLibs` 中如何调用 `IRibFeatureData2` / `InsertRib2()`
- `CreoLibs` 中如何组织 `ProRib.h` 对应的 element tree

本文目标是先把 `Creo <-> CADExchange <-> SW` 之间可稳定承载的筋特征统一语义定清楚，并给出第一阶段可直接落代码的字段定义、枚举取值、序列化格式与映射规则。

---

## 2. 第一阶段设计原则

### 2.1 第一阶段只做模型层，不做桥接补偿层

第一阶段的 `CRib` 只负责：

- 承载统一语义
- 保证 tinyxml 可稳定保存与回读
- 保证 validator 能拦住明显非法数据

第一阶段不负责：

- 解决 SW/Creo API 选择集差异
- 解决缺失语义下的自动补偿策略
- 保证所有 SW rib 高级行为都能在 Creo 上无损重建

### 2.2 统一结构采用“小公共主干 + SW 扩展块”

当前仓内已有调研结论表明：

- Creo rib 的公开参数面较窄
- SW rib 比 Creo rib 多出明显的原生高级语义

因此第一阶段不应把所有 SW 字段都硬塞进公共主字段，而应分为两层：

1. `CRib` 公共主干  
   只放当前证据下 `SW / Creo` 都能稳定承接的语义

2. `CRibSwOptions`  
   放 SW rib 独有、但对 round-trip 有价值的原生语义

### 2.3 vendor residue 只保留“接口定位信息”，不保留“产品语义替身”

像下面这些字段描述的是“筋本身是什么”，而不是某个 API 如何调用：

- 厚度
- 厚度分布侧
- 材料侧
- 是否 normal/parallel to sketch
- 是否 linear/natural
- draft 语义

因此其中前四类里只有确实没有 Creo 对位、且第一阶段不准备标准化的部分，才放进 `CRibSwOptions`。

像下面这些字段更像接口或选择定位残留：

- `ReferenceEdgeIndex`
- `RefSketchIndex`

它们不进入统一主干，只保留在 SW 扩展块中。

---

## 3. 第一阶段建议数据结构

### 3.0 统一的完整结构

下面给出第一阶段建议直接落地到 `CADExchange` 的完整结构草案。

```cpp
enum class FeatureType {
  Unknown,
  Extrude,
  Revolve,
  Sweep,
  Fillet,
  Chamfer,
  Rib,
  Sketch,
  DatumPlane
};

enum class RibThicknessSideMode {
  Unknown = 0,
  Symmetric,
  OneSideA,
  OneSideB
};

enum class RibMaterialSide {
  Unknown = 0,
  SideA,
  SideB
};

enum class SwRibType {
  Unknown = 0,
  Linear,
  Natural
};

enum class SwRibExtrusionDirection {
  Unknown = 0,
  ParallelToSketch,
  NormalToSketch
};

struct CRibSwDraftOption {
  bool enabled = false;
  double angle = 0.0;
  bool outward = false;
  std::optional<bool> fromWall;
};

struct CRibSwOptions {
  SwRibType ribType = SwRibType::Unknown;
  SwRibExtrusionDirection extrusionDirection =
      SwRibExtrusionDirection::Unknown;
  std::optional<int> referenceEdgeIndex;
  std::optional<int> refSketchIndex;
  std::optional<CRibSwDraftOption> draft;
};

struct CRib : public CFeatureBase {
  std::string sectionSketchID;

  double thickness = 0.0;
  RibThicknessSideMode thicknessSideMode =
      RibThicknessSideMode::Unknown;
  RibMaterialSide materialSide = RibMaterialSide::Unknown;

  std::shared_ptr<CRefEntityBase> targetBody;

  std::optional<CRibSwOptions> swOptions;

  CRib() { featureType = FeatureType::Rib; }
};
```

说明：

- 这是第一阶段“统一完整结构”
- 其中“完整”指的是：
  - 对第一阶段范围内要实现的 rib 模型来说是完整的
  - 已经包含公共主干与 SW 扩展块
- 它不是第二阶段最终桥接实现的全部辅助状态
- 也不是要把 Creo element tree 的内部组织细节直接镜像进 `CRib`

为避免结构膨胀，第一阶段刻意不增加：

- `creoOptions`
- 非统一语义的 API 调用缓存字段
- 仅用于某一写回补偿流程的临时状态字段

如果第二阶段确认 Creo rib 存在新的、稳定且值得保真的原生语义，再单独评估是否提升为：

1. `CRib` 公共主字段
2. `CRibCreoOptions`
3. 写回时的桥接临时上下文

---

### 3.1 `FeatureType`

在 `CADExchange::FeatureType` 中新增：

```cpp
enum class FeatureType {
  Unknown,
  Extrude,
  Revolve,
  Sweep,
  Fillet,
  Chamfer,
  Rib,
  Sketch,
  DatumPlane
};
```

`Rib` 表示“筋特征”这一独立 feature family，不视为 `Extrude` 的别名，也不复用 `ThinWall Extrude` 来冒充 rib。

---

### 3.2 公共枚举

#### 3.2.1 `RibThicknessSideMode`

```cpp
enum class RibThicknessSideMode {
  Unknown = 0,
  Symmetric,
  OneSideA,
  OneSideB
};
```

含义：

- `Symmetric`
  - 厚度相对筋的中性面/截面中心线对称分布
  - 对应 Creo：`PRO_RIB_SYMMETRIC`
  - 对应 SW：`IsTwoSided = true`

- `OneSideA`
  - 单侧分布，落在统一定义的 A 侧
  - 对应 Creo：`PRO_RIB_SIDE_ONE`
  - 对应 SW：`IsTwoSided = false` 且 `ReverseThicknessDir = false`

- `OneSideB`
  - 单侧分布，落在统一定义的 B 侧
  - 对应 Creo：`PRO_RIB_SIDE_TWO`
  - 对应 SW：`IsTwoSided = false` 且 `ReverseThicknessDir = true`

- `Unknown`
  - 未识别
  - 第一阶段允许 tinyxml 回读存在 `Unknown`
  - 但写回 CAD 前通常应视为非法或至少高风险

这里的 `A/B` 是统一层约定的抽象两侧，不直接把字段名绑死在某 CAD 的“正/反”“side1/side2”命名上。

---

#### 3.2.2 `RibMaterialSide`

```cpp
enum class RibMaterialSide {
  Unknown = 0,
  SideA,
  SideB
};
```

含义：

- `SideA`
  - 材料落在截面一侧
  - 对应 Creo：`PRO_E_STD_MATRLSIDE` 的一侧取值
  - 对应 SW：`FlipSide / ReverseMaterialDir = false`

- `SideB`
  - 材料落在截面另一侧
  - 对应 Creo：`PRO_E_STD_MATRLSIDE` 的另一侧取值
  - 对应 SW：`FlipSide / ReverseMaterialDir = true`

- `Unknown`
  - 未识别或未提供

注意：

- `materialSide` 与 `thicknessSideMode` 不是同一个概念
- 前者描述“材料在哪一侧”
- 后者描述“厚度相对筋中性面如何分布”

---

### 3.3 SW 扩展枚举

#### 3.3.1 `SwRibType`

```cpp
enum class SwRibType {
  Unknown = 0,
  Linear,
  Natural
};
```

含义：

- `Linear`
  - 对应 `swRibLinear`
- `Natural`
  - 对应 `swRibNatural`
- `Unknown`
  - 未识别或当前模型未提供

该字段第一阶段不提升为公共主干，因为当前 Creo rib 公开参数面未见稳定对位。

---

#### 3.3.2 `SwRibExtrusionDirection`

```cpp
enum class SwRibExtrusionDirection {
  Unknown = 0,
  ParallelToSketch,
  NormalToSketch
};
```

含义：

- `ParallelToSketch`
  - 对应 `swRibParallelToSketch`
- `NormalToSketch`
  - 对应 `swRibNormalToSketch`
  - 也对应 `InsertRib2(..., IsNormToSketch = true)`
- `Unknown`
  - 未识别

该字段也先保留在 SW 扩展块中，不进入公共主干。

---

### 3.4 SW draft 扩展块

```cpp
struct CRibSwDraftOption {
  bool enabled = false;
  double angle = 0.0;
  bool outward = false;
  std::optional<bool> fromWall;
};
```

含义：

- `enabled`
  - 对应 SW `EnableDraft`

- `angle`
  - 对应 SW `DraftAngle`
  - 单位与模型长度单位无关，为角度
  - 建议统一沿用当前 `CADExchange` 角度字段习惯：
    - 文档语义写“角度”
    - tinyxml 直接保存数值
    - 具体内部存储单位由实现时与现有角度字段风格保持一致

- `outward`
  - 对应 SW `DraftOutward`

- `fromWall`
  - 对应 SW `DraftFromWall`
  - `std::optional` 的原因是：
    - 当前仅确认接口存在
    - 对其精确 UI/建模语义尚未完成实测

---

### 3.5 SW 扩展选项块

```cpp
struct CRibSwOptions {
  SwRibType ribType = SwRibType::Unknown;
  SwRibExtrusionDirection extrusionDirection =
      SwRibExtrusionDirection::Unknown;
  std::optional<int> referenceEdgeIndex;
  std::optional<int> refSketchIndex;
  std::optional<CRibSwDraftOption> draft;
};
```

字段含义：

- `ribType`
  - 承载 `Linear / Natural`

- `extrusionDirection`
  - 承载 `ParallelToSketch / NormalToSketch`

- `referenceEdgeIndex`
  - 保留 SW 创建/定义对象中的引用边索引语义
  - 第一阶段只做保真保存，不赋予统一产品语义

- `refSketchIndex`
  - 保留 SW 创建/定义对象中的草图索引定位语义
  - 第一阶段只做保真保存

- `draft`
  - SW rib 的拔模参数集合

---

### 3.6 `CRib`

```cpp
struct CRib : public CFeatureBase {
  std::string sectionSketchID;

  double thickness = 0.0;
  RibThicknessSideMode thicknessSideMode =
      RibThicknessSideMode::Unknown;
  RibMaterialSide materialSide = RibMaterialSide::Unknown;

  std::shared_ptr<CRefEntityBase> targetBody;

  std::optional<CRibSwOptions> swOptions;

  CRib() { featureType = FeatureType::Rib; }
};
```

各字段含义如下。

#### 3.6.1 `sectionSketchID`

- 含义：
  - 定义 rib 截面的草图特征 ID
- 来源：
  - SW：选中的草图/轮廓最终归一到一个草图特征 ID
  - Creo：`PRO_E_STD_SECTION`
- 约束：
  - 第一阶段要求它必须引用一个有效草图特征

#### 3.6.2 `thickness`

- 含义：
  - 筋厚度
- 来源：
  - SW：`Thickness`
  - Creo：`PRO_E_RIB_THICKNESS`
- 约束：
  - 必须大于 `0`

#### 3.6.3 `thicknessSideMode`

- 含义：
  - 厚度是对称分布，还是落在单侧 A/B
- 来源：
  - SW：`IsTwoSided + ReverseThicknessDir`
  - Creo：`PRO_E_RIB_SIDE_OPTS`

#### 3.6.4 `materialSide`

- 含义：
  - 截面相对材料落在哪侧
- 来源：
  - SW：`FlipSide / ReverseMaterialDir`
  - Creo：`PRO_E_STD_MATRLSIDE`

#### 3.6.5 `targetBody`

- 含义：
  - rib 所属或作用的目标 body
- 来源：
  - SW：`Body`
  - Creo：`PRO_E_BODY`
- 约束：
  - 第一阶段允许为空
  - 对于单实体零件或 CAD 默认行为可推导场景，空值可接受

#### 3.6.6 `swOptions`

- 含义：
  - 保留 SW rib 无法进入公共主干但仍需保真的原生语义
- 约束：
  - 允许为空
  - Creo -> CADExchange 时通常为空

---

## 4. 为什么第一阶段不把更多字段拉进公共主干

第一阶段明确不把以下字段提升为公共主干：

- `Linear / Natural`
- `ParallelToSketch / NormalToSketch`
- `Draft`
- `ReferenceEdgeIndex`
- `RefSketchIndex`

原因分两类：

### 4.1 公开证据下没有稳定 Creo 对位

以下字段当前更像 SW 原生扩展能力：

- `Linear / Natural`
- `ParallelToSketch / NormalToSketch`
- `Draft`

如果在第一阶段就把它们定义为公共标准字段，会造成两个问题：

1. `CRib` 会被 SW 语义绑架
2. Creo 写回端在没有稳定对位的情况下会被迫伪造解释

### 4.2 接口定位信息不应伪装成统一产品语义

以下字段更像 API/选择残留：

- `ReferenceEdgeIndex`
- `RefSketchIndex`

它们对 round-trip 保真有用，但不应在 `CRib` 主干里冒充“筋本身的标准定义”。

---

## 5. TinyXML 建议格式

### 5.1 基本形态

建议新增：

```xml
<Feature Type="Rib" ID="..." Name="...">
  <Section SketchID="..." />
  <Thickness Value="..." SideMode="..." />
  <MaterialSide Value="..." />
  <TargetBody>
    ...
  </TargetBody>
  <SwOptions
    RibType="..."
    ExtrusionDirection="..."
    ReferenceEdgeIndex="..."
    RefSketchIndex="...">
    <Draft
      Enabled="true"
      Angle="..."
      Outward="false"
      FromWall="true" />
  </SwOptions>
</Feature>
```

说明：

- `Section`、`Thickness`、`MaterialSide` 是主干字段
- `TargetBody` 可选
- `SwOptions` 整块可选
- `Draft` 只有在 `swOptions.draft.has_value()` 时才输出

---

### 5.2 枚举字符串建议

#### `RibThicknessSideMode`

```text
Unknown
Symmetric
OneSideA
OneSideB
```

#### `RibMaterialSide`

```text
Unknown
SideA
SideB
```

#### `SwRibType`

```text
Unknown
Linear
Natural
```

#### `SwRibExtrusionDirection`

```text
Unknown
ParallelToSketch
NormalToSketch
```

字符串规则建议与现有 `TinyXMLSerializer` 风格一致：

- 优先使用可读字符串，不优先使用整数
- 读侧可以保留兼容整数枚举值的 fallback
- 写侧统一输出字符串

---

### 5.3 最小示例：Creo 来源 rib

```xml
<Feature Type="Rib" ID="rib_1" Name="Rib1">
  <Section SketchID="sketch_10" />
  <Thickness Value="0.004" SideMode="Symmetric" />
  <MaterialSide Value="SideA" />
</Feature>
```

特点：

- 不包含 `SwOptions`
- 只包含公共主干

---

### 5.4 最小示例：SW 来源 rib

```xml
<Feature Type="Rib" ID="rib_2" Name="Rib2">
  <Section SketchID="sketch_20" />
  <Thickness Value="0.003" SideMode="OneSideB" />
  <MaterialSide Value="SideA" />
  <SwOptions
    RibType="Natural"
    ExtrusionDirection="NormalToSketch"
    ReferenceEdgeIndex="1"
    RefSketchIndex="0">
    <Draft
      Enabled="true"
      Angle="3.0"
      Outward="false"
      FromWall="true" />
  </SwOptions>
</Feature>
```

特点：

- 主干字段稳定可被 Creo 端理解
- SW 高级语义通过 `SwOptions` 保真

---

## 6. Creo <-> CADExchange <-> SW 映射规则

## 6.1 Creo -> CADExchange

### 6.1.1 直接映射字段

| Creo | CADExchange | 规则 |
|---|---|---|
| `PRO_E_STD_SECTION` | `sectionSketchID` | 解析为草图特征 ID |
| `PRO_E_RIB_THICKNESS` | `thickness` | 直接赋值 |
| `PRO_E_RIB_SIDE_OPTS = PRO_RIB_SYMMETRIC` | `thicknessSideMode = Symmetric` | 直接映射 |
| `PRO_E_RIB_SIDE_OPTS = PRO_RIB_SIDE_ONE` | `thicknessSideMode = OneSideA` | 统一映射 |
| `PRO_E_RIB_SIDE_OPTS = PRO_RIB_SIDE_TWO` | `thicknessSideMode = OneSideB` | 统一映射 |
| `PRO_E_STD_MATRLSIDE` | `materialSide` | 归一为 `SideA / SideB` |
| `PRO_E_BODY` | `targetBody` | 可解析则写入引用 |

### 6.1.2 未进入第一阶段主干的部分

当前证据下，Creo rib 没有明确稳定对位到：

- `SwRibType`
- `SwRibExtrusionDirection`
- `CRibSwDraftOption`

因此：

- `swOptions` 留空

---

## 6.2 CADExchange -> Creo

### 6.2.1 主干字段写回

| CADExchange | Creo | 规则 |
|---|---|---|
| `sectionSketchID` | `PRO_E_STD_SECTION` | 直接引用草图 section |
| `thickness` | `PRO_E_RIB_THICKNESS` | 直接赋值 |
| `thicknessSideMode = Symmetric` | `PRO_RIB_SYMMETRIC` | 直接映射 |
| `thicknessSideMode = OneSideA` | `PRO_RIB_SIDE_ONE` | 统一约定写回 |
| `thicknessSideMode = OneSideB` | `PRO_RIB_SIDE_TWO` | 统一约定写回 |
| `materialSide = SideA/SideB` | `PRO_E_STD_MATRLSIDE` | 按统一 A/B 约定映射 |
| `targetBody` | `PRO_E_BODY` | 有值则写；无值则按默认 body 行为 |

### 6.2.2 来自 SW 的扩展语义如何处理

| CADExchange `swOptions` | Creo 写回策略 |
|---|---|
| `ribType` | 第一阶段忽略，不报错，但可记录 warning |
| `extrusionDirection` | 第一阶段忽略，不报错，但可记录 warning |
| `draft` | 第一阶段忽略，不报错，但可记录 warning |
| `referenceEdgeIndex` | 第一阶段忽略 |
| `refSketchIndex` | 第一阶段忽略 |

这意味着：

- `SW -> CADExchange -> Creo` 在第一阶段是“公共主干保真 + SW 扩展保留但不消费”
- tinyxml 不丢字段
- 但 Creo 写回不承诺实现这些 SW 高级语义

---

## 6.3 SW -> CADExchange

### 6.3.1 主干字段

| SW | CADExchange | 规则 |
|---|---|---|
| 草图/轮廓选择 | `sectionSketchID` | 归一到草图特征 ID |
| `Thickness` | `thickness` | 直接赋值 |
| `IsTwoSided = true` | `thicknessSideMode = Symmetric` | 直接映射 |
| `IsTwoSided = false && ReverseThicknessDir = false` | `thicknessSideMode = OneSideA` | 统一映射 |
| `IsTwoSided = false && ReverseThicknessDir = true` | `thicknessSideMode = OneSideB` | 统一映射 |
| `FlipSide / ReverseMaterialDir = false` | `materialSide = SideA` | 统一映射 |
| `FlipSide / ReverseMaterialDir = true` | `materialSide = SideB` | 统一映射 |
| `Body` | `targetBody` | 可解析则写引用 |

### 6.3.2 SW 扩展块

| SW | `CRibSwOptions` | 规则 |
|---|---|---|
| `Type = swRibLinear` | `ribType = Linear` | 直接映射 |
| `Type = swRibNatural` | `ribType = Natural` | 直接映射 |
| `ExtrusionDirection = swRibParallelToSketch` | `extrusionDirection = ParallelToSketch` | 直接映射 |
| `ExtrusionDirection = swRibNormalToSketch` | `extrusionDirection = NormalToSketch` | 直接映射 |
| `ReferenceEdgeIndex` | `referenceEdgeIndex` | 直接保留 |
| `RefSketchIndex` | `refSketchIndex` | 直接保留 |
| `EnableDraft` | `draft.enabled` | 直接映射 |
| `DraftAngle` | `draft.angle` | 直接映射 |
| `DraftOutward` | `draft.outward` | 直接映射 |
| `DraftFromWall` | `draft.fromWall` | 若可读到则直接映射 |

---

## 6.4 CADExchange -> SW

### 6.4.1 主干字段写回

| CADExchange | SW | 规则 |
|---|---|---|
| `sectionSketchID` | 选择草图/轮廓 | 解析后加入创建选择集 |
| `thickness` | `Thickness` | 直接赋值 |
| `thicknessSideMode = Symmetric` | `Is2Sided = true` | 直接映射 |
| `thicknessSideMode = OneSideA` | `Is2Sided = false, ReverseThicknessDir = false` | 统一映射 |
| `thicknessSideMode = OneSideB` | `Is2Sided = false, ReverseThicknessDir = true` | 统一映射 |
| `materialSide = SideA` | `ReverseMaterialDir = false` | 统一映射 |
| `materialSide = SideB` | `ReverseMaterialDir = true` | 统一映射 |
| `targetBody` | `Body` | 有值则绑定 |

### 6.4.2 SW 扩展块写回

| `CRibSwOptions` | SW | 规则 |
|---|---|---|
| `ribType` | `IRibFeatureData2::Type` / 对应创建参数 | 直接写回 |
| `extrusionDirection` | `IRibFeatureData2::ExtrusionDirection` / `IsNormToSketch` | 直接写回 |
| `referenceEdgeIndex` | `ReferenceEdgeIndex` | 若创建 API 需要则回填 |
| `refSketchIndex` | `RefSketchIndex` | 若定义对象支持则回填 |
| `draft.enabled` | `EnableDraft` | 直接写回 |
| `draft.angle` | `DraftAngle` | 直接写回 |
| `draft.outward` | `DraftOutward` | 直接写回 |
| `draft.fromWall` | `DraftFromWall` | 有值才写 |

如果 `swOptions` 为空：

- SW 端只按主干字段重建最基础 rib
- 其余行为采用 SW 默认值

---

## 7. 第一阶段 Validator 规则建议

第一阶段 `ModelValidator` 只做基础合法性检查，不做高级 CAD 可建性判定。

建议新增以下规则。

### 7.1 必须项

- `RIB_001`
  - `sectionSketchID` 不能为空

- `RIB_002`
  - `sectionSketchID` 必须能引用到一个 `FeatureType::Sketch`

- `RIB_003`
  - `thickness` 必须 `> 0`

- `RIB_004`
  - `thicknessSideMode` 不能为 `Unknown`

- `RIB_005`
  - `materialSide` 不能为 `Unknown`

### 7.2 引用项

- `RIB_006`
  - `targetBody` 若存在，则必须是可接受的引用类型

### 7.3 扩展字段合理性

- `RIB_007`
  - 若 `swOptions.draft.enabled = true`，则 `draft.angle` 应为非负

- `RIB_008`
  - `referenceEdgeIndex` / `refSketchIndex` 若存在，建议 `>= 0`

### 7.4 warning 级别

- `RIB_WARN_001`
  - 发现 `swOptions`，但当前目标 CAD 为 Creo
  - 说明这些扩展字段第一阶段不会被消费

---

## 8. 第一阶段 round-trip 语义预期

### 8.1 Creo -> CADExchange -> Creo

目标：

- 公共主干字段尽量原样往返
- 不引入 SW 扩展块

预期保真项：

- `sectionSketchID`
- `thickness`
- `thicknessSideMode`
- `materialSide`
- `targetBody`

### 8.2 SW -> CADExchange -> SW

目标：

- 公共主干字段保真
- `swOptions` 内字段也保真

预期保真项：

- 主干字段
- `ribType`
- `extrusionDirection`
- `draft`
- `referenceEdgeIndex`
- `refSketchIndex`

### 8.3 SW -> CADExchange -> Creo

目标：

- 主干字段尽量保真
- `swOptions` 仅保存，不承诺 Creo 消费

结果预期：

- tinyxml 中不会丢失 SW 原生高级字段
- 但 Creo 重建时只能消费主干字段

### 8.4 Creo -> CADExchange -> SW

目标：

- 主干字段保真
- `swOptions` 为空时按 SW 默认行为创建

结果预期：

- 能创建基础 rib
- 不承诺生成特定 `Linear/Natural`、`Parallel/Normal`、`Draft` 行为

---

## 9. 第一阶段与第二阶段的边界

第一阶段完成后，`CADExchange` 应具备：

- 统一数据结构
- 稳定 tinyxml 表达
- 基础合法性校验

但仍未完成：

- `SwLibs` rib 读实现
- `SwLibs` rib 写实现
- `CreoLibs` rib 读实现
- `CreoLibs` rib 写实现
- 跨系统高级语义降级/告警策略

因此本设计是“模型与序列化先行”，不是“筋特征已完全跨系统打通”。

---

## 10. 结论

第一阶段的合理落地方式是：

1. 在 `FeatureType` 中引入 `Rib`
2. 用 `CRib` 承载当前稳定公共主干：
   - `sectionSketchID`
   - `thickness`
   - `thicknessSideMode`
   - `materialSide`
   - `targetBody`
3. 用 `CRibSwOptions` 保留 SW 独有但重要的高级语义：
   - `ribType`
   - `extrusionDirection`
   - `draft`
   - `referenceEdgeIndex`
   - `refSketchIndex`
4. 通过 tinyxml 保证这些信息在 `CADExchange` 层可完整存取
5. 在 validator 中只做第一阶段需要的基础合法性拦截

这套设计可以支撑：

- `Creo -> CADExchange -> Creo` 的基础语义保真
- `SW -> CADExchange -> SW` 的高保真保存
- `SW -> CADExchange -> Creo` 时“主干可消费、扩展可保留”的受控降级

它适合作为后续第二阶段桥接实现的稳定模型前提。
