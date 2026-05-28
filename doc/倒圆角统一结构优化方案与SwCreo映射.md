# 倒圆角统一结构优化方案与 Sw/Creo 映射

本文档整理当前代码已经落地的倒圆角统一结构，以及 `Creo <-> CADExchange <-> SolidWorks` 的现行映射规则。

本版本已经不再描述旧的过渡结构，不再以 `DefaultRadius / DefaultDistance / DefaultDistance2 / radius2` 作为统一结构本体，而是以标准化语义字段为中心。

## 1. 当前统一结构

当前 `CADExchange` 中倒圆角统一结构定义在：

- `CADExchange/core/UnifiedFeatures.h`

当前结构如下：

```cpp
enum class FilletDriveType {
  UNKNOWN = 0,
  RADIUS,
  SINGLE_DISTANCE,
  TWO_DISTANCES
};

struct CFilletRadiusPoint {
  double position = 0.0;
  std::optional<double> primaryValue;
  std::optional<double> secondValue;
  std::shared_ptr<CRefEdge> edgeRef;
};

struct CFilletParams {
  FilletDriveType driveType{FilletDriveType::UNKNOWN};
  std::optional<double> primaryValue;
  std::optional<double> secondValue;
  std::vector<CFilletRadiusPoint> radiusPoints;
  FilletCrossSection crossSection{FilletCrossSection::UNKNOWN};
  FilletConicValueMode conicValueMode{FilletConicValueMode::NONE};
  std::optional<double> conicValue;
  bool tangentPropagation = false;
};

struct CFillet : public CFeatureBase {
  FilletMode mode{FilletMode::UNKNOWN};
  FilletReferenceMode referenceMode{FilletReferenceMode::UNKNOWN};
  CFilletParams params;
  std::vector<std::shared_ptr<CRefEntityBase>> references;
  std::vector<std::shared_ptr<CRefFace>> side1Faces;
  std::vector<std::shared_ptr<CRefFace>> side2Faces;
  std::vector<std::shared_ptr<CRefFace>> centerFaces;
  std::optional<CPoint3D> firstEndFaceMarker;
  std::optional<std::string> swOverflowType;
  bool swKeepFeatures = false;
  std::optional<int> creoAttachType;
  std::optional<int> creoConicDepOption;
};
```

## 2. 标准语义字段

### 2.1 mode

- 当前统一口径仅使用：
  - `Constant`
  - `Variable`
  - `FullRound`

说明：

- `Chordal` 不再作为统一结构内的标准 mode 保留。
- `ConstantRadius` 与 `VariableRadius` 统一归一化为：
  - `Constant`
  - `Variable`

### 2.2 driveType

- `RADIUS`
  - 半径驱动
- `SINGLE_DISTANCE`
  - 单距离驱动
- `TWO_DISTANCES`
  - 双距离驱动

说明：

- 这是统一结构里最关键的驱动语义字段。
- 它用于归一化 `Creo chordal`、`Creo D1XD2`、`SW asymmetric D1/D2`。
- 内存结构中会区分：
  - `SINGLE_DISTANCE`
  - `TWO_DISTANCES`
- 但 XML 写出时统一收敛为：
  - `DriveType="Distances"`
  - 是否存在 `SecondValue` 决定它是单距离还是双距离

### 2.3 primaryValue / secondValue

- `primaryValue`
  - 主驱动值
- `secondValue`
  - 第二驱动值，仅双值驱动时使用

统一解释规则：

- `driveType = RADIUS`
  - `primaryValue` 表示半径
- `driveType = SINGLE_DISTANCE`
  - `primaryValue` 表示单距离值
- `driveType = TWO_DISTANCES`
  - `primaryValue` 表示第一距离
  - `secondValue` 表示第二距离

### 2.4 crossSection / conicValueMode / conicValue

- `crossSection`
  - `Circular`
  - `Conic`
  - `CurvatureContinuous`
- `conicValueMode`
  - 当前常见为 `GenericValue`
- `conicValue`
  - `Conic` 或 `CurvatureContinuous` 的附加值

### 2.5 radiusPoints

- `radiusPoints[*].primaryValue`
  - 控制点主值
- `radiusPoints[*].secondValue`
  - 控制点第二值
- `radiusPoints[*].edgeRef`
  - 控制点挂靠边引用

### 2.6 referenceMode

- 当前仍保留在统一结构内
- 但默认 `EdgeChain` 不再写到 XML
- 仅以下非默认模式才有保留意义：
  - `FaceFace`
  - `FullRoundThreeFaces`

### 2.7 firstEndFaceMarker

- 主要用于跨系统重建时辅助保持端面关联
- 它不是倒圆角主语义字段
- 但在写回稳定性上仍有工程价值

## 3. 旧结构处理原则

以下字段不再是统一结构本体：

- `DefaultRadius`
- `DefaultDistance`
- `DefaultDistance2`
- `DefaultRadius2`
- `Radius`
- `Distance`
- `Distance2`
- `Radius2`

当前规则是：

- 新 XML 不再写这些旧字段
- 读取旧 XML 时仍兼容解析，并归一化映射到：
  - `DriveType`
  - `PrimaryValue`
  - `SecondValue`

因此：

- `CADExchange` 现在是标准语义层
- 不再是 SW / Creo 字段名的兼容直通层

## 4. 当前 XML 口径

### 4.1 半径驱动

```xml
<Feature Type="Fillet" Mode="Constant">
    <Parameters DriveType="Radius"
                PrimaryValue="10"
                CrossSection="Circular"/>
</Feature>
```

### 4.2 单距离驱动

```xml
<Feature Type="Fillet" Mode="Constant">
    <Parameters DriveType="Distances"
                PrimaryValue="14.14"
                CrossSection="Conic"
                ConicValueMode="GenericValue"
                ConicValue="0.7"/>
</Feature>
```

### 4.3 双距离驱动

```xml
<Feature Type="Fillet" Mode="Constant">
    <Parameters DriveType="Distances"
                PrimaryValue="10"
                SecondValue="12"
                CrossSection="CurvatureContinuous"
                ConicValueMode="GenericValue"
                ConicValue="0.3"/>
</Feature>
```

### 4.4 变值控制点

```xml
<Feature Type="Fillet" Mode="Variable">
    <Parameters DriveType="Distances"
                PrimaryValue="10"
                SecondValue="12"
                CrossSection="Conic"
                ConicValueMode="GenericValue"
                ConicValue="0.7"/>
    <RadiusPoints>
        <RadiusPoint Position="0"
                     PrimaryValue="10"
                     SecondValue="12"/>
        <RadiusPoint Position="1"
                     PrimaryValue="10"
                     SecondValue="12"/>
    </RadiusPoints>
</Feature>
```

### 4.5 ReferenceMode 规则

- 默认 `EdgeChain` 不写出
- 只有非默认模式才写

补充：

- XML 中 `DriveType` 当前只写两类：
  - `Radius`
  - `Distances`
- 其中：
  - 只有 `PrimaryValue` 表示单距离驱动
  - 同时有 `PrimaryValue / SecondValue` 表示双距离驱动

例如：

```xml
<Feature Type="Fillet" Mode="FullRound" ReferenceMode="FullRoundThreeFaces">
```

## 5. Creo -> CADExchange

### 5.1 普通半径圆角

当 Creo 提取到普通半径圆角时：

- `Mode = Constant` 或 `Variable`
- `DriveType = Radius`
- `PrimaryValue = radius`

### 5.2 Chordal

当 Creo 提取到弦圆角时：

- 统一归一化到 `Mode = Constant`
- `DriveType = Distances`
- 仅保留 `PrimaryValue`

shape 继续保留：

- `Circular`
- `Conic`
- `CurvatureContinuous`

示意：

```xml
<Feature Type="Fillet" Mode="Constant">
    <Parameters DriveType="Distances"
                PrimaryValue="14.14"
                CrossSection="Conic"
                ConicValueMode="GenericValue"
                ConicValue="0.7"/>
</Feature>
```

### 5.3 D1XD2

当 Creo 提取到 `D1XD2` 圆角时：

- 统一归一化到 `Mode = Constant`
- `DriveType = Distances`
- `PrimaryValue = D1`
- `SecondValue = D2`

shape 继续保留：

- `Circular`
- `Conic`
- `CurvatureContinuous`

### 5.4 Variable

当 Creo 提取到变圆角时：

- `Mode = Variable`
- `driveType` 由实际驱动方式决定：
  - `Radius`
  - `Distances`
- 每个控制点写到：
  - `radiusPoints[*].primaryValue`
  - 必要时 `radiusPoints[*].secondValue`

### 5.5 ReferenceMode

- 当前普通边圆角在内存中仍会保留 `referenceMode = EDGE_CHAIN`
- 但 XML 不再写默认 `EdgeChain`

## 6. CADExchange -> Creo

### 6.1 Constant + Radius

如果读到：

- `Mode = Constant`
- `DriveType = Radius`

则按普通半径圆角创建。

### 6.2 Constant + Distances + 只有 PrimaryValue

如果读到：

- `Mode = Constant`
- `DriveType = Distances`
- 仅有 `PrimaryValue`

则创建为：

- Creo `Chordal`

同时继续保留：

- `Circular`
- `Conic`
- `CurvatureContinuous`

### 6.3 Constant + Distances + PrimaryValue/SecondValue

如果读到：

- `Mode = Constant`
- `DriveType = Distances`
- 同时存在 `PrimaryValue` 与 `SecondValue`

则创建为：

- Creo `D1XD2`

同时继续保留：

- `Circular`
- `Conic`
- `CurvatureContinuous`

### 6.4 Variable

如果读到：

- `Mode = Variable`

则按控制点创建：

- `radiusPoints[*].primaryValue`
- `radiusPoints[*].secondValue`

## 7. Sw -> CADExchange

### 7.1 对称普通圆角

如果 SW 提取到普通对称圆角：

- `Mode = Constant` 或 `Variable`
- `DriveType = Radius`
- `PrimaryValue = radius`

### 7.2 非对称 D1/D2

如果 SW 提取到非对称距离驱动圆角：

- 先读取 `D1` 与 `D2`
- 再归一化：
  - 如果 `D1 == D2`
    - 归一化为 `DriveType = Distances`
    - 仅保留 `PrimaryValue`
  - 如果 `D1 != D2`
    - 归一化为 `DriveType = Distances`
    - 保留 `PrimaryValue = D1`
    - 保留 `SecondValue = D2`

这条规则的目标是：

- `SW asymmetric D1=D2`
  归一化为与 `Creo chordal` 相同的单距离语义

### 7.3 Variable

SW 变圆角控制点同理：

- 对称控制点
  - 仅 `PrimaryValue`
- 非对称且 `D1 = D2`
  - 只保留 `PrimaryValue`
- 非对称且 `D1 != D2`
  - 同时保留 `PrimaryValue / SecondValue`

### 7.4 Shape

SW 读取后统一映射为：

- `Circular`
- `Conic`
- `CurvatureContinuous`

并继续写：

- `ConicValueMode`
- `ConicValue`

## 8. CADExchange -> Sw

### 8.1 Radius

如果读到：

- `DriveType = Radius`

则按普通对称半径圆角创建。

### 8.2 Distances

如果读到：

- `DriveType = Distances`

则统一走 SW 非对称创建路径。

创建规则：

- 只有 `PrimaryValue`
  - `D1 = PrimaryValue`
  - `D2 = PrimaryValue`
- 同时有 `PrimaryValue / SecondValue`
  - `D1 = PrimaryValue`
  - `D2 = SecondValue`

这条规则就是当前统一结构对：

- `Creo chordal`
- `Creo D1XD2`
- `SW asymmetric D1/D2`

的标准降级与标准重建方式。

### 8.3 Shape

创建时继续保留：

- `Circular`
- `Conic`
- `CurvatureContinuous`

以及：

- `ConicValueMode`
- `ConicValue`

### 8.4 Variable RadiusPoint Position 方向归一化

当前代码已实际落地如下规则：

- `SW -> CADExchange -> SW`
- `SW -> CADExchange -> Creo`

在变半径倒圆角写侧，若目标 CAD 中被选中的重建边方向与源 XML 记录的参考边方向相反，则：

```text
mappedPosition = 1.0 - sourcePosition
```

否则：

```text
mappedPosition = sourcePosition
```

源边身份恢复方式为：

- 优先使用 `RadiusPoint.EdgeMidPoint`
- 在 fillet 顶层 `References` 中反查对应 edge
- 取其 `StartPoint / EndPoint`

若控制点没有 `EdgeMidPoint`，则回退到：

- fillet 第一条 edge reference

该规则当前已在两侧桥接中落地：

- `SwLibs/read_write/src/internal/feature_fillet.cpp`
  - 作用于 `POINTREF` 预选点
  - 作用于 `SetControlPointRadiusAtIndex(...)`
- `CreoLibs/read_write/src/internal/feature_fillet.cpp`
  - 作用于 `PRO_EDGE_PNT` 的 `uv[0]`

已验证现象：

- `varRound_test_1`
  中 `SW -> Creo` 路径上，即使 compare 报告提示：
  - `References[0].Start/End reversed`
  控制点位置仍能正确落到目标边反向后的对应参数位置
- 当前该 warning 不再演变成控制点位置错误或几何失败

## 9. 标准降级规则

当前统一结构中，以下语义已被视为标准归一化结果：

### 9.1 Creo Chordal

统一表示为：

- `Mode = Constant`
- `DriveType = Distances`
- 仅 `PrimaryValue`

### 9.2 Sw D1 = D2

统一表示为：

- `Mode = Constant`
- `DriveType = Distances`
- 仅 `PrimaryValue`

因此这两类在 `CADExchange` 内部是同一语义层级。

### 9.3 Sw 创建时的标准路径

当 `CADExchange -> SW` 遇到：

- `DriveType = Distances`
- 仅一个值

则统一按：

- SW 非对称创建
- `D1 = D2 = PrimaryValue`

这就是当前工程中：

- `Chordal -> SW`

的标准降级实现。

## 10. 当前实现状态

当前代码已完成：

- 倒圆角统一结构切换到：
  - `DriveType`
  - `PrimaryValue`
  - `SecondValue`
- 旧字段停止写出
- 旧 XML 保持兼容读取
- 默认 `ReferenceMode="EdgeChain"` 不再写出
- `Creo <-> CADExchange <-> SW` 主映射规则已按本文口径落地

## 11. 当前已知问题

截至当前验证，仍存在的剩余问题主要集中在：

- `ParentFeatureID`
  - 回读时仍可能绑定到目标 CAD 侧重建后的特征名
- `FirstEndFaceMarker`
  - 跨系统回读时可能额外出现或位置不完全一致
- `CurvatureContinuous` 的 `ConicValue`
  - 在部分 `SW` 或 `Creo` 回读路径中仍可能丢失或回成 `0`

这些问题属于桥接实现细节问题，不再是统一结构设计问题。
