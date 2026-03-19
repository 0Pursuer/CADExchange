# CADExchange Feature Builder / Accessor 详细使用参考（Sketch / Extrude / DatumPlane）

更新时间：2026-03-18

## 1. 文档定位

这不是单一 demo，而是调用参考手册。目标：

1. 覆盖主要 Builder / Accessor 的可用调用方式。
2. 以链式调用为主，补充必须用局部变量（如 sketch localID）的场景。
3. 给出 UG / Creo 参数提取后如何映射到 CADExchange 的落地示例。

## 2. 推荐包含头文件

```cpp
#include "service/builders/FeatureBuilders.h"
#include "service/accessors/FeatureAccessors.h"
```

---

## 3. SketchBuilder 详细用法

`SketchBuilder` 主要方法：

1. `SetReferencePlane(...)`
2. `SetCSys(origin, xDir, yDir, zDir)`
3. `AddLine(...)`
4. `AddCircle(...)`
5. `AddArc(...)`
6. `AddPoint(...)`
7. `AddCoincident(...)`
8. `AddHorizontal(...)`
9. `AddVertical(...)`
10. `AddTangent(...)`
11. `AddDistanceDimension(...)`
12. `Build()`

注意：

1. `AddLine/AddCircle/AddArc/AddPoint` 返回 localID，约束通常依赖 localID，所以草图会是“链式+局部变量”混合写法。
2. `SetReferencePlane` 支持 `Ref::XY()/YZ()/ZX()`、`Ref::Plane(...)`、`Ref::Face(...)`。

### 3.1 完整草图示例（Line + Circle + Arc + Point）

```cpp
using namespace CADExchange;
using namespace CADExchange::Builder;

UnifiedModel model(UnitType::METER, "usage_ref_demo");

SketchBuilder sk(model, "Sketch_Full");
sk.SetReferencePlane(Ref::XY())
  .SetCSys(CPoint3D{0, 0, 0},
           CVector3D{1, 0, 0},
           CVector3D{0, 1, 0},
           CVector3D{0, 0, 1});

// rectangle
std::string l1 = sk.AddLine(CPoint3D{0.00, 0.00, 0}, CPoint3D{0.05, 0.00, 0});
std::string l2 = sk.AddLine(CPoint3D{0.05, 0.00, 0}, CPoint3D{0.05, 0.02, 0});
std::string l3 = sk.AddLine(CPoint3D{0.05, 0.02, 0}, CPoint3D{0.00, 0.02, 0});
std::string l4 = sk.AddLine(CPoint3D{0.00, 0.02, 0}, CPoint3D{0.00, 0.00, 0});

// circle / arc / point
std::string c1 = sk.AddCircle(CPoint3D{0.025, 0.010, 0}, 0.004);
std::string a1 = sk.AddArc(CPoint3D{0.025, 0.010, 0}, 0.008, 0.0, 1.57, false);
std::string p1 = sk.AddPoint(CPoint3D{0.010, 0.010, 0});

// constraints
sk.AddHorizontal(l1)
  .AddVertical(l2)
  .AddHorizontal(l3)
  .AddVertical(l4)
  .AddCoincident(l1, l2)
  .AddCoincident(l2, l3)
  .AddCoincident(l3, l4)
  .AddCoincident(l4, l1)
  .AddTangent(a1, c1)
  .AddDistanceDimension(p1, l1, 0.010);

std::string sketchID = sk.Build();
```

### 3.2 SketchAccessor 读取参考

`SketchAccessor` 主要方法：

1. `HasReferencePlane()` / `GetReferencePlane()`
2. `GetCSys(...)`
3. `GetSegmentCount()` / `GetSegment(i)` / `GetSegmentByLocalID(id)`
4. `GetConstraintCount()` / `GetConstraint(i)`

`SketchSegmentAccessor` 主要方法：

1. `GetType()` / `GetLocalID()` / `IsConstruction()`
2. `GetLineCoords(...)`
3. `GetCircleParams(...)`
4. `GetArcParams(...)`
5. `GetPointCoord(...)`
6. `As<CSketchLine/CSketchCircle/CSketchArc/CSketchPoint>()`

```cpp
using namespace CADExchange::Accessor;

ModelAccessor ma;
ma.SetModel(model);

if (auto feat = ma.GetFeatureByID(sketchID)) {
  auto skOpt = feat->As<SketchAccessor>();
  if (skOpt.has_value()) {
    SketchAccessor skAcc = *skOpt;

    int segN = skAcc.GetSegmentCount();
    for (int i = 0; i < segN; ++i) {
      auto seg = skAcc.GetSegment(i);
      if (!seg.IsValid()) continue;

      if (seg.GetType() == CSketchSeg::SegType::ARC) {
        CPoint3D c; double s, e, r; bool cw;
        seg.GetArcParams(c, s, e, r, cw);
      }
    }
  }
}
```

---

## 4. ExtrudeBuilder 详细用法

### 4.1 Builder 主接口

`ExtrudeBuilder` 主要方法：

1. `SetProfile(sketchID)` / `SetProfileByName(sketchName)`
2. `SetDirection(...)`（自动归一化）
3. `SetOperation(BooleanOp::BOSS/CUT/MERGE)`
4. `SetEndCondition1(...)`
5. `SetEndCondition2(...)`
6. `SetDraft(angle, outward)`
7. `SetThinWall(thickness, isOneSided, isOutward, isCovered)`
8. `Build()`

### 4.2 EndCondition 工厂全部方式

`EndCondition` 支持：

1. `Blind(depth)`
2. `ThroughAll()`
3. `UpToFace(ref, offset)`
4. `UpToRefPlane(refPlane, offset)`
5. `UpToVertex(ref, offset)`
6. `UpToRefPoint(refPoint, offset)`
7. `UpToNext()`
8. `MidPlane(depth)`
9. `ThroughAllBothSides()`

#### A) Blind

```cpp
std::string extBlind =
  Builder::ExtrudeBuilder(model, "Ext_Blind")
    .SetProfile(sketchID)
    .SetDirection(CVector3D{0, 0, 1})
    .SetOperation(BooleanOp::BOSS)
    .SetEndCondition1(Builder::EndCondition::Blind(0.015))
    .Build();
```

#### B) ThroughAll

```cpp
std::string extThroughAll =
  Builder::ExtrudeBuilder(model, "Ext_ThroughAll")
    .SetProfile(sketchID)
    .SetDirection(CVector3D{0, 0, 1})
    .SetOperation(BooleanOp::CUT)
    .SetEndCondition1(Builder::EndCondition::ThroughAll())
    .Build();
```

#### C) UpToFace（拓扑面）

```cpp
auto faceRef = Builder::Ref::Face("SomeFeatureID", 0)
                 .Centroid(CPoint3D{0.01, 0.01, 0.0})
                 .Normal(CVector3D{0, 0, 1})
                 .Build();

std::string extUpToFace =
  Builder::ExtrudeBuilder(model, "Ext_UpToFace")
    .SetProfile(sketchID)
    .SetOperation(BooleanOp::BOSS)
    .SetEndCondition1(Builder::EndCondition::UpToFace(faceRef, 0.001))
    .Build();
```

#### D) UpToRefPlane（基准面）

```cpp
auto planeRef = Builder::Ref::Plane("DatumPlane_001")
                  .Origin(CPoint3D{0,0,0})
                  .XDir(CVector3D{1,0,0})
                  .YDir(CVector3D{0,1,0})
                  .Normal(CVector3D{0,0,1})
                  .Build();

std::string extUpToPlane =
  Builder::ExtrudeBuilder(model, "Ext_UpToRefPlane")
    .SetProfile(sketchID)
    .SetEndCondition1(Builder::EndCondition::UpToRefPlane(planeRef, 0.002))
    .Build();
```

#### E) UpToVertex / UpToRefPoint

```cpp
auto vertexRef = Builder::Ref::Vertex("SomeFeatureID", 2)
                   .Pos(CPoint3D{0.01, 0.02, 0.03})
                   .Build();

auto pointRef = Builder::Ref::Point("DatumPoint_001")
                  .Position(CPoint3D{0.01, 0.02, 0.03});

std::string extToVertex =
  Builder::ExtrudeBuilder(model, "Ext_UpToVertex")
    .SetProfile(sketchID)
    .SetEndCondition1(Builder::EndCondition::UpToVertex(vertexRef, 0.0))
    .Build();

std::string extToPoint =
  Builder::ExtrudeBuilder(model, "Ext_UpToRefPoint")
    .SetProfile(sketchID)
    .SetEndCondition1(Builder::EndCondition::UpToRefPoint(pointRef, 0.0))
    .Build();
```

#### F) UpToNext / MidPlane / ThroughAllBothSides

```cpp
std::string extUpToNext =
  Builder::ExtrudeBuilder(model, "Ext_UpToNext")
    .SetProfile(sketchID)
    .SetEndCondition1(Builder::EndCondition::UpToNext())
    .Build();

std::string extMidPlane =
  Builder::ExtrudeBuilder(model, "Ext_MidPlane")
    .SetProfile(sketchID)
    .SetEndCondition1(Builder::EndCondition::MidPlane(0.02))
    .Build();

std::string extBothSides =
  Builder::ExtrudeBuilder(model, "Ext_ThroughAllBothSides")
    .SetProfile(sketchID)
    .SetEndCondition1(Builder::EndCondition::ThroughAllBothSides())
    .Build();
```

#### G) 双向终止（EndCondition1 + EndCondition2）

```cpp
std::string extTwoDir =
  Builder::ExtrudeBuilder(model, "Ext_TwoDir")
    .SetProfile(sketchID)
    .SetDirection(CVector3D{0, 0, 1})
    .SetEndCondition1(Builder::EndCondition::Blind(0.01))
    .SetEndCondition2(Builder::EndCondition::Blind(0.005))
    .Build();
```

#### H) Draft / ThinWall 组合

```cpp
std::string extWithOptions =
  Builder::ExtrudeBuilder(model, "Ext_With_Draft_ThinWall")
    .SetProfile(sketchID)
    .SetEndCondition1(Builder::EndCondition::Blind(0.02))
    .SetDraft(1.5, false)
    .SetThinWall(0.0012, true, false, true)
    .Build();
```

### 4.3 ExtrudeAccessor 读取参考

`ExtrudeAccessor` 主要方法：

1. 基础：`GetProfileSketchID()` `GetDirection()` `GetOperation()`
2. 方向1：`GetEndType1()` `GetDepth1()` `GetOffset1()` `HasOffset1()` `IsFlip1()` `IsFlipMaterialSide1()` `GetReference1()`
3. 方向2：`HasDirection2()` `GetEndType2()` `GetDepth2()` `GetOffset2()` `HasOffset2()` `IsFlip2()` `IsFlipMaterialSide2()` `GetReference2()`
4. 拔模：`HasDraft()` `GetDraftAngle()` `IsDraftOutward()`
5. 薄壁：`HasThinWall()` `GetThinWallThickness()` `IsThinWallOneSided()` `IsThinWallOutward()` `IsThinWallCovered()`

```cpp
if (auto feat = ma.GetFeatureByID(extWithOptions)) {
  auto extOpt = feat->As<Accessor::ExtrudeAccessor>();
  if (extOpt.has_value()) {
    auto ext = *extOpt;

    auto dir = ext.GetDirection();
    auto op = ext.GetOperation();

    auto end1 = ext.GetEndType1();
    double d1 = ext.GetDepth1();

    bool hasDir2 = ext.HasDirection2();
    double d2 = ext.GetDepth2();

    bool hasDraft = ext.HasDraft();
    double draftA = ext.GetDraftAngle();

    bool hasThin = ext.HasThinWall();
    double thinT = ext.GetThinWallThickness();

    (void)dir; (void)op; (void)end1; (void)d1;
    (void)hasDir2; (void)d2; (void)hasDraft; (void)draftA;
    (void)hasThin; (void)thinT;
  }
}
```

---

## 5. DatumPlaneBuilder 详细用法

### 5.1 Builder 主接口

`DatumPlaneBuilder` 主要方法：

1. `SetMethod(PlaneMethod)`
2. `SetLineMethod()`
3. `AddReference(Ref::...)`
4. `SetReferences(Ref::..., Ref::..., ...)`
5. `AddReferenceAndGetIndex(Ref::...)`
6. `AddConstraint(const PlaneConstraint&)`
7. `AddConstraint(PlaneConstraintType, refIndex, value, defaultDir, reversed)`
8. `AddConstraintByFactory(PlaneConstraintBuilder::...)`
9. `AddReferenceWithConstraint(Ref::..., PlaneConstraintType, ...)`
10. `ClearConstraints()` / `ClearReferences()`
11. `Build()`

`PlaneMethod`（UnifiedFeatures.h）：

1. `OFFSET`
2. `FIXED`
3. `ANGLE`
4. `PARALLEL`
5. `PERPENDICULAR`
6. `MID_PLANE`
7. `THREE_POINTS`
8. `LINE`
9. `TANGENT`

### 5.2 PlaneConstraintBuilder 全方式

1. `Parallel(ref, reversed)`
2. `Perpendicular(ref, reversed)`
3. `Coincident(ref, reversed)`
4. `Distance(ref, value, defaultDir, reversed)`
5. `Angle(ref, value, defaultDir, reversed)`
6. `Symmetric(ref, reversed)`
7. `Tangent(ref, reversed)`
8. `Projection(ref, reversed)`

### 5.3 典型示例

#### A) OFFSET

```cpp
// defaultDir: 建议传入“期望偏移正方向”用于跨CAD稳定重建
// reversed:   当源CAD标记反向时可置 true
std::optional<CVector3D> offsetDir = CVector3D{0, 0, 1};

std::string dpOffset =
  Builder::DatumPlaneBuilder(model, "Datum_Offset")
    .SetMethod(PlaneMethod::OFFSET)
    .AddReference(Builder::Ref::XY())
    .AddConstraint(PlaneConstraintType::DISTANCE, 0, 0.01, offsetDir, false)
    .Build();
```

#### B) ANGLE

```cpp
// 角度方向建议显式传 defaultDir，按右手定则解释旋转方向：
// 右手四指从参考面转向目标面，拇指指向 defaultDir。
std::optional<CVector3D> angleDir = CVector3D{0, 0, 1};

std::string dpAngle =
  Builder::DatumPlaneBuilder(model, "Datum_Angle")
    .SetMethod(PlaneMethod::ANGLE)
    .AddReference(Builder::Ref::YZ())
    .AddConstraint(PlaneConstraintType::ANGLE, 0, 15.0, angleDir, false)
    .Build();
```

#### C) PARALLEL / PERPENDICULAR

```cpp
std::string dpParallel =
  Builder::DatumPlaneBuilder(model, "Datum_Parallel")
    .SetMethod(PlaneMethod::PARALLEL)
    .AddReference(Builder::Ref::Plane("DatumPlane_001"))
    .AddConstraintByFactory(Builder::PlaneConstraintBuilder::Parallel(0))
    .Build();

std::string dpPerp =
  Builder::DatumPlaneBuilder(model, "Datum_Perp")
    .SetMethod(PlaneMethod::PERPENDICULAR)
    .AddReference(Builder::Ref::Axis("DatumAxis_001"))
    .AddConstraintByFactory(Builder::PlaneConstraintBuilder::Perpendicular(0))
    .Build();
```

#### D) MID_PLANE（双引用）

```cpp
Builder::DatumPlaneBuilder dpMid(model, "Datum_Mid");
int r0 = dpMid.AddReferenceAndGetIndex(Builder::Ref::Plane("DatumPlane_A"));
int r1 = dpMid.AddReferenceAndGetIndex(Builder::Ref::Plane("DatumPlane_B"));

dpMid.SetMethod(PlaneMethod::MID_PLANE)
  .AddConstraintByFactory(Builder::PlaneConstraintBuilder::Symmetric(r0))
  .AddConstraintByFactory(Builder::PlaneConstraintBuilder::Symmetric(r1));

std::string dpMidID = dpMid.Build();
```

#### E) THREE_POINTS（三点）

```cpp
Builder::DatumPlaneBuilder dp3p(model, "Datum_3Points");
int p0 = dp3p.AddReferenceAndGetIndex(Builder::Ref::Point("DatumPoint_001"));
int p1 = dp3p.AddReferenceAndGetIndex(Builder::Ref::Point("DatumPoint_002"));
int p2 = dp3p.AddReferenceAndGetIndex(Builder::Ref::Point("DatumPoint_003"));

dp3p.SetMethod(PlaneMethod::THREE_POINTS)
   .AddConstraint(PlaneConstraintType::COINCIDENT, p0)
   .AddConstraint(PlaneConstraintType::COINCIDENT, p1)
   .AddConstraint(PlaneConstraintType::COINCIDENT, p2);

std::string dp3pID = dp3p.Build();
```

#### F) TANGENT（面相切）

```cpp
std::string dpTangent =
  Builder::DatumPlaneBuilder(model, "Datum_Tangent")
    .SetMethod(PlaneMethod::TANGENT)
    .AddReference(Builder::Ref::Face("BodyFeature_001", 0))
    .AddConstraintByFactory(Builder::PlaneConstraintBuilder::Tangent(0))
    .Build();
```

#### G) LINE（通过两条线/两条边）

```cpp
// 典型场景：UG / NX 的 MethodTypeLine，可由两条线性边定义一个基准面。
// 这里显式使用 SetLineMethod()，与 SetMethod(PlaneMethod::LINE) 等价。
std::string dpLine =
  Builder::DatumPlaneBuilder(model, "Datum_Line")
    .SetLineMethod()
    .AddReference(Builder::Ref::Edge("Boss_Extrude", 4))
    .AddReference(Builder::Ref::Edge("Boss_Extrude", 7))
    .AddConstraintByFactory(Builder::PlaneConstraintBuilder::Coincident(0))
    .AddConstraintByFactory(Builder::PlaneConstraintBuilder::Coincident(1))
    .Build();
```

### 5.4 DatumPlaneAccessor 读取参考

`DatumPlaneAccessor` 主要方法：

1. `GetMethod()`
2. `IsLineMethod()`
3. `HasReferences()` `GetReferenceCount()` `GetReference(i)` `GetReferenceEntities()`
4. `HasConstraints()` `GetConstraintCount()` `GetConstraint(i)` `GetConstraints()`

```cpp
if (auto feat = ma.GetFeatureByID(dpOffset)) {
  auto dpOpt = feat->As<Accessor::DatumPlaneAccessor>();
  if (dpOpt.has_value()) {
    auto dp = *dpOpt;

    PlaneMethod m = dp.GetMethod();
    bool isLine = dp.IsLineMethod();
    int rn = dp.GetReferenceCount();
    int cn = dp.GetConstraintCount();

    for (int i = 0; i < cn; ++i) {
      PlaneConstraint c = dp.GetConstraint(i);
      // c.type / c.ref / c.value / c.defaultDir / c.reversed
    }

    (void)m; (void)rn; (void)cn;
  }
}
```

---

## 6. UG 提取参数 -> Builder 映射参考

来源：`UgLibs/doc/ug_datum_plane_read_create_research_20260318.md`

### 6.1 UG fixed_dplane

UG 读取到：

1. `origin[3]`
2. `normal[3]`

当前 CADExchange（现有字段）可先映射为 `FIXED + (参考面 + 0 距离)` 的最小表达：

```cpp
std::string fromUgFixed =
  Builder::DatumPlaneBuilder(model, "Datum_From_UG_Fixed")
    .SetMethod(PlaneMethod::FIXED)
    .AddReference(Builder::Ref::XY())
    .AddConstraintByFactory(Builder::PlaneConstraintBuilder::Distance(0, 0.0))
    .Build();
```

如果后续 `CDatumPlane` 增加 point/normal 直存字段，应优先改成直接写 point/normal。

### 6.2 UG relative_dplane

UG 读取到：

1. `object_tags[]` -> 映射到 `Ref::*`
2. `offset_string` -> 解析成 `double offsetValue`
3. `angle_string` -> 解析成 `double angleValue`
4. `which_plane/reference_point` -> 可先映射到 `reversed/defaultDir`（能力范围内）

偏置示例：

```cpp
double offsetValue = 0.012;
std::optional<CVector3D> offsetDir = CVector3D{0, 0, 1};
std::string fromUgOffset =
  Builder::DatumPlaneBuilder(model, "Datum_From_UG_Offset")
    .SetMethod(PlaneMethod::OFFSET)
    .AddReference(Builder::Ref::Plane("UG_REF_PLANE"))
    .AddConstraint(PlaneConstraintType::DISTANCE, 0, offsetValue, offsetDir, false)
    .Build();
```

角度示例：

```cpp
double angleValue = 20.0;
// 方向由读取侧统一到右手定则语义后写入
std::optional<CVector3D> angleDir = CVector3D{0, 0, 1};
std::string fromUgAngle =
  Builder::DatumPlaneBuilder(model, "Datum_From_UG_Angle")
    .SetMethod(PlaneMethod::ANGLE)
    .AddReference(Builder::Ref::Plane("UG_REF_PLANE"))
    .AddConstraint(PlaneConstraintType::ANGLE, 0, angleValue, angleDir, false)
    .Build();
```

---

## 7. Creo 提取参数 -> Builder 映射参考

来源：`CreoLibs/doc/creo_datum_plane_read_create_research_20260318.md`

### 7.1 PRO_DTMPLN_OFFS

Creo 读取到：

1. `PRO_E_DTMPLN_CONSTR_REF`
2. `PRO_E_DTMPLN_CONSTR_REF_OFFSET`
3. `PRO_E_DTMPLN_FLIP_DIR`

```cpp
double offsetValue = 0.008;
bool flipDir = false;
std::optional<CVector3D> offsetDir = CVector3D{0, 0, 1};

std::string fromCreoOffs =
  Builder::DatumPlaneBuilder(model, "Datum_From_Creo_OFFS")
    .SetMethod(PlaneMethod::OFFSET)
    .AddReference(Builder::Ref::Plane("CREO_REF_PLANE"))
    .AddConstraint(PlaneConstraintType::DISTANCE, 0, offsetValue, offsetDir, flipDir)
    .Build();
```

### 7.2 PRO_DTMPLN_ANG

Creo 读取到：

1. `PRO_E_DTMPLN_CONSTR_REF`
2. `PRO_E_DTMPLN_CONSTR_REF_ANGLE`
3. `PRO_E_DTMPLN_FLIP_DIR`

```cpp
double angleValue = 15.0;
bool flipDir = true;
// 对 Creo 场景建议将 flip 与 defaultDir 组合映射到统一右手定则方向
std::optional<CVector3D> angleDir = CVector3D{0, 0, 1};

std::string fromCreoAng =
  Builder::DatumPlaneBuilder(model, "Datum_From_Creo_ANG")
    .SetMethod(PlaneMethod::ANGLE)
    .AddReference(Builder::Ref::Plane("CREO_REF_PLANE"))
    .AddConstraint(PlaneConstraintType::ANGLE, 0, angleValue, angleDir, flipDir)
    .Build();
```

### 7.3 PRO_DTMPLN_PRL / MIDPLN / TANG（建议映射）

1. `PRO_DTMPLN_PRL` -> `PlaneMethod::PARALLEL` + `PlaneConstraintType::PARALLEL`
2. `PRO_DTMPLN_MIDPLN` -> `PlaneMethod::MID_PLANE` + 双引用 + `SYMMETRIC`
3. `PRO_DTMPLN_TANG` -> `PlaneMethod::TANGENT` + `PlaneConstraintType::TANGENT`

---

## 8. 常见调用建议

1. 草图先 `Build` 再拉伸，优先用 `SetProfile(sketchID)`。
2. 终止面/点/顶点尽量使用 `Ref::*` 包装，避免直接操作底层引用结构。
3. 基准面约束里 `ref` 是“referenceEntities 下标”，建议用 `AddReferenceAndGetIndex(...)` 获取，避免硬编码。
4. Accessor 回读时优先检查 `IsValid()/Has*()` 再取值。

## 9. 最小验收建议

1. Builder 创建后调用 `UnifiedModel::Validate()`。
2. 通过 Accessor 回读关键字段：
   1. Sketch: segment/constraint 数量和类型。
   2. Extrude: endCondition 类型、深度、引用。
   3. DatumPlane: method、referenceCount、constraint(type/ref/value/reversed)。
3. 对桥接模块执行你现有的 round-trip 五步校验链路。
