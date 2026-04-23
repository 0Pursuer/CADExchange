# CADExchange Feature Builder / Accessor 详细使用参考（Sketch / Extrude / Revolve / Sweep / DatumPlane）

更新时间：2026-03-29

> 注：自 `2026-03-29` 起，`Extrude / Revolve` 已统一到共享的 `SweepExtent` 结构。
> 旧的 `Revolve.AngleKind/PrimaryAngle/SecondaryAngle` 已被 `extent1/extent2` 取代，旧 XML 也不再兼容这些字段。
> `ExtrudeBuilder::SetEndCondition1/2(...)` 当前仍是主入口，但参数类型已经是 `SweepExtent`；
> `Builder::EndCondition::*` 和 `Builder::Extent::*` 都只是在构造 `SweepExtent`，不再对应旧的 `ExtrudeEndCondition` 结构体。
> 对 `RevolveAccessor` 而言，访问接口已经直接切换到 `GetExtentType1/2`、`GetExtentValue1/2`、
> `GetExtentOffset1/2`、`HasOffset1/2`、`GetReference1/2` 等共享 `SweepExtent` 读取接口，
> 不再保留旧的角度语法糖接口。
> `Revolve` 的角度值自 `2026-04-07` 起在 `CADExchange` 内部统一使用弧度；Creo / UG 的读写桥接在边界处负责与原生“度”互转。

## 1. 文档定位

这不是单一 demo，而是调用参考手册。目标：

1. 覆盖主要 Builder / Accessor 的可用调用方式。
2. 以链式调用为主，补充必须用局部变量（如 sketch localID）的场景。
3. 优先记录当前推荐接口，对仍保留的兼容命名明确标注。
4. 给出 UG / Creo 参数提取后如何映射到 CADExchange 的落地示例。

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
3. 对草图来说，`Build()` 前完全不落局部变量通常不可行；只要 localID 捕获是为了继续链式加约束，这仍然属于推荐写法。

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
4. `SetEndCondition1(const SweepExtent&)`
5. `SetEndCondition2(const SweepExtent&)`
6. `SetDraft(angle, outward)`
7. `SetThinWallOffsets(startOffset, endOffset, isCovered)`
8. `Build()`

说明：

1. `ExtrudeBuilder` 当前统一写入 `extent1 / extent2`。
2. `SetEndCondition1/2` 这个命名仍保留，但参数已经是 `SweepExtent`。
3. `Builder::EndCondition::*` 和 `Builder::Extent::*` 都只是便捷工厂，返回值本质也是 `SweepExtent`。
4. 薄壁统一使用 `StartOffset/EndOffset/Covered`：
   `StartOffset` 表示相对草图轮廓的内侧偏置，向内为负；
   `EndOffset` 表示相对草图轮廓的外侧偏置，向外为正。

### 4.2 推荐写法：通过 Extent / EndCondition / EndConditionHelper 获取 SweepExtent

当前没有独立的 `SweepExtentBuilder`。在调用侧，**不建议手工修改 `SweepExtent` 底层字段**，
否则会破坏 Builder 模式“链式调用 + 屏蔽底层结构细节”的设计目标。

因此，推荐做法是：

1. 优先通过 `Builder::Extent::*` 获取通用范围
2. `Extrude` 场景下也可以继续使用 `Builder::EndCondition::*`
3. 需要引用实体时，通过 `Builder::Ref::*` 或 `EndConditionHelper::*` 先构造引用
4. 再把结果传给 `SetEndCondition1/2`

#### A) 通用 VALUE

```cpp
std::string extBlind =
  Builder::ExtrudeBuilder(model, "Ext_Blind")
    .SetProfile(sketchID)
    .SetDirection(CVector3D{0, 0, 1})
    .SetOperation(BooleanOp::BOSS)
    .SetEndCondition1(Builder::Extent::Value(0.015))
    .Build();
```

#### B) 通用 UP_TO_ENTITY（到面）

```cpp
auto faceRef = Builder::Ref::Face("SomeFeatureID", 0)
                 .Centroid(CPoint3D{0.01, 0.01, 0.0})
                 .Normal(CVector3D{0, 0, 1})
                 .Build();

std::string extUpToFace =
  Builder::ExtrudeBuilder(model, "Ext_UpToFace")
    .SetProfile(sketchID)
    .SetOperation(BooleanOp::BOSS)
    .SetEndCondition1(Builder::Extent::UpToEntity(faceRef, 0.001))
    .Build();
```

#### C) 通用双向范围

```cpp
std::string extTwoDir =
  Builder::ExtrudeBuilder(model, "Ext_TwoDir")
    .SetProfile(sketchID)
    .SetDirection(CVector3D{0, 0, 1})
    .SetEndCondition1(Builder::Extent::Value(0.01))
    .SetEndCondition2(Builder::Extent::Value(0.005))
    .Build();
```

#### D) 通用 ThroughAll / UpToNext / ThroughAllBothSides

```cpp
std::string extThroughAll =
  Builder::ExtrudeBuilder(model, "Ext_ThroughAll_ByExtent")
    .SetProfile(sketchID)
    .SetDirection(CVector3D{0, 0, 1})
    .SetOperation(BooleanOp::CUT)
    .SetEndCondition1(Builder::Extent::ThroughAll())
    .Build();

std::string extUpToNext =
  Builder::ExtrudeBuilder(model, "Ext_UpToNext_ByExtent")
    .SetProfile(sketchID)
    .SetDirection(CVector3D{0, 0, 1})
    .SetOperation(BooleanOp::BOSS)
    .SetEndCondition1(Builder::Extent::UpToNext())
    .Build();

std::string extBothSides =
  Builder::ExtrudeBuilder(model, "Ext_BothSides_ByExtent")
    .SetProfile(sketchID)
    .SetDirection(CVector3D{0, 0, 1})
    .SetOperation(BooleanOp::CUT)
    .SetEndCondition1(Builder::Extent::ThroughAllBothSides())
    .Build();
```

### 4.3 兼容工厂：EndCondition / EndConditionHelper

`Builder::EndCondition` 当前支持：

1. `Blind(depth)`
2. `ThroughAll()`
3. `UpToFace(ref, offset)`
4. `UpToRefPlane(refPlane, offset)`
5. `UpToVertex(ref, offset)`
6. `UpToRefPoint(refPoint, offset)`
7. `UpToNext()`
8. `ThroughAllBothSides()`

说明：

1. `Builder::EndCondition::*` 主要为 `ExtrudeBuilder` 保留旧调用风格。
2. 新代码若同时覆盖 `Extrude / Revolve`，优先统一使用 `Builder::Extent::*`。
3. 不要手工写 `SweepExtent e; e.type = ...;` 这类底层赋值。

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

#### F) UpToNext / ThroughAllBothSides

```cpp
std::string extUpToNext =
  Builder::ExtrudeBuilder(model, "Ext_UpToNext")
    .SetProfile(sketchID)
    .SetEndCondition1(Builder::EndCondition::UpToNext())
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
    .SetThinWallOffsets(-0.0012, 0.0, true)
    .Build();
```

### 4.4 ExtrudeAccessor 读取参考

`ExtrudeAccessor` 主要方法：

1. 基础：`GetProfileSketchID()` `GetDirection()` `GetOperation()`
2. 推荐直接访问：`Data()->extent1` / `Data()->extent2`
3. 第一方向兼容读取：`GetEndType1()` `GetDepth1()` `GetOffset1()` `HasOffset1()` `IsFlip1()` `IsFlipMaterialSide1()` `GetReference1()`
4. 第二方向兼容读取：`HasDirection2()` `GetEndType2()` `GetDepth2()` `GetOffset2()` `HasOffset2()` `IsFlip2()` `IsFlipMaterialSide2()` `GetReference2()`
5. 拔模：`HasDraft()` `GetDraftAngle()` `IsDraftOutward()`
6. 薄壁：`HasThinWall()` `GetThinWallStartOffset()` `GetThinWallEndOffset()` `IsThinWallCovered()`

建议：

1. 调试或桥接逻辑里，优先读取 `Data()->extent1 / extent2`，因为它保留了完整共享语义。
2. 若只是业务层轻量判断，可继续使用 `GetEndType1 / GetDepth1` 这组兼容命名。

```cpp
if (auto feat = ma.GetFeatureByID(extWithOptions)) {
  auto extOpt = feat->As<Accessor::ExtrudeAccessor>();
  if (extOpt.has_value()) {
    auto ext = *extOpt;

    auto dir = ext.GetDirection();
    auto op = ext.GetOperation();

    SweepExtent e1 = ext.Data()->extent1;
    bool hasDir2 = ext.HasDirection2();
    std::optional<SweepExtent> e2 = ext.Data()->extent2;

    bool hasDraft = ext.HasDraft();
    double draftA = ext.GetDraftAngle();

    bool hasThin = ext.HasThinWall();
    double thinStart = ext.GetThinWallStartOffset();
    double thinEnd = ext.GetThinWallEndOffset();

    // 兼容命名接口仍可用，但推荐优先读取 e1/e2：
    auto compatType1 = ext.GetEndType1();
    double compatValue1 = ext.GetDepth1();

    (void)dir; (void)op; (void)e1; (void)hasDir2; (void)e2;
    (void)hasDraft; (void)draftA; (void)compatType1; (void)compatValue1;
    (void)hasThin; (void)thinStart; (void)thinEnd;
  }
}
```

---

## 5. RevolveBuilder 详细用法

### 5.1 Builder 主接口

`RevolveBuilder` 主要方法：

1. `SetProfile(sketchID)`
2. `SetAxisFromSketchLine(localSketchLineID)`
3. `SetAxisExplicit(origin, direction)`
4. `SetAxisRef(refEntity)`
5. `SetOperation(BooleanOp::BOSS/CUT/MERGE)`
6. `SetExtent1(const SweepExtent&)`
7. `SetExtent2(const SweepExtent&)`
8. `SetAngle(angle)` / `SetTwoWayAngle(angle1, angle2)` / `SetSymmetricAngle(totalAngle)`
9. `SetThinWallOffsets(startOffset, endOffset, isCovered)`
10. `Build()`

说明：

1. `Revolve` 已不再使用 `AngleKind / PrimaryAngle / SecondaryAngle` 作为模型层字段。
2. 推荐优先使用 `SetExtent1/2`，这样与 `Extrude` 的共享语义完全一致。
3. `SetAngle / SetTwoWayAngle / SetSymmetricAngle` 仍可用，但只是对 `extent1 / extent2` 的语法糖，输入角度统一使用弧度。
4. 若想保持和旧 `EndCondition` 类似的调用风格，优先使用 `Builder::Extent::*`。
5. `SetExtent1/2` 接收到 `SweepExtent::Type::UNKNOWN` 时会忽略该输入，不会覆盖已设置值。
6. 薄壁统一使用 `StartOffset/EndOffset/Covered`：
   `StartOffset` 表示相对草图轮廓的内侧偏置，向内为负；
   `EndOffset` 表示相对草图轮廓的外侧偏置，向外为正。
7. `Builder::Extent::Angle(...)` / `Builder::Extent::Symmetric(...)` / `GetExtentValue1/2()` 的 Revolve 角度值也统一使用弧度。

### 5.2 典型示例

#### A) 单向角度

```cpp
std::string revSingle =
  Builder::RevolveBuilder(model, "Rev_Single")
    .SetProfile(sketchID)
    .SetAxisExplicit(CPoint3D{0, 0, 0}, CVector3D{0, 0, 1})
    .SetOperation(BooleanOp::BOSS)
    .SetAngle(CADExchange::DegreesToRadians(360.0))
    .Build();
```

#### B) 双向角度

```cpp
std::string revTwoWay =
  Builder::RevolveBuilder(model, "Rev_TwoWay")
    .SetProfile(sketchID)
    .SetAxisExplicit(CPoint3D{0, 0, 0}, CVector3D{0, 1, 0})
    .SetOperation(BooleanOp::CUT)
    .SetTwoWayAngle(CADExchange::DegreesToRadians(120.0),
                    CADExchange::DegreesToRadians(30.0))
    .Build();
```

#### C) 通过 Axis + Extent 保持链式风格

```cpp
std::string revExtent =
  Builder::RevolveBuilder(model, "Rev_ByExtent")
    .SetProfile(sketchID)
    .SetAxisRef(Builder::Ref::Axis("DatumAxis_001"))
    .SetOperation(BooleanOp::BOSS)
    .SetExtent1(Builder::Extent::Angle(CADExchange::DegreesToRadians(270.0)))
    .Build();
```

#### D) 对称旋转

```cpp
std::string revSym =
  Builder::RevolveBuilder(model, "Rev_Symmetric")
    .SetProfile(sketchID)
    .SetAxisFromSketchLine("CenterLine_1")
    .SetExtent1(Builder::Extent::Symmetric(CADExchange::DegreesToRadians(180.0)))
    .Build();
```

#### E) 通过引用终止范围（保持与 Extrude 一致）

```cpp
std::string revUpToEntity =
  Builder::RevolveBuilder(model, "Rev_UpToEntity")
    .SetProfile(sketchID)
    .SetAxisRef(
      Builder::Ref::Axis("DatumAxis_001")
        .Origin(CPoint3D{0, 0, 0})
        .Direction(CVector3D{0, 0, 1}))
    .SetOperation(BooleanOp::BOSS)
    .SetExtent1(
      Builder::Extent::UpToEntity(
        Builder::Ref::Plane("LimitPlane_001")
          .Origin(CPoint3D{0, 0, 0})
          .XDir(CVector3D{1, 0, 0})
          .YDir(CVector3D{0, 1, 0})
          .Normal(CVector3D{0, 0, 1}),
        0.002))
    .Build();
```

### 5.3 RevolveAccessor 读取参考

`RevolveAccessor` 主要方法：

1. 基础：`GetProfileSketchID()` `GetOperation()`
2. 旋转轴：`GetAxisOrigin()` `GetAxisDirection()` `GetAxisReference()`
3. 范围1：`GetExtentType1()` `GetExtentValue1()` `GetExtentOffset1()` `HasOffset1()` `IsFlip1()` `IsFlipMaterialSide1()` `GetReference1()`
4. 范围2：`HasExtent2()` `GetExtentType2()` `GetExtentValue2()` `GetExtentOffset2()` `HasOffset2()` `IsFlip2()` `IsFlipMaterialSide2()` `GetReference2()`
5. 薄壁：`HasThinWall()` `GetThinWallStartOffset()` `GetThinWallEndOffset()` `IsThinWallCovered()`
6. 推荐直接访问：`Data()->extent1` / `Data()->extent2`

建议：

1. 若只关心角度值，`GetExtentValue1/2()` 足够；返回值单位统一是弧度。
2. 若要处理与 `Extrude` 共用的 offset / flip / reference 语义，必须同时读取 `HasOffset* / IsFlip* / GetReference*`。

```cpp
if (auto feat = ma.GetFeatureByID(revSingle)) {
  auto revOpt = feat->As<Accessor::RevolveAccessor>();
  if (revOpt.has_value()) {
    auto rev = *revOpt;

    std::string profile = rev.GetProfileSketchID();
    auto op = rev.GetOperation();

    CPoint3D axisOrigin = rev.GetAxisOrigin();
    CVector3D axisDir = rev.GetAxisDirection();
    auto axisRef = rev.GetAxisReference();

    SweepExtent e1 = rev.Data()->extent1;
    auto t1 = rev.GetExtentType1();
    double v1 = rev.GetExtentValue1();
    bool hasOffset1 = rev.HasOffset1();
    auto ref1 = rev.GetReference1();

    bool hasE2 = rev.HasExtent2();
    double v2 = rev.GetExtentValue2();
    double o2 = rev.GetExtentOffset2();
    bool flip2 = rev.IsFlip2();
    auto ref2 = rev.GetReference2();

    (void)profile; (void)op; (void)axisOrigin; (void)axisDir;
    (void)axisRef; (void)e1; (void)t1; (void)v1; (void)hasOffset1;
    (void)ref1; (void)hasE2; (void)v2; (void)o2; (void)flip2; (void)ref2;
  }
}
```

---

## 6. SweepBuilder 详细用法

### 6.1 Builder 主接口

`SweepBuilder` 当前按 V1 unified sweep 定义实现，只覆盖三系统都容易落地的实体扫掠基础语义：

1. `SetProfile(sketchID)` / `SetProfileByName(sketchName)`：外部草图引用截面（`SketchReference`）
2. `SetEmbeddedProfile(sketch)`：内嵌截面草绘（`EmbeddedSketch`）
3. `SetCircularProfile(outerRadius, innerRadius)`：参数圆/管截面（`Circular`）
4. `SetSectionPlacement(SweepSectionPlacement::ExistingProfilePlane/PathNormalAtStart)`
5. `SetProfilePathAngleCos(cosValue)`（可选）
6. `AddPathReference(refEntity)`
7. `SetPathReferences(refEntities)`
8. `SetPathStartPoint(point)` / `SetPathEndPoint(point)`
9. `SetPathClosed(isClosed)`
10. `AddGuidePathReference(refEntity, guidePathIndex)` / `AddGuidePath(guidePath)`
11. `SetGuidePathStartPoint(index, point)` / `SetGuidePathEndPoint(index, point)`
12. `SetGuidePathClosed(index, isClosed)`
13. `SetOrientation(SweepPathOrientation::FollowPath/KeepProfileNormal)`
14. `SetOperation(BooleanOp::BOSS/CUT/MERGE)`
15. `SetThinWallOffsets(startOffset, endOffset, isCovered)`
16. `Build()`

说明：

1. Sweep 轮廓改为 tagged-union：`SketchReference / EmbeddedSketch / Circular`；仅在 `SketchReference` 下要求外部 `profileSketchID`。
2. `path` 使用 `CRefEntityBase` 引用链，不限定为草图；可以是 `Ref::Sketch(...)`、`Ref::SketchSegment(...)`、`Ref::Edge(...)` 等可解析为曲线链的引用。
3. `guidePaths` 是可选参数，元素同样使用 `CSweepPath`，因此引导线与主扫掠路径使用完全相同的引用实体表达方式。
4. `sectionPlacement` 用于表达截面相对路径位置语义：`ExistingProfilePlane` 或 `PathNormalAtStart`。
5. 当前 V1 不表达曲面扫掠、centerline、twist、draft along path 等高级选项。

### 6.2 典型写法：草图圆轮廓 + 草图段路径

```cpp
using namespace CADExchange;
using namespace CADExchange::Builder;

std::string sweepID =
  Builder::SweepBuilder(model, "Sweep_Round")
    .SetProfile(profileSketchID)
    .SetOperation(BooleanOp::BOSS)
    .AddPathReference(Builder::Ref::SketchSegment(pathSketchID, pathLineLocalID, 0))
    .SetPathStartPoint(CPoint3D{0.0, 0.0, 0.0})
    .SetPathEndPoint(CPoint3D{0.05, 0.0, 0.0})
    .SetOrientation(SweepPathOrientation::FollowPath)
    .Build();
```

### 6.2.1 典型写法：圆形参数截面（无 profile sketch）

```cpp
std::string circularSweepID =
  Builder::SweepBuilder(model, "Sweep_Circular")
    .SetCircularProfile(0.004, 0.002) // 外半径/内半径（内半径 0 表示实心）
    .SetSectionPlacement(SweepSectionPlacement::PathNormalAtStart)
    .AddPathReference(Builder::Ref::SketchSegment(pathSketchID, pathLineLocalID, 0))
    .Build();
```

### 6.2.2 典型写法：内嵌截面草绘（Creo/UG 变化扫掠语义）

```cpp
CSketch section;
section.sketchCSys.origin = CPoint3D{0.0, 0.0, 0.0};
section.sketchCSys.xDir = CVector3D{1.0, 0.0, 0.0};
section.sketchCSys.yDir = CVector3D{0.0, 1.0, 0.0};
section.sketchCSys.zDir = CVector3D{0.0, 0.0, 1.0};
// 省略 section.segments 构造（至少 1 个段）

std::string embeddedSweepID =
  Builder::SweepBuilder(model, "Sweep_Embedded")
    .SetEmbeddedProfile(section)
    .SetSectionPlacement(SweepSectionPlacement::PathNormalAtStart)
    .AddPathReference(Builder::Ref::SketchSegment(pathSketchID, pathLineLocalID, 0))
    .Build();
```

### 6.3 典型写法：实体边作为路径

路径不是草图时，不要设计 `pathSketchID` 这类专用字段；统一通过引用实体表达：

```cpp
std::string sweepByEdge =
  Builder::SweepBuilder(model, "Sweep_By_Model_Edge")
    .SetProfile(profileSketchID)
    .SetOperation(BooleanOp::CUT)
    .AddPathReference(
      Builder::Ref::Edge("Base_Extrude", 3)
        .StartPoint(CPoint3D{0.0, 0.0, 0.0})
        .EndPoint(CPoint3D{0.05, 0.0, 0.0})
        .MidPoint(CPoint3D{0.025, 0.0, 0.0}))
    .SetPathStartPoint(CPoint3D{0.0, 0.0, 0.0})
    .SetPathEndPoint(CPoint3D{0.05, 0.0, 0.0})
    .Build();
```

### 6.4 可选引导线

引导线与主路径一样使用 `CRefEntityBase` 引用链。没有引导线时不需要设置；有一条引导线时可以直接使用默认 `guidePathIndex=0`：

```cpp
std::string guidedSweep =
  Builder::SweepBuilder(model, "Sweep_With_Guide")
    .SetProfile(profileSketchID)
    .AddPathReference(Builder::Ref::SketchSegment(pathSketchID, pathLineLocalID, 0))
    .AddGuidePathReference(Builder::Ref::SketchSegment(guideSketchID, guideLineLocalID, 0))
    .SetGuidePathStartPoint(0, CPoint3D{0.0, 0.01, 0.0})
    .SetGuidePathEndPoint(0, CPoint3D{0.05, 0.01, 0.0})
    .Build();
```

多条引导线可以通过不同 `guidePathIndex` 分组：

```cpp
std::string multiGuideSweep =
  Builder::SweepBuilder(model, "Sweep_With_Two_Guides")
    .SetProfile(profileSketchID)
    .AddPathReference(Builder::Ref::SketchSegment(pathSketchID, pathLineLocalID, 0))
    .AddGuidePathReference(Builder::Ref::SketchSegment(guideSketchA, guideLineA, 0), 0)
    .AddGuidePathReference(Builder::Ref::SketchSegment(guideSketchB, guideLineB, 0), 1)
    .Build();
```

### 6.5 薄壁扫掠

薄壁字段沿用 `Extrude/Revolve` 的 `ThinWallOption` 语义，统一使用 `StartOffset/EndOffset/Covered`：

```cpp
std::string thinSweep =
  Builder::SweepBuilder(model, "Sweep_Thin")
    .SetProfile(profileSketchID)
    .AddPathReference(Builder::Ref::SketchSegment(pathSketchID, pathLineLocalID, 0))
    .SetThinWallOffsets(-0.001, 0.001, false)
    .Build();
```

### 6.6 SweepAccessor 读取参考

`SweepAccessor` 主要方法：

1. 轮廓语义：`GetProfileKind()` `GetProfileSketchID()` `GetSectionPlacement()` `HasProfilePathAngleCos()` `GetProfilePathAngleCos()`
2. 圆形参数截面：`HasCircularProfile()` `GetCircularOuterRadius()` `GetCircularInnerRadius()`
3. 内嵌截面草绘：`HasEmbeddedProfile()` `GetEmbeddedProfileSegmentCount()` `GetEmbeddedProfileSketch()`
4. 路径：`GetPathReferenceCount()` `GetPathReference(i)` `IsPathClosed()`
5. 路径方向消歧：`HasPathStartPoint()` `GetPathStartPoint()` `HasPathEndPoint()` `GetPathEndPoint()`
6. 引导线：`GetGuidePathCount()` `GetGuidePathReferenceCount(i)` `GetGuidePathReference(i,j)` `HasGuidePathStartPoint(i)` `HasGuidePathEndPoint(i)`
7. 薄壁：`HasThinWall()` `GetThinWallStartOffset()` `GetThinWallEndOffset()` `IsThinWallCovered()`
8. 推荐直接读取：`Data()->path.references` 与 `Data()->guidePaths`，用于桥接层保留完整路径引用链顺序。

```cpp
using namespace CADExchange::Accessor;

if (auto feat = ma.GetFeatureByID(sweepID)) {
  auto sweepOpt = feat->As<Accessor::SweepAccessor>();
  if (sweepOpt.has_value()) {
    auto sweep = *sweepOpt;

    auto profileKind = sweep.GetProfileKind();
    std::string profile = sweep.GetProfileSketchID();
    auto placement = sweep.GetSectionPlacement();
    int pathN = sweep.GetPathReferenceCount();
    auto firstPathRef = sweep.GetPathReference(0);
    int guideN = sweep.GetGuidePathCount();
    auto firstGuideRef = sweep.GetGuidePathReference(0, 0);
    bool hasStart = sweep.HasPathStartPoint();
    CPoint3D start = sweep.GetPathStartPoint();
    auto orientation = sweep.GetOrientation();
    double outerR = sweep.GetCircularOuterRadius();
    int embeddedSegCount = sweep.GetEmbeddedProfileSegmentCount();

    (void)profileKind; (void)profile; (void)placement;
    (void)pathN; (void)firstPathRef;
    (void)guideN; (void)firstGuideRef; (void)hasStart;
    (void)start; (void)orientation;
    (void)outerR; (void)embeddedSegCount;
  }
}
```

---

## 7. DatumPlaneBuilder 详细用法

### 7.1 Builder 主接口

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

### 7.2 PlaneConstraintBuilder 全方式

1. `Parallel(ref, reversed)`
2. `Perpendicular(ref, reversed)`
3. `Coincident(ref, reversed)`
4. `Distance(ref, value, defaultDir, reversed)`
5. `Angle(ref, value, defaultDir, reversed)`
6. `Symmetric(ref, reversed)`
7. `Tangent(ref, reversed)`
8. `Projection(ref, reversed)`

### 7.3 典型示例

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

### 7.4 DatumPlaneAccessor 读取参考

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

## 8. UG 提取参数 -> Builder 映射参考

来源：`UgLibs/doc/ug_datum_plane_read_create_research_20260318.md`

### 8.1 UG fixed_dplane

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

### 8.2 UG relative_dplane

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

## 9. Creo 提取参数 -> Builder 映射参考

来源：`CreoLibs/doc/creo_datum_plane_read_create_research_20260318.md`

### 9.1 PRO_DTMPLN_OFFS

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

### 9.2 PRO_DTMPLN_ANG

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

### 9.3 PRO_DTMPLN_PRL / MIDPLN / TANG（建议映射）

1. `PRO_DTMPLN_PRL` -> `PlaneMethod::PARALLEL` + `PlaneConstraintType::PARALLEL`
2. `PRO_DTMPLN_MIDPLN` -> `PlaneMethod::MID_PLANE` + 双引用 + `SYMMETRIC`
3. `PRO_DTMPLN_TANG` -> `PlaneMethod::TANGENT` + `PlaneConstraintType::TANGENT`

---

## 10. 常见调用建议

1. 草图先 `Build` 再拉伸，优先用 `SetProfile(sketchID)`。
2. 对 `Extrude / Revolve`，优先把终止范围理解为 `SweepExtent`，而不是旧的 “depth / angle” 专有结构。
3. 对 `Sweep`，优先把主路径和引导线都理解为 `CRefEntityBase` 引用链，而不是草图专用 ID；草图路径和实体边路径应使用同一套 `Ref::*` 入口。
3. 终止面/点/顶点尽量使用 `Ref::*` 包装，避免直接操作底层引用结构。
4. 基准面约束里 `ref` 是“referenceEntities 下标”，建议用 `AddReferenceAndGetIndex(...)` 获取，避免硬编码。
5. Accessor 回读时优先检查 `IsValid()/Has*()`，复杂字段优先看 `Data()->extent1/extent2`。
6. 除了草图 localID 这种必须落局部变量的场景，其余 builder 示例尽量保持单条链式调用到 `.Build()`。

## 11. 最小验收建议

1. Builder 创建后调用 `UnifiedModel::Validate()`。
2. 通过 Accessor 回读关键字段：
   1. Sketch: segment/constraint 数量和类型。
   2. Extrude: extent 类型、value/offset、引用。
   3. Revolve: axis、extent 类型、value、operation。
   4. Sweep: profileSketchID、path/guide reference count、reference type/localID、orientation、thinWall。
   5. DatumPlane: method、referenceCount、constraint(type/ref/value/reversed)。
3. 对桥接模块执行你现有的 round-trip 五步校验链路。
