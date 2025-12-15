 `Builder` 层实现，尤其是 `BuilderMacros.h`, `ReferenceBuilder.h` 和各个 `FeatureBuilder` 文件，你的设计已经相当成熟：强类型、流式接口、辅助宏减少样板代码。

为了保持 **对称性 (Symmetry)** 和 **易用性 (Usability)**，`Accessor` 层应该提供与 Builder 相反但风格一致的接口。

以下是针对 Accessor 层的详细设计方案，重点在于如何“拆包”那些通过 Builder 精心封装的复杂结构。

---

### 设计目标

1.  **只读与安全**：Accessor 只提供 `Get` 接口，持有 `std::shared_ptr<const T>`。
2.  **扁平化复杂性**：比如 `CRefFace`，用户不需要知道它继承自 `CRefSubTopo`，只需要 `GetNormal()`。
3.  **对齐 Builder**：Builder 有 `SetProfile(string id)`，Accessor 就应该有 `GetProfileID()`。
4.  **智能空值处理**：对于 `std::optional` (如 `draft`, `thinWall`)，Accessor 应该提供 `HasXXX()` + `GetXXX()` 的模式。

---

### 1. 基础访问器 (FeatureAccessorBase.h)

对应 `FeatureBuilderBase.h`，这是所有 Accessor 的基类。

```cpp
#pragma once
#include "../../core/UnifiedModel.h"
#include "../../core/UnifiedFeatures.h"
#include <memory>
#include <string>

namespace CADExchange {
namespace Accessor {

// 简单的类型枚举，避免暴露 typeid
enum class AccessorType {
    Unknown, Sketch, Extrude, Revolve
};

class FeatureAccessorBase {
protected:
    std::shared_ptr<const CFeatureBase> m_feature;

public:
    explicit FeatureAccessorBase(std::shared_ptr<const CFeatureBase> feat) 
        : m_feature(std::move(feat)) {}
    
    virtual ~FeatureAccessorBase() = default;

    bool IsValid() const { return m_feature != nullptr; }

    // 通用属性
    std::string GetID() const { return IsValid() ? m_feature->featureID : ""; }
    std::string GetName() const { return IsValid() ? m_feature->featureName : ""; }
    std::string GetExternalID() const { return IsValid() ? m_feature->externalID : ""; }
    bool IsSuppressed() const { return IsValid() ? m_feature->isSuppressed : false; }

    // 类型查询
    AccessorType GetType() const {
        if (!IsValid()) return AccessorType::Unknown;
        if (std::dynamic_pointer_cast<const CSketch>(m_feature)) return AccessorType::Sketch;
        if (std::dynamic_pointer_cast<const CExtrude>(m_feature)) return AccessorType::Extrude;
        if (std::dynamic_pointer_cast<const CRevolve>(m_feature)) return AccessorType::Revolve;
        return AccessorType::Unknown;
    }
};

} // namespace Accessor
} // namespace CADExchange
```

---

### 2. 引用访问器 (ReferenceAccessor.h)

对应 `ReferenceBuilder.h`。
这里最关键的是：**如何让用户方便地读取多态的 `CRefEntityBase`？**
建议使用 **Visitor 模式的简化版** 或者 **类型查询 + Getter**。

```cpp
#pragma once
#include "../../core/UnifiedFeatures.h"
#include "../../core/UnifiedTypes.h"
#include <memory>

namespace CADExchange {
namespace Accessor {

class ReferenceAccessor {
    std::shared_ptr<const CRefEntityBase> m_ref;

public:
    explicit ReferenceAccessor(std::shared_ptr<const CRefEntityBase> ref) 
        : m_ref(std::move(ref)) {}

    bool IsValid() const { return m_ref != nullptr; }

    // 直接暴露 RefType 枚举，因为它在 UnifiedFeatures.h 里已经是 public 的
    RefType GetRefType() const { 
        return IsValid() ? m_ref->refType : RefType::FEATURE_DATUM_PLANE; // 默认值需谨慎
    }

    // --- 通用属性 (尝试转换) ---
    
    // 获取父特征 ID (适用于 SubTopo)
    std::string GetParentID() const {
        if (auto topo = std::dynamic_pointer_cast<const CRefSubTopo>(m_ref)) {
            return topo->parentFeatureID;
        }
        return "";
    }

    // 获取目标特征 ID (适用于 Feature Reference)
    std::string GetTargetID() const {
        if (auto feat = std::dynamic_pointer_cast<const CRefFeature>(m_ref)) {
            return feat->targetFeatureID;
        }
        return "";
    }

    int GetTopologyIndex() const {
        if (auto topo = std::dynamic_pointer_cast<const CRefSubTopo>(m_ref)) {
            return topo->topologyIndex;
        }
        return -1;
    }

    // --- 几何指纹获取 (扁平化) ---

    // 尝试获取面法向
    bool GetFaceNormal(CVector3D& outNormal) const {
        if (auto face = std::dynamic_pointer_cast<const CRefFace>(m_ref)) {
            outNormal = face->normal;
            return true;
        }
        return false;
    }

    // 尝试获取面质心
    bool GetFaceCentroid(CPoint3D& outCentroid) const {
        if (auto face = std::dynamic_pointer_cast<const CRefFace>(m_ref)) {
            outCentroid = face->centroid;
            return true;
        }
        return false;
    }

    // 尝试获取顶点位置
    bool GetVertexPos(CPoint3D& outPos) const {
        if (auto v = std::dynamic_pointer_cast<const CRefVertex>(m_ref)) {
            outPos = v->pos;
            return true;
        }
        return false;
    }

    // 尝试获取草图段 ID
    std::string GetSketchSegLocalID() const {
        if (auto seg = std::dynamic_pointer_cast<const CRefSketchSeg>(m_ref)) {
            return seg->segmentLocalID;
        }
        return "";
    }
};

}
}
```

---

### 3. 拉伸访问器 (ExtrudeAccessor.h)

对应 `ExtrudeBuilder.h`。重点是解包 `EndCondition` 和 `Optional`。

```cpp
#pragma once
#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"

namespace CADExchange {
namespace Accessor {

class ExtrudeAccessor : public FeatureAccessorBase {
    std::shared_ptr<const CExtrude> m_extrude;

public:
    explicit ExtrudeAccessor(std::shared_ptr<const CFeatureBase> feat)
        : FeatureAccessorBase(feat) {
        m_extrude = std::dynamic_pointer_cast<const CExtrude>(feat);
    }

    bool IsValid() const { return m_extrude != nullptr; }

    // --- 基础属性 ---
    std::string GetProfileSketchID() const { 
        // 假设 profile 是 CSketch 指针，我们需要它的 ID
        if (IsValid() && m_extrude->sketchProfile) {
            return m_extrude->sketchProfile->featureID;
        }
        return "";
    }

    CVector3D GetDirection() const {
        return IsValid() ? m_extrude->direction : CVector3D{0,0,1};
    }

    BooleanOp GetOperation() const {
        return IsValid() ? m_extrude->operation : BooleanOp::BOSS;
    }

    // --- 结束条件 1 ---
    ExtrudeEndCondition::Type GetEndType1() const {
        return IsValid() ? m_extrude->endCondition1.type : ExtrudeEndCondition::Type::BLIND;
    }

    double GetDepth1() const {
        return IsValid() ? m_extrude->endCondition1.depth : 0.0;
    }

    // 获取引用对象 (返回 ReferenceAccessor)
    ReferenceAccessor GetReference1() const {
        if (IsValid()) {
            return ReferenceAccessor(m_extrude->endCondition1.referenceEntity);
        }
        return ReferenceAccessor(nullptr);
    }

    double GetOffset1() const {
        return IsValid() ? m_extrude->endCondition1.offset : 0.0;
    }

    // --- 结束条件 2 (Optional) ---
    bool HasDirection2() const {
        return IsValid() && m_extrude->endCondition2.has_value();
    }

    double GetDepth2() const {
        if (HasDirection2()) return m_extrude->endCondition2->depth;
        return 0.0;
    }
    
    // --- 拔模 (Optional) ---
    bool HasDraft() const {
        return IsValid() && m_extrude->draft.has_value();
    }

    double GetDraftAngle() const {
        return HasDraft() ? m_extrude->draft->angle : 0.0;
    }

    bool IsDraftOutward() const {
        return HasDraft() ? m_extrude->draft->outward : false;
    }

    // --- 薄壁 (Optional) ---
    bool HasThinWall() const {
        return IsValid() && m_extrude->thinWall.has_value();
    }

    double GetThinThickness() const {
        return HasThinWall() ? m_extrude->thinWall->thickness : 0.0;
    }
};

}
}
```

---

### 4. 草图访问器 (SketchAccessor.h)

对应 `SketchBuilder.h`。这里比较复杂，因为需要遍历几何段。建议提供 **Visitor 模式** 或者 **Index-based Getters**。

```cpp
#pragma once
#include "FeatureAccessorBase.h"
#include "ReferenceAccessor.h"

namespace CADExchange {
namespace Accessor {

// 草图几何段访问器 (辅助类)
class SketchSegmentAccessor {
    std::shared_ptr<const CSketchSeg> m_seg;
public:
    explicit SketchSegmentAccessor(std::shared_ptr<const CSketchSeg> seg) : m_seg(seg) {}
    
    bool IsValid() const { return m_seg != nullptr; }
    
    CSketchSeg::SegType GetType() const { 
        return IsValid() ? m_seg->type : CSketchSeg::SegType::LINE; // Default?
    }
    
    std::string GetLocalID() const { return IsValid() ? m_seg->localID : ""; }
    bool IsConstruction() const { return IsValid() ? m_seg->isConstruction : false; }

    // Line 特有
    bool GetLineCoords(CPoint3D& start, CPoint3D& end) const {
        if (auto l = std::dynamic_pointer_cast<const CSketchLine>(m_seg)) {
            start = l->startPos; end = l->endPos; return true;
        }
        return false;
    }

    // Circle 特有
    bool GetCircleParams(CPoint3D& center, double& radius) const {
        if (auto c = std::dynamic_pointer_cast<const CSketchCircle>(m_seg)) {
            center = c->center; radius = c->radius; return true;
        }
        return false;
    }
    
    // ... Arc, Point ...
};

class SketchAccessor : public FeatureAccessorBase {
    std::shared_ptr<const CSketch> m_sketch;

public:
    explicit SketchAccessor(std::shared_ptr<const CFeatureBase> feat)
        : FeatureAccessorBase(feat) {
        m_sketch = std::dynamic_pointer_cast<const CSketch>(feat);
    }

    bool IsValid() const { return m_sketch != nullptr; }

    // 获取参考面
    ReferenceAccessor GetReferencePlane() const {
        if (IsValid()) return ReferenceAccessor(m_sketch->referencePlane);
        return ReferenceAccessor(nullptr);
    }

    // 遍历几何段
    int GetSegmentCount() const {
        return IsValid() ? m_sketch->segments.size() : 0;
    }

    SketchSegmentAccessor GetSegment(int index) const {
        if (IsValid() && index >= 0 && index < m_sketch->segments.size()) {
            return SketchSegmentAccessor(m_sketch->segments[index]);
        }
        return SketchSegmentAccessor(nullptr);
    }

    // 遍历约束 (略)
};

}
}
```

---

### 5. 模型访问器 (ModelAccessor.h)

这是入口。

```cpp
#pragma once
#include "../../core/UnifiedModel.h"
// #include "SerializerService.h" // 假设有
#include "FeatureAccessorBase.h"

namespace CADExchange {
namespace Accessor {

class ModelAccessor {
    UnifiedModel m_model; // 持有数据副本或引用

public:
    // 如果是 Load 模式，它持有数据
    // 如果是 Inspect 模式 (比如 Builder 刚建完)，它可能只是引用了一个外部 Model
    explicit ModelAccessor() = default;

    // Load 接口 (实际应该调用 Serializer)
    bool Load(const std::string& path) {
        // ... SerializerService::Load(path, m_model);
        return true; 
    }

    int GetFeatureCount() const { return m_model.GetFeatures().size(); }

    std::shared_ptr<FeatureAccessorBase> GetFeature(int index) {
        auto feats = m_model.GetFeatures();
        if (index < 0 || index >= feats.size()) return nullptr;

        return std::make_shared<FeatureAccessorBase>(feats[index]);
    }
    
    // 提供强类型的转换辅助
    std::shared_ptr<ExtrudeAccessor> GetAsExtrude(int index) {
        auto feat = GetFeature(index); // 这里需要微调，FeatureAccessorBase不能直接转
        // 实际上应该直接从 model 取出来构造 ExtrudeAccessor
        auto raw = m_model.GetFeatures()[index];
        if (std::dynamic_pointer_cast<const CExtrude>(raw)) {
            return std::make_shared<ExtrudeAccessor>(raw);
        }
        return nullptr;
    }
};

}
}
```

### 总结

Accessor 的设计哲学就是 **“安全地拆包”**：

1.  **ReferenceAccessor**: 像剥洋葱一样，先看 `RefType`，再尝试 `dynamic_cast` 并提取 `Normal`, `Centroid` 等指纹信息。
2.  **ExtrudeAccessor**: 像吃自助餐一样，想要 `Depth` 就拿，想要 `Draft` 就先问 `HasDraft`。
3.  **SketchAccessor**: 像翻书一样，通过 `Index` 获取每一页（Segment），每页再看是直线还是圆。

这套设计与你的 Builder 层完美对称，确保了“写进去的数据”都能“方便地读出来”。