# CADExchange 代码改进清单：Accessor 简化方案

## 1. 问题现状 (Current Issue)

当前 `CADExchange/service/accessors` 目录下的 Accessor 类（如 `ExtrudeAccessor`）存在大量重复的样板代码。
几乎每一个 `UnifiedFeatures` 中的结构体字段，都需要在 Accessor 中手写一个对应的 `GetXxx()` 函数。

**痛点：**
1.  **代码冗余**：`UnifiedFeatures.h` 定义了数据，Accessor 又定义了一遍读取接口。
2.  **维护成本**：每当给特征增加一个属性（例如给拉伸增加“拔模角度”），需要同时修改 Struct 和 Accessor。
3.  **低价值工作**：大部分 Accessor 逻辑仅仅是 `return IsValid() ? m_ptr->field : default;`。

---

## 2. 改进方案 A：直接暴露只读数据 (Direct Const Access) - **强烈推荐**

这是最激进但也最有效的简化方式。既然 `UnifiedFeatures` 中的结构体（如 `CExtrude`）本身就是纯数据容器（DTO），且 Accessor 的目的是“只读访问”，那么直接暴露 `const` 指针或引用是最简单的方法。

### 代码对比

**修改前 (Current):**
```cpp
// ExtrudeAccessor.h
class ExtrudeAccessor {
    // ...
    double GetDepth() const { return IsValid() ? m_extrude->depth : 0.0; }
    bool IsCut() const { return IsValid() ? m_extrude->isCut : false; }
    // 还要写几十个这样的函数...
};
```

**修改后 (Proposed):**
```cpp
// ExtrudeAccessor.h
class ExtrudeAccessor : public FeatureAccessorBase {
public:
    // ... 构造函数保持不变 ...

    /**
     * @brief 直接获取底层只读数据指针。
     * 调用方需配合 IsValid() 使用，或者自行判空。
     */
    const CExtrude* Data() const {
        return m_extrude.get(); 
    }
    
    // 仅保留计算逻辑复杂的 Accessor，简单的字段直接通过 Data() 访问
    // 例如: accessor.Data()->depth;
};
```

### 实施步骤
1.  在 `FeatureAccessorBase` 或各个具体 Accessor 中添加 `Data()` 方法，返回 `const T*`。
2.  将现有的 `GetXxx()` 标记为 `[[deprecated]]` 或直接移除。
3.  更新调用方代码，从 `acc.GetDepth()` 改为 `if(acc.IsValid()) acc.Data()->depth`。

### 优缺点评估
*   **优点**：代码量减少 90% 以上；新增字段无需修改 Accessor；编译速度加快。
*   **缺点**：破坏了部分封装性（但在内部库中，DTO 的封装性通常不是首要考虑）。

---

## 3. 改进方案 B：宏定义生成 (Macro Generation)

如果项目组规范要求必须保持 `GetXxx()` 形式的 API 风格，或者为了兼容旧代码，可以使用宏来自动生成 Getter。这类似于 `BuilderMacros.h` 的做法。

### 新增文件 `service/accessors/AccessorMacros.h`

```cpp
#pragma once

// 生成基础类型的 Getter
// 使用: ACCESSOR_GETTER(double, Depth, depth, 0.0)
// 生成: double GetDepth() const { return IsValid() ? m_data->depth : 0.0; }
#define ACCESSOR_GETTER(ReturnType, FuncName, MemberName, DefaultValue) \
    ReturnType Get##FuncName() const { \
        return IsValid() ? m_data->MemberName : DefaultValue; \
    }

// 生成对象引用的 Getter (处理 shared_ptr)
#define ACCESSOR_REF_GETTER(WrapperType, FuncName, MemberName) \
    WrapperType Get##FuncName() const { \
        if (!IsValid() || !m_data->MemberName) return WrapperType(nullptr); \
        return WrapperType(m_data->MemberName); \
    }
```

### 在 Accessor 中使用

```cpp
class ExtrudeAccessor : public FeatureAccessorBase {
    std::shared_ptr<const CExtrude> m_data;
public:
    // ...
    
    // 一行代码定义一个属性，清晰明了
    ACCESSOR_GETTER(double, Depth, depth, 0.0)
    ACCESSOR_GETTER(bool, IsCut, isCut, false)
    ACCESSOR_REF_GETTER(SketchAccessor, Profile, sketchProfile)
};
```

### 优缺点评估
*   **优点**：保持了 API 的一致性；代码整洁；减少了手写错误。
*   **缺点**：仍然需要为每个字段写一行宏调用；宏调试相对困难。

---

## 4. 综合建议

建议采用 **混合策略**：

1.  **主要策略 (方案 A)**：为所有 Accessor 添加 `const T* Data() const` 接口。
2.  **过渡策略**：对于现有的高频使用的 Getter，可以暂时保留或用宏重写。
3.  **新代码规范**：对于新增加的 Feature，不再编写繁琐的 Getter，直接通过 `Data()` 指针访问字段。

### 示例：重构后的 ExtrudeAccessor

```cpp
class ExtrudeAccessor : public FeatureAccessorBase {
    std::shared_ptr<const CExtrude> m_extrude;
public:
    // ... 构造函数 ...

    bool IsValid() const { return m_extrude != nullptr; }

    // 1. 提供直接访问入口 (新推荐用法)
    const CExtrude* operator->() const { return m_extrude.get(); }
    const CExtrude* Get() const { return m_extrude.get(); }

    // 2. 仅保留计算型或复杂逻辑的访问器 (如单位转换、坐标变换)
    CVector3D GetWorldDirection() const {
        // 假设这里有复杂的计算逻辑
        return IsValid() ? Transform(m_extrude->direction) : CVector3D::Zero();
    }
};
```
