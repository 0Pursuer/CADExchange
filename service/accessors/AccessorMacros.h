#pragma once

// 生成基础类型的 Getter
// 使用: ACCESSOR_GETTER(double, Depth, depth, 0.0)
// 生成: double GetDepth() const { return IsValid() ? Data()->depth :
// DefaultValue; } 注意：这里使用 Data() 而不是 m_data，要求使用宏的类必须实现
// Data() 方法
#define ACCESSOR_GETTER(ReturnType, FuncName, MemberName, DefaultValue)        \
  ReturnType Get##FuncName() const {                                           \
    return IsValid() ? Data()->MemberName : DefaultValue;                      \
  }

// 生成对象引用的 Getter (处理 shared_ptr)
// 使用: ACCESSOR_REF_GETTER(SketchAccessor, Profile, sketchProfile)
// 要求 MemberName 是一个 shared_ptr
#define ACCESSOR_REF_GETTER(WrapperType, FuncName, MemberName)                 \
  WrapperType Get##FuncName() const {                                          \
    if (!IsValid() || !Data()->MemberName)                                     \
      return WrapperType(nullptr);                                             \
    return WrapperType(Data()->MemberName);                                    \
  }

// 生成 std::optional 类型字段的 Getter
// 使用: ACCESSOR_OPTIONAL_GETTER(double, DraftAngle, draft, angle, 0.0)
// 假设结构体中有 std::optional<Struct> MemberName; Struct 中有 FieldName 字段
#define ACCESSOR_OPTIONAL_GETTER(ReturnType, FuncName, MemberName, FieldName,  \
                                 DefaultValue)                                 \
  ReturnType Get##FuncName() const {                                           \
    if (!IsValid() || !Data()->MemberName.has_value())                         \
      return DefaultValue;                                                     \
    return Data()->MemberName->FieldName;                                      \
  }

// 生成判断 std::optional 是否存在的 Has 方法
// 使用: ACCESSOR_HAS_GETTER(Draft, draft)
#define ACCESSOR_HAS_GETTER(FuncName, MemberName)                              \
  bool Has##FuncName() const {                                                 \
    return IsValid() && Data()->MemberName.has_value();                        \
  }

// 生成 bool 类型的 Is 方法 (针对 bool 字段)
// 使用: ACCESSOR_IS_GETTER(Cut, isCut)
#define ACCESSOR_IS_GETTER(FuncName, MemberName)                               \
  bool Is##FuncName() const { return IsValid() && Data()->MemberName; }
