#pragma once

// Generate a basic getter.
// Usage: ACCESSOR_GETTER(double, Depth, depth, 0.0)
// Expands to:
//   double GetDepth() const { return IsValid() ? Data()->depth : DefaultValue; }
// The owner class must implement Data().
#define ACCESSOR_GETTER(ReturnType, FuncName, MemberName, DefaultValue)        \
  ReturnType Get##FuncName() const {                                           \
    return IsValid() ? Data()->MemberName : DefaultValue;                      \
  }

// Generate a getter for shared_ptr-like members.
// Usage: ACCESSOR_REF_GETTER(SketchAccessor, Profile, sketchProfile)
#define ACCESSOR_REF_GETTER(WrapperType, FuncName, MemberName)                 \
  WrapperType Get##FuncName() const {                                          \
    if (!IsValid() || !Data()->MemberName)                                     \
      return WrapperType(nullptr);                                             \
    return WrapperType(Data()->MemberName);                                    \
  }

// Generate a getter for std::optional fields.
// Usage: ACCESSOR_OPTIONAL_GETTER(double, DraftAngle, draft, angle, 0.0)
// Assumes std::optional<Struct> MemberName and Struct::FieldName.
#define ACCESSOR_OPTIONAL_GETTER(ReturnType, FuncName, MemberName, FieldName,  \
                                 DefaultValue)                                 \
  ReturnType Get##FuncName() const {                                           \
    if (!IsValid() || !Data()->MemberName.has_value())                         \
      return DefaultValue;                                                     \
    return Data()->MemberName->FieldName;                                      \
  }

// Generate a HasXxx() helper for std::optional fields.
// Usage: ACCESSOR_HAS_GETTER(Draft, draft)
#define ACCESSOR_HAS_GETTER(FuncName, MemberName)                              \
  bool Has##FuncName() const {                                                 \
    return IsValid() && Data()->MemberName.has_value();                        \
  }

// Generate an IsXxx() helper for bool fields.
// Usage: ACCESSOR_IS_GETTER(Cut, isCut)
#define ACCESSOR_IS_GETTER(FuncName, MemberName)                               \
  bool Is##FuncName() const { return IsValid() && Data()->MemberName; }
