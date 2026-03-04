#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <utility>

namespace CADExchange::BridgeCommon {

template <typename F>
class ScopeExit {
public:
  explicit ScopeExit(F fn) : fn_(std::move(fn)) {}
  ScopeExit(const ScopeExit &) = delete;
  ScopeExit &operator=(const ScopeExit &) = delete;

  ScopeExit(ScopeExit &&other) noexcept
      : fn_(std::move(other.fn_)), active_(other.active_) {
    other.active_ = false;
  }

  ~ScopeExit() {
    if (active_) {
      fn_();
    }
  }

  void Dismiss() noexcept { active_ = false; }

private:
  F fn_;
  bool active_ = true;
};

template <typename F>
ScopeExit<F> MakeScopeExit(F fn) {
  return ScopeExit<F>(std::move(fn));
}

inline std::wstring ReplaceExtension(const std::wstring &path,
                                     const std::wstring &newExt) {
  std::filesystem::path p(path);
  p.replace_extension(newExt);
  return p.wstring();
}

inline std::wstring JsonEscape(const std::wstring &value) {
  std::wstring out;
  out.reserve(value.size() + 16);
  auto appendHex4 = [&out](wchar_t ch) {
    static const wchar_t *kHex = L"0123456789ABCDEF";
    out += L"\\u";
    out.push_back(kHex[(ch >> 12) & 0xF]);
    out.push_back(kHex[(ch >> 8) & 0xF]);
    out.push_back(kHex[(ch >> 4) & 0xF]);
    out.push_back(kHex[ch & 0xF]);
  };

  for (wchar_t ch : value) {
    switch (ch) {
    case L'\\':
      out += L"\\\\";
      break;
    case L'"':
      out += L"\\\"";
      break;
    case L'\n':
      out += L"\\n";
      break;
    case L'\r':
      out += L"\\r";
      break;
    case L'\t':
      out += L"\\t";
      break;
    default:
      if (ch < 0x20 || ch > 0x7E) {
        appendHex4(ch);
      } else {
        out.push_back(ch);
      }
      break;
    }
  }
  return out;
}

inline bool TryGetJsonStringValue(const std::wstring &json, std::wstring_view key,
                                  std::wstring &value) {
  const std::wstring keyPattern = L"\"" + std::wstring(key) + L"\"";
  size_t keyPos = json.find(keyPattern);
  if (keyPos == std::wstring::npos) {
    return false;
  }
  size_t colonPos = json.find(L':', keyPos + keyPattern.size());
  if (colonPos == std::wstring::npos) {
    return false;
  }

  size_t firstQuote = json.find(L'"', colonPos + 1);
  if (firstQuote == std::wstring::npos) {
    return false;
  }

  std::wstring out;
  out.reserve(64);
  bool escaping = false;
  for (size_t i = firstQuote + 1; i < json.size(); ++i) {
    const wchar_t ch = json[i];
    if (escaping) {
      switch (ch) {
      case L'"':
        out.push_back(L'"');
        break;
      case L'\\':
        out.push_back(L'\\');
        break;
      case L'n':
        out.push_back(L'\n');
        break;
      case L'r':
        out.push_back(L'\r');
        break;
      case L't':
        out.push_back(L'\t');
        break;
      default:
        out.push_back(ch);
        break;
      }
      escaping = false;
      continue;
    }

    if (ch == L'\\') {
      escaping = true;
      continue;
    }
    if (ch == L'"') {
      value = std::move(out);
      return true;
    }
    out.push_back(ch);
  }
  return false;
}

} // namespace CADExchange::BridgeCommon

