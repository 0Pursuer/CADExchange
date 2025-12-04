#pragma once
#include <string>
#include <windows.h> // 使用 Windows API 进行转换，稳定且支持中文

class StringHelper {
public:
  // 1. 宽字符串 (CAD) -> UTF-8 (内部结构/XML)
  static std::string ToUtf8(const std::wstring &wstr) {
    if (wstr.empty())
      return std::string();

    // 计算需要的缓冲区长度
    int size_needed = WideCharToMultiByte(
        CP_UTF8, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);

    std::string strTo(size_needed, 0);
    WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), &strTo[0],
                        size_needed, NULL, NULL);

    return strTo;
  }

  // 重载：支持 wchar_t*
  static std::string ToUtf8(const wchar_t *wstr) {
    if (!wstr)
      return std::string();
    return ToUtf8(std::wstring(wstr));
  }

  static std::string ToUtf8(wchar_t *wstr) {
    if (!wstr)
      return std::string();
    return ToUtf8(static_cast<const wchar_t *>(wstr));
  }

  // 2. UTF-8 (内部结构/XML) -> 宽字符串 (CAD)
  static std::wstring ToWide(const std::string &str) {
    if (str.empty())
      return std::wstring();

    int size_needed =
        MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), NULL, 0);

    std::wstring wstrTo(size_needed, 0);
    MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), &wstrTo[0],
                        size_needed);

    return wstrTo;
  }
};
