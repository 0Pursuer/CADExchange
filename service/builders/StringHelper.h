#pragma once
// clang-format off
#include <atomic>
#include <memory>
#include <string>
#include <windows.h> // 使用 Windows API 进行转换，稳定且支持中文
// clang-format on

namespace CADExchange {

class StringHelper {
public:
  /**
   * @brief 生成简单的递增 UUID，用于测试和占位符。
   *
   * @return 形如 "FB-1", "FB-2", ... 的字符串。
   */
  static std::string GenerateUUID() {
    static std::atomic<int> counter{0};
    return "FB-" +
           std::to_string(counter.fetch_add(1, std::memory_order_relaxed) + 1);
  }

  /**
   * @brief 将宽字符串转换为 UTF-8 编码的字符串。
   *
   * @param wstr 宽字符串输入。
   * @return 转换后的 UTF-8 字符串。
   */
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

  /**
   * @brief 将宽字符串指针转换为 UTF-8 编码的字符串。
  */
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
  
  /**
   * @brief 将 UTF-8 编码的字符串转换为宽字符串。
   *
   * @param str UTF-8 字符串输入。
   * @return 转换后的宽字符串。
   */
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
} // namespace CADExchange