#pragma once
// cadex_logger.h  —  Lightweight printf-style logger for CADExchange CAD bridges.
//
// Features:
//   - Millisecond-resolution timestamp (no windows.h required)
//   - Log level filter (DEBUG / INFO / WARN / ERROR)
//   - Short filename + line number via __FILE__ / __LINE__
//   - Console output (stdout for DEBUG/INFO/WARN, stderr for ERROR)
//   - Optional append-to-file via cadex::Logger::Get().SetFile("bridge.log")
//   - Thread-safe singleton
//
// Usage (printf-style format strings):
//   LOG_DEBUG("entering loop i=%d", i);
//   LOG_INFO("Sketch created id=%s segs=%d", id.c_str(), count);
//   LOG_WARN("fallback path used for feature id=%s", id.c_str());
//   LOG_ERROR("UF_PART_new failed code=%d", code);
//   LOG_WARNING(...)  // backward-compat alias for LOG_WARN
//
// Wide-string helper (ASCII-safe lossy conversion for log messages):
//   LOG_INFO("path=%s", cadex::WN(widePath).c_str());

#ifndef CADEX_LOGGER_H
#define CADEX_LOGGER_H

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <mutex>
#include <string>

namespace cadex {

enum class LogLevel : int { Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4 };

class Logger {
public:
    static Logger& Get() noexcept {
        static Logger s_instance;
        return s_instance;
    }

    void SetLevel(LogLevel level) noexcept { m_level = level; }

    void SetFile(const char* path) {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_file.open(path, std::ios::app);
    }

    // Called by LOG_* macros — do not invoke directly.
    void DoLog(LogLevel level, const char* file, int line,
               const char* fmt, ...) noexcept {
        if (level < m_level) return;

        char msgbuf[2048];
        {
            va_list args;
            va_start(args, fmt);
            std::vsnprintf(msgbuf, sizeof(msgbuf), fmt, args);
            va_end(args);
        }

        // Keep only the filename part of __FILE__.
        const char* f = file;
        for (const char* p = file; *p; ++p) {
            if (*p == '/' || *p == '\\') f = p + 1;
        }

        // Millisecond timestamp using std::chrono (no windows.h needed).
        using clock = std::chrono::system_clock;
        const auto now = clock::now();
        const auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now.time_since_epoch()) % 1000;
        const std::time_t t = clock::to_time_t(now);
        struct tm ti{};
#ifdef _WIN32
        localtime_s(&ti, &t);
#else
        localtime_r(&t, &ti);
#endif
        char linebuf[2560];
        std::snprintf(linebuf, sizeof(linebuf),
                      "[%02d:%02d:%02d.%03d] [%s] [%s:%d] %s\n",
                      ti.tm_hour, ti.tm_min, ti.tm_sec,
                      static_cast<int>(ms.count()),
                      LevelStr(level), f, line, msgbuf);

        std::lock_guard<std::mutex> lk(m_mutex);
        std::fputs(linebuf, level >= LogLevel::Error ? stderr : stdout);
        if (m_file.is_open()) {
            m_file << linebuf;
            m_file.flush();
        }
    }

private:
    Logger()  = default;
    ~Logger() = default;
    Logger(const Logger&)            = delete;
    Logger& operator=(const Logger&) = delete;

    static const char* LevelStr(LogLevel l) noexcept {
        switch (l) {
            case LogLevel::Debug: return "DEBUG";
            case LogLevel::Info:  return "INFO ";
            case LogLevel::Warn:  return "WARN ";
            case LogLevel::Error: return "ERROR";
            default:              return "?????";
        }
    }

    LogLevel      m_level{LogLevel::Info};
    std::mutex    m_mutex;
    std::ofstream m_file;
};

// ASCII-safe lossy wide→narrow helper for logging wide strings and paths.
inline std::string WN(const std::wstring& ws) {
    std::string out;
    out.reserve(ws.size());
    for (wchar_t c : ws) {
        out.push_back(c < 128 ? static_cast<char>(c) : '?');
    }
    return out;
}

} // namespace cadex

// ---------------------------------------------------------------------------
// Printf-style logging macros — capture file/line automatically.
// ---------------------------------------------------------------------------
#define LOG_DEBUG(fmt, ...) \
    cadex::Logger::Get().DoLog(cadex::LogLevel::Debug, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) \
    cadex::Logger::Get().DoLog(cadex::LogLevel::Info,  __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) \
    cadex::Logger::Get().DoLog(cadex::LogLevel::Warn,  __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) \
    cadex::Logger::Get().DoLog(cadex::LogLevel::Warn,  __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) \
    cadex::Logger::Get().DoLog(cadex::LogLevel::Error, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#endif // CADEX_LOGGER_H
