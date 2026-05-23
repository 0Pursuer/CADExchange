#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <iostream>

namespace cadex {

struct ProfileData {
    std::string name;
    double totalDurationMs{0.0};
    size_t callCount{0};
};

class Profiler {
public:
    static Profiler& Get() noexcept {
        static Profiler s_instance;
        return s_instance;
    }

    void Start(const std::string& name) {
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(m_mutex);
        m_activeTimers[name] = now;
    }

    void Stop(const std::string& name) {
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_activeTimers.find(name);
        if (it != m_activeTimers.end()) {
            double duration = std::chrono::duration<double, std::milli>(now - it->second).count();
            auto& data = m_profileData[name];
            data.name = name;
            data.totalDurationMs += duration;
            data.callCount++;
            m_activeTimers.erase(it);
        }
    }

    std::wstring GetReport() {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::vector<ProfileData> sortedData;
        sortedData.reserve(m_profileData.size());
        for (const auto& pair : m_profileData) {
            sortedData.push_back(pair.second);
        }

        // Sort by total duration descending
        std::sort(sortedData.begin(), sortedData.end(), [](const ProfileData& a, const ProfileData& b) {
            return a.totalDurationMs > b.totalDurationMs;
        });

        std::wstringstream ss;
        ss << L"\n====================== PERFORMANCE REPORT ======================\n";
        ss << std::left << std::setw(35) << L"Scope Name"
           << std::right << std::setw(15) << L"Total Time (ms)"
           << std::setw(12) << L"Calls"
           << std::setw(15) << L"Avg Time (ms)" << L"\n";
        ss << L"----------------------------------------------------------------\n";

        for (const auto& data : sortedData) {
            double avg = data.callCount > 0 ? (data.totalDurationMs / data.callCount) : 0.0;
            
            // Convert narrow string name to wide string for output
            std::wstring wname(data.name.begin(), data.name.end());
            
            ss << std::left << std::setw(35) << wname
               << std::right << std::setiosflags(std::ios::fixed) << std::setprecision(2)
               << std::setw(15) << data.totalDurationMs
               << std::setw(12) << data.callCount
               << std::setw(15) << avg << L"\n";
        }
        ss << L"================================================================\n";
        return ss.str();
    }

    void PrintReport() {
        std::wcout << GetReport();
    }

    void Reset() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_activeTimers.clear();
        m_profileData.clear();
    }

private:
    Profiler() = default;
    ~Profiler() = default;
    Profiler(const Profiler&) = delete;
    Profiler& operator=(const Profiler&) = delete;

    std::mutex m_mutex;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> m_activeTimers;
    std::unordered_map<std::string, ProfileData> m_profileData;
};

class ProfileScope {
public:
    explicit ProfileScope(std::string name) : m_name(std::move(name)) {
        Profiler::Get().Start(m_name);
    }
    ~ProfileScope() {
        Profiler::Get().Stop(m_name);
    }
private:
    std::string m_name;
};

} // namespace cadex

#define PROFILE_SCOPE(name) ::cadex::ProfileScope profileScope##__LINE__(name)
#define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)
