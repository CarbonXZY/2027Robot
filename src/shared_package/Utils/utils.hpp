/**
 * @file    utils.hpp
 * @author  Carbon
 * @brief   通用工具函数库（忙等重试 / 参数读取）
 * @version 1.0
 * @date    2026-09-27
 *
 * @note    header-only，无 ROS 运行时依赖，供 shared_package 内部及下游包复用
 */

#pragma once

#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

namespace Utils
{

/**
 * @brief 忙等重试：反复执行 action，直到 judge() 判定成功才返回 true
 * @tparam Action 每次重试要执行的动作（无返回值）
 * @tparam Judge 判定是否成功的谓词（返回 bool）
 * @param action 动作
 * @param judge 判定
 * @param wait_time 两次尝试之间的间隔
 * @param retry_times 最大尝试次数，0 表示无限重试
 * @return true 判定成功；false 重试次数耗尽仍未成功
 */
template <typename Action, typename Judge>
inline bool RetryUntil(Action action, Judge judge,
                       std::chrono::milliseconds wait_time,
                       uint32_t retry_times = 0)
{
    uint32_t attempt = 0;
    while (true)
    {
        action();
        if (judge())
        {
            return true;
        }
        if (retry_times != 0 && ++attempt >= retry_times)
        {
            return false;
        }
        std::this_thread::sleep_for(wait_time);
    }
}

/**
 * @brief 从参数表里读一个参数，找不到返回默认值
 * @tparam T 目标类型
 * @param params 参数表（键值对）
 * @param key 要读取的键
 * @param def 找不到时的默认值
 * @return 参数值；找不到则返回 def
 */
template <typename T>
inline T ReadParam(const std::unordered_map<std::string, std::string> & params,
                   const char * key, T def)
{
    const auto it = params.find(key);
    if (it == params.end())
    {
        return def;
    }
    return static_cast<T>(std::stod(it->second));
}

/**
 * @brief 调试日志输出：按 windows_name 写入独立日志文件，每个名字对应一个独立终端窗口
 */
namespace Debug_Log
{

/**
 * @brief 日志文件目录前缀（默认 /tmp/robot_debug_）
 */
inline std::string& FilePrefix()
{
    static std::string prefix = "/tmp/robot_debug_";
    return prefix;
}

/**
 * @brief 打开调试窗口用的终端命令（默认 x-terminal-emulator，可改成自己系统的终端）
 */
inline std::string& Terminal()
{
    static std::string cmd = "x-terminal-emulator -e tail -f ";
    return cmd;
}

/**
 * @brief 输出模式：流式（滚动）还是表格更新式（原地刷新）
 */
enum class Mode
{
    Stream,   // 流式：每行换行追加，滚动显示
    Refresh,  // 表格更新式：回到行首原地刷新，不滚动
};

// 每个窗口的输出模式（默认 Stream），全局变量，跨翻译单元共享
inline std::unordered_map<std::string, Mode> g_window_modes;

/**
 * @brief 设置某个窗口的输出模式
 * @param windows_name 窗口标识符
 * @param mode 输出模式（Stream / Refresh）
 */
inline void SetMode(const std::string& windows_name, Mode mode)
{
    g_window_modes[windows_name] = mode;
}

/**
 * @brief 根据 windows_name 生成日志文件路径
 */
inline std::string FileFor(const std::string& windows_name)
{
    return FilePrefix() + windows_name + ".log";
}

// 已开过窗口的名字集合 + 保护锁
inline std::set<std::string>& SpawnedWindows()
{
    static std::set<std::string> names;
    return names;
}

inline std::mutex& WindowMutex()
{
    static std::mutex m;
    return m;
}

/**
 * @brief 首次使用某个 windows_name 时，开一个独立终端窗口 tail -f 对应日志
 * @param windows_name 窗口标识符
 */
inline void SpawnWindow(const std::string& windows_name)
{
    std::lock_guard<std::mutex> lock(WindowMutex());
    if (SpawnedWindows().count(windows_name) != 0)
    {
        return;  // 已开过
    }
    SpawnedWindows().insert(windows_name);

    // 无图形环境（DISPLAY/WAYLAND 都没有）就不开窗口，只写文件
    if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr)
    {
        return;
    }
    std::system((Terminal() + FileFor(windows_name) + " &").c_str());
}

// ---- 打印频率节流 ----

// 每个窗口的打印最小间隔（毫秒），默认 0 = 每次调用都打印
inline std::unordered_map<std::string, std::chrono::milliseconds>& PrintIntervals()
{
    static std::unordered_map<std::string, std::chrono::milliseconds> m;
    return m;
}

// 每个窗口上次打印的时间点
inline std::unordered_map<std::string, std::chrono::steady_clock::time_point>& LastPrintTimes()
{
    static std::unordered_map<std::string, std::chrono::steady_clock::time_point> m;
    return m;
}

// 保护节流状态的锁
inline std::mutex& ThrottleMutex()
{
    static std::mutex m;
    return m;
}

/**
 * @brief 设置某个窗口的打印最小间隔
 * @param windows_name 窗口标识符
 * @param interval 最小间隔，0 表示每次调用都打印
 */
inline void SetInterval(const std::string& windows_name, std::chrono::milliseconds interval)
{
    std::lock_guard<std::mutex> lock(ThrottleMutex());
    PrintIntervals()[windows_name] = interval;
}

/**
 * @brief 判断是否到了该打印的时候（内部用），到了则记录本次打印时间
 */
inline bool ShouldPrint(const std::string& windows_name)
{
    std::lock_guard<std::mutex> lock(ThrottleMutex());

    const auto it = PrintIntervals().find(windows_name);
    if (it == PrintIntervals().end() || it->second.count() <= 0)
    {
        return true;  // 没设置间隔，每拍都打
    }

    const auto now = std::chrono::steady_clock::now();
    const auto last_it = LastPrintTimes().find(windows_name);
    if (last_it != LastPrintTimes().end() && now - last_it->second < it->second)
    {
        return false;  // 间隔未到，跳过本次
    }
    LastPrintTimes()[windows_name] = now;
    return true;
}

/**
 * @brief 调试输出：按当前模式写入 windows_name 对应的日志，首次调用自动开窗口
 * @param windows_name 窗口标识符（同一标识符打到同一窗口）
 * @param fmt printf 风格格式串（字符串参数请用 %s + .c_str()，size_t 用 %zu）
 */
template <typename... Args>
inline void Print(const std::string& windows_name, const char* fmt, Args... args)
{
    if (!ShouldPrint(windows_name))
    {
        return;
    }
    std::FILE* f = std::fopen(FileFor(windows_name).c_str(), "a");
    if (f == nullptr)
    {
        return;
    }
    SpawnWindow(windows_name);

    const auto it = g_window_modes.find(windows_name);
    if (it != g_window_modes.end() && it->second == Mode::Refresh)
    {
        std::fprintf(f, "\033[1A\r\033[K");  // 上移一行并清行，配合末尾换行实现原地刷新
    }
    std::fprintf(f, fmt, args...);
    std::fprintf(f, "\n");  // 始终换行，保证 tail -f 逐行立即输出
    std::fflush(f);
    std::fclose(f);
}

}  // namespace Debug_Log

}  // namespace Utils
