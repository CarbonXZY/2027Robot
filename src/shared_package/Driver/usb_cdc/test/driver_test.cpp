/**
 * @file driver_test.cpp
 * @author lyh
 * @date 2026-10-12
 * @brief USB_CDC 封装的自包含测试.
 *
 * 设计目标:
 * - 可复现: 全部测试使用 pty(伪终端) 充当假设备, 不依赖任何真实硬件, 不依赖第三方库.
 *           用例之间互相隔离, 每个用例独占一对新的 pty, 可反复运行结果一致.
 * - 可回溯: 运行时打印构建时的 git 版本与测试用例名; 失败时给出 文件:行号 与表达式.
 * - 有目的: 每个用例对应封装的一条行为约定, 覆盖打开/收发/队列/回调/断连/重连/析构/并发.
 *
 * 用法:
 *   ./usb_cdc_test            # 运行全部用例
 *   ./usb_cdc_test -v         # 同时保留驱动自身的 stderr 诊断输出(默认会被屏蔽)
 */

#include "../usb_cdc.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <numeric>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <type_traits>
#include <vector>
#include <unistd.h>

// 由 Makefile 通过 -D 传入, 便于回溯是哪次提交构建的
#ifndef USB_CDC_TEST_GIT_REV
#define USB_CDC_TEST_GIT_REV "unknown"
#endif

namespace
{

// ============================================================================
// 极简测试框架: 记录检查次数与失败信息, 逐用例执行并打印结果
// ============================================================================

int  g_checks      = 0;   // 已执行的断言次数
int  g_failed      = 0;   // 失败的断言次数
bool g_case_failed = false;
std::vector<std::string> g_failure_messages;

template <typename T>
std::string Show(const T& value)
{
    std::ostringstream os;
    if constexpr (std::is_same_v<std::decay_t<T>, bool>)
    {
        os << (value ? "true" : "false");
    }
    else if constexpr (std::is_integral_v<std::decay_t<T>>)
    {
        os << static_cast<long long>(value);
    }
    else
    {
        os << value;
    }
    return os.str();
}

void ReportFailure(const char* file, int line, const std::string& message)
{
    ++g_failed;
    g_case_failed = true;

    std::ostringstream os;
    os << file << ":" << line << ": " << message;
    g_failure_messages.push_back(os.str());
}

#define CHECK(cond)                                                              \
    do {                                                                         \
        ++g_checks;                                                              \
        if (!(cond))                                                             \
        {                                                                        \
            ReportFailure(__FILE__, __LINE__, "断言失败: " #cond);                \
        }                                                                        \
    } while (0)

#define CHECK_EQ(a, b)                                                           \
    do {                                                                         \
        ++g_checks;                                                              \
        auto&& lhs_ = (a);                                                       \
        auto&& rhs_ = (b);                                                       \
        if (!(lhs_ == rhs_))                                                     \
        {                                                                        \
            ReportFailure(__FILE__, __LINE__,                                    \
                          std::string("期望 " #a " == " #b ", 实际 ") +           \
                          Show(lhs_) + " vs " + Show(rhs_));                     \
        }                                                                        \
    } while (0)

#define CHECK_LE(a, b)                                                           \
    do {                                                                         \
        ++g_checks;                                                              \
        auto&& lhs_ = (a);                                                       \
        auto&& rhs_ = (b);                                                       \
        if (!(lhs_ <= rhs_))                                                     \
        {                                                                        \
            ReportFailure(__FILE__, __LINE__,                                    \
                          std::string("期望 " #a " <= " #b ", 实际 ") +           \
                          Show(lhs_) + " vs " + Show(rhs_));                     \
        }                                                                        \
    } while (0)

// ============================================================================
// 工具: 关掉驱动自身的 stderr 诊断输出, 让测试日志保持干净
// ============================================================================

int g_saved_stderr = -1;

void SilenceStderr()
{
    if (g_saved_stderr >= 0)
    {
        return;
    }
    g_saved_stderr = dup(STDERR_FILENO);
    const int devnull = open("/dev/null", O_WRONLY);
    if (devnull >= 0)
    {
        dup2(devnull, STDERR_FILENO);
        close(devnull);
    }
}

void RestoreStderr()
{
    if (g_saved_stderr < 0)
    {
        return;
    }
    dup2(g_saved_stderr, STDERR_FILENO);
    close(g_saved_stderr);
    g_saved_stderr = -1;
}

// 在超时时间内轮询等待条件成立
template <typename Predicate>
bool WaitUntil(Predicate pred, int timeout_ms)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (pred())
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pred();
}

#if !defined(__SANITIZE_THREAD__)

// 本进程累计消耗的 CPU 时间(含所有线程), 单位秒
double CpuSeconds()
{
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    return static_cast<double>(usage.ru_utime.tv_sec) +
           static_cast<double>(usage.ru_utime.tv_usec) / 1e6 +
           static_cast<double>(usage.ru_stime.tv_sec) +
           static_cast<double>(usage.ru_stime.tv_usec) / 1e6;
}

#endif  // !__SANITIZE_THREAD__

// ============================================================================
// 假设备: 一对 pty, master 端由测试持有(模拟 USB 线), slave 端交给驱动打开
// ============================================================================

class FakeDevice
{
public:
    FakeDevice()
    {
        master_ = posix_openpt(O_RDWR | O_NOCTTY);
        if (master_ < 0)
        {
            std::fprintf(stderr, "posix_openpt failed: %s\n", std::strerror(errno));
            std::abort();
        }
        grantpt(master_);
        unlockpt(master_);

        char name[256] = {0};
        if (ptsname_r(master_, name, sizeof(name)) != 0)
        {
            std::fprintf(stderr, "ptsname_r failed: %s\n", std::strerror(errno));
            std::abort();
        }
        path_ = name;
    }

    ~FakeDevice()
    {
        Unplug();
    }

    FakeDevice(const FakeDevice&) = delete;
    FakeDevice& operator=(const FakeDevice&) = delete;

    // 驱动打开的路径, 形如 /dev/pts/3
    const std::string& Path() const { return path_; }

    // 模拟拔线: 关闭 master 端, 驱动那侧会收到 POLLHUP
    void Unplug()
    {
        if (master_ >= 0)
        {
            close(master_);
            master_ = -1;
        }
    }

    bool Alive() const { return master_ >= 0; }

    // 测试 -> 驱动 方向发送数据
    bool Send(const std::vector<uint8_t>& data)
    {
        if (master_ < 0)
        {
            return false;
        }
        size_t sent = 0;
        while (sent < data.size())
        {
            const ssize_t n = write(master_, data.data() + sent, data.size() - sent);
            if (n <= 0)
            {
                return false;
            }
            sent += static_cast<size_t>(n);
        }
        return true;
    }

    // 驱动 -> 测试 方向收取数据, 最多等 timeout_ms, 收满 want 字节或超时返回
    std::vector<uint8_t> Recv(size_t want, int timeout_ms)
    {
        std::vector<uint8_t> out;
        if (master_ < 0)
        {
            return out;
        }

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);
        while (out.size() < want)
        {
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  deadline - std::chrono::steady_clock::now()).count();
            if (left <= 0)
            {
                break;
            }

            pollfd pfd{};
            pfd.fd     = master_;
            pfd.events = POLLIN;
            if (poll(&pfd, 1, static_cast<int>(left)) <= 0)
            {
                break;
            }

            uint8_t buf[1024];
            const ssize_t n = read(master_, buf, sizeof(buf));
            if (n <= 0)
            {
                break;
            }
            out.insert(out.end(), buf, buf + n);
        }
        return out;
    }

private:
    int         master_ = -1;
    std::string path_;
};

// ============================================================================
// 回调挂载点: 驱动的回调是裸函数指针, 这里用文件级单例转接到当前用例的对象上
// ============================================================================

class RxRecorder
{
public:
    void Reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.clear();
        calls_ = 0;
    }

    void OnData(uint8_t* data, uint16_t length)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.insert(data_.end(), data, data + length);
        ++calls_;
    }

    std::vector<uint8_t> Data()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_;
    }

    int Calls()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return calls_;
    }

private:
    std::mutex           mutex_;
    std::vector<uint8_t> data_;
    int                  calls_ = 0;
};

RxRecorder* g_rx = nullptr;

void RxTrampoline(uint8_t* data, uint16_t length)
{
    if (g_rx != nullptr)
    {
        g_rx->OnData(data, length);
    }
}

std::atomic<int> g_disconnect_calls{0};

void DisconnectTrampoline()
{
    ++g_disconnect_calls;
}

// 在断连回调里直接重连的挂载点: 记录 Connect() 的返回值, 供用例判断是否卡死
std::atomic<Driver::USB_CDC*> g_reconnect_cdc{nullptr};
std::atomic<int>              g_reconnect_result{0};  // 0 未执行, 1 成功, -1 失败

void ReconnectInCallbackTrampoline()
{
    ++g_disconnect_calls;

    Driver::USB_CDC* cdc = g_reconnect_cdc.load();
    if (cdc == nullptr)
    {
        g_reconnect_result = -1;
        return;
    }

    // 这就是本次改动支持的用法: 回调里直接重连, 不会 join 当前线程而死锁
    g_reconnect_result = cdc->Connect() ? 1 : -1;
}

// ============================================================================
// 用例
// ============================================================================

// ---- 打开与生命周期 -------------------------------------------------------

void Case_Construct_opens_and_starts()
{
    FakeDevice  dev;
    Driver::USB_CDC cdc(dev.Path());

    CHECK(cdc.IsOpen());
    CHECK(cdc.IsRunning());
}

void Case_Construct_failed_device_reports_closed()
{
    Driver::USB_CDC cdc("/dev/usb_cdc_test_no_such_device");

    CHECK(!cdc.IsOpen());
    CHECK(!cdc.IsRunning());

    // 设备没打开, 同步发送必须直接失败而不是崩溃
    uint8_t data[4] = {1, 2, 3, 4};
    CHECK_EQ(cdc.TransmitNow(data, sizeof(data)), 0);
}

void Case_Connect_is_idempotent()
{
    FakeDevice       dev;
    Driver::USB_CDC  cdc(dev.Path());

    // 重复 Connect 不能触发 std::terminate, 也不能泄漏 fd
    CHECK(cdc.Connect());
    CHECK(cdc.IsOpen());
    CHECK(cdc.IsRunning());

    // 重连之后链路依然可用
    const std::vector<uint8_t> frame(8, 0x5A);
    CHECK_EQ(cdc.TransmitNow(const_cast<uint8_t*>(frame.data()),
                             static_cast<uint16_t>(frame.size())), 1);
    CHECK_EQ(dev.Recv(frame.size(), 500).size(), frame.size());
}

void Case_Disconnect_closes_device()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    cdc.Disconnect();

    CHECK(!cdc.IsOpen());
    CHECK(!cdc.IsRunning());
}

void Case_Reconnect_after_manual_disconnect()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    cdc.Disconnect();
    CHECK(!cdc.IsOpen());

    CHECK(cdc.Connect());
    CHECK(cdc.IsOpen());
    CHECK(cdc.IsRunning());

    // 重连后数据通路恢复
    const std::vector<uint8_t> frame(16, 0x33);
    CHECK_EQ(cdc.TransmitNow(const_cast<uint8_t*>(frame.data()),
                             static_cast<uint16_t>(frame.size())), 1);
    CHECK(dev.Recv(frame.size(), 500) == frame);
}

// ---- 同步发送 -------------------------------------------------------------

void Case_TransmitNow_writes_exact_bytes()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    std::vector<uint8_t> frame(64);
    for (size_t i = 0; i < frame.size(); ++i)
    {
        frame[i] = static_cast<uint8_t>(i * 7 + 1);
    }

    CHECK_EQ(cdc.TransmitNow(frame.data(), static_cast<uint16_t>(frame.size())), 1);
    CHECK(dev.Recv(frame.size(), 500) == frame);
}

void Case_TransmitNow_rejects_invalid_args()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    uint8_t data[4] = {1, 2, 3, 4};

    CHECK_EQ(cdc.TransmitNow(nullptr, 4), 0);   // 空指针
    CHECK_EQ(cdc.TransmitNow(data, 0), 0);      // 长度为零
}

void Case_TransmitNow_fails_after_disconnect()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    cdc.Disconnect();

    uint8_t data[4] = {1, 2, 3, 4};
    CHECK_EQ(cdc.TransmitNow(data, sizeof(data)), 0);
}

// ---- 异步发送与队列 -------------------------------------------------------

void Case_TransmitAdd_delivers_frame()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    std::vector<uint8_t> frame(32);
    std::iota(frame.begin(), frame.end(), uint8_t{0x10});

    CHECK_EQ(cdc.TransmitAdd(frame.data(), static_cast<uint16_t>(frame.size())), 1);
    CHECK(dev.Recv(frame.size(), 500) == frame);
}

void Case_TransmitAdd_preserves_fifo_order()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    const int kFrames = 12;
    const int kSize   = 8;
    std::vector<uint8_t> expected;
    for (int i = 0; i < kFrames; ++i)
    {
        std::vector<uint8_t> frame(kSize, static_cast<uint8_t>(0x80 + i));
        CHECK_EQ(cdc.TransmitAdd(frame.data(), static_cast<uint16_t>(frame.size())), 1);
        expected.insert(expected.end(), frame.begin(), frame.end());
    }

    CHECK(dev.Recv(expected.size(), 1000) == expected);
}

void Case_TransmitAdd_deep_copies_buffer()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    std::vector<uint8_t> frame(16, 0xAA);
    const std::vector<uint8_t> original = frame;

    CHECK_EQ(cdc.TransmitAdd(frame.data(), static_cast<uint16_t>(frame.size())), 1);

    // 入队后立刻覆写调用方缓冲区, 驱动内部应是深拷贝, 收到的不受影响
    std::fill(frame.begin(), frame.end(), uint8_t{0x55});

    CHECK(dev.Recv(original.size(), 500) == original);
}

void Case_TransmitAdd_queue_capacity()
{
    // 用打不开的设备: 收发线程不会启动, 队列只进不出, 容量行为可确定地测出来
    Driver::USB_CDC cdc("/dev/usb_cdc_test_no_such_device");
    CHECK(!cdc.IsOpen());

    std::vector<uint8_t> frame(4, 0x01);

    for (uint32_t i = 0; i < Driver::USB_CDC::kTxQueueSize; ++i)
    {
        CHECK_EQ(cdc.TransmitAdd(frame.data(), static_cast<uint16_t>(frame.size())), 1);
    }

    // 第 65 帧: 队列已满
    CHECK_EQ(cdc.TransmitAdd(frame.data(), static_cast<uint16_t>(frame.size())), 0);

    // 参数非法
    CHECK_EQ(cdc.TransmitAdd(nullptr, 4), 0);
    CHECK_EQ(cdc.TransmitAdd(frame.data(), 0), 0);
}

// ---- 接收回调 -------------------------------------------------------------

void Case_Rx_receives_exact_bytes()
{
    FakeDevice        dev;
    Driver::USB_CDC   cdc(dev.Path());
    RxRecorder        recorder;

    g_rx = &recorder;
    cdc.RegisterRxCallback(&RxTrampoline);

    std::vector<uint8_t> payload(48);
    for (size_t i = 0; i < payload.size(); ++i)
    {
        payload[i] = static_cast<uint8_t>(i ^ 0x5A);
    }

    CHECK(dev.Send(payload));
    CHECK(WaitUntil([&] { return recorder.Data() == payload; }, 1000));

    g_rx = nullptr;
}

void Case_Rx_accumulates_split_frames()
{
    FakeDevice        dev;
    Driver::USB_CDC   cdc(dev.Path());
    RxRecorder        recorder;

    g_rx = &recorder;
    cdc.RegisterRxCallback(&RxTrampoline);

    // 分三批发送, 驱动侧可能分多次回调, 累计内容应与原始数据一致
    std::vector<uint8_t> all;
    for (int chunk = 0; chunk < 3; ++chunk)
    {
        std::vector<uint8_t> part(20, static_cast<uint8_t>(0xC0 + chunk));
        CHECK(dev.Send(part));
        all.insert(all.end(), part.begin(), part.end());
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    CHECK(WaitUntil([&] { return recorder.Data() == all; }, 1000));
    CHECK(recorder.Calls() >= 1);

    g_rx = nullptr;
}

void Case_Rx_callback_can_be_replaced()
{
    FakeDevice        dev;
    Driver::USB_CDC   cdc(dev.Path());
    RxRecorder        first;
    RxRecorder        second;

    g_rx = &first;
    cdc.RegisterRxCallback(&RxTrampoline);
    const std::vector<uint8_t> a(8, 0x11);
    CHECK(dev.Send(a));
    CHECK(WaitUntil([&] { return first.Data() == a; }, 1000));

    // 换成第二个回调, 之后的数据只应进第二个
    g_rx = &second;
    cdc.RegisterRxCallback(&RxTrampoline);
    const int first_calls = first.Calls();

    const std::vector<uint8_t> b(8, 0x22);
    CHECK(dev.Send(b));
    CHECK(WaitUntil([&] { return second.Data() == b; }, 1000));
    CHECK_EQ(first.Calls(), first_calls);

    g_rx = nullptr;
}

void Case_Rx_callback_can_be_unregistered()
{
    FakeDevice        dev;
    Driver::USB_CDC   cdc(dev.Path());
    RxRecorder        recorder;

    g_rx = &recorder;
    cdc.RegisterRxCallback(&RxTrampoline);
    cdc.RegisterRxCallback(nullptr);   // 取消注册

    CHECK(dev.Send(std::vector<uint8_t>(16, 0x77)));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    CHECK_EQ(recorder.Calls(), 0);

    g_rx = nullptr;
}

void Case_Rx_raw_mode_has_no_byte_translation()
{
    FakeDevice        dev;
    Driver::USB_CDC   cdc(dev.Path());
    RxRecorder        recorder;

    g_rx = &recorder;
    cdc.RegisterRxCallback(&RxTrampoline);

    // 换行, 回车, NUL 等特殊字节在原始模式下必须原样通过
    const std::vector<uint8_t> tricky = {0x0A, 0x0D, 0x00, 0x11, 0x13, 0x1A, 0x7F, 0xFF};

    CHECK(dev.Send(tricky));
    CHECK(WaitUntil([&] { return recorder.Data() == tricky; }, 1000));

    // 反方向(驱动 -> 测试)同样不应被转换
    CHECK_EQ(cdc.TransmitNow(const_cast<uint8_t*>(tricky.data()),
                             static_cast<uint16_t>(tricky.size())), 1);
    CHECK(dev.Recv(tricky.size(), 500) == tricky);

    g_rx = nullptr;
}

// ---- 断连检测与回调 -------------------------------------------------------

void Case_Unplug_is_detected_and_reported()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    g_disconnect_calls = 0;
    cdc.RegisterDisconnectCallback(&DisconnectTrampoline);

    CHECK(cdc.IsOpen());
    dev.Unplug();   // 拔线

    CHECK(WaitUntil([&] { return g_disconnect_calls.load() == 1; }, 1000));
    CHECK(WaitUntil([&] { return !cdc.IsRunning(); }, 1000));
    CHECK(!cdc.IsOpen());

    // 只应通知一次
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    CHECK_EQ(g_disconnect_calls.load(), 1);
}

void Case_Manual_disconnect_does_not_notify()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    g_disconnect_calls = 0;
    cdc.RegisterDisconnectCallback(&DisconnectTrampoline);

    cdc.Disconnect();   // 用户主动断开

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK_EQ(g_disconnect_calls.load(), 0);
    CHECK(!cdc.IsOpen());
    CHECK(!cdc.IsRunning());
}

// 断连回调里直接 Connect() 重连. 回调跑在接收线程上, 而 Connect 内部会先 Disconnect,
// 所以这里验证的是自 join 保护: 卡住的话下面的 WaitUntil 永远等不到结果.
// 设备路径用软链接当"稳定名" —— 拔出后 pty 号会变, 名字不变, 模拟真实的 by-id 路径.
void Case_Reconnect_from_disconnect_callback()
{
    const std::string link = "/tmp/usb_cdc_test_link_" + std::to_string(getpid());

    FakeDevice dev_old;
    ::unlink(link.c_str());
    CHECK(symlink(dev_old.Path().c_str(), link.c_str()) == 0);

    Driver::USB_CDC cdc(link);
    CHECK(cdc.IsOpen());

    // 重新插入的"新设备": 先把稳定路径指过去, 制造设备重新枚举的效果
    FakeDevice dev_new;
    ::unlink(link.c_str());
    CHECK(symlink(dev_new.Path().c_str(), link.c_str()) == 0);

    RxRecorder recorder;
    g_rx = &recorder;
    // 回调在重连前注册, 重连后必须仍然有效(Connect 不会清掉已注册的回调)
    cdc.RegisterRxCallback(&RxTrampoline);
    g_disconnect_calls = 0;
    g_reconnect_result = 0;
    g_reconnect_cdc    = &cdc;
    cdc.RegisterDisconnectCallback(&ReconnectInCallbackTrampoline);

    dev_old.Unplug();   // 拔线, 回调里立刻重连

    CHECK(WaitUntil([&] { return g_reconnect_result.load() != 0; }, 2000));
    CHECK_EQ(g_reconnect_result.load(), 1);
    CHECK_EQ(g_disconnect_calls.load(), 1);
    CHECK(cdc.IsOpen());
    CHECK(cdc.IsRunning());

    // 重连后的新链路必须真的能用
    const std::vector<uint8_t> frame = {0xAA, 0x5A, 0x55};
    CHECK(dev_new.Send(frame));
    CHECK(WaitUntil([&] { return recorder.Data() == frame; }, 1000));

    g_reconnect_cdc = nullptr;
    g_rx            = nullptr;   // 旧的接收线程已被 detach, 这里保证它不再解引用

    ::unlink(link.c_str());
}

// ---- 析构 -----------------------------------------------------------------

void Case_Destructor_returns_promptly_when_idle()
{
    FakeDevice dev;

    const auto begin = std::chrono::steady_clock::now();
    {
        // 设备空闲无数据, 析构不能挂在阻塞读上
        Driver::USB_CDC cdc(dev.Path());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - begin).count();

    CHECK_LE(elapsed_ms, 500);
}

void Case_Destructor_returns_promptly_with_pending_frames()
{
    FakeDevice dev;

    const auto begin = std::chrono::steady_clock::now();
    {
        Driver::USB_CDC cdc(dev.Path(), 50);   // 长周期, 让队列来得及积压
        for (int i = 0; i < 16; ++i)
        {
            std::vector<uint8_t> frame(8, static_cast<uint8_t>(i));
            cdc.TransmitAdd(frame.data(), static_cast<uint16_t>(frame.size()));
        }
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - begin).count();

    CHECK_LE(elapsed_ms, 1000);
}

void Case_Destructor_returns_promptly_after_unplug()
{
    FakeDevice dev;

    const auto begin = std::chrono::steady_clock::now();
    {
        Driver::USB_CDC cdc(dev.Path());
        dev.Unplug();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - begin).count();

    CHECK_LE(elapsed_ms, 500);
}

// ---- 并发与帧完整性 -------------------------------------------------------

// 一帧 = 32 个相同字节, 两个发送方向的 tag 取值互不重叠且相邻帧必然不同,
// 因此把收到的字节流按"连续相同字节"切段后, 每段长度必须恰好是一帧.
void Case_No_frame_tearing_under_concurrent_senders()
{
    constexpr size_t kFrameSize = 32;
    constexpr int    kRounds    = 200;

    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path(), 2);   // 短周期, 加大后台线程的发送压力

    std::vector<uint8_t> stream;
    std::mutex           stream_mutex;
    std::atomic<bool>    keep_reading{true};

    // 持续抽干 master, 否则 pty 缓冲写满会阻塞驱动线程.
    // 读取的分块方式不影响判定: 所有分块按序拼接后就是驱动写出的完整字节流.
    std::thread drain([&] {
        while (keep_reading)
        {
            std::vector<uint8_t> chunk = dev.Recv(kFrameSize, 2);
            if (!chunk.empty())
            {
                std::lock_guard<std::mutex> lock(stream_mutex);
                stream.insert(stream.end(), chunk.begin(), chunk.end());
            }
        }
    });

    // 同步发送方向
    std::thread sync_sender([&] {
        for (int i = 0; i < kRounds; ++i)
        {
            std::vector<uint8_t> frame(kFrameSize, static_cast<uint8_t>(0xA0 + (i % 16)));
            cdc.TransmitNow(frame.data(), static_cast<uint16_t>(frame.size()));
        }
    });

    // 异步发送方向
    for (int i = 0; i < kRounds; ++i)
    {
        std::vector<uint8_t> frame(kFrameSize, static_cast<uint8_t>(0xB0 + (i % 16)));
        cdc.TransmitAdd(frame.data(), static_cast<uint16_t>(frame.size()));
    }

    sync_sender.join();

    // 等流稳定(连续 200ms 没有新数据)再停读, 避免把最后一帧截断成半帧
    {
        size_t last_size = 1;
        int    quiet     = 0;
        while (quiet < 10)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            size_t current = 0;
            {
                std::lock_guard<std::mutex> lock(stream_mutex);
                current = stream.size();
            }
            if (current == last_size)
            {
                ++quiet;
            }
            else
            {
                quiet     = 0;
                last_size = current;
            }
        }
    }
    keep_reading = false;
    drain.join();

    std::vector<uint8_t> collected;
    {
        std::lock_guard<std::mutex> lock(stream_mutex);
        collected = stream;
    }

    CHECK(!collected.empty());
    CHECK_EQ(collected.size() % kFrameSize, size_t{0});

    // 切段检查: 任何一段长度不等于整帧都说明发生了撕裂或交错
    int    segments  = 0;
    int    torn      = 0;
    size_t index     = 0;
    while (index < collected.size())
    {
        size_t end = index;
        while (end < collected.size() && collected[end] == collected[index])
        {
            ++end;
        }
        ++segments;
        if (end - index != kFrameSize)
        {
            ++torn;
        }
        index = end;
    }

    CHECK_EQ(torn, 0);
}

// ---- 周期配置 -------------------------------------------------------------

void Case_SetTxPeriodMs_is_observable()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path(), 20);

    CHECK_EQ(cdc.GetTxPeriodMs(), uint32_t{20});
    cdc.SetTxPeriodMs(5);
    CHECK_EQ(cdc.GetTxPeriodMs(), uint32_t{5});
}

// ---- 设备查找 -------------------------------------------------------------

void Case_FindDevice_direct_path_passthrough()
{
    CHECK_EQ(Driver::FindDevice("/dev/ttyACM99"), std::string("/dev/ttyACM99"));
}

void Case_FindDevice_empty_and_invalid_regex()
{
    CHECK_EQ(Driver::FindDevice(""), std::string(""));
    CHECK_EQ(Driver::FindDevice("["), std::string(""));   // 非法正则
}

void Case_FindDevice_no_match_returns_empty()
{
    CHECK_EQ(Driver::FindDevice("usb_cdc_test_no_such_device_zzz"), std::string(""));
}

// ============================================================================
// 计时类用例: 测量本身有系统噪声, 且线程检查器下会失真, 因此单独分组
// ============================================================================

#if !defined(__SANITIZE_THREAD__)

void Case_Idle_cpu_usage_is_low()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path());

    std::this_thread::sleep_for(std::chrono::milliseconds(100));   // 预热

    const double cpu_begin  = CpuSeconds();
    const auto   wall_begin = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const double cpu  = CpuSeconds() - cpu_begin;
    const double wall = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - wall_begin).count();

    // 空闲时收发线程都应睡眠等待, 占用远低于一个核(阈值放宽以避免偶发抖动)
    const double percent = cpu / wall * 100.0;
    CHECK_LE(percent, 25.0);
}

void Case_TransmitAdd_latency_bounded_by_period()
{
    FakeDevice      dev;
    Driver::USB_CDC cdc(dev.Path(), 10);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));   // 让发送线程进入睡眠

    double worst_ms = 0.0;
    for (int i = 0; i < 10; ++i)
    {
        std::vector<uint8_t> frame(8, static_cast<uint8_t>(i));
        const auto begin = std::chrono::steady_clock::now();
        CHECK_EQ(cdc.TransmitAdd(frame.data(), static_cast<uint16_t>(frame.size())), 1);

        const std::vector<uint8_t> got = dev.Recv(frame.size(), 1000);
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - begin).count();

        CHECK(got == frame);
        worst_ms = std::max(worst_ms, ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 入队到发出的延迟上界是一个发送周期, 阈值放宽到 200ms 以容忍调度抖动
    CHECK_LE(worst_ms, 200.0);
}

#endif  // !__SANITIZE_THREAD__

// ============================================================================
// 用例注册与执行
// ============================================================================

struct TestCase
{
    const char* name;
    void (*fn)();
};

const TestCase kCases[] = {
    {"Construct_opens_and_starts",              Case_Construct_opens_and_starts},
    {"Construct_failed_device_reports_closed",  Case_Construct_failed_device_reports_closed},
    {"Connect_is_idempotent",                   Case_Connect_is_idempotent},
    {"Disconnect_closes_device",                Case_Disconnect_closes_device},
    {"Reconnect_after_manual_disconnect",       Case_Reconnect_after_manual_disconnect},
    {"TransmitNow_writes_exact_bytes",          Case_TransmitNow_writes_exact_bytes},
    {"TransmitNow_rejects_invalid_args",        Case_TransmitNow_rejects_invalid_args},
    {"TransmitNow_fails_after_disconnect",      Case_TransmitNow_fails_after_disconnect},
    {"TransmitAdd_delivers_frame",              Case_TransmitAdd_delivers_frame},
    {"TransmitAdd_preserves_fifo_order",        Case_TransmitAdd_preserves_fifo_order},
    {"TransmitAdd_deep_copies_buffer",          Case_TransmitAdd_deep_copies_buffer},
    {"TransmitAdd_queue_capacity",              Case_TransmitAdd_queue_capacity},
    {"Rx_receives_exact_bytes",                 Case_Rx_receives_exact_bytes},
    {"Rx_accumulates_split_frames",             Case_Rx_accumulates_split_frames},
    {"Rx_callback_can_be_replaced",             Case_Rx_callback_can_be_replaced},
    {"Rx_callback_can_be_unregistered",         Case_Rx_callback_can_be_unregistered},
    {"Rx_raw_mode_has_no_byte_translation",     Case_Rx_raw_mode_has_no_byte_translation},
    {"Unplug_is_detected_and_reported",         Case_Unplug_is_detected_and_reported},
    {"Manual_disconnect_does_not_notify",       Case_Manual_disconnect_does_not_notify},
    {"Reconnect_from_disconnect_callback",      Case_Reconnect_from_disconnect_callback},
    {"Destructor_returns_promptly_when_idle",   Case_Destructor_returns_promptly_when_idle},
    {"Destructor_returns_promptly_with_pending_frames",
                                                Case_Destructor_returns_promptly_with_pending_frames},
    {"Destructor_returns_promptly_after_unplug", Case_Destructor_returns_promptly_after_unplug},
    {"No_frame_tearing_under_concurrent_senders",
                                                Case_No_frame_tearing_under_concurrent_senders},
    {"SetTxPeriodMs_is_observable",             Case_SetTxPeriodMs_is_observable},
    {"FindDevice_direct_path_passthrough",      Case_FindDevice_direct_path_passthrough},
    {"FindDevice_empty_and_invalid_regex",      Case_FindDevice_empty_and_invalid_regex},
    {"FindDevice_no_match_returns_empty",       Case_FindDevice_no_match_returns_empty},
#if !defined(__SANITIZE_THREAD__)
    {"Idle_cpu_usage_is_low",                   Case_Idle_cpu_usage_is_low},
    {"TransmitAdd_latency_bounded_by_period",   Case_TransmitAdd_latency_bounded_by_period},
#endif
};

void PrintHeader(bool verbose)
{
    char timestamp[64] = {0};
    const std::time_t now = std::time(nullptr);
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

    std::printf("================================================================\n");
    std::printf(" USB_CDC 驱动测试\n");
    std::printf(" git 版本 : %s\n", USB_CDC_TEST_GIT_REV);
    std::printf(" 编译器   : %s\n", __VERSION__);
    std::printf(" 构建时间 : %s %s\n", __DATE__, __TIME__);
    std::printf(" 运行时间 : %s\n", timestamp);
    std::printf(" 设备来源 : 伪终端 pty (无需真实硬件)\n");
    std::printf(" stderr   : %s\n", verbose ? "保留驱动诊断" : "已屏蔽驱动诊断");
    std::printf("================================================================\n\n");
}

}  // namespace

int main(int argc, char** argv)
{
    bool verbose = false;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "-v") == 0 || std::strcmp(argv[i], "--verbose") == 0)
        {
            verbose = true;
        }
        else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0)
        {
            std::printf("用法: %s [-v|--verbose]\n", argv[0]);
            return 0;
        }
        else
        {
            std::printf("未知参数: %s (用 -h 查看用法)\n", argv[i]);
            return 2;
        }
    }

    PrintHeader(verbose);
    if (!verbose)
    {
        SilenceStderr();
    }

    int passed = 0;
    int failed_cases = 0;

    for (const TestCase& test : kCases)
    {
        g_case_failed = false;
        g_failure_messages.clear();

        const auto begin = std::chrono::steady_clock::now();
        std::printf("[ RUN      ] %s\n", test.name);
        std::fflush(stdout);

        try
        {
            test.fn();
        }
        catch (const std::exception& e)
        {
            ReportFailure(__FILE__, __LINE__,
                          std::string("抛出异常: ") + e.what());
        }
        catch (...)
        {
            ReportFailure(__FILE__, __LINE__, "抛出未知异常");
        }

        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - begin).count();

        if (g_case_failed)
        {
            ++failed_cases;
            for (const std::string& message : g_failure_messages)
            {
                std::printf("             %s\n", message.c_str());
            }
            std::printf("[  FAILED  ] %s (%.1f ms)\n", test.name, ms);
        }
        else
        {
            ++passed;
            std::printf("[       OK ] %s (%.1f ms)\n", test.name, ms);
        }
        std::fflush(stdout);
    }

    if (!verbose)
    {
        RestoreStderr();
    }

    const int total = static_cast<int>(sizeof(kCases) / sizeof(kCases[0]));
    std::printf("\n================================================================\n");
    std::printf(" 用例: %d 通过, %d 失败, 共 %d\n", passed, failed_cases, total);
    std::printf(" 断言: %d 次, %d 次失败\n", g_checks, g_failed);
    std::printf(" 结果: %s\n", failed_cases == 0 ? "PASS" : "FAIL");
    std::printf("================================================================\n");

    return failed_cases == 0 ? 0 : 1;
}
