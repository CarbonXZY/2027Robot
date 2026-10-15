/**
 * @file    tsk_config_and_callback.cpp
 * @author  Carbon
 * @brief   任务配置与回调：USB-CDC 收发接线、链路存活检测
 * @version 1.0
 * @date    2026-09-27
 *
 * @note    连接底层 USB-CDC 驱动与中间层 CommunicationInterface
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include "Communication_Interface.hpp"
#include "usb_cdc.hpp"
#include "utils.hpp"

namespace Task
{

/* Private variables ---------------------------------------------------------*/

static std::unique_ptr<Driver::USB_CDC> g_usb_device;  // USB-CDC 设备实例
static uint32_t g_alive_flag = 0;                      // 累计收包计数
static uint32_t g_pre_alive_flag = 0;                  // 上一拍收包计数
static std::string g_device_path;                      // 设备实际打开时用的路径

// "进程正在关闭"谓词。由 TaskInit 从参数里存下来，供断连回调使用 ——
// 回调是裸函数指针、没有捕获，拿不到 TaskInit 的参数。
// 写入发生在建立任何回调关系之前，之后只读，因此跨线程访问是安全的。
static std::function<bool()> g_shutdown_requested;
static std::string g_reported_path;  // 已经告警过的"设备跑到别的节点去了"路径

/* Public variables ----------------------------------------------------------*/

bool g_mcu_alive = false;    // 下位机链路存活标志位
bool g_initialized = false;  // TaskInit() 是否已完成

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 尝试打开设备一次。首次启动要连设备带对象一起建，重连则只重开已有对象
 * @return true 设备已打开
 */
static bool TryOpenDevice()
{
    if (g_usb_device == nullptr)
    {
        // 首次启动：设备可能还没枚举出来，每次都要重新找路径。
        // 找不到就保持为空，留给下一轮
        const std::string device = Driver::FindDevice(STM32_CDC);
        if (device.empty())
        {
            return false;
        }

        g_device_path = device;
        g_usb_device = std::make_unique<Driver::USB_CDC>(device);
        return g_usb_device->IsOpen();
    }

    // 重连：设备路径在构造时就绑在对象上了，Connect 只会重开这同一条路径，直接重开即可。
    // 收发回调是成员变量，Connect 不会清掉，所以**不需要**重新注册 ——
    // 反过来，如果改成销毁对象再重建，就必须重新注册，否则重连后收不到任何数据。
    if (g_usb_device->Connect())
    {
        g_reported_path.clear();  // 重连成功，下次掉线重新计数告警
        return true;
    }

    // 重开失败时多查一次设备：若设备其实回来了、只是节点号变了，Connect 会一直失败。
    // 这是本实现覆盖不到的情况，给一行告警，避免无声空转（同一条路径只告警一次）
    const std::string found = Driver::FindDevice(STM32_CDC);
    if (!found.empty() && found != g_device_path && found != g_reported_path)
    {
        g_reported_path = found;
        std::fprintf(stderr,
                     "Task: 设备已出现在 %s，但对象绑定的是 %s，重连会一直失败；"
                     "请检查设备是否被别的进程占用了原来的节点\n",
                     found.c_str(), g_device_path.c_str());
    }

    return false;
}

/**
 * @brief 一直重试到设备打开为止，期间收到关闭请求就放弃
 * @param shutdown_requested 可选的"进程正在关闭"谓词，为空表示不响应关闭请求
 * @return true 设备已打开；false 被关闭请求打断
 * @note TaskInit(首次启动)与 McuDisconnectCallback(掉线重连)共用这一段：
 *       同一套"找设备 -> 打开 -> 判定 IsOpen"的动作，同样 500ms 一轮、无限重试、
 *       同样能被 Ctrl+C 打断，所以重连的表现与刚启动完全一致
 */
static bool WaitForDeviceOpen(const std::function<bool()> & shutdown_requested)
{
    return Utils::RetryUntil(
        TryOpenDevice,
        [] { return g_usb_device != nullptr && g_usb_device->IsOpen(); },
        std::chrono::milliseconds(500),
        0,
        [&] { return shutdown_requested != nullptr && shutdown_requested(); });
}

/**
 * @brief 收：USB 接收线程把一整包递进来。先记一次计数，链路确认之前一律不解包
 */
static void McuRxCallback(uint8_t *data, uint16_t length)
{
    g_alive_flag += 1;

    if (!g_mcu_alive)
    {
        return;
    }
    Middleware::g_usb_communication_interface.RxRptlCallback(data, length);
}

/**
 * @brief USB 掉线后立刻反复尝试重连，重复刚启动时的行为
 * @warning 只能重连构造函数时绑定的ttyACM设备，换了编号会一直连不上
 * @note 本回调由驱动在接收线程里调用，调用时设备资源已经归还（fd 已关闭、
 *       收发线程正在退出），所以可以直接在这里 Connect 重连，不会自 join 死锁。
 *       该接收线程会被驱动 detach，回调返回后自行结束，因此可以放心地在这里一直
 *       等待设备回来；需要保证的是"能返回"，关闭请求就是那个出口。
 */
static void McuDisconnectCallback()
{
    std::fprintf(stderr, "Task: 下位机断开，开始重连\n");

    // 设备没回来之前 TaskLoop 不会往外发（靠 AlivePeriodElapsedCallback 判定），
    // 重连后收到下位机回包会自动恢复，和刚启动时一样，这里不需要碰存活标志位
    if (!WaitForDeviceOpen(g_shutdown_requested))
    {
        // 正在关闭进程：不再重连，直接返回让这个线程结束
        std::fprintf(stderr, "Task: 重连等待中收到关闭请求，放弃重连\n");
        return;
    }

    std::fprintf(stderr, "Task: 下位机已重连\n");
}

/**
 * @brief 发：Send() 打包好一整包后在这里交给设备
 */
static int64_t McuSendData(uint8_t *data, size_t length)
{
    if (g_usb_device->TransmitAdd(data, static_cast<uint16_t>(length)) == 0)
    {
        return -1;
    }
    return static_cast<int64_t>(length);
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief 到点检测一次下位机是否存活，由控制循环按窗口周期调用
 */
void AlivePeriodElapsedCallback()
{
    if (g_alive_flag == g_pre_alive_flag)
    {
        // 下位机断开连接
        g_mcu_alive = false;
    }
    else
    {
        // 下位机保持连接
        g_mcu_alive = true;
    }
    g_pre_alive_flag = g_alive_flag;
}

/**
 * @brief 控制循环每拍调一次：存活才往外发
 */
void TaskLoop()
{
    // 设备还没开起来：可能所有组件都还没走到 on_configure
    if (!g_initialized)
    {
        return;
    }

    AlivePeriodElapsedCallback();

    // 没收到下位机回显之前一个字节都不发
    if (!g_mcu_alive)
    {
        return;
    }

    Middleware::g_usb_communication_interface.Send();
}

bool TaskInit(const std::function<bool()> & shutdown_requested)
{
    // 设备只开一次：chassis 和 leg 各自都会调进来，第二次起直接返回
    if (g_initialized)
    {
        return true;
    }

    // 存下关闭谓词：断连回调在驱动的接收线程里跑，拿不到本函数的参数。
    // 三个调用点传的都是无捕获 lambda，存成静态量跨线程调用是安全的
    g_shutdown_requested = shutdown_requested;

    // USB 可能还没枚举完成(刚上电/刚插拔), 打不开就忙等重试,
    // 而不是直接返回 false 让 on_configure 报错、把 ROS 整个带崩。
    // shutdown_requested 是给"Ctrl+C"留的出口: 本函数跑在 ros2_control_node 的构造期
    // (on_configure 里), 主线程一旦卡在这里, 服务/控制器都起不来, 也收不到信号;
    // 收到 SIGINT/SIGTERM 后该谓词为真, 这里就能跳出等待。
    // 这段和掉线重连共用 WaitForDeviceOpen，两边行为完全一致
    const bool opened = WaitForDeviceOpen(shutdown_requested);

    if (!opened)
    {
        // 进程正在关闭，这里不能 return false：Humble 的 ControllerManager 在构造期
        // 遇到硬件 configure 失败会抛异常（实测报 "Failed to set the initial state of
        // the component : ... to active"），异常没人接 → std::terminate → abort，
        // 于是每次 Ctrl+C 都留下一份 core dump（本机 core_pattern 走 apport，落在
        // /var/crash），表现成"崩溃"而不是"被打断"。此时设备从没打开过、本库也没起
        // 任何线程，用 _Exit 跳过析构最安全，也让退出足够快。
        std::fprintf(stderr, "TaskInit: 等待 USB-CDC 设备时收到关闭请求，退出\n");
        std::_Exit(EXIT_FAILURE);
    }

    g_usb_device->RegisterRxCallback(McuRxCallback);

    // 注册连接断开回调
    g_usb_device->RegisterDisconnectCallback(McuDisconnectCallback);

    Middleware::g_usb_communication_interface.Init(McuSendData);

    g_initialized = true;
    return true;
}

} // namespace Task
