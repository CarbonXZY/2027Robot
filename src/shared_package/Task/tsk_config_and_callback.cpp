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

/* Public variables ----------------------------------------------------------*/

bool g_mcu_alive = false;    // 下位机链路存活标志位
bool g_initialized = false;  // TaskInit() 是否已完成

/* Private functions ---------------------------------------------------------*/

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

bool TaskInit()
{
    // 设备只开一次：chassis 和 leg 各自都会调进来，第二次起直接返回
    if (g_initialized)
    {
        return true;
    }

    // USB 可能还没枚举完成(刚上电/刚插拔), 打不开就忙等重试,
    // 而不是直接返回 false 让 on_configure 报错、把 ROS 整个带崩。
    Utils::RetryUntil(
        [&] {
            const std::string device = Driver::FindDevice("^usb-STMicroelectronics_STM32_Virtual.*-if00$");
            if (device.empty())
            {
                return;  // 设备还没枚举出来，保持为空，下一轮再找
            }
            g_usb_device = std::make_unique<Driver::USB_CDC>(device);
        },
        [&] { return g_usb_device != nullptr && g_usb_device->IsOpen(); },
        std::chrono::milliseconds(500));

    g_usb_device->RegisterRxCallback(McuRxCallback);
    Middleware::g_usb_communication_interface.Init(McuSendData);

    g_initialized = true;
    return true;
}

} // namespace Task
