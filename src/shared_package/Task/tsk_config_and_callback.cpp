/**
 * @file    tsk_config_and_callback.cpp
 * @author  Carbon
 * @brief   任务配置与回调：USB-CDC 收发接线、链路存活检测
 * @version 1.0
 * @date    2026-09-27
 *
 * @note    连接底层 USB-CDC 驱动与中间层 Communication_Interface
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.hpp"

#include <chrono>
#include <cstdint>
#include <memory>

#include "Communication_Interface.hpp"
#include "usb_cdc.hpp"
#include "utils.hpp"

namespace Task
{

/* Private variables ---------------------------------------------------------*/

static std::unique_ptr<Driver::USB_CDC> g_usb_device;  // USB-CDC 设备实例
static uint32_t Alive_Flag = 0;                // 累计收包计数
static uint32_t Pre_Alive_Flag = 0;            // 上一拍收包计数

/* Public variables ----------------------------------------------------------*/

bool MCU_Alive = false;    // 下位机链路存活标志位
bool initialized = false;  // Task_Init() 是否已完成

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 收：USB 接收线程把一整包递进来。先记一次计数，链路确认之前一律不解包
 */
static void MCU_RxCallback(uint8_t *data, uint16_t length)
{
    Alive_Flag += 1;

    if (!MCU_Alive)
    {
        return;
    }
    Middleware::USB_Communication_Interface.Rx_RptlCallback(data, length);
}

/**
 * @brief 发：Send() 打包好一整包后在这里交给设备
 */
static int64_t MCU_SendData(uint8_t *data, size_t length)
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
void Alive_PeriodElapsedCallback()
{
    if (Alive_Flag == Pre_Alive_Flag)
    {
        // 下位机断开连接
        MCU_Alive = false;
    }
    else
    {
        // 下位机保持连接
        MCU_Alive = true;
    }
    Pre_Alive_Flag = Alive_Flag;
}

/**
 * @brief 控制循环每拍调一次：存活才往外发
 */
void Task_Loop()
{
    // 设备还没开起来：可能所有组件都还没走到 on_configure
    if (!initialized)
    {
        return;
    }

    Alive_PeriodElapsedCallback();

    // 没收到下位机回显之前一个字节都不发
    if (!MCU_Alive)
    {
        return;
    }

    Middleware::USB_Communication_Interface.Send();
}

bool Task_Init()
{
    // 设备只开一次：chassis 和 leg 各自都会调进来，第二次起直接返回
    if (initialized)
    {
        return true;
    }

    // USB 可能还没枚举完成(刚上电/刚插拔), 打不开就忙等重试,
    // 而不是直接返回 false 让 on_configure 报错、把 ROS 整个带崩。
    Utils::RetryUntil(
        [&] { g_usb_device = std::make_unique<Driver::USB_CDC>(Driver::USB_CDC_DEFAULT_DEVICE); },
        [&] { return g_usb_device->IsOpen(); },
        std::chrono::milliseconds(500));

    g_usb_device->RegisterRxCallback(MCU_RxCallback);
    Middleware::USB_Communication_Interface.Init(MCU_SendData);

    initialized = true;
    return true;
}

} // namespace Task
